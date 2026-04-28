#!/usr/bin/env python3

import math
import rclpy
import numpy
import time
import tf2_ros
import yaml
from typing import Optional
import logging

from rclpy.qos import qos_profile_sensor_data

try:
    from djitellopy import Tello
except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "Missing Python dependency 'djitellopy'. Install it with:\n"
        "  python3 -m pip install --user djitellopy\n"
        "or inside your venv, then rebuild/re-source your workspace."
    ) from e

from tello_msg.msg import TelloStatus, TelloID, TelloWifiConfig
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

# Reduce djitellopy log spam (rc commands, etc.)
logging.getLogger("djitellopy").setLevel(logging.WARNING)

# CvBridge is preferred when available, but this project can also run without it.
try:
    from cv_bridge import CvBridge  # type: ignore
except Exception:
    CvBridge = None  # type: ignore

# Tello ROS node class, inherits from the Tello controller object.
#
# Can be configured to be used by multiple drones, publishes, all data collected from the drone and provides control using ROS messages.
class TelloNode():
    def __init__(self, node):
        # ROS node
        self.node = node

        # Timers (keep references so they won't get GC'd)
        self._video_timer = None
        self._odom_timer = None
        self._status_timer = None
        self._rc_timer = None

        # Frame reader / image conversion
        self._frame_read = None
        self._bridge = CvBridge() if CvBridge is not None else None

        # Control state (deadman)
        self._rc_last_time = 0.0
        self._rc = (0, 0, 0, 0)  # lr, fb, ud, yaw in [-100..100]

        # Video publish diagnostics
        self._video_pub_count = 0
        self._video_pub_first_ts: Optional[float] = None
        self._video_pub_last_log_ts: float = 0.0

        # Declare parameters
        self.node.declare_parameter('connect_timeout', 10.0)
        self.node.declare_parameter('tello_ip', '192.168.10.1')
        self.node.declare_parameter('tf_base', 'map')
        self.node.declare_parameter('tf_drone', 'drone')
        self.node.declare_parameter('tf_pub', False)
        self.node.declare_parameter('camera_info_file', '')
        self.node.declare_parameter('video_scale', 1.0)
        self.node.declare_parameter('video_target_fps', 30.0)
        self.node.declare_parameter('rc_rate_hz', 20.0)
        self.node.declare_parameter('rc_timeout_sec', 0.35)

        # Get parameters
        self.connect_timeout = float(self.node.get_parameter('connect_timeout').value)
        self.tello_ip = str(self.node.get_parameter('tello_ip').value)
        self.tf_base = str(self.node.get_parameter('tf_base').value)
        self.tf_drone = str(self.node.get_parameter('tf_drone').value)
        self.tf_pub = bool(self.node.get_parameter('tf_pub').value)
        self.camera_info_file = str(self.node.get_parameter('camera_info_file').value)
        self.video_scale = float(self.node.get_parameter('video_scale').value)
        self.video_target_fps = float(self.node.get_parameter('video_target_fps').value)
        self.rc_rate_hz = float(self.node.get_parameter('rc_rate_hz').value)
        self.rc_timeout_sec = float(self.node.get_parameter('rc_timeout_sec').value)
        if self.video_scale <= 0.0:
            self.video_scale = 1.0
        if self.video_target_fps <= 1.0:
            self.video_target_fps = 30.0
        if self.rc_rate_hz <= 1.0:
            self.rc_rate_hz = 20.0
        if self.rc_timeout_sec <= 0.05:
            self.rc_timeout_sec = 0.35

        # Camera information loaded from calibration yaml
        self.camera_info = None
        
        # Check if camera info file was received as argument
        if len(self.camera_info_file) == 0:
            share_directory = get_package_share_directory('tello')
            self.camera_info_file = share_directory + '/ost.yaml'

        # Read camera info from YAML file
        with open(self.camera_info_file, 'r') as file:
            self.camera_info = yaml.safe_load(file)

        # Configure drone connection
        Tello.TELLO_IP = self.tello_ip
        Tello.RESPONSE_TIMEOUT = int(self.connect_timeout)

        # Connect to drone
        self.node.get_logger().info('Tello: Connecting to drone')

        self.tello = Tello()
        self.tello.connect()

        self.node.get_logger().info('Tello: Connected to drone')

        # Initialize video before starting timers
        self.tello.streamon()
        try:
            self._frame_read = self.tello.get_frame_read(with_queue=False)
        except TypeError:
            self._frame_read = self.tello.get_frame_read()

        # Publishers and subscribers
        self.setup_publishers()
        self.setup_subscribers()

        # Timers (ROS 2 idiomatic, non-blocking)
        self._video_timer = self.node.create_timer(1.0 / float(self.video_target_fps), self._on_video_timer)
        self._odom_timer = self.node.create_timer(1.0 / 10.0, self._on_odom_timer)
        self._status_timer = self.node.create_timer(1.0 / 2.0, self._on_status_timer)
        self._rc_timer = self.node.create_timer(1.0 / float(self.rc_rate_hz), self._on_rc_timer)

        self.node.get_logger().info('Tello: Driver node ready')

    # Setup ROS publishers of the node.
    def setup_publishers(self):
        # Use sensor data QoS to minimize latency (best effort, small depth).
        self.pub_image_raw = self.node.create_publisher(Image, 'image_raw', qos_profile_sensor_data)
        self.pub_camera_info = self.node.create_publisher(CameraInfo, 'camera_info', qos_profile_sensor_data)
        self.pub_status = self.node.create_publisher(TelloStatus, 'status', 1)
        self.pub_id = self.node.create_publisher(TelloID, 'id', 1)
        self.pub_imu = self.node.create_publisher(Imu, 'imu', 1)
        self.pub_battery = self.node.create_publisher(BatteryState, 'battery', 1)
        self.pub_temperature = self.node.create_publisher(Temperature, 'temperature', 1)
        self.pub_odom = self.node.create_publisher(Odometry, 'odom', 1)

        # TF broadcaster
        if self.tf_pub:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
    
    # Setup the topic subscribers of the node.
    def setup_subscribers(self):
        self.sub_emergency = self.node.create_subscription(Empty, 'emergency', self.cb_emergency, 1)
        self.sub_takeoff = self.node.create_subscription(Empty, 'takeoff', self.cb_takeoff, 1)
        self.sub_land = self.node.create_subscription(Empty, 'land', self.cb_land, 1)
        self.sub_control = self.node.create_subscription(Twist, 'control', self.cb_control, 1)
        self.sub_flip = self.node.create_subscription(String, 'flip', self.cb_flip, 1)
        self.sub_wifi_config = self.node.create_subscription(TelloWifiConfig, 'wifi_config', self.cb_wifi_config, 1)

    # Get the orientation of the drone as a quaternion
    def get_orientation_quaternion(self):
        deg_to_rad = math.pi / 180.0
        return euler_to_quaternion([
            self.tello.get_yaw() * deg_to_rad,
            self.tello.get_pitch() * deg_to_rad,
            self.tello.get_roll() * deg_to_rad
        ])

    def _on_odom_timer(self):
        # TF
        if self.tf_pub:
            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = self.tf_base
            t.child_frame_id = self.tf_drone
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = (self.tello.get_barometer()) / 100.0
            self.tf_broadcaster.sendTransform(t)

        if self.pub_imu.get_subscription_count() > 0:
            q = self.get_orientation_quaternion()
            msg = Imu()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.tf_drone
            msg.linear_acceleration.x = self.tello.get_acceleration_x() / 100.0
            msg.linear_acceleration.y = self.tello.get_acceleration_y() / 100.0
            msg.linear_acceleration.z = self.tello.get_acceleration_z() / 100.0
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            self.pub_imu.publish(msg)

        if self.pub_odom.get_subscription_count() > 0:
            q = self.get_orientation_quaternion()
            odom_msg = Odometry()
            odom_msg.header.stamp = self.node.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.tf_base
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            odom_msg.twist.twist.linear.x = float(self.tello.get_speed_x()) / 100.0
            odom_msg.twist.twist.linear.y = float(self.tello.get_speed_y()) / 100.0
            odom_msg.twist.twist.linear.z = float(self.tello.get_speed_z()) / 100.0
            self.pub_odom.publish(odom_msg)

    def _on_status_timer(self):
        # Battery
        if self.pub_battery.get_subscription_count() > 0:
            msg = BatteryState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.tf_drone
            msg.percentage = float(self.tello.get_battery()) / 100.0
            msg.voltage = 3.8
            msg.design_capacity = 1.1
            msg.present = True
            msg.power_supply_technology = 2  # POWER_SUPPLY_TECHNOLOGY_LION
            msg.power_supply_status = 2  # POWER_SUPPLY_STATUS_DISCHARGING
            self.pub_battery.publish(msg)

        # Temperature
        if self.pub_temperature.get_subscription_count() > 0:
            msg = Temperature()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.tf_drone
            msg.temperature = self.tello.get_temperature()
            msg.variance = 0.0
            self.pub_temperature.publish(msg)

        # Tello Status
        if self.pub_status.get_subscription_count() > 0:
            msg = TelloStatus()
            msg.acceleration.x = self.tello.get_acceleration_x()
            msg.acceleration.y = self.tello.get_acceleration_y()
            msg.acceleration.z = self.tello.get_acceleration_z()

            msg.speed.x = float(self.tello.get_speed_x())
            msg.speed.y = float(self.tello.get_speed_y())
            msg.speed.z = float(self.tello.get_speed_z())

            msg.pitch = self.tello.get_pitch()
            msg.roll = self.tello.get_roll()
            msg.yaw = self.tello.get_yaw()

            msg.barometer = int(self.tello.get_barometer())
            msg.distance_tof = self.tello.get_distance_tof()
            msg.fligth_time = self.tello.get_flight_time()
            msg.battery = self.tello.get_battery()
            msg.highest_temperature = self.tello.get_highest_temperature()
            msg.lowest_temperature = self.tello.get_lowest_temperature()
            msg.temperature = self.tello.get_temperature()
            msg.wifi_snr = self.tello.query_wifi_signal_noise_ratio()

            self.pub_status.publish(msg)

        # Tello ID
        if self.pub_id.get_subscription_count() > 0:
            msg = TelloID()
            msg.sdk_version = self.tello.query_sdk_version()
            msg.serial_number = self.tello.query_serial_number()
            self.pub_id.publish(msg)

        # Camera info
        if self.pub_camera_info.get_subscription_count() > 0:
            self.pub_camera_info.publish(self._camera_info_msg())


    def _on_video_timer(self):
        if self.pub_image_raw.get_subscription_count() == 0:
            return

        if self._frame_read is None:
            return

        frame = self._frame_read.frame
        if frame is None:
            return

        img = numpy.array(frame)
        if self.video_scale != 1.0:
            img = self._resize_bgr(img, self.video_scale)

        msg = None
        if self._bridge is not None:
            try:
                msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
            except Exception:
                msg = None

        if msg is None:
            msg = self._bgr8_numpy_to_imgmsg(img)

        msg.header.frame_id = self.tf_drone
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.pub_image_raw.publish(msg)

        now = time.time()
        self._video_pub_count += 1
        if self._video_pub_first_ts is None:
            self._video_pub_first_ts = now
            self.node.get_logger().info("Video: published first frame on /image_raw")
        if now - self._video_pub_last_log_ts > 5.0 and self._video_pub_first_ts is not None:
            dt = max(1e-6, now - self._video_pub_first_ts)
            fps = float(self._video_pub_count) / dt
            self.node.get_logger().info(f"Video: published {self._video_pub_count} frames, ~{fps:.1f} FPS")
            self._video_pub_last_log_ts = now

    # Terminate the code and shutdown node.
    def terminate(self, err):
        self.node.get_logger().error(str(err))
        self.tello.end()
        rclpy.shutdown()

    def stop(self):
        try:
            self.tello.send_rc_control(0, 0, 0, 0)
        except Exception:
            pass
        try:
            self.tello.streamoff()
        except Exception:
            pass
        try:
            self.tello.end()
        except Exception:
            pass

    def _camera_info_msg(self) -> CameraInfo:
        info = self.camera_info or {}

        msg = CameraInfo()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.tf_drone

        msg.width = int(info.get('image_width', 0))
        msg.height = int(info.get('image_height', 0))
        msg.distortion_model = str(info.get('distortion_model', 'plumb_bob'))

        d = info.get('distortion_coefficients', {}).get('data', [])
        msg.d = [float(x) for x in d]

        k = info.get('camera_matrix', {}).get('data', [])
        if len(k) == 9:
            msg.k = [float(x) for x in k]

        r = info.get('rectification_matrix', {}).get('data', [])
        if len(r) == 9:
            msg.r = [float(x) for x in r]

        p = info.get('projection_matrix', {}).get('data', [])
        if len(p) == 12:
            msg.p = [float(x) for x in p]

        return msg

    def _bgr8_numpy_to_imgmsg(self, img: "numpy.ndarray") -> Image:
        # Ensure HxWx3 uint8 contiguous buffer
        if img is None:
            raise ValueError("Frame is None")
        if img.dtype != numpy.uint8:
            img = img.astype(numpy.uint8, copy=False)
        if img.ndim != 3 or img.shape[2] != 3:
            raise ValueError(f"Expected HxWx3 BGR image, got shape={getattr(img, 'shape', None)}")

        img = numpy.ascontiguousarray(img)
        height, width, _ = img.shape

        msg = Image()
        msg.height = int(height)
        msg.width = int(width)
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = int(width * 3)
        msg.data = img.tobytes()
        return msg

    def _resize_bgr(self, img: "numpy.ndarray", scale: float) -> "numpy.ndarray":
        # Fast nearest-neighbor downscale without OpenCV dependency.
        # Good enough for low-latency preview/control.
        if scale >= 1.0:
            return img

        # Common fast paths (no allocations, just stride-based subsampling)
        if abs(scale - 0.5) < 1e-6:
            return img[::2, ::2]
        if abs(scale - 0.25) < 1e-6:
            return img[::4, ::4]

        h, w = int(img.shape[0]), int(img.shape[1])
        cache = self._resize_cache
        if cache is None or cache[0] != h or cache[1] != w or abs(cache[2] - scale) > 1e-9:
            nh = max(1, int(h * scale))
            nw = max(1, int(w * scale))
            ys = (numpy.linspace(0, h - 1, nh)).astype(numpy.int32)
            xs = (numpy.linspace(0, w - 1, nw)).astype(numpy.int32)
            self._resize_cache = (h, w, scale, ys, xs)
        else:
            ys, xs = cache[3], cache[4]

        return img[ys][:, xs]

    # Stop all movement in the drone
    def cb_emergency(self, msg):
        self.tello.emergency()

    # Drone takeoff message control
    def cb_takeoff(self, msg):
        try:
            self.tello.takeoff()
        except Exception as e:
            self.node.get_logger().warn(f"Takeoff failed: {e}")

    # Land the drone message callback
    def cb_land(self, msg):
        try:
            self.tello.land()
        except Exception as e:
            # Don't crash the node on failed land retries.
            self.node.get_logger().warn(f"Land failed: {e}")

    # Control messages received use to control the drone "analogically"
    #
    # This method of controls allow for more precision in the drone control.
    #
    # Receives the linear and angular velocities to be applied from -100 to 100.
    def cb_control(self, msg):
        lr = int(msg.linear.x)
        fb = int(msg.linear.y)
        ud = int(msg.linear.z)
        yaw = int(msg.angular.z)

        lr = max(-100, min(100, lr))
        fb = max(-100, min(100, fb))
        ud = max(-100, min(100, ud))
        yaw = max(-100, min(100, yaw))

        self._rc = (lr, fb, ud, yaw)
        self._rc_last_time = time.time()

    # Configure the wifi credential that should be used by the drone.
    #
    # The drone will be restarted after the credentials are changed.
    def cb_wifi_config(self, msg):
        try:
            self.tello.set_wifi_credentials(msg.ssid, msg.password)
        except Exception as e:
            self.node.get_logger().warn(f"WiFi config failed: {e}")
    
    # Perform a drone flip in a direction specified.
    # 
    # Directions can be "r" for right, "l" for left, "f" for forward or "b" for backward.
    def cb_flip(self, msg):
        try:
            self.tello.flip(msg.data)
        except Exception as e:
            self.node.get_logger().warn(f"Flip failed: {e}")

    def _on_rc_timer(self):
        now = time.time()
        rc = self._rc
        age = now - self._rc_last_time

        if age > self.rc_timeout_sec:
            rc = (0, 0, 0, 0)
            self._rc = rc

        try:
            self.tello.send_rc_control(int(rc[0]), int(rc[1]), int(rc[2]), int(rc[3]))
        except Exception:
            pass

# Convert a rotation from euler to quaternion.
def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# Convert rotation from quaternion to euler.
def quaternion_to_euler(q):
    (x, y, z, w) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('tello')
    drone = TelloNode(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        drone.stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
