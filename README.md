# Tello VIO (ROS 2 Humble, Ubuntu 22.04)

This is a **research project** aimed at implementing and studying **Visual-Inertial Odometry (VIO)** on a **DJI Tello** drone using **ROS 2 Humble on Ubuntu 22.04**.

The project is **work in progress**: I am currently exploring how to integrate VIO on the Tello and how to implement this **pose estimation** approach as robustly as possible using the camera and IMU.

In addition, this repo includes a ROS 2 driver for the Tello (based on [DJITelloPy](https://github.com/damiafuentes/DJITelloPy) and the official SDK: [Tello-Python](https://github.com/dji-sdk/Tello-Python)). Multi-drone control is also possible (swarm is supported on [Tello EDU](https://www.ryzerobotics.com/tello-edu)).

- It is recommended to update the Tello firmware to the latest version available.
- The workspace is divided into sub-workspaces that contain different logic.
  - `tello` package is the main package, includes access to the drone information, camera image and  control.
  - `tello_msg` package defines custom messages to access specific Tello data.
    - Defines the `TelloStatus`, `TelloID` and `TelloWifiConfig` messages 
  - `tello_control` package is a sample control package that displays the drone image and provides keyboard control.
    - Shows a live camera window and overlays battery + controls.
    - Controls: `T` takeoff, `L` land, `E` emergency, `F` flip forward, arrows/WASD movement (`W/S` up/down, `A/D` yaw).

- Bellow is the list of topics published and consumed by the `tello` package
- The list of published topics alongside their description and frequency. These topics are only published when some node subscribed to them, otherwise they are not processed.

| Topic        | Type                           | Description                                                  | Frequency |
| ------------ | ------------------------------ | ------------------------------------------------------------ | --------- |
| /image_raw   | sensor_msgs/Image              | Image of the Tello camera                                    | 30hz      |
| /camera_info | sensor_msgs/CameraInfo         | Camera information (size, calibration, etc)                  | 2hz       |
| /status      | tello_msg/TelloStatus          | Status of the drone (wifi strength, batery, temperature, etc) | 2hz       |
| /id          | tello_msg/TelloID              | Identification of the drone w/ serial number and firmware    | 2hz       |
| /imu         | sensor_msgs/Imu                | Imu data capture from the drone                              | 10hz      |
| /battery     | sensor_msgs/BatteryState       | Battery status                                               | 2hz       |
| /temperature | sensor_msgs/Temperature        | Temperature of the drone                                     | 2hz       |
| /odom        | nav_msgs/Odometry              | Odometry (only orientation and speed)                        | 10hz      |
| /tf          | geometry_msgs/TransformStamped | Transform from base to drone tf, prefer a external publisher. | 10hz      |

- The list of topics subscribed by the node, these topics can be renamed in the launch file.

| Topic        | Type                      | Description                                                  |
| ------------ | ------------------------- | ------------------------------------------------------------ |
| \emergency   | std_msgs/Empty            | When received the drone instantly shuts its motors off (even when flying), used for safety purposes |
| \takeoff     | std_msgs/Empty            | Drone takeoff message, make sure that the drone has space to takeoff safely before usage. |
| \land        | std_msgs/Empty            | Land the drone.                                              |
| \control     | geometry_msgs/Twist       | Control the drone analogically. Linear values should range from -100 to 100, speed can be set in x, y, z for movement in 3D space. Angular rotation is performed in the z coordinate. Coordinates are relative to the drone position (x always relative to the direction of the drone) |
| \flip        | std_msgs/String           | Do a flip with the drone in a direction specified. Possible directions can be "r" for right, "l" for left, "f" for forward or "b" for backward. |
| \wifi_config | tello_msg/TelloWifiConfig | Configure the wifi credential that should be used by the drone. The drone will be restarted after the credentials are changed. |

- The list of parameters used to configure the node. These should be defined on a launch file.

| Name             | Type    | Description                                                  | Default        |
| ---------------- | ------- | ------------------------------------------------------------ | -------------- |
| connect_timeout  | float   | Time  (seconds) until the node is killed if connection to the drone is not available. | 10.0           |
| tello_ip         | string  | IP of the tello drone. When using multiple drones multiple nodes with different IP can be launched. | '192.168.10.1' |
| tf_base          | string  | Base tf to be used when publishing data                      | 'map'          |
| tf_drone         | string  | Name of the drone tf to use when publishing data             | 'drone'        |
| tf_pub           | boolean | If true a static TF from tf_base to tf_drone is published    | False          |
| camera_info_file | string  | Path to a YAML camera calibration file (obtained with the calibration tool) | ''             |
| video_backend    | string  | Video decoder backend. `pyav` is recommended; `opencv` is a good fallback; `djitellopy` uses the internal reader (more buffering). | 'pyav' |
| video_scale      | float   | Downscale factor before publishing (reduces CPU + latency). | 1.0 |
| video_target_fps | float   | Target publish rate. Frames are dropped to keep latency low. | 30.0 |
| rc_rate_hz       | float   | Rate for sending RC commands to the drone. | 20.0 |
| rc_timeout_sec   | float   | Deadman timeout: if no new control is received, RC is set to zero. | 0.35 |



### Camera Calibration

- To allow the drone to be used for 3D vision tasks, as for example monocular SLAM the camera should be first calibrated.
- A sample calibration file is provided with parameters captures from the drone used for testing but it is recommended to perform individual calibrations for each drone used.
- Calibration can be achieved using the [camera_calibration](https://navigation.ros.org/tutorials/docs/camera_calibration.html) package. Calibration pattern can be generated using the [calib.io pattern generator](https://calib.io/pages/camera-calibration-pattern-generator) tool.

```bash
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.16 image:=/image_raw camera:=/camera_info
```

- Take as many frame as possible and measure your check board grid size to ensure good accuracy in the process. When the process ends a `calibrationdata.tar.gz` will be created in the `/tmp` path.



### Launch File

Launch files in ROS 2 are defined using Python. This repository ships a ready-to-use launch file:

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-clean-cache
source install/setup.bash

# Start driver + control GUI + RViz + TF publisher
ros2 launch tello tello.launch.py
```

You can tune video latency/CPU by changing the parameters:

```bash
# Lower latency / lower CPU (recommended starting point)
ros2 launch tello tello.launch.py video_scale:=0.5 video_target_fps:=20

# If PyAV has trouble decoding on your machine, try OpenCV backend
ros2 launch tello tello.launch.py video_backend:=opencv video_scale:=0.5
```



### Overheating Problems

- The motor drivers in the DJI Tello overheat after a while when the drone is not flying. To cool down the drivers i have removed the plastic section on top of the heat spreader (as seen in the picture).
- If you are comfortable with leaving the PCB exposed removing the plastic cover should result in even better thermals.
- If possible place the drone on top of an old computer fan or use a laptop cooler to prevent the drone from shutting down due to overheating.

### Install

This repo is tested on **ROS 2 Humble** (Ubuntu 22.04).

Install ROS dependencies:

```bash
sudo apt update
rosdep update
rosdep install -i --from-paths workspace/src slam/src --rosdistro humble -y
```

Install Python dependencies (driver/video decode):

```bash
python3 -m pip install --user djitellopy av
```

Build:

```bash
colcon build --cmake-clean-cache
```



### Visual SLAM

- The drone is equipped with a IMU and a camera that can be used for visual SLAM in order to obtain the location of the drone and a map of the environment.
- [ORB SLAM 2](https://github.com/raulmur/ORB_SLAM2) is a monocular visual based algorithm for SLAM that can be easily integrated with the Tello drone using this package.
- The wrapper provided alongside with this repository is based on the [alsora/ros2-ORB-SLAM2](https://github.com/alsora/ros2-ORB_SLAM2/tree/f890df18983ead8cd2ae36676036d535ee52951b) project using the [alsora/ORB_SLAM2](alsora/ORB_SLAM2) modified version of ORB Slam that does not depend on pangolin.
- The `orbslam2` package is **optional**: it will build even if ORB-SLAM2 is not installed (it will skip building the `mono` executable and print a CMake warning).

To enable it, build/install ORB-SLAM2 separately and set `ORB_SLAM2_ROOT_DIR` to the ORB-SLAM2 directory (must contain `include/ORB_SLAM2/System.h` and `lib*/libORB_SLAM2.*`).

To run the monocular SLAM node after installing all dependencies and building the package run:

```bash
ros2 run orbslam2 mono <VOCABULARY FILE> <CONFIG_FILE>
```

- The vocabulary file can be obtained from the ORB_SLAM2 repository ( `ORB_SLAM2/Vocabulary/ORBvoc.txt`).
- Sample configuration files can be found inside the package at `orbslam2/src/monocular/config.yaml` for monocular SLAM.

### Theoretical references (to be added)

This section is reserved for theoretical references on VIO/SLAM/state estimation that will be added later. For now it acts as a placeholder:

- **VIO / Visual-Inertial Navigation**: TBD
- **State Estimation / Filtering (EKF/MSCKF)**: TBD
- **Optimization / Factor Graphs (Bundle Adjustment, Smoothing)**: TBD
- **IMU Preintegration**: TBD
- **Camera Models & Calibration**: TBD
- **ROS 2 + VIO integration notes**: TBD



### Setup ROS 2 Humble

- Run the install script to setup the ROS 2 (Humble Hawksbill) environment. 
- Check the [ROS2 Tutorials](https://index.ros.org/doc/ros2/Tutorials/) page to learn how to setup workspace and create packages.

##### Workspace

- To install dependencies of the packages available in a workspace directory `src` run `rosdep install -i --from-paths src --rosdistro humble -y`
- To build workspace you can use the command `colcon build`,  some useful arguments for `colcon build`:

  - `--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
  - `--symlink-install` saves you from having to rebuild every time you tweak python scripts
  - `--event-handlers console_direct+` shows console output while building (can otherwise be found in the `log` directory)

##### Packages

- To create a new ROS2 package (C++ or Python) for development move to the `src` package and run

```bash
# CPP Package
ros2 pkg create --build-type ament_cmake --node-name <node_name> <package_name>

# Python Package
ros2 pkg create --build-type ament_python --node-name <node_name> <package_name>
```

##### Tools

- `rqt_topic` Used to monitor topics and their values in a list
- `rqt_graph` Draw the graph of connection between the currently active nodes and explore communication between them
- `rviz` Visualize topics in 3D space.

##### Bags

- Bags can be used to record data from topics that can be later replayed for off-line testing. Bags can be manipulated using the `ros2 bag` command. To 

```bash
# Record a bag containing data from some topics into a file
ros2 bag record -o <bag_file_name> /turtle1/cmd_vel /turtle1/pose ...

# Check the content of a bag run the command
ros2 bag info <bag_file_name>

# Replay the content of some topics recorded into a bag file
 ros2 bag play <bag_file_name>
```

- To play ROS 1 bags in ROS 2 you will need to first install ROS 1, and the ROS bag adapter plugin. The the bags can be run using the command.

```bash
ros2 bag play -s rosbag_v2 <path_to_bagfile>
```

##### Camera calibration

- Calibration files provided were obtained using our test drone.
- To get your own calibration file use the [ROS camera calibration tool]()



### Ubuntu 22.04 notes

- This repository targets **Ubuntu 22.04 (jammy)** + **ROS 2 Humble**.
- If any dependency scripts rely on `lsb_release -cs`, it should return `jammy` on Ubuntu 22.04.



