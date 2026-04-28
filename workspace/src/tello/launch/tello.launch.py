from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config = get_package_share_directory('tello_control') + '/rviz.rviz'

    return LaunchDescription(
        [
            Node(
                package='tello',
                executable='tello',
                output='screen',
                namespace='/',
                name='tello',
                parameters=[
                    {'connect_timeout': 10.0},
                    {'tello_ip': '192.168.10.1'},
                    {'tf_base': 'map'},
                    {'tf_drone': 'drone'},
                ],
                # Publish camera on the standard topic name: /image_raw
                # Don't respawn on missing Python deps (avoids restart spam).
                respawn=False,
            ),
            Node(
                package='tello_control',
                executable='tello_control',
                namespace='/',
                name='control',
                output='screen',
                respawn=False,
            ),
            Node(
                package='rqt_gui',
                executable='rqt_gui',
                output='screen',
                namespace='/',
                name='rqt',
                respawn=False,
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                namespace='/',
                name='rviz2',
                respawn=True,
                arguments=['-d', rviz_config],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                namespace='/',
                name='tf',
                arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'drone'],
                respawn=True,
            ),
        ]
    )

