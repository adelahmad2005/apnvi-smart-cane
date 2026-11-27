"""
The Jetson load test: Stages 1 and 2, the LiDAR, RTAB-Map and Nav2 together.

  ros2 launch apnvi_stage3 loadtest.launch.py
Starts Stage 1 and 2 (stage2.launch.py) and Stage 3 (stage3.launch.py) as they would run on the
device. Then watch tegrastats for 10 minutes and count how many levels a second Stage 1 sends
(/hazard/level, it must stay at 10). Options:
  test_mode:=false      use the real ESP32 (give its path with port:=/dev/serial/by-id/...)
  port:=...             the ESP32's by-id path (only with test_mode:=false)
  pc:=<campus PC IP>    where Qwen and the speech server run (default: none, no questions asked)
  mic:=pulse            the microphone
  camera:=false         no camera (default true: Stage 1 needs it)
  camera_map:=true      the camera's colour and depth also go into the map (place recognition)
All the other options of stage2.launch.py and stage3.launch.py can be added here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

STAGE2_OPTIONS = ['test_mode', 'port', 'pc', 'mic', 'camera']


def generate_launch_description():
    stage2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('apnvi_stage2'), 'launch', 'stage2.launch.py'])),
        launch_arguments={**{name: LaunchConfiguration(name) for name in STAGE2_OPTIONS},
                          'align_depth': LaunchConfiguration('camera_map')}.items())
    stage3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('apnvi_stage3'), 'launch', 'stage3.launch.py'])),
        launch_arguments={'camera_map': LaunchConfiguration('camera_map')}.items())
    return LaunchDescription([
        DeclareLaunchArgument('test_mode', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('pc', default_value='127.0.0.1'),
        DeclareLaunchArgument('mic', default_value='default'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('camera_map', default_value='false'),
        stage2,
        stage3,
    ])
