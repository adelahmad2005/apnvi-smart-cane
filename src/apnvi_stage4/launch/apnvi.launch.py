"""
The whole device: Stages 1 and 2 (stage2.launch.py), Stage 3 (stage3.launch.py) and Stage 4.

On the Jetson:
  ros2 launch apnvi_stage4 apnvi.launch.py pc:=<campus PC IP> port:=<Stage 1 ESP32 by-id>
Stage 4 off (Stages 1 to 3 exactly as before, nothing talks to the master ESP32):
  ros2 launch apnvi_stage4 apnvi.launch.py stage4:=false
Every option of stage2.launch.py, stage3.launch.py and stage4.launch.py works here too.
Each stage keeps its own defaults (the stages are started in separate scopes, so one stage's
default never changes another's, for example camera_height in Stage 1 and Stage 3).

Careful with ports: Stage 1's `port` defaults to /dev/ttyUSB0, which may be the LiDAR or the
master. Give Stage 1's ESP32 by its /dev/serial/by-id name.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, condition=None):
    """Start one stage's launch file in its own scope."""
    source = PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare(package), 'launch', launch_file]))
    return GroupAction([IncludeLaunchDescription(source)], scoped=True, condition=condition)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stage3', default_value='true'),
        DeclareLaunchArgument('stage4', default_value='true'),
        include('apnvi_stage2', 'stage2.launch.py'),            # Stage 1 and Stage 2
        include('apnvi_stage3', 'stage3.launch.py',
                IfCondition(LaunchConfiguration('stage3'))),
        include('apnvi_stage4', 'stage4.launch.py',
                IfCondition(LaunchConfiguration('stage4'))),
    ])
