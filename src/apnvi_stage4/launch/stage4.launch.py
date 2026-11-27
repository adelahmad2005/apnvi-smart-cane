"""
Stage 4 alone: the drive, the guide, the vibration, health and free mode steering.

The drive link, the hazard gate, the guide, the vibration, Nav2's follower, the health
program and (when switched on) free mode steering.

Usually started by apnvi.launch.py (Stages 1 to 4 together). On its own (Stages 1 to 3
already running):
  ros2 launch apnvi_stage4 stage4.launch.py
Options:
  drive:=false            do not start the drive link (nothing talks to the master ESP32)
  dry_run:=true           start the drive link without opening any port (prints what it
                          would send)
  master_port:=...        the master's /dev/serial/by-id (or by-path) name; default: look for
                          the CP2102 with serial 0001 in /dev/serial/by-id
  follow:=false           do not start Nav2's route follower (controller_server)
  rear_check:=true        switch on the master's "nobody behind the device" stop (only once
                          the rear HC-SR04 is wired)
  test_power:=true        let keyboard_drive set raw wheel power (for measuring)
  allow_reverse:=true     allow backing up (wheels-in-the-air tests only)
  wheel_spacing_m:=0.367  middle of the left tyre to the middle of the right (measured 13 Nov)
  wheel_diameter_m:=0.074 across one wheel (measured 13 Nov, about 74 mm)
  speed_at_cap_mps:=0.64  wheel speed at 100% in the air (13 Nov). On the floor the measured
                          table in drive.py (FLOOR_TABLE, 15 Nov) is used instead
  max_speed_mps:=0.5      top speed (Adel, 11 Nov; may change)
  min_power_pct:=45       lowest wheel power that moves the device on the floor (15 Nov)
  turn_gain:=1.0          extra on turns while driving (1.8 until 23 Nov: it zigzagged)
  camera_to_front_m:=0.83 camera to the base's front edge (Adel, 15 Nov). The hazard
                          stop (0.5 m) and the vibration are measured from the device's front
  free_steer:=true        free mode steering: with no trip, the device turns the
                          user away from what is in the way while they push it. It never
                          drives forward. Default false: switch it on only after its floor
                          tests
  free_mode:=false        no vibration signals while the user pushes with no trip (default true)
  two_sides:=false        the grip's two motors are NOT on its left and right: left is one
                          tick, right is three (default true: the side's own motor ticks)
  left_motor:=2           which grip motor sits on the left (default 1)
  careful_m:=1.0          the "careful" signal starts this far from the device's front
  free_turn_rps:=0.4      how fast free mode steering turns the device (rad/s)
  free_centre:=false      free mode steering does not centre the device in a narrow gap
  stand_off_m:=0.8        how far in front of a thing a trip to it ends
  unstuck:=false          a trip stopped by the camera does not get itself free (17 Nov,
                          unstuck.py): it only waits and asks to be turned, as before
  health:=false           do not start the program that says "Ready" and speaks failures
Stage 4's programs run at a lower priority than Stage 1's (nice), so the safety warnings
always get the processor first. The drive link and the hazard gate get a little more than
the rest, because the master stops if the Jetson is quiet for 0.5 s.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

LOWER_PRIORITY = 'nice -n 10'           # Stage 1 first
DRIVE_PRIORITY = 'nice -n 5'            # below Stage 1, above the rest of Stage 4
TO_MEASURE = {'wheel_spacing_m': '0.367', 'wheel_diameter_m': '0.074',
              'speed_at_cap_mps': '0.64', 'max_speed_mps': '0.5', 'min_power_pct': '45',
              'turn_gain': '1.0'}


def follow_params():
    """Find the Nav2 follower's settings file (installed), or the source copy for tests."""
    try:
        share = get_package_share_directory('apnvi_stage4')
    except LookupError:
        share = os.path.join(os.path.dirname(__file__), '..')
    return os.path.join(share, 'config', 'nav2_follow.yaml')


def generate_launch_description():
    config = LaunchConfiguration
    nav2 = follow_params()
    drive_params = {name: ParameterValue(config(name), value_type=float) for name in TO_MEASURE}
    drive_params.update({
        'port': config('master_port'),
        'dry_run': ParameterValue(config('dry_run'), value_type=bool),
        'rear_check': ParameterValue(config('rear_check'), value_type=bool),
        'test_power': ParameterValue(config('test_power'), value_type=bool),
        'allow_reverse': ParameterValue(config('allow_reverse'), value_type=bool),
    })
    front = {'camera_to_front_m': ParameterValue(config('camera_to_front_m'), value_type=float)}
    vibration = dict(front, **{
        'careful_m': ParameterValue(config('careful_m'), value_type=float),
        'two_sides': ParameterValue(config('two_sides'), value_type=bool),
        'left_motor': ParameterValue(config('left_motor'), value_type=int),
        'free_mode': ParameterValue(config('free_mode'), value_type=bool)})
    return LaunchDescription([
        DeclareLaunchArgument('drive', default_value='true'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('master_port', default_value=''),
        DeclareLaunchArgument('follow', default_value='true'),
        DeclareLaunchArgument('rear_check', default_value='false'),
        DeclareLaunchArgument('test_power', default_value='false'),
        DeclareLaunchArgument('allow_reverse', default_value='false'),
        DeclareLaunchArgument('camera_to_front_m', default_value='0.83'),
        DeclareLaunchArgument('free_steer', default_value='false'),
        DeclareLaunchArgument('free_mode', default_value='true'),
        DeclareLaunchArgument('two_sides', default_value='true'),
        DeclareLaunchArgument('left_motor', default_value='1'),
        DeclareLaunchArgument('careful_m', default_value='1.0'),
        DeclareLaunchArgument('health', default_value='true'),
        DeclareLaunchArgument('free_turn_rps', default_value='0.4'),
        DeclareLaunchArgument('free_centre', default_value='true'),
        DeclareLaunchArgument('stand_off_m', default_value='0.8'),
        DeclareLaunchArgument('unstuck', default_value='true'),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in TO_MEASURE.items()],

        # The only program that talks to the master ESP32.
        Node(package='apnvi_stage4', executable='drive_link_node', output='screen',
             prefix=DRIVE_PRIORITY, parameters=[drive_params],
             condition=IfCondition(config('drive'))),
        # Stage 1 hazard -> forward speed 0. Between Nav2 and the drive link.
        Node(package='apnvi_stage4', executable='hazard_gate_node', output='screen',
             prefix=DRIVE_PRIORITY, parameters=[front]),
        # The grip's vibration signals: careful, stopped, going left / right, heard you,
        # arrived, need help.
        Node(package='apnvi_stage4', executable='vibration_node', output='screen',
             prefix=LOWER_PRIORITY, parameters=[vibration]),
        # Free mode steering: turns only, and only while the user pushes.
        Node(package='apnvi_stage4', executable='free_mode_node', output='screen',
             prefix=DRIVE_PRIORITY,
             parameters=[{'turn_rps': ParameterValue(config('free_turn_rps'), value_type=float),
                          'centre': ParameterValue(config('free_centre'), value_type=bool)}],
             condition=IfCondition(config('free_steer'))),
        # Says "Ready", and speaks up when a part of the device stops working.
        Node(package='apnvi_stage4', executable='health_node', output='screen',
             prefix=LOWER_PRIORITY, condition=IfCondition(config('health'))),
        # Goals, arrival, red and green presses, follow-ups.
        Node(package='apnvi_stage4', executable='guide_node', output='screen',
             prefix=LOWER_PRIORITY,
             parameters=[dict(front, **{
                 'stand_off_m': ParameterValue(config('stand_off_m'), value_type=float),
                 'unstuck': ParameterValue(config('unstuck'), value_type=bool)})]),
        # Nav2's route follower: Regulated Pure Pursuit, 0.5 m/s, 0.5 m arrival.
        Node(package='nav2_controller', executable='controller_server', output='screen',
             prefix=LOWER_PRIORITY, parameters=[nav2],
             remappings=[('cmd_vel', '/cmd_vel_nav')],
             condition=IfCondition(config('follow'))),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_follow', output='screen', prefix=LOWER_PRIORITY,
             parameters=[nav2], condition=IfCondition(config('follow'))),
    ])
