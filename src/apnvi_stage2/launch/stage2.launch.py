"""
Start Stage 1 and Stage 2 together with one command.

On the Jetson, with the campus PC running Ollama and the speech server:
  ros2 launch apnvi_stage2 stage2.launch.py pc:=<campus PC IP address>
On a laptop with no hardware (typed questions only, Stage 1 in test mode):
  ros2 launch apnvi_stage2 stage2.launch.py camera:=false test_mode:=true pc:=<IP>
Options (Stage 1's options all work here too, see stage1.launch.py):
  pc:=10.1.2.3          the campus PC's IP address (Qwen 11434, speech 8090, voice 8091)
  mic:=plughw:2,0       the microphone, as listed by "arecord -l" (default: 'default')
  model:=qwen3-vl:2b    use the smaller model (default qwen3-vl:4b-instruct)
  answer_voice:=false   do not speak Stage 2's sentences (on by default)
  quiet_volume:=0.4     Stage 2's voice volume while Stage 1 is at warning (0 to 1)
  voice_engine:=kokoro  the natural voice made on the campus PC (port 8091), espeak-ng if
                        the PC does not answer. Default espeak (the Jetson's own voice)
  beeps:=true           switch Stage 1's beeps back on (off since 26 Oct, Adel)
  sound_out:=jetson     play and record on the Jetson itself. Default browser (Adel, 15 Nov):
                        the 8081 page plays the device's voice and records the question,
                        on whatever computer has it open (click "Sound on" once)
  labels:=false         the labeller does not look by itself ("scan the room" still works).
                        Default true: as the device moves it pins signs, room names and
                        landmarks on the map
  labeler:=false        do not start the labeller at all
  camera_tilt:=3.5      the camera is tipped down this many degrees (Stage 1's floor removal)
  camera_back_m:=0.70   how far the camera sits behind the wheels' centre
  stage2_view:=false    do not start the Stage 2 browser view (port 8081)
  view:=true            also start Stage 1's depth view (port 8080). Off by default here
                        (22 Oct): it used a whole processor core, and the Stage 2 page
                        already shows Stage 1's level, distance and camera status
Stage 2's programs run at a lower priority than Stage 1's (nice), so the safety warnings
always get the processor first.
Type a question instead of speaking:
  ros2 topic pub --once /voice/question std_msgs/String "{data: 'where is the exit'}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

STAGE1_OPTIONS = ['camera', 'test_mode', 'port', 'rviz', 'view', 'voice', 'beeps',
                  'camera_height', 'align_depth', 'camera_tilt']
LOWER_PRIORITY = 'nice -n 10'           # Stage 1 first: Stage 2 gets what is left


def generate_launch_description():
    pc = LaunchConfiguration('pc')
    mic = LaunchConfiguration('mic')
    model = LaunchConfiguration('model')
    answer_voice = LaunchConfiguration('answer_voice')
    quiet_volume = LaunchConfiguration('quiet_volume')
    voice_engine = LaunchConfiguration('voice_engine')
    stage2_view = LaunchConfiguration('stage2_view')
    color_topic = LaunchConfiguration('color_topic')
    sound_out = LaunchConfiguration('sound_out')
    camera_back = ParameterValue(LaunchConfiguration('camera_back_m'), value_type=float)

    # Stage 1 exactly as before, with any Stage 1 option given on the command line.
    stage1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('apnvi_stage1'), 'launch', 'stage1.launch.py'])),
        launch_arguments={name: LaunchConfiguration(name) for name in STAGE1_OPTIONS}.items())

    return LaunchDescription([
        DeclareLaunchArgument('pc', default_value=''),
        DeclareLaunchArgument('mic', default_value='default'),
        DeclareLaunchArgument('model', default_value='qwen3-vl:4b-instruct'),
        DeclareLaunchArgument('answer_voice', default_value='true'),
        DeclareLaunchArgument('quiet_volume', default_value='0.4'),
        DeclareLaunchArgument('voice_engine', default_value='espeak'),
        DeclareLaunchArgument('stage2_view', default_value='true'),
        DeclareLaunchArgument('color_topic', default_value='/camera/camera/color/image_raw'),
        # Stage 1's options, with Stage 1's own defaults.
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('test_mode', default_value='false'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('view', default_value='false'),     # Stage 1's own default: true
        DeclareLaunchArgument('voice', default_value='false'),
        DeclareLaunchArgument('beeps', default_value='false'),    # Adel, 26 Oct: no beeps
        DeclareLaunchArgument('camera_height', default_value='0.86'),   # on the device, 13 Nov
        DeclareLaunchArgument('align_depth', default_value='false'),
        DeclareLaunchArgument('camera_tilt', default_value='0.0'),      # tipped down, degrees
        DeclareLaunchArgument('sound_out', default_value='browser'),    # Adel, 15 Nov
        DeclareLaunchArgument('camera_back_m', default_value='0.70'),   # camera behind the wheels
        DeclareLaunchArgument('labeler', default_value='true'),         # start the labeller
        DeclareLaunchArgument('labels', default_value='true'),          # it looks by itself

        stage1,
        Node(package='apnvi_stage2', executable='stage2_node', output='screen',
             prefix=LOWER_PRIORITY,
             parameters=[{'pc': pc, 'mic': mic, 'model': model, 'voice': answer_voice,
                          'quiet_volume': ParameterValue(quiet_volume, value_type=float),
                          'voice_engine': voice_engine, 'sound_out': sound_out,
                          'camera_back_m': camera_back,
                          'color_topic': color_topic}]),
        # The map labels itself: looks now and then, pins signs and landmarks.
        Node(package='apnvi_stage2', executable='labeler_node', output='screen',
             prefix=LOWER_PRIORITY,
             parameters=[{'pc': pc, 'model': model, 'color_topic': color_topic,
                          'camera_back_m': camera_back,
                          'labels': ParameterValue(LaunchConfiguration('labels'),
                                                   value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('labeler'))),
        Node(package='apnvi_stage2', executable='stage2_view', output='screen',
             prefix=LOWER_PRIORITY,
             parameters=[{'color_topic': color_topic}],
             condition=IfCondition(stage2_view)),
    ])
