"""
Stage 3, LiDAR only: the device draws a map while it is pushed around.

On the Jetson (the LiDAR plugged in):
  ros2 launch apnvi_stage3 stage3.launch.py
The map is the topic /map. The device's position is the frame base_link in the frame map.
The map manager owns /map and the frame map: RTAB-Map draws in its own frame rtabmap_map and
sends its grid to /rtabmap/live_map. While mapping the manager passes both on unchanged; after
a load it shows the clean saved map and moves rtabmap_map so the LiDAR sweep fits the walls.
Options:
  lidar:=false          do not start the LiDAR driver (play a recording instead)
  lidar_port:=...       the LiDAR's /dev/serial/by-id path (default: the C1 on this Jetson)
  lidar_ws:=...         where the LiDAR driver was built (default ~/lidar_ws)
  rate:=1.0             how many times a second the map is updated (default 1)
  map_view:=false       do not make the map picture for the 8081 page (on by default)
  nav2:=false           do not start Nav2's route planner (on by default; it plans, nothing drives)
  camera_map:=true      add the camera's colour and depth to the map (place recognition; the
                        camera's depth must be aligned: loadtest.launch.py does that)
  map:=ESB117           load this saved map when starting (the device then finds itself on it)
  db_path:=...          the map file (default: a new file in ~/.ros/apnvi_maps each run)
Where the sensors sit, in metres from base_link (the spot on the floor midway between the two
wheel centres, where the device turns). Measured on the device 13 Nov (Adel, approximate);
checked by driving 15 Nov: forward is forward, and turns needed lidar_roll.
  lidar_height:=0.77    LiDAR height above the floor (on the device, 13 Nov; 0.84 on the chair)
  lidar_x:=-0.70        LiDAR forward of base_link (negative: behind; 13 Nov, about 70 cm)
  lidar_y:=0.0          LiDAR to the left of base_link (negative: right)
  lidar_yaw:=3.14159    LiDAR turn in radians (measured 31 Oct: its zero angle points backward)
  lidar_roll:=3.14159   LiDAR mounted upside down (15 Nov: on the device a left turn showed as
                        a right turn on the map; 0 = the right way up, as on the chair)
  camera_height:=0.86   camera lens height (on the device, 13 Nov, straight above the LiDAR)
  camera_x:=-0.70       camera forward of base_link (straight above the LiDAR)
  camera_y:=0.0         camera to the left of base_link
Stage 3's programs run at a lower priority than Stage 1's (nice), so the safety warnings
always get the processor first.
"""

from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

LOWER_PRIORITY = 'nice -n 10'           # Stage 1 first: Stage 3 gets what is left
LIDAR_PORT = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
              '027476fbb631f1118c53875f30d20014-if00-port0')
BASE = 'base_link'                      # on the floor, under the middle of the device
RTABMAP_FRAME = 'rtabmap_map'           # RTAB-Map's map frame; the manager hangs it under map
LIVE_MAP = '/rtabmap/live_map'          # RTAB-Map's grid; the manager passes it on to /map
LIDAR_FRAME = 'laser'                   # the name the LiDAR driver puts on its scans
CAMERA_FRAME = 'camera_link'            # the RealSense driver's own top frame
CAMERA = '/camera/camera'               # the RealSense driver's topic prefix

# RTAB-Map settings, all text because RTAB-Map reads them that way.
ICP = {
    'Reg/Strategy': '1',                # 1: line up LiDAR sweeps (0 would use the camera)
    'Reg/Force3DoF': 'true',            # flat floor: only forward, sideways and turn
    'Icp/PointToPlane': 'false',        # 2D sweeps have no surfaces, so match points to points
    'Icp/VoxelSize': '0.05',            # thin the sweep to one point per 5 cm: less work
    'Icp/MaxCorrespondenceDistance': '0.3',    # only match points within 30 cm of each other
    'Icp/Iterations': '15',             # tries to line up one sweep
    'Icp/CorrespondenceRatio': '0.2',   # at least 20% of points must match, or it is lost
    'Icp/MaxTranslation': '1.0',        # refuse a jump of more than 1 m between sweeps
    'Optimizer/GravitySigma': '0',      # no motion sensor, so no gravity hints
}


def rtabmap(with_camera):
    """Make the map node: LiDAR only, or LiDAR plus the camera (chosen by camera_map)."""
    camera_map = LaunchConfiguration('camera_map')
    settings = {
        'frame_id': BASE,
        'odom_frame_id': 'odom',
        'map_frame_id': RTABMAP_FRAME,
        'approx_sync': with_camera,     # LiDAR and camera stamps differ a little
        'wait_for_transform': 0.2,      # wait up to 0.2 s for the LiDAR's frame
        'subscribe_scan': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_rgbd': with_camera,
        'database_path': LaunchConfiguration('db_path'),
        # Lean: one map update per 1/rate seconds.
        'Rtabmap/DetectionRate': ParameterValue(LaunchConfiguration('rate'), value_type=str),
        'RGBD/LinearUpdate': '0.05',    # skip until moved 5 cm...
        'RGBD/AngularUpdate': '0.05',   # ...or turned about 3 degrees
        'Grid/Sensor': '0',             # draw the map from the LiDAR
        'Grid/CellSize': '0.05',        # 5 cm squares
        'Grid/RangeMin': '0.3',         # skip own body (under 30 cm); the user: scan_filter
        'Grid/RangeMax': '8.0',         # ignore anything past 8 m
        'Grid/RayTracing': 'true',      # clear the floor between us and a wall
        'Grid/Scan2dUnknownSpaceFilled': 'false',  # unseen gaps stay unknown
        'RGBD/ProximityBySpace': 'true',    # close the loop when back at a place
        'RGBD/NeighborLinkRefining': 'true',    # tidy each step with a match
        'Mem/NotLinkedNodesKept': 'false',  # forget spots that matched nothing
        'RGBD/StartAtOrigin': 'true',   # a loaded map starts at the spot where it was started
        'pub_loc_pose_only_when_localizing': True,  # /localization_pose: only when really found
        **ICP,
        # Matching an old place to the live sweep must be strict: a lab has long straight
        # walls that line up with almost any other view. (Odometry keeps 0.2.)
        'Icp/CorrespondenceRatio': '0.5',
        'RGBD/MaxOdomCacheSize': '0',   # accept a found position at once (the manager checks it)
    }
    if with_camera:
        settings.update({
            # 19 Nov: with 1 (LiDAR only) every sight match was rejected: the sweeps were lined up
            # from no guess at all (86% sure, 100+ matching points, "RejectedHypothesis" each
            # time). 2 (picture, then LiDAR) was rejected too: the strict LiDAR check (0.5) fails
            # in a room full of chairs. 0: the matching picture points alone give the position;
            # the map manager checks it against the saved walls (camera_guess.py).
            'Reg/Strategy': '0',
            # 19 Nov, outside EB2: sight matches there score 7-8%, under RTAB-Map's 11% bar, so
            # it seldom even tried (one guess in 3 minutes; the first one was right). More tries;
            # every guess is still checked by the map manager (walls, seen twice).
            'Rtabmap/LoopThr': '0.05',
            'Vis/MinInliers': '10',     # a place is recognised from at least 10 matching points
            'Vis/MaxFeatures': '500',   # at most 500 picture points a frame: lean
            'Kp/MaxFeatures': '400',    # at most 400 words per place for recognising it
        })
    return Node(package='rtabmap_slam', executable='rtabmap', output='screen',
                prefix=LOWER_PRIORITY, parameters=[settings],
                remappings=[('scan', '/scan'), ('odom', '/odom'), ('rgbd_image', '/rgbd_image'),
                            ('map', LIVE_MAP)],
                condition=IfCondition(camera_map) if with_camera
                else UnlessCondition(camera_map))


def generate_launch_description():
    lidar_ws = LaunchConfiguration('lidar_ws')
    camera_map = LaunchConfiguration('camera_map')
    maps_dir = os.path.expanduser('~/.ros/apnvi_maps')
    os.makedirs(maps_dir, exist_ok=True)
    new_db = os.path.join(maps_dir, datetime.now().strftime('map_%Y%m%d_%H%M%S.db'))

    def frame(parent, child, x, y, z, yaw, roll='0'):
        return Node(package='tf2_ros', executable='static_transform_publisher', output='log',
                    arguments=['--x', x, '--y', y, '--z', z, '--yaw', yaw,
                               '--pitch', '0', '--roll', roll,
                               '--frame-id', parent, '--child-frame-id', child])

    nav2_params = os.path.join(get_package_share_directory('apnvi_stage3'), 'config',
                               'nav2_params.yaml')
    odom_and_map = {
        'frame_id': BASE,
        'odom_frame_id': 'odom',
        'approx_sync': False,           # the only input is /scan: nothing to line up in time
        'wait_for_transform': 0.2,      # wait up to 0.2 s for the LiDAR's frame
    }
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value=LIDAR_PORT),
        DeclareLaunchArgument('lidar_ws', default_value=os.path.expanduser('~/lidar_ws')),
        DeclareLaunchArgument('camera_map', default_value='false'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('map_view', default_value='true'),
        DeclareLaunchArgument('nav2', default_value='true'),
        DeclareLaunchArgument('db_path', default_value=new_db),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('lidar_height', default_value='0.77'),
        DeclareLaunchArgument('lidar_x', default_value='-0.70'),   # 13 Nov: 70 cm behind
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='3.14159'),
        DeclareLaunchArgument('lidar_roll', default_value='3.14159'),   # upside down, 15 Nov
        DeclareLaunchArgument('camera_height', default_value='0.86'),
        DeclareLaunchArgument('camera_x', default_value='-0.70'),  # straight above the LiDAR
        DeclareLaunchArgument('camera_y', default_value='0.0'),

        # The LiDAR driver lives in its own workspace: add it to the search path here, so
        # this launch works without sourcing that workspace first.
        AppendEnvironmentVariable('AMENT_PREFIX_PATH',
                                  [lidar_ws, '/install/sllidar_ros2']),

        # Where the sensors sit (the picture the map software needs of the device).
        frame(BASE, LIDAR_FRAME, LaunchConfiguration('lidar_x'), LaunchConfiguration('lidar_y'),
              LaunchConfiguration('lidar_height'), LaunchConfiguration('lidar_yaw'),
              LaunchConfiguration('lidar_roll')),
        frame(BASE, CAMERA_FRAME, LaunchConfiguration('camera_x'),
              LaunchConfiguration('camera_y'), LaunchConfiguration('camera_height'), '0.0'),

        # The LiDAR: 10 sweeps a second on /scan (same settings as the driver's own C1 launch).
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
             output='screen', prefix=LOWER_PRIORITY,
             parameters=[{'channel_type': 'serial',
                          'serial_port': LaunchConfiguration('lidar_port'),
                          'serial_baudrate': 460800, 'frame_id': LIDAR_FRAME,
                          'inverted': False, 'angle_compensate': True,
                          'scan_mode': 'Standard'}],
             remappings=[('scan', '/scan_raw')],
             condition=IfCondition(LaunchConfiguration('lidar'))),

        # The user, the handle and the pole: close points behind the device taken out of the
        # sweep (15 Nov). Everything in front and to the sides is kept. /scan_raw -> /scan.
        Node(package='apnvi_stage3', executable='scan_filter', output='screen',
             prefix=LOWER_PRIORITY),

        # Movement from the LiDAR alone: lines each sweep up with its memory of the room.
        # The memory (16 Nov): a sweep is added to it when the match score is under
        # Odom/ScanKeyFrameThr. The score counts against all 720 beams and about a third never
        # return here (the pole and the person block the back), so it tops out near 0.55. With
        # RTAB-Map's 0.9 every sweep was added, the 2000 points held only the last second, and
        # each sweep was matched against a smear of the last few: turning was lost at every turn
        # (a lap ended 35 degrees off). 0.45 adds a sweep only when the view has changed, and
        # 10000 points give a memory five times longer. 20000 was no more accurate in replays
        # and took twice the time per sweep. Details: results/map-drift/drift-findings-16nov.md
        # Deskewing (16 Nov night): one sweep takes 0.1 s, so in a quick turn the room turns
        # while it is swept and each sweep comes out twisted. In a tight hairpin at the back of
        # ESB117 that twist slid the heading 40 deg (two copies of the room on the map). Each
        # sweep is now straightened with the movement between sweeps (slerp). Replays: that lap
        # 5.7 m / 40 deg -> 0.6 m / 2 deg; other laps and a spin on the spot stay under 0.8 m.
        Node(package='rtabmap_odom', executable='icp_odometry', output='screen',
             prefix=LOWER_PRIORITY,
             parameters=[{**odom_and_map, 'publish_tf': True,
                          'deskewing': True,                     # straighten twisted sweeps
                          'deskewing_slerp': True,               # ...from the last movement
                          'OdomF2M/ScanSubtractRadius': '0.05',  # one point per 5 cm in memory
                          'OdomF2M/ScanMaxSize': '10000',        # keep at most 10000 points
                          'Odom/ScanKeyFrameThr': '0.45',        # add a sweep if the view changed
                          'Odom/ResetCountdown': '1'},           # when lost, start again at once
                         ICP],
             remappings=[('scan', '/scan'), ('odom', '/odom')]),

        # The camera's colour and aligned depth, joined into one message for the map.
        Node(package='rtabmap_sync', executable='rgbd_sync', output='screen',
             prefix=LOWER_PRIORITY,
             parameters=[{'approx_sync': True, 'approx_sync_max_interval': 0.05,
                          'queue_size': 10}],
             remappings=[('rgb/image', CAMERA + '/color/image_raw'),
                         ('depth/image', CAMERA + '/aligned_depth_to_color/image_raw'),
                         ('rgb/camera_info', CAMERA + '/color/camera_info'),
                         ('rgbd_image', '/rgbd_image')],
             condition=IfCondition(camera_map)),

        # The map: adds a spot to the map as the device moves and joins the sweeps together.
        # With camera_map the camera's colour and depth join in: the camera does not move the
        # device (the LiDAR does). It only suggests places it has seen before; the LiDAR sweep
        # decides whether the suggestion is right (a view of the desks looks the same from afar).
        rtabmap(with_camera=False),
        rtabmap(with_camera=True),

        # Saves and loads maps by name, finds the device on a loaded map, owns /map and the
        # frame map (map -> rtabmap_map); says when the device cannot find itself.
        Node(package='apnvi_stage3', executable='map_manager_node', output='screen',
             prefix=LOWER_PRIORITY,
             parameters=[{'db_path': LaunchConfiguration('db_path'),
                          'map': LaunchConfiguration('map')}]),

        # The map as a small picture for the 8081 page. Draws only while the page is open.
        Node(package='apnvi_stage3', executable='map_view_node', output='screen',
             prefix=LOWER_PRIORITY, condition=IfCondition(LaunchConfiguration('map_view'))),

        # Nav2: only the route planner and its obstacle map. Nothing steers.
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', prefix=LOWER_PRIORITY, parameters=[nav2_params],
             condition=IfCondition(LaunchConfiguration('nav2'))),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_planner', output='screen', prefix=LOWER_PRIORITY,
             parameters=[nav2_params], condition=IfCondition(LaunchConfiguration('nav2'))),
    ])
