"""
Save and load maps by name, find the device on a loaded map, and own the map the others see.

Listens: /stage3/command  a JSON line, for example {"cmd": "save_map", "name": "ESB117"}
                          cmd: save_map, load_map, new_map, and the pins (16 Nov):
                            save_place  {"name": "my desk"}  pins where the device stands
                            pin         {"name", "x", "y", "from_x", "from_y", "kind", "sure"}
                                        a thing or a sign at a map point (from Stage 2 and
                                        the labeller); kind: thing or sign
                            remove_pin  {"name"}             rename_pin {"name", "to"}
                            set_entrance {}  marks where the device stands as the room's
                                        entrance (17 Nov): stand in the doorway facing in.
                                        save_place with the name "entrance" (or "door") does
                                        the same and also pins it as a place
                          "speak": true also sends the reply to /voice/say
         /rtabmap/live_map  RTAB-Map's grid (drawn in RTAB-Map's own frame rtabmap_map)
         /odom            to know how far the device has moved since the map was loaded
         /localization_pose  with camera_map:=true, RTAB-Map's guess by sight of where the
                          device is; checked by camera_guess.py before it places.
                          RTAB-Map runs without a namespace, so it is not /rtabmap/... (19 Nov:
                          listening there, the camera was never heard)
Sends:   /map               the map for the 8081 page and Nav2, latched: RTAB-Map's grid while
                          mapping, the clean saved map after a load
         the frame map -> rtabmap_map  (on /tf_static): no shift while mapping; after a load,
                          the shift that puts the LiDAR where its sweep fits the saved walls
         /stage3/reply      a short sentence for the user ("Map saved as ESB117"), spoken later
         /stage3/map_name   the name of the map that is running ("" for a new one), latched
         /stage3/placement  "placed", "searching" or "lost" after a load, latched
         /stage3/position   where the device is on the map (PoseStamped in frame map), 5 a second
         /stage3/pins       the map's pins as JSON, latched: {"map", "pins": [{name, kind, x, y,
                            seen}], "waiting": n}. Only pins that are usable (map_names.py);
                            "waiting" counts things seen once so far
         /voice/say         a reply that should be spoken (commands with "speak": true)
Reads:   /scan, the loaded map file's wall picture, the frames rtabmap_map, odom, base_link
         and laser.
Calls:   RTAB-Map's services backup, load_database, set_mode_localization, set_mode_mapping,
         pause and resume.

How the device finds itself: RTAB-Map is not trusted for this. After a load it believes the
device is at the spot where the map was started, and holds that belief even when the sweep
clearly does not fit there; later it moves its own frame when it thinks it recognises a spot,
often wrongly in this room (16 Nov night: 0.7 m and 16 degrees, standing still). So movement
comes from the movement tracker alone (the frame odom), this node decides where odom sits on
the map, and it moves rtabmap_map (5 times a second) so that RTAB-Map's own jumps cancel out.
Once a second the LiDAR sweep is checked against the saved walls:
  - not placed yet: walk-and-match (placing.py, 17 Nov). The sweeps of the last few metres
    of walking are put together into one wider outline with the tracker, and fitted to the
    map. The whole map is searched (relocalizer.py, about 3 s) when the outline has grown;
    its answer is used when it is sure (fits `sure_fit` and beats every other place by
    `sure_gap`). A room can look the same from two places (ESB117 does, turned round), so
    every good answer is followed as the device moves (Candidates), and the start spot always:
    the room's entrance once it is marked, else the spot where the map was started. A place
    wins only clearly: an average fit of `placed_fit` and `sure_gap` ahead of every other
    place. The start spot no longer wins a tie (17 Nov: started beside the taped spot, a 60%
    fit there placed the device wrong); it wins outright only once the user was asked to go
    there. Not placed after the first checks, it says once "Finding where you are. Keep
    walking."; placed, "Found where you are."
    The fit forgives close things that are not on the map (people, chairs, desk edges): see
    relocalizer.py (16 Nov night; before that the device was never placed between the desks).
  - placed: the position follows the tracker's movement, and the fit is refined in small steps
    (a few cm, a degree) to take out drift; if that does not fit, a wider look around (the
    tracker can jump half a metre standing still). A poor fit alone does not undo it (the
    person pushing, people walking past, moved chairs): after 3 poor fits in a row the map is
    searched, and the device is moved only if the search is sure of a place. After 5 unsure
    searches in a row (18 Nov), and once it has walked 1 m since the last good fit (19 Nov:
    standing still it cannot drift), it is not placed any more and walk-and-match finds it again.
    With the camera on (19 Nov, outside EB2) it never gives up, and only the camera moves it
    (seen twice, one guess the walls fit well, or the camera and the search agreeing): a "sure"
    LiDAR search alone once moved it 9 m to the wrong place mid-trip.
Not placed after `find_after_m` metres or `find_after_s` seconds of moving: it asks the user to
keep walking it around the room, and again after each such stretch (18 Nov, Adel: a blind user
cannot find the entrance).
Pins live in the map's table (map_names.py), next to the saved map; a map with no name yet
keeps them in work/unsaved.json, and saving the map copies them to its own table.
How a save works: RTAB-Map writes a safe copy of the running map (backup); this node copies it
to <maps folder>/named/<name>.db. How a load works: the saved file is copied to
<maps folder>/work/ and RTAB-Map is pointed at the copy, so the saved map is never changed.
RTAB-Map is paused while it changes its map file: told to load a map while it was working on a
sweep it crashed (16 Nov night, 2 of 4 starts with a saved map).

Settings (--ros-args -p name:=value):
  maps_dir      the maps folder, default ~/.ros/apnvi_maps
  db_path       the file RTAB-Map is writing now (the launch file gives it)
  map           a saved map to load when the node starts, default none
  find_after_m  metres of movement before "keep walking" is said again, default 8 (1.5 until
                17 Nov: walking is now how it finds itself)
  find_after_s  seconds of movement-time allowed, default 60 (was 10)
  placed_fit    the fit a followed place needs to win, default 0.7 (0.55 until 17 Nov)
  sure_fit      share a map-wide search answer needs, default 0.75
  sure_gap      how far either must be ahead of every other place, default 0.1 (was 0.08)
"""

import array
import json
import math
import os
import queue
import threading
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import MapGraph
from rtabmap_msgs.srv import LoadDatabase
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from .map_names import (add_pin, add_place, all_pins, copy_file, empty_table, entrance,
                        follow_path, forget_session, tie_pin,
                        is_entrance, load_table, map_paths, merge_tables, remove_pin, rename_pin,
                        save_table, saved_grid, saved_maps, set_entrance, table_path, usable,
                        with_open_floor, work_path)
from .camera_guess import anchors_agree, CameraGuesses
from .placing import Placer
from .relocalizer import combine, frame_offset, inverse, Relocalizer

SERVICE_WAIT_S = 20.0
POSITION_RATE_HZ = 5.0
MOVING_M_PER_S = 0.05           # the device counts as moving above this speed
MAP_FRAME = 'map'               # the map everyone uses: this node owns it
RTABMAP_FRAME = 'rtabmap_map'   # RTAB-Map's own map frame, hung under map by this node
BASE_FRAME = 'base_link'        # the device: on the floor, midway between the wheels
ODOM_FRAME = 'odom'             # the movement tracker's frame: where the device was started
NO_SHIFT = (0.0, 0.0, 0.0)
START_LOAD_TRIES = 5            # the start-up map load is tried this many times (23 Nov)
START_LOAD_GAP_S = 5.0          # ...this long apart
# A service that has not answered by then has lost the request (a load took 1 to 2 s; was 120 s,
# so a lost start-up load cost 2 minutes). 60 s leaves room for saving a big map.
SERVICE_ANSWER_S = 60.0
SETTLE_S = 1.5                  # after pausing RTAB-Map: let it finish the sweep it is on
NUDGE_GAIN = 0.02               # move the map only when the fit gets this much better (no jitter)
RECHECK_AFTER = 3               # poor fits in a row (one a second) before searching the map
# 18 Nov (lab): after drift the one-sweep search kept naming the same right place at 70 to 74 %,
# under sure_fit, so the wrong position stayed. After this many unsure searches in a row (about
# 15 s of poor fits) the device is not placed any more and walk-and-match finds it again.
LOST_AFTER_RECHECKS = 5
LOST_WALK_M = 1.0               # 19 Nov: ...and only after walking this far with a poor fit
GRAPH_SETTLE_S = 5.0            # path messages are ignored this long after a new map or a load
REGRAPH_EVERY_S = 2.0           # move the pins with the path at most this often
MOVING_NOW_S = 3.0              # ...and only while moving (moved within this many seconds)
SAY_FINDING_AFTER = 3           # not placed after this many checks: "Finding where you are"
FINDING = 'Finding where you are. Keep walking.'
FOUND = 'Found where you are.'
KEEP_WALKING = "I still don't know where I am. Keep walking me around the room."
KEEP_WALKING_EVERY_S = 120.0    # 18 Nov (Adel): said every 3 s it was annoying; at most this often
MAX_STEP_M = 0.5                # a tracker jump bigger than this in one message is not walking


def sweep_angles(scan, upside_down):
    """
    Give the sweep as (ranges, angle_min, angle_increment) for a LiDAR the right way up.

    Upside down (rolled 180 deg), a beam at angle a points where a beam at -a would point on a
    LiDAR the right way up with the same heading: so the angles are reversed (15 Nov: without
    this the map manager read the room mirrored and placed the device 3.5 m off, facing back).
    """
    if upside_down:
        return scan.ranges, -scan.angle_min, -scan.angle_increment
    return scan.ranges, scan.angle_min, scan.angle_increment


def near_point(command):
    """Give the map point a console click sent with a pin command, (x, y), or None (by voice)."""
    try:
        point = (float(command['x']), float(command['y']))
    except (KeyError, TypeError, ValueError):
        return None
    return point if all(math.isfinite(v) for v in point) else None


class MapManagerNode(Node):
    """Take commands for saving and loading maps, and watch for the device finding itself."""

    def __init__(self):
        super().__init__('map_manager_node')
        self.declare_parameter('maps_dir', os.path.expanduser('~/.ros/apnvi_maps'))
        self.declare_parameter('db_path', '')
        self.declare_parameter('map', '')
        self.declare_parameter('find_after_m', 8.0)
        self.declare_parameter('find_after_s', 60.0)
        self.declare_parameter('placed_fit', 0.7)
        self.declare_parameter('sure_fit', 0.75)
        self.declare_parameter('sure_gap', 0.1)
        self.maps_dir = self.get_parameter('maps_dir').value
        self.db_path = self.get_parameter('db_path').value
        self.reply_spoken = False               # speak the reply of the map command being done
        self.map_name = ''                      # the saved map that is running, '' if a new one
        self.find_after_m = float(self.get_parameter('find_after_m').value)
        self.find_after_s = float(self.get_parameter('find_after_s').value)
        self.placed_fit = float(self.get_parameter('placed_fit').value)
        self.sure_fit = float(self.get_parameter('sure_fit').value)
        self.sure_gap = float(self.get_parameter('sure_gap').value)

        self.lock = threading.Lock()
        self.commands = queue.Queue()
        self.placing = False                    # loaded a map, still looking for itself
        self.placed_at = 0.0                    # when the device found itself (0: not yet)
        self.moved_m = 0.0                      # metres moved since the map was loaded
        self.moving_s = 0.0                     # seconds of moving since the map was loaded
        self.walked_m = 0.0                     # metres moved since the node started (never reset)
        self.moving_at = -1e9                   # when the device last moved
        self.asked = False                      # already asked to go to the start spot
        self.said_keep_at = -1e9                # when "keep walking" was last said
        self.start_pose = None                  # the loaded map's entrance (x, y, yaw), or None
        self.last_odom = None
        self.scan = None                        # the newest LiDAR sweep
        self.saved_grid = None                  # the loaded map's wall picture, for the search
        self.loaded_at = 0.0
        self.shift = NO_SHIFT                   # map -> rtabmap_map as (x, y, yaw)
        self.anchor = None                      # map -> odom once placed on a loaded map
        self.placer = None                      # walk-and-match while not placed (placing.py)
        self.state = ''                         # what /stage3/placement said last
        self.camera_pose = None                 # RTAB-Map's newest sight guess
        self.guesses = CameraGuesses(placed_fit=self.placed_fit)
        self.searched = None                    # the last recheck's good search answer (anchor)
        self.nodes = {}                         # RTAB-Map's path points {id: (x, y, yaw)}
        self.nodes_since = time.monotonic()     # path messages before this are an old map's
        self.regraph_at = 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reply_pub = self.create_publisher(String, '/stage3/reply', 10)
        self.voice_pub = self.create_publisher(String, '/voice/say', 10)
        self.pins_pub = self.create_publisher(String, '/stage3/pins', latched)
        self.last_pose = None                   # (x, y, yaw) on the map, newest
        self.name_pub = self.create_publisher(String, '/stage3/map_name', latched)
        self.position_pub = self.create_publisher(PoseStamped, '/stage3/position', 10)
        self.placement_pub = self.create_publisher(String, '/stage3/placement', latched)
        grid_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', grid_qos)
        self.frame_pub = StaticTransformBroadcaster(self)
        self.create_subscription(OccupancyGrid, '/rtabmap/live_map', self.on_live_map, grid_qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 5)
        self.create_subscription(String, '/stage3/command', self.on_command, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        # 19 Nov: the path, so pins move with the map when RTAB-Map straightens it
        self.create_subscription(MapGraph, '/mapGraph', self.on_graph, 1)
        # camera_map:=true: RTAB-Map sends this when it recognises a place by sight
        self.create_subscription(PoseWithCovarianceStamped, '/localization_pose',
                                 self.on_camera_pose, 10)
        self.backup = self.create_client(Empty, '/rtabmap/backup')
        self.load_db = self.create_client(LoadDatabase, '/rtabmap/load_database')
        self.localize = self.create_client(Empty, '/rtabmap/set_mode_localization')
        self.mapping = self.create_client(Empty, '/rtabmap/set_mode_mapping')
        self.pause = self.create_client(Empty, '/rtabmap/pause')
        self.resume = self.create_client(Empty, '/rtabmap/resume')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1 / POSITION_RATE_HZ, self.publish_position)
        self.create_timer(1.0, self.check_placing)
        threading.Thread(target=self.placement_loop, daemon=True).start()
        self.name_pub.publish(String(data=''))
        self.set_shift(NO_SHIFT)
        self.forget_unsaved_pins()              # a new run starts a new map: no old pins
        threading.Thread(target=self.worker, daemon=True).start()
        first = self.get_parameter('map').value
        if first:
            # 23 Nov: with the camera on, RTAB-Map was still starting and never answered the
            # first pause; the load gave up after 120 s and the device stayed on a new map.
            # The start-up load is tried again (START_LOAD_TRIES), quietly until the last try.
            self.commands.put({'cmd': 'load_map', 'name': first, 'tries': START_LOAD_TRIES})
        self.get_logger().info('map_manager_node started; saved maps: %s'
                               % (saved_maps(self.maps_dir) or 'none'))

    # ---- what the device is doing ---------------------------------------------------------

    def on_command(self, msg):
        try:
            command = json.loads(msg.data)
            command['cmd']
        except (ValueError, KeyError, TypeError):
            self.say('I did not understand that map command.')
            return
        self.commands.put(command)

    def on_live_map(self, msg):
        """While mapping, pass RTAB-Map's grid on as /map. After a load the saved map is shown."""
        with self.lock:
            showing_saved = self.saved_grid is not None
        if not showing_saved:
            msg.header.frame_id = MAP_FRAME
            self.map_pub.publish(msg)

    def on_graph(self, msg):
        """Keep RTAB-Map's path; while drawing a map, let the pins follow it (worker)."""
        if time.monotonic() - self.nodes_since < GRAPH_SETTLE_S:
            return                      # just after a new map or a load: may be the old path
        nodes = {}
        for ident, pose in zip(msg.poses_id, msg.poses):
            turn = pose.orientation
            nodes[int(ident)] = (pose.position.x, pose.position.y,
                                 math.atan2(2 * (turn.w * turn.z + turn.x * turn.y),
                                            1 - 2 * (turn.y ** 2 + turn.z ** 2)))
        with self.lock:
            self.nodes = nodes
            drawing = not self.placing
        if drawing and time.monotonic() - self.regraph_at >= REGRAPH_EVERY_S:
            self.regraph_at = time.monotonic()
            self.commands.put({'cmd': 'follow_path'})

    def nodes_on_map(self):
        """The path points on the map everyone uses (map, not rtabmap_map)."""
        with self.lock:
            nodes, shift = dict(self.nodes), self.shift
        return {ident: combine(shift, node) for ident, node in nodes.items()}

    def follow_path(self, _command):
        """RTAB-Map moved its path: move the pins tied to it (19 Nov)."""
        table = load_table(self.table_file(), self.map_name)
        moved = follow_path(table, self.nodes_on_map())
        if moved:
            save_table(self.table_file(), table)
            self.publish_pins(table)
            self.get_logger().info('the map was straightened: %d pins moved with it' % moved)

    def tie_to_path(self, pin):
        """While drawing, tie a new or seen-again pin to the newest path point."""
        with self.lock:
            drawing = not self.placing
        nodes = self.nodes_on_map() if drawing else {}
        if not nodes:
            return
        newest = max(nodes)
        tie_pin(pin, newest, nodes[newest])

    def on_scan(self, msg):
        self.scan = msg

    def on_camera_pose(self, msg):
        if self.placing:                # placed too (19 Nov): it confirms or corrects the place
            self.camera_pose = msg
            self.guesses.heard_at = time.monotonic()    # the camera is working

    def on_odom(self, msg):
        position = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_odom is not None:
            step = math.hypot(position.x - self.last_odom[0], position.y - self.last_odom[1])
            seconds = stamp - self.last_odom[2]
            with self.lock:
                if 0 < seconds < 1.0 and step / seconds > MOVING_M_PER_S and step < MAX_STEP_M:
                    self.moved_m += step
                    self.moving_s += seconds
                    self.walked_m += step
                    self.moving_at = time.monotonic()
        self.last_odom = (position.x, position.y, stamp)

    def check_placing(self):
        # 18 Nov (Adel): a blind user cannot find the entrance, so the device does not ask to be
        # taken there; it asks to keep walking, again every find_after_m metres or find_after_s
        # seconds of moving, and places only on a clear fit (the entrance gets no free win).
        with self.lock:
            lost = (self.placing and not self.placed_at
                    and time.monotonic() - self.said_keep_at >= KEEP_WALKING_EVERY_S
                    and (self.moved_m >= self.find_after_m or self.moving_s >= self.find_after_s))
            if lost:
                self.moved_m = self.moving_s = 0.0
                self.said_keep_at = time.monotonic()
        if lost:
            self.set_state('lost')
            self.say(KEEP_WALKING, speak=True)

    # ---- finding itself on a loaded map -----------------------------------------------------

    def placement_loop(self):
        """After a load: check the sweep against the saved walls once a second."""
        finder, built_for, misses, unsure = None, 0.0, 0, 0
        good_at_m = 0.0                 # walked_m when the position last fitted
        while rclpy.ok():
            time.sleep(1.0)
            with self.lock:
                grid, loaded_at = self.saved_grid, self.loaded_at
            if grid is None or self.scan is None:
                continue
            try:
                if finder is None or built_for != loaded_at:
                    finder, built_for, misses, unsure = Relocalizer(*grid), loaded_at, 0, 0
                    self.placer = self.new_placer(finder)
                scan = self.scan
                *seen, upside_down = self.laser_in_tracker(scan)
                seen = tuple(seen)
                points = finder.sweep_points(*sweep_angles(scan, upside_down))
                if not self.placed_at:
                    if not self.try_camera(finder, points, seen):
                        self.search(points, seen, self.laser_on_device(scan))
                    continue
                if self.follow(finder, points, seen):
                    misses = unsure = 0
                    good_at_m = self.walked_m
                    self.camera_pose = None
                    continue
                if self.camera_check(finder, points, seen):
                    misses = unsure = 0         # the camera says it is where it is shown
                    good_at_m = self.walked_m
                    continue
                misses += 1
                if misses >= RECHECK_AFTER:
                    misses = 0
                    unsure = 0 if self.recheck(finder, points, seen) else unsure + 1
                    if not unsure:
                        good_at_m = self.walked_m
                # 19 Nov (Adel): standing still in the middle of the room it lost its place: the
                # sweep there is mostly chairs and desks. Standing still cannot drift, so it gives
                # up only after walking LOST_WALK_M with a poor fit. Later on 19 Nov it still gave
                # up while standing (the metre was walked before the stop): now it decides only
                # while moving, when walking can find it again anyway.
                # Later still (outside EB2): in the open the walls fit poorly everywhere, so it
                # gave up a right place after a few metres. With the camera on it never gives up:
                # the tracker carries the place, and only evidence moves it (recheck,
                # camera_check). Without the camera the old rule stays.
                if (unsure >= LOST_AFTER_RECHECKS and not self.camera_on()
                        and self.walked_m - good_at_m >= LOST_WALK_M
                        and time.monotonic() - self.moving_at <= MOVING_NOW_S):
                    unsure = 0
                    self.unplace(finder)
            except TransformException:
                continue                        # the frames are not there yet
            except Exception as error:          # keep looking whatever happens
                self.get_logger().warning('placement step failed: %s' % error)

    def laser_in_tracker(self, scan):
        """
        Give (x, y, yaw, upside_down) of the LiDAR in the tracker's frame at the sweep's time.

        upside_down: the LiDAR's own "up" points down (on the device it is mounted upside down,
        15 Nov). Its sweep is then a mirror image and must be read with the angles reversed.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                ODOM_FRAME, scan.header.frame_id, rclpy.time.Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=0.2))
        except TransformException:
            transform = self.tf_buffer.lookup_transform(ODOM_FRAME, scan.header.frame_id,
                                                        rclpy.time.Time())
        rotation = transform.transform.rotation
        yaw = math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                         1 - 2 * (rotation.y ** 2 + rotation.z ** 2))
        up_z = 1 - 2 * (rotation.x ** 2 + rotation.y ** 2)     # the LiDAR's up, seen from the map
        return (transform.transform.translation.x, transform.transform.translation.y, yaw,
                up_z < 0)

    def laser_on_device(self, scan):
        """Give (x, y, yaw) of the LiDAR on the device: its place on the map at the start spot."""
        transform = self.tf_buffer.lookup_transform(BASE_FRAME, scan.header.frame_id,
                                                    rclpy.time.Time())
        rotation = transform.transform.rotation
        return (transform.transform.translation.x, transform.transform.translation.y,
                math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                           1 - 2 * (rotation.y ** 2 + rotation.z ** 2)))

    def search(self, points, seen, laser_on_device):
        """Not placed yet: one walk-and-match check (placing.py); place on a clear win."""
        self.set_state('searching')
        placer = self.placer
        won = placer.step(points, seen, laser_on_device, asked=self.asked)
        self.get_logger().info(placer.note)
        if won is not None and not self.guesses.lets_lidar_place(won[0], time.monotonic()):
            self.get_logger().info('the walls point to a place the camera does not: waiting')
            won = None
        if won is not None:
            self.place_at(won[0])
            self.mark_placed(won[1])
            self.say(FOUND, speak=True)
        elif placer.checks == SAY_FINDING_AFTER:
            self.say(FINDING, speak=True)

    def try_camera(self, finder, points, seen):
        """
        Not placed yet: check RTAB-Map's newest sight guess (camera_guess.py). True if placed.

        The guess is where the device was on the saved map at the guess's time. With the
        tracker's own position at that time it gives the anchor (odom on the map); the LiDAR
        sweep now is scored against the saved walls there.
        """
        found = self.camera_anchor(finder, points, seen)
        if found is None:
            return False
        anchor, fit, on_map = found
        why = self.guesses.add(anchor, fit, time.monotonic())
        self.get_logger().info('%s at (%.1f, %.1f)' % (self.guesses.note, on_map[0], on_map[1]))
        if why is None and fit >= self.guesses.min_fit:
            # 19 Nov: one guess, and the map-wide LiDAR search names the same place: two senses
            searched = finder.find(points)
            if searched is not None and anchors_agree(
                    frame_offset((searched.x, searched.y, searched.yaw), seen), anchor):
                why = 'the walls search agrees'
        if why is None:
            return False
        self.place_at(anchor)
        self.mark_placed('camera: ' + why)
        self.say(FOUND, speak=True)
        return True

    def camera_check(self, finder, points, seen):
        """
        Placed, but the sweep fits poorly (19 Nov: lost its place in the middle of ESB117).

        True if the camera's newest guess agrees with where the device is shown: it stays.
        A guess somewhere else is collected; seen twice there (camera_guess.py), the device is
        moved there and this also returns True.
        """
        found = self.camera_anchor(finder, points, seen)
        if found is None:
            return False
        anchor, fit, on_map = found
        if anchors_agree(anchor, self.anchor):
            self.get_logger().info('poor wall fit, but the camera agrees at (%.1f, %.1f)'
                                   % on_map[:2])
            return True
        why = self.guesses.add(anchor, fit, time.monotonic())
        self.get_logger().info('placed elsewhere; %s at (%.1f, %.1f)'
                               % (self.guesses.note, on_map[0], on_map[1]))
        if why is None:
            return False
        self.place_at(anchor)
        self.get_logger().info('moved to where the camera says the device is (%s)' % why)
        return True

    def camera_anchor(self, finder, points, seen):
        """Turn RTAB-Map's newest sight guess into (anchor, wall fit, (x, y) on map), or None."""
        msg, self.camera_pose = self.camera_pose, None
        if msg is None:
            return None
        pose = msg.pose.pose
        turn = pose.orientation
        on_map = (pose.position.x, pose.position.y,
                  math.atan2(2 * (turn.w * turn.z + turn.x * turn.y),
                             1 - 2 * (turn.y ** 2 + turn.z ** 2)))
        try:
            found = self.tf_buffer.lookup_transform(ODOM_FRAME, BASE_FRAME,
                                                    rclpy.time.Time.from_msg(msg.header.stamp),
                                                    timeout=Duration(seconds=0.2))
        except TransformException:
            found = self.tf_buffer.lookup_transform(ODOM_FRAME, BASE_FRAME, rclpy.time.Time())
        turn = found.transform.rotation
        in_odom = (found.transform.translation.x, found.transform.translation.y,
                   math.atan2(2 * (turn.w * turn.z + turn.x * turn.y),
                              1 - 2 * (turn.y ** 2 + turn.z ** 2)))
        anchor = combine(on_map, inverse(in_odom))
        x, y, yaw, fit = finder.refine(points, *combine(anchor, seen))   # a small tidy-up
        anchor = frame_offset((x, y, yaw), seen)
        self.guesses.heard(anchor, time.monotonic())
        return anchor, float(fit), on_map

    def follow(self, finder, points, seen):
        """Placed: refine the fit a little. Returns True when the sweep fits where it is shown."""
        shown = combine(self.anchor, seen)
        shown_fit = float(finder.score(points, *shown)[0])
        x, y, yaw, fit = finder.refine(points, *shown)
        if fit < self.placed_fit:
            x, y, yaw, fit = finder.refine_wide(points, *shown)     # the tracker may have jumped
            if fit < self.placed_fit:
                return False
        if fit - shown_fit >= NUDGE_GAIN:
            self.place_at(frame_offset((x, y, yaw), seen))
        return True

    def recheck(self, finder, points, seen):
        """Placed, but the sweep fits poorly: move only if the search is sure. True if moved."""
        found = finder.find(points)
        shown = combine(self.anchor, seen)
        self.get_logger().info('poor fit where shown (%.1f, %.1f); search: %s'
                               % (shown[0], shown[1], found))
        if found is None:
            return False
        anchor, now = frame_offset((found.x, found.y, found.yaw), seen), time.monotonic()
        if anchors_agree(anchor, self.anchor):
            self.place_at(anchor)       # the search names where it is shown: it stays
            return True
        if self.guesses.agrees(anchor, now):
            # 19 Nov: the search named the right place at 82% (runner-up 80%, not sure) while
            # the camera said the same; two different senses agreeing is enough
            self.place_at(anchor)
            self.get_logger().info('moved to where the search and the camera agree')
            return True
        if self.camera_on():
            # 19 Nov, outside EB2: mid-trip a "sure" search (76% against 61%) moved the device
            # 9 m to the wrong place; the guide then went round in circles. With the camera on,
            # the LiDAR alone never moves a placed device: only the camera (with or without it).
            return False
        if (found.score >= self.sure_fit and found.score - found.runner_up >= self.sure_gap
                and self.guesses.lets_lidar_place(anchor, now)):
            self.place_at(anchor)
            self.get_logger().info('moved to where the search is sure the device is')
            return True
        # 19 Nov: the same place named twice in a row with a good fit is enough (the 18 Nov
        # drift: the search kept naming the right place at 70 to 74 %, under sure_fit)
        before, self.searched = self.searched, (anchor if found.score >= self.placed_fit else None)
        if (self.searched is not None and before is not None and anchors_agree(before, anchor)
                and self.guesses.lets_lidar_place(anchor, now)):
            self.searched = None
            self.place_at(anchor)
            self.get_logger().info('moved to where the search named it twice')
            return True
        return False

    def camera_on(self):
        """True when RTAB-Map uses the camera (camera_map:=true): it sends /localization_pose."""
        return self.count_publishers('/localization_pose') > 0

    def new_placer(self, finder):
        """Make a Placer for the loaded map: the entrance and (23 Nov) the map start spot."""
        door = self.start_pose is not None
        return Placer(finder, self.start_pose or NO_SHIFT, self.placed_fit, self.sure_fit,
                      self.sure_gap, entrance=door, also_start=NO_SHIFT if door else None)

    def unplace(self, finder):
        """The position fits poorly and no search is sure: find the device again by walking."""
        with self.lock:
            self.placed_at, self.asked = 0.0, False
            self.moved_m = self.moving_s = 0.0
        self.placer = self.new_placer(finder)
        self.guesses.forget()
        self.get_logger().info('position fits poorly and no search is sure: finding it again')
        self.set_state('searching')

    def place_at(self, anchor):
        """Put the tracker's frame (odom) on the map with this (x, y, yaw), and hold it there."""
        self.anchor = anchor
        self.hold_anchor()

    def hold_anchor(self):
        """Move rtabmap_map so that odom stays where it was placed, whatever RTAB-Map does."""
        anchor = self.anchor
        if anchor is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(RTABMAP_FRAME, ODOM_FRAME,
                                                        rclpy.time.Time())
        except TransformException:
            return
        turn = transform.transform.rotation
        rtabmap_to_odom = (transform.transform.translation.x, transform.transform.translation.y,
                           math.atan2(2 * (turn.w * turn.z + turn.x * turn.y),
                                      1 - 2 * (turn.y ** 2 + turn.z ** 2)))
        wanted = combine(anchor, inverse(rtabmap_to_odom))
        if (math.hypot(wanted[0] - self.shift[0], wanted[1] - self.shift[1]) > 0.005
                or abs(math.atan2(math.sin(wanted[2] - self.shift[2]),
                                  math.cos(wanted[2] - self.shift[2]))) > 0.002):
            self.set_shift(wanted)

    def set_shift(self, shift):
        """Put RTAB-Map's frame on the map with this (x, y, yaw) shift."""
        self.shift = shift
        frame = TransformStamped()
        frame.header.stamp = self.get_clock().now().to_msg()
        frame.header.frame_id = MAP_FRAME
        frame.child_frame_id = RTABMAP_FRAME
        frame.transform.translation.x, frame.transform.translation.y = shift[0], shift[1]
        frame.transform.rotation.z = math.sin(shift[2] / 2)
        frame.transform.rotation.w = math.cos(shift[2] / 2)
        self.frame_pub.sendTransform(frame)

    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.placement_pub.publish(String(data=state))

    def mark_placed(self, why):
        with self.lock:
            if self.placed_at:
                return
            self.placed_at = time.time()
        self.get_logger().info('placed on the map: %s, map -> rtabmap_map x %.2f y %.2f '
                               'yaw %.0f deg' % (why, self.shift[0], self.shift[1],
                                                 math.degrees(self.shift[2])))
        self.set_state('placed')

    def publish_position(self):
        self.hold_anchor()
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.orientation = transform.transform.rotation
        self.position_pub.publish(pose)
        turn = transform.transform.rotation
        yaw = math.atan2(2 * (turn.w * turn.z + turn.x * turn.y),
                         1 - 2 * (turn.y * turn.y + turn.z * turn.z))
        self.last_pose = (pose.pose.position.x, pose.pose.position.y, yaw)

    # ---- the commands -----------------------------------------------------------------------

    def say(self, words, speak=False):
        self.get_logger().info('reply: ' + words)
        self.reply_pub.publish(String(data=words))
        if speak:
            self.voice_pub.publish(String(data=words))

    def worker(self):
        """Do the commands one at a time, in the order they came."""
        handlers = {'save_map': self.save_map, 'load_map': self.load_map,
                    'new_map': self.new_map}
        pin_handlers = {'save_place': self.save_place, 'pin': self.pin,
                        'remove_pin': self.remove_pin, 'rename_pin': self.rename_pin,
                        'set_entrance': self.set_entrance, 'follow_path': self.follow_path}
        while rclpy.ok():
            command = self.commands.get()
            handler = handlers.get(command['cmd'])
            pin_handler = pin_handlers.get(command['cmd'])
            if handler is None and pin_handler is None:
                continue                # a command for another node
            try:
                if pin_handler is not None:
                    pin_handler(command)
                    continue
                # 18 Nov evening: "Map saved as ..." and "Loaded ..." were never spoken
                self.reply_spoken = bool(command.get('speak'))
                handler(str(command.get('name', '')).strip())
            except Exception as error:      # keep the worker alive whatever happens
                self.get_logger().error('%s failed: %s' % (command['cmd'], error))
                tries = int(command.get('tries', 0))
                if tries > 1:
                    self.get_logger().info('trying %s again in %.0f s (%d tries left)'
                                           % (command['cmd'], START_LOAD_GAP_S, tries - 1))
                    time.sleep(START_LOAD_GAP_S)
                    self.commands.put(dict(command, tries=tries - 1))
                    continue
                self.say('Sorry, that did not work.')

    def call(self, client, request):
        """Call a service and wait for the answer (the worker thread waits, the node does not)."""
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            raise RuntimeError('service %s is not there' % client.srv_name)
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=SERVICE_ANSWER_S):
            raise RuntimeError('service %s did not answer' % client.srv_name)
        return future.result()

    def change_map_file(self, request, mode):
        """Point RTAB-Map at another map file and set its mode, with RTAB-Map paused meanwhile."""
        self.call(self.pause, Empty.Request())
        time.sleep(SETTLE_S)
        try:
            self.call(self.load_db, request)
            self.call(mode, Empty.Request())
        finally:
            self.call(self.resume, Empty.Request())

    # ---- pins: places, signs and things on the map --------------------------------------

    def table_file(self):
        return table_path(self.maps_dir, self.map_name)

    def forget_unsaved_pins(self):
        """Empty the table of the map that has no name yet, and tell the listeners."""
        try:
            os.remove(table_path(self.maps_dir, ''))
        except OSError:
            pass
        forget_session(table_path(self.maps_dir, ''))
        self.publish_pins()

    def publish_pins(self, table=None):
        if table is None:
            table = load_table(self.table_file(), self.map_name)
        waiting = sum(1 for group in ('signs', 'objects') for pin in table[group]
                      if not usable(pin))
        self.pins_pub.publish(String(data=json.dumps(
            {'map': self.map_name, 'pins': all_pins(table), 'waiting': waiting})))

    def save_place(self, command):
        """Pin the spot where the device stands, under the name the user gave."""
        name, speak = str(command.get('name', '')).strip(), bool(command.get('speak'))
        if not name:
            self.say('Please say a name for this place.', speak)
            return
        with self.lock:
            unplaced = self.placing and not self.placed_at
        if self.last_pose is None or unplaced:
            self.say("I don't know where I am on the map yet.", speak)
            return
        table = load_table(self.table_file(), self.map_name)
        add_place(table, name, *self.last_pose)
        if is_entrance(name):
            set_entrance(table, *self.last_pose)
        save_table(self.table_file(), table)
        self.publish_pins(table)
        self.say('Saved this place as %s%s' % (
            name, ', the entrance of the room' if is_entrance(name) else ''), speak)

    def set_entrance(self, command):
        """Mark where the device stands (in the doorway, facing in) as the room's entrance."""
        speak = bool(command.get('speak'))
        with self.lock:
            unplaced = self.placing and not self.placed_at
        if self.last_pose is None or unplaced:
            self.say("I don't know where I am on the map yet.", speak)
            return
        table = load_table(self.table_file(), self.map_name)
        set_entrance(table, *self.last_pose)
        save_table(self.table_file(), table)
        self.say('This is the entrance of the room now', speak)

    def pin(self, command):
        """Pin a thing or a sign that Stage 2 or the labeller located on the map."""
        try:
            name = str(command['name']).strip()
            x, y = float(command['x']), float(command['y'])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning('bad pin command: %s' % command)
            return
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        from_x, from_y = command.get('from_x'), command.get('from_y')
        table = load_table(self.table_file(), self.map_name)
        pin = add_pin(table, name, x, y,
                      None if from_x is None else float(from_x),
                      None if from_y is None else float(from_y),
                      kind=str(command.get('kind', 'thing')), sure=bool(command.get('sure')),
                      movable=bool(command.get('movable')))
        if pin is None:
            return
        self.tie_to_path(pin)
        save_table(self.table_file(), table)
        self.publish_pins(table)
        self.get_logger().info('pin: %s at (%.2f, %.2f), seen %d%s' % (
            pin.get('name'), pin['x'], pin['y'], pin.get('seen', 1),
            ', sure' if pin.get('sure') else ''))

    def remove_pin(self, command):
        name, speak = str(command.get('name', '')).strip(), bool(command.get('speak'))
        table = load_table(self.table_file(), self.map_name)
        removed = remove_pin(table, name, near_point(command))
        if removed:
            save_table(self.table_file(), table)
            self.publish_pins(table)
        self.say('Removed %s from the map' % name if removed
                 else "There is no %s on the map" % name, speak)

    def rename_pin(self, command):
        name, new = str(command.get('name', '')).strip(), str(command.get('to', '')).strip()
        table = load_table(self.table_file(), self.map_name)
        renamed = rename_pin(table, name, new, near_point(command))
        if renamed:
            save_table(self.table_file(), table)
            self.publish_pins(table)
        self.say('%s is now called %s' % (name, new) if renamed
                 else "There is no %s on the map" % name, bool(command.get('speak')))

    # ---- the maps ---------------------------------------------------------------------------

    def save_map(self, name):
        if not name:
            self.say('Please say a name for the map.', self.reply_spoken)
            return
        old_table = load_table(self.table_file(), self.map_name)   # the pins made so far
        database, _table = map_paths(self.maps_dir, name)
        live = self.db_path
        self.call(self.backup, Empty.Request())
        backup = live + '.back'
        for _ in range(50):             # the copy is finished when its size stops changing
            first = os.path.getsize(backup) if os.path.exists(backup) else -1
            time.sleep(0.2)
            if first > 0 and first == os.path.getsize(backup):
                break
        copy_file(backup, database)
        table = merge_tables(empty_table(name), old_table)       # the pins go with the map
        save_table(table_path(self.maps_dir, name), table)
        self.map_name = name
        self.name_pub.publish(String(data=name))
        self.publish_pins(table)
        self.say('Map saved as %s' % name, self.reply_spoken)

    def load_map(self, name):
        database, _table = map_paths(self.maps_dir, name)
        if not os.path.exists(database):
            self.say("I don't have a map called %s." % name, self.reply_spoken)
            return
        work = work_path(self.maps_dir, name)
        copy_file(database, work)
        request = LoadDatabase.Request()
        request.database_path = work
        request.clear = False
        self.change_map_file(request, self.localize)
        self.db_path = work
        grid = saved_grid(work)
        forget_session(table_path(self.maps_dir, name))     # movable things: this session only
        table = load_table(table_path(self.maps_dir, name), name)
        door = entrance(table)
        route_grid = with_open_floor(grid, table)   # 18 Nov: placing keeps the map as saved
        with self.lock:
            self.map_name = name
            self.placing, self.placed_at, self.asked = True, 0.0, False
            self.moved_m = self.moving_s = 0.0
            self.saved_grid, self.loaded_at = grid, time.time()
            self.start_pose = door
            self.anchor = None
            self.camera_pose = None
        self.guesses.forget(new_map=True)
        self.searched = None
        # (19 Nov afternoon: making the LiDAR wait for the camera after a load cost 2 minutes
        # outside: the walls were sure and right after 8 s. A sure LiDAR win places again; only
        # a camera guess that disagrees holds it back.)
        with self.lock:
            self.nodes, self.nodes_since = {}, time.monotonic()
        self.set_shift(NO_SHIFT)
        if grid is None:
            self.get_logger().warning('%s has no finished map picture: cannot place itself' % name)
        else:
            self.map_pub.publish(grid_message(route_grid, self.get_clock().now().to_msg()))
        self.set_state('searching')
        self.name_pub.publish(String(data=name))
        self.publish_pins()
        no_door = ' (no entrance marked: placing starts from where the map was started)'
        self.say('Loaded %s%s' % (name, '' if door else no_door), self.reply_spoken)

    def new_map(self, _name):
        """Start an empty map (RTAB-Map starts a fresh file) and draw again."""
        fresh = os.path.join(self.maps_dir, time.strftime('map_%Y%m%d_%H%M%S.db'))
        request = LoadDatabase.Request()
        request.database_path = fresh
        request.clear = True
        self.change_map_file(request, self.mapping)
        self.db_path = fresh
        with self.lock:
            self.map_name, self.placing = '', False
            self.saved_grid = None
            self.anchor = None
        self.set_shift(NO_SHIFT)
        with self.lock:
            self.nodes, self.nodes_since = {}, time.monotonic()
        self.set_state('')
        self.name_pub.publish(String(data=''))
        self.forget_unsaved_pins()
        self.say('Started a new map.', self.reply_spoken)


def grid_message(grid, stamp):
    """Make /map from a saved wall picture: (rows of cells, resolution, (x, y) of the corner)."""
    cells, resolution, origin = grid
    message = OccupancyGrid()
    message.header.frame_id = MAP_FRAME
    message.header.stamp = stamp
    message.info.map_load_time = stamp
    message.info.resolution = float(resolution)
    message.info.height, message.info.width = cells.shape
    message.info.origin.position.x, message.info.origin.position.y = map(float, origin)
    message.info.origin.orientation.w = 1.0
    message.data = array.array('b', cells.astype('int8').tobytes())
    return message


def main(args=None):
    rclpy.init(args=args)
    node = MapManagerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
