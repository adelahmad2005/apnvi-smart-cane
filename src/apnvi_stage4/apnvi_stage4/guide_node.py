"""
guide_node: takes the user to a goal with Nav2, and stops when asked or on arrival.

How a trip goes: a goal comes in (a map point or a saved place name), the route is planned
by Stage 3's Nav2 planner and followed by Nav2's Regulated Pure Pursuit follower (Stage 4's
controller_server), whose speeds go through the hazard gate to the drive link. The route is
planned again every second, so it redraws when someone steps in (Stage 3 block 6).
The decisions themselves are in guide.py.

A thing that is not on the map (16 Nov): "take me to the water bottle" first looks in the
map's table (places, signs, things: places.py). If it is not there, Stage 2 is asked on
/stage2/locate to find it in the camera picture now; its answer on /stage2/located is either
a map point (the trip starts) or the reason it could not be placed (a sentence says why).

Listens: /stage2/request    String  JSON from Stage 2 after each question:
                                    {"kind": "guide", "place": "restroom"},
                                    {"kind": "follow_up"} or {"kind": "other"}
         /stage4/goal       String  for testing: a place name, "x y" on the map, or "cancel"
         /goal_pose         PoseStamped  a point clicked on the map (RViz "2D Goal Pose")
         /stage3/position   PoseStamped  where the device is on the map (Stage 3)
         /stage3/map_name   String  the running map, for looking up places (Stage 3)
         /stage3/placement  String  placed / searching / lost (Stage 3, latched): a trip
                                    waits while the device has not found itself (17 Nov)
         /grip/red          Bool    the talk button (from the drive link)
         /button/pressed    Bool    the old talk button on GPIO 23 (Stage 1's ESP32)
         /grip/green        Bool    the emergency stop
         /stage2/located    String  JSON from Stage 2: the answer to /stage2/locate
         /stage4/hazard     Bool    forward driving is blocked (from the hazard gate), latched
         for getting unstuck (unstuck.py):
         /hazard/level, /hazard/distance  Stage 1 (quiet = it never moves)
         /scan, /camera/obstacle_scan     LaserScan  what the LiDAR and the camera see
         /odom              Odometry  how far it has turned and driven
         /stage4/stops      String  the master's stops (any stop ends a try at once)
         the frames base_link -> laser and base_link -> camera_link
Sends:   /voice/say         String  sentences for Stage 2's voice ("You have arrived"...)
         /stage4/guide_state String JSON for the 8081 page: state (idle, guiding, paused),
                                    goal, distance_m, paused_by, route_ok, unstuck (the
                                    phase of a try: off, announce, turn, gap, settle, drive);
                                    twice a second
         /stage4/route      Path    the route being followed (Stage 3 block 6)
         /cmd_vel_nav       Twist   one zero speed whenever the route is cancelled; while
                                    getting unstuck, its turn and short drive (through the
                                    hazard gate like the route follower's)
         /stage2/locate     String  JSON {"name", "id"}: find this thing in the picture now
Uses:    Nav2 actions compute_path_to_pose (planner_server) and follow_path (controller_server)

No route (16 Nov): the camera's marks stay on the planners' obstacle maps until the camera
sees that floor empty again, so a thing that was seen close up and then taken away can block a
route for good. When no route has been found for a few seconds, the node says so and empties
both planners' obstacle maps (the walls come back from the saved map and the LiDAR within
a second), then keeps trying, emptying them again every 10 s while there is still no route.

Getting unstuck (17 Nov, unstuck.py): stopped by the camera for 4 s with nothing the LiDAR sees
straight ahead, the device says "Finding a way round. Turning left", turns on the spot until the
way ahead is clear, drives about 0.5 m and hands back to the route. While it does, the route
follower and the re-planning are held. It also does the talking about being blocked
("Something is in the way. Waiting", "The way is still blocked. Turn me a little...").
Switch it off without a restart: ros2 param set /guide_node unstuck false.

Settings: maps_dir (~/.ros/apnvi_maps), arrived_m (0.5), replan_s (1.0), stand_off_m (0.8:
          how far in front of a thing the trip ends),
          planner_id (GridBased), controller_id (FollowPath), goal_checker_id
          (general_goal_checker), camera_to_front_m (0.83), base_frame (base_link),
          unstuck (true), and unstuck_<name> for each number in unstuck.UnstuckSettings
          (unstuck_wait_s 4.0, unstuck_turn_rps 0.6, unstuck_drive_m 0.5, ...).
"""

from dataclasses import fields
import json
import math
import os
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from .free_mode import FreeSettings, quaternion_matrix, sweep_points
from .guide import ARRIVED_M, Goal, Guide, move_stop_point, NO_WAY
from .places import clean_name, find_goal, STAND_OFF_M, thing_goal
from .reach import fits_point, reachable_point
from .rules import CAMERA_TO_FRONT_M, from_front, HAZARD_LEVELS, LEVEL_TIMEOUT_S
from .unstuck import Seen, Unstuck, UnstuckSettings

LOCATE_TIMEOUT_S = 15.0         # no answer from Stage 2 this long after asking: say so
RECLEAR_S = 10.0                # still no route after an emptying: empty the obstacle maps again
FOLLOW_CLEAR_S = 3.0            # the follower gave up: empty the obstacle maps, at most this often
FRESH_S = 0.5                   # a sweep or a movement reading older than this is not used


def yaw_of(q):
    """Give the heading (radians, anticlockwise from x) of a quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Eyes:
    """What the LiDAR and the camera's low sweep see, in the device's frame (as free mode)."""

    def __init__(self, node, base_frame, max_range_m):
        self.node, self.base_frame, self.max_range_m = node, base_frame, max_range_m
        self.sweeps = {}                # 'lidar' / 'camera' -> (time, message)
        self.frames = {}                # frame name -> (rotation 3 x 3, shift)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        node.create_subscription(LaserScan, '/scan', lambda m: self.keep('lidar', m),
                                 qos_profile_sensor_data)
        node.create_subscription(LaserScan, '/camera/obstacle_scan',
                                 lambda m: self.keep('camera', m), qos_profile_sensor_data)

    def keep(self, name, msg):
        self.sweeps[name] = (time.monotonic(), msg)

    def frame(self, name):
        if name not in self.frames:
            try:
                found = self.tf_buffer.lookup_transform(self.base_frame, name, rclpy.time.Time())
            except TransformException:
                return None
            turn, move = found.transform.rotation, found.transform.translation
            self.frames[name] = (quaternion_matrix(turn.x, turn.y, turn.z, turn.w),
                                 (move.x, move.y, move.z))
        return self.frames[name]

    def points(self, now, names=('lidar', 'camera')):
        """Give the points seen (N x 2), or None if the LiDAR sweep is old or not placed."""
        seen = []
        for name in names:
            entry = self.sweeps.get(name)
            frame = None if entry is None or now - entry[0] > FRESH_S \
                else self.frame(entry[1].header.frame_id)
            if frame is None:
                if name == 'lidar':
                    return None
                continue
            sweep = entry[1]
            seen.append(sweep_points(sweep.ranges, sweep.angle_min, sweep.angle_increment,
                                     frame[0], frame[1], self.max_range_m))
        return np.vstack(seen) if seen else np.zeros((0, 2))


class GuideNode(Node):
    """Run guide.py's decisions against Nav2, the buttons and the voice."""

    def __init__(self):
        super().__init__('guide_node')
        self.declare_parameter('maps_dir', os.path.expanduser('~/.ros/apnvi_maps'))
        self.declare_parameter('arrived_m', ARRIVED_M)
        self.declare_parameter('replan_s', 1.0)
        self.declare_parameter('stand_off_m', STAND_OFF_M)
        self.stand_off_m = float(self.get_parameter('stand_off_m').value)
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'general_goal_checker')
        self.maps_dir = self.get_parameter('maps_dir').value
        self.guide = Guide(float(self.get_parameter('arrived_m').value))
        self.lock = threading.RLock()
        self.map_name = ''
        self.red = self.button = self.green = False
        self.trip = 0                   # goes up on every cancel: old answers are ignored
        self.planning = False
        self.follow_handle = None
        self.route_ok = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.say_pub = self.create_publisher(String, '/voice/say', 10)
        self.state_pub = self.create_publisher(String, '/stage4/guide_state', latched)
        self.route_pub = self.create_publisher(Path, '/stage4/route', latched)
        self.zero_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.locate_pub = self.create_publisher(String, '/stage2/locate', 10)
        self.locating = None            # (id, name, asked at) while Stage 2 looks for a thing
        self.locate_no = 0
        self.create_subscription(String, '/stage2/located', self.on_located, 10)
        self.blocked = False
        self.clearers = [
            self.create_client(ClearEntireCostmap,
                               '/global_costmap/clear_entirely_global_costmap'),
            self.create_client(ClearEntireCostmap,
                               '/local_costmap/clear_entirely_local_costmap')]
        self.cleared_at = 0.0           # when the obstacle maps were last emptied
        self.create_subscription(Bool, '/stage4/hazard', self.on_blocked, latched)
        self.costmap = None             # the planner's map, to move a stop point it cannot reach
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, 'costmap', m), latched)
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.follower = ActionClient(self, FollowPath, 'follow_path')
        self.create_subscription(String, '/stage2/request', self.on_request, 10)
        self.create_subscription(String, '/stage4/goal', self.on_goal_text, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, 10)
        self.create_subscription(PoseStamped, '/stage3/position', self.on_position, 10)
        self.create_subscription(String, '/stage3/map_name', self.on_map_name, latched)
        self.create_subscription(String, '/stage3/placement', self.on_placement, latched)
        self.create_subscription(Bool, '/grip/red', self.on_red, latched)
        self.create_subscription(Bool, '/button/pressed', self.on_button, 10)
        self.create_subscription(Bool, '/grip/green', self.on_green, latched)
        self.create_timer(float(self.get_parameter('replan_s').value), self.on_tick)
        self.create_timer(0.5, self.publish_state)
        self.start_unstuck(latched)
        self.publish_state()

    def start_unstuck(self, latched):
        """Set up getting unstuck (unstuck.py): its settings, sensors and 10 Hz step."""
        self.declare_parameter('unstuck', True)
        self.declare_parameter('camera_to_front_m', CAMERA_TO_FRONT_M)
        self.declare_parameter('base_frame', 'base_link')
        self.camera_to_front_m = float(self.get_parameter('camera_to_front_m').value)
        defaults = UnstuckSettings()
        numbers = {}
        for field in fields(UnstuckSettings):
            value = getattr(defaults, field.name)
            self.declare_parameter('unstuck_' + field.name, value)
            numbers[field.name] = type(value)(self.get_parameter('unstuck_' + field.name).value)
        lanes = FreeSettings()
        self.unstuck = Unstuck(UnstuckSettings(**numbers), lanes)
        self.eyes = Eyes(self, self.get_parameter('base_frame').value, lanes.max_range_m)
        self.level = self.distance = None
        self.level_at = self.distance_at = self.odom_at = 0.0
        self.odom = None                # (x, y, yaw) from the movement tracking
        self.stops = []
        self.unstuck_phase = 'off'
        self.create_subscription(String, '/hazard/level', self.on_level, 10)
        self.create_subscription(Float32, '/hazard/distance', self.on_distance, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(String, '/stage4/stops', self.on_stops, latched)
        self.create_timer(0.1, self.on_unstuck_tick)

    # ---- inputs ---------------------------------------------------------------------------

    def on_request(self, msg):
        try:
            request = json.loads(msg.data)
            kind = request['kind']
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn('bad request: %r' % msg.data)
            return
        if kind != 'guide':
            self.locating = None                # another question came: forget the look
        if kind == 'guide':
            self.go_to_text(request.get('place', ''))
        elif kind == 'follow_up':
            self.act(self.guide.follow_up(self.green))
        else:
            self.act(self.guide.other_question())

    def on_goal_text(self, msg):
        text = msg.data.strip()
        if text.lower() in ('cancel', 'stop'):
            self.act(self.guide.drop())
        else:
            self.go_to_text(text)

    def go_to_text(self, text):
        """Go to a point, a place or a thing on the map; else ask Stage 2 to look for it."""
        goal = find_goal(text, self.maps_dir, self.map_name, self.guide.position,
                         self.stand_off_m)
        if goal is not None:
            self.locating = None
            self.act(self.guide.new_goal(goal))
            return
        name = clean_name(text)
        if not name:
            self.act(self.guide.unknown_place('that'))
            return
        self.locate_no += 1
        self.locating = (self.locate_no, name, time.monotonic())
        self.act(self.guide.looking_for(name))
        self.locate_pub.publish(String(data=json.dumps({'name': name, 'id': self.locate_no})))

    def on_located(self, msg):
        """Stage 2 looked for the thing: start the trip, or say why it could not."""
        try:
            answer = json.loads(msg.data)
            ident, found = answer.get('id'), bool(answer.get('found'))
        except (ValueError, AttributeError):
            return
        if self.locating is None or ident != self.locating[0]:
            return                              # an old answer, or another question came first
        name, self.locating = self.locating[1], None
        if not found:
            self.act(self.guide.not_located(name, str(answer.get('problem', ''))))
            return
        self.act(self.guide.new_goal(thing_goal(
            'the ' + name, answer['x'], answer['y'], answer.get('from_x'),
            answer.get('from_y'), self.guide.position, self.stand_off_m)))

    def on_goal_pose(self, msg):
        self.act(self.guide.new_goal(Goal(msg.pose.position.x, msg.pose.position.y)))

    def on_position(self, msg):
        turn = msg.pose.orientation
        yaw = math.atan2(2 * (turn.w * turn.z + turn.x * turn.y),
                         1 - 2 * (turn.y * turn.y + turn.z * turn.z))
        self.act(self.guide.moved(msg.pose.position.x, msg.pose.position.y, yaw))

    def on_map_name(self, msg):
        self.map_name = msg.data

    def on_placement(self, msg):
        self.act(self.guide.placement(msg.data not in ('searching', 'lost')))

    def on_red(self, msg):
        if msg.data and not self.red:
            self.act(self.guide.talk_pressed())
        self.red = msg.data

    def on_button(self, msg):
        if msg.data and not self.button:
            self.act(self.guide.talk_pressed())
        self.button = msg.data

    def on_green(self, msg):
        if msg.data and not self.green:
            self.act(self.guide.emergency_on())
        self.green = msg.data

    def on_blocked(self, msg):
        self.blocked = bool(msg.data)

    def on_level(self, msg):
        self.level, self.level_at = msg.data, time.monotonic()

    def on_distance(self, msg):
        self.distance, self.distance_at = msg.data, time.monotonic()

    def on_odom(self, msg):
        place = msg.pose.pose.position
        self.odom, self.odom_at = (place.x, place.y, yaw_of(msg.pose.pose.orientation)), \
            time.monotonic()

    def on_stops(self, msg):
        try:
            self.stops = list(json.loads(msg.data))
        except (ValueError, TypeError):
            self.stops = ['unreadable']

    def goal_side(self):
        """Give +1 if the goal is to the device's left, -1 to its right, 0 if not known."""
        goal, place, yaw = self.guide.goal, self.guide.position, self.guide.yaw
        if goal is None or place is None or yaw is None:
            return 0
        angle = math.atan2(goal.y - place[1], goal.x - place[0]) - yaw
        return 1 if math.sin(angle) >= 0 else -1

    def seen(self, now):
        """Gather what getting unstuck needs to know now."""
        stage1_ok = self.level is not None and now - self.level_at <= LEVEL_TIMEOUT_S \
            and now - self.distance_at <= LEVEL_TIMEOUT_S
        odom = self.odom if self.odom is not None and now - self.odom_at <= FRESH_S else None
        return Seen(
            guiding=self.guide.state == 'guiding', blocked=self.blocked, stage1_ok=stage1_ok,
            camera_stop=self.level in HAZARD_LEVELS,
            camera_m=from_front(self.distance, self.camera_to_front_m) if stage1_ok else None,
            master_stop=bool(self.stops), points=self.eyes.points(now),
            lidar=self.eyes.points(now, ('lidar',)),
            yaw=None if odom is None else odom[2], x=None if odom is None else odom[0],
            y=None if odom is None else odom[1], goal_side=self.goal_side(),
            no_route=self.route_ok is False)

    def on_unstuck_tick(self):
        """10 times a second: let unstuck.py decide; while it drives, send its speeds."""
        now = time.monotonic()
        on = bool(self.get_parameter('unstuck').value)
        with self.lock:
            seen = self.seen(now) if on else Seen()     # off: as if no trip (stops a try)
            command, actions = self.unstuck.step(now, seen)
            self.act(actions)
            if command is not None:
                out = Twist()
                out.linear.x = max(0.0, float(command[0]))      # never backwards
                out.angular.z = float(command[1])
                self.zero_pub.publish(out)
            if self.unstuck.phase != self.unstuck_phase:
                self.get_logger().info('unstuck: %s -> %s (try %d%s)' % (
                    self.unstuck_phase, self.unstuck.phase, self.unstuck.tries,
                    '' if self.unstuck.active else ', ' + self.unstuck.why))
                self.unstuck_phase = self.unstuck.phase
                self.publish_state()

    def on_tick(self):
        # With getting unstuck on, it does the talking about being blocked (unstuck.py).
        talk_blocked = self.blocked and not bool(self.get_parameter('unstuck').value)
        self.act(self.guide.waiting(time.monotonic(), talk_blocked, self.route_ok))
        # 17 Nov (lab A2): one emptying was not enough. A chair seen closer than the camera can
        # clear (1.3 m) kept the device boxed in for 1.5 minutes after it was turned away.
        # While there is still no route, empty the obstacle maps again every RECLEAR_S.
        if self.guide.state == 'guiding' and self.route_ok is False and \
                time.monotonic() - self.cleared_at >= RECLEAR_S and self.cleared_at > 0.0:
            self.clear_obstacle_maps()
        if self.locating is not None and \
                time.monotonic() - self.locating[2] > LOCATE_TIMEOUT_S:
            name, self.locating = self.locating[1], None
            self.act(self.guide.not_located(name, 'timeout'))
        if not self.unstuck.active:             # while getting unstuck, no re-planning
            self.act(self.guide.tick())

    # ---- carrying out the guide's actions ---------------------------------------------------

    def act(self, actions):
        with self.lock:
            for action in actions:
                if action[0] == 'follow':
                    self.follow(action[1])
                elif action[0] == 'cancel':
                    self.cancel()
                elif action[0] == 'hold':           # getting unstuck drives now
                    self.cancel()
                elif action[0] == 'resume':         # back to the route: re-plan now
                    self.act(self.guide.tick())
                elif action[0] == 'say':
                    self.get_logger().info('say: ' + action[1])
                    self.say_pub.publish(String(data=action[1]))
                    if action[1] == NO_WAY:
                        self.clear_obstacle_maps()
            if actions:
                self.publish_state()

    def follow(self, goal):
        """Plan a route to goal; when it comes back, hand it to the follower."""
        if self.planning:
            return                              # the last plan is still being made
        if not self.planner.server_is_ready():
            self.get_logger().warn('Nav2 planner not running (compute_path_to_pose)',
                                   throttle_duration_sec=5.0)
            self.route_ok = False
            return
        self.reachable(goal)
        request = ComputePathToPose.Goal()
        request.goal.header.frame_id = 'map'
        request.goal.header.stamp = self.get_clock().now().to_msg()
        request.goal.pose.position.x = float(goal.x)
        request.goal.pose.position.y = float(goal.y)
        request.goal.pose.orientation.w = 1.0
        request.planner_id = self.get_parameter('planner_id').value
        request.use_start = False               # from where the device is now
        self.planning = True
        trip = self.trip
        future = self.planner.send_goal_async(request)
        future.add_done_callback(lambda f: self.on_plan_accepted(f, trip))

    def reachable(self, goal):
        """
        Move the stop point out of a spot the device cannot reach (18 Nov, the bin in a corner).

        The goal itself is moved, so arrival is judged at the new point; a thing's own place
        (goal.thing) stays, so "the bin is ahead of you" is still said on arrival.
        """
        grid = self.costmap
        if grid is None:
            return
        info = grid.info

        def finder(x, y):
            return reachable_point(grid.data, info.width, info.height, info.resolution,
                                   info.origin.position.x, info.origin.position.y, x, y,
                                   near=self.guide.position)

        def fits(x, y):
            return fits_point(grid.data, info.width, info.height, info.resolution,
                              info.origin.position.x, info.origin.position.y, x, y)

        old = move_stop_point(goal, finder, fits)
        if old is not None:
            self.get_logger().info('stop point (%.2f, %.2f) asked (%.2f, %.2f): using (%.2f, %.2f)'
                                   % (old[0], old[1], goal.wanted[0], goal.wanted[1],
                                      goal.x, goal.y))

    def on_plan_accepted(self, future, trip):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.planning = False
            self.route_ok = False
            return
        handle.get_result_async().add_done_callback(lambda f: self.on_plan(f, trip))

    def on_plan(self, future, trip):
        with self.lock:
            self.planning = False
            answer = future.result()
            if trip != self.trip or self.guide.state != 'guiding':
                return                          # cancelled while planning
            path = answer.result.path if answer is not None else None
            if answer is None or answer.status != GoalStatus.STATUS_SUCCEEDED \
                    or path is None or not path.poses:
                self.route_ok = False
                if self.guide.goal is not None:
                    self.guide.goal.held = False    # 23 Nov: pick the stop point again
                self.get_logger().warn('no route to the goal right now; trying again',
                                       throttle_duration_sec=5.0)
                return                          # the follower keeps the last route
            self.route_ok = True
            if self.guide.goal is not None:
                self.guide.goal.held = True         # 23 Nov: keep this stop point while it fits
            self.route_pub.publish(path)
            self.send_path(path, trip)

    def send_path(self, path, trip):
        if not self.follower.server_is_ready():
            self.get_logger().warn('Nav2 follower not running (follow_path)',
                                   throttle_duration_sec=5.0)
            return
        request = FollowPath.Goal()
        request.path = path
        request.controller_id = self.get_parameter('controller_id').value
        request.goal_checker_id = self.get_parameter('goal_checker_id').value
        future = self.follower.send_goal_async(request)
        future.add_done_callback(lambda f: self.on_follow_accepted(f, trip))

    def on_follow_accepted(self, future, trip):
        handle = future.result()
        with self.lock:
            if handle is None or not handle.accepted:
                return
            if trip != self.trip:               # cancelled meanwhile: stop it at once
                handle.cancel_goal_async()
                return
            self.follow_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.on_follow_done(f, handle, trip))

    def on_follow_done(self, future, handle, trip):
        answer = future.result()
        with self.lock:
            if handle is not self.follow_handle or trip != self.trip:
                return                          # an old route (replaced by a new plan)
            self.follow_handle = None
            succeeded = answer is not None and answer.status == GoalStatus.STATUS_SUCCEEDED
        # 17 Nov (lab A3): the follower refused ("collision ahead") because of old camera marks
        # 0.2 to 0.5 m ahead (legs, the box while it was set out). The camera cannot wipe marks
        # that close to it, and the route was fine, so nothing emptied them. When the follower
        # gives up, empty the obstacle maps (at most every FOLLOW_CLEAR_S): real things come
        # back within a second, and anything at the device's front is still Stage 1's stop.
        # 23 Nov (Adel, stuck at the same desk corner again and again): only the follower's map.
        # The LiDAR (77 cm high) does not see desks, so the camera's marks on the planner's map are
        # the only record of the desk corner; emptying it sent the next route back into the corner.
        # No route at all still empties both (waiting(), RECLEAR_S).
        if not succeeded and time.monotonic() - self.cleared_at >= FOLLOW_CLEAR_S:
            self.clear_obstacle_maps(planner_too=False)
        self.act(self.guide.follower_finished(succeeded, blocked=bool(self.blocked)))

    def clear_obstacle_maps(self, planner_too=True):
        """Empty the obstacle maps once (the follower's, and the planner's if planner_too)."""
        self.cleared_at = time.monotonic()
        for client in self.clearers:
            if not planner_too and 'global' in client.srv_name:
                continue
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
                self.get_logger().info('cleared %s' % client.srv_name)

    def cancel(self):
        """Stop following at once."""
        self.trip += 1
        self.planning = False
        if self.follow_handle is not None:
            self.follow_handle.cancel_goal_async()
            self.follow_handle = None
        self.zero_pub.publish(Twist())          # belt and braces: one zero speed now
        self.route_pub.publish(Path())

    # ---- outputs ----------------------------------------------------------------------------

    def publish_state(self):
        summary = self.guide.summary()
        summary['route_ok'] = self.route_ok
        summary['map'] = self.map_name
        summary['unstuck'] = self.unstuck.phase
        self.state_pub.publish(String(data=json.dumps(summary)))


def main(args=None):
    rclpy.init(args=args)
    node = GuideNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
