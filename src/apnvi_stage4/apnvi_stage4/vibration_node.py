"""
vibration_node: the grip's vibration signals, each with one meaning (rules.vibration_signal).

  careful      something ahead, 0.5 to 1.0 m from the device's front
  stopped      forward is blocked
  going left / going right   the device is turning the user that way
  heard you    the talk button was pressed
  arrived      "You have arrived"
  need help    no way to the goal, or the device does not know where it is
Silent otherwise (Adel, 15 Nov: vibration only when it matters).
They run while guiding, and with no trip while the user pushes the device (free mode).
Standing still with no trip: silent, except heard you.

Listens: /hazard/level       String   Stage 1's level, about 10 times a second
         /hazard/distance    Float32  metres from the camera to the nearest thing, -1 = nothing
         /stage4/guide_state String   the guide's state as JSON (latched)
         /stage4/cmd_vel     Twist    the speeds the wheels are asked for (after the hazard gate)
         /odom               Odometry the device's own movement (is the user pushing it?)
         /grip/red, /button/pressed  Bool  the talk buttons
         /voice/say          String   "You have arrived..." starts the arrived signal
         /stage3/placement   String   "lost" = the device does not know where it is
         /stage4/vibration_test String  for trying the signals by hand: "left 3" plays going
                                      left for 3 s; "motor1 2" / "motor2 2" run one grip motor
                                      steadily (to find which side each is on); "off" ends it
Sends:   /stage4/vibration        UInt8  strength 0 to 100, 10 times a second
         /stage4/vibration_motor  UInt8  which grip motor, 1 or 2
         /stage4/vibration_signal String the signal's name on every change (for the recorder)

Settings: careful_m (1.0), camera_to_front_m (0.83, measured 15 Nov), stop_m (0.5),
          two_sides (true: the grip's two motors sit left and right), left_motor (1),
          pushed_mps (0.15), free_mode (true: signals also run while the user pushes).
"""

import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, String, UInt8

from .free_mode import PushMeter
from .guide import ARRIVED
from .rules import (BOTH_MOTORS, CAMERA_TO_FRONT_M, CAREFUL_M, from_front, gate,
                    HELP_AFTER_S, LEVEL_TIMEOUT_S, NEAR_M, PUSHED_MPS, signal_input,
                    SIGNAL_NAMES, STOP_M, TURNING_RPS, VibeInput, vibration_signal)

VIBE_PUSH_MPS = 0.05            # free mode: moving at least this counts as being pushed
PUSH_HOLD_S = 3.0               # ...and the signals go on this long after it stops
CMD_FRESH_S = 0.3               # a wheel speed older than this counts as "no turning"


class VibrationNode(Node):
    """Turn what the device is doing into one vibration signal at a time."""

    def __init__(self):
        super().__init__('vibration_node')
        self.declare_parameter('camera_to_front_m', CAMERA_TO_FRONT_M)
        self.declare_parameter('careful_m', CAREFUL_M)
        self.declare_parameter('near_m', NEAR_M)      # 17 Nov: "near" starts this far
        self.declare_parameter('stop_m', STOP_M)
        self.declare_parameter('two_sides', True)
        self.declare_parameter('left_motor', 1)
        self.declare_parameter('pushed_mps', PUSHED_MPS)
        self.declare_parameter('free_mode', True)
        self.camera_to_front_m = float(self.get_parameter('camera_to_front_m').value)
        self.careful_m = float(self.get_parameter('careful_m').value)
        self.near_m = float(self.get_parameter('near_m').value)
        self.stop_m = float(self.get_parameter('stop_m').value)
        self.two_sides = bool(self.get_parameter('two_sides').value)
        self.left_motor = int(self.get_parameter('left_motor').value)
        self.pushed_mps = float(self.get_parameter('pushed_mps').value)
        self.free_mode = bool(self.get_parameter('free_mode').value)
        self.level, self.level_at = None, -1e9
        self.distance, self.distance_at = None, -1e9
        self.side, self.side_at = None, -1e9          # Stage 1: coming in from a side
        self.guiding, self.route_bad_since = False, None
        self.lost = False
        self.turn, self.turn_at, self.turn_since = 0.0, -1e9, None
        self.meter = PushMeter()        # how fast the user pushes (free mode)
        self.pushed_at = -1e9           # last time the device moved under the user's push
        self.blocked_since = None
        self.heard_at = self.arrived_at = None
        self.buttons = {}
        self.last_name = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(UInt8, '/stage4/vibration', 10)
        self.motor_pub = self.create_publisher(UInt8, '/stage4/vibration_motor', 10)
        self.name_pub = self.create_publisher(String, '/stage4/vibration_signal', 10)
        self.create_subscription(String, '/hazard/level', self.on_level, 10)
        self.create_subscription(Float32, '/hazard/distance', self.on_distance, 10)
        self.create_subscription(String, '/hazard/side', self.on_side, 10)
        self.create_subscription(String, '/stage4/guide_state', self.on_guide, latched)
        self.create_subscription(Twist, '/stage4/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Bool, '/grip/red', lambda m: self.on_button('red', m), latched)
        self.create_subscription(Bool, '/button/pressed',
                                 lambda m: self.on_button('button', m), 10)
        self.create_subscription(String, '/voice/say', self.on_say, 10)
        self.create_subscription(String, '/stage3/placement', self.on_placement, latched)
        self.test = None                # (name, started, until) while a signal is being tried
        self.create_subscription(String, '/stage4/vibration_test', self.on_test, 10)
        self.create_timer(0.1, self.tick)

    def on_level(self, msg):
        self.level, self.level_at = msg.data, time.monotonic()

    def on_side(self, msg):
        """Stage 1: something coming in from the 'left' or 'right' (empty: nothing)."""
        side = msg.data.strip().lower()
        if side in ('left', 'right'):
            self.side, self.side_at = side, time.monotonic()

    def on_distance(self, msg):
        self.distance, self.distance_at = msg.data, time.monotonic()

    def on_guide(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            state = {}
        self.guiding = state.get('state') == 'guiding'
        if self.guiding and state.get('route_ok') is False:
            if self.route_bad_since is None:
                self.route_bad_since = time.monotonic()
        else:
            self.route_bad_since = None

    def on_cmd(self, msg):
        now = time.monotonic()
        turn = msg.angular.z
        same_way = abs(turn) >= TURNING_RPS and abs(self.turn) >= TURNING_RPS \
            and (turn > 0) == (self.turn > 0) and now - self.turn_at <= CMD_FRESH_S
        if abs(turn) < TURNING_RPS:
            self.turn_since = None
        elif not same_way or self.turn_since is None:
            self.turn_since = now
        self.turn, self.turn_at = turn, now

    def on_odom(self, msg):
        position, turn = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (turn.w * turn.z + turn.x * turn.y),
                         1 - 2 * (turn.y * turn.y + turn.z * turn.z))
        self.meter.add(time.monotonic(), position.x, position.y, yaw)

    def on_button(self, name, msg):
        if msg.data and not self.buttons.get(name):
            self.heard_at = time.monotonic()
        self.buttons[name] = msg.data

    def on_say(self, msg):
        if msg.data.startswith(ARRIVED):
            self.arrived_at = time.monotonic()

    def on_placement(self, msg):
        self.lost = msg.data == 'lost'

    def on_test(self, msg):
        """Play one signal (or run one motor) for a few seconds, to try it by hand."""
        words = msg.data.split()
        name = words[0].lower() if words else 'off'
        if name not in SIGNAL_NAMES + ('motor1', 'motor2', 'both'):
            self.test = None
            return
        try:
            seconds = min(20.0, float(words[1])) if len(words) > 1 else 3.0
        except ValueError:
            seconds = 3.0
        now = time.monotonic()
        self.test = (name, now, now + seconds)
        self.get_logger().info('trying the signal "%s" for %.0f s' % (name, seconds))

    def snapshot(self, now):
        """Gather what is known now into a VibeInput."""
        front_m = None
        if now - self.distance_at <= LEVEL_TIMEOUT_S:
            front_m = from_front(self.distance, self.camera_to_front_m)
        _f, _t, blocked = gate(1.0, 0.0, self.level, now - self.level_at, front_m,
                               stop_m=self.stop_m)
        if not blocked:
            self.blocked_since = None
        elif self.blocked_since is None:
            self.blocked_since = now
        turning = now - self.turn_at <= CMD_FRESH_S and self.turn_since is not None
        # 17 Nov (lab C2): pushed slowly into a chair, the speed fell under pushed_mps and the
        # grip went quiet just when it mattered. Free mode signals while the device moved even
        # slowly in the last PUSH_HOLD_S, so slowing into a thing keeps "near" and gives
        # "stopped"; standing still for longer is quiet.
        if (self.meter.speed(now) or 0.0) >= VIBE_PUSH_MPS:
            self.pushed_at = now
        moving = self.free_mode and not self.guiding and now - self.pushed_at <= PUSH_HOLD_S
        no_route = self.route_bad_since is not None \
            and now - self.route_bad_since >= HELP_AFTER_S
        return VibeInput(
            guiding=self.guiding, moving=moving, blocked=blocked,
            blocked_for_s=0.0 if self.blocked_since is None else now - self.blocked_since,
            front_m=front_m, turn_rps=self.turn if turning else 0.0,
            turning_for_s=now - self.turn_since if turning else 0.0,
            need_help=self.guiding and (no_route or self.lost),
            heard_ago_s=None if self.heard_at is None else now - self.heard_at,
            arrived_ago_s=None if self.arrived_at is None else now - self.arrived_at,
            side=self.side if now - self.side_at <= LEVEL_TIMEOUT_S else None)

    def tick(self):
        now = time.monotonic()
        if self.test is not None and now >= self.test[2]:
            self.test = None
        if self.test is not None and self.test[0] in ('motor1', 'motor2', 'both'):
            motor = BOTH_MOTORS if self.test[0] == 'both' else int(self.test[0][-1])
            strength, name = 80, 'test ' + self.test[0]
        elif self.test is not None:
            strength, motor, name = vibration_signal(
                signal_input(self.test[0], now - self.test[1]), now, self.near_m,
                self.two_sides, self.left_motor)
            name = 'test ' + self.test[0]
        else:
            strength, motor, name = vibration_signal(
                self.snapshot(now), now, self.near_m, self.two_sides, self.left_motor)
        self.motor_pub.publish(UInt8(data=motor))
        self.pub.publish(UInt8(data=strength))
        if name != self.last_name:
            self.last_name = name
            self.name_pub.publish(String(data=name))


def main(args=None):
    rclpy.init(args=args)
    node = VibrationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
