"""
hazard_gate_node: sits between the route follower (Nav2) and the drive link.

Forward speed is set to zero while something is 0.5 m or closer to the device's front, or Stage 1
says "stop", or Stage 1 has been quiet for 0.5 s. The route stays and the turn rate is passed on.
Stage 1 measures from the camera, so camera_to_front_m is taken off first.

17 Nov (lab A3): once stopped, it drives on only when the thing is 0.15 m further than the stop
distance and has looked clear for 1 s (no more flicker between stopped and careful). While
stopped, if the route bends, it turns on the spot that way at 0.6 rad/s or more, so it swings
round a desk corner instead of standing still; straight ahead (a person) it just waits.

23 Nov (Adel, the LiDAR stopped twice mid-trip): with no LiDAR sweep for lidar_quiet_s the
position freezes, and the route follower and the gate's pivot kept turning blind (a half turn the
device could not see). Now no sweep for lidar_quiet_s (or none yet) sends zero, trips and free mode
alike. Health already says "The laser scanner has stopped". need_lidar:=false turns this off.

Free mode (16 Nov): with no trip on, free_mode_node sends turns on /cmd_vel_free.
They are passed on only while the guide is idle, and their forward speed is thrown away here
whatever it is: in free mode the device only turns, it never drives forward.

Listens: /cmd_vel_nav      Twist    from Nav2's route follower (and the keyboard tool)
         /cmd_vel_free     Twist    turns from free_mode_node, used only while the guide is idle
         /stage4/guide_state String the guide's state as JSON (latched)
         /hazard/level     String   Stage 1's level, about 10 times a second
         /hazard/distance  Float32  Stage 1's distance from the camera, -1 = nothing
         /scan             LaserScan the LiDAR's sweeps, only their arrival time (23 Nov)
Sends:   /stage4/cmd_vel Twist  to the drive link
         /stage4/hazard  Bool   True while forward driving is blocked (on change, latched)

Settings: camera_to_front_m (default 0.83, measured 15 Nov), stop_m (0.5).
"""

import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from .free_mode import steering_allowed
from .rules import (CAMERA_TO_FRONT_M, from_front, gate, hold_blocked, LEVEL_TIMEOUT_S,
                    lidar_quiet, LIDAR_QUIET_S, pivot_when_blocked, RELEASE_M, STOP_M,
                    TurnLatch)


class HazardGateNode(Node):
    """Zero the forward speed while something is too close to the device's front."""

    def __init__(self):
        super().__init__('hazard_gate_node')
        self.declare_parameter('camera_to_front_m', CAMERA_TO_FRONT_M)
        self.declare_parameter('stop_m', STOP_M)
        self.camera_to_front_m = float(self.get_parameter('camera_to_front_m').value)
        self.stop_m = float(self.get_parameter('stop_m').value)
        self.declare_parameter('need_lidar', True)
        self.declare_parameter('lidar_quiet_s', LIDAR_QUIET_S)
        self.need_lidar = bool(self.get_parameter('need_lidar').value)
        self.lidar_quiet_s = float(self.get_parameter('lidar_quiet_s').value)
        self.scan_at = None
        self.level = None
        self.level_at = 0.0
        self.distance = None
        self.distance_at = 0.0
        self.blocked = None
        self.clear_since = None
        self.unstuck_on = False
        self.latch = TurnLatch()        # 18 Nov: no left-right-left wiggle with the goal behind
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, '/stage4/cmd_vel', 10)
        self.blocked_pub = self.create_publisher(Bool, '/stage4/hazard', latched)
        self.create_subscription(String, '/hazard/level', self.on_level, 10)
        self.create_subscription(Float32, '/hazard/distance', self.on_distance, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_cmd, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.guide_idle = False
        self.create_subscription(Twist, '/cmd_vel_free', self.on_free, 10)
        self.create_subscription(String, '/stage4/guide_state', self.on_guide, latched)
        self.get_logger().info('hazard gate: stop at %.2f m from the front, camera %.2f m behind'
                               ' the front' % (self.stop_m, self.camera_to_front_m))

    def on_level(self, msg):
        self.level, self.level_at = msg.data, time.monotonic()

    def on_distance(self, msg):
        self.distance, self.distance_at = msg.data, time.monotonic()

    def on_scan(self, _msg):
        self.scan_at = time.monotonic()

    def blind(self):
        """Say if the wheels must stay still because the LiDAR is quiet (23 Nov)."""
        if not self.need_lidar or not lidar_quiet(time.monotonic(), self.scan_at,
                                                  self.lidar_quiet_s):
            return False
        self.get_logger().warn('no LiDAR sweep for %.1f s: wheels stopped' % self.lidar_quiet_s,
                               throttle_duration_sec=5.0)
        return True

    def on_guide(self, msg):
        try:
            state = json.loads(msg.data)
            self.guide_idle = steering_allowed(state)
            self.unstuck_on = state.get('unstuck', 'off') not in ('off', None)
        except (ValueError, AttributeError):
            self.guide_idle = False
            self.unstuck_on = False

    def on_free(self, msg):
        """Pass a free mode turn to the wheels: only while idle, and never any forward speed."""
        if not self.guide_idle:
            return
        out = Twist()
        if not self.blind():
            out.angular.z = float(msg.angular.z)
        self.cmd_pub.publish(out)

    def on_cmd(self, msg):
        now = time.monotonic()
        front_m = None
        if now - self.distance_at <= LEVEL_TIMEOUT_S:
            front_m = from_front(self.distance, self.camera_to_front_m)
        stop_m = self.stop_m + (RELEASE_M if self.blocked else 0.0)
        forward, turn, blocked_now = gate(msg.linear.x, msg.angular.z, self.level,
                                          now - self.level_at, front_m, stop_m=stop_m)
        if self.unstuck_on:
            self.latch.release()        # the guide's own turn ("turning right") is sent as asked
        else:
            turn = self.latch.apply(now, msg.linear.x, turn)
        blocked, self.clear_since = hold_blocked(blocked_now, bool(self.blocked),
                                                 self.clear_since, now)
        if blocked:
            forward = min(forward, 0.0)
        if self.level is not None and now - self.level_at <= LEVEL_TIMEOUT_S:
            turn = pivot_when_blocked(msg.linear.x, turn, blocked)   # never blind: camera alive
        out = Twist()
        if not self.blind():
            out.linear.x, out.angular.z = float(forward), float(turn)
        self.cmd_pub.publish(out)
        if blocked != self.blocked:
            self.blocked = blocked
            self.blocked_pub.publish(Bool(data=blocked))
            if blocked:
                self.get_logger().info('forward driving blocked (Stage 1: %s, %s m from the front)'
                                       % (self.level, 'nothing' if front_m is None
                                          else '%.2f' % front_m))


def main(args=None):
    rclpy.init(args=args)
    node = HazardGateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
