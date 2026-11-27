"""
free_mode_node: with no trip on, the user pushes and the device steers them round obstacles.

The rules are in free_mode.py. The one that matters: this node only ever asks for a TURN.
Its forward speed is always 0 (the hazard gate checks that again), so the device cannot move
forward by itself. It turns only while the LiDAR's movement tracking shows the user pushing.

It works only when all of these hold, and sends nothing otherwise:
  - the guide is idle (no trip, not paused)
  - the LiDAR sweep is fresh (no eyes, no steering)
  - the frames of the LiDAR and the camera are known

Listens: /scan                  LaserScan  the LiDAR sweep (the user behind is filtered out)
         /camera/obstacle_scan  LaserScan  what the camera sees low down (Stage 1)
         /odom                  Odometry   the device's own movement
         /stage4/guide_state    String     idle / guiding / paused (latched)
         the frames base_link -> laser and base_link -> camera_link (set by the launch files)
Sends:   /cmd_vel_free          Twist      linear.x always 0; angular.z the turn rate. The
                                           hazard gate passes it on only while the guide is idle
         /stage4/free_mode      String     JSON twice a second and on every change: pushing
                                           speed, what it is doing (why), the turn rate

Settings (all in free_mode.FreeSettings; --ros-args -p name:=value):
  push_mps (0.15), keep_mps (0.10), lane_m (1.5), half_width_m (0.45), front_m (0.13),
  turn_rps (0.4), centre_rps (0.2), centre (true), max_turn_deg (60), max_turn_s (3.0),
  rest_s (1.0), base_frame (base_link)
"""

import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .free_mode import (FreeSettings, PushMeter, quaternion_matrix, Steering, steering_allowed,
                        sweep_points)

SWEEP_FRESH_S = 0.5             # a sweep older than this is not used
RATE_HZ = 10.0


def yaw_of(q):
    """Give the heading (radians, anticlockwise from x) of a quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class FreeModeNode(Node):
    """Turn the device away from what is in the way, only while the user pushes it."""

    def __init__(self):
        super().__init__('free_mode_node')
        defaults = FreeSettings()
        numbers = ('push_mps', 'keep_mps', 'lane_m', 'half_width_m', 'front_m', 'turn_rps',
                   'centre_rps', 'max_turn_s', 'rest_s')
        for name in numbers:
            self.declare_parameter(name, float(getattr(defaults, name)))
        self.declare_parameter('centre', True)
        self.declare_parameter('max_turn_deg', 60.0)
        self.declare_parameter('base_frame', 'base_link')
        settings = FreeSettings(
            **{name: float(self.get_parameter(name).value) for name in numbers},
            centre=bool(self.get_parameter('centre').value),
            max_turn_rad=math.radians(float(self.get_parameter('max_turn_deg').value)))
        self.settings = settings
        self.steering = Steering(settings)
        self.meter = PushMeter()
        self.base_frame = self.get_parameter('base_frame').value
        self.idle = False
        # 18 Nov (lab D1): on arrival the device was still rolling from its own drive; free mode
        # took that for a push and steered it away from the bottle. After a trip, steering waits
        # until the device has stood still once.
        self.settled = True
        self.sweeps = {}                # 'lidar' / 'camera' -> (time, message)
        self.frames = {}                # frame name -> (rotation 3 x 3, shift)
        self.last_status, self.status_at = None, 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_free', 10)
        self.status_pub = self.create_publisher(String, '/stage4/free_mode', 10)
        self.create_subscription(LaserScan, '/scan',
                                 lambda m: self.on_sweep('lidar', m), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/camera/obstacle_scan',
                                 lambda m: self.on_sweep('camera', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(String, '/stage4/guide_state', self.on_guide, latched)
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info(
            'free mode steering on: turns at %.2f rad/s while pushed faster than %.2f m/s; '
            'lane %.1f m long, %.2f m wide. It never drives forward.' % (
                settings.turn_rps, settings.push_mps, settings.lane_m,
                2 * settings.half_width_m))

    def on_sweep(self, name, msg):
        self.sweeps[name] = (time.monotonic(), msg)

    def on_odom(self, msg):
        position = msg.pose.pose.position
        self.meter.add(time.monotonic(), position.x, position.y,
                       yaw_of(msg.pose.pose.orientation))

    def on_guide(self, msg):
        try:
            idle = steering_allowed(json.loads(msg.data))
        except (ValueError, AttributeError):
            idle = False
        if idle and not self.idle:
            self.settled = False
        self.idle = idle

    def frame(self, name):
        """Give (rotation, shift) of a sensor's frame in the device's frame, or None."""
        if name not in self.frames:
            try:
                found = self.tf_buffer.lookup_transform(self.base_frame, name, rclpy.time.Time())
            except TransformException:
                return None
            turn, move = found.transform.rotation, found.transform.translation
            self.frames[name] = (quaternion_matrix(turn.x, turn.y, turn.z, turn.w),
                                 (move.x, move.y, move.z))
        return self.frames[name]

    def points(self, now):
        """Give everything seen, in the device's frame, or None if the LiDAR sweep is old."""
        seen = []
        for name in ('lidar', 'camera'):
            entry = self.sweeps.get(name)
            if entry is None or now - entry[0] > SWEEP_FRESH_S:
                if name == 'lidar':
                    return None
                continue
            sweep = entry[1]
            frame = self.frame(sweep.header.frame_id)
            if frame is None:
                if name == 'lidar':
                    return None
                continue
            seen.append(sweep_points(sweep.ranges, sweep.angle_min, sweep.angle_increment,
                                     frame[0], frame[1], self.settings.max_range_m))
        return np.vstack(seen) if seen else np.zeros((0, 2))

    def tick(self):
        now = time.monotonic()
        speed = self.meter.speed(now)
        turn, why = 0.0, 'guide_busy'
        if self.idle:
            points = self.points(now)
            if not self.settled and (speed or 0.0) < self.settings.push_mps:
                self.settled = True
            if not self.settled:
                why = 'after_trip'
                self.steering.stop(why)
            elif points is None:
                why = 'no_sweep'
                self.steering.stop(why)
            else:
                turn = self.steering.step(now, speed, self.meter.yaw(), points)
                why = self.steering.why
            command = Twist()
            command.angular.z = float(turn)     # linear.x stays 0: free mode never drives
            self.cmd_pub.publish(command)
        status = (why, round(turn, 2))
        if status != self.last_status or now - self.status_at >= 0.5:
            self.last_status, self.status_at = status, now
            self.status_pub.publish(String(data=json.dumps(
                {'why': why, 'turn_rps': round(turn, 2), 'push_mps': round(speed, 2)})))


def main(args=None):
    rclpy.init(args=args)
    node = FreeModeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
