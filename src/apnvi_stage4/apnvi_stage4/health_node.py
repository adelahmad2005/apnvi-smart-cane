"""
health_node: says "Ready" once the device is up, and speaks up when a part stops working.

The judging is in health.py. Each sentence is said once, through Stage 2's voice.

Listens: /hazard/level       String  Stage 1's level (its heartbeat: about 10 a second)
         /sensor/status      String  camera_ok / camera_blocked (Stage 1)
         /scan               LaserScan  the LiDAR's heartbeat
         /stage4/drive_status String JSON from the drive link: connected, master
         /grip/connected     Bool    the handle's radio (latched)
         /stage3/placement   String  placed / searching / lost (latched)
Sends:   /voice/say          String  the sentences
         /stage4/health      String  JSON once a second: each part True / False / null

Settings: speak (true; false = only the status line, for tests)
"""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .health import Health


class HealthNode(Node):
    """Watch the parts of the device and tell the user when one stops or comes back."""

    def __init__(self):
        super().__init__('health_node')
        self.declare_parameter('speak', True)
        self.speak = bool(self.get_parameter('speak').value)
        self.health = Health(time.monotonic())
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.say_pub = self.create_publisher(String, '/voice/say', 10)
        self.status_pub = self.create_publisher(String, '/stage4/health', 10)
        self.create_subscription(String, '/hazard/level',
                                 lambda m: self.health.ok('warnings', time.monotonic()), 10)
        self.create_subscription(String, '/sensor/status', self.on_camera, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/stage4/drive_status', self.on_drive, 10)
        self.create_subscription(Bool, '/grip/connected', self.on_handle, latched)
        self.create_subscription(String, '/stage3/placement', self.on_placement, latched)
        self.create_timer(1.0, self.tick)

    def report(self, name, good):
        now = time.monotonic()
        if good:
            self.health.ok(name, now)
        else:
            self.health.bad(name, now)

    def on_camera(self, msg):
        self.report('camera', msg.data == 'camera_ok')

    def on_scan(self, _msg):
        self.health.ok('lidar', time.monotonic())

    def on_drive(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        self.report('wheels', bool(status.get('connected')) and bool(status.get('master')))

    def on_handle(self, msg):
        self.report('handle', bool(msg.data))

    def on_placement(self, msg):
        if msg.data == 'lost':
            self.report('place', False)
        elif msg.data == 'placed':
            self.report('place', True)

    def tick(self):
        now = time.monotonic()
        for words in self.health.step(now):
            self.get_logger().info('health: ' + words)
            if self.speak:
                self.say_pub.publish(String(data=words))
        self.status_pub.publish(String(data=json.dumps(self.health.summary(now))))


def main(args=None):
    rclpy.init(args=args)
    node = HealthNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
