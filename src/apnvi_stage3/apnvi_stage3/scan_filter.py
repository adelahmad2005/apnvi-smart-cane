"""
scan_filter: drop the LiDAR points that are the user, the handle and the pole (15 Nov).

On the device the LiDAR sits on the handle pole, so the person holding the handle, the handle
and the pole are right next to it, behind the device. They move with the LiDAR, so lining up
sweeps with them in pulls towards "not moving" (3.5 m off after one lap of ESB117). Only points
that are BOTH behind the device (within rear_half_deg of straight back) AND close (under
rear_range_m) are dropped. Everything in front and to the sides is kept at every distance, so
desk edges and corners passed closely still count.

Listens: /scan_raw  LaserScan  from the LiDAR driver
Sends:   /scan      LaserScan  the same sweep, those points set to "nothing seen" (inf)

Settings: rear_angle_deg (the LiDAR's own angle that points straight back from the device,
default 0: its zero angle points backward, 31 Oct), rear_half_deg (60), rear_range_m (0.7).
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def rear_mask(angle_min, angle_increment, count, rear_angle, half):
    """Say for each point in the sweep whether it lies in the rear sector (list of bools)."""
    mask = []
    for i in range(count):
        a = angle_min + i * angle_increment - rear_angle
        a = math.atan2(math.sin(a), math.cos(a))        # -pi..pi from straight back
        mask.append(abs(a) <= half)
    return mask


def filter_ranges(ranges, mask, rear_range_m):
    """Set the close points in the rear sector to inf; keep everything else."""
    return [math.inf if behind and r < rear_range_m else r
            for r, behind in zip(ranges, mask)]


class ScanFilter(Node):
    """Republish the LiDAR sweep without the user and the handle."""

    def __init__(self):
        super().__init__('scan_filter')
        self.declare_parameter('rear_angle_deg', 0.0)
        self.declare_parameter('rear_half_deg', 60.0)
        self.declare_parameter('rear_range_m', 0.7)
        self.rear_angle = math.radians(float(self.get_parameter('rear_angle_deg').value))
        self.half = math.radians(float(self.get_parameter('rear_half_deg').value))
        self.rear_range_m = float(self.get_parameter('rear_range_m').value)
        self.mask = None
        self.mask_key = None
        # Reliable, like the LiDAR driver: a reliable sender reaches both kinds of listener (the
        # map manager listens reliably; a best-effort sender never reaches it, 15 Nov).
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        key = (msg.angle_min, msg.angle_increment, len(msg.ranges))
        if key != self.mask_key:
            self.mask = rear_mask(msg.angle_min, msg.angle_increment, len(msg.ranges),
                                  self.rear_angle, self.half)
            self.mask_key = key
        msg.ranges = filter_ranges(msg.ranges, self.mask, self.rear_range_m)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
