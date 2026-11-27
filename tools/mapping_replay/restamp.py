"""Replay test (16 Nov night): give each LiDAR sweep an even time stamp.

The driver stamps a sweep with the time the Jetson read it, so the gaps between stamps run from
0.04 to 0.16 s although the LiDAR turns at a steady 0.1026 s. This node re-stamps /scan_raw onto a
steady beat (scan_time per sweep, a skipped sweep counted as two) that slowly follows the real stamps,
and publishes /scan_rs.   python3 restamp.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

FOLLOW = 0.05          # how fast the beat follows the real stamps (per sweep)


class Restamp(Node):
    def __init__(self):
        super().__init__('restamp')
        self.pub = self.create_publisher(LaserScan, '/scan_rs', 10)
        self.create_subscription(LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)
        self.out = None

    def on_scan(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        period = m.scan_time if m.scan_time > 0 else 0.1
        if self.out is None or abs(st - self.out) > 2.0:
            self.out = st                               # first sweep, or a long gap: start again
        else:
            beats = max(1, round((st - self.out) / period))
            self.out += beats * period
            self.out += FOLLOW * (st - self.out)
        m.header.stamp = Time(seconds=self.out).to_msg()
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Restamp())


if __name__ == '__main__':
    main()
