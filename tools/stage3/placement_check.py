"""
Check that the device is placed correctly on the loaded map, without tape marks.

  python3 placement_check.py
Takes the LiDAR sweep, puts it on the map from the device's reported position, and reports
which share of the sweep's points land on (or right next to) a wall of the saved map. Placed
right: about 60% or more. Placed wrong: usually under 30%. Prints the position too.
"""

import math
import sys
import time

import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

OCCUPIED_ABOVE = 50
NEAR_CELLS = 2                  # a point within 2 cells (10 cm) of a wall counts as a hit


def main():
    rclpy.init()
    node = Node('placement_check')
    buffer = Buffer()
    TransformListener(buffer, node)
    holder = {}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map', lambda m: holder.update(map=m), latched)
    node.create_subscription(LaserScan, '/scan', lambda m: holder.update(scan=m), 5)
    end = time.time() + 10
    while time.time() < end and not ('map' in holder and 'scan' in holder):
        rclpy.spin_once(node, timeout_sec=0.2)
    if 'map' not in holder or 'scan' not in holder:
        print('no map or no scan')
        return 1
    scan, grid = holder['scan'], holder['map']
    transform = None
    while transform is None and time.time() < end + 5:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            transform = buffer.lookup_transform('map', scan.header.frame_id, scan.header.stamp)
        except Exception:           # the tree is not there yet for this time: try again
            scan = holder['scan']
    if transform is None:
        print('no transform map -> %s' % scan.header.frame_id)
        return 1
    q = transform.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    ranges = np.array(scan.ranges)
    angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges))
    good = np.isfinite(ranges) & (ranges > 0.3) & (ranges < 6.0)
    px, py = ranges[good] * np.cos(angles[good]), ranges[good] * np.sin(angles[good])
    mx = transform.transform.translation.x + px * math.cos(yaw) - py * math.sin(yaw)
    my = transform.transform.translation.y + px * math.sin(yaw) + py * math.cos(yaw)
    info = grid.info
    cells = np.array(grid.data, np.int8).reshape(info.height, info.width) > OCCUPIED_ABOVE
    col = ((mx - info.origin.position.x) / info.resolution).astype(int)
    row = ((my - info.origin.position.y) / info.resolution).astype(int)
    hits = 0
    for c, r in zip(col, row):
        window = cells[max(r - NEAR_CELLS, 0):r + NEAR_CELLS + 1,
                       max(c - NEAR_CELLS, 0):c + NEAR_CELLS + 1]
        hits += bool(window.size and window.any())
    base = buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
    print('position x=%.2f y=%.2f yaw=%.1f deg; %d of %d scan points on a wall (%.0f%%)' % (
        base.transform.translation.x, base.transform.translation.y, math.degrees(yaw),
        hits, len(col), 100 * hits / max(len(col), 1)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
