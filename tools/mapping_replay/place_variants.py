"""Which way of reading the sweep fits the map at the start spot? (16 Nov night)
Device running, standing on START. Tries the sweep mirrored or not, turned 0 or 180 deg, and the
LiDAR's position as the frames give it or mirrored, and prints the fit after a small refine.
   python3 place_variants.py MAP_NAME
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
import rclpy.duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

import apnvi_stage3.relocalizer as rl
from apnvi_stage3.map_manager_node import RTABMAP_FRAME
from apnvi_stage3.map_names import saved_grid, work_path

grid = saved_grid(work_path(os.path.expanduser('~/.ros/apnvi_maps'), sys.argv[1]))
finder = rl.Relocalizer(*grid)
rclpy.init()
node = rclpy.create_node('place_variants')
buf = Buffer()
TransformListener(buf, node)
scans = []
node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
end = time.time() + 6
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
scan = scans[-1]
for frame in ('base_link',):
    t = buf.lookup_transform(RTABMAP_FRAME, frame, rclpy.time.Time(),
                             timeout=rclpy.duration.Duration(seconds=3.0))
    q = t.transform.rotation
    print('%s in %s: x %.2f y %.2f yaw %.0f deg' % (frame, RTABMAP_FRAME, t.transform.translation.x,
          t.transform.translation.y, math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                            1 - 2 * (q.y ** 2 + q.z ** 2)))))
tf = buf.lookup_transform(RTABMAP_FRAME, scan.header.frame_id, rclpy.time.Time(),
                          timeout=rclpy.duration.Duration(seconds=3.0))
q = tf.transform.rotation
yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
x0, y0 = tf.transform.translation.x, tf.transform.translation.y
bl = buf.lookup_transform(RTABMAP_FRAME, 'base_link', rclpy.time.Time())
bx, by = bl.transform.translation.x, bl.transform.translation.y
print('laser at x %.2f y %.2f yaw %.0f deg (up_z %.2f)' % (x0, y0, math.degrees(yaw),
                                                          1 - 2 * (q.x ** 2 + q.y ** 2)))
rows = []
for mirror in (False, True):
    amin, inc = (-scan.angle_min, -scan.angle_increment) if mirror else (scan.angle_min, scan.angle_increment)
    pts = finder.sweep_points(scan.ranges, amin, inc)
    for name, (lx, ly) in (('laser where the frames say', (x0, y0)),
                           ('laser mirrored about the centre', (2 * bx - x0, 2 * by - y0))):
        for turn in (0, 180):
            a = yaw + math.radians(turn)
            s0 = float(finder.score(pts, np.array([lx]), np.array([ly]), np.array([a]))[0])
            rx, ry, ra, rs = finder.refine(pts, lx, ly, a)
            rows.append((rs, 'mirror %-5s %-32s turn %3d: at once %3.0f%%, refined %3.0f%% (moved %.2f m, %.0f deg)'
                         % (mirror, name, turn, 100 * s0, 100 * rs, math.hypot(rx - lx, ry - ly),
                            math.degrees(ra - a))))
for _, line in sorted(rows, reverse=True):
    print(line)
