"""Try the placing search with different nearest-point limits on one live sweep (16 Nov night).
Device running with a loaded map, standing on START.
   python3 place_try.py MAP_NAME
For each limit: the fit at the start spot, and what the whole-map search finds (best, runner-up,
how far the best is from the start spot).
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
from apnvi_stage3.map_manager_node import RTABMAP_FRAME, sweep_angles
from apnvi_stage3.map_names import saved_grid, work_path

grid = saved_grid(work_path(os.path.expanduser('~/.ros/apnvi_maps'), sys.argv[1]))
finder = rl.Relocalizer(*grid)
rclpy.init()
node = rclpy.create_node('place_try')
buf = Buffer()
TransformListener(buf, node)
scans = []
node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
end = time.time() + 6
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
scan = scans[-1]
tf = buf.lookup_transform(RTABMAP_FRAME, scan.header.frame_id, rclpy.time.Time(),
                          timeout=rclpy.duration.Duration(seconds=3.0))
q = tf.transform.rotation
yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
up_down = (1 - 2 * (q.x ** 2 + q.y ** 2)) < 0
x0, y0 = tf.transform.translation.x, tf.transform.translation.y
for rmin in (1.0, 2.0, 2.5, 3.0, 3.5):
    rl.RANGE_MIN_M = rmin
    pts = finder.sweep_points(*sweep_angles(scan, up_down))
    s0 = float(finder.score(pts, np.array([x0]), np.array([y0]), np.array([yaw]))[0])
    sx, sy, syaw, ss = finder.refine(pts, x0, y0, yaw)
    found = finder.find(pts)
    far = math.hypot(found.x - x0, found.y - y0) if found else float('nan')
    print('nearest %.1f m: %3d points | start spot %.0f%%, refined there %.0f%% | search: %s,'
          ' %.1f m from the start spot' % (rmin, pts.shape[1], 100 * s0, 100 * ss, found, far))
