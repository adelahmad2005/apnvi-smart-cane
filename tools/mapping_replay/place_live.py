"""Is the device where the map manager shows it? (16 Nov night) Device running, a map loaded.

Takes one live sweep and the LiDAR's place on the map (the frames map -> laser, as everyone
else sees them), and draws the saved map with the sweep on it: green points are on a wall,
orange are close things on free floor (people, chairs), magenta are far things on free floor,
red are beyond the walls. Prints the fit the map manager computes.
   python3 place_live.py MAP_NAME OUT.png
"""
import math
import os
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import rclpy.duration  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

from apnvi_stage3.map_manager_node import sweep_angles  # noqa: E402
from apnvi_stage3.map_names import saved_grid, work_path  # noqa: E402
from apnvi_stage3.relocalizer import BEYOND, FLOOR, HIT, Relocalizer  # noqa: E402

name, out = sys.argv[1], sys.argv[2]
grid = saved_grid(work_path(os.path.expanduser('~/.ros/apnvi_maps'), name))
finder = Relocalizer(*grid)
rclpy.init()
node = rclpy.create_node('place_live')
buf = Buffer()
TransformListener(buf, node)
scans = []
node.create_subscription(LaserScan, '/scan', scans.append, 5)
end = time.time() + 6
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
scan = scans[-1]
tf = buf.lookup_transform('map', scan.header.frame_id, rclpy.time.Time(),
                          timeout=rclpy.duration.Duration(seconds=3.0))
q = tf.transform.rotation
yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
x0, y0 = tf.transform.translation.x, tf.transform.translation.y
pts = finder.sweep_points(*sweep_angles(scan, (1 - 2 * (q.x ** 2 + q.y ** 2)) < 0))
fit = float(finder.score(pts, x0, y0, yaw)[0])
found = finder.find(pts)
print('LiDAR on the map: x %.2f y %.2f yaw %.0f deg; fit there %.0f%%' % (
    x0, y0, math.degrees(yaw), 100 * fit))
print('whole-map search with this sweep: %s; %.2f m from where it is shown' % (
    found, math.hypot(found.x - x0, found.y - y0)))
g, res, org = grid
wx = x0 + pts[0] * math.cos(yaw) - pts[1] * math.sin(yaw)
wy = y0 + pts[0] * math.sin(yaw) + pts[1] * math.cos(yaw)
kind = finder.look(finder.kind, wx, wy)
close = pts[2] < 0.5
fig, ax = plt.subplots(figsize=(13, 9))
ax.imshow(np.where(g > 50, 0, np.where(g == 0, 230, 160)), cmap='gray', origin='lower',
          extent=(org[0], org[0] + g.shape[1] * res, org[1], org[1] + g.shape[0] * res),
          vmin=0, vmax=255)
for sel, col, lab in ((kind == HIT, 'lime', 'on a wall'),
                      ((kind == FLOOR) & close, 'orange', 'on free floor, close'),
                      ((kind == FLOOR) & ~close, 'magenta', 'on free floor, far'),
                      (kind == BEYOND, 'red', 'beyond the walls')):
    ax.plot(wx[sel], wy[sel], 'o', color=col, ms=4, label='%s: %d' % (lab, int(sel.sum())))
ax.plot([x0], [y0], '*', color='blue', ms=16, label='the LiDAR')
ax.legend()
ax.set_title('%s: the live sweep where the device is shown, fit %.0f%%' % (name, 100 * fit))
fig.savefig(out, dpi=80)
print('saved', out)
