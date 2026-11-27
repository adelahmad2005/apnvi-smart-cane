"""Why isn't the device placed? (16 Nov night) Device running with a loaded map, on START.

Takes one live sweep and where the LiDAR is in RTAB-Map's frame (as the map manager does), scores
it on the saved map's walls at that spot, searches nearby (2 m, all turns) for the best fit, and
draws the saved map with the sweep at both.
   python3 place_look.py MAP_NAME OUT.png
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
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

from apnvi_stage3.map_manager_node import RTABMAP_FRAME, sweep_angles  # noqa: E402
from apnvi_stage3.map_names import saved_grid, work_path  # noqa: E402
from apnvi_stage3.relocalizer import Relocalizer  # noqa: E402

name, out = sys.argv[1], sys.argv[2]
flip = sys.argv[3] if len(sys.argv) > 3 else "auto"   # auto, yes (reverse angles), no
grid = saved_grid(work_path(os.path.expanduser('~/.ros/apnvi_maps'), name))
finder = Relocalizer(*grid)
rclpy.init()
node = rclpy.create_node('place_look')
buf = Buffer()
TransformListener(buf, node)
got = {}
node.create_subscription(LaserScan, '/scan', lambda m: got.setdefault('scan', m), qos_profile_sensor_data)
end = time.time() + 10
while time.time() < end and 'scan' not in got:
    rclpy.spin_once(node, timeout_sec=0.2)
for _ in range(40):
    rclpy.spin_once(node, timeout_sec=0.1)
scan = got['scan']
tf = buf.lookup_transform(RTABMAP_FRAME, scan.header.frame_id, rclpy.time.Time(),
                         timeout=rclpy.duration.Duration(seconds=3.0))
q = tf.transform.rotation
yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
up_down = (1 - 2 * (q.x ** 2 + q.y ** 2)) < 0
x0, y0 = tf.transform.translation.x, tf.transform.translation.y
if flip != "auto":
    up_down = flip == "yes"
pts = finder.sweep_points(*sweep_angles(scan, up_down))
s0 = float(finder.score(pts, np.array([x0]), np.array([y0]), np.array([yaw]))[0])
print('LiDAR in %s: x %.2f y %.2f yaw %.0f deg, upside down %s, %d points' % (
    RTABMAP_FRAME, x0, y0, math.degrees(yaw), up_down, pts.shape[1]))
print('fit at the start spot: %.0f%%' % (100 * s0))
gx, gy = np.meshgrid(np.arange(-2, 2.01, 0.1), np.arange(-2, 2.01, 0.1))
best = (s0, x0, y0, yaw)
for dyaw in np.radians(np.arange(-180, 180, 3)):
    sc = finder.score(pts, x0 + gx.ravel(), y0 + gy.ravel(), np.full(gx.size, yaw + dyaw))
    k = int(sc.argmax())
    if sc[k] > best[0]:
        best = (float(sc[k]), x0 + gx.ravel()[k], y0 + gy.ravel()[k], yaw + dyaw)
bx, by, byaw = finder.refine(pts, best[1], best[2], best[3])[:3]
bs = float(finder.score(pts, np.array([bx]), np.array([by]), np.array([byaw]))[0])
print('best within 2 m: %.0f%% at x %.2f y %.2f yaw %.0f deg (moved %.2f m, turned %.0f deg)' % (
    100 * bs, bx, by, math.degrees(byaw), math.hypot(bx - x0, by - y0),
    math.degrees(math.atan2(math.sin(byaw - yaw), math.cos(byaw - yaw)))))
g, res, org = grid
fig, ax = plt.subplots(figsize=(12, 9))
ax.imshow(np.where(g >= 50, 0, np.where(g == 0, 230, 160)), cmap='gray', origin='lower',
          extent=(org[0], org[0] + g.shape[1] * res, org[1], org[1] + g.shape[0] * res), vmin=0, vmax=255)
for (px, py, pa), col, lab in (((x0, y0, yaw), 'red', 'sweep at the start spot %.0f%%' % (100 * s0)),
                               ((bx, by, byaw), 'lime', 'best fit within 2 m %.0f%%' % (100 * bs))):
    wx = px + pts[0] * math.cos(pa) - pts[1] * math.sin(pa)
    wy = py + pts[0] * math.sin(pa) + pts[1] * math.cos(pa)
    ax.plot(wx, wy, '.', color=col, ms=4, label=lab)
ax.legend()
ax.set_title('Saved map %s: walls black, free light, unknown grey' % name)
fig.savefig(out, dpi=90)
print('saved', out)
