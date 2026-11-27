"""Does a recorded sweep fit a saved map? (16 Nov night, placing) No device needed.

Takes sweeps out of a recording (/scan_raw, with /odom for where the tracker said the device
was), filters them like scan_filter, and scores them on a saved map's walls with the map
manager's own code: at the spot the tracker gave, after a small refine, and with a search of the
whole map. Also searches with the sweep read the other way round (mirror). Draws one picture.
   python3 place_bag.py BAG_DIR MAP_NAME OUT.png [seconds into the recording ...]
"""
import glob
import math
import os
import sqlite3
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from apnvi_stage3.map_names import map_paths, saved_grid  # noqa: E402
from apnvi_stage3.relocalizer import combine, Relocalizer  # noqa: E402
from apnvi_stage3.scan_filter import filter_ranges, rear_mask  # noqa: E402

LASER_IN_BASE = (-0.70, 0.0, math.pi)       # stage3.launch.py: lidar_x, lidar_y, lidar_yaw

bag, name, out = sys.argv[1], sys.argv[2], sys.argv[3]
times = [float(t) for t in sys.argv[4:]] or [1.0]
database = map_paths(os.path.expanduser('~/.ros/apnvi_maps'), name)[0]
grid = saved_grid(database)
finder = Relocalizer(*grid)
print('map %s: %d x %d cells, corner (%.2f, %.2f), %d wall cells' % (
    name, grid[0].shape[1], grid[0].shape[0], grid[2][0], grid[2][1], int((grid[0] > 50).sum())))

db = sqlite3.connect(glob.glob(os.path.join(bag, '*.db3'))[0])
topics = dict(db.execute('select name, id from topics'))
scans = db.execute('select timestamp, data from messages where topic_id=? order by timestamp',
                   (topics['/scan_raw'],)).fetchall()
odoms = []
if '/odom' in topics:
    for stamp, data in db.execute('select timestamp, data from messages where topic_id=? '
                                  'order by timestamp', (topics['/odom'],)):
        m = deserialize_message(data, Odometry)
        q = m.pose.pose.orientation
        odoms.append((stamp, m.pose.pose.position.x, m.pose.pose.position.y,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))))
t0 = scans[0][0]
print('%d sweeps over %.0f s, %d /odom' % (len(scans), (scans[-1][0] - t0) / 1e9, len(odoms)))


def points_of(scan, upside_down):
    mask = rear_mask(scan.angle_min, scan.angle_increment, len(scan.ranges), 0.0,
                     math.radians(60))
    ranges = filter_ranges(scan.ranges, mask, 0.7)
    if upside_down:
        return finder.sweep_points(ranges, -scan.angle_min, -scan.angle_increment)
    return finder.sweep_points(ranges, scan.angle_min, scan.angle_increment)


drawn = None
for want in times:
    stamp, data = min(scans, key=lambda s: abs((s[0] - t0) / 1e9 - want))
    scan = deserialize_message(data, LaserScan)
    base = (0.0, 0.0, 0.0)
    if odoms:
        near = min(odoms, key=lambda o: abs(o[0] - stamp))
        base = near[1:]
    laser = combine(base, LASER_IN_BASE)
    print('--- %.1f s: tracker says base x %.2f y %.2f yaw %.0f deg; sweep %d beams, '
          'angle_min %.3f inc %.5f' % ((stamp - t0) / 1e9, base[0], base[1],
                                       math.degrees(base[2]), len(scan.ranges),
                                       scan.angle_min, scan.angle_increment))
    for label, flipped in (('as the map manager reads it', True), ('the other way round', False)):
        pts = points_of(scan, flipped)
        here = float(finder.score(pts, np.array([laser[0]]), np.array([laser[1]]),
                                  np.array([laser[2]]))[0])
        rx, ry, ra, rs = finder.refine(pts, *laser)
        found = finder.find(pts)
        print('  %-28s %3d points; at the tracker spot %3.0f%%, refined %3.0f%%; search: %s, '
              '%.2f m from the tracker spot' % (label, pts.shape[1], 100 * here, 100 * rs, found,
                                                math.hypot(found.x - laser[0], found.y - laser[1])))
        if flipped and drawn is None:
            drawn = (pts, laser, here, found)

pts, laser, here, found = drawn
g, res, org = grid
fig, ax = plt.subplots(figsize=(14, 10))
ax.imshow(np.where(g > 50, 0, np.where(g == 0, 230, 160)), cmap='gray', origin='lower',
          extent=(org[0], org[0] + g.shape[1] * res, org[1], org[1] + g.shape[0] * res),
          vmin=0, vmax=255)
for (px, py, pa), col, lab in ((laser, 'red', 'sweep at the tracker spot %.0f%%' % (100 * here)),
                               ((found.x, found.y, found.yaw), 'lime',
                                'search answer %.0f%%' % (100 * found.score))):
    wx = px + pts[0] * math.cos(pa) - pts[1] * math.sin(pa)
    wy = py + pts[0] * math.sin(pa) + pts[1] * math.cos(pa)
    ax.plot(wx, wy, 'o', color=col, ms=3, label=lab)
    ax.plot([px], [py], '*', color=col, ms=14)
ax.legend()
ax.set_title('%s with a sweep from %s' % (name, os.path.basename(bag.rstrip('/'))))
fig.savefig(out, dpi=100)
print('saved', out)
