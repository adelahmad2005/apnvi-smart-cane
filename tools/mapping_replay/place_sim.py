"""Would the device place itself? Runs the map manager's placing on a recording (16 Nov night).

No device needed. Takes /scan_raw and /odom from a recording and a saved map, and does what the
map manager does after a load: the start spot and the search answers are followed, and it says
when and where the device would be placed. The placing code is loaded from a file, so a changed
relocalizer.py can be tried before it is installed.
   python3 place_sim.py RELOCALIZER.py BAG_DIR MAP_NAME start [from_s] [ticks] [fresh]
       the start-up: from `from_s` seconds into the recording (default 2), `ticks` checks
       (default 12). "fresh": count the tracker's position from zero at from_s, as after a
       restart on that spot.
   python3 place_sim.py RELOCALIZER.py BAG_DIR MAP_NAME lap [every_s]
       one whole-map search every `every_s` seconds: is the answer where the tracker says?
"""
import glob
import importlib.util
import math
import os
import sqlite3
import sys

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

from apnvi_stage3.map_names import map_paths, saved_grid
from apnvi_stage3.scan_filter import filter_ranges, rear_mask

LASER = (-0.70, 0.0, math.pi)       # the LiDAR on the device (stage3.launch.py)
PLACED_FIT, SURE_FIT, SURE_GAP, CANDIDATE_FIT = 0.65, 0.75, 0.08, 0.6
for k, v in (a.split('=') for a in sys.argv if '=' in a and a[0].isupper()):
    globals()[k] = float(v)
argv = [a for a in sys.argv if not ('=' in a and a[0].isupper())]

spec = importlib.util.spec_from_file_location('reloc', argv[1])
rl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rl)
bag, name, mode = argv[2], argv[3], argv[4]
finder = rl.Relocalizer(*saved_grid(map_paths(os.path.expanduser('~/.ros/apnvi_maps'), name)[0]))
db = sqlite3.connect(glob.glob(os.path.join(bag, '*.db3'))[0])
topics = dict(db.execute('select name, id from topics'))
scans = db.execute('select timestamp, data from messages where topic_id=? order by timestamp',
                   (topics['/scan_raw'],)).fetchall()
odoms = []
for stamp, data in db.execute('select timestamp, data from messages where topic_id=? '
                              'order by timestamp', (topics['/odom'],)):
    m = deserialize_message(data, Odometry)
    q = m.pose.pose.orientation
    odoms.append((stamp, m.pose.pose.position.x, m.pose.pose.position.y,
                  math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))))
ostamps = np.array([o[0] for o in odoms])
sstamps = np.array([s[0] for s in scans])
t0 = scans[0][0]


def inverse(pose):
    x, y, yaw = pose
    cos, sin = math.cos(yaw), math.sin(yaw)
    return (-(cos * x + sin * y), -(-sin * x + cos * y), -yaw)


def at(seconds):
    """The sweep's points and the tracker's base pose at this time into the recording."""
    k = int(np.argmin(np.abs(sstamps - (t0 + seconds * 1e9))))
    scan = deserialize_message(scans[k][1], LaserScan)
    base = odoms[int(np.argmin(np.abs(ostamps - scans[k][0])))][1:]
    mask = rear_mask(scan.angle_min, scan.angle_increment, len(scan.ranges), 0.0,
                     math.radians(60))
    points = finder.sweep_points(filter_ranges(scan.ranges, mask, 0.7), -scan.angle_min,
                                 -scan.angle_increment)
    return points, base


def off(a, b):
    return (math.hypot(a[0] - b[0], a[1] - b[1]),
            abs(math.degrees(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2])))))


if mode == 'lap':
    every = float(argv[5]) if len(argv) > 5 else 8.0
    right = sure_right = sure_wrong = count = 0
    seconds = 0.0
    while seconds < (scans[-1][0] - t0) / 1e9:
        points, base = at(seconds)
        laser = rl.combine(base, LASER)
        found = finder.find(points)
        metres, degrees = off((found.x, found.y, found.yaw), laser)
        there = finder.refine(points, *finder.refine(points, *laser)[:3])[3]
        ok = metres < 1.0 and degrees < 20
        sure = found.score >= SURE_FIT and found.score - found.runner_up >= SURE_GAP
        count, right = count + 1, right + ok
        sure_right, sure_wrong = sure_right + (ok and sure), sure_wrong + (sure and not ok)
        print('%6.1f s base %6.2f %6.2f %5.0f | search %.2f/%.2f, %5.2f m %3.0f deg off: %s%s | '
              'fit at the tracker spot %.2f' % (seconds, base[0], base[1], math.degrees(base[2]),
                                                found.score, found.runner_up, metres, degrees,
                                                'RIGHT' if ok else 'wrong',
                                                ' SURE' if sure else '', there), flush=True)
        seconds += every
    print('right %d of %d; sure and right %d; sure and wrong %d' % (right, count, sure_right,
                                                                    sure_wrong))
    sys.exit()

begin = float(argv[5]) if len(argv) > 5 else 2.0
ticks = int(argv[6]) if len(argv) > 6 else 12
fresh = 'fresh' in argv
zero = inverse(at(begin)[1]) if fresh else (0.0, 0.0, 0.0)
candidates = rl.Candidates()
candidates.add((0.0, 0.0, 0.0), 0.0, start=True)
seconds = begin
for tick in range(ticks):
    points, base = at(seconds)
    seen = rl.combine(rl.combine(zero, base), LASER)
    try:
        candidates.update(finder, points, seen, at_start=LASER)
    except TypeError:                                   # the old code
        candidates.update(finder, points, seen)
    placed = None
    if tick % 3 == 0 or not hasattr(finder, 'refine_wide'):
        found = finder.find(points)
        seconds += 3.0
        if found.score >= SURE_FIT and found.score - found.runner_up >= SURE_GAP:
            placed = (rl.frame_offset((found.x, found.y, found.yaw), seen), 'search is sure %s' % found)
        else:
            candidates.add_answers([(found.x, found.y, found.yaw, found.score)] + found.others,
                                   seen, CANDIDATE_FIT)
    following = ', '.join('%s(%.1f, %.1f) %.0f%%' % (
        'START ' if p['start'] else '', *rl.combine(p['shift'], seen)[:2],
        100 * rl.Candidates.average(p)) for p in candidates.places)
    print('%5.1f s tick %2d: %s' % (seconds, tick, following), flush=True)
    placed = placed or candidates.winner(PLACED_FIT, SURE_GAP)
    if placed:
        base_on_map = rl.combine(rl.combine(placed[0], seen), inverse(LASER))
        metres, degrees = off(base_on_map, base)
        print('PLACED after %.0f s: %s. Device on the map: x %.2f y %.2f heading %.0f deg; the '
              'tracker (this map was started on START) says x %.2f y %.2f heading %.0f deg: '
              '%.2f m, %.0f deg apart' % (
                  seconds - begin, placed[1], base_on_map[0], base_on_map[1],
                  math.degrees(base_on_map[2]), base[0], base[1], math.degrees(base[2]),
                  metres, degrees))
        break
    seconds += 1.0
else:
    print('NOT PLACED in %d checks' % ticks)
