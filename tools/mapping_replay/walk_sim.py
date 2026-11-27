"""Walk-and-match on a recording (17 Nov): does the device place itself, and right? No device.

Runs the map manager's not-placed checks (placing.py's Placer) on a recording, one check per
second of the recording (a whole-map search counts 3 s, as on the Jetson), from many start
times. At each start the tracker counts from zero, as after a restart on that spot.
   python3 walk_sim.py CODE_DIR RECORDING MAP [every_s] [checks] [start=...] [old=FILE]
CODE_DIR    the folder that holds the apnvi_stage3 package to try (a changed copy is fine)
RECORDING   a bag folder (/scan_raw and /odom; needs ROS sourced) or an .npz made by
            export_lap.py (no ROS needed: can run on any PC with numpy and opencv)
MAP         a saved map name (in ~/.ros/apnvi_maps) or an .npz (grid, resolution, origin)
every_s     a start every this many seconds (default 10); checks: at most this many (40)
start=map   the start spot is where the map was started (no entrance marked; the default)
start=here[,dx,dy,deg]   the start spot is where the device really is at each start (as if it
            stood in the doorway that was marked as the entrance), moved by dx, dy metres (in
            the device's own frame) and turned deg: "started not exactly on the spot"
old=FILE    instead run the 16 Nov rules with that relocalizer.py (one sweep, placed_fit 0.55,
            the start spot wins a tie)
The right answer comes from following the whole recording from its start (laps start on the
map's start spot) with small refines, as the map manager does once placed. Right: within
0.5 m and 10 degrees.
"""
import glob
import importlib.util
import math
import os
import sys

import numpy as np

LASER = (-0.70, 0.0, math.pi)       # the LiDAR on the device (stage3.launch.py), upside down
RIGHT_M, RIGHT_DEG = 0.5, 10.0
OLD_PLACED_FIT, OLD_SURE_FIT, OLD_SURE_GAP = 0.55, 0.75, 0.08
args = [a for a in sys.argv[1:] if '=' not in a]
options = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
sys.path.insert(0, os.path.abspath(args[0]))
from apnvi_stage3 import placing, relocalizer as rl  # noqa: E402

recording, map_name = args[1], args[2]
every = float(args[3]) if len(args) > 3 else 10.0
most = int(args[4]) if len(args) > 4 else 40


def load_map(name):
    if name.endswith('.npz'):
        data = np.load(name)
        return data['grid'], float(data['resolution']), tuple(data['origin'])
    from apnvi_stage3.map_names import map_paths, saved_grid
    return saved_grid(map_paths(os.path.expanduser('~/.ros/apnvi_maps'), name)[0])


def load_recording(path):
    """Give (stamps s, ranges list, angle_min, angle_inc, odom rows stamp s, x, y, yaw)."""
    if path.endswith('.npz'):
        data = np.load(path)
        return (data['stamps'] / 1e9, data['ranges'], data['angle_min'], data['angle_inc'],
                np.column_stack([data['odom'][:, 0] / 1e9, data['odom'][:, 1:]]))
    import sqlite3
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from apnvi_stage3.scan_filter import filter_ranges, rear_mask
    db = sqlite3.connect(glob.glob(os.path.join(path, '*.db3'))[0])
    topics = dict(db.execute('select name, id from topics'))
    rows = 'select timestamp, data from messages where topic_id=? order by timestamp'
    odom = []
    for stamp, data in db.execute(rows, (topics['/odom'],)):
        m = deserialize_message(data, Odometry)
        q = m.pose.pose.orientation
        odom.append((stamp / 1e9, m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))))
    stamps, ranges, first, step = [], [], [], []
    for stamp, data in db.execute(rows, (topics['/scan_raw'],)):
        s = deserialize_message(data, LaserScan)
        mask = rear_mask(s.angle_min, s.angle_increment, len(s.ranges), 0.0, math.radians(60))
        stamps.append(stamp / 1e9)
        ranges.append(np.array(filter_ranges(s.ranges, mask, 0.7)))
        first.append(s.angle_min)
        step.append(s.angle_increment)
    return np.array(stamps), ranges, np.array(first), np.array(step), np.array(odom)


finder = rl.Relocalizer(*load_map(map_name))
stamps, ranges, angle_min, angle_inc, odom = load_recording(recording)
t0 = stamps[0]
length = stamps[-1] - t0


def at(seconds):
    """The sweep's points and the tracker's device pose, this many seconds into the recording."""
    k = int(np.argmin(np.abs(stamps - (t0 + seconds))))
    o = odom[int(np.argmin(np.abs(odom[:, 0] - stamps[k])))]
    points = finder.sweep_points(ranges[k], -angle_min[k], -angle_inc[k])  # upside down
    return points, (o[1], o[2], o[3])


def apart(a, b):
    return (math.hypot(a[0] - b[0], a[1] - b[1]),
            abs(math.degrees(placing.turn_between(a[2], b[2]))))


def follow(anchor):
    """Follow the whole recording from `anchor` (the tracker's frame on the map) like a placed
    device. Returns (device pose each second, fit each second)."""
    track, fits = {}, []
    for second in range(int(length) + 1):
        points, base = at(second)
        seen = rl.combine(base, LASER)
        shown = rl.combine(anchor, seen)
        x, y, yaw, fit = finder.refine(points, *shown)
        if fit < 0.6 or second == 0:
            x, y, yaw, fit = max((x, y, yaw, fit), finder.refine_wide(points, *shown),
                                 key=lambda answer: answer[3])
        if fit >= 0.55:
            anchor = rl.frame_offset((x, y, yaw), seen)
        fits.append(fit)
        track[second] = rl.combine(anchor, base)
    return track, fits


def truth_track():
    """
    Where the device really is, each second.

    Laps start on the map's start spot with the tracker at zero, but not every recording does
    (lap_202918 starts with the tracker at -0.46, 0.78, -177 deg). So the whole recording is
    followed from zero and from the best answers of a whole-map search on the first sweep, and
    the start that fits best over the whole recording is the truth.
    """
    points, base = at(0)
    seen = rl.combine(base, LASER)
    found = finder.find(points)
    starts = [('tracker zero', (0.0, 0.0, 0.0))] + [
        ('search answer %.0f%%' % (100 * a[3]), rl.frame_offset(a[:3], seen))
        for a in [(found.x, found.y, found.yaw, found.score)] + found.others[:3]]
    tried = [(name,) + follow(anchor) for name, anchor in starts]
    name, track, fits = max(tried, key=lambda t: np.mean(t[2]))
    print('truth from %s (of %s): followed %d s, fit mean %.2f, lowest %.2f, under 0.6 in %d s'
          % (name, ', '.join('%s %.2f' % (t[0], np.mean(t[2])) for t in tried), len(fits),
             np.mean(fits), min(fits), sum(f < 0.6 for f in fits)), flush=True)
    return track


def start_spot(truth_now):
    """The start spot for a run starting with the device at truth_now (see start= above)."""
    how = options.get('start', 'map').split(',')
    if how[0] == 'map':
        return (0.0, 0.0, 0.0)
    dx, dy, deg = (float(v) for v in (how[1:] + ['0', '0', '0'])[:3])
    return rl.combine(truth_now, (dx, dy, math.radians(deg)))


def run_new(begin, start):
    """placing.Placer from `begin`. Returns (seconds, metres, shift, zero, why) or None."""
    placer = placing.Placer(finder, start)
    seconds, zero = begin, rl.inverse(at(begin)[1])
    for _ in range(most):
        if seconds > length:
            return None
        points, base = at(seconds)
        seen = rl.combine(rl.combine(zero, base), LASER)
        searched = placer.searched_at
        won = placer.step(points, seen, LASER)
        if 'v' in options:
            print('   %5.1f s %s' % (seconds, placer.note))
        if won is not None:
            return seconds - begin, placer.walked_m, won[0], zero, won[1], seconds
        seconds += 3.0 if placer.searched_at != searched else 1.0
    return None


def run_old(begin, start, old):
    """The 16 Nov map manager's not-placed checks, with that relocalizer module."""
    candidates, seconds = old.Candidates(), begin
    zero = rl.inverse(at(begin)[1])
    at_start = rl.combine(start, LASER)
    walked, last = 0.0, None
    for tick in range(most):
        if seconds > length:
            return None
        points, base = at(seconds)
        seen = rl.combine(rl.combine(zero, base), LASER)
        walked += 0.0 if last is None else math.hypot(seen[0] - last[0], seen[1] - last[1])
        last = seen
        if tick == 0:
            candidates.add(rl.frame_offset(at_start, seen), 0.0, start=True)
            candidates.places[0]['fits'] = []
        candidates.update(old_finder, points, seen, at_start)
        if tick % 3 == 0:
            found = old_finder.find(points)
            seconds += 2.0
            if found.score >= OLD_SURE_FIT and found.score - found.runner_up >= OLD_SURE_GAP:
                return (seconds - begin, walked, rl.frame_offset((found.x, found.y, found.yaw),
                                                                 seen), zero, 'search sure',
                        seconds)
            candidates.add_answers([(found.x, found.y, found.yaw, found.score)]
                                   + found.others, seen, 0.6)
        won = candidates.winner(OLD_PLACED_FIT, OLD_SURE_GAP)
        if won is not None:
            return seconds - begin, walked, won[0], zero, won[1], seconds
        seconds += 1.0
    return None


old = None
if 'old' in options:
    spec = importlib.util.spec_from_file_location('old_relocalizer', options['old'])
    old = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    old_finder = old.Relocalizer(*load_map(map_name))
truth = truth_track()
counts = {'right': 0, 'WRONG': 0, 'not placed': 0}
times, walks = [], []
begin = 2.0
while begin < length - 10:
    now = truth[int(round(begin))]
    start = start_spot(now)
    result = run_old(begin, start, old) if old else run_new(begin, start)
    if result is None:
        counts['not placed'] += 1
        print('%5.0f s at (%5.1f, %5.1f): NOT PLACED in %d checks' % (begin, now[0], now[1], most),
              flush=True)
    else:
        took, walked, shift, zero, why, ended = result
        points, base = at(ended)
        device = rl.combine(rl.combine(shift, rl.combine(zero, base)), (0.0, 0.0, 0.0))
        metres, degrees = apart(device, truth[min(int(round(ended)), int(length))])
        verdict = 'right' if metres < RIGHT_M and degrees < RIGHT_DEG else 'WRONG'
        counts[verdict] += 1
        if verdict == 'right':
            times.append(took)
            walks.append(walked)
        print('%5.0f s at (%5.1f, %5.1f): placed %s after %2.0f s, %4.1f m walked, %.2f m %3.0f '
              'deg off; %s' % (begin, now[0], now[1], verdict, took, walked, metres, degrees, why),
              flush=True)
    begin += every
print('SUMMARY %s %s start=%s: right %d, wrong %d, not placed %d; right ones took %s s and '
      '%s m walked (median)' % (
          os.path.basename(recording), 'old rules' if old else 'walk-and-match',
          options.get('start', 'map'), counts['right'], counts['WRONG'], counts['not placed'],
          '%.0f' % np.median(times) if times else '-', '%.1f' % np.median(walks) if walks else '-'))
