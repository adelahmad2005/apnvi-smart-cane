import sys, math, time
import numpy as np
sys.path.insert(0, '.')
from sim import make_room, lap_path, make_sweeps
from lidar_odom import Tracker

def rear_filter(sweeps, half=math.radians(60), rng_m=0.7):
    out = []
    for sw in sweeps:
        a = sw['angle_min'] + np.arange(len(sw['ranges'])) * sw['angle_inc']
        a = np.arctan2(np.sin(a), np.cos(a))
        r = sw['ranges'].copy(); r[(np.abs(a) <= half) & (r < rng_m)] = np.inf
        out.append(dict(sw, ranges=r))
    return out

def end_err(log, truth):
    x, y, th = log[-1, 1:4]
    ex, ey = x - (truth[-1, 1] - truth[0, 1]), y - (truth[-1, 2] - truth[0, 2])
    # tracker starts at 0,0,0; truth start heading 0
    dth = math.degrees(th - truth[-1, 3]); dth = (dth + 180) % 360 - 180
    return math.hypot(ex, ey), dth

def turn_counted(log):
    return math.degrees(np.sum((np.diff(log[:, 3]) + math.pi) % (2 * math.pi) - math.pi))

if __name__ == '__main__':
    rate = float(sys.argv[1]) if len(sys.argv) > 1 else 35
    segs, _ = make_room()
    path = lap_path(turn_rate=math.radians(rate))
    print('lap: %.0f s, %d segments' % (path[0][-1], len(segs)))
    CASES = [
        ('no skew, rt', dict(skew=False), dict(mode='rt')),
        ('skew, rt', dict(), dict(mode='rt')),
        ('no skew, rt map 500', dict(skew=False), dict(mode='rt', map_max=500)),
        ('skew, rt map 500', dict(), dict(mode='rt', map_max=500)),
        ('skew, rt map 500 deskew', dict(), dict(mode='rt', map_max=500, deskew=True)),
        ('skew, rt map 500 deskew wrong way', dict(), dict(mode='rt', map_max=500, deskew=True, time_dir=1)),
        ('skew, ref', dict(), dict(mode='ref')),
    ]
    for name, kw_sim, kw_tr in CASES:
        t0 = time.time()
        sweeps, truth = make_sweeps(path, segs, **kw_sim)
        sweeps = rear_filter(sweeps)
        tr = Tracker(**kw_tr)
        log = tr.run(sweeps)
        d, a = end_err(log, truth)
        print('%-34s end %.2f m %+6.1f deg | turn counted %+.0f (true %+.0f) | lost %d | min ratio %.2f | map %d | %.0f s'
              % (name, d, a, turn_counted(log), math.degrees(truth[-1, 3] - truth[0, 3]), int(log[:, 5].sum()),
                 log[1:, 4].min(), len(tr.map), time.time() - t0))
