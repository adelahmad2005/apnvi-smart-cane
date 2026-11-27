"""Find sudden jumps in a replayed track: steps where the heading or position changes faster than a
pushed device can (16 Nov). Prints the worst steps with the tracker's own account around them.
   python3 jump_find.py /tmp/seq_NAME/track.npz
"""
import math
import sys

import numpy as np

d = np.load(sys.argv[1], allow_pickle=True)
p = d['poses']                                  # t, x, y, yaw
t0 = p[0, 0]
dt = np.diff(p[:, 0])
dyaw = np.degrees(np.abs(np.arctan2(np.sin(np.diff(p[:, 3])), np.cos(np.diff(p[:, 3])))))
dpos = np.hypot(np.diff(p[:, 1]), np.diff(p[:, 2]))
rate = dyaw / np.maximum(dt, 1e-3)
print('steps: %d, median step %.3f s; heading rate median %.0f deg/s, 99th %.0f deg/s' % (
    len(dt), np.median(dt), np.median(rate), np.percentile(rate, 99)))
info = d['info'] if 'info' in d.files else None
if info is not None and len(info):
    print('info columns: %d' % info.shape[1])
worst = np.argsort(dyaw)[::-1][:12]
for i in sorted(worst):
    line = 't %6.1f s  pose %.2f %.2f %4.0f deg -> %.2f %.2f %4.0f deg  | turn %.0f deg, move %.2f m in %.2f s' % (
        p[i, 0] - t0, p[i, 1], p[i, 2], math.degrees(p[i, 3]), p[i + 1, 1], p[i + 1, 2],
        math.degrees(p[i + 1, 3]), dyaw[i], dpos[i], dt[i])
    if info is not None and len(info):
        k = np.argmin(np.abs(info[:, 0] - p[i + 1, 0]))
        line += ' | info ' + ' '.join('%.3g' % v for v in info[k, 1:])
    print(line)
