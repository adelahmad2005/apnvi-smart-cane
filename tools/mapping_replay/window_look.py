"""What the LiDAR and the tracker saw in one stretch of a replay, second by second (16 Nov).
   python3 window_look.py /tmp/seq_NAME/track.npz FROM_S TO_S
Per second: pose, how many beams came back (0.3 to 6 m) and how far, how many came back from close
by (under 1 m), the tracker's match score, points matched, and whether the sweep went into memory.
"""
import math
import sys

import numpy as np

d = np.load(sys.argv[1], allow_pickle=True)
a, b = float(sys.argv[2]), float(sys.argv[3])
P, ST, SR = d['poses'], d['sweep_t'], d['sweep_r']
info = d['info']                     # t, inliers ratio, memory size, key frame added, time, matches, ...
t0 = P[0, 0]
for s in np.arange(a, b, 1.0):
    pm = (P[:, 0] - t0 >= s) & (P[:, 0] - t0 < s + 1)
    sm = (ST - t0 >= s) & (ST - t0 < s + 1)
    im = (info[:, 0] - t0 >= s) & (info[:, 0] - t0 < s + 1)
    if not pm.any():
        continue
    x, y, yaw = P[pm][-1, 1], P[pm][-1, 2], math.degrees(P[pm][-1, 3])
    beams = near = 0
    med = float('nan')
    if sm.any():
        r = np.concatenate([SR[i] for i in np.where(sm)[0]])
        ok = np.isfinite(r) & (r > 0.3) & (r < 6.0)
        beams = ok.sum() / sm.sum()
        near = (np.isfinite(r) & (r > 0.05) & (r < 1.0)).sum() / sm.sum()
        med = np.median(r[ok]) if ok.any() else float('nan')
    sc = info[im]
    print('%5.0f s  %6.2f %6.2f %5.0f deg | beams %4.0f, close %3.0f, median %.1f m | score %.2f'
          ' matched %4.0f added %d/%d' % (
              s, x, y, yaw, beams, near, med, sc[:, 1].mean() if len(sc) else float('nan'),
              sc[:, 5].mean() if len(sc) else float('nan'), sc[:, 3].sum() if len(sc) else 0, len(sc)))
