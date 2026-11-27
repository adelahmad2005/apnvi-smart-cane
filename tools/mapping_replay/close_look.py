"""Where are the close returns (under 1 m) around the device, and do they move with it? (16 Nov)
   python3 close_look.py /tmp/seq_NAME/track.npz FROM_S TO_S
Per recorded sweep: the device-frame directions (0 = straight ahead, + = left) of runs of close
beams, with their range. A run that keeps the same direction and range while the device moves or
turns is being carried along (a person, the handle); furniture changes as the device moves.
"""
import math
import sys

import numpy as np

d = np.load(sys.argv[1], allow_pickle=True)
a, b = float(sys.argv[2]), float(sys.argv[3])
P, ST, SR = d['poses'], d['sweep_t'], d['sweep_r']
t0, amin, ainc, nb = d['meta']
ang = np.degrees(-1.0 * (amin + np.arange(int(nb)) * ainc) + np.pi)     # as in room_angle.py
ang = (ang + 180) % 360 - 180
for ts, r in zip(ST, SR):
    s = ts - t0
    if s < a or s >= b:
        continue
    i = min(np.searchsorted(P[:, 0], ts), len(P) - 1)
    close = np.isfinite(r) & (r > 0.05) & (r < 1.0)
    runs, cur = [], []
    for k in np.argsort(ang):
        if close[k]:
            cur.append(k)
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    runs = [c for c in runs if len(c) >= 4]
    desc = '  '.join('%+4.0f..%+4.0f deg @%.2f m' % (ang[c[0]], ang[c[-1]], np.median(r[c])) for c in runs)
    print('%5.1f s  heading %5.0f | %s' % (s, math.degrees(P[i, 3]), desc))
