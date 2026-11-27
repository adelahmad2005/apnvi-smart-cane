"""Split a seq_replay track at time gaps (one piece per recording) and report each piece (16 Nov).

For each piece: where it starts and ends, and the furthest it strays from its start. For a spin on
the spot, start and end are really the same place and heading, so "end - start" is the error.
   python3 seg_eval.py /tmp/seq_NAME/track.npz
"""
import math
import sys

import numpy as np

p = np.load(sys.argv[1])['poses']           # t, x, y, yaw
cuts = [0] + [i + 1 for i in range(len(p) - 1) if p[i + 1, 0] - p[i, 0] > 30.0] + [len(p)]
for k in range(len(cuts) - 1):
    s = p[cuts[k]:cuts[k + 1]]
    t0, x0, y0, a0 = s[0]
    t1, x1, y1, a1 = s[-1]
    stray = np.hypot(s[:, 1] - x0, s[:, 2] - y0)
    turn = np.unwrap(s[:, 3])
    dyaw = math.degrees(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0)))
    print('piece %d: %.0f s, start %.2f %.2f %.0f deg, end %.2f %.2f %.0f deg | end-start %.2f m %.0f deg'
          ' | strays up to %.2f m | turned %.0f deg in all' % (
              k + 1, t1 - t0, x0, y0, math.degrees(a0), x1, y1, math.degrees(a1),
              math.hypot(x1 - x0, y1 - y0), dyaw, stray.max(),
              math.degrees(np.abs(np.diff(turn)).sum())))
