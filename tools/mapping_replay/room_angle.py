# When does the heading slip? The room's walls are straight and never turn, so the direction of the
# long straight edges in each sweep (as placed by the movement tracker) should stay the same all lap.
# Any change of that direction is heading error. Draws it against time.
#   python3 room_angle.py track.npz out.png
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

d = np.load(sys.argv[1])
P, ST, SR = d['poses'], d['sweep_t'], d['sweep_r']
t0, amin, ainc, nb = d['meta']
LX, LYAW, FLIP = -0.70, np.pi, -1.0
ang = FLIP * (amin + np.arange(int(nb)) * ainc) + LYAW

times, angles, strength = [], [], []
for ts, r in zip(ST, SR):
    i = np.searchsorted(P[:, 0], ts)
    if i <= 0 or i >= len(P):
        continue
    _, x, y, yaw = P[i]
    ok = np.isfinite(r) & (r > 0.3) & (r < 6.0)
    bx = LX + r * np.cos(ang)
    by = r * np.sin(ang)
    wx = bx * np.cos(yaw) - by * np.sin(yaw)
    wy = bx * np.sin(yaw) + by * np.cos(yaw)
    # direction of each short edge between neighbouring beams that are both seen and close together
    seg_ok = ok[:-1] & ok[1:]
    dx, dy = np.diff(wx), np.diff(wy)
    length = np.hypot(dx, dy)
    seg_ok &= (length > 0.02) & (length < 0.15)
    if seg_ok.sum() < 40:
        continue
    a = np.mod(np.arctan2(dy[seg_ok], dx[seg_ok]), np.pi / 2)        # walls: mod 90 deg
    # circular mean on the 90-degree circle, weighted by edge length
    w = length[seg_ok]
    c, s = np.sum(w * np.cos(4 * a)), np.sum(w * np.sin(4 * a))
    angles.append(np.degrees(np.arctan2(s, c) / 4))
    strength.append(np.hypot(c, s) / np.sum(w))
    times.append(ts - t0)
times, angles, strength = map(np.array, (times, angles, strength))
angles = np.degrees(np.unwrap(np.radians(angles) * 4) / 4)      # follow it smoothly past 0/90
angles -= np.median(angles[:20])                                # 0 = the walls as seen at START
good = strength > 0.3

fig, ax = plt.subplots(2, 1, figsize=(13, 7), sharex=True,
                       gridspec_kw={'height_ratios': [3, 1]})
ax[0].plot(times[good], angles[good], '.', ms=3, color='tab:blue', label='wall direction, each sweep')
k = 9
if good.sum() > k:
    sm = np.convolve(angles[good], np.ones(k) / k, mode='same')
    ax[0].plot(times[good], sm, color='k', lw=2, label='smoothed')
for a, b in ((150, 160), (180, 190), (240, 280)):
    ax[0].axvspan(a, b, color='red', alpha=0.15)
    ax[1].axvspan(a, b, color='red', alpha=0.15)
ax[0].set_ylabel('heading error (deg)\n(walls turned by)')
ax[0].grid(True, alpha=0.3)
ax[0].legend()
ax[0].set_title('When does the heading slip? 0 = walls as seen at START. Red: the weak stretches')
ax[1].plot(times, strength, color='0.4')
ax[1].set_ylabel('how wall-like')
ax[1].set_xlabel('seconds into the lap')
ax[1].grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(sys.argv[2], dpi=100)
for start in range(0, int(times[-1]) + 1, 20):
    m = good & (times >= start) & (times < start + 20)
    if m.any():
        print('%3d-%3d s: heading error %+5.1f deg' % (start, start + 20, np.median(angles[m])))
print('saved', sys.argv[2])
