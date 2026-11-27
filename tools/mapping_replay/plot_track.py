# Draw the replayed lap: the path coloured by time, time marks every 20 s, the weak stretches
# (scan_flicker.py) in red, and the LiDAR points seen along the way (a rough picture of the room).
#   python3 plot_track.py track.npz out.png ["150-160,180-190,240-280"]
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

data = np.load(sys.argv[1])
P = data['poses']                       # t, x, y, yaw (movement tracker)
ST, SR = data['sweep_t'], data['sweep_r']
t0, amin, ainc, nbeams = data['meta']
weak = [tuple(float(v) for v in w.split('-')) for w in
        (sys.argv[3] if len(sys.argv) > 3 else '150-160,180-190,240-280').split(',')]
T = P[:, 0] - t0

# LiDAR -> base_link on the device (15 Nov): 0.70 m behind the wheels, turned 180 deg, upside down
LX, LYAW, FLIP = -0.70, np.pi, -1.0
ang = FLIP * (amin + np.arange(int(nbeams)) * ainc) + LYAW
pts = []
for ts, r in zip(ST, SR):
    i = np.searchsorted(P[:, 0], ts)
    if i <= 0 or i >= len(P):
        continue
    _, x, y, yaw = P[i]
    ok = np.isfinite(r) & (r > 0.1) & (r < 8.0)
    bx = LX + r[ok] * np.cos(ang[ok])
    by = r[ok] * np.sin(ang[ok])
    pts.append(np.c_[x + bx * np.cos(yaw) - by * np.sin(yaw), y + bx * np.sin(yaw) + by * np.cos(yaw)])
pts = np.vstack(pts) if pts else np.zeros((0, 2))

fig, ax = plt.subplots(figsize=(13, 11))
ax.scatter(pts[:, 0], pts[:, 1], s=0.3, c='0.75', lw=0, label='what the LiDAR saw')
sc = ax.scatter(P[:, 1], P[:, 2], c=T, cmap='viridis', s=4, lw=0)
for a, b in weak:
    m = (T >= a) & (T <= b)
    ax.plot(P[m, 1], P[m, 2], color='red', lw=4, alpha=0.8)
    if m.any():
        j = np.where(m)[0][len(np.where(m)[0]) // 2]
        ax.annotate('weak %d-%d s' % (a, b), (P[j, 1], P[j, 2]), xytext=(12, -14),
                    textcoords='offset points', color='red', fontsize=11, weight='bold')
for mark in range(0, int(T[-1]) + 1, 20):
    j = np.searchsorted(T, mark)
    if j < len(P):
        ax.plot(P[j, 1], P[j, 2], 'ko', ms=4)
        ax.annotate('%d s' % mark, (P[j, 1], P[j, 2]), xytext=(5, 5), textcoords='offset points',
                    fontsize=9)
ax.plot(P[0, 1], P[0, 2], 'g^', ms=16, label='START (where it really ended too)')
ax.plot(P[-1, 1], P[-1, 2], 'rs', ms=13, label='where the tracker thinks it ended')
k = max(1, len(P) // 40)
ax.quiver(P[::k, 1], P[::k, 2], np.cos(P[::k, 3]), np.sin(P[::k, 3]), color='k', width=0.002,
          scale=40, alpha=0.6)
fig.colorbar(sc, ax=ax, label='seconds into the lap')
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xlabel('x (m)  - the device faced +x at START')
ax.set_ylabel('y (m)')
ax.set_title('ESB117 lap, 15 Nov: path as the movement tracker saw it (arrows: which way it faced)')
ax.legend(loc='upper right', markerscale=3)
fig.tight_layout()
fig.savefig(sys.argv[2], dpi=110)
print('saved', sys.argv[2])
