# Look inside the recorded lap: does the LiDAR's view flicker (desk edges coming and going), and
# which way can the sweeps not pin the position down?  (15 Nov)
#   source /opt/ros/humble/setup.bash; python3 /tmp/scan_flicker.py [bag_dir]
# Read only: it never changes the bag.
import math, sys, glob, os
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.expanduser('~/lap_bags/lap_*')))[-1]
JUMP = 0.5        # m: a beam "jumps" if its range changes by more than this between sweeps
BACK = 0.15       # m: ...and "comes back" if it returns within this of where it was
WIN = 3           # sweeps: how soon it must come back to count as flicker (0.3 s)
REAR_CUT, REAR_HALF = 0.7, math.radians(60)   # same as scan_filter: drop close points behind

# ---- read the bag -------------------------------------------------------------------------
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
scans, tf_static = [], []
while r.has_next():
    topic, data, t = r.read_next()
    msg = deserialize_message(data, get_message(types[topic]))
    if topic == '/scan_raw':
        scans.append((t * 1e-9, msg))
    elif topic == '/tf_static':
        tf_static += msg.transforms

# LiDAR -> base_link in 2D (the LiDAR is mounted upside down: roll pi flips its y)
tx = ty = yaw = 0.0
flip = 1.0
for tr in tf_static:
    if tr.header.frame_id == 'base_link' and 'laser' in tr.child_frame_id:
        q = tr.transform.rotation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        flip = -1.0 if abs(abs(roll) - math.pi) < 0.3 else 1.0
        tx, ty = tr.transform.translation.x, tr.transform.translation.y
        print('LiDAR frame %s: x %.2f y %.2f yaw %.0f deg, upside down: %s'
              % (tr.child_frame_id, tx, ty, math.degrees(yaw), flip < 0))

m0 = scans[0][1]
N = len(m0.ranges)
ang_l = m0.angle_min + np.arange(N) * m0.angle_increment
# beam direction in base_link (upside down: the beam angle is mirrored)
ang_b = np.arctan2(np.sin(flip * ang_l + yaw), np.cos(flip * ang_l + yaw))
R = np.array([[s.ranges[i] if len(s.ranges) == N else np.nan for i in range(N)] for _, s in scans],
             dtype=float)
R[~np.isfinite(R) | (R < m0.range_min) | (R > m0.range_max)] = np.nan
rear = np.abs(np.abs(ang_b) - math.pi) < REAR_HALF
R[:, rear] = np.where(R[:, rear] < REAR_CUT, np.nan, R[:, rear])
T = np.array([t for t, _ in scans]); T -= T[0]
print('bag %s: %d sweeps, %.0f s, %.1f per second, %d beams (%.2f deg apart)'
      % (os.path.basename(BAG), len(T), T[-1], len(T) / T[-1], N, math.degrees(m0.angle_increment)))

# ---- 1. still or moving -------------------------------------------------------------------
d = np.abs(np.diff(R, axis=0))
still = np.r_[False, np.nanmedian(d, axis=1) < 0.01]          # whole view barely changed
print('device still in %d sweeps (%.0f%%)' % (still.sum(), 100 * still.mean()))

# ---- 2. flicker: jump away and come back within WIN sweeps --------------------------------
flick = np.zeros_like(R, dtype=bool)
near_r, far_r, flick_ang = [], [], []
for k in range(1, len(R) - WIN):
    a, b = R[k - 1], R[k]
    jumped = np.abs(b - a) > JUMP
    for w in range(1, WIN + 1):
        c = R[k + w]
        back = jumped & (np.abs(c - a) < BACK)
        flick[k] |= back
    idx = np.where(flick[k])[0]
    for i in idx:
        near_r.append(min(a[i], R[k, i])); far_r.append(max(a[i], R[k, i])); flick_ang.append(ang_b[i])
valid = np.isfinite(R)
frac = flick.sum(axis=1) / np.maximum(valid.sum(axis=1), 1)
print('\nFLICKER (a beam jumps > %.1f m and comes back within %d sweeps)' % (JUMP, WIN))
print('  per sweep, moving: mean %.1f%% of points, 90th pct %.1f%%'
      % (100 * frac[~still].mean(), 100 * np.percentile(frac[~still], 90)))
if still.any():
    print('  per sweep, still : mean %.1f%% of points' % (100 * frac[still].mean()))
near_r, far_r, flick_ang = map(np.array, (near_r, far_r, flick_ang))
if len(near_r):
    print('  near end of the jump: median %.2f m (25-75%%: %.2f-%.2f m)'
          % (np.median(near_r), *np.percentile(near_r, [25, 75])))
    print('  far end of the jump : median %.2f m (25-75%%: %.2f-%.2f m)'
          % (np.median(far_r), *np.percentile(far_r, [25, 75])))
    deg = np.degrees(flick_ang)
    side = (np.abs(deg) > 45) & (np.abs(deg) < 135)
    front = np.abs(deg) <= 45
    print('  where: front (+-45) %.0f%%, sides %.0f%%, back %.0f%%  (all beams: front %.0f%% sides %.0f%%)'
          % (100 * front.mean(), 100 * side.mean(), 100 * (1 - front.mean() - side.mean()),
             100 * (np.abs(np.degrees(ang_b)) <= 45).mean(),
             100 * ((np.abs(np.degrees(ang_b)) > 45) & (np.abs(np.degrees(ang_b)) < 135)).mean()))

# ---- 3. which way can a sweep not pin the position down? ----------------------------------
# For each point, the direction its surface faces (normal from its neighbours). A sweep pins
# the position firmly in a direction only if many surfaces face that way. H = sum n n^T.
def weak_dir(r):
    x, y = r * np.cos(ang_b), r * np.sin(ang_b)
    ok = np.isfinite(r)
    H = np.zeros((2, 2)); used = 0
    for i in range(2, N - 2):
        sl = slice(i - 2, i + 3)
        if not ok[sl].all():
            continue
        px, py = x[sl], y[sl]
        if math.hypot(px[-1] - px[0], py[-1] - py[0]) > 0.3:   # neighbours on different surfaces
            continue
        P = np.c_[px - px.mean(), py - py.mean()]
        w, v = np.linalg.eigh(P.T @ P)
        n = v[:, 0]
        H += np.outer(n, n); used += 1
    if used < 30:
        return None
    w, v = np.linalg.eigh(H)
    return w[0] / w[1], math.degrees(math.atan2(v[1, 0], v[0, 0])), used

ratios, weak = [], []
for k in range(0, len(R), 5):                     # every 5th sweep (2 a second) is plenty
    out = weak_dir(R[k])
    if out:
        ratios.append(out[0]); weak.append(out[1])
ratios, weak = np.array(ratios), np.array(weak)
along = (np.abs(((weak + 90) % 180) - 90) < 25)     # weak direction within 25 deg of forward/back
print('\nWEAK DIRECTION (sweeps checked: %d)' % len(ratios))
print('  strength ratio weak/strong: median %.2f; below 0.15 (badly pinned) in %.0f%% of sweeps'
      % (np.median(ratios), 100 * (ratios < 0.15).mean()))
print('  weak direction within 25 deg of forward/back: %.0f%% of all sweeps, %.0f%% of badly pinned ones'
      % (100 * along.mean(), 100 * along[ratios < 0.15].mean() if (ratios < 0.15).any() else 0))

# ---- 3b. things that move with the device (the user, handle, pole) ------------------------
# A beam that stays close and at the same range for 5 s while the device is moving is looking at
# something carried along: to the matcher it is a wall that never moves, so it pulls the estimate
# toward "standing still". The rear filter only drops < 0.7 m within 60 deg of straight back.
CLOSE, HOLD, TOL = 1.2, 50, 0.03
mov = ~still
stuck = np.zeros(N)
count = 0
for k in range(0, len(R) - HOLD, 5):
    if not mov[k:k + HOLD].all():
        continue
    seg = R[k:k + HOLD]
    ok = np.isfinite(seg).all(axis=0) & (np.nanmax(seg, axis=0) < CLOSE)
    ok &= (np.nanmax(seg, axis=0) - np.nanmin(seg, axis=0)) < TOL
    stuck += ok; count += 1
stuck /= max(count, 1)
deg_b = np.degrees(ang_b)
print('\nCARRIED ALONG (closer than %.1f m, same range within %d cm for %d s while moving; %d windows)'
      % (CLOSE, TOL * 100, HOLD // 10, count))
hot = np.where(stuck > 0.3)[0]
if len(hot) == 0:
    print('  none: no beam sees something carried along in more than 30% of windows')
else:
    groups, cur = [], [hot[0]]
    for i in hot[1:]:
        (cur.append(i) if i == cur[-1] + 1 else (groups.append(cur), cur := [i]))
    groups.append(cur)
    for g in groups:
        a0, a1 = deg_b[g[0]], deg_b[g[-1]]
        rr = np.nanmedian(R[mov][:, g])
        filt = rear[g].mean() > 0.5 and rr < REAR_CUT
        print('  bearing %+5.0f to %+5.0f deg (0 = forward, 180 = back): %d beams, range ~%.2f m, '
              'in %.0f%% of windows; removed by the rear filter: %s'
              % (a0, a1, len(g), rr, 100 * stuck[g].mean(), 'yes' if filt else 'NO'))
    print('  share of an average moving sweep: %.1f%% of points' %
          (100 * stuck[hot].sum() / max(np.isfinite(R[mov]).sum(axis=1).mean(), 1)))

# ---- 4. over time (10 s blocks) -----------------------------------------------------------
print('\nOVER TIME (10 s blocks): flicker %% of points | weak/strong ratio')
for s0 in range(0, int(T[-1]), 10):
    m = (T >= s0) & (T < s0 + 10)
    rk = np.arange(0, len(R), 5)
    mk = (T[rk] >= s0) & (T[rk] < s0 + 10)
    rr = ratios[mk[:len(ratios)]] if len(ratios) == len(rk) else np.array([])
    print('  %3d-%3d s: %4.1f%% | %s%s' % (s0, s0 + 10, 100 * frac[m].mean(),
          ('%.2f' % np.median(rr)) if len(rr) else ' -  ', '  (still)' if still[m].mean() > 0.8 else ''))
