"""
A made-up lab to try the trackers on: walls, rows of desk-top clutter, a pushed lap with pivot
turns, the LiDAR 0.7 m behind the turning point, each sweep taking 0.1 s (so it bends while the
device moves). Output: sweeps like the real ones, and the true pose at every sweep start.
"""
import math
import numpy as np

LX = -0.70


def make_room(seed=1, w=13.0, h=11.0, clutter=1.0):
    rng = np.random.default_rng(seed)
    segs = [((0, 0), (w, 0)), ((w, 0), (w, h)), ((w, h), (0, h)), ((0, h), (0, 0))]
    # a pillar and a cupboard on the walls so the walls are not featureless
    for (x, y, a, b) in [(4, 0, 0.6, 0.4), (9, h - 0.5, 1.2, 0.5), (0, 5, 0.4, 1.0), (w - 0.5, 3, 0.5, 1.5)]:
        segs += [((x, y), (x + a, y)), ((x + a, y), (x + a, y + b)), ((x + a, y + b), (x, y + b)), ((x, y + b), (x, y))]
    # desk rows: islands in the middle, lanes around and between them
    islands = [(2.5, 2.5, 4.5, 3.9), (2.5, 5.2, 4.5, 6.6), (2.5, 7.9, 4.5, 8.8),
               (6.5, 2.5, 10.5, 3.9), (6.5, 5.2, 10.5, 6.6), (6.5, 7.9, 10.5, 8.8)]
    for (x0, y0, x1, y1) in islands:
        n = int(clutter * (x1 - x0) * (y1 - y0) * 2.2)
        for _ in range(n):                              # monitors, chair backs, boxes on the desks
            cx, cy = rng.uniform(x0, x1), rng.uniform(y0, y1)
            L = rng.uniform(0.2, 0.6); a = rng.choice([0, math.pi / 2]) + rng.normal(0, 0.25)
            dx, dy = 0.5 * L * math.cos(a), 0.5 * L * math.sin(a)
            segs.append(((cx - dx, cy - dy), (cx + dx, cy + dy)))
    return np.array(segs, dtype=float), islands


def lap_path(speed=0.3, turn_rate=math.radians(35), dt=0.002, pause=1.0):
    """True base_link pose over time: drive lanes, pivot at the corners, one dead end with a pull
    back. Start and end at the same pose. Returns arrays t, x, y, th."""
    # way-points (lane centres) for one left-hand lap with a detour between the islands
    wps = [(1.5, 1.4), (11.6, 1.4), (11.6, 4.55), (5.5, 4.55), (5.5, 7.25), (11.6, 7.25),
           (11.6, 9.8), (1.5, 9.8), (1.5, 1.4)]
    x, y, th = wps[0][0], wps[0][1], 0.0
    T, X, Y, TH = [0.0], [x], [y], [th]

    def hold(sec):
        for _ in range(int(sec / dt)):
            T.append(T[-1] + dt); X.append(X[-1]); Y.append(Y[-1]); TH.append(TH[-1])

    def pivot(target):
        d = target - TH[-1]
        n = max(1, int(abs(d) / turn_rate / dt))
        for i in range(n):                              # smooth start and stop
            s = (1 - math.cos(math.pi * (i + 1) / n)) / 2
            T.append(T[-1] + dt); X.append(X[-1]); Y.append(Y[-1]); TH.append(th0 + d * s)

    def drive(tx, ty, v):
        x0, y0 = X[-1], Y[-1]
        L = math.hypot(tx - x0, ty - y0)
        n = max(1, int(L / abs(v) / dt))
        for i in range(n):
            s = (1 - math.cos(math.pi * (i + 1) / n)) / 2
            wob = 0.02 * math.sin(2 * math.pi * 0.9 * T[-1])        # the pusher's small weave
            T.append(T[-1] + dt); X.append(x0 + (tx - x0) * s); Y.append(y0 + (ty - y0) * s)
            TH.append(th_d + wob)

    hold(pause)
    for (tx, ty) in wps[1:]:
        th_d = math.atan2(ty - Y[-1], tx - X[-1])
        th0 = TH[-1]
        th_d = th0 + (th_d - th0 + math.pi) % (2 * math.pi) - math.pi     # keep the angle unwrapped
        pivot(th_d)
        drive(tx, ty, speed)
    th0 = TH[-1]; pivot(th0 + (0 - th0 + math.pi) % (2 * math.pi) - math.pi)                    # face the start way again (one full turn)
    hold(pause)
    return np.array(T), np.array(X), np.array(Y), np.array(TH)


def ray_cast(ox, oy, ang, segs):
    """Distance along each ray (origin ox,oy per ray; world angle per ray) to the nearest segment."""
    dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
    ax, ay = segs[:, 0, 0][None, :], segs[:, 0, 1][None, :]
    ex, ey = (segs[:, 1, 0] - segs[:, 0, 0])[None, :], (segs[:, 1, 1] - segs[:, 0, 1])[None, :]
    den = dx * ey - dy * ex
    den = np.where(np.abs(den) < 1e-12, 1e-12, den)
    wx, wy = ax - ox[:, None], ay - oy[:, None]
    t = (wx * ey - wy * ex) / den
    u = (wx * dy - wy * dx) / den
    t = np.where((t > 0) & (u >= 0) & (u <= 1), t, np.inf)
    return t.min(axis=1)


def make_sweeps(path, segs, n_beams=720, sweep_s=0.1, noise=0.012, seed=2, skew=True,
                body=None, max_range=12.0, lx=LX, time_dir=-1, drop=0.0):
    """body=(dist, half_width): the pusher as a short wall behind the LiDAR that moves with it."""
    rng = np.random.default_rng(seed)
    T, X, Y, TH = path
    sweeps, truth = [], []
    a_min, a_inc = -math.pi, 2 * math.pi / n_beams
    t = 0.0
    while t + sweep_s < T[-1]:
        idx = np.arange(n_beams)
        a = a_min + idx * a_inc                          # the message's angle
        tb = (idx if time_dir > 0 else (n_beams - 1 - idx)) * (sweep_s / n_beams)
        tt = t + (tb if skew else 0 * tb)
        bx, by, bth = np.interp(tt, T, X), np.interp(tt, T, Y), np.interp(tt, T, TH)
        ox, oy = bx + lx * np.cos(bth), by + lx * np.sin(bth)
        world_ang = bth + math.pi - a                    # mount: yaw pi, upside down
        r = ray_cast(ox, oy, world_ang, segs)
        if body is not None:
            dist, half = body
            # a wall across the back of the LiDAR, in the LiDAR's own frame around angle 0
            ca = np.cos(a)
            rb = np.where((ca > 0.05) & (np.abs(dist * np.tan(a)) < half), dist / np.maximum(ca, 1e-6), np.inf)
            r = np.minimum(r, rb)
        r = r + rng.normal(0, noise, n_beams)
        r[(r > max_range) | (rng.random(n_beams) < drop)] = np.inf
        sweeps.append(dict(t=t, ranges=r.astype(np.float32), angle_min=a_min, angle_inc=a_inc,
                           time_inc=sweep_s / n_beams))
        truth.append((t, np.interp(t, T, X), np.interp(t, T, Y), np.interp(t, T, TH)))
        t += sweep_s
    return sweeps, np.array(truth)
