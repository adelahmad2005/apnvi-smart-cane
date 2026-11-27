"""
2D LiDAR movement tracking in plain numpy, for looking at recorded sweeps away from the Jetson.

Two uses:
  - mode 'rt': built like RTAB-Map's icp_odometry as the device runs it (frame to a small local
    map kept first-in-first-out, point to point, 15 rounds, 0.3 m match distance, 85 % trimmed,
    "keeps moving the same way" guess, reset when lost). Not the same code, the same recipe.
  - mode 'ref': a sturdier tracker used as the yardstick (whole-room map, point to line, sweep
    de-skewing).

A sweep is a dict: t (s, start of sweep), ranges (float array), angle_min, angle_inc, time_inc.
"""
import math
import numpy as np
from scipy.spatial import cKDTree

LX, LYAW, FLIP = -0.70, math.pi, -1.0       # LiDAR mount on the device (stage3.launch.py)


def se2(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def pose_of(T):
    return T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0])


def apply(T, P):
    return P @ T[:2, :2].T + T[:2, 2]


def sweep_points(sw, rmin=0.15, rmax=12.0, vel=None, time_dir=-1, lx=LX):
    """Sweep -> points in base_link. vel=(vx, vy, wz) of base_link in its own frame de-skews the
    sweep to its middle time. time_dir=-1: the last range in the list was measured first."""
    r = np.asarray(sw['ranges'], dtype=float)
    n = len(r)
    idx = np.arange(n)
    a = sw['angle_min'] + idx * sw['angle_inc']
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    ang = FLIP * a + LYAW
    r = np.where(ok, r, 0.0)
    P = np.stack([lx + r * np.cos(ang), r * np.sin(ang)], axis=1)
    if vel is not None:
        tinc = sw.get('time_inc', 0.0) or 0.1 / n
        dt = idx * tinc if time_dir > 0 else (n - 1 - idx) * tinc
        dt = dt - 0.5 * n * tinc             # to the MIDDLE of the sweep (to its start: the estimate swings)
        vx, vy, wz = vel
        th = wz * dt
        c, s = np.cos(th), np.sin(th)
        # pose of base at time dt, seen from base at time 0 (constant twist, small-step form)
        px, py = vx * dt, vy * dt
        P = np.stack([px + c * P[:, 0] - s * P[:, 1], py + s * P[:, 0] + c * P[:, 1]], axis=1)
    return P[ok], idx[ok]


def voxel(P, size):
    if size <= 0 or len(P) == 0:
        return P
    key = np.floor(P / size).astype(np.int64)
    _, inv = np.unique(key[:, 0] * 1000003 + key[:, 1], return_inverse=True)
    cnt = np.bincount(inv)
    out = np.stack([np.bincount(inv, P[:, 0]) / cnt, np.bincount(inv, P[:, 1]) / cnt], axis=1)
    return out


def normals2d(P, k=6):
    """Line direction around each point from its k neighbours -> unit normal and a 0..1 'line-ness'."""
    if len(P) < k + 1:
        return np.zeros_like(P), np.zeros(len(P))
    tree = cKDTree(P)
    _, nb = tree.query(P, k=k + 1)
    Q = P[nb] - P[nb].mean(axis=1, keepdims=True)
    cxx = (Q[:, :, 0] ** 2).mean(1); cyy = (Q[:, :, 1] ** 2).mean(1); cxy = (Q[:, :, 0] * Q[:, :, 1]).mean(1)
    th = 0.5 * np.arctan2(2 * cxy, cxx - cyy)           # main direction
    tr = cxx + cyy; d = np.sqrt(((cxx - cyy) / 2) ** 2 + cxy ** 2)
    l1, l2 = tr / 2 + d, tr / 2 - d
    N = np.stack([-np.sin(th), np.cos(th)], axis=1)
    return N, 1 - l2 / np.maximum(l1, 1e-12)


def solve_p2p(A, B):
    """Rigid T with T*A ~ B (least squares)."""
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    t = cb - R @ ca
    return se2(t[0], t[1], th)


def solve_p2l(A, B, N, w=None):
    """Small rigid step moving A towards the lines through B with normals N."""
    if w is None:
        w = np.ones(len(A))
    J = np.stack([N[:, 0], N[:, 1], -A[:, 1] * N[:, 0] + A[:, 0] * N[:, 1]], axis=1)
    r = ((B - A) * N).sum(1)
    Jw = J * w[:, None]
    H = Jw.T @ J + 1e-9 * np.eye(3)
    x = np.linalg.solve(H, Jw.T @ r)
    return se2(x[0], x[1], x[2]), H


class Tracker:
    def __init__(self, mode='rt', **kw):
        p = dict(voxel=0.05, max_dist=0.3, iters=15, trim=0.85, min_ratio=0.2, p2l=False,
                 map_max=2000, sub_radius=0.05, kf_thr=0.9, guess=True, reset=True,
                 deskew=False, time_dir=-1, rmin=0.15, rmax=12.0, max_trans=1.0, eps=1e-5,
                 fifo=True, lx=LX)
        if mode == 'ref':
            p.update(max_dist=0.25, iters=40, p2l=True, map_max=10 ** 9, fifo=False,
                     deskew=True, reset=False, min_ratio=0.05)
        p.update(kw)
        self.p = p
        self.T = np.eye(3)
        self.vel = None                 # last step as a 3x3 transform per second
        self.twist = (0.0, 0.0, 0.0)
        self.last_t = None
        self.chunks = []                # local map as a list of point arrays (oldest first)
        self.map = np.zeros((0, 2)); self.tree = None; self.mapN = None
        self.log = []                   # per sweep: t, x, y, th, ratio, lost, npts, new

    def _rebuild(self):
        self.map = np.concatenate(self.chunks) if self.chunks else np.zeros((0, 2))
        self.tree = cKDTree(self.map) if len(self.map) else None
        self.mapN = normals2d(self.map)[0] if (self.p['p2l'] and len(self.map) > 8) else None

    def _add(self, Pw):
        p = self.p
        if self.tree is not None and p['sub_radius'] > 0:
            d, _ = self.tree.query(Pw, k=1)
            Pw = Pw[d > p['sub_radius']]
        if len(Pw) == 0:
            return 0
        self.chunks.append(Pw)
        if p['fifo']:
            total = sum(len(c) for c in self.chunks)
            while len(self.chunks) > 1 and total > p['map_max']:
                total -= len(self.chunks[0]); self.chunks.pop(0)
        self._rebuild()
        return len(Pw)

    def step(self, sw):
        p = self.p
        t = sw['t']
        dt = (t - self.last_t) if self.last_t is not None else 0.1
        vel = self.twist if (p['deskew'] and self.vel is not None) else None
        P, _ = sweep_points(sw, p['rmin'], p['rmax'], vel, p['time_dir'], p['lx'])
        n_raw = len(sw['ranges'])
        P = voxel(P, p['voxel'])
        if self.tree is None:                          # first sweep (or just reset)
            new = self._add(apply(self.T, P))
            self.last_t = t
            self.log.append((t, *pose_of(self.T), 1.0, 0, len(P), new))
            return
        G = np.eye(3)
        if p['guess'] and self.vel is not None:
            x, y, th = self.twist
            G = se2(x * dt, y * dt, th * dt)
        T = self.T @ G
        ratio, lost = 0.0, False
        for it in range(p['iters']):
            Pw = apply(T, P)
            d, j = self.tree.query(Pw, k=1, distance_upper_bound=p['max_dist'])
            m = np.isfinite(d)
            if m.sum() < 10:
                lost = True; break
            keep = np.where(m)[0]
            if p['trim'] < 1.0:
                thr = np.quantile(d[keep], p['trim'])
                keep = keep[d[keep] <= thr]
            A, B = Pw[keep], self.map[j[keep]]
            if p['p2l'] and self.mapN is not None:
                D, _ = solve_p2l(A, B, self.mapN[j[keep]])
            else:
                D = solve_p2p(A, B)
            T = D @ T
            dx, dy, dth = pose_of(D)
            if abs(dx) + abs(dy) < p['eps'] and abs(dth) < p['eps']:
                break
        if not lost:
            Pw = apply(T, P)
            d, _ = self.tree.query(Pw, k=1, distance_upper_bound=p['max_dist'])
            ratio = float(np.isfinite(d).sum()) / max(len(P), 1)
            step = np.linalg.inv(self.T) @ T
            sx, sy, sth = pose_of(step)
            if ratio < p['min_ratio'] or math.hypot(sx, sy) > p['max_trans']:
                lost = True
        new = 0
        if lost:
            if p['reset']:                              # start again from the last good pose
                self.chunks, self.tree, self.map = [], None, np.zeros((0, 2))
                self.vel = None; self.twist = (0.0, 0.0, 0.0)
                new = self._add(apply(self.T, P))
        else:
            step = np.linalg.inv(self.T) @ T
            sx, sy, sth = pose_of(step)
            self.twist = (sx / dt, sy / dt, sth / dt); self.vel = True
            self.T = T
            if ratio < p['kf_thr'] or p['kf_thr'] <= 0:
                new = self._add(apply(T, P))
        self.last_t = t
        self.log.append((t, *pose_of(self.T), ratio, int(lost), len(P), new))

    def run(self, sweeps):
        for sw in sweeps:
            self.step(sw)
        return np.array(self.log)
