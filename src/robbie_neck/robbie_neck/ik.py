"""Inverse kinematics for the neck: head roll, pitch, yaw -> the three sled angles (README 6.0). numpy only.

The table is the sim's sweep of sled_1 and sled_2 on a grid with sled_3 = -(sled_1 + sled_2) (scripts/plots/sim.py), so
every row sits at zero common-mode. Roll and pitch come from inverting it: each grid cell is two triangles in (roll,
pitch), and the triangles a target falls in are its solutions, interpolated inside the triangle. Yaw is all sleds in
unison, a pure yaw of the platform: the common-mode added is the requested yaw minus the solution's residual yaw.

Roll is a band, not a point: the graceful allowance (README 5.x) lets the head roll within GRACEFUL_ROLL of what was
asked, so the solver takes anything in that band at the exact pitch and chooses by the policy below. A sheet is one
connected run of the table across the band (one branch of the inverse); several sheets means several distinct sled
triples give the pose. Off centre is a quality, not a target. A target whose band misses the table gets the nearest
point on the table's edge, with the per-axis error (roll error measured from the band's edge).

Frames: roll, pitch and yaw are the ZYX Euler angles of the head site in the world, as the sim's head pose sensor
reports them. Nose up is negative pitch. Yaw is continuous, not wrapped: unwrap it before calling. Sled angles are
offsets (deg) from sled.start_deg, CCW seen from above, as the sweep and Sim.go use them. One NeckIK per pole.

Which solution (README 6.2, 6.3):
  none   the one needing the least sled movement (the sum over the sleds, yaw included)
  force  the most central (least off centre)
  auto   the least movement, unless a more central one is available and gets through the gate. "Use the best" of what
         gets in: best="shortest" (least movement) or "central" (least off centre). corrected is set when that pick
         differs from the least-movement one: the caller resets its since clock (seconds since the last correction).

The gates (gate=), to tune and compare; cur is the head's off centre now, short the off centre where the least
movement goes, and off a candidate's:
  first_blush  off < kappa * cur, kappa = gain * min(since, gamma). Straight after a correction nothing gets in; the
               gate opens over gamma seconds to gain * gamma, a bit under 1. The README 6.3 gate.
  margin       after a dwell of gamma seconds, off < short - margin (mm): the candidate must beat where the head is
               going anyway by a real amount, not a ratio.
  deadband     margin, but only when short > GRACEFUL_OFF: while the least movement keeps the head graceful nothing is
               corrected at all.
  cost         the default. No clock: the candidate with the least move + lam * off + penalty (deg, unless it is the
               least-movement one). A switching penalty is hysteresis in cost space. lam 3, penalty 1 won just ik-bench:
               the event gates above can't hold a drift that the least movement resumes within a second.
"""
import math
from dataclasses import dataclass

import numpy as np

from .geometry import head_facing_deg

COLS = {k: i for i, k in enumerate(["jam_deg", "sled_1", "sled_2", "sled_3", "roll", "pitch", "yaw", "off_x", "off_y", "off"])}
GRACEFUL_ROLL, GRACEFUL_OFF = 1.0, 10.0   # deg, mm: as scripts/plots/common.py
MODES = ("auto", "none", "force")
BESTS = ("shortest", "central")
GATES = ("first_blush", "margin", "deadband", "cost")
EPS = 1e-9
R, P, Y, OFF, S1, S2 = range(6)   # vertex columns


def wrap(deg):
    return (deg + 180) % 360 - 180


def cross(a, b):
    """2-D cross product of [..., 2] arrays."""
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def to_head(rows, deg):
    """rows with roll, pitch, yaw (ZYX) re-expressed for a head turned deg (CCW from the top) on the platform. Yaw stays
    a residual: the head's world yaw is yaw + deg + common-mode. As scripts/plots/common.py to_frame."""
    r, p, y = (np.radians(rows[:, COLS[k]]) for k in ("roll", "pitch", "yaw"))
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    h = math.radians(deg); ch, sh = math.cos(h), math.sin(h)
    c0 = np.stack([cy * cp, sy * cp, -sp]); c1 = np.stack([cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr])
    n0, n1 = c0 * ch + c1 * sh, -c0 * sh + c1 * ch   # columns of R Rz(h)
    out = rows.copy()
    out[:, COLS["pitch"]] = np.degrees(-np.arcsin(np.clip(n0[2], -1, 1)))
    out[:, COLS["roll"]] = np.degrees(np.arctan2(n1[2], cp * cr))
    out[:, COLS["yaw"]] = wrap(np.degrees(np.arctan2(n0[1], n0[0])) - deg)
    return out


def _root(parent, k):
    while parent[k] != k:
        k = parent[k]
    return k


@dataclass
class Pose:
    roll: float
    pitch: float
    yaw: float       # world
    off: float       # mm
    inside: bool     # the sleds are inside the table; outside, the nearest table point is given


@dataclass
class Solution:
    sleds: np.ndarray   # the three sled angles, deg
    off: float          # head off centre there, mm
    err_roll: float     # reachable error, deg: how far outside the roll band; 0 when the band meets the table
    err_pitch: float
    graceful: bool      # |roll| <= 1 deg and off <= 10 mm
    corrected: bool     # auto took a path correction: reset the caller's since clock
    roll: float         # the pose the sleds give; yaw is always as asked
    pitch: float
    solutions: int      # sheets that offered the band; 0 means the nearest edge point was returned


class NeckIK:
    def __init__(self, rows, facing_deg, gate="cost", gain=0.45, gamma=2.0, margin=3.0, lam=3.0, penalty=1.0,
                 best="shortest", roll_tol=GRACEFUL_ROLL):
        """rows: a sweep (sim.HEADER columns), platform frame. facing_deg: the head's forward direction on the platform
        at rest, as a world angle (geometry.head_facing_deg). gate and its settings, best: the module docstring.
        roll_tol: the roll band, |roll - requested| <= roll_tol; the graceful limit unless told otherwise."""
        if best not in BESTS:
            raise ValueError(f"best must be one of {BESTS}, not {best!r}")
        if gate not in GATES:
            raise ValueError(f"gate must be one of {GATES}, not {gate!r}")
        self.facing, self.best, self.roll_tol = float(facing_deg), best, float(roll_tol)
        self.gate, self.gain, self.gamma, self.margin, self.lam, self.penalty = gate, gain, gamma, margin, lam, penalty
        rows = to_head(np.asarray(rows, float), facing_deg)
        s1, s2 = rows[:, COLS["sled_1"]], rows[:, COLS["sled_2"]]
        self.axis = np.unique(np.r_[s1, s2]); n = self.n = len(self.axis)
        self.step = float(self.axis[1] - self.axis[0])
        i = np.rint((s2 - self.axis[0]) / self.step).astype(int); j = np.rint((s1 - self.axis[0]) / self.step).astype(int)
        # vertices: every grid point, pose columns NaN where the sweep has no row
        v = np.full((n * n, 6), np.nan)
        jj, ii = np.meshgrid(self.axis, self.axis)
        v[:, S1], v[:, S2] = jj.ravel(), ii.ravel()
        v[i * n + j, :4] = rows[:, [COLS["roll"], COLS["pitch"], COLS["yaw"], COLS["off"]]]
        self.v = v
        self.tri = self._triangles(~np.isnan(v[:, R]).reshape(n, n))
        pts = v[self.tri][:, :, :2]                                   # [T, 3, 2] roll, pitch
        det = cross(pts[:, 1] - pts[:, 0], pts[:, 2] - pts[:, 0])
        keep = np.abs(det) > EPS
        self.tri, self.pts, self.det = self.tri[keep], pts[keep], det[keep]
        self.lo, self.hi = self.pts.min(1), self.pts.max(1)
        e = np.sort(np.concatenate([self.tri[:, [0, 1]], self.tri[:, [1, 2]], self.tri[:, [2, 0]]]), 1)
        self.edge = np.unique(e, axis=0)
        self.ea = v[self.edge[:, 0], :2]; self.eab = v[self.edge[:, 1], :2] - self.ea
        self.el2 = np.maximum((self.eab ** 2).sum(1), 1e-12)
        self.feasible = np.flatnonzero(~np.isnan(v[:, R]))
        self.rest_yaw = self.forward([0, 0, 0]).yaw

    @classmethod
    def from_sweep(cls, rows, cfg, pole, **kw):
        """From a sweep of neck.yaml (cfg) twisted to pole; the head's facing comes from cfg's head.rotation_deg."""
        return cls(rows, head_facing_deg(cfg, pole), **kw)

    @staticmethod
    def _triangles(ok):
        """Two triangles per full grid cell, one where a cell has three corners, as flat vertex indices [T, 3]."""
        n = ok.shape[0]
        ii, jj = np.meshgrid(np.arange(n - 1), np.arange(n - 1), indexing="ij")
        corner = [ii * n + jj, ii * n + jj + 1, (ii + 1) * n + jj + 1, (ii + 1) * n + jj]   # round the cell
        flag = [ok[:-1, :-1], ok[:-1, 1:], ok[1:, 1:], ok[1:, :-1]]
        count = sum(f.astype(int) for f in flag)
        full = count == 4
        tris = [np.stack([corner[0][full], corner[1][full], corner[2][full]], 1),
                np.stack([corner[0][full], corner[2][full], corner[3][full]], 1)]
        for k in range(4):
            m = (count == 3) & ~flag[k]
            tris.append(np.stack([corner[(k + 1) % 4][m], corner[(k + 2) % 4][m], corner[(k + 3) % 4][m]], 1))
        return np.concatenate(tris)

    # ------------------------------------------------------------------------------------------- lookups
    def forward(self, sleds):
        """The pose the table gives for three sled angles (bilinear in the zero common-mode sleds)."""
        sleds = np.asarray(sleds, float); cm = sleds.mean()
        fj, fi = (sleds[0] - cm - self.axis[0]) / self.step, (sleds[1] - cm - self.axis[0]) / self.step
        i0, j0 = int(np.clip(np.floor(fi), 0, self.n - 2)), int(np.clip(np.floor(fj), 0, self.n - 2))
        u, w = fi - i0, fj - j0
        c = self.v[[i0 * self.n + j0, i0 * self.n + j0 + 1, (i0 + 1) * self.n + j0, (i0 + 1) * self.n + j0 + 1], :4]
        inside = 0 <= u <= 1 and 0 <= w <= 1 and not np.isnan(c).any()
        if inside:
            r, p, y, off = (1 - u) * ((1 - w) * c[0] + w * c[1]) + u * ((1 - w) * c[2] + w * c[3])
        else:
            f = self.v[self.feasible]
            r, p, y, off = f[np.argmin(np.hypot(f[:, S1] - (sleds[0] - cm), f[:, S2] - (sleds[1] - cm))), :4]
        return Pose(float(r), float(p), float(y + self.facing + cm), float(off), inside)

    def _band(self, roll, pitch, tol):
        """The table across the roll band |roll' - roll| <= tol at this pitch, per triangle the band crosses: the
        vertex-column values at the band's centre (va) and per degree of roll (gv), so values at roll + d are
        va + gv d; the d range the triangle covers [dlo, dhi]; and the sheet each triangle belongs to (triangles meeting
        at a crossing of their shared edge are one sheet). None when the band misses the table."""
        q = np.array([roll, pitch], float)
        t = np.flatnonzero((self.lo[:, 0] <= roll + tol + EPS) & (self.hi[:, 0] >= roll - tol - EPS) &
                           (self.lo[:, 1] <= pitch + EPS) & (self.hi[:, 1] >= pitch - EPS))
        if not len(t):
            return None
        p = self.pts[t]; v0, v1 = p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]

        def bary(x):
            v2 = x - p[:, 0]
            w1 = cross(v2, v1) / self.det[t]; w2 = cross(v0, v2) / self.det[t]
            return np.stack([1 - w1 - w2, w1, w2], 1)

        wa = bary(q); g = bary(q + [1, 0]) - wa          # weights at roll + d: wa + g d, affine along the band
        dlo, dhi = np.full(len(t), -tol), np.full(len(t), tol)
        for i in range(3):                                # inside the triangle while every weight >= -EPS
            gi, wi = g[:, i], wa[:, i] + EPS
            up, down = gi > EPS, gi < -EPS
            dlo[up] = np.maximum(dlo[up], -wi[up] / gi[up])
            dhi[down] = np.minimum(dhi[down], -wi[down] / gi[down])
            dhi[~up & ~down & (wi < 0)] = -np.inf
        ok = dlo <= dhi + 1e-12
        if not ok.any():
            return None
        t, wa, g, dlo, dhi = t[ok], wa[ok], g[ok], dlo[ok], np.maximum(dhi[ok], dlo[ok])
        V = self.v[self.tri[t]]
        va, gv = np.einsum("tk,tkc->tc", wa, V), np.einsum("tk,tkc->tc", g, V)
        va[:, R], gv[:, R], va[:, P], gv[:, P] = roll, 1, pitch, 0   # exact: the weights reproduce the point
        sheet, ends = np.arange(len(t)), {}
        for k in range(len(t)):
            for d in (dlo[k], dhi[k]):
                key = (round(float(d), 6), round(float(va[k, S1] + gv[k, S1] * d), 4), round(float(va[k, S2] + gv[k, S2] * d), 4))
                a, b = _root(sheet, ends.setdefault(key, k)), _root(sheet, k)
                sheet[max(a, b)] = min(a, b)
        return va, gv, dlo, dhi, np.array([_root(sheet, k) for k in range(len(t))])

    def _points(self, roll, pitch, yaw, sleds, tol):
        """The candidate solutions in the band: vertex-column values [M, 6], sled angles [M, 3], the sheet of each, and
        how many sheets. Per triangle crossed: both ends of its run and, inside it, wherever one sled's movement from
        `sleds` changes sign; the least total movement over the run is at one of those, and so is the least off."""
        b = self._band(roll, pitch, tol)
        if b is None:
            return None
        va, gv, dlo, dhi, sheet = b
        cma, gcm = yaw - self.facing - va[:, Y], -gv[:, Y]   # common-mode for the requested yaw, and per degree of roll
        sa = np.stack([va[:, S1] + cma, va[:, S2] + cma, -va[:, S1] - va[:, S2] + cma], 1)
        gs = np.stack([gv[:, S1] + gcm, gv[:, S2] + gcm, -gv[:, S1] - gv[:, S2] + gcm], 1)
        with np.errstate(divide="ignore", invalid="ignore"):
            roots = (sleds - sa) / gs
        roots = np.where((roots > dlo[:, None]) & (roots < dhi[:, None]), roots, np.nan)
        vals, s, sh, seen = [], [], [], set()
        for k, ds in enumerate(np.column_stack([dlo, dhi, roots])):
            for d in ds[~np.isnan(ds)]:
                sk = sa[k] + gs[k] * d
                key = (round(float(d), 6), round(float(sk[0]), 4), round(float(sk[1]), 4))
                if key not in seen:   # neighbouring triangles both offer the crossing of their shared edge
                    seen.add(key); vals.append(va[k] + gv[k] * d); s.append(sk); sh.append(sheet[k])
        return np.array(vals), np.array(s), np.array(sh), len(set(sheet))

    def _in_table(self, q):
        """Every table solution for the roll/pitch q exactly: [M, 6] vertex-column values, one row per sheet."""
        pts = self._points(float(q[0]), float(q[1]), self.facing, np.zeros(3), 0.0)
        return np.empty((0, 6)) if pts is None else pts[0]

    def _nearest(self, q):
        """The table point nearest to the roll/pitch q: on some triangle's edge, since q is in no triangle."""
        t = np.clip(((q - self.ea) * self.eab).sum(1) / self.el2, 0, 1)
        k = int(np.argmin(((q - self.ea - t[:, None] * self.eab) ** 2).sum(1)))
        return (1 - t[k]) * self.v[self.edge[k, 0]] + t[k] * self.v[self.edge[k, 1]]

    def _nearest_band(self, roll, pitch, tol):
        """The table point nearest to a band that crosses no triangle: nearest to either end of the band on an edge, or
        a vertex whose roll is within the band, whichever is closest to the band."""
        cands = [self._nearest(np.array([roll + e, pitch])) for e in (-tol, tol)]
        f = self.v[self.feasible]; m = np.abs(f[:, R] - roll) <= tol
        if m.any():
            cands.append(f[m][np.argmin(np.abs(f[m, P] - pitch))])
        return min(cands, key=lambda c: math.hypot(max(0.0, abs(c[R] - roll) - tol), c[P] - pitch))

    # -------------------------------------------------------------------------------------------- solver
    def solve(self, roll, pitch, yaw, sleds, mode="auto", since=math.inf):
        """Sled angles for the head pose roll (within the band), pitch, yaw (deg, world), from the current sled angles.
        mode: auto, none or force (module docstring). since: seconds since the last correction, auto only."""
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, not {mode!r}")
        sleds = np.asarray(sleds, float); tol = self.roll_tol
        pts = self._points(roll, pitch, yaw, sleds, tol)
        if pts is None:
            c = self._nearest_band(roll, pitch, tol)[None]; n = 0
            cm = yaw - self.facing - c[:, Y]
            s = np.stack([c[:, S1] + cm, c[:, S2] + cm, -c[:, S1] - c[:, S2] + cm], 1)
        else:
            c, s, _, n = pts
        move, off = np.abs(s - sleds).sum(1), c[:, OFF]
        shortest = pick = int(np.argmin(move))
        if len(c) > 1:
            if mode == "force":
                pick = int(np.lexsort((move, off))[0])
            elif mode == "auto":
                pick = self._auto(move, off, shortest, sleds, since)
        r, p, o = float(c[pick, R]), float(c[pick, P]), float(off[pick])
        err_roll = abs(roll - r) - tol
        return Solution(s[pick], o, err_roll if err_roll > 1e-9 else 0.0, abs(pitch - p),
                        abs(r) <= GRACEFUL_ROLL and o <= GRACEFUL_OFF, mode == "auto" and pick != shortest, r, p, n)

    def _auto(self, move, off, shortest, sleds, since):
        """The auto policy's pick among the candidates: the gated ones' best, else the least movement (module docstring)."""
        if self.gate == "cost":
            return int(np.argmin(move + self.lam * off + self.penalty * (np.arange(len(move)) != shortest)))
        if self.gate == "first_blush":
            cur = self.forward(sleds).off
            admit = (off < cur) & (off < self.gain * min(since, self.gamma) * cur)
        else:
            admit = (off < off[shortest] - self.margin) if since >= self.gamma else np.zeros(len(off), bool)
            if self.gate == "deadband" and off[shortest] <= GRACEFUL_OFF:
                admit[:] = False
        if not admit.any():
            return shortest
        k = np.flatnonzero(admit)
        return int(k[np.argmin(move[k] if self.best == "shortest" else off[k])])
