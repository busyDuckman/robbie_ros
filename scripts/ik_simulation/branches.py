# /// script
# requires-python = ">=3.12"
# dependencies = ["matplotlib", "numpy", "pyyaml"]
# ///
"""Solution branches at one jam angle, from sweep.py's pass A. Data only, no solver.

A branch is a connected region of sled space where d(roll, pitch)/d(sled_1, sled_2) keeps one sign, so
the map to roll/pitch is one-to-one inside it. Branches meet along the det = 0 folds. Each 1 deg grid
cell is split into two triangles (as in jam_charts.py); a triangle's sign is the sign of its area in
roll/pitch, and triangles of one sign sharing an edge are one branch.

Usage (Linux or WSL, from the repo root):
    uv run scripts/ik_simulation/branches.py .output/ik_sweep 75
"""
import csv, os, sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

src, jam = sys.argv[1], float(sys.argv[2])
out = os.path.join(src, f"{jam:g}_jam", "branches")
os.makedirs(out, exist_ok=True)
INK, MUTED = "#222222", "#666666"
CAT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.edgecolor": MUTED, "figure.dpi": 130})
MIN_GAP = 55.8          # deg, as sweep.py
ROLL_TOL, OFF_TOL = 1.0, 10.0   # deg, mm: the 14_v2 middle limits
MIN_BRANCH = 100        # triangles; smaller same-sign pieces are the thin strips along folds

with open(os.path.join(src, "sweep_pass_a.csv")) as f:
    r = csv.reader(f); head = next(r)
    A = np.array([[float(x) for x in row] for row in r])
col = {h: i for i, h in enumerate(head)}
A = A[A[:, 0] == jam]
LIM = int(np.max(np.abs(A[:, 1]))); N = 2 * LIM + 1
axis = np.arange(-LIM, LIM + 1)


def grid(key):
    g = np.full((N, N), np.nan)   # [sled_2, sled_1]
    g[(A[:, 2] + LIM).astype(int), (A[:, 1] + LIM).astype(int)] = A[:, col[key]]
    return g


R, P, Y, O = (grid(k) for k in ("roll", "pitch", "yaw", "off"))
S1, S2 = np.meshgrid(axis, axis)
S3 = -S1 - S2
GAP = np.minimum.reduce([120 + S2 - S1, 120 + S3 - S2, 120 + S1 - S3]) - MIN_GAP   # deg margin to the gap limit

# --- Jacobian at grid points (central differences, as jam_charts.py) -------------------------------
def d(g):
    d1 = np.full_like(g, np.nan); d2 = np.full_like(g, np.nan)
    d1[:, 1:-1] = (g[:, 2:] - g[:, :-2]) / 2; d2[1:-1, :] = (g[2:, :] - g[:-2, :]) / 2
    return d1, d2


(r1, r2), (p1, p2) = d(R), d(P)
J = np.stack([np.stack([r1, r2], -1), np.stack([p1, p2], -1)], -2)
okJ = ~np.isnan(J).any(axis=(-1, -2))
SV = np.full((N, N, 2), np.nan); SV[okJ] = np.linalg.svd(J[okJ], compute_uv=False)

# --- triangles and branches ----------------------------------------------------------------------
tris, tri_id = [], {}
for i in range(N - 1):
    for j in range(N - 1):
        for t, tri in enumerate((((i, j), (i, j + 1), (i + 1, j)), ((i + 1, j + 1), (i + 1, j), (i, j + 1)))):
            xy = np.array([[R[a], P[a]] for a in tri])
            if np.isnan(xy).any():
                continue
            (x0, y0), (x1, y1), (x2, y2) = xy
            den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
            if abs(den) < 1e-12:
                continue
            tri_id[i, j, t] = len(tris); tris.append((tri, xy, np.sign(den), den))
# Both triangles are listed counter-clockwise in sled space, so the sign of den is the sign of det.
parent = list(range(len(tris)))


def find(a):
    while parent[a] != a:
        parent[a] = parent[parent[a]]; a = parent[a]
    return a


for (i, j, t), k in tri_id.items():
    if t == 0:
        nbrs = [(i, j, 1), (i - 1, j, 1), (i, j - 1, 1)]
    else:
        nbrs = []
    for nb in nbrs:
        if nb in tri_id and tris[tri_id[nb]][2] == tris[k][2]:
            parent[find(k)] = find(tri_id[nb])
roots = np.array([find(k) for k in range(len(tris))])
uniq, inv, size = np.unique(roots, return_inverse=True, return_counts=True)
# Name the big branches: sign then angle of the centroid in sled space, starting at sled_1 axis.
big = [u for u, s in zip(range(len(uniq)), size) if s >= MIN_BRANCH]
cent = {}
for u in big:
    pts = np.array([[axis[a[1]], axis[a[0]]] for k in np.nonzero(inv == u)[0] for a in tris[k][0]])
    cent[u] = pts.mean(0)
sign_of = {u: tris[np.nonzero(inv == u)[0][0]][2] for u in big}
order = sorted(big, key=lambda u: (-sign_of[u], np.degrees(np.arctan2(cent[u][1], cent[u][0])) % 360))
BR = {u: n for n, u in enumerate(order)}            # branch index 0..; -1 = strip
tri_br = np.array([BR.get(u, -1) for u in inv])
NB = len(order)
names = []
for n, u in enumerate(order):
    names.append(f"{'+' if sign_of[u] > 0 else '-'}{n % 3 + 1}")
print("triangles", len(tris), "branches", NB, "sizes", [int(size[u]) for u in order],
      "strip triangles", int((tri_br < 0).sum()), "in", len(uniq) - NB, "pieces", flush=True)
for n, u in enumerate(order):
    print(f"  {names[n]}: det {'+' if sign_of[u] > 0 else '-'}, centroid sled_1 {cent[u][0]:.1f}, sled_2 {cent[u][1]:.1f}, "
          f"{size[u]} triangles", flush=True)

# point -> branches it touches (fold points touch two)
pt_br = defaultdict(set)
for k, (tri, *_ ) in enumerate(tris):
    for a in tri:
        pt_br[a].add(tri_br[k])

# --- solutions per roll/pitch target -------------------------------------------------------------
STEP = 0.25
rs = np.arange(np.floor(np.nanmin(R)), np.ceil(np.nanmax(R)) + STEP, STEP)
ps = np.arange(np.floor(np.nanmin(P)), np.ceil(np.nanmax(P)) + STEP, STEP)
F = {"s1": S1.astype(float), "s2": S2.astype(float), "off": O, "yaw": Y, "gap": GAP.astype(float),
     "smax": SV[..., 0], "smin": SV[..., 1]}
sols = defaultdict(list)   # (pi, ci) -> list of (branch, dict)
for k, (tri, xy, sg, den) in enumerate(tris):
    (x0, y0), (x1, y1), (x2, y2) = xy
    ci = np.arange(np.searchsorted(rs, xy[:, 0].min()), np.searchsorted(rs, xy[:, 0].max(), "right"))
    pi = np.arange(np.searchsorted(ps, xy[:, 1].min()), np.searchsorted(ps, xy[:, 1].max(), "right"))
    if not len(ci) or not len(pi):
        continue
    X, Yy = np.meshgrid(rs[ci], ps[pi])
    l0 = ((y1 - y2) * (X - x2) + (x2 - x1) * (Yy - y2)) / den
    l1 = ((y2 - y0) * (X - x2) + (x0 - x2) * (Yy - y2)) / den
    l2 = 1 - l0 - l1
    inside = (l0 >= 0) & (l1 >= 0) & (l2 > 0)
    for a, b in zip(*np.nonzero(inside)):
        w = (l0[a, b], l1[a, b], l2[a, b])
        v = {key: sum(wi * F[key][p] for wi, p in zip(w, tri)) for key in F}
        sols[pi[a], ci[b]].append((tri_br[k], v))

rows = []
nsol_tri = np.zeros((len(ps), len(rs)), int); nsol_br = np.zeros((len(ps), len(rs)), int)
dup = 0
for (a, b), lst in sols.items():
    nsol_tri[a, b] = len(lst)
    brs = [x[0] for x in lst if x[0] >= 0]
    nsol_br[a, b] = len(set(brs))
    dup += len(brs) != len(set(brs))
    for br, v in lst:
        rows.append([round(rs[b], 2), round(ps[a], 2), len(lst), br, names[br] if br >= 0 else "strip",
                     *(round(v[k], 3) for k in ("s1", "s2", "off", "yaw", "gap", "smax", "smin"))])
print("targets", len(sols), "targets where one big branch appears twice", dup, flush=True)
reach = nsol_br > 0
for n in range(1, NB + 1):
    print(f"  targets with {n} big-branch solutions: {(nsol_br == n).sum()} ({(nsol_br == n).sum() / reach.sum() * 100:.1f}%)",
          flush=True)
with open(os.path.join(out, "solutions.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["roll", "pitch", "n_solutions", "branch_idx", "branch", "sled_1", "sled_2", "off", "yaw",
                "gap_margin", "sigma_max", "sigma_min"])
    w.writerows(rows)

# --- which branches share targets, and how they differ there --------------------------------------
combo = defaultdict(int)
for (a, b), lst in sols.items():
    brs = tuple(sorted(set(x[0] for x in lst if x[0] >= 0)))
    combo[brs] += 1
print("branch sets per target:", flush=True)
for k, v in sorted(combo.items(), key=lambda kv: -kv[1])[:16]:
    print("  ", [names[i] for i in k], v, flush=True)
# per branch image in roll/pitch and stats over the whole branch
for n in range(NB):
    pts = [a for a, s in pt_br.items() if n in s]
    rr = np.array([R[a] for a in pts]); pp = np.array([P[a] for a in pts]); oo = np.array([O[a] for a in pts])
    yy = np.array([Y[a] for a in pts]); sm = np.array([SV[a][1] for a in pts]); sx = np.array([SV[a][0] for a in pts])
    tilt = np.hypot(rr, pp)
    print(f"{names[n]}: pts {len(pts)} roll {rr.min():.1f}..{rr.max():.1f} pitch {pp.min():.1f}..{pp.max():.1f} "
          f"min tilt {tilt.min():.2f} max tilt {tilt.max():.1f} off {oo.min():.1f}..{oo.max():.1f} "
          f"yaw {yy.min():.1f}..{yy.max():.1f} smax med {np.nanmedian(sx):.3f} smin med {np.nanmedian(sm):.3f}", flush=True)

# per combo: median of each attribute per branch
by_combo = defaultdict(lambda: defaultdict(list))
for (a, b), lst in sols.items():
    brs = tuple(sorted(set(x[0] for x in lst if x[0] >= 0)))
    if len(brs) in (2, 3) and combo[brs] > 100:
        for br, v in lst:
            if br >= 0:
                by_combo[brs][br].append(v)
for k, d_ in by_combo.items():
    print("combo", [names[i] for i in k], flush=True)
    for br, vs in d_.items():
        m = {key: np.nanmedian([v[key] for v in vs]) for key in ("off", "yaw", "gap", "smax", "smin")}
        print(f"   {names[br]}: off {m['off']:.1f} yaw {m['yaw']:.1f} gap {m['gap']:.1f} smax {m['smax']:.3f} smin {m['smin']:.3f}",
              flush=True)

# --- pitch up/down by head heading, per branch ----------------------------------------------------
# Same definitions as 14_v2 in jam_charts.py. Per branch and heading: the sweep points of that branch with
# |roll| <= ROLL_TOL and off <= OFF_TOL, joined along the branch's triangle edges; the start is the most
# level such point (smallest |pitch|), and up/down are the largest pitch up and down in its connected set.
rr_, pp_ = np.radians(R), np.radians(P)
r20, r21 = -np.sin(pp_), np.cos(pp_) * np.sin(rr_)
heads = np.arange(0, 360, 1)
edges = [set() for _ in range(NB)]
for k, (tri, *_ ) in enumerate(tris):
    if tri_br[k] >= 0:
        for a, b in ((0, 1), (1, 2), (2, 0)):
            edges[tri_br[k]].add(tuple(sorted((tri[a], tri[b]))))
br_pts = [sorted({p for e in edges[n] for p in e}) for n in range(NB)]
all_pts = sorted(pt_br)
all_edges = set().union(*edges)
for k, (tri, *_ ) in enumerate(tris):      # strips too, for the all-branches context line
    for a, b in ((0, 1), (1, 2), (2, 0)):
        all_edges.add(tuple(sorted((tri[a], tri[b]))))


def reach(pts, edg, hp, hr):
    """(up, down, start pitch, n reachable, both-up-and-down-from-start) for one point set and heading."""
    ok = {p for p in pts if abs(hr[p]) <= ROLL_TOL and O[p] <= OFF_TOL}
    if not ok:
        return np.nan, np.nan, np.nan, 0
    lvl = min(abs(hp[p]) for p in ok)
    adj = defaultdict(list)
    for a, b in edg:
        if a in ok and b in ok:
            adj[a].append(b); adj[b].append(a)
    # Start: the most level point, but among points within 0.5 deg of the most level take the one in the
    # largest connected set, so a lone grid point doesn't stand in for the branch.
    best, done = set(), set()
    for s0 in sorted((p for p in ok if abs(hp[p]) <= lvl + 0.5), key=lambda p: abs(hp[p])):
        if s0 in done:
            continue
        comp, stack = {s0}, [s0]
        while stack:
            for q in adj[stack.pop()]:
                if q not in comp:
                    comp.add(q); stack.append(q)
        done |= comp
        if len(comp) > len(best):
            best, start = comp, s0
    seen = best
    v = np.array([hp[p] for p in seen])
    return max(v.max(), 0), max(-v.min(), 0), hp[start], len(seen)


LIMITS = [(ROLL_TOL, OFF_TOL), (ROLL_TOL, 20.0)]    # the second is a looser off-centre check
RES = {}
pbh = []
for rt, ot in LIMITS:
    ROLL_TOL, OFF_TOL = rt, ot
    res = np.full((NB + 1, len(heads), 4), np.nan)    # last row: all of sled space, folds crossed
    for hi, h in enumerate(heads):
        c, s = np.cos(np.radians(h)), np.sin(np.radians(h))
        hp = np.degrees(np.arcsin(np.clip(r20 * c + r21 * s, -1, 1)))
        hr = np.degrees(np.arcsin(np.clip(-r20 * s + r21 * c, -1, 1)))
        for n in range(NB):
            res[n, hi] = reach(br_pts[n], edges[n], hp, hr)
        res[NB, hi] = reach(all_pts, all_edges, hp, hr)
        if hi % 90 == 0:
            print(f"[{rt:g} deg, {ot:g} mm] heading {h}: " + ", ".join(
                f"{names[n]} up {res[n, hi, 0]:.1f} down {res[n, hi, 1]:.1f}" for n in range(NB))
                + f", all up {res[NB, hi, 0]:.1f} down {res[NB, hi, 1]:.1f}", flush=True)
    RES[rt, ot] = res
    for hi, h in enumerate(heads):
        for n in range(NB + 1):
            pbh.append([rt, ot, h, names[n] if n < NB else "all", *(round(x, 3) for x in res[n, hi, :3]), int(res[n, hi, 3])])
    up, dn = res[:NB, :, 0], res[:NB, :, 1]
    single = np.nanmax(np.minimum(up, dn), axis=0)                    # best branch for min(up, down)
    split = np.minimum(np.nanmax(up, axis=0), np.nanmax(dn, axis=0))  # best up branch and best down branch
    tot1 = np.nanmax(up + dn, axis=0); tot2 = np.nanmax(up, axis=0) + np.nanmax(dn, axis=0)
    print(f"[{rt:g} deg, {ot:g} mm] best-up and best-down branch differ at {int((np.nanargmax(up, 0) != np.nanargmax(dn, 0)).sum())}"
          f" of {len(heads)} headings; min(up, down): one branch max {np.nanmax(single):.1f} median {np.nanmedian(single):.1f}, "
          f"two branches max {np.nanmax(split):.1f} median {np.nanmedian(split):.1f}, largest gain {np.nanmax(split - single):.1f}; "
          f"up + down: one branch median {np.nanmedian(tot1):.1f}, two branches median {np.nanmedian(tot2):.1f}, "
          f"largest gain {np.nanmax(tot2 - tot1):.1f}", flush=True)
    for h in range(0, 120, 15):
        print(f"   h {h}: " + ", ".join(f"{names[n]} {up[n, h]:.1f}/{dn[n, h]:.1f}" for n in range(NB))
              + f" | all {res[NB, h, 0]:.1f}/{res[NB, h, 1]:.1f}", flush=True)
ROLL_TOL, OFF_TOL = LIMITS[0]
with open(os.path.join(out, "pitch_by_heading.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["roll_tol", "off_tol", "heading", "branch", "pitch_up", "pitch_down", "start_pitch", "n_points"])
    w.writerows(pbh)
c0 = (LIM, LIM)
print("sleds at start_deg touch", [names[n] if n >= 0 else "strip" for n in pt_br[c0]], flush=True)
for n in range(NB):
    print(f"  {names[n]} nearest point to sleds at start_deg: "
          f"{min(np.hypot(p[0] - LIM, p[1] - LIM) for p in br_pts[n]):.1f} deg", flush=True)

# --- charts ---------------------------------------------------------------------------------------
def save(fig, name, title):
    fig.suptitle(title, fontsize=10)
    fig.savefig(os.path.join(out, name)); plt.close(fig)


def sled_axes(ax):
    ax.set_xlabel("sled_1 (deg)"); ax.set_ylabel("sled_2 (deg)"); ax.set_aspect("equal")
    for s in ax.spines.values():
        s.set_visible(False)


from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
STRIP = "#bdbdbd"


def tri_patches(ax, coords, colours, alpha=1.0):
    pc = PatchCollection([Polygon(c) for c in coords], facecolors=colours, edgecolors=colours, linewidths=0.2, alpha=alpha)
    ax.add_collection(pc)


# 01: branches in sled space
fig, ax = plt.subplots(figsize=(6.6, 6.0), constrained_layout=True)
sled_xy = [np.array([[axis[a[1]], axis[a[0]]] for a in t[0]]) for t in tris]
tri_patches(ax, sled_xy, [CAT[b] if b >= 0 else STRIP for b in tri_br])
for n, u in enumerate(order):
    ax.text(*cent[u], names[n], ha="center", va="center", fontsize=10, color=INK, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.85))
ax.plot([0], [0], "o", color=INK, ms=4)
ax.plot([], [], "o", color=INK, ms=4, label="sleds at start_deg")
ax.set_xlim(-LIM - 1, LIM + 1); ax.set_ylim(-LIM - 1, LIM + 1)
for n in range(NB):
    ax.plot([], [], "s", color=CAT[n], label=f"branch {names[n]}")
ax.plot([], [], "s", color=STRIP, label="thin strips along folds")
ax.legend(loc="upper right", fontsize=8, frameon=False)
sled_axes(ax)
save(fig, "01_branches_sled_space.png", f"Solution branches in sled space, jam {jam:g}\n"
     "+: det d(roll, pitch)/d(sled_1, sled_2) > 0, -: det < 0; folds (det = 0) between colours")

# 02: each branch's image in roll/pitch, over the whole reachable set
fig, axes = plt.subplots(2, 3, figsize=(12, 8.4), constrained_layout=True, sharex=True, sharey=True)
all_xy = [t[1] for t in tris]
for n, ax in enumerate(axes.flat):
    tri_patches(ax, all_xy, ["#e4e4e4"] * len(all_xy))
    tri_patches(ax, [t[1] for k, t in enumerate(tris) if tri_br[k] == n], [CAT[n]] * int((tri_br == n).sum()))
    ax.plot([0], [0], "o", color=INK, ms=3)
    ax.set_title(f"branch {names[n]}", fontsize=9); ax.set_aspect("equal"); ax.grid(color="#e6e6e6", lw=0.5)
    ax.set_xlim(np.nanmin(R) - 0.5, np.nanmax(R) + 0.5); ax.set_ylim(np.nanmin(P) - 0.5, np.nanmax(P) + 0.5)
    if n >= 3: ax.set_xlabel("roll (deg)")
    if n % 3 == 0: ax.set_ylabel("pitch (deg)")
save(fig, "02_branches_roll_pitch.png", "Roll/pitch reached by each branch (colour) within everything reachable (grey)\n"
     "roll and pitch are the head's world Euler angles, as in 11_solution_count")

# 03: how the solutions differ where a target has a + and a - solution
pair = defaultdict(lambda: ([], []))   # metric -> (+ values, - values) over targets with both
for (a, b), lst in sols.items():
    pos = [v for br, v in lst if br >= 0 and sign_of[order[br]] > 0]
    neg = [v for br, v in lst if br >= 0 and sign_of[order[br]] < 0]
    if pos and neg:
        for key in ("off", "yaw", "gap", "smin", "smax"):
            pair[key][0].extend(v[key] if key != "yaw" else abs(v[key]) for v in pos)
            pair[key][1].extend(v[key] if key != "yaw" else abs(v[key]) for v in neg)
n_pair = sum(1 for lst in sols.values() if {np.sign(sign_of[order[br]]) for br, _ in lst if br >= 0} == {1, -1})
print("targets with both a + and a - solution:", n_pair, flush=True)
for key, (pv, nv) in pair.items():
    print(f"  {key}: + median {np.nanmedian(pv):.3f} (p10 {np.nanpercentile(pv, 10):.3f}, p90 {np.nanpercentile(pv, 90):.3f}),"
          f" - median {np.nanmedian(nv):.3f} (p10 {np.nanpercentile(nv, 10):.3f}, p90 {np.nanpercentile(nv, 90):.3f})", flush=True)
fig, axes = plt.subplots(1, 5, figsize=(13, 4.2), constrained_layout=True)
for ax, (key, label, better) in zip(axes, [("off", "off centre (mm)", "lower is better"),
                                           ("yaw", "|residual yaw| (deg)", "lower is better"),
                                           ("smin", "sigma_min (deg/deg)", "higher is better: further from a fold"),
                                           ("smax", "sigma_max (deg/deg)", "lower is better: less swing"),
                                           ("gap", "sled-gap margin (deg)", "higher is better")]):
    pv, nv = (np.array(x)[np.isfinite(x)] for x in pair[key])
    bp = ax.boxplot([pv, nv], widths=0.5, patch_artist=True, showfliers=False, medianprops=dict(color=INK))
    for patch, c_ in zip(bp["boxes"], ("#8fb8ea", "#f4b392")):
        patch.set_facecolor(c_); patch.set_edgecolor(MUTED)
    ax.set_xticks([1, 2], ["+ branches", "- branches"]); ax.set_ylim(bottom=0)
    ax.set_title(f"{label}\n{better}", fontsize=8.5); ax.grid(axis="y", color="#e6e6e6", lw=0.5)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
save(fig, "03_branch_differences.png", f"Same roll/pitch, different branch: the {n_pair} targets (0.25 deg bins) "
     "with both a + and a - solution\nboxes: 25-75%, whiskers: 1.5 IQR, median line")

# 04: pitch up and down by heading, per branch (|roll| <= 1 deg, off <= 10 mm)
for (rt, ot), fname in ((LIMITS[0], "04_pitch_by_heading.png"), (LIMITS[1], "04b_pitch_by_heading_off20.png")):
    res = RES[rt, ot]
    fig, axes = plt.subplots(2, 1, figsize=(11, 7.6), constrained_layout=True, sharex=True)
    for ax, k, way in zip(axes, (0, 1), ("up", "down")):
        ax.plot(heads, res[NB, :, k], color=MUTED, lw=1.2, ls=":", label="all sled space (folds crossed)")
        for n in range(NB):
            ax.plot(heads, res[n, :, k], color=CAT[n], lw=1.6, label=f"branch {names[n]}")
        ax.set_ylim(0, np.nanmax(res[:, :, :2]) * 1.05); ax.set_xlim(0, 359)
        ax.set_ylabel(f"pitch {way} (deg)"); ax.set_title(f"Pitch {way}, higher is better", fontsize=9, loc="left")
        ax.grid(color="#e6e6e6", lw=0.5); ax.set_xticks(range(0, 361, 30))
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    axes[1].set_xlabel("head heading on the platform (deg, 0 = +X, counter-clockwise)")
    axes[0].legend(loc="upper left", bbox_to_anchor=(1.0, 1.0), fontsize=8, frameon=False)
    save(fig, fname, f"Largest pitch from the most level pose, staying on one branch, |roll| <= {rt:g} deg, off <= {ot:g} mm\n"
         "same pitch/roll definitions as 14_v2; branch names as in 01; the dotted line is hidden where a branch matches it")

# 05: what switching branch between up and down buys
fig, axes = plt.subplots(1, 2, figsize=(12, 4.4), constrained_layout=True)
for (rt, ot), ls_c in zip(LIMITS, (CAT[0], CAT[1])):
    res = RES[rt, ot]; up, dn = res[:NB, :, 0], res[:NB, :, 1]
    single = np.nanmax(np.minimum(up, dn), axis=0); split = np.minimum(np.nanmax(up, 0), np.nanmax(dn, 0))
    tot1 = np.nanmax(up + dn, axis=0); tot2 = np.nanmax(up, 0) + np.nanmax(dn, 0)
    axes[0].plot(heads, split - single, color=ls_c, lw=1.6, label=f"|roll| <= {rt:g} deg, off <= {ot:g} mm")
    axes[1].plot(heads, tot2 - tot1, color=ls_c, lw=1.6, label=f"|roll| <= {rt:g} deg, off <= {ot:g} mm")
for ax, t in zip(axes, ("Gain in min(up, down), higher is better", "Gain in up + down, higher is better")):
    ax.set_title(t, fontsize=9, loc="left"); ax.set_xlim(0, 359); ax.set_ylim(0, 3.5); ax.set_xticks(range(0, 361, 60))
    ax.set_xlabel("head heading on the platform (deg)"); ax.set_ylabel("deg"); ax.grid(color="#e6e6e6", lw=0.5)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
axes[0].legend(loc="upper left", fontsize=8, frameon=False)
save(fig, "05_branch_switch_gain.png", "Pitch gained by using the best branch for up and the best (other) branch for down,\n"
     "over the best single branch, per head heading")
print("written", out, flush=True)
