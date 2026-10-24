# /// script
# requires-python = ">=3.12"
# dependencies = ["matplotlib", "numpy", "pyyaml"]
# ///
"""Kinematics charts for one jam angle, from sweep.py's pass A. Data only.

Usage (Linux or WSL, from the repo root):
    uv run scripts/ik_simulation/jam_charts.py .output/ik_sweep 75
"""
import csv, os, sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

src, jam = sys.argv[1], float(sys.argv[2])
out = os.path.join(src, f"{jam:g}_jam")
os.makedirs(out, exist_ok=True)
INK, MUTED = "#222222", "#666666"
plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.edgecolor": MUTED, "figure.dpi": 130})

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
    return np.ma.masked_invalid(g)


G = {k: grid(k) for k in ("roll", "pitch", "yaw", "off")}
EXT = [-LIM - .5, LIM + .5, -LIM - .5, LIM + .5]


def sled_axes(ax):
    ax.set_xlabel("sled_1 (deg)"); ax.set_ylabel("sled_2 (deg)"); ax.set_aspect("equal")
    for s in ax.spines.values():
        s.set_visible(False)


def save(fig, name, title):
    fig.suptitle(title, fontsize=10)
    fig.savefig(os.path.join(out, name)); plt.close(fig)


def field(name, key, title, unit, diverging, step):
    g = G[key]
    fig, ax = plt.subplots(figsize=(6.4, 5.6), constrained_layout=True)
    if diverging:
        v = np.abs(g).max(); im = ax.imshow(g, origin="lower", extent=EXT, cmap="coolwarm", vmin=-v, vmax=v)
    else:
        im = ax.imshow(g, origin="lower", extent=EXT, cmap="Blues", vmin=0, vmax=g.max())
    levels = np.arange(np.floor(g.min() / step) * step, g.max() + step, step)
    cs = ax.contour(axis, axis, g, levels=levels, colors=INK, linewidths=0.5)
    ax.clabel(cs, fmt="%g", fontsize=7)
    sled_axes(ax); fig.colorbar(im, ax=ax, shrink=0.8, label=unit)
    save(fig, name, f"{title} (contours every {step:g} {unit})")


# --- 01-04: the four outputs over sled space ---------------------------------------------------
field("01_roll.png", "roll", "Head roll", "deg", True, 2)
field("02_pitch.png", "pitch", "Head pitch", "deg", True, 2)
field("03_yaw.png", "yaw", "Head yaw (residual)", "deg", True, 5)
field("04_off.png", "off", "Off centre", "mm", False, 2)

# --- 00: roll, pitch and off centre side by side ------------------------------------------------
fig, axes = plt.subplots(1, 3, figsize=(16, 5.2), constrained_layout=True, sharey=True)
for ax, (key, title, unit, div, step) in zip(axes, [("roll", "Head roll", "deg", True, 2),
                                                     ("pitch", "Head pitch", "deg", True, 2),
                                                     ("off", "Off centre", "mm", False, 2)]):
    g = G[key]
    if div:
        v = np.abs(g).max(); im = ax.imshow(g, origin="lower", extent=EXT, cmap="coolwarm", vmin=-v, vmax=v)
    else:
        im = ax.imshow(g, origin="lower", extent=EXT, cmap="Blues", vmin=0, vmax=g.max())
    cs = ax.contour(axis, axis, g, levels=np.arange(np.floor(g.min() / step) * step, g.max() + step, step),
                    colors=INK, linewidths=0.5)
    ax.clabel(cs, fmt="%g", fontsize=7)
    sled_axes(ax); ax.set_title(f"{title} (contours every {step:g} {unit})", fontsize=9)
    fig.colorbar(im, ax=ax, shrink=0.8, label=unit)
for ax in axes[1:]:
    ax.set_ylabel("")
save(fig, "00_roll_pitch_off.png", "Roll, pitch and off centre")

# --- 05: iso-roll and iso-pitch lines over off centre --------------------------------------------
fig, ax = plt.subplots(figsize=(6.4, 5.6), constrained_layout=True)
light_greys = matplotlib.colors.LinearSegmentedColormap.from_list("light_greys", ["#ffffff", "#9a9a9a"])
im = ax.imshow(G["off"], origin="lower", extent=EXT, cmap=light_greys, vmin=0, vmax=G["off"].max())
for key, colour in (("roll", "#c0392b"), ("pitch", "#2a6fdb")):
    g = G[key]; levels = np.arange(np.floor(g.min() / 2) * 2, g.max() + 2, 2)
    cs = ax.contour(axis, axis, g, levels=levels, colors=colour, linewidths=0.7)
    ax.clabel(cs, fmt="%g", fontsize=6)
ax.plot([], [], color="#c0392b", label="roll (every 2 deg)"); ax.plot([], [], color="#2a6fdb", label="pitch (every 2 deg)")
ax.legend(loc="lower left", fontsize=8, frameon=False)
sled_axes(ax); fig.colorbar(im, ax=ax, shrink=0.8, label="off centre (mm)")
save(fig, "05_iso_roll_pitch.png", "Iso-roll and iso-pitch lines over off centre\n"
     "where a roll line crosses a pitch line, that sled pair gives that roll/pitch")

# --- 06: the sled grid drawn in roll/pitch space -------------------------------------------------
fig, ax = plt.subplots(figsize=(6.4, 6.0), constrained_layout=True)
R, P = G["roll"], G["pitch"]
for k in range(0, N, 4):
    ax.plot(R[:, k], P[:, k], color="#c0392b", lw=0.5)   # constant sled_1
    ax.plot(R[k, :], P[k, :], color="#2a6fdb", lw=0.5)   # constant sled_2
ax.plot([], [], color="#c0392b", label="constant sled_1 (every 4 deg)")
ax.plot([], [], color="#2a6fdb", label="constant sled_2 (every 4 deg)")
ax.plot([0], [0], "o", color=INK, ms=4, label="sleds at start_deg")
ax.legend(loc="lower right", fontsize=8, frameon=False)
ax.set_xlabel("roll (deg)"); ax.set_ylabel("pitch (deg)"); ax.set_aspect("equal"); ax.grid(color="#e6e6e6", lw=0.5)
save(fig, "06_sled_grid_in_roll_pitch.png", "The sled_1/sled_2 grid mapped into roll/pitch\n"
     "where the grid overlaps itself, a roll/pitch has more than one sled solution")

# --- Jacobian ---------------------------------------------------------------------------------
def d(g):
    g = np.ma.filled(g.astype(float), np.nan)
    d1 = np.full_like(g, np.nan); d2 = np.full_like(g, np.nan)
    d1[:, 1:-1] = (g[:, 2:] - g[:, :-2]) / 2; d2[1:-1, :] = (g[2:, :] - g[:-2, :]) / 2
    return d1, d2


(r1, r2), (p1, p2), (o1, o2) = d(G["roll"]), d(G["pitch"]), d(G["off"])
det = r1 * p2 - r2 * p1
J = np.stack([np.stack([r1, r2], -1), np.stack([p1, p2], -1)], -2)
ok = ~np.isnan(J).any(axis=(-1, -2))
sv = np.full((N, N, 2), np.nan); sv[ok] = np.linalg.svd(J[ok], compute_uv=False)

fig, ax = plt.subplots(figsize=(6.4, 5.6), constrained_layout=True)
v = np.nanpercentile(np.abs(det), 99.5)
im = ax.imshow(np.ma.masked_invalid(det), origin="lower", extent=EXT, cmap="coolwarm", vmin=-v, vmax=v)
ax.contour(axis, axis, np.ma.masked_invalid(det), levels=[0], colors=INK, linewidths=1.0)
sled_axes(ax); fig.colorbar(im, ax=ax, shrink=0.8, label="(deg/deg)^2")
save(fig, "07_jacobian_det.png", "det d(roll, pitch)/d(sled_1, sled_2), black line is det = 0\n"
     "colour clipped at the 99.5th percentile")

for name, data, title, unit in [
        ("08_sigma_max.png", sv[..., 0], "Largest singular value of d(roll, pitch)/d(sled_1, sled_2)", "deg/deg"),
        ("09_sigma_min.png", sv[..., 1], "Smallest singular value of d(roll, pitch)/d(sled_1, sled_2)", "deg/deg"),
        ("10_off_gradient.png", np.hypot(o1, o2), "|d off / d(sled_1, sled_2)|", "mm/deg")]:
    fig, ax = plt.subplots(figsize=(6.4, 5.6), constrained_layout=True)
    im = ax.imshow(np.ma.masked_invalid(data), origin="lower", extent=EXT, cmap="Blues", vmin=0,
                   vmax=np.nanpercentile(data, 99.5))
    sled_axes(ax); fig.colorbar(im, ax=ax, shrink=0.8, label=unit)
    save(fig, name, f"{title}\ncolour clipped at the 99.5th percentile")

# --- 11-12: per roll/pitch target, how many sled solutions and their off centre ------------------
# The sled grid is split into triangles, each mapped linearly into roll/pitch. A target's solutions are
# the triangles that contain it; off centre is interpolated in each.
STEP = 0.25
rs = np.arange(np.floor(R.min()), np.ceil(R.max()) + STEP, STEP)
ps = np.arange(np.floor(P.min()), np.ceil(P.max()) + STEP, STEP)
count = np.zeros((len(ps), len(rs)), int)
offs = [[[] for _ in rs] for _ in ps]
Rf, Pf, Of = (np.ma.filled(G[k].astype(float), np.nan) for k in ("roll", "pitch", "off"))
for i in range(N - 1):
    for j in range(N - 1):
        for tri in (((i, j), (i, j + 1), (i + 1, j)), ((i + 1, j + 1), (i + 1, j), (i, j + 1))):
            xy = np.array([[Rf[a], Pf[a]] for a in tri]); o = np.array([Of[a] for a in tri])
            if np.isnan(xy).any():
                continue
            (x0, y0), (x1, y1), (x2, y2) = xy
            den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
            if abs(den) < 1e-12:
                continue
            ci = np.arange(np.searchsorted(rs, xy[:, 0].min()), np.searchsorted(rs, xy[:, 0].max(), "right"))
            pi = np.arange(np.searchsorted(ps, xy[:, 1].min()), np.searchsorted(ps, xy[:, 1].max(), "right"))
            if not len(ci) or not len(pi):
                continue
            X, Y = np.meshgrid(rs[ci], ps[pi])
            l0 = ((y1 - y2) * (X - x2) + (x2 - x1) * (Y - y2)) / den
            l1 = ((y2 - y0) * (X - x2) + (x0 - x2) * (Y - y2)) / den
            l2 = 1 - l0 - l1
            inside = (l0 >= 0) & (l1 >= 0) & (l2 > 0)
            for a, b in zip(*np.nonzero(inside)):
                count[pi[a], ci[b]] += 1
                offs[pi[a]][ci[b]].append(l0[a, b] * o[0] + l1[a, b] * o[1] + l2[a, b] * o[2])

min_off = np.array([[min(v) if v else np.nan for v in row] for row in offs])
max_off = np.array([[max(v) if v else np.nan for v in row] for row in offs])
with open(os.path.join(out, "roll_pitch_targets.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["roll", "pitch", "n_solutions", "min_off", "max_off"])
    for a, p in enumerate(ps):
        for b, r_ in enumerate(rs):
            if count[a, b]:
                w.writerow([round(r_, 2), round(p, 2), count[a, b], round(min_off[a, b], 3), round(max_off[a, b], 3)])

REXT = [rs[0] - STEP / 2, rs[-1] + STEP / 2, ps[0] - STEP / 2, ps[-1] + STEP / 2]
fig, ax = plt.subplots(figsize=(6.4, 5.6), constrained_layout=True)
n = int(count.max())
# 1-3 in clearly stepped blues, 4 and up in oranges/reds; every count distinct, 1 not white
cols = ["#9ecae1", "#4292c6", "#08306b"][:n]
if n > 3:
    cols += list(plt.get_cmap("OrRd")(np.linspace(0.35, 1.0, n - 3)))
cmap = matplotlib.colors.ListedColormap(cols)
im = ax.imshow(np.ma.masked_equal(count, 0), origin="lower", extent=REXT, cmap=cmap, vmin=0.5, vmax=n + 0.5)
ax.set_xlabel("roll (deg)"); ax.set_ylabel("pitch (deg)"); ax.set_aspect("equal"); ax.grid(color="#e6e6e6", lw=0.5)
fig.colorbar(im, ax=ax, shrink=0.8, label="number of sled solutions", ticks=range(1, n + 1))
save(fig, "11_solution_count.png", "Sled solutions per roll/pitch target (white: not reachable)\n"
     "counts in the thin strips (mostly 4+) depend on the 1 deg grid; unchecked")

for name, data, title in [("12_min_off.png", min_off, "Least off centre among the solutions for each roll/pitch target"),
                          ("13_off_spread.png", max_off - min_off, "Off centre spread (max - min) among the solutions")]:
    fig, ax = plt.subplots(figsize=(6.4, 5.6), constrained_layout=True)
    im = ax.imshow(np.ma.masked_invalid(data), origin="lower", extent=REXT, cmap="Blues", vmin=0, vmax=np.nanmax(data))
    ax.set_xlabel("roll (deg)"); ax.set_ylabel("pitch (deg)"); ax.set_aspect("equal"); ax.grid(color="#e6e6e6", lw=0.5)
    fig.colorbar(im, ax=ax, shrink=0.8, label="mm")
    save(fig, name, title)

# --- 14: pure pitch by head heading on the platform ----------------------------------------------
# The head can be mounted facing any heading on the platform. For a heading h (platform frame, 0 = +X,
# counter-clockwise), the head's pitch is the elevation of its forward axis and its roll the elevation
# of its left axis. For each heading: the largest pitch up and down over the sweep points within one
# limit, roll on the left chart and off centre on the right.
with open(os.path.join(src, "sweep_settings.yaml")) as f:
    settings = yaml.safe_load(f)
start = settings["neck_yaml"]["sled"]["start_deg"]
rr, pp, yy = (np.radians(A[:, col[k]]) for k in ("roll", "pitch", "yaw"))
# Third row of R = Rz(yaw) Ry(pitch) Rx(roll): the z components of the rotated platform x and y axes.
r20, r21 = -np.sin(pp), np.cos(pp) * np.sin(rr)
heads = np.radians(np.arange(0, 361, 1))
c, s = np.cos(heads)[:, None], np.sin(heads)[:, None]
h_pitch = np.degrees(np.arcsin(np.clip(r20 * c + r21 * s, -1, 1)))    # [heading, point]
h_roll = np.degrees(np.arcsin(np.clip(-r20 * s + r21 * c, -1, 1)))
off = A[:, col["off"]][None, :]
UP = ["#9ecae1", "#4292c6", "#08306b"]     # tight to loose
DOWN = ["#fdae6b", "#f16913", "#8c2d04"]
ROLL_TOLS, OFF_TOLS = [0.5, 1, 2], [3, 5, 10]   # deg, mm
abs_roll, off_all = np.abs(h_roll), np.broadcast_to(off, h_pitch.shape)


def best_pitch(ok, sign):
    best = np.where(ok, sign * h_pitch, -np.inf).max(axis=1)
    return np.where(np.isfinite(best), np.maximum(best, 0), 0)


def polar_frame(ax, rmax):
    ax.set_ylim(0, np.ceil(rmax) + 1)
    # Joints at rest: sled (bottom) pivots at start_deg, rod top pivots twisted round by the jam.
    # Orbit angles are 0 = -Y, so in this frame they sit at orbit - 90.
    for i, d in enumerate(start, 1):
        for a, style, name in ((d - 90, "-", f"sled {i}"), (d + jam - 90, "--", f"top {i}")):
            ax.plot([np.radians(a)] * 2, [0, np.ceil(rmax) + 1], color="#444444", lw=1.0, ls=style)
            ax.text(np.radians(a), np.ceil(rmax) * 0.93, name, fontsize=7, color="#444444", ha="center", va="center",
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none"))
    ax.set_rlabel_position(200); ax.tick_params(axis="y", labelsize=7)
    ax.set_xticks(np.radians(np.arange(0, 360, 30)))


# v1: roll and off centre limited together; one chart for up, one for down.
fig, axes = plt.subplots(1, 2, figsize=(12, 6.2), subplot_kw={"projection": "polar"}, constrained_layout=True)
curves = [[(best_pitch((abs_roll <= r) & (off_all <= o), sign), colour, f"|roll| <= {r:g} deg, off <= {o:g} mm")
           for r, o, colour in zip(ROLL_TOLS, OFF_TOLS, UP)] for sign in (1, -1)]
rmax = max(c[0].max() for cs in curves for c in cs)
for ax, cs, label in zip(axes, curves, ("Pitch up (head looks up)", "Pitch down (head looks down)")):
    for best, colour, name in cs:
        ax.plot(heads, best, color=colour, lw=2, label=name)
    ax.set_title(label, fontsize=9, pad=14)
    polar_frame(ax, rmax)
axes[1].legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8, frameon=False)
save(fig, "14_pitch_by_heading_v1.png", "Largest pitch (deg) by head heading on the platform, within roll and off-centre limits\n"
     "angle: heading, 0 = +X; grey solid: sled pivots, grey dashed: rod top pivots, at rest")

# v2: rows 1-2 limit roll and off centre separately (columns), up and down (rows);
# row 3 limits both together, pitch up on the left and down on the right.
V2_OFF_TOLS = [5, 10, 20]   # mm
fig, axes = plt.subplots(3, 2, figsize=(12, 21.5), subplot_kw={"projection": "polar"}, constrained_layout=True)
fig.get_layout_engine().set(w_pad=0.02, h_pad=0.04, hspace=0.02, wspace=0.02)
grid_curves = {}
for col_i, (err, tols, fmt) in enumerate([(abs_roll, ROLL_TOLS, "|roll| <= {:g} deg"),
                                          (off_all, V2_OFF_TOLS, "off <= {:g} mm")]):
    for row_i, (sign, colours) in enumerate(((1, UP), (-1, DOWN))):
        way = ("up", "down")[row_i]
        grid_curves[row_i, col_i] = (f"Pitch {way}, limited by {('roll', 'off centre')[col_i]} only",
                                     [(best_pitch(err <= t, sign), c, fmt.format(t)) for t, c in zip(tols, colours)])
for col_i, (sign, colours, way) in enumerate(((1, UP, "up"), (-1, DOWN, "down"))):
    grid_curves[2, col_i] = (f"Pitch {way}, limited by roll and off centre together",
                             [(best_pitch((abs_roll <= r) & (off_all <= o), sign), c, f"{r:g} deg, {o:g} mm")
                              for r, o, c in zip(ROLL_TOLS, V2_OFF_TOLS, colours)])
rmax = max(c[0].max() for _, cs in grid_curves.values() for c in cs)
for (row_i, col_i), (title, cs) in grid_curves.items():
    ax = axes[row_i, col_i]
    for best, colour, name in cs:
        ax.plot(heads, best, color=colour, lw=2, label=name)
    ax.set_title(title, fontsize=10, pad=14)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.06), ncol=3, fontsize=8, frameon=False,
              columnspacing=1.2, handlelength=1.5)
    polar_frame(ax, rmax)
save(fig, "14_pitch_by_heading_v2.png", "Largest pitch (deg) by head heading on the platform\n"
     "angle: heading, 0 = +X; grey solid: sled pivots, grey dashed: rod top pivots, at rest")
print("written", out, "max solutions", n, flush=True)
