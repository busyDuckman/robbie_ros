# /// script
# requires-python = ">=3.12"
# dependencies = ["matplotlib", "numpy"]
# ///
"""CW vs CCW (viewed from the top) charts and mirror check, from cw_ccw.py's sweeps. Data only.

Usage (Linux or WSL, from the repo root):
    uv run scripts/ik_simulation/cw_ccw_charts.py .output/ik_sweep/cw_ccw

Same metric definitions as plots.py (tilt, coverage) and jam_charts.py (heatmaps, chart 14_v2: head
heading in the platform frame, 0 = +X, CCW; pitch = elevation of the head's forward axis, roll =
elevation of its left axis).
"""
import csv, itertools, math, os, sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

out = sys.argv[1]
src = os.path.dirname(os.path.abspath(out.rstrip("/")))   # .output/ik_sweep
INK, MUTED = "#222222", "#666666"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
COL = {"CW": SERIES[0], "CCW": SERIES[1]}
plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.edgecolor": MUTED, "figure.dpi": 130})
JAM = 75
START = [0, 120, 240]


def load(path, jam=None):
    with open(path) as f:
        r = csv.reader(f); head = next(r)
        a = np.array([[float(x) for x in row] for row in r])
    if jam is not None:
        a = a[a[:, 0] == jam]
    return {h: i for i, h in enumerate(head)}, a


rest = {r["direction_viewed_from_top"]: r for r in csv.DictReader(open(os.path.join(out, "rest_checks.csv")))}
ORDER = [d for d in ("CW", "CCW") if d in rest]
CURRENT = next(d for d, r in rest.items() if r["model"] == "current")
col, _ = load(os.path.join(out, f"sweep_{ORDER[0].lower()}_pass_a.csv"))
A = {d: load(os.path.join(out, f"sweep_{d.lower()}_pass_a.csv"))[1] for d in ORDER}
B = {d: load(os.path.join(out, f"sweep_{d.lower()}_pass_b.csv"))[1] for d in ORDER}
LIM = int(np.max(np.abs(A[ORDER[0]][:, 1]))); N = 2 * LIM + 1
EXT = [-LIM - .5, LIM + .5, -LIM - .5, LIM + .5]
axis = np.arange(-LIM, LIM + 1)
LABEL = {d: f"{d} viewed from top" + (" (current model)" if d == CURRENT else " (mirrored)") for d in ORDER}
summary = []   # (name, value) lines for summary.csv


def keyed(a):
    return {(int(r[1]), int(r[2])): r for r in a}


# ---------------------------------------------------------------- reproduction and pass checks
_, OLD = load(os.path.join(src, "sweep_pass_a.csv"), JAM)
new, old = keyed(A[CURRENT]), keyed(OLD)
common = sorted(set(new) & set(old))
with open(os.path.join(out, "reproduce_check.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["column", "rows_in_existing", "rows_in_rerun", "rows_matched", "max_abs_diff"])
    for k in ("roll", "pitch", "yaw", "off_x", "off_y", "off"):
        dmax = max(abs(new[c][col[k]] - old[c][col[k]]) for c in common)
        w.writerow([k, len(old), len(new), len(common), round(dmax, 4)])
        summary.append((f"reproduce {CURRENT} vs existing sweep_pass_a jam 75, max |d {k}|", round(dmax, 4)))
for d in ORDER:
    a, b = keyed(A[d]), keyed(B[d])
    pd = max(max(abs(b[c][col["roll"]] - a[c][col["roll"]]), abs(b[c][col["pitch"]] - a[c][col["pitch"]])) for c in a)
    summary.append((f"{d} pass B - pass A, max(|d roll|, |d pitch|) deg", round(pd, 4)))


def grid(a, key):
    g = np.full((N, N), np.nan)
    g[(a[:, 2] + LIM).astype(int), (a[:, 1] + LIM).astype(int)] = a[:, col[key]]
    return np.ma.masked_invalid(g)


def sled_axes(ax):
    ax.set_xlabel("sled_1 (deg)"); ax.set_ylabel("sled_2 (deg)"); ax.set_aspect("equal")
    for s in ax.spines.values():
        s.set_visible(False)


def save(fig, name, title):
    fig.suptitle(title, fontsize=10)
    fig.savefig(os.path.join(out, name)); plt.close(fig)


# ---------------------------------------------------------------- 01 heatmaps
FIELDS = [("roll", "Head roll", "deg", True, 2), ("pitch", "Head pitch", "deg", True, 2),
          ("yaw", "Head yaw (residual)", "deg", True, 5), ("off", "Off centre", "mm", False, 2)]
fig, axes = plt.subplots(len(FIELDS), 2, figsize=(10.5, 4.6 * len(FIELDS)), constrained_layout=True)
for ri, (key, title, unit, div, step) in enumerate(FIELDS):
    gs = {d: grid(A[d], key) for d in ORDER}
    v = max(np.abs(g).max() for g in gs.values())
    for ci, d in enumerate(ORDER):
        ax, g = axes[ri, ci], gs[d]
        if div:
            im = ax.imshow(g, origin="lower", extent=EXT, cmap="coolwarm", vmin=-v, vmax=v)
        else:
            im = ax.imshow(g, origin="lower", extent=EXT, cmap="Blues", vmin=0, vmax=v)
        cs = ax.contour(axis, axis, g, levels=np.arange(np.floor(g.min() / step) * step, g.max() + step, step),
                        colors=INK, linewidths=0.5)
        ax.clabel(cs, fmt="%g", fontsize=6)
        sled_axes(ax); ax.set_title(f"{title}, {LABEL[d]}\ncontours every {step:g} {unit}", fontsize=9)
    fig.colorbar(im, ax=axes[ri, :], shrink=0.85, label=unit)
save(fig, "01_heatmaps_cw_ccw.png", f"Roll, pitch, yaw and off centre over (sled_1, sled_2), jam {JAM}, pass A")

# ---------------------------------------------------------------- 02 coverage
fig, axes = plt.subplots(1, 2, figsize=(11, 5.4), constrained_layout=True, sharex=True, sharey=True)
vmax = max(np.percentile(A[d][:, col["off"]], 99.5) for d in ORDER)
for ax, d in zip(axes, ORDER):
    sc = ax.scatter(A[d][:, col["roll"]], A[d][:, col["pitch"]], c=A[d][:, col["off"]], s=2, cmap="Blues",
                    vmin=0, vmax=vmax, linewidths=0)
    ax.set_title(LABEL[d], fontsize=9); ax.set_aspect("equal"); ax.grid(color="#e6e6e6", lw=0.5)
    ax.set_xlabel("roll (deg)"); ax.set_ylabel("pitch (deg)")
fig.colorbar(sc, ax=axes, shrink=0.8, label="off centre (mm), colour clipped at the 99.5th percentile")
save(fig, "02_roll_pitch_coverage_cw_ccw.png", f"Roll vs pitch (Euler, as in roll.png/pitch.png) at every sweep point, jam {JAM}, pass A")

# ---------------------------------------------------------------- 03 max tilt
TILT_BINS = [5, 10, 15]
rows = []
for d in ORDER:
    a = A[d]
    tilt = np.degrees(np.arccos(np.cos(np.radians(a[:, col["roll"]])) * np.cos(np.radians(a[:, col["pitch"]]))))
    i = int(np.argmax(tilt))
    means = [a[np.abs(tilt - t) <= 0.5, col["off"]].mean() for t in TILT_BINS]
    rows.append([d, round(tilt[i], 2), *a[i, 1:4].astype(int), round(a[i, col["off"]], 2), *np.round(means, 2)])
with open(os.path.join(out, "max_tilt.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["direction_viewed_from_top", "max_tilt", "sled_1", "sled_2", "sled_3", "off",
                *(f"mean_off_at_{t}deg" for t in TILT_BINS)])
    w.writerows(rows)
for r in rows:
    summary.append((f"{r[0]} max tilt deg (sleds {r[2]}, {r[3]}, {r[4]}; off {r[5]} mm)", r[1]))
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.8), constrained_layout=True)
x = np.arange(len(ORDER))
ax1.bar(x, [r[1] for r in rows], color=[COL[d] for d in ORDER], width=0.5)
for xi, r in zip(x, rows):
    ax1.text(xi, r[1], f"{r[1]:.2f}", ha="center", va="bottom", fontsize=8)
ax1.set_xticks(x, [f"{d}\nviewed from top" for d in ORDER]); ax1.set_ylabel("max tilt (deg)\nhigher is better")
labels = ["at max tilt", *(f"mean at {t} deg tilt" for t in TILT_BINS)]
for k, d in enumerate(ORDER):
    r = rows[k]
    ax2.plot(range(4), [r[5], *r[6:]], "o--", color=COL[d], lw=2, ms=5, label=f"{d} viewed from top")
ax2.set_xticks(range(4), labels, fontsize=8); ax2.set_ylabel("off centre (mm)\nlower is better"); ax2.legend(frameon=False, fontsize=8)
for ax in (ax1, ax2):
    ax.set_ylim(0, ax.get_ylim()[1] * 1.1); ax.grid(color="#e6e6e6", lw=0.5, axis="y")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
save(fig, "03_max_tilt_cw_ccw.png", f"Max platform tilt (angle from level) and off centre, jam {JAM}, pass A\n"
     "bars: higher is better; dashed: lower is better")


# ---------------------------------------------------------------- pitch by heading (jam_charts.py 14)
def by_heading(a):
    rr, pp = np.radians(a[:, col["roll"]]), np.radians(a[:, col["pitch"]])
    r20, r21 = -np.sin(pp), np.cos(pp) * np.sin(rr)
    heads = np.radians(np.arange(0, 361, 1))
    c, s = np.cos(heads)[:, None], np.sin(heads)[:, None]
    h_pitch = np.degrees(np.arcsin(np.clip(r20 * c + r21 * s, -1, 1)))
    h_roll = np.degrees(np.arcsin(np.clip(-r20 * s + r21 * c, -1, 1)))
    return heads, h_pitch, h_roll, np.broadcast_to(a[:, col["off"]][None, :], h_pitch.shape)


def best_pitch(h_pitch, ok, sign):
    best = np.where(ok, sign * h_pitch, -np.inf).max(axis=1)
    return np.where(np.isfinite(best), np.maximum(best, 0), 0)


def best_index(h_pitch, ok, sign):
    return np.where(ok, sign * h_pitch, -np.inf).argmax(axis=1)


HB = {d: by_heading(A[d]) for d in ORDER}
UP = ["#9ecae1", "#4292c6", "#08306b"]; DOWN = ["#fdae6b", "#f16913", "#8c2d04"]
ROLL_TOLS, V2_OFF_TOLS = [0.5, 1, 2], [5, 10, 20]


def polar_frame(ax, rmax, d):
    ax.set_ylim(0, np.ceil(rmax) + 1)
    orbit = [float(rest[d][f"orbit_{i}"]) for i in (1, 2, 3)]   # measured in the sim at rest
    for i, (st, ob) in enumerate(zip(START, orbit), 1):
        for a_, style, name in ((st - 90, "-", f"sled {i}"), (st + ob - 90, "--", f"top {i}")):
            ax.plot([np.radians(a_)] * 2, [0, np.ceil(rmax) + 1], color="#444444", lw=1.0, ls=style)
            ax.text(np.radians(a_), np.ceil(rmax) * 0.93, name, fontsize=7, color="#444444", ha="center", va="center",
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none"))
    ax.set_rlabel_position(200); ax.tick_params(axis="y", labelsize=7)
    ax.set_xticks(np.radians(np.arange(0, 360, 30)))


def v2_curves(d):
    heads, hp, hr, off = HB[d]; ar = np.abs(hr); g = {}
    for col_i, (err, tols, fmt) in enumerate([(ar, ROLL_TOLS, "|roll| <= {:g} deg"), (off, V2_OFF_TOLS, "off <= {:g} mm")]):
        for row_i, (sign, colours) in enumerate(((1, UP), (-1, DOWN))):
            way = ("up", "down")[row_i]
            g[row_i, col_i] = (f"Pitch {way}, limited by {('roll', 'off centre')[col_i]} only",
                               [(best_pitch(hp, err <= t, sign), c, fmt.format(t)) for t, c in zip(tols, colours)])
    for col_i, (sign, colours, way) in enumerate(((1, UP, "up"), (-1, DOWN, "down"))):
        g[2, col_i] = (f"Pitch {way}, limited by roll and off centre together",
                       [(best_pitch(hp, (ar <= r) & (off <= o), sign), c, f"{r:g} deg, {o:g} mm")
                        for r, o, c in zip(ROLL_TOLS, V2_OFF_TOLS, colours)])
    return heads, g


V2 = {d: v2_curves(d) for d in ORDER}
RMAX = max(c[0].max() for d in ORDER for _, cs in V2[d][1].values() for c in cs)
for d in ORDER:
    heads, g = V2[d]
    fig, axes = plt.subplots(3, 2, figsize=(12, 21.5), subplot_kw={"projection": "polar"}, constrained_layout=True)
    fig.get_layout_engine().set(w_pad=0.02, h_pad=0.04, hspace=0.02, wspace=0.02)
    for (row_i, col_i), (title, cs) in g.items():
        ax = axes[row_i, col_i]
        for best, colour, name in cs:
            ax.plot(heads, best, color=colour, lw=2, label=name)
        ax.set_title(title, fontsize=10, pad=14)
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.06), ncol=3, fontsize=8, frameon=False,
                  columnspacing=1.2, handlelength=1.5)
        polar_frame(ax, RMAX, d)
    save(fig, f"04_pitch_by_heading_v2_{d.lower()}.png",
         f"Largest pitch (deg) by head heading on the platform, {LABEL[d]}, jam {JAM} (same scale for CW and CCW)\n"
         "angle: heading, 0 = +X, CCW; grey solid: sled pivots, grey dashed: rod top pivots, at rest")

# 05: CW and CCW on the same axes, row 3 limits of v2 (roll and off together), up and down.
heads_deg = np.arange(0, 361)
fig, axes = plt.subplots(3, 2, figsize=(12, 10), constrained_layout=True, sharex=True)
pbh_rows = {}
for row_i, (rt, ot) in enumerate(zip(ROLL_TOLS, V2_OFF_TOLS)):
    for col_i, (sign, way) in enumerate(((1, "up"), (-1, "down"))):
        ax = axes[row_i, col_i]
        for d in ORDER:
            heads, hp, hr, off = HB[d]
            b = best_pitch(hp, (np.abs(hr) <= rt) & (off <= ot), sign)
            pbh_rows[d, way, rt, ot] = b
            ax.plot(heads_deg, b, "-", color=COL[d], lw=2, label=f"{d} viewed from top")
        ax.set_ylim(0, RMAX * 1.05); ax.set_xlim(0, 360); ax.set_xticks(range(0, 361, 30))
        ax.set_title(f"Pitch {way}, |roll| <= {rt:g} deg and off <= {ot:g} mm", fontsize=9)
        ax.set_ylabel(f"largest pitch {way} (deg)\nhigher is better"); ax.grid(color="#e6e6e6", lw=0.5)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
for ax in axes[-1]:
    ax.set_xlabel("head heading on the platform (deg, 0 = +X, CCW)")
axes[0, 1].legend(frameon=False, fontsize=8)
save(fig, "05_pitch_by_heading_cw_vs_ccw.png",
     f"Largest pitch up and down by head heading, CW vs CCW viewed from top, jam {JAM} (14_v2 row 3 definitions)")

with open(os.path.join(out, "pitch_by_heading.csv"), "w", newline="") as f:
    w = csv.writer(f)
    keys = sorted({(rt, ot) for _, _, rt, ot in pbh_rows})
    w.writerow(["heading", *(f"{d.lower()}_{way}_roll{rt:g}_off{ot:g}" for rt, ot in keys for d in ORDER for way in ("up", "down"))])
    for hi, h in enumerate(heads_deg[:-1]):
        w.writerow([h, *(round(pbh_rows[d, way, rt, ot][hi], 3) for rt, ot in keys for d in ORDER for way in ("up", "down"))])

# 06: CCW minus CW at the same heading, per limit set (positive: CCW reaches further).
fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True, sharey=True)
if len(ORDER) == 2:
    for col_i, way in enumerate(("up", "down")):
        ax = axes[col_i]
        for k, (rt, ot) in enumerate(zip(ROLL_TOLS, V2_OFF_TOLS)):
            diff = pbh_rows["CCW", way, rt, ot] - pbh_rows["CW", way, rt, ot]
            ax.plot(heads_deg, diff, "-", color=SERIES[k + 2], lw=1.6, label=f"|roll| <= {rt:g} deg, off <= {ot:g} mm")
        ax.axhline(0, color=INK, lw=0.8, ls=":")
        ax.set_xlim(0, 360); ax.set_xticks(range(0, 361, 30)); ax.grid(color="#e6e6e6", lw=0.5)
        ax.set_title(f"Pitch {way}: CCW minus CW at the same heading", fontsize=9)
        ax.set_xlabel("head heading on the platform (deg, 0 = +X, CCW)")
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    axes[0].set_ylabel("deg (above 0: CCW reaches further,\nbelow 0: CW reaches further)")
    fig.legend(*axes[1].get_legend_handles_labels(), frameon=False, fontsize=8, loc="outside lower center", ncol=3)
save(fig, "06_pitch_by_heading_difference.png", "Largest pitch, CCW minus CW viewed from top, at each head heading\n"
     "dotted: zero, context")

# ---------------------------------------------------------------- mirror check
def rotm(r, p, y):
    r, p, y = np.radians(r), np.radians(p), np.radians(y)
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.empty(r.shape + (3, 3))
    R[..., 0, 0] = cy * cp; R[..., 0, 1] = cy * sp * sr - sy * cr; R[..., 0, 2] = cy * sp * cr + sy * sr
    R[..., 1, 0] = sy * cp; R[..., 1, 1] = sy * sp * sr + cy * cr; R[..., 1, 2] = sy * sp * cr - cy * sr
    R[..., 2, 0] = -sp; R[..., 2, 1] = cp * sr; R[..., 2, 2] = cp * cr
    return R


def rz(deg):
    a = math.radians(deg); return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


if len(ORDER) == 2:
    cw, ccw = keyed(A["CW"]), keyed(A["CCW"])
    results = []
    for perm in itertools.permutations(range(3)):
        for sg in (1, -1):
            pairs = []
            for (s1, s2), r in ccw.items():
                s = (s1, s2, -s1 - s2)
                m = tuple(sg * s[perm[k]] for k in range(3))   # CW sleds that should match this CCW point
                if (m[0], m[1]) in cw:
                    pairs.append((r, cw[m[0], m[1]]))
            if len(pairs) < 100:
                continue
            Pc = np.array([p[0] for p in pairs]); Pw = np.array([p[1] for p in pairs])
            Rc = rotm(Pc[:, col["roll"]], Pc[:, col["pitch"]], Pc[:, col["yaw"]])
            Rw = rotm(Pw[:, col["roll"]], Pw[:, col["pitch"]], Pw[:, col["yaw"]])
            pc = np.stack([Pc[:, col["off_x"]], Pc[:, col["off_y"]], np.zeros(len(Pc))], 1)
            pw = np.stack([Pw[:, col["off_x"]], Pw[:, col["off_y"]], np.zeros(len(Pw))], 1)
            for k in range(6):
                for refl in (False, True):
                    M = rz(60 * k) @ (np.diag([-1, 1, 1]) if refl else np.eye(3))
                    Rp = M @ Rw @ M.T
                    ang = np.degrees(np.arccos(np.clip((np.einsum("nij,nij->n", Rc, Rp) - 1) / 2, -1, 1)))
                    dp = np.linalg.norm(pc - pw @ M.T, axis=1)
                    results.append((np.median(ang), perm, sg, k * 60, refl, len(pairs), np.median(ang), ang.max(),
                                    np.median(dp), dp.max(), Pc, Pw, M))
    results.sort(key=lambda t: t[0])
    with open(os.path.join(out, "mirror_check.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["ccw_sled_k_equals", "world_map", "points", "rot_err_median_deg", "rot_err_max_deg",
                    "pos_err_median_mm", "pos_err_max_mm"])
        for _, perm, sg, rot, refl, n, am, ax_, dm, dx, *_ in results:
            mapping = ", ".join(f"ccw s{k + 1} = {'-' if sg < 0 else '+'}cw s{perm[k] + 1}" for k in range(3))
            wm = f"Rz({rot}){' * mirror x->-x' if refl else ''}"
            w.writerow([mapping, wm, n, round(am, 4), round(ax_, 4), round(dm, 4), round(dx, 4)])
    best = results[0]
    _, perm, sg, rot, refl, n, am, amax, dm, dmax, Pc, Pw, M = best
    mapping = ", ".join(f"ccw s{k + 1} = {'-' if sg < 0 else '+'}cw s{perm[k] + 1}" for k in range(3))
    summary.append(("best mirror map (sleds)", mapping))
    summary.append(("best mirror map (world)", f"Rz({rot}){' * mirror x->-x' if refl else ''}"))
    summary.append(("best mirror map: points matched", n))
    summary.append(("best mirror map: orientation error median / max deg", f"{am:.4f} / {amax:.4f}"))
    summary.append(("best mirror map: position error median / max mm", f"{dm:.4f} / {dmax:.4f}"))
    summary.append(("next best map: orientation error median deg", round(results[1][6], 4)))
    # The plain mirror x -> -x (plane through sled_1's start, Rz(0) above): per-angle sign tests on its pairs.
    xm = next(r for r in results if r[3] == 0 and r[4])
    Pc, Pw = xm[10], xm[11]
    summary.append(("mirror x->-x (sled map ccw s1=-cw s1, s2=-cw s3, s3=-cw s2): orientation error median / max deg",
                    f"{xm[6]:.4f} / {xm[7]:.4f}"))
    for k in ("roll", "pitch", "yaw", "off_x", "off_y"):
        a_, b_ = Pc[:, col[k]], Pw[:, col[k]]
        summary.append((f"mirror x->-x pairs: {k} ccw = +cw, max |diff|", round(np.abs(a_ - b_).max(), 4)))
        summary.append((f"mirror x->-x pairs: {k} ccw = -cw, max |diff|", round(np.abs(a_ + b_).max(), 4)))
    # Head pitch/roll at heading h (14_v2 definitions) on those pairs: ccw at h vs cw at 180 - h.
    _, hpc, hrc, _ = by_heading(Pc); _, hpw, hrw, _ = by_heading(Pw)
    hidx = (180 - np.arange(361)) % 360
    summary.append(("mirror x->-x pairs: head pitch ccw(h) = +cw(180-h), max |diff| over all h", round(np.abs(hpc - hpw[hidx]).max(), 4)))
    summary.append(("mirror x->-x pairs: head roll ccw(h) = -cw(180-h), max |diff| over all h", round(np.abs(hrc + hrw[hidx]).max(), 4)))
    # Largest pitch by heading: ccw at h vs cw at c - h (the mirror planes through the three sleds).
    for rt, ot in zip(ROLL_TOLS, V2_OFF_TOLS):
        for way in ("up", "down"):
            yc = pbh_rows["CCW", way, rt, ot][:-1]; yw = pbh_rows["CW", way, rt, ot][:-1]
            fits = [f"cw {way}({c}-h): {np.abs(yc - yw[(c - np.arange(360)) % 360]).max():.3f}" for c in (60, 180, 300)]
            summary.append((f"|roll|<={rt:g}, off<={ot:g}: ccw {way}(h) vs mirrored heading, max |diff| deg", "; ".join(fits)))
            same = np.abs(yc - pbh_rows["CW", way, rt, ot][:-1]).max()
            summary.append((f"|roll|<={rt:g}, off<={ot:g}: ccw {way}(h) vs cw {way}(h) same heading, max |diff| deg", round(same, 3)))
        du = pbh_rows["CCW", "up", rt, ot][:-1] - pbh_rows["CW", "up", rt, ot][:-1]
        dd = pbh_rows["CCW", "down", rt, ot][:-1] - pbh_rows["CW", "down", rt, ot][:-1]
        e = 0.05
        counts = {"CW further up and CCW further down": int(((du < -e) & (dd > e)).sum()),
                  "CCW further up and CW further down": int(((du > e) & (dd < -e)).sum()),
                  "CW further both ways": int(((du < -e) & (dd < -e)).sum()),
                  "CCW further both ways": int(((du > e) & (dd > e)).sum())}
        counts["within 0.05 deg one way or both"] = 360 - sum(counts.values())
        summary.append((f"|roll|<={rt:g}, off<={ot:g}: headings (of 360, 1 deg) where", "; ".join(f"{k}: {v}" for k, v in counts.items())))

# ---------------------------------------------------------------- poses for the renders
# Balanced heading as plots.py: the heading with the largest min(up, down) at |roll| <= 1 deg, off <= 10 mm.
poses, best_h = [], {}
for d in ORDER:
    up, dn = pbh_rows[d, "up", 1, 10][:-1], pbh_rows[d, "down", 1, 10][:-1]
    best_h[d] = int(np.argmax(np.minimum(up, dn)))
    summary.append((f"{d} balanced heading (|roll|<=1, off<=10): heading deg / up deg / down deg",
                    f"{best_h[d]} / {up[best_h[d]]:.2f} / {dn[best_h[d]]:.2f}"))
for h in sorted(set(best_h.values())):
    who = " and ".join(d for d in ORDER if best_h[d] == h)
    name = f"07_poses_heading_{h:03d}"
    title = (f"CW vs CCW viewed from top at head heading {h} deg ({who}'s balanced heading), jam {JAM}: "
             "rest, largest pitch up, largest pitch down (|roll| <= 1 deg, off <= 10 mm)")
    for d in ORDER:
        heads, hp, hr, off = HB[d]; a = A[d]
        ok = (np.abs(hr) <= 1) & (off <= 10)
        r0 = int(np.flatnonzero((a[:, 1] == 0) & (a[:, 2] == 0))[0])
        sel = [("rest", r0), ("largest pitch up", int(best_index(hp, ok, 1)[h])),
               ("largest pitch down", int(best_index(hp, ok, -1)[h]))]
        for pose, i in sel:
            poses.append([name, title, pose, d, h, *a[i, 1:4].astype(int), round(hp[h, i], 3), round(hr[h, i], 3),
                          round(a[i, col["off"]], 3)])
with open(os.path.join(out, "poses.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["figure", "title", "pose", "direction", "heading", "sled_1", "sled_2", "sled_3", "head_pitch",
                "head_roll", "off"])
    w.writerows(poses)
with open(os.path.join(out, "summary.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["measure", "value"]); w.writerows(summary)
for k, v in summary:
    print(f"{k}: {v}", flush=True)
