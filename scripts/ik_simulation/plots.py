# /// script
# requires-python = ">=3.12"
# dependencies = ["matplotlib", "numpy"]
# ///
"""Derived tables and plots from sweep.py's output. Data only.

Usage (Linux or WSL, from the repo root):
    uv run scripts/ik_simulation/plots.py .output/ik_sweep
"""
import csv, os, sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

out = sys.argv[1]
INK, MUTED = "#222222", "#666666"
plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.edgecolor": MUTED, "figure.dpi": 130})


def load(name):
    with open(os.path.join(out, name)) as f:
        r = csv.reader(f); head = next(r)
        a = np.array([[float(x) for x in row] for row in r])
    return head, a


head, A = load("sweep_pass_a.csv")
_, B = load("sweep_pass_b.csv")
col = {h: i for i, h in enumerate(head)}
jams = sorted(set(A[:, 0]))
LIM = int(np.max(np.abs(A[:, 1])))
N = 2 * LIM + 1


def grid(data, jam, key):
    g = np.full((N, N), np.nan)   # [sled_2, sled_1]
    rows = data[data[:, 0] == jam]
    g[(rows[:, 2] + LIM).astype(int), (rows[:, 1] + LIM).astype(int)] = rows[:, col[key]]
    return g


def by_key(data, values):
    """Grids for a per-row value array, one per jam."""
    gs = {}
    for jam in jams:
        m = data[:, 0] == jam
        g = np.full((N, N), np.nan)
        g[(data[m, 2] + LIM).astype(int), (data[m, 1] + LIM).astype(int)] = values[m]
        gs[jam] = g
    return gs


# --- pass B - pass A ------------------------------------------------------------------------
keyA = {(r[0], r[1], r[2]): r for r in A}
diff = []
for r in B:
    a = keyA[(r[0], r[1], r[2])]
    diff.append([r[0], r[1], r[2], *(r[col[k]] - a[col[k]] for k in ("roll", "pitch", "yaw", "off"))])
diff = np.array(diff)
with open(os.path.join(out, "pass_diff.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["jam_deg", "sled_1", "sled_2", "d_roll", "d_pitch", "d_yaw", "d_off"])
    w.writerows([[int(r[0]), int(r[1]), int(r[2]), *np.round(r[3:], 3)] for r in diff])

# --- Jacobian (pass A), central differences on the 1 deg grid --------------------------------
jac_rows = []
jac = {}
for jam in jams:
    g = {k: grid(A, jam, k) for k in ("roll", "pitch", "off")}
    J = {}
    for k in g:
        d1 = (g[k][1:-1, 2:] - g[k][1:-1, :-2]) / 2   # d/d sled_1
        d2 = (g[k][2:, 1:-1] - g[k][:-2, 1:-1]) / 2   # d/d sled_2
        J[k] = (np.pad(d1, 1, constant_values=np.nan), np.pad(d2, 1, constant_values=np.nan))
    smax, smin = np.full((N, N), np.nan), np.full((N, N), np.nan)
    for i in range(N):
        for j in range(N):
            m = np.array([[J["roll"][0][i, j], J["roll"][1][i, j]], [J["pitch"][0][i, j], J["pitch"][1][i, j]]])
            if np.isnan(m).any() or np.isnan(J["off"][0][i, j]) or np.isnan(J["off"][1][i, j]):
                continue
            s = np.linalg.svd(m, compute_uv=False)
            smax[i, j], smin[i, j] = s
            jac_rows.append([int(jam), j - LIM, i - LIM, *np.round(m.ravel(), 4),
                             round(J["off"][0][i, j], 4), round(J["off"][1][i, j], 4), round(s[0], 4), round(s[1], 4)])
    jac[jam] = (smax, smin)
with open(os.path.join(out, "jacobian.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["jam_deg", "sled_1", "sled_2", "droll_ds1", "droll_ds2", "dpitch_ds1", "dpitch_ds2",
                "doff_ds1", "doff_ds2", "sigma_max", "sigma_min"])
    w.writerows(jac_rows)


# --- heatmaps ---------------------------------------------------------------------------------
def heatmaps(name, title, unit, gs, diverging, fixed_max=None):
    vals = np.concatenate([g[~np.isnan(g)] for g in gs.values()])
    if diverging:
        v = np.percentile(np.abs(vals), 99.5); vmin, vmax, cmap = -v, v, "coolwarm"
    else:
        vmin, vmax, cmap = 0, fixed_max or np.percentile(vals, 99.5), "Blues"
    n = len(gs); cols = 3; rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(3.2 * cols, 3.0 * rows + 0.6), sharex=True, sharey=True,
                             constrained_layout=True)
    for ax, (jam, g) in zip(axes.flat, gs.items()):
        im = ax.imshow(g, origin="lower", extent=[-LIM - .5, LIM + .5, -LIM - .5, LIM + .5],
                       cmap=cmap, vmin=vmin, vmax=vmax, interpolation="nearest")
        ax.set_title(f"jam {jam:g} deg", fontsize=9)
        ax.set_aspect("equal")
        for s in ax.spines.values():
            s.set_visible(False)
    for ax in axes.flat[n:]:
        ax.axis("off")
    for ax in axes[-1, :]:
        ax.set_xlabel("sled_1 (deg)")
    for ax in axes[:, 0]:
        ax.set_ylabel("sled_2 (deg)")
    fig.colorbar(im, ax=axes, shrink=0.6, label=unit)
    clip = "" if fixed_max else "\ncolour clipped at the 99.5th percentile"
    fig.suptitle(f"{title}{clip}", fontsize=10)
    fig.savefig(os.path.join(out, f"{name}.png"))
    plt.close(fig)


for key, title, unit, div in [("roll", "Head roll", "deg", True), ("pitch", "Head pitch", "deg", True),
                              ("yaw", "Head yaw (residual)", "deg", True), ("off", "Off centre", "mm", False),
                              ("off_x", "Off centre, x", "mm", True), ("off_y", "Off centre, y", "mm", True)]:
    heatmaps(key, f"{title}, pass A", unit, by_key(A, A[:, col[key]]), div)
heatmaps("jacobian_sigma_max", "Largest singular value of d(roll, pitch)/d(sled_1, sled_2), pass A", "deg/deg",
         {j: jac[j][0] for j in jams}, False)
heatmaps("jacobian_sigma_min", "Smallest singular value of d(roll, pitch)/d(sled_1, sled_2), pass A", "deg/deg",
         {j: jac[j][1] for j in jams}, False)
pd = np.maximum(np.abs(diff[:, 3]), np.abs(diff[:, 4]))
heatmaps("pass_diff", f"Pass B - pass A, max(|d_roll|, |d_pitch|), largest {pd.max():.3f} deg", "deg",
         by_key(diff, pd), False, fixed_max=0.01)

# --- roll/pitch coverage ------------------------------------------------------------------------
n = len(jams); cols = 3; rows = int(np.ceil(n / cols))
fig, axes = plt.subplots(rows, cols, figsize=(3.2 * cols, 3.0 * rows + 0.6), sharex=True, sharey=True,
                         constrained_layout=True)
vmax = np.percentile(A[:, col["off"]], 99.5)
for ax, jam in zip(axes.flat, jams):
    m = A[:, 0] == jam
    sc = ax.scatter(A[m, col["roll"]], A[m, col["pitch"]], c=A[m, col["off"]], s=2, cmap="Blues", vmin=0, vmax=vmax,
                    linewidths=0)
    ax.set_title(f"jam {jam:g} deg", fontsize=9); ax.set_aspect("equal"); ax.grid(color="#e6e6e6", lw=0.5)
for ax in axes.flat[n:]:
    ax.axis("off")
for ax in axes[-1, :]:
    ax.set_xlabel("roll (deg)")
for ax in axes[:, 0]:
    ax.set_ylabel("pitch (deg)")
fig.colorbar(sc, ax=axes, shrink=0.6, label="off centre (mm)")
fig.suptitle("Roll vs pitch at every sweep point, coloured by off centre, pass A", fontsize=10)
fig.savefig(os.path.join(out, "roll_pitch_coverage.png"))
plt.close(fig)

# --- neutral height -------------------------------------------------------------------------------
_, cal = load("neutral_height.csv")
fig, ax = plt.subplots(figsize=(6, 3.6), constrained_layout=True)
ax.plot(cal[:, 0], cal[:, 1], color="#2a6fdb", lw=2)
ax.set_xlabel("jam_deg (twist at rest, deg)"); ax.set_ylabel("neutral height (mm)")
ax.set_title("Neutral height (sleds 120 deg apart) vs jam angle", fontsize=10)
ax.grid(color="#e6e6e6", lw=0.5)
for s in ("top", "right"):
    ax.spines[s].set_visible(False)
fig.savefig(os.path.join(out, "neutral_height.png"))
plt.close(fig)

# --- max platform tilt (angle from level) per jam, pass A ------------------------------------------
tilt = np.degrees(np.arccos(np.cos(np.radians(A[:, col["roll"]])) * np.cos(np.radians(A[:, col["pitch"]]))))
TILT_BINS = [5, 10, 15]   # deg; mean off centre over points within +-0.5 deg of each tilt
rows = []
for jam in jams:
    m = A[:, 0] == jam
    i = np.flatnonzero(m)[np.argmax(tilt[m])]
    means = []
    for t in TILT_BINS:
        sel = m & (np.abs(tilt - t) <= 0.5)
        means.append(round(A[sel, col["off"]].mean(), 2) if sel.any() else "")
    rows.append([int(jam), round(tilt[i], 2), *A[i, 1:4].astype(int), round(A[i, col["off"]], 2), *means])
with open(os.path.join(out, "max_tilt.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["jam_deg", "max_tilt", "sled_1", "sled_2", "sled_3", "off", *(f"mean_off_at_{t}deg" for t in TILT_BINS)])
    w.writerows(rows)
mt = np.array([[np.nan if v == "" else v for v in r] for r in rows], float)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 5.8), sharex=True, constrained_layout=True)
ax1.plot(mt[:, 0], mt[:, 1], "o-", color="#2a6fdb", lw=2, ms=5)
ax1.set_ylabel("max tilt (deg)\nhigher is better")
# Line style: solid = higher is better, dashed = lower is better.
ax2.plot(mt[:, 0], mt[:, 5], "o--", color="#d35400", lw=2, ms=5, label="at max tilt")
for k, (t, colour) in enumerate(zip(TILT_BINS, ["#fdd0a2", "#fdae6b", "#f16913"])):
    ax2.plot(mt[:, 0], mt[:, 6 + k], "o--", color=colour, lw=1.5, ms=4, label=f"mean at {t} deg tilt")
ax2.legend(fontsize=7.5, frameon=False, loc="lower right")
ax2.set_ylabel("off centre (mm)\nlower is better"); ax2.set_xlabel("jam_deg (twist at rest, deg)")
for ax in (ax1, ax2):
    ax.set_ylim(0, ax.get_ylim()[1] * 1.05)
    ax.grid(color="#e6e6e6", lw=0.5)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
fig.suptitle("Max platform tilt (angle from level) vs jam angle\nall at the sled-gap limit", fontsize=10)
fig.savefig(os.path.join(out, "max_tilt.png"))
plt.close(fig)

# --- trade-off: every factor vs jam angle -----------------------------------------------------------
# Balanced pitch: for each head heading on the platform, the smaller of the largest pitch up and down
# with |roll| <= 1 deg and off <= 10 mm (as in jam_charts.py chart 14); the best heading's value.
heads = np.radians(np.arange(0, 360, 1))
c_h, s_h = np.cos(heads)[:, None], np.sin(heads)[:, None]
cal_h = dict(zip(cal[:, 0], cal[:, 1]))
factors = {k: [] for k in ("max_tilt", "off_at_max_tilt", "mean_off_10", "least_off_10", "balanced_pitch",
                           "sigma_max_p99", "neutral_height")}
for jam in jams:
    m = A[:, 0] == jam
    near10 = m & (np.abs(tilt - 10) <= 0.5)
    rr, pp = np.radians(A[m, col["roll"]]), np.radians(A[m, col["pitch"]])
    r20, r21 = -np.sin(pp), np.cos(pp) * np.sin(rr)
    h_pitch = np.degrees(np.arcsin(np.clip(r20 * c_h + r21 * s_h, -1, 1)))
    h_roll = np.degrees(np.arcsin(np.clip(-r20 * s_h + r21 * c_h, -1, 1)))
    ok = (np.abs(h_roll) <= 1) & (A[m, col["off"]][None, :] <= 10)
    up = np.where(ok, h_pitch, 0).max(axis=1)
    down = np.where(ok, -h_pitch, 0).max(axis=1)
    i = np.flatnonzero(m)[np.argmax(tilt[m])]
    factors["max_tilt"].append(tilt[i])
    factors["off_at_max_tilt"].append(A[i, col["off"]])
    factors["mean_off_10"].append(A[near10, col["off"]].mean())
    factors["least_off_10"].append(A[near10, col["off"]].min())
    factors["balanced_pitch"].append(np.minimum(up, down).max())
    factors["sigma_max_p99"].append(np.nanpercentile(jac[jam][0], 99))
    factors["neutral_height"].append(cal_h[jam])
with open(os.path.join(out, "tradeoff.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["jam_deg", *factors])
    w.writerows([[int(j), *(round(factors[k][n], 3) for k in factors)] for n, j in enumerate(jams)])

# (key, title, unit, higher is better? None = context only)
PANELS = [("max_tilt", "Max platform tilt", "deg", True),
          ("balanced_pitch", "Pitch up and down at the best heading\n(|roll| <= 1 deg, off <= 10 mm)", "deg", True),
          ("least_off_10", "Least off centre at 10 deg tilt", "mm", False),
          ("mean_off_10", "Mean off centre at 10 deg tilt", "mm", False),
          ("off_at_max_tilt", "Off centre at max tilt", "mm", False),
          ("sigma_max_p99", "Swing: d(roll, pitch)/d(sleds), 99th percentile", "deg/deg", False),
          ("neutral_height", "Neutral height (context)", "mm", None)]
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]   # dataviz reference palette, slots 1-6
x = np.array(jams)
fig, axes = plt.subplots(2, 4, figsize=(19, 7.6), constrained_layout=True)
for ax, (key, title, unit, higher) in zip(axes.flat, PANELS):
    v = np.array(factors[key])
    style = "o:" if higher is None else ("o-" if higher else "o--")   # solid higher, dashed lower is better
    ax.plot(x, v, style, color=INK if higher is None else SERIES[[p[0] for p in PANELS].index(key)], lw=2, ms=4)
    ax.set_ylim(0, v.max() * 1.1)
    better = "" if higher is None else ("\nhigher is better" if higher else "\nlower is better")
    ax.set_title(title, fontsize=9); ax.set_ylabel(f"{unit}{better}")
normed = axes.flat[7]
for (key, title, unit, higher), colour in zip(PANELS[:6], SERIES):
    v = np.array(factors[key])
    score = (v - v.min()) / (v.max() - v.min())
    normed.plot(x, score if higher else 1 - score, "o-" if higher else "o--", color=colour, lw=1.6, ms=3,
                label=title.split("\n")[0])
normed.set_ylim(-0.05, 1.05); normed.set_ylabel("1 = best across jam angles")
normed.set_title("All factors, rescaled (no weighting)", fontsize=9)
normed.legend(fontsize=7.5, frameon=False, loc="upper left", bbox_to_anchor=(1.02, 1.0))
for ax in axes.flat:
    ax.axvspan(74, 76, color="#e3e3e3", lw=0, zorder=0)
    ax.set_xlabel("jam_deg (deg)"); ax.set_xticks(x[::2])
    ax.grid(color="#e6e6e6", lw=0.5)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
fig.suptitle("Jam angle trade-off, pass A (grey band: jam 75)\n"
             "solid: higher is better, dashed: lower is better, dotted: context", fontsize=11)
fig.savefig(os.path.join(out, "tradeoff.png"))
plt.close(fig)
print("plots written", flush=True)
