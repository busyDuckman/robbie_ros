# /// script
# requires-python = ">=3.12"
# dependencies = ["matplotlib", "numpy", "pyyaml"]
# ///
"""Chart 14 v3: largest pitch by head heading, graceful and ugly, for both twists (CW and CCW from the top).

Graceful: |roll| <= 1 deg and off centre <= 10 mm. Ugly: no roll or off-centre limit (full sled range).
Reads the jam 75 sweeps from cw_ccw.py. Data only.

Usage (Linux or WSL, from the repo root):
    uv run scripts/ik_simulation/pitch_by_heading_v3.py .output/ik_sweep
"""
import csv, os, sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

src = sys.argv[1]
JAM = 75
INK, MUTED = "#222222", "#666666"
plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.edgecolor": MUTED, "figure.dpi": 130})
with open(os.path.join(src, "sweep_settings.yaml")) as f:
    start = yaml.safe_load(f)["neck_yaml"]["sled"]["start_deg"]

heads = np.radians(np.arange(0, 361, 1))
c, s = np.cos(heads)[:, None], np.sin(heads)[:, None]


def pitch_by_heading(path):
    with open(path) as f:
        r = csv.reader(f); head = next(r)
        A = np.array([[float(x) for x in row] for row in r])
    col = {h: i for i, h in enumerate(head)}
    A = A[A[:, 0] == JAM]
    rr, pp = np.radians(A[:, col["roll"]]), np.radians(A[:, col["pitch"]])
    r20, r21 = -np.sin(pp), np.cos(pp) * np.sin(rr)   # third row of R = Rz(yaw) Ry(pitch) Rx(roll)
    h_pitch = np.degrees(np.arcsin(np.clip(r20 * c + r21 * s, -1, 1)))   # [heading, point]
    h_roll = np.degrees(np.arcsin(np.clip(-r20 * s + r21 * c, -1, 1)))
    graceful = (np.abs(h_roll) <= 1) & (A[:, col["off"]][None, :] <= 10)
    out = {}
    for sign, way in ((1, "up"), (-1, "down")):
        best = np.where(graceful, sign * h_pitch, -np.inf).max(axis=1)
        out["graceful", way] = np.where(np.isfinite(best), np.maximum(best, 0), 0)
        out["ugly", way] = np.maximum((sign * h_pitch).max(axis=1), 0)
    return out


TWISTS = {"CW": pitch_by_heading(os.path.join(src, "cw_ccw", "sweep_cw_pass_a.csv")),
          "CCW": pitch_by_heading(os.path.join(src, "cw_ccw", "sweep_ccw_pass_a.csv"))}
COLOURS = {"graceful": "#2a78d6", "ugly": "#eb6834"}
STYLES = {"CW": dict(ls="-", lw=2.0, alpha=1.0), "CCW": dict(ls="-", lw=2.0, alpha=1.0)}
rmax = max(v.max() for t in TWISTS.values() for v in t.values())
R = np.ceil(rmax) + 1

fig, axes = plt.subplots(2, 2, figsize=(12, 12.4), subplot_kw={"projection": "polar"}, constrained_layout=True)
fig.get_layout_engine().set(h_pad=0.04, hspace=0.03)
for row, (twist, sign) in zip(axes, (("CW", -1), ("CCW", 1))):
    curves = TWISTS[twist]
    for ax, way in zip(row, ("up", "down")):
        # Beyond ugly (the full sled range) is unreachable.
        ax.fill_between(heads, curves["ugly", way], R, color="#d9d9d9", lw=0, zorder=0, label="unreachable")
        for kind in ("graceful", "ugly"):
            ax.plot(heads, curves[kind, way], color=COLOURS[kind], label=kind, **STYLES[twist])
        # Rod top pivots at rest: sled pivot at start_deg, twisted round by the jam (+ CCW, - CW).
        # Orbit angles are 0 = -Y, so heading = orbit - 90.
        # Sled pivots (grey solid) and nod lines (red), halfway round the twist from sled to top pivot.
        for i, d in enumerate(start, 1):
            for off, colour, ls, name in ((0, "#444444", "-", "sled"), (sign * JAM, "#444444", "--", "top"),
                                          (sign * JAM / 2, "#d62728", "-", "nod")):
                a = np.radians(d + off - 90)
                ax.plot([a, a], [0, R], color=colour, lw=1.4 if name == "nod" else 1.0, ls=ls)
                ax.text(a, R * 0.93, f"{name} {i}", fontsize=7, color=colour, ha="center", va="center",
                        bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none"))
        ax.set_ylim(0, R); ax.set_title(f"{twist}: pitch {way} (head looks {way})", fontsize=10, pad=14)
        ax.set_rlabel_position(200); ax.tick_params(axis="y", labelsize=7)
        ax.set_xticks(np.radians(np.arange(0, 360, 30)))
    row[1].legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8, frameon=False)
fig.suptitle("Largest pitch by head orientation on the platform\n"
             "angle = head orientation (deg), distance = pitch (deg)",
             fontsize=14)
out = os.path.join(src, f"{JAM}_jam", "14_pitch_by_heading_v3.png")
fig.savefig(out); plt.close(fig)
print("written", out, flush=True)
