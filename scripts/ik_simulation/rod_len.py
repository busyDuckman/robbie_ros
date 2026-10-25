# /// script
# requires-python = "==3.12.*"  # mujoco 3.12.0 has no wheels for newer Pythons
# dependencies = [
#     "mujoco==3.12.0",  # the version mujoco_vendor bundles for the sim
#     "numpy",
#     "pyyaml",
#     "matplotlib",
# ]
# ///
"""Rod length study: sweep.py's sweep (pass A only) repeated over several rod lengths, plus plots. Data only.

Usage (Linux or WSL, from the repo root):
    uv run scripts/ik_simulation/rod_len.py sweep .output/ik_sweep/rod_len
    uv run scripts/ik_simulation/rod_len.py plots .output/ik_sweep/rod_len

The sweep writes one CSV per (rod length, jam) under runs/, so a re-run skips finished ones.
rod length is pivot to pivot. jam_deg is the twist at rest, as in sweep.py; at a fixed jam_deg a
different rod length gives a different neutral height and different rod-end angles at rest, so those
are plotted too.
"""
import copy, csv, importlib.util, math, os, sys, time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("sweep", os.path.join(HERE, "sweep.py"))
sw = importlib.util.module_from_spec(spec); spec.loader.exec_module(sw)

ROD_LENS = [0.070, 0.078, 0.087, 0.096, 0.105, 0.115]   # m; 0.087 is neck.yaml's measured rod
JAMS = sw.SWEEP_JAMS
CURRENT_LEN, CURRENT_JAM = 0.087, 75
HEADER = ["rod_len_mm", "jam_deg", "sled_1", "sled_2", "sled_3", "roll", "pitch", "yaw", "off_x", "off_y", "off"]


def config(rod_len, jam):
    cfg = copy.deepcopy(sw.BASE)
    cfg["rod"]["length"] = rod_len
    cfg["platform"]["rest_height"] = sw.rest_height(cfg, jam)
    return cfg


class Sim(sw.Sim):
    """sw.Sim, built for a given rod length."""

    def __init__(self, rod_len, jam):
        self.cfg = config(rod_len, jam)
        self.m = sw.mujoco.MjModel.from_xml_string(sw.gen.build(self.cfg))
        self.m.geom_contype[:] = 0      # as sweep.py: MIN_GAP bounds the sweep, not the sim's sled boxes
        self.m.geom_conaffinity[:] = 0
        self.d = sw.mujoco.MjData(self.m)
        self.start = np.radians(self.cfg["sled"]["start_deg"])
        self.cur = np.zeros(3)
        self.d.ctrl[:] = self.start
        for _ in range(2000):
            sw.mujoco.mj_step(self.m, self.d)

    def state(self):
        (roll, pitch, yaw), p = self.head()
        return roll, pitch, yaw, p[2] * 1000


def rod_end_angles(cfg, jam):
    """Rod-end angles at rest (deg): the rod's angle out of the plane square to each rod end's radial screw."""
    rb, rp, L = cfg["sled"]["pivot_radius"], cfg["platform"]["pivot_radius"], cfg["rod"]["length"]
    h = cfg["platform"]["rest_height"]
    a = math.radians(jam)
    ub, ut = np.array([0, -1, 0]), np.array([math.sin(a), -math.cos(a), 0])
    d = rp * ut + [0, 0, h] - rb * ub
    d /= np.linalg.norm(d)
    return abs(math.degrees(math.asin(d @ ub))), abs(math.degrees(math.asin(d @ ut)))


def run(args):
    rod_len, jam, out_dir = args
    path = os.path.join(out_dir, "runs", f"L{rod_len * 1000:.0f}_J{jam}.csv")
    if os.path.exists(path):
        return path, None
    t0 = time.time()
    sim = Sim(rod_len, jam)
    rest0 = sim.state()
    springs = [sim.d.ten_length[i] * 1000 for i in range(3)]
    vals = list(range(-sw.LIM, sw.LIM + 1, sw.STEP))
    rows, max_yaw = [], 0
    for i, s1 in enumerate(vals):
        for s2 in (vals if i % 2 == 0 else vals[::-1]):
            if not sw.feasible(s1, s2):
                continue
            s3 = -s1 - s2
            sim.go([s1, s2, s3])
            (roll, pitch, yaw), p = sim.head()
            max_yaw = max(max_yaw, abs(yaw))
            ox, oy = p[0] * 1000, p[1] * 1000
            rows.append([round(rod_len * 1000, 1), jam, s1, s2, s3, round(roll, 3), round(pitch, 3), round(yaw, 3),
                         round(ox, 3), round(oy, 3), round(math.hypot(ox, oy), 3)])
    sim.go([0, 0, 0]); sim.go([0, 0, 0])
    rest1 = sim.state()
    tmp = path + ".tmp"
    with open(tmp, "w", newline="") as f:
        w = csv.writer(f); w.writerow(HEADER); w.writerows(rows)
    os.replace(tmp, path)
    bottom, top = rod_end_angles(sim.cfg, jam)
    check = [round(rod_len * 1000, 1), jam, round(sim.cfg["platform"]["rest_height"] * 1000, 2),
             round(rest0[3] - (sim.cfg["collar_z"] + sim.cfg["sled"]["pivot_height"]) * 1000, 2),
             *(round(v, 3) for v in rest0[:3]), *(round(v, 3) for v in rest1[:3]),
             round(max(abs(a - b) for a, b in zip(rest0, rest1)), 3), round(max_yaw, 2),
             round(min(springs), 2), round(bottom, 2), round(top, 2), len(rows)]
    print(f"L {rod_len * 1000:.0f} mm, jam {jam}: {len(rows)} points, {time.time() - t0:.0f} s, "
          f"return-to-rest change {check[10]}, max |yaw| {check[11]}", flush=True)
    return path, check


CHECK_HEADER = ["rod_len_mm", "jam_deg", "rest_height_geom_mm", "neutral_height_mm",
                "rest_roll_before", "rest_pitch_before", "rest_yaw_before", "rest_roll_after", "rest_pitch_after",
                "rest_yaw_after", "rest_change_max", "max_abs_yaw", "spring_len_rest_mm", "rod_end_bottom_deg",
                "rod_end_top_deg", "points"]


def cmd_sweep(out_dir):
    os.makedirs(os.path.join(out_dir, "runs"), exist_ok=True)
    jobs = [(L, j, out_dir) for L in ROD_LENS for j in JAMS]
    print(f"{len(jobs)} runs", flush=True)
    checks_path = os.path.join(out_dir, "checks.csv")
    have = {}
    if os.path.exists(checks_path):
        with open(checks_path) as f:
            r = csv.reader(f); next(r)
            have = {(float(row[0]), int(row[1])): row for row in r}
    with Pool(30) as pool:
        for n, (path, check) in enumerate(pool.imap_unordered(run, jobs), 1):
            if check:
                have[(check[0], check[1])] = check
            print(f"{n}/{len(jobs)} done", flush=True)
    with open(checks_path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(CHECK_HEADER)
        w.writerows(sorted(have.values(), key=lambda r: (float(r[0]), int(r[1]))))
    print("sweep written", flush=True)


# ---------------------------------------------------------------------------------------------------
# plots

def cmd_plots(out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    INK, MUTED = "#222222", "#666666"
    plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.edgecolor": MUTED, "figure.dpi": 130})
    lens = [round(L * 1000, 1) for L in ROD_LENS]
    cur_len = round(CURRENT_LEN * 1000, 1)
    N = 2 * sw.LIM + 1
    with open(os.path.join(out_dir, "checks.csv")) as f:
        r = csv.reader(f); next(r)
        checks = {(float(row[0]), int(row[1])): [float(v) for v in row] for row in r}

    heads = np.radians(np.arange(0, 360, 1))
    c_h, s_h = np.cos(heads)[:, None], np.sin(heads)[:, None]

    def load(L, jam):
        with open(os.path.join(out_dir, "runs", f"L{L:.0f}_J{jam}.csv")) as f:
            r = csv.reader(f); next(r)
            return np.array([[float(x) for x in row] for row in r])

    def heading_pitch(a):
        rr, pp = np.radians(a[:, 5]), np.radians(a[:, 6])
        r20, r21 = -np.sin(pp), np.cos(pp) * np.sin(rr)
        return (np.degrees(np.arcsin(np.clip(r20 * c_h + r21 * s_h, -1, 1))),
                np.degrees(np.arcsin(np.clip(-r20 * s_h + r21 * c_h, -1, 1))))

    def sigma_p99(a):
        g = {k: np.full((N, N), np.nan) for k in (5, 6)}
        for k in g:
            g[k][(a[:, 3] + sw.LIM).astype(int), (a[:, 2] + sw.LIM).astype(int)] = a[:, k]
        d = {}
        for k in g:
            d1 = np.full((N, N), np.nan); d2 = np.full((N, N), np.nan)
            d1[:, 1:-1] = (g[k][:, 2:] - g[k][:, :-2]) / 2; d2[1:-1, :] = (g[k][2:, :] - g[k][:-2, :]) / 2
            d[k] = (d1, d2)
        J = np.stack([np.stack(d[5], -1), np.stack(d[6], -1)], -2)
        ok = ~np.isnan(J).any(axis=(-1, -2))
        return np.percentile(np.linalg.svd(J[ok], compute_uv=False)[:, 0], 99)

    KEYS = ["max_tilt", "balanced_pitch", "least_off_10", "mean_off_10", "off_at_max_tilt", "sigma_max_p99",
            "neutral_height", "rod_end_bottom", "rod_end_top"]
    F = {k: np.full((len(lens), len(JAMS)), np.nan) for k in KEYS}
    polar = {}
    for li, L in enumerate(lens):
        for ji, jam in enumerate(JAMS):
            a = load(L, jam)
            tilt = np.degrees(np.arccos(np.cos(np.radians(a[:, 5])) * np.cos(np.radians(a[:, 6]))))
            near10 = np.abs(tilt - 10) <= 0.5
            hp, hr = heading_pitch(a)
            ok = (np.abs(hr) <= 1) & (a[:, 10][None, :] <= 10)
            up, down = np.where(ok, hp, 0).max(axis=1), np.where(ok, -hp, 0).max(axis=1)
            if jam == CURRENT_JAM:
                polar[L] = (up, down)
            i = np.argmax(tilt)
            ck = checks[(L, jam)]
            vals = dict(max_tilt=tilt[i], balanced_pitch=np.minimum(up, down).max(),
                        least_off_10=a[near10, 10].min() if near10.any() else np.nan,
                        mean_off_10=a[near10, 10].mean() if near10.any() else np.nan,
                        off_at_max_tilt=a[i, 10], sigma_max_p99=sigma_p99(a), neutral_height=ck[3],
                        rod_end_bottom=ck[13], rod_end_top=ck[14])
            for k, v in vals.items():
                F[k][li, ji] = v
            print(f"metrics L {L:g} jam {jam}", flush=True)

    with open(os.path.join(out_dir, "metrics.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["rod_len_mm", "jam_deg", *KEYS])
        for li, L in enumerate(lens):
            for ji, jam in enumerate(JAMS):
                w.writerow([L, jam, *(round(F[k][li, ji], 3) for k in KEYS)])

    # (key, title, unit, higher is better? None = context only)
    PANELS = [("max_tilt", "Max platform tilt", "deg", True),
              ("balanced_pitch", "Pitch up and down at the best heading\n(|roll| <= 1 deg, off <= 10 mm)", "deg", True),
              ("least_off_10", "Least off centre at 10 deg tilt", "mm", False),
              ("mean_off_10", "Mean off centre at 10 deg tilt", "mm", False),
              ("off_at_max_tilt", "Off centre at max tilt", "mm", False),
              ("sigma_max_p99", "Swing: d(roll, pitch)/d(sleds), 99th percentile", "deg/deg", False),
              ("neutral_height", "Neutral height (context)", "mm", None)]
    SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]   # dataviz reference palette
    STYLE = {True: "o-", False: "o--", None: "o:"}                                 # solid higher, dashed lower is better
    LEN_COLOURS = plt.get_cmap("Blues")(np.linspace(0.35, 1.0, len(lens)))
    cur_li, cur_ji = lens.index(cur_len), JAMS.index(CURRENT_JAM)

    def tidy(ax):
        ax.grid(color="#e6e6e6", lw=0.5)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)

    def better(higher):
        return "" if higher is None else ("\nhigher is better" if higher else "\nlower is better")

    # 1. Every factor vs rod length, at jam 75 (same layout as ../tradeoff.png).
    fig, axes = plt.subplots(2, 4, figsize=(19, 7.6), constrained_layout=True)
    x = np.array(lens)
    for n, (ax, (key, title, unit, higher)) in enumerate(zip(axes.flat, PANELS)):
        v = F[key][:, cur_ji]
        ax.plot(x, v, STYLE[higher], color=INK if higher is None else SERIES[n], lw=2, ms=4)
        ax.set_ylim(0, np.nanmax(v) * 1.1)
        ax.set_title(title, fontsize=9); ax.set_ylabel(f"{unit}{better(higher)}")
    normed = axes.flat[7]
    for (key, title, unit, higher), colour in zip(PANELS[:6], SERIES):
        v = F[key][:, cur_ji]
        s = (v - np.nanmin(v)) / (np.nanmax(v) - np.nanmin(v))
        normed.plot(x, s if higher else 1 - s, STYLE[higher], color=colour, lw=1.6, ms=3, label=title.split("\n")[0])
    normed.set_ylim(-0.05, 1.05); normed.set_ylabel("1 = best across rod lengths")
    normed.set_title("All factors, rescaled (no weighting)", fontsize=9)
    normed.legend(fontsize=7.5, frameon=False, loc="upper left", bbox_to_anchor=(1.02, 1.0))
    for ax in axes.flat:
        ax.axvspan(cur_len - 1, cur_len + 1, color="#e3e3e3", lw=0, zorder=0)
        ax.set_xlabel("rod length, pivot to pivot (mm)"); ax.set_xticks(x)
        tidy(ax)
    fig.suptitle(f"Rod length trade-off at jam {CURRENT_JAM} (grey band: current rod, {cur_len:g} mm)\n"
                 "solid: higher is better, dashed: lower is better, dotted: context", fontsize=11)
    fig.savefig(os.path.join(out_dir, "01_tradeoff_vs_rod_len_jam75.png")); plt.close(fig)

    # 2. Every factor vs jam, one line per rod length (same panels as ../tradeoff.png).
    fig, axes = plt.subplots(2, 4, figsize=(19, 7.6), constrained_layout=True)
    xj = np.array(JAMS)
    for ax, (key, title, unit, higher) in zip(axes.flat, PANELS):
        for li, L in enumerate(lens):
            ax.plot(xj, F[key][li], STYLE[higher], color=LEN_COLOURS[li], lw=2.4 if L == cur_len else 1.4, ms=3,
                    label=f"{L:g} mm" + (" (current)" if L == cur_len else ""))
        ax.set_ylim(0, np.nanmax(F[key]) * 1.1)
        ax.set_title(title, fontsize=9); ax.set_ylabel(f"{unit}{better(higher)}")
    ax = axes.flat[7]
    for li, L in enumerate(lens):
        ax.plot(xj, F["rod_end_bottom"][li], "o:", color=LEN_COLOURS[li], lw=1.4, ms=3)
    ax.set_title("Rod-end angle at rest, sled end (context)", fontsize=9); ax.set_ylabel("deg")
    ax.set_ylim(0, np.nanmax(F["rod_end_bottom"]) * 1.1)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend([plt.Line2D([], [], color=h.get_color(), lw=h.get_linewidth()) for h in handles], labels,
               title="rod length", fontsize=8, title_fontsize=8, frameon=False, loc="outside lower center", ncol=6)
    for ax in axes.flat:
        ax.axvspan(CURRENT_JAM - 1, CURRENT_JAM + 1, color="#e3e3e3", lw=0, zorder=0)
        ax.set_xlabel("jam_deg (deg)"); ax.set_xticks(xj[::2])
        tidy(ax)
    fig.suptitle(f"Jam angle trade-off for each rod length (grey band: jam {CURRENT_JAM}; thick line: current rod)\n"
                 "solid: higher is better, dashed: lower is better, dotted: context", fontsize=11)
    fig.savefig(os.path.join(out_dir, "02_tradeoff_vs_jam_by_rod_len.png")); plt.close(fig)

    # 3. Heatmaps over (jam, rod length).
    fig, axes = plt.subplots(2, 4, figsize=(19, 9.0), constrained_layout=True)
    fig.get_layout_engine().set(h_pad=0.25, hspace=0.06)
    for ax, (key, title, unit, higher) in zip(axes.flat, PANELS + [("rod_end_bottom", "Rod-end angle at rest, sled end (context)", "deg", None)]):
        v = F[key]
        im = ax.imshow(v, origin="lower", aspect="auto", cmap="Blues",
                       extent=[JAMS[0] - 2.5, JAMS[-1] + 2.5, -0.5, len(lens) - 0.5])
        for li in range(len(lens)):
            for ji, jam in enumerate(JAMS):
                if np.isfinite(v[li, ji]):
                    ax.text(jam, li, f"{v[li, ji]:.1f}", ha="center", va="center", fontsize=6.5,
                            color="white" if v[li, ji] > np.nanmin(v) + 0.6 * (np.nanmax(v) - np.nanmin(v)) else INK)
        ax.add_patch(plt.Rectangle((CURRENT_JAM - 2.5, cur_li - 0.5), 5, 1, fill=False, ec="#d35400", lw=2))
        ax.set_yticks(range(len(lens))); ax.set_yticklabels([f"{L:g}" for L in lens])
        ax.set_xticks(JAMS[::2]); ax.set_xlabel("jam_deg (deg)"); ax.set_ylabel("rod length (mm)")
        ax.set_title(f"{title.splitlines()[0]} ({unit}){better(higher).replace(chr(10), ', ')}", fontsize=9)
        fig.colorbar(im, ax=ax, shrink=0.85)
    fig.suptitle(f"Every factor over jam angle and rod length (orange box: current, jam {CURRENT_JAM}, {cur_len:g} mm)",
                 fontsize=11)
    fig.savefig(os.path.join(out_dir, "03_heatmaps_jam_vs_rod_len.png")); plt.close(fig)

    # 4. Geometry at rest: neutral height and rod-end angles vs jam, per rod length.
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4), constrained_layout=True)
    for ax, key, title in zip(axes, ("neutral_height", "rod_end_bottom", "rod_end_top"),
                              ("Neutral height", "Rod-end angle at rest, sled end",
                               "Rod-end angle at rest, platform end")):
        for li, L in enumerate(lens):
            ax.plot(xj, F[key][li], "o:", color=LEN_COLOURS[li], lw=2.4 if L == cur_len else 1.4, ms=3,
                    label=f"{L:g} mm" + (" (current)" if L == cur_len else ""))
        ax.set_title(title, fontsize=9); ax.set_ylabel("mm" if key == "neutral_height" else "deg")
        ax.set_xlabel("jam_deg (deg)"); ax.set_ylim(0, np.nanmax(F[key]) * 1.1)
        ax.axvspan(CURRENT_JAM - 1, CURRENT_JAM + 1, color="#e3e3e3", lw=0, zorder=0)
        tidy(ax)
    axes[2].legend(title="rod length", fontsize=7.5, title_fontsize=8, frameon=False, loc="upper left",
                   bbox_to_anchor=(1.02, 1.0))
    fig.suptitle("Geometry at rest for each rod length (rod-end angle: the rod's angle out of the plane square to "
                 "the rod end's screw)", fontsize=10)
    fig.savefig(os.path.join(out_dir, "04_rest_geometry.png")); plt.close(fig)

    # 5. Pitch by head heading at jam 75, one line per rod length (limits as chart 14 v1, middle level).
    fig, axes = plt.subplots(1, 2, figsize=(13, 6.6), subplot_kw={"projection": "polar"}, constrained_layout=True)
    rmax = max(max(u.max(), d.max()) for u, d in polar.values())
    for ax, k, label in zip(axes, (0, 1), ("Pitch up", "Pitch down")):
        for li, L in enumerate(lens):
            ax.plot(heads, polar[L][k], color=LEN_COLOURS[li], lw=2.4 if L == cur_len else 1.4,
                    label=f"{L:g} mm" + (" (current)" if L == cur_len else ""))
        ax.set_ylim(0, np.ceil(rmax) + 1); ax.set_title(label, fontsize=10, pad=14)
        ax.set_rlabel_position(200); ax.tick_params(axis="y", labelsize=7)
        ax.set_xticks(np.radians(np.arange(0, 360, 30)))
    axes[1].legend(title="rod length", fontsize=8, title_fontsize=8, frameon=False, loc="upper left",
                   bbox_to_anchor=(1.05, 1.0))
    fig.suptitle(f"Largest pitch (deg) by head heading, jam {CURRENT_JAM}, |roll| <= 1 deg and off <= 10 mm\n"
                 "angle: heading on the platform, 0 = +X", fontsize=10)
    fig.savefig(os.path.join(out_dir, "05_pitch_by_heading_by_rod_len.png")); plt.close(fig)

    # 6. Same rod ends: every factor vs rod length at a fixed rod-end angle at rest (sled end, the larger
    # of the two, so the one that binds first). Each rod length's factors are interpolated linearly
    # across the swept jam angles to the jam that gives that rod-end angle.
    cur_angle = F["rod_end_bottom"][cur_li, cur_ji]
    ANGLES = [20.0, round(cur_angle, 1), 25.0]
    ANGLE_COLOURS = ["#9ecae1", "#4292c6", "#08306b"]

    def at_angle(key, li, angle):
        return np.interp(angle, F["rod_end_bottom"][li], F[key][li])   # rod-end angle rises with jam

    rows = []
    for angle in ANGLES:
        for li, L in enumerate(lens):
            rows.append([angle, L, round(at_angle("neutral_height", li, angle), 2),
                         round(np.interp(angle, F["rod_end_bottom"][li], JAMS), 2),
                         *(round(at_angle(k, li, angle), 3) for k, *_ in PANELS[:6])])
    with open(os.path.join(out_dir, "same_rod_ends.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["rod_end_angle_deg", "rod_len_mm", "neutral_height", "jam_deg", *(k for k, *_ in PANELS[:6])])
        w.writerows(rows)
    fig, axes = plt.subplots(2, 4, figsize=(19, 7.8), constrained_layout=True)
    extra = [("jam_deg", "Jam (twist at rest) that gives that angle (context)", "deg", None)]
    for n, (ax, (key, title, unit, higher)) in enumerate(zip(axes.flat, PANELS + extra)):
        for angle, colour in zip(ANGLES, ANGLE_COLOURS):
            if key == "jam_deg":
                v = [np.interp(angle, F["rod_end_bottom"][li], JAMS) for li in range(len(lens))]
            else:
                v = [at_angle(key, li, angle) for li in range(len(lens))]
            ax.plot(x, v, STYLE[higher], color=colour, lw=2.4 if angle == ANGLES[1] else 1.4, ms=3,
                    label=f"{angle:g} deg" + (" (current geometry)" if angle == ANGLES[1] else ""))
        ax.set_ylim(0, ax.get_ylim()[1] * 1.05)
        ax.set_title(title, fontsize=9); ax.set_ylabel(f"{unit}{better(higher)}")
        ax.axvspan(cur_len - 1, cur_len + 1, color="#e3e3e3", lw=0, zorder=0)
        ax.set_xlabel("rod length, pivot to pivot (mm)"); ax.set_xticks(x)
        tidy(ax)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend([plt.Line2D([], [], color=h.get_color(), lw=h.get_linewidth()) for h in handles], labels,
               title="rod-end angle at rest, sled end", fontsize=8, title_fontsize=8, frameon=False,
               loc="outside lower center", ncol=3)
    fig.suptitle("Same rod ends, different rod length: factors at a fixed rod-end angle at rest (grey band: current rod)\n"
                 "solid: higher is better, dashed: lower is better, dotted: context", fontsize=11)
    fig.savefig(os.path.join(out_dir, "06_same_rod_ends_vs_rod_len.png")); plt.close(fig)

    # 7. Summary of 03: each scored factor rescaled over the whole (jam, rod length) grid, 1 = best, on one
    # shared colour scale. No weighting, no combined score.
    fig, axes = plt.subplots(2, 3, figsize=(15, 9.0), constrained_layout=True)
    fig.get_layout_engine().set(h_pad=0.25, hspace=0.06)
    for ax, (key, title, unit, higher) in zip(axes.flat, PANELS[:6]):
        v = F[key]
        s = (v - np.nanmin(v)) / (np.nanmax(v) - np.nanmin(v))
        s = s if higher else 1 - s
        im = ax.imshow(s, origin="lower", aspect="auto", cmap="Blues", vmin=0, vmax=1,
                       extent=[JAMS[0] - 2.5, JAMS[-1] + 2.5, -0.5, len(lens) - 0.5])
        for li in range(len(lens)):
            for ji, jam in enumerate(JAMS):
                if np.isfinite(s[li, ji]):
                    ax.text(jam, li, f"{s[li, ji]:.2f}", ha="center", va="center", fontsize=6.5,
                            color="white" if s[li, ji] > 0.6 else INK)
        ax.add_patch(plt.Rectangle((CURRENT_JAM - 2.5, cur_li - 0.5), 5, 1, fill=False, ec="#d35400", lw=2))
        ax.set_yticks(range(len(lens))); ax.set_yticklabels([f"{L:g}" for L in lens])
        ax.set_xticks(JAMS[::2]); ax.set_xlabel("jam_deg (deg)"); ax.set_ylabel("rod length (mm)")
        ax.set_title(f"{title.splitlines()[0]}, {'higher' if higher else 'lower'} is better", fontsize=9)
    fig.colorbar(im, ax=axes, shrink=0.6, label="1 = best over the grid")
    fig.suptitle(f"All factors from 03, rescaled so 1 = best over the grid (no weighting)\n"
                 f"dark is good for every panel; orange box: current, jam {CURRENT_JAM}, {cur_len:g} mm", fontsize=11)
    fig.savefig(os.path.join(out_dir, "07_rescaled_heatmaps_jam_vs_rod_len.png")); plt.close(fig)
    print("plots written", flush=True)


if __name__ == "__main__":
    {"sweep": cmd_sweep, "plots": cmd_plots}[sys.argv[1]](sys.argv[2])
