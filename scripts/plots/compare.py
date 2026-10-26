"""ik_sweep: comparison charts over jam angle and rod length, built on neck.yaml. Data only.

Ported from scripts/ik_simulation/plots.py and rod_len.py.
"""
import numpy as np
import matplotlib.pyplot as plt

import common
from common import COLS, PANELS, SERIES, STYLE, INK, tidy, better, metrics, graceful_ugly, num

F_ = "ik_sweep"


def run(book, D):
    jams, cur_jam, rods, cur_rod, LIM = D.jams, D.cur_jam, D.rods, D.cur_rod, D.lim
    N = 2 * LIM + 1
    cur_li, cur_ji = rods.index(cur_rod), jams.index(cur_jam)
    jl = lambda j: f"{j:g}" + (" (neck.yaml)" if j == cur_jam else "")
    A = np.concatenate([D.runs[cur_rod, j, "a"][0] for j in jams])
    B = np.concatenate([D.runs[cur_rod, j, "b"][0] for j in jams])

    # ---------------------------------------------------------------- metrics over (rod, jam)
    KEYS = ["max_tilt", "balanced_pitch", "least_off_10", "mean_off_10", "off_at_max_tilt", "sigma_max_p99",
            "neutral_height", "rod_end_bottom", "rod_end_top"]
    F = {k: np.full((len(rods), len(jams)), np.nan) for k in KEYS}
    GU = {}
    for li, L in enumerate(rods):
        for ji, j in enumerate(jams):
            a, ck = D.runs[L, j, "a"]
            for k, v in metrics(a, ck, LIM).items():
                F[k][li, ji] = v
            GU[L, j] = graceful_ugly(a)
    book.csv(F_, "metrics.csv", ["rod_len_mm", "jam_deg", *KEYS],
             [[L, j, *(round(F[k][li, ji], 3) for k in KEYS)] for li, L in enumerate(rods) for ji, j in enumerate(jams)])
    xj = np.array(jams)

    def band_jam(ax):
        ax.axvspan(cur_jam - 1, cur_jam + 1, color="#e3e3e3", lw=0, zorder=0)

    # ---------------------------------------------------------------- 1 trade-off vs jam
    fig, axes = plt.subplots(2, 4, figsize=(19, 7.6), constrained_layout=True)
    for n, (ax, (key, title, unit, higher)) in enumerate(zip(axes.flat, PANELS)):
        v = F[key][cur_li]
        ax.plot(xj, v, STYLE[higher], color=INK if higher is None else SERIES[n], lw=2, ms=4)
        ax.set_ylim(0, np.nanmax(v) * 1.1); ax.set_title(title, fontsize=9); ax.set_ylabel(f"{unit}{better(higher)}")
    normed = axes.flat[7]
    for (key, title, unit, higher), colour in zip(PANELS[:6], SERIES):
        v = F[key][cur_li]; s = (v - np.nanmin(v)) / (np.nanmax(v) - np.nanmin(v))
        normed.plot(xj, s if higher else 1 - s, STYLE[higher], color=colour, lw=1.6, ms=3, label=title.split("\n")[0])
    normed.set_ylim(-0.05, 1.05); normed.set_ylabel("1 = best across jam angles")
    normed.set_title("All factors, rescaled (no weighting)", fontsize=9)
    normed.legend(fontsize=7.5, frameon=False, loc="upper left", bbox_to_anchor=(1.02, 1.0))
    for ax in axes.flat:
        band_jam(ax); ax.set_xlabel("jam_deg (deg)"); tidy(ax)
    book.save(fig, F_, "tradeoff_vs_jam", "Trade-off vs jam angle",
              f"Each factor against jam angle at the neck.yaml rod length ({cur_rod:g} mm), from pass A. The last panel "
              "rescales the six scored factors so 1 is the best value across the jam angles, with no weighting and no "
              "combined score. Grey band: neck.yaml's jam.",
              title=f"Jam angle trade-off, rod {cur_rod:g} mm (grey band: neck.yaml jam {cur_jam:g})\n"
                    "solid: higher is better, dashed: lower is better, dotted: context")
    book.table(F_, "Trade-off vs jam angle (table)", ["jam (deg)", "max tilt (deg)", "balanced pitch (deg)",
               "least off at 10 deg (mm)", "mean off at 10 deg (mm)", "off at max tilt (mm)", "swing (deg/deg)",
               "neutral height (mm)"],
               [[jl(j), *(num(F[k][cur_li, ji]) for k, *_ in PANELS)] for ji, j in enumerate(jams)],
               f"The values plotted in the trade-off chart, rod {cur_rod:g} mm.")

    # ---------------------------------------------------------------- 2 max tilt vs jam
    t_all = {}
    rows = []
    for j in jams:
        a = D.runs[cur_rod, j, "a"][0]
        t = np.degrees(np.arccos(np.cos(np.radians(a[:, COLS["roll"]])) * np.cos(np.radians(a[:, COLS["pitch"]]))))
        i = int(np.argmax(t))
        means = [a[np.abs(t - b) <= 0.5, COLS["off"]].mean() if (np.abs(t - b) <= 0.5).any() else np.nan for b in (5, 10, 15)]
        rows.append([j, t[i], *a[i, 1:4], a[i, COLS["off"]], *means])
    mt = np.array(rows, float)
    book.csv(F_, "max_tilt.csv", ["jam_deg", "max_tilt", "sled_1", "sled_2", "sled_3", "off", "mean_off_at_5deg",
                                  "mean_off_at_10deg", "mean_off_at_15deg"], np.round(mt, 3).tolist())
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 5.8), sharex=True, constrained_layout=True)
    ax1.plot(mt[:, 0], mt[:, 1], "o-", color="#2a6fdb", lw=2, ms=5); ax1.set_ylabel("max tilt (deg)\nhigher is better")
    ax2.plot(mt[:, 0], mt[:, 5], "o--", color="#d35400", lw=2, ms=5, label="at max tilt")
    for k, (b, colour) in enumerate(zip((5, 10, 15), ["#fdd0a2", "#fdae6b", "#f16913"])):
        ax2.plot(mt[:, 0], mt[:, 6 + k], "o--", color=colour, lw=1.5, ms=4, label=f"mean at {b} deg tilt")
    ax2.legend(fontsize=7.5, frameon=False, loc="lower right")
    ax2.set_ylabel("off centre (mm)\nlower is better"); ax2.set_xlabel("jam_deg (twist at rest, deg)")
    for ax in (ax1, ax2):
        ax.set_ylim(0, ax.get_ylim()[1] * 1.05); band_jam(ax); tidy(ax)
    book.save(fig, F_, "max_tilt_vs_jam", "Max platform tilt vs jam angle",
              "Top: the largest platform tilt (angle from level) reached anywhere in the sweep. Bottom: off centre at that "
              "point, and the mean off centre over all sweep points within 0.5 deg of 5, 10 and 15 deg tilt.",
              title="Max platform tilt (angle from level) vs jam angle")

    # ---------------------------------------------------------------- 3 neutral height vs jam
    cal = D.cal
    fig, ax = plt.subplots(figsize=(6, 3.6), constrained_layout=True)
    ax.plot(cal[:, 0], cal[:, 1], ":", color=INK, lw=2)
    ax.set_xlabel("jam_deg (twist at rest, deg)"); ax.set_ylabel("neutral height (mm)"); ax.set_ylim(0, cal[:, 1].max() * 1.1)
    band_jam(ax); tidy(ax)
    book.save(fig, F_, "neutral_height_vs_jam", "Neutral height vs jam angle",
              f"Platform height above the sled pivots with the sleds 120 deg apart, for each jam angle (1 deg steps), rod "
              f"{cur_rod:g} mm. Measured in the sim at rest.",
              title="Neutral height (sleds 120 deg apart) vs jam angle")

    # ---------------------------------------------------------------- 4-12 heatmaps over sled space, per jam
    def grid(data, key_or_vals):
        gs = {}
        for j in jams:
            m = data[:, 0] == j
            v = data[m, COLS[key_or_vals]] if isinstance(key_or_vals, str) else key_or_vals[m]
            g = np.full((N, N), np.nan)
            g[(data[m, 2] + LIM).astype(int), (data[m, 1] + LIM).astype(int)] = v
            gs[j] = g
        return gs

    def heatmaps(name, heading, about, title, unit, gs, diverging, fixed_max=None):
        vals = np.concatenate([g[~np.isnan(g)] for g in gs.values()])
        if diverging:
            v = np.percentile(np.abs(vals), 99.5); vmin, vmax, cmap = -v, v, "coolwarm"
        else:
            vmin, vmax, cmap = 0, fixed_max or np.percentile(vals, 99.5), "Blues"
        n = len(gs); cols = 4; rows_ = int(np.ceil(n / cols))
        fig, axes = plt.subplots(rows_, cols, figsize=(3.2 * cols, 3.0 * rows_ + 0.6), sharex=True, sharey=True, squeeze=False,
                                 constrained_layout=True)
        for ax, (j, g) in zip(axes.flat, gs.items()):
            im = ax.imshow(g, origin="lower", extent=[-LIM - .5, LIM + .5, -LIM - .5, LIM + .5], cmap=cmap, vmin=vmin,
                           vmax=vmax, interpolation="nearest")
            ax.set_title(f"jam {jl(j)}", fontsize=9); ax.set_aspect("equal")
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
        book.save(fig, F_, name, heading, about, title=f"{title}{clip}")

    sled_about = ("One panel per jam angle, over the sled offsets sled_1 and sled_2 (sled_3 = -(sled_1 + sled_2)), "
                  "on a shared colour scale. ")
    for key, title, unit, div, what in [
            ("roll", "Head roll", "deg", True, "Head roll (world frame, ZYX Euler)."),
            ("pitch", "Head pitch", "deg", True, "Head pitch (world frame, ZYX Euler)."),
            ("yaw", "Head yaw (residual)", "deg", True, "Head yaw that comes with each sled setting."),
            ("off", "Off centre", "mm", False, "Distance of the platform centre from the neck axis."),
            ("off_x", "Off centre, x", "mm", True, "x component of off centre."),
            ("off_y", "Off centre, y", "mm", True, "y component of off centre.")]:
        heatmaps(f"{key}_by_jam", f"{title} by jam angle", sled_about + what, f"{title}, pass A", unit, grid(A, key), div)

    # Jacobian of (roll, pitch) wrt (sled_1, sled_2), central differences, per jam.
    smax_g, smin_g = {}, {}
    Gr, Gp = grid(A, "roll"), grid(A, "pitch")
    for j in jams:
        g = Gr[j], Gp[j]
        d = []
        for gg in g:
            d1 = np.full((N, N), np.nan); d2 = np.full((N, N), np.nan)
            d1[:, 1:-1] = (gg[:, 2:] - gg[:, :-2]) / 2; d2[1:-1, :] = (gg[2:, :] - gg[:-2, :]) / 2
            d.append((d1, d2))
        J = np.stack([np.stack(d[0], -1), np.stack(d[1], -1)], -2)
        ok = ~np.isnan(J).any(axis=(-1, -2))
        sv = np.full((N, N, 2), np.nan); sv[ok] = np.linalg.svd(J[ok], compute_uv=False)
        smax_g[j], smin_g[j] = sv[..., 0], sv[..., 1]
    heatmaps("sigma_max_by_jam", "Largest Jacobian singular value by jam angle",
             sled_about + "Largest singular value of d(roll, pitch)/d(sled_1, sled_2), central differences on the 1 deg grid.",
             "Largest singular value of d(roll, pitch)/d(sled_1, sled_2), pass A", "deg/deg", smax_g, False)
    heatmaps("sigma_min_by_jam", "Smallest Jacobian singular value by jam angle",
             sled_about + "Smallest singular value of d(roll, pitch)/d(sled_1, sled_2).",
             "Smallest singular value of d(roll, pitch)/d(sled_1, sled_2), pass A", "deg/deg", smin_g, False)
    keyA = {(r[0], r[1], r[2]): r for r in A}
    diff = np.array([[r[0], r[1], r[2], *(r[COLS[k]] - keyA[r[0], r[1], r[2]][COLS[k]] for k in ("roll", "pitch", "yaw", "off"))]
                     for r in B])
    book.csv(F_, "pass_diff.csv", ["jam_deg", "sled_1", "sled_2", "d_roll", "d_pitch", "d_yaw", "d_off"], np.round(diff, 3).tolist())
    pdm = np.maximum(np.abs(diff[:, 3]), np.abs(diff[:, 4]))
    heatmaps("pass_diff_by_jam", "Pass B minus pass A",
             sled_about + "Pass A walks the grid with sled_1 as the outer loop, pass B with sled_2. Each point shows "
             "max(|d roll|, |d pitch|) between the two passes, on a fixed 0 to 0.01 deg scale.",
             f"Pass B - pass A, max(|d_roll|, |d_pitch|), largest {pdm.max():.3f} deg", "deg", grid(diff, pdm), False, fixed_max=0.01)

    # ---------------------------------------------------------------- 13, 14 roll/pitch coverage
    cov_jams = [j for j in jams if j not in (min(jams), max(jams))]   # end jams dropped for presentation
    cov = {"jam": [(f"jam {jl(j)}", D.runs[cur_rod, j, "a"][0]) for j in cov_jams],
           "rod": [(f"rod {L:g} mm" + (" (neck.yaml)" if L == cur_rod else ""), D.runs[L, cur_jam, "a"][0]) for L in rods]}
    vmax = np.percentile(np.concatenate([a for p in cov.values() for _, a in p])[:, COLS["off"]], 99.5)
    coverage_about = ("Every pass A sweep point plotted at its head roll and pitch (world frame), coloured by off centre. "
                      "Blue: off centre within the graceful limit (10 mm), orange: beyond it; the bluer points are drawn on top. "
                      "Both coverage charts share one colour scale.")
    for kind, heading, about, title in (
            ("jam", "Roll/pitch coverage by jam angle",
             f"One panel per jam angle at rod {cur_rod:g} mm ({cov_jams[0]:g} to {cov_jams[-1]:g} deg). ",
             f"Roll vs pitch at every sweep point by jam angle, rod {cur_rod:g} mm, pass A"),
            ("rod", "Roll/pitch coverage by rod length",
             f"One panel per rod length at the neck.yaml jam ({cur_jam:g} deg). ",
             f"Roll vs pitch at every sweep point by rod length, jam {cur_jam:g} deg, pass A")):
        panels = cov[kind]
        n = len(panels); cols = 3 if n in (5, 6, 9) else 4; rows_ = int(np.ceil(n / cols))
        fig, axes = plt.subplots(rows_, cols, figsize=(3.2 * cols, 3.0 * rows_ + 0.6), sharex=True, sharey=True, squeeze=False,
                                 constrained_layout=True)
        for ax, (label, a) in zip(axes.flat, panels):
            sc = common.off_scatter(ax, a, vmax)
            ax.set_title(label, fontsize=9); ax.set_aspect("equal"); ax.grid(color="#e6e6e6", lw=0.5)
        for ax in axes.flat[n:]:
            ax.axis("off")
        for ax in axes[-1, :]:
            ax.set_xlabel("roll (deg)")
        for ax in axes[:, 0]:
            ax.set_ylabel("pitch (deg)")
        common.off_colorbar(fig, sc, axes, shrink=0.8, label="off centre (mm)")
        book.save(fig, F_, f"roll_pitch_coverage_by_{kind}", heading, about + coverage_about, title=title)

    # ================================================================= rod length
    lens = rods
    LEN_COLOURS = plt.get_cmap("Blues")(np.linspace(0.35, 1.0, len(lens)))
    x = np.array(lens)
    ll = lambda L: f"{L:g} mm" + (" (neck.yaml)" if L == cur_rod else "")

    def band_len(ax):
        ax.axvspan(cur_rod - 1, cur_rod + 1, color="#e3e3e3", lw=0, zorder=0)

    # 14: every factor vs rod length at neck.yaml's jam
    fig, axes = plt.subplots(2, 4, figsize=(19, 7.6), constrained_layout=True)
    for n, (ax, (key, title, unit, higher)) in enumerate(zip(axes.flat, PANELS)):
        v = F[key][:, cur_ji]
        ax.plot(x, v, STYLE[higher], color=INK if higher is None else SERIES[n], lw=2, ms=4)
        ax.set_ylim(0, np.nanmax(v) * 1.1); ax.set_title(title, fontsize=9); ax.set_ylabel(f"{unit}{better(higher)}")
    normed = axes.flat[7]
    for (key, title, unit, higher), colour in zip(PANELS[:6], SERIES):
        v = F[key][:, cur_ji]; s = (v - np.nanmin(v)) / (np.nanmax(v) - np.nanmin(v))
        normed.plot(x, s if higher else 1 - s, STYLE[higher], color=colour, lw=1.6, ms=3, label=title.split("\n")[0])
    normed.set_ylim(-0.05, 1.05); normed.set_ylabel("1 = best across rod lengths")
    normed.set_title("All factors, rescaled (no weighting)", fontsize=9)
    normed.legend(fontsize=7.5, frameon=False, loc="upper left", bbox_to_anchor=(1.02, 1.0))
    for ax in axes.flat:
        band_len(ax); ax.set_xlabel("rod length, pivot to pivot (mm)"); ax.set_xticks(x); tidy(ax)
    book.save(fig, F_, "tradeoff_vs_rod_len", "Trade-off vs rod length",
              f"The same factors as the jam trade-off, against rod length (pivot to pivot) at neck.yaml's jam ({cur_jam:g} deg). "
              "At a fixed jam, a different rod length also changes the rod-end angles at rest (see the rest geometry chart). "
              "Grey band: neck.yaml's rod.",
              title=f"Rod length trade-off at jam {cur_jam:g} (grey band: neck.yaml rod, {cur_rod:g} mm)\n"
                    "solid: higher is better, dashed: lower is better, dotted: context")
    book.table(F_, "Trade-off vs rod length (table)", ["rod (mm)", "max tilt (deg)", "balanced pitch (deg)",
               "least off at 10 deg (mm)", "mean off at 10 deg (mm)", "off at max tilt (mm)", "swing (deg/deg)",
               "neutral height (mm)"],
               [[ll(L), *(num(F[k][li, cur_ji]) for k, *_ in PANELS)] for li, L in enumerate(lens)],
               f"The values plotted in the rod length trade-off chart, jam {cur_jam:g}.")

    # 15: every factor vs jam, one line per rod length
    fig, axes = plt.subplots(2, 4, figsize=(19, 7.6), constrained_layout=True)
    for ax, (key, title, unit, higher) in zip(axes.flat, PANELS):
        for li, L in enumerate(lens):
            ax.plot(xj, F[key][li], STYLE[higher], color=LEN_COLOURS[li], lw=2.4 if L == cur_rod else 1.4, ms=3, label=ll(L))
        ax.set_ylim(0, np.nanmax(F[key]) * 1.1); ax.set_title(title, fontsize=9); ax.set_ylabel(f"{unit}{better(higher)}")
    ax = axes.flat[7]
    for li, L in enumerate(lens):
        ax.plot(xj, F["rod_end_bottom"][li], "o:", color=LEN_COLOURS[li], lw=1.4, ms=3)
    ax.set_title("Rod-end angle at rest, sled end (context)", fontsize=9); ax.set_ylabel("deg")
    ax.set_ylim(0, np.nanmax(F["rod_end_bottom"]) * 1.1)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend([plt.Line2D([], [], color=h.get_color(), lw=h.get_linewidth()) for h in handles], labels,
               title="rod length", fontsize=8, title_fontsize=8, frameon=False, loc="outside lower center", ncol=len(lens))
    for ax in axes.flat:
        band_jam(ax); ax.set_xlabel("jam_deg (deg)"); tidy(ax)
    book.save(fig, F_, "tradeoff_vs_jam_by_rod_len", "Trade-off vs jam angle, per rod length",
              "The jam trade-off factors with one line per rod length; the thick line is neck.yaml's rod. Grey band: "
              "neck.yaml's jam.",
              title=f"Jam angle trade-off for each rod length (grey band: jam {cur_jam:g}; thick line: neck.yaml rod)\n"
                    "solid: higher is better, dashed: lower is better, dotted: context")

    # 16: heatmaps over (jam, rod length); 17: the same rescaled
    def heat_axes(ax):
        ax.add_patch(plt.Rectangle((cur_ji - 0.5, cur_li - 0.5), 1, 1, fill=False, ec="#d35400", lw=2))
        ax.set_yticks(range(len(lens))); ax.set_yticklabels([f"{L:g}" for L in lens])
        ax.set_xticks(range(len(jams))); ax.set_xticklabels([f"{j:g}" for j in jams], fontsize=7, rotation=45)
        ax.set_xlabel("jam_deg (deg)"); ax.set_ylabel("rod length (mm)")

    ext = [-0.5, len(jams) - 0.5, -0.5, len(lens) - 0.5]
    fig, axes = plt.subplots(2, 4, figsize=(19, 9.0), constrained_layout=True)
    fig.get_layout_engine().set(h_pad=0.25, hspace=0.06)
    for ax, (key, title, unit, higher) in zip(axes.flat, PANELS + [("rod_end_bottom", "Rod-end angle at rest, sled end (context)", "deg", None)]):
        v = F[key]
        im = ax.imshow(v, origin="lower", aspect="auto", cmap="Blues", extent=ext)
        for li in range(len(lens)):
            for ji in range(len(jams)):
                if np.isfinite(v[li, ji]):
                    ax.text(ji, li, f"{v[li, ji]:.1f}", ha="center", va="center", fontsize=6.5,
                            color="white" if v[li, ji] > np.nanmin(v) + 0.6 * (np.nanmax(v) - np.nanmin(v)) else INK)
        heat_axes(ax)
        ax.set_title(f"{title.splitlines()[0]} ({unit}){better(higher).replace(chr(10), ', ')}", fontsize=9)
        fig.colorbar(im, ax=ax, shrink=0.85)
    book.save(fig, F_, "heatmaps_jam_vs_rod_len", "Every factor over jam angle and rod length",
              "Each factor over (jam angle, rod length), with the values printed in the cells. Orange box: neck.yaml.",
              title=f"Every factor over jam angle and rod length (orange box: neck.yaml, jam {cur_jam:g}, {cur_rod:g} mm)")

    fig, axes = plt.subplots(2, 3, figsize=(15, 9.0), constrained_layout=True)
    fig.get_layout_engine().set(h_pad=0.25, hspace=0.06)
    for ax, (key, title, unit, higher) in zip(axes.flat, PANELS[:6]):
        v = F[key]; s = (v - np.nanmin(v)) / (np.nanmax(v) - np.nanmin(v)); s = s if higher else 1 - s
        im = ax.imshow(s, origin="lower", aspect="auto", cmap="Blues", vmin=0, vmax=1, extent=ext)
        for li in range(len(lens)):
            for ji in range(len(jams)):
                if np.isfinite(s[li, ji]):
                    ax.text(ji, li, f"{s[li, ji]:.2f}", ha="center", va="center", fontsize=6.5,
                            color="white" if s[li, ji] > 0.6 else INK)
        heat_axes(ax)
        ax.set_title(f"{title.splitlines()[0]}, {'higher' if higher else 'lower'} is better", fontsize=9)
    fig.colorbar(im, ax=axes, shrink=0.6, label="1 = best over the grid")
    book.save(fig, F_, "rescaled_heatmaps_jam_vs_rod_len", "Every factor rescaled over jam angle and rod length",
              "The six scored factors from the previous chart, each rescaled over the whole grid so 1 is its best value. "
              "Dark is the better end in every panel. No weighting and no combined score. Orange box: neck.yaml.",
              title="All factors rescaled so 1 = best over the grid (no weighting)\n"
                    f"dark is good for every panel; orange box: neck.yaml, jam {cur_jam:g}, {cur_rod:g} mm")

    # 18: rest geometry
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4), constrained_layout=True)
    for ax, key, title in zip(axes, ("neutral_height", "rod_end_bottom", "rod_end_top"),
                              ("Neutral height", "Rod-end angle at rest, sled end", "Rod-end angle at rest, platform end")):
        for li, L in enumerate(lens):
            ax.plot(xj, F[key][li], "o:", color=LEN_COLOURS[li], lw=2.4 if L == cur_rod else 1.4, ms=3, label=ll(L))
        ax.set_title(title, fontsize=9); ax.set_ylabel("mm" if key == "neutral_height" else "deg")
        ax.set_xlabel("jam_deg (deg)"); ax.set_ylim(0, np.nanmax(F[key]) * 1.1); band_jam(ax); tidy(ax)
    axes[2].legend(title="rod length", fontsize=7.5, title_fontsize=8, frameon=False, loc="upper left", bbox_to_anchor=(1.02, 1.0))
    book.save(fig, F_, "rest_geometry", "Rest geometry per rod length",
              "Neutral height and the rod-end angles at rest against jam angle, one line per rod length. The rod-end angle "
              "is the rod's angle out of the plane square to the rod end's screw.",
              title="Geometry at rest for each rod length", )

    # 19: pitch by heading (graceful) per rod length at neck.yaml's jam
    heads = np.radians(np.arange(0, 361))
    fig, axes = plt.subplots(1, 2, figsize=(13, 6.6), subplot_kw={"projection": "polar"}, constrained_layout=True)
    rmax = max(max(GU[L, cur_jam]["graceful", "up"].max(), GU[L, cur_jam]["graceful", "down"].max()) for L in lens)
    for ax, way in zip(axes, ("up", "down")):
        for li, L in enumerate(lens):
            ax.plot(heads, GU[L, cur_jam]["graceful", way], color=LEN_COLOURS[li], lw=2.4 if L == cur_rod else 1.4, label=ll(L))
        ax.set_ylim(0, np.ceil(rmax) + 1); ax.set_title(f"Pitch {way}", fontsize=10, pad=14)
        ax.set_rlabel_position(200); ax.tick_params(axis="y", labelsize=7); ax.set_xticks(np.radians(np.arange(0, 360, 30)))
    axes[1].legend(title="rod length", fontsize=8, title_fontsize=8, frameon=False, loc="upper left", bbox_to_anchor=(1.05, 1.0))
    book.save(fig, F_, "pitch_by_heading_by_rod_len", "Graceful pitch by head orientation, per rod length",
              f"Largest graceful pitch up and down for each head orientation on the platform, one line per rod length, at "
              f"neck.yaml's jam ({cur_jam:g} deg), {D.twist_name} twist (the default). Head orientation is head.rotation_deg (0 = rod end 1).",
              title=f"Largest graceful pitch by head orientation, jam {cur_jam:g}\n"
                    "angle = head orientation (deg), distance = pitch (deg)")

    # Table: graceful up / down at head orientation 0, rod x jam
    book.table(F_, "Graceful pitch up / down at head orientation 0, rod length x jam (table)",
               ["rod (mm) \\ jam", *(f"{j:g}" for j in jams)],
               [[f"{L:g}", *(f"{GU[L, j]['graceful', 'up'][0]:.1f} / {GU[L, j]['graceful', 'down'][0]:.1f}" for j in jams)]
                for L in lens],
               f"Each cell is the largest graceful pitch up / down (deg) with the head at head.rotation_deg 0 (facing rod end 1), for "
               f"the {D.twist_name} twist (the default).")

    # 20: same rod ends
    cur_angle = F["rod_end_bottom"][cur_li, cur_ji]
    ANGLES = sorted({20.0, round(cur_angle, 1), 25.0})
    ANGLE_COLOURS = ["#9ecae1", "#4292c6", "#08306b"]

    def at_angle(key, li, angle):
        return np.interp(angle, F["rod_end_bottom"][li], F[key][li])   # rod-end angle rises with jam

    rows = []
    for angle in ANGLES:
        for li, L in enumerate(lens):
            rows.append([angle, L, round(at_angle("neutral_height", li, angle), 2),
                         round(np.interp(angle, F["rod_end_bottom"][li], jams), 2),
                         *(round(at_angle(k, li, angle), 3) for k, *_ in PANELS[:6])])
    book.csv(F_, "same_rod_ends.csv", ["rod_end_angle_deg", "rod_len_mm", "neutral_height", "jam_deg", *(k for k, *_ in PANELS[:6])], rows)
    fig, axes = plt.subplots(2, 4, figsize=(19, 7.8), constrained_layout=True)
    extra = [("jam_deg", "Jam (twist at rest) that gives that angle (context)", "deg", None)]
    for ax, (key, title, unit, higher) in zip(axes.flat, PANELS + extra):
        for angle, colour in zip(ANGLES, ANGLE_COLOURS):
            if key == "jam_deg":
                v = [np.interp(angle, F["rod_end_bottom"][li], jams) for li in range(len(lens))]
            else:
                v = [at_angle(key, li, angle) for li in range(len(lens))]
            cur = abs(angle - cur_angle) < 0.06
            ax.plot(x, v, STYLE[higher], color=colour, lw=2.4 if cur else 1.4, ms=3,
                    label=f"{angle:g} deg" + (" (neck.yaml)" if cur else ""))
        ax.set_ylim(0, ax.get_ylim()[1] * 1.05); ax.set_title(title, fontsize=9); ax.set_ylabel(f"{unit}{better(higher)}")
        band_len(ax); ax.set_xlabel("rod length, pivot to pivot (mm)"); ax.set_xticks(x); tidy(ax)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend([plt.Line2D([], [], color=h.get_color(), lw=h.get_linewidth()) for h in handles], labels,
               title="rod-end angle at rest, sled end", fontsize=8, title_fontsize=8, frameon=False, loc="outside lower center", ncol=3)
    book.save(fig, F_, "same_rod_ends_vs_rod_len", "Trade-off vs rod length at the same rod-end angle",
              "The factors against rod length with the rod-end angle at rest held fixed (sled end, the larger of the two) "
              "instead of the jam. Each rod length's values are interpolated linearly across the swept jam angles to the "
              "jam that gives that angle. The last panel shows that jam.",
              title="Same rod ends, different rod length: factors at a fixed rod-end angle at rest (grey band: neck.yaml rod)\n"
                    "solid: higher is better, dashed: lower is better, dotted: context")
