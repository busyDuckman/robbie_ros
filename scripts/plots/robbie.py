"""this_robbie: charts for the neck exactly as neck.yaml defines it, both twists. Data only.

Ported from scripts/ik_simulation/jam_charts.py, pitch_by_heading_v3.py and cw_ccw_charts.py.
"""
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

import common
from common import COLS, SERIES, INK, tidy, by_heading, best_pitch, graceful_ugly, tilt

F_ = "this_robbie"
UP = ["#9ecae1", "#4292c6", "#08306b"]     # tight to loose
DOWN = ["#fdae6b", "#f16913", "#8c2d04"]
SEQ_BLUES = matplotlib.colors.LinearSegmentedColormap.from_list("blues", plt.get_cmap("Blues")(np.linspace(0.3, 1.0, 256)))   # palest step still clear of white
SLED_ABOUT = "Over the sled offsets sled_1 and sled_2 (sled_3 = -(sled_1 + sled_2)), CCW twist (the default), pass A. "
HEAD_ABOUT = ("Head orientation is head.rotation_deg, the direction the head faces on the platform: 0 is centre to rod end 1, "
              "counter-clockwise from the top (docs/head_rotation.svg); "
              "pitch is the elevation of the head's forward axis and roll the elevation of its left axis. Green line: "
              "neck.yaml's head.rotation_deg. ")


def run(book, D):
    LIM = D.lim; N = 2 * LIM + 1
    EXT = [-LIM - .5, LIM + .5, -LIM - .5, LIM + .5]
    axis = np.arange(-LIM, LIM + 1)
    R = D.robbie                      # {"CW"|"CCW": (pass a, pass b, checks)}
    ORDER = [d for d in ("CW", "CCW") if d in R]
    YD = D.twist_name                 # the default twist, CCW
    LABEL = {d: f"{d}" + (" (default)" if d == YD else "") for d in ORDER}
    # R is in the rod-end-1 frame (heading charts read as head.rotation_deg); HF is the head's own frame
    ROT = D.head_rot
    HF = {d: common.to_frame(R[d][0], ROT) for d in ORDER}
    A = HF[YD]
    start = D.start
    Z = {d: R[d][2]["head_zero_deg"] for d in ORDER}   # world angle of rod end 1 at rest

    def grid(a, key):
        g = np.full((N, N), np.nan)
        g[(a[:, 2] + LIM).astype(int), (a[:, 1] + LIM).astype(int)] = a[:, COLS[key]]
        return np.ma.masked_invalid(g)

    G = {k: grid(A, k) for k in ("roll", "pitch", "yaw", "off")}

    def sled_axes(ax):
        ax.set_xlabel("sled_1 (deg)"); ax.set_ylabel("sled_2 (deg)"); ax.set_aspect("equal")
        for s in ax.spines.values():
            s.set_visible(False)

    def show(ax, g, div):
        if div:
            v = np.abs(g).max(); return ax.imshow(g, origin="lower", extent=EXT, cmap="coolwarm", vmin=-v, vmax=v)
        return ax.imshow(g, origin="lower", extent=EXT, cmap="Blues", vmin=0, vmax=g.max())

    def contours(ax, g, step):
        cs = ax.contour(axis, axis, g, levels=np.arange(np.floor(g.min() / step) * step, g.max() + step, step),
                        colors=INK, linewidths=0.5)
        ax.clabel(cs, fmt="%g", fontsize=7)

    # ---------------------------------------------------------------- 1 roll, pitch, off side by side
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2), constrained_layout=True, sharey=True)
    for ax, (key, title, unit, div, step) in zip(axes, [("roll", "Head roll", "deg", True, 2), ("pitch", "Head pitch", "deg", True, 2),
                                                         ("off", "Off centre", "mm", False, 2)]):
        im = show(ax, G[key], div); contours(ax, G[key], step)
        sled_axes(ax); ax.set_title(f"{title} (contours every {step:g} {unit})", fontsize=9)
        fig.colorbar(im, ax=ax, shrink=0.8, label=unit)
    for ax in axes[1:]:
        ax.set_ylabel("")
    book.save(fig, F_, "roll_pitch_off", "Roll, pitch and off centre",
              SLED_ABOUT + "Head roll and pitch (head frame at head.rotation_deg, ZYX Euler) and off centre side by side.",
              title="Roll, pitch and off centre")

    # ---------------------------------------------------------------- 2-5 each field
    for key, title, unit, div, step, what in [("roll", "Head roll", "deg", True, 2, "Head roll (head frame at head.rotation_deg, ZYX Euler)."),
                                              ("pitch", "Head pitch", "deg", True, 2, "Head pitch (head frame at head.rotation_deg, ZYX Euler)."),
                                              ("yaw", "Head yaw (residual)", "deg", True, 5, "Head yaw that comes with each sled setting."),
                                              ("off", "Off centre", "mm", False, 2, "Distance of the platform centre from the neck axis.")]:
        fig, ax = plt.subplots(figsize=(6.4, 5.6), constrained_layout=True)
        im = show(ax, G[key], div); contours(ax, G[key], step)
        sled_axes(ax); fig.colorbar(im, ax=ax, shrink=0.8, label=unit)
        book.save(fig, F_, key, title, SLED_ABOUT + what, title=f"{title} (contours every {step:g} {unit})")

    # ---------------------------------------------------------------- 6 iso roll/pitch
    fig, ax = plt.subplots(figsize=(6.4, 5.6), constrained_layout=True)
    light_greys = matplotlib.colors.LinearSegmentedColormap.from_list("light_greys", ["#ffffff", "#9a9a9a"])
    im = ax.imshow(G["off"], origin="lower", extent=EXT, cmap=light_greys, vmin=0, vmax=G["off"].max())
    for key, colour in (("roll", "#c0392b"), ("pitch", "#2a6fdb")):
        g = G[key]
        cs = ax.contour(axis, axis, g, levels=np.arange(np.floor(g.min() / 2) * 2, g.max() + 2, 2), colors=colour, linewidths=0.7)
        ax.clabel(cs, fmt="%g", fontsize=6)
    ax.plot([], [], color="#c0392b", label="roll (every 2 deg)"); ax.plot([], [], color="#2a6fdb", label="pitch (every 2 deg)")
    ax.legend(loc="lower left", fontsize=8, frameon=False)
    sled_axes(ax); fig.colorbar(im, ax=ax, shrink=0.8, label="off centre (mm)")
    book.save(fig, F_, "iso_roll_pitch", "Iso-roll and iso-pitch lines",
              SLED_ABOUT + "Lines of constant roll and constant pitch (every 2 deg) drawn over off centre in light grey.",
              title="Iso-roll and iso-pitch lines over off centre\n"
                    "where a roll line crosses a pitch line, that sled pair gives that roll/pitch")

    # ---------------------------------------------------------------- 7 sled grid in roll/pitch
    fig, ax = plt.subplots(figsize=(6.4, 6.0), constrained_layout=True)
    Rg, Pg = G["roll"], G["pitch"]
    for k in range(0, N, 4):
        ax.plot(Rg[:, k], Pg[:, k], color="#c0392b", lw=0.5); ax.plot(Rg[k, :], Pg[k, :], color="#2a6fdb", lw=0.5)
    ax.plot([], [], color="#c0392b", label="constant sled_1 (every 4 deg)")
    ax.plot([], [], color="#2a6fdb", label="constant sled_2 (every 4 deg)")
    ax.plot([0], [0], "o", color=INK, ms=4, label="sleds at start_deg")
    ax.legend(loc="lower right", fontsize=8, frameon=False)
    ax.set_xlabel("roll (deg)"); ax.set_ylabel("pitch (deg)"); ax.set_aspect("equal"); ax.grid(color="#e6e6e6", lw=0.5)
    book.save(fig, F_, "sled_grid_in_roll_pitch", "Sled grid in roll/pitch",
              "Lines of constant sled_1 and constant sled_2 (every 4 deg) mapped into head roll and pitch (head frame), CCW twist (the default).",
              title="The sled_1/sled_2 grid mapped into roll/pitch\n"
                    "where the grid overlaps itself, a roll/pitch has more than one sled solution")

    # ---------------------------------------------------------------- 8-11 Jacobian
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
    book.save(fig, F_, "jacobian_det", "Jacobian determinant",
              SLED_ABOUT + "Determinant of d(roll, pitch)/d(sled_1, sled_2), central differences on the 1 deg grid; the "
              "black line is det = 0.",
              title="det d(roll, pitch)/d(sled_1, sled_2), black line is det = 0\ncolour clipped at the 99.5th percentile")
    for name, heading, data, title, unit in [
            ("sigma_max", "Largest Jacobian singular value", sv[..., 0], "Largest singular value of d(roll, pitch)/d(sled_1, sled_2)", "deg/deg"),
            ("sigma_min", "Smallest Jacobian singular value", sv[..., 1], "Smallest singular value of d(roll, pitch)/d(sled_1, sled_2)", "deg/deg"),
            ("off_gradient", "Off-centre gradient", np.hypot(o1, o2), "|d off / d(sled_1, sled_2)|", "mm/deg")]:
        fig, ax = plt.subplots(figsize=(6.4, 5.6), constrained_layout=True)
        im = ax.imshow(np.ma.masked_invalid(data), origin="lower", extent=EXT, cmap="Blues", vmin=0, vmax=np.nanpercentile(data, 99.5))
        sled_axes(ax); fig.colorbar(im, ax=ax, shrink=0.8, label=unit)
        book.save(fig, F_, name, heading, SLED_ABOUT + f"{title}, central differences on the 1 deg grid.",
                  title=f"{title}\ncolour clipped at the 99.5th percentile")

    # ---------------------------------------------------------------- 12-16 per roll/pitch target
    # The sled grid is split into triangles, each mapped linearly into roll/pitch. A target's solutions are
    # the triangles that contain it; off centre, yaw and sled angles are interpolated in each.
    STEP = 0.25
    rs = np.arange(np.floor(Rg.min()), np.ceil(Rg.max()) + STEP, STEP)
    ps = np.arange(np.floor(Pg.min()), np.ceil(Pg.max()) + STEP, STEP)
    count = np.zeros((len(ps), len(rs)), int)
    offs = [[[] for _ in rs] for _ in ps]   # per target: (off, yaw, sled_1, sled_2) of each solution
    Rf, Pf, Of, Yf = (np.ma.filled(G[k].astype(float), np.nan) for k in ("roll", "pitch", "off", "yaw"))
    for i in range(N - 1):
        for j in range(N - 1):
            for tri in (((i, j), (i, j + 1), (i + 1, j)), ((i + 1, j + 1), (i + 1, j), (i, j + 1))):
                xy = np.array([[Rf[a], Pf[a]] for a in tri])
                o = np.array([[Of[a], Yf[a], a[1] - LIM, a[0] - LIM] for a in tri])   # grid row = sled_2, column = sled_1
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
    def per_target(f):
        return np.array([[f(np.array(v)) if v else np.nan for v in row] for row in offs])

    def sled_spread(v):   # largest sled angle difference between any two solutions, sled_3 = -(sled_1 + sled_2)
        s = np.column_stack([v[:, 2], v[:, 3], -(v[:, 2] + v[:, 3])])
        return np.abs(s[:, None, :] - s[None, :, :]).max()

    min_off, max_off = per_target(lambda v: v[:, 0].min()), per_target(lambda v: v[:, 0].max())
    yaw_spread = per_target(lambda v: np.ptp(v[:, 1]))
    sled_spread_ = per_target(sled_spread)
    book.csv(F_, "roll_pitch_targets.csv", ["roll", "pitch", "n_solutions", "min_off", "max_off", "yaw_spread", "sled_spread"],
             [[round(r_, 2), round(p, 2), count[a, b], round(min_off[a, b], 3), round(max_off[a, b], 3),
               round(yaw_spread[a, b], 3), round(sled_spread_[a, b], 3)]
              for a, p in enumerate(ps) for b, r_ in enumerate(rs) if count[a, b]])
    REXT = [rs[0] - STEP / 2, rs[-1] + STEP / 2, ps[0] - STEP / 2, ps[-1] + STEP / 2]
    fig, ax = plt.subplots(figsize=(6.4, 5.6), constrained_layout=True)
    n = int(count.max())
    cols = ["#9ecae1", "#4292c6", "#08306b"][:n]   # 1-3 in stepped blues, 4 and up in oranges/reds
    if n > 3:
        cols += list(plt.get_cmap("OrRd")(np.linspace(0.35, 1.0, n - 3)))
    im = ax.imshow(np.ma.masked_equal(count, 0), origin="lower", extent=REXT, cmap=matplotlib.colors.ListedColormap(cols),
                   vmin=0.5, vmax=n + 0.5)
    ax.set_xlabel("roll (deg)"); ax.set_ylabel("pitch (deg)"); ax.set_aspect("equal"); ax.grid(color="#e6e6e6", lw=0.5)
    fig.colorbar(im, ax=ax, shrink=0.8, label="number of sled solutions", ticks=range(1, n + 1))
    book.save(fig, F_, "solution_count", "Sled solutions per roll/pitch target",
              "For each roll/pitch target (0.25 deg bins), how many sled settings give it, from the 1 deg grid split into "
              "triangles mapped linearly into roll/pitch. White is not reachable.",
              title="Sled solutions per roll/pitch target (white: not reachable)\n"
                    "counts in the thin strips (mostly 4+) depend on the 1 deg grid; unchecked")
    for name, heading, data, title, unit in [
            ("min_off", "Least off centre per target", min_off, "Least off centre among the solutions for each roll/pitch target", "mm"),
            ("off_spread", "Off-centre spread per target", max_off - min_off, "Off centre spread (max - min) among the solutions", "mm"),
            ("yaw_spread", "Yaw spread per target", yaw_spread, "Yaw spread (max - min) among the solutions", "deg"),
            ("sled_spread", "Sled spread per target", sled_spread_,
             "Sled spread among the solutions: the largest difference in any one sled's angle between two solutions", "deg")]:
        fig, ax = plt.subplots(figsize=(6.4, 5.6), constrained_layout=True)
        spread = "spread" in name
        if spread:   # one solution: flat grey, off the colour scale
            ax.imshow(np.ma.masked_where(count != 1, np.ones_like(data)), origin="lower", extent=REXT,
                      cmap=matplotlib.colors.ListedColormap(["#d9d9d9"]))
            data = np.where(count == 1, np.nan, data)
        im = ax.imshow(np.ma.masked_invalid(data), origin="lower", extent=REXT, cmap=SEQ_BLUES, vmin=0, vmax=np.nanmax(data))
        ax.set_xlabel("roll (deg)"); ax.set_ylabel("pitch (deg)"); ax.set_aspect("equal"); ax.grid(color="#e6e6e6", lw=0.5)
        fig.colorbar(im, ax=ax, shrink=0.8, label=unit)
        book.save(fig, F_, name, heading, f"{title}, over the same roll/pitch bins as the solution count. White: not reachable" +
                  ("; grey: one solution." if spread else "."),
                  title=title.replace(": ", ":\n") + ("\nwhite: not reachable, grey: one solution" if spread else "\nwhite: not reachable"))

    # ---------------------------------------------------------------- pitch by head orientation
    heads = np.radians(np.arange(0, 361))

    HEAD_COLOUR = SERIES[5]

    def joint_lines(ax, d, R_, nod=False, tops_style="--"):
        """Sled pivots, rod top pivots, nod lines and the head (head.rotation_deg), in the rod-end-1 frame."""
        orbit = R[d][2]["orbit_deg"]   # measured in the sim at rest, CCW positive
        for i, (st, ob) in enumerate(zip(start, orbit), 1):
            lines = [(st - 90 - Z[d], "#444444", "-", "sled"), (st + ob - 90 - Z[d], "#444444", tops_style, "top")]
            if nod:
                lines.append((st + ob / 2 - 90 - Z[d], "#d62728", "-", "nod"))
            for a_, colour, ls, name in lines:
                ax.plot([np.radians(a_)] * 2, [0, R_], color=colour, lw=1.4 if name == "nod" else 1.0, ls=ls)
                ax.text(np.radians(a_), R_ * 0.93, f"{name} {i}", fontsize=7, color=colour, ha="center", va="center",
                        bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none"))
        ax.plot([np.radians(ROT)] * 2, [0, R_], color=HEAD_COLOUR, lw=2.0)
        ax.text(np.radians(ROT), R_ * 0.8, "head", fontsize=7, color=HEAD_COLOUR, ha="center", va="center",
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none"))

    def nods(d):
        """Nod line orientations (deg, 0-360, rod-end-1 frame): halfway round each rod's twist from its sled pivot, at rest."""
        return [(st + ob / 2 - 90 - Z[d]) % 360 for st, ob in zip(start, R[d][2]["orbit_deg"])]

    NOD_LS = {"CW": "-", "CCW": "--"}

    def nod_vlines(ax, twists):
        for d in twists:
            for h in nods(d):
                ax.axvline(h, color="#d62728", lw=1.0, ls=NOD_LS[d], zorder=0)
        ax.axvline(ROT % 360, color=HEAD_COLOUR, lw=2.0, zorder=0)

    def polar(ax, R_):
        ax.set_ylim(0, R_); ax.set_rlabel_position(200); ax.tick_params(axis="y", labelsize=7)
        ax.set_xticks(np.radians(np.arange(0, 360, 30)))

    HB = {d: by_heading(R[d][0]) for d in ORDER}
    ROLL_TOLS, OFF_TOLS, V2_OFF_TOLS = [0.5, 1, 2], [3, 5, 10], [5, 10, 20]

    # v1: roll and off centre limited together
    hp, hr, off = HB[YD]; ar = np.abs(hr)
    fig, axes = plt.subplots(1, 2, figsize=(12, 6.2), subplot_kw={"projection": "polar"}, constrained_layout=True)
    curves = [[(best_pitch(hp, (ar <= r) & (off <= o), sign), c, f"|roll| <= {r:g} deg, off <= {o:g} mm")
               for r, o, c in zip(ROLL_TOLS, OFF_TOLS, UP)] for sign in (1, -1)]
    R_ = np.ceil(max(c[0].max() for cs in curves for c in cs)) + 1
    for ax, cs, label in zip(axes, curves, ("Pitch up (head looks up)", "Pitch down (head looks down)")):
        for best, colour, name in cs:
            ax.plot(heads, best, color=colour, lw=2, label=name)
        ax.set_title(label, fontsize=9, pad=14); polar(ax, R_); joint_lines(ax, YD, R_, nod=True)
    axes[1].legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8, frameon=False)
    book.save(fig, F_, "pitch_by_head_orientation_v1", "Pitch by head orientation, three limit levels",
              HEAD_ABOUT + "For each head orientation, the largest pitch up and down among the sweep points within each "
              "roll and off-centre limit pair. Grey lines: sled pivots (solid) and rod top pivots (dashed) at rest; red: nod lines.",
              title="Largest pitch by head orientation on the platform, within roll and off-centre limits\n"
                    "angle = head orientation (deg), distance = pitch (deg)")

    # v2: roll and off limited separately, then together
    def v2_curves(d):
        hp, hr, off = HB[d]; ar = np.abs(hr); g = {}
        for col_i, (err, tols, fmt) in enumerate([(ar, ROLL_TOLS, "|roll| <= {:g} deg"), (off, V2_OFF_TOLS, "off <= {:g} mm")]):
            for row_i, (sign, colours) in enumerate(((1, UP), (-1, DOWN))):
                way = ("up", "down")[row_i]
                g[row_i, col_i] = (f"Pitch {way}, limited by {('roll', 'off centre')[col_i]} only",
                                   [(best_pitch(hp, err <= t, sign), c, fmt.format(t)) for t, c in zip(tols, colours)])
        for col_i, (sign, colours, way) in enumerate(((1, UP, "up"), (-1, DOWN, "down"))):
            g[2, col_i] = (f"Pitch {way}, limited by roll and off centre together",
                           [(best_pitch(hp, (ar <= r) & (off <= o), sign), c, f"{r:g} deg, {o:g} mm")
                            for r, o, c in zip(ROLL_TOLS, V2_OFF_TOLS, colours)])
        return g

    V2 = {d: v2_curves(d) for d in ORDER}
    R2 = np.ceil(max(c[0].max() for d in ORDER for _, cs in V2[d].values() for c in cs)) + 1
    for d in [YD] + [o for o in ORDER if o != YD]:
        fig, axes = plt.subplots(3, 2, figsize=(12, 21.5), subplot_kw={"projection": "polar"}, constrained_layout=True)
        fig.get_layout_engine().set(w_pad=0.02, h_pad=0.04, hspace=0.02, wspace=0.02)
        for (row_i, col_i), (title, cs) in V2[d].items():
            ax = axes[row_i, col_i]
            for best, colour, name in cs:
                ax.plot(heads, best, color=colour, lw=2, label=name)
            ax.set_title(title, fontsize=10, pad=14)
            ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.06), ncol=3, fontsize=8, frameon=False, columnspacing=1.2, handlelength=1.5)
            polar(ax, R2); joint_lines(ax, d, R2, nod=True)
        book.save(fig, F_, f"pitch_by_head_orientation_v2_{d.lower()}", f"Pitch by head orientation, limits split, {LABEL[d]}",
                  HEAD_ABOUT + "Rows 1-2: pitch up and down limited by roll only (left) and off centre only (right). Row 3: "
                  "both together. Same radial scale for both twists. Grey lines: sled pivots (solid) and rod top pivots "
                  "(dashed) at rest; red: nod lines.",
                  title=f"Largest pitch by head orientation on the platform, {LABEL[d]}\n"
                        "angle = head orientation (deg), distance = pitch (deg)")

    # v3: graceful and ugly, CW on top, CCW below, with nod lines
    GU = {d: graceful_ugly(R[d][0]) for d in ORDER}
    R3 = np.ceil(max(v.max() for t in GU.values() for v in t.values())) + 1
    fig, axes = plt.subplots(len(ORDER), 2, figsize=(12, 6.2 * len(ORDER)), subplot_kw={"projection": "polar"},
                             constrained_layout=True, squeeze=False)
    fig.get_layout_engine().set(h_pad=0.04, hspace=0.03)
    for row, d in zip(axes, ORDER):
        for ax, way in zip(row, ("up", "down")):
            ax.fill_between(heads, GU[d]["ugly", way], R3, color="#d9d9d9", lw=0, zorder=0, label="unreachable")
            ax.plot(heads, GU[d]["graceful", way], color=SERIES[0], lw=2, label="graceful")
            ax.plot(heads, GU[d]["ugly", way], color=SERIES[1], lw=2, label="ugly")
            joint_lines(ax, d, R3, nod=True); polar(ax, R3)
            ax.set_title(f"{LABEL[d]}: pitch {way} (head looks {way})", fontsize=10, pad=14)
        row[1].legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8, frameon=False)
    book.save(fig, F_, "pitch_by_head_orientation", "Graceful and ugly pitch by head orientation, both twists",
              HEAD_ABOUT + "Blue: largest graceful pitch; orange: largest ugly pitch; grey: beyond ugly, unreachable. CW "
              "twist on the top row, CCW below (viewed from the top). Grey lines: sled pivots (solid) and rod top pivots "
              "(dashed) at rest, measured in the sim; red: nod lines.",
              title="Largest pitch by head orientation on the platform\nangle = head orientation (deg), distance = pitch (deg)")
    hi = int(round(ROT)) % 360
    book.table(F_, "Pitch at neck.yaml's head orientation (table)", ["Twist", "Up (graceful)", "Down (graceful)", "Up (ugly)", "Down (ugly)"],
               [[LABEL[d], *(f"{GU[d][k, w][hi]:.1f} deg" for k in ("graceful", "ugly") for w in ("up", "down"))] for d in ORDER],
               f"Largest pitch with the head at head.rotation_deg = {ROT:g}, from the chart above.")
    book.csv(F_, "pitch_by_head_orientation.csv", ["orientation_deg", *(f"{d.lower()}_{k}_{w}" for d in ORDER for k in ("graceful", "ugly") for w in ("up", "down"))],
             [[h, *(round(GU[d][k, w][h], 3) for d in ORDER for k in ("graceful", "ugly") for w in ("up", "down"))] for h in range(360)])

    # ---------------------------------------------------------------- CW vs CCW comparisons
    FIELDS = [("roll", "Head roll", "deg", True, 2), ("pitch", "Head pitch", "deg", True, 2),
              ("yaw", "Head yaw (residual)", "deg", True, 5), ("off", "Off centre", "mm", False, 2)]
    fig, axes = plt.subplots(len(FIELDS), len(ORDER), figsize=(5.25 * len(ORDER), 4.6 * len(FIELDS)), constrained_layout=True, squeeze=False)
    for ri, (key, title, unit, div, step) in enumerate(FIELDS):
        gs = {d: grid(HF[d], key) for d in ORDER}
        v = max(np.abs(g).max() for g in gs.values())
        for ci, d in enumerate(ORDER):
            ax, g = axes[ri, ci], gs[d]
            im = ax.imshow(g, origin="lower", extent=EXT, cmap="coolwarm" if div else "Blues", vmin=-v if div else 0, vmax=v)
            cs = ax.contour(axis, axis, g, levels=np.arange(np.floor(g.min() / step) * step, g.max() + step, step), colors=INK, linewidths=0.5)
            ax.clabel(cs, fmt="%g", fontsize=6)
            sled_axes(ax); ax.set_title(f"{title}, {LABEL[d]}\ncontours every {step:g} {unit}", fontsize=9)
        fig.colorbar(im, ax=axes[ri, :], shrink=0.85, label=unit)
    book.save(fig, F_, "heatmaps_cw_ccw", "Roll, pitch, yaw and off centre, both twists",
              "Roll, pitch, yaw and off centre over the sled offsets for each twist, side by side on shared colour scales, pass A.",
              title="Roll, pitch, yaw and off centre over (sled_1, sled_2), CW and CCW viewed from the top")

    fig, axes = plt.subplots(1, len(ORDER), figsize=(5.5 * len(ORDER), 5.4), constrained_layout=True, sharex=True, sharey=True, squeeze=False)
    vmax = max(np.percentile(HF[d][:, COLS["off"]], 99.5) for d in ORDER)
    for ax, d in zip(axes[0], ORDER):
        a = HF[d]
        sc = common.off_scatter(ax, a, vmax)
        ax.set_title(LABEL[d], fontsize=9); ax.set_aspect("equal"); ax.grid(color="#e6e6e6", lw=0.5)
        ax.set_xlabel("roll (deg)"); ax.set_ylabel("pitch (deg)")
    common.off_colorbar(fig, sc, axes, shrink=0.8, label="off centre (mm), colour clipped at the 99.5th percentile")
    book.save(fig, F_, "roll_pitch_coverage_cw_ccw", "Roll/pitch coverage, both twists",
              "Every pass A sweep point at its head roll and pitch (head frame at head.rotation_deg), coloured by off centre, one panel per twist. "
              "Blue: off centre within the graceful limit (10 mm), orange: beyond it; the bluer points are drawn on top.",
              title="Roll vs pitch at every sweep point, CW and CCW viewed from the top, pass A")

    TILT_BINS = [5, 10, 15]
    rows = []
    for d in ORDER:
        a = R[d][0]; t = tilt(a); i = int(np.argmax(t))
        means = [a[np.abs(t - b) <= 0.5, COLS["off"]].mean() for b in TILT_BINS]
        rows.append([d, round(t[i], 2), *a[i, 1:4].astype(int), round(a[i, COLS["off"]], 2), *np.round(means, 2)])
    book.csv(F_, "max_tilt.csv", ["twist", "max_tilt", "sled_1", "sled_2", "sled_3", "off", *(f"mean_off_at_{b}deg" for b in TILT_BINS)], rows)
    COL = {"CW": SERIES[0], "CCW": SERIES[1]}
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.8), constrained_layout=True)
    x = np.arange(len(ORDER))
    ax1.bar(x, [r[1] for r in rows], color=[COL[d] for d in ORDER], width=0.5)
    for xi, r in zip(x, rows):
        ax1.text(xi, r[1], f"{r[1]:.2f}", ha="center", va="bottom", fontsize=8)
    ax1.set_xticks(x, [LABEL[d] for d in ORDER]); ax1.set_ylabel("max tilt (deg)\nhigher is better")
    for k, d in enumerate(ORDER):
        ax2.plot(range(4), [rows[k][5], *rows[k][6:]], "o--", color=COL[d], lw=2, ms=5, label=LABEL[d])
    ax2.set_xticks(range(4), ["at max tilt", *(f"mean at {b} deg tilt" for b in TILT_BINS)], fontsize=8)
    ax2.set_ylabel("off centre (mm)\nlower is better"); ax2.legend(frameon=False, fontsize=8)
    for ax in (ax1, ax2):
        ax.set_ylim(0, ax.get_ylim()[1] * 1.1); ax.grid(color="#e6e6e6", lw=0.5, axis="y")
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    book.save(fig, F_, "max_tilt_cw_ccw", "Max platform tilt, both twists",
              "Left: the largest platform tilt (angle from level) reached in the sweep, per twist. Right: off centre at that "
              "point and the mean off centre within 0.5 deg of 5, 10 and 15 deg tilt.",
              title="Max platform tilt (angle from level) and off centre, CW and CCW viewed from the top\n"
                    "bars: higher is better; dashed: lower is better")

    heads_deg = np.arange(0, 361)
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), constrained_layout=True, sharex=True)
    pbh = {}
    for row_i, (rt, ot) in enumerate(zip(ROLL_TOLS, V2_OFF_TOLS)):
        for col_i, (sign, way) in enumerate(((1, "up"), (-1, "down"))):
            ax = axes[row_i, col_i]
            for d in ORDER:
                hp, hr, off = HB[d]
                pbh[d, way, rt, ot] = best_pitch(hp, (np.abs(hr) <= rt) & (off <= ot), sign)
                ax.plot(heads_deg, pbh[d, way, rt, ot], "-", color=COL[d], lw=2, label=LABEL[d])
            nod_vlines(ax, ORDER)
            ax.set_ylim(0, R2 * 1.05); ax.set_xlim(0, 360); ax.set_xticks(range(0, 361, 30))
            ax.set_title(f"Pitch {way}, |roll| <= {rt:g} deg and off <= {ot:g} mm", fontsize=9)
            ax.set_ylabel(f"largest pitch {way} (deg)\nhigher is better"); tidy(ax)
    for ax in axes[-1]:
        ax.set_xlabel("head orientation, head.rotation_deg (deg, 0 = rod end 1)")
    h_, l_ = axes[0, 1].get_legend_handles_labels()
    h_ += [plt.Line2D([], [], color="#d62728", lw=1.0, ls=NOD_LS[d]) for d in ORDER]
    l_ += [f"nod lines, {LABEL[d]}" for d in ORDER]
    axes[0, 1].legend(h_, l_, frameon=False, fontsize=8)
    book.save(fig, F_, "pitch_by_head_orientation_cw_vs_ccw", "Pitch by head orientation, CW and CCW on the same axes",
              HEAD_ABOUT + "Largest pitch up (left) and down (right) for each twist, at three roll and off-centre limit "
              "pairs (rows). Red vertical lines: each twist's nod lines.", title="Largest pitch up and down by head orientation, CW vs CCW viewed from the top")

    if len(ORDER) == 2:
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True, sharey=True)
        for col_i, way in enumerate(("up", "down")):
            ax = axes[col_i]
            for k, (rt, ot) in enumerate(zip(ROLL_TOLS, V2_OFF_TOLS)):
                ax.plot(heads_deg, pbh["CCW", way, rt, ot] - pbh["CW", way, rt, ot], "-", color=SERIES[k + 2], lw=1.6,
                        label=f"|roll| <= {rt:g} deg, off <= {ot:g} mm")
            ax.axhline(0, color=INK, lw=0.8, ls=":")
            nod_vlines(ax, ORDER)
            ax.set_xlim(0, 360); ax.set_xticks(range(0, 361, 30)); tidy(ax)
            ax.set_title(f"Pitch {way}: CCW minus CW at the same head orientation", fontsize=9)
            ax.set_xlabel("head orientation, head.rotation_deg (deg, 0 = rod end 1)")
        axes[0].set_ylabel("deg (above 0: CCW reaches further,\nbelow 0: CW reaches further)")
        h_, l_ = axes[1].get_legend_handles_labels()
        h_ += [plt.Line2D([], [], color="#d62728", lw=1.0, ls=NOD_LS[d]) for d in ORDER]
        l_ += [f"nod lines, {LABEL[d]}" for d in ORDER]
        fig.legend(h_, l_, frameon=False, fontsize=8, loc="outside lower center", ncol=5)
        book.save(fig, F_, "pitch_by_head_orientation_difference", "Pitch difference between the twists",
                  "Largest pitch for CCW minus CW at the same head orientation, at three roll and off-centre limit pairs. Red "
                  "vertical lines: each twist's nod lines.",
                  title="Largest pitch, CCW minus CW viewed from the top, at each head orientation\ndotted: zero, context")

    # poses for the renders: rest, largest graceful pitch up and down with the head at head.rotation_deg.
    # heading and nods are in the rod-end-1 frame; zero turns them to the platform frame the renders draw in.
    poses = []
    for d in ORDER:
        hp, hr, off = HB[d]; a = R[d][0]
        ok = (np.abs(hr) <= 1) & (off <= 10)
        r0 = int(np.flatnonzero((a[:, 1] == 0) & (a[:, 2] == 0))[0])
        for pose, i in (("rest", r0), ("largest graceful pitch up", int(np.where(ok, hp, -np.inf).argmax(axis=1)[hi])),
                        ("largest graceful pitch down", int(np.where(ok, -hp, -np.inf).argmax(axis=1)[hi]))):
            poses.append(dict(direction=d, heading=hi, zero=Z[d], pose=pose, sleds=[float(v) for v in a[i, 1:4]], nods=nods(d)))
    return poses
