"""Shared chart style, neck metrics and the plot book that writes plots.md. Data only."""
import csv, os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

INK, MUTED = "#222222", "#666666"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]   # dataviz reference palette, slots 1-6
STYLE = {True: "o-", False: "o--", None: "o:"}                                 # solid higher, dashed lower is better, dotted context
plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.edgecolor": MUTED, "figure.dpi": 130})

GRACEFUL_ROLL, GRACEFUL_OFF = 1.0, 10.0   # deg, mm
COLS = {k: i for i, k in enumerate(["jam_deg", "sled_1", "sled_2", "sled_3", "roll", "pitch", "yaw", "off_x", "off_y", "off"])}
HEADS_DEG = np.arange(0, 361)


def tidy(ax):
    ax.grid(color="#e6e6e6", lw=0.5)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def better(higher):
    return "" if higher is None else ("\nhigher is better" if higher else "\nlower is better")


# ------------------------------------------------------------------------------------------- metrics
def tilt(a):
    return np.degrees(np.arccos(np.cos(np.radians(a[:, COLS["roll"]])) * np.cos(np.radians(a[:, COLS["pitch"]]))))


def by_heading(a):
    """Head pitch/roll at each head heading on the platform (0 = a's +X, CCW): [heading, point].

    Pitch is the elevation of the head's forward axis (cos h, sin h), roll the elevation of its left axis,
    from the third row of R = Rz(yaw) Ry(pitch) Rx(roll)."""
    rr, pp = np.radians(a[:, COLS["roll"]]), np.radians(a[:, COLS["pitch"]])
    r20, r21 = -np.sin(pp), np.cos(pp) * np.sin(rr)
    h = np.radians(HEADS_DEG)
    c, s = np.cos(h)[:, None], np.sin(h)[:, None]
    h_pitch = np.degrees(np.arcsin(np.clip(r20 * c + r21 * s, -1, 1)))
    h_roll = np.degrees(np.arcsin(np.clip(-r20 * s + r21 * c, -1, 1)))
    return h_pitch, h_roll, np.broadcast_to(a[:, COLS["off"]][None, :], h_pitch.shape)


def to_frame(a, deg):
    """a with roll, pitch and yaw (ZYX) re-expressed for a head turned deg (CCW from the top) on the platform.
    Yaw stays a residual: at rest it's unchanged."""
    r, p, y = (np.radians(a[:, COLS[k]]) for k in ("roll", "pitch", "yaw"))
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    h = np.radians(deg); ch, sh = np.cos(h), np.sin(h)
    # columns 0 and 1 of R = Rz(y) Ry(p) Rx(r), and row 2 of column 2; R' = R Rz(h)
    c0 = np.stack([cy * cp, sy * cp, -sp]); c1 = np.stack([cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr])
    n0, n1 = c0 * ch + c1 * sh, -c0 * sh + c1 * ch
    out = a.copy()
    out[:, COLS["pitch"]] = np.degrees(-np.arcsin(np.clip(n0[2], -1, 1)))
    out[:, COLS["roll"]] = np.degrees(np.arctan2(n1[2], cp * cr))
    out[:, COLS["yaw"]] = (np.degrees(np.arctan2(n0[1], n0[0])) - deg + 180) % 360 - 180
    return out


def best_pitch(h_pitch, ok, sign):
    best = np.where(ok, sign * h_pitch, -np.inf).max(axis=1)
    return np.where(np.isfinite(best), np.maximum(best, 0), 0)


def graceful_ugly(a):
    """Largest pitch up/down by heading: {("graceful"|"ugly", "up"|"down"): [361]}."""
    hp, hr, off = by_heading(a)
    ok = (np.abs(hr) <= GRACEFUL_ROLL) & (off <= GRACEFUL_OFF)
    out = {}
    for sign, way in ((1, "up"), (-1, "down")):
        out["graceful", way] = best_pitch(hp, ok, sign)
        out["ugly", way] = np.maximum((sign * hp).max(axis=1), 0)
    return out


def sigma_p99(a, lim):
    n = 2 * lim + 1
    g = {}
    for k in ("roll", "pitch"):
        g[k] = np.full((n, n), np.nan)
        g[k][(a[:, 2] + lim).astype(int), (a[:, 1] + lim).astype(int)] = a[:, COLS[k]]
    d = {}
    for k in g:
        d1 = np.full((n, n), np.nan); d2 = np.full((n, n), np.nan)
        d1[:, 1:-1] = (g[k][:, 2:] - g[k][:, :-2]) / 2; d2[1:-1, :] = (g[k][2:, :] - g[k][:-2, :]) / 2
        d[k] = (d1, d2)
    J = np.stack([np.stack(d["roll"], -1), np.stack(d["pitch"], -1)], -2)
    ok = ~np.isnan(J).any(axis=(-1, -2))
    return np.percentile(np.linalg.svd(J[ok], compute_uv=False)[:, 0], 99)


def metrics(a, checks, lim):
    t = tilt(a)
    near10 = np.abs(t - 10) <= 0.5
    gu = graceful_ugly(a)
    i = int(np.argmax(t))
    return dict(max_tilt=t[i], balanced_pitch=np.minimum(gu["graceful", "up"], gu["graceful", "down"]).max(),
                least_off_10=a[near10, COLS["off"]].min() if near10.any() else np.nan,
                mean_off_10=a[near10, COLS["off"]].mean() if near10.any() else np.nan,
                off_at_max_tilt=a[i, COLS["off"]], sigma_max_p99=sigma_p99(a, lim),
                neutral_height=checks["neutral_height_mm"], rod_end_bottom=checks["rod_end_bottom_deg"],
                rod_end_top=checks["rod_end_top_deg"])


# (key, title, unit, higher is better? None = context only)
PANELS = [("max_tilt", "Max platform tilt", "deg", True),
          ("balanced_pitch", "Pitch up and down at the best heading\n(graceful: |roll| <= 1 deg, off <= 10 mm)", "deg", True),
          ("least_off_10", "Least off centre at 10 deg tilt", "mm", False),
          ("mean_off_10", "Mean off centre at 10 deg tilt", "mm", False),
          ("off_at_max_tilt", "Off centre at max tilt", "mm", False),
          ("sigma_max_p99", "Swing: d(roll, pitch)/d(sleds), 99th percentile", "deg/deg", False),
          ("neutral_height", "Neutral height (context)", "mm", None)]


# ------------------------------------------------------------------------------------------ tables
def md_table(header, rows, align=None):
    """Markdown table with padded columns, so it reads as plain text too. align: 'l' or 'r' per column."""
    cells = [[str(c) for c in header]] + [[str(c) for c in r] for r in rows]
    align = align or ["l"] + ["r"] * (len(header) - 1)
    w = [max(len(r[i]) for r in cells) for i in range(len(header))]
    pad = lambda s, i: s.ljust(w[i]) if align[i] == "l" else s.rjust(w[i])
    lines = ["| " + " | ".join(pad(c, i) for i, c in enumerate(cells[0])) + " |",
             "|" + "|".join(("-" * (w[i] + 1) + ":") if align[i] == "r" else ("-" * (w[i] + 2)) for i in range(len(w))) + "|"]
    lines += ["| " + " | ".join(pad(c, i) for i, c in enumerate(r)) + " |" for r in cells[1:]]
    return "\n".join(lines)


class Book:
    """Saves numbered plots (NN_name.png) and derived CSVs per folder, and collects them for plots.md."""

    def __init__(self, root):
        self.root = root
        self.entries = {}   # folder -> [("plot", file, title, about) | ("table", title, markdown, about)]

    def _dir(self, folder, *sub):
        d = os.path.join(self.root, folder, *sub)
        os.makedirs(d, exist_ok=True)
        return d

    def save(self, fig, folder, name, heading, about, title=None, dpi=None):
        n = sum(1 for e in self.entries.get(folder, []) if e[0] == "plot") + 1
        fname = f"{n:02d}_{name}.png"
        if title:
            fig.suptitle(title, fontsize=10)
        fig.savefig(os.path.join(self._dir(folder), fname), dpi=dpi)
        plt.close(fig)
        self.entries.setdefault(folder, []).append(("plot", fname, heading, about))
        print(f"plot {folder}/{fname}", flush=True)

    def table(self, folder, heading, header, rows, about, align=None):
        self.entries.setdefault(folder, []).append(("table", heading, md_table(header, rows, align), about))

    def csv(self, folder, name, header, rows):
        with open(os.path.join(self._dir(folder, "data"), name), "w", newline="") as f:
            w = csv.writer(f); w.writerow(header); w.writerows(rows)


def off_cmap(vmax):
    """Off-centre colour scale that switches hue at the graceful limit: blues up to it, oranges beyond."""
    from matplotlib.colors import LinearSegmentedColormap
    f = min(GRACEFUL_OFF / vmax, 0.999)
    return LinearSegmentedColormap.from_list("off", [(0, "#d6e6f7"), (f, "#08306b"), (f, "#fdd0a2"), (1, "#7f2704")])


def off_scatter(ax, a, vmax):
    """Roll vs pitch scatter coloured by off centre, graceful points drawn on top."""
    o = np.argsort(-a[:, COLS["off"]])
    return ax.scatter(a[o, COLS["roll"]], a[o, COLS["pitch"]], c=a[o, COLS["off"]], s=2, cmap=off_cmap(vmax), vmin=0, vmax=vmax,
                      linewidths=0)


def off_colorbar(fig, sc, axes, label, **kw):
    cb = fig.colorbar(sc, ax=axes, label=f"{label}\nblue: graceful (<= {GRACEFUL_OFF:g} mm), orange: beyond", **kw)
    cb.ax.axhline(GRACEFUL_OFF, color="white", lw=2)
    return cb


def num(v, fmt="{:.2f}"):
    return "-" if not np.isfinite(v) else fmt.format(v)
