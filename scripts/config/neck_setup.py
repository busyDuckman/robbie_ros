# /// script
# requires-python = "==3.12.*"  # mujoco 3.12.0 has no wheels for newer Pythons
# dependencies = [
#     "mujoco==3.12.0",  # the version mujoco_vendor bundles for the sim
#     "numpy",
#     "pyyaml",
#     "matplotlib",
# ]
# ///
"""Neck setup: a window for fitting neck.yaml to your Robbie, and checking it.

Usage (Linux or WSL, from the repo root):
    just config
    uv run scripts/config/neck_setup.py

Hover over anything for what it means. It edits neck.yaml: written only when you press Save, comments kept.
Only the CW/CCW view is kept elsewhere (.plots/neck_setup.json), as it isn't part of the neck. Sims share .plots/sim_cache with just plot, so anything either one has run is instant in the other.
"""
import copy, glob, json, math, multiprocessing, os, queue, re, subprocess, sys, threading

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "plots"))

import matplotlib
import matplotlib.image as mpimg
import numpy as np
from matplotlib.figure import Figure
from matplotlib.patches import Polygon, Rectangle
try:
    import tkinter as tk
    matplotlib.use("TkAgg")
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from tkinter import messagebox, ttk
except ImportError:   # no tkinter (e.g. CI's system Python): the helpers still import for tests, the window can't open
    tk = None

import common
import sim as S

PLOTS = os.path.join(S.REPO, ".plots")
CACHE = os.path.join(PLOTS, "sim_cache")
STATE = os.path.join(PLOTS, "neck_setup.json")
JAM_STEPS = [-8, -4, -2, 2, 4, 8]   # deg either side of the jam, for "How sure?"
STUDIED_JAMS = (60, 100)            # deg, the range just plot compares
HEX_INSET = 0.006                   # as gen_neck_mjcf.py: hexagon edge middles sit this far inside the rod ends
OK, WARN, BAD, TODO = "ok", "warn", "bad", "todo"
COLOUR = {OK: "#1a7f37", WARN: "#b35900", BAD: "#c0392b", TODO: "#777777"}
BASE_R, COLLAR_R, COLLAR_H = 75, 62.5, 10   # mm, as gen_neck_mjcf.py draws the base and collar
PART = "#3d5be0"                     # printed parts, blue as in robbie_openscad renders
UP, DOWN, HEAD, NOD = "#2a78d6", "#eb6834", "#008300", "#b8860b"

# (neck.yaml path, label, unit, scale from yaml, spin step, tooltip)
MAIN = [
    (("rod", "length"), "Rod length", "mm", 1000, 0.5,
     "Centre of one rod-end ball to the centre of the other. Measure it."),
    (("rod_end", "max_angle_deg"), "Rod-end max angle", "deg", 1, 0.5,
     "From the rod end's spec sheet: how far the ball can tilt in its housing.\n"
     "If the sheet gives the total swing (both ways), use half.\n"
     "The rod ends jamming at this angle is what holds the neck in its pole.\n"
     "Leave it blank if you don't know it."),
    (("platform", "measured_height"), "Measured height", "mm", 1000, 0.5,
     "Neck at rest: sleds 120° apart, head level, twisted into its pole.\n"
     "Measure from the table to the centre of a top rod-end ball (see the side view).\n"
     "Leave it blank if you haven't measured it."),
    (("platform", "rest_height"), "Rest height", "mm", 1000, 0.1,
     "Sled pivots to platform pivots, at rest. This sets the jam (the twist at rest).\n"
     "Type it, or fill it from the rod-end spec or your measured height with the buttons."),
    (("head", "rotation_deg"), "Head rotation", "deg", 1, 1,
     "Which way the head faces on the platform: 0 is centre → rod end 1, + is CCW seen from above.\n"
     "Roll and pitch are in this frame. Facing the up end of a nod line gives the most graceful pitch up, the down end pitch down.\n"
     "Your robot, your call."),
]
OPENSCAD = [
    (("collar_z",), "Base height", "mm", 1000, 0.5, "openscad: base_h, the base the collar sits on (neck.yaml collar_z)."),
    (("sled", "pivot_radius"), "Sled pivot radius", "mm", 1000, 0.5, "openscad: rod-end centre from the collar axis."),
    (("sled", "pivot_height"), "Sled pivot height", "mm", 1000, 0.5, "openscad: rod-end centre above the collar bottom."),
    (("sled", "half_width"), "Sled half width", "mm", 1000, 0.5, "openscad: sled_r, for the sled-to-sled collision boxes."),
    (("gears", "module"), "Gear module", "mm", 1000, 0.1, "openscad: robbie_common.scad."),
    (("gears", "ring_teeth"), "Ring teeth", "", None, 1, "openscad: robbie_common.scad."),
    (("gears", "pinion_teeth"), "Pinion teeth", "", None, 1, "openscad: robbie_common.scad."),
]
MEASURED = [
    (("platform", "pivot_radius"), "Platform pivot radius", "mm", 1000, 0.5,
     "Rod-end centre from the platform centre. Measure it."),
    (("platform", "radius"), "Platform radius", "mm", 1000, 0.5, "For drawing only."),
    (("springs", "from_top"), "Spring mount", "mm", 1000, 0.5, "Spring mount point, along the rod from the top pivot."),
    (("springs", "free_length"), "Spring free length", "mm", 1000, 0.5,
     "Unstretched length. The springs only pull once stretched past this."),
]
OPTIONAL = {("rod_end", "max_angle_deg"), ("platform", "measured_height")}   # blank in neck.yaml is fine
README_CHARTS = [   # (title, folder, file name after the NN_ number), the charts README.md shows
    ("Pitch by head orientation, both twists", "this_robbie", "pitch_by_head_orientation"),
    ("Turned by hand to the other twist", "this_robbie", "render_turned_by_hand"),
    ("Solutions per roll/pitch target", "this_robbie", "solution_count"),
    ("Least off centre", "this_robbie", "min_off"),
    ("Off-centre spread", "this_robbie", "off_spread"),
    ("Roll/pitch coverage by rod length", "ik_sweep", "roll_pitch_coverage_by_rod"),
    ("Trade-off vs rod length", "ik_sweep", "tradeoff_vs_rod_len"),
    ("Roll/pitch coverage by jam", "ik_sweep", "roll_pitch_coverage_by_jam"),
    ("Trade-off vs jam", "ik_sweep", "tradeoff_vs_jam"),
]


# ------------------------------------------------------------------------------------------- neck.yaml
def get(cfg, path):
    for k in path:
        cfg = cfg[k]
    return cfg


def put(cfg, path, v):
    for k in path[:-1]:
        cfg = cfg[k]
    cfg[path[-1]] = v


def shown(v, scale):
    """A neck.yaml value as the field shows it."""
    return "" if v is None else str(v) if scale is None else f"{round(v * scale, 4):g}"


def parsed(text, field):
    """A field's text as the neck.yaml value; ValueError if it doesn't parse."""
    path, label, unit, scale, *_ = field
    if not text.strip() and tuple(path) in OPTIONAL:
        return None
    return int(text) if scale is None else float(text) / scale


def differs(a, b):
    return (a is None) != (b is None) or (a is not None and abs(a - b) > 1e-9)


def yaml_text(v, unit):
    if v is None:
        return ""
    if unit != "mm":
        return f"{v:g}"
    s = f"{v:.6f}".rstrip("0")
    return s + "0" * max(0, 3 - len(s.split(".")[1]))   # 0.07 -> 0.070, as neck.yaml writes them


def yaml_set(text, path, value):
    """text with path's value replaced, keeping the line's comment where it was."""
    sec = None
    lines = text.split("\n")
    for i, ln in enumerate(lines):
        m = re.match(r"^( *)(\w+):( *)([^#]*?)( *)(#.*)?$", ln)
        if not m:
            continue
        indent, k, gap, old, pad, comment = m.groups()
        if not indent:
            sec = k
        here = (k,) if not indent else (sec, k)
        if here == tuple(path) and (old or indent):   # a top-level key with no value is a section
            new = f"{indent}{k}: {value}".rstrip()
            lines[i] = (new.ljust(ln.index("#") - 1) + " " + comment) if comment else new   # comment stays put
            return "\n".join(lines)
    raise KeyError(".".join(path))


# ------------------------------------------------------------------------------------------- analysis
def table_height(cfg):
    """Table to a top rod-end centre (m), at rest."""
    return S.pivot_z(cfg) + cfg["platform"]["rest_height"]


def sweep_path(cfg):
    return os.path.join(CACHE, S.key("sweep_a", cfg) + ".csv")


def load_sweep(cfg):
    p = sweep_path(cfg)
    if not os.path.exists(p):
        return None
    with open(p[:-4] + ".json") as f:
        ck = json.load(f)["checks"]
    a = np.loadtxt(p, delimiter=",", skiprows=1, ndmin=2)
    return common.graceful_ugly(common.to_frame(a, ck["head_zero_deg"])), ck   # by head.rotation_deg


METHODS = ("nod line (up)", "nod line (down)", "best compromise")   # how Find best picks the head rotation
UP_POLE, DOWN_POLE = "CW", "CCW"   # the convention: the head is set up so that twisted CW it looks up, twisted CCW down


def head_lines(cfg, method):
    """The three head.rotation_deg (120 deg apart) Find best offers, by `method`. CW is the twist that looks up and CCW
    the one that looks down; the two are mirror images, so that is a convention, and robbie_ros defaults to CW.
    nod line (up): the up ends of CW's nod lines, jam/2 from a rod end. Twisted CW the head looks up as far as it can;
    twisted CCW it looks down, a little short of its best.
    nod line (down): the down ends of CCW's nod lines, 60 - jam/2 from a rod end. The reverse: twisted CCW the head
    looks down as far as it can, twisted CW it looks up a little short of its best.
    best compromise: CW looks up and CCW looks down about equally. Halfway between the two, so always 30 deg from a
    rod end whatever the geometry. None needs the sim."""
    if method == "nod line (up)":
        return S.nod_lines(cfg, UP_POLE)
    if method == "nod line (down)":
        return [(n + 60 + 180) % 360 - 180 for n in S.nod_lines(cfg, DOWN_POLE)]
    return [(30 + 120 * k + 180) % 360 - 180 for k in range(3)]


def nearest(lines, rot):
    return min(lines, key=lambda n: abs((n - rot + 180) % 360 - 180))


def call(gu, deg):
    up, down = gu["graceful", "up"][int(round(deg)) % 360], gu["graceful", "down"][int(round(deg)) % 360]
    return "up" if up > down + 0.1 else "down" if down > up + 0.1 else "even"


def runs(cfg, sure=False):
    """{(pole, jam step): cfg} for the sims the pitch tab needs."""
    out = {}
    for pole in ("CW", "CCW"):
        c = S.with_twist(cfg, pole)
        out[pole, 0] = c
        if sure:
            for d in JAM_STEPS:
                out[pole, d] = S.with_jam(c, S.jam_of(c) + d)
    return out


def checks(cfg, notes, sims):
    """[(state, short text, tooltip)]."""
    out = []
    try:
        jam = S.jam_of(cfg)
    except (ValueError, ZeroDivisionError):
        return [(BAD, "Rods don't fit", "Rod length, the pivot radii and the rest height can't make a neck:\n"
                 "the rod is too short or too long to span them.")]
    out.append((OK, f"Rods fit: jam {jam:.1f}°", "The rest height and rod length give this twist at rest."))
    lo, hi = STUDIED_JAMS
    out.append((OK, f"Jam within {lo}–{hi}°", "The range just plot compares.") if lo <= jam <= hi else
               (WARN, f"Jam {jam:.1f}° outside {lo}–{hi}°", "Outside the range studied; the trade-off charts won't cover it."))
    bottom, top = S.rod_end_angles(cfg)
    spec = notes.get("rod_end_max_deg")
    if spec is None:
        out.append((WARN, "Rod-end spec not entered", f"Rod ends at rest: bottom {bottom:.1f}°, top {top:.1f}°.\n"
                    "Enter the spec sheet's max angle to check them."))
    else:
        worst = max(bottom, top)
        tip = (f"Rod ends at rest: bottom {bottom:.1f}°, top {top:.1f}°; spec {spec:g}°.\n"
               "The rod ends jam at the spec angle, so the larger of the two should be close to it.")
        if worst > spec + 1:
            out.append((BAD, f"Rod ends past spec ({worst:.1f}° > {spec:g}°)", tip + "\nRest height too low, or rods too short."))
        elif worst < spec - 1:
            out.append((WARN, f"Rod ends short of spec ({worst:.1f}° < {spec:g}°)", tip +
                        "\nThe real neck would twist further before jamming: is the rest height a bit high?"))
        else:
            out.append((OK, f"Rod ends at spec ({worst:.1f}°)", tip))
    meas = notes.get("measured_height_mm")
    if meas is not None:
        h = table_height(cfg) * 1000
        tip = f"Model: {h:.1f} mm table to top rod-end centre. You measured {meas:g} mm."
        out.append((OK, "Measured height matches", tip) if abs(h - meas) <= 1 else
                   (WARN, f"Measured height off by {meas - h:+.1f} mm", tip + "\nWithin 1 mm counts as a match."))
    s, free = S.spring_rest_len(cfg) * 1000, cfg["springs"]["free_length"] * 1000
    out.append((OK, f"Springs stretched at rest ({s:.1f} mm)", f"Free length {free:g} mm.") if s > free else
               (BAD, "Springs slack at rest", f"Mounts {s:.1f} mm apart, free length {free:g} mm.\n"
                "Slack springs can't hold the neck in its pole."))
    if not all((p, 0) in sims for p in ("CW", "CCW")):
        out.append((TODO, "Sim not run for these values", "Look up / down tab: Run sim."))
    else:
        bad = [p for p in ("CW", "CCW") if sims[p, 0][1]["direction"] != p]
        out.append((OK, "Sim rests in both poles", "Twisted CW or CCW, the sim settles level in that pole.") if not bad else
                   (BAD, f"Sim won't rest in {', '.join(bad)}", "It settled in the other pole: the springs or jam can't hold it."))
    return out


# ------------------------------------------------------------------------------------------- drawing
def at(deg, r):
    """Head-frame angle (0 = rod end 1, CCW) and radius to plot x, y with rod end 1 up, as docs/head_rotation.svg."""
    a = np.radians(deg)
    return -r * np.sin(a), r * np.cos(a)


def draw_layout(fig, cfg, pole, notes):
    fig.clear()
    top, side = fig.subplots(1, 2, gridspec_kw=dict(width_ratios=[1.25, 1]))
    try:
        jam = S.jam_of(cfg)
    except (ValueError, ZeroDivisionError):
        top.text(0.5, 0.5, "Rods don't fit", ha="center", transform=top.transAxes, color=COLOUR[BAD], fontsize=14)
        top.axis("off"); side.axis("off")
        return
    twist = jam if pole == "CCW" else -jam
    rb, rp = cfg["sled"]["pivot_radius"] * 1000, cfg["platform"]["pivot_radius"] * 1000
    starts = cfg["sled"]["start_deg"]

    # top view, in the head frame
    t = np.linspace(0, 360, 200)
    top.plot(*at(t, rb), color="#bbbbbb", ls=":", lw=1)
    corner = (rp - HEX_INSET * 1000) / math.cos(math.radians(30))
    hx = [at(30 + 60 * k, corner) for k in range(7)]
    top.fill([p[0] for p in hx], [p[1] for p in hx], fc="#eef2f7", ec="#445566", lw=1.5)
    top.plot(*at(t, cfg["platform"]["radius"] * 1000), color="#cccccc", lw=0.8)
    for k in range(3):
        end, sled = at(120 * k, rp), at(starts[k] - starts[0] - twist, rb)
        top.plot([end[0], sled[0]], [end[1], sled[1]], color="#888888", ls="--", lw=1.2)
        top.plot(*sled, "o", mfc="white", mec="#888888", ms=6)
        top.annotate(f"sled {k + 1}", sled, xytext=(6, -10), textcoords="offset points", fontsize=7, color="#777777")
        top.plot(*end, "o", color="#1f6fb2", ms=8)
        top.annotate(f"rod end {k + 1}", end, xytext=(6, 4), textcoords="offset points", fontsize=7, weight="bold")
    R = rb * 1.25
    rot = cfg["head"]["rotation_deg"]
    for n in S.nod_lines(cfg, pole):
        top.plot(*zip((0, 0), at(n, R)), color=NOD, ls=":", lw=1.5)
    top.annotate("nod", at(S.nod_lines(cfg, pole)[0], R), fontsize=7, color=NOD)
    top.annotate("", at(0, rp * 0.9), (0, 0), arrowprops=dict(arrowstyle="-|>", color="#c0392b", lw=2))
    top.text(*at(-4, rp * 0.55), "0°", color="#c0392b", fontsize=8, weight="bold")
    arc = np.linspace(rot + 15, rot + 45, 30)   # + direction, just past the head
    top.plot(*at(arc, R * 1.1), color="#1f6fb2", lw=1.5)
    top.annotate("", at(rot + 47, R * 1.1), at(rot + 43, R * 1.1), arrowprops=dict(arrowstyle="-|>", color="#1f6fb2"))
    top.text(*at(rot + 30, R * 1.2), "+", color="#1f6fb2", fontsize=12, weight="bold", ha="center", va="center")
    top.annotate("", at(rot, rp * 1.15), (0, 0), arrowprops=dict(arrowstyle="-|>", color=HEAD, lw=2.5))
    top.annotate(f"head {rot:g}°", at(rot, rp * 1.2), fontsize=8, color=HEAD, weight="bold", ha="center")
    top.set_aspect("equal"); top.axis("off")
    top.set_xlim(-R * 1.3, R * 1.3); top.set_ylim(-R * 1.3, R * 1.3)
    # in the axes, just above the drawing: the square axes leave a gap above it for set_title
    top.text(0, R * 1.02, f"From above, twisted {pole}: jam {jam:.1f}°", fontsize=9, ha="center", va="bottom")

    # side view: what to measure
    z_c, z_p, h = cfg["collar_z"] * 1000, S.pivot_z(cfg) * 1000, cfg["platform"]["rest_height"] * 1000
    z_t = z_p + h
    chord = math.sqrt(max(0, (cfg["rod"]["length"] * 1000) ** 2 - h * h))
    x0, x1 = -chord / 2, chord / 2
    hw = cfg["sled"]["half_width"] * 1000
    # base, collar and the sled under rod end 1, sized as gen_neck_mjcf.py draws them; the taper is for looks
    side.add_patch(Polygon([(-BASE_R, 0), (BASE_R, 0), (BASE_R, z_c * 0.6), (COLLAR_R, z_c), (-COLLAR_R, z_c),
                            (-BASE_R, z_c * 0.6)], fc="#b0b0b0", ec="#808080"))
    side.add_patch(Rectangle((-COLLAR_R, z_c), 2 * COLLAR_R, COLLAR_H, fc=PART, ec="#1f2f99"))
    side.add_patch(Rectangle((x0 - hw, z_c + COLLAR_H), 2 * hw, max(1, z_p - z_c - COLLAR_H - 4), fc=PART, ec="#1f2f99"))
    side.add_patch(Rectangle((-rp - 6, z_t - 4), 2 * rp + 12, 8, fc="#eef2f7", ec="#445566"))
    side.plot([x0, x1], [z_p, z_t], color="#888888", lw=3)
    side.plot([x0, x1], [z_p, z_t], "o", color="#1f6fb2", mec="white", ms=8)
    side.axhline(0, color="#444444", lw=2)

    def dim(x, z0, z1, text, colour, lw=1.2, left=False):
        side.annotate("", (x, z1), (x, z0), arrowprops=dict(arrowstyle="<|-|>", color=colour, lw=lw))
        side.text(x + (-3 if left else 3), (z0 + z1) / 2, text, color=colour, fontsize=8, va="center",
                  ha="right" if left else "left")

    dim(-BASE_R - 4, 0, z_c, f"base\n{z_c:g}", "#555555", left=True)
    dim(-BASE_R - 4, z_c, z_p, f"pivot\n{z_p - z_c:g}", "#555555", left=True)
    dim(-BASE_R - 4, z_p, z_t, f"rest height\n{h:.1f}", "#222222", left=True)
    xm = max(BASE_R, rp + 6) + 6   # clear of the base and platform
    side.plot([-BASE_R - 4, x0], [z_p, z_p], color="#bbbbbb", lw=0.8, ls=":")
    side.plot([-BASE_R - 4, xm], [z_t, z_t], color="#bbbbbb", lw=0.8, ls=":")
    dim(xm, 0, z_t, f"measure this\n{z_t:.1f} mm", "#c0392b", 2)
    meas = notes.get("measured_height_mm")
    if meas is not None:
        side.plot([xm - 6, xm + 6], [meas, meas], color="#c0392b", lw=1, ls="--")
        side.text(xm + 3, meas - 6, f"measured {meas:g}", color="#c0392b", fontsize=7, va="center")
    side.text(0, -6, "table", ha="center", va="top", fontsize=8, color="#444444")
    side.set_aspect("equal"); side.axis("off")
    side.set_xlim(-BASE_R - 40, xm + 65); side.set_ylim(-14, z_t + 14)
    side.set_title("From the side (one rod, at rest)", fontsize=9)


def draw_pitch(fig, cfg, sims):
    fig.clear()
    axes = fig.subplots(1, 2, subplot_kw=dict(projection="polar"))
    rot = cfg["head"]["rotation_deg"]
    th = np.radians(common.HEADS_DEG)
    top = max(max(sims[p, 0][0]["ugly", w].max() for w in ("up", "down")) for p in ("CW", "CCW")) * 1.1
    for ax, pole in zip(axes, ("CW", "CCW")):
        gu = sims[pole, 0][0]
        for w, colour in (("up", UP), ("down", DOWN)):
            ax.plot(th, gu["graceful", w], color=colour, lw=2, label=f"{w}, graceful")
            ax.plot(th, gu["ugly", w], color=colour, lw=1, ls="--", label=f"{w}, ugly")
        for n in S.nod_lines(cfg, pole):
            ax.plot([math.radians(n)] * 2, [0, top], color=NOD, ls=":", lw=1.5)
        ax.plot([math.radians(rot)] * 2, [0, top], color=HEAD, lw=2.5)
        ax.set_theta_zero_location("N"); ax.set_ylim(0, top)
        ax.set_rlabel_position(200); ax.tick_params(labelsize=7)
        ax.set_title(f"Twisted {pole}", fontsize=10, pad=12)
    axes[1].legend(loc="upper left", bbox_to_anchor=(1.05, 1.0), fontsize=8, frameon=False)
    fig.text(0.01, 0.01, "Angle: head rotation (0 = rod end 1, CCW from above).  Distance: largest pitch (deg).  "
             "Green: your head.  Gold: nod lines (up ends).", fontsize=8, color="#555555")


# ------------------------------------------------------------------------------------------- widgets
class Tip:
    """Tooltip; text can be a function, read when it shows."""

    def __init__(self, widget, text):
        self.w, self.text, self.win, self.job = widget, text, None, None
        widget.bind("<Enter>", lambda e: self._later(), add="+")
        widget.bind("<Leave>", lambda e: self.hide(), add="+")
        widget.bind("<ButtonPress>", lambda e: self.hide(), add="+")

    def _later(self):
        self.job = self.w.after(450, self.show)

    def show(self):
        text = self.text() if callable(self.text) else self.text
        if not text:
            return
        self.win = tk.Toplevel(self.w); self.win.wm_overrideredirect(True)
        self.win.wm_geometry(f"+{self.w.winfo_rootx() + 12}+{self.w.winfo_rooty() + self.w.winfo_height() + 4}")
        tk.Label(self.win, text=text, justify="left", bg="#ffffe0", relief="solid", bd=1, padx=6, pady=4,
                 font=("TkDefaultFont", 9)).pack()

    def hide(self):
        if self.job:
            self.w.after_cancel(self.job); self.job = None
        if self.win:
            self.win.destroy(); self.win = None


def mark(parent, state, size):
    """A tick, warning, cross or pending dot, drawn so it doesn't depend on the font having the glyph."""
    c = tk.Canvas(parent, width=size, height=size, highlightthickness=0, bg=ttk.Style().lookup("TFrame", "background"))
    k, w, col = size / 10, max(2, size / 7), COLOUR[state]
    if state == OK:
        c.create_line(1.5 * k, 5.5 * k, 4 * k, 8 * k, 8.5 * k, 2 * k, fill=col, width=w, capstyle="round", joinstyle="round")
    elif state == BAD:
        c.create_line(2 * k, 2 * k, 8 * k, 8 * k, fill=col, width=w, capstyle="round")
        c.create_line(8 * k, 2 * k, 2 * k, 8 * k, fill=col, width=w, capstyle="round")
    elif state == WARN:
        c.create_polygon(5 * k, 1 * k, 9.5 * k, 9 * k, 0.5 * k, 9 * k, fill=col)
        c.create_line(5 * k, 3.8 * k, 5 * k, 6.3 * k, fill="white", width=max(1, w / 2))
        c.create_oval(4.6 * k, 7.2 * k, 5.4 * k, 8 * k, fill="white", outline="white")
    else:
        c.create_oval(3 * k, 3 * k, 7 * k, 7 * k, outline=col, width=max(1, w / 2))
    return c


def open_path(path):
    if os.path.exists("/mnt/wslg") or "microsoft" in os.uname().release.lower():
        subprocess.Popen(["explorer.exe", subprocess.check_output(["wslpath", "-w", path], text=True).strip()])
    else:
        subprocess.Popen(["xdg-open", path])


class App:
    def __init__(self, root):
        self.root = root
        self.q = queue.Queue()
        self.busy = False
        self.sims = {}
        self.loaded = {}   # sweep path -> (graceful_ugly, checks)
        self.vars = {}
        self.pending = None
        with open(S.NECK) as f:
            self.text = f.read()
        self.saved = S.load_neck()
        try:
            with open(STATE) as f:
                self.notes = json.load(f)
        except (OSError, ValueError):
            self.notes = {}
        self.pole = tk.StringVar(value=self.notes.get("pole", UP_POLE))
        self.build()
        self.refresh()
        root.after(100, self.poll)
        root.protocol("WM_DELETE_WINDOW", self.close)

    # ---------------------------------------------------------------- layout
    def build(self):
        r = self.root
        r.title("Robbie neck setup"); r.geometry("1450x900")
        left = ttk.Frame(r, padding=8); left.pack(side="left", fill="y")
        right = ttk.Frame(r, padding=(0, 8, 8, 8)); right.pack(side="left", fill="both", expand=True)

        g = ttk.LabelFrame(left, text="Your Robbie", padding=6); g.pack(fill="x")
        for f in MAIN:
            self.field(g, f)
        row = ttk.Frame(g); row.pack(fill="x", pady=(2, 4))
        ttk.Label(row, text="Rest height from:").pack(side="left")
        b = ttk.Button(row, text="spec", width=9, command=self.from_spec); b.pack(side="left", padx=2)
        Tip(b, "Set the rest height to where the rod ends reach their spec angle: where they jam.")
        b = ttk.Button(row, text="measured", width=9, command=self.from_measured); b.pack(side="left", padx=2)
        Tip(b, "Set the rest height from your measured table height, less the collar and sled pivot heights.")
        row = ttk.Frame(g); row.pack(fill="x", pady=(0, 4))
        ttk.Label(row, text="Head:").pack(side="left")
        st = ttk.Style()
        st.configure("Best.TButton", background="#cdeccd")
        st.map("Best.TButton", background=[("active", "#b5e2b5")])
        b = ttk.Button(row, text="Find best", style="Best.TButton", command=self.to_best); b.pack(side="left", padx=2)
        Tip(b, self.best_tip)
        self.method = tk.StringVar(value=METHODS[0])
        cb = ttk.Combobox(row, textvariable=self.method, values=METHODS, state="readonly", width=16); cb.pack(side="left", padx=2)
        Tip(cb, "How Find best picks the head rotation. Twisted CW the head looks up, twisted CCW it looks down: the convention.\n"
                "nod line (up): the up end of the nearest CW nod line. Twisted CW it looks up as far as it can; twisted CCW it\n"
                "looks down, a little short of its best.\n"
                "nod line (down): the down end of the nearest CCW nod line. The reverse: CCW looks down as far as it can.\n"
                "best compromise: CW looks up and CCW looks down about equally; a few degrees off either nod line.")
        row = ttk.Frame(g); row.pack(fill="x")
        ttk.Label(row, text="Drawn twisted:").pack(side="left")
        for pole in ("CW", "CCW"):
            b = ttk.Radiobutton(row, text=pole, value=pole, variable=self.pole, command=self.changed); b.pack(side="left", padx=4)
            Tip(b, "Which pole the drawings show, seen from above. CW is the pole that looks up and the default;\n"
                   "the owner can turn the neck into either by hand, so it's set at runtime (just up --ccw, just twist on the Pi).")

        for title, fields, tip in (("openscad values", OPENSCAD, "Change these if you changed them in robbie_openscad."),
                                   ("Other measurements", MEASURED, None)):
            self.fold(left, title, fields, tip)

        row = ttk.Frame(left); row.pack(fill="x", pady=8)
        self.save_b = ttk.Button(row, text="Save neck.yaml", command=self.save); self.save_b.pack(side="left")
        Tip(self.save_b, lambda: "Writes these changes to neck.yaml (comments kept):\n" + "\n".join(self.diff())
            if self.diff() else "No changes.")
        b = ttk.Button(row, text="Revert", command=self.revert); b.pack(side="left", padx=4)
        Tip(b, "Back to neck.yaml as saved.")

        c = ttk.LabelFrame(left, text="Checks", padding=6); c.pack(fill="both", expand=True)
        self.check_rows = ttk.Frame(c); self.check_rows.pack(fill="x")

        self.nb = nb = ttk.Notebook(right); nb.pack(fill="both", expand=True)
        self.layout_fig = Figure(figsize=(9, 5), constrained_layout=True)
        t1 = ttk.Frame(nb); nb.add(t1, text="Layout")
        self.layout_canvas = FigureCanvasTkAgg(self.layout_fig, t1); self.layout_canvas.get_tk_widget().pack(fill="both", expand=True)

        t2 = ttk.Frame(nb, padding=6); nb.add(t2, text="Look up / down")
        row = ttk.Frame(t2); row.pack(fill="x")
        self.sim_b = ttk.Button(row, text="Run sim", command=lambda: self.run_sims(False)); self.sim_b.pack(side="left")
        Tip(self.sim_b, "Sweeps the sled range in the sim, twisted CW and CCW: about 2 minutes. Cached, so only once per neck.")
        self.sure_b = ttk.Button(row, text="How sure?", command=lambda: self.run_sims(True)); self.sure_b.pack(side="left", padx=4)
        Tip(self.sure_b, f"Re-runs with the jam {min(JAM_STEPS)}° to +{max(JAM_STEPS)}° out, to find how wrong your measurements\n"
                         "can be before up and down swap. About 3 minutes.")
        self.prog = ttk.Progressbar(row, length=200); self.prog.pack(side="left", padx=8)
        self.status = ttk.Label(row, foreground="#555555"); self.status.pack(side="left")
        self.verdict = ttk.Label(t2, font=("TkDefaultFont", 12), justify="left"); self.verdict.pack(anchor="w", pady=6)
        self.pitch_fig = Figure(figsize=(9, 4.5), constrained_layout=True)
        self.pitch_canvas = FigureCanvasTkAgg(self.pitch_fig, t2); self.pitch_canvas.get_tk_widget().pack(fill="both", expand=True)

        t3 = ttk.Frame(nb, padding=6); nb.add(t3, text="Charts")
        row = ttk.Frame(t3); row.pack(fill="x")
        self.plot_b = ttk.Button(row, text="Run just plot", command=self.run_plot); self.plot_b.pack(side="left")
        Tip(self.plot_b, lambda: "Save neck.yaml first: just plot reads it." if self.diff() else
            "Every chart for this neck, into .plots/. Tens of minutes the first time; cached after.")
        self.chart = tk.StringVar()
        cb = ttk.Combobox(row, textvariable=self.chart, values=[c[0] for c in README_CHARTS], state="readonly", width=40)
        cb.pack(side="left", padx=8); cb.bind("<<ComboboxSelected>>", lambda e: self.show_chart())
        Tip(cb, "The charts README.md explains, for your neck. All of them are in .plots/plots.md.")
        b = ttk.Button(row, text="Open plots folder", command=lambda: open_path(PLOTS)); b.pack(side="left")
        self.plot_status = ttk.Label(row, foreground="#555555"); self.plot_status.pack(side="left", padx=8)
        self.chart_fig = Figure(figsize=(9, 6), constrained_layout=True)
        self.chart_canvas = FigureCanvasTkAgg(self.chart_fig, t3); self.chart_canvas.get_tk_widget().pack(fill="both", expand=True)
        self.chart.set(README_CHARTS[0][0]); self.show_chart()

    def field(self, parent, f):
        path, label, unit, scale, step, tip = f
        row = ttk.Frame(parent); row.pack(fill="x", pady=1)
        lab = ttk.Label(row, text=label, width=20); lab.pack(side="left")
        var = tk.StringVar(value=shown(get(self.saved, path), scale))
        sb = ttk.Spinbox(row, textvariable=var, from_=-1e6, to=1e6, increment=step, width=9); sb.pack(side="left")
        ttk.Label(row, text=unit, width=4).pack(side="left")
        for w in (lab, sb):
            Tip(w, tip)
        var.trace_add("write", lambda *_: self.changed())
        self.vars[label] = (var, f, sb)

    def fold(self, parent, title, fields, tip):
        box = ttk.Frame(parent); box.pack(fill="x", pady=(6, 0))
        inner = ttk.Frame(box, padding=(12, 0, 0, 0))
        shown = tk.BooleanVar(value=False)

        def toggle():
            shown.set(not shown.get())
            b.config(text=("▾ " if shown.get() else "▸ ") + title)
            inner.pack(fill="x") if shown.get() else inner.pack_forget()
        b = ttk.Button(box, text="▸ " + title, command=toggle, style="Toolbutton"); b.pack(anchor="w")
        if tip:
            Tip(b, tip)
        for f in fields:
            self.field(inner, f)

    # ---------------------------------------------------------------- values
    def cfg(self):
        """neck.yaml as edited, or None while a field doesn't parse."""
        c = copy.deepcopy(self.saved)
        for var, f, sb in self.vars.values():
            try:
                put(c, f[0], parsed(var.get(), f))
            except ValueError:
                return None
        return c

    def read_notes(self):
        """The optional measurements, for checks and the drawing: {"rod_end_max_deg", "measured_height_mm"}."""
        c, out = self.cfg() or self.saved, {}
        if c["rod_end"].get("max_angle_deg") is not None:
            out["rod_end_max_deg"] = c["rod_end"]["max_angle_deg"]
        if c["platform"].get("measured_height") is not None:
            out["measured_height_mm"] = c["platform"]["measured_height"] * 1000
        return out

    def set_field(self, label, value):
        self.vars[label][0].set(f"{value:g}")

    def diff(self):
        c = self.cfg()
        if c is None:
            return []
        out = []
        for var, (path, label, unit, scale, *_), sb in self.vars.values():
            if differs(get(c, path), get(self.saved, path)):
                out.append(f"  {'.'.join(path)}: {yaml_text(get(self.saved, path), unit)} → {yaml_text(get(c, path), unit)}")
        return out

    def changed(self):
        if self.pending:
            self.root.after_cancel(self.pending)
        self.pending = self.root.after(250, self.refresh)

    def refresh(self):
        self.pending = None
        c, notes = self.cfg(), self.read_notes()
        for var, f, sb in self.vars.values():   # mark fields that don't parse
            try:
                parsed(var.get(), f)
                sb.state(["!invalid"])
            except ValueError:
                sb.state(["invalid"])
        self.notes["pole"] = self.pole.get()
        try:
            os.makedirs(PLOTS, exist_ok=True)
            with open(STATE, "w") as f:
                json.dump(self.notes, f, indent=1)
        except OSError:
            pass
        self.root.title("Robbie neck setup" + (" *" if self.diff() else ""))
        self.save_b.state(["!disabled"] if self.diff() else ["disabled"])
        if c is None:
            return
        self.sims = {}
        try:
            for k, rc in runs(c, sure=True).items():
                p = sweep_path(rc)
                if p not in self.loaded:
                    got = load_sweep(rc)
                    if not got:
                        continue
                    self.loaded[p] = got
                self.sims[k] = self.loaded[p]
        except (ValueError, ZeroDivisionError):   # rods don't fit; checks() says so
            pass
        draw_layout(self.layout_fig, c, self.pole.get(), notes)
        self.layout_canvas.draw_idle()
        self.show_checks(checks(c, notes, self.sims))
        self.show_pitch(c)

    def show_checks(self, rows):
        for w in self.check_rows.winfo_children():
            w.destroy()
        for state, text, tip in rows:
            row = ttk.Frame(self.check_rows); row.pack(fill="x", anchor="w", pady=1)
            mark(row, state, 14).pack(side="left", padx=(0, 6))
            lab = ttk.Label(row, text=text); lab.pack(side="left")
            Tip(lab, tip)

    def show_pitch(self, c):
        busy = self.busy
        have = all((p, 0) in self.sims for p in ("CW", "CCW"))
        sure = all(k in self.sims for k in runs(c, sure=True))
        self.sim_b.state(["disabled"] if busy or have else ["!disabled"])
        self.sure_b.state(["disabled"] if busy or sure else ["!disabled"])
        if not have:
            self.pitch_fig.clear(); self.pitch_canvas.draw_idle()
            if not busy:
                self.verdict.config(text="Not run for these values yet. Run sim to see which twist looks up.")
            return
        rot = c["head"]["rotation_deg"]
        lines = [f"With the head at {rot:g}°:"]
        for p in ("CW", "CCW"):
            gu = self.sims[p, 0][0]; i = int(round(rot)) % 360
            up, down = gu["graceful", "up"][i], gu["graceful", "down"][i]
            way = {"up": "looks UP better", "down": "looks DOWN better", "even": "looks up and down about the same"}[call(gu, rot)]
            lines.append(f"    twisted {p:<3}  {way}   (graceful up {up:.1f}°, down {down:.1f}°;  "
                         f"ugly up {gu['ugly', 'up'][i]:.1f}°, down {gu['ugly', 'down'][i]:.1f}°)")
        lines.append(self.sure_text(c) if sure else "How sure? not run yet: it says how far off your measurements can be "
                                                    "before up and down swap.")
        self.verdict.config(text="\n".join(lines))
        draw_pitch(self.pitch_fig, c, self.sims)
        self.pitch_canvas.draw_idle()

    def sure_text(self, c):
        rot = c["head"]["rotation_deg"]
        base = {p: call(self.sims[p, 0][0], rot) for p in ("CW", "CCW")}
        holds = lambda d: all(call(self.sims[p, d][0], rot) == base[p] for p in ("CW", "CCW"))
        jam = S.jam_of(c)

        def reach(steps):   # (last step that holds, first that flips or None)
            last = 0
            for d in steps:
                if not holds(d):
                    return last, d
                last = d
            return last, None
        lo, hi = reach(sorted([d for d in JAM_STEPS if d < 0], reverse=True)), reach(sorted(d for d in JAM_STEPS if d > 0))
        mm = lambda d: table_height(S.with_jam(c, jam + d)) * 1000

        def end(r):
            last, flip = r
            return f"{last:+d}°" + (f" (swaps by {flip:+d}°)" if flip is not None else " (no swap seen)")
        rows = [f"How sure: up and down hold for a jam {end(lo)} to {end(hi)} from yours.",
                f"    That's a table-to-rod-end height of {mm(hi[0]):.1f} to {mm(lo[0]):.1f} mm (yours: {mm(0):.1f} mm)."]
        a = next((d for d in range(1, 181) if any(call(self.sims[p, 0][0], rot + d) != base[p] for p in ("CW", "CCW"))), 180)
        b = next((d for d in range(1, 181) if any(call(self.sims[p, 0][0], rot - d) != base[p] for p in ("CW", "CCW"))), 180)
        rows.append(f"    Head rotation can go from {rot - b + 1:g}° to {rot + a - 1:g}° before they swap.")
        return "\n".join(rows)

    # ---------------------------------------------------------------- actions
    def from_spec(self):
        c, spec = self.cfg(), self.read_notes().get("rod_end_max_deg")
        if c is None or spec is None:
            return messagebox.showinfo("Rest height", "Enter the rod-end max angle first.")
        jam = S.jam_for_rod_end(c, spec)
        if jam is None:
            return messagebox.showwarning("Rest height", f"No rest height puts the rod ends at {spec:g}° with these rods.")
        self.set_field("Rest height", round(S.with_jam(c, jam)["platform"]["rest_height"] * 1000, 2))

    def from_measured(self):
        c, meas = self.cfg(), self.read_notes().get("measured_height_mm")
        if c is None or meas is None:
            return messagebox.showinfo("Rest height", "Enter the measured height first.")
        self.set_field("Rest height", round(meas - S.pivot_z(c) * 1000, 2))

    def have_sims(self):
        return all((p, 0) in self.sims for p in ("CW", "CCW"))

    def best_tip(self):
        c, method = self.cfg(), self.method.get()
        if c is None:
            return "Turn the head to where twisted CW looks up and twisted CCW looks down. Fix the values first."
        try:
            lines = head_lines(c, method)
        except (ValueError, ZeroDivisionError):
            return "Turn the head to where twisted CW looks up and twisted CCW looks down. The rods don't fit these values."
        d = nearest(lines, c["head"]["rotation_deg"])
        text = {"nod line (up)": f"Turn the head to {d:.1f}°, the up end of the nearest CW nod line. Twisted CW it looks up as "
                                 "far as it can; twisted CCW it looks down, a little short of its best.\n",
                "nod line (down)": f"Turn the head to {d:.1f}°, the down end of the nearest CCW nod line. Twisted CCW it looks "
                                   "down as far as it can; twisted CW it looks up, a little short of its best.\n",
                "best compromise": f"Turn the head to {d:.1f}°, where twisted CW looks up and twisted CCW looks down about "
                                   "equally.\n"}[method]
        text += "The other two, 120° round, are the same: " + ", ".join(f"{n:.1f}°" for n in sorted(lines) if n != d) + "."
        if self.have_sims():
            i = int(round(d)) % 360
            text += "\nThere, graceful: " + ";  ".join(
                f"twisted {p} up {self.sims[p, 0][0]['graceful', 'up'][i]:.1f}°, down {self.sims[p, 0][0]['graceful', 'down'][i]:.1f}°"
                for p in (UP_POLE, DOWN_POLE))
        return text

    def to_best(self):
        c = self.cfg()
        if c is None:
            return
        try:
            lines = head_lines(c, self.method.get())
        except (ValueError, ZeroDivisionError):
            return messagebox.showwarning("Head", "The rods don't fit these values, so there are no nod lines.")
        self.set_field("Head rotation", round(nearest(lines, c["head"]["rotation_deg"]), 1))

    def run_sims(self, sure):
        c = self.cfg()
        if c is None or self.busy:
            return
        self.busy = True
        self.show_pitch(c)
        jobs = {k: (rc, "a") for k, rc in runs(c, sure).items()}

        def work():
            try:
                S.ensure_sweeps(CACHE, jobs, procs=os.cpu_count() or 4,
                                log=lambda text, done, total: self.q.put(("sim", done, total)),
                                pool=multiprocessing.get_context("spawn").Pool)
                self.q.put(("sim_done", None, None))
            except Exception as e:
                self.q.put(("sim_fail", str(e), None))
        self.status.config(text="starting the sim…"); self.prog.config(value=0)
        threading.Thread(target=work, daemon=True).start()

    def run_plot(self):
        if self.diff():
            return messagebox.showinfo("just plot", "Save neck.yaml first: just plot reads it.")
        if self.busy:
            return
        self.busy = True
        self.plot_b.state(["disabled"])

        def work():
            p = subprocess.Popen([sys.executable, "-u", os.path.join(S.REPO, "scripts/plots/plot.py"), PLOTS],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=S.REPO)
            for line in p.stdout:
                self.q.put(("plot", line.strip(), None))
            self.q.put(("plot_done", p.wait(), None))
        threading.Thread(target=work, daemon=True).start()

    def poll(self):
        try:
            while True:
                kind, a, b = self.q.get_nowait()
                if kind == "sim":
                    self.prog.config(maximum=max(b, 1), value=a)
                    self.status.config(text=f"{a} of {b} sweeps done" if b else "all cached")
                elif kind in ("sim_done", "sim_fail"):
                    self.busy = False
                    self.status.config(text="done" if kind == "sim_done" else f"failed: {a}")
                    self.refresh()
                elif kind == "plot":
                    m = re.match(r"sweep (\d+)/(\d+)", a)
                    self.plot_status.config(text=f"{m.group(1)} of {m.group(2)} sweeps done" if m else a[:80])
                elif kind == "plot_done":
                    self.busy = False
                    self.plot_b.state(["!disabled"])
                    self.plot_status.config(text="done: .plots/plots.md" if a == 0 else f"just plot failed ({a})")
                    self.show_chart(); self.refresh()
        except queue.Empty:
            pass
        self.root.after(200, self.poll)

    def show_chart(self):
        _, folder, name = next(c for c in README_CHARTS if c[0] == self.chart.get())
        found = sorted(glob.glob(os.path.join(PLOTS, folder, f"[0-9][0-9]_{name}.png")))
        self.chart_fig.clear()
        ax = self.chart_fig.add_subplot(); ax.axis("off")
        if found:
            ax.imshow(mpimg.imread(found[0]))
            if os.path.getmtime(found[0]) < max(os.path.getmtime(S.NECK), os.path.getmtime(S.GEN)):
                self.plot_status.config(text="older than neck.yaml: Run just plot to update")
        else:
            ax.text(0.5, 0.5, "Not made yet: Run just plot.", ha="center", va="center", transform=ax.transAxes, color="#777777")
        self.chart_canvas.draw_idle()

    def save(self):
        d = self.diff()
        if not d or not messagebox.askokcancel("Save neck.yaml", "Write these changes?\n\n" + "\n".join(d)):
            return
        c, text = self.cfg(), self.text
        for var, (path, label, unit, *_), sb in self.vars.values():
            if differs(get(c, path), get(self.saved, path)):
                text = yaml_set(text, path, yaml_text(get(c, path), unit))
        with open(S.NECK, "w", newline="\n") as f:
            f.write(text)
        self.text, self.saved = text, S.load_neck()
        self.refresh()

    def revert(self):
        for var, (path, label, unit, scale, *_), sb in self.vars.values():
            var.set(shown(get(self.saved, path), scale))

    def close(self):
        if self.diff():
            ans = messagebox.askyesnocancel("Unsaved changes", "Save your changes to neck.yaml first?")
            if ans is None:
                return
            if ans:
                self.save()
        self.root.destroy()
        os._exit(0)   # don't wait on a sim still running


if __name__ == "__main__":
    App(tk.Tk())
    tk.mainloop()
