"""this_robbie: sim renders (top and side views), both twists. Needs MUJOCO_GL=egl. Data only.

Ported from scripts/ik_simulation/cw_ccw.py.
"""
import math

import matplotlib.pyplot as plt
import mujoco
import numpy as np

import sim as S

F_ = "this_robbie"
W, H = 900, 900
TOP_FOVY = 0.2


def add_arrow(scn, a, b, rgba, width=0.003):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9), np.array(rgba, np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, width, np.array(a, float), np.array(b, float))
    scn.ngeom += 1


def snap(sim, r, cam, heading=None):
    """Render with the platform see-through so the top pivots show. heading draws the head's forward axis."""
    for g in ("platform", "head_disk"):
        sim.m.geom_rgba[sim.m.geom(g).id][3] = 0.25
    r.update_scene(sim.d, camera=cam)
    if heading is not None:
        p = sim.d.body("platform").xpos.copy(); R = sim.d.body("platform").xmat.reshape(3, 3)
        h = math.radians(heading)
        fwd = R[:, 0] * math.cos(h) + R[:, 1] * math.sin(h)
        add_arrow(r.scene, p + R[:, 2] * 0.006, p + R[:, 2] * 0.006 + fwd * 0.075, [0.92, 0.41, 0.2, 1], 0.004)
    return r.render().copy()


def to_px(xy):
    s = H / TOP_FOVY
    return W / 2 + xy[0] * s, H / 2 - xy[1] * s


def side_cam(heading):
    c = mujoco.MjvCamera()
    c.type = mujoco.mjtCamera.mjCAMERA_FREE
    c.lookat[:] = [0, 0, 0.115]
    c.distance, c.elevation = 0.22, -4
    c.azimuth = heading + 90   # looking along the head's left axis, so its pitch shows side on
    return c


def nods_at_rest(sim):
    """Nod line orientations (deg, platform frame) from the sim's rod twist, which must be at rest."""
    return [st + ob / 2 - 90 for st, ob in zip(sim.cfg["sled"]["start_deg"], sim.orbit())]


def top_panel(ax, img, sim, label, nods):
    ax.imshow(img); ax.set_xticks([]); ax.set_yticks([])
    # nod lines, drawn in the platform frame from its centre
    p = sim.d.body("platform").xpos; R = sim.d.body("platform").xmat.reshape(3, 3)
    for k, h in enumerate(nods, 1):
        u = R[:, 0] * math.cos(math.radians(h)) + R[:, 1] * math.sin(math.radians(h))
        (x0, y0), (x1, y1) = to_px(p[:2]), to_px(p[:2] + u[:2] * 0.05)
        ax.plot([x0, x1], [y0, y1], color="#d62728", lw=1.6)
        ax.text(x1, y1, f"nod {k}", fontsize=7.5, color="#d62728", ha="center", va="center",
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85))
    b, t = sim.pivots(); orb = sim.orbit()
    for i in range(3):
        bx, by = to_px(b[i]); tx, ty = to_px(t[i])
        ax.plot(bx, by, "o", ms=7, mfc="none", mec="#2a78d6", mew=2)
        ax.plot(tx, ty, "s", ms=7, mfc="none", mec="#eb6834", mew=2)
        a0 = math.atan2(b[i][1], b[i][0]); a1 = a0 + math.radians(orb[i]); rr = 0.058
        arc = np.linspace(a0, a1, 40)
        px = [to_px((rr * math.cos(a), rr * math.sin(a))) for a in arc]
        ax.annotate("", xy=px[-1], xytext=px[-3], arrowprops=dict(arrowstyle="-|>", color="#1baf7a", lw=1.5))
        ax.plot([p[0] for p in px], [p[1] for p in px], color="#1baf7a", lw=1.5)
        lx, ly = to_px((0.07 * math.cos(a0), 0.07 * math.sin(a0)))
        ax.text(lx, ly, f"sled_{i + 1}", fontsize=8, ha="center", va="center", color="#2a78d6",
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85))
        mx, my = to_px((0.07 * math.cos(arc[20]), 0.07 * math.sin(arc[20])))
        ax.text(mx, my, f"{orb[i]:+.1f} deg", fontsize=7.5, ha="center", va="center", color="#1baf7a",
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85))
    for dx, dy, t in ((100, 0, "+X"), (0, -100, "+Y")):
        ax.annotate("", xy=(W - 140 + dx, H - 40 + dy), xytext=(W - 140, H - 40), arrowprops=dict(arrowstyle="-|>", color="white"))
        ax.text(W - 140 + dx * 1.25, H - 40 + dy * 1.25, t, color="white", fontsize=8, ha="center", va="center")
    ax.set_title(label, fontsize=10)


def run(book, cfgs, label, poses):
    """cfgs: {"CW"|"CCW": cfg}; label: {"CW"|"CCW": display label}."""
    order = [d for d in ("CW", "CCW") if d in cfgs]
    key = "blue circle: rod bottom pivot on the sled, orange square: rod top pivot, green: orbit bottom to top"

    fig, axes = plt.subplots(1, len(order), figsize=(6.5 * len(order), 7), constrained_layout=True, squeeze=False)
    for ax, d in zip(axes[0], order):
        sim = S.Sim(cfgs[d], render=True); sim.step(S.CAL_EXTRA)
        r = mujoco.Renderer(sim.m, H, W)
        top_panel(ax, snap(sim, r, "top"), sim, f"{label[d]} viewed from top, height {sim.height_mm():.2f} mm", nods_at_rest(sim))
    book.save(fig, F_, "render_rest_top", "Rest pose from the top, both twists",
              "Sim render looking straight down (orthographic) at rest, platform and head disk drawn see-through; eyes and orange stripe: the head at neck.yaml's head.rotation_deg (0 = rod end 1). The twist "
              "angle printed on each green arc is measured in the sim. " + key + "; red: nod lines.",
              title="Rest pose from the top (sim render; white eyes and orange stripe: the head)\n" + key, dpi=110)

    fig, axes = plt.subplots(1, len(order), figsize=(6.5 * len(order), 7), constrained_layout=True, squeeze=False)
    for ax, d in zip(axes[0], order):
        sim = S.Sim(cfgs[d], render=True); sim.step(1000)
        tau = -1.0 if d == "CCW" else 1.0
        sim.d.xfrc_applied[sim.m.body("platform").id, 5] = tau
        sim.step(1000)
        sim.d.xfrc_applied[:] = 0
        sim.step(2000)
        r = mujoco.Renderer(sim.m, H, W)
        top_panel(ax, snap(sim, r, "top"), sim,
                  f"{label[d]} after {tau:+g} N m about +Z for 1 s, released 2 s\n"
                  f"now {S.direction(sim.orbit())} viewed from top, height {sim.height_mm():.2f} mm", nods_at_rest(sim))
    book.save(fig, F_, "render_turned_by_hand", "Platform turned the other way by hand (sim)",
              "Each twist after a 1 N m torque about the vertical axis is applied to the platform for 1 s against its twist "
              "(sleds held) and then released for 2 s; top view after release. " + key + "; red: nod lines of the new twist.",
              title="Turning the platform the other way by hand (sim), top view after release\n" + key, dpi=110)

    rows = sorted({p["pose"] for p in poses}, key=lambda s: [q["pose"] for q in poses].index(s))
    fig, axes = plt.subplots(len(rows), 2 * len(order), figsize=(10 * len(order), 5.2 * len(rows)), constrained_layout=True, squeeze=False)
    for ri, pose in enumerate(rows):
        for ci, d in enumerate(order):
            p = next(q for q in poses if q["pose"] == pose and q["direction"] == d)
            sim = S.Sim(cfgs[d], render=True); sim.go(p["sleds"])
            _, pos = sim.head()
            r = mujoco.Renderer(sim.m, H, W)
            h = p["heading"]; hf = h + p["zero"]   # head.rotation_deg, and the same in the platform frame
            top, side = snap(sim, r, "top", hf), snap(sim, r, side_cam(hf), hf)
            R = sim.d.body("platform").xmat.reshape(3, 3); hr = math.radians(hf)
            fwd = R[:, 0] * math.cos(hr) + R[:, 1] * math.sin(hr)
            left = -R[:, 0] * math.sin(hr) + R[:, 1] * math.cos(hr)
            hp, hroll = math.degrees(math.asin(fwd[2])), math.degrees(math.asin(left[2]))
            s = p["sleds"]
            top_panel(axes[ri, 2 * ci], top, sim, f"{label[d]}, head.rotation_deg {h}: {pose}\nsleds {s[0]:+g}, {s[1]:+g}, {s[2]:+g} deg",
                      [n + p["zero"] for n in p["nods"]])
            ax = axes[ri, 2 * ci + 1]
            ax.imshow(side); ax.set_xticks([]); ax.set_yticks([])
            ax.set_title(f"{label[d]}: side view along the head's left axis\nhead pitch {hp:+.1f} deg, head roll {hroll:+.1f} deg, "
                         f"off {math.hypot(*pos[:2]) * 1000:.1f} mm", fontsize=10)
    book.save(fig, F_, "render_poses", "Rest, largest graceful pitch up and down (sim renders)",
              "For each twist, the head at neck.yaml's head.rotation_deg: the rest pose and the sweep points with the largest "
              "graceful pitch up and down at that head orientation, re-driven in the sim. Red: nod lines (at rest). Top view and a side "
              "view along the head's left axis; orange arrow: the head's forward axis.",
              title="Rest, largest graceful pitch up and down, head at head.rotation_deg (sim renders)", dpi=90)
