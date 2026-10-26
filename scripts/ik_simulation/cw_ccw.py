# /// script
# requires-python = "==3.12.*"  # mujoco 3.12.0 has no wheels for newer Pythons
# dependencies = [
#     "mujoco==3.12.0",  # the version mujoco_vendor bundles for the sim
#     "numpy",
#     "pyyaml",
#     "matplotlib",
# ]
# ///
"""The neck twisted both ways (CW and CCW viewed from the top), jam 75. Data and renders only.

Usage (Linux or WSL, from the repo root, MUJOCO_GL=egl for the renders):
    uv run scripts/ik_simulation/cw_ccw.py sweep  .output/ik_sweep/cw_ccw   # rest checks + 1 deg sweeps
    uv run scripts/ik_simulation/cw_ccw.py checks .output/ik_sweep/cw_ccw   # rest checks only
    uv run scripts/ik_simulation/cw_ccw.py flipcheck .output/ik_sweep/cw_ccw  # each model turned the other way, swept
    uv run scripts/ik_simulation/cw_ccw.py render .output/ik_sweep/cw_ccw   # after cw_ccw_charts.py (needs poses.csv)

The model is sweep.py's (neck.yaml through gen_neck_mjcf.py, sled contact off, same ramp/settle). The
other twist is the same config with platform.twist_sign negated, in memory only. Which way each one
twists is measured from the sim: the orbit angle from each rod's bottom pivot to its top pivot, viewed
from +Z (CCW positive), and a torque test on the platform.
"""
import copy, csv, importlib.util, math, os, sys, time
from multiprocessing import Pool

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("sweep", os.path.join(HERE, "sweep.py"))
sw = importlib.util.module_from_spec(spec); spec.loader.exec_module(sw)

JAM = 75
CUR = 1 if sw.POLE == "CCW" else -1   # twist_sign as neck.yaml had it: +1 CCW, -1 CW (the current model)
SIGNS = {"current": CUR, "mirrored": -CUR}
TOP_CAM = '<camera name="top" pos="0 0 0.6" xyaxes="1 0 0 0 1 0" projection="orthographic" fovy="0.2"/>'
TOP_FOVY = 0.2


def xml(sign, jam=JAM):
    cfg = copy.deepcopy(sw.BASE)
    cfg["platform"]["rest_height"] = sw.rest_height(cfg, jam)
    # Render-only additions: a top camera and a bigger offscreen buffer. Neither touches the dynamics.
    vis = '<visual><global offwidth="1000" offheight="1000"/></visual>\n  '
    return cfg, sw.gen.build(cfg, "CCW" if sign > 0 else "CW").replace("<worldbody>", vis + "<worldbody>\n    " + TOP_CAM, 1)


class Sim(sw.Sim):
    def __init__(self, sign, jam=JAM):
        self.cfg, x = xml(sign, jam)
        self.m = mujoco.MjModel.from_xml_string(x)
        self.m.geom_contype[:] = 0      # as sweep.model: sled contact off
        self.m.geom_conaffinity[:] = 0
        self.d = mujoco.MjData(self.m)
        self.start = np.radians(self.cfg["sled"]["start_deg"])
        self.cur = np.zeros(3)
        self.d.ctrl[:] = self.start
        for _ in range(2000):
            mujoco.mj_step(self.m, self.d)

    def pivots(self):
        b = np.array([self.d.site(f"sled_{i}_pivot").xpos for i in (1, 2, 3)])
        t = np.array([self.d.body(f"rod_{i}_top").xpos for i in (1, 2, 3)])
        return b, t

    def orbit(self):
        """Signed orbit angle bottom -> top pivot per rod, viewed from +Z, CCW positive (deg)."""
        b, t = self.pivots()
        a = np.degrees(np.arctan2(t[:, 1], t[:, 0]) - np.arctan2(b[:, 1], b[:, 0]))
        return (a + 180) % 360 - 180


def direction(orbit):
    return "CCW" if np.all(orbit > 0) else "CW" if np.all(orbit < 0) else "mixed"


# ------------------------------------------------------------------------------------------- sweep
def sweep(args):
    name, sign, pass_name, *flip = args
    sim = Sim(sign)
    if flip:   # turn the platform the other way first: torque about +Z for 1 s, then released for 2 s
        flip_platform(sim, flip[0])
    vals = list(range(-sw.LIM, sw.LIM + 1, sw.STEP))
    rows, t0 = [], time.time()
    for i, outer in enumerate(vals):
        for inner in (vals if i % 2 == 0 else vals[::-1]):
            s1, s2 = (outer, inner) if pass_name == "a" else (inner, outer)
            if not sw.feasible(s1, s2):
                continue
            s3 = -s1 - s2
            sim.go([s1, s2, s3])
            (roll, pitch, yaw), p = sim.head()
            off_x, off_y = p[0] * 1000, p[1] * 1000
            rows.append([JAM, s1, s2, s3, round(roll, 3), round(pitch, 3), round(yaw, 3),
                         round(off_x, 3), round(off_y, 3), round(math.hypot(off_x, off_y), 3)])
        if i % 16 == 0:
            print(f"{name} pass {pass_name}: {i + 1}/{len(vals)} rows, {len(rows)} points, {time.time() - t0:.0f} s", flush=True)
    print(f"{name} pass {pass_name}: done, {len(rows)} points, {time.time() - t0:.0f} s", flush=True)
    return name, pass_name, rows


def swivels(sim):
    """Rod-end swivel angles (deg): bottom 1-3 then top 1-3. Range is -untwist_deg to +0.29 (the jam side)."""
    return [math.degrees(sim.d.joint(f"rod_{i}_{j}").qpos[0]) for j in ("swivel", "top_swivel") for i in (1, 2, 3)]


def rest_checks(sign):
    """Rest pose: orbit angles, neutral height and drift, and the platform torque test."""
    sim = Sim(sign)
    z_pivot = sim.cfg["collar_z"] + sim.cfg["sled"]["pivot_height"]
    for _ in range(3000):
        mujoco.mj_step(sim.m, sim.d)
    h0 = (sim.d.sensor("head_pos").data[2] - z_pivot) * 1000
    rpy0 = sim.head()[0]
    for _ in range(5000):   # 5 s more, sleds held
        mujoco.mj_step(sim.m, sim.d)
    h1 = (sim.d.sensor("head_pos").data[2] - z_pivot) * 1000
    rpy1 = sim.head()[0]
    out = {"swivel": swivels(sim), "orbit": sim.orbit(), "height": h0, "height_after_5s": h1, "rpy": rpy0, "rpy_after_5s": rpy1, "torque": []}
    # Torque about +Z (CCW viewed from the top) on the platform, sleds held, for 1 s; then released for 1 s.
    pid = sim.m.body("platform").id
    for tau in (-3, -1, -0.3, -0.1, 0.1, 0.3, 1, 3):
        s = Sim(sign)
        for _ in range(1000):
            mujoco.mj_step(s.m, s.d)
        y0 = s.head()[0][2]
        s.d.xfrc_applied[pid, 5] = tau
        for _ in range(1000):
            mujoco.mj_step(s.m, s.d)
        y1 = s.head()[0][2]
        h_t = (s.d.sensor("head_pos").data[2] - z_pivot) * 1000
        sv_t = swivels(s)
        s.d.xfrc_applied[pid, 5] = 0
        for _ in range(1000):
            mujoco.mj_step(s.m, s.d)
        y2 = s.head()[0][2]
        h2 = (s.d.sensor("head_pos").data[2] - z_pivot) * 1000
        out["torque"].append([tau, round(y1 - y0, 3), round(h_t, 2), round(y2 - y0, 3), round(h2, 3),
                              *np.round(s.orbit(), 3), *np.round(sv_t, 2)])
    return out


def cmd_checks(out):
    os.makedirs(out, exist_ok=True)
    checks = {}
    sv_names = [f"{j}_swivel_{i}" for j in ("bottom", "top") for i in (1, 2, 3)]
    with open(os.path.join(out, "rest_checks.csv"), "w", newline="") as f, \
         open(os.path.join(out, "torque_test.csv"), "w", newline="") as g:
        w = csv.writer(f); wt = csv.writer(g)
        w.writerow(["model", "twist_sign", "direction_viewed_from_top", "orbit_1", "orbit_2", "orbit_3",
                    "height_mm", "height_after_5s_mm", "roll", "pitch", "yaw", "yaw_after_5s", *sv_names])
        wt.writerow(["direction_viewed_from_top", "torque_z_Nm", "yaw_change_under_torque_deg",
                     "height_under_torque_mm", "yaw_change_after_release_deg", "height_after_release_mm",
                     "orbit_1_after_release", "orbit_2_after_release", "orbit_3_after_release", *(n + "_under_torque" for n in sv_names)])
        for name, sign in SIGNS.items():
            c = rest_checks(sign); d = direction(c["orbit"]); checks[name] = d
            w.writerow([name, sign, d, *np.round(c["orbit"], 3), round(c["height"], 3), round(c["height_after_5s"], 3),
                        *np.round(c["rpy"], 3), round(c["rpy_after_5s"][2], 3), *np.round(c["swivel"], 3)])
            for row in c["torque"]:
                wt.writerow([d, *row])
            print(name, "twist_sign", sign, "->", d, "orbit", np.round(c["orbit"], 2), "height", round(c["height"], 3),
                  "swivel", np.round(c["swivel"], 2), "torque", c["torque"], flush=True)
    return checks


def flip_platform(sim, tau):
    sim.d.xfrc_applied[sim.m.body("platform").id, 5] = tau
    for _ in range(1000):
        mujoco.mj_step(sim.m, sim.d)
    sim.d.xfrc_applied[:] = 0
    for _ in range(2000):
        mujoco.mj_step(sim.m, sim.d)


def cmd_flipcheck(out):
    """Each model turned the other way by hand, then swept (pass A), against the other model's pass A."""
    names = {r["direction_viewed_from_top"]: int(r["twist_sign"]) for r in csv.DictReader(open(os.path.join(out, "rest_checks.csv")))}
    jobs = [(f"{d} model turned {'CW' if d == 'CCW' else 'CCW'}", names[d], "a", -1.0 if d == "CCW" else 1.0) for d in names]
    with Pool(len(jobs)) as pool:
        res = pool.map(sweep, jobs)
    def rot(r):
        ro, pi, ya = (math.radians(float(x)) for x in r[4:7])
        cz, sz, cy, sy, cx, sx = math.cos(ya), math.sin(ya), math.cos(pi), math.sin(pi), math.cos(ro), math.sin(ro)
        return np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])             @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])

    with open(os.path.join(out, "flip_check.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sweep", "compared_with", "points", *(f"max_abs_diff_{k}" for k in ("roll", "pitch", "yaw", "off_x", "off_y", "off")),
                    "platform_yaw_offset_at_rest_deg", "max_orientation_diff_after_removing_offset_deg", "max_abs_diff_tilt_deg"])
        for (name, _, rows), (d, _, _, _) in zip(res, jobs):
            other = "CW" if d.startswith("CCW") else "CCW"
            with open(os.path.join(out, f"sweep_{d.split()[0].lower()}_turned_{other.lower()}_pass_a.csv"), "w", newline="") as g:
                wr = csv.writer(g); wr.writerow(sw.HEADER); wr.writerows(rows)
            ref = {(int(r[1]), int(r[2])): r for r in csv.reader(open(os.path.join(out, f"sweep_{other.lower()}_pass_a.csv"))) if r[0] != "jam_deg"}
            diffs = np.array([[abs(row[k] - float(ref[row[1], row[2]][k])) for k in range(4, 10)] for row in rows])
            rest_row = next(r for r in rows if r[1] == 0 and r[2] == 0)
            c = (rest_row[6] - float(ref[0, 0][6]) + 180) % 360 - 180   # platform yaw offset at rest
            Rc = np.array([[math.cos(math.radians(c)), -math.sin(math.radians(c)), 0], [math.sin(math.radians(c)), math.cos(math.radians(c)), 0], [0, 0, 1]])
            ang, dt = [], []
            for row in rows:
                Rf, Ro = rot(row), rot(ref[row[1], row[2]]) @ Rc
                ang.append(math.degrees(math.acos(max(-1, min(1, (np.trace(Ro.T @ Rf) - 1) / 2)))))
                dt.append(abs(math.degrees(math.acos(max(-1, min(1, Rf[2, 2])))) - math.degrees(math.acos(max(-1, min(1, Ro[2, 2]))))))
            w.writerow([name, f"{other} model pass A", len(rows), *np.round(diffs.max(axis=0), 3), round(c, 3),
                        round(max(ang), 4), round(max(dt), 4)])
            print(name, "vs", other, "max diffs", np.round(diffs.max(axis=0), 3), "yaw offset", round(c, 3),
                  "orientation diff after offset", round(max(ang), 4), "tilt diff", round(max(dt), 4), flush=True)


def cmd_sweep(out):
    checks = cmd_checks(out)
    jobs = [(checks[n], s, p) for n, s in SIGNS.items() for p in "ab"]
    with Pool(len(jobs)) as pool:
        res = pool.map(sweep, jobs)
    for name, p, rows in res:
        with open(os.path.join(out, f"sweep_{name.lower()}_pass_{p}.csv"), "w", newline="") as f:
            w = csv.writer(f); w.writerow(sw.HEADER); w.writerows(rows)
    print("written", flush=True)


# ------------------------------------------------------------------------------------------ render
W, H = 900, 900


def add_arrow(scn, a, b, rgba, width=0.003):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9), np.array(rgba, np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, width, np.array(a, float), np.array(b, float))
    scn.ngeom += 1


def snap(sim, r, cam, heading=None):
    """Render; the platform is drawn see-through so the top pivots show. heading draws the head's forward axis."""
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
    """World x, y (m) to pixel in the orthographic top camera."""
    s = H / TOP_FOVY
    return W / 2 + xy[0] * s, H / 2 - xy[1] * s


def side_cam(heading):
    c = mujoco.MjvCamera()
    c.type = mujoco.mjtCamera.mjCAMERA_FREE
    c.lookat[:] = [0, 0, 0.115]
    c.distance, c.elevation = 0.22, -4
    c.azimuth = heading + 90   # looking along the head's left axis, so its pitch shows side on
    return c


def top_panel(ax, img, sim, label, heading=None):
    ax.imshow(img); ax.set_xticks([]); ax.set_yticks([])
    b, t = sim.pivots(); orb = sim.orbit()
    for i in range(3):
        bx, by = to_px(b[i]); tx, ty = to_px(t[i])
        ax.plot(bx, by, "o", ms=7, mfc="none", mec="#2a78d6", mew=2)
        ax.plot(tx, ty, "s", ms=7, mfc="none", mec="#eb6834", mew=2)
        # orbit arc from bottom to top pivot, at the bottom pivot radius + 12 mm
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
        ax.annotate("", xy=(W - 140 + dx, H - 40 + dy), xytext=(W - 140, H - 40),
                    arrowprops=dict(arrowstyle="-|>", color="white"))
        ax.text(W - 140 + dx * 1.25 + (0 if dx else 0), H - 40 + dy * 1.25, t, color="white", fontsize=8, ha="center", va="center")
    ax.set_title(label, fontsize=10)


def cmd_render(out):
    names = {}
    with open(os.path.join(out, "rest_checks.csv")) as f:
        for row in csv.DictReader(f):
            names[row["direction_viewed_from_top"]] = int(row["twist_sign"])
    order = [d for d in ("CW", "CCW") if d in names]

    # --- rest, top view, labelled ---
    fig, axes = plt.subplots(1, 2, figsize=(13, 7), constrained_layout=True)
    for ax, d in zip(axes, order):
        sim = Sim(names[d])
        for _ in range(3000):
            mujoco.mj_step(sim.m, sim.d)
        r = mujoco.Renderer(sim.m, H, W)
        img = snap(sim, r, "top")
        cur = " (current model, CCW default)" if names[d] == CUR else " (mirrored, twist_sign negated)"
        top_panel(ax, img, sim, f"{d} viewed from top{cur}\ntwist_sign {names[d]:+d}, jam {JAM}")
        plt.imsave(os.path.join(out, f"rest_top_{d.lower()}_raw.png"), img)
    fig.suptitle("Rest pose from the top (orthographic, looking down -Z; platform see-through; white eyes: the head, at head.rotation_deg)\n"
                 "blue circle: rod bottom pivot on the sled, orange square: rod top pivot, green: orbit bottom to top",
                 fontsize=10)
    fig.savefig(os.path.join(out, "rest_top_cw_ccw.png"), dpi=110); plt.close(fig)

    # --- each model turned by hand the other way: torque on the platform for 1 s, then released for 2 s ---
    fig, axes = plt.subplots(1, 2, figsize=(13, 7), constrained_layout=True)
    for ax, d in zip(axes, order):
        sim = Sim(names[d])
        for _ in range(1000):
            mujoco.mj_step(sim.m, sim.d)
        tau = -1.0 if d == "CCW" else 1.0
        sim.d.xfrc_applied[sim.m.body("platform").id, 5] = tau
        for _ in range(1000):
            mujoco.mj_step(sim.m, sim.d)
        sim.d.xfrc_applied[:] = 0
        for _ in range(2000):
            mujoco.mj_step(sim.m, sim.d)
        z = (sim.d.sensor("head_pos").data[2] - sim.cfg["collar_z"] - sim.cfg["sled"]["pivot_height"]) * 1000
        r = mujoco.Renderer(sim.m, H, W)
        top_panel(ax, snap(sim, r, "top"), sim,
                  f"{d} model after {tau:+g} N m about +Z for 1 s, released 2 s\n"
                  f"now {direction(sim.orbit())} viewed from top, height {z:.2f} mm")
    fig.suptitle("Turning the platform the other way by hand (sim): each model, top view after release\n"
                 "blue circle: rod bottom pivot, orange square: rod top pivot, green: orbit bottom to top", fontsize=10)
    fig.savefig(os.path.join(out, "rest_top_turned_by_hand.png"), dpi=110); plt.close(fig)
    print("rest renders written", flush=True)

    # --- poses from cw_ccw_charts.py ---
    poses = list(csv.DictReader(open(os.path.join(out, "poses.csv"))))
    groups = {}
    for p in poses:
        groups.setdefault(p["figure"], []).append(p)
    check = []
    for fig_name, ps in groups.items():
        rows = sorted({p["pose"] for p in ps}, key=lambda s: [q["pose"] for q in ps].index(s))
        fig, axes = plt.subplots(len(rows), 4, figsize=(17, 4.6 * len(rows)), constrained_layout=True, squeeze=False)
        for ri, pose in enumerate(rows):
            for ci, d in enumerate(order):
                p = next(q for q in ps if q["pose"] == pose and q["direction"] == d)
                sim = Sim(names[d])
                s = [float(p["sled_1"]), float(p["sled_2"]), float(p["sled_3"])]
                sim.go(s)
                (roll, pitch, yaw), pos = sim.head()
                r = mujoco.Renderer(sim.m, H, W)
                h = float(p["heading"])
                top = snap(sim, r, "top", h)
                side = snap(sim, r, side_cam(h), h)
                R = sim.d.body("platform").xmat.reshape(3, 3)
                hr = math.radians(h)
                fwd = R[:, 0] * math.cos(hr) + R[:, 1] * math.sin(hr)
                left = -R[:, 0] * math.sin(hr) + R[:, 1] * math.cos(hr)
                hp, hroll = math.degrees(math.asin(fwd[2])), math.degrees(math.asin(left[2]))
                off = math.hypot(*pos[:2]) * 1000
                check.append([fig_name, pose, d, *s, p["head_pitch"], round(hp, 3), p["head_roll"], round(hroll, 3),
                              p["off"], round(off, 3)])
                ax_t, ax_s = axes[ri, 2 * ci], axes[ri, 2 * ci + 1]
                top_panel(ax_t, top, sim, f"{d} viewed from top: {pose}\nsleds {s[0]:+g}, {s[1]:+g}, {s[2]:+g} deg")
                ax_s.imshow(side); ax_s.set_xticks([]); ax_s.set_yticks([])
                ax_s.set_title(f"{d}: side view, looking along the head's left axis\n"
                               f"head pitch {hp:+.1f} deg, head roll {hroll:+.1f} deg, off {off:.1f} mm", fontsize=10)
                print(fig_name, pose, d, "pitch", round(hp, 2), "expected", p["head_pitch"], flush=True)
        fig.suptitle(ps[0]["title"] + "\norange arrow: head forward axis at that heading; pitch/roll as in 14_v2", fontsize=11)
        fig.savefig(os.path.join(out, f"{fig_name}.png"), dpi=90); plt.close(fig)
    with open(os.path.join(out, "pose_render_check.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["figure", "pose", "direction", "sled_1", "sled_2", "sled_3", "sweep_head_pitch", "render_head_pitch",
                    "sweep_head_roll", "render_head_roll", "sweep_off", "render_off"])
        w.writerows(check)
    print("pose renders written", flush=True)


if __name__ == "__main__":
    cmd, out = sys.argv[1], sys.argv[2]
    {"checks": cmd_checks, "sweep": cmd_sweep, "flipcheck": cmd_flipcheck, "render": cmd_render}[cmd](out)
    sys.stdout.flush(); os._exit(0)   # skip EGL teardown errors at exit
