"""MuJoCo neck sweeps with an on-disk cache keyed by everything that changes the result. Data only.

The model is neck.yaml through gen_neck_mjcf.py, sled contact off (the sim's sled boxes aren't the sled.scad
profile and jam at ~69 deg; MIN_GAP bounds the sweep instead). Same sweep as scripts/ik_simulation/sweep.py.
"""
import copy, csv, hashlib, importlib.util, json, math, os, time
from multiprocessing import Pool

import mujoco
import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
GEN = os.path.join(REPO, "src/robbie_description/scripts/gen_neck_mjcf.py")
NECK = os.path.join(REPO, "src/robbie_description/config/neck.yaml")
spec = importlib.util.spec_from_file_location("gen", GEN); gen = importlib.util.module_from_spec(spec); spec.loader.exec_module(gen)

MIN_GAP = 55.8      # deg, sled centre to centre: 2 asin(sled_r 16.5 / pinion_dist 35.25), sled.scad
LIM = 64            # deg, 120 - MIN_GAP, rounded down
STEP = 1            # deg, grid step on sled_1 and sled_2
RAMP_PER_DEG = 100  # sim steps (1 ms) of ramp per degree of the largest sled move
SETTLE = 800        # sim steps after each move
INIT = 2000         # sim steps at start_deg before anything is measured
CAL_EXTRA = 3000    # extra sim steps at rest for the neutral height calibration
SETTINGS = dict(min_gap_deg=MIN_GAP, grid_limit_deg=LIM, grid_step_deg=STEP, ramp_steps_per_deg=RAMP_PER_DEG,
                settle_steps=SETTLE, init_steps=INIT, cal_extra_steps=CAL_EXTRA, sled_contact="off")
HEADER = ["jam_deg", "sled_1", "sled_2", "sled_3", "roll", "pitch", "yaw", "off_x", "off_y", "off"]
TOP_CAM = '<camera name="top" pos="0 0 0.6" xyaxes="1 0 0 0 1 0" projection="orthographic" fovy="0.2"/>'


def load_neck():
    with open(NECK) as f:
        return yaml.safe_load(f)


# ------------------------------------------------------------------------------------------ geometry
def jam_of(cfg):
    """Twist at rest (deg, unsigned) from rod length, pivot radii and rest height."""
    rb, rp, L, h = cfg["sled"]["pivot_radius"], cfg["platform"]["pivot_radius"], cfg["rod"]["length"], cfg["platform"]["rest_height"]
    return math.degrees(math.acos((rb * rb + rp * rp - (L * L - h * h)) / (2 * rb * rp)))


def with_jam(cfg, jam):
    c = copy.deepcopy(cfg)
    rb, rp, L = c["sled"]["pivot_radius"], c["platform"]["pivot_radius"], c["rod"]["length"]
    c["platform"]["rest_height"] = math.sqrt(L * L - (rb * rb + rp * rp - 2 * rb * rp * math.cos(math.radians(jam))))
    return c


def with_rod(cfg, length):
    c = copy.deepcopy(cfg); c["rod"]["length"] = length
    return c


def with_twist(cfg, pole):
    """pole: "CW" or "CCW" seen from above. It isn't in neck.yaml (it's set at runtime), so every sim cfg gets one here."""
    c = copy.deepcopy(cfg); c["twist"] = pole
    return c


def rod_end_angles(cfg):
    """Rod-end angles at rest (deg): the rod's angle out of the plane square to each rod end's radial screw."""
    rb, rp, h = cfg["sled"]["pivot_radius"], cfg["platform"]["pivot_radius"], cfg["platform"]["rest_height"]
    a = math.radians(jam_of(cfg))
    ub, ut = np.array([0, -1, 0]), np.array([math.sin(a), -math.cos(a), 0])
    d = rp * ut + [0, 0, h] - rb * ub
    d /= np.linalg.norm(d)
    return abs(math.degrees(math.asin(d @ ub))), abs(math.degrees(math.asin(d @ ut)))


def jam_for_rod_end(cfg, max_deg):
    """Jam (deg) at which the larger rod-end angle at rest reaches max_deg; it grows with jam. None if out of reach."""
    lo, hi = 1.0, 179.0
    try:
        if not max(rod_end_angles(with_jam(cfg, lo))) <= max_deg <= max(rod_end_angles(with_jam(cfg, hi))):
            return None
    except ValueError:   # the rod can't span that twist
        return None
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if max(rod_end_angles(with_jam(cfg, mid))) < max_deg else (lo, mid)
    return (lo + hi) / 2


def pivot_z(cfg):
    """Height (m) of the sled pivots above the table."""
    return cfg["collar_z"] + cfg["sled"]["pivot_height"]


def nod_lines(cfg, pole):
    """head.rotation_deg of the three nod lines at rest: halfway round each rod's twist, so -twist/2 + 120 k."""
    twist = jam_of(cfg) * (1 if pole == "CCW" else -1)
    return [(-twist / 2 + 120 * k + 180) % 360 - 180 for k in range(3)]


def spring_rest_len(cfg):
    """Distance (m) between neighbouring spring mounts at rest; the same for all three. As gen_neck_mjcf.py."""
    rb, rp, h = cfg["sled"]["pivot_radius"], cfg["platform"]["pivot_radius"], cfg["platform"]["rest_height"]
    a = math.radians(jam_of(cfg))
    f = cfg["springs"]["from_top"] / cfg["rod"]["length"]

    def mount(k):   # from_top along rod k from its top pivot
        top = np.array([rp * math.cos(a + k), rp * math.sin(a + k), h])
        return top + f * (np.array([rb * math.cos(k), rb * math.sin(k), 0]) - top)
    return float(np.linalg.norm(mount(2 * math.pi / 3) - mount(0)))


def feasible(s1, s2):
    s3 = -s1 - s2
    return min(120 + s2 - s1, 120 + s3 - s2, 120 + s1 - s3) >= MIN_GAP


# ------------------------------------------------------------------------------------------------ sim
class Sim:
    def __init__(self, cfg, render=False):
        self.cfg = cfg
        if not render:   # the eyes' 2 g sit on the head; keep them at 0 so sweeps don't depend on head.rotation_deg
            cfg = copy.deepcopy(cfg); cfg["head"]["rotation_deg"] = 0
        x = gen.build(cfg, cfg["twist"])
        if render:   # a top camera and a bigger offscreen buffer; neither touches the dynamics
            vis = '<visual><global offwidth="1000" offheight="1000"/></visual>\n  '
            x = x.replace("<worldbody>", vis + "<worldbody>\n    " + TOP_CAM, 1)
        self.m = mujoco.MjModel.from_xml_string(x)
        self.m.geom_contype[:] = 0
        self.m.geom_conaffinity[:] = 0
        self.d = mujoco.MjData(self.m)
        self.start = np.radians(cfg["sled"]["start_deg"])
        self.cur = np.zeros(3)   # deg offsets
        self.d.ctrl[:] = self.start
        self.step(INIT)

    def step(self, n):
        for _ in range(n):
            mujoco.mj_step(self.m, self.d)

    def go(self, off):
        off = np.asarray(off, float)
        ramp = max(1, int(RAMP_PER_DEG * np.max(np.abs(off - self.cur))))
        for k in range(ramp):
            self.d.ctrl[:] = self.start + np.radians(self.cur + (off - self.cur) * (k + 1) / ramp)
            mujoco.mj_step(self.m, self.d)
        self.d.ctrl[:] = self.start + np.radians(off)
        self.step(SETTLE)
        self.cur = off

    def head(self):
        """Roll, pitch, yaw (deg, ZYX) and position of the platform, in its own frame: +X is world +X at rest.
        The plots work in this frame and turn it to any head orientation, so head.rotation_deg doesn't apply here."""
        w, x, y, z = self.d.body("platform").xquat
        rpy = [math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
               math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x))))),
               math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))]
        return rpy, self.d.body("platform").xpos.copy()

    def head_zero(self):
        """World angle (deg) of rod end 1 from the platform centre: head.rotation_deg's zero. Call at rest."""
        _, t = self.pivots(); p = self.d.body("platform").xpos
        return math.degrees(math.atan2(t[0][1] - p[1], t[0][0] - p[0]))

    def height_mm(self):
        return (self.d.sensor("head_pos").data[2] - self.cfg["collar_z"] - self.cfg["sled"]["pivot_height"]) * 1000

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
    return "CCW" if np.all(np.asarray(orbit) > 0) else "CW" if np.all(np.asarray(orbit) < 0) else "mixed"


# ---------------------------------------------------------------------------------------------- cache
def _rounded(v):
    if isinstance(v, float):
        return round(v, 12)
    if isinstance(v, dict):
        return {k: _rounded(x) for k, x in v.items()}
    if isinstance(v, list):
        return [_rounded(x) for x in v]
    return v


def _without(cfg, sec, k):
    return {**cfg, sec: {x: v for x, v in cfg[sec].items() if x != k}} if sec in cfg else cfg


with open(GEN, "rb") as _f:
    GEN_HASH = hashlib.sha256(_f.read()).hexdigest()


def key(kind, cfg, extra=None):
    cfg = {k: v for k, v in cfg.items() if k != "head"}   # the sweeps are in the platform frame; see Sim.head
    cfg = _without(_without(cfg, "rod_end", "max_angle_deg"), "platform", "measured_height")   # just config's checks only
    blob = json.dumps({"kind": kind, "cfg": _rounded(cfg), "extra": extra, "settings": SETTINGS, "gen": GEN_HASH,
                       "mujoco": mujoco.__version__}, sort_keys=True)
    return f"{kind}_{hashlib.sha256(blob.encode()).hexdigest()[:16]}"


def _sweep(job):
    """One 1 deg serpentine sweep over the feasible triangle. Pass a: sled_1 outer loop; pass b: sled_2."""
    cfg, pass_name, path = job
    t0 = time.time()
    sim = Sim(cfg)
    jam = round(jam_of(cfg), 3)
    (r0, p0, y0), _ = sim.head(); h0 = sim.height_mm(); orbit = sim.orbit(); zero = sim.head_zero()
    springs = [sim.d.ten_length[i] * 1000 for i in range(3)]
    vals = list(range(-LIM, LIM + 1, STEP))
    rows, max_yaw = [], 0
    for i, outer in enumerate(vals):
        for inner in (vals if i % 2 == 0 else vals[::-1]):
            s1, s2 = (outer, inner) if pass_name == "a" else (inner, outer)
            if not feasible(s1, s2):
                continue
            sim.go([s1, s2, -s1 - s2])
            (roll, pitch, yaw), p = sim.head()
            max_yaw = max(max_yaw, abs(yaw))
            ox, oy = p[0] * 1000, p[1] * 1000
            rows.append([jam, s1, s2, -s1 - s2, round(roll, 3), round(pitch, 3), round(yaw, 3),
                         round(ox, 3), round(oy, 3), round(math.hypot(ox, oy), 3)])
    sim.go([0, 0, 0]); sim.go([0, 0, 0])
    (r1, p1, y1), _ = sim.head(); h1 = sim.height_mm()
    bottom, top = rod_end_angles(cfg)
    checks = dict(jam_deg=jam, rod_len_mm=round(cfg["rod"]["length"] * 1000, 3), twist=cfg["twist"],
                  pass_name=pass_name, points=len(rows), neutral_height_mm=round(h0, 3),
                  rest_change_max=round(max(abs(a - b) for a, b in zip((r0, p0, y0, h0), (r1, p1, y1, h1))), 4),
                  max_abs_yaw=round(max_yaw, 3), spring_len_rest_mm=round(min(springs), 3),
                  orbit_deg=[round(v, 3) for v in orbit], direction=direction(orbit), head_zero_deg=round(zero, 3),
                  rod_end_bottom_deg=round(bottom, 3), rod_end_top_deg=round(top, 3), seconds=round(time.time() - t0))
    tmp = path + ".tmp"
    with open(tmp, "w", newline="") as f:
        w = csv.writer(f); w.writerow(HEADER); w.writerows(rows)
    with open(path[:-4] + ".json", "w") as f:
        json.dump({"checks": checks, "cfg": cfg}, f, indent=1)
    os.replace(tmp, path)
    return path


def _cal(job):
    cfg, jam = job
    sim = Sim(with_jam(cfg, jam))
    sim.step(CAL_EXTRA)
    return [jam, round(sim.height_mm(), 3)]


def ensure_sweeps(cache, jobs, procs=30, log=None, pool=Pool):
    """jobs: {name: (cfg, pass)}. Runs the missing ones. Returns {name: (array, checks)}.
    log(text, done, total) gets progress (default: print); pool makes the process pool (the GUI spawns)."""
    log = log or (lambda text, done, total: print(text, flush=True))
    os.makedirs(cache, exist_ok=True)
    paths = {n: os.path.join(cache, key(f"sweep_{p}", cfg) + ".csv") for n, (cfg, p) in jobs.items()}
    todo = sorted({paths[n]: (cfg, p, paths[n]) for n, (cfg, p) in jobs.items() if not os.path.exists(paths[n])}.values(),
                  key=lambda j: j[2])
    log(f"sweeps: {len(set(paths.values()))} needed, {len(todo)} to run", 0, len(todo))
    if todo:
        t0 = time.time()
        with pool(min(procs, len(todo))) as p:
            for n, path in enumerate(p.imap_unordered(_sweep, todo), 1):
                log(f"sweep {n}/{len(todo)} done ({os.path.basename(path)}), {time.time() - t0:.0f} s", n, len(todo))
    out = {}
    for n, path in paths.items():
        with open(path) as f:
            r = csv.reader(f); next(r)
            a = np.array([[float(x) for x in row] for row in r])
        with open(path[:-4] + ".json") as f:
            out[n] = (a, json.load(f)["checks"])
    return out


def ensure_cal(cache, cfg, jams, procs=30):
    """Neutral height (mm) vs jam for cfg's rod and pivots."""
    os.makedirs(cache, exist_ok=True)
    path = os.path.join(cache, key("cal", cfg, list(jams)) + ".csv")
    if not os.path.exists(path):
        print(f"neutral height calibration: {len(jams)} jam angles", flush=True)
        with Pool(procs) as pool:
            rows = pool.map(_cal, [(cfg, j) for j in jams])
        with open(path, "w", newline="") as f:
            w = csv.writer(f); w.writerow(["jam_deg", "height_mm"]); w.writerows(rows)
    with open(path) as f:
        r = csv.reader(f); next(r)
        return np.array([[float(x) for x in row] for row in r])
