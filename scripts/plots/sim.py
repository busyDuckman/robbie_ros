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


def with_twist(cfg, sign):
    c = copy.deepcopy(cfg); c["platform"]["twist_sign"] = sign
    return c


def rod_end_angles(cfg):
    """Rod-end angles at rest (deg): the rod's angle out of the plane square to each rod end's radial screw."""
    rb, rp, h = cfg["sled"]["pivot_radius"], cfg["platform"]["pivot_radius"], cfg["platform"]["rest_height"]
    a = math.radians(jam_of(cfg))
    ub, ut = np.array([0, -1, 0]), np.array([math.sin(a), -math.cos(a), 0])
    d = rp * ut + [0, 0, h] - rb * ub
    d /= np.linalg.norm(d)
    return abs(math.degrees(math.asin(d @ ub))), abs(math.degrees(math.asin(d @ ut)))


def feasible(s1, s2):
    s3 = -s1 - s2
    return min(120 + s2 - s1, 120 + s3 - s2, 120 + s1 - s3) >= MIN_GAP


# ------------------------------------------------------------------------------------------------ sim
class Sim:
    def __init__(self, cfg, render=False):
        self.cfg = cfg
        x = gen.build(cfg)
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
        w, x, y, z = self.d.sensor("head_quat").data
        rpy = [math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
               math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x))))),
               math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))]
        return rpy, self.d.sensor("head_pos").data.copy()

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


with open(GEN, "rb") as _f:
    GEN_HASH = hashlib.sha256(_f.read()).hexdigest()


def key(kind, cfg, extra=None):
    blob = json.dumps({"kind": kind, "cfg": _rounded(cfg), "extra": extra, "settings": SETTINGS, "gen": GEN_HASH,
                       "mujoco": mujoco.__version__}, sort_keys=True)
    return f"{kind}_{hashlib.sha256(blob.encode()).hexdigest()[:16]}"


def _sweep(job):
    """One 1 deg serpentine sweep over the feasible triangle. Pass a: sled_1 outer loop; pass b: sled_2."""
    cfg, pass_name, path = job
    t0 = time.time()
    sim = Sim(cfg)
    jam = round(jam_of(cfg), 3)
    (r0, p0, y0), _ = sim.head(); h0 = sim.height_mm(); orbit = sim.orbit()
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
    checks = dict(jam_deg=jam, rod_len_mm=round(cfg["rod"]["length"] * 1000, 3), twist_sign=cfg["platform"]["twist_sign"],
                  pass_name=pass_name, points=len(rows), neutral_height_mm=round(h0, 3),
                  rest_change_max=round(max(abs(a - b) for a, b in zip((r0, p0, y0, h0), (r1, p1, y1, h1))), 4),
                  max_abs_yaw=round(max_yaw, 3), spring_len_rest_mm=round(min(springs), 3),
                  orbit_deg=[round(v, 3) for v in orbit], direction=direction(orbit),
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


def ensure_sweeps(cache, jobs, procs=30):
    """jobs: {name: (cfg, pass)}. Runs the missing ones. Returns {name: (array, checks)}."""
    os.makedirs(cache, exist_ok=True)
    paths = {n: os.path.join(cache, key(f"sweep_{p}", cfg) + ".csv") for n, (cfg, p) in jobs.items()}
    todo = sorted({paths[n]: (cfg, p, paths[n]) for n, (cfg, p) in jobs.items() if not os.path.exists(paths[n])}.values(),
                  key=lambda j: j[2])
    print(f"sweeps: {len(set(paths.values()))} needed, {len(todo)} to run", flush=True)
    if todo:
        t0 = time.time()
        with Pool(min(procs, len(todo))) as pool:
            for n, path in enumerate(pool.imap_unordered(_sweep, todo), 1):
                print(f"sweep {n}/{len(todo)} done ({os.path.basename(path)}), {time.time() - t0:.0f} s", flush=True)
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
