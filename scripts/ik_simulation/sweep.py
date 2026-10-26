# /// script
# requires-python = "==3.12.*"  # mujoco 3.12.0 has no wheels for newer Pythons
# dependencies = [
#     "mujoco==3.12.0",  # the version mujoco_vendor bundles for the sim
#     "numpy",
#     "pyyaml",
# ]
# ///
"""IK groundwork data: sweep the sled offsets in the sim at several jam angles. Data only, no solver.

Usage (Linux or WSL, from the repo root):
    uv run scripts/ik_simulation/sweep.py .output/ik_sweep

jam_deg is the twist at rest: the orbit angle from a rod's bottom pivot round to its top pivot. The
rod-end swivel limits sit at the rest pose, so this is where the model jams. The model is built for
each jam_deg by setting platform.rest_height from the rod length and the two pivot radii.

Pass A walks the grid with sled_1 as the outer loop, pass B with sled_2, both serpentine with ramped
1 deg moves, so any path dependence shows up as a difference between them.
"""
import copy, csv, importlib.util, math, os, sys, time
from multiprocessing import Pool

import mujoco
import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
GEN = os.path.join(REPO, "src/robbie_description/scripts/gen_neck_mjcf.py")
NECK = os.path.join(REPO, "src/robbie_description/config/neck.yaml")

MIN_GAP = 55.8                       # deg, sled centre to centre: 2 asin(sled_r 16.5 / pinion_dist 35.25), sled.scad
SWEEP_JAMS = list(range(60, 101, 5))  # deg
CAL_JAMS = list(range(40, 121))       # deg, neutral height only
STEP = 1                              # deg, grid step on sled_1 and sled_2
LIM = 64                              # deg, 120 - MIN_GAP, rounded down
RAMP_PER_DEG = 100                    # sim steps (1 ms) of ramp per degree of the largest sled move
SETTLE = 800                          # sim steps after each move (settling was flat by 750)

spec = importlib.util.spec_from_file_location("gen", GEN); gen = importlib.util.module_from_spec(spec); spec.loader.exec_module(gen)
BASE = yaml.safe_load(open(NECK))
POLE = "CCW"                          # the old neck.yaml twist_sign +1; the pole is now set at runtime, not in neck.yaml


def rest_height(cfg, jam):
    rb, rp, L = cfg["sled"]["pivot_radius"], cfg["platform"]["pivot_radius"], cfg["rod"]["length"]
    chord_sq = rb * rb + rp * rp - 2 * rb * rp * math.cos(math.radians(jam))
    return math.sqrt(L * L - chord_sq)


def model(jam):
    cfg = copy.deepcopy(BASE)
    cfg["platform"]["rest_height"] = rest_height(cfg, jam)
    m = mujoco.MjModel.from_xml_string(gen.build(cfg, POLE))
    # The sim's sled boxes aren't the sled.scad profile and jam at ~69 deg. MIN_GAP bounds the sweep instead.
    m.geom_contype[:] = 0
    m.geom_conaffinity[:] = 0
    return cfg, m, mujoco.MjData(m)


def rpy(q):
    w, x, y, z = q
    return [math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x))))),
            math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))]


def feasible(s1, s2):
    s3 = -s1 - s2
    return min(120 + s2 - s1, 120 + s3 - s2, 120 + s1 - s3) >= MIN_GAP


class Sim:
    def __init__(self, jam):
        self.cfg, self.m, self.d = model(jam)
        self.start = np.radians(self.cfg["sled"]["start_deg"])
        self.cur = np.zeros(3)   # deg offsets
        self.d.ctrl[:] = self.start
        for _ in range(2000):
            mujoco.mj_step(self.m, self.d)

    def go(self, off):
        off = np.asarray(off, float)
        ramp = max(1, int(RAMP_PER_DEG * np.max(np.abs(off - self.cur))))
        for k in range(ramp):
            self.d.ctrl[:] = self.start + np.radians(self.cur + (off - self.cur) * (k + 1) / ramp)
            mujoco.mj_step(self.m, self.d)
        self.d.ctrl[:] = self.start + np.radians(off)
        for _ in range(SETTLE):
            mujoco.mj_step(self.m, self.d)
        self.cur = off

    def head(self):
        # the platform, +X forward: head_quat now follows head.rotation_deg, which these scripts predate
        return rpy(self.d.body("platform").xquat), self.d.sensor("head_pos").data.copy()


def sweep(args):
    jam, pass_name = args
    sim = Sim(jam)
    vals = list(range(-LIM, LIM + 1, STEP))
    rows, t0 = [], time.time()
    for i, outer in enumerate(vals):
        for inner in (vals if i % 2 == 0 else vals[::-1]):
            s1, s2 = (outer, inner) if pass_name == "a" else (inner, outer)
            if not feasible(s1, s2):
                continue
            s3 = -s1 - s2
            sim.go([s1, s2, s3])
            (roll, pitch, yaw), p = sim.head()
            off_x, off_y = p[0] * 1000, p[1] * 1000
            rows.append([jam, s1, s2, s3, round(roll, 3), round(pitch, 3), round(yaw, 3),
                         round(off_x, 3), round(off_y, 3), round(math.hypot(off_x, off_y), 3)])
        if i % 16 == 0:
            print(f"jam {jam} pass {pass_name}: {i + 1}/{len(vals)} rows, {len(rows)} points, {time.time() - t0:.0f} s", flush=True)
    print(f"jam {jam} pass {pass_name}: done, {len(rows)} points, {time.time() - t0:.0f} s", flush=True)
    return pass_name, rows


def neutral_height(jam):
    sim = Sim(jam)
    for _ in range(3000):
        mujoco.mj_step(sim.m, sim.d)
    z_pivot = sim.cfg["collar_z"] + sim.cfg["sled"]["pivot_height"]
    return [jam, round((sim.d.sensor("head_pos").data[2] - z_pivot) * 1000, 3)]


HEADER = ["jam_deg", "sled_1", "sled_2", "sled_3", "roll", "pitch", "yaw", "off_x", "off_y", "off"]


def main():
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    with Pool(min(30, 2 * len(SWEEP_JAMS) + 1)) as pool:
        cal = pool.map_async(neutral_height, CAL_JAMS)
        res = pool.map(sweep, [(j, p) for j in SWEEP_JAMS for p in "ab"])
        cal = cal.get()
    for p in "ab":
        with open(os.path.join(out_dir, f"sweep_pass_{p}.csv"), "w", newline="") as f:
            w = csv.writer(f); w.writerow(HEADER)
            for name, rows in res:
                if name == p:
                    w.writerows(rows)
    with open(os.path.join(out_dir, "neutral_height.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["jam_deg", "height_mm"]); w.writerows(cal)
    used = {"neck_yaml": BASE, "min_gap_deg": MIN_GAP, "sweep_jams_deg": SWEEP_JAMS, "cal_jams_deg": CAL_JAMS,
            "grid_step_deg": STEP, "grid_limit_deg": LIM, "ramp_steps_per_deg": RAMP_PER_DEG,
            "settle_steps": SETTLE, "timestep_s": 0.001, "mujoco": mujoco.__version__}
    with open(os.path.join(out_dir, "sweep_settings.yaml"), "w") as f:
        yaml.safe_dump(used, f, sort_keys=False)
    print("written", flush=True)


if __name__ == "__main__":
    main()
