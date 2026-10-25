# /// script
# requires-python = "==3.12.*"  # mujoco 3.12.0 has no wheels for newer Pythons
# dependencies = [
#     "mujoco==3.12.0",  # the version mujoco_vendor bundles for the sim
#     "numpy",
#     "pyyaml",
# ]
# ///
"""Path dependence check at one jam angle: drive to the same sled command from several far-apart starts
and compare the settled head pose. Uses sweep.py's Sim (same model, ramp and settle). Data only.

Usage (Linux or WSL, from the repo root):
    uv run scripts/ik_simulation/branches_hysteresis.py .output/ik_sweep 75
"""
import csv, importlib.util, math, os, sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("sweep", os.path.join(HERE, "sweep.py"))
sw = importlib.util.module_from_spec(spec); spec.loader.exec_module(sw)

src, jam = sys.argv[1], int(float(sys.argv[2]))
out = os.path.join(src, f"{jam}_jam", "branches")
os.makedirs(out, exist_ok=True)
STARTS = {"centre": (0, 0), "corner_1": (60, 0), "corner_2": (-60, 60), "corner_3": (0, -60)}
rng = np.random.default_rng(1)
targets = [(0, 0), (5, 5), (-5, 0), (0, -5), (10, -5), (-10, 10), (20, -20), (-20, 0), (0, 20), (30, 0), (-30, 30), (0, -30)]
while len(targets) < 24:
    t = tuple(int(x) for x in rng.integers(-62, 63, 2))
    if sw.feasible(*t) and t not in targets:
        targets.append(t)

rows = []
for k, (s1, s2) in enumerate(targets):
    for name, (a1, a2) in STARTS.items():
        sim = sw.Sim(jam)
        sim.go([a1, a2, -a1 - a2])
        sim.go([s1, s2, -s1 - s2])     # one straight ramp in sled space, inside the feasible triangle
        (roll, pitch, yaw), p = sim.head()
        rows.append([s1, s2, name, round(roll, 3), round(pitch, 3), round(yaw, 3),
                     round(math.hypot(p[0], p[1]) * 1000, 3)])
    sp = np.array([r[3:] for r in rows[-len(STARTS):]])
    print(f"{k + 1}/{len(targets)} sled ({s1}, {s2}): spread roll {np.ptp(sp[:, 0]):.3f} pitch {np.ptp(sp[:, 1]):.3f} "
          f"yaw {np.ptp(sp[:, 2]):.3f} deg, off {np.ptp(sp[:, 3]):.3f} mm", flush=True)
with open(os.path.join(out, "hysteresis.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["sled_1", "sled_2", "approach_from", "roll", "pitch", "yaw", "off"]); w.writerows(rows)
a = np.array([r[3:] for r in rows]).reshape(len(targets), len(STARTS), 4)
print("largest spread over all targets: roll %.3f pitch %.3f yaw %.3f deg, off %.3f mm" % tuple(np.ptp(a, axis=1).max(0)),
      flush=True)
