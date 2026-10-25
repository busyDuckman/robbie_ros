# /// script
# requires-python = "==3.12.*"  # mujoco 3.12.0 has no wheels for newer Pythons
# dependencies = [
#     "mujoco==3.12.0",
#     "numpy",
#     "pyyaml",
#     "matplotlib",
# ]
# ///
"""Sim renders of several sled settings that give the same head roll/pitch at jam 75. Data only.

Usage (Linux or WSL, from the repo root):
    MUJOCO_GL=egl uv run scripts/ik_simulation/branch_renders.py .output/ik_sweep/75_jam/branches/renders
"""
import importlib.util, os, sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("sweep", os.path.join(HERE, "sweep.py"))
sw = importlib.util.module_from_spec(spec); spec.loader.exec_module(sw)

JAM = 75
# (title, sled_1, sled_2): sled settings from sweep_pass_a.csv at jam 75
SETS = {
    "roll_5_pitch_0": [("rest", 0, 0), ("A", -38, 36), ("B", 1, 16), ("C", 18, 18)],
    "roll_0_pitch_-5": [("rest", 0, 0), ("A", 15, -22), ("B", 43, 8)],
}
# (label, azimuth, elevation): MuJoCo free camera, azimuth 0 looks along +X
VIEWS = [("side, from -Y", 90, -5), ("side, from +X (head faces +X)", 180, -5), ("top", 90, -89)]


def main(out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(out_dir, exist_ok=True)
    sim = sw.Sim(JAM)
    r = mujoco.Renderer(sim.m, 480, 480)
    cam = mujoco.MjvCamera()
    for name, rows in SETS.items():
        fig, axes = plt.subplots(len(rows), len(VIEWS), figsize=(3.4 * len(VIEWS), 3.6 * len(rows)),
                                 constrained_layout=True)
        for ri, (label, s1, s2) in enumerate(rows):
            sim.go([0, 0, 0]); sim.go([s1, s2, -s1 - s2])
            (roll, pitch, yaw), pos = sim.head()
            off = np.hypot(pos[0], pos[1]) * 1000
            print(f"{name} {label}: sleds ({s1}, {s2}, {-s1 - s2}) roll {roll:.2f} pitch {pitch:.2f} "
                  f"yaw {yaw:.1f} off {off:.1f}", flush=True)
            for vi, (vlabel, az, el) in enumerate(VIEWS):
                cam.lookat[:] = [0, 0, pos[2] * 0.6]
                cam.distance, cam.azimuth, cam.elevation = 0.38, az, el
                r.update_scene(sim.d, cam)
                ax = axes[ri, vi]
                ax.imshow(r.render()); ax.set_xticks([]); ax.set_yticks([])
                if ri == 0:
                    ax.set_title(vlabel, fontsize=10)
                if vi == 0:
                    ax.set_ylabel(f"{label}: sleds ({s1}, {s2}, {-s1 - s2})\nroll {roll:.1f}, pitch {pitch:.1f}, "
                                  f"yaw {yaw:.1f}\noff {off:.1f} mm", fontsize=9)
        fig.suptitle(f"Jam {JAM}: different sled settings, same head roll/pitch (sim renders)", fontsize=11)
        fig.savefig(os.path.join(out_dir, f"{name}.png"), dpi=110); plt.close(fig)


if __name__ == "__main__":
    main(sys.argv[1])
