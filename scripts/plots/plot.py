# /// script
# requires-python = "==3.12.*"  # mujoco 3.12.0 has no wheels for newer Pythons
# dependencies = [
#     "mujoco==3.12.0",  # the version mujoco_vendor bundles for the sim
#     "numpy",
#     "pyyaml",
#     "matplotlib",
# ]
# ///
"""Populate .plots/ with the neck kinematics plots, from the sim. Data only.

Usage (Linux or WSL, from the repo root):
    just plot
    uv run scripts/plots/plot.py [out_dir]     # default .plots

Sweeps are cached in <out_dir>/sim_cache, keyed by the exact config, sweep settings, generator source and MuJoCo
version, so a change to neck.yaml re-runs only the sweeps it affects. Plots are rebuilt from the CSVs every time.
  ik_sweep/     comparisons built on neck.yaml: jam angle and rod length varied
  this_robbie/  neck.yaml exactly, both twists (CW and CCW viewed from the top)
  plots.md      every plot and table, with what each one shows
"""
import glob, os, sys, time
from types import SimpleNamespace

os.environ.setdefault("MUJOCO_GL", "egl")   # for the renders
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np

import sim as S
import common, compare, robbie

COMPARE_JAMS = list(range(60, 101, 5))                  # deg, plus neck.yaml's own jam
COMPARE_RODS_MM = [70, 78, 87, 96, 105, 115]            # plus neck.yaml's own rod
CAL_JAMS = list(range(40, 121))                         # deg, neutral height calibration


def main(out):
    t0 = time.time()
    base = S.with_twist(S.load_neck(), "CCW")   # the runtime default; ik_sweep uses it, this_robbie does both
    jam_exact = S.jam_of(base)
    cur_jam = round(jam_exact, 1)
    cur_rod = round(base["rod"]["length"] * 1000, 1)
    jams = sorted(set(COMPARE_JAMS) | {cur_jam})
    rods = sorted(set(COMPARE_RODS_MM) | {cur_rod})
    print(f"neck.yaml: rod {cur_rod:g} mm, rest height {base['platform']['rest_height'] * 1000:g} mm -> jam {jam_exact:.3f} deg, "
          f"head.rotation_deg {base['head']['rotation_deg']:g}", flush=True)

    def cfg_for(L, j):
        if j == cur_jam and L == cur_rod:
            return base                                     # neck.yaml verbatim
        return S.with_jam(S.with_rod(base, L / 1000), jam_exact if j == cur_jam else j)

    jobs = {}
    for L in rods:
        for j in jams:
            jobs[L, j, "a"] = (cfg_for(L, j), "a")
            if L == cur_rod:
                jobs[L, j, "b"] = (cfg_for(L, j), "b")
    other = S.with_twist(base, "CW")
    jobs["other", "a"] = (other, "a"); jobs["other", "b"] = (other, "b")
    cache = os.path.join(out, "sim_cache")
    res = S.ensure_sweeps(cache, jobs)
    cal = S.ensure_cal(cache, base, CAL_JAMS)

    for k, (a, ck) in list(res.items()):   # label each run with the jam it was asked for
        a[:, 0] = cur_jam if k[0] == "other" else k[1]
        # head frame: 0 = centre to rod end 1 (docs/head_rotation.svg), so heading axes read as head.rotation_deg
        res[k] = (common.to_frame(a, ck["head_zero_deg"]), ck)
    yaml_dir = res[cur_rod, cur_jam, "a"][1]["direction"]
    other_dir = res["other", "a"][1]["direction"]
    if (yaml_dir, other_dir) != ("CCW", "CW"):
        sys.exit(f"twist directions measured as {yaml_dir} and {other_dir}: expected CCW and CW")

    # Fresh plots each time; the sim cache is kept.
    for folder in ("ik_sweep", "this_robbie"):
        for f in glob.glob(os.path.join(out, folder, "*.png")) + glob.glob(os.path.join(out, folder, "data", "*.csv")):
            os.remove(f)
    book = common.Book(out)
    D = SimpleNamespace(jams=jams, cur_jam=cur_jam, rods=rods, cur_rod=cur_rod, lim=S.LIM, cal=cal, twist_name=yaml_dir,
                        head_rot=base["head"]["rotation_deg"],
                        runs={k: v for k, v in res.items() if k[0] != "other"}, start=base["sled"]["start_deg"],
                        robbie={yaml_dir: (res[cur_rod, cur_jam, "a"][0], res[cur_rod, cur_jam, "b"][0], res[cur_rod, cur_jam, "a"][1]),
                                other_dir: (res["other", "a"][0], res["other", "b"][0], res["other", "a"][1])})
    compare.run(book, D)
    poses = robbie.run(book, D)
    import renders
    try:
        S.mujoco.Renderer(S.Sim(base, render=True).m, 8, 8).close()
    except Exception as e:   # no EGL: say what's missing rather than work round it
        print(f"renders skipped: {type(e).__name__}: {e}\n  (offscreen rendering needs EGL in WSL, e.g. "
              f"sudo apt install libegl1 libgl1)", flush=True)
    else:
        renders.run(book, {yaml_dir: base, other_dir: other},
                    {yaml_dir: f"{yaml_dir} (default)", other_dir: other_dir}, poses)

    cols = ("jam_deg", "rod_len_mm", "twist", "pass_name", "direction", "points", "neutral_height_mm", "rest_change_max",
            "max_abs_yaw", "spring_len_rest_mm", "head_zero_deg")
    checks = [[str(k), *(v[1][c] for c in cols)] for k, v in res.items()]
    book.csv("ik_sweep", "run_checks.csv", ["run", "jam_deg", "rod_len_mm", "twist", "pass", "direction", "points",
                                            "neutral_height_mm", "rest_change_max", "max_abs_yaw", "spring_len_rest_mm",
                                            "head_zero_deg"], checks)
    write_md(out, book, base, cur_jam, cur_rod, yaml_dir, jams, rods)
    print(f"done in {time.time() - t0:.0f} s: {os.path.join(out, 'plots.md')}", flush=True)


def write_md(out, book, base, cur_jam, cur_rod, yaml_dir, jams, rods):
    L = ["# Neck kinematics plots", "",
         "Generated by `just plot` (`scripts/plots/plot.py`) from the MuJoCo sim of "
         "`src/robbie_description/config/neck.yaml`. Everything here is regenerated on each run; sweeps are cached in "
         "`sim_cache/` and re-run when the config they depend on changes.", "",
         "## How the data is made", "",
         f"- **neck.yaml as used:** rod {cur_rod:g} mm, rest height {base['platform']['rest_height'] * 1000:g} mm, which "
         f"gives a jam (twist at rest) of {cur_jam:g} deg; head.rotation_deg {base['head']['rotation_deg']:g}. The twist "
         f"(CW or CCW viewed from the top) is set at runtime, not in neck.yaml; {yaml_dir} is the default.",
         f"- **Sweep:** sled_1 and sled_2 offsets on a {S.STEP} deg grid (sled_3 = -(sled_1 + sled_2)), keeping "
         f"neighbouring sleds at least {S.MIN_GAP} deg apart. Each move is ramped over {S.RAMP_PER_DEG / 1000:g} s per "
         f"degree and settled for {S.SETTLE / 1000:g} s. Pass A walks with sled_1 as the outer loop, pass B with sled_2.",
         "- **Sled contact is off** in the sim; the sled-gap limit bounds the sweep instead. The jam model is unmeasured.",
         "- **Roll, pitch, yaw** are ZYX Euler angles in the head's frame. ik_sweep uses a head at 0; this_robbie uses "
         "neck.yaml's head.rotation_deg. **Off centre** is the platform centre's distance from the neck axis.",
         "- **Head orientation** (head.rotation_deg) is the direction the head faces on the platform: 0 is centre to rod end 1, "
         "counter-clockwise from the top (`docs/head_rotation.svg`).",
         "- **Graceful:** |roll| <= 1 deg and off centre <= 10 mm. **Ugly:** no roll or off-centre limit, the full sled range.",
         "- **Nod line:** halfway round the twist between a sled pivot and its rod's top pivot, at rest.",
         "- **Line styles in trade-off charts:** solid = higher is better, dashed = lower is better, dotted = context.", ""]
    sections = {"ik_sweep": ("ik_sweep: comparisons", f"Built on neck.yaml with the jam angle ({', '.join(f'{j:g}' for j in jams)} deg) "
                             f"and the rod length ({', '.join(f'{r:g}' for r in rods)} mm) varied, {yaml_dir} twist, head at 0. "
                             "Derived CSVs are in `ik_sweep/data/`."),
                "this_robbie": ("this_robbie: neck.yaml", "neck.yaml exactly, in both twists, roll and pitch in the frame of "
                                "the head at head.rotation_deg. Derived CSVs are in `this_robbie/data/`.")}
    for folder, (title, intro) in sections.items():
        L += [f"## {title}", "", intro, ""]
        for e in book.entries.get(folder, []):
            if e[0] == "plot":
                _, fname, heading, about = e
                L += [f"### {fname[:2]} {heading}", "", f"![{heading}]({folder}/{fname})", "", about, ""]
            else:
                _, heading, table, about = e
                L += [f"#### {heading}", "", about, "", table, ""]
    with open(os.path.join(out, "plots.md"), "w") as f:
        f.write("\n".join(L))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".plots")
    sys.stdout.flush(); os._exit(0)   # skip EGL teardown errors at exit
