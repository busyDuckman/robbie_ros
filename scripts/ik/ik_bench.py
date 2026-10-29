# /// script
# requires-python = "==3.12.*"  # mujoco 3.12.0 has no wheels for newer Pythons
# dependencies = [
#     "mujoco==3.12.0",  # the version mujoco_vendor bundles for the sim
#     "numpy",
#     "pyyaml",
# ]
# ///
"""Tune and compare the IK's auto gates: ik_test headless for every gate with a few settings each, over the patterns
and both poles, in parallel. Prints progress as runs finish, then one table over everything and one per pattern.

    just ik-bench [--seconds 16] [--patterns figure8,sweep,loop,box,step] [--workers N]

What to read: ugly % and off centre say how well a gate keeps the head graceful, corrections and hops say how much it
fidgets to do it, gaze error says what tracking it costs. none and force are the two policies without a gate, for
reference. Edit CONFIGS to try other settings.
"""
import argparse, contextlib, io, multiprocessing as mp, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ik_test as T

CONFIGS = [   # label, flags as ik_test takes them
    ("none", dict(mode="none")),
    ("force", dict(mode="force")),
    ("first_blush gain .45 gamma 2", dict(gate="first_blush", gain=0.45, gamma=2)),
    ("first_blush gain .9 gamma 1", dict(gate="first_blush", gain=0.9, gamma=1)),
    ("margin dwell 1 margin 3", dict(gate="margin", gamma=1, margin=3)),
    ("deadband dwell 1 margin 3", dict(gate="deadband", gamma=1, margin=3)),
    ("cost lam 1 penalty 0", dict(gate="cost", lam=1, penalty=0)),
    ("cost lam 3 penalty .5", dict(gate="cost", lam=3, penalty=0.5)),
    ("cost lam 3 penalty 1", dict(gate="cost", lam=3, penalty=1)),
    ("cost lam 3 penalty 2", dict(gate="cost", lam=3, penalty=2)),
    ("cost lam 5 penalty 1", dict(gate="cost", lam=5, penalty=1)),
    ("cost lam 10 penalty 1", dict(gate="cost", lam=10, penalty=1)),
]
COLS = [("gaze_mean", "gaze mean", "{:6.2f}"), ("gaze_max", "gaze max", "{:6.2f}"), ("corrections", "corr", "{:5.0f}"),
        ("hops", "hops", "{:5.0f}"), ("ugly_pct", "ugly %", "{:6.1f}"), ("ugly_roll_pct", "by roll", "{:6.1f}"),
        ("ugly_off_pct", "by off", "{:6.1f}"), ("roll_max", "roll max", "{:6.2f}"), ("off_mean", "off mm", "{:6.1f}")]


def run(job):
    label, flags, pattern, pole, seconds = job
    a = T.parser().parse_args([])
    vars(a).update(flags, pattern=pattern, cw=pole == "CW", ccw=pole == "CCW", headless=True, seconds=seconds)
    with contextlib.redirect_stdout(io.StringIO()):   # the harness's status lines
        h = T.Harness(a)
        while h.t < seconds - 1e-9:
            h.tick()
    return label, pattern, pole, h.metrics()


def table(title, rows, labels):
    """rows: {label: [metrics dicts]}; aggregated per label: means, except gaze max (max) and counts (sums)."""
    print(f"\n{title}")
    print(f"{'':32}" + "".join(f"{name:>10}" for _, name, _ in COLS))
    for label in labels:
        ms = rows.get(label, [])
        if not ms:
            continue
        cells = []
        for key, _, fmt in COLS:
            vals = [m[key] for m in ms]
            v = max(vals) if key == "gaze_max" else sum(vals) if key in ("corrections", "hops") else sum(vals) / len(vals)
            cells.append(f"{fmt.format(v):>10}")
        flipped = sum(m["flipped"] for m in ms)
        print(f"{label:32}" + "".join(cells) + (f"   FLIPPED x{flipped}" if flipped else ""))


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--seconds", type=float, default=16, help="sim seconds per run")
    p.add_argument("--patterns", default="figure8,sweep,loop,box,step", help="comma separated")
    p.add_argument("--workers", type=int, default=max(1, min(6, os.cpu_count() or 1)))
    a = p.parse_args()
    patterns = a.patterns.split(",")
    jobs = [(label, flags, pat, pole, a.seconds) for label, flags in CONFIGS for pat in patterns for pole in ("CW", "CCW")]
    labels = [label for label, _ in CONFIGS]
    print(f"{len(jobs)} runs of {a.seconds:g} s on {a.workers} workers", flush=True)
    t0 = time.time()
    results = []
    with mp.Pool(a.workers) as pool:
        for n, (label, pat, pole, m) in enumerate(pool.imap_unordered(run, jobs), 1):
            results.append((label, pat, pole, m))
            print(f"  {n:3d}/{len(jobs)}  {time.time() - t0:5.0f} s  {label:32} {pat:8} {pole:3}  gaze {m['gaze_mean']:.2f}"
                  f"  corr {m['corrections']:3d}  ugly {m['ugly_pct']:3.0f}%", file=sys.stderr, flush=True)
    by_label = {}
    for label, pat, pole, m in results:
        by_label.setdefault(label, []).append(m)
    table(f"all patterns and both poles ({len(patterns) * 2} runs each; gaze deg, counts summed, the rest averaged)", by_label, labels)
    for pat in patterns:
        rows = {}
        for label, p_, pole, m in results:
            if p_ == pat:
                rows.setdefault(label, []).append(m)
        table(f"{pat} (both poles)", rows, labels)


if __name__ == "__main__":
    main()
