# /// script
# requires-python = "==3.12.*"  # mujoco 3.12.0 has no wheels for newer Pythons
# dependencies = [
#     "mujoco==3.12.0",  # the version mujoco_vendor bundles for the sim
#     "numpy",
#     "pyyaml",
# ]
# ///
"""Neck IK test: the solver drives the sim to keep the head's gaze on a point moving in front of it.

Usage (Linux or WSL, from the repo root):
    just ik-test [flags]
    uv run scripts/ik/ik_test.py [--ccw|--cw] [--pattern NAME] [--mode auto|none|force] [--gate NAME] [--gain G]
                                 [--gamma S] [--margin MM] [--lam DEG_PER_MM] [--penalty DEG] [--best shortest|central]
                                 [--sled-rate DEG_S] [--period S] [--side M] [--up M] [--dist M] [--headless] [--seconds N]

The target (red) moves on a pattern centred --dist in front of the eyes' rest position; its path is drawn faintly. The
gaze (yellow) runs from the eyes along the head's forward axis. Every 10 ms of sim time the solver gets the head pitch
and yaw that look at the target, roll 0 within the graceful band, and the sleds being commanded; the sleds move towards
its answer at no more than --sled-rate (the sim's springs can flip the neck to the other pole on big fast moves).
A status line each second of sim time, a summary at the end: gaze error, corrections, hops (the solver's answer
moving more than 2 deg of sled in one tick), time ugly (|roll| over 1 deg or off centre over 10 mm) and mean off
centre. The auto gates and their settings are described in robbie_neck/ik.py; just ik-bench compares them.

Patterns (--pattern, or press the key in the viewer; the target glides to the new path over a second):
    1 figure8   sideways sine with a vertical sine at twice the rate (the default)
    2 sweep     sideways sine, level
    3 nod       vertical sine, centred
    4 loop      a vertical ellipse, --side wide and --up high
    5 box       round the rectangle --side by --up at a steady pace
    6 step      jumps between the rectangle's corners every quarter period, crossing the centre: exercises corrections
    7 still     the centre
Push --side or --up past the envelope to see the reachable error. The sweep comes from .plots/sim_cache, shared with
just plot and just config (run once if neck.yaml changed).
"""
import argparse, math, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "plots"))

import mujoco
import numpy as np

import sim as S

sys.path.insert(0, os.path.join(S.REPO, "src", "robbie_neck"))
from robbie_neck import ik

TICK = 10           # sim steps (1 ms) per control tick
EYES = np.array([0.03, 0, 0.025])   # between the eyes, head body frame (gen_neck_mjcf.py)
UNREACH = 0.01      # deg, reachable error above which a tick counts as unreachable
HOP = 2.0           # deg, the solver's answer moving this much in one tick counts as a hop
PATTERNS = ("figure8", "sweep", "nod", "loop", "box", "step", "still")
BOX = [(-1, -1), (1, -1), (1, 1), (-1, 1)]    # round the rectangle
STEP = [(-1, -1), (1, 1), (-1, 1), (1, -1)]   # corner to corner through the centre
BLEND = 1.0         # s, the target glides to a new pattern
PATH_POINTS = 64    # drawn path samples per period


def pattern(name, t, side, up, period):
    """Target offset (sideways, up) in m on pattern name, t seconds into it."""
    w = 2 * math.pi * t / period
    if name == "figure8":
        return side * math.sin(w), up * math.sin(2 * w)
    if name == "sweep":
        return side * math.sin(w), 0.0
    if name == "nod":
        return 0.0, up * math.sin(w)
    if name == "loop":
        return side * math.sin(w), -up * math.cos(w)
    if name == "box":
        u = (t / period) % 1 * 4; k = int(u); f = u - k
        (a, b), (c, d) = BOX[k], BOX[(k + 1) % 4]
        return side * (a + (c - a) * f), up * (b + (d - b) * f)
    if name == "step":
        a, b = STEP[int(t / period * 4) % 4]
        return side * a, up * b
    return 0.0, 0.0


def wrap(deg):
    return (deg + 180) % 360 - 180


def rpy(R):
    """ZYX roll, pitch, yaw (deg) of a rotation matrix."""
    return (math.degrees(math.atan2(R[2, 1], R[2, 2])), math.degrees(-math.asin(max(-1, min(1, R[2, 0])))),
            math.degrees(math.atan2(R[1, 0], R[0, 0])))


def angle(a, b):
    return math.degrees(math.acos(max(-1, min(1, a @ b / (np.linalg.norm(a) * np.linalg.norm(b))))))


class Harness:
    def __init__(self, a):
        self.a = a
        self.pole = "CCW" if a.ccw else "CW"
        cfg = S.with_twist(S.load_neck(), self.pole)
        cache = os.path.join(S.REPO, ".plots", "sim_cache")
        rows, checks = S.ensure_sweeps(cache, {"a": (cfg, "a")})["a"]
        self.ik = ik.NeckIK.from_sweep(rows, cfg, self.pole, gate=a.gate, gain=a.gain, gamma=a.gamma, margin=a.margin,
                                       lam=a.lam, penalty=a.penalty, best=a.best)
        self.sim = S.Sim(cfg, render=True)   # render keeps head.rotation_deg, so the head site faces the real way
        o, R = self.eyes()
        f = math.radians(self.ik.rest_yaw)
        self.side = np.array([-math.sin(f), math.cos(f), 0])
        self.centre = o + a.dist * np.array([math.cos(f), math.sin(f), 0])
        self.pattern, self.t_pat, self.blend, self.want = a.pattern, 0.0, ((0.0, 0.0), 0.0), None   # glide in from the centre
        self.path = self.path_of(a.pattern)
        self.cmd, self.last = np.zeros(3), None   # sled angles being commanded; the solver's last answer (deg)
        self.yaw = rpy(R)[2]                    # unwrapped
        self.since, self.t, self.n = math.inf, 0.0, 0
        self.corrections, self.unreach_s, self.flips, self.hops = 0, 0.0, 0, 0
        self.errs, self.offs, self.rolls = [], [], []   # per tick: gaze error, the sim's off centre and |roll|
        self.sec = self.blank()
        self.tgt = self.target()
        print(f"{self.pole}, pattern {a.pattern}, mode {a.mode}, gate {a.gate} (gain {a.gain}, gamma {a.gamma} s,"
              f" margin {a.margin} mm, lam {a.lam}, penalty {a.penalty}), best {a.best},"
              f" sled rate {a.sled_rate} deg/s; sweep {checks['points']} points;"
              f" rest yaw {self.ik.rest_yaw:.2f} deg (sim {rpy(R)[2]:.2f})", flush=True)

    @staticmethod
    def blank():
        return dict(gaze=0.0, er=0.0, ep=0.0, step=0.0, jump=0.0, sols=set(), graceful=True)

    def eyes(self):
        s = self.sim.d.site("head")
        R = s.xmat.reshape(3, 3).copy()
        return s.xpos + R @ EYES, R

    def place(self, x, z):
        return self.centre + x * self.side + np.array([0, 0, z])

    def offset(self):
        """The target's (sideways, up) now: the pattern, blended from where the last one left it."""
        o = pattern(self.pattern, self.t_pat, self.a.side, self.a.up, self.a.period)
        if self.blend:
            (fx, fz), t0 = self.blend
            u = min(1.0, (self.t - t0) / BLEND)
            o = (fx + (o[0] - fx) * u, fz + (o[1] - fz) * u)
            if u >= 1:
                self.blend = None
        return o

    def target(self):
        return self.place(*self.offset())

    def path_of(self, name):
        if name == "still":
            return []
        pts = [self.place(*pattern(name, self.a.period * i / PATH_POINTS, self.a.side, self.a.up, self.a.period))
               for i in range(PATH_POINTS)]
        return [(pts[i], pts[(i + 1) % PATH_POINTS]) for i in range(PATH_POINTS)]

    def switch(self, name):
        if name != self.pattern:
            self.blend = (self.offset(), self.t)
            self.pattern, self.t_pat, self.path = name, 0.0, self.path_of(name)
            print(f"pattern: {name}", flush=True)

    def key(self, code):
        """Viewer key presses (its own thread): digits pick a pattern, applied on the next tick."""
        if 49 <= code < 49 + len(PATTERNS):   # GLFW key codes of '1'..'7'
            self.want = PATTERNS[code - 49]

    def tick(self):
        a, sec = self.a, self.sec
        if self.want:
            self.switch(self.want); self.want = None
        tgt = self.target()
        o, R = self.eyes()
        d = tgt - o
        gaze = angle(R[:, 0], d)
        yaw = math.degrees(math.atan2(d[1], d[0])); yaw = self.yaw + wrap(yaw - self.yaw); self.yaw = yaw
        pitch = -math.degrees(math.asin(d[2] / np.linalg.norm(d)))
        sol = self.ik.solve(0.0, pitch, yaw, self.cmd, mode=a.mode, since=self.since)
        if sol.corrected:
            self.since = 0.0; self.corrections += 1
        if self.last is not None:   # the solver's answer moving between ticks: a sheet change shows as a jump
            jump = float(np.max(np.abs(sol.sleds - self.last)))
            sec["jump"] = max(sec["jump"], jump); self.hops += jump > HOP
        self.last = sol.sleds
        lim = a.sled_rate * TICK / 1000
        step = np.clip(sol.sleds - self.cmd, -lim, lim)
        self.cmd = self.cmd + step
        self.sim.d.ctrl[:] = self.sim.start + np.radians(self.cmd)
        self.sim.step(TICK)
        self.t += TICK / 1000; self.t_pat += TICK / 1000; self.since += TICK / 1000; self.n += 1
        unreach = max(sol.err_roll, sol.err_pitch) > UNREACH
        self.unreach_s += unreach * TICK / 1000
        self.errs.append(gaze)
        off = math.hypot(*self.sim.head()[1][:2]) * 1000
        self.offs.append(off); self.rolls.append(abs(rpy(R)[0]))
        sec.update(gaze=max(sec["gaze"], gaze), er=max(sec["er"], sol.err_roll), ep=max(sec["ep"], sol.err_pitch),
                   step=max(sec["step"], float(np.max(np.abs(step)))), graceful=sec["graceful"] and sol.graceful)
        sec["sols"].add(sol.solutions)
        self.tgt = tgt
        if self.n % (1000 // TICK) == 0:
            self.status()

    def status(self):
        sec = self.sec
        pole = S.direction(self.sim.orbit())
        if pole != self.pole:
            self.flips += 1
        off = math.hypot(*self.sim.head()[1][:2]) * 1000
        print(f"t {self.t:5.1f} s  {self.pattern:7} {self.a.mode:5}  gaze {self.errs[-1]:5.2f} (max {sec['gaze']:5.2f}) deg"
              f"  reach err r {sec['er']:4.2f} p {sec['ep']:5.2f} deg  off {off:5.1f} mm"
              f"  graceful {'yes' if sec['graceful'] else 'no ':3}  corr {self.corrections:3d}"
              f"  sols {','.join(map(str, sorted(sec['sols'])))}  step {sec['step']:.2f} jump {sec['jump']:5.2f} deg"
              + ("" if pole == self.pole else f"  POLE {pole}"), flush=True)
        self.sec = self.blank()

    def metrics(self):
        e = np.array(self.errs)
        rolls, offs = np.array(self.rolls), np.array(self.offs)
        by_roll, by_off = rolls > ik.GRACEFUL_ROLL, offs > ik.GRACEFUL_OFF
        return dict(gaze_mean=float(e.mean()), gaze_max=float(e.max()), corrections=self.corrections, hops=self.hops,
                    ugly_pct=100 * float((by_roll | by_off).mean()), ugly_roll_pct=100 * float(by_roll.mean()),
                    ugly_off_pct=100 * float(by_off.mean()), roll_max=float(rolls.max()), off_mean=float(offs.mean()),
                    unreach_s=self.unreach_s, flipped=S.direction(self.sim.orbit()) != self.pole)

    def summary(self):
        m = self.metrics()
        pole = S.direction(self.sim.orbit())
        print(f"\nsummary, {self.t:.1f} s {self.pole} {self.a.mode} {self.a.gate}, last pattern {self.pattern}:"
              f" gaze error mean {m['gaze_mean']:.2f} max {m['gaze_max']:.2f} deg; corrections {m['corrections']};"
              f" hops {m['hops']}; ugly {m['ugly_pct']:.0f}% of the time (roll {m['ugly_roll_pct']:.0f}%, off"
              f" {m['ugly_off_pct']:.0f}%); roll max {m['roll_max']:.2f} deg; off centre mean {m['off_mean']:.1f} mm;"
              f" unreachable {m['unreach_s']:.2f} s; pole {pole} ({'still correct' if pole == self.pole else 'FLIPPED'}"
              f", {self.flips} bad checks)", flush=True)


def draw(scn, h):
    scn.ngeom = 0
    eye = np.eye(3).ravel()

    def capsule(a, b, radius, rgba):
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]; scn.ngeom += 1
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), eye, np.array(rgba, np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, a, b)

    g = scn.geoms[scn.ngeom]; scn.ngeom += 1
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.01, 0, 0]), h.tgt, eye,
                        np.array([0.9, 0.1, 0.1, 1], np.float32))   # target
    o, R = h.eyes()
    capsule(o, o + R[:, 0] * np.linalg.norm(h.tgt - o), 0.002, [1, 0.85, 0.1, 1])   # gaze
    capsule(o, h.tgt, 0.002, [0.9, 0.1, 0.1, 0.3])                                  # eyes to target, faint
    for a, b in h.path:                                                             # the pattern's path
        capsule(a, b, 0.001, [0.9, 0.1, 0.1, 0.25])


def parser():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    pole = p.add_mutually_exclusive_group()
    pole.add_argument("--ccw", action="store_true", help="the CCW pole, the one that looks down")
    pole.add_argument("--cw", action="store_true", help="the CW pole, the one that looks up (default)")
    p.add_argument("--pattern", choices=PATTERNS, default="figure8", help="the target's path; keys 1-7 in the viewer")
    p.add_argument("--mode", choices=ik.MODES, default="auto")
    p.add_argument("--gate", choices=ik.GATES, default="cost", help="auto's gate; robbie_neck/ik.py describes them")
    p.add_argument("--gain", type=float, default=0.45, help="first_blush")
    p.add_argument("--gamma", type=float, default=2.0, help="s: first_blush's ramp, the others' dwell")
    p.add_argument("--margin", type=float, default=3.0, help="mm: margin and deadband")
    p.add_argument("--lam", type=float, default=3.0, help="deg of sled per mm off centre: cost")
    p.add_argument("--penalty", type=float, default=1.0, help="deg of sled for not taking the least movement: cost")
    p.add_argument("--best", choices=ik.BESTS, default="shortest")
    p.add_argument("--sled-rate", type=float, default=40, help="deg/s, the most any sled is commanded to move")
    p.add_argument("--period", type=float, default=8, help="s, once round the pattern")
    p.add_argument("--side", type=float, default=0.3, help="m, target sideways amplitude")
    p.add_argument("--up", type=float, default=0.02, help="m, target vertical amplitude")
    p.add_argument("--dist", type=float, default=0.5, help="m, pattern centre in front of the eyes at rest")
    p.add_argument("--headless", action="store_true", help="no viewer: run --seconds of sim time, print the summary")
    p.add_argument("--seconds", type=float, default=20, help="sim seconds, --headless only")
    return p


def main():
    a = parser().parse_args()
    h = Harness(a)
    try:
        if a.headless:
            while h.t < a.seconds - 1e-9:
                h.tick()
        else:
            import mujoco.viewer
            with mujoco.viewer.launch_passive(h.sim.m, h.sim.d, key_callback=h.key) as v:
                t0 = time.time()
                while v.is_running():
                    with v.lock():
                        h.tick()
                        draw(v.user_scn, h)
                    v.sync()
                    ahead = h.t - (time.time() - t0)   # paced to wall-clock time
                    if ahead > 0:
                        time.sleep(ahead)
    except KeyboardInterrupt:   # Ctrl-C: still give the summary
        pass
    if h.errs:
        h.summary()


if __name__ == "__main__":
    main()
