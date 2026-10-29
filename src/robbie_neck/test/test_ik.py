"""The IK solver on made-up tables (always), and on neck.yaml's cached sweep and the sim when .plots/sim_cache has it."""
import math, os

import numpy as np
import pytest

from robbie_neck import ik

LIM = 20


def table(f):
    """A sweep-shaped array from f(s1, s2) -> (roll, pitch, yaw, off), on the |s| <= LIM grid, no gap limit."""
    rows = []
    for s1 in range(-LIM, LIM + 1):
        for s2 in range(-LIM, LIM + 1):
            roll, pitch, yaw, off = f(s1, s2)
            rows.append([0, s1, s2, -s1 - s2, roll, pitch, yaw, off, 0, off])
    return np.array(rows, float)


AFFINE = table(lambda s1, s2: (0.3 * s1 - 0.1 * s2, 0.1 * s1 + 0.3 * s2, 0.05 * (s1 - s2) + 2, 0.8 * math.hypot(s1, s2)))
# two sheets: roll = s1^2 / 40 - 5 folds at s1 = 0; the s1 < 0 sheet is the central one (off = |s1 + 10|). No residual
# yaw, so the sled angles read straight off the sheet.
FOLDED = table(lambda s1, s2: (s1 * s1 / 40 - 5, 0.3 * s2, 0, abs(s1 + 10)))


def test_affine_round_trip_is_exact():
    assert ik.NeckIK(AFFINE, facing_deg=30).rest_yaw == pytest.approx(32)   # residual yaw 2 at rest, plus the facing
    k = ik.NeckIK(AFFINE, facing_deg=0, roll_tol=0)   # facing 0 keeps the table affine: turning the head re-expresses the angles
    rng = np.random.default_rng(0)
    for roll, pitch, yaw in rng.uniform(-3, 3, (50, 3)):
        sol = k.solve(roll, pitch, yaw, [1, 2, 3], mode="none")
        assert sol.solutions == 1 and sol.err_roll == 0 and sol.err_pitch == 0
        f = k.forward(sol.sleds)
        assert (f.roll, f.pitch, f.yaw) == pytest.approx((roll, pitch, yaw), abs=1e-9)
        assert f.inside and sol.off == pytest.approx(f.off, abs=0.05)   # off is a norm, not affine: two interpolations


def test_roll_band_takes_the_least_movement_within_the_band():
    exact, band = ik.NeckIK(AFFINE, facing_deg=0, roll_tol=0), ik.NeckIK(AFFINE, facing_deg=0)   # the graceful band
    rng = np.random.default_rng(2)
    for roll, pitch, yaw in rng.uniform(-3, 3, (30, 3)):
        cur = rng.uniform(-5, 5, 3)
        a, b = exact.solve(roll, pitch, yaw, cur, mode="none"), band.solve(roll, pitch, yaw, cur, mode="none")
        assert b.solutions == 1 and b.err_roll == 0 and b.err_pitch == 0
        assert abs(b.roll - roll) <= ik.GRACEFUL_ROLL + 1e-9
        f = band.forward(b.sleds)
        assert (f.roll, f.pitch, f.yaw) == pytest.approx((b.roll, pitch, yaw), abs=1e-9)
        assert np.abs(b.sleds - cur).sum() <= np.abs(a.sleds - cur).sum() + 1e-9   # the band can only save movement


def test_roll_band_on_the_folded_table():
    k = ik.NeckIK(FOLDED, facing_deg=0)   # at pitch 3 (s2 = 10) the band |roll| <= 1 is s1 in +-[12.65, 15.49]
    sol = k.solve(0, 3, 0, [14, 10, -24], mode="none")   # already in the band: stay put
    assert sol.solutions == 2 and sol.sleds == pytest.approx([14, 10, -24], abs=1e-6) and sol.roll == pytest.approx(-0.1)
    sol = k.solve(0, 3, 0, [16, 10, -26], mode="none")   # roll 1.4, outside: move to the band's edge
    assert sol.sleds[0] == pytest.approx(math.sqrt(240), abs=0.05) and sol.roll == pytest.approx(1, abs=1e-6)
    sol = k.solve(0, 3, 0, [13, 11, -24], mode="none")   # movement is flat for s1 in [13, 14]: both ends are candidates
    assert 13 - 1e-6 <= sol.sleds[0] <= 14 + 1e-6 and np.abs(sol.sleds - [13, 11, -24]).sum() == pytest.approx(2, abs=1e-6)


def test_unreachable_returns_the_nearest_edge_point():
    for tol in (0, ik.GRACEFUL_ROLL):
        k = ik.NeckIK(AFFINE, facing_deg=0, roll_tol=tol)
        sol = k.solve(50, 0, 0, [0, 0, 0])
        assert sol.solutions == 0 and sol.err_roll > 0 and sol.err_pitch >= 0
        assert k.forward(sol.sleds).inside
        d = math.hypot(sol.err_roll, sol.err_pitch)
        v = k.v[k.feasible]
        assert d <= np.hypot(np.maximum(np.abs(v[:, ik.R] - 50) - tol, 0), v[:, ik.P]).min() + 1e-9   # as near as any grid point


def test_graceful_flag():
    k = ik.NeckIK(AFFINE, facing_deg=0, roll_tol=0)
    assert k.solve(0.5, 0, 0, [0, 0, 0]).graceful
    assert not k.solve(2, 0, 0, [0, 0, 0]).graceful          # roll
    assert not k.solve(0, 5, 0, [0, 0, 0]).graceful          # off: 0.8 * |(5, 15)| = 12.6 mm


def test_bad_arguments():
    k = ik.NeckIK(AFFINE, facing_deg=0)
    with pytest.raises(ValueError):
        k.solve(0, 0, 0, [0, 0, 0], mode="maybe")
    with pytest.raises(ValueError):
        ik.NeckIK(AFFINE, facing_deg=0, best="nicest")


def folded():
    k = ik.NeckIK(FOLDED, facing_deg=0, gate="first_blush", gain=0.45, gamma=2, roll_tol=0)   # exact roll: one point per sheet
    c = k._in_table(np.array([0.0, 3.0]))
    assert len(c) == 2
    assert sorted(c[:, ik.S1]) == pytest.approx([-math.sqrt(200), math.sqrt(200)], abs=0.05)
    assert c[:, ik.S2] == pytest.approx([10, 10], abs=1e-6)
    return k


def test_none_takes_the_shortest_move():
    k = folded()
    assert k.solve(0, 3, 0, [14, 10, -24], mode="none").sleds[0] == pytest.approx(14.14, abs=0.05)
    assert k.solve(0, 3, 0, [-14, 10, 4], mode="none").sleds[0] == pytest.approx(-14.14, abs=0.05)


def test_force_takes_the_most_central():
    k = folded()
    sol = k.solve(0, 3, 0, [14, 10, -24], mode="force")
    assert sol.sleds[0] == pytest.approx(-14.14, abs=0.05) and sol.off == pytest.approx(4.14, abs=0.05)
    assert not sol.corrected


def test_auto_gate_opens_with_time():
    k = folded()
    cur = [14, 10, -24]   # on the far sheet, off about 24; the other sheet offers off about 4
    assert k.forward(cur).off == pytest.approx(24, abs=0.05)
    just_now = k.solve(0, 3, 0, cur, mode="auto", since=0)
    assert just_now.sleds[0] == pytest.approx(14.14, abs=0.05) and not just_now.corrected
    later = k.solve(0, 3, 0, cur, mode="auto", since=100)
    assert later.sleds[0] == pytest.approx(-14.14, abs=0.05) and later.corrected
    # half open: kappa = 0.45 * 1 = 0.45, gate is off < 0.45 * 24 = 10.8, the 4.14 candidate gets in
    assert k.solve(0, 3, 0, cur, mode="auto", since=1).corrected
    # a gate that never lets anything in (gain * gamma tiny)
    tight = ik.NeckIK(FOLDED, facing_deg=0, gate="first_blush", gain=0.01, gamma=2, roll_tol=0)
    assert not tight.solve(0, 3, 0, cur, mode="auto", since=100).corrected


def test_corrected_only_when_the_pick_changes():
    k = folded()
    sol = k.solve(0, 3, 0, [-14, 10, 4], mode="auto", since=100)   # already on the central sheet
    assert sol.sleds[0] == pytest.approx(-14.14, abs=0.05) and not sol.corrected
    one = ik.NeckIK(AFFINE, facing_deg=0, roll_tol=0)                # one sheet, one point: nothing to correct to
    sol = one.solve(0, 0, 0, [10, 10, -20], mode="auto", since=100)
    assert sol.solutions == 1 and sol.off < one.forward([10, 10, -20]).off and not sol.corrected
    # the gate admits the least-movement pick itself (it is more central than where the head is): no correction
    wide = ik.NeckIK(FOLDED, facing_deg=0, gate="first_blush", gain=10, gamma=1, roll_tol=0)
    sol = wide.solve(0, 3, 0, [15, 10, -25], mode="auto", since=100)   # from off 25, gate off < 250: both sheets get in
    assert sol.sleds[0] == pytest.approx(14.14, abs=0.05) and not sol.corrected


# two sheets again, but off = |s1 - 3|: at roll -4.375 (s1 = +-5) the s1 < 0 sheet is graceful at 8 mm and the other
# sheet is better still at 2 mm
OFFSET = table(lambda s1, s2: (s1 * s1 / 40 - 5, 0.3 * s2, 0, abs(s1 - 3)))


def test_margin_gate_needs_a_real_gain_after_a_dwell():
    k = ik.NeckIK(FOLDED, facing_deg=0, gate="margin", gamma=1, margin=3, roll_tol=0)
    cur = [14, 10, -24]   # least movement stays at off 24.14; the other sheet offers 4.14
    assert not k.solve(0, 3, 0, cur, mode="auto", since=0.5).corrected          # still in the dwell
    sol = k.solve(0, 3, 0, cur, mode="auto", since=1)
    assert sol.corrected and sol.sleds[0] == pytest.approx(-14.14, abs=0.05)
    small = ik.NeckIK(FOLDED, facing_deg=0, gate="margin", gamma=1, margin=25, roll_tol=0)   # 20 mm better is not enough
    assert not small.solve(0, 3, 0, cur, mode="auto", since=100).corrected
    assert ik.NeckIK(OFFSET, facing_deg=0, gate="margin", gamma=1, margin=3, roll_tol=0).solve(
        -4.375, 3, 0, [-5, 10, -5], mode="auto", since=10).corrected               # 8 -> 2 mm beats the margin


def test_deadband_gate_leaves_a_graceful_head_alone():
    k = ik.NeckIK(OFFSET, facing_deg=0, gate="deadband", gamma=1, margin=3, roll_tol=0)
    sol = k.solve(-4.375, 3, 0, [-5, 10, -5], mode="auto", since=10)   # going to 8 mm: graceful, so no correction
    assert not sol.corrected and sol.sleds[0] == pytest.approx(-5, abs=0.05) and sol.off == pytest.approx(8, abs=0.05)
    k = ik.NeckIK(FOLDED, facing_deg=0, gate="deadband", gamma=1, margin=3, roll_tol=0)
    sol = k.solve(0, 3, 0, [14, 10, -24], mode="auto", since=10)      # going to 24 mm: ugly, correct to 4 mm
    assert sol.corrected and sol.sleds[0] == pytest.approx(-14.14, abs=0.05)
    assert not k.solve(0, 3, 0, [14, 10, -24], mode="auto", since=0.5).corrected   # but not inside the dwell


def test_cost_gate_weighs_movement_against_off_centre():
    far = [-5, 10, -5]   # from here: stay costs 8 mm of off; moving 20 deg of sled (sleds 1 and 3, 10 each) buys 6 mm
    assert not ik.NeckIK(OFFSET, facing_deg=0, gate="cost", lam=1, penalty=5, roll_tol=0).solve(
        -4.375, 3, 0, far, mode="auto").corrected                                # 8 vs 20 + 2 + 5
    sol = ik.NeckIK(OFFSET, facing_deg=0, gate="cost", lam=4, penalty=0, roll_tol=0).solve(-4.375, 3, 0, far, mode="auto")
    assert sol.corrected and sol.sleds[0] == pytest.approx(5, abs=0.05)         # 32 vs 20 + 8
    with pytest.raises(ValueError):
        ik.NeckIK(OFFSET, facing_deg=0, gate="vibes")
    k = ik.NeckIK(OFFSET, facing_deg=0)   # the default gate and settings: cost, lam 3, penalty 1 (the ik-bench pick)
    assert (k.gate, k.lam, k.penalty) == ("cost", 3, 1)


def test_yaw_is_all_sleds_in_unison():
    k = ik.NeckIK(AFFINE, facing_deg=10, roll_tol=0)
    a = k.solve(1, 1, 0, [0, 0, 0]).sleds
    b = k.solve(1, 1, 25, [0, 0, 0]).sleds
    assert b - a == pytest.approx([25, 25, 25], abs=1e-9)


# ----------------------------------------------------------------------------- neck.yaml's sweep and the sim
def real():
    S = pytest.importorskip("sim")   # scripts/plots/sim.py needs the mujoco bindings: just test has them, colcon test may not
    cfg = S.load_neck()
    out = {}
    for pole in ("CW", "CCW"):
        c = S.with_twist(cfg, pole)
        p = os.path.join(S.REPO, ".plots", "sim_cache", S.key("sweep_a", c) + ".csv")
        if not os.path.exists(p):
            pytest.skip("no cached sweep for neck.yaml: run just plot or just config first")
        out[pole] = (c, np.loadtxt(p, delimiter=",", skiprows=1, ndmin=2))
    return out


@pytest.mark.parametrize("pole", ["CW", "CCW"])
def test_sweep_rows_are_recovered(pole):
    cfg, rows = real()[pole]
    k = ik.NeckIK.from_sweep(rows, cfg, pole, roll_tol=0)
    band = ik.NeckIK.from_sweep(rows, cfg, pole)
    head = ik.to_head(rows, k.facing)
    rng = np.random.default_rng(1)
    for r in rng.choice(len(rows), 100, replace=False):
        c = k._in_table(head[r, [ik.COLS["roll"], ik.COLS["pitch"]]])
        assert np.hypot(c[:, ik.S1] - rows[r, 1], c[:, ik.S2] - rows[r, 2]).min() < 0.05
        sol = k.solve(head[r, 4], head[r, 5], k.rest_yaw + 10, rows[r, 1:4], mode="none")
        assert sol.err_roll == 0 and sol.err_pitch == 0
        f = k.forward(sol.sleds)   # bilinear vs the triangles: agree to well under a cell where the table is curved
        assert (f.roll, f.pitch) == pytest.approx((head[r, 4], head[r, 5]), abs=0.3)
        assert f.yaw == pytest.approx(k.rest_yaw + 10, abs=0.3)
        b = band.solve(head[r, 4], head[r, 5], k.rest_yaw + 10, rows[r, 1:4], mode="none")
        assert b.err_roll == 0 and b.err_pitch == 0 and abs(b.roll - head[r, 4]) <= ik.GRACEFUL_ROLL + 1e-9
        assert np.abs(b.sleds - rows[r, 1:4]).sum() <= np.abs(sol.sleds - rows[r, 1:4]).sum() + 1e-6


def test_sim_head_lands_on_the_solved_pose():
    S = pytest.importorskip("sim")
    cfg, rows = real()["CCW"]
    k = ik.NeckIK.from_sweep(rows, cfg, "CCW")
    sim = S.Sim(cfg, render=True)   # keeps head.rotation_deg, so the head site faces as neck.yaml says

    def head_site():
        m = sim.d.site("head").xmat.reshape(3, 3)
        return math.degrees(math.atan2(m[2, 1], m[2, 2])), math.degrees(-math.asin(m[2, 0])), math.degrees(math.atan2(m[1, 0], m[0, 0]))

    assert head_site()[2] == pytest.approx(k.rest_yaw, abs=0.01)
    cur = np.zeros(3)
    for roll, pitch, dyaw in ((0, 6, 15), (-2, 2, -20)):
        sol = k.solve(roll, pitch, k.rest_yaw + dyaw, cur, mode="none"); cur = sol.sleds
        assert sol.solutions and sol.err_roll == 0 and sol.err_pitch == 0
        assert abs(sol.roll - roll) <= ik.GRACEFUL_ROLL + 1e-9 and sol.pitch == pitch
        sim.go(cur)
        assert head_site() == pytest.approx((sol.roll, sol.pitch, k.rest_yaw + dyaw), abs=0.05)
        _, p = sim.head()
        assert math.hypot(p[0], p[1]) * 1000 == pytest.approx(sol.off, abs=0.2)
