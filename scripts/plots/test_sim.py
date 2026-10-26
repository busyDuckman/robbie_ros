"""Sanity tests for the neck model and the sim helpers the plots trust. A few seconds; run by just test and CI.

The golden points are neck.yaml as committed. Changing neck.yaml or gen_neck_mjcf.py on purpose means updating
GOLDEN on purpose: print the new values with `uv run ... python -c` and check they make sense first.
"""
import math

import mujoco
import numpy as np
import pytest

import common
import sim as S

CFG = S.with_twist(S.load_neck(), "CCW")

# sled offsets (deg) -> platform roll, pitch, yaw (deg), off (mm). Straight from rest, neck.yaml, CCW twist.
GOLDEN = {
    (20, 0, -20): (-2.721, -2.083, -8.490, 11.26),
    (-20, 10, 10): (-2.320, 1.098, -8.389, 10.34),
    (10, 25, -35): (7.658, 2.644, -15.285, 14.66),
    (-30, -10, 40): (-3.040, 13.741, -14.640, 11.66),
    (0, -25, 25): (-0.862, 3.991, -10.801, 13.64),
}


@pytest.mark.parametrize("pole", ["CW", "CCW"])
def test_model_builds_and_loads(pole):
    m = mujoco.MjModel.from_xml_string(S.gen.build(CFG, pole))
    assert m.nu == 3


def test_pole_must_be_cw_or_ccw():
    with pytest.raises(ValueError):
        S.gen.build(CFG, "ccw")


@pytest.mark.parametrize("jam", [60, 70, 76.9, 85, 100])
def test_jam_round_trip(jam):
    assert S.jam_of(S.with_jam(CFG, jam)) == pytest.approx(jam, abs=1e-9)


def test_jam_of_neck_yaml():
    assert S.jam_of(CFG) == pytest.approx(76.9, abs=0.05)


@pytest.mark.parametrize("pole", ["CW", "CCW"])
def test_rest_is_level_centred_and_matches_the_maths(pole):
    cfg = S.with_twist(CFG, pole)
    sim = S.Sim(cfg)
    (roll, pitch, _), p = sim.head()
    assert abs(roll) < 0.01 and abs(pitch) < 0.01
    assert math.hypot(p[0], p[1]) * 1000 < 0.01
    assert S.direction(sim.orbit()) == pole
    assert abs(sim.orbit()).max() == pytest.approx(S.jam_of(cfg), abs=1.0)   # sim 77.6 vs maths 76.9 today


def head_site_yaw(sim):
    x = sim.d.site("head").xmat.reshape(3, 3)[:, 0]
    return math.degrees(math.atan2(x[1], x[0]))


@pytest.mark.parametrize("pole", ["CW", "CCW"])
@pytest.mark.parametrize("rotation", [0, 90, -150])
def test_head_faces_rotation_deg_from_rod_end_1(pole, rotation):
    cfg = S.with_twist(CFG, pole); cfg["head"]["rotation_deg"] = rotation
    sim = S.Sim(cfg, render=True)   # render keeps head.rotation_deg; sweeps don't need it
    want = (sim.head_zero() + rotation + 180) % 360 - 180
    assert (head_site_yaw(sim) - want + 180) % 360 - 180 == pytest.approx(0, abs=0.1)


def test_to_frame_turns_roll_into_pitch():
    a = np.zeros((1, len(common.COLS))); a[0, common.COLS["roll"]] = 5
    b = common.to_frame(a, 90)   # the head now faces the old left axis, which roll tipped up
    assert b[0, common.COLS["roll"]] == pytest.approx(0, abs=1e-9)
    assert b[0, common.COLS["pitch"]] == pytest.approx(-5, abs=1e-9)   # ZYX pitch: nose up is negative
    assert b[0, common.COLS["yaw"]] == pytest.approx(0, abs=1e-9)


def test_to_frame_round_trip():
    rng = np.random.default_rng(1)
    a = np.zeros((50, len(common.COLS)))
    for k in ("roll", "pitch", "yaw"):
        a[:, common.COLS[k]] = rng.uniform(-20, 20, 50)
    assert common.to_frame(common.to_frame(a, 37), -37) == pytest.approx(a, abs=1e-9)


def test_sleds_together_is_pure_yaw():
    sim = S.Sim(CFG)
    (_, _, y0), _ = sim.head()
    sim.go([30, 30, 30])
    (roll, pitch, yaw), p = sim.head()
    assert abs(roll) < 0.01 and abs(pitch) < 0.01
    assert math.hypot(p[0], p[1]) * 1000 < 0.01
    assert yaw - y0 == pytest.approx(30, abs=0.01)


@pytest.mark.parametrize("sleds", GOLDEN)
def test_golden_points(sleds):
    sim = S.Sim(CFG)
    sim.go(list(sleds))
    (roll, pitch, yaw), p = sim.head()
    want = GOLDEN[sleds]
    assert (roll, pitch, yaw) == pytest.approx(want[:3], abs=0.05)
    assert math.hypot(p[0], p[1]) * 1000 == pytest.approx(want[3], abs=0.05)


def test_feasible_edge():
    # sled_1 and sled_2 moving apart by d each close their gap by 2d; MIN_GAP is reached at d
    d = (120 - S.MIN_GAP) / 2
    assert S.feasible(d - 0.01, -d + 0.01)
    assert not S.feasible(d + 0.01, -d - 0.01)
