"""Sanity tests for the neck model and the sim helpers the plots trust. A few seconds; run by just test and CI.

The golden points are neck.yaml as committed. Changing neck.yaml or gen_neck_mjcf.py on purpose means updating
GOLDEN on purpose: print the new values with `uv run ... python -c` and check they make sense first.
"""
import math

import mujoco
import pytest

import sim as S

CFG = S.load_neck()

# sled offsets (deg) -> roll, pitch, yaw (deg), off (mm). Straight from rest, neck.yaml, CCW twist.
GOLDEN = {
    (20, 0, -20): (-2.721, -2.083, -8.490, 11.26),
    (-20, 10, 10): (-2.320, 1.098, -8.389, 10.34),
    (10, 25, -35): (7.658, 2.644, -15.285, 14.66),
    (-30, -10, 40): (-3.040, 13.741, -14.640, 11.66),
    (0, -25, 25): (-0.862, 3.991, -10.801, 13.64),
}


def test_model_builds_and_loads():
    m = mujoco.MjModel.from_xml_string(S.gen.build(CFG))
    assert m.nu == 3


@pytest.mark.parametrize("jam", [60, 70, 76.9, 85, 100])
def test_jam_round_trip(jam):
    assert S.jam_of(S.with_jam(CFG, jam)) == pytest.approx(jam, abs=1e-9)


def test_jam_of_neck_yaml():
    assert S.jam_of(CFG) == pytest.approx(76.9, abs=0.05)


@pytest.mark.parametrize("sign, direction", [(1, "CCW"), (-1, "CW")])
def test_rest_is_level_centred_and_matches_the_maths(sign, direction):
    cfg = S.with_twist(CFG, sign)
    sim = S.Sim(cfg)
    (roll, pitch, _), p = sim.head()
    assert abs(roll) < 0.01 and abs(pitch) < 0.01
    assert math.hypot(p[0], p[1]) * 1000 < 0.01
    assert S.direction(sim.orbit()) == direction
    assert abs(sim.orbit()).max() == pytest.approx(S.jam_of(cfg), abs=1.0)   # sim 77.6 vs maths 76.9 today


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
