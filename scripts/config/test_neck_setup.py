"""Tests for the setup GUI's maths and neck.yaml writing (no window). Run by just test and CI."""
import os, re, sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "plots"))
import neck_setup as N
import sim as S

CFG = S.load_neck()


@pytest.mark.parametrize("spec", [15, 20, 22.95, 30])
def test_jam_for_rod_end_round_trip(spec):
    jam = S.jam_for_rod_end(CFG, spec)
    assert max(S.rod_end_angles(S.with_jam(CFG, jam))) == pytest.approx(spec, abs=1e-6)


def test_jam_for_rod_end_out_of_reach():
    assert S.jam_for_rod_end(CFG, 89) is None


def test_nod_lines_are_half_the_twist_back():
    jam = S.jam_of(CFG)
    assert S.nod_lines(CFG, "CCW")[0] == pytest.approx(-jam / 2)
    assert S.nod_lines(CFG, "CW")[0] == pytest.approx(jam / 2)
    assert S.nod_lines(CFG, "CCW")[1] - S.nod_lines(CFG, "CCW")[0] == pytest.approx(120)


def test_spring_rest_len_matches_the_model():
    x = S.gen.build(S.with_twist(CFG, "CCW"), "CCW")
    mm = float(re.search(r"springs ([\d.]+) mm between mounts", x).group(1))
    assert S.spring_rest_len(CFG) * 1000 == pytest.approx(mm, abs=0.05)


def test_yaml_set_keeps_comments_and_other_lines():
    with open(S.NECK) as f:
        text = f.read()
    out = N.yaml_set(text, ("platform", "rest_height"), N.yaml_text(0.0712, "mm"))
    out = N.yaml_set(out, ("collar_z",), N.yaml_text(0.041, "mm"))
    out = N.yaml_set(out, ("head", "rotation_deg"), N.yaml_text(-38.5, "deg"))
    changed = [(a, b) for a, b in zip(text.split("\n"), out.split("\n")) if a != b]
    assert len(changed) == 3
    for a, b in changed:
        assert a[a.index("#"):] == b[b.index("#"):]   # comment kept, same column
    import yaml
    c = yaml.safe_load(out)
    assert (c["platform"]["rest_height"], c["collar_z"], c["head"]["rotation_deg"]) == (0.0712, 0.041, -38.5)
    assert c["platform"]["radius"] == CFG["platform"]["radius"]   # same key name in another section untouched


def test_yaml_text():
    assert N.yaml_text(0.07, "mm") == "0.070"
    assert N.yaml_text(0.0015, "mm") == "0.0015"
    assert N.yaml_text(20.0, "deg") == "20"


def test_best_rotation_picks_the_balanced_peak_nearest():
    h = N.np.arange(361)
    up = 10 + 2 * N.np.cos(N.np.radians(3 * (h - 30)))     # peaks at 30, 150, 270
    down = 10 + 2 * N.np.cos(N.np.radians(3 * (h - 90)))   # peaks at 90, 210, 330
    weak = N.np.zeros(361)
    sims = {("CW", 0): ({("graceful", "up"): up, ("graceful", "down"): weak}, {}),
            ("CCW", 0): ({("graceful", "up"): weak, ("graceful", "down"): down}, {})}
    # min(up, down) peaks where the two cross, every 60 deg from 0
    assert N.best_rotation(sims, 50) == 60
    assert N.best_rotation(sims, -170) == 180


def test_yaml_set_fills_and_clears_an_optional_value():
    with open(S.NECK) as f:
        text = f.read()
    filled = N.yaml_set(text, ("rod_end", "max_angle_deg"), N.yaml_text(13.5, "deg"))
    import yaml
    assert yaml.safe_load(filled)["rod_end"]["max_angle_deg"] == 13.5
    assert N.yaml_set(filled, ("rod_end", "max_angle_deg"), N.yaml_text(None, "deg")) == text


def test_optional_measurements_dont_change_the_sim_cache_key():
    c = N.copy.deepcopy(CFG); c["rod_end"]["max_angle_deg"] = 13.5; c["platform"]["measured_height"] = 0.134
    assert S.key("sweep_a", S.with_twist(c, "CW")) == S.key("sweep_a", S.with_twist(CFG, "CW"))
