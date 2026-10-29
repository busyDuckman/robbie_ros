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


def test_head_lines():
    assert N.head_lines(CFG, "nod line (up)") == S.nod_lines(CFG, "CW")
    down = N.head_lines(CFG, "nod line (down)")
    assert sorted(down) == sorted((n + 60 + 180) % 360 - 180 for n in S.nod_lines(CFG, "CCW"))   # CCW's down ends
    assert sorted(N.head_lines(CFG, "best compromise")) == [-90, 30, 150]
    # the compromise is halfway between CW's up end and CCW's down end
    up0, down0 = N.nearest(N.head_lines(CFG, "nod line (up)"), 30), N.nearest(down, 30)
    assert N.nearest(N.head_lines(CFG, "best compromise"), 30) == pytest.approx((up0 + down0) / 2)
    assert N.METHODS == ("nod line (up)", "nod line (down)", "best compromise")


def test_nearest_wraps_round():
    lines = [38.4, 158.4, -81.6]
    assert N.nearest(lines, 20) == 38.4
    assert N.nearest(lines, 200) == 158.4
    assert N.nearest(lines, -100) == -81.6
    assert N.nearest(lines, -170) == 158.4   # 31.6 deg away across the wrap, not 151.6


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
