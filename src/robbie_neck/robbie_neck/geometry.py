"""The neck's rest-pose geometry, from neck.yaml (cfg) and the pole. Matches gen_neck_mjcf.py.

Angles are in degrees. Orbit angles are as in robbie_openscad (0 is -Y, CCW seen from above);
head rotations are from rod end 1, CCW seen from above (docs/head_rotation.svg).
"""

import math


def orbit_dir(deg):
    """Unit vector (x, y, 0) from the collar centre to orbit angle deg."""
    a = math.radians(deg)
    return (math.sin(a), -math.cos(a), 0.0)


def pivot_z(cfg):
    """Height (m) of the sled pivots above the table."""
    return cfg["collar_z"] + cfg["sled"]["pivot_height"]


def twist_deg(cfg, pole):
    """The twist at rest: how far round the top rod ends sit from their sleds. + is CCW."""
    rb, rp = cfg["sled"]["pivot_radius"], cfg["platform"]["pivot_radius"]
    chord_sq = cfg["rod"]["length"] ** 2 - cfg["platform"]["rest_height"] ** 2
    twist = math.degrees(math.acos((rb * rb + rp * rp - chord_sq) / (2 * rb * rp)))
    return twist if pole == "CCW" else -twist


def platform_pivots(cfg, pole):
    """The top rod-end centres (m), in the platform frame: level, centred, world-aligned at rest."""
    rp, twist = cfg["platform"]["pivot_radius"], twist_deg(cfg, pole)
    return [tuple(rp * c for c in orbit_dir(deg + twist)) for deg in cfg["sled"]["start_deg"]]


def head_facing_deg(cfg, pole):
    """The head's forward direction in the platform frame, as a math angle (from +X, CCW)."""
    return cfg["sled"]["start_deg"][0] + twist_deg(cfg, pole) - 90 + cfg["head"]["rotation_deg"]


def nod_lines(cfg, pole):
    """head.rotation_deg of the three nod lines: halfway round each rod's twist at rest."""
    twist = twist_deg(cfg, pole)
    return [(-twist / 2 + 120 * k + 180) % 360 - 180 for k in range(3)]
