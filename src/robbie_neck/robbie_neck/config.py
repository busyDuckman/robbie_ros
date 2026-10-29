"""Neck settings that robbie_ros nodes share.

- head.rotation_deg, from robbie_description's neck.yaml: which way the head faces on the platform.
  0 is centre to rod end 1, CCW seen from above (docs/head_rotation.svg). Fixed by the build.
- The pole, CW or CCW twist seen from above. Not in neck.yaml: the owner can turn the neck into
  either one by hand, so it's set at runtime. Nothing on the robot can sense it. By convention the
  head is set up so that CW looks up and CCW looks down (just config), and CW is the default.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

POLES = ("CW", "CCW")
DEFAULT_POLE = "CW"
POLE_FILE = os.path.expanduser("~/.config/robbie/twist")   # on the Pi, written by: just twist cw|ccw


def neck_yaml():
    path = os.path.join(get_package_share_directory("robbie_description"), "config", "neck.yaml")
    with open(path) as f:
        return yaml.safe_load(f)


def head_rotation_deg():
    return float(neck_yaml()["head"]["rotation_deg"])


def pole(value=""):
    """CW or CCW. value (e.g. a launch argument) wins, then POLE_FILE, then DEFAULT_POLE."""
    if not value and os.path.exists(POLE_FILE):
        with open(POLE_FILE) as f:
            value = f.read()
    value = (value or DEFAULT_POLE).strip().upper()
    if value not in POLES:
        raise ValueError(f"twist must be CW or CCW, not {value!r}")
    return value
