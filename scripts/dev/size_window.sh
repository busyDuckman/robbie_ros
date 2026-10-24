#!/usr/bin/env bash
# Resizes the MuJoCo viewer window once it appears. mujoco_ros2_control always opens it at the
# primary monitor's size, with no setting for it. Usage: size_window.sh <width> <height>
set -u
for _ in $(seq 1 20); do
    w="$(xdotool search --onlyvisible --name '^MuJoCo' 2>/dev/null | head -n1)"
    if [ -n "$w" ]; then
        xdotool windowsize "$w" "$1" "$2"
        exit 0
    fi
    sleep 0.5
done
echo "MuJoCo window not found, left at its default size." >&2
