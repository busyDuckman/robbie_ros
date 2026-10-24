#!/usr/bin/env bash
# Runs a command in the dev container with ROS 2 and the workspace (if built) sourced.
# Used by just; bash -c doesn't read .bashrc. Usage: ros.sh <command> [args...]
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ws/robbie_ros/install/setup.bash ]; then
    source /ws/robbie_ros/install/setup.bash
fi
exec "$@"
