_banner_line := "------------------------------------------------------------"

# Native Windows is not supported for development (ADR 2).
_platform_guard := if os() == "windows" { error("\n" + RED + _banner_line + "\nrobbie_ros is developed on Linux or WSL 2.\nRun just from a WSL or Linux shell.\n" + _banner_line + NORMAL + "\n") } else { "" }

# Dev container (ADR 2). On WSL, the WSLg overlay is added so GUI apps show up.
_sim_container := "robbie_ros_sim"
_sim_window := "1920 1080"   # MuJoCo window size (width height)
_compose_dev := "USER_UID=$(id -u) USER_GID=$(id -g) docker compose -f docker/dev/compose.yaml" + if path_exists("/mnt/wslg") == "true" { " -f docker/dev/compose.wslg.yaml" } else { "" }

# List the available recipes
default:
    @just --justfile {{ quote(justfile()) }} --list


## --------------------------------------------------------------------------------------------------------------------
# Dev helpers

# Gets the repo 100% ready for all development work: checks Docker and the robbie checkout, builds the dev container
setup:
    @bash scripts/setup/setup.sh
    {{ _compose_dev }} build
    @just --justfile {{ quote(justfile()) }} banner "Ready. Try: just shell" green

# open a shell in the dev container (ROS 2 Jazzy, ros2_control, MuJoCo)
shell:
    @{{ _compose_dev }} run --rm dev bash

# Build the ROS 2 workspace (colcon)
# robbie_interfaces gets added to --base-paths (ADR 3) once it exists in the robbie repo.
build:
    @{{ _compose_dev }} run --rm -T dev bash scripts/dev/ros.sh \
        colcon build --symlink-install --base-paths src --event-handlers console_direct-

# Basic unit test run (also what CI runs: .github/workflows/test.yml)
test:
    uv run --python 3.12 --with mujoco==3.12.0 --with numpy --with pyyaml --with matplotlib --with pytest pytest -q scripts/plots scripts/config

# Full testing, including sim runs (homing, pose-to-sled, unwinding)
vet: test

# remove build output: build/, install/ and log/ (their Docker volumes, and any copies on the host)
# Stops the sim first, since it holds the volumes. .third_party/ and .plots/ are left alone.
clean: down
    docker volume rm -f robbie_ros_build robbie_ros_install robbie_ros_log
    rm -rf build install log


## --------------------------------------------------------------------------------------------------------------------
# robbie management

# create a flash image with a properly configured Ubuntu 24.04 to .output/robbie_ros.img
# robbie will be available via robbie-ros.local
setup-flash:
    @{{ _compose_dev }} run --rm -e ROBBIE_SSH_PUBKEY_DEFAULT="$(bash scripts/flash/host_ssh_key.sh)" -e ROBBIE_WIFI_KNOWN="$(bash scripts/flash/host_wifi_list.sh)" dev bash scripts/flash/setup_flash.sh


# install robbie_ros to a pi, or update the installation on a pi
# The pi will be running th image created by setup-flash
deploy:

# ssh into the pi, as the user and hostname chosen in setup-flash (.env)
login *args:
    #!/usr/bin/env bash
    set -euo pipefail
    [ -f .env ] && . ./.env
    exec ssh "${ROBBIE_USER:-robbie}@${ROBBIE_HOSTNAME:-robbie-ros}.local" {{ args }}

# start robbie_ros on the pi
run:

# stop robbie_ros on the pi
stop:


## --------------------------------------------------------------------------------------------------------------------
# local dev

# (re-)start the MuJoCo sim, the neck poser window, the Rerun viewer and robbie_ros against the sim
#  --headless to not show the simulator window, --cw to start in the CW pole (default --ccw)
#  Rerun goes where ROBBIE_RERUN (.env, from just setup) says, starting the viewer if needed (scripts/dev/rerun_viewer.sh)
up *flags: down build
    #!/usr/bin/env bash
    set -euo pipefail
    headless=false poser=true twist=ccw
    [[ " {{ flags }} " == *" --headless "* ]] && headless=true poser=false
    [[ " {{ flags }} " == *" --cw "* ]] && twist=cw
    rerun="$(bash scripts/dev/rerun_viewer.sh $headless)"
    {{ _compose_dev }} run -d --name {{ _sim_container }} dev bash scripts/dev/ros.sh \
        ros2 launch robbie_bringup sim.launch.py headless:=$headless poser:=$poser twist:=$twist rerun:=$rerun >/dev/null
    for i in $(seq 1 30); do
        if docker logs {{ _sim_container }} 2>&1 | grep -q "Configured and activated head_pose_broadcaster"; then
            [ "$headless" = true ] || docker exec {{ _sim_container }} bash scripts/dev/size_window.sh {{ _sim_window }}
            just --justfile {{ quote(justfile()) }} banner "Sim is up. Logs: docker logs -f {{ _sim_container }}   Poke it: just shell" green
            exit 0
        fi
        sleep 1
    done
    just --justfile {{ quote(justfile()) }} banner "Sim started, but the controllers aren't up after 30 s. Check: docker logs {{ _sim_container }}. Rarely, DDS discovery stalls; just up again." yellow

# stop everything started by just up
down:
    @docker stop -s INT -t 10 {{ _sim_container }} >/dev/null 2>&1 || true   # as Ctrl-C: ros2 launch ignores SIGTERM as PID 1. Lets Rerun finish its .rrd
    @docker rm -f {{ _sim_container }} >/dev/null 2>&1 || true

# Set up neck.yaml for your Robbie, in a window: measurements, checks, which twist looks up, and the charts
config:
    uv run scripts/config/neck_setup.py

# Populate .plots/ with the neck kinematics plots (sweeps the sim when neck.yaml changes); see .plots/plots.md
plot:
    uv run scripts/plots/plot.py .plots

## --------------------------------------------------------------------------------------------------------------------
# utils

# Print a message inside a coloured banner. color: green, yellow, red or blue.
[private]
banner msg color="green":
    @printf '%s\n%s\n%s%s\n' "{{ if color == "red" { RED } else if color == "yellow" { YELLOW } else if color == "blue" { BLUE } else { GREEN } }}{{ _banner_line }}" {{ quote(msg) }} "{{ _banner_line }}" "{{ NORMAL }}"
