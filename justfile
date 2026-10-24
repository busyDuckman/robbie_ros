_banner_line := "------------------------------------------------------------"

# Native Windows is not supported for development (ADR 2).
_platform_guard := if os() == "windows" { error("\n" + RED + _banner_line + "\nrobbie_ros is developed on Linux or WSL 2.\nRun just from a WSL or Linux shell.\n" + _banner_line + NORMAL + "\n") } else { "" }

# Dev container (ADR 2). On WSL, the WSLg overlay is added so GUI apps show up.
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
build:

# Basic unit test run
test:

# Full testing, including sim runs (homing, pose-to-sled, unwinding)
vet: test

# remove temporary files (build/, install/, log/, recordings)
clean:


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

# (re-)start the MuJoCo sim, the Rerun viewer and robbie_ros against the sim
#  --headless to not show the simulator window
up:

# stop everything started by just up
down:

## --------------------------------------------------------------------------------------------------------------------
# utils

# Print a message inside a coloured banner. color: green, yellow, red or blue.
[private]
banner msg color="green":
    @printf '%s\n%s\n%s%s\n' "{{ if color == "red" { RED } else if color == "yellow" { YELLOW } else if color == "blue" { BLUE } else { GREEN } }}{{ _banner_line }}" {{ quote(msg) }} "{{ _banner_line }}" "{{ NORMAL }}"
