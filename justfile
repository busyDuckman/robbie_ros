_banner_line := "------------------------------------------------------------"

# Native Windows is not supported for development (ADR 2).
_platform_guard := if os() == "windows" { error("\n" + RED + _banner_line + "\nrobbie_ros is developed on Linux or WSL 2.\nRun just from a WSL or Linux shell.\n" + _banner_line + NORMAL + "\n") } else { "" }

# List the available recipes
default:
    @just --justfile {{ quote(justfile()) }} --list


## --------------------------------------------------------------------------------------------------------------------
# Dev helpers

# Gets the repo 100% ready for all development work (ROS 2, MuJoCo, Rerun, virtual environments etc.)
setup *flags:

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


# install robbie_ros to a pi, or update the installation on a pi
# The pi will be running th image created by setup-flash
deploy:

# start robbie_ros on the pi
run:

# stop robbie_ros on the pi
stop:


## --------------------------------------------------------------------------------------------------------------------
# local dev

# (re-)start the MuJoCo sim, the Rerun viewer and robbie_ros against the sim
up:

# stop everything started by just up
down:


## --------------------------------------------------------------------------------------------------------------------
# utils

# Print a message inside a coloured banner. color: green, yellow, red or blue.
[private]
banner msg color="green":
    @printf '%s\n%s\n%s%s\n' "{{ if color == "red" { RED } else if color == "yellow" { YELLOW } else if color == "blue" { BLUE } else { GREEN } }}{{ _banner_line }}" {{ quote(msg) }} "{{ _banner_line }}" "{{ NORMAL }}"
