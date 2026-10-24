# Architecture Decision Records

## ADR 1: Simulator is MuJoCo with Rerun

Status: Accepted

MuJoCo does the physics and Rerun does the visualisation and logging. MuJoCo handles the neck's closed loop (rods with ball joints, springs as tendons). 

The sim plugs in at the hardware driver: motor commands go in, encoder angles and the home switch come out. Everything above that is the same code in the sim and on the Pi.

## ADR 2: Development runs in a Docker dev container

Status: Accepted

The dev container (`docker/dev/`) is Ubuntu 24.04 with ROS 2 Jazzy, ros2_control and the MuJoCo sim. MuJoCo runs inside the ROS control process, so it lives in the container too. The host only needs Docker and just, on Linux or WSL 2, with any distro. Nothing ROS is installed on the host. Native Windows is not supported.

Build output (`build/`, `install/`, `log/`) lives in Docker volumes. The repo is still faster in the WSL filesystem (for example under `/home`) than under `/mnt/c`.

Replaces the first version of this ADR, which installed ROS on the host and required Ubuntu 24.04 there.

## ADR 3: robbie_interfaces is built from a sibling checkout of robbie

Status: Accepted

The robbie repo is checked out next to robbie_ros, and colcon builds `robbie_interfaces` in place with `--base-paths`. Nothing is copied in, and vcstool and submodules aren't used. `just setup` checks for the checkout and offers to clone it. `just vet` fails if it's missing. look_around is installed with pip.

## ADR 4: Motor control uses ros2_control

Status: Accepted

The three sleds are joints in a ros2_control hardware plugin (C++). The sim plugs in the same way, with MuJoCo driven from C++ (probably via `mujoco_ros2_control`). Homing runs as a custom controller, or as a node that runs before the controllers start. Chosen partly because it's the standard way and partly out of curiosity.

## ADR 5: Python at the high level, C++ where it makes sense

Status: Accepted

High-level nodes can be Python to keep them simple. ros2_control plugins, the sim and anything performance-critical are C++.

## ADR 6: The Pi runs Ubuntu 24.04 Server and ROS 2 Jazzy, installed natively

Status: Accepted

robbie_ros is installed natively on the Pi. `just setup-flash` builds the SD card image, and `just deploy` installs or updates robbie_ros on the Pi. Docker is used on the Pi only where it helps, such as for the eyes and the parts of the harness that run on the Pi.

## ADR 7: robbie_ros doesn't provide a fake driver for the robbie repo

Status: Accepted

The robbie repo keeps its own fake Layer C stand-ins.
