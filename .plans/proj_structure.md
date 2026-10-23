# Proposed project structure

A standard colcon workspace: the repo root is the workspace, and the packages go in `src/`.

```
robbie_ros/
├── justfile
├── .third_party/         # gitignored downloads from just setup, e.g. mujoco-3.17.0/ (dev machine only)
├── scripts/              # setup, flash-image and deploy scripts called by just
├── docker/               # Pi-side containers (eyes)
├── docs/                 # ADRs, photos, neck write-up
└── src/
    ├── robbie_description/  # URDF with the <ros2_control> block, MuJoCo MJCF, meshes from robbie_openscad
    ├── robbie_hardware/     # C++ ros2_control plugin: motors, AS5600s, home switch (ADR 4)
    ├── robbie_controllers/  # homing, sled collision guard
    ├── robbie_neck/         # pose <-> sled kinematics, unwinding
    ├── robbie_gaze/         # wraps look_around
    ├── robbie_eyes/         # eye node; the renderer/driver may run in docker/
    ├── robbie_safety/       # watchdog, e-stop, heartbeat
    └── robbie_bringup/      # launch files and config: pi.launch.py, sim.launch.py, controllers.yaml
```

Notes:
- The sim is not its own package. `mujoco_ros2_control` stands in for `robbie_hardware`, and
  `robbie_bringup` picks one or the other.
- The URDF only needs the three sled joints for ros2_control. URDF can't describe the neck's
  closed loop; the full neck lives in the MJCF, and the platform pose comes from `robbie_neck`.
- Config (gear ratio, sled size, joint limits) lives in the package that uses it, as ROS 2
  expects, not in a top-level `config/`.
- `robbie_interfaces` stays in the robbie repo, checked out next to robbie_ros. colcon builds it
  in place using `--base-paths` (ADR 3). No vcstool.
- look_around comes in through pip.
- `just setup` downloads pinned, checksummed MuJoCo into `.third_party/`, and `just build` points
  CMake at it. `just clean` leaves it alone.
- Tests live inside each package, as ROS 2 expects.
- The bridge node to the harness lives in the robbie repo (Layer B), not here.
