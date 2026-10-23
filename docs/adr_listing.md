# Architecture Decision Records

## ADR1: Simulator — MuJoCo with Rerun

**Status:** Accepted (2026-10-23)

### Context

robbie_ros must run in simulation so the system is proven before it is deployed to the Pi.
The sim is for:
- developing without crashing the real motors;
- working out the pose-to-sled logic;
- building modern, job-ready robotics skills;
- the usual sim uses: testing, tuning and debugging.

No simulated camera image is needed; camera and audio belong to the harness (Layer B).

The neck is a closed kinematic chain. Three sleds on a shared ring gear drive three rods to
the platform, with no central pivot. Springs between the rods twist the linkage to one side
and set the platform's pose. Pose is therefore a spring-equilibrium problem, not rigid
kinematics. Development is on Windows.

### Options

- **MuJoCo + Rerun.** Closed loops through equality constraints; springs as tendons with
  stiffness. Native Windows install (`pip install mujoco`). Rerun logs and views both sim and
  real-robot data.
- **Gazebo.** URDF can't describe closed loops. SDF with DART can, but it is brittle. Its
  strengths (sensor and camera simulation, worlds) aren't needed here. Heavy on Windows
  (needs WSL2 or Docker).
- **Others.** Drake is excellent at closed chains but heavier than needed. Isaac Sim is
  overkill and needs an NVIDIA GPU. PyBullet is effectively in maintenance mode. Webots is
  weak on closed loops.

### Decision

Use MuJoCo for physics and Rerun for visualisation and logging.

- Model each sled as a rotary joint about the collar axis. Apply the pinion-to-ring ratio in
  the motor model rather than simulating gear teeth.
- Model each rod as a body with a ball joint at each end: one on its sled, one closing the
  loop on the platform through an equality constraint.
- Model the three rod-to-rod springs, between neighbouring rod midpoints, as spatial tendons
  with stiffness.
- The sim plugs in at the hardware driver: motor command in; raw encoder angle and home
  switch level out. Everything above that is the same code in sim and on the Pi: homing,
  sled collision guard, kinematics, look_around, unwinding and the safety watchdog.
- Log the same Rerun streams from the sim and the real robot, so they can be compared in one
  viewer.

### Consequences

- Spring stiffness barely matters. At a pole the system sits stable with the springs doing
  no work; they only push it back if it leaves the pole. So the pose comes from geometry,
  and the sim's springs only need to be stiff enough to return the system to its pole. No
  tuning against the real springs is needed. (This is deliberate: it keeps the solve simple
  and lets builders use whatever springs they can buy.)
- RViz2 and TF can still be added later for standard ROS tooling.
- A no-physics fake of the same driver interface is cheap and could serve the robbie repo's
  planned fake stand-ins.
