# robbie_ros: frame of reference


## Where robbie_ros sits

A small robot called Robbie, is built in three layers:

- A, the LLM. Decides when to speak or take an action. Lives in the robbie repo.
- B, the harness. State, behaviour, people present, attention target, mood, relationships,
  beeps and TTS. It owns the camera, microphone and speakers. Lives in the robbie repo.
- C, robot control. Drives the motors, reads the sensors (encoders, home switch), runs
  look_around, and draws and displays the eyes. This is robbie_ros.


Relevant works (read only for the scope of this project)
  - Umbrella project: robbie repo
  - 3D printable parts and assembly: robbie_openscad repo
  - Eye graphics rendering, other bits and pieces: robbie_restoration repo
  - Look around behaviour: lookaround repo
  - robbie_ros (this, project, known as Layer C by the umbrella proj): robbie_ros repo


#  What is robbie_ros (probably)

This is hand-wavey, and open to adjustment changes etc.

  - robbie_ros is a ROS 2 system in its own repo 
  - IRL; Robbie ros runs on a raspberry Pi 4 
  - We also need robbie ros running in a simulated environment, so the system is proven before deploying.
  - Layer B talks to robbie_ros through a single ROS 2 bridge node;.
  - The aim is that any robot can be connected in place of robbie_ros and become Robbie, (possibly just use the node).
  - aside from the bridge node, ASSUME the other layers don't use or import ROS
  - Standard ROS 2 messagetypes are used where possible, so another ROS 2 robot could be connected by remapping topics.
  - Custom messages live in a `robbie_interfaces` package in the robbie repo.

No camera or audio data crosses the interface between the harness and robbie_ros.


## The hardware robbie_ros controls

The current robot is a stationary device with a moving head (earlier versions differed, and are discontinued). 

The mechanics are in robbie_openscad.

Neck:

  - A 3-DOF Stewart platform gives the head roll, pitch and yaw, with yaw being a full 360.
  - the platform is connected via 3 rods to 3 motorised planetary sleds on a ring gear.
  - three springs midway along the rods tie each rod to the one next to it and torsion the system.
  - There is only one solution to the platforms location, the one most twisted to the side the springs pull it.
  - The three motorised sleds circle a collar. Each sled is a planet with a small pinion on its
    own GA12-N20 motor, running on the collars fixed internal ring gear. 
  - There is a as5600 rotary sensor tracking each pinions rotation.
  - There is a home switch thats fixed relative to the collar. The sleds bump it as they pass.
  - Current ratio: pinion 9 tooth, internal ring gear 56 tooth (module 1.5). 
    - that openscad file is configurable so gear ratios might change. 
  - The sleds share one track and cannot pass each other. Their order is fixed and the
    controller must keep them from colliding.
  - If the sleds are spaced 120 degrees, the system is level.
  - If the sleds rotate in unison, the platform rotates (yaw) without adjustment its roll or pitch.
  - The N20s are high reduction and the platform can't back-drive them, so the controller can
     move to a pose and then cut power.
  - The platform can do several continuous rotations without issue, the power cable just gets a bit twisted; and an allowance 
    is made for this in the cable design.
  - The platform should attempt to unwind itself if it has made several rotations. It can override harness commands, 
    or fulfill look at actions in an alternate rotation to protect itself.
  - Every sled has a fob that presses the same switch, so the switch doesn't say which sled.
  - Important note: The sled fob is thinner than the sled, and centred on the sled.
  - The fob presses from either direction, closed over some degrees of orbit. So it's a significant press window.



## Thoughts on homing.

When turned on the system does not know where the sleds are, the orientation of the head etc.

Important note: the part of the sled that contacts the home switch (the fob) is thinner than the sled.
The sleds angular size comes from a config file. The fob is centered on the sled.

Stage A)
  - It's not safe to move a sled by it self, you don't know their relative positions.
  - But it is safe to drive them all at the same speed, monitoring the rotation sensor to do so.
  - The home switch will be pressed, held down, then released as a portion of the sled goes over it.
  - Do a full rotation in this manner and the home switch will cycle 3 times.
  - You can now determine:
    - the angle between the three sleds
    - the position of the sleds relative to the home switch
    - how far each sled can move in each direction.
    - how many pinion turns are needed for one orbit.
    - BUT; you don't know which motor drives which sled.

Stage B):
  - if not already so, rotate (unwinding stage A) until a sled suitable clearance on both sides is on the home SW
  - Power only only motor at a time, checking  the rotary sensor moved freely (ie not jamming). Discover which sled is on the home.
    NOTE: some minor movements of the other sleds may happen as the rod spring apply tension on the other sleds. Sm don't start 
    with a sleds edge just on the side of the home switch.
  - When all motors are known:
    - space all sleds at 120deg to level the platform
    - unwind Stage A's rotation, checking the home switch fires when expected.
   

## Encoders:

- One AS5600 magnetic rotary sensor per sled, over a magnet on the pinion. 12 bit.
- Absolute only within one pinion turn, so orbit position needs to be calculated from that.
- The I2C address is fixed (0x36), so three sleds need a mux or separate buses.
- 4 wires per sled.

# Home switch:
- The same micro switch commonly used in mice (I forget the model).
- NC/NO io-pin etc not decided yet.

# Eyes (from robbie_restoration, being salvaged):

- Two 1.28" GC9A01 round SPI LCDs, left on SPI-0 and right on SPI-1, with PWM backlights.
  Pin wiring is documented in `code_robbie/eye_manager/eye_service.py`.
- A C++ display driver (`code_robbie/cpp/eye_manager/`) using canvas_ity, with a PC build for
  testing on a desktop.
- Python eye animation, including eyelid blinking based on tuned curves (`eye_lids.py`,
  `eye_renderer_cv2.py`).
- Dirty-region SPI updates: only push changed rectangles, and only draw alternate lines.
  The idea is good; that code is rough, and a later version exists.
- Pi 4 docker setup (udev rules and Dockerfiles) giving a container GPIO and SPI access.

## Software robbie_ros runs

look_around (lookaround repo) generates natural looking gaze motion. A settings file
describes the looker's parts (eyes, head, torso, base) and their limits. Each tick it takes a
target point and, optionally, the measured joint pose, and returns setpoints per joint. It
does not do motor control; its setpoints feed robbie_ros's own controllers. If a joint lags
or is held by something else, the parts further out make up the difference. It also reports
whether the target is reachable. The spec and a Python reference implementation exist; C++
and Rust ports are planned.

Kinematics and control: the robbie README lists an inverse kinematics and controller rewrite
as planned. The old system used the camera to establish the pose; that was abandoned, and the
new system uses the magnetic rotary sensor on each motor.

## The interface with the harness (draft, from robbie docs/interfaces/layer_b_c_interface.md)

The harness sends robbie_ros:

- A gaze target: a point to look at, or "neutral". robbie_ros turns it into motion with
  look_around.
- Expressive gaze parameters (mainly eyebrows, possibly pupil dilation). Set not yet agreed.
- The expression state the eyes should show (mood, blinks, lid position).
- A request to home the robot.
- A heartbeat.

robbie_ros sends the harness:

- The timestamped pose of the camera, in the same frame as gaze targets, so the harness can
  turn a detected face into a gaze target without knowing the robot's joints.
- Whether the current gaze target is reachable.
- Status, such as faults or the e-stop being engaged.
- A heartbeat.

The harness relies on robbie_ros to enforce its own joint limits and safety, and to stop
safely if the harness goes silent. How robbie_ros does this is up to robbie_ros.

NOTE:   RELM (Robbie's Expressive Language Model output) hs a ROS2 mode (amongst other modes).
Thats NOT related to robbie_ros. RELM is slated to be forked to a separate independent project 
(like lookaround). It's ROS feature is related to its own project feature set. 
**AI agents often get confused and think the RELM listener talks to layer C via ROS, ignore them**


The robbie repo plans fake stand ins for robbie_ros so the LLM and harness run without
hardware. Thats not may or man not be related to the robot simulator this project plans 
(not decided yet).



# NOTES AND DISCUSSION

## Review round 1 (2026-10-23)

### Neck mechanism
- No central pivot. The three rods are the only linkage between the sleds and the platform.
- 3 rods and 3 springs. Each spring joins two neighbouring rods at about their midpoints,
  making a ring of springs. Each rod has a ball-joint rod end at both ends: one on its sled,
  one on the platform. The rods lean and cross past each other.
- Photos: `docs/reference_photos/platform_assembly.jpg` (the bare neck) and
  `docs/reference_photos/pi_position.jpg` (the Pi and eyes mounted on the platform).
- It looks visually similar to a 3-strut tensegrity prism, but it is NOT one. Don't reason
  about it as a tensegrity.
- This is a reimplementation of a mechanism that was built years ago and works. Its behaviour
  is known from the physical build, not from theory:
  - The springs don't hold the platform up. They twist the otherwise floppy linkage towards
    one side. At rest the springs are at least compression, so gravity can't move the platform
    without fighting them.
  - The platform has no up/down travel.
  - It is very stable. If knocked, it is springy, resists damage and settles back.
  - There are two twist poles, about 120 deg apart. Sled motion can't flip the system between
    them.
  - It makes a good "how did you think of that" talking point.
- The spring design is deliberately robust. With suitable springs the system sits stable at
  a pole with no pull on the springs at all. Their only job is to pull it back to the pole if
  it leaves. So the springs' properties have no effect on the dynamics, and the pose comes
  from geometry.
- This is deliberate: it keeps the solve easy, and anyone following along can use whatever
  springs they can get off eBay.
- Needs a proper write-up (the user will draft it in the README).

### Wiring
- The Pi (controller) is on top of the platform, so the sled, encoder and eye cables don't
  twist relative to it.
- The twist is on the power cable only. It will use a coiled cable or a rotating cable collar.
- An e-stop, if present, will be on the head, attached to the controller board.

### Encoders
- Plan A: one AS5600 on each of the Pi's spare I2C buses.
- AS5600L (settable address) and I2C muxes have been considered and rejected for now. The
  plain AS5600 is on cheap, widely available breakout boards, which keeps the build easy for
  people following along.
- A dedicated microcontroller for motor moves may come later, which would change this again.
  Not decided.

### Homing
- Assembly order of the sleds around the ring varies; the robot gets pulled apart often.
  Homing must not depend on wiring or assembly order.
- Homing must also handle:
  - stall or jam detection from the encoders during the unison move;
  - a sled already on the switch at power-on.
- The gear ratio isn't a whole number (56/9 ≈ 6.22 pinion turns per orbit), so the encoder
  angle says nothing about orbit position after power-off. Home on every boot and count turns
  continuously. Turns per orbit can be computed from the geometry; the value measured during
  homing is a check for skipped gear teeth.

### Interfaces and timing
- `robbie_interfaces` stays in the robbie repo for now. Moving it out is on the TODO list and
  will get its own ADR later.
- Parts of the harness may run on both the PC and the Pi. The Pi's clock can sync to the PC's
  (needed for the timestamped camera pose).

### Yaw unwinding vs look_around
- look_around doesn't know about unwinding, and doesn't need to. It is built on the
  assumption that its suggested yaw may be ignored for operational reasons, and handles that.

### Simulator
- Goals:
  - develop without crashing the real motors;
  - work out the "I want this pose, so do this with the sleds" logic;
  - learn and demonstrate modern, job-ready robotics skills;
  - everything else a sim is normally used for.
- No simulated camera image is needed.
- Decision: see ADR1 in `docs/adr_listing.md`.

### Still open (next planning stage)
- Dev setup for ROS 2 (WSL2, Docker, Linux box).
- ros2_control vs a lighter custom driver node.