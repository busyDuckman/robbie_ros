# Open questions

## 1. ros2_control vs a custom driver node

**The seam** (ADR 1): motor commands go in; raw encoder angles and the home switch come out.

**ros2_control.** The three sleds become three joints in a hardware plugin.
- The motor command is a velocity command interface.
- The encoder angle is a state interface.
- The home switch is a GPIO state interface.
- The sim swaps in through the existing `mujoco_ros2_control` package.

For:
- It's the standard, job-relevant way.
- It gives a controller manager, joint state broadcasting and a fixed-rate control loop for free.
- The sim and the real robot swap via a single config change.

Against:
- The plugins are C++ and need pluginlib, which means a lot of boilerplate.
- Homing is a sequence that works out which motor drives which sled, so it doesn't fit as a normal controller. It would be a custom controller, or a node that runs before the controllers start.
- Stopping the sleds colliding needs logic across all three joints.

**Custom node.** One driver node (Python or C++) publishes and subscribes to plain topics, and the sim is a second node with the same topics.

For:
- It's simple and fast to write.
- Homing and the collision guard sit naturally in normal code.

Against:
- It teaches less of the job-relevant skill.
- The control loop, timing and controller switching would all be hand-rolled.

**Leaning:** ros2_control, because of the job-skills goal and because `mujoco_ros2_control` already exists. The cost is C++ plugins and fitting homing in as a custom controller.


ANSWER: ros2_control, just because I am very curious to see how this works. Kinda the point of a hobby proj sometimes.

## 2. MuJoCo from C++ or Python

This follows from question 1. With ros2_control, the sim side is C++ (probably via `mujoco_ros2_control`). With a custom node, Python is quicker. Either way, the neck model file is the same.

ANSWER: c++ it is.

## 3. Language for the rest of robbie_ros

- Python or C++ for homing, kinematics, unwinding and safety. It could be mixed.
- look_around is Python (via pip), with C++ and Rust ports planned.

Some python is fine, makes sense to keep it simple at a high level. C++ where it makes sense to do so.


## 4. ROS 2 version and the Pi's OS

- Jazzy (Ubuntu 24.04) is the obvious choice.
- On the Pi, the choice is Ubuntu 24.04 Server, which installs ROS 2 directly, or Raspberry Pi OS with ROS 2 in Docker. robbie_restoration already has a Pi Docker setup for GPIO and SPI.

Lets go Ubuntu 24.04 Server.
The previous ver used a lot of docker, because I love docker. Not against docker but not going to bend over backward for it ATM.

## 5. How robbie_ros is deployed to the Pi

- Native install or Docker.
- What `just deploy-robbie` actually does.

Native install

its now "just deploy" and "just setup-flash"

I think docker should be on the pi and used for:
  - eyes  (possibly, not decided)
  - parts of the harness that live on the pi (possibly a marryTTS server)

## 6. Eyes

- Reuse the C++ GC9A01 driver from robbie_restoration, or rewrite it.
- How the eye renderer runs as a ROS 2 node, and how it draws to a window on the PC.


Will see, but prob a docker container. For the PC, possibly an API endpoint or IDk cross that bridge later.

### History (what we know)

- **Python (2023):** the eyes were drawn with PIL and sent through Waveshare's `LCD_1inch28` library. Python also did the dirty regions and alternate lines. It was too slow and CPU-heavy, and a numba port was tried.
- **A popular open-source driver** did fast dirty-region updates, but it was GPL. Its scanline-based approach also made the eyes look odd, which is why the eyes moved away from it.
- **C++ in robbie_restoration (Jul 2023):** only the start of the port is there, in a single commit. It has:
  - the user's own GC9A01 driver, using pigpio for SPI and GPIO at 40 MHz;
  - an OpenCV renderer with a PC preview window.

  It has no dirty regions, canvas_ity is included but not used, and it only draws test patterns. It has no licence today, though the repo was GPL v3 until the 2026 archive. The user wrote it, so it can be relicensed as MIT.
- **Later C++ version (late 2024 or 2025, the "long flight"):** it had the user's own dirty-region ideas and newer improvements. It was left on a laptop that has since been wiped, and backed up to a server whose drives are failing. It may be lost; the user will look.
- **Mismatch:** the C++ code says SPI0 is the right eye, but the Python says SPI0 is the left eye.

### Plan

- If the later version turns up, start from it.
- If not, rewrite it in C++ or Rust:
  - Keep the Python drawing tricks: faked pupil rounding, the blink curves in `eye_lids.py`, and the overall look. These are what make the eyes Robbie's.
  - Use spidev and libgpiod rather than pigpio, which is poorly supported on Ubuntu 24.04.
  - Use the user's own dirty-region approach, not scanlines.

## 7. Where the bridge node lives

ANSWER: Already settled. The robbie repo's `docs/interfaces/layer_b_c_interface.md` says "B has a single ROS 2 bridge node", so it lives in the robbie repo. robbie_ros only provides the topics.


## 8. Clock sync between the Pi and the PC

This is needed for the timestamped camera pose. chrony with the PC as the time server is the usual setup.

Answer: its a nothing burger. The other side won't enter a feedback loop and will prop just use the last image anyway. If it matters, that part of the harness will just have to run on the pi.

Note: the harnes (layer b) is a bunch of docker containers that can live on PC or Pi as needed.

## 9. Hardware details not yet written down

- The motor driver board and the pins it uses.
- Home switch wiring (normally open or normally closed) and its pin.
- The e-stop input.

Will cross that bridge later.

## 10. A no-physics fake driver

Should robbie_ros provide one for the robbie repo's fake stand-ins, or does the robbie repo keep its own?

ANS: robbie_ros does not provide one for the robbie repo.

## 11. ROS networking between the PC and the Pi

ANSWER: Settled by an ADR in the robbie repo. Layer B owns the bridge node, and it runs on the Pi, so ROS traffic between B and C stays on the Pi. The Pi is on WiFi.
