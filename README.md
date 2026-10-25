# robbie_ros

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Status: planning](https://img.shields.io/badge/status-early%20planning-orange)](#)
[![TRL 3](https://img.shields.io/badge/TRL-2-lightgrey)](https://en.wikipedia.org/wiki/Technology_readiness_level)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![MuJoCo](https://img.shields.io/badge/sim-MuJoCo-0b5fff)](https://mujoco.org/)
[![Rerun](https://img.shields.io/badge/viz-Rerun-5c2d91)](https://rerun.io/)
[![Raspberry Pi 4](https://img.shields.io/badge/Raspberry%20Pi-4-C51A4A?logo=raspberrypi&logoColor=white)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)
[![Python](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![YouTube](https://img.shields.io/badge/YouTube-video-red?logo=youtube)](https://youtu.be/8_o7KxZShpw)
[![Parent project](https://img.shields.io/badge/part%20of-robbie-8a2be2?logo=github)](https://github.com/busyDuckman/robbie)

Robot control for Robbie, a desktop robot with a moving head and two round LCD eyes.

robbie_ros is a ROS 2 system. It runs on a Raspberry Pi 4, or on the PC for simulation.


> Status: ROS 2 setup, deployment to PI-4 works, MuJoCo simulation.

## 1.0 Position in the larger Robbie project

Robbie is built in three layers:

- **A, the LLM.**  AI control and dialogue.  
- **B, the harness.** State, attention, mood, camera, microphone and speakers. 
- **C, robot control.** This repo. Drives the neck motors, reads the sensors, runs gaze  motion, and renders the eyes.
- **D, physical layer.** Robot physical design, and CAD files. 

Layers A nd B live in the [robbie] repo. Layer D cad files are in [robbie_openscad].

This repo is holds **layer C**, and provides the **layer D** neck joint design, because that requires the MuJoCo simulator. 

The harness talks to robbie_ros through a single ROS 2 bridge node. Nothing else in the
harness uses ROS. Standard ROS 2 message types are used where possible, so another ROS 2
robot could stand in for robbie_ros by remapping topics.



## 2.0 What robbie_ros does

Firmware:
- **Neck control.** Turns a desired head pose into sled motion, and keeps the sleds from colliding.
- **Homing.** Works out where the sleds are at power-on, using the encoders and a single
  shared home switch. It must work whatever order the sleds were assembled in.
- **Gaze.** Turns gaze targets from the harness into natural motion using [lookaround].
- **Unwinding.** The head can yaw through several full turns. robbie_ros unwinds it when
  needed, overriding or reinterpreting harness requests to do so.
- **Eyes.** Draws and animates the two eyes (gaze, blinks, lids, expression).
- **Safety.** Enforces joint limits, and stops safely if the harness goes silent or the
  e-stop is pressed.
- **Reporting.** Sends the harness the timestamped camera pose, whether the gaze target is
  reachable, status, and a heartbeat.

Simulation:
  - A Virtual robbie to play with
  - Resolves part specs to make a well behaved neck linkage and explains/demonstrates the trade off in play.
  - Allows the development of the IK solver.

## 3.0 The hardware

### 3.1 The neck

- Three motorised sleds orbit a ring gear on a shared track.
- Three rods link the sleds to the head platform.
- Three springs each join a pair of neighbouring rods.
- Sleds spaced at 120 deg give a level head.
- Moving them in unison yaws the head, with unlimited turns.
- Moving them out of unison angles the head (roll/pitch).
- The sleds use geared N20 motors that can't be back-driven, so motor power can be cut once a pose is reached.

- The springs only return the system to one of two stable poses. 
   - They don't affect the dynamics, so any reasonable springs work.
   - It looks a lot like a three-sided tensegrity prism, but it's different. Like a tensegrity prism, the rods
     lock to a stable pose, but because the top and lower platforms are fixed (not flexible) the vertical ropes 
     are not needed. 
   - if the 3-legged platform is fully torsioned (either way) it becomes stable because it binds the movement 
     limits of the upper and lower rod ends. 
   - The springs do nothing except ensure that we don't leave the bound state.
   - manipulating the lower rod positions now controls the roll/pitch of the top platform.
   - yaw comes from all 3 moving together.
 
 **Notes for AI agents**: I have not found an AI agent that can figure out how the neck works. Any AI attempting 
to directly work on the neck should be wary, it is very likely to make mistakes.

## 3.2 Sensors and outputs
  - One AS5600 magnetic encoder per sled pinion, one on each of the Pi's I2C buses.
  - One home switch, pressed by every sled as it passes.
  - Two GC9A01 round SPI LCDs for eyes.
  
  
## 3.3 Photos etc
These photos are quite old; this repo is a re-implementation (using modern tools) of a device I made quite a few years ago.

<p>
  <img src="docs/referencw_potos/platform_assembly.jpg" alt="The neck: three sleds, rods and springs under the head platform" width="300">
  <img src="docs/referencw_potos/pi_position.jpg" alt="The Pi and eyes mounted on the head platform" width="300">
</p>

Assembly animation (from [robbie_openscad]):

<a href="https://youtu.be/8_o7KxZShpw">
  <img src="https://img.youtube.com/vi/8_o7KxZShpw/0.jpg" alt="Robbie assembly animation on YouTube" width="600">
</a>


## 4.0 Simulation

The simulation uses MuJoCo for physics and Rerun for visualisation and logging. It plugs in at the
hardware driver, so everything above the driver is the same code in sim and on the Pi.

<img src="docs/referencw_potos/simulator.jpg" alt="The neck in the MuJoCo viewer" width="600">


From Linux or a WSL shell, after `just setup`:

```
just up              # build, then start the sim
just up --headless   # without the windows
just down            # stop the simulation (don't just close the window)
```

"just up" also opens a small gui for driving the robots joints, etc.

<img src="docs/referencw_potos/pose_in_simulation.jpg" alt="The neck in the MuJoCo viewer" width="600">


**Notes**

- The geometry is defined in [neck.yaml], and the MuJoCo model is rebuilt from it at "just up"
     
- The rod ends jamming creates a fully stable pose,working in consert with the springs which effectively
  torsions the system into the jam. But the exact jamming angle of the ball joint is not measured. 
- So there is a **gap between the simulation and reality**, that probably changes with the brand of push-rod end used.
- The simulations's springs are also stiffer than the real ones. Softer springs in the sim result in the system 
  moving to the other twist pose (stable pole) when doing large/fast movements. In reality thats not possible, 
  without significant external force.

# 5.0 Design of the neck linkage

The system is non trivial, so I used the simulation to study the behaviour.

## 5.1 Overview
We essentially have a tensegrity-like prism for a neck joint which behaves to transform the sled orientation, to a platform roll/pitch. This system is not perfect and performs some motions really well, and others quite poorly. And thats actually OK, provided we get the system behaving well where it counts.

**Important terms used form now on**: 
  - **gracefull**: means no unwanted roll, jerky motion, or going off-center.
    - |roll| <= 1° and off_centre <= 10 mm. 
  - **ugly**: means, by any means necessary.


Things a head joint must do "gracefully":
  - look up and down (pitch)
  - turn side to side (yaw)
  - track between positions smoothly.

Things a neck must be able to do, "ugly".
  - tilt to the side (roll)
  - lateral movements

The linkages dynamics change according to:
  - rod length
  - jam angle of the push rod linkage
  - radius of upper platform
  - radius of sled linkages.

To simplify the system a bit, the last two could be condensed down to 
the ratio of the two radii. And further the ratio has its own trade of,
structural strength, vs speed of motion.  Also there is an issue of bad ratios 
providing enough leverage to back-drive the motors.

Experimentally (having tested on real life setup) I think the best ratio is 
about 0.9 (with he top sled being smaller).  For the current setup we will use 0.93,
because it keeps the top hexagon size a nice round number.

If you want to scale the size in robbie_openscad, just scale the rod lengths to mach and 
all the charts below should still work.

**tldr;** So now we have a stable system, that does not back-drive motors, and we need 
to tune in the rod length and the jam angle of the sled linkage.

## 5.2 What is the available movement?


## 5.3 What does th movement proceed poorly?


## 5.4 Rod length trade off

Results at the current rod-end angle (22°), going from 70 mm to 87 mm to 115 mm:

Factor	Better	70 mm	87 mm	115 mm
Max tilt (°)	higher	18.3	15.9	12.0
Pitch up and down at the best heading (°)	higher	4.1	5.7	5.9 (6.0 at 105)
Least off centre at 10° tilt (mm)	lower	5.1	7.3	12.8
Off centre at max tilt (mm)	lower	11.5	14.0	16.9
Swing (deg/deg)	lower	0.91	0.78	0.74

## 5.5 Jam angle trade off


## 5.6 The loop up vs look down trade off.

Depending on configuration, the neck joint either looks up better, or looks down better. 


As it turns out, manually twisting the head platform between the two stable poses, lets you 
choose between the look up/down modes.

| Twist | Up (graceful) | Down (graceful) | Up (ugly) | Down (ugly) |
|-------|--------------:|----------------:|----------:|------------:|
| CW    |         11.6° |            5.0° |     15.9° |        9.1° |
| CCW   |          5.0° |           11.6° |      9.1° |       15.9° |

That true of any type of neck, but the "polarity" changes; so depending on your build, you may 
need to twist it the other way.

Rod length effect the trade of as well, as per:

| Rod (mm) \ jam |         60 |         70 |         75 |         80 |         85 |         90 |         100 |
|---------------:|-----------:|-----------:|-----------:|-----------:|-----------:|-----------:|------------:|
|             70 | 3.0 / 16.2 | 4.5 / 17.3 | 6.8 / 10.5 |  9.0 / 8.5 | 11.2 / 8.7 | 13.6 / 9.0 | 17.6 / 11.0 |
|             87 | 2.4 / 12.3 | 3.6 / 12.7 | 5.0 / 11.6 |  6.4 / 7.7 |  8.2 / 7.1 |  9.2 / 6.3 | 10.2 /  6.9 |
|            115 |  1.8 / 8.9 |  2.9 / 8.9 |  3.7 / 8.4 |  4.6 / 8.0 |  5.3 / 6.5 |  6.0 / 5.1 |  6.5 /  4.7 |

_(Each cell is the graceful up/down in degrees)_

The [lookaround ] controller works well with this, it can set the preferred movement range of a neck, and use the eyes to give robbie a good gaze, only going into the _ugly_ mode if the eyes are at full extent.


## 5.7 Putting it all together


# 6.0 Inverse Kinematics


# 7.0 Conclusion

IDK; After a document this long I kind write a conclusion by reflex...   
um...  
IDK, if you DO build a robbie please send me a photo;   
That's all folks.

## 7.1 Related repos

- [robbie]: the umbrella project (layers A and B), and `robbie_interfaces`.
- [robbie_openscad]: 3D-printable parts and assembly.
- [robbie_restoration]: eye rendering and display drivers being salvaged.
- [lookaround]: natural gaze motion.

[robbie]: https://github.com/busyDuckman/robbie
[robbie_openscad]: https://github.com/busyDuckman/robbie_openscad
[robbie_restoration]: https://github.com/busyDuckman/robbie_restoration
[lookaround]: https://github.com/busyDuckman/look_around
[neck.yaml]: src/robbie_description/config/neck.yaml
[random_thoughts_on_the_ik.md]: .plans\random_thoughts_on_the_ik.md
