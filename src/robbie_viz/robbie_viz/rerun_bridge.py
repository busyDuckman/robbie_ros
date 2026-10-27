"""Logs the neck to Rerun: a 3D view of the linkage, and time series of the sleds and head pose.

    ros2 run robbie_viz rerun_bridge --ros-args -p rerun:=docker [-p twist:=cw]

rerun is where the log goes (ROBBIE_RERUN in .env, set by just setup):
  - docker: the viewer sim.launch.py starts in the dev container, on 127.0.0.1:9876.
  - host:port: streams to a viewer there. It must be the same version as rerun-sdk.
  - a path ending .rrd: saves a recording.

Time is the sim's (the message stamps). The head pose comes from /head_pose_broadcaster/pose, so
this is sim only for now. The 3D view draws the neck from neck.yaml: the platform from the head
pose, the sleds from /joint_states, and the rods and springs between them. Red lines on the
platform are the nod lines.
"""

import math
import os
import socket
import sys

import numpy as np
import rclpy
import rerun as rr
import rerun.blueprint as rrb
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robbie_neck import config, geometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ["sled_1", "sled_2", "sled_3"]
GRACEFUL_ROLL = 1.0    # deg
GRACEFUL_OFF = 10.0    # mm
DOCKER_VIEWER = "127.0.0.1:9876"

RED, BLUE, GREY, YELLOW, ORANGE = (214, 39, 40), (51, 89, 204), (190, 190, 190), (204, 204, 77), (235, 105, 51)


def quat_to_rpy(q):
    """Roll, pitch, yaw in degrees (ZYX), as robbie_poser shows them."""
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(a) for a in (roll, pitch, yaw)]


def quat_mul(a, b):
    """Hamilton product of xyzw quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_z(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def rotate(q, v):
    x, y, z, w = q
    u = np.array([x, y, z])
    v = np.asarray(v, dtype=float)
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


def start_rerun(sink):
    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(name="Neck", origin="world"),
            rrb.Vertical(
                rrb.TimeSeriesView(name="Head roll / pitch (deg)", origin="head/angle"),
                rrb.TimeSeriesView(name="Head yaw (deg)", origin="head/yaw"),
                rrb.TimeSeriesView(name="Off centre (mm), graceful", origin="head/centre"),
                rrb.TimeSeriesView(name="Sleds (deg)", origin="sleds"),
            ),
            column_shares=[3, 2],
        ),
        collapse_panels=True,
    )
    rr.init("robbie_ros")
    if sink == "docker":
        sink = DOCKER_VIEWER
    if sink.endswith(".rrd"):
        os.makedirs(os.path.dirname(os.path.abspath(sink)), exist_ok=True)
        rr.save(sink, default_blueprint=blueprint)
    else:
        rr.connect_grpc(f"rerun+http://{sink}/proxy", default_blueprint=blueprint)


def reachable(sink):
    """False if sink is host:port and nothing is listening there."""
    host, _, port = sink.rpartition(":")
    try:
        socket.create_connection((host, int(port)), timeout=1).close()
    except (OSError, ValueError):
        return False
    return True


class RerunBridge(Node):
    def __init__(self):
        super().__init__("rerun_bridge")
        sink = self.declare_parameter("rerun", "docker").value
        self.pole = config.pole(self.declare_parameter("twist", "").value)
        self.cfg = config.neck_yaml()
        start_rerun(sink)
        self.get_logger().info(f"Logging to Rerun: {sink}")
        if sink not in ("docker", DOCKER_VIEWER) and not sink.endswith(".rrd") and not reachable(sink):
            # The SDK keeps retrying, and holds everything logged in memory until it connects.
            self.get_logger().warn(f"No Rerun viewer at {sink} yet. Start one (rerun --port {sink.rpartition(':')[2]}).")
        self.log_static()

        self.sleds = None   # deg
        self.stamp = None
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(PoseStamped, "/head_pose_broadcaster/pose", self.on_pose, 10)
        self.create_subscription(Float64MultiArray, "/sled_position_controller/commands", self.on_command, 10)

    def log_static(self):
        cfg, pole = self.cfg, self.pole
        rb, rp, zp = cfg["sled"]["pivot_radius"], cfg["platform"]["pivot_radius"], geometry.pivot_z(cfg)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        ring = [(rb * math.cos(a), rb * math.sin(a), zp) for a in np.linspace(0, 2 * math.pi, 73)]
        rr.log("world/ring", rr.LineStrips3D([ring], colors=GREY, radii=0.0005), static=True)

        # The platform frame is level, centred and world-aligned at rest; the head sits on it at facing.
        self.tops = [np.array(p) for p in geometry.platform_pivots(cfg, pole)]
        facing = geometry.head_facing_deg(cfg, pole)
        self.head_to_platform = quat_z(-facing)
        hexagon = [(rp * math.cos(math.radians(a)), rp * math.sin(math.radians(a)), 0) for a in range(0, 361, 60)]
        rr.log("world/platform/outline", rr.LineStrips3D([hexagon], colors=GREY, radii=0.0006), static=True)
        rr.log("world/platform/pivots", rr.Points3D(self.tops, colors=BLUE, radii=0.003), static=True)
        rot = cfg["head"]["rotation_deg"]
        reach = rp * 1.25
        nods = geometry.nod_lines(cfg, pole)
        rr.log("world/platform/nod_lines", rr.LineStrips3D(
            [[(0, 0, 0.001), (reach * math.cos(math.radians(facing - rot + n)), reach * math.sin(math.radians(facing - rot + n)), 0.001)]
             for n in nods],
            colors=RED, radii=0.0006, labels=[f"nod {k}" for k in range(1, 4)]), static=True)
        rr.log("world/platform/head", rr.Transform3D(quaternion=rr.Quaternion(xyzw=quat_z(facing))), static=True)
        rr.log("world/platform/head/forward", rr.Arrows3D(origins=[(0, 0, 0.002)], vectors=[(rp, 0, 0)], colors=ORANGE,
                                                          radii=0.0015), static=True)
        rr.log("world/platform/head/axes", rr.TransformAxes3D(0.02), static=True)

        nearest = min(nods, key=lambda n: abs((rot - n + 180) % 360 - 180))
        rr.log("settings", rr.TextDocument(
            f"twist: {pole} ({geometry.twist_deg(cfg, pole):.1f} deg at rest)\n"
            f"head rotation: {rot:.1f} deg from rod end 1\n"
            f"nod lines: {', '.join(f'{n:.1f}' for n in nods)} deg\n"
            f"head is {(rot - nearest + 180) % 360 - 180:+.1f} deg from the nearest nod line"), static=True)
        rr.log("head/angle/roll", rr.SeriesLines(colors=RED, names="roll"), static=True)
        rr.log("head/angle/pitch", rr.SeriesLines(colors=(44, 160, 44), names="pitch"), static=True)
        rr.log("head/yaw", rr.SeriesLines(colors=BLUE, names="yaw"), static=True)
        rr.log("head/centre/off_mm", rr.SeriesLines(colors=ORANGE, names="off centre"), static=True)
        rr.log("head/centre/graceful", rr.SeriesLines(colors=(44, 160, 44), names="graceful (1 = yes)"), static=True)
        for j, colour in zip(JOINTS, (RED, (44, 160, 44), BLUE)):
            rr.log(f"sleds/actual/{j}", rr.SeriesLines(colors=colour, names=j), static=True)
            rr.log(f"sleds/command/{j}", rr.SeriesPoints(colors=colour, names=f"{j} cmd", marker_sizes=3), static=True)

    @staticmethod
    def set_time(stamp):
        rr.set_time("sim", duration=stamp.sec + stamp.nanosec * 1e-9)

    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if not all(j in pos for j in JOINTS):
            return
        self.sleds = [math.degrees(pos[j]) for j in JOINTS]
        self.stamp = msg.header.stamp
        self.set_time(msg.header.stamp)
        for j, a in zip(JOINTS, self.sleds):
            rr.log(f"sleds/actual/{j}", rr.Scalars(a))

    def on_command(self, msg):
        if len(msg.data) != len(JOINTS) or self.stamp is None:
            return
        # Commands have no stamp: use the latest joint state's. Cheaper than use_sim_time, whose
        # /clock comes at 1 kHz.
        self.set_time(self.stamp)
        for j, a in zip(JOINTS, msg.data):
            rr.log(f"sleds/command/{j}", rr.Scalars(math.degrees(a)))

    def on_pose(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        head_pos = np.array([p.x, p.y, p.z])
        head_q = (o.x, o.y, o.z, o.w)
        self.set_time(msg.header.stamp)

        roll, pitch, yaw = quat_to_rpy(head_q)
        off = math.hypot(p.x, p.y) * 1000
        rr.log("head/angle/roll", rr.Scalars(roll))
        rr.log("head/angle/pitch", rr.Scalars(pitch))
        rr.log("head/yaw", rr.Scalars(yaw))
        rr.log("head/centre/off_mm", rr.Scalars(off))
        rr.log("head/centre/graceful", rr.Scalars(float(abs(roll) <= GRACEFUL_ROLL and off <= GRACEFUL_OFF)))

        plat_q = quat_mul(head_q, self.head_to_platform)
        rr.log("world/platform", rr.Transform3D(translation=head_pos, quaternion=rr.Quaternion(xyzw=plat_q)))
        if self.sleds is None:
            return
        rb, zp = self.cfg["sled"]["pivot_radius"], geometry.pivot_z(self.cfg)
        bottoms = [np.array(geometry.orbit_dir(a)) * rb + [0, 0, zp] for a in self.sleds]
        tops = [head_pos + rotate(plat_q, t) for t in self.tops]
        f = self.cfg["springs"]["from_top"] / self.cfg["rod"]["length"]
        mounts = [t + f * (b - t) for b, t in zip(bottoms, tops)]
        rr.log("world/sleds", rr.Points3D(bottoms, colors=BLUE, radii=0.004, labels=["1", "2", "3"]))
        rr.log("world/rods", rr.LineStrips3D([[b, t] for b, t in zip(bottoms, tops)], colors=GREY, radii=0.0015))
        rr.log("world/springs", rr.LineStrips3D([[mounts[i], mounts[(i + 1) % 3]] for i in range(3)],
                                                colors=YELLOW, radii=0.001))


def main():
    rclpy.init(args=sys.argv)
    node = RerunBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    rr.disconnect()   # flushes, and finishes a .rrd properly


if __name__ == "__main__":
    main()
