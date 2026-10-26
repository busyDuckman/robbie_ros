"""A small window for posing the neck by hand.

Sliders set a yaw (all sleds together) and an offset per sled on top of it. The command goes to
/sled_position_controller/commands as orbit angles. The head pose is read back from
/head_pose_broadcaster/pose (sim only, in the head's frame), with the sled angles from /joint_states.

    ros2 run robbie_poser poser [--ros-args -p twist:=cw]
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QApplication, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget)
from rclpy.node import Node
from robbie_neck import config
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ["sled_1", "sled_2", "sled_3"]
YAW_RANGE = 360      # deg either way. The neck can turn further; wider makes the slider too coarse.
OFFSET_RANGE = 50    # deg either way; the sleds can't pass each other


def quat_to_rpy(q):
    """Roll, pitch, yaw in degrees (ZYX)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(a) for a in (roll, pitch, yaw)]


class PoserNode(Node):
    def __init__(self):
        super().__init__("robbie_poser")
        self.pole = config.pole(self.declare_parameter("twist", "").value)
        self.head_rotation = config.head_rotation_deg()
        self.pub = self.create_publisher(Float64MultiArray, "/sled_position_controller/commands", 10)
        self.sleds = None   # deg, from /joint_states
        self.pose = None
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(PoseStamped, "/head_pose_broadcaster/pose", self.on_pose, 10)

    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.sleds = [math.degrees(pos[j]) for j in JOINTS]

    def on_pose(self, msg):
        self.pose = msg.pose

    def send(self, deg):
        self.pub.publish(Float64MultiArray(data=[math.radians(d) for d in deg]))


class Slider(QWidget):
    """A labelled integer-degree slider."""

    def __init__(self, name, limit, on_change):
        super().__init__()
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(-limit, limit)
        self.slider.setMinimumWidth(320)
        self.value = QLabel()
        self.value.setMinimumWidth(60)
        self.value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        label = QLabel(name)
        label.setMinimumWidth(60)
        layout.addWidget(label)
        layout.addWidget(self.slider)
        layout.addWidget(self.value)
        self.slider.valueChanged.connect(lambda v: (self.value.setText(f"{v} deg"), on_change()))
        self.value.setText("0 deg")

    def get(self):
        return self.slider.value()

    def set(self, v, quiet=False):
        self.slider.blockSignals(quiet)
        self.slider.setValue(int(round(v)))
        self.value.setText(f"{self.slider.value()} deg")
        self.slider.blockSignals(False)


class PoserWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.start = config.neck_yaml()["sled"]["start_deg"]
        self.synced = False   # sliders are set from the robot's current pose before they send anything
        self.setWindowTitle("Robbie neck poser")

        self.yaw = Slider("Yaw", YAW_RANGE, self.send)
        self.offsets = [Slider(f"Sled {i + 1}", OFFSET_RANGE, self.send) for i in range(3)]

        level = QPushButton("Level (offsets to 0)")
        level.clicked.connect(lambda: self.set_all(self.yaw.get(), [0, 0, 0]))
        home = QPushButton("Home (all to 0)")
        home.clicked.connect(lambda: self.set_all(0, [0, 0, 0]))
        buttons = QHBoxLayout()
        buttons.addWidget(level)
        buttons.addWidget(home)

        command = QGroupBox("Command")
        cl = QVBoxLayout(command)
        cl.addWidget(self.yaw)
        for s in self.offsets:
            cl.addWidget(s)
        cl.addLayout(buttons)

        mono = QFont("Monospace")
        mono.setStyleHint(QFont.TypeWriter)
        self.readout = {}
        state = QGroupBox("Measured")
        grid = QGridLayout(state)
        rows = ["Twist", "Head rotation", "Head roll", "Head pitch", "Head yaw", "Head height", "Sled angles", "Sled gaps"]
        for r, name in enumerate(rows):
            grid.addWidget(QLabel(name), r, 0)
            self.readout[name] = QLabel("-")
            self.readout[name].setFont(mono)
            grid.addWidget(self.readout[name], r, 1)
        self.readout["Twist"].setText(f"{node.pole:>7} seen from above")
        self.readout["Head rotation"].setText(f"{node.head_rotation:7.1f} deg from rod end 1")

        layout = QVBoxLayout(self)
        layout.addWidget(command)
        layout.addWidget(state)
        self.set_enabled(False)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)

    def set_enabled(self, on):
        for s in [self.yaw, *self.offsets]:
            s.setEnabled(on)

    def set_all(self, yaw, offsets):
        self.yaw.set(yaw, quiet=True)
        for s, o in zip(self.offsets, offsets):
            s.set(o, quiet=True)
        self.send()

    def send(self):
        if self.synced:
            self.node.send([s + self.yaw.get() + o.get() for s, o in zip(self.start, self.offsets)])

    def sync_from_robot(self):
        """Start the sliders at the robot's current pose, so opening the window doesn't move it."""
        rel = [a - s for a, s in zip(self.node.sleds, self.start)]
        yaw = sum(rel) / 3
        self.yaw.set(yaw, quiet=True)
        for s, r in zip(self.offsets, rel):
            s.set(r - yaw, quiet=True)
        self.synced = True
        self.set_enabled(True)

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.node.sleds is not None:
            if not self.synced:
                self.sync_from_robot()
            a = self.node.sleds
            self.readout["Sled angles"].setText("  ".join(f"{x:7.1f}" for x in a) + "  deg")
            gaps = [(a[(i + 1) % 3] - a[i]) % 360 for i in range(3)]
            self.readout["Sled gaps"].setText("  ".join(f"{g:7.1f}" for g in gaps) + "  deg")
        if self.node.pose is not None:
            roll, pitch, yaw = quat_to_rpy(self.node.pose.orientation)
            self.readout["Head roll"].setText(f"{roll:7.1f} deg")
            self.readout["Head pitch"].setText(f"{pitch:7.1f} deg")
            self.readout["Head yaw"].setText(f"{yaw:7.1f} deg")
            self.readout["Head height"].setText(f"{self.node.pose.position.z * 1000:7.1f} mm")


def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    node = PoserNode()
    window = PoserWindow(node)
    window.show()
    code = app.exec_()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
