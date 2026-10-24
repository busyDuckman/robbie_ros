"""Robbie in the MuJoCo sim: ros2_control with mujoco_ros2_control standing in for the hardware.

    ros2 launch robbie_bringup sim.launch.py [headless:=true]

Move the sleds (orbit angles in radians; 0, 120 and 240 deg is level):
    ros2 topic pub --once /sled_position_controller/commands std_msgs/msg/Float64MultiArray \
        "data: [0.5, 2.59, 4.69]"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

CONTROLLERS = ["joint_state_broadcaster", "sled_position_controller", "head_pose_broadcaster"]


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    urdf = PathJoinSubstitution([FindPackageShare("robbie_description"), "urdf", "robbie.urdf.xacro"])
    controllers = PathJoinSubstitution([FindPackageShare("robbie_bringup"), "config", "controllers.yaml"])
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", urdf, " sim:=true headless:=", headless]), value_type=str),
    }

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        # mujoco_ros2_control needs its own build of ros2_control_node, for now.
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            output="both",
            emulate_tty=True,
            parameters=[{"use_sim_time": True}, ParameterFile(controllers)],
            on_exit=Shutdown(),
        ),
    ]
    # One spawner for all of them: separate spawners queue on a shared lock.
    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[*CONTROLLERS, "--param-file", controllers],
            output="both",
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", description="Run without the MuJoCo window"),
        *nodes,
    ])
