"""仅 RViz 查看模型（无物理仿真）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("gazebo_robot_had2503_demo")
    xacro_path = os.path.join(pkg, "urdf", "had2503.gazebo.xacro")
    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_path]), value_type=str)
    }

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(pkg, "rviz", "had2503.rviz")],
        ),
    ])
