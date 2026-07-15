"""单独启动 HAD2503 调试 UI（需已起 Gazebo / 控制器）。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="gazebo_robot_had2503_demo",
                executable="had2503_debug_ui.py",
                name="had2503_debug_ui",
                output="screen",
            )
        ]
    )
