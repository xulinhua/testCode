"""
Nova Robot Tools Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nova_robot_tools',
            executable='nova_robot_tools_ui',
            name='nova_robot_tools_node',
            output='screen',
        )
    ])
