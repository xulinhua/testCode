#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_alg_demo',
            executable='aruco_alg_demo_node',
            name='aruco_demo',
            output='screen',
            parameters=[
                {'camera_id': 0},
                {'use_real_sense': False},
                {'display_results': True},
                {'print_console': True},
                {'marker_length': 0.1},
                {'enable_scaling': False},
                {'scale_factor': 0.5}
            ]
        ),
    ])