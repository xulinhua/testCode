#!/usr/bin/env python3
"""
Launch file for fixed position grasper demo with MoveIt
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('xtrainer_moveit_config')

    # Path to the robot description and semantic description files
    xacro_file = os.path.join(pkg_share, 'config', 'x-trainer_asm-0226.urdf.xacro')
    srdf_file = os.path.join(pkg_share, 'config', 'x-trainer_asm-0226.srdf')

    # Process xacro file to get URDF
    robot_description_content = Command([
        'xacro ', xacro_file
    ])

    # Read the SRDF file
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # Launch the fixed position grasper node with proper parameters and QoS settings
    grasper_node = Node(
        package='xtrainer_moveit_config',
        executable='fixed_position_grasper_cpp',
        name='fixed_position_grasper',
        output='screen',
        parameters=[
            # {'robot_description': robot_description_content},
            # {'robot_description_semantic': robot_description_semantic},
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        grasper_node
    ])