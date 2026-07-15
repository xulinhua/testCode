"""一键：Gazebo + 控制器 + MoveIt2 move_group + 调试 UI。

沿用终端中的 GAZEBO_MASTER_URI / ROS_DOMAIN_ID，不做覆盖。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("gazebo_robot_had2503_demo")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "gazebo.launch.py")),
        launch_arguments={
            "spawn_z": LaunchConfiguration("spawn_z"),
        }.items(),
    )

    # 延后：等模型 mesh / ros2_control 就绪
    move_group = TimerAction(
        period=30.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "move_group.launch.py")),
                launch_arguments={"use_sim_time": "true"}.items(),
            )
        ],
    )

    rviz = TimerAction(
        period=32.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(pkg, "rviz", "had2503.rviz")],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    debug_ui = TimerAction(
        period=25.0,
        actions=[
            Node(
                package="gazebo_robot_had2503_demo",
                executable="had2503_debug_ui.py",
                name="had2503_debug_ui",
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("spawn_z", default_value="0.10"),
        gazebo,
        move_group,
        rviz,
        debug_ui,
    ])
