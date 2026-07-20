"""启动抓取 Qt UI（仿真与 move_group 需已运行）。"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _pkg_lib_env_actions():
    prefix = get_package_prefix("nova_grasp_moveit")
    lib_dir = os.path.join(prefix, "lib")
    return [
        AppendEnvironmentVariable(name="LD_LIBRARY_PATH", value=lib_dir),
        AppendEnvironmentVariable(name="AMENT_PREFIX_PATH", value=prefix),
    ]


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("nova_grasp_moveit"), "config", "grasp_moveit.yaml"]
        ),
        description="grasp Qt UI 参数文件",
    )

    ui_node = Node(
        package="nova_grasp_moveit",
        executable="grasp_qt_ui_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription(_pkg_lib_env_actions() + [config_arg, ui_node])
