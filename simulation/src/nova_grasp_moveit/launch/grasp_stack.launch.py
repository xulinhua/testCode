"""一键启动：本包 MoveIt move_group + 抓取执行器 + Qt UI。

与 Isaac 联调时只需：Isaac 插件 Play + 本 launch。
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Isaac 默认 false；其它仿真时钟源再改",
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nova_grasp_moveit"),
                "launch",
                "move_group.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    ui_node = Node(
        package="nova_grasp_moveit",
        executable="grasp_qt_ui_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    executor_node = Node(
        package="nova_grasp_moveit",
        executable="grasp_arm_executor_node",
        output="screen",
    )

    return LaunchDescription(
        _pkg_lib_env_actions()
        + [config_arg, use_sim_time_arg, move_group, executor_node, ui_node]
    )
