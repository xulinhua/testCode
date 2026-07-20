"""本包内嵌 MoveIt2 move_group，对外 /compute_ik。供 Isaac + 抓取栈独立使用。"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_file(rel_path: str) -> str:
    share = get_package_share_directory("nova_grasp_moveit")
    with open(os.path.join(share, rel_path), "r", encoding="utf-8") as f:
        return f.read()


def _load_yaml(rel_path: str):
    share = get_package_share_directory("nova_grasp_moveit")
    with open(os.path.join(share, rel_path), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Isaac 一般用墙钟；仅 Gazebo 等 ROS 时钟时设 true",
    )
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    robot_description = {"robot_description": _load_file("config/moveit/nova_robot.urdf")}
    robot_description_semantic = {
        "robot_description_semantic": _load_file("config/moveit/nova_robot.srdf")
    }
    robot_description_kinematics = {
        "robot_description_kinematics": _load_yaml("config/moveit/kinematics.yaml")
    }
    joint_limits = {
        "robot_description_planning": _load_yaml("config/moveit/joint_limits.yaml")
    }
    planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": _load_yaml("config/moveit/ompl_planning.yaml"),
    }
    trajectory_execution = _load_yaml("config/moveit/moveit_controllers.yaml")

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline_config,
            trajectory_execution,
            {"allow_trajectory_execution": False},
            {"publish_planning_scene": False},
            {"publish_geometry_updates": False},
            {"publish_state_updates": True},
            {"publish_transforms_updates": False},
        ],
    )

    # 不启 robot_state_publisher：Isaac 已发 /tf /joint_states，避免 TF 双源冲突。
    # IK 请求里会带 joint seed，move_group 可直接解。
    return LaunchDescription([use_sim_time_arg, move_group_node])
