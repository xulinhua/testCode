"""本包 MoveIt2 move_group：左右臂 IK / 规划。

Octomap / “vertices in collision” 警告与 Gazebo 飞车无关；
飞车来自高面数 mesh 碰撞，URDF 里视觉保留 STL、碰撞已改为盒/柱。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(path: str):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg = get_package_share_directory("gazebo_robot_had2503_demo")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    xacro_path = os.path.join(pkg, "urdf", "had2503.gazebo.xacro")
    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_path]), value_type=str)
    }
    with open(os.path.join(pkg, "moveit", "had2503.srdf"), "r", encoding="utf-8") as f:
        srdf = f.read()

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            {"robot_description_semantic": srdf},
            {"robot_description_kinematics": _load_yaml(os.path.join(pkg, "moveit", "kinematics.yaml"))},
            {"robot_description_planning": _load_yaml(os.path.join(pkg, "moveit", "joint_limits.yaml"))},
            {
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
                "ompl": _load_yaml(os.path.join(pkg, "moveit", "ompl_planning.yaml")),
            },
            _load_yaml(os.path.join(pkg, "moveit", "moveit_controllers.yaml")),
            {"allow_trajectory_execution": True},
            {"publish_robot_description_semantic": True},
            {"publish_planning_scene": True},
            {"publish_state_updates": True},
        ],
    )

    return LaunchDescription([use_sim_time_arg, move_group])
