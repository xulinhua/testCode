import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def load_file(package_name: str, relative_path: str) -> str:
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as f:
        return f.read()


def load_yaml(package_name: str, relative_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    nova_test_share = get_package_share_directory("nova_test")
    urdf_path = os.path.join(nova_test_share, "urdf", "nova_robot_position.urdf")

    robot_description = {"robot_description": Command(["xacro ", urdf_path])}
    robot_description_semantic = {
        "robot_description_semantic": load_file("nova_moveit_config", "config/nova_robot.srdf")
    }
    robot_description_kinematics = load_yaml("nova_moveit_config", "config/kinematics.yaml")
    joint_limits = {"robot_description_planning": load_yaml("nova_moveit_config", "config/joint_limits.yaml")}

    planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": load_yaml("nova_moveit_config", "config/ompl_planning.yaml"),
    }
    trajectory_execution = load_yaml("nova_moveit_config", "config/moveit_controllers.yaml")

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline_config,
            trajectory_execution,
            {"allow_trajectory_execution": False},
            {"publish_planning_scene": True},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
        ],
    )

    return LaunchDescription([move_group_node])
