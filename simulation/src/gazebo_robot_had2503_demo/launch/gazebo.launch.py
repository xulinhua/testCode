"""Gazebo Classic + HAD2503 + ros2_control（差速底盘 + 双臂）。

沿用终端里的 GAZEBO_MASTER_URI / ROS_DOMAIN_ID。
强制修正 GAZEBO_MODEL_PATH：URDF 的 package:// 会变成 model://包名/...，
必须能解析到 install/.../share/<包名>/meshes，否则模型空载、GUI 卡在 Preparing、
也不会出现 /controller_manager。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("gazebo_robot_had2503_demo")
    gazebo_ros = get_package_share_directory("gazebo_ros")

    # install/.../share  —— model://gazebo_robot_had2503_demo/... 在此下查找
    share_parent = os.path.dirname(pkg)
    system_models = "/usr/share/gazebo-11/models"

    # 覆盖环境里错误的 /opt/ros/humble/share（会扫出满屏 Missing model.config）
    set_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        f"{share_parent}:{system_models}",
    )
    # 避免连接已失效的 models.gazebosim.org 卡住 Preparing
    set_model_db = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg, "worlds", "empty.world"),
    )
    spawn_z_arg = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.10",
        description="Spawn Z so drive wheels touch ground (tune with wheel_radius)",
    )

    xacro_path = os.path.join(pkg, "urdf", "had2503.gazebo.xacro")
    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_path]), value_type=str)
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gazebo.launch.py")),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "had2503",
            "-z", LaunchConfiguration("spawn_z"),
            "-timeout", "120.0",
        ],
        output="screen",
    )

    load_controllers = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            r"""
set -e
echo "[had2503] GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-<default>} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "[had2503] GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"
echo "[had2503] waiting for /controller_manager/load_controller ..."
READY=0
for i in $(seq 1 180); do
  if ros2 service list 2>/dev/null | grep -q '/controller_manager/load_controller'; then
    echo "[had2503] controller_manager is up (attempt $i)"
    READY=1
    break
  fi
  sleep 2
done
if [ "$READY" != "1" ]; then
  echo "[had2503] ERROR: controller_manager never appeared."
  echo "  Mesh path / plugin failed. Check GAZEBO_MODEL_PATH contains:"
  echo "    .../install/gazebo_robot_had2503_demo/share"
  exit 1
fi
echo "[had2503] loading joint_state_broadcaster"
ros2 control load_controller --set-state active joint_state_broadcaster
# 先让物理稳定再上臂控制器（Hold 位置），避免刚 spawn 就炸飞
sleep 3
echo "[had2503] loading diff_drive_controller"
ros2 control load_controller --set-state active diff_drive_controller
sleep 2
for c in \
  left_arm_controller \
  right_arm_controller \
  torso_controller \
  left_gripper_controller \
  right_gripper_controller
do
  echo "[had2503] loading $c"
  ros2 control load_controller --set-state active "$c"
done
echo "[had2503] all controllers active"
""",
        ],
        output="screen",
    )

    after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[TimerAction(period=5.0, actions=[load_controllers])],
        )
    )

    return LaunchDescription([
        set_model_path,
        set_model_db,
        world_arg,
        spawn_z_arg,
        gazebo,
        rsp,
        spawn,
        after_spawn,
    ])
