from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 构建MoveIt配置
    moveit_config = MoveItConfigsBuilder("x-trainer_asm-0226", package_name="xtrainer_moveit_config").to_moveit_configs()
    
    # ros2_control配置文件路径
    ros2_controllers_path = PathJoinSubstitution([
        FindPackageShare("xtrainer_moveit_config"),
        "config", 
        "ros2_controllers.yaml",
    ])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )
    
    # ros2_control节点
    ros2_control_node = Node(
        package="controller_manager", 
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )
    
    # 只启动必要的控制器
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            )
        ],
    )
    
    # 启动所有控制器（一次性）
    controllers_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager", 
                executable="spawner",
                arguments=[
                    "l_arm_controller", 
                    "r_arm_controller", 
                    "l_gripper_controller", 
                    "r_gripper_controller",
                    "--controller-manager", "/controller_manager"
                ],
            )
        ],
    )
    
    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        controllers_spawner,
    ])