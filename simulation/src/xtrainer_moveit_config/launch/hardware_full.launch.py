from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    left_arm_ip = LaunchConfiguration("left_arm_ip")
    right_arm_ip = LaunchConfiguration("right_arm_ip")
    left_gripper_port = LaunchConfiguration("left_gripper_port")
    right_gripper_port = LaunchConfiguration("right_gripper_port")

    # Generate moveit config
    moveit_config = MoveItConfigsBuilder("x-trainer_asm-0226", package_name="xtrainer_moveit_config").to_moveit_configs()

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("xtrainer_moveit_config"), "config", "moveit.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem (if use_fake_hardware is true)
    fake_ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            PathJoinSubstitution([FindPackageShare("xtrainer_moveit_config"), "config", "ros2_controllers.yaml"]),
        ],
        condition=IfCondition(use_fake_hardware),
    )

    # ros2_control using real hardware (if use_fake_hardware is false)
    real_ros2_control_node = Node(
        package="controller_manager", 
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            PathJoinSubstitution([FindPackageShare("xtrainer_moveit_config"), "config", "ros2_controllers.yaml"]),
            {
                "left_arm_ip": left_arm_ip,
                "right_arm_ip": right_arm_ip, 
                "left_gripper_port": left_gripper_port,
                "right_gripper_port": right_gripper_port,
            }
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    # Load controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    l_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["l_arm_controller", "--controller-manager", "/controller_manager"],
    )

    r_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner", 
        arguments=["r_arm_controller", "--controller-manager", "/controller_manager"],
    )

    l_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["l_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    r_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["r_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    nodes_to_start = [
        static_tf,
        robot_state_publisher,
        fake_ros2_control_node,
        real_ros2_control_node,
        joint_state_broadcaster_spawner,
        l_arm_controller_spawner,
        r_arm_controller_spawner,
        l_gripper_controller_spawner, 
        r_gripper_controller_spawner,
        run_move_group_node,
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    
    # Hardware-specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_arm_ip",
            default_value="192.168.5.1",
            description="Left arm IP address",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_arm_ip",
            default_value="192.168.5.2", 
            description="Right arm IP address",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_gripper_port",
            default_value="/dev/ttyUSB0",
            description="Left gripper serial port",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_gripper_port", 
            default_value="/dev/ttyUSB1",
            description="Right gripper serial port",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
