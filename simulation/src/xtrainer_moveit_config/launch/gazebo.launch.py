from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_prefix


def generate_launch_description():

    # Set GAZEBO_MODEL_PATH to include our custom models
    models_path = os.path.join(get_package_prefix("xtrainer_moveit_config"), 'share', 'xtrainer_moveit_config', 'models')
    pkg_share = os.path.join(get_package_prefix("x-trainer_asm-0226"), 'share')

    os.environ['GAZEBO_MODEL_PATH'] = models_path + os.pathsep + pkg_share + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='')

    # MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "x-trainer_asm-0226",
        package_name="xtrainer_moveit_config"
    ).to_moveit_configs()

    launch_package_path = moveit_config.package_path

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'use_sim_time': use_sim_time,
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': use_sim_time}
        ],
    )

    # Spawn robot in Gazebo
    gazebo_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'x-trainer_asm-0226',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # Spawn camera model separately with delay
    gazebo_spawn_camera = TimerAction(
        period=3.0,  # 3 seconds delay
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'top_camera_mount',
                    '-database', 'top_camera_mount',
                    '-x', '0.5',
                    '-y', '-0.5',
                    '-z', '1',
                    '-R', '0',
                    '-P', '1.30',
                    '-Y', '1.57'
                ],
                output='screen'
            )
        ]
    )

        # Static TF transform for camera
    camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf_publisher",
        output="screen",
        arguments=["0.5", "-0.5", "1", "0", "1.3", "1.57", "base_link", "camera_link"]
    )

    # Load controllers - connect to Gazebo's controller manager
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                    "--controller-manager", "/controller_manager"],
        output="screen"
    )

    l_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["l_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    r_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["r_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    l_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["l_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    r_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["r_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # MoveIt move_group node
    moveit_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            {"publish_robot_description_semantic": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config_file = "/home/hoson/xtrainer_moveit_config/config/moveit.rviz"

    # Set environment variables for RViz to fix OpenGL issues
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time}
        ],
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "info"]
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description='Gazebo world file'
        ),
        gazebo_launch,
        robot_state_publisher,
        gazebo_spawn_entity,
        gazebo_spawn_camera,
        camera_tf_node,
        joint_state_broadcaster_spawner,
        l_arm_controller_spawner,
        r_arm_controller_spawner,
        l_gripper_controller_spawner,
        r_gripper_controller_spawner,
        moveit_group_node,
        rviz_node,
    ])

    return ld