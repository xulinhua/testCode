from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 定义启动参数
    world_arg = DeclareLaunchArgument(
        name='world',
        description='World file to load in Gazebo',
        default_value=PathJoinSubstitution([FindPackageShare('nova_sim'), 'worlds', 'simple.world'])
    )

    urdf_file_arg = DeclareLaunchArgument(
        name='urdf_file',
        description='URDF/xacro file absolute path',
        default_value=PathJoinSubstitution([FindPackageShare('nova_sim'), 'urdf', 'nova_robot_position.urdf'])
    )

    rvizconfig_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=PathJoinSubstitution([FindPackageShare('nova_sim'), 'config', 'nova_sim.rviz']),
    )
    control_tools_arg = DeclareLaunchArgument(
        name='with_control_tools',
        default_value='true',
        description='Start moveit2_arm_executor_cpp and control UI',
    )
    moveit_arg = DeclareLaunchArgument(
        name='with_moveit',
        default_value='true',
        description='Start MoveIt move_group launch',
    )
    moveit_pkg_arg = DeclareLaunchArgument(
        name='moveit_launch_package',
        default_value='nova_moveit_config',
        description='MoveIt launch package name',
    )
    moveit_file_arg = DeclareLaunchArgument(
        name='moveit_launch_file',
        default_value='move_group.launch.py',
        description='MoveIt launch file name',
    )

    # 启动Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nova_sim'), 'launch', 'gazebo_only.launch.py']),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
            ('urdf_file', LaunchConfiguration('urdf_file'))
        ]
    )

    # RViz2 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # 延时加载控制器
    load_controllers = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
                output='screen'
            )
        ]
    )

    # 可选启动 MoveIt（提供 /compute_ik）
    moveit_process = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            LaunchConfiguration('moveit_launch_package'),
            LaunchConfiguration('moveit_launch_file'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_moveit')),
    )
    start_moveit = TimerAction(
        period=1.5,
        actions=[moveit_process],
        condition=IfCondition(LaunchConfiguration('with_moveit')),
    )

    # 可选启动控制工具（IK执行节点 + UI）
    moveit2_executor_node = Node(
        package='nova_sim',
        executable='moveit2_arm_executor_cpp',
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_control_tools')),
    )
    control_ui_process = ExecuteProcess(
        cmd=['ros2', 'run', 'nova_sim', 'nova_control_ui_qt'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_control_tools')),
    )
    start_control_tools = TimerAction(
        period=3.5,
        actions=[moveit2_executor_node, control_ui_process],
        condition=IfCondition(LaunchConfiguration('with_control_tools')),
    )

    return LaunchDescription([
        world_arg,
        urdf_file_arg,
        rvizconfig_arg,
        control_tools_arg,
        moveit_arg,
        moveit_pkg_arg,
        moveit_file_arg,
        gazebo_launch,
        rviz_node,
        load_controllers,
        start_moveit,
        start_control_tools,
    ])
