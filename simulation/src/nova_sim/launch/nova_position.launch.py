from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    disable_color_log = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '0')

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
    spawn_z_arg = DeclareLaunchArgument(
        name='spawn_z',
        default_value='0.06',
        description='Initial spawn Z offset for Gazebo entity',
    )

    rvizconfig_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=PathJoinSubstitution([FindPackageShare('nova_sim'), 'config', 'nova_sim.rviz']),
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
            ('urdf_file', LaunchConfiguration('urdf_file')),
            ('spawn_z', LaunchConfiguration('spawn_z')),
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
    )
    start_moveit = TimerAction(
        period=1.5,
        actions=[moveit_process],
    )

    # 启动控制工具（IK执行节点 + 桥接 + UI）
    moveit2_executor_node = Node(
        package='nova_sim',
        executable='moveit2_arm_executor_cpp',
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='2',
    )
    bridge_node = Node(
        package='nova_sim',
        executable='calib_sim_bridge_node',
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='2',
    )
    all_joints_reset_node = Node(
        package='nova_sim',
        executable='all_joints_reset_node',
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='2',
    )
    control_ui_process = Node(
        package='nova_sim',
        executable='nova_control_ui_qt',
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='2',
    )
    start_control_tools = TimerAction(
        period=3.5,
        actions=[moveit2_executor_node, bridge_node, all_joints_reset_node, control_ui_process],
    )

    return LaunchDescription([
        disable_color_log,
        world_arg,
        urdf_file_arg,
        spawn_z_arg,
        rvizconfig_arg,
        moveit_pkg_arg,
        moveit_file_arg,
        gazebo_launch,
        rviz_node,
        load_controllers,
        start_moveit,
        start_control_tools,
    ])
