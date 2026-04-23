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

    urdf_file_arg = DeclareLaunchArgument(
        name='urdf_file',
        description='URDF/xacro file absolute path',
        default_value=PathJoinSubstitution([FindPackageShare('nova_sim_mujoco'), 'urdf', 'nova_robot_position.urdf'])
    )

    rvizconfig_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=PathJoinSubstitution([FindPackageShare('nova_sim_mujoco'), 'config', 'nova_sim.rviz']),
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

    # 启动 MuJoCo（包含 robot_state_publisher + MuJoCo viewer 进程）
    mujoco_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nova_sim_mujoco'), 'launch', 'mujoco_only.launch.py']),
        launch_arguments=[
            ('urdf_file', LaunchConfiguration('urdf_file')),
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
        package='nova_sim_mujoco',
        executable='moveit2_arm_executor_cpp',
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='2',
    )
    bridge_node = Node(
        package='nova_sim_mujoco',
        executable='calib_sim_bridge_node',
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='2',
    )
    all_joints_reset_node = Node(
        package='nova_sim_mujoco',
        executable='all_joints_reset_node',
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='2',
    )
    control_ui_process = Node(
        package='nova_sim_mujoco',
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
        urdf_file_arg,
        rvizconfig_arg,
        moveit_pkg_arg,
        moveit_file_arg,
        mujoco_launch,
        rviz_node,
        load_controllers,
        start_moveit,
        start_control_tools,
    ])
