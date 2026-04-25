import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.actions import SetEnvironmentVariable
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    disable_color_log = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '0')
    has_display = bool(os.environ.get('DISPLAY'))

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
    start_moveit_arg = DeclareLaunchArgument(
        name='start_moveit',
        default_value='true',
        description='Whether to launch move_group (sets up /compute_ik for Pose Control)',
    )
    start_motion_stack_arg = DeclareLaunchArgument(
        name='start_motion_stack',
        default_value='true',
        description='Whether to launch moveit2_arm_executor + calib_sim_bridge (needed for /nova_target_pose execution)',
    )
    start_tf_render_cameras_arg = DeclareLaunchArgument(
        name='start_tf_render_cameras',
        default_value='false',
        description='Publish TF-rendered ArUco camera streams in MuJoCo (for calibration).',
    )
    start_mujoco_raster_cameras_arg = DeclareLaunchArgument(
        name='start_mujoco_raster_cameras',
        default_value='true',
        description='Publish real MuJoCo raster RGB/depth camera streams.',
    )
    start_all_joints_reset_node_arg = DeclareLaunchArgument(
        name='start_all_joints_reset_node',
        default_value='false',
        description='Start all_joints_reset_node listener for /nova_sim/reset_all_joints.',
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
        condition=IfCondition(LaunchConfiguration('start_moveit')),
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
        condition=IfCondition(LaunchConfiguration('start_all_joints_reset_node')),
        sigterm_timeout='2',
        sigkill_timeout='2',
    )
    tf_render_camera_node = Node(
        package='nova_sim_mujoco',
        executable='tf_aruco_camera_publishers.py',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_tf_render_cameras')),
    )
    mujoco_raster_camera_node = Node(
        package='nova_sim_mujoco',
        executable='mujoco_raster_camera_publishers.py',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_mujoco_raster_cameras')),
    )
    if has_display:
        control_ui_process = Node(
            package='nova_sim_mujoco',
            executable='nova_control_ui_qt',
            output='screen',
            sigterm_timeout='2',
            sigkill_timeout='2',
        )
        ui_log = LogInfo(msg='DISPLAY detected: launching Qt UI (nova_control_ui_qt).')
    else:
        control_ui_process = Node(
            package='nova_sim_mujoco',
            executable='nova_control_ui_cpp',
            output='screen',
            sigterm_timeout='2',
            sigkill_timeout='2',
        )
        ui_log = LogInfo(msg='DISPLAY not set: fallback to CLI UI (nova_control_ui_cpp).')
    start_motion_tools = TimerAction(
        period=3.5,
        actions=[moveit2_executor_node, bridge_node],
        condition=IfCondition(LaunchConfiguration('start_motion_stack')),
    )
    start_ui_tools = TimerAction(
        period=3.5,
        actions=[all_joints_reset_node, control_ui_process],
    )

    return LaunchDescription([
        disable_color_log,
        urdf_file_arg,
        rvizconfig_arg,
        moveit_pkg_arg,
        moveit_file_arg,
        start_moveit_arg,
        start_motion_stack_arg,
        start_tf_render_cameras_arg,
        start_mujoco_raster_cameras_arg,
        start_all_joints_reset_node_arg,
        mujoco_launch,
        rviz_node,
        ui_log,
        start_moveit,
        start_motion_tools,
        start_ui_tools,
        tf_render_camera_node,
        mujoco_raster_camera_node,
    ])
