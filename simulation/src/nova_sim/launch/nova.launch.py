from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess, TimerAction
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
        default_value=PathJoinSubstitution([FindPackageShare('nova_sim'), 'urdf', 'nova_robot.urdf'])
    )
    
    rvizconfig_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=PathJoinSubstitution([FindPackageShare('nova_sim'), 'config', 'nova_sim.rviz']),
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
        
    return LaunchDescription([
        world_arg,
        urdf_file_arg,
        rvizconfig_arg,
        gazebo_launch,
        rviz_node,
        load_controllers,
    ])
