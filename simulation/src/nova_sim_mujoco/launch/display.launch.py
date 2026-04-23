import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('nova_sim_mujoco').find('nova_sim_mujoco')
    urdf_file = os.path.join(pkg_share, 'urdf', 'nova_robot_position.urdf')
    
    # 直接读取 URDF 文件（不需要 xacro）
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'nova_sim.rviz')],
        ),
    ])
