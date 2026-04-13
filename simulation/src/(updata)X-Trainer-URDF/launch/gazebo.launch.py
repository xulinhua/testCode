import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('x_trainer_urdf').find('x_trainer_urdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'x-trainer_asm-0226.SLDASM.urdf')
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py'
        ]),
        launch_arguments={'world': 'empty.world'}.items()
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'x_trainer', '-topic', 'robot_description'],
        output='screen',
    )
    
    return LaunchDescription([
        gazebo_server,
        robot_state_publisher,
        spawn_entity,
    ])
