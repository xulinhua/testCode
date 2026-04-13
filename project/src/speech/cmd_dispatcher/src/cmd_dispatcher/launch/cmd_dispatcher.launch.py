from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_dispatcher',
            executable='cmd_dispatcher',
            name='cmd_dispatcher',
            output='screen'
        )
    ])