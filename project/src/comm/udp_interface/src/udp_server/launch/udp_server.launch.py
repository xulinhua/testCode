import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
     # 设置日志目录
    log_dir = os.path.join(os.getenv('HOME'), 'ros2_logs', 'udp_server')
    os.makedirs(log_dir, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = log_dir

    config = os.path.join(
        get_package_share_directory('udp_server'),
        'config',
        'udp_server_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='udp_server',
            executable='udp_server_node',
            name='udp_server',
            output='screen',
            parameters=[config]
        )
    ])
