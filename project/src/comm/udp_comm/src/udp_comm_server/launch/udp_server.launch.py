import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明启动参数
    client_ip_arg = DeclareLaunchArgument(
        'client_ip',
        default_value='192.168.10.61',
        description='Client IP address'
    )
    
    bShowRunInfo_arg = DeclareLaunchArgument(
        'bShowRunInfo',
        default_value='true',
        description='Show runtime information'
    )
    
    # 获取配置文件路径
    server_config_path = os.path.join(
        get_package_share_directory('udp_comm_server'),
        'config',
        'udp_server_params.yaml'
    )

    # 添加路径验证
    if not os.path.exists(server_config_path):
        raise Exception(f"Server config file not found: {server_config_path}")
    print(f"Loading server parameters from: {server_config_path}")  # 调试输出

    # 创建UDP服务器节点
    udp_server_node = Node(
        package='udp_comm_server',
        executable='udp_server_node',
        name='udp_comm_server',
        output='screen',
        parameters=[server_config_path, {
            'client_ip': LaunchConfiguration('client_ip'),
            'bShowRunInfo': LaunchConfiguration('bShowRunInfo'),
            'log_interval': 1
        }]  # 使用服务器配置文件和启动参数
    )

    return LaunchDescription([
        client_ip_arg,
        bShowRunInfo_arg,
        udp_server_node
    ])