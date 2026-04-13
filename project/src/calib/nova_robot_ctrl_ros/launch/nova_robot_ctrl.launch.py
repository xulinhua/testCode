from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_name = 'nova_robot_ctrl_ros'
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    # 配置文件路径
    config_file_path = os.path.join(pkg_share_dir, 'config', 'nova_robot_ctrl_ros.yaml')
    
    # 创建节点
    nova_robot_ctrl_node = Node(
        package=pkg_name,
        executable='nova_robot_ctrl_node',
        name='nova_robot_ctrl_node',
        parameters=[config_file_path],
        output='screen'
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    ld.add_action(nova_robot_ctrl_node)
    
    return ld