import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 设置日志目录
    log_dir = os.path.join(os.getenv('HOME'), 'ros2_logs', 'udp_comm_client')
    os.makedirs(log_dir, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = log_dir

    # 添加相机类型参数
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='',  # 默认为空，表示使用配置文件中的值
        description='Camera type: "Realsense" or "Gemini" (overrides config file)'
    )

    # 获取配置文件路径
    client_config_path = os.path.join(
        get_package_share_directory('udp_comm_client'),
        'config',
        'udp_client_params.yaml'
    )

    # 添加路径验证
    if not os.path.exists(client_config_path):
        raise Exception(f"Client config file not found: {client_config_path}")
    print(f"Loading client parameters from: {client_config_path}")  # 调试输出

    # 使用条件表达式检查是否提供了camera_type参数
    node_params = PythonExpression([
        "{'camera_type': '", LaunchConfiguration('camera_type'), "'} if '", 
        LaunchConfiguration('camera_type'), "' != '' else {}"
    ])

    return LaunchDescription([
        camera_type_arg,
        Node(
            package='udp_comm_client',
            executable='udp_client_node',
            name='udp_comm_client',
            output='screen',
            parameters=[client_config_path, node_params]  # 只使用客户端配置文件和节点参数
        )
    ])