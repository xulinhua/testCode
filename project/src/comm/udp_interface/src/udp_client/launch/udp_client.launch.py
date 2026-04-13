import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 设置日志目录
    log_dir = os.path.join(os.getenv('HOME'), 'ros2_logs', 'udp_cilent')
    os.makedirs(log_dir, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = log_dir

    # 添加相机类型参数  添加了DeclareLaunchArgument来声明camera_type参数
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='',  # 默认为空，表示使用配置文件中的值
        description='Camera type: "Realsense" or "Gemini" (overrides config file)'
    )

    config_path = os.path.join(
        get_package_share_directory('udp_client'),
        'config',
        'udp_client_params.yaml'
    )

    # 添加路径验证
    if not os.path.exists(config_path):
        raise LaunchError(f"Config file not found: {config_path}")
    print(f"Loading parameters from: {config_path}")  # 调试输出

    # 创建节点，如果launch参数提供了相机类型，则覆盖配置文件中的值
    #node_params = [config_path]
    #camera_type = LaunchConfiguration('camera_type') #使用LaunchConfiguration获取参数值
    #if camera_type != '':
    #    # 如果提供了launch参数，将其添加到参数列表中
    #    node_params.append({'camera_type': camera_type})

    # 使用条件表达式检查是否提供了camera_type参数
    node_params = PythonExpression([
        "{'camera_type': '", LaunchConfiguration('camera_type'), "'} if '", 
        LaunchConfiguration('camera_type'), "' != '' else {}"
    ])

    return LaunchDescription([
        camera_type_arg,
        Node(
            package='udp_client',
            executable='udp_client_node',
            name='udp_client',
            output='screen',
            #parameters=[config_path]
            #parameters=node_params  # 使用合并后的参数列表 
            parameters=[config_path, node_params]  # 先加载配置文件，再覆盖参数
        )
    ])
