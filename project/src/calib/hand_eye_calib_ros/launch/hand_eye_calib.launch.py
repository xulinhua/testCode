from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 设置ROS2日志格式环境变量
    set_env_var = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '{message}'
    )
    
    # 获取包目录
    pkg_dir = get_package_share_directory('hand_eye_calib_ros')
    
    # 配置文件路径
    config_file = os.path.join(pkg_dir, 'config', 'hand_eye_calib_ros.yaml')
    
    # 创建节点
    hand_eye_calib_node = Node(
        package='hand_eye_calib_ros',
        executable='hand_eye_calib_node',
        name='hand_eye_calib_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--disable-external-lib-logs', '--log-level', 'INFO']
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    ld.add_action(set_env_var)
    ld.add_action(hand_eye_calib_node)
    
    return ld