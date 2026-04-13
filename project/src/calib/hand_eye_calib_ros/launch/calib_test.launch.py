from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的share目录
    package_share_directory = get_package_share_directory('hand_eye_calib_ros')
    
    # 配置文件路径
    config_file_path = os.path.join(package_share_directory, 'config', 'calib_test_node.yaml')
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加测试节点
    calib_test_node = Node(
        package='hand_eye_calib_ros',
        executable='calib_test_node',
        name='calib_test_node',
        parameters=[config_file_path],
        output='screen',
        emulate_tty=True,  # 确保TTY终端支持
        shell=False,
        # 添加额外的参数以确保标准输入可用
        additional_env={'PYTHONUNBUFFERED': '1'},
        # 强制使用PTY来确保TTY支持
        prefix=['stdbuf -i0 -o0 -e0']  # 禁用输入/输出/错误缓冲
    )
    
    # 添加节点到启动描述
    ld.add_action(calib_test_node)
    
    return ld