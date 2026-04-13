from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    生成启动描述文件
    
    该文件定义了如何启动音频I/O模块的ROS节点。
    包括音频发布节点和音频订阅节点。
    """
    
    return LaunchDescription([
        # 音频发布节点
        Node(
            package='audio_io_module_ros',
            executable='audio_publisher',
            name='audio_publisher',
            parameters=[
                {'mic_device_index': 24},      # USB麦克风设备索引
                {'sample_rate': 44100},        # 采样率
                {'channels': 1},               # 声道数
                {'chunk_size': 1024}           # 块大小
            ],
            output='screen'
        ),
        
        # 音频订阅节点
        Node(
            package='audio_io_module_ros',
            executable='audio_subscriber',
            name='audio_subscriber',
            parameters=[
                {'speaker_device_index': 25},  # USB扬声器设备索引
                {'sample_rate': 44100},        # 采样率
                {'channels': 1}                # 声道数
            ],
            output='screen'
        )
    ])