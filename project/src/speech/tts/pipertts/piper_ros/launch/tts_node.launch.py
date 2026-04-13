from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    save_audio_to_file = DeclareLaunchArgument(
        'save_audio_to_file',
        default_value='false',
        description='是否保存合成的音频数据到文件'
    )
    
    audio_save_directory = DeclareLaunchArgument(
        'audio_save_directory',
        default_value=PathJoinSubstitution([
            FindPackageShare('piper_ros'),
            'resources',
            'audio_data_ros'
        ]),
        description='音频文件保存目录'
    )
    
    model_path = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('piper_ros'),
            'resources',
            'models',
            'zh_CN',
            'medium',
            'zh_CN-huayan-medium.onnx'
        ]),
        description='Piper模型路径'
    )
    
    config_path = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('piper_ros'),
            'resources',
            'models',
            'zh_CN',
            'medium',
            'zh_CN-huayan-medium.onnx.json'
        ]),
        description='Piper模型配置文件路径'
    )
    
    sample_rate = DeclareLaunchArgument(
        'sample_rate',
        default_value='22050',
        description='音频采样率'
    )
    
    speaker_id = DeclareLaunchArgument(
        'speaker_id',
        default_value='0',
        description='说话人ID'
    )
    
    # 定义TTS节点 - 使用Python模块方式启动
    tts_node = Node(
        package='piper_ros',
        executable='tts_node',
        name='piper_tts_node',
        output='screen',
        parameters=[
            {'save_audio_to_file': LaunchConfiguration('save_audio_to_file')},
            {'audio_save_directory': LaunchConfiguration('audio_save_directory')},
            {'model_path': LaunchConfiguration('model_path')},
            {'config_path': LaunchConfiguration('config_path')},
            {'sample_rate': LaunchConfiguration('sample_rate')},
            {'speaker_id': LaunchConfiguration('speaker_id')},
        ]
    )
    
    return LaunchDescription([
        save_audio_to_file,
        audio_save_directory,
        model_path,
        config_path,
        sample_rate,
        speaker_id,
        tts_node
    ])