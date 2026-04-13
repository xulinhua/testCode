from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成launch描述文件"""
    
    # 声明launch参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='../resources/models/paraformer-zh-streaming/iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online',
        description='Path to the ASR model'
    )
    
    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='pt',
        description='Model type: pt (PyTorch) or onnx'
    )
    
    microphone_topic_arg = DeclareLaunchArgument(
        'microphone_topic',
        default_value='/audio/microphone',
        description='Microphone audio input topic'
    )
    
    asr_result_topic_arg = DeclareLaunchArgument(
        'asr_result_topic',
        default_value='/asr/result',
        description='ASR result output topic'
    )
    
    debug_audio_arg = DeclareLaunchArgument(
        'debug_audio',
        default_value='False',
        description='Enable audio debugging output'
    )
    
    # ASR节点
    asr_node = Node(
        package='funasr_ros',
        executable='asr_node.py',
        name='asr_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'model_type': LaunchConfiguration('model_type'),
            'microphone_topic': LaunchConfiguration('microphone_topic'),
            'asr_result_topic': LaunchConfiguration('asr_result_topic'),
            'audio_format': 16,
            'audio_channels': 1,
            'sample_rate': 16000,
            'chunk_size': 512,
            'enable_vad': True,
            'enable_punctuation': True,
            'debug_audio': LaunchConfiguration('debug_audio')  # 添加调试参数
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        model_path_arg,
        model_type_arg,
        microphone_topic_arg,
        asr_result_topic_arg,
        debug_audio_arg,  # 添加调试参数声明
        asr_node
    ])