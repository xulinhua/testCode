#!/usr/bin/env python3

"""
FunASR ROS节点
订阅audio_common项目发布的音频话题，调用FunASR进行语音识别，并将结果发布到指定话题
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from audio_common_msgs.msg import AudioStamped
from std_msgs.msg import String
# 添加custom_msgs_comm的导入
from custom_msgs_comm.msg import VoiceCommand
import sys
import os
import numpy as np
import time  # 添加time模块用于生成ID

# 添加funasr_py路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'funasr_py'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'funasr_py', 'asr_core'))
# 添加audio_basic_py路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'audio_basic_py'))

# 导入必要的模块
from asr_core.asr_processor import ASRProcessor
from asr_core.model_manager import ModelManager
# 导入音频工具模块
from asr_core.audio_utils import convert_audio_data, is_valid_result, resample_audio
# 导入音频调试工具模块，修复导入路径问题
try:
    # 首先尝试直接从audio_basic_py.audio_utils导入
    from audio_basic_py.audio_utils import analyze_audio_data, save_audio_data, print_audio_stats
except ImportError:
    # 如果失败，尝试从audio_basic_py.audio_basic_py.audio_utils导入（开发环境）
    try:
        from audio_basic_py.audio_basic_py.audio_utils import analyze_audio_data, save_audio_data, print_audio_stats
    except ImportError:
        # 最后的备选方案
        from audio_basic_py.audio_basic_py.audio_utils import analyze_audio_data, save_audio_data, print_audio_stats

class ASRNode(Node):
    """ASR ROS节点类"""

    def __init__(self):
        """初始化ASR节点"""
        super().__init__('asr_node')
        
        # 声明参数
        self.declare_parameter('microphone_topic', '/audio/microphone')
        self.declare_parameter('asr_result_topic', '/asr/result')
        self.declare_parameter('audio_format', 16)
        self.declare_parameter('audio_channels', 1)
        self.declare_parameter('sample_rate', 16000)  # 修改为16000，与模型要求一致
        self.declare_parameter('chunk_size', 512)  # 修改为512，与音频采集节点一致
        self.declare_parameter('enable_vad', True)
        self.declare_parameter('enable_punctuation', True)
        self.declare_parameter('debug_audio', False)  # 修改默认值为False，让launch文件参数生效

        # 获取参数
        self.microphone_topic = self.get_parameter('microphone_topic').get_parameter_value().string_value
        self.asr_result_topic = self.get_parameter('asr_result_topic').get_parameter_value().string_value
        self.audio_format = self.get_parameter('audio_format').get_parameter_value().integer_value
        self.audio_channels = self.get_parameter('audio_channels').get_parameter_value().integer_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value
        self.enable_vad = self.get_parameter('enable_vad').get_parameter_value().bool_value
        self.enable_punctuation = self.get_parameter('enable_punctuation').get_parameter_value().bool_value
        self.debug_audio = self.get_parameter('debug_audio').get_parameter_value().bool_value  # 获取调试开关参数

        # 打印debug_audio参数值用于调试
        self.get_logger().info(f'Debug audio parameter value: {self.debug_audio}')

        # 初始化ASR处理器和模型管理器
        self.asr_processor = ASRProcessor()
        self.model_manager = ModelManager()
        
        # 初始化流式识别缓存，用于累积音频数据
        self.streaming_cache = {}
        self.audio_buffer = []  # 音频数据缓冲区
        self.is_first_chunk = True  # 标记是否为第一个音频块
        
        # 获取预配置的模型信息
        model_info = self.model_manager.get_model_info()
        if not model_info or not self.asr_processor.load_asr_model(model_info):
            self.get_logger().error('Failed to load ASR model')
            return

        # 创建订阅者，使用与发布者兼容的QoS配置
        sensor_data_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 与SensorDataQoS兼容
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            AudioStamped,
            self.microphone_topic,
            self.audio_callback,
            sensor_data_qos)  # 使用兼容的QoS配置
            
        # 创建发布者
        self.asr_result_publisher = self.create_publisher(
            String,
            self.asr_result_topic,
            10)
            
        # 创建VoiceCommand发布者，发布到/voice_command话题
        self.voice_command_publisher = self.create_publisher(
            VoiceCommand,
            '/voice_command',
            10)

        self.get_logger().info('ASR Node initialized successfully')
        self.get_logger().info(f'Subscribing to: {self.microphone_topic}')
        self.get_logger().info(f'Publishing to: {self.asr_result_topic}')
        self.get_logger().info('ASR node is now listening for audio input...')

    def audio_callback(self, msg):
        """
        音频数据回调函数
        
        Args:
            msg (AudioStamped): 接收到的音频数据消息
        """
        print_info = True
        # 转换音频数据
        audio_data = convert_audio_data(msg, self.audio_format)
        
        # 保存voice_type信息，用于后续发布识别结果
        voice_type = msg.audio.info.voice_type if hasattr(msg.audio.info, 'voice_type') else 3  # 默认为NORMAL_COMMAND
        
        # 添加调试信息
        if print_info:
            self.get_logger().info(f"接收到音频数据块，长度: {len(audio_data)}, 格式: {self.audio_format}, 声道数: {self.audio_channels}, 声音类型: {voice_type}")
        
        if len(audio_data) == 0:
            if print_info:
                self.get_logger().info("音频数据为空，跳过处理")
            return
            
        # 将新音频数据添加到缓冲区
        self.audio_buffer.extend(audio_data)
        
        # 保存voice_type信息到实例变量，以便在发布识别结果时使用
        self.current_voice_type = voice_type
        
        # 添加调试信息
        # if print_info:
        #     self.get_logger().info(f"音频缓冲区当前长度: {len(self.audio_buffer)}")
        
        # 累积足够的音频数据再进行识别（至少0.5秒的数据，16000Hz * 0.5 = 8000个采样点）
        # 但也要防止缓冲区过大（最多2.0秒数据，16000Hz * 2.0 = 32000个采样点）
        buffer_length = len(self.audio_buffer)
        if buffer_length >= 8000:
            # if print_info:
            #     self.get_logger().info(f"开始处理音频数据，缓冲区长度: {buffer_length}")
            
            # 执行流式语音识别
            result = self.perform_asr_streaming(self.audio_buffer, msg.audio.info.rate)  # 使用实际的采样率
            
            # 清空缓冲区
            # if print_info:
                # self.get_logger().info("清空音频缓冲区")
            self.audio_buffer = []
            self.is_first_chunk = True  # 重置第一个块标记
            
            # 只有当结果有效时才发布
            if is_valid_result(result):
                # 发布识别结果，附带voice_type信息
                self.publish_asr_result(result, self.current_voice_type)
            elif print_info:
                self.get_logger().info("识别结果无效，跳过发布")
        else:
            # 缓冲区数据不足，继续累积
            if print_info:
                self.get_logger().info(f"音频缓冲区数据不足({buffer_length})，继续累积音频数据")
                 
    def perform_asr_streaming(self, audio_data, input_sample_rate):
        """
        执行流式语音识别
        
        Args:
            audio_data (list): 音频数据
            input_sample_rate (int): 输入音频的采样率
            
        Returns:
            str: 识别结果
        """
        # 调用ASR处理器中的perform_asr_streaming方法
        return self.asr_processor.perform_asr_streaming(audio_data, input_sample_rate, self.debug_audio)

    def publish_asr_result(self, result, voice_type=3):
        """
        发布ASR结果
        
        Args:
            result (str): 识别结果
            voice_type (int): 声音类型，默认为NORMAL_COMMAND
        """
        if not result:
            return
            
        # 添加局部临时变量控制是否打印语音识别结果输出，默认为True
        print_result = True
            
        # 提取文本内容
        text = ""
        if isinstance(result, dict):
            if 'text' in result and result['text']:
                text = result['text'].strip()
        else:
            text = str(result).strip()
            
        # 确保文本不为空
        if not text:
            if print_result:
                self.get_logger().info('ASR result is empty, skipping publish')
            return
            
        # 发布std_msgs/String类型消息（保持原有功能）
        msg = String()
        msg.data = text
        self.asr_result_publisher.publish(msg)
        # if print_result:
        #     self.get_logger().info(f'Published ASR result (String): {text}')
            
        # 发布custom_msgs_comm/VoiceCommand类型消息
        voice_cmd = VoiceCommand()
        # 生成唯一的语音ID（使用时间戳）
        voice_cmd.voice_id = int(time.time() * 1000000) % 2147483647  # 确保ID在int32范围内
        voice_cmd.txt = text
        voice_cmd.voice_type = voice_type  # 使用传入的voice_type参数
        
        self.voice_command_publisher.publish(voice_cmd)
        if print_result:
            self.get_logger().info(f'Published VoiceCommand: id={voice_cmd.voice_id}, txt="{text}", type={voice_cmd.voice_type}')

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 创建ASR节点
    asr_node = ASRNode()
    
    try:
        # 保持节点运行
        rclpy.spin(asr_node)
    except KeyboardInterrupt:
        asr_node.get_logger().info('ASR node stopped by user')
    finally:
        # 销毁节点并关闭ROS 2
        asr_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()