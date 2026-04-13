#!/usr/bin/env python3
"""
Piper TTS ROS节点
================

该节点订阅语音文本消息，使用Piper引擎进行语音合成，
并将合成的音频数据发布到ROS话题中。

功能特点：
1. 订阅 '/voice_reply' 话题获取文本输入
2. 使用Piper进行语音合成
3. 将音频数据发布到 'audio/speaker' 话题
4. 支持配置是否保存音频文件
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData, AudioStamped, Audio, AudioInfo
import numpy as np
import os
import datetime
import sys

# 添加piper_py到Python路径
# 由于项目结构调整，需要更新路径引用
piper_py_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'piper_py')
sys.path.insert(0, piper_py_path)

# 导入Piper相关模块
try:
    # 首先尝试直接导入（适用于已安装的包）
    from piper_py.tts_engine import TTSEngine
except ImportError as e:
    # 如果直接导入失败，尝试通过修改后的路径导入
    print(f"无法从piper_py导入TTSEngine: {e}")
    try:
        # 尝试从piper_py.piper_py包导入
        from piper_py.piper_py.tts_engine import TTSEngine
    except ImportError as e2:
        print(f"无法从piper_py.piper_py导入TTSEngine: {e2}")
        raise

# 导入音频处理工具
# 注意：这里直接使用speech/src/audio_basic_py/audio_basic_py/audio_utils.py中的接口
try:
    audio_basic_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'audio_basic_py', 'audio_basic_py')
    sys.path.insert(0, audio_basic_path)
    import audio_utils
except ImportError as e:
    # 如果导入失败，使用本地副本
    audio_utils = None
    print(f"警告: 无法导入audio_utils模块: {e}")


class PiperTTSNode(Node):
    """Piper TTS ROS节点类"""
    
    def __init__(self):
        """初始化节点"""
        super().__init__('piper_tts_node')
        
        # 获取包的安装路径
        from ament_index_python.packages import get_package_share_directory
        piper_ros_share_dir = get_package_share_directory('piper_ros')
        
        # 声明参数
        self.declare_parameter('save_audio_to_file', False)
        self.declare_parameter('audio_save_directory', 'resources/audio_data_ros')
        # 使用安装后的模型路径
        default_model_path = os.path.join(piper_ros_share_dir, 'resources', 'models', 'zh_CN', 'medium', 'zh_CN-huayan-medium.onnx')
        default_config_path = os.path.join(piper_ros_share_dir, 'resources', 'models', 'zh_CN', 'medium', 'zh_CN-huayan-medium.onnx.json')
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('config_path', default_config_path)
        self.declare_parameter('sample_rate', 22050)
        self.declare_parameter('speaker_id', 0)
        
        # 获取参数值
        self.save_audio_to_file = self.get_parameter('save_audio_to_file').get_parameter_value().bool_value
        self.audio_save_directory = self.get_parameter('audio_save_directory').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.speaker_id = self.get_parameter('speaker_id').get_parameter_value().integer_value
        
        # 创建订阅者，订阅语音文本消息
        self.subscription = self.create_subscription(
            String,
            '/voice_reply',
            self.text_callback,
            10)
        self.subscription  # 防止未使用变量警告
        
        # 创建发布者，发布音频数据
        self.audio_publisher = self.create_publisher(
            AudioStamped,  # 修改为AudioStamped消息类型
            'audio/speaker',  # 发布的话题名称
            10)
        
        # 初始化Piper TTS引擎
        self.tts_engine = TTSEngine()
        
        # 加载模型
        model_file = model_path
        config_file = config_path
        
        if self.tts_engine.load_model(model_file, config_file):
            self.get_logger().info(f'成功加载Piper模型: {model_file}')
        else:
            self.get_logger().error(f'加载Piper模型失败: {model_file}')
            raise RuntimeError("无法加载Piper模型")
        
        # 确保音频保存目录存在
        if self.save_audio_to_file:
            # 使用相对于项目根目录的路径
            project_root = os.path.join(os.path.dirname(__file__), '..', '..')
            audio_save_dir = os.path.join(project_root, self.audio_save_directory)
            os.makedirs(audio_save_dir, exist_ok=True)
        
        self.get_logger().info('Piper TTS节点初始化完成')
        self.get_logger().info(f'订阅话题: /voice_reply')
        self.get_logger().info(f'发布话题: audio/speaker')
        self.get_logger().info(f'音频保存: {"启用" if self.save_audio_to_file else "禁用"}')
    
    def text_callback(self, msg):
        """
        文本消息回调函数
        
        参数:
            msg (std_msgs.msg.String): 包含待合成文本的消息
        """
        text = msg.data
        self.get_logger().info(f'收到文本消息: "{text}"')
        
        # 使用Piper进行语音合成
        audio_data = self.tts_engine.synthesize(text, self.speaker_id)
        
        if audio_data is not None and len(audio_data) > 0:
            self.get_logger().info(f'语音合成成功，音频长度: {len(audio_data)} 采样点')
            
            # 发布音频数据到ROS话题
            self.publish_audio_data(audio_data)
            
            # 如果启用，保存音频到文件
            if self.save_audio_to_file:
                self.save_audio_to_file_func(audio_data, text)
        else:
            self.get_logger().error('语音合成失败')
    
    def publish_audio_data(self, audio_data):
        """
        发布音频数据到ROS话题
        
        参数:
            audio_data (np.ndarray): 音频数据数组
        """
        # 创建AudioStamped消息
        audio_msg = AudioStamped()
        
        # 设置时间戳
        audio_msg.header.stamp = self.get_clock().now().to_msg()
        audio_msg.header.frame_id = "piper_tts"
        
        # 设置音频数据
        audio_msg.audio.audio_data.int16_data = audio_data.tolist()
        
        # 设置音频信息
        audio_msg.audio.info.format = 8  # paInt16 (修正：paInt16的值是8，不是16)
        audio_msg.audio.info.channels = 1  # 单声道
        audio_msg.audio.info.rate = self.sample_rate
        audio_msg.audio.info.chunk = len(audio_data)
        
        # 发布消息
        self.audio_publisher.publish(audio_msg)
        self.get_logger().info(f'发布音频数据，长度: {len(audio_msg.audio.audio_data.int16_data)}')
    
    def save_audio_to_file_func(self, audio_data, text):
        """
        保存音频数据到文件
        
        参数:
            audio_data (np.ndarray): 音频数据数组
            text (str): 合成的文本（用于生成文件名）
        """
        try:
            # 使用相对于项目根目录的路径
            project_root = os.path.join(os.path.dirname(__file__), '..', '..')
            audio_save_dir = os.path.join(project_root, self.audio_save_directory)
            
            # 生成文件名
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            # 简化文件名，只取前20个字符
            text_part = text[:20].replace(' ', '_').replace('/', '_')
            filename = f"tts_{timestamp}_{text_part}.wav"
            filepath = os.path.join(audio_save_dir, filename)
            
            # 如果audio_utils可用，使用它来保存和分析音频
            if audio_utils is not None:
                # 使用audio_utils保存音频文件
                # 注意：audio_utils.save_audio_data期望float32格式，所以我们需要转换
                audio_float = audio_data.astype(np.float32) / 32768.0
                saved_path = audio_utils.save_audio_data(
                    audio_float, 
                    self.sample_rate, 
                    filepath, 
                    audio_save_dir
                )
                
                self.get_logger().info(f'音频已保存到: {saved_path}')
                
                # 分析音频质量
                stats = audio_utils.analyze_audio_data(audio_float, self.sample_rate)
                audio_utils.print_audio_stats(stats)
            else:
                # 如果audio_utils不可用，使用简单的保存方法
                import wave
                with wave.open(filepath, 'w') as wf:
                    wf.setnchannels(1)  # 单声道
                    wf.setsampwidth(2)  # 16位
                    wf.setframerate(self.sample_rate)
                    wf.writeframes(audio_data.tobytes())
                
                self.get_logger().info(f'音频已保存到: {filepath}')
            
        except Exception as e:
            self.get_logger().error(f'保存音频文件失败: {e}')


def main(args=None):
    """主函数"""
    # 初始化ROS 2
    rclpy.init(args=args)
    
    piper_tts_node = None
    try:
        # 创建节点
        piper_tts_node = PiperTTSNode()
        
        # 保持节点运行
        rclpy.spin(piper_tts_node)
        
    except Exception as e:
        print(f"节点运行出错: {e}")
    
    finally:
        # 销毁节点
        if piper_tts_node is not None:
            piper_tts_node.destroy_node()
        
        # 关闭ROS 2
        rclpy.shutdown()


if __name__ == '__main__':
    main()