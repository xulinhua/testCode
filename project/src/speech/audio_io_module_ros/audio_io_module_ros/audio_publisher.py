#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
音频发布节点
============

该节点从麦克风设备实时获取音频数据，并将其发布到ROS话题中供其他模块使用，
例如语音识别处理模块（ASR）。

功能：
- 实时从麦克风采集音频数据
- 将音频数据以sensor_msgs/msg/Image格式发布到ROS话题
- 支持动态配置麦克风设备索引和采样参数
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import sys
import os

# 添加audio_io_module到Python路径 - 使用相对于ROS包的路径
# 在ROS环境中，我们需要使用相对于包安装位置的路径
try:
    # 首先尝试直接导入（在安装后应该可以找到）
    from audio_io_manager import AudioIOManager
except ImportError:
    # 如果直接导入失败，尝试使用相对路径
    try:
        # 获取当前文件的目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # 构建到audio_io_module的路径（在同一个src目录下）
        audio_io_module_path = os.path.join(current_dir, '..', '..', '..', 'audio_io_module')
        if os.path.exists(audio_io_module_path):
            sys.path.insert(0, audio_io_module_path)
            from audio_io_manager import AudioIOManager
        else:
            # 尝试在安装目录中查找
            install_dir = os.path.join(current_dir, '..', '..', '..', 'install', 'audio_io_module')
            # 检查lib/python3.10/site-packages目录
            site_packages_dir = os.path.join(install_dir, 'lib', 'python3.10', 'site-packages')
            if os.path.exists(site_packages_dir):
                sys.path.append(site_packages_dir)
            
            from audio_io_manager import AudioIOManager
    except ImportError as e:
        print(f"无法导入audio_io_module: {e}")
        print("请确保audio_io_module目录在正确位置")
        sys.exit(1)

class AudioPublisher(Node):
    """
    音频发布节点类
    
    该类继承自rclpy.Node，负责从麦克风采集音频数据并发布到ROS话题。
    """
    
    def __init__(self):
        """
        初始化音频发布节点
        """
        super().__init__('audio_publisher')
        
        # 声明参数
        self.declare_parameter('mic_device_index', 24)  # 默认USB麦克风索引
        self.declare_parameter('sample_rate', 44100)    # 默认采样率
        self.declare_parameter('channels', 1)           # 默认单声道
        self.declare_parameter('chunk_size', 1024)     # 默认块大小
        
        # 获取参数
        self.mic_device_index = self.get_parameter('mic_device_index').get_parameter_value().integer_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value
        
        # 创建发布者
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'audio_data', 10)
        
        # 初始化音频管理器
        try:
            self.audio_manager = AudioIOManager(
                input_device_index=self.mic_device_index
            )
            self.get_logger().info(f'音频管理器初始化成功，使用麦克风设备索引: {self.mic_device_index}')
        except Exception as e:
            self.get_logger().error(f'音频管理器初始化失败: {e}')
            raise
        
        # 启动音频采集
        self.start_audio_capture()
        
        self.get_logger().info('音频发布节点已启动')
    
    def start_audio_capture(self):
        """
        启动音频采集
        
        使用audio_io_manager的capture_audio_stream方法实时获取音频数据并发布。
        """
        try:
            def audio_callback(data):
                try:
                    # 创建UInt8MultiArray消息
                    msg = UInt8MultiArray()
                    msg.data = list(data)  # 将bytes转换为list of uint8
                    
                    # 发布消息
                    self.publisher_.publish(msg)
                    
                    # 记录日志（改为INFO级别以便观察）
                    self.get_logger().info(f'发布音频数据，大小: {len(data)} 字节')
                except Exception as e:
                    self.get_logger().error(f'发布音频数据时出错: {e}')
            
            # 启动实时音频采集（在新线程中运行以避免阻塞）
            import threading
            capture_thread = threading.Thread(
                target=self.audio_manager.capture_audio_stream,
                kwargs={
                    'chunk': self.chunk_size,
                    'channels': self.channels,
                    'rate': self.sample_rate,
                    'callback': audio_callback
                },
                daemon=True
            )
            capture_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'启动音频采集时出错: {e}')
            # 尝试重新初始化音频管理器
            try:
                self.get_logger().info('尝试重新初始化音频管理器...')
                if hasattr(self, 'audio_manager'):
                    try:
                        self.audio_manager.close()
                    except:
                        pass
                self.audio_manager = AudioIOManager(input_device_index=self.mic_device_index)
                self.get_logger().info('音频管理器重新初始化成功')
                
                # 重新启动音频采集
                import threading
                capture_thread = threading.Thread(
                    target=self.audio_manager.capture_audio_stream,
                    kwargs={
                        'chunk': self.chunk_size,
                        'channels': self.channels,
                        'rate': self.sample_rate,
                        'callback': lambda data: self._safe_publish_audio_data(data)
                    },
                    daemon=True
                )
                capture_thread.start()
            except Exception as e2:
                self.get_logger().error(f'重新初始化音频管理器失败: {e2}')
    
    def _safe_publish_audio_data(self, data):
        """安全发布音频数据的方法"""
        try:
            # 创建UInt8MultiArray消息
            msg = UInt8MultiArray()
            msg.data = list(data)  # 将bytes转换为list of uint8
            
            # 发布消息
            self.publisher_.publish(msg)
            
            # 记录日志
            self.get_logger().info(f'发布音频数据，大小: {len(data)} 字节')
        except Exception as e:
            self.get_logger().error(f'发布音频数据时出错: {e}')
def main(args=None):
    """
    主函数
    
    初始化ROS节点并运行音频发布器。
    """
    rclpy.init(args=args)
    
    audio_publisher = None
    try:
        audio_publisher = AudioPublisher()
        rclpy.spin(audio_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行出错: {e}")
    finally:
        # 销毁节点并关闭ROS
        if audio_publisher is not None:
            audio_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()