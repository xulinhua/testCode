#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
音频订阅节点
============

该节点订阅来自其他模块（如语音识别处理模块）的音频数据话题，
并将接收到的音频数据通过扬声器设备播放出来。

功能：
- 订阅ROS话题中的音频数据
- 实时播放接收到的音频数据到扬声器设备
- 支持动态配置扬声器设备索引和播放参数
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

class AudioSubscriber(Node):
    """
    音频订阅节点类
    
    该类继承自rclpy.Node，负责订阅ROS话题中的音频数据并通过扬声器播放。
    """
    
    def __init__(self):
        """
        初始化音频订阅节点
        """
        super().__init__('audio_subscriber')
        
        # 声明参数
        self.declare_parameter('speaker_device_index', 25)  # 默认USB扬声器索引
        self.declare_parameter('sample_rate', 44100)        # 默认采样率
        self.declare_parameter('channels', 1)               # 默认单声道
        
        # 获取参数
        self.speaker_device_index = self.get_parameter('speaker_device_index').get_parameter_value().integer_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        
        # 初始化音频管理器
        try:
            self.audio_manager = AudioIOManager(output_device_index=self.speaker_device_index)
            self.get_logger().info(f'音频管理器初始化成功，使用扬声器设备索引: {self.speaker_device_index}')
        except Exception as e:
            self.get_logger().error(f'音频管理器初始化失败: {e}')
            raise
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'audio_data',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告
        
        self.get_logger().info('音频订阅节点已启动')
    
    def listener_callback(self, msg):
        """
        订阅回调函数
        
        该函数在接收到音频数据话题消息时被调用，将音频数据通过扬声器播放。
        
        :param msg: 接收到的UInt8MultiArray消息，包含音频数据
        """
        try:
            # 获取音频数据
            audio_data = bytes(msg.data)
            
            # 播放音频数据
            self.audio_manager.play_audio_data(
                audio_data=audio_data,
                rate=self.sample_rate,
                channels=self.channels
            )
            
            # 记录日志
            self.get_logger().info(f'播放音频数据，大小: {len(audio_data)} 字节')
            
        except Exception as e:
            self.get_logger().error(f'播放音频数据时出错: {e}')
            # 尝试重新初始化音频管理器
            try:
                self.get_logger().info('尝试重新初始化音频管理器...')
                if hasattr(self, 'audio_manager'):
                    try:
                        self.audio_manager.close()
                    except:
                        pass
                self.audio_manager = AudioIOManager(output_device_index=self.speaker_device_index)
                self.get_logger().info('音频管理器重新初始化成功')
            except Exception as e2:
                self.get_logger().error(f'重新初始化音频管理器失败: {e2}')

def main(args=None):
    """
    主函数
    
    初始化ROS节点并运行音频订阅器。
    """
    rclpy.init(args=args)
    
    audio_subscriber = None
    try:
        audio_subscriber = AudioSubscriber()
        rclpy.spin(audio_subscriber)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行出错: {e}")
    finally:
        # 销毁节点并关闭ROS
        if audio_subscriber is not None:
            audio_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()