#!/usr/bin/env python3
"""
Piper TTS ROS节点测试脚本
=====================

该脚本用于测试Piper TTS ROS节点的基本功能。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class TTSNodeTester(Node):
    """TTS节点测试类"""
    
    def __init__(self):
        """初始化测试节点"""
        super().__init__('tts_node_tester')
        
        # 创建发布者
        self.publisher = self.create_publisher(
            String,
            '/voice reply',
            10)
        
        self.get_logger().info('TTS节点测试器初始化完成')
    
    def send_test_message(self, text):
        """
        发送测试消息
        
        参数:
            text (str): 要发送的测试文本
        """
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f'发送测试消息: "{text}"')


def main(args=None):
    """主函数"""
    # 初始化ROS 2
    rclpy.init(args=args)
    
    # 创建测试节点
    tester = TTSNodeTester()
    
    # 发送几个测试消息
    test_messages = [
        "你好，世界！",
        "欢迎使用Piper语音合成系统。",
        "这是一个测试消息。"
    ]
    
    for i, message in enumerate(test_messages):
        tester.send_test_message(message)
        time.sleep(2)  # 等待2秒再发送下一个消息
    
    # 保持节点运行一段时间
    rclpy.spin_once(tester, timeout_sec=5)
    
    # 销毁节点
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()