#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TTSTestNode(Node):
    def __init__(self):
        super().__init__('tts_test_node')
        self.publisher_ = self.create_publisher(String, '/voice_reply', 10)
        timer_period = 5.0  # 每5秒发送一次
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
        self.test_messages = [
            "我在",
            "你好世界",
            "测试语音合成",
            "欢迎使用音频系统"
        ]
        
    def timer_callback(self):
        msg = String()
        msg.data = self.test_messages[self.counter % len(self.test_messages)]
        self.publisher_.publish(msg)
        self.get_logger().info(f'发送文本消息: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TTSTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()