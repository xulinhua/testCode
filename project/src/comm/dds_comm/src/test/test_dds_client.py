#!/usr/bin/env python3

"""
DDS客户端测试脚本
用于测试DDS客户端功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class DDSClientTestNode(Node):
    """DDS客户端测试节点"""
    
    def __init__(self):
        super().__init__('dds_client_test_node')
        
        # 创建发布者和订阅者
        self.command_pub = self.create_publisher(String, '/dds/client/command_out', 10)
        self.status_sub = self.create_subscription(String, '/dds/client/status_in', 
                                                 self.status_callback, 10)
        
        # 测试项目列表
        self.test_projects = [
            "hand_eye_calib",
            "camera", 
            "pcl2laserscan",
            "yolo_det"
        ]
        
        self.current_project_index = 0
        self.current_command = "start"
        
        # 创建定时器
        self.timer = self.create_timer(5.0, self.test_timer_callback)
        
        self.get_logger().info("DDS客户端测试节点启动")
    
    def status_callback(self, msg):
        """状态消息回调"""
        try:
            status_data = json.loads(msg.data)
            self.get_logger().info(f"接收到状态响应: 项目={status_data.get('project_type', 'unknown')}, "
                                 f"状态={status_data.get('status', 'unknown')}")
        except json.JSONDecodeError:
            self.get_logger().warn("状态消息JSON解析失败")
    
    def test_timer_callback(self):
        """定时器回调，发送测试命令"""
        if self.current_project_index >= len(self.test_projects):
            self.get_logger().info("所有项目测试完成")
            self.timer.cancel()
            return
        
        # 获取当前测试项目
        project = self.test_projects[self.current_project_index]
        
        # 发送命令
        self.send_command(project, self.current_command)
        
        # 切换命令类型
        if self.current_command == "start":
            self.current_command = "status"
        elif self.current_command == "status":
            self.current_command = "stop"
        else:
            # 切换到下一个项目
            self.current_command = "start"
            self.current_project_index += 1
    
    def send_command(self, project_type, command_type):
        """发送命令"""
        command_msg = {
            "command_type": command_type,
            "project_type": project_type,
            "payload": "{}",
            "timestamp": int(time.time() * 1000),
            "source": "dds_client_test",
            "destination": "dds_server"
        }
        
        # 发布命令
        msg = String()
        msg.data = json.dumps(command_msg)
        self.command_pub.publish(msg)
        
        self.get_logger().info(f"发送命令: 项目={project_type}, 命令={command_type}")

def main():
    rclpy.init()
    
    try:
        node = DDSClientTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()