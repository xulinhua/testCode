#!/usr/bin/env python3

"""
测试Marker识别结果服务响应的脚本
"""

import rclpy
from rclpy.node import Node
from custom_msgs_comm.srv import GetMarkerDetection
import sys
import time


class MarkerServiceTester(Node):
    def __init__(self):
        super().__init__('Marker_service_tester')
        self.cli = self.create_client(GetMarkerDetection, '/Marker_detection/get_result')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Marker检测服务不可用，等待中...')
        self.req = GetMarkerDetection.Request()

    def send_request(self, request_id):
        self.req.request_id = request_id
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    Marker_tester = MarkerServiceTester()
    
    # 发送测试请求
    request_id = "test_request_" + str(int(time.time()))
    marker_tester.get_logger().info(f'发送Marker检测服务请求，请求ID: {request_id}')
    
    try:
        response = marker_tester.send_request(request_id)
        marker_tester.get_logger().info('收到服务响应:')
        marker_tester.get_logger().info(f'  Success: {response.success}')
        marker_tester.get_logger().info(f'  Message: {response.message}')
        marker_tester.get_logger().info(f'  Object Class: {response.object_class}')
        marker_tester.get_logger().info(f'  Confidence: {response.confidence}')
        marker_tester.get_logger().info(f'  Position: {response.position}')
        marker_tester.get_logger().info(f'  Orientation: {response.orientation}')
        marker_tester.get_logger().info(f'  Frame ID: {response.frame_id}')
        
        # 检查响应是否成功
        if response.success:
            Marker_tester.get_logger().info('✓ 服务调用成功')
            return 0
        else:
            Marker_tester.get_logger().warn('⚠ 服务调用完成但未检测到Marker标记')
            return 1
            
    except Exception as e:
        marker_tester.get_logger().error(f'✗ 服务调用失败: {e}')
        return 2
    finally:
        marker_tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())