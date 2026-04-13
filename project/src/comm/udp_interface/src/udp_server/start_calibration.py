#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

def main(args=None):
    rclpy.init(args=args)
    node = Node('calibration_starter')
    
    # 创建服务客户端
    cli = node.create_client(Trigger, '/calibration/start')
    
    # 等待服务可用
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Calibration start service not available')
        return 1
        
    # 发送请求
    req = Trigger.Request()
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    # 处理响应
    response = future.result()
    if response.success:
        node.get_logger().info('Calibration started successfully')
    else:
        node.get_logger().error(f'Failed to start calibration: {response.message}')
    
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    main()