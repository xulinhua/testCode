#!/usr/bin/env python3
"""
测试脚本：ROS 2节点订阅LaserScanData数据
用于验证UDP服务端接收并发布的激光雷达数据
在服务端运行此脚本
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# 导入消息类型
from custom_msgs_comm.msg import LaserScanData


class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laserscan_test_subscriber')
        
        # 设置QoS策略
        qos_profile = QoSProfile(depth=10)
        
        # 创建激光雷达数据订阅者
        self.subscription_ = self.create_subscription(
            LaserScanData,
            '/jetson/laserscan',  # UDP服务端发布的话题
            self.laserscan_callback,
            qos_profile
        )
        
        # 接收计数
        self.receive_count = 0
        self.max_count = 50  # 最大接收次数
        
        self.get_logger().info('=== LaserScan UDP测试订阅节点 ===')
        self.get_logger().info(f'订阅话题: /jetson/laserscan')
        self.get_logger().info(f'等待接收数据...')
        self.get_logger().info('')
    
    def laserscan_callback(self, msg):
        """接收到激光雷达数据的回调函数"""
        self.receive_count += 1
        
        # 打印接收信息
        self.get_logger().info(f'[{self.receive_count}] 接收到激光雷达数据')
        self.get_logger().info(f'  扫描点数: {len(msg.ranges)}')
        self.get_logger().info(f'  角度范围: [{msg.angle_min:.2f}, {msg.angle_max:.2f}] rad')
        self.get_logger().info(f'  角度增量: {msg.angle_increment:.4f} rad')
        self.get_logger().info(f'  距离范围: [{msg.range_min:.2f}, {msg.range_max:.2f}] m')
        self.get_logger().info(f'  扫描时间: {msg.scan_time:.4f} s')
        self.get_logger().info(f'  坐标系: {msg.header.frame_id}')
        self.get_logger().info(f'  时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        
        # 打印距离统计
        if len(msg.ranges) > 0:
            valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
            if valid_ranges:
                min_range = min(valid_ranges)
                max_range = max(valid_ranges)
                avg_range = sum(valid_ranges) / len(valid_ranges)
                self.get_logger().info(f'  距离统计: min={min_range:.2f}, max={max_range:.2f}, avg={avg_range:.2f} m')
        
        # 打印前5个距离值作为示例
        if len(msg.ranges) >= 5:
            sample_ranges = msg.ranges[:5]
            self.get_logger().info(f'  前5个距离值: {[f"{r:.2f}" for r in sample_ranges]} m')
        
        # 打印强度信息（如果有）
        if len(msg.intensities) > 0:
            self.get_logger().info(f'  强度数据点数: {len(msg.intensities)}')
            if len(msg.intensities) >= 5:
                sample_intensities = msg.intensities[:5]
                self.get_logger().info(f'  前5个强度值: {[f"{i:.3f}" for i in sample_intensities]}')
        
        self.get_logger().info('')
        
        # 达到最大接收次数后停止
        if self.receive_count >= self.max_count:
            self.get_logger().info(f'已完成 {self.max_count} 个数据包的接收')
            raise KeyboardInterrupt


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    subscriber = None
    try:
        # 创建节点
        subscriber = LaserScanSubscriber()
        
        # 运行节点
        rclpy.spin(subscriber)
        
    except KeyboardInterrupt:
        pass
    finally:
        # 销毁节点
        if subscriber is not None:
            subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
