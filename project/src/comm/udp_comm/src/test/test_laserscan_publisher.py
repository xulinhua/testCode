#!/usr/bin/env python3
"""
测试脚本：ROS 2节点发布LaserScanData数据
用于测试UDP客户端订阅并通过UDP发送到服务端的功能
在客户端运行此脚本
"""

import rclpy
from rclpy.node import Node
import math
from datetime import datetime

# 导入消息类型
from custom_msgs_comm.msg import LaserScanData
from std_msgs.msg import Header


class LaserScanPublisher(Node):
    def __init__(self):
        super().__init__('laserscan_test_publisher')
        
        # 创建激光雷达数据发布者
        self.publisher_ = self.create_publisher(
            LaserScanData,
            '/camera_scan',  # 与UDP客户端订阅的话题一致
            10
        )
        
        # 发布频率（Hz）
        self.publish_rate = 10
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # 发布计数
        self.publish_count = 0
        self.max_count = 50  # 最大发布次数
        
        # 激光雷达参数（模拟真实激光雷达）
        self.angle_min = -math.pi  # 起始角度 -180度
        self.angle_max = math.pi   # 结束角度 180度
        self.angle_increment = math.pi / 180  # 角度增量 1度
        self.range_min = 0.1   # 最小距离 0.1米
        self.range_max = 10.0  # 最大距离 10米
        
        self.get_logger().info('=== LaserScan UDP测试发布节点 ===')
        self.get_logger().info(f'发布话题: /camera_scan')
        self.get_logger().info(f'发布频率: {self.publish_rate} Hz')
        self.get_logger().info(f'计划发布: {self.max_count} 个数据包')
        self.get_logger().info('')
    
    def generate_laserscan_data(self):
        """生成模拟的激光雷达数据"""
        # 创建激光雷达消息
        msg = LaserScanData()
        
        # 设置header
        msg.header = Header()
        msg.header.frame_id = "laser_frame"
        
        # 时间戳 - 使用当前时间
        now = datetime.now()
        msg.header.stamp.sec = int(now.timestamp())
        msg.header.stamp.nanosec = now.microsecond * 1000
        
        # 激光雷达参数
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0  # 时间增量（通常为0）
        msg.scan_time = 1.0 / self.publish_rate  # 扫描时间
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        
        # 计算扫描点数
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        
        # 生成距离数据（模拟障碍物）
        msg.ranges = []
        for i in range(num_readings):
            angle = msg.angle_min + i * msg.angle_increment
            
            # 模拟一个简单的环境：
            # - 前方（0度附近）有一个障碍物
            # - 左右两侧较远
            if abs(angle) < math.pi / 6:  # 前方30度范围内
                # 模拟前方障碍物距离：2-3米
                distance = 2.5 + 0.5 * math.sin(angle * 3)
            else:
                # 其他方向：5-8米
                distance = 6.0 + 2.0 * math.sin(angle * 2)
            
            # 添加一些随机噪声
            import random
            distance += random.uniform(-0.1, 0.1)
            
            # 限制在有效范围内
            distance = max(self.range_min, min(self.range_max, distance))
            msg.ranges.append(distance)
        
        # 生成强度数据（可选）
        msg.intensities = []
        for i in range(num_readings):
            # 模拟强度：距离越近强度越高
            intensity = 1.0 / (msg.ranges[i] + 0.1)
            msg.intensities.append(intensity)
        
        return msg
    
    def timer_callback(self):
        """定时器回调函数，定期发布激光雷达数据"""
        if self.publish_count >= self.max_count:
            self.get_logger().info(f'已完成 {self.max_count} 个数据包的发布')
            self.timer.cancel()
            return
        
        try:
            # 生成激光雷达数据
            msg = self.generate_laserscan_data()
            
            # 发布数据
            self.publisher_.publish(msg)
            
            self.publish_count += 1
            
            # 打印发布信息
            self.get_logger().info(f'[{self.publish_count}/{self.max_count}] 发布激光雷达数据')
            self.get_logger().info(f'  扫描点数: {len(msg.ranges)}')
            self.get_logger().info(f'  角度范围: [{msg.angle_min:.2f}, {msg.angle_max:.2f}] rad')
            self.get_logger().info(f'  距离范围: [{msg.range_min:.2f}, {msg.range_max:.2f}] m')
            self.get_logger().info(f'  坐标系: {msg.header.frame_id}')
            self.get_logger().info(f'  时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
            
            # 打印前5个距离值作为示例
            sample_ranges = msg.ranges[:5]
            self.get_logger().info(f'  前5个距离值: {[f"{r:.2f}" for r in sample_ranges]} m')
            self.get_logger().info('')
            
        except Exception as e:
            self.get_logger().error(f'发布失败: {e}')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        # 创建节点
        publisher = LaserScanPublisher()
        
        # 运行节点
        rclpy.spin(publisher)
        
    except KeyboardInterrupt:
        pass
    finally:
        # 销毁节点
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
