#!/usr/bin/env python3
"""
测试脚本：ROS 2节点发布PointCloudData数据
用于测试UDP客户端订阅并通过UDP发送到服务端的功能
"""

import rclpy
from rclpy.node import Node
import time
import random
import struct
from datetime import datetime

# 导入消息类型
from custom_msgs_comm.msg import PointCloudData
from std_msgs.msg import Header
from sensor_msgs.msg import PointField

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_test_publisher')
        
        # 创建点云数据发布者
        self.publisher_ = self.create_publisher(
            PointCloudData,
            '/camera/pointcloud',  # 与UDP客户端订阅的话题一致
            10
        )
        
        # 发布频率（Hz）
        self.publish_rate = 2
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # 发布计数
        self.publish_count = 0
        self.max_count = 20  # 最大发布次数
        
        self.get_logger().info('=== PointCloud UDP测试发布节点 ===')
        self.get_logger().info(f'发布话题: /camera/pointcloud')
        self.get_logger().info(f'发布频率: {self.publish_rate} Hz')
        self.get_logger().info(f'计划发布: {self.max_count} 个数据包')
        self.get_logger().info('')
    
    def generate_pointcloud_data(self):
        """生成模拟的点云数据"""
        # 创建点云数据消息
        msg = PointCloudData()
        
        # 基本参数
        msg.height = 1
        msg.width = 1000  # 100个点
        msg.point_step = 32  # 每个点32字节
        msg.row_step = msg.width * msg.point_step  # 行步长
        msg.is_bigendian = False  # 小端格式
        msg.is_dense = True  # 密集点云
        
        # 生成点云数据
        data = bytearray()
        for i in range(msg.width):
            # 生成随机点
            x = random.uniform(-1.0, 1.0)
            y = random.uniform(-1.0, 1.0)
            z = random.uniform(0.0, 2.0)
            intensity = random.uniform(0.0, 1.0)
            
            # 打包为float32
            data.extend(struct.pack('!f', x))
            data.extend(struct.pack('!f', y))
            data.extend(struct.pack('!f', z))
            data.extend(struct.pack('!f', intensity))
            # 添加一些额外数据以达到32字节
            data.extend(b'\x00' * 16)
        
        msg.data = list(data)
        
        # 设置字段信息
        fields = []
        # 添加x, y, z, intensity字段
        fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        fields.append(PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1))
        msg.fields = fields
        
        # 设置header
        msg.header = Header()
        msg.header.frame_id = "camera_depth_optical_frame"
        
        # 时间戳
        now = datetime.now()
        msg.header.stamp.sec = int(now.timestamp())
        msg.header.stamp.nanosec = now.microsecond * 1000
        
        return msg
    
    def timer_callback(self):
        """定时器回调函数，定期发布点云数据"""
        if self.publish_count >= self.max_count:
            self.get_logger().info(f'已完成 {self.max_count} 个数据包的发布')
            self.timer.cancel()
            return
        
        try:
            # 生成点云数据
            msg = self.generate_pointcloud_data()
            
            # 发布数据
            self.publisher_.publish(msg)
            
            self.publish_count += 1
            
            # 打印发布信息
            self.get_logger().info(f'[{self.publish_count}/{self.max_count}] 发布点云数据')
            self.get_logger().info(f'  点数量: {msg.width}')
            self.get_logger().info(f'  数据大小: {len(msg.data)} 字节')
            self.get_logger().info(f'  坐标系: {msg.header.frame_id}')
            self.get_logger().info(f'  时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
            self.get_logger().info(f'  点步长: {msg.point_step} 字节')
            self.get_logger().info(f'  行步长: {msg.row_step} 字节')
            self.get_logger().info('')
            
        except Exception as e:
            self.get_logger().error(f'发布失败: {e}')

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        # 创建节点
        publisher = PointCloudPublisher()
        
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
