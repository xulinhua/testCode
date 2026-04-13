#!/usr/bin/env python3
"""
简单的机械臂关节控制脚本
用法：python3 arm_control.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # 创建发布者，发布到所有关节的位置命令
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/joint_trajectory_controller/joint_trajectory',  # 或者 /arm_controller/joint_trajectory
            10
        )
        
        # 定义关节名称列表（按 URDF 中的顺序）
        self.joint_names = [
            'J1-1_joint', 'J1-2_joint', 'J1-3_joint', 'J1-4_joint', 'J1-5_joint', 'J1-6_joint',
            'J2-1_joint', 'J2-2_joint', 'J2-3_joint', 'J2-4_joint', 'J2-5_joint', 'J2-6_joint',
            'J3-1_joint', 'J3-2_joint', 'J3-3_joint', 'J3-4_joint', 'J3-5_joint', 'J3-6_joint',
            'J4-1_joint', 'J4-2_joint', 'J4-3_joint', 'J4-4_joint', 'J4-5_joint', 'J4-6_joint'
        ]
        
        timer_period = 1.0  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        
    def timer_callback(self):
        # 示例：让 J1-1 关节正弦摆动
        msg = Float64MultiArray()
        positions = [0.0] * len(self.joint_names)
        
        # 只移动第一个关节
        positions[0] = 0.5 * (self.count % 10) * 0.1
        
        msg.data = positions
        self.publisher.publish(msg)
        self.get_logger().info(f'发布关节位置：{positions[0]:.2f}')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
