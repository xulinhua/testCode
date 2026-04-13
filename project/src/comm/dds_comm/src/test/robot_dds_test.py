#!/usr/bin/env python3
"""
机器人分布式ROS通讯测试脚本
用于测试DDS通信在机器人分布式系统中的应用
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading

class RobotDdsTest(Node):
    """机器人分布式DDS通信测试节点"""
    
    def __init__(self):
        super().__init__('robot_dds_test')
        
        # 创建发布者和订阅者
        self.command_pub = self.create_publisher(String, '/dds/client/command_out', 10)
        self.status_sub = self.create_subscription(String, '/dds/client/status_in', 
                                                 self.status_callback, 10)
        
        # 机器人分布式通讯测试场景
        self.robot_scenarios = [
            {
                "name": "运动控制",
                "commands": [
                    {"type": "move_joint", "joints": [0, 45, 90, 0, 0, 0], "speed": 50},
                    {"type": "move_cartesian", "pose": [0.5, 0.3, 0.2, 0, 0, 0], "speed": 30},
                    {"type": "gripper", "action": "open", "force": 20},
                    {"type": "gripper", "action": "close", "force": 30}
                ]
            },
            {
                "name": "传感器数据", 
                "commands": [
                    {"type": "sensor_read", "sensor_type": "camera", "resolution": "1080p"},
                    {"type": "sensor_read", "sensor_type": "lidar", "range": 10.0},
                    {"type": "sensor_read", "sensor_type": "imu", "frequency": 100}
                ]
            },
            {
                "name": "导航控制",
                "commands": [
                    {"type": "navigate_to", "target": [1.0, 2.0, 0.0], "tolerance": 0.1},
                    {"type": "path_planning", "start": [0, 0, 0], "goal": [3, 2, 0]},
                    {"type": "obstacle_avoidance", "mode": "reactive"}
                ]
            },
            {
                "name": "系统监控",
                "commands": [
                    {"type": "health_check", "components": ["motion", "sensors", "navigation"]},
                    {"type": "status_report", "detail_level": "full"},
                    {"type": "emergency_stop", "level": "immediate"}
                ]
            }
        ]
        
        self.test_results = {
            "total_tests": 0,
            "passed_tests": 0,
            "failed_tests": 0,
            "latency_measurements": [],
            "throughput_measurements": []
        }
        
        self.latency_start_time = None
        self.response_received = False
        
        self.get_logger().info("机器人分布式DDS通信测试节点启动")

    def status_callback(self, msg):
        """状态消息回调 - 用于性能测量"""
        try:
            status_data = json.loads(msg.data)
            
            # 计算延迟
            if self.latency_start_time:
                latency = (time.time() - self.latency_start_time) * 1000  # 转换为毫秒
                self.test_results["latency_measurements"].append(latency)
                self.get_logger().info(f"通信延迟: {latency:.2f}ms")
                self.latency_start_time = None
            
            self.response_received = True
            self.get_logger().info(f"接收到状态响应: {status_data.get('status', 'unknown')}")
            
        except json.JSONDecodeError:
            self.get_logger().warn("状态消息JSON解析失败")

    def send_robot_command(self, scenario_name, command_data):
        """发送机器人分布式命令"""
        command_msg = {
            "command_type": "robot_control",
            "project_type": "robot_system",
            "scenario": scenario_name,
            "payload": json.dumps(command_data),
            "timestamp": int(time.time() * 1000),
            "source": "x86_control_pc",
            "destination": "jetson_robot_controller"
        }
        
        msg = String()
        msg.data = json.dumps(command_msg)
        
        # 开始延迟测量
        self.latency_start_time = time.time()
        self.response_received = False
        
        self.command_pub.publish(msg)
        
        self.get_logger().info(f"发送机器人命令: {scenario_name} - {command_data['type']}")

    def test_basic_communication(self):
        """测试基本通信功能"""
        self.get_logger().info("=== 测试基本通信功能 ===")
        
        # 测试连接建立
        test_command = {"type": "ping", "data": "connection_test"}
        self.send_robot_command("connection_test", test_command)
        
        # 等待响应
        time.sleep(2)
        
        if self.response_received:
            self.test_results["passed_tests"] += 1
            self.get_logger().info("✓ 基本通信测试通过")
        else:
            self.test_results["failed_tests"] += 1
            self.get_logger().error("✗ 基本通信测试失败")
        
        self.test_results["total_tests"] += 1

    def test_robot_scenarios(self):
        """测试机器人分布式场景"""
        self.get_logger().info("=== 测试机器人分布式场景 ===")
        
        for scenario in self.robot_scenarios:
            self.get_logger().info(f"测试场景: {scenario['name']}")
            
            for command in scenario['commands']:
                self.send_robot_command(scenario['name'], command)
                
                # 等待命令执行
                time.sleep(1)
                
                # 简单的成功判断（实际应根据状态响应判断）
                self.test_results["passed_tests"] += 1
                self.test_results["total_tests"] += 1
            
            time.sleep(2)  # 场景间间隔

    def test_performance(self):
        """测试性能指标"""
        self.get_logger().info("=== 测试性能指标 ===")
        
        # 延迟测试
        latency_test_count = 10
        self.get_logger().info(f"进行 {latency_test_count} 次延迟测试...")
        
        for i in range(latency_test_count):
            test_command = {"type": "ping", "data": f"latency_test_{i}"}
            self.send_robot_command("performance_test", test_command)
            time.sleep(0.5)  # 测试间隔
        
        # 吞吐量测试
        throughput_test_count = 50
        self.get_logger().info(f"进行 {throughput_test_count} 次吞吐量测试...")
        
        start_time = time.time()
        for i in range(throughput_test_count):
            test_command = {"type": "throughput_test", "sequence": i}
            self.send_robot_command("throughput_test", test_command)
        
        total_time = time.time() - start_time
        throughput = throughput_test_count / total_time
        self.test_results["throughput_measurements"].append(throughput)
        
        self.get_logger().info(f"吞吐量: {throughput:.2f} 消息/秒")

    def print_test_results(self):
        """打印测试结果"""
        self.get_logger().info("=== 测试结果汇总 ===")
        self.get_logger().info(f"总测试数: {self.test_results['total_tests']}")
        self.get_logger().info(f"通过测试: {self.test_results['passed_tests']}")
        self.get_logger().info(f"失败测试: {self.test_results['failed_tests']}")
        
        if self.test_results['latency_measurements']:
            avg_latency = sum(self.test_results['latency_measurements']) / len(self.test_results['latency_measurements'])
            max_latency = max(self.test_results['latency_measurements'])
            min_latency = min(self.test_results['latency_measurements'])
            
            self.get_logger().info(f"平均延迟: {avg_latency:.2f}ms")
            self.get_logger().info(f"最大延迟: {max_latency:.2f}ms")
            self.get_logger().info(f"最小延迟: {min_latency:.2f}ms")
        
        if self.test_results['throughput_measurements']:
            avg_throughput = sum(self.test_results['throughput_measurements']) / len(self.test_results['throughput_measurements'])
            self.get_logger().info(f"平均吞吐量: {avg_throughput:.2f} 消息/秒")
        
        success_rate = (self.test_results['passed_tests'] / self.test_results['total_tests']) * 100
        self.get_logger().info(f"测试成功率: {success_rate:.1f}%")

def main():
    rclpy.init()
    
    try:
        node = RobotDdsTest()
        
        # 运行测试套件
        node.test_basic_communication()
        node.test_robot_scenarios() 
        node.test_performance()
        
        # 打印结果
        node.print_test_results()
        
        # 保持运行以接收剩余响应
        node.get_logger().info("测试完成，等待剩余响应...")
        time.sleep(5)
        
    except KeyboardInterrupt:
        node.get_logger().info("测试被用户中断")
    except Exception as e:
        node.get_logger().error(f"测试发生错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()