#!/usr/bin/env python3
# 项目控制命令行工具
import rclpy
from rclpy.node import Node
import sys
import time
import socket
import struct
import json
import os
import yaml
from enum import IntEnum
from ament_index_python.packages import get_package_share_directory
from custom_msgs.msg import ProjectCommand, ProjectStatus

# 消息类型枚举 (保持与客户端一致)
class MessageType(IntEnum):
    """消息类型枚举"""
    PROJECT_COMMAND = 6
    PROJECT_STATUS = 7

def main(args=None):
    rclpy.init(args=args)
    
    # 创建临时节点
    node = Node('project_control_tool')
    
    if len(sys.argv) < 3:
        node.get_logger().error("Usage: ros2 run udp_server project_control <command> <project>")
        node.get_logger().error("Commands: start, stop, status")
        node.get_logger().error("Projects: camera, elevator, pcl2laser, detection")
        return 1
        
    command = sys.argv[1]
    project = sys.argv[2]
    
    # 使用自定义消息中的枚举
    command_map = {
        "start": ProjectCommand.PROJECT_START,
        "stop": ProjectCommand.PROJECT_STOP,
        "status": ProjectCommand.PROJECT_STATUS
    }
    
    if command not in command_map:
        node.get_logger().error(f"Invalid command: {command}. Valid commands: start, stop, status")
        return 1
        
    if project not in ["camera", "elevator", "pcl2laser", "detection", "cmd_dispatcher"]:
        node.get_logger().error(f"Invalid project: {project}. Valid projects: camera, elevator, pcl2laser, detection", "cmd_dispatcher")
        return 1
        
    # 从配置文件读取客户端地址
    try:
        # 获取包共享目录
        pkg_share_dir = get_package_share_directory('udp_server')
        config_path = os.path.join(pkg_share_dir, 'config', 'udp_server_params.yaml')
        
        # 读取配置文件
        with open(config_path, 'r') as f:
            config_data = yaml.safe_load(f)
            
        # 获取客户端配置
        client_ip = config_data['udp_server']['ros__parameters']['client_ip']
        client_port = config_data['udp_server']['ros__parameters']['client_port']
        
        node.get_logger().info(f"Loaded client config: {client_ip}:{client_port}")
    except Exception as e:
        node.get_logger().error(f"Failed to load config: {str(e)}")
        return 1
        
    # 创建 UDP 套接字（不绑定端口）
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        # 创建命令消息
        command_data = {
            "command_type": command_map[command],
            "project_name": project
        }
        
        # 序列化命令
        json_data = json.dumps(command_data)
        payload = json_data.encode('utf-8')
        
        # 添加消息头 (PROJECT_COMMAND = 6)
        header = struct.pack('!I', MessageType.PROJECT_COMMAND)
        packet = header + payload
        
        # 发送到 Jetson
        udp_socket.sendto(packet, (client_ip, client_port))
        
        # 日志记录
        action = {
            ProjectCommand.PROJECT_START: "Start",
            ProjectCommand.PROJECT_STOP: "Stop",
            ProjectCommand.PROJECT_STATUS: "Status"
        }[command_map[command]]
        node.get_logger().info(f"Sent {action} command for project: {project}")
        
        # 等待响应
        node.get_logger().info("Waiting for status response...")
        start_time = time.time()
        while time.time() - start_time < 5.0:
            try:
                # 设置超时避免永久阻塞
                udp_socket.settimeout(0.1)
                data, addr = udp_socket.recvfrom(1024)
                
                if len(data) >= 4:
                    msg_type = struct.unpack('!I', data[:4])[0]
                    if msg_type == MessageType.PROJECT_STATUS:
                        json_data = data[4:].decode('utf-8')
                        status_data = json.loads(json_data)
                        
                        # 使用自定义消息中的状态枚举
                        status_map = {
                            ProjectStatus.STATUS_STOPPED: "Stopped",
                            ProjectStatus.STATUS_RUNNING: "Running",
                            ProjectStatus.STATUS_STARTING: "Starting",
                            ProjectStatus.STATUS_STOPPING: "Stopping",
                            ProjectStatus.STATUS_ERROR: "Error"
                        }
                        
                        status_name = status_map.get(status_data['status'], "Unknown")
                        node.get_logger().info(f"Project status: {status_data['project_name']} - {status_name}: {status_data['message']}")
                        break
            except socket.timeout:
                continue
            except Exception as e:
                node.get_logger().error(f"Error receiving response: {str(e)}")
    except Exception as e:
        node.get_logger().error(f"Error sending command: {str(e)}")
        return 1
    finally:
        # 关闭套接字
        udp_socket.close()
    
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())
