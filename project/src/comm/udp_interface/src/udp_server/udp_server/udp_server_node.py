#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import threading
import struct
import json
import queue  # 添加队列模块
import time
import random  # 添加随机数生成
from enum import IntEnum
from std_srvs.srv import Trigger
from custom_msgs.msg import PointCloudData, AICoordinateData, LaserScanData, CalibrationCommand, CalibrationResponse
from custom_msgs.msg import ProjectCommand, ProjectStatus
from custom_msgs.srv import ProjectStatus as ProjectStatusSrv  # 导入服务类型
from custom_msgs.srv import ProjectControl as ProjectControlSrv

from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time
from std_msgs.msg import String  # 用于发送指令
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose  # 检测结果消息
from custom_msgs_comm.msg import InteractiveCommand  # 交互指令消息

class MessageType(IntEnum):
    """消息类型枚举"""
    COMMAND = 0  #命令消息：服务器端发送给客户端
    AI_COORDINATE = 1
    LASERSCAN = 2
    POINTCLOUD = 3
    CALIBRATION_COMMAND = 4  
    CALIBRATION_RESPONSE = 5
    PROJECT_COMMAND = 6
    PROJECT_STATUS = 7
    DETECTION_IMAGE = 8  # 检测图像数据
    DETECTION_RESULT = 9  # 检测结果数据
    INTERACTIVE_COMMAND = 10  # 交互指令数据

class UDPServerNode(Node):
    def __init__(self):
        super().__init__('udp_server')
        
        # 声明参数
        self.declare_parameter('udp_port', 8888)
        self.declare_parameter('udp_buffer_size', 65507)

        # 添加客户端IP和端口参数
        self.declare_parameter('client_ip', '192.168.10.61') # Jetson默认IP
        self.declare_parameter('client_port', 8889)  # 客户端命令接收端口 
 
        # 添加参数控制运行时的调试信息是否输出
        self.declare_parameter('bShowRunInfo', True)  # 控制是否显示运行时的调试信息

        #添加控制发送指令频率的参数，避免网络拥塞或客户端过载：指令在队列中积压（延迟增加）;UDP 丢包率上升（影响可靠性）
        self.declare_parameter('command_max_rate', 20)  # 默认20Hz，最大发送频率：20条/秒
        self.declare_parameter('log_interval', 10)  # 添加日志频率控制间隔参数（每10帧输出1次）

        # 获取参数
        udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.buffer_size = self.get_parameter('udp_buffer_size').get_parameter_value().integer_value
        
        # 获取客户端地址
        self.client_ip = self.get_parameter('client_ip').get_parameter_value().string_value
        self.client_port = self.get_parameter('client_port').get_parameter_value().integer_value

        self.bShowRunInfo = self.get_parameter('bShowRunInfo').get_parameter_value().bool_value
        self.command_max_rate = self.get_parameter('command_max_rate').get_parameter_value().integer_value
        self.log_interval = self.get_parameter('log_interval').get_parameter_value().integer_value  # 日志频率控制间隔参数
		# 日志计数器
        self.log_counter = 0

        # 标定控制状态
        self.calibration_active = False
        self.current_calibration_point = 0
        self.max_calibration_points = 100

        # 添加项目管理器
        # from udp_server.project_control import ProjectManager
        # self.project_manager = ProjectManager(self.get_logger())

        # 项目状态跟踪
        self.project_status = {}

        # 创建UDP套接字
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 仅当作为主服务器时才绑定端口
        self.udp_socket.bind(('0.0.0.0', udp_port))
        self.udp_socket.settimeout(0.1)
        
        # 创建指令队列
        self.command_queue = queue.Queue()
        
        # 创建指令订阅者
        self.command_sub = self.create_subscription(
            String,
            '/udp_server/command',
            self.command_callback,
            10
        )

        # 标定指令发布者
        self.calibration_pub = self.create_publisher(
            CalibrationCommand,  # 使用新定义的消息类型
            '/calibration/command', 
            10
        )

        # 创建ROS2发布者
        self.command_pub = self.create_publisher(String, '/udp_server/command', 10)  # 指令发布者（用于接收外部指令）
        self.ai_coordinate_pub = self.create_publisher(AICoordinateData, '/jetson/ai_coordinates', 10)
        self.laserscan_pub = self.create_publisher(LaserScan, '/jetson/camera_scan', 10)
        self.pointcloud_pub = self.create_publisher(PointCloudData, '/jetson/pointcloud', 10)
        
        # 新增发布者：检测图像、检测结果、交互指令
        self.detection_image_pub = self.create_publisher(String, '/jetson/det_image_png', 10)
        self.detection_result_pub = self.create_publisher(Detection2DArray, '/jetson/det_res', 10)
        self.interactive_command_pub = self.create_publisher(InteractiveCommand, '/jetson/interactive_command', 10) 

        # 标定响应订阅者
        self.calibration_sub = self.create_subscription(
            CalibrationResponse,  # 使用新定义的消息类型
            '/calibration/response',
            self.calibration_response_callback,
            10
        )

        # 启动接收线程
        self.is_running = True
        self.receive_thread = threading.Thread(target=self.receive_loop)
        self.receive_thread.start()
        
        # 启动指令发送线程
        self.command_thread = threading.Thread(target=self.command_listener)
        self.command_thread.start()

        # 启动标定服务
        self.start_calibration_srv = self.create_service(
            Trigger,
            '/calibration/start',
            self.start_calibration_callback
        )
        if self.start_calibration_srv:
            self.get_logger().info('Calibration service created successfully')
        else:
            self.get_logger().error('Failed to create calibration service')

        #增加ROS服务接口：​​添加项目状态查询服务、添加项目控制服务
        # 项目状态服务
        self.project_status_srv = self.create_service(
            ProjectStatusSrv,  # 使用服务类型
            '/project/status',
            self.handle_project_status_request
        )
        
        # 项目控制服务
        self.project_control_srv = self.create_service(
            ProjectControlSrv,  # 使用服务类型
            '/project/control',
            self.handle_project_control
        )

        # 根据bShowRunInfo控制运行时的信息是否输出 
        # if self.bShowRunInfo:
        self.get_logger().info(f'UDP Server started on port {udp_port}')
        self.get_logger().info(f'Command target: {self.client_ip}:{self.client_port}')
        self.get_logger().info(f'Log interval: every {self.log_interval} messages')
    
    def handle_project_status_request(self, request, response):
        """处理项目状态请求"""
        try:
            project_name = request.project_name
            status, message, description = self.project_manager.get_status(project_name)
            
            response.status = status
            response.message = message
            return response
        except Exception as e:
            self.get_logger().error(f"Error handling status request: {str(e)}")
            response.status = ProjectStatus.STATUS_ERROR
            response.message = f"Error: {str(e)}"
            return response
    
    # 处理项目控制请求
    def handle_project_control(self, request, response):
        """处理项目控制请求"""
        try:
            project_name = request.project_name
            command_type = request.command_type
            
            if command_type == ProjectCommand.PROJECT_START:
                success, message = self.project_manager.start_project(project_name)
            elif command_type == ProjectCommand.PROJECT_STOP:
                success, message = self.project_manager.stop_project(project_name)
            else:
                success = False
                message = f"Invalid command type: {command_type}"
                
            response.success = success
            response.message = message
            return response
        except Exception as e:
            self.get_logger().error(f"Error handling control request: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    # 处理项目状态响应
    def handle_project_status(self, status_msg):
        """处理项目状态响应"""
        try:
            #project_name = status_data['project_name']
            #status = status_data['status']
            #message = status_data['message']
            project_name = status_msg.project_name
            status = status_msg.status
            message = status_msg.message

            # 更新状态
            self.project_status[project_name] = {
                "status": status,
                "message": message,
                "timestamp": time.time()
            }
            
            # 使用自定义消息中的状态枚举
            status_map = {
                ProjectStatus.STATUS_STOPPED: "Stopped",
                ProjectStatus.STATUS_RUNNING: "Running",
                ProjectStatus.STATUS_STARTING: "Starting",
                ProjectStatus.STATUS_STOPPING: "Stopping",
                ProjectStatus.STATUS_ERROR: "Error"
            }
            
            status_name = status_map.get(status, "Unknown")
            self.get_logger().info(
                f"Project status: {project_name} - {status_name}: {message}"
            )
        except Exception as e:
            self.get_logger().error(f"Error handling project status: {str(e)}")
    
    # 发送项目命令
    def send_project_command(self, command_type, project_name):
        """发送项目控制命令"""
        try:
            # 创建命令消息
            command_data = {
                "command_type": command_type,
                "project_name": project_name
            }
            
            json_data = json.dumps(command_data)
            payload = json_data.encode('utf-8')
            header = struct.pack('!I', MessageType.PROJECT_COMMAND)
            packet = header + payload
            
            # 添加调试信息
            if self.bShowRunInfo:
                self.get_logger().info(f"Sending PROJECT_COMMAND: {json_data}")

            # 发送到Jetson
            self.udp_socket.sendto(packet, (self.client_ip, self.client_port))
            
            # 使用自定义消息中的命令枚举
            action_map = {
                ProjectCommand.PROJECT_START: "Start",
                ProjectCommand.PROJECT_STOP: "Stop",
                ProjectCommand.PROJECT_STATUS: "Status"
            }
            
            action = action_map.get(command_type, "Unknown")
            self.get_logger().info(f"Sent {action} command for project: {project_name}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending project command: {str(e)}")
            return False

    def receive_loop(self):
        """循环接收UDP数据"""
        while self.is_running and rclpy.ok():
            try:
                data, addr = self.udp_socket.recvfrom(self.buffer_size)
                self.process_data(data, addr)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'Error receiving data: {str(e)}')

    # ======== process_data处理响应 ========
    def process_data(self, data, addr):
        """处理接收到的数据"""
        try:
            if len(data) < 4:
                if self.bShowRunInfo:
                    self.get_logger().warn(f'Received incomplete data from {addr}')
                return
            
            msg_type = struct.unpack('!I', data[:4])[0]
            payload = data[4:]
            if msg_type == MessageType.PROJECT_STATUS: # 项目状态处理
                try:
                    json_data = json.loads(data[4:].decode('utf-8'))
                    self.get_logger().info(f"Received project status: {json_data}")
                    # 创建消息对象并处理
                    status_msg = ProjectStatus()
                    status_msg.status = json_data["status"]
                    status_msg.project_name = json_data["project_name"]
                    status_msg.message = json_data["message"]
                    self.handle_project_status(status_msg) # 传递消息对象
                except Exception as e:
                    self.get_logger().error(f"Error processing project status: {str(e)}")
            elif msg_type == MessageType.CALIBRATION_RESPONSE:
                self.process_calibration_response(payload, addr)
            else:
                if msg_type == MessageType.COMMAND:# 处理指令响应
                    response = payload.decode('utf-8')
                    if self.bShowRunInfo:
                        self.get_logger().info(f'Received response from {addr}: {response}')
                elif msg_type == MessageType.AI_COORDINATE:
                    self.process_ai_coordinate_data(payload, addr)
                elif msg_type == MessageType.LASERSCAN:
                    self.process_laserscan_data(payload, addr)
                elif msg_type == MessageType.POINTCLOUD:
                    self.process_pointcloud_data(payload, addr)
                elif msg_type == MessageType.DETECTION_IMAGE:
                    self.process_detection_image_data(payload, addr)
                elif msg_type == MessageType.DETECTION_RESULT:
                    self.process_detection_result_data(payload, addr)
                elif msg_type == MessageType.INTERACTIVE_COMMAND:
                    self.process_interactive_command_data(payload, addr)
                else:
                    if self.bShowRunInfo:
                        self.get_logger().warn(f'Unknown message type: {msg_type} from {addr}')

        except Exception as e:
            self.get_logger().error(f'Error processing data from {addr}: {str(e)}')
    
    def process_calibration_response(self, data, addr):
        """处理UDP标定响应"""
        try:
            # 解析响应: response_type(1B) + aruco_pixel(2 * 4B) + robot_pose(6 * 4B)
            response_type = struct.unpack('!B', data[:1])[0]

            # 标定完成响应处理
            if response_type in [CalibrationResponse.RES_COMPLETED_SUCCESS, CalibrationResponse.RES_COMPLETED_FAILED]:
                # 结束标定流程
                self.finish_calibration()
                
                # 根据结果记录日志
                if response_type == CalibrationResponse.RES_COMPLETED_SUCCESS:
                    self.get_logger().info('Calibration completed successfully')
                else:
                    self.get_logger().error('Calibration completed with errors')
                return

            # 常规响应处理
            aruco_pixel = struct.unpack('!2f', data[1:9])
            robot_pose = struct.unpack('!6f', data[9:33])
            
            # 创建ROS消息
            msg = CalibrationResponse()
            msg.response_type = response_type
            msg.aruco_pixel_pose = list(aruco_pixel)
            msg.robot_pose = list(robot_pose)
            
            # 发布到ROS话题
            self.calibration_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing calibration response: {str(e)}')

    # ======== 标定服务回调 ========
    def start_calibration_callback(self, request, response):
        """处理开始标定服务请求"""
        try:
            if not self.calibration_active:
                self.start_calibration()
                response.success = True
                response.message = "Calibration started"
            else:
                response.success = False
                response.message = "Calibration already in progress"
        except Exception as e:
            response.success = False
            response.message = f"Error starting calibration: {str(e)}"
        return response

    def start_calibration(self):
        """开始标定流程"""
        self.calibration_active = True
        self.current_calibration_point = 0
        self.get_logger().info('Calibration control started')
        self.send_next_calibration_point()

    def send_next_calibration_point(self):
        """发送下一个标定点指令"""
        if not self.calibration_active or self.current_calibration_point >= self.max_calibration_points:
            return
  
        # 添加重试机制
        max_retries = 3
        for attempt in range(max_retries):
            try:
                # 获取当前机械手位姿 (实际项目中从机械手控制器获取)
                current_pose = self.get_robot_pose()

                # 创建标定指令
                command = CalibrationCommand()
                command.command_type = CalibrationCommand.CALIB_NEXT_POINT # 移动到下一个点
                command.robot_pose = current_pose

                # 发布到话题
                self.calibration_pub.publish(command)
        
                # 同时通过UDP发送给Jetson
                self.send_calibration_command(command)
        
                self.current_calibration_point += 1
                self.get_logger().info(f'Sent calibration point {self.current_calibration_point}/{self.max_calibration_points}')
                return  # 成功发送后退出
            except Exception as e:
                self.get_logger().error(f'Failed to send point {self.current_calibration_point} (attempt {attempt+1}/{max_retries}): {str(e)}')
                time.sleep(1.0)  # 等待后重试
        
        # 多次失败后终止标定
        self.get_logger().error(f'Aborting calibration after {max_retries} failures')
        self.finish_calibration()

    def get_robot_pose(self):
        """获取机械手当前位姿 - 实际项目中替换为真实接口"""
        # 示例：返回随机位姿
        return [
            random.uniform(0.1, 1.0),  # x
            random.uniform(-0.5, 0.5),  # y
            random.uniform(0.1, 0.5),   # z
            random.uniform(-3.14, 3.14), # roll
            random.uniform(-1.57, 1.57), # pitch
            random.uniform(-3.14, 3.14)  # yaw
        ]

    # 作为ROS订阅者的回调函数，当ROS节点发布指令到/udp_server/command主题时触发。
    # •功能：将接收到的指令 ​​放入队列​​（command_queue），不直接发送。
    # •定位​​：快速响应 ROS 事件（非阻塞主线程）。
    def command_callback(self, msg):
        """接收ROS指令并放入队列"""
        try:
            self.command_queue.put(msg.data)
            if self.bShowRunInfo:
                self.get_logger().info(f'Received command from ROS: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error in command callback: {str(e)}')

    # ======== 标定响应处理 ========
    def calibration_response_callback(self, msg):
        """处理标定响应"""
        try:
            # 处理完成响应 - 新增部分
            if msg.response_type in [CalibrationResponse.RES_COMPLETED_SUCCESS, CalibrationResponse.RES_COMPLETED_FAILED]:
                self.finish_calibration()
                return
            
            # 常规响应处理
            if msg.response_type == CalibrationResponse.RES_SUCCESS:  # 识别成功
                self.handle_calibration_success(msg)
            elif msg.response_type == CalibrationResponse.RES_FAILURE:  # 识别失败
                self.handle_calibration_failure()
        except Exception as e:
            self.get_logger().error(f'Error in calibration response: {str(e)}')

    def handle_calibration_success(self, msg):
        """处理标定成功响应"""
        if self.calibration_active:
            self.get_logger().info(f'Calibration point {self.current_calibration_point} succeeded')

            # 如果还没达到最大点数，发送下一个点
            if self.current_calibration_point < self.max_calibration_points:
                self.send_next_calibration_point()
            # else:
            #     self.finish_calibration()
            #     # 通知客户端标定完成
            #     self.get_logger().info('All calibration points completed, waiting for final result')
                
    def handle_calibration_failure(self):
        """处理标定失败响应"""
        self.calibration_active = False
        self.get_logger().error('Calibration failed at point {self.current_calibration_point}')
        
        # # 发送结束指令
        # end_command = CalibrationCommand()
        # end_command.command_type = CalibrationCommand.CALIB_FINISHED  # 结束标定
        # self.calibration_pub.publish(end_command)
        # self.send_calibration_command(end_command)
    
    def finish_calibration(self):
        """结束标定流程"""
        self.calibration_active = False
        self.get_logger().info('Calibration process ended')
        
        # 发送结束指令
        # end_command = CalibrationCommand()
        # end_command.command_type = CalibrationCommand.CALIB_FINISHED  # 结束标定
        # self.calibration_pub.publish(end_command)
        # self.send_calibration_command(end_command)

    # ======== UDP标定指令发送函数 ========
    def send_calibration_command(self, command):
        """通过UDP发送标定指令"""
        try:
            # 打包消息: msg_type(4B) + command_type(1B) + robot_pose(6 * 4B)
            header = struct.pack('!I', MessageType.CALIBRATION_COMMAND)
            data = struct.pack('!B', command.command_type)
            data += struct.pack('!6f', *command.robot_pose)
            packet = header + data
            
            self.udp_socket.sendto(packet, (self.client_ip, self.client_port))
        except Exception as e:
            self.get_logger().error(f'Error sending calibration command: {str(e)}')

    bUseCmdMaxRate = True  # 是否使用命令发送频率限制
    # 指令监听函数（在服务器端）：作为独立线程运行，持续监听队列。
    #•功能：从队列中取出指令，打包为 UDP 消息后 ​​发送给客户端​​。
    #•定位​​：异步处理队列中的指令，避免阻塞 ROS 主线程。
    if bUseCmdMaxRate:
        def command_listener(self):
            """从队列获取指令并发送到客户端"""
            MAX_RATE = self.command_max_rate  # 最大发送频率：例如20条/秒
            MIN_INTERVAL = 1.0 / MAX_RATE

            while self.is_running and rclpy.ok():
                start_time = time.time()  # 记录循环开始时间
                
                try:
                    # 获取指令（超时避免永久阻塞）
                    command = self.command_queue.get(timeout=0.1)
                    
                    # 打包指令
                    header = struct.pack('!I', MessageType.COMMAND)
                    packet = header + command.encode('utf-8')
                    
                    # 发送到客户端
                    self.udp_socket.sendto(packet, (self.client_ip, self.client_port))
                    
                    # 调试日志
                    if self.bShowRunInfo:
                        self.get_logger().info(f'Sent command: {command}')
                
                except queue.Empty:
                    continue  # 队列为空时继续循环
                
                except Exception as e:
                    self.get_logger().error(f'Command send error: {str(e)}')
                
                finally:
                    # 速率控制核心：确保每次循环至少间隔MIN_INTERVAL
                    elapsed = time.time() - start_time
                    if elapsed < MIN_INTERVAL:
                        time.sleep(MIN_INTERVAL - elapsed)
    else:
        def command_listener(self):
            """从队列获取指令并发送到客户端"""
            while self.is_running and rclpy.ok():
                try:
                    # 从队列获取指令（带超时避免永久阻塞）
                    command = self.command_queue.get(timeout=0.1)
                    
                    # 打包指令消息
                    header = struct.pack('!I', MessageType.COMMAND)
                    packet = header + command.encode('utf-8')
                    
                    # 发送到客户端
                    self.udp_socket.sendto(packet, (self.client_ip, self.client_port))
                    
                    if self.bShowRunInfo:
                        self.get_logger().info(f'Sent command to client: {command}')
                        
                except queue.Empty:
                    # 队列为空时继续循环
                    continue
                except Exception as e:
                    self.get_logger().error(f'Error sending command: {str(e)}')
    
    def process_ai_coordinate_data(self, data, addr):
        """处理AI坐标数据（保持JSON，因数据量小）"""
        try:
            json_data = json.loads(data.decode('utf-8'))
            msg = AICoordinateData()
            msg.object_class = json_data['object_class']
            msg.position = json_data['position']
            msg.orientation = json_data['orientation']
            msg.confidence = json_data['confidence']
            msg.frame_id = json_data['frame_id']
            msg.stamp.sec = json_data['stamp']['sec']
            msg.stamp.nanosec = json_data['stamp']['nanosec']
            
            self.ai_coordinate_pub.publish(msg)
            
            # 日志频率控制
            self.log_counter += 1
            # 根据bShowRunInfo控制运行时的调试信息是否输出
            if self.bShowRunInfo and self.log_counter % self.log_interval == 0:
                self.get_logger().info(f'Published AI coordinate data from {addr}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing AI coordinate data: {str(e)}')
  
    def process_laserscan_data(self, data, addr):
        """处理雷达扫描数据（二进制协议）"""
        try:
            # 二进制解包（替代JSON）
            # 格式: [angle_min(4f), angle_max(4f), angle_increment(4f), time_increment(4f), 
            #        scan_time(4f), range_min(4f), range_max(4f), frame_id_len(4I), 
            #        frame_id(str), stamp_sec(4I), stamp_nanosec(4I), 
            #        ranges_len(4I), ranges(var), intensities_len(4I), intensities(var)]
            offset = 0
            fmt = '!7f'  # 7个float
            size = struct.calcsize(fmt)
            angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max = struct.unpack(fmt, data[offset:offset+size])
            offset += size
            
            frame_id_len = struct.unpack('!I', data[offset:offset+4])[0]
            offset += 4
            frame_id = data[offset:offset+frame_id_len].decode('utf-8')
            offset += frame_id_len
            
            stamp_sec, stamp_nanosec = struct.unpack('!II', data[offset:offset+8])
            offset += 8
            
            ranges_len = struct.unpack('!I', data[offset:offset+4])[0]
            offset += 4
            ranges = list(struct.unpack(f'!{ranges_len}f', data[offset:offset+4*ranges_len]))
            offset += 4 * ranges_len
            
            intensities_len = struct.unpack('!I', data[offset:offset+4])[0]
            offset += 4
            intensities = list(struct.unpack(f'!{intensities_len}f', data[offset:offset+4*intensities_len])) if intensities_len > 0 else []
            
            # 创建ROS2消息
            msg = LaserScan()
            msg.header.frame_id = frame_id
            msg.header.stamp.sec = stamp_sec
            msg.header.stamp.nanosec = stamp_nanosec
            msg.angle_min = angle_min
            msg.angle_max = angle_max
            msg.angle_increment = angle_increment
            msg.time_increment = time_increment
            msg.scan_time = scan_time
            msg.range_min = range_min
            msg.range_max = range_max
            msg.ranges = ranges
            msg.intensities = intensities
            
            self.laserscan_pub.publish(msg)
            
            # 日志频率控制
            self.log_counter += 1
            # 根据bShowRunInfo控制运行时的调试信息是否输出
            if self.bShowRunInfo and self.log_counter % self.log_interval == 0:
                self.get_logger().info(f'Published laser scan data from {addr}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing laser scan data: {str(e)}')

    def process_pointcloud_data(self, data, addr):
        """处理点云数据（二进制协议）"""
        try:
            # 二进制解包（替代JSON）
            # 格式: [height(4I), width(4I), point_step(4I), stamp_sec(4I), stamp_nanosec(4I),
            #        camera_intrinsic(36f), frame_id_len(4I), frame_id(str), data_len(4I), data(var)]
            offset = 0
            fmt = '!III'  # height, width, point_step (3x uint32)
            size = struct.calcsize(fmt)
            height, width, point_step = struct.unpack(fmt, data[offset:offset+size])
            offset += size
            
            stamp_sec, stamp_nanosec = struct.unpack('!II', data[offset:offset+8])
            offset += 8
            
            intrinsic_size = struct.calcsize('!9f')
            camera_intrinsic = struct.unpack('!9f', data[offset:offset+intrinsic_size])
            offset += intrinsic_size
            
            frame_id_len = struct.unpack('!I', data[offset:offset+4])[0]
            offset += 4
            frame_id = data[offset:offset+frame_id_len].decode('utf-8')
            offset += frame_id_len
            
            data_len = struct.unpack('!I', data[offset:offset+4])[0]
            offset += 4
            point_data = data[offset:offset+data_len]
            
            # 创建ROS2消息
            msg = PointCloudData()
            msg.height = height
            msg.width = width
            msg.point_step = point_step
            msg.data = point_data
            msg.camera_intrinsic = list(camera_intrinsic)
            msg.frame_id = frame_id
            msg.stamp.sec = stamp_sec
            msg.stamp.nanosec = stamp_nanosec
            
            self.pointcloud_pub.publish(msg)
            
            # 日志频率控制
            self.log_counter += 1
            # 根据bShowRunInfo控制运行时的调试信息是否输出
            if self.bShowRunInfo and self.log_counter % self.log_interval == 0:
                self.get_logger().info(f'Published pointcloud data from {addr}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing pointcloud data: {str(e)}')
    
    def process_detection_image_data(self, data, addr):
        """处理检测图像数据（PNG base64格式）"""
        try:
            # 检测图像数据格式：JSON格式包含base64编码的PNG图像
            json_data = json.loads(data.decode('utf-8'))
            
            # 创建ROS消息
            msg = String()
            msg.data = json_data['image_data']  # base64编码的PNG图像
            
            # 发布到ROS话题
            self.detection_image_pub.publish(msg)
            
            # 日志频率控制
            self.log_counter += 1
            if self.bShowRunInfo and self.log_counter % self.log_interval == 0:
                self.get_logger().info(f'Published detection image data from {addr}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing detection image data: {str(e)}')
    
    def process_detection_result_data(self, data, addr):
        """处理检测结果数据（JSON格式）"""
        try:
            # 检测结果数据格式：JSON格式的Detection2DArray
            json_data = json.loads(data.decode('utf-8'))
            
            # 创建ROS消息
            msg = Detection2DArray()
            msg.header.stamp.sec = json_data['header']['stamp']['sec']
            msg.header.stamp.nanosec = json_data['header']['stamp']['nanosec']
            msg.header.frame_id = json_data['header']['frame_id']
            
            # 解析检测结果
            for det_data in json_data['detections']:
                detection = Detection2D()
                
                # 边界框信息
                detection.bbox.center.position.x = det_data['bbox']['center']['position']['x']
                detection.bbox.center.position.y = det_data['bbox']['center']['position']['y']
                detection.bbox.size_x = det_data['bbox']['size_x']
                detection.bbox.size_y = det_data['bbox']['size_y']
                
                # 检测结果
                for hyp_data in det_data['results']:
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = hyp_data['hypothesis']['class_id']
                    hypothesis.hypothesis.score = hyp_data['hypothesis']['score']
                    
                    # 3D位置信息
                    hypothesis.pose.pose.position.x = hyp_data['pose']['pose']['position']['x']
                    hypothesis.pose.pose.position.y = hyp_data['pose']['pose']['position']['y']
                    hypothesis.pose.pose.position.z = hyp_data['pose']['pose']['position']['z']
                    hypothesis.pose.pose.orientation.x = hyp_data['pose']['pose']['orientation']['x']
                    hypothesis.pose.pose.orientation.y = hyp_data['pose']['pose']['orientation']['y']
                    hypothesis.pose.pose.orientation.z = hyp_data['pose']['pose']['orientation']['z']
                    hypothesis.pose.pose.orientation.w = hyp_data['pose']['pose']['orientation']['w']
                    
                    detection.results.append(hypothesis)
                
                msg.detections.append(detection)
            
            # 发布到ROS话题
            self.detection_result_pub.publish(msg)
            
            # 日志频率控制
            self.log_counter += 1
            if self.bShowRunInfo and self.log_counter % self.log_interval == 0:
                self.get_logger().info(f'Published detection result data from {addr}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing detection result data: {str(e)}')
    
    def process_interactive_command_data(self, data, addr):
        """处理交互指令数据（JSON格式）"""
        try:
            # 交互指令数据格式：JSON格式的InteractiveCommand
            json_data = json.loads(data.decode('utf-8'))
            
            # 创建ROS消息
            msg = InteractiveCommand()
            msg.act_command_type = json_data['act_command_type']
            msg.data = json_data['data']
            
            # 位置信息
            msg.pose.header.stamp.sec = json_data['pose']['header']['stamp']['sec']
            msg.pose.header.stamp.nanosec = json_data['pose']['header']['stamp']['nanosec']
            msg.pose.header.frame_id = json_data['pose']['header']['frame_id']
            msg.pose.pose.position.x = json_data['pose']['pose']['position']['x']
            msg.pose.pose.position.y = json_data['pose']['pose']['position']['y']
            msg.pose.pose.position.z = json_data['pose']['pose']['position']['z']
            msg.pose.pose.orientation.x = json_data['pose']['pose']['orientation']['x']
            msg.pose.pose.orientation.y = json_data['pose']['pose']['orientation']['y']
            msg.pose.pose.orientation.z = json_data['pose']['pose']['orientation']['z']
            msg.pose.pose.orientation.w = json_data['pose']['pose']['orientation']['w']
            
            # 发布到ROS话题
            self.interactive_command_pub.publish(msg)
            
            # 日志频率控制
            self.log_counter += 1
            if self.bShowRunInfo and self.log_counter % self.log_interval == 0:
                self.get_logger().info(f'Published interactive command data from {addr}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing interactive command data: {str(e)}')
    
    def destroy_node(self):
        self.is_running = False

        # 等待接收线程结束
        if self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)

        # 等待指令线程结束
        if self.command_thread.is_alive():
            self.command_thread.join(timeout=2.0)

        # 关闭UDP套接字
        try:
            self.udp_socket.close()
        except Exception as e:
            self.get_logger().error(f"Error closing UDP socket: {str(e)}")

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UDPServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()