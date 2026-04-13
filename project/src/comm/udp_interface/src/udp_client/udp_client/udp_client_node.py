#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
import socket
import struct
import json
import threading
import time
import subprocess
import os
import psutil
from enum import IntEnum
import errno  # 添加errno模块用于处理socket错误
import signal

from custom_msgs.msg import PointCloudData, AICoordinateData, LaserScanData, CalibrationCommand, CalibrationResponse
from custom_msgs.msg import ProjectCommand, ProjectStatus
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String  # 用于接收指令
from vision_msgs.msg import Detection2DArray  # 检测结果消息
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

class CameraCommandFactory:
    """相机命令工厂类，封装不同相机的命令差异"""
    
    @staticmethod
    def get_commands(camera_type):
        """根据相机类型获取相应的启动和停止命令"""
        if camera_type == "Realsense":
            return {
                #"start": "source /home/user/project/install/setup.bash && ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_sync:=true",
                "start": "source /home/user/project/install/setup.bash && ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_sync:=true depth_width:=640 depth_height:=480 depth_fps:=15 color_width:=640 color_height:=480 color_fps:=15 decimation_filter.enable:=true",
                "stop": "pkill -f 'realsense2_camera'; pkill -f 'rs_launch.py'",
                "check": "pgrep -f 'realsense2_camera\|rs_launch.py'",
                "description": "RealSense相机项目"
            }
        elif camera_type == "Gemini":
            return {
                "start": "source /home/user/project/install/setup.bash && ros2 launch orbbec_camera gemini2.launch.py depth_width:=640 depth_height:=480 depth_fps:=15 color_width:=640 color_height:=480 color_fps:=15 depth_decimation_filter:=true enable_hardware_noise_removal:=false enable_point_cloud:=true",
                "stop": "pkill -f 'orbbec_camera'; pkill -f 'orbbec_camera.launch.py'",
                "check": "pgrep -f 'orbbec_camera\|orbbec_camera.launch.py'",
                "description": "Gemini相机项目"
            }
        else:
            raise ValueError(f"不支持的相机类型: {camera_type}")

class ProjectManager:
    def __init__(self, logger, udp_client_node=None):
        self.logger = logger
        self.udp_client_node = udp_client_node  # 添加UDP客户端节点引用
        self.project_status = {} # 初始化一个空的项目状态字典，稍后在 update_camera_commands 中填充
        #self.project_status = {name: ProjectStatus.STATUS_STOPPED for name in self.PROJECTS}
        self.project_processes = {}
        self.project_pids = {}  # 存储项目的主进程ID
        #self.project_lock = threading.Lock()
        self.project_lock = threading.RLock()  # 改为可重入锁：使用RLock替代普通锁，确保重入安全性
        self.camera_type = "Gemini"  # 默认值，将在初始化后更新
        self.update_camera_commands()

    def set_camera_type(self, camera_type):
        """设置相机类型"""
        with self.project_lock:
            self.camera_type = camera_type
            # 更新相机项目命令
            self.update_camera_commands()

    def update_camera_commands(self):
        """更新相机项目命令"""
        with self.project_lock:
            # 获取当前设置的相机类型
            current_camera_type = self.camera_type

             # 根据相机类型获取相应的命令配置
            camera_commands = CameraCommandFactory.get_commands(current_camera_type)

            self.PROJECTS = {
                "camera": camera_commands,
                "elevator": {
                    "start": "source /home/user/project/install/setup.bash && ros2 launch elevator_button_detector detection.launch.py",
                    "stop": "pkill -f 'elevator_button_detector\|detection.launch.py'",
                    "check": "pgrep -f 'elevator_button_detector\|detection.launch.py'",
                    "description": "电梯按钮识别项目"
                },
                "pcl2laser": {
                    #"start": "source /home/user/project/install/setup.bash && ros2 launch pcl2laserscan_trans pcl2laserscan_trans.launch.py",
                    "start": f"source /home/user/project/install/setup.bash && ros2 launch pcl2laserscan_trans pcl2laserscan_trans.launch.py mode:=camera_only camera_type:={current_camera_type}", # 修改pcl2laser启动命令，使其适配不同相机类型
                    "stop": "pkill -f 'pcl2laserscan_trans.launch.py'; pkill -f 'ros2.*pcl2laserscan_trans'",
                    "check": "pgrep -f 'pcl2laserscan_trans\|pcl2laserscan_trans.launch.py'",
                    "working_dir": "/home/user/project/install",
                    #"setup_script": "install/setup.bash"
                    "setup_script": "setup.bash",
                    "description": "相机点云转激光扫描项目"
                },
                "detection": {
                    "start": f"source /home/user/project/install/setup.bash && ros2 launch yolo_detection yolo_det.py camera_type:={current_camera_type}",
                    "stop": "pkill -f 'yolo_detection\|yolo_det.py'",
                    "check": "pgrep -f 'yolo_detection\|yolo_det.py'",
                    "working_dir": "/home/user/project/install",
                    "setup_script": "setup.bash",
                    "description": "目标检测项目"
                },
                "cmd_dispatcher": {
                    "start": f"source /home/user/project/install/setup.bash && ros2 launch cmd_dispatcher cmd_dispatcher.launch.py",
                    "stop": "pkill -f 'cmd_dispatcher\|cmd_dispatcher.launch.py'",
                    "check": "pgrep -f 'cmd_dispatcher\|cmd_dispatcher.launch.py'",
                    "working_dir": "/home/user/project/install",
                    "setup_script": "setup.bash",
                    "description": "语音识别中间调度项目"
                },
            }
            # 初始化项目状态
            for project_name in self.PROJECTS:
                if project_name not in self.project_status:
                    self.project_status[project_name] = ProjectStatus.STATUS_STOPPED

    def set_udp_client_node(self, udp_client_node):
        """设置UDP客户端节点引用"""
        self.udp_client_node = udp_client_node

    def start_project(self, project_name):
        with self.project_lock:
            if project_name not in self.PROJECTS:
                return ProjectStatus.STATUS_ERROR, f"Unknown project: {project_name}"
            
            # 检查项目是否已在运行
            if self.is_project_running(project_name):
                return ProjectStatus.STATUS_ERROR, f"Project {project_name} is already running"
        
            try:
                # 获取项目配置
                project_config = self.PROJECTS[project_name]
                working_dir = project_config.get("working_dir")
                setup_script = project_config.get("setup_script")
                
                # 构建启动命令
                start_cmd = project_config["start"]
                
                # 如果有工作目录和环境设置脚本，构建完整的命令
                if working_dir and setup_script:
                    start_cmd = f"cd {working_dir} && source {setup_script} && {start_cmd}"
                
                self.logger.info(f"Starting project {project_name} with command: {start_cmd}")

                # 启动项目，使用nohup和setsid确保进程独立
                process = subprocess.Popen(
                    start_cmd,
                    shell=True,
                    executable="/bin/bash",
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid,
                    cwd=working_dir if working_dir else None
                )
                
                self.project_processes[project_name] = process
                self.project_pids[project_name] = process.pid
                
                # 等待一段时间，检查进程是否启动成功
                time.sleep(3)  # 等待3秒，给进程启动时间
                
                # 多次检查进程是否启动成功
                is_running = False
                for i in range(15):  # 检查5次，每次间隔2秒
                    if self.is_project_running(project_name):
                        is_running = True
                        self.logger.info(f'check project running status: process running successfully')
                        break
                    time.sleep(2)  # 每次检查间隔2秒
                
                if is_running:
                    self.project_status[project_name] = ProjectStatus.STATUS_RUNNING
                    
                    # 启动监控线程
                    monitor_thread = threading.Thread(
                        target=self.monitor_project,
                        args=(project_name, process.pid)
                    )
                    monitor_thread.daemon = True
                    monitor_thread.start()
                    
                    return ProjectStatus.STATUS_RUNNING, f"Project {project_name} started successfully"
                else:
                    self.project_status[project_name] = ProjectStatus.STATUS_ERROR
                    # 尝试终止进程
                    try:
                        if process.poll() is None:  # 检查进程是否还在运行
                            os.killpg(os.getpgid(process.pid), 15)
                            process.wait(timeout=5)  # 等待进程结束
                    except (ProcessLookupError, subprocess.TimeoutExpired):
                        pass
                    # 获取错误输出
                    stdout, stderr = process.communicate()
                    # 输出详细的错误日志
                    error_msg = f"Project {project_name} failed to start. stdout: {stdout.decode()}, stderr: {stderr.decode()}"
                    self.logger.error(error_msg)
                    return ProjectStatus.STATUS_ERROR, error_msg
            except Exception as e:
                self.project_status[project_name] = ProjectStatus.STATUS_ERROR
                error_msg = f"Failed to start project {project_name}: {str(e)}"
                self.logger.error(error_msg)
                return ProjectStatus.STATUS_ERROR, error_msg

    def stop_project(self, project_name):
        with self.project_lock:
            if project_name not in self.PROJECTS:
                return ProjectStatus.STATUS_ERROR, f"Unknown project: {project_name}"
            
            if not self.is_project_running(project_name):
                return ProjectStatus.STATUS_ERROR, f"Project {project_name} is not running"
            
            try:
                # 方法1: 终止进程组
                if project_name in self.project_pids:
                    try:
                        # 获取进程组ID并终止整个进程组
                        pgid = os.getpgid(self.project_pids[project_name])
                        os.killpg(pgid, 9)  # 直接使用SIGKILL，确保立即终止
                        time.sleep(2)  # 等待进程终止
                        
                        # 检查是否还有残留进程
                        if self.is_pid_running(self.project_pids[project_name]):
                            os.killpg(pgid, 9)  # SIGKILL
                            time.sleep(1)  # 短暂等待
                    except ProcessLookupError:
                        pass  # 进程已不存在
                
                # 方法2: 使用pkill终止相关进程
                subprocess.run(f"pkill -f '{self.PROJECTS[project_name]['stop']}'", shell=True)
                
                # 方法3: 终止特定模式的进程
                if project_name == "camera":
                    subprocess.run("pkill -f 'realsense2_camera'", shell=True)
                    subprocess.run("pkill -f 'rs_launch.py'", shell=True)
                elif project_name == "elevator":
                    subprocess.run("pkill -f 'elevator_button_detector'", shell=True)
                    subprocess.run("pkill -f 'detection.launch.py'", shell=True)
                elif project_name == "pcl2laser":
                    subprocess.run("pkill -f 'pcl2laserscan_trans'", shell=True)
                    subprocess.run("pkill -f 'pcl2laserscan_trans.launch.py'", shell=True)
                elif project_name == "detection":
                    subprocess.run("pkill -f 'yolo_detection'", shell=True)
                    subprocess.run("pkill -f 'yolo_det.py'", shell=True)
                elif project_name == "cmd_dispatcher":
                    subprocess.run("pkill -f 'cmd_dispatcher'", shell=True)
                    subprocess.run("pkill -f 'cmd_dispatcher.launch.py'", shell=True)
                
                # 等待一段时间，检查进程是否终止
                time.sleep(2)  # 等待2秒，给进程更多时间停止
                
                # 多次检查确保进程已停止
                is_stopped = True
                for i in range(3):  # 检查3次
                    if self.is_project_running(project_name):
                        is_stopped = False
                        time.sleep(1)  # 每次检查间隔1秒
                    else:
                        is_stopped = True
                        break
                
                if is_stopped:
                    # 清理资源
                    if project_name in self.project_processes:
                        del self.project_processes[project_name]
                    if project_name in self.project_pids:
                        del self.project_pids[project_name]
                    
                    self.project_status[project_name] = ProjectStatus.STATUS_STOPPED
                    return ProjectStatus.STATUS_STOPPED, f"Project {project_name} stopped successfully"
                else:
                    self.project_status[project_name] = ProjectStatus.STATUS_ERROR
                    return ProjectStatus.STATUS_ERROR, f"Failed to stop project {project_name}"
                    
            except Exception as e:
                self.project_status[project_name] = ProjectStatus.STATUS_ERROR
                return ProjectStatus.STATUS_ERROR, f"Failed to stop project {project_name}: {str(e)}"

    def is_pid_running(self, pid):
        """检查指定PID的进程是否在运行"""
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False

    def is_project_running(self, project_name):
        """检查项目是否真的在运行（基于进程ID和进程名）"""
        # 检查存储的进程ID
        if project_name in self.project_pids and self.is_pid_running(self.project_pids[project_name]):
            self.logger.info(f"Project {project_name} is running based on PID {self.project_pids[project_name]}")
            return True
        
         # 使用项目特定的检查命令
        check_cmd = self.PROJECTS[project_name].get("check", "")
        if check_cmd:
            result = subprocess.run(check_cmd, shell=True, capture_output=True)
            if result.returncode == 0:
                return True
        # 对于pcl2laser项目，额外检查是否有相关的ROS节点在运行
        if project_name == "pcl2laser":
            try:
                node_list = subprocess.run(
                    "ros2 node list",
                    shell=True, capture_output=True, text=True, timeout=5.0
                )
                # 查找与pcl2laser相关的节点
                if "pcl2laser" in node_list.stdout or "pointcloud_to_laserscan" in node_list.stdout:
                    return True
            except:
                pass
        
        return False

    def monitor_project(self, project_name, pid):
        """监控项目进程，如果进程意外退出则更新状态"""
        try:
            # 等待进程结束
            process = psutil.Process(pid)
            # 添加超时机制，避免无限等待
            try:
                return_code = process.wait(timeout=0.1)
            except psutil.TimeoutExpired:
                # 进程仍在运行，继续监控
                threading.Timer(2.0, self.monitor_project, args=(project_name, pid)).start()
                return
  
            # 记录详细的退出信息
            self.logger.info(f"Project {project_name} (PID: {pid}) exited with return code: {return_code}")

            # 进程已结束，检查是否是正常退出
            with self.project_lock:
                if (project_name in self.project_pids and 
                    self.project_pids[project_name] == pid):
                    
                    # 只有当仍然在跟踪这个项目时才更新状态
                    # 如果状态已经被设置为停止（例如通过stop命令），则不更新
                    if self.project_status[project_name] == ProjectStatus.STATUS_RUNNING:

                        # 更新状态为停止
                        self.project_status[project_name] = ProjectStatus.STATUS_STOPPED

                        # 清理资源
                        if project_name in self.project_processes:
                            del self.project_processes[project_name]
                        if project_name in self.project_pids:
                            del self.project_pids[project_name]

                         # 发送状态更新到服务器
                        if self.udp_client_node:
                            status_msg = ProjectStatus()
                            status_msg.status = ProjectStatus.STATUS_STOPPED
                            status_msg.project_name = project_name
                            
                            # 区分正常退出和异常退出
                            if return_code == 0:
                                status_msg.message = f"Project exited normally"
                                self.logger.info(f"Project {project_name} exited normally")
                            else:
                                status_msg.message = f"Project exited unexpectedly with return code {return_code}"
                                self.logger.warn(f"Project {project_name} exited unexpectedly with return code {return_code}")
                            
                            self.udp_client_node.send_project_status(status_msg)
                       
        except (psutil.NoSuchProcess, ProcessLookupError):
            # 进程已不存在
            with self.project_lock:
                if project_name in self.project_pids and self.project_pids[project_name] == pid:
                    # 同样，只有当仍然在跟踪这个项目时才更新状态
                    if self.project_status[project_name] == ProjectStatus.STATUS_RUNNING:
                        self.project_status[project_name] = ProjectStatus.STATUS_STOPPED
                        if project_name in self.project_processes:
                            del self.project_processes[project_name]
                        if project_name in self.project_pids:
                            del self.project_pids[project_name]
                        
                        # 发送状态更新到服务器
                        if self.udp_client_node:
                            status_msg = ProjectStatus()
                            status_msg.status = ProjectStatus.STATUS_STOPPED
                            status_msg.project_name = project_name
                            status_msg.message = "Project process not found"
                            self.udp_client_node.send_project_status(status_msg)

    def get_status(self, project_name):
        """获取项目状态"""
        with self.project_lock:
            if project_name not in self.PROJECTS:
                return ProjectStatus.STATUS_ERROR, f"Unknown project: {project_name}", ""
            
            # 基于实际进程状态更新状态
            is_running = self.is_project_running(project_name)
            current_status = self.project_status[project_name]

            # 状态一致性检查
            if is_running and current_status != ProjectStatus.STATUS_RUNNING:
                self.project_status[project_name] = ProjectStatus.STATUS_RUNNING
            elif not is_running and current_status not in [ProjectStatus.STATUS_STOPPED, ProjectStatus.STATUS_ERROR]:
                self.project_status[project_name] = ProjectStatus.STATUS_STOPPED

            status_msg = {
                ProjectStatus.STATUS_STOPPED: "Stopped",
                ProjectStatus.STATUS_RUNNING: "Running",
                ProjectStatus.STATUS_STARTING: "Starting",
                ProjectStatus.STATUS_STOPPING: "Stopping",
                ProjectStatus.STATUS_ERROR: "Error"
            }.get(self.project_status[project_name], "Unknown")
            
            return self.project_status[project_name], self.PROJECTS[project_name].get("description", ""), status_msg
        
class UDPClientNode(Node):
    def __init__(self):
        super().__init__('udp_client')
        
        # 声明参数
        self.declare_parameter('camera_type', 'Gemini')  # 相机类型参数
        self.declare_parameter('server_ip', '192.168..33')  # x86主板IP
        self.declare_parameter('server_port', 8888)
        # 添加客户端命令接收端口
        self.declare_parameter('client_port', 8889) # 客户端监听端口
        # 添加参数控制运行时的调试信息是否输出
        self.declare_parameter('bShowRunInfo', True)  # 控制是否显示运行时的调试信息
        self.declare_parameter('log_interval', 10)  # 添加日志频率控制间隔参数（每10帧输出1次）
  
        self.declare_parameter('udp_blocking_mode', False)  # 控制UDP发送是否为阻塞模式

        self.declare_parameter('auto_start_enabled', False)  # 是否启用自动启动项目（总开关）
        self.declare_parameter('auto_start_camera', False)   # 是否自动启动camera项目
        self.declare_parameter('auto_start_pcl2laser', False) # 是否自动启动pcl2laser项目
        self.declare_parameter('auto_start_detection', False) # 是否自动启动detection项目
        self.declare_parameter('auto_start_cmd_dispatcher', False) # 是否自动启动cmd_dispatcher项目
        self.declare_parameter('auto_start_delay', 3.0)      # 自动启动延迟(秒)：在启动UDP客户端多少秒以后自动启动camera项目

        # 获取参数
        camera_type = self.get_parameter('camera_type').get_parameter_value().string_value
        server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.client_port = self.get_parameter('client_port').get_parameter_value().integer_value
        self.bShowRunInfo = self.get_parameter('bShowRunInfo').get_parameter_value().bool_value
        self.log_interval = self.get_parameter('log_interval').get_parameter_value().integer_value  # 日志频率控制间隔参数

        # 获取新增的UDP阻塞模式参数
        udp_blocking_mode = self.get_parameter('udp_blocking_mode').get_parameter_value().bool_value

        self.auto_start_enabled = self.get_parameter('auto_start_enabled').get_parameter_value().bool_value
        self.auto_start_camera = self.get_parameter('auto_start_camera').get_parameter_value().bool_value
        self.auto_start_pcl2laser = self.get_parameter('auto_start_pcl2laser').get_parameter_value().bool_value
        self.auto_start_detection = self.get_parameter('auto_start_detection').get_parameter_value().bool_value
        self.auto_start_cmd_dispatcher = self.get_parameter('auto_start_cmd_dispatcher').get_parameter_value().bool_value
        self.auto_start_delay = self.get_parameter('auto_start_delay').get_parameter_value().double_value

		# 日志计数器
        self.ai_log_counter = 0
        self.laser_log_counter = 0
        self.pointcloud_log_counter = 0

        # 标定状态变量
        self.calibration_active = False
        self.calibration_points = []  # 存储(像素坐标, 机械坐标)
        self.current_calibration_index = 0
        self.max_calibration_points = 100

        # 添加项目管理器并设置相机类型
        self.project_manager = ProjectManager(self.get_logger(), self)
        self.project_manager.set_camera_type(camera_type)
        
        # 创建UDP套接字
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = (server_ip, server_port)

        # 设置发送缓冲大小（1MB）
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024 * 1024)

        # 设置UDP发送模式
        self.set_udp_blocking_mode(udp_blocking_mode)
   
        # 绑定客户端端口用于接收指令
        self.udp_socket.bind(('0.0.0.0', self.client_port))
        self.udp_socket.settimeout(0.1)
        
        # 创建ROS2订阅者
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            #reliability=QoSReliabilityPolicy.BEST_EFFORT,
            reliability=QoSReliabilityPolicy.RELIABLE,  # 改为RELIABLE确保消息不丢失 
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 对于激光扫描数据，使用更合适的QoS配置
        laser_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # 激光数据使用BEST_EFFORT
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # 指令订阅者
        self.command_sub = self.create_subscription(
            String,
            '/udp_server/command',
            self.command_callback,
            10
        )

        # 指令响应发布者
        self.response_pub = self.create_publisher(String, '/udp_client/response', 10)
     
        # 创建标定发布者
        self.calibration_pub = self.create_publisher(
            CalibrationResponse,  # 使用新定义的消息类型
            '/calibration/response', 
            10
        )
        
        self.ai_coordinate_sub = self.create_subscription(
            AICoordinateData,
            '/ai_detection/coordinates',
            self.ai_coordinate_callback,
            qos_profile
        )

        self.laserscan_sub = self.create_subscription(
            LaserScan,
            '/camera_scan',
            self.laserscan_callback,
            laser_qos  # 使用专门的QoS配置
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloudData,
            '/camera/pointcloud',
            self.pointcloud_callback,
            qos_profile
        )

        # 标定指令订阅者
        self.calibration_sub = self.create_subscription(
            CalibrationCommand,  # 使用新定义的消息类型
            '/calibration/command',
            self.calibration_callback,
            10
        )

        # 新增订阅者：检测图像、检测结果、交互指令
        self.detection_image_sub = self.create_subscription(
            String,
            '/det_image_png',
            self.detection_image_callback,
            10
        )

        self.detection_result_sub = self.create_subscription(
            Detection2DArray,
            '/det_res',
            self.detection_result_callback,
            10
        )

        self.interactive_command_sub = self.create_subscription(
            InteractiveCommand,
            '/interactive_command',
            self.interactive_command_callback,
            10
        )

        # 启动接收线程
        self.is_running = True
        self.receive_thread = threading.Thread(target=self.receive_loop)
        self.receive_thread.start()

        # 根据bShowRunInfo控制运行时的信息是否输出 
        # if self.bShowRunInfo:
        self.get_logger().info(f'UDP Client initialized, sending to {server_ip}:{server_port}')
        self.get_logger().info(f'Listening for commands on port {self.client_port}')
        self.get_logger().info(f'Log interval: every {self.log_interval} messages')
        self.get_logger().info(f'camera type: {camera_type}')
		
        # 注册信号处理:在UDP客户端节点中添加信号处理，使其能够更快响应系统关闭信号
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)

        # 最后在末尾添加自动启动逻辑
        if self.auto_start_enabled:
            self.auto_start_timer = self.create_timer(
                self.auto_start_delay, 
                self.auto_start_callback
            )

    def signal_handler(self, signum, frame):
        """处理系统信号"""
        self.get_logger().info(f"Received signal {signum}, shutting down...")
        self.destroy_node()
        rclpy.shutdown()
    
    # 添加设置阻塞模式的方法
    def set_udp_blocking_mode(self, blocking):
        """设置UDP发送模式"""
        self.udp_blocking_mode = blocking
        self.udp_socket.setblocking(blocking)
        mode = "阻塞" if blocking else "非阻塞"
        self.get_logger().info(f"UDP发送模式设置为: {mode}")

    def auto_start_callback(self):
        """自动启动回调函数"""
        self.destroy_timer(self.auto_start_timer)  # 一次性定时器，执行后销毁
        
        if self.auto_start_camera:
            self.get_logger().info("Auto starting camera project...")
            self._execute_project_command_async(
                ProjectCommand.PROJECT_START, 
                "camera"
            )
            
            # 如果也需要自动启动pcl2laser，设置另一个定时器
            if self.auto_start_pcl2laser:
                self.auto_start_pcl2laser_timer = self.create_timer(
                    5.0,  # 5秒后启动
                    self.auto_start_pcl2laser_callback
                )
            if self.auto_start_detection:
                self.auto_start_detection_timer = self.create_timer(
                    5.0,  # 5秒后启动
                    self.auto_start_detection_callback
                )
            if self.auto_start_cmd_dispatcher:
                self.auto_start_cmd_dispatcher_timer = self.create_timer(
                    7.0,  # 7秒后启动
                    self.auto_start_cmd_dispatcher_callback
                )

    def auto_start_pcl2laser_callback(self):
        """自动启动pcl2laser回调函数"""
        self.destroy_timer(self.auto_start_pcl2laser_timer)
        time.sleep(3.0) # 增加额外延迟，确保camera项目已完全启动
        self.get_logger().info("Auto starting pcl2laser project...")
        # 添加重试机制
        max_retries = 3
        for attempt in range(max_retries):
            success, message = self.project_manager.start_project("pcl2laser")
            if success:
                self.get_logger().info(f"pcl2laser project started successfully on attempt {attempt+1}")
                break
            else:
                self.get_logger().warn(f"Failed to start pcl2laser project on attempt {attempt+1}: {message}")
                if attempt < max_retries - 1:
                    time.sleep(5.0)  # 等待5秒后重试
        """
        self.get_logger().info("Auto starting pcl2laser project...")
        self._execute_project_command_async(
            ProjectCommand.PROJECT_START, 
            "pcl2laser"
        )
        """

    def auto_start_detection_callback(self):
        """自动启动detection回调函数"""
        self.destroy_timer(self.auto_start_detection_timer)
        time.sleep(3.0) # 增加额外延迟，确保camera项目已完全启动
        self.get_logger().info("Auto starting detection project...")
        # 添加重试机制
        max_retries = 3
        for attempt in range(max_retries):
            success, message = self.project_manager.start_project("detection")
            if success:
                self.get_logger().info(f"detection project started successfully on attempt {attempt+1}")
                break
            else:
                self.get_logger().warn(f"Failed to start detection project on attempt {attempt+1}: {message}")
                if attempt < max_retries - 1:
                    time.sleep(5.0)  # 等待5秒后重试
        """
        self.get_logger().info("Auto starting detection project...")
        self._execute_project_command_async(
            ProjectCommand.PROJECT_START, 
            "detection"
        )
        """

    def auto_start_cmd_dispatcher_callback(self):
        """自动启动cmd_dispatcher回调函数"""
        self.destroy_timer(self.auto_start_cmd_dispatcher_timer)
        time.sleep(3.0) # 增加额外延迟，确保camera项目已完全启动
        self.get_logger().info("Auto starting cmd_dispatcher project...")
        # 添加重试机制
        max_retries = 3
        for attempt in range(max_retries):
            success, message = self.project_manager.start_project("cmd_dispatcher")
            if success:
                self.get_logger().info(f"cmd_dispatcher project started successfully on attempt {attempt+1}")
                break
            else:
                self.get_logger().warn(f"Failed to start cmd_dispatcher project on attempt {attempt+1}: {message}")
                if attempt < max_retries - 1:
                    time.sleep(5.0)  # 等待5秒后重试
        """
        self.get_logger().info("Auto starting cmd_dispatcher project...")
        self._execute_project_command_async(
            ProjectCommand.PROJECT_START, 
            "cmd_dispatcher"
        )
        """

    def handle_project_command(self, command_msg):
        """处理项目控制命令（异步方式）"""
        try:
            project_name = command_msg.project_name
            command_type = command_msg.command_type
            
            # 创建线程异步执行项目操作
            thread = threading.Thread(
                target=self._execute_project_command_async,
                args=(command_type, project_name)
            )
            thread.daemon = True
            thread.start()
            
        except Exception as e:
            self.get_logger().error(f"Error handling project command: {str(e)}")
            # 发送错误状态
            status_msg = ProjectStatus()
            status_msg.status = ProjectStatus.STATUS_ERROR
            status_msg.project_name = command_msg.project_name if 'project_name' in locals() else "unknown"
            status_msg.message = f"Error: {str(e)}"
            self.send_project_status(status_msg)

    def _execute_project_command_async(self, command_type, project_name):
        """异步执行项目命令"""
        try:
            # 发送中间状态
            if command_type == ProjectCommand.PROJECT_START:
                # 先发送启动中状态
                status_msg = ProjectStatus()
                status_msg.status = ProjectStatus.STATUS_STARTING
                status_msg.project_name = project_name
                status_msg.message = "Starting project"
                self.send_project_status(status_msg)
                
                # 执行启动操作
                status, message = self.project_manager.start_project(project_name)
                
            elif command_type == ProjectCommand.PROJECT_STOP:
                # 先发送停止中状态
                status_msg = ProjectStatus()
                status_msg.status = ProjectStatus.STATUS_STOPPING
                status_msg.project_name = project_name
                status_msg.message = "Stopping project"
                self.send_project_status(status_msg)
                
                # 执行停止操作
                status, message = self.project_manager.stop_project(project_name)
                
            elif command_type == ProjectCommand.PROJECT_STATUS:
                # 状态查询不需要中间状态
                status, _, message = self.project_manager.get_status(project_name)
            else:
                status = ProjectStatus.STATUS_ERROR
                message = f"Invalid command type: {command_type}"
            
            # 发送最终状态
            status_msg = ProjectStatus()
            status_msg.status = status
            status_msg.project_name = project_name
            status_msg.message = message
            self.send_project_status(status_msg) # 通过UDP发送响应
            
        except Exception as e:
            self.get_logger().error(f"Error executing project command: {str(e)}")
            # 即使出错也发送错误响应
            status_msg = ProjectStatus()
            status_msg.status = ProjectStatus.STATUS_ERROR
            status_msg.project_name = project_name
            status_msg.message = f"Error: {str(e)}"
            self.send_project_status(status_msg)

    # 发送项目状态
    def send_project_status(self, status_msg):
        try:
            # 序列化状态消息
            json_data = json.dumps({
                "status": status_msg.status,
                "project_name": status_msg.project_name,
                "message": status_msg.message
            })
            
            payload = json_data.encode('utf-8')
            header = struct.pack('!I', MessageType.PROJECT_STATUS)
            packet = header + payload
            self.udp_socket.sendto(packet, self.server_address)
            
            if self.bShowRunInfo:
                self.get_logger().info(f"Sent project status: {status_msg.project_name} - {status_msg.message}")
                
        except Exception as e:
            self.get_logger().error(f"Error sending project status: {str(e)}")

    # ======== receive_loop处理指令 ========
    def receive_loop(self):
        """循环接收UDP数据（指令）"""
        while self.is_running and rclpy.ok():
            try:
                data, addr = self.udp_socket.recvfrom(1024)
                if len(data) < 4:
                    continue
                    
                msg_type = struct.unpack('!I', data[:4])[0]
                if msg_type == MessageType.PROJECT_COMMAND: # 项目控制处理
                    try:
                        # 添加调试信息
                        if self.bShowRunInfo:
                            self.get_logger().info(f"Received PROJECT_COMMAND: {data[4:]}")
                            
                        json_data = json.loads(data[4:].decode('utf-8'))

                        # 验证必要字段
                        if "command_type" not in json_data or "project_name" not in json_data:
                            self.get_logger().error("Missing required fields in PROJECT_COMMAND")
                            raise ValueError("Missing required fields in PROJECT_COMMAND")
                        
                        command_msg = ProjectCommand()
                        command_msg.command_type = json_data["command_type"]
                        command_msg.project_name = json_data["project_name"]
                        self.handle_project_command(command_msg)
                    except UnicodeDecodeError as e:
                        self.get_logger().error(f"UTF-8 decode error: {str(e)}, data: {data[4:]}")
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f"JSON decode error: {str(e)}, data: {data[4:]}")
                    except ValueError as e:
                        self.get_logger().error(f"Value error: {str(e)}, data: {json_data}")
                    except Exception as e:
                        self.get_logger().error(f"Error processing project command: {str(e)}")       
                elif msg_type == MessageType.CALIBRATION_COMMAND: # 处理标定指令
                    self.process_calibration_command(data[4:], addr)
                else:
                    self.process_command(data, addr)  # 原有命令处理
                    
            except socket.timeout:
                continue # 超时是正常的，继续循环
            except OSError as e:
                if e.errno == errno.EINTR:
                    continue  # 被信号中断，继续循环
                elif e.errno == errno.EBADF:
                    # 套接字已关闭，退出循环
                    break
                self.get_logger().error(f'OSError receiving data: {str(e)}')
            except Exception as e:
                self.get_logger().error(f'Error receiving data: {str(e)}')
        self.get_logger().info("Receive loop exited")

    def process_calibration_command(self, data, addr):
        """处理UDP标定指令"""
        # 如果标定已结束，忽略指令
        if not self.calibration_active:
            self.get_logger().warn('Received calibration command but calibration is not active')
            return
        try:
            # 解析标定指令 (格式: command_type(1B) + robot_pose(6 * 4B))
            command_type = struct.unpack('!B', data[:1])[0]
            pose_data = struct.unpack('!6f', data[1:25])
            
            # 创建ROS消息
            msg = CalibrationCommand()
            msg.command_type = command_type
            msg.robot_pose = list(pose_data)
            
            # 发布到ROS话题
            self.calibration_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing calibration command: {str(e)}')

    # 处理接收到的指令
    def process_command(self, data, addr):
        """处理接收到的命令"""
        try:
            if len(data) < 4:
                if self.bShowRunInfo:
                    self.get_logger().warn(f'Received incomplete command from {addr}')
                return
            
            msg_type = struct.unpack('!I', data[:4])[0]
            if msg_type != MessageType.COMMAND:
                return
                
            command = data[4:].decode('utf-8')
            if self.bShowRunInfo:
                self.get_logger().info(f'Received command from {addr}: {command}')
                
            # 处理指令（此处添加您的业务逻辑）
            response = f"Executed: {command}"
            
            # 发布到ROS话题
            response_msg = String()
            response_msg.data = response
            self.response_pub.publish(response_msg)
            
            # 可选：发送响应回服务器
            self.send_response(response, addr)
                
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')

     # 发送响应
    def send_response(self, response, addr):
        try:
            header = struct.pack('!I', MessageType.COMMAND)
            packet = header + response.encode('utf-8')
            self.udp_socket.sendto(packet, addr)
        except Exception as e:
            self.get_logger().error(f'Error sending response: {str(e)}')

    # 指令回调函数
    def command_callback(self, msg):
        """处理从ROS收到的指令"""
        if self.bShowRunInfo:
            self.get_logger().info(f'Received command from ROS: {msg.data}')
        # 实际业务中可在此执行指令

    # ======== 标定指令处理函数 ========
    def calibration_callback(self, msg):
        """处理标定指令"""
        # 使用锁保护标定状态访问
        with self.calibration_lock:
            # 如果标定已结束，忽略指令
            if not self.calibration_active and msg.command_type != CalibrationCommand.CALIB_START:
                self.get_logger().warn('Received calibration command but calibration is not active')
                return
        try:
            if msg.command_type == CalibrationCommand.CALIB_START:  #开始标定
                self.start_calibration()
            elif msg.command_type == CalibrationCommand.CALIB_NEXT_POINT:  #移动到下一个点
                self.process_calibration_point(msg.robot_pose)
            elif msg.command_type == CalibrationCommand.CALIB_FINISHED:  #结束标定
                self.finish_calibration(success=True)
        except Exception as e:
            self.get_logger().error(f'Error in calibration callback: {str(e)}')
    
    def start_calibration(self):
        """开始标定流程"""
        with self.calibration_lock:
            self.calibration_active = True
            self.calibration_points = []
            self.current_calibration_index = 0
        self.get_logger().info('Calibration started')
        
    def process_calibration_point(self, robot_pose):
        """处理单个标定点"""
        if not self.calibration_active:
            self.get_logger().warn('Received calibration point but calibration not active')
            return
            
        # 模拟Aruco码识别 - 实际项目中替换为真实的识别代码
        aruco_detected = True  # 假设成功识别
        aruco_pixel = [320.0, 240.0]  # 模拟像素坐标
        
        if aruco_detected:
            # 保存点数据
            self.calibration_points.append((aruco_pixel, robot_pose))
            self.current_calibration_index += 1
            
            # 发送成功响应
            response = CalibrationResponse()
            response.response_type = CalibrationResponse.RES_SUCCESS  # 识别成功
            response.aruco_pixel_pose = aruco_pixel
            response.robot_pose = robot_pose
            self.calibration_pub.publish(response)
            
            if self.bShowRunInfo:
                self.get_logger().info(f'Calibration point {self.current_calibration_index}/{self.max_calibration_points} recorded')
                
            # 检查是否完成
            if self.current_calibration_index >= self.max_calibration_points:
                self.finish_calibration(success=True)
        else:
            # 发送失败响应
            response = CalibrationResponse()
            response.response_type = CalibrationResponse.RES_FAILURE  # 识别失败
            self.calibration_pub.publish(response)
            self.get_logger().error('Aruco detection failed for calibration point')
    
    def finish_calibration(self, success):
        """完成标定流程"""
        if not self.calibration_active:
            return
        
        result_saved = self.save_calibration_results() # 保存标定结果  
        # 根据结果发送不同的完成响应
        response = CalibrationResponse()
        if result_saved:
            response.response_type = CalibrationResponse.RES_COMPLETED_SUCCESS
            self.get_logger().info('Calibration completed successfully')
        else:
            response.response_type = CalibrationResponse.RES_COMPLETED_FAILED
            self.get_logger().error('Calibration completed with errors')

        # 发布完成响应
        self.calibration_pub.publish(response)
        # 结束标定状态
        self.calibration_active = False

    def save_calibration_results(self):
        """保存标定结果到文件"""
        try:
             # 计算标定矩阵 (实际项目中实现具体算法)
            if len(self.calibration_points) >= 3:
                # 这里添加实际标定矩阵计算代码
                calibration_matrix = self.calculate_calibration_matrix()  # 实现实际标定算法
                self.get_logger().info(f'Calibration completed with {len(self.calibration_points)} points')
                
                # 保存到文件
                import json
                from ament_index_python.packages import get_package_share_directory
                pkg_path = get_package_share_directory('udp_server')
                file_path = os.path.join(pkg_path, 'config', 'calibration_results.json')
                
                with open(file_path, 'w') as f:
                    json.dump({
                        'matrix': calibration_matrix,
                        'points': self.calibration_points,
                        'timestamp': time.time()
                    }, f)
                    
                self.get_logger().info(f'Saved calibration results to {file_path}')
                return True
            else:
                self.get_logger().error('Calibration failed: not enough points')
                return False
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration results: {str(e)}')
            return False
    def calculate_calibration_matrix(self):
        """计算标定矩阵（模拟）"""
        # 实际项目中实现具体算法
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ]

    def ai_coordinate_callback(self, msg):
        """AI坐标数据回调（保持JSON，因数据量小）"""
        try:
            data_dict = {
                'object_class': msg.object_class,
                'position': msg.position,
                'orientation': msg.orientation,
                'confidence': msg.confidence,
                'frame_id': msg.frame_id,
                'stamp': {
                    'sec': msg.stamp.sec,
                    'nanosec': msg.stamp.nanosec
                }
            }
            json_data = json.dumps(data_dict)
            payload = json_data.encode('utf-8')
            header = struct.pack('!I', MessageType.AI_COORDINATE)
            packet = header + payload
            
            # 添加重试机制和错误处理
            try:
                self.udp_socket.sendto(packet, self.server_address)
            except socket.timeout:
                if self.bShowRunInfo:
                    self.get_logger().warn('UDP send timeout, dropping AI coordinate data')
            except BlockingIOError:
                if self.bShowRunInfo:
                    self.get_logger().warn('UDP send buffer full, dropping AI coordinate data')
            except Exception as e:
                self.get_logger().error(f'Error sending AI coordinate data: {str(e)}')
            
            # 日志频率控制
            self.ai_log_counter += 1
             # 根据bShowRunInfo控制运行时的调试信息是否输出
            if self.bShowRunInfo and self.ai_log_counter % self.log_interval == 0:
                self.get_logger().info('Sent AI coordinate data via UDP')
            
        except Exception as e:
            self.get_logger().error(f'Error processing AI coordinate data: {str(e)}')   
    
    def laserscan_callback(self, msg):
        """雷达扫描数据回调（二进制协议）"""
        try:
            # 二进制打包（替代JSON）
            frame_id_bytes = msg.header.frame_id.encode('utf-8')
            frame_id_len = len(frame_id_bytes)
            
            # 格式: [angle_min(4f), angle_max(4f), angle_increment(4f), time_increment(4f), 
            #        scan_time(4f), range_min(4f), range_max(4f), frame_id_len(4I), 
            #        frame_id(str), stamp_sec(4I), stamp_nanosec(4I), 
            #        ranges_len(4I), ranges(var), intensities_len(4I), intensities(var)]
            packet = struct.pack('!7f', 
                                msg.angle_min, msg.angle_max, msg.angle_increment,
                                msg.time_increment, msg.scan_time,
                                msg.range_min, msg.range_max)
            packet += struct.pack('!I', frame_id_len)
            packet += frame_id_bytes
            packet += struct.pack('!II', msg.header.stamp.sec, msg.header.stamp.nanosec)
            
            # 处理ranges
            ranges_len = len(msg.ranges)
            packet += struct.pack('!I', ranges_len)
            if ranges_len > 0:
                packet += struct.pack(f'!{ranges_len}f', *msg.ranges)

            # 处理intensities
            intensities_len = len(msg.intensities) if msg.intensities else 0
            packet += struct.pack('!I', intensities_len)
            if intensities_len > 0:
                packet += struct.pack(f'!{intensities_len}f', *msg.intensities)
            
            # 添加消息头
            header = struct.pack('!I', MessageType.LASERSCAN)
            
            # 添加重试机制和错误处理
            try:
                self.udp_socket.sendto(header + packet, self.server_address)
            except socket.timeout:
                if self.bShowRunInfo:
                    self.get_logger().warn('UDP send timeout, dropping laser scan data')
            except BlockingIOError:
                if self.bShowRunInfo:
                    self.get_logger().warn('UDP send buffer full, dropping laser scan data')
            except Exception as e:
                self.get_logger().error(f'Error sending laser scan data: {str(e)}')
            
            # 日志频率控制
            self.laser_log_counter += 1
             # 根据bShowRunInfo控制运行时的调试信息是否输出
            if self.bShowRunInfo and self.laser_log_counter % self.log_interval == 0:
                self.get_logger().info('Sent laser scan data via UDP')
            
        except Exception as e:
            self.get_logger().error(f'Error processing laser scan data: {str(e)}')
  
    def pointcloud_callback(self, msg):
        """点云数据回调（二进制协议）"""
        try:
            # 二进制打包（替代JSON）
            # 关键优化: 直接使用msg.data（bytes），不转换为list
            frame_id_bytes = msg.frame_id.encode('utf-8')
            frame_id_len = len(frame_id_bytes)
            data_len = len(msg.data)
            
            # 数据分片（UDP包最大65KB）
            max_udp_size = 65507 - 100  # 预留头空间
            if data_len > max_udp_size:
                if self.bShowRunInfo:
                    self.get_logger().warn(f'Pointcloud too large ({data_len} bytes), truncating to {max_udp_size} bytes')
                point_data = msg.data[:max_udp_size]
                data_len = max_udp_size
            else:
                point_data = msg.data
            
            # 格式: [height(4I), width(4I), point_step(4I), stamp_sec(4I), stamp_nanosec(4I),
            #        camera_intrinsic(36f), frame_id_len(4I), frame_id(str), data_len(4I), data(var)]
            packet = struct.pack('!III', msg.height, msg.width, msg.point_step)
            packet += struct.pack('!II', msg.stamp.sec, msg.stamp.nanosec)
            packet += struct.pack('!9f', *msg.camera_intrinsic)  # 假设camera_intrinsic是9个float
            packet += struct.pack('!I', frame_id_len)
            packet += frame_id_bytes
            packet += struct.pack('!I', data_len)
            packet += point_data
            
            # 添加消息头并发送
            header = struct.pack('!I', MessageType.POINTCLOUD)
            
            # 添加重试机制和错误处理
            try:
                self.udp_socket.sendto(header + packet, self.server_address)
            except socket.timeout:
                if self.bShowRunInfo:
                    self.get_logger().warn('UDP send timeout, dropping pointcloud data')
            except BlockingIOError:
                if self.bShowRunInfo:
                    self.get_logger().warn('UDP send buffer full, dropping pointcloud data')
            except Exception as e:
                self.get_logger().error(f'Error sending pointcloud data: {str(e)}')
            
            # 日志频率控制
            self.pointcloud_log_counter += 1
             # 根据bShowRunInfo控制运行时的调试信息是否输出
            if self.bShowRunInfo and self.pointcloud_log_counter % self.log_interval == 0:
                self.get_logger().info('Sent pointcloud data via UDP')
            
        except Exception as e:
            self.get_logger().error(f'Error processing pointcloud data: {str(e)}')

    def detection_image_callback(self, msg):
        """检测图像数据回调（PNG base64格式）"""
        try:
            # 创建JSON格式的数据
            data_dict = {
                'image_data': msg.data  # base64编码的PNG图像
            }
            
            json_data = json.dumps(data_dict)
            payload = json_data.encode('utf-8')
            
            # 添加消息头并发送
            header = struct.pack('!I', MessageType.DETECTION_IMAGE)
            packet = header + payload
            
            # 添加重试机制和错误处理
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    self.udp_socket.sendto(packet, self.server_address)
                    
                    # 日志频率控制
                    self.log_counter += 1
                    if self.bShowRunInfo and self.log_counter % self.log_interval == 0:
                        self.get_logger().info(f'Sent detection image data to {self.server_address}')
                    break
                except socket.error as e:
                    if attempt == max_retries - 1:
                        self.get_logger().error(f'Failed to send detection image data after {max_retries} attempts: {str(e)}')
                    else:
                        self.get_logger().warning(f'Attempt {attempt + 1} failed, retrying...')
                        time.sleep(0.1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing detection image data: {str(e)}')

    def detection_result_callback(self, msg):
        """检测结果数据回调（JSON格式）"""
        try:
            # 创建JSON格式的数据
            data_dict = {
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                },
                'detections': []
            }
            
            # 解析检测结果
            for detection in msg.detections:
                det_data = {
                    'bbox': {
                        'center': {
                            'position': {
                                'x': detection.bbox.center.position.x,
                                'y': detection.bbox.center.position.y
                            }
                        },
                        'size_x': detection.bbox.size_x,
                        'size_y': detection.bbox.size_y
                    },
                    'results': []
                }
                
                # 检测结果
                for hypothesis in detection.results:
                    hyp_data = {
                        'hypothesis': {
                            'class_id': hypothesis.hypothesis.class_id,
                            'score': hypothesis.hypothesis.score
                        },
                        'pose': {
                            'pose': {
                                'position': {
                                    'x': hypothesis.pose.pose.position.x,
                                    'y': hypothesis.pose.pose.position.y,
                                    'z': hypothesis.pose.pose.position.z
                                },
                                'orientation': {
                                    'x': hypothesis.pose.pose.orientation.x,
                                    'y': hypothesis.pose.pose.orientation.y,
                                    'z': hypothesis.pose.pose.orientation.z,
                                    'w': hypothesis.pose.pose.orientation.w
                                }
                            }
                        }
                    }
                    det_data['results'].append(hyp_data)
                
                data_dict['detections'].append(det_data)
            
            json_data = json.dumps(data_dict)
            payload = json_data.encode('utf-8')
            
            # 添加消息头并发送
            header = struct.pack('!I', MessageType.DETECTION_RESULT)
            packet = header + payload
            
            # 添加重试机制和错误处理
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    self.udp_socket.sendto(packet, self.server_address)
                    
                    # 日志频率控制
                    self.log_counter += 1
                    if self.bShowRunInfo and self.log_counter % self.log_interval == 0:
                        self.get_logger().info(f'Sent detection result data to {self.server_address}')
                    break
                except socket.error as e:
                    if attempt == max_retries - 1:
                        self.get_logger().error(f'Failed to send detection result data after {max_retries} attempts: {str(e)}')
                    else:
                        self.get_logger().warning(f'Attempt {attempt + 1} failed, retrying...')
                        time.sleep(0.1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing detection result data: {str(e)}')

    def interactive_command_callback(self, msg):
        """交互指令数据回调（JSON格式）"""
        try:
            # 创建JSON格式的数据
            data_dict = {
                'act_command_type': msg.act_command_type,
                'data': msg.data,
                'pose': {
                    'header': {
                        'stamp': {
                            'sec': msg.pose.header.stamp.sec,
                            'nanosec': msg.pose.header.stamp.nanosec
                        },
                        'frame_id': msg.pose.header.frame_id
                    },
                    'pose': {
                        'position': {
                            'x': msg.pose.pose.position.x,
                            'y': msg.pose.pose.position.y,
                            'z': msg.pose.pose.position.z
                        },
                        'orientation': {
                            'x': msg.pose.pose.orientation.x,
                            'y': msg.pose.pose.orientation.y,
                            'z': msg.pose.pose.orientation.z,
                            'w': msg.pose.pose.orientation.w
                        }
                    }
                }
            }
            
            json_data = json.dumps(data_dict)
            payload = json_data.encode('utf-8')
            
            # 添加消息头并发送
            header = struct.pack('!I', MessageType.INTERACTIVE_COMMAND)
            packet = header + payload
            
            # 添加重试机制和错误处理
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    self.udp_socket.sendto(packet, self.server_address)
                    
                    # 日志频率控制
                    self.log_counter += 1
                    if self.bShowRunInfo and self.log_counter % self.log_interval == 0:
                        self.get_logger().info(f'Sent interactive command data to {self.server_address}')
                    break
                except socket.error as e:
                    if attempt == max_retries - 1:
                        self.get_logger().error(f'Failed to send interactive command data after {max_retries} attempts: {str(e)}')
                    else:
                        self.get_logger().warning(f'Attempt {attempt + 1} failed, retrying...')
                        time.sleep(0.1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing interactive command data: {str(e)}')

    def destroy_node(self):
        self.is_running = False

        # 等待接收线程结束
        if self.receive_thread.is_alive():
            self.receive_thread.join()

        # 立即停止所有项目
        if hasattr(self, 'project_manager'):
            for project_name in list(self.project_manager.PROJECTS.keys()):
                try:
                    self.get_logger().info(f"Stopping project {project_name}...")
                    self.project_manager.stop_project(project_name)
                except Exception as e:
                    self.get_logger().error(f"Error stopping project {project_name}: {str(e)}")

        # 关闭UDP套接字（这会中断recvfrom阻塞）
        try:
            if hasattr(self, 'udp_socket'):
                self.udp_socket.close()
        except Exception as e:
            self.get_logger().error(f"Error closing UDP socket: {str(e)}")
        
        # 等待接收线程结束（设置超时）
        if hasattr(self, 'receive_thread') and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=5.0)  # 最多等待5秒
            if self.receive_thread.is_alive():
                self.get_logger().warn("Receive thread did not terminate gracefully")

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UDPClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
