#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
越疆 Nova2 机械臂控制类
基于 Nova2 机械臂控制接口实现
支持固件版本: 3.5.8.1 及以上
"""

import socket
import logging
import time
import json


class Nova2Robot:
    """越疆 Nova2 机械臂控制类（模块化封装版）"""
    
    def __init__(self, ip_address: str = "192.168.5.1"):
        """初始化 Nova2 机械臂
        
        Args:
            ip_address: 机械臂IP地址，默认为"192.168.5.1"
        """
        self.ip_address = ip_address
        self.sock_29999 = None
        self.is_connected = False
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # 机械手标准位置和移动范围
        self.standard_pose = [-163.0518, -325.5861, 262.9900, -179.9815, 0.0051, 90.0711]
        self.x_max = 273.0518
        self.x_min = -373.0518
        self.y_max = -195.5861
        self.y_min = -495.5861
        self.z_max = 373.00
        self.z_min = 93.000
    
    def connect(self, timeout: int = 5) -> bool:
        """连接到机械臂
        
        Args:
            timeout: 连接超时时间（秒），默认为5秒
            
        Returns:
            bool: 连接成功返回True，失败返回False
        """
        try:
            self.logger.info(f"连接到 Nova2 机械臂: {self.ip_address}")
            self.sock_29999 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock_29999.settimeout(timeout)
            self.sock_29999.connect((self.ip_address, 29999))
            self.logger.info("✅ 端口 29999 连接成功")
            self.is_connected = True
            return True
        except Exception as e:
            self.logger.error(f"❌ 连接失败: {e}")
            self.disconnect()
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.sock_29999:
            try:
                self.sock_29999.close()
            except:
                pass
            self.sock_29999 = None
        self.is_connected = False
        self.logger.info("已断开连接")
    
    def _send_command(self, command: str) -> str:
        """发送命令到机械臂
        
        Args:
            command: 要发送的命令字符串
            
        Returns:
            str: 机械臂返回的响应字符串
        """
        if not self.is_connected:
            self.logger.error("未连接到机械臂")
            return ""
        try:
            self.sock_29999.sendall(command.encode('utf-8'))
            response = self.sock_29999.recv(1024).decode('utf-8', errors='ignore')
            return response
        except Exception as e:
            self.logger.error(f"发送命令失败: {e}")
            return ""
    
    def enable_robot(self) -> bool:
        """使能机械臂
        
        Returns:
            bool: 使能成功返回True，失败返回False
        """
        self.logger.info("使能机械臂...")
        response = self._send_command("EnableRobot()")
        if response:
            self.logger.info(f"✅ 机械臂已使能")
            return True
        return False
    
    def disable_robot(self) -> bool:
        """去使能机械臂
        
        Returns:
            bool: 去使能成功返回True，失败返回False
        """
        self.logger.info("去使能机械臂...")
        response = self._send_command("DisableRobot()")
        if response:
            self.logger.info(f"✅ 机械臂已去使能")
            return True
        return False
    
    def get_pose(self) -> list:
        """获取当前笛卡尔坐标 [X, Y, Z, Rx, Ry, Rz]
        
        Returns:
            list: 包含6个元素的列表[X, Y, Z, Rx, Ry, Rz]，获取失败返回空列表
        """
        response = self._send_command("GetPose()")
        if not response:
            return []
        try:
            start = response.find('{')
            end = response.find('}')
            if start == -1 or end == -1:
                return []
            pose_str = response[start+1:end]
            pose = [float(x.strip()) for x in pose_str.split(',')]
            if len(pose) != 6:
                return []
            return pose
        except Exception as e:
            self.logger.error(f"解析坐标失败: {e}")
            return []
    
    def move_to_standard_pose(self) -> bool:
        """移动到标准位置
        
        Returns:
            bool: 移动成功返回True，失败返回False
        """
        self.logger.info("移动到标准位置...")
        return self.move_l(
            self.standard_pose[0],
            self.standard_pose[1],
            self.standard_pose[2],
            self.standard_pose[3],
            self.standard_pose[4],
            self.standard_pose[5]
        )
    
    def is_position_valid(self, x: float, y: float, z: float) -> bool:
        """检查位置是否在允许范围内
        
        Args:
            x, y, z: 要检查的笛卡尔坐标 (mm)
            
        Returns:
            bool: 位置有效返回True，超出范围返回False
        """
        if x > self.x_max or x < self.x_min:
            self.logger.error(f"X坐标超出范围: {x} (允许范围: {self.x_min} ~ {self.x_max})")
            return False
        if y > self.y_max or y < self.y_min:
            self.logger.error(f"Y坐标超出范围: {y} (允许范围: {self.y_min} ~ {self.y_max})")
            return False
        if z > self.z_max or z < self.z_min:
            self.logger.error(f"Z坐标超出范围: {z} (允许范围: {self.z_min} ~ {self.z_max})")
            return False
        return True
    
    def move_l(self, x: float, y: float, z: float, rx: float = 180, ry: float = 0, rz: float = 90, wait: bool = True) -> bool:
        """直线运动到目标位置
        
        Args:
            x, y, z: 目标笛卡尔坐标 (mm)
            rx, ry, rz: 姿态角 (度)，默认为180, 0, 90
            wait: 是否等待运动完成，默认为True
            
        Returns:
            bool: 移动成功返回True，失败返回False
        """
        # 检查目标位置是否在允许范围内
        if not self.is_position_valid(x, y, z):
            self.logger.error(f"目标位置超出允许范围，拒绝移动。目标位置：x={x:.2f}，y={y:.2f}，z={z:.2f}，rx={rx:.2f}，ry={ry:.2f}，rz={rz:.2f}")
            return False
        
        # 先获取当前位置
        start_pose = self.get_pose()
        if start_pose is None:
            self.logger.error("无法获取起始位置")
            return False
        
        self.logger.debug(f"起始位置: X={start_pose[0]:.1f}, Y={start_pose[1]:.1f}, Z={start_pose[2]:.1f}")
        self.logger.debug(f"目标位置: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")
        
        # 发送移动命令
        command = f"MovL({{{x:.2f},{y:.2f},{z:.2f},{rx:.2f},{ry:.2f},{rz:.2f}}})"
        response = self._send_command(command)
        if response is None:
            self.logger.error("MovL命令发送失败")
            return False
        
        # 检查响应中是否有错误信息
        if "error" in response.lower() or "alarm" in response.lower():
            self.logger.error(f"MovL命令返回错误: {response}")
            return False
        
        if not wait:
            return True
        
        # 等待运动完成：检查是否到达目标位置
        time.sleep(0.5)  # 等待运动启动
        max_wait = 30  # 最大等待30秒
        tolerance = 2.0  # 位置容差 2mm
        
        # 记录是否有移动
        has_moved = False
        
        for i in range(max_wait * 10):  # 每100ms检查一次
            current_pose = self.get_pose()
            if current_pose is None:
                time.sleep(0.1)
                continue
            
            # 检查是否开始移动
            if not has_moved:
                move_dist = (
                    (current_pose[0] - start_pose[0])**2 + 
                    (current_pose[1] - start_pose[1])**2 + 
                    (current_pose[2] - start_pose[2])**2
                )**0.5
                if move_dist > 1.0:  # 移动超过1mm
                    has_moved = True
                    self.logger.debug(f"机械臂开始移动，已移动 {move_dist:.1f}mm")
            
            # 检查XYZ位置误差
            dx = abs(current_pose[0] - x)
            dy = abs(current_pose[1] - y)
            dz = abs(current_pose[2] - z)
            
            if dx < tolerance and dy < tolerance and dz < tolerance:
                # 已到达目标位置
                self.logger.debug(f"✅ 已到达目标位置，误差: dX={dx:.2f}, dY={dy:.2f}, dZ={dz:.2f}mm")
                return True
            
            time.sleep(0.1)
        
        # 超时检查
        final_pose = self.get_pose()
        if final_pose:
            final_dx = abs(final_pose[0] - x)
            final_dy = abs(final_pose[1] - y)
            final_dz = abs(final_pose[2] - z)
            
            if not has_moved:
                self.logger.error(f"❌ 机械臂未移动！")
                self.logger.error(f"   目标位置: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")
                self.logger.error(f"   当前位置: X={final_pose[0]:.1f}, Y={final_pose[1]:.1f}, Z={final_pose[2]:.1f}")
                self.logger.error(f"   可能原因: 运动范围超限、碰撞检测、机械臂未使能")
                return False
            else:
                self.logger.warning(f"移动目标位置: X={x:.1f}, Y={y:.1f}, Z={z:.1f} 超时")
                self.logger.warning(f"实际当前位置: X={final_pose[0]:.1f}, Y={final_pose[1]:.1f}, Z={final_pose[2]:.1f}")
                self.logger.warning(f"位置误差: dX={final_dx:.1f}, dY={final_dy:.1f}, dZ={final_dz:.1f}mm")
                # 虽然超时但已经移动了，继续执行
                return True
        
        return False
    
    def clear_error(self) -> bool:
        """清除错误
        
        Returns:
            bool: 清除成功返回True，失败返回False
        """
        response = self._send_command("ClearError()")
        return response is not None


# 使用示例
if __name__ == "__main__":
    # 配置日志
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 创建机械臂实例
    robot = Nova2Robot(ip_address="192.168.5.1")
    
    try:
        # 连接机械臂
        if not robot.connect():
            print("连接机械臂失败")
            exit(1)
        
        # 使能机械臂
        if not robot.enable_robot():
            print("使能机械臂失败")
            exit(1)
        
        # 移动到标准位置
        if not robot.move_to_standard_pose():
            print("移动到标准位置失败")
            exit(1)
        
        # 获取当前位置
        pose = robot.get_pose()
        if pose:
            print(f"当前位置: X={pose[0]:.2f}, Y={pose[1]:.2f}, Z={pose[2]:.2f}")
        
        # 直线移动到指定位置
        success = robot.move_l(-200, -400, 200)
        if success:
            print("移动成功")
        else:
            print("移动失败")
        
        # 获取移动后的位置
        pose = robot.get_pose()
        if pose:
            print(f"移动后位置: X={pose[0]:.2f}, Y={pose[1]:.2f}, Z={pose[2]:.2f}")
            
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 断开连接
        robot.disconnect()