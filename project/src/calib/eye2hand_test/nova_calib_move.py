#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
越疆 Nova2 机械臂 - 眼在手外(Eye-to-Hand)实时移动控制脚本
基于 Nova2 机械臂控制接口实现
支持固件版本: 3.5.8.1 及以上

功能：
1. 相机采图显示
2. 实时输出Aruco码识别结果显示
3. 根据Aruco码识别后的标识坐标，和加载的标定矩阵，换算成对应的机械手坐标
4. 指定机械手移动到换算的位置去
"""

import sys
import os
import json
import numpy as np
import time

# 尝试导入cv2
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    cv2 = None
    CV2_AVAILABLE = False

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 导入外部封装的类
from nova_robot import Nova2Robot
from camsdk_realsense import CamSDK_Realsense
from aruco_detector import Aruco_Detector

# 从其他文件导入需要的函数
def load_calibration_matrix(save_dir="save_parms"):
    """
    加载标定矩阵
    
    Args:
        save_dir: 保存目录
        
    Returns:
        numpy.ndarray: 相机到基座的变换矩阵
    """
    import numpy as np
    import os
    cam2base_file = os.path.join(save_dir, "camera2base.npy")
    if not os.path.exists(cam2base_file):
        raise FileNotFoundError(f"标定矩阵文件不存在: {cam2base_file}")
    
    T_cam2base = np.load(cam2base_file)
    return T_cam2base

class NovaCalibMove:
    """Nova2 机械臂实时移动控制类"""
    
    def __init__(self, robot_ip: str = "192.168.5.1", visualize: bool = True):
        """
        初始化实时移动控制系统
        
        Args:
            robot_ip: 机械臂IP地址
            visualize: 是否启用可视化窗口
        """
        self.robot_ip = robot_ip
        self.visualize = visualize
        
        # 初始化机械臂
        self.robot = Nova2Robot(ip_address=robot_ip)
        
        # 初始化相机
        self.camera_sdk = CamSDK_Realsense()
        
        # 初始化Aruco检测器
        self.aruco_detector = Aruco_Detector(marker_length=0.1)  # 100mm标记
        
        # 标定矩阵
        self.T_cam2base = None
        
        # 检查OpenCV是否可用
        if not CV2_AVAILABLE and visualize:
            print("警告: OpenCV不可用，禁用可视化")
            self.visualize = False
        
        # 可视化窗口
        self.window_name = "Aruco实时检测与机械臂控制"
        
        # 控制参数
        self.marker_offset_z = 0  # 标记在末端执行器下方的偏移量(mm)
        self.printDbgInfo = False
        
        # 夹爪顶端中心坐标的偏差补偿值（基于手动示教校准）
        # 计算坐标 -> 实际示教坐标的偏差
        self.gripper_offset_x = 0.85    # X轴偏差补偿 (mm)
        self.gripper_offset_y = -139.47 # Y轴偏差补偿 (mm)
        self.gripper_offset_z = 122.52  # Z轴偏差补偿 (mm)
        
    def load_calibration(self, save_dir="save_parms"):
        """
        加载标定矩阵
        
        Args:
            save_dir: 保存目录
            
        Returns:
            bool: 加载成功返回True，失败返回False
        """
        try:
            cam2base_file = os.path.join(save_dir, "camera2base.npy")
            if not os.path.exists(cam2base_file):
                print(f"❌ 标定矩阵文件不存在: {cam2base_file}")
                return False
            
            self.T_cam2base = np.load(cam2base_file)
            print("✅ 标定矩阵加载成功")
            print(f"   矩阵形状: {self.T_cam2base.shape}")
            return True
        except Exception as e:
            print(f"❌ 标定矩阵加载失败: {e}")
            return False
    
    def initialize_system(self):
        """
        初始化系统（相机、机械臂等）
        
        Returns:
            bool: 初始化成功返回True，失败返回False
        """
        try:
            # 1. 初始化相机
            print("1. 初始化RealSense相机...")
            self.pipeline, profile = self.camera_sdk.initialize_camera(width=1280, height=720, fps=30)
            
            # 获取相机内参
            cam_matrix, dist, intrinsics = self.camera_sdk.get_cam_intrinsics()
            if cam_matrix is None:
                print("❌ 无法获取相机内参")
                return False
            
            # 设置相机内参到aruco_detector
            self.aruco_detector.set_camera_intrinsics(cam_matrix, dist)
            print("✅ RealSense相机初始化成功")
            
            # 2. 连接机械臂
            print("2. 连接机械臂...")
            if not self.robot.connect():
                print("❌ 机械臂连接失败")
                return False
            
            if not self.robot.enable_robot():
                print("❌ 机械臂使能失败")
                return False
            
            print("✅ 机械臂连接成功")
            
            # 3. 加载标定矩阵
            print("\n3. 加载标定矩阵...")
            if not self.load_calibration():
                print("❌ 标定矩阵加载失败")
                return False
            
            # 3.1 显示偏差补偿值
            print("\n3.1 夹爪顶端坐标偏差补偿值:")
            print(f"   X轴偏差: {self.gripper_offset_x} mm")
            print(f"   Y轴偏差: {self.gripper_offset_y} mm") 
            print(f"   Z轴偏差: {self.gripper_offset_z} mm")
            
            # 4. 创建可视化窗口
            if self.visualize and cv2 is not None:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 1280, 720)
                print("✅ 可视化窗口创建成功")
            
            # 5. 移动到标准位置
            print("4. 移动到标准位置...")
            if not self.robot.move_to_standard_pose():
                print("⚠️  移动到标准位置失败（继续执行）")
            else:
                print("✅ 已移动到标准位置")
            
            return True
            
        except Exception as e:
            print(f"❌ 系统初始化失败: {e}")
            return False
    
    def detect_aruco_marker(self, color_image):
        """
        检测Aruco标记并返回其在相机坐标系下的3D坐标
        
        Args:
            color_image: 彩色图像
            
        Returns:
            tuple: (marker_position, rendered_image, marker_info) 标记位置、渲染后的图像和标记详细信息
        """
        try:
            # 检测ArUco标记
            result = self.aruco_detector.detect_and_process_markers(
                frame=color_image,
                draw_results=True,
                print_results=False
            )
            
            marker_position = None
            marker_info = None
            if result['found'] and len(result['markers_info']) > 0:
                # 使用第一个检测到的标记的位置信息
                marker_info = result['markers_info'][0]
                if marker_info.get('position') is not None:
                    marker_position = list(marker_info['position'])
            
            rendered_image = result.get('frame') if 'frame' in result else None
            return marker_position, rendered_image, marker_info
            
        except Exception as e:
            print(f"检测ArUco标记失败: {e}")
            return None, None, None
    
    def convert_camera_to_robot(self, marker_position):
        """
        将相机坐标系下的标记位置转换为机器人基座坐标系下的末端位置
        
        Args:
            marker_position: 标记在相机坐标系中的位置 (x, y, z) (单位: 米)
            
        Returns:
            tuple: (calc_x, calc_y, calc_z, corrected_x, corrected_y, corrected_z) 
                   计算坐标和修正后的夹爪顶端中心坐标 (单位: 毫米)
        """
        try:
            # 将标记位置从相机坐标系转换到基座坐标系
            marker_cam = np.array([*marker_position, 1.0])
            marker_base = self.T_cam2base @ marker_cam
            marker_base_mm = marker_base[:3] #* 1000  # 转换为毫米
            
            # 计算末端执行器位置（标记在末端下方marker_offset_z mm处）
            calc_x = marker_base_mm[0]
            calc_y = marker_base_mm[1]
            calc_z = marker_base_mm[2] + self.marker_offset_z
            
            # 应用偏差补偿值，得到实际夹爪顶端中心坐标
            corrected_x = calc_x + self.gripper_offset_x
            corrected_y = calc_y + self.gripper_offset_y
            corrected_z = calc_z + self.gripper_offset_z
            
            return (calc_x, calc_y, calc_z, corrected_x, corrected_y, corrected_z)
            
        except Exception as e:
            print(f"坐标转换失败: {e}")
            return None
    
    def move_robot_to_position(self, x, y, z, rx=-179.98, ry=0.005, rz=90.07):
        """
        控制机械臂移动到指定位置
        
        Args:
            x, y, z: 目标位置 (mm)
            rx, ry, rz: 目标姿态角 (度)
            
        Returns:
            bool: 移动成功返回True，失败返回False
        """
        try:
            print(f"移动机械臂到位置: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            success = self.robot.move_l(x, y, z, rx, ry, rz)
            
            if success:
                print("✅ 移动成功")
            else:
                print("❌ 移动失败")
            
            return success
            
        except Exception as e:
            print(f"机械臂移动失败: {e}")
            return False
    
    def run_realtime_control(self):
        """
        运行实时控制循环
        """
        print("开始实时控制循环...")
        print("按 'q' 或 ESC 键退出")
        print("按 's' 键移动机械臂到检测到的Aruco标记位置")
        print("按 'h' 键返回初始位置")
        print("-" * 50)
        
        try:
            while True:
                # 获取相机帧
                color_image, depth_frame, color_frame = self.camera_sdk.get_frames(timeout_ms=2000)
                
                if color_image is None:
                    print("警告: 无法获取相机帧")
                    continue
                
                # 检测Aruco标记
                marker_position, rendered_image, marker_info = self.detect_aruco_marker(color_image)
                
                # 显示结果
                if self.visualize and rendered_image is not None and cv2 is not None:
                    # 添加状态信息到图像
                    if marker_position is not None and marker_info is not None:
                        # 直接使用从detect_aruco_marker函数返回的marker_info信息
                        
                        if self.printDbgInfo:
                            print(f"DEBUG: marker_info = {marker_info}")

                        # 显示Aruco码的像素坐标和位姿信息
                        center_2d = marker_info.get('center_2d', None)
                        position = marker_info.get('position', None)
                        rotation = marker_info.get('rotation', None)
                        
                        if self.printDbgInfo:
                            print(f"DEBUG: center_2d = {center_2d}")
                            print(f"DEBUG: position = {position}")
                            print(f"DEBUG: rotation = {rotation}")
                        
                        # 显示像素坐标
                        if center_2d is not None:
                            cv2.putText(
                                rendered_image,
                                f"Aruco Pixel Coords: X={center_2d[0]:.1f}, Y={center_2d[1]:.1f}",
                                (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7,
                                (0, 255, 255),
                                2
                            )
                        
                        # 显示相机坐标系下的3D位置和旋转
                        if position is not None and rotation is not None:
                            cv2.putText(
                                rendered_image,
                                f"Aruco 3D (Cam): X={position[0]:.3f}, Y={position[1]:.3f}, Z={position[2]:.3f} (m)",
                                (20, 70),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7,
                                (0, 255, 0),
                                2
                            )
                            cv2.putText(
                                rendered_image,
                                f"Aruco Rot (Cam): Rx={rotation[0]:.1f}, Ry={rotation[1]:.1f}, Rz={rotation[2]:.1f} (deg)",
                                (20, 100),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7,
                                (0, 128, 255),
                                2
                            )
                            
                            # 基于标定矩阵计算机器人末端执行器位姿
                            robot_pose = self.convert_camera_to_robot(position)
                            if self.printDbgInfo:
                                print(f"DEBUG: robot_pose = {robot_pose}")  # 添加调试信息
                            if robot_pose is not None:
                                # 显示计算得到的坐标（修正前）
                                cv2.putText(
                                    rendered_image,
                                    f"Robot Pose (Calc): X={robot_pose[0]:.2f}, Y={robot_pose[1]:.2f}, Z={robot_pose[2]:.2f} (mm)",
                                    (20, 130),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7,
                                    (255, 0, 0),
                                    2
                                )
                                # 显示修正后的夹爪顶端中心坐标
                                cv2.putText(
                                    rendered_image,
                                    f"Gripper Center: X={robot_pose[3]:.2f}, Y={robot_pose[4]:.2f}, Z={robot_pose[5]:.2f} (mm)",
                                    (20, 160),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7,
                                    (0, 255, 0),
                                    2
                                )
                            
                            # 获取当前机械手实际位姿坐标
                            current_pose = self.robot.get_pose()
                            if self.printDbgInfo:
                                print(f"DEBUG: current_pose = {current_pose}")  # 添加调试信息
                            if current_pose and len(current_pose) >= 6:
                                cv2.putText(
                                    rendered_image,
                                    f"Robot Pose (Actual): X={current_pose[0]:.2f}, Y={current_pose[1]:.2f}, Z={current_pose[2]:.2f} (mm)",
                                    (20, 190),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7,
                                    (0, 0, 255),
                                    2
                                )
                                cv2.putText(
                                    rendered_image,
                                    f"Robot Rot (Actual): Rx={current_pose[3]:.2f}, Ry={current_pose[4]:.2f}, Rz={current_pose[5]:.2f} (deg)",
                                    (20, 220),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7,
                                    (255, 0, 255),
                                    2
                                )
                    else:
                        cv2.putText(
                            rendered_image,
                            "No marker detected",
                            (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 0, 255),
                            2
                        )
                    
                    cv2.imshow(self.window_name, rendered_image)
                
                # 键盘控制
                key = None
                if cv2 is not None:
                    key = cv2.waitKey(1) & 0xFF
                
                if key is not None and (key == ord('q') or key == 27):  # 'q' 或 ESC 键退出
                    break
                elif key is not None and key == ord('s'):  # 's' 键移动机械臂
                    if marker_position is not None:
                        # 转换坐标
                        robot_pose = self.convert_camera_to_robot(marker_position)
                        if robot_pose is not None:
                            calc_x, calc_y, calc_z, corrected_x, corrected_y, corrected_z = robot_pose
                            print(f"计算坐标: X={calc_x:.2f}, Y={calc_y:.2f}, Z={calc_z:.2f} (mm)")
                            print(f"修正坐标: X={corrected_x:.2f}, Y={corrected_y:.2f}, Z={corrected_z:.2f} (mm)")
                            # 移动机械臂到修正后的夹爪顶端中心坐标
                            self.move_robot_to_position(corrected_x, corrected_y, corrected_z)
                        else:
                            print("❌ 坐标转换失败")
                    else:
                        print("❌ 未检测到Aruco标记")
                elif key is not None and key == ord('h'):  # 'h' 键返回初始位置
                    print("返回初始位置...")
                    if self.robot.move_to_standard_pose():
                        print("✅ 已返回初始位置")
                    else:
                        print("❌ 返回初始位置失败")
                
        except KeyboardInterrupt:
            print("用户中断程序")
        except Exception as e:
            print(f"实时控制过程中出错: {e}")
            import traceback
            traceback.print_exc()
    
    def cleanup(self):
        """释放资源"""
        print("释放资源...")
        
        # 断开机械臂连接
        try:
            self.robot.disconnect()
            print("✅ 机械臂连接已断开")
        except Exception as e:
            print(f"断开机械臂连接时出错: {e}")
        
        # 停止相机
        try:
            self.camera_sdk.stop_camera()
            print("✅ 相机已停止")
        except Exception as e:
            print(f"停止相机时出错: {e}")
        
        # 关闭窗口
        try:
            if self.visualize and cv2 is not None:
                cv2.destroyAllWindows()
            print("✅ 窗口已关闭")
        except Exception as e:
            print(f"关闭窗口时出错: {e}")
        
        print("资源释放完成")

def main():
    """主函数"""
    print("越疆 Nova2 机械臂 - 眼在手外实时移动控制")
    print("=" * 50)
    
    # 创建控制实例
    controller = NovaCalibMove(robot_ip="192.168.5.1", visualize=True)
    
    try:
        # 初始化系统
        if not controller.initialize_system():
            print("❌ 系统初始化失败")
            return
        
        # 运行实时控制
        controller.run_realtime_control()
        
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"程序执行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 释放资源
        controller.cleanup()

if __name__ == "__main__":
    main()