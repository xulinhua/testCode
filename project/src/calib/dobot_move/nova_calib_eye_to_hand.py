#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
越疆 Nova2 机械臂 - 眼在手外(Eye-to-Hand)手眼标定脚本
基于 Nova2 机械臂控制接口实现
支持固件版本: 3.5.8.1 及以上
"""

import sys
import os
import argparse
import logging
import time
import random
import json
import threading

import numpy as np
import cv2
import pyrealsense2 as rs

# 导入外部封装的类
from nova_robot import Nova2Robot
from camsdk_realsense import CamSDK_Realsense
from aruco_detector import Aruco_Detector

# ========== 配置日志 ==========
logging.basicConfig(
    level=logging.DEBUG,  # 使用DEBUG级别显示详细信息
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# ========== 生成标定点 ==========
def generate_cali_points(xy_offsets, z_vals):
    """生成标定点列表"""
    return [(x, y, z) for z in z_vals for (x, y) in xy_offsets]


class NovaCalibration:
    """Nova2 机械臂眼在手外标定类"""
    
    def __init__(self, robot_ip: str, visualize: bool = False, use_file_pos: bool = False, wait_for_key: bool = True, use_calibration_file: bool = False):
        """
        初始化标定系统
        
        Args:
            robot_ip: 机械臂IP地址
            visualize: 是否启用可视化窗口
            use_file_pos: 是否使用保存的坐标点数据文件
            wait_for_key: 是否等待空格键继续识别，默认为True
            use_calibration_file: 是否从文件加载标定结果，默认为False
        """
        self.logger = logging.getLogger(self.__class__.__name__)
        self.use_file_pos = use_file_pos
        self.wait_for_key = wait_for_key  # 新增变量控制是否等待按键
        self.move_orientation = True  # 新增变量控制是否移动标定点时改变rx/ry/rz，默认为True
        self.use_calibration_file = use_calibration_file  # 新增变量控制是否从文件加载标定结果
        # 坐标点数据文件保存在save_parms目录下
        save_dir = "save_parms"
        os.makedirs(save_dir, exist_ok=True)
        self.calibration_points_file = os.path.join(save_dir, "calibration_points.json")
        self.logger.info(f"标定点数据文件路径: {self.calibration_points_file}")
        
        # 控制是否保存源图的变量，默认为True

        self.save_source_images = True
        
        # 标定参数
        self.marker_size = 0.1       # m，ArUco标记尺寸（100mm）
        self.marker_id = 0           # ArUco标记ID
        
        # 标定板中心相对于夹爪中心的偏移量（机械臂基座标系）
        # 根据用户最新描述更新偏移量：
        # 用户描述：我站在机械臂基座位置同时面向工作区域
        # 1) 标定板Aruco码中心相对于机械臂X轴正方向偏移3mm（即靠机械臂旋转轴中心或夹爪水平中心靠左边3mm）
        #    所以标定板在X轴负方向3mm处
        # 2) 标定板Aruco码中心相对于机械臂Y负方向偏移135mm（即靠机械臂旋转轴中心或夹爪水平中心靠远离我的方向135mm，即向前方移动）
        #    所以标定板在Y轴负方向135mm处
        # 3) 标定板Aruco码中心平面在距离夹爪末端上方的111mm处
        #    所以标定板在Z轴正方向11.1mm处
        self.marker_offset_x = -1.0     # mm，X轴负方向偏移（左侧）
        self.marker_offset_y = -135.0   # mm，Y轴负方向偏移（前方）
        #self.marker_offset_z = 111      # mm，Z轴正方向偏移（上方）
        self.marker_offset_z = -124 #mm,机械手末端面到Aruco面的距离
        
        self.marker_offset_x = 0.0     # mm，X轴负方向偏移（左侧）
        self.marker_offset_y = 0.0   # mm，Y轴负方向偏移（前方）
        self.marker_offset_z = 0.0 #mm,机械手末端面到Aruco面的距离

        # 更新描述信息
        self.logger.info(f"标定板相对于夹爪的偏移量已更新:")
        self.logger.info(f"  X轴偏移: {self.marker_offset_x} mm (X轴负方向，表示左侧)")
        self.logger.info(f"  Y轴偏移: {self.marker_offset_y} mm (Y轴负方向，表示前方)")
        self.logger.info(f"  Z轴偏移: {self.marker_offset_z} mm (Z轴正方向，表示上方)")
        
        # ========== 标定精度模式配置 ==========
        # 模式选择："NORMAL" - 常规模式（快速，231点）, "HIGH_PRECISION" - 高精度模式（慢速，455点）
        self.calibration_mode = "NORMAL"  # 👈 修改此处切换模式
        
        # 标定点配置：使用相对偏移量代替绝对位置
        self.use_safe_mode = True  # 启用安全模式
        self.initial_pose = None   # 记录初始位置
        
        # 根据模式选择标定点配置
        if self.calibration_mode == "NORMAL":
            # ========== 模式1：常规模式（快速，约32点）==========
            # 以初始位置为中心，在小范围内移动，标定点间距更大
            self.xy_offsets = [
                # 中心点
                (0, 0),
                # 四角点（±50mm）
                (50, 50), (-50, 50), (-50, -50), (50, -50),
                # 边中点（±50mm）
                (50, 0), (0, 50), (-50, 0), (0, -50),
                # 内圈点（±30mm）
                (30, 30), (-30, 30), (-30, -30), (30, -30),
                # 内圈边中点（±30mm）
                (30, 0), (0, 30), (-30, 0), (0, -30)
            ]
            # Z轴偏移：3层，范围±30mm，间距30mm
            self.z_vals = [-30, 0, 30]
            
        elif self.calibration_mode == "HIGH_PRECISION":
            # ========== 模式2：高精度模式（慢速，455点）==========
            # 高精度标定点配置：密集采样模式（不考虑效率）
            # 以初始位置为中心，在安全范围内进行密集采样
            # 采用多层次、高密度的空间覆盖策略
            self.xy_offsets = [
                # 中心点
                (0, 0),
                # 第1圈：半径10mm（8个点，间距约7.8mm）
                (10, 0), (0, 10), (-10, 0), (0, -10),
                (7, 7), (-7, 7), (-7, -7), (7, -7),
                # 第2圈：半径20mm（8个点，间距约15.7mm）
                (20, 0), (0, 20), (-20, 0), (0, -20),
                (14, 14), (-14, 14), (-14, -14), (14, -14),
                # 第3圈：半径30mm（12个点，间距约15.7mm）
                (30, 0), (0, 30), (-30, 0), (0, -30),
                (21, 21), (-21, 21), (-21, -21), (21, -21),
                (30, 15), (15, 30), (-15, 30), (-30, 15),
                (-30, -15), (-15, -30), (15, -30), (30, -15),
                # 第4圈：半径40mm（16个点，间距约15.7mm）
                (40, 0), (0, 40), (-40, 0), (0, -40),
                (28, 28), (-28, 28), (-28, -28), (28, -28),
                (40, 20), (20, 40), (-20, 40), (-40, 20),
                (-40, -20), (-20, -40), (20, -40), (40, -20),
                (35, 10), (10, 35), (-10, 35), (-35, 10),
                (-35, -10), (-10, -35), (10, -35), (35, -10),
                # 第5圈：半径50mm（20个点，间距约15.7mm）
                (50, 0), (0, 50), (-50, 0), (0, -50),
                (35, 35), (-35, 35), (-35, -35), (35, -35),
                (50, 25), (25, 50), (-25, 50), (-50, 25),
                (-50, -25), (-25, -50), (25, -50), (50, -25),
                (45, 15), (15, 45), (-15, 45), (-45, 15),
                (-45, -15), (-15, -45), (15, -45), (45, -15),
                (48, 10), (10, 48), (-10, 48), (-48, 10),
                (-48, -10), (-10, -48), (10, -48), (48, -10),
            ]
            # Z轴偏移：7层，范围±45mm，间距15mm
            self.z_vals = [-45, -30, -15, 0, 15, 30, 45, 60]
            
        else:
            raise ValueError(f"未知的标定模式: {self.calibration_mode}，请使用 'NORMAL' 或 'HIGH_PRECISION'")
        
        # 生成标定点列表
        self.cali_points = generate_cali_points(self.xy_offsets, self.z_vals)
        self.visualize = visualize
        
        # Nova2 机械臂初始化
        self.robot = Nova2Robot(ip_address=robot_ip)
        
        # RealSense 相机初始化（使用封装的类）
        self.camera_sdk = CamSDK_Realsense()
        self.pipeline = None
        try:
            # 设置是否保存相机内参
            self.camera_sdk.set_save_intrinsics(True)
            
            # 初始化相机
            self.logger.info("正在启动RealSense相机...")
            self.pipeline, profile = self.camera_sdk.initialize_camera(width=1280, height=720, fps=30)
            
            # 等待相机稳定（丢弃前几帧）
            self.logger.info("等待相机稳定...")
            for _ in range(30):
                try:
                    self.pipeline.wait_for_frames(timeout_ms=1000)
                except:
                    pass
            self.logger.info("✅ RealSense相机已就绪")
            
        except Exception as e:
            self.logger.error(f"❌ RealSense相机初始化失败: {e}")
            self.logger.error("请检查：")
            self.logger.error("  1. 相机是否正确连接")
            self.logger.error("  2. 相机是否被其他程序占用")
            self.logger.error("  3. USB端口是否为USB 3.0")
            raise
        
        # 可视化窗口配置
        self.window_name = "手眼标定 - 实时视图"
        self.last_frame = None  # 缓存最新帧
        self.frame_lock = threading.Lock()  # 帧数据锁
        if self.visualize:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 2560, 720)
            self.logger.info(f"✅ 已创建可视化窗口: {self.window_name}")
        
        self.aruco_detector = Aruco_Detector(marker_length=self.marker_size)
        
        # 相机内参缓存（通过camera_sdk获取）
        self.camera_matrix = None      # 3x3相机内参矩阵
        self.dist_coeffs = None         # 畸变系数
        
        # 变换矩阵
        self.T_camera2base = None
        self.T_base2camera = None
        
        # 不再启动后台预览线程，改为主线程按需刷新
        # 这样可以避免多线程操作OpenCV窗口导致的卡死问题

    def update_preview_window(self, image=None):
        """在主线程中更新预览窗口（线程安全）
        
        Args:
            image: 要显示的图像，如果为None则显示最后一帧
        """
        if not self.visualize:
            return
        
        try:
            if image is not None:
                with self.frame_lock:
                    self.last_frame = image.copy()
            
            # 显示图像
            with self.frame_lock:
                if self.last_frame is not None:
                    cv2.imshow(self.window_name, self.last_frame)
            
            # 处理窗口事件（必须在主线程中调用）
            cv2.waitKey(1)
            
        except Exception as e:
            self.logger.debug(f"更新预览窗口失败: {e}")
    
    def get_aruco_center(self, color_image, depth_frame, color_frame):
        """
        检测ArUco标记并返回其在相机坐标系下的3D坐标
        
        Args:
            color_image: 彩色图像
            depth_frame: 深度帧
            color_frame: 彩色帧对象
            
        Returns:
            list: 标记中心3D坐标[x,y,z]
        """
        try:
            # 检查帧数据是否完整
            if color_image is None or depth_frame is None or color_frame is None:
                self.logger.error("无法获取相机帧或帧数据不完整")
                return None
                
            # 获取相机内参（使用封装的相机SDK）
            # 首先尝试获取已缓存的内参
            cam_matrix, dist, intrinsics = self.camera_sdk.get_cam_intrinsics()
            
            # 如果尚未获取内参，则获取一次
            if cam_matrix is None or dist is None:
                cam_matrix, dist = self.camera_sdk.acquire_cam_intrinsics(color_frame)
            
            if cam_matrix is None:
                self.logger.error("无法获取相机内参")
                return None
            
            # 检测ArUco标记并估计位姿
            corners, ids, rejectedImgPoints = self.aruco_detector.detect_markers(color_image)
            
            # 使用aruco_detector.py中的接口获取Aruco码的坐标数据和旋转角度
            aruco_results = self.aruco_detector.get_aruco_results(corners, ids, cam_matrix, dist, self.marker_id)
            
            center_point = None
            if aruco_results['found'] and aruco_results['position'] is not None:
                # 直接使用aruco_results中的位置信息
                center_point = list(aruco_results['position'])

            # 显示可视化窗口（主线程中刷新） # 根据需要显示与否调用相应的渲染接口
            if self.visualize:
                # 为了避免频繁复制大图像，我们直接在原始图像上绘制
                vis_image = color_image.copy()  # 只复制一次
                vis_image = self.aruco_detector.draw_aruco_results(vis_image, aruco_results, corners, ids)
                self.update_preview_window(vis_image)

            # 只返回中心点坐标
            return center_point
            
        except Exception as e:
            self.logger.error(f"获取ArUco中心失败: {e}")
            return None
    
    def run_calibration(self):
        """
        执行眼在手外标定流程
        """
        try:
            # 连接机械臂
            if not self.robot.connect():
                self.logger.error("机械臂连接失败")
                return False
            
            if not self.robot.enable_robot():
                self.logger.error("机械臂使能失败")
                return False
            
            # 移动到标准位置
            self.logger.info("移动到标准位置...")
            if not self.robot.move_to_standard_pose():
                self.logger.error("移动到标准位置失败")
                return False
            
            # 获取并记录当前位置作为初始位置
            self.logger.info("获取当前位置...")
            current_pose = self.robot.get_pose()
            if current_pose is None:
                self.logger.error("无法获取当前位置")
                return False
            
            self.initial_pose = current_pose
            self.logger.info(f"   姿态: Rx={current_pose[3]:.1f}, Ry={current_pose[4]:.1f}, Rz={current_pose[5]:.1f}")
            
            # 显示当前标定模式信息
            mode_name = "常规模式" if self.calibration_mode == "NORMAL" else "高精度模式"
            xy_count = len(self.xy_offsets)
            z_count = len(self.z_vals)
            total_points = xy_count * z_count
            self.logger.info(f"")
            self.logger.info(f"📊 当前标定模式: {mode_name} ({self.calibration_mode})")
            self.logger.info(f"   XY平面点数: {xy_count}")
            self.logger.info(f"   Z轴层数: {z_count}")
            self.logger.info(f"   总标定点数: {total_points}")
            self.logger.info(f"")
            
            # 提示用户固定ArUco标记
            self.logger.info("=" * 60)
            self.logger.info(f"标记规格: Dictionary 5x5_50, ID {self.marker_id}, Size {int(self.marker_size*1000)}mm")
            if self.move_orientation:
                self.logger.info("⚠️  姿态角变化：标定点移动时将同时改变Rx/Ry/Rz角度（0-15°范围内）")
            else:
                self.logger.info("⚠️  姿态角固定：标定点移动时保持Rx=180°, Ry=0°, Rz=90°")
            self.logger.info("准备好后，标定将在5秒后自动开始...")
            self.logger.info("按 Ctrl+C 可随时中止并返回初始位置")
            self.logger.info("=" * 60)
            
            # 清理上一次标定保存的数据
            self._cleanup_previous_calibration_data()
            
            time.sleep(5)
            
            # 准备标定数据目录
            save_dir = "save_parms"
            os.makedirs(save_dir, exist_ok=True)
            cam2base_file = os.path.join(save_dir, "camera2base.npy")
            base2cam_file = os.path.join(save_dir, "base2camera.npy")
            
            # 准备数据容器
            n = len(self.cali_points)
            base_coords = np.ones((4, n))  # 机械臂基座标系坐标 (齐次坐标)
            cam_coords = np.ones((4, n))   # 相机坐标系坐标 (齐次坐标)
            
            self.logger.info(f"开始标定，共 {n} 个标定点...")
            
            # 遍历所有标定点（相对位置模式）
            for i, (dx, dy, dz) in enumerate(self.cali_points):
                # 使用相对位置：当前位置 + 偏移量
                # dx, dy, dz 是相对于初始位置的偏移量
                marker_target_x = self.initial_pose[0] + dx
                marker_target_y = self.initial_pose[1] + dy
                marker_target_z = self.initial_pose[2] + dz
                
                # 计算夹爪中心的目标位置
                gripper_x = marker_target_x - self.marker_offset_x
                gripper_y = marker_target_y - self.marker_offset_y
                gripper_z = marker_target_z - self.marker_offset_z
                
                # 根据move_orientation变量决定是否改变姿态角
                # 确保至少有一些点保持原始姿态角不变
                # 第一个点（i==0）和每隔3个点保持原始姿态角不变，其余点随机变化
                if self.move_orientation and i != 0 and i % 3 != 0:
                    # 在0到15°范围内随机生成姿态角变化
                    rx = self.initial_pose[3] + random.uniform(-15, 15)
                    ry = self.initial_pose[4] + random.uniform(-15, 15)
                    rz = self.initial_pose[5] + random.uniform(-15, 15)
                    orientation_changed = True
                else:
                    # 保持默认姿态角（标定前的原始值）
                    rx = self.initial_pose[3]
                    ry = self.initial_pose[4]
                    rz = self.initial_pose[5]
                    orientation_changed = False
                
                self.logger.info(f"标定点 {i+1}/{n}: 相对偏移 dX={dx:+.0f}, dY={dy:+.0f}, dZ={dz:+.0f}")
                self.logger.info(f"         标定板目标 X={marker_target_x:.1f}, Y={marker_target_y:.1f}, Z={marker_target_z:.1f}")
                if orientation_changed:
                    self.logger.info(f"         姿态角 Rx={rx:.1f}°, Ry={ry:.1f}°, Rz={rz:.1f}° (变化)")
                else:
                    self.logger.info(f"         姿态角 Rx={rx:.1f}°, Ry={ry:.1f}°, Rz={rz:.1f}° (保持原始值)")
                
                # 移动到目标位置（等待运动完成）
                if not self.robot.move_l(gripper_x, gripper_y, gripper_z, rx, ry, rz, wait=True):
                    self.logger.error(f"移动到标定点 {i+1} 失败")
                    return False
                
                # 额外等待稳定
                time.sleep(1)
                
                # 记录机械臂基座标系坐标（ArUco标记的实际位置）
                # 注意：这里是标定板中心在机械臂基座标系下的位置
                # 通过夹爪位置加上偏移量计算得到
                marker_base_x = gripper_x + self.marker_offset_x
                marker_base_y = gripper_y + self.marker_offset_y
                marker_base_z = gripper_z + self.marker_offset_z
                base_coords[:3, i] = [marker_base_x, marker_base_y, marker_base_z]
                
                # 先获取相机帧数据，检查帧数据是否完整
                color_image, depth_frame, color_frame = self.camera_sdk.get_frames(timeout_ms=2000)
    
                # Aruco码识别据
                center = self.get_aruco_center(color_image, depth_frame, color_frame)
                
                if center is None:
                    self.logger.warning(f"❌ 标定点 {i+1} 未检测到ArUco标记")
                    self.logger.warning("可能原因：标记超出相机视野、光照不足或标记遮挡")
                    self.logger.warning("请调整相机位置或标定点范围后重新标定")
                    return False
                
                # 记录相机坐标系坐标
                # 注意：center是标定板中心在相机坐标系下的位置
                cam_coords[:3, i] = center
                self.logger.info(f"✅ 相机观测: x={center[0]:.3f}, y={center[1]:.3f}, z={center[2]:.3f}")
                
                # 获取机械手当前姿态
                robot_pose = self.robot.get_pose()
                if robot_pose is None:
                    self.logger.warning(f"⚠️  无法获取标定点 {i+1} 的机械手姿态")
                    robot_pose = [0, 0, 0, 0, 0, 0]  # 默认值
                
                # 实时保存成功的标定点到文件
                save_result = self.save_successful_calibration_point(
                    index=i+1,
                    gripper_pos=(gripper_x, gripper_y, gripper_z),
                    marker_pos=(marker_base_x, marker_base_y, marker_base_z),
                    camera_obs=tuple(center),
                    initial_pose=self.initial_pose,
                    robot_pose=robot_pose,
                    image=color_image if self.save_source_images else None
                )
                if save_result:
                    self.logger.info(f"💾 标定点 {i+1} 已保存到文件")
                else:
                    self.logger.warning(f"⚠️  标定点 {i+1} 保存失败（但标定继续）")
            
            # 计算变换矩阵
            self.logger.info("\n计算变换矩阵...")
            # 使用SVD方法计算更稳定的变换矩阵
            # T_camera2base 使得：base_coords ≈ T_camera2base * cam_coords
            # 使用更稳定的计算方法
            # 注意：在手眼标定中，我们需要求解 AX = XB 问题
            # 这里我们使用直接线性变换法（DLT）
            
            # 检查数据维度
            self.logger.info(f"数据维度检查:")
            self.logger.info(f"  base_coords: {base_coords.shape}")
            self.logger.info(f"  cam_coords: {cam_coords.shape}")
            
            # 使用伪逆计算变换矩阵
            # T_camera2base = base_coords * pinv(cam_coords)
            self.T_camera2base = base_coords @ np.linalg.pinv(cam_coords)
            self.T_base2camera = np.linalg.pinv(self.T_camera2base)
            
            # 验证变换矩阵
            self.logger.info("验证变换矩阵...")
            R = self.T_camera2base[:3, :3]
            det_R = np.linalg.det(R)
            self.logger.info(f"旋转矩阵行列式: {det_R:.6f}")
            
            # 检查矩阵是否合理
            if abs(det_R) < 0.1 or abs(det_R) > 10.0:
                self.logger.warning("⚠️  旋转矩阵行列式异常，可能存在数据对应错误")
                self.logger.warning("建议检查:")
                self.logger.warning("  1. 标定点数据对应关系")
                self.logger.warning("  2. 坐标系方向一致性")
                self.logger.warning("  3. 标定点分布合理性")
            
            # 保存标定结果
            np.save(cam2base_file, self.T_camera2base)
            np.save(base2cam_file, self.T_base2camera)
            
            self.logger.info("=" * 60)
            self.logger.info("✅ 标定完成！")
            self.logger.info(f"变换矩阵已保存到: {save_dir}/")
            self.logger.info(f"标定点数据已保存到: {self.calibration_points_file}")
            self.logger.info(f"成功标定点总数: {n} 个")
            
            # 标定精度分析
            self._analyze_calibration_quality(base_coords, cam_coords)
            
            self.logger.info("=" * 60)
            
            return True
            
        except KeyboardInterrupt:
            self.logger.warning("\n⚠️  用户中断标定")
            return False
        except Exception as e:
            self.logger.error(f"\n❌ 标定过程出错: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def run_recognition(self):
        """
        执行识别与抓取演示（安全优化版）
        
        流程：
        1. 加载标定矩阵
        2. 记录当前位置
        3. 实时检测ArUco标记
        4. 将相机坐标转换为机械臂基座标系
        5. 控制机械臂移动到目标位置抓取
        6. 移动到随机位置放置
        7. 异常时自动返回初始位置
        """
        try:
            # 加载或设置标定矩阵
            if self.use_calibration_file:
                # 从文件加载标定矩阵
                cam2base_file = os.path.join("save_parms", "camera2base.npy")
                if not os.path.exists(cam2base_file):
                    self.logger.error("❌ 标定数据不存在，请先执行标定")
                    return False
                
                if self.T_camera2base is None:
                    self.T_camera2base = np.load(cam2base_file)
            else:
                # 使用临时指定的标定结果
                R_camera2base = np.array([
                    [ 0.04061,   0.852922,  0.520457],
                    [ 0.997892, -0.008235, -0.064366],
                    [-0.050613,  0.521974, -0.851458]
                ])
                t_camera2base = np.array([
                    [-452.848818],
                    [-582.951309],
                    [1007.351921]
                ])
                # 构建齐次变换矩阵
                self.T_camera2base = np.eye(4)
                self.T_camera2base[:3, :3] = R_camera2base
                self.T_camera2base[:3, 3:4] = t_camera2base
                
                self.logger.info("使用临时指定的标定结果:")
                self.logger.info(f"旋转矩阵 R_camera2base:\n{R_camera2base}")
                self.logger.info(f"平移向量 t_camera2base:\n{t_camera2base.flatten()}")
                self.logger.info(f"齐次矩阵 T_camera2base:\n{self.T_camera2base}")
            
            # 连接机械臂（如果未连接）
            if not self.robot.is_connected:
                if not self.robot.connect():
                    return False
                if not self.robot.enable_robot():
                    return False
            
            # 记录当前位置
            self.logger.info("获取当前位置...")
            current_pose = self.robot.get_pose()
            if current_pose:
                self.initial_pose = current_pose
                self.logger.info(f"✅ 已记录初始位置: X={current_pose[0]:.1f}, Y={current_pose[1]:.1f}, Z={current_pose[2]:.1f}")
            
            self.logger.info("=" * 60)
            self.logger.info("开始识别与抓取演示")
            self.logger.info("请在相机视野内放置ArUco标记")
            if self.wait_for_key:
                self.logger.info("每次识别到目标后，将暂停3秒并移动机械臂，然后等待按空格键继续")
            else:
                self.logger.info("每次识别到目标后，将暂停3秒并移动机械臂，然后自动继续识别")
            self.logger.info("按 Ctrl+C 可随时终止并返回初始位置")
            self.logger.info("=" * 60)
            
            time.sleep(1)
            
            recognition_count = 0  # 添加识别计数器
            K = np.array([
                [0.9650654388,  0.1235157369,  0.2310683052, 236.000327802],
                [0.1026441719,  0.6331871635, -0.7671624273, 134.5263665765],
                [-0.2410661173, 0.7640797594,  0.5983888773, 278.4078020322],
                [0.,            0.,            0.,           1.           ],
            ])
            while True:
               # 先获取相机帧数据，检查帧数据是否完整
                color_image, depth_frame, color_frame = self.camera_sdk.get_frames(timeout_ms=2000)

                # 检测ArUco标记
                center = self.get_aruco_center(color_image, depth_frame, color_frame)
                
                if center is None:
                    continue  # 未检测到标记，继续等待
                
                #aruco识别的xyz，放这里替换 1，3，5
                # 使用aruco识别的中心点坐标替换默认值
                camera_point = np.array([[center[0]], [center[1]], [center[2]], [1.0]])

                print("像素点的在相机坐标系下对应的三维坐标：\n",camera_point)

                # 手眼矩阵（相机到基座,注意单位，mm还是m，以及z的单位，要对应

                # 将相机坐标系下的点 T 转换到基坐标系下，点先左乘手眼矩阵   end/base @ cam/end  @ cam
                H = K @ camera_point

                print("像素点在基坐标系对应的三维坐标下为：\n", H[:3])

                # 坐标变换：相机坐标 -> 基座标系
                cam_pt = np.array([*center, 1.0])
                base_pt = self.T_camera2base @ cam_pt
                
                self.logger.info(f"检测到目标: 基座标系 X={base_pt[0]:.2f}, Y={base_pt[1]:.2f}, Z={base_pt[2]:.2f}")
                
                # 计算接近点和抓取点
                # base_pt 是标定板中心在基座标系下的位置
                # 需要转换为夹爪中心的目标位置
                gripper_approach_x = base_pt[0] - self.marker_offset_x
                gripper_approach_y = base_pt[1] - self.marker_offset_y
                gripper_approach_z = base_pt[2] - self.marker_offset_z + 100  # 接近点在标定板上方100mm
                
                gripper_pick_x = base_pt[0] - self.marker_offset_x
                gripper_pick_y = base_pt[1] - self.marker_offset_y
                gripper_pick_z = base_pt[2] - self.marker_offset_z - 8  # 抓取点在标定板下方8mm
                
                approach_pos = [gripper_approach_x, gripper_approach_y, gripper_approach_z]
                pick_pos = [gripper_pick_x, gripper_pick_y, gripper_pick_z]
                
				# 使用H矩阵计算的结果作为接近点
                approach_pos = [H[0][0], H[1][0], H[2][0]]  # H值作为接近点，并在Z轴方向上抬高100mm
                
                # 计算抓取点，基于approach_pos在Z轴方向下降108mm（100mm+8mm）
                pick_pos = [H[0][0], H[1][0], H[2][0]]
                
                # 移动到接近点
                self.logger.info("移动到接近点...")
                self.robot.move_l(approach_pos[0], approach_pos[1], approach_pos[2], 180, 0, 90)
                time.sleep(3)
                
                # 移动到抓取点
                self.logger.info("移动到抓取点...")
                self.robot.move_l(pick_pos[0], pick_pos[1], pick_pos[2], 180, 0, 90)
                time.sleep(2)
                
                # 注意：Nova2机械臂需要外部夹爪控制，此处模拟抓取
                self.logger.info("执行抓取动作（需要外部夹爪控制）")
                time.sleep(1)
                
                # 抬起
                self.logger.info("抬起...")
                self.robot.move_l(approach_pos[0], approach_pos[1], approach_pos[2], 180, 0, 90)
                time.sleep(2)
                
                # 移动到随机放置位置（夹爪中心位置）
                drop_x = random.randint(250, 380)
                drop_y = random.randint(-110, 210)
                drop_z = 100  # 夹爪高度
                drop_pos = [drop_x, drop_y, drop_z]
                
                self.logger.info(f"移动到放置位置: X={drop_x}, Y={drop_y}, Z={drop_z}")
                self.robot.move_l(drop_pos[0], drop_pos[1], drop_pos[2], 180, 0, 90)
                time.sleep(3)
                
                # 释放物体
                self.logger.info("释放物体（需要外部夹爪控制）")
                time.sleep(1)
                
                self.logger.info("✅ 完成一次抓取放置，等待下一个目标...")
                
        except KeyboardInterrupt:
            self.logger.info("\n识别抓取已终止")
        except Exception as e:
            self.logger.error(f"识别抓取出错: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _analyze_calibration_quality(self, base_coords, cam_coords):
        """
        分析标定质量并输出评估结果
        
        Args:
            base_coords: 机械臂基座标系坐标 (4×n)
            cam_coords: 相机坐标系坐标 (4×n)
        """
        try:
            self.logger.info("\n" + "=" * 60)
            self.logger.info("🎯 标定精度分析")
            self.logger.info("=" * 60)
            
            # 输出坐标数据统计信息
            self.logger.info(f"坐标数据统计:")
            self.logger.info(f"  机械臂坐标范围:")
            self.logger.info(f"    X: {np.min(base_coords[0, :]):.2f} ~ {np.max(base_coords[0, :]):.2f} mm")
            self.logger.info(f"    Y: {np.min(base_coords[1, :]):.2f} ~ {np.max(base_coords[1, :]):.2f} mm")
            self.logger.info(f"    Z: {np.min(base_coords[2, :]):.2f} ~ {np.max(base_coords[2, :]):.2f} mm")
            
            self.logger.info(f"  相机坐标范围:")
            self.logger.info(f"    x: {np.min(cam_coords[0, :]):.3f} ~ {np.max(cam_coords[0, :]):.3f} m")
            self.logger.info(f"    y: {np.min(cam_coords[1, :]):.3f} ~ {np.max(cam_coords[1, :]):.3f} m")
            self.logger.info(f"    z: {np.min(cam_coords[2, :]):.3f} ~ {np.max(cam_coords[2, :]):.3f} m")
            
            # 计算重投影误差
            n_points = base_coords.shape[1]
            errors = []
            
            for i in range(n_points):
                # 通过变换矩阵计算预测值
                predicted = self.T_camera2base @ cam_coords[:, i]
                # 计算与真实值的误差
                error = np.linalg.norm(predicted[:3] - base_coords[:3, i])
                errors.append(error)
            
            errors = np.array(errors)
            mean_error = np.mean(errors)
            std_error = np.std(errors)
            max_error = np.max(errors)
            min_error = np.min(errors)
            
            # 输出统计结果
            self.logger.info(f"📊 重投影误差统计:")
            self.logger.info(f"   平均误差: {mean_error:.3f} mm")
            self.logger.info(f"   标准差:   {std_error:.3f} mm")
            self.logger.info(f"   最大误差: {max_error:.3f} mm")
            self.logger.info(f"   最小误差: {min_error:.3f} mm")
            
            # 质量评级
            if mean_error < 2.0:
                quality = "⭐⭐⭐⭐⭐ 优秀 (亚毫米级精度)"
                recommendation = "✅ 标定质量极佳，可直接用于高精度应用"
            elif mean_error < 5.0:
                quality = "⭐⭐⭐⭐ 良好 (毫米级精度)"
                recommendation = "✅ 标定质量良好，适用于大多数工业应用"
            elif mean_error < 10.0:
                quality = "⭐⭐⭐ 一般 (较粗略)"
                recommendation = "⚠️ 标定质量一般，建议检查标定点分布和环境条件"
            else:
                quality = "⭐ 需要改进"
                recommendation = "❌ 标定质量较差，建议重新标定"
            
            self.logger.info(f"\n🏆 质量评级: {quality}")
            self.logger.info(f"💡 建议: {recommendation}")
            
            # 误差分布统计
            self.logger.info(f"\n📈 误差分布:")
            self.logger.info(f"   < 2mm:  {np.sum(errors < 2):4d} 点 ({np.sum(errors < 2)/len(errors)*100:5.1f}%)")
            self.logger.info(f"   < 5mm:  {np.sum(errors < 5):4d} 点 ({np.sum(errors < 5)/len(errors)*100:5.1f}%)")
            self.logger.info(f"   <10mm:  {np.sum(errors < 10):4d} 点 ({np.sum(errors < 10)/len(errors)*100:5.1f}%)")
            self.logger.info(f"   >=10mm: {np.sum(errors >= 10):4d} 点 ({np.sum(errors >= 10)/len(errors)*100:5.1f}%)")
            
            # 变换矩阵质量检查
            self.logger.info(f"\n🔄 变换矩阵质量检查:")
            R = self.T_camera2base[:3, :3]
            det_R = np.linalg.det(R)
            self.logger.info(f"   旋转矩阵行列式: {det_R:.6f}")
            
            if abs(abs(det_R) - 1.0) < 0.01:
                self.logger.info("   ✅ 旋转矩阵性质良好")
            else:
                self.logger.info("   ❌ 旋转矩阵可能存在问题")
                self.logger.info("   建议检查:")
                self.logger.info("   1. 坐标系方向是否一致")
                self.logger.info("   2. 标定点数据对应关系是否正确")
                self.logger.info("   3. 标定点分布是否合理")
            
        except Exception as e:
            self.logger.warning(f"标定精度分析失败: {e}")
    
    def _cleanup_previous_calibration_data(self):
        """
        清理上一次标定保存的所有数据，确保新标定任务不受历史数据干扰
        """
        # 分别调用两个独立的清理方法
        self._cleanup_previous_calibration_points()
        self._cleanup_previous_calibration_results()
    
    def _cleanup_previous_calibration_points(self):
        """
        清理上一次保存的标定坐标数据
        """
        try:
            # 删除标定点数据文件
            if os.path.exists(self.calibration_points_file):
                os.remove(self.calibration_points_file)
                self.logger.info(f"🗑️  已清理旧标定点数据文件: {self.calibration_points_file}")
        except Exception as e:
            self.logger.warning(f"清理旧标定点数据失败: {e}")
    
    def _cleanup_previous_calibration_results(self):
        """
        清理上一次的标定结果数据
        """
        try:
            # 删除保存参数目录下的所有文件
            save_dir = "save_parms"
            if os.path.exists(save_dir):
                import shutil
                shutil.rmtree(save_dir)
                self.logger.info(f"🗑️  已清理旧标定参数目录: {save_dir}")
            
            # 重新创建保存参数目录
            os.makedirs(save_dir, exist_ok=True)
        except Exception as e:
            self.logger.warning(f"清理旧标定结果数据失败: {e}")
    
    def save_calibration_points(self, initial_pose: list):
        """
        保存标定点数据到JSON文件
        
        Args:
            initial_pose: 初始位置 [X, Y, Z, Rx, Ry, Rz]
        """
        # 生成完整的标定点数据
        calibration_data = {
            "description": "Nova2机械臂手眼标定坐标点数据",
            "version": "2.1",
            "initial_pose": {
                "X": initial_pose[0],
                "Y": initial_pose[1],
                "Z": initial_pose[2],
                "Rx": initial_pose[3],
                "Ry": initial_pose[4],
                "Rz": initial_pose[5]
            },
            "marker_offset": {
                "X": self.marker_offset_x,
                "Y": self.marker_offset_y,
                "Z": self.marker_offset_z,
                "description": f"标定板中心相对于夹爪中心的偏移（X:{self.marker_offset_x}mm, Y:{self.marker_offset_y}mm, Z:{self.marker_offset_z}mm）"
            },
            "calibration_points": {
                "total_count": len(self.cali_points),
                "xy_layers": len(self.xy_offsets),
                "z_layers": len(self.z_vals),
                "description": "标定点按 [Z层1的所有XY点, Z层2的所有XY点, ...] 排列",
                "points": []
            }
        }
        
        # 生成每个标定点的详细信息
        for idx, (dx, dy, dz) in enumerate(self.cali_points, 1):
            # 计算标定板目标位置
            marker_x = initial_pose[0] + dx
            marker_y = initial_pose[1] + dy
            marker_z = initial_pose[2] + dz
            
            # 计算夹爪目标位置
            gripper_x = marker_x - self.marker_offset_x
            gripper_y = marker_y - self.marker_offset_y
            gripper_z = marker_z - self.marker_offset_z
            
            point_data = {
                "index": idx,
                "relative_offset": {"dX": dx, "dY": dy, "dZ": dz},
                "marker_position": {"X": marker_x, "Y": marker_y, "Z": marker_z},
                "gripper_position": {"X": gripper_x, "Y": gripper_y, "Z": gripper_z}
            }
            calibration_data["calibration_points"]["points"].append(point_data)
        
        # 保存到JSON文件
        try:
            with open(self.calibration_points_file, 'w', encoding='utf-8') as f:
                json.dump(calibration_data, f, indent=2, ensure_ascii=False)
            self.logger.debug(f"✅ 已保存标定点数据到: {self.calibration_points_file}")
            return True
        except Exception as e:
            self.logger.error(f"保存标定点数据失败: {e}")
            return False
    
    def save_successful_calibration_point(self, index: int, gripper_pos: tuple, marker_pos: tuple, 
                                          camera_obs: tuple, initial_pose: list, robot_pose: list, image=None):
        """
        实时保存成功的标定点到文件
        
        Args:
            index: 标定点索引
            gripper_pos: 夹爪位置 (X, Y, Z)
            marker_pos: 标定板位置 (X, Y, Z)
            camera_obs: 相机观测 (x, y, z)
            initial_pose: 初始姿态 [X, Y, Z, Rx, Ry, Rz]
            robot_pose: 机械手当前姿态 [X, Y, Z, Rx, Ry, Rz]
            image: ArUco码识别的源图（可选）
        """
        try:
            # 保存源图（如果启用且提供了图像）
            image_filename = None
            if self.save_source_images and image is not None:
                image_dir = os.path.join("save_parms", "calibration_images")
                os.makedirs(image_dir, exist_ok=True)
                image_filename = f"calibration_point_{index:03d}.png"
                image_path = os.path.join(image_dir, image_filename)
                cv2.imwrite(image_path, image)
                self.logger.debug(f"✅ 已保存源图: {image_path}")
            
            # 读取现有数据（如果存在）
            if os.path.exists(self.calibration_points_file):
                with open(self.calibration_points_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
            else:
                # 创建新的数据结构
                data = {
                    "description": "Nova2机械臂手眼标定坐标点数据（实时保存）",
                    "version": "2.1",
                    "initial_pose": {
                        "X": initial_pose[0],
                        "Y": initial_pose[1],
                        "Z": initial_pose[2],
                        "Rx": initial_pose[3],
                        "Ry": initial_pose[4],
                        "Rz": initial_pose[5]
                    },
                    "marker_offset": {
                        "X": self.marker_offset_x,
                        "Y": self.marker_offset_y,
                        "Z": self.marker_offset_z,
                        "description": "标定板中心相对于夹爪中心的偏移"
                    },
                    "calibration_points": {
                        "total_count": 0,
                        "successful_count": 0,
                        "description": "实时保存的成功标定点",
                        "points": []
                    }
                }
            
            # 添加新的成功标定点
            point_data = {
                "index": index,
                "gripper_position": {"X": gripper_pos[0], "Y": gripper_pos[1], "Z": gripper_pos[2]},
                "marker_position": {"X": marker_pos[0], "Y": marker_pos[1], "Z": marker_pos[2]},
                "camera_observation": {"x": camera_obs[0], "y": camera_obs[1], "z": camera_obs[2]},
                "robot_pose": {
                    "X": robot_pose[0], 
                    "Y": robot_pose[1], 
                    "Z": robot_pose[2],
                    "Rx": robot_pose[3],
                    "Ry": robot_pose[4],
                    "Rz": robot_pose[5]
                },
                "image_file": image_filename,
                "status": "success",
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            
            # 检查是否已存在该索引，如果存在则更新，否则添加
            existing_indices = [p["index"] for p in data["calibration_points"]["points"]]
            if index in existing_indices:
                # 更新现有点
                for i, p in enumerate(data["calibration_points"]["points"]):
                    if p["index"] == index:
                        data["calibration_points"]["points"][i] = point_data
                        break
            else:
                # 添加新点
                data["calibration_points"]["points"].append(point_data)
            
            # 更新成功计数
            data["calibration_points"]["successful_count"] = len(data["calibration_points"]["points"])
            
            # 保存到文件
            with open(self.calibration_points_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            self.logger.debug(f"✅ 已保存标定点 {index} 到文件 (总计: {data['calibration_points']['successful_count']})")
            return True
            
        except Exception as e:
            self.logger.warning(f"保存标定点 {index} 失败: {e}")
            return False
    
    def load_calibration_points(self):
        """
        从JSON文件加载标定点数据
        
        Returns:
            tuple: (initial_pose, calibration_points) 或 (None, None) 如果失败
        """
        if not os.path.exists(self.calibration_points_file):
            self.logger.warning(f"坐标点数据文件不存在: {self.calibration_points_file}")
            return None, None
        
        try:
            with open(self.calibration_points_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 验证数据完整性
            if "initial_pose" not in data:
                self.logger.error("坐标点数据文件缺少初始位置信息")
                return None, None
            
            if "calibration_points" not in data:
                self.logger.error("坐标点数据文件缺少标定点信息")
                return None, None
            
            total_count = data["calibration_points"]["total_count"]
            if total_count < 56:
                self.logger.error(f"标定点数量不足: {total_count} < 56（最低要求）")
                return None, None
            
            # 提取初始位置
            initial_pose_dict = data["initial_pose"]
            initial_pose = [
                initial_pose_dict["X"],
                initial_pose_dict["Y"],
                initial_pose_dict["Z"],
                initial_pose_dict["Rx"],
                initial_pose_dict["Ry"],
                initial_pose_dict["Rz"]
            ]
            
            # 提取标定点
            points = data["calibration_points"]["points"]
            calibration_points = []
            for point in points:
                gripper_pos = point["gripper_position"]
                calibration_points.append((gripper_pos["X"], gripper_pos["Y"], gripper_pos["Z"]))
            
            self.logger.info(f"✅ 已加载标定点数据文件: {self.calibration_points_file}")
            self.logger.info(f"   初始位置: X={initial_pose[0]:.1f}, Y={initial_pose[1]:.1f}, Z={initial_pose[2]:.1f}")
            self.logger.info(f"   标定点数量: {len(calibration_points)}")
            
            return initial_pose, calibration_points
            
        except json.JSONDecodeError as e:
            self.logger.error(f"坐标点数据文件格式错误: {e}")
            return None, None
        except KeyError as e:
            self.logger.error(f"坐标点数据文件缺少必要字段: {e}")
            return None, None
        except Exception as e:
            self.logger.error(f"加载标定点数据失败: {e}")
            return None, None
    

    def cleanup(self, return_to_initial=True):
        """释放资源并返回初始位置
        
        Args:
            return_to_initial: 是否返回初始位置
        """
        # 返回初始位置（如果有记录）
        if return_to_initial and self.initial_pose is not None and self.robot.is_connected:
            try:
                self.logger.info("返回标定前的初始位置...")
                self.robot.move_l(
                    self.initial_pose[0], 
                    self.initial_pose[1], 
                    self.initial_pose[2],
                    self.initial_pose[3],
                    self.initial_pose[4],
                    self.initial_pose[5]
                )
                time.sleep(2)
                self.logger.info("✅ 已返回初始位置")
            except Exception as e:
                self.logger.error(f"返回初始位置失败: {e}")
        
        # 释放相机和窗口资源
        try:
            if self.pipeline is not None:
                self.pipeline.stop()
        except Exception as e:
            self.logger.warning(f"停止相机流失败: {e}")
        
        try:
            cv2.destroyAllWindows()
            # 确保窗口关闭
            for _ in range(5):
                cv2.waitKey(1)
        except:
            pass
        
        # 断开机械臂连接
        self.robot.disconnect()
        self.logger.info("已释放所有资源。")

def main():
    """主函数（安全优化版）"""
    parser = argparse.ArgumentParser(
        description="越疆 Nova2 机械臂 眼在手外(Eye-to-Hand) 手眼标定脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  1. 仅执行标定：
     python3 nova_calib_eye_to_hand.py --ip 192.168.5.1 --calibrate --visualize
     
  2. 仅执行识别抓取：
     python3 nova_calib_eye_to_hand.py --ip 192.168.5.1 --recognize --visualize
     
  3. 先标定后识别：
     python3 nova_calib_eye_to_hand.py --ip 192.168.5.1 --visualize
     
注意：
  - 程序会自动记录初始位置
  - 按 Ctrl+C 中断时会自动返回初始位置
  - 标定采用小距离渐进式移动，避免撞机
        """
    )
    
    parser.add_argument("--ip", default="192.168.5.1", help="机械臂IP地址")
    parser.add_argument("--visualize", action="store_true", help="启用可视化窗口")
    parser.add_argument("--calibrate", action="store_true", help="仅执行标定")
    parser.add_argument("--recognize", action="store_true", help="仅执行识别抓取")
    parser.add_argument("--use-file-pos", action="store_true", help="使用保存的坐标点数据文件")
    parser.add_argument("--use-calibration-file", action="store_true", help="从文件加载标定结果")
    parser.add_argument("--no-calibration-file", action="store_true", default=True, help="使用临时指定的标定结果，而不是从文件加载（默认行为）")
    
    args = parser.parse_args()
    
    # 确定是否使用文件加载标定结果
    use_calibration_file = args.use_calibration_file and not args.no_calibration_file
    # 如果没有明确指定参数，使用默认值False（使用临时标定结果）
    if not args.use_calibration_file and not args.no_calibration_file:
        use_calibration_file = False  # 默认使用临时标定结果
    
    cal = NovaCalibration(robot_ip=args.ip, visualize=args.visualize, use_file_pos=args.use_file_pos, use_calibration_file=use_calibration_file)
    
    try:
        if args.calibrate:
            # 仅执行标定
            cal.run_calibration()
        elif args.recognize:
            # 仅执行识别
            cal.run_recognition()
        else:
            # 默认：先标定后识别
            if cal.run_calibration():
                cal.run_recognition()
    except KeyboardInterrupt:
        print("\n\n⚠️  程序被用户中断，正在返回初始位置...")
    except Exception as e:
        print(f"\n\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # cleanup 会自动返回初始位置
        cal.cleanup(return_to_initial=True)


if __name__ == "__main__":
    main()