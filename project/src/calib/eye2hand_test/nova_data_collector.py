#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
越疆 Nova2 机械臂数据采集模块
基于 Nova2 机械臂控制接口实现
支持固件版本: 3.5.8.1 及以上
"""

import sys
import os
import logging
import time
import random
import json
import threading
from datetime import datetime

import numpy as np
import cv2

# 导入外部封装的类
from nova_robot import Nova2Robot
from camsdk_realsense import CamSDK_Realsense
from aruco_detector import Aruco_Detector

# ========== 生成数据采集点 ==========
def generate_data_points(xy_offsets, z_vals):
    """生成数据采集点列表"""
    return [(x, y, z) for z in z_vals for (x, y) in xy_offsets]


class NovaDataCollector:
    """Nova2 机械臂数据采集类"""
    
    def __init__(self, robot_ip: str, visualize: bool = True, save_source_images: bool = True):
        """
        初始化数据采集系统
        
        Args:
            robot_ip: 机械臂IP地址
            visualize: 是否启用可视化窗口
            save_source_images: 是否保存源图像
        """
        self.logger = logging.getLogger(self.__class__.__name__)
        self.visualize = visualize
        self.save_source_images = save_source_images
        self.auto_load_data = False  # 控制是否自动加载之前保存的数据
        
        # 数据采集参数
        self.marker_size = 0.1       # m，ArUco标记尺寸（100mm）
        self.marker_id = 0           # ArUco标记ID
        
        # 数据采集点配置
        self.move_orientation = False  # 控制是否移动采集点时改变rx/ry/rz角度
        
        # ========== 数据采集点配置 ==========
        # 以标准位置为中心，在小范围内移动，采集点间距更大
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
        self.z_vals = [-150, -90, -60, -30, 0, 30]
        #self.z_vals = [-30, 0, 30]
        # 生成数据采集点列表
        self.data_points = generate_data_points(self.xy_offsets, self.z_vals)
        
        # Nova2 机械臂初始化
        self.robot = Nova2Robot(ip_address=robot_ip)
        
        # 获取机械臂标准位置作为初始位置
        self.standard_pose = self.robot.get_standard_pose()
        
        # 初始化 ArUco 检测器
        self.aruco_detector = Aruco_Detector(marker_length=self.marker_size)
        
        # RealSense 相机初始化（使用封装的类）
        self.camera_sdk = CamSDK_Realsense()
        self.pipeline = None
        try:
            # 设置是否保存相机内参
            self.camera_sdk.set_save_intrinsics(True)
            
            # 初始化相机
            self.logger.info("正在启动RealSense相机...")
            self.pipeline, profile = self.camera_sdk.initialize_camera(width=1280, height=720, fps=30)
             # 获取相机内参
            cam_matrix, dist, intrinsics = self.camera_sdk.get_cam_intrinsics()

            if cam_matrix is None:
                self.logger.error("无法获取相机内参")
                return
            
            # 设置相机内参到aruco_detector
            if cam_matrix is not None and dist is not None:
                self.aruco_detector.set_camera_intrinsics(cam_matrix, dist)

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
        self.window_name = "数据采集 - 实时视图"
        self.last_frame = None  # 缓存最新帧
        self.frame_lock = threading.Lock()  # 帧数据锁
        if self.visualize:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1280, 720)
            self.logger.info(f"✅ 已创建可视化窗口: {self.window_name}")
        
        # 数据保存配置 - 分别创建坐标数据和图像数据目录
        self.base_save_dir = "save_parms"
        self.coords_save_dir = os.path.join(self.base_save_dir, "coordinates")
        self.images_save_dir = os.path.join(self.base_save_dir, "images")
        # 新增目录用于保存Aruco码识别渲染后的图像
        self.aruco_images_save_dir = os.path.join(self.base_save_dir, "aruco_rendered")
        
        # 创建必要的目录
        os.makedirs(self.base_save_dir, exist_ok=True)
        os.makedirs(self.coords_save_dir, exist_ok=True)
        os.makedirs(self.images_save_dir, exist_ok=True)
        os.makedirs(self.aruco_images_save_dir, exist_ok=True)
        
        self.logger.info(f"坐标数据保存目录: {self.coords_save_dir}")
        self.logger.info(f"图像数据保存目录: {self.images_save_dir}")
        self.logger.info(f"Aruco渲染图像保存目录: {self.aruco_images_save_dir}")
        
        # 相机内参缓存
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 如果设置了自动加载数据，则加载之前保存的数据
        self.loaded_data = None
        if self.auto_load_data:
            self.loaded_data = self.load_saved_data()
    
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
    
    def detect_aruco_marker(self, color_image):
        """
        检测ArUco标记并返回其在相机坐标系下的3D坐标
        
        Args:
            color_image: 彩色图像
            
        Returns:
            tuple: (center_point, rendered_image) 标记中心3D坐标和渲染后的图像
        """
        try:
            # 检查图像是否有效
            if color_image is None:
                self.logger.error("输入图像为空")
                return None, None

            # 检测ArUco标记并估计位姿
            # 注意：现在不需要传入相机内参，因为它们已保存在aruco_detector实例中
            result = self.aruco_detector.detect_and_process_markers(
                frame=color_image,
                draw_results=True,  # 始终绘制结果，以便保存渲染后的图像
                print_results=False  # 不在检测时打印结果，避免输出过多信息
            )
            
            # 提取中心点坐标
            center_point = None
            if result['found'] and len(result['markers_info']) > 0:
                # 使用第一个检测到的标记的位置信息
                marker_info = result['markers_info'][0]
                if marker_info.get('position') is not None:
                    center_point = list(marker_info['position'])

            # 如果需要可视化，更新预览窗口
            if self.visualize and 'frame' in result:
                self.update_preview_window(result['frame'])
            
            # 返回中心点坐标和渲染后的图像
            rendered_image = result.get('frame') if 'frame' in result else None
            return center_point, rendered_image
            
        except Exception as e:
            self.logger.error(f"检测ArUco标记失败: {e}")
            return None, None
    
    def collect_data(self):
        """
        执行数据采集流程
        
        流程：
        1. 连接机械臂并使能
        2. 移动到标准位置
        3. 记录当前位置作为初始位置
        4. 移动到各个数据采集点
        5. 在每个点停稳后获取相机图像并识别Aruco码
        6. 若识别成功，则保存当前点和图像数据
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
            
            # 清理上一次采集保存的数据
            self._cleanup_previous_data()
            
            # 获取并记录当前位置作为初始位置
            self.logger.info("获取当前位置...")
            current_pose = self.robot.get_pose()
            if current_pose is None:
                self.logger.error("无法获取当前位置")
                return False
            
            self.initial_pose = current_pose
            self.logger.info(f"✅ 已记录初始位置: X={current_pose[0]:.1f}, Y={current_pose[1]:.1f}, Z={current_pose[2]:.1f}")
            self.logger.info(f"   姿态: Rx={current_pose[3]:.1f}, Ry={current_pose[4]:.1f}, Rz={current_pose[5]:.1f}")
            
            # 准备数据容器
            n = len(self.data_points)
            self.logger.info(f"开始数据采集，共 {n} 个采集点...")
            
            # 成功采集的数据点计数
            successful_points = 0
            
            # 遍历所有数据采集点
            for i, (dx, dy, dz) in enumerate(self.data_points):
                # 使用相对位置：标准位置 + 偏移量
                # dx, dy, dz 是相对于标准位置的偏移量
                target_x = self.standard_pose[0] + dx
                target_y = self.standard_pose[1] + dy
                target_z = self.standard_pose[2] + dz
                
                # 根据move_orientation变量决定是否改变姿态角
                if self.move_orientation and i != 0:
                    # 在0到15°范围内随机生成姿态角变化
                    rx = self.standard_pose[3] + random.uniform(-15, 15)
                    ry = self.standard_pose[4] + random.uniform(-15, 15)
                    rz = self.standard_pose[5] + random.uniform(-15, 15)
                    orientation_changed = True
                else:
                    # 保持默认姿态角（标准位置的姿态角）
                    rx = self.standard_pose[3]
                    ry = self.standard_pose[4]
                    rz = self.standard_pose[5]
                    orientation_changed = False
                
                self.logger.info(f"采集点 {i+1}/{n}: 相对偏移 dX={dx:+.0f}, dY={dy:+.0f}, dZ={dz:+.0f}")
                self.logger.info(f"         目标位置 X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}")
                if orientation_changed:
                    self.logger.info(f"         姿态角 Rx={rx:.1f}°, Ry={ry:.1f}°, Rz={rz:.1f}° (变化)")
                else:
                    self.logger.info(f"         姿态角 Rx={rx:.1f}°, Ry={ry:.1f}°, Rz={rz:.1f}° (保持原始值)")
                
                # 移动到目标位置（等待运动完成）
                if not self.robot.move_l(target_x, target_y, target_z, rx, ry, rz, wait=True):
                    self.logger.error(f"移动到采集点 {i+1} 失败")
                    continue
                
                # 额外等待稳定
                time.sleep(1)
                
                # 获取相机图像并识别Aruco码
                self.logger.info("获取相机图像并识别Aruco码...")
                
                # 获取帧数据
                color_image, depth_frame, color_frame = self.camera_sdk.get_frames(timeout_ms=2000)
                
                if color_image is None or depth_frame is None or color_frame is None:
                    self.logger.warning(f"❌ 采集点 {i+1} 无法获取相机帧数据")
                    continue
                
                # 检测ArUco标记
                center, rendered_image = self.detect_aruco_marker(color_image)
                
                if center is None:
                    self.logger.warning(f"❌ 采集点 {i+1} 未检测到ArUco标记")
                    continue
                
                # 识别成功，保存数据
                self.logger.info(f"✅ 采集点 {i+1} 识别成功: x={center[0]:.3f}, y={center[1]:.3f}, z={center[2]:.3f}")
                
                # 获取机械手当前姿态
                robot_pose = self.robot.get_pose()
                if robot_pose is None:
                    self.logger.warning(f"⚠️  无法获取采集点 {i+1} 的机械手姿态")
                    robot_pose = [0, 0, 0, 0, 0, 0]  # 默认值
                
                # 保存成功的数据点
                save_result = self.save_data_point(
                    index=successful_points+1,
                    robot_pos=(target_x, target_y, target_z, rx, ry, rz),
                    marker_pos=tuple(center),
                    robot_pose=robot_pose,
                    image=color_image if self.save_source_images else None,
                    rendered_image=rendered_image
                )
                
                if save_result:
                    self.logger.info(f"💾 采集点 {i+1} 数据已保存")
                    successful_points += 1
                else:
                    self.logger.warning(f"⚠️  采集点 {i+1} 数据保存失败")
            
            self.logger.info("=" * 60)
            self.logger.info(f"✅ 数据采集完成！")
            self.logger.info(f"成功采集点数: {successful_points}/{n}")
            self.logger.info(f"坐标数据已保存到: {self.coords_save_dir}")
            self.logger.info(f"图像数据已保存到: {self.images_save_dir}")
            self.logger.info("=" * 60)
            
            return True
            
        except KeyboardInterrupt:
            self.logger.warning("\n⚠️  用户中断数据采集")
            return False
        except Exception as e:
            self.logger.error(f"\n❌ 数据采集过程出错: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            # 确保程序结束时返回标准位置
            self._return_to_standard_position()
            
            # 释放资源
            try:
                if self.pipeline is not None:
                    self.pipeline.stop()
            except:
                pass
            
            try:
                cv2.destroyAllWindows()
            except:
                pass
            
            self.robot.disconnect()
            print("已释放所有资源。")

    def save_data_point(self, index, robot_pos, marker_pos, robot_pose, image=None, rendered_image=None):
        """
        保存数据点到文件
        
        Args:
            index: 数据点索引
            robot_pos: 机械臂位置 (x, y, z, rx, ry, rz)
            marker_pos: 标记位置 (x, y, z)
            robot_pose: 机械臂姿态
            image: 图像数据（可选）
            rendered_image: Aruco码识别渲染后的图像（可选）
            
        Returns:
            bool: 保存成功返回True，失败返回False
        """
        try:
            # 准备数据
            data = {
                "index": index,
                "timestamp": datetime.now().isoformat(),
                "robot_position": {
                    "x": robot_pos[0],
                    "y": robot_pos[1],
                    "z": robot_pos[2],
                    "rx": robot_pos[3],
                    "ry": robot_pos[4],
                    "rz": robot_pos[5]
                },
                "marker_position": {
                    "x": marker_pos[0],
                    "y": marker_pos[1],
                    "z": marker_pos[2]
                },
                "robot_pose": {
                    "x": robot_pose[0],
                    "y": robot_pose[1],
                    "z": robot_pose[2],
                    "rx": robot_pose[3],
                    "ry": robot_pose[4],
                    "rz": robot_pose[5]
                }
            }
            
            # 保存坐标数据到JSON文件
            filename = f"data_point_{index:03d}.json"
            filepath = os.path.join(self.coords_save_dir, filename)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            # 如果需要保存图像，则保存图像文件到图像目录
            if image is not None:
                image_filename = f"data_point_{index:03d}.png"
                image_filepath = os.path.join(self.images_save_dir, image_filename)
                success = cv2.imwrite(image_filepath, image)
                if success:
                    self.logger.debug(f"图像已保存到: {image_filepath}")
                else:
                    self.logger.warning(f"图像保存失败: {image_filepath}")
            
            # 保存Aruco码识别渲染后的图像
            if rendered_image is not None:
                rendered_image_filename = f"data_point_{index:03d}_aruco.png"
                rendered_image_filepath = os.path.join(self.aruco_images_save_dir, rendered_image_filename)
                success = cv2.imwrite(rendered_image_filepath, rendered_image)
                if success:
                    self.logger.debug(f"Aruco渲染图像已保存到: {rendered_image_filepath}")
                else:
                    self.logger.warning(f"Aruco渲染图像保存失败: {rendered_image_filepath}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"保存数据点失败: {e}")
            return False
    
    def load_saved_data(self):
        """
        加载之前通过save_data_point接口保存的坐标数据和图片信息
        
        Returns:
            dict: 包含加载的数据信息，包括数据点列表和初始位置等
        """
        if not os.path.exists(self.coords_save_dir):
            self.logger.warning(f"坐标数据目录不存在: {self.coords_save_dir}")
            return None
        
        try:
            # 获取所有坐标数据文件
            coord_files = [f for f in os.listdir(self.coords_save_dir) if f.endswith('.json')]
            coord_files.sort()  # 按文件名排序
            
            if not coord_files:
                self.logger.info("未找到任何保存的坐标数据文件")
                return None
            
            # 加载所有数据点
            data_points = []
            initial_pose = None
            
            for filename in coord_files:
                filepath = os.path.join(self.coords_save_dir, filename)
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    
                    # 提取数据
                    index = data.get("index", 0)
                    timestamp = data.get("timestamp", "")
                    robot_pos = data.get("robot_position", {})
                    marker_pos = data.get("marker_position", {})
                    robot_pose = data.get("robot_pose", {})
                    
                    # 转换为元组格式
                    robot_position = (
                        robot_pos.get("x", 0),
                        robot_pos.get("y", 0),
                        robot_pos.get("z", 0),
                        robot_pos.get("rx", 0),
                        robot_pos.get("ry", 0),
                        robot_pos.get("rz", 0)
                    )
                    
                    marker_position = (
                        marker_pos.get("x", 0),
                        marker_pos.get("y", 0),
                        marker_pos.get("z", 0)
                    )
                    
                    robot_pose_tuple = (
                        robot_pose.get("x", 0),
                        robot_pose.get("y", 0),
                        robot_pose.get("z", 0),
                        robot_pose.get("rx", 0),
                        robot_pose.get("ry", 0),
                        robot_pose.get("rz", 0)
                    )
                    
                    # 记录第一个数据点的机械臂位置作为初始位置（如果尚未设置）
                    if initial_pose is None:
                        initial_pose = robot_position
                    
                    # 检查图像文件是否存在
                    image_filename = f"data_point_{index:03d}.png"
                    image_filepath = os.path.join(self.images_save_dir, image_filename)
                    image_exists = os.path.exists(image_filepath)
                    
                    # 检查Aruco渲染图像文件是否存在
                    rendered_image_filename = f"data_point_{index:03d}_aruco.png"
                    rendered_image_filepath = os.path.join(self.aruco_images_save_dir, rendered_image_filename)
                    rendered_image_exists = os.path.exists(rendered_image_filepath)
                    
                    data_points.append({
                        "index": index,
                        "timestamp": timestamp,
                        "robot_position": robot_position,
                        "marker_position": marker_position,
                        "robot_pose": robot_pose_tuple,
                        "image_exists": image_exists,
                        "image_filepath": image_filepath if image_exists else None,
                        "rendered_image_exists": rendered_image_exists,
                        "rendered_image_filepath": rendered_image_filepath if rendered_image_exists else None
                    })
                    
                except Exception as e:
                    self.logger.warning(f"加载数据文件 {filename} 失败: {e}")
                    continue
            
            if data_points:
                self.logger.info(f"✅ 成功加载 {len(data_points)} 个数据点")
                if initial_pose:
                    self.logger.info(f"   初始位置: X={initial_pose[0]:.1f}, Y={initial_pose[1]:.1f}, Z={initial_pose[2]:.1f}")
                
                return {
                    "data_points": data_points,
                    "initial_pose": initial_pose,
                    "total_count": len(data_points)
                }
            else:
                self.logger.warning("未成功加载任何数据点")
                return None
                
        except Exception as e:
            self.logger.error(f"加载保存的数据失败: {e}")
            return None
    
    def get_loaded_data(self):
        """
        获取已加载的数据
        
        Returns:
            dict: 已加载的数据，如果没有加载数据或加载失败则返回None
        """
        return self.loaded_data
    
    def _cleanup_previous_data(self):
        """
        清理上一次数据采集保存的数据文件
        """
        try:
            # 清理坐标数据目录
            if os.path.exists(self.coords_save_dir):
                coord_files = [f for f in os.listdir(self.coords_save_dir) if f.endswith('.json')]
                for filename in coord_files:
                    filepath = os.path.join(self.coords_save_dir, filename)
                    try:
                        os.remove(filepath)
                        self.logger.debug(f"已删除坐标数据文件: {filename}")
                    except Exception as e:
                        self.logger.warning(f"删除坐标数据文件 {filename} 失败: {e}")
            
            # 清理图像数据目录
            if os.path.exists(self.images_save_dir):
                image_files = [f for f in os.listdir(self.images_save_dir) if f.endswith('.png')]
                for filename in image_files:
                    filepath = os.path.join(self.images_save_dir, filename)
                    try:
                        os.remove(filepath)
                        self.logger.debug(f"已删除图像文件: {filename}")
                    except Exception as e:
                        self.logger.warning(f"删除图像文件 {filename} 失败: {e}")
            
            # 清理Aruco渲染图像目录
            if os.path.exists(self.aruco_images_save_dir):
                aruco_image_files = [f for f in os.listdir(self.aruco_images_save_dir) if f.endswith('.png')]
                for filename in aruco_image_files:
                    filepath = os.path.join(self.aruco_images_save_dir, filename)
                    try:
                        os.remove(filepath)
                        self.logger.debug(f"已删除Aruco渲染图像文件: {filename}")
                    except Exception as e:
                        self.logger.warning(f"删除Aruco渲染图像文件 {filename} 失败: {e}")
            
            self.logger.info("✅ 已清理上一次采集保存的数据文件")
        except Exception as e:
            self.logger.warning(f"清理数据文件时出错: {e}")
    
    def _return_to_standard_position(self):
        """
        返回标准位置
        """
        try:
            if self.robot.is_connected:
                # 返回标准位置
                self.logger.info("返回标准位置...")
                if self.robot.move_to_standard_pose():
                    self.logger.info("✅ 已返回标准位置")
                else:
                    self.logger.error("❌ 返回标准位置失败")
        except Exception as e:
            self.logger.error(f"返回标准位置时出错: {e}")