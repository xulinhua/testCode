#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
越疆 Nova2 机械臂数据采集脚本
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
from datetime import datetime

import numpy as np
import cv2

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

# ========== 生成数据采集点 ==========
def generate_data_points(xy_offsets, z_vals):
    """生成数据采集点列表"""
    return [(x, y, z) for z in z_vals for (x, y) in xy_offsets]


class NovaDataCollector:
    """Nova2 机械臂数据采集类"""
    
    def __init__(self, robot_ip: str, visualize: bool = False, save_source_images: bool = True):
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
        
        # 数据采集参数
        self.marker_size = 0.1       # m，ArUco标记尺寸（100mm）
        self.marker_id = 0           # ArUco标记ID
        
        # 数据采集点配置
        self.move_orientation = False  # 控制是否移动采集点时改变rx/ry/rz角度
        self.initial_pose = None      # 记录初始位置
        
        # ========== 数据采集点配置 ==========
        # 以初始位置为中心，在小范围内移动，采集点间距更大
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
        
        # 生成数据采集点列表
        self.data_points = generate_data_points(self.xy_offsets, self.z_vals)
        
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
        self.window_name = "数据采集 - 实时视图"
        self.last_frame = None  # 缓存最新帧
        self.frame_lock = threading.Lock()  # 帧数据锁
        if self.visualize:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1280, 720)
            self.logger.info(f"✅ 已创建可视化窗口: {self.window_name}")
        
        self.aruco_detector = Aruco_Detector(marker_length=self.marker_size)
        
        # 数据保存配置 - 分别创建坐标数据和图像数据目录
        self.base_save_dir = "collected_data"
        self.coords_save_dir = os.path.join(self.base_save_dir, "coordinates")
        self.images_save_dir = os.path.join(self.base_save_dir, "images")
        
        # 创建必要的目录
        os.makedirs(self.base_save_dir, exist_ok=True)
        os.makedirs(self.coords_save_dir, exist_ok=True)
        os.makedirs(self.images_save_dir, exist_ok=True)
        
        self.logger.info(f"坐标数据保存目录: {self.coords_save_dir}")
        self.logger.info(f"图像数据保存目录: {self.images_save_dir}")
        
        # 相机内参缓存
        self.camera_matrix = None
        self.dist_coeffs = None
    
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
            list: 标记中心3D坐标[x,y,z]，如果未检测到则返回None
        """
        try:
            # 检查图像是否有效
            if color_image is None:
                self.logger.error("输入图像为空")
                return None
                
            # 获取相机内参
            cam_matrix, dist, intrinsics = self.camera_sdk.get_cam_intrinsics()
            
            # 如果尚未获取内参，则获取一次
            if cam_matrix is None or dist is None:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if color_frame:
                    cam_matrix, dist = self.camera_sdk.acquire_cam_intrinsics(color_frame)
            
            if cam_matrix is None:
                self.logger.error("无法获取相机内参")
                return None
            
            # 设置相机内参到aruco_detector
            if cam_matrix is not None and dist is not None:
                self.aruco_detector.set_camera_intrinsics(cam_matrix, dist)
            
            # 检测ArUco标记并估计位姿
            # 注意：现在不需要传入相机内参，因为它们已保存在aruco_detector实例中
            result = self.aruco_detector.detect_and_process_markers(
                frame=color_image,
                draw_results=self.visualize,  # 根据visualize参数决定是否绘制结果
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
            
            # 只返回中心点坐标
            return center_point
            
        except Exception as e:
            self.logger.error(f"检测ArUco标记失败: {e}")
            return None
    
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
                # 使用相对位置：当前位置 + 偏移量
                # dx, dy, dz 是相对于初始位置的偏移量
                target_x = self.initial_pose[0] + dx
                target_y = self.initial_pose[1] + dy
                target_z = self.initial_pose[2] + dz
                
                # 根据move_orientation变量决定是否改变姿态角
                if self.move_orientation and i != 0:
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
                center = self.detect_aruco_marker(color_image)
                
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
                    image=color_image if self.save_source_images else None
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
    
    def save_data_point(self, index, robot_pos, marker_pos, robot_pose, image=None):
        """
        保存数据点到文件
        
        Args:
            index: 数据点索引
            robot_pos: 机械臂位置 (x, y, z, rx, ry, rz)
            marker_pos: 标记位置 (x, y, z)
            robot_pose: 机械臂姿态
            image: 图像数据（可选）
            
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
            
            return True
            
        except Exception as e:
            self.logger.error(f"保存数据点失败: {e}")
            return False


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="越疆 Nova2 机械臂 数据采集脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  1. 执行数据采集（带可视化）：
     python3 nova_move_data.py --ip 192.168.5.1 --visualize
     
  2. 执行数据采集（不保存图像）：
     python3 nova_move_data.py --ip 192.168.5.1 --no-save-images
        """
    )
    
    parser.add_argument("--ip", default="192.168.5.1", help="机械臂IP地址")
    parser.add_argument("--visualize", action="store_true", help="启用可视化窗口")
    parser.add_argument("--no-save-images", action="store_true", help="不保存源图像")
    
    args = parser.parse_args()
    
    # 确定是否保存图像
    save_images = not args.no_save_images
    
    collector = NovaDataCollector(
        robot_ip=args.ip, 
        visualize=args.visualize, 
        save_source_images=save_images
    )
    
    try:
        collector.collect_data()
    except KeyboardInterrupt:
        print("\n\n⚠️  程序被用户中断")
    except Exception as e:
        print(f"\n\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 释放资源
        try:
            if collector.pipeline is not None:
                collector.pipeline.stop()
        except:
            pass
        
        try:
            cv2.destroyAllWindows()
        except:
            pass
        
        collector.robot.disconnect()
        print("已释放所有资源。")


if __name__ == "__main__":
    main()