#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CSI双目摄像头图像处理模块 - JetCam优化版
使用NVIDIA官方JetCam库解决GStreamer管道问题
默认参数：
    video_device_id:=[0,1]
    image_size:=[640,480]
    fps:=30
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from ruamel.yaml import YAML
import os
import time
from jetcam.csi_camera import CSICamera  # JetCam核心库


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__("stereo_cam_node")

        # === 声明参数 ===
        self.declare_parameter("video_device_id", [0, 0])
        self.declare_parameter("image_size", [640, 480])
        self.declare_parameter("fps", 30)
        self.declare_parameter("calibration_file_path", None)
        self.declare_parameter("algorithm_type", "bm")

        # === 获取参数 ===
        self.left_id, self.right_id = self.get_parameter("video_device_id").value
        self.width, self.height = self.get_parameter("image_size").value
        self.fps = self.get_parameter("fps").value
        self.algorithm_type = self.get_parameter("algorithm_type").value
        self.calibration_file_path = self.get_parameter("calibration_file_path").value
        
        # 单目模式检测
        self.is_mono = (self.left_id == self.right_id)

        # === 加载校准数据 ===
        self._load_calibration_data()

        # === 打印配置信息 ===
        self._log_configuration()

        # === 创建发布者 ===
        self._setup_publishers()

        # === 初始化CvBridge ===
        self.bridge = CvBridge()

        # === 初始化摄像头（JetCam） ===
        self._init_cameras()

    def _load_calibration_data(self):
        """加载相机校准数据"""
        self.calibration_data_left = None
        self.calibration_data_right = None
        
        if self.calibration_file_path:
            # 如果提供了校准文件路径
            if isinstance(self.calibration_file_path, list) and len(self.calibration_file_path) >= 2:
                self.calibration_data_left = self.load_camera_info(self.calibration_file_path[0])
                self.calibration_data_right = self.load_camera_info(self.calibration_file_path[1])
        else:
            # 自动查找默认校准文件
            self.get_logger().info("No calibration file specified, trying to find default files")
            package_share_path = os.path.join(os.path.dirname(__file__), '..', '..', '..')
            package_share_path = os.path.abspath(package_share_path)
            
            possible_left_paths = [
                os.path.join(package_share_path, "share", "csi_cam_service", "config", "left.yaml"),
                os.path.join(os.path.dirname(__file__), "..", "..", "config", "left.yaml")
            ]
            
            possible_right_paths = [
                os.path.join(package_share_path, "share", "csi_cam_service", "config", "right.yaml"),
                os.path.join(os.path.dirname(__file__), "..", "..", "config", "right.yaml")
            ]
            
            for path in possible_left_paths:
                if os.path.exists(path):
                    self.calibration_data_left = self.load_camera_info(path)
                    self.get_logger().info(f"Auto-loaded left calibration file: {path}")
                    break
            
            for path in possible_right_paths:
                if os.path.exists(path):
                    self.calibration_data_right = self.load_camera_info(path)
                    self.get_logger().info(f"Auto-loaded right calibration file: {path}")
                    break

    def _log_configuration(self):
        """打印配置信息"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("CSI Stereo Camera Node (JetCam) Started")
        self.get_logger().info(f"Mode: {'MONO' if self.is_mono else 'STEREO'}")
        self.get_logger().info(f"Left Camera ID: {self.left_id}")
        self.get_logger().info(f"Right Camera ID: {self.right_id}")
        self.get_logger().info(f"Resolution: {self.width}x{self.height}")
        self.get_logger().info(f"FPS: {self.fps}")
        self.get_logger().info(f"Algorithm: {self.algorithm_type}")
        self.get_logger().info(f"Calibration: {self.calibration_file_path or 'None'}")
        self.get_logger().info("=" * 60)

    def _setup_publishers(self):
        """设置所有发布者 - 完整版"""
        camera_id = self.left_id
        
        # 原始图像
        self.publisher_left_raw = self.create_publisher(
            Image, f"/csi_camera_{camera_id}/color/image_raw", 10
        )
        self.publisher_right_raw = self.create_publisher(
            Image, f"/csi_camera_{camera_id}/color/image_raw", 10
        )
        
        # 合成图像
        self.publisher_combine_raw = self.create_publisher(
            Image, f"/csi_camera_{camera_id}/combine/image_raw", 10
        )
        
        # 相机信息
        self.publisher_camera_info_left = self.create_publisher(
            CameraInfo, f"/csi_camera_{camera_id}/color/camera_info", 10
        )
        self.publisher_camera_info_right = self.create_publisher(
            CameraInfo, f"/csi_camera_{camera_id}/color/camera_info", 10
        )
        
        # 压缩图像 - 左右都定义
        self.publisher_left_compressed = self.create_publisher(
            Image, f"/csi_camera_{camera_id}/image_compressed", 10
        )
        self.publisher_right_compressed = self.create_publisher(  # 添加这一行
            Image, f"/csi_camera_{camera_id}/image_compressed", 10
        )
        
        # 深度图
        self.publisher_left_depth = self.create_publisher(
            Image, f"/csi_camera_{camera_id}/depth/image_raw", 10
        )
        self.publisher_right_depth = self.create_publisher(
            Image, f"/csi_camera_{camera_id}/depth/image_raw", 10
        )
        
        self.get_logger().info("✅ All publishers created successfully")

    def _init_cameras(self):
        """使用JetCam初始化摄像头"""
        try:
            # 初始化左相机
            self.get_logger().info(f"Initializing left camera (ID: {self.left_id})...")
            self.cam_left = CSICamera(
                capture_device=self.left_id,
                width=self.width,
                height=self.height,
                capture_width=self.width,
                capture_height=self.height,
                capture_fps=self.fps
            )
            self.get_logger().info("✅ Left camera initialized")
            
            # 测试读取一帧
            test_frame = self.cam_left.read()
            self.get_logger().info(f"   Test frame shape: {test_frame.shape}")
            
            if self.is_mono:
                # 单目模式：共用同一个摄像头
                self.cam_right = self.cam_left
                self.get_logger().info("ℹ️ Mono mode: using same camera for both channels")
            else:
                # 双目模式：初始化右相机
                self.get_logger().info(f"Initializing right camera (ID: {self.right_id})...")
                self.cam_right = CSICamera(
                    capture_device=self.right_id,
                    width=self.width,
                    height=self.height,
                    capture_width=self.width,
                    capture_height=self.height,
                    capture_fps=self.fps
                )
                self.get_logger().info("✅ Right camera initialized")
                
                # 测试读取
                test_frame = self.cam_right.read()
                self.get_logger().info(f"   Test frame shape: {test_frame.shape}")
            
            # 启动自动捕获线程
            self.cam_left.running = True
            if not self.is_mono:
                self.cam_right.running = True
                
        except Exception as e:
            self.get_logger().error(f"❌ Camera initialization failed: {e}")
            self.get_logger().error("Please check:")
            self.get_logger().error("  1. Camera ribbon cable is properly connected")
            self.get_logger().error("  2. Run 'ls /dev/video*' to verify camera is detected")
            self.get_logger().error("  3. Try 'sudo systemctl restart nvargus-daemon'")
            raise RuntimeError("Camera init failed")

    def capture_image(self):
        """捕获并发布图像 - 使用JetCam的自动捕获"""
        try:
            # 从JetCam获取最新帧
            if self.is_mono:
                # 单目模式
                frame_left = self.cam_left.value  # 或 self.cam_left.read()
                frame_right = frame_left
            else:
                # 双目模式
                frame_left = self.cam_left.value
                frame_right = self.cam_right.value

            # 相机校准
            if frame_left is not None:
                frame_left = self.calibrate_camera(frame_left, self.calibration_data_left)
            if frame_right is not None:
                frame_right = self.calibrate_camera(frame_right, self.calibration_data_right)

            # 发布左右原始图像
            if frame_left is not None:
                img_msg = self.bridge.cv2_to_imgmsg(frame_left, encoding="bgr8")
                self.publisher_left_raw.publish(img_msg)

            if frame_right is not None:
                img_msg = self.bridge.cv2_to_imgmsg(frame_right, encoding="bgr8")
                self.publisher_right_raw.publish(img_msg)

            # 如果两帧都成功，处理合成、压缩和深度图
            if frame_left is not None and frame_right is not None:
                # 发布合成图像
                combined = cv2.hconcat([frame_left, frame_right])
                combined_msg = self.bridge.cv2_to_imgmsg(combined, encoding="bgr8")
                self.publisher_combine_raw.publish(combined_msg)

                # 压缩并发布
                compressed_left = self.compress_image(frame_left)
                compressed_right = self.compress_image(frame_right)
                
                self.publisher_left_compressed.publish(
                    self.bridge.cv2_to_imgmsg(compressed_left, encoding="bgr8")
                )
                self.publisher_right_compressed.publish(
                    self.bridge.cv2_to_imgmsg(compressed_right, encoding="bgr8")
                )

                # 计算深度图（双目模式且不是单目模拟时）
                if not self.is_mono:
                    depth_map = self.process_depth_map(compressed_left, compressed_right)
                    if depth_map is not None:
                        depth_normalized = cv2.normalize(
                            depth_map, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
                        )
                        depth_msg = self.bridge.cv2_to_imgmsg(depth_normalized, encoding="mono8")
                        self.publisher_left_depth.publish(depth_msg)
                        self.publisher_right_depth.publish(depth_msg)
                else:
                    # 单目模式下发布占位符深度图
                    dummy_depth = np.zeros((self.height, self.width), dtype=np.uint8)
                    depth_msg = self.bridge.cv2_to_imgmsg(dummy_depth, encoding="mono8")
                    self.publisher_left_depth.publish(depth_msg)
                    self.publisher_right_depth.publish(depth_msg)

                # 发布相机信息
                self.publish_camera_calibration()

            self.get_logger().debug("Images captured and published", throttle_duration_sec=1.0)

        except Exception as e:
            self.get_logger().error(f"Capture error: {e}")

    # === 以下方法保持原样，无需修改 ===
    def process_depth_map(self, left_img, right_img):
        """深度算法选择"""
        depth_map = None
        if self.algorithm_type == "bm":
            depth_map = self.get_depth_map_bm(left_img, right_img)
        elif self.algorithm_type == "sgbm":
            depth_map = self.get_depth_map_sgbm(left_img, right_img)
        else:
            raise ValueError("Invalid algorithm type")
        return depth_map

    def compress_image(self, img, quality=90):
        """压缩图像"""
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        _, compressed_img = cv2.imencode(".jpg", img, encode_param)
        return cv2.imdecode(compressed_img, 1)

    def get_depth_map_bm(self, left_img, right_img, num_disparities=16, block_size=15):
        """使用BM算法获取深度图像"""
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        stereo = cv2.StereoBM_create(
            numDisparities=num_disparities, blockSize=block_size
        )
        disparity = stereo.compute(left_gray, right_gray)

        if disparity.max() > disparity.min():
            disparity_normalized = (disparity - disparity.min()) / (disparity.max() - disparity.min())
            return disparity_normalized
        return np.zeros_like(disparity)

    def get_depth_map_sgbm(self, left_img, right_img, num_disparities=16, block_size=15):
        """使用SGBM算法获取深度图像"""
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * 3 * block_size**2,
            P2=32 * 3 * block_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
        )
        disparity = stereo.compute(left_gray, right_gray)

        if disparity.max() > disparity.min():
            disparity_normalized = (disparity - disparity.min()) / (disparity.max() - disparity.min())
            return disparity_normalized
        return np.zeros_like(disparity)

    def load_camera_calibration_from_yaml(self, input_file):
        """从YAML文件中加载相机标定数据"""
        if not input_file:
            return None
        yaml = YAML()
        with open(input_file, "r") as f:
            data = yaml.load(f)
        return data

    def load_camera_info(self, calibration_file_path):
        """加载相机信息"""
        camera_info_msg = None
        if calibration_file_path is not None:
            yaml_data = self.load_camera_calibration_from_yaml(calibration_file_path)
            camera_info_msg = CameraInfo()
            camera_info_msg.height = yaml_data["image_height"]
            camera_info_msg.width = yaml_data["image_width"]
            camera_info_msg.header.frame_id = yaml_data["camera_name"]
            camera_info_msg.distortion_model = yaml_data["distortion_model"]
            camera_info_msg.d = yaml_data["distortion_coefficients"]["data"]
            camera_info_msg.k = yaml_data["camera_matrix"]["data"]
            camera_info_msg.r = yaml_data["rectification_matrix"]["data"]
            camera_info_msg.p = yaml_data["projection_matrix"]["data"]
        return camera_info_msg

    def calibrate_camera(self, frame, calibration_data):
        """相机校准"""
        if calibration_data:
            try:
                frame = cv2.undistort(
                    frame,
                    np.array(calibration_data.k).reshape(3, 3),
                    np.array(calibration_data.d),
                )
            except Exception as e:
                self.get_logger().warn(f"Calibration failed: {e}")
        return frame

    def publish_camera_calibration(self):
        """发布相机校准信息"""
        if self.calibration_data_left:
            self.calibration_data_left.header.stamp = self.get_clock().now().to_msg()
            self.publisher_camera_info_left.publish(self.calibration_data_left)
        
        if self.calibration_data_right:
            self.calibration_data_right.header.stamp = self.get_clock().now().to_msg()
            self.publisher_camera_info_right.publish(self.calibration_data_right)

    def destroy_node(self):
        """清理资源"""
        self.get_logger().info("Shutting down...")
        
        # 释放JetCam资源
        if hasattr(self, 'cam_left'):
            self.cam_left.running = False
            if hasattr(self.cam_left, 'cap'):
                self.cam_left.cap.release()
        
        if hasattr(self, 'cam_right') and self.cam_right != self.cam_left:
            self.cam_right.running = False
            if hasattr(self.cam_right, 'cap'):
                self.cam_right.cap.release()
        
        super().destroy_node()


def main():
    rclpy.init()
    node = StereoCameraNode()
    
    # 创建定时器（替代time.sleep循环）
    timer_period = 1.0 / node.fps
    timer = node.create_timer(timer_period, node.capture_image)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()