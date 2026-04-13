import numpy as np
import cv2
import pyrealsense2 as rs
import os
import time
import json


class CamSDK_Realsense:
    """相机SDK模块，负责管理相机的参数设置和参数或信息获取"""
    
    def __init__(self):
        self.pipeline = None
        self.profile = None
        self.mtx = None
        self.dist = None
        self.intrinsics_acquired = False
        self.save_intrinsics_enabled = True  # 控制是否保存相机内参
        self.intrinsics = None  # 保存相机内参对象
        self.save_frames_enabled = False  # 控制是否自动保存帧
        self.default_save_format = "bmp"  # 默认保存格式
        self.auto_acquire_intrinsics = True  # 控制相机初始化后是否自动获取相机内参
    
    def initialize_camera(self, width=1280, height=720, fps=30):
        """
        初始化RealSense相机
        
        Args:
            width (int): 图像宽度，默认1280
            height (int): 图像高度，默认720
            fps (int): 帧率，默认30
            
        Returns:
            tuple: (pipeline, profile) RealSense管道和配置对象
        """
        # 初始化RealSense相机
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.align = rs.align(rs.stream.color)
        
        # 启动相机
        self.profile = self.pipeline.start(config)
        
        # 如果启用自动获取内参，则获取第一帧内参
        if self.auto_acquire_intrinsics:
            try:
                # 等待第一帧
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if color_frame:
                    self.acquire_cam_intrinsics(color_frame)
            except Exception as e:
                print(f"自动获取相机内参失败: {e}")
        
        return self.pipeline, self.profile
    
    def stop_camera(self):
        """停止相机并释放资源"""
        if self.pipeline:
            self.pipeline.stop()
    
    def acquire_cam_intrinsics(self, color_frame):
        """
        获取相机内参和畸变系数
        
        Args:
            color_frame: RealSense彩色帧对象
            
        Returns:
            tuple: (mtx, dist) 相机内参矩阵和畸变系数
        """
        if not self.intrinsics_acquired:
            # 获取相机内参
            self.intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            self.mtx = np.array([
                [self.intrinsics.fx, 0, self.intrinsics.ppx],
                [0, self.intrinsics.fy, self.intrinsics.ppy],
                [0, 0, 1]
            ])
            
            # 畸变系数
            self.dist = np.array(self.intrinsics.coeffs)
            
            # 打印详细的相机内参信息
            print("=" * 50)
            print("相机内参详细信息:")
            print("=" * 50)
            print(f"相机型号: {self.intrinsics.model}")
            print(f"分辨率: {self.intrinsics.width} x {self.intrinsics.height}")
            print(f"焦距: fx={self.intrinsics.fx:.2f}, fy={self.intrinsics.fy:.2f}")
            print(f"主点: cx={self.intrinsics.ppx:.2f}, cy={self.intrinsics.ppy:.2f}")
            print(f"畸变模型: {self.intrinsics.model}")
            print("畸变系数:", self.intrinsics.coeffs)
            print("\n相机内参矩阵:")
            print(self.mtx)
            print("\n畸变系数:")
            print(self.dist)
            print("=" * 50)
            self.intrinsics_acquired = True
            # 如果启用保存内参，则保存到文件
            if self.save_intrinsics_enabled:
                self.save_camera_intrinsics()  
        
        return self.mtx, self.dist
    
    def get_cam_intrinsics(self):
        """
        获取当前类中的相机内参参数
        
        Returns:
            tuple: (mtx, dist, intrinsics) 相机内参矩阵、畸变系数和内参对象
                   如果尚未获取内参，则返回(None, None, None)
        """
        if self.intrinsics_acquired:
            return self.mtx, self.dist, self.intrinsics
        else:
            return None, None, None
    
    def is_intrinsics_acquired(self):
        """
        检查是否已获取相机内参
        
        Returns:
            bool: 是否已获取相机内参
        """
        return self.intrinsics_acquired
    
    # 新增接口：设置是否保存相机内参
    def set_save_intrinsics(self, enable=True):
        """
        设置是否保存相机内参到文件
        
        Args:
            enable (bool): 是否启用保存相机内参，默认为True
        """
        self.save_intrinsics_enabled = enable
        print(f"相机内参保存功能已{'启用' if enable else '禁用'}")
    
    # 新增接口：设置是否自动保存帧
    def set_auto_save_frames(self, enable=True, format="bmp"):
        """
        设置是否自动保存采集的帧
        
        Args:
            enable (bool): 是否启用自动保存帧，默认为True
            format (str): 保存格式，支持 'bmp', 'jpg', 'png'，默认为 'bmp'
        """
        self.save_frames_enabled = enable
        self.default_save_format = format.lower() if format.lower() in ["bmp", "jpg", "png"] else "bmp"
        print(f"自动保存帧功能已{'启用' if enable else '禁用'}，默认格式: {self.default_save_format}")
    
    # 新增接口：保存当前帧图像
    def save_frame(self, frame, filename=None, format=None):
        """
        保存当前帧图像
        
        Args:
            frame (np.ndarray): 要保存的图像帧
            filename (str): 保存的文件名，如果为None则自动生成
            format (str): 保存格式，支持 'bmp', 'jpg', 'png'，如果为None则使用默认格式
            
        Returns:
            str: 保存的文件路径
        """
        # 使用默认格式如果未指定
        if format is None:
            format = self.default_save_format
        
        # 支持的格式检查
        supported_formats = ["bmp", "jpg", "png"]
        if format.lower() not in supported_formats:
            raise ValueError(f"不支持的格式: {format}，支持的格式: {supported_formats}")
        
        # 生成文件名
        if filename is None:
            timestamp = int(time.time())
            filename = f"frame_{timestamp}.{format.lower()}"
        
        # 确保文件扩展名正确
        if not filename.endswith(f".{format.lower()}"):
            filename = f"{os.path.splitext(filename)[0]}.{format.lower()}"
        
        # 保存图像
        success = cv2.imwrite(filename, frame)
        if success:
            print(f"帧已保存到: {filename}")
            return filename
        else:
            raise RuntimeError(f"保存帧失败: {filename}")
    
    # 新增接口：保存相机内参到文件
    def save_camera_intrinsics(self):
        """
        保存相机内参到文件
        
        保存两个文件：
        1. camera_intrinsics.json - 详细的JSON格式（可读）
        2. camera_intrinsics.npz - NumPy格式（高效加载）
        """
        if not self.intrinsics_acquired:
            print("警告: 相机内参尚未获取，无法保存")
            return False
        
        try:
            # 准备保存目录
            save_dir = "save_parms"
            os.makedirs(save_dir, exist_ok=True)
            
            # 保存JSON格式（便于查看）
            json_file = os.path.join(save_dir, "camera_intrinsics.json")
            intrinsics_data = {
                "description": "RealSense相机内参（动态获取）",
                "version": "1.0",
                "camera_model": "RealSense D435/D405",
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "resolution": {
                    "width": int(self.intrinsics.width),
                    "height": int(self.intrinsics.height)
                },
                "intrinsics": {
                    "fx": float(self.intrinsics.fx),
                    "fy": float(self.intrinsics.fy),
                    "ppx": float(self.intrinsics.ppx),
                    "ppy": float(self.intrinsics.ppy),
                    "model": str(self.intrinsics.model)
                },
                "distortion": {
                    "coefficients": self.dist.tolist(),
                    "model": "Brown-Conrady"
                },
                "camera_matrix": self.mtx.tolist(),
                "notes": "此内参从相机第一帧动态获取，保证与当前相机配置匹配"
            }
            
            with open(json_file, 'w', encoding='utf-8') as f:
                json.dump(intrinsics_data, f, indent=2, ensure_ascii=False)
            
            # 保存NumPy格式（便于加载）
            npz_file = os.path.join(save_dir, "camera_intrinsics.npz")
            np.savez(
                npz_file,
                camera_matrix=self.mtx,
                dist_coeffs=self.dist,
                width=self.intrinsics.width,
                height=self.intrinsics.height,
                fx=self.intrinsics.fx,
                fy=self.intrinsics.fy,
                ppx=self.intrinsics.ppx,
                ppy=self.intrinsics.ppy
            )
            
            print(f"相机内参已保存到:")
            print(f"  JSON: {os.path.abspath(json_file)}")
            print(f"  NPZ:  {os.path.abspath(npz_file)}")
            
            return True
            
        except Exception as e:
            print(f"保存相机内参失败: {e}")
            return False
    
    def get_frames(self, timeout_ms=5000):
        """
        获取相机帧数据（彩色帧和深度帧）
        Args:
            timeout_ms (int): 等待帧的超时时间（毫秒），默认5000ms
            
        Returns:
            tuple: (color_image, depth_frame, color_frame) 彩色图像数组、深度帧、彩色帧对象
                   如果获取失败则返回(None, None, None)
        """
        try:
            if not self.pipeline:
                print("错误: 相机未初始化")
                return None, None, None
            """
            # 增加超时时间并添加重试机制
            frames = None
            for attempt in range(3):
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=2000)
                    break
                except RuntimeError as e:
                    if attempt < 2:
                        self.logger.warning(f"获取帧失败，重试 {attempt+1}/3...")
                        time.sleep(0.5)
                    else:
                        raise
            """
            # 获取帧数据
            frames = self.pipeline.wait_for_frames(timeout_ms=timeout_ms)
            
            # 对齐深度帧到彩色帧
            aligned_frames = self.align.process(frames)
            
            # 获取深度帧和彩色帧
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                print("警告: 帧数据不完整")
                return None, None, None
            
            # 将彩色帧转换为numpy数组
            color_image = np.asanyarray(color_frame.get_data())
            
            return color_image, depth_frame, color_frame
            
        except Exception as e:
            print(f"获取相机帧失败: {e}")
            return None, None, None
