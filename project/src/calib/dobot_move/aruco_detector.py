import numpy as np
import cv2
import cv2.aruco as aruco
from sklearn.decomposition import PCA  # 导入PCA模块

# 尝试导入pyrealsense2，如果不存在则设置为None
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    rs = None
    REALSENSE_AVAILABLE = False

class Aruco_Detector:
    """Aruco码识别模块"""
    
    def __init__(self, marker_length=0.1, aruco_dict_type=aruco.DICT_5X5_100):
        """
        初始化Aruco检测器
        
        Args:
            marker_length (float): 标记实际大小（单位：米），默认0.1m
            aruco_dict_type: Aruco字典类型，默认DICT_5X5_100
        """
        self.marker_length = marker_length
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)
        self.parameters = aruco.DetectorParameters()
        # 优化检测参数以提高性能
        self.parameters.minCornerDistanceRate = 0.05
        self.parameters.minDistanceToBorder = 5
        self.parameters.minMarkerPerimeterRate = 0.1
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.05
        self.parameters.minOtsuStdDev = 5.0
        self.parameters.errorCorrectionRate = 0.6
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.camera_intrinsics = None  # 缓存相机内参
        self.enable_scaling = False  # 是否启用图像缩放以提高性能
        self.scale_factor = 0.5  # 图像缩放因子
        self.force_pnp = True  # 是否强制使用PnP算法，如果为True则不管是否有深度信息都使用PnP算法
    
    def set_camera_intrinsics(self, mtx, dist):
        """
        设置相机内参
        
        Args:
            mtx: 相机内参矩阵
            dist: 畸变系数
        """
        self.camera_intrinsics = (mtx, dist)

    def detect_markers(self, frame):
        """
        检测Aruco标记
        
        Args:
            frame: 输入图像帧
            
        Returns:
            tuple: (corners, ids, rejectedImgPoints) 检测结果
        """
        # 如果启用了图像缩放，则先缩放图像
        if self.enable_scaling and self.scale_factor != 1.0:
            height, width = frame.shape[:2]
            scaled_frame = cv2.resize(frame, (int(width * self.scale_factor), int(height * self.scale_factor)))
            scale_x, scale_y = width / (width * self.scale_factor), height / (height * self.scale_factor)
        else:
            scaled_frame = frame
            scale_x, scale_y = 1.0, 1.0
        
        # 转换为灰度图
        gray = cv2.cvtColor(scaled_frame, cv2.COLOR_BGR2GRAY)
        
        # 检测标记（新版API）
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
        
        # 如果进行了缩放，需要将角点坐标还原到原始图像尺寸
        if self.enable_scaling and self.scale_factor != 1.0 and corners is not None:
            for i in range(len(corners)):
                corners[i][0][:, 0] *= scale_x
                corners[i][0][:, 1] *= scale_y
        
        return corners, ids, rejectedImgPoints
    
    def get_marker_center_pixel(self, corners, marker_index=0):
        """
        获取Aruco标记的2D中心点像素坐标（优化方法）
        
        Args:
            corners: 检测到的标记角点坐标
            marker_index (int): 标记索引，默认为0
            
        Returns:
            tuple: (x, y) 中心点像素坐标
        """
        if corners is None or len(corners) <= marker_index:
            return None, None
            
        corner = corners[marker_index][0]
        # 使用对角线点计算中心点，提高精度
        x = float((corner[0][0] + corner[2][0]) / 2)
        y = float((corner[0][1] + corner[2][1]) / 2)
        
        return x, y
    
    def get_marker_result_with_depth(self, corners, depth_frame, intrinsics, marker_index=0):
        """
        利用深度相机获取Aruco标记的完整位姿信息（高精度方法，需要深度相机支持）
        
        Args:
            corners: 检测到的标记角点坐标
            depth_frame: 深度帧数据
            intrinsics: 相机内参对象
            marker_index (int): 标记索引，默认为0
            
        Returns:
            dict: 包含完整位姿信息的字典
                - 'center_2d': 2D中心点像素坐标 (x, y)
                - 'position': (x, y, z) 位置坐标（单位：米）
                - 'rotation': (rx, ry, rz) 旋转角度（单位：度）
                - 'rotation_matrix': 3x3旋转矩阵
                - 'rvec': 旋转向量
                - 'tvec': 平移向量
                - 'distance': 距离（单位：米）
                - 'marker_id': 标记ID
        """
        # 检查是否支持RealSense
        if not REALSENSE_AVAILABLE:
            raise RuntimeError("pyrealsense2未安装，无法获取3D坐标")
            
        if corners is None or depth_frame is None or intrinsics is None:
            return None
            
        try:
            # 获取标记的四个角点像素坐标
            corner_points = corners[marker_index][0].astype(np.int32)
            
            # 在标记区域内采样多个点生成3D点云
            point_cloud = []
            rect = cv2.boundingRect(corner_points)
            x, y, w, h = rect
            
            # 在标记区域内均匀采样（使用更密的采样，步长3）
            for i in range(0, w, 3):
                for j in range(0, h, 3):
                    u = x + i
                    v = y + j
                    
                    # 确保采样点在标记内部
                    if cv2.pointPolygonTest(corner_points, (u, v), False) >= 0:
                        depth = depth_frame.get_distance(u, v)
                        # 使用严格的深度过滤（0.1-10.0米）
                        if 0.1 < depth < 10.0:
                            point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth)  # 反投影：2D像素坐标 + 深度 -> 3D相机坐标系坐标
                            point_cloud.append(point_3d)
            
            # 需要足够多的点进行PCA分析（至少10个点）
            if len(point_cloud) < 10:
                return None
                
            point_cloud = np.array(point_cloud)
            
            # 使用PCA分析点云，获取标记平面的法向量
            pca = PCA(n_components=3)
            pca.fit(point_cloud)
            
            # 获取主成分（第三个主成分近似为法向量）
            normal_vector = pca.components_[2]
            
            # 调整法向量方向（指向相机）
            if normal_vector[2] > 0:
                normal_vector = -normal_vector
                
            # 归一化法向量
            normal_vector = normal_vector / np.linalg.norm(normal_vector)
            
            # 构造旋转矩阵（使用更稳定的方法）
            # 假设法向量为Z轴，构造X轴和Y轴
            temp_up = np.array([0, 0, -1.0])  # 临时上向量
            x_axis = np.cross(temp_up, normal_vector)
            if np.linalg.norm(x_axis) < 1e-6:
                # 如果法向量与临时向量平行，选择另一个临时向量
                temp_up = np.array([0, 1.0, 0])
                x_axis = np.cross(temp_up, normal_vector)
                
            x_axis = x_axis / np.linalg.norm(x_axis)
            y_axis = np.cross(normal_vector, x_axis)
            y_axis = y_axis / np.linalg.norm(y_axis)
            
            # 构建旋转矩阵（使用column_stack方法）
            rotation_matrix = np.column_stack((x_axis, y_axis, normal_vector))
            
            # 从旋转矩阵提取欧拉角（ZYX顺序：偏航、俯仰、滚转）
            sy = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
            singular = sy < 1e-6
            
            if not singular:
                rz = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])   # 偏航角Yaw
                ry = np.arctan2(-rotation_matrix[2, 0], sy)                     # 俯仰角Pitch
                rx = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])    # 滚转角Roll
            else:
                rz = np.arctan2(-rotation_matrix[0, 1], rotation_matrix[1, 1])
                ry = np.arctan2(-rotation_matrix[2, 0], sy)
                rx = 0.0
            
            # 转换为角度（度）
            rx_deg = np.degrees(rx)
            ry_deg = np.degrees(ry)
            rz_deg = np.degrees(rz)
            
            # 获取Aruco码中心的3D位置
            cnter_xPixel, cnter_yPixel = self.get_marker_center_pixel(corners, marker_index)
            if cnter_xPixel is None or cnter_yPixel is None:
                return None
                
            cnter_zPixel = depth_frame.get_distance(int(cnter_xPixel), int(cnter_yPixel))
            if cnter_zPixel <= 0:
                return None
                
            # 将像素坐标转换为3D空间坐标
            position = rs.rs2_deproject_pixel_to_point(intrinsics, [cnter_xPixel, cnter_yPixel], cnter_zPixel)
            
            # 计算距离
            distance = np.linalg.norm(position)
            
            # 从旋转矩阵计算旋转向量
            rvec, _ = cv2.Rodrigues(rotation_matrix)
            tvec = np.array(position, dtype=np.float32)
            
            # 获取2D中心点坐标
            center_2d = (cnter_xPixel, cnter_yPixel)
            
            return {
                'center_2d': center_2d,
                'position': tuple(position),
                'rotation': (rx_deg, ry_deg, rz_deg),
                'rotation_matrix': rotation_matrix,
                'rvec': rvec.flatten(),
                'tvec': tvec,
                'distance': distance,
                'marker_id': None  # marker_id需要在调用时传入
            }
            
        except Exception as e:
            print(f"点云法向量计算错误: {e}")
            return None

    def get_marker_result_pnp(self, corners, ids, mtx, dist, target_id=None):
        """
        使用PnP算法获取Aruco标记的完整位姿信息（位置和旋转）
        
        Args:
            corners: 标记角点
            ids: 标记ID数组
            mtx: 相机内参矩阵
            dist: 畸变系数
            target_id: 目标标记ID（可选，默认返回第一个检测到的标记）
            
        Returns:
            dict: 包含完整位姿信息的字典
                - 'center_2d': 2D中心点像素坐标 (x, y)
                - 'position': (x, y, z) 位置坐标（单位：米）
                - 'rotation': (rx, ry, rz) 旋转角度（单位：度）
                - 'rvec': 旋转向量
                - 'tvec': 平移向量
                - 'rotation_matrix': 3x3旋转矩阵
                - 'distance': 距离（单位：米）
                - 'marker_id': 标记ID
        """
        if ids is None or len(ids) == 0:
            return None
            
        # 估计每个标记的位姿
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, mtx, dist)
        
        # 查找目标标记
        target_idx = 0
        if target_id is not None:
            for i in range(len(ids)):
                if ids[i][0] == target_id:
                    target_idx = i
                    break
            else:
                # 未找到目标标记
                return None
        else:
            target_id = ids[target_idx][0]
        
        # 获取位置和旋转信息
        x, y, z = tvec[target_idx][0]
        rx, ry, rz = rvec[target_idx][0]
        position = (x, y, z)
        
        # 计算旋转矩阵
        rotation_matrix, _ = cv2.Rodrigues(rvec[target_idx][0])
        
        # 计算距离
        distance = np.linalg.norm(tvec[target_idx])
        
        # 转换旋转角度为度
        rx_deg = np.degrees(rx)
        ry_deg = np.degrees(ry)
        rz_deg = np.degrees(rz)
        
        # 获取2D中心点坐标
        center_x, center_y = self.get_marker_center_pixel(corners, target_idx)
        center_2d = (center_x, center_y) if center_x is not None and center_y is not None else None
        
        return {
            'center_2d': center_2d,
            'position': position,
            'rotation': (rx_deg, ry_deg, rz_deg),
            'rvec': rvec[target_idx][0],
            'tvec': tvec[target_idx][0],
            'rotation_matrix': rotation_matrix,
            'distance': distance,
            'marker_id': target_id        
        }

    def get_aruco_results(self, corners, ids, mtx, dist, depth_frame=None, intrinsics=None):
        """
        获取多个Aruco标记的识别结果
        
        根据是否提供深度信息自动选择计算方法
        
        Args:
            corners: 检测到的标记角点
            ids: 标记ID数组
            mtx: 相机内参矩阵
            dist: 畸变系数
            depth_frame: 深度帧数据（可选）
            intrinsics: 相机内参对象（可选）
            
        Returns:
            list: 包含每个标记识别结果的字典列表
                - 'center_2d': 2D中心点像素坐标 (x, y)
                - 'position': (x, y, z) 位置坐标（单位：米）
                - 'rotation': (rx, ry, rz) 旋转角度
                - 'rotation_matrix': 3x3旋转矩阵
                - 'rvec': 旋转向量
                - 'tvec': 平移向量
                - 'distance': 距离（单位：米）
                - 'marker_id': 标记ID
        """
        results = []
        
        if ids is None or len(ids) == 0:
            return results
            
        # 如果强制使用PnP算法，或者没有提供深度信息，则使用PnP方法计算位姿
        if self.force_pnp or depth_frame is None or intrinsics is None or not REALSENSE_AVAILABLE:
            # 使用PnP方法计算位姿
            # 为每个检测到的标记计算位姿
            for i in range(len(ids)):
                try:
                    pose_info = self.get_marker_result_pnp(corners, ids, mtx, dist, ids[i][0])
                    if pose_info is not None:
                        results.append(pose_info)
                except Exception as e:
                    # 即使某个标记计算失败，也继续处理其他标记
                    print(f"标记 {ids[i][0]} 计算失败: {e}")
                    continue
        else:
            # 使用深度方法计算完整位姿
            # 为每个检测到的标记计算位姿
            for i in range(len(ids)):
                try:
                    pose_info = self.get_marker_result_with_depth(corners, depth_frame, intrinsics, i)
                    if pose_info is not None:
                        # 更新marker_id
                        pose_info['marker_id'] = ids[i][0]
                        results.append(pose_info)
                except Exception as e:
                    # 即使某个标记计算失败，也继续处理其他标记
                    print(f"标记 {ids[i][0]} 计算失败: {e}")
                    continue
                        
        return results
    
    def draw_aruco_results(self, frame, markers_info, corners, ids, mtx, dist):
        """
        渲染绘制多个Aruco标记检测结果
        
        Args:
            frame: 输入图像帧
            markers_info: 标记信息列表，来自get_aruco_results的返回值
                - 'center_2d': 2D中心点像素坐标 (x, y)
                - 'position': (x, y, z) 位置坐标（单位：米）
                - 'rotation': (rx, ry, rz) 旋转角度（单位：度）
                - 'rotation_matrix': 3x3旋转矩阵
                - 'rvec': 旋转向量
                - 'tvec': 平移向量
                - 'distance': 距离（单位：米）
                - 'marker_id': 标记ID
            corners: 标记角点
            ids: 标记ID数组
            mtx: 相机内参矩阵
            dist: 畸变系数
            
        Returns:
            frame: 绘制了检测结果的图像帧
        """
        # 检查是否找到了标记
        if ids is None or len(ids) == 0 or len(markers_info) == 0:
            cv2.putText(frame, "No markers detected", (20, 40), self.font, 1, (0, 0, 255), 2)
            return frame
            
        # 绘制标记边界
        aruco.drawDetectedMarkers(frame, corners)
        
        # 为每个检测到的标记绘制坐标系和显示信息
        for i, marker_info in enumerate(markers_info):
            # 获取标记信息
            marker_id = marker_info['marker_id']
            position = marker_info['position']
            rotation = marker_info['rotation']
            center_2d = marker_info.get('center_2d', None)
            rvec = marker_info.get('rvec', None)
            tvec = marker_info.get('tvec', None)
            
            # 使用提供的rvec和tvec，或者从旋转矩阵计算
            if rvec is not None and tvec is not None:
                # 确保rvec和tvec是正确的形状
                rvec = np.array(rvec, dtype=np.float32)
                tvec = np.array(tvec, dtype=np.float32)
                if rvec.shape[0] == 3:
                    rvec = rvec.reshape((3, 1))
                if tvec.shape[0] == 3:
                    tvec = tvec.reshape((3, 1))
            elif 'rotation_matrix' in marker_info:
                # 从旋转矩阵计算rvec和tvec
                rvec, _ = cv2.Rodrigues(marker_info['rotation_matrix'])
                tvec = np.array(position, dtype=np.float32)
                if rvec.shape[0] == 3:
                    rvec = rvec.reshape((3, 1))
                if tvec.shape[0] == 3:
                    tvec = tvec.reshape((3, 1))
            else:
                # 如果没有旋转信息，使用默认值
                rvec = np.array([0, 0, 0], dtype=np.float32).reshape((3, 1))
                tvec = np.array(position, dtype=np.float32).reshape((3, 1))
            
            # 绘制标记坐标系
            cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, self.marker_length)
            
            # 显示信息（每个标记单独显示）
            text_y = 40 + i * 120  # 增加垂直间距以容纳更多信息
            cv2.putText(frame, f"ID: {marker_id}", (20, text_y), self.font, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f"Position (m): X:{position[0]:.3f} Y:{position[1]:.3f} Z:{position[2]:.3f}",
                        (20, text_y + 25), self.font, 0.6, (0, 255, 0), 1)
            
            cv2.putText(frame, f"Rotation (deg): X:{rotation[0]:.3f} Y:{rotation[1]:.3f} Z:{rotation[2]:.3f}",
                        (20, text_y + 50), self.font, 0.6, (0, 255, 0), 1)
            
            # 显示距离信息
            distance = marker_info.get('distance', None)
            if distance is not None:
                cv2.putText(frame, f"Distance: {distance:.3f}m",
                            (20, text_y + 75), self.font, 0.6, (255, 0, 0), 2)

            # 显示2D中心点坐标（如果可用）
            if center_2d is not None:
                cv2.putText(frame, f"Center (px): X:{center_2d[0]:.1f} Y:{center_2d[1]:.1f}",
                            (20, text_y + 100), self.font, 0.6, (255, 255, 0), 1)
            
            # 在标记旁边显示ID（使用center_2d或角点中心）
            if center_2d is not None:
                cv2.putText(frame, str(marker_id), (int(center_2d[0]), int(center_2d[1])), self.font, 0.8, (0, 0, 255), 2)
            elif corners is not None and i < len(corners):
                center = corners[i][0].mean(axis=0).astype(int)
                cv2.putText(frame, str(marker_id), (center[0], center[1]), self.font, 0.8, (0, 0, 255), 2)
        
        return frame

    def print_aruco_results(self, markers_info):
        """
        打印多个标记位姿信息到控制台
        
        Args:
            markers_info (list): 包含每个标记位姿信息的字典列表，来自get_aruco_results的返回值
                - 'center_2d': 2D中心点像素坐标 (x, y)
                - 'position': (x, y, z) 位置坐标（单位：米）
                - 'rotation': (rx, ry, rz) 旋转角度（单位：度）
                - 'rotation_matrix': 3x3旋转矩阵
                - 'rvec': 旋转向量
                - 'tvec': 平移向量
                - 'distance': 距离（单位：米）
                - 'marker_id': 标记ID
        """
        # 为每个检测到的标记打印信息
        for i, marker_info in enumerate(markers_info):
            marker_id = marker_info['marker_id']
            position = marker_info['position']
            rotation = marker_info['rotation']
            center_2d = marker_info.get('center_2d', None)
            distance = marker_info.get('distance', None)
            
            print(f"=== Marker {marker_id} ===")
            print(f"Position (m): X:{position[0]:.3f} Y:{position[1]:.3f} Z:{position[2]:.3f}")
            print(f"Rotation (deg): X:{rotation[0]:.3f} Y:{rotation[1]:.3f} Z:{rotation[2]:.3f}")
            
            # 打印距离信息（如果可用）
            if distance is not None:
                print(f"Distance: {distance:.3f}m")
            
            # 打印2D中心点坐标（如果可用）
            if center_2d is not None:
                print(f"Center (px): X:{center_2d[0]:.1f} Y:{center_2d[1]:.1f}")
            
            # 如果有旋转矩阵，计算并显示欧拉角（角度制）
            if 'rotation_matrix' in marker_info:
                # 从旋转矩阵计算欧拉角（角度制）
                sy = np.sqrt(marker_info['rotation_matrix'][0, 0]**2 + marker_info['rotation_matrix'][1, 0]**2)
                singular = sy < 1e-6
                
                if not singular:
                    rz = np.arctan2(marker_info['rotation_matrix'][1, 0], marker_info['rotation_matrix'][0, 0])   # 偏航角Yaw
                    ry = np.arctan2(-marker_info['rotation_matrix'][2, 0], sy)                     # 俯仰角Pitch
                    rx = np.arctan2(marker_info['rotation_matrix'][2, 1], marker_info['rotation_matrix'][2, 2])    # 滚转角Roll
                else:
                    rz = np.arctan2(-marker_info['rotation_matrix'][0, 1], marker_info['rotation_matrix'][1, 1])
                    ry = np.arctan2(-marker_info['rotation_matrix'][2, 0], sy)
                    rx = 0.0
                
                # 转换为角度
                rx_deg = np.degrees(rx)
                ry_deg = np.degrees(ry)
                rz_deg = np.degrees(rz)
                
                print(f"Rotation (deg): X:{rx_deg:.1f} Y:{ry_deg:.1f} Z:{rz_deg:.1f}")
            
            # 分隔线（如果不是最后一个标记）
            if i < len(markers_info) - 1:
                print("-" * 30)
    
    def detect_and_process_markers(self, frame, depth_frame=None, draw_results=True, print_results=True):
        """
        检测并处理Aruco标记，封装了完整的检测和处理流程
        
        Args:
            frame: 输入图像帧
            depth_frame: 深度帧数据（可选）
            draw_results (bool): 是否绘制检测结果，默认True
            print_results (bool): 是否打印位姿信息，默认True
            
        Returns:
            dict: 包含处理结果的字典
                - 'markers_info': 标记信息列表
                - 'frame': 处理后的图像帧（如果draw_results为True）
                - 'found': 是否检测到标记
        """
        # 检测Aruco标记
        corners, ids, rejectedImgPoints = self.detect_markers(frame)
        
        # 检查是否已设置相机内参
        if self.camera_intrinsics is None:
            raise RuntimeError("相机内参未设置，请先调用set_camera_intrinsics方法")
        
        # 获取相机内参
        mtx, dist = self.camera_intrinsics
        
        # 获取标记的完整位姿信息
        # 如果提供了depth_frame，则尝试从中提取intrinsics对象
        intrinsics = None
        if depth_frame is not None:
            # 尝试从depth_frame中提取intrinsics对象
            try:
                if hasattr(depth_frame, 'profile'):
                    intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            except Exception as e:
                print(f"无法从depth_frame提取intrinsics: {e}")
        
        markers_info = self.get_aruco_results(corners, ids, mtx, dist, depth_frame, intrinsics)
        
        result = {
            'markers_info': markers_info,
            'found': len(markers_info) > 0
        }
        
        # 如果需要绘制结果
        if draw_results and len(markers_info) > 0:
            processed_frame = self.draw_aruco_results(frame, markers_info, corners, ids, mtx, dist)
            result['frame'] = processed_frame
        elif draw_results:
            # 没有检测到标记，但在需要绘制结果的情况下显示提示信息
            cv2.putText(frame, "No markers detected", (20, 40), self.font, 1, (0, 0, 255), 2)
            result['frame'] = frame
        
        # 如果需要打印结果
        if print_results and len(markers_info) > 0:
            self.print_aruco_results(markers_info)
        elif print_results:
            print("No markers detected")
        
        return result
