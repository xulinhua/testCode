#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
眼在手外标定矩阵生成脚本
读取相机内参和标定点数据，生成眼在手外的标定矩阵
"""

import os
import json
import numpy as np
import glob

# 尝试导入scipy.spatial.transform.Rotation，如果失败则使用自定义实现
try:
    from scipy.spatial.transform import Rotation as R
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("警告: 未找到scipy.spatial.transform.Rotation，将使用自定义实现")

# 如果没有scipy，定义自定义旋转实现
def euler_to_matrix(rx, ry, rz):
    """将欧拉角转换为旋转矩阵（单位为度）"""
    # 转换为弧度
    rx = np.radians(rx)
    ry = np.radians(ry)
    rz = np.radians(rz)
    
    # 计算旋转矩阵
    sx, cx = np.sin(rx), np.cos(rx)
    sy, cy = np.sin(ry), np.cos(ry)
    sz, cz = np.sin(rz), np.cos(rz)
    
    # R = Rz * Ry * Rx
    R = np.array([
        [cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz],
        [cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz],
        [-sy,   sx*cy,            cx*cy           ]
    ])
    return R

def matrix_to_euler(R):
    """将旋转矩阵转换为欧拉角（单位为度）"""
    # 检查奇异性
    if abs(R[2,0]) != 1:
        ry = -np.arcsin(R[2,0])
        rx = np.arctan2(R[2,1]/np.cos(ry), R[2,2]/np.cos(ry))
        rz = np.arctan2(R[1,0]/np.cos(ry), R[0,0]/np.cos(ry))
    else:
        rz = 0
        if R[2,0] == -1:
            ry = np.pi/2
            rx = rz + np.arctan2(R[0,1], R[0,2])
        else:
            ry = -np.pi/2
            rx = -rz + np.arctan2(-R[0,1], -R[0,2])
    
    # 转换为度
    return np.degrees([rx, ry, rz])

def load_camera_intrinsics(intrinsics_path):
    """
    读取相机内参
    
    Args:
        intrinsics_path: 相机内参文件路径
        
    Returns:
        tuple: (camera_matrix, dist_coeffs)
    """
    with open(intrinsics_path, 'r') as f:
        data = json.load(f)
    
    # 提取相机内参矩阵
    camera_matrix = np.array(data['camera_matrix'])
    
    # 提取畸变系数
    dist_coeffs = np.array(data['distortion']['coefficients'])
    
    return camera_matrix, dist_coeffs

def load_calibration_data(coordinates_dir):
    """
    读取标定点数据
    
    Args:
        coordinates_dir: 标定点坐标数据目录
        
    Returns:
        list: 标定点数据列表，每个元素包含机器人位姿和标记位置
    """
    data_points = []
    
    # 获取所有坐标数据文件
    coord_files = glob.glob(os.path.join(coordinates_dir, "data_point_*.json"))
    coord_files.sort()  # 按文件名排序
    
    for filepath in coord_files:
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            # 提取数据
            index = data.get("index", 0)
            robot_pose = data.get("robot_pose", {})
            marker_position = data.get("marker_position", {})
            
            # 转换为所需的格式
            robot_pose_tuple = (
                robot_pose.get("x", 0),
                robot_pose.get("y", 0),
                robot_pose.get("z", 0),
                robot_pose.get("rx", 0),
                robot_pose.get("ry", 0),
                robot_pose.get("rz", 0)
            )
            
            marker_pos_tuple = (
                marker_position.get("x", 0),
                marker_position.get("y", 0),
                marker_position.get("z", 0)
            )
            
            data_points.append({
                "index": index,
                "robot_pose": robot_pose_tuple,
                "marker_position": marker_pos_tuple
            })
            
        except Exception as e:
            print(f"警告: 加载数据文件 {filepath} 失败: {e}")
            continue
    
    print(f"成功加载 {len(data_points)} 个标定点数据")
    return data_points

def euler_to_rotation_matrix(euler_angles):
    """
    将欧拉角(度)转换为旋转矩阵
    
    Args:
        euler_angles: (rx, ry, rz) 欧拉角，单位为度
        
    Returns:
        numpy.ndarray: 3x3 旋转矩阵
    """
    rx, ry, rz = euler_angles
    if HAS_SCIPY:
        # 使用scipy实现
        r = R.from_euler('xyz', [rx, ry, rz], degrees=True)
        return r.as_matrix()
    else:
        # 使用自定义实现
        return euler_to_matrix(rx, ry, rz)

def rotation_matrix_to_euler(rotation_matrix):
    """
    将旋转矩阵转换为欧拉角(度)
    
    Args:
        rotation_matrix: 3x3 旋转矩阵
        
    Returns:
        numpy.ndarray: (rx, ry, rz) 欧拉角，单位为度
    """
    if HAS_SCIPY:
        # 使用scipy实现
        r = R.from_matrix(rotation_matrix)
        return r.as_euler('xyz', degrees=True)
    else:
        # 使用自定义实现
        return matrix_to_euler(rotation_matrix)

def pose_to_homogeneous_matrix(pose):
    """
    将位姿(x, y, z, rx, ry, rz)转换为齐次变换矩阵
    
    Args:
        pose: (x, y, z, rx, ry, rz) 位姿，角度单位为度
        
    Returns:
        numpy.ndarray: 4x4 齐次变换矩阵
    """
    x, y, z, rx, ry, rz = pose
    rotation_matrix = euler_to_rotation_matrix((rx, ry, rz))
    
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = [x, y, z]
    
    return transform_matrix

def homogeneous_matrix_to_pose(transform_matrix):
    """
    将齐次变换矩阵转换为位姿(x, y, z, rx, ry, rz)
    
    Args:
        transform_matrix: 4x4 齐次变换矩阵
        
    Returns:
        tuple: (x, y, z, rx, ry, rz) 位姿，角度单位为度
    """
    rotation_matrix = transform_matrix[:3, :3]
    translation = transform_matrix[:3, 3]
    
    euler_angles = rotation_matrix_to_euler(rotation_matrix)
    
    return (translation[0], translation[1], translation[2], 
            euler_angles[0], euler_angles[1], euler_angles[2])

def compute_eye_to_hand_calibration(robot_poses, marker_positions):
    """
    计算眼在手外标定矩阵
    
    Args:
        robot_poses: 机器人位姿列表 [(x, y, z, rx, ry, rz), ...]
        marker_positions: 标记位置列表 [(x, y, z), ...]
        
    Returns:
        tuple: (T_cam2base, T_base2cam) 相机到基座的变换矩阵和基座到相机的变换矩阵
    """
    n = len(robot_poses)
    if n != len(marker_positions):
        raise ValueError("机器人位姿和标记位置数量不匹配")
    
    if n < 4:
        raise ValueError("至少需要4个标定点来计算标定矩阵")
    
    # 构建坐标矩阵
    # 机器人基座坐标系下的标记位置 (4×n)
    base_coords = np.ones((4, n))
    # 相机坐标系下的标记位置 (4×n)
    cam_coords = np.ones((4, n))
    
    # 填充坐标数据
    for i in range(n):
        # 机器人位姿转换为齐次变换矩阵
        T_base2end = pose_to_homogeneous_matrix(robot_poses[i])
        
        # 标记在相机坐标系下的位置
        cam_coords[:3, i] = marker_positions[i]
        
        # 标记在基座坐标系下的位置 (通过机器人位姿转换)
        marker_cam = np.array([*marker_positions[i], 1.0])
        marker_base = T_base2end @ marker_cam
        base_coords[:3, i] = marker_base[:3]
    
    # 使用SVD方法计算变换矩阵
    # T_cam2base 使得：base_coords ≈ T_cam2base * cam_coords
    T_cam2base = base_coords @ np.linalg.pinv(cam_coords)
    
    # 计算逆变换
    T_base2cam = np.linalg.pinv(T_cam2base)
    
    return T_cam2base, T_base2cam

def save_calibration_results(T_cam2base, T_base2cam, save_dir="save_parms"):
    """
    保存标定结果
    
    Args:
        T_cam2base: 相机到基座的变换矩阵
        T_base2cam: 基座到相机的变换矩阵
        save_dir: 保存目录
    """
    os.makedirs(save_dir, exist_ok=True)
    
    # 保存为.npy文件
    np.save(os.path.join(save_dir, "camera2base.npy"), T_cam2base)
    np.save(os.path.join(save_dir, "base2camera.npy"), T_base2cam)
    
    # 保存为.json文件（便于查看）
    cam2base_pose = homogeneous_matrix_to_pose(T_cam2base)
    base2cam_pose = homogeneous_matrix_to_pose(T_base2cam)
    
    result = {
        "camera2base": {
            "matrix": T_cam2base.tolist(),
            "pose": {
                "x": cam2base_pose[0],
                "y": cam2base_pose[1],
                "z": cam2base_pose[2],
                "rx": cam2base_pose[3],
                "ry": cam2base_pose[4],
                "rz": cam2base_pose[5]
            }
        },
        "base2camera": {
            "matrix": T_base2cam.tolist(),
            "pose": {
                "x": base2cam_pose[0],
                "y": base2cam_pose[1],
                "z": base2cam_pose[2],
                "rx": base2cam_pose[3],
                "ry": base2cam_pose[4],
                "rz": base2cam_pose[5]
            }
        }
    }
    
    with open(os.path.join(save_dir, "calibration_result.json"), 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    
    print(f"标定结果已保存到 {save_dir} 目录")

def main():
    """主函数"""
    # 配置路径
    camera_intrinsics_path = "save_parms/camera_intrinsics.json"
    coordinates_dir = "save_parms/coordinates"
    save_dir = "save_parms"
    
    print("眼在手外标定矩阵生成脚本")
    print("=" * 50)
    
    # 1. 读取相机内参
    print("1. 读取相机内参...")
    try:
        camera_matrix, dist_coeffs = load_camera_intrinsics(camera_intrinsics_path)
        print(f"   相机内参矩阵:\n{camera_matrix}")
        print(f"   畸变系数: {dist_coeffs}")
    except Exception as e:
        print(f"❌ 读取相机内参失败: {e}")
        return
    
    # 2. 读取标定点数据
    print("\n2. 读取标定点数据...")
    try:
        data_points = load_calibration_data(coordinates_dir)
        if len(data_points) == 0:
            print("❌ 未找到标定点数据")
            return
        
        # 提取机器人位姿和标记位置
        robot_poses = [point["robot_pose"] for point in data_points]
        marker_positions = [point["marker_position"] for point in data_points]
        
        print(f"   成功读取 {len(data_points)} 个标定点")
    except Exception as e:
        print(f"❌ 读取标定点数据失败: {e}")
        return
    
    # 3. 计算眼在手外标定矩阵
    print("\n3. 计算眼在手外标定矩阵...")
    try:
        T_cam2base, T_base2cam = compute_eye_to_hand_calibration(robot_poses, marker_positions)
        print("✅ 标定矩阵计算完成")
        print(f"   相机到基座变换矩阵:\n{T_cam2base}")
        print(f"   基座到相机变换矩阵:\n{T_base2cam}")
    except Exception as e:
        print(f"❌ 计算标定矩阵失败: {e}")
        return
    
    # 4. 保存标定结果
    print("\n4. 保存标定结果...")
    try:
        save_calibration_results(T_cam2base, T_base2cam, save_dir)
        print("✅ 标定结果保存完成")
    except Exception as e:
        print(f"❌ 保存标定结果失败: {e}")
        return
    
    print("\n" + "=" * 50)
    print("🎉 眼在手外标定完成！")

if __name__ == "__main__":
    main()