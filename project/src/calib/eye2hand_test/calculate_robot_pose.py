#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从ArUco标记位置反推机器人末端执行器位姿
演示如何使用手眼标定矩阵进行坐标转换
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

def euler_to_rotation_matrix(euler_angles):
    """
    将欧拉角(度)转换为旋转矩阵
    
    Args:
        euler_angles: (rx, ry, rz) 欧拉角，单位为度
        
    Returns:
        numpy.ndarray: 3x3 旋转矩阵
    """
    rx, ry, rz = euler_angles
    r = R.from_euler('xyz', [rx, ry, rz], degrees=True)
    return r.as_matrix()

def rotation_matrix_to_euler(rotation_matrix):
    """
    将旋转矩阵转换为欧拉角(度)
    
    Args:
        rotation_matrix: 3x3 旋转矩阵
        
    Returns:
        numpy.ndarray: (rx, ry, rz) 欧拉角，单位为度
    """
    r = R.from_matrix(rotation_matrix)
    return r.as_euler('xyz', degrees=True)

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

def load_calibration_matrix(save_dir="save_parms"):
    """
    加载标定矩阵
    
    Args:
        save_dir: 保存目录
        
    Returns:
        numpy.ndarray: 相机到基座的变换矩阵
    """
    import os
    cam2base_file = os.path.join(save_dir, "camera2base.npy")
    if not os.path.exists(cam2base_file):
        raise FileNotFoundError(f"标定矩阵文件不存在: {cam2base_file}")
    
    T_cam2base = np.load(cam2base_file)
    return T_cam2base

def main():
    """主函数"""
    print("从ArUco标记位置反推机器人末端执行器位姿")
    print("=" * 60)
    
    # 从data_point_001.json中读取的数据
    marker_position_cam = (0.14661683731548902, -0.0735653743515614, 0.7658054498192921)
    print(f"1. ArUco标记在相机坐标系中的位置:")
    print(f"   [{marker_position_cam[0]:.3f}, {marker_position_cam[1]:.3f}, {marker_position_cam[2]:.3f}]")
    print()
    
    # 从文件中读取的实际机器人位姿
    actual_robot_pose = (-163.049988, -325.589844, 232.990005, -179.979996, 0.010006, 90.07)
    print(f"2. 实际的机器人末端执行器位姿 (从文件读取):")
    print(f"   [{actual_robot_pose[0]:.3f}, {actual_robot_pose[1]:.3f}, {actual_robot_pose[2]:.3f},")
    print(f"    {actual_robot_pose[3]:.3f}, {actual_robot_pose[4]:.3f}, {actual_robot_pose[5]:.3f}]")
    print()
    
    try:
        # 加载标定矩阵
        T_cam2base = load_calibration_matrix()
        print("3. 加载的相机到基座坐标系的变换矩阵:")
        print(f"   {T_cam2base}")
        print()
        
        # 将标记位置从相机坐标系转换到基座坐标系
        marker_cam = np.array([*marker_position_cam, 1.0])
        marker_base = T_cam2base @ marker_cam
        marker_base_pos = marker_base[:3]
        print(f"4. 使用标定矩阵将标记转换到基座坐标系:")
        print(f"   [{marker_base_pos[0]:.3f}, {marker_base_pos[1]:.3f}, {marker_base_pos[2]:.3f}]")
        print()
        
        # 为了从标记位置反推机器人位姿，我们需要知道：
        # 1. 标记相对于机器人末端执行器的固定偏移
        # 2. 这个偏移通常在标定过程中是已知的
        
        # 假设我们知道标记相对于机器人末端执行器的偏移
        # 这个值通常是在标定设置时测量得到的
        # 例如，假设标记在末端执行器下方100mm处
        marker_offset_from_end = np.array([0.0, 0.0, -0.0, 1.0])  # 示例偏移
        
        # 计算机器人末端执行器在基座坐标系中的位置
        # end_base = marker_base - marker_offset_from_end
        end_base_pos = marker_base_pos - marker_offset_from_end[:3]
        print(f"5. 假设标记相对于末端执行器的偏移为 [0, 0, 0] mm:")
        print(f"   机器人末端执行器位置: [{end_base_pos[0]:.3f}, {end_base_pos[1]:.3f}, {end_base_pos[2]:.3f}]")
        print()
        
        # 更准确的方法是使用逆向运动学
        # 但这需要机器人的具体参数和逆解算法
        print("6. 说明:")
        print("   要准确地从标记位置计算机器人位姿，需要:")
        print("   a) 精确的标记相对于末端执行器的偏移量")
        print("   b) 机器人逆运动学算法")
        print("   c) 考虑机器人关节角度的约束")
        print()
        print("   在实际应用中，这个过程通常由机器人控制器完成")
        
    except FileNotFoundError:
        print("未找到标定矩阵文件，请先运行标定程序")
    except Exception as e:
        print(f"计算过程中出错: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()