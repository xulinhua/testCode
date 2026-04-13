#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
改进的标定矩阵精度验证脚本
通过重投影误差分析标定矩阵的精度，并根据ArUco标记位置反推机器人位姿
"""

import os
import json
import numpy as np
import glob
from scipy.spatial.transform import Rotation as R

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
    
    print("加载的标定点数据:")
    print("-" * 120)
    print(f"{'索引':<6} {'机器人末端位姿 (x, y, z, rx, ry, rz)':<50} {'相机中标记位置 (x, y, z)':<30}")
    print("-" * 120)
    
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
            
            # 打印每个点的坐标数据
            robot_pose_str = f"({robot_pose_tuple[0]:8.3f}, {robot_pose_tuple[1]:8.3f}, {robot_pose_tuple[2]:8.3f}, {robot_pose_tuple[3]:7.3f}, {robot_pose_tuple[4]:7.3f}, {robot_pose_tuple[5]:7.3f})"
            marker_pos_str = f"({marker_pos_tuple[0]:8.3f}, {marker_pos_tuple[1]:8.3f}, {marker_pos_tuple[2]:8.3f})"
            print(f"{index:<6} {robot_pose_str:<50} {marker_pos_str:<30}")
            
        except Exception as e:
            print(f"警告: 加载数据文件 {filepath} 失败: {e}")
            continue
    
    print("-" * 120)
    print(f"成功加载 {len(data_points)} 个标定点数据\n")
    print("注意: 上表中的'机器人末端位姿'是机械臂末端执行器的位姿")
    print("      '相机中标记位置'是ArUco标记在相机坐标系中的位置\n")
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
    # 注意: 这里假设欧拉角顺序为ZYX(即Rz*Ry*Rx)
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

def calculate_robot_pose_from_marker(T_cam2base, marker_position, T_marker2end=None):
    """
    根据ArUco标记位置计算机器人末端执行器位姿
    
    Args:
        T_cam2base: 相机到基座的变换矩阵
        marker_position: 标记在相机坐标系中的位置 (x, y, z)
        T_marker2end: 标记到末端执行器的变换矩阵（如果为None，则假设有固定偏移）
        
    Returns:
        tuple: (position, T_end2base) 末端执行器位置和变换矩阵
    """
    # 将标记位置从相机坐标系转换到基座坐标系
    marker_cam = np.array([*marker_position, 1.0])
    marker_base = T_cam2base @ marker_cam
    
    if T_marker2end is None:
        # 假设标记相对于末端执行器有一个固定的偏移
        # 这里我们使用一个示例偏移量，实际应用中需要根据具体设置调整
        # 例如，假设标记在末端执行器下方100mm处
        marker_offset = np.array([0.0, 0.0, 0.0, 1.0])
        
        # 计算末端执行器在基座坐标系中的位置
        end_base = marker_base - marker_offset
    else:
        # 使用提供的变换矩阵
        end_base = T_marker2end @ marker_base
    
    return end_base[:3]

def analyze_calibration_accuracy(T_cam2base, data_points):
    """
    分析标定精度
    
    Args:
        T_cam2base: 相机到基座的变换矩阵
        data_points: 标定点数据列表
        
    Returns:
        dict: 精度分析结果
    """
    print("\n手眼标定原理说明:")
    print("==================")
    print("在手眼标定中，我们计算相机坐标系到机器人基座坐标系的变换矩阵T_cam2base")
    print("验证方法：使用该矩阵将标记在相机坐标系中的位置转换到基座坐标系，与实际位置进行比较")
    print("实际位置：通过机器人正运动学，将末端执行器位姿转换为标记在基座坐标系中的位置\n")
    
    errors = []
    errors_x, errors_y, errors_z = [], [], []
    
    print("开始精度分析...")
    print("-" * 80)
    
    for i, point in enumerate(data_points):
        # 获取实际的机器人位姿
        robot_pose = point["robot_pose"]
        marker_position = point["marker_position"]
        
        # 将相机坐标系中的标记位置转换到基座坐标系（预测位置）
        marker_cam = np.array([*marker_position, 1.0])
        marker_base_predicted = T_cam2base @ marker_cam
        predicted_position = marker_base_predicted[:3]
        
        # 实际的标记位置（通过机器人位姿获得）
        T_base2end = pose_to_homogeneous_matrix(robot_pose)
        marker_base_actual = T_base2end @ marker_cam
        actual_position = marker_base_actual[:3]
        
        # 计算重投影误差
        error = np.linalg.norm(predicted_position - actual_position)
        error_xyz = np.abs(predicted_position - actual_position)
        
        errors.append(error)
        errors_x.append(error_xyz[0])
        errors_y.append(error_xyz[1])
        errors_z.append(error_xyz[2])
        
        # 根据ArUco标记位置计算机器人位姿并与文件中保存的位姿比较
        if i < 5:  # 只打印前5个点的详细信息
            print(f"点 {point['index']:2d}:")
            print(f"        相机中标记位置: [{marker_position[0]:8.3f}, {marker_position[1]:8.3f}, {marker_position[2]:8.3f}]")
            print(f"        预测标记位置:   [{predicted_position[0]:8.3f}, {predicted_position[1]:8.3f}, {predicted_position[2]:8.3f}]")
            print(f"        实际标记位置:   [{actual_position[0]:8.3f}, {actual_position[1]:8.3f}, {actual_position[2]:8.3f}]")
            print(f"        重投影误差: {error:6.3f} mm")
            
            # 根据标记位置反推机器人位姿
            calculated_end_position = calculate_robot_pose_from_marker(T_cam2base, marker_position)
            
            # 文件中保存的机器人位姿
            saved_end_position = np.array([robot_pose[0], robot_pose[1], robot_pose[2]])
            
            # 计算差值
            position_diff = calculated_end_position - saved_end_position
            
            print(f"        文件中保存的机器人位置: [{saved_end_position[0]:8.3f}, {saved_end_position[1]:8.3f}, {saved_end_position[2]:8.3f}]")
            print(f"        根据标记计算的机器人位置: [{calculated_end_position[0]:8.3f}, {calculated_end_position[1]:8.3f}, {calculated_end_position[2]:8.3f}]")
            print(f"        位置差值: [{position_diff[0]:8.3f}, {position_diff[1]:8.3f}, {position_diff[2]:8.3f}] mm")
            print(f"        差值模长: {np.linalg.norm(position_diff):6.3f} mm")
            print()
    
    # 计算统计信息
    errors = np.array(errors)
    errors_x = np.array(errors_x)
    errors_y = np.array(errors_y)
    errors_z = np.array(errors_z)
    
    mean_error = np.mean(errors)
    std_error = np.std(errors)
    max_error = np.max(errors)
    min_error = np.min(errors)
    
    mean_error_xyz = [np.mean(errors_x), np.mean(errors_y), np.mean(errors_z)]
    std_error_xyz = [np.std(errors_x), np.std(errors_y), np.std(errors_z)]
    
    # 计算机器人位姿差值统计
    print("机器人位姿差值统计 (前5个点):")
    print("-" * 60)
    position_diffs = []
    for i, point in enumerate(data_points[:5]):
        robot_pose = point["robot_pose"]
        marker_position = point["marker_position"]
        
        # 根据标记位置反推机器人位姿
        calculated_end_position = calculate_robot_pose_from_marker(T_cam2base, marker_position)
        
        # 文件中保存的机器人位姿
        saved_end_position = np.array([robot_pose[0], robot_pose[1], robot_pose[2]])
        
        # 计算差值
        position_diff = calculated_end_position - saved_end_position
        position_diffs.append(np.linalg.norm(position_diff))
        
        print(f"点 {point['index']:2d}: 差值模长 = {np.linalg.norm(position_diff):6.3f} mm")
    
    if position_diffs:
        mean_diff = np.mean(position_diffs)
        std_diff = np.std(position_diffs)
        print(f"\n平均差值: {mean_diff:.3f} mm (标准差: {std_diff:.3f} mm)")
    
    # 计算误差分布
    error_distribution = {
        "< 1mm": np.sum(errors < 1),
        "1-2mm": np.sum((errors >= 1) & (errors < 2)),
        "2-5mm": np.sum((errors >= 2) & (errors < 5)),
        "5-10mm": np.sum((errors >= 5) & (errors < 10)),
        ">= 10mm": np.sum(errors >= 10)
    }
    
    return {
        "total_points": len(errors),
        "mean_error": mean_error,
        "std_error": std_error,
        "max_error": max_error,
        "min_error": min_error,
        "mean_error_xyz": mean_error_xyz,
        "std_error_xyz": std_error_xyz,
        "error_distribution": error_distribution,
        "all_errors": errors
    }

def print_accuracy_report(analysis_result):
    """
    打印精度分析报告
    
    Args:
        analysis_result: 精度分析结果
    """
    print("=" * 60)
    print("标定精度分析报告")
    print("=" * 60)
    
    print(f"标定点总数: {analysis_result['total_points']}")
    print()
    
    print("总体误差统计:")
    print(f"  平均误差: {analysis_result['mean_error']:.3f} mm")
    print(f"  标准差:   {analysis_result['std_error']:.3f} mm")
    print(f"  最大误差: {analysis_result['max_error']:.3f} mm")
    print(f"  最小误差: {analysis_result['min_error']:.3f} mm")
    print()
    
    print("各轴向误差统计:")
    print(f"  X轴平均误差: {analysis_result['mean_error_xyz'][0]:.3f} mm (标准差: {analysis_result['std_error_xyz'][0]:.3f} mm)")
    print(f"  Y轴平均误差: {analysis_result['mean_error_xyz'][1]:.3f} mm (标准差: {analysis_result['std_error_xyz'][1]:.3f} mm)")
    print(f"  Z轴平均误差: {analysis_result['mean_error_xyz'][2]:.3f} mm (标准差: {analysis_result['std_error_xyz'][2]:.3f} mm)")
    print()
    
    print("误差分布:")
    total = analysis_result['total_points']
    for range_desc, count in analysis_result['error_distribution'].items():
        percentage = count / total * 100
        print(f"  {range_desc:8s}: {count:3d} 点 ({percentage:5.1f}%)")
    print()
    
    # 精度评级
    mean_error = analysis_result['mean_error']
    if mean_error < 1.0:
        quality = "⭐⭐⭐⭐⭐ 优秀 (亚毫米级精度)"
        recommendation = "✅ 标定质量极佳，可直接用于高精度应用"
    elif mean_error < 2.0:
        quality = "⭐⭐⭐⭐ 良好 (毫米级精度)"
        recommendation = "✅ 标定质量良好，适用于大多数工业应用"
    elif mean_error < 5.0:
        quality = "⭐⭐⭐ 一般"
        recommendation = "⚠️ 标定质量一般，建议检查标定点分布和环境条件"
    else:
        quality = "⭐ 需要改进"
        recommendation = "❌ 标定质量较差，建议重新标定"
    
    print("精度评级:")
    print(f"  {quality}")
    print(f"  {recommendation}")
    print("=" * 60)

def load_calibration_matrix(save_dir="save_parms"):
    """
    加载标定矩阵
    
    Args:
        save_dir: 保存目录
        
    Returns:
        numpy.ndarray: 相机到基座的变换矩阵
    """
    cam2base_file = os.path.join(save_dir, "camera2base.npy")
    if not os.path.exists(cam2base_file):
        raise FileNotFoundError(f"标定矩阵文件不存在: {cam2base_file}")
    
    T_cam2base = np.load(cam2base_file)
    return T_cam2base

def main():
    """主函数"""
    # 配置路径
    camera_intrinsics_path = "save_parms/camera_intrinsics.json"
    coordinates_dir = "save_parms/coordinates"
    save_dir = "save_parms"
    
    print("改进的标定矩阵精度验证脚本")
    print("=" * 50)
    
    try:
        # 1. 加载标定矩阵
        print("1. 加载标定矩阵...")
        T_cam2base = load_calibration_matrix(save_dir)
        print("✅ 标定矩阵加载成功")
        print(f"   矩阵形状: {T_cam2base.shape}")
        print()
        
        # 2. 读取标定点数据
        print("2. 读取标定点数据...")
        data_points = load_calibration_data(coordinates_dir)
        if len(data_points) == 0:
            print("❌ 未找到标定点数据")
            return
        print()
        
        # 3. 分析标定精度
        print("3. 分析标定精度...")
        analysis_result = analyze_calibration_accuracy(T_cam2base, data_points)
        print()
        
        # 4. 打印精度报告
        print_accuracy_report(analysis_result)
        
    except Exception as e:
        print(f"❌ 精度验证过程中出错: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()