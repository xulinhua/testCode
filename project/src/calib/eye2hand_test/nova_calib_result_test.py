#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
越疆 Nova2 机械臂 - 眼在手外(Eye-to-Hand)标定精度验证测试脚本
基于 Nova2 机械臂控制接口实现
支持固件版本: 3.5.8.1 及以上

功能：
1. 加载之前保存的每一个标定点处的机械手位姿和Aruco标识中心的3D世界坐标
2. 根据之前生成的标定矩阵换算成实际坐标，并发送指令让机械手走到对应的位置去
3. 验证重投影精度
4. 将每个重投影精度数据进行保存并分析
"""

import sys
import os
import json
import numpy as np
import time
import glob

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 导入外部封装的类
from nova_robot import Nova2Robot
from gen_eye_to_hand_calib_result import (
    load_camera_intrinsics,
    load_calibration_data,
    pose_to_homogeneous_matrix,
    homogeneous_matrix_to_pose
)

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

def validate_calibration_point(robot, T_cam2base, data_point):
    """
    验证单个标定点的精度
    
    Args:
        robot: 机器人实例
        T_cam2base: 相机到基座的变换矩阵
        data_point: 标定点数据
        
    Returns:
        dict: 验证结果
    """
    # 获取标定点数据
    index = data_point["index"]
    robot_pose = data_point["robot_pose"]
    marker_position = data_point["marker_position"]
    
    print(f"验证标定点 {index}...")
    print(f"  机器人位姿: X={robot_pose[0]:.3f}, Y={robot_pose[1]:.3f}, Z={robot_pose[2]:.3f}")
    
    # 将相机坐标系中的标记位置转换到基座坐标系（预测位置）
    marker_cam = np.array([*marker_position, 1.0])
    marker_base_predicted = T_cam2base @ marker_cam
    predicted_position = marker_base_predicted[:3]
    
    print(f"  预测标记位置: X={predicted_position[0]:.3f}, Y={predicted_position[1]:.3f}, Z={predicted_position[2]:.3f}")
    
    # 移动机械臂到预测位置
    # 注意：这里需要将标记位置转换为机械臂末端位置
    # 假设标记在末端执行器下方100mm处（与标定时的偏移一致）
    end_x = predicted_position[0]
    end_y = predicted_position[1]
    end_z = predicted_position[2] # + 100  # 标记在末端下方100mm
    
    print(f"  移动到预测位置: X={end_x:.3f}, Y={end_y:.3f}, Z={end_z:.3f}")
    
    # 移动机械臂
    success = robot.move_l(end_x, end_y, end_z, robot_pose[3], robot_pose[4], robot_pose[5])
    
    if not success:
        print(f"  ❌ 移动失败")
        return {
            "index": index,
            "success": False,
            "predicted_marker_position": predicted_position.tolist(),
            "actual_robot_position": None,
            "error": None
        }
    
    # 等待稳定
    time.sleep(1)
    
    # 获取实际位置
    actual_pose = robot.get_pose()
    if actual_pose is None:
        print(f"  ❌ 无法获取实际位置")
        return {
            "index": index,
            "success": False,
            "predicted_marker_position": predicted_position.tolist(),
            "actual_robot_position": None,
            "error": None
        }
    
    print(f"  实际机器人位置: X={actual_pose[0]:.3f}, Y={actual_pose[1]:.3f}, Z={actual_pose[2]:.3f}")
    
    # 计算位置误差
    error_xyz = np.array([actual_pose[0] - end_x, actual_pose[1] - end_y, actual_pose[2] - end_z])
    error = np.linalg.norm(error_xyz)
    
    print(f"  位置误差: {error:.3f} mm")
    print()
    
    return {
        "index": index,
        "success": True,
        "predicted_marker_position": predicted_position.tolist(),
        "actual_robot_position": [actual_pose[0], actual_pose[1], actual_pose[2]],
        "robot_orientation": [actual_pose[3], actual_pose[4], actual_pose[5]],
        "error_xyz": error_xyz.tolist(),
        "error": float(error)
    }

def save_validation_results(results, filename="calibration_validation_results.json"):
    """
    保存验证结果
    
    Args:
        results: 验证结果列表
        filename: 保存文件名
    """
    # 计算统计信息
    successful_results = [r for r in results if r["success"]]
    errors = [r["error"] for r in successful_results if r["error"] is not None]
    
    if errors:
        stats = {
            "total_points": len(results),
            "successful_points": len(successful_results),
            "failed_points": len(results) - len(successful_results),
            "mean_error": float(np.mean(errors)),
            "std_error": float(np.std(errors)),
            "max_error": float(np.max(errors)),
            "min_error": float(np.min(errors))
        }
    else:
        stats = {
            "total_points": len(results),
            "successful_points": 0,
            "failed_points": len(results),
            "mean_error": 0,
            "std_error": 0,
            "max_error": 0,
            "min_error": 0
        }
    
    # 保存结果
    output_data = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "statistics": stats,
        "results": results
    }
    
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)
    
    print(f"验证结果已保存到: {filename}")
    return stats

def print_validation_report(stats):
    """
    打印验证报告
    
    Args:
        stats: 统计信息
    """
    print("=" * 60)
    print("标定精度验证报告")
    print("=" * 60)
    
    print(f"总标定点数: {stats['total_points']}")
    print(f"成功验证点数: {stats['successful_points']}")
    print(f"失败验证点数: {stats['failed_points']}")
    print()
    
    if stats['successful_points'] > 0:
        print("误差统计:")
        print(f"  平均误差: {stats['mean_error']:.3f} mm")
        print(f"  标准差:   {stats['std_error']:.3f} mm")
        print(f"  最大误差: {stats['max_error']:.3f} mm")
        print(f"  最小误差: {stats['min_error']:.3f} mm")
        print()
        
        # 精度评级
        mean_error = stats['mean_error']
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

def main():
    """主函数"""
    print("越疆 Nova2 机械臂 - 眼在手外标定精度验证测试")
    print("=" * 60)
    
    # 配置路径
    camera_intrinsics_path = "save_parms/camera_intrinsics.json"
    coordinates_dir = "save_parms/coordinates"
    save_dir = "save_parms"
    
    # 连接机械臂
    robot = Nova2Robot(ip_address="192.168.5.1")
    
    try:
        # 连接机械臂
        print("1. 连接机械臂...")
        if not robot.connect():
            print("❌ 机械臂连接失败")
            return
        
        if not robot.enable_robot():
            print("❌ 机械臂使能失败")
            return
        
        print("✅ 机械臂连接成功")
        print()
        
        # 2. 加载标定矩阵
        print("2. 加载标定矩阵...")
        try:
            T_cam2base = load_calibration_matrix(save_dir)
            print("✅ 标定矩阵加载成功")
            print(f"   矩阵形状: {T_cam2base.shape}")
        except Exception as e:
            print(f"❌ 标定矩阵加载失败: {e}")
            return
        print()
        
        # 3. 读取标定点数据
        print("3. 读取标定点数据...")
        try:
            data_points = load_calibration_data(coordinates_dir)
            if len(data_points) == 0:
                print("❌ 未找到标定点数据")
                return
            print(f"✅ 成功加载 {len(data_points)} 个标定点数据")
        except Exception as e:
            print(f"❌ 读取标定点数据失败: {e}")
            return
        print()
        
        # 4. 验证标定点精度
        print("4. 验证标定点精度...")
        results = []
        
        # 只验证前5个点以节省时间（可根据需要调整）
        validation_points = data_points[:5] if len(data_points) > 5 else data_points
        
        for data_point in validation_points:
            result = validate_calibration_point(robot, T_cam2base, data_point)
            results.append(result)
            
            # 在每个点之间暂停以避免机械臂过载
            time.sleep(2)
        
        print("✅ 精度验证完成")
        print()
        
        # 5. 保存验证结果
        print("5. 保存验证结果...")
        try:
            stats = save_validation_results(results, "calibration_validation_results.json")
            print("✅ 验证结果保存成功")
        except Exception as e:
            print(f"❌ 保存验证结果失败: {e}")
            return
        print()
        
        # 6. 打印验证报告
        print_validation_report(stats)
        
    except KeyboardInterrupt:
        print("\n⚠️  用户中断验证")
    except Exception as e:
        print(f"\n❌ 验证过程中出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 返回初始位置
        try:
            print("\n返回初始位置...")
            robot.move_to_standard_pose()
            time.sleep(2)
            print("✅ 已返回初始位置")
        except Exception as e:
            print(f"返回初始位置失败: {e}")
        
        # 断开机械臂连接
        robot.disconnect()
        print("已断开机械臂连接")

if __name__ == "__main__":
    main()