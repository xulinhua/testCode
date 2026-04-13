#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>  // 添加时间测量头文件
#include "hand_eye_calib/calib_robot_pos_mgr.hpp"

using namespace handeyecalib;

int main() {
    std::cout << "=== 机械臂坐标数据管理模块测试 ===" << std::endl;
    
    // 创建机械臂坐标数据管理器
    CalibRobotPosMgr mgr;
    
    // 设置标准位置（示例值，实际应用中可能需要从配置文件读取）
    mgr.setStandardPose(100.0, 200.0, 300.0, 0.0, 0.0, 0.0);
    
    // 获取标准位置
    RobotPoseData standard_pose = mgr.getStandardPose();
    std::cout << "标准位置: X=" << standard_pose.x << ", Y=" << standard_pose.y 
              << ", Z=" << standard_pose.z << ", Rx=" << standard_pose.rx 
              << ", Ry=" << standard_pose.ry << ", Rz=" << standard_pose.rz << std::endl;
    
    // 初始化缓存数据点（在调用getPointCount或getDataPoint之前必须调用）
    std::cout << "\n初始化缓存数据点..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    mgr.initializeCachedDataPoints();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "initializeCachedDataPoints耗时: " << duration.count() << " 微秒" << std::endl;
    
    // 测试性能优化 - 测量多次调用getDataPoint的时间
    std::cout << "\n性能测试:" << std::endl;
    
    start_time = std::chrono::high_resolution_clock::now();
    
    // 多次调用getDataPoint测试性能
    for (int i = 0; i < 1000; ++i) {
        RobotPoseData point = mgr.getDataPoint(10);
        // 避免编译器优化掉循环
        if (i == 0) {
            std::cout << "  点 10: X=" << point.x << ", Y=" << point.y 
                      << ", Z=" << point.z << ", Rx=" << point.rx 
                      << ", Ry=" << point.ry << ", Rz=" << point.rz << std::endl;
        }
    }
    
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "1000次getDataPoint调用耗时: " << duration.count() << " 微秒" << std::endl;
    
    // 生成数据点
    start_time = std::chrono::high_resolution_clock::now();
    std::vector<RobotPoseData> data_points = mgr.generateDataPoints();
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "generateDataPoints耗时: " << duration.count() << " 微秒" << std::endl;
    
    std::cout << "生成的数据点数量: " << data_points.size() << std::endl;
    
    // 显示前几个数据点
    std::cout << "\n前5个数据点:" << std::endl;
    for (std::size_t i = 0; i < std::min(static_cast<std::size_t>(5), data_points.size()); ++i) {
        const RobotPoseData& point = data_points[i];
        std::cout << "  点 " << i << ": X=" << point.x << ", Y=" << point.y 
                  << ", Z=" << point.z << ", Rx=" << point.rx 
                  << ", Ry=" << point.ry << ", Rz=" << point.rz << std::endl;
    }
    
    // 演示获取指定索引的数据点
    std::cout << "\n演示获取指定索引的数据点:" << std::endl;
    RobotPoseData point10 = mgr.getDataPoint(10);
    std::cout << "  点 10: X=" << point10.x << ", Y=" << point10.y 
              << ", Z=" << point10.z << ", Rx=" << point10.rx 
              << ", Ry=" << point10.ry << ", Rz=" << point10.rz << std::endl;
              
    // 测试边界情况
    std::cout << "\n边界情况测试:" << std::endl;
    RobotPoseData invalid_point = mgr.getDataPoint(10000);  // 应该返回标准位置
    std::cout << "  无效索引点: X=" << invalid_point.x << ", Y=" << invalid_point.y 
              << ", Z=" << invalid_point.z << ", Rx=" << invalid_point.rx 
              << ", Ry=" << invalid_point.ry << ", Rz=" << invalid_point.rz << std::endl;
    
    // 测试getPointCount
    std::cout << "\n数据点总数: " << mgr.getPointCount() << std::endl;
    
    std::cout << "\n✅ 机械臂坐标数据管理模块测试完成" << std::endl;
    
    return 0;
}