#include <iostream>
#include <vector>
#include <chrono>
#include "hand_eye_calib/calib_data_collector.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "hand_eye_calib/calib_config.hpp"
#include "calib_data_operate.hpp"
#include "internal_constants.h"
#include "log_system/log_macros.hpp"

using namespace handeyecalib;

int main() 
{
    LOG_INFO("🚀 开始手眼标定核心算法测试...");
    
    // 创建配置管理器并加载配置
    CalibConfig config;
    std::string config_file = "config/calib_config.yaml";
    if (!config.loadConfig(config_file)) {
        LOG_WARN("⚠️  加载配置文件失败，使用默认配置");
    }
    
    // 创建数据管理器并添加测试数据
    CalibDataCollector collector;
    
    // 添加测试标定点数据（模拟实际采集的数据）
    std::vector<std::pair<std::vector<double>, std::vector<double>>> test_data = {
        {{0.1, 0.2, 0.3, 0.0, 0.0, 0.0}, {0.05, 0.1, 0.15}},
        {{0.2, 0.3, 0.4, 0.0, 0.0, 0.0}, {0.1, 0.15, 0.2}},
        {{0.3, 0.4, 0.5, 0.0, 0.0, 0.0}, {0.15, 0.2, 0.25}},
        {{0.4, 0.5, 0.6, 0.0, 0.0, 0.0}, {0.2, 0.25, 0.3}},
        {{0.5, 0.6, 0.7, 0.0, 0.0, 0.0}, {0.25, 0.3, 0.35}}
    };
    
    LOG_INFO("添加测试标定点数据...");
    for (std::size_t i = 0; i < test_data.size(); ++i) {
        const auto& data = test_data[i];
        if (collector.addCalibrationPoint(data.first, data.second)) {
            LOG_INFO("✅ 成功添加第 %d 个标定点", static_cast<int>(i + 1));
        } else {
            LOG_ERROR("❌ 添加第 %d 个标定点失败", static_cast<int>(i + 1));
        }
    }
    
    // 检查是否有足够的标定点
    if (!collector.hasEnoughPoints(config.getMinCalibrationPoints())) {
        LOG_ERROR("❌ 标定点数据不足，至少需要%d个点", config.getMinCalibrationPoints());
        return 1;
    }
    
    // 获取标定计算所需的数据格式
    auto calibration_data = collector.getCalibrationData();
    const auto& robot_poses = calibration_data.first;
    const auto& marker_positions = calibration_data.second;
    
    // 计算眼在手外标定矩阵
    LOG_INFO("正在计算眼在手外标定矩阵...");
    handeyecalib::CalibRes result = computeEyeToHandCalibration(robot_poses, marker_positions);
    
    // 保存标定结果到输出目录
    std::string bas_config_data_path = config.getOutputDataDir(); // 使用输出目录作为配置数据路径
    
    // 将CalibRes转换为CalibInfo进行保存
    handeyecalib::CalibInfo calib_info;
    calib_info.calib_res = result;
    calib_info.calib_method = "Park-Martin";
    calib_info.timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0);
    
    // 设置机械臂标定信息
    ArmCalibInfo arm_calib_info;
    arm_calib_info.setCalibInfo(calib_info);
    arm_calib_info.arm_id = 0; // 设置默认机械臂ID
    uint8_t cam_id = 0; // 默认相机ID
    LOG_INFO("尝试保存相机cam_id= %d 机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_calib_info.arm_id));
    if (saveArmCalibInfo(bas_config_data_path, cam_id, arm_calib_info)) {
        LOG_INFO("✅ 成功保存相机cam_id= %d 机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_calib_info.arm_id));
    } else {
        LOG_ERROR("❌ 保存相机cam_id= %d 机械臂arm_id=%d 的标定结果失败", static_cast<int>(cam_id), static_cast<int>(arm_calib_info.arm_id));
        return 1;
    }
    LOG_INFO("\n✅ 手眼标定核心算法测试完成");
    return 0;
}