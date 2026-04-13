#include <iostream>
#include <vector>
#include <filesystem>
#include "hand_eye_calib/calib_data_collector.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "hand_eye_calib/calib_config.hpp"
#include "calib_data_operate.hpp"
#include "internal_constants.h"
#include "log_system/log_macros.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "bas_operate/bas_utils.hpp"
#include "bas_operate/file_operate.hpp" 
namespace fs = std::filesystem;

using namespace handeyecalib;

int main() {
    LOG_INFO("=== 标定矩阵生成模块测试 ===");
    
    // 创建配置管理器并加载配置
    CalibConfig config;
    std::string config_file = "config/calib_config.yaml";

    // 检查当前目录下是否存在配置文件
    if (!fs::exists(config_file)) {
        // 尝试在上一级目录查找
        config_file = "../config/calib_config.yaml";
        if (!fs::exists(config_file)) {
            // 尝试在上两级目录查找
            config_file = "../../config/calib_config.yaml";
            if (!fs::exists(config_file)) {
                // 尝试通过 ROS 包路径查找
                try {
                    std::string package_path = ament_index_cpp::get_package_share_directory("hand_eye_calib");
                    config_file = package_path + "/config/calib_config.yaml";
                    if (!fs::exists(config_file)) {
                        config_file = "";
                    }
                } catch (const std::exception& e) {
                    config_file = "";
                }
            }
        }
    }
    
    if (!config.loadConfig(config_file)) {
        LOG_WARN("⚠️  加载配置文件失败，使用默认配置");
    }
    
    // 构造标定点数据目录路径
    std::string install_path_str = basmodule::get_install_dir();
    fs::path install_path(install_path_str);
    fs::path config_path = install_path.parent_path() / "sys_config" / "source_calib_data" / "camera_intrinsics.json";
    std::string coordinates_dir = config_path.string();

    // 创建数据管理器并加载数据
    CalibDataCollector collector;
    // 从源数据目录加载标定数据
    if (!collector.loadCalibrationData(config.getSourceDataDir() + "/coordinates")) {
        LOG_ERROR("❌ 加载标定数据失败");
        return 1;
    }
    
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
    CalibRes result = computeEyeToHandCalibration(robot_poses, marker_positions);
    
    // 计算标定质量评估指标
    LOG_INFO("正在分析标定质量...");
    QualityMetrics quality_metrics = analyzeCalibrationQuality(robot_poses, marker_positions, result.cam_to_base_transform);
    
    // 创建标定信息对象
    std::string timestamp = basmodule::get_timestamp(); // 获取当前时间戳
    std::string calib_method = "Park-Martin"; // 标定方法
    CalibInfo calib_info(result, quality_metrics, timestamp, calib_method);
    
    // 设置机械臂标定信息
    ArmCalibInfo arm_calib_info;
    arm_calib_info.setCalibInfo(calib_info);
    arm_calib_info.arm_id = 0; // 设置默认机械臂ID
    
    // 保存标定结果到输出目录
    std::string bas_config_data_path = config.getOutputDataDir(); // 使用输出目录作为配置数据路径
    uint8_t cam_id = 0; // 默认相机ID
    LOG_INFO("尝试保存相机cam_id= %d 机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_calib_info.arm_id));
    if (saveArmCalibInfo(bas_config_data_path, cam_id, arm_calib_info)) {
        LOG_INFO("✅ 成功保存相机cam_id= %d 机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_calib_info.arm_id));
    } else {
        LOG_ERROR("❌ 保存相机cam_id= %d 机械臂arm_id=%d 的标定结果失败", static_cast<int>(cam_id), static_cast<int>(arm_calib_info.arm_id));
        return 1;
    }
    LOG_INFO("\n✅ 标定矩阵生成模块测试完成");
    return 0;
}