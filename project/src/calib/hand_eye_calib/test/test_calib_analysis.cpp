#include <iostream>
#include <vector>
#include "hand_eye_calib/calib_data_collector.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "hand_eye_calib/calib_config.hpp"
#include "calib_data_operate.hpp"
#include "internal_constants.h"
#include "log_system/log_macros.hpp"
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "bas_operate/file_operate.hpp" 
namespace fs = std::filesystem;

using namespace handeyecalib;

int main() {
    LOG_INFO("=== 标定质量分析模块测试 ===");
    
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
    
    // 加载标定结果
    ArmCalibInfo arm_calib_info;
    std::string bas_config_data_path = config.getOutputDataDir(); // 使用输出目录作为配置数据路径
    
    // 从配置或默认值获取相机ID和机械臂ID
    // 注意：在实际部署中，这些值应从系统配置中获取
    uint8_t cam_id = 0; // 默认相机ID
    uint8_t arm_id = 0; // 默认机械臂ID
    LOG_INFO("尝试加载相机cam_id= %d 机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_id));
    if (!loadArmCalibInfo(bas_config_data_path, cam_id, arm_id, arm_calib_info)) {
        LOG_ERROR("❌ 加载相机cam_id= %d 机械臂arm_id=%d 的标定结果失败", static_cast<int>(cam_id), static_cast<int>(arm_id));
        return 1;
    }
    LOG_INFO("✅ 成功加载相机cam_id= %d 机械臂arm_id=%d 的标定信息", static_cast<int>(cam_id), static_cast<int>(arm_id));
    cv::Mat cam_to_base_transform = arm_calib_info.calib_info.calib_res.cam_to_base_transform;
    cv::Mat base_to_cam_transform = arm_calib_info.calib_info.calib_res.base_to_cam_transform;
    
    // 分析标定质量
    QualityMetrics metrics = analyzeCalibrationQuality(
        robot_poses, marker_positions, cam_to_base_transform);
    
    LOG_INFO("标定质量分析结果:");
    // 假设焦距是650，在0.75米的高度下，1mm对应像素值是1.15像素（这个值需要根据实际相机参数调整）
    double mm_to_pixel = 1.15;
    LOG_INFO("重投影误差: %f mm (%f pixels)", metrics.reprojection_error, metrics.reprojection_error * mm_to_pixel);
    LOG_INFO("平移误差: %f mm (%f meters)", metrics.translation_error, metrics.translation_error / 1000);
    LOG_INFO("旋转误差: %f degrees", metrics.rotation_error);
    LOG_INFO("矩阵条件数: %f", metrics.condition_number);
    LOG_INFO("数据点数量: %d", static_cast<int>(metrics.data_point_count));
    
    return 0;
}