#include <iostream>
#include <vector>
#include <filesystem>
#include "hand_eye_calib/calib_data_collector.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "hand_eye_calib/calib_robot_pos_mgr.hpp"
#include "hand_eye_calib/calib_config.hpp"
#include "internal_constants.h"
#include "log_system/log_macros.hpp"

// 使用命名空间
using namespace handeyecalib;

/**
 * @brief 打印使用说明
 */
void printUsage() {
    LOG_INFO("手眼标定工具使用说明:");
    LOG_INFO("1. 数据采集模式: 请运行 'ros2 run hand_eye_calib test_data_collector --collect'");
    LOG_INFO("2. 标定矩阵生成模式: 请运行 'ros2 run hand_eye_calib test_calib_matrix'");
    LOG_INFO("3. 标定结果应用模式: 请运行 'ros2 run hand_eye_calib test_calib_application'");
    LOG_INFO("4. 标定质量分析模式: 请运行 'ros2 run hand_eye_calib test_calib_analysis'");
    LOG_INFO("5. 机械臂坐标数据管理模式: 请运行 'ros2 run hand_eye_calib test_calib_robot_pos_mgr'");
    LOG_INFO("6. 显示帮助: hand_eye_calib --help");
}

/**
 * @brief 数据采集模式
 */
void runDataCollectionMode(const CalibConfig& config) {
    LOG_INFO("=== 数据采集模式 ===");
    
    LOG_INFO("请运行以下命令进行数据采集测试:");
    LOG_INFO("ros2 run hand_eye_calib test_data_collector --collect");
}

/**
 * @brief 标定矩阵生成模式
 */
void runCalibrationMode(const CalibConfig& config) {
    LOG_INFO("=== 标定矩阵生成模式 ===");
    
    LOG_INFO("请运行以下命令进行标定矩阵生成测试:");
    LOG_INFO("ros2 run hand_eye_calib test_calib_matrix");
}

/**
 * @brief 标定结果应用模式
 */
void runApplicationMode(const CalibConfig& config) {
    LOG_INFO("=== 标定结果应用模式 ===");
    
    LOG_INFO("请运行以下命令进行标定结果应用测试:");
    LOG_INFO("ros2 run hand_eye_calib test_calib_application");
}

/**
 * @brief 标定质量分析模式
 */
void runAnalysisMode(const CalibConfig& config) {
    LOG_INFO("=== 标定质量分析模式 ===");
    
    LOG_INFO("请运行以下命令进行标定质量分析测试:");
    LOG_INFO("ros2 run hand_eye_calib test_calib_analysis");
}

/**
 * @brief 机械臂坐标数据管理模式
 */
void runRobotPoseManagementMode() {
    LOG_INFO("=== 机械臂坐标数据管理模式 ===");
    
    LOG_INFO("请运行以下命令进行机械臂坐标数据管理测试:");
    LOG_INFO("ros2 run hand_eye_calib test_calib_robot_pos_mgr");
}

/**
 * @brief 主函数
 */
int main(int argc, char* argv[]) {

    LOG_INFO("开始 hand_eye_calib 程序");
    
    // 创建配置管理器并加载配置
    CalibConfig config;
    std::string config_file = "config/calib_config.yaml";
    if (!config.loadConfig(config_file)) {
        LOG_WARN("⚠️  加载配置文件失败，使用默认配置");
    }
    
    // 检查参数
    if (argc < 2) {
        printUsage();
        return 0;
    }
    
    std::string mode = argv[1];
    
    if (mode == "--collect") {
        runDataCollectionMode(config);
    } else if (mode == "--calibrate") {
        runCalibrationMode(config);
    } else if (mode == "--apply") {
        runApplicationMode(config);
    } else if (mode == "--analyze") {
        runAnalysisMode(config);
    } else if (mode == "--generate-poses") {
        runRobotPoseManagementMode();
    } else if (mode == "--help") {
        printUsage();
    } else {
        LOG_ERROR("❌ 未知模式: %s", mode.c_str());
        printUsage();
        return 1;
    }
    
    LOG_INFO("hand_eye_calib 程序执行完毕");
    
    return 0;
}
