#include <iostream>
#include <vector>
#include "hand_eye_calib/calib_utils.hpp"
#include "hand_eye_calib/calib_config.hpp"
#include "calib_data_operate.hpp"
#include "internal_constants.h"
#include "log_system/log_macros.hpp"

using namespace handeyecalib;

int main() 
{
    LOG_INFO("=== 标定结果应用模块测试 ===");
    
    // 创建配置管理器并加载配置
    CalibConfig config;
    std::string config_file = "config/calib_config.yaml";
    if (!config.loadConfig(config_file)) {
        LOG_WARN("⚠️  加载配置文件失败，使用默认配置");
    }
    
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
    
    // 加载第一个标定点数据并计算位姿偏差
    std::vector<double> theoretical_pose, actual_pose, pose_deviation;
    if (loadFirstCalibrationPointAndEvaluate(config.getSourceDataDir() + "/coordinates", cam_to_base_transform, 
                                           theoretical_pose, actual_pose, pose_deviation)) 
	{
        // 输出理论位姿和实际位姿的比较结果
        LOG_INFO("\n📊 标定结果评估:");
        LOG_INFO("理论位姿: [X: %f, Y: %f, Z: %f, Rx: %f, Ry: %f, Rz: %f]", 
            theoretical_pose[0], theoretical_pose[1], theoretical_pose[2], theoretical_pose[3], theoretical_pose[4], theoretical_pose[5]);
        LOG_INFO("实际位姿: [X: %f, Y: %f, Z: %f, Rx: %f, Ry: %f, Rz: %f]", 
            actual_pose[0], actual_pose[1], actual_pose[2], actual_pose[3], actual_pose[4], actual_pose[5]);
        LOG_INFO("位姿偏差: [X: %f, Y: %f, Z: %f, Rx: %f, Ry: %f, Rz: %f]", 
            pose_deviation[0], pose_deviation[1], pose_deviation[2], pose_deviation[3], pose_deviation[4], pose_deviation[5]);
    } else {
        LOG_ERROR("❌ 评估标定结果失败");
        return 1;
    }
    
    // 示例：计算机械手实际位姿（假设标记在相机坐标系下的位置为[0.1, 0.2, 0.3]）
    std::vector<double> marker_pose = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0};  // 添加姿态信息
    std::vector<double> offset_compensation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 初始化偏移补偿值
    TransformResult result = computeRobotPoseFromMarker(marker_pose, cam_to_base_transform, &offset_compensation);    
    LOG_INFO("\n🔧 示例计算结果:");
    LOG_INFO("根据标记位置计算得到的机械手位姿:");
    LOG_INFO("X: %f Y: %f Z: %f", result.transformed_pose[0], result.transformed_pose[1], result.transformed_pose[2]);
    LOG_INFO("Rx: %f Ry: %f Rz: %f", result.transformed_pose[3], result.transformed_pose[4], result.transformed_pose[5]);           
    LOG_INFO("\n✅ 标定结果应用模块测试完成");
    return 0;
}