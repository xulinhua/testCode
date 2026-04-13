#ifndef HAND_EYE_CALIB__CALIB_DATA_OPERATE_HPP_
#define HAND_EYE_CALIB__CALIB_DATA_OPERATE_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <string>
#include <nlohmann/json.hpp>
#include "hand_eye_calib/calib_struct.hpp"

// 命名空间定义
namespace handeyecalib 
{

// 从配置文件加载TCP偏移数据
bool loadTcpOffsetFromYaml(const std::string& config_file_path, std::vector<double>& offset_compensation);

// 保存TCP偏移数据到配置文件
bool saveTcpOffsetToYaml(const std::string& config_file_path, const std::vector<double>& offset_compensation);

// 加载指定相机和机械臂的偏移补偿参数
bool loadTcpOffset(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, CalibRes& calib_res);

// 保存指定相机和机械臂的偏移补偿参数
bool saveTcpOffset(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, const CalibRes& calib_res);

bool loadQualityMetrics(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, QualityMetrics& quality_metrics);

bool saveQualityMetrics(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, const QualityMetrics& quality_metrics);

// 加载指定相机和机械臂的标定结果
bool loadCalibRes(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, CalibRes& calib_res);

// 保存指定相机和机械臂的标定结果
bool saveCalibRes(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, const CalibRes& calib_res);

/**
 * @brief 从JSON文件加载标定信息
 * @param file_path 加载文件路径
 * @param calib_info 标定信息（输出）
 * @return 是否加载成功
 * 
 * 该函数从JSON文件中加载标定信息，包括标定结果、质量评估指标等。
 * 注意：加载的CalibInfo信息和CalibRes实际上是存储在同一个文件calib_result_cam0_arm0.json中的
 */
bool loadCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, CalibInfo& calib_info);

/**
 * @brief 保存标定信息到JSON文件
 * @param file_path 保存文件路径
 * @param calib_info 标定信息
 * @return 是否保存成功
 * 
 * 该函数将标定信息保存到JSON文件中，包括标定结果、质量评估指标等。
 * 注意：保存的CalibInfo信息和CalibRes实际上是存储在同一个文件calib_result_cam0_arm0.json中的
 */
bool saveCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, const CalibInfo& calib_info);

bool loadArmCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, ArmCalibInfo& arm_calib_info);

bool loadArmCalibInfoList(const std::string& bas_config_data_path, const uint8_t cam_id, ArmCalibInfoList& arm_calib_list);

bool saveArmCalibInfoList(const std::string& bas_config_data_path, const uint8_t cam_id, const ArmCalibInfoList& arm_calib_list);

// 保存指定相机和机械臂的手眼标定数据
bool saveArmCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, const ArmCalibInfo& arm_calib_info);

// 加载指定相机目录下的所有机械臂标定数据及偏移补偿参数
bool loadCamCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, 
    const std::vector<uint8_t>& arm_id_list, const uint8_t sys_cam_num, CamCalibInfo& cam_calib_info);
    
bool saveCamCalibInfo(const std::string& bas_config_data_path, const CamCalibInfo& cam_calib_info);

} // namespace handeyecalib

#endif // HAND_EYE_CALIB__CALIB_DATA_OPERATE_HPP_