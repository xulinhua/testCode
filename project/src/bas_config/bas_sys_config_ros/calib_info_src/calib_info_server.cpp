/**
 * @file param_to_server.cpp
 * @brief ROS通信工具函数实现(将配置写入参数服务器)
 * 
 * 实现系统配置信息与参数服务器格式之间的转换函数，以及通信信息解析功能
 */
#include "bas_operate_ros/param_to_server.hpp"
#include "bas_operate_ros/param_from_server.hpp"
#include "bas_sys_config/config_reflector.hpp"
#include "bas_sys_config_ros/node_names.h"
#include "hand_eye_calib/calib_reflector.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "hand_eye_calib/calib_struct.hpp"
#include "log_system/log_macros.hpp"
#include "bas_operate/bas_utils.hpp"
#include "bas_operate/file_operate.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include "bas_operate_ros/param_utils.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cstdio>
#include <sstream>

namespace RosComm
{

/**
 * @brief 获取ArmCalibInfo参数前缀
 * @param cam_id 机械臂标定信息结构体引用
 * @param arm_id 参数前缀字符串，不能为空
 * @return std::string ArmCalibInfo参数前缀
 */
std::string getArmCalibInfoParamPrefix(uint8_t cam_id, uint8_t arm_id)
{
    std::string prefix_node = std::string(SYS_CAM_CALIB_LIST) + std::string(NODE_NAME_CAM_PREFIX) + std::to_string(cam_id);
    std::string prefix_arm_info = basros::updateNodeName(prefix_node, NODE_NAME_ARM_INFO);
    std::string prefix = prefix_arm_info + NODE_NAME_ARM_PREFIX + std::to_string(arm_id);// 构建参数前缀
    return prefix;
}

/**
 * @brief 将CalibRes转换为参数服务器可存储的格式
 * 该函数将CalibRes结构体中的所有参数转换为参数向量，便于存储到ROS 2参数服务器中。
 * @param para_info 标定结果结构体引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool calibResToRos(const handeyecalib::CalibRes& para_info, const std::string& param_prefix,
    std::vector<rclcpp::Parameter>& ros_paras)
{
    ros_paras.clear();
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    LOG_INFO("===== 开始转换标定结果到参数服务器格式===== ");
    LOG_DEBUG("参数前缀: %s", param_prefix.c_str());
    handeyecalib::CalibResReflector reflector(const_cast<handeyecalib::CalibRes&>(para_info));// 使用反射器自动处理参数
    const auto& para_infos = reflector.getParams();
    LOG_DEBUG("反射器获取到 %d 个参数", static_cast<int>(para_infos.size()));
    bool bRet = basros::paraInfoToRos(para_infos, param_prefix, ros_paras);
    if (!bRet) 
    {
        LOG_ERROR("转换参数失败");
        return false;
    }
    logsys::Level log_level = logsys::Level::DEBUG;
    logsys::Color color = logsys::Color::BLUE;
    if (LOG_ON(log_level))
    {
        LOG_OUT(log_level, "完成标定结果到参数服务器格式的转换, 共生成 %d 个参数：", static_cast<int>(ros_paras.size()));
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
        bRet = basros::printLog_paraInfo_rosPara(para_infos, ros_paras, project_name, 
            (int)log_level, "", (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    LOG_INFO("===== 结束转换标定结果到参数服务器格式 ===== ");
    return true;
}

/**
 * @brief 将QualityMetrics转换为参数服务器可存储的格式
 * 该函数将QualityMetrics结构体中的所有参数转换为参数向量，便于存储到ROS 2参数服务器中。
 * @param para_info 质量评估指标结构体引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool qualityMetricsToRos(const handeyecalib::QualityMetrics& para_info, const std::string& param_prefix,
    std::vector<rclcpp::Parameter>& ros_paras)
{
    ros_paras.clear();
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    LOG_INFO("===== 开始转换标定质量评估指标到参数服务器格式===== ");
    LOG_DEBUG("参数前缀: %s", param_prefix.c_str());
    handeyecalib::QualityMetricsReflector reflector(const_cast<handeyecalib::QualityMetrics&>(para_info));// 使用反射器自动处理参数
    const auto& para_infos = reflector.getParams();
    LOG_DEBUG("反射器获取到 %d 个参数", static_cast<int>(para_infos.size()));
    bool bRet = basros::paraInfoToRos(para_infos, param_prefix, ros_paras);
    if (bRet)
    {
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "完成标定质量评估指标到参数服务器格式的转换, 共生成 %d 个参数：", static_cast<int>(ros_paras.size()));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
            bRet = basros::printLog_paraInfo_rosPara(para_infos, ros_paras, project_name, 
                (int)log_level, "", (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
    }
    else 
    {
        LOG_ERROR("转换参数失败");
        return false;
    }
    LOG_INFO("===== 结束转换标定质量评估指标到参数服务器格式 ===== ");
    return true;
}

/**
 * @brief 将ArmCalibInfo转换为参数服务器可存储的格式
 * 该函数使用反射机制自动转换ArmCalibInfo的所有参数，避免手动逐个转换。
 * @param para_info 机械臂标定信息结构体引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool armCalibInfoToRos(const handeyecalib::ArmCalibInfo& para_info, 
    const std::string& param_prefix, std::vector<rclcpp::Parameter>& ros_paras)
{
    ros_paras.clear();
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    LOG_INFO("===== 开始转换机械臂arm_id= %d 的标定信息到参数服务器格式===== ", static_cast<int>(para_info.arm_id));
    LOG_DEBUG("参数前缀: %s", param_prefix.c_str());
    handeyecalib::ArmCalibInfoReflector reflector(const_cast<handeyecalib::ArmCalibInfo&>(para_info));// 使用反射器自动处理参数
    const auto& para_infos = reflector.getParams();
    LOG_DEBUG("ArmCalibInfoReflector反射器获取到 %d 个参数：", static_cast<int>(para_infos.size()));
    bool bRet = basros::paraInfoToRos(para_infos, param_prefix, ros_paras);
    if (!bRet)
    {
        LOG_ERROR("转换机械臂arm_id= %d 的标定信息到参数服务器格式时出错", para_info.arm_id);
        return false;
    }
    logsys::Level log_level = logsys::Level::INFO;
    logsys::Color color = logsys::Color::BLUE;
    if (LOG_ON(log_level))
    {
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
        bRet = basros::printLog_paraInfo_rosPara(para_infos, ros_paras, project_name, 
            (int)log_level, "", (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    handeyecalib::CalibResReflector calib_res_reflector(const_cast<handeyecalib::CalibRes&>(para_info.calib_info.calib_res));
    LOG_DEBUG("当前标定结果中的电机角度信息有 %d 个参数", static_cast<int>(para_info.calib_info.calib_res.head_motor_angles.size()));
    const auto& calib_res_infos = calib_res_reflector.getParams();
    LOG_DEBUG("CalibResReflector反射器获取到 %d 个参数", static_cast<int>(calib_res_infos.size()));
    std::vector<rclcpp::Parameter> calib_ros_paras;
    bRet = basros::paraInfoToRos(calib_res_infos, param_prefix, calib_ros_paras);
    if (!bRet) 
    {
        LOG_ERROR("转换机械臂arm_id= %d 的标定结果信息到参数服务器格式时出错", para_info.arm_id);
        return false;
    }
    if (LOG_ON(log_level))
    {
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
        bRet = basros::printLog_paraInfo_rosPara(calib_res_infos, calib_ros_paras, project_name, 
            (int)log_level, "", (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    for (const auto& param : calib_ros_paras) // 将calib_ros_paras中的参数添加到ros_paras中
    {
        ros_paras.push_back(param);
    }
    std::vector<datahandler::ParamInfo> all_para_infos = para_infos;
    for (const auto& param : calib_res_infos) // 将calib_ros_paras中的参数添加到ros_paras中
    {
        all_para_infos.push_back(param);
    }
    if (LOG_ON(log_level))
    {
        LOG_OUT(log_level, "完成机械臂标定信息转换, 共生成 %d 个参数：", static_cast<int>(ros_paras.size()));
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
        bRet = basros::printLog_paraInfo_rosPara(all_para_infos, ros_paras, project_name, 
            (int)log_level, "", (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    LOG_INFO("===== 结束转换机械臂arm_id= %d 的标定信息到参数服务器格式 ===== ", static_cast<int>(para_info.arm_id));
    std::cout << std::endl;// 输出空行，确保下次终端输出时能空一行再显示
    return true;
}

/**
 * @brief 将ArmCalibInfoList转换为参数服务器可存储的格式
 * 该函数用于将机械臂标定信息列表转换为参数向量，便于批量存储到ROS 2参数服务器中。
 * @param para_infos 机械臂标定信息列表引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool armCalibInfoListToRos(const handeyecalib::ArmCalibInfoList& para_infos, 
    const std::string& param_prefix, std::vector<rclcpp::Parameter>& ros_paras)
{
    ros_paras.clear();
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    LOG_INFO("===== 开始转换相机的机械臂标定信息列表到参数服务器格式===== ");
    std::vector<uint8_t> arm_ids = handeyecalib::getArmIds(para_infos);// 获取机械臂ID列表
    std::string arm_ids_str = basmodule::get_list_string(arm_ids);
    LOG_DEBUG("机械臂个数：%d %s, 参数前缀: %s", static_cast<int>(arm_ids.size()), arm_ids_str.c_str(), param_prefix.c_str());
    for (const auto& arm_pair : para_infos)// 遍历机械臂标定信息列表，为每个机械臂生成参数 
    {
        const auto& arm_calib_info = arm_pair.second;
        std::string prefix_node = param_prefix + NODE_NAME_ARM_PREFIX + std::to_string(arm_calib_info.arm_id);        
        std::vector<rclcpp::Parameter> arm_calib_params;
        bool result = armCalibInfoToRos(arm_calib_info, prefix_node, arm_calib_params);
        if (!result) 
        {
            LOG_WARN("转换机械臂arm_id= %d 的标定信息失败", static_cast<int>(arm_calib_info.arm_id));
            continue; // 继续处理下一个机械臂
        }
        LOG_DEBUG("机械臂arm_id= %d 的标定信息生成 %d 个参数", static_cast<int>(arm_calib_info.arm_id), static_cast<int>(arm_calib_params.size()));
        for (const auto& param : arm_calib_params) // 将参数向量中的所有参数添加到输出向量中
        {
            ros_paras.push_back(param);
        }
    }
    logsys::Level log_level = logsys::Level::INFO;
    if (LOG_ON(log_level))
    {
        logsys::Color color = logsys::Color::BLUE;
        LOG_OUT(log_level, "完成相机的机械臂标定信息列表 %s 的配置转换, 共生成 %d 个参数：", arm_ids_str.c_str(), static_cast<int>(ros_paras.size()));
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
        basros::printLog_rosParam(ros_paras, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    LOG_INFO("===== 完成转换相机的机械臂标定信息列表到参数服务器格式===== \n");
    return true;
}

/**
 * @brief 将CamCalibInfo转换为参数服务器可存储的格式
 * 该函数使用反射机制自动转换CamCalibInfo的所有参数，避免手动逐个转换。
 * @param para_info 相机标定信息结构体引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool camCalibInfoToRos(const handeyecalib::CamCalibInfo& para_info, 
    const std::string& param_prefix, std::vector<rclcpp::Parameter>& ros_paras)
{
    ros_paras.clear();
    if (param_prefix.empty()) // 检查参数前缀是否为空 
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    LOG_INFO("===== 开始转换相机 %d 的所有标定信息到参数服务器格式===== ", static_cast<int>(para_info.cam_id));
    LOG_DEBUG("参数前缀: %s", param_prefix.c_str());
    
    // 使用反射器自动处理参数
    handeyecalib::CamCalibInfoReflector reflector(const_cast<handeyecalib::CamCalibInfo&>(para_info));
    const auto& para_infos = reflector.getParams();
    LOG_DEBUG("反射器获取到 %d 个参数", static_cast<int>(para_infos.size()));
    bool bRet = basros::paraInfoToRos(para_infos, param_prefix, ros_paras);
    if (!bRet)
    {
        LOG_ERROR("转换相机cam_id= %d 的标定信息到参数服务器格式时出错", para_info.cam_id);
        return false;
    }
    logsys::Level log_level = logsys::Level::INFO;
    logsys::Color color = logsys::Color::BLUE;
    if (LOG_ON(log_level))
    {
        LOG_OUT(log_level, "完成相机标定基础信息转换, 共生成 %d 个参数：", static_cast<int>(ros_paras.size()));
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
        bRet = basros::printLog_paraInfo_rosPara(para_infos, ros_paras, project_name, 
            (int)log_level, "", (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    const std::string prefix_arm_info = basros::updateNodeName(param_prefix, NODE_NAME_ARM_INFO);
    LOG_DEBUG("开始处理相机关联的机械臂标定信息列表");
    std::vector<rclcpp::Parameter> arm_params;
    bool arm_result = armCalibInfoListToRos(para_info.arm_calib1D, prefix_arm_info, arm_params);
    if (!arm_result)
    {
        LOG_WARN("处理相机关联的机械臂标定信息列表失败");
    } 
    else 
    {
        LOG_DEBUG("机械臂标定信息列表转换完成，共生成 %d 个参数", static_cast<int>(arm_params.size()));
        // 将机械臂参数添加到输出向量中
        for (const auto& param : arm_params) 
        {
            ros_paras.push_back(param);
            LOG_DEBUG("添加机械臂参数: %s = %s", param.get_name().c_str(), param.value_to_string().c_str());
        }
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            std::vector<uint8_t> arm_ids = handeyecalib::getArmIds(para_info.arm_calib1D);// 获取机械臂ID列表
            std::string arm_ids_str = basmodule::get_list_string(arm_ids);
            LOG_OUT(log_level, "完成转换相机cam_id= %d （含机械臂%s）的标定信息到参数服务器格式, 共生成 %d 个参数：", 
                static_cast<int>(para_info.cam_id), arm_ids_str.c_str(), static_cast<int>(ros_paras.size()));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
            bRet = basros::printLog_rosParam(ros_paras, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
    }
    LOG_INFO("===== 开始转换相机 %d 的所有标定信息到参数服务器格式===== ", static_cast<int>(para_info.cam_id));
    return true;
}

/**
 * @brief 将CamCalibInfoList转换为参数服务器可存储的格式
 * 该函数用于将相机标定信息列表转换为参数向量，便于批量存储到ROS 2参数服务器中。
 * @param para_infos 相机标定信息列表引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool camCalibInfoListToRos(const handeyecalib::CamCalibInfoList& para_infos, 
    const std::string& param_prefix, std::vector<rclcpp::Parameter>& ros_paras)
{
    ros_paras.clear();
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    LOG_INFO("===== 开始转换相机标定信息列表到参数服务器格式===== ");
    std::vector<uint8_t> cam_ids = handeyecalib::getCamIds(para_infos);// 获取相机ID列表
    std::string cam_ids_str = basmodule::get_list_string(cam_ids);
    LOG_DEBUG("标定相机配置个数: %d %s, 参数前缀: %s", static_cast<int>(cam_ids.size()), cam_ids_str.c_str(), param_prefix.c_str());
    basros::paramToRos(ros_paras, NODE_NAME_CAM_IDS, cam_ids, param_prefix);// 将启用的相机ID列表写入参数服务器
    
    // 生成全局版本号（用于整个列表）
    uint64_t global_version = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    // 添加全局版本号
    std::string global_version_param_name = param_prefix + ".version";
    ros_paras.emplace_back(global_version_param_name, static_cast<int64_t>(global_version));
    LOG_DEBUG("添加全局版本号参数: %s = %lu", global_version_param_name.c_str(), global_version);
    
    for (size_t i = 0; i < para_infos.size(); ++i) // 遍历相机标定信息列表，为每个相机生成参数
    {
        const auto& cam_calib_info = para_infos[i];
        LOG_DEBUG("处理第 %d 个相机[cam_id = %d]的标定信息", static_cast<int>(i + 1), static_cast<int>(cam_calib_info.cam_id));
        std::string prefix_node = param_prefix + NODE_NAME_CAM_PREFIX + std::to_string(cam_calib_info.cam_id);        
        std::vector<rclcpp::Parameter> cam_calib_params;
        bool result = camCalibInfoToRos(cam_calib_info, prefix_node, cam_calib_params);
        if (!result) 
        {
            LOG_WARN("转换第 %d 个相机[cam_id = %d]的标定信息失败", static_cast<int>(i + 1), static_cast<int>(cam_calib_info.cam_id));
            continue; // 继续处理下一个相机
        }
        
        // 为每个相机添加版本号
        uint64_t cam_version = global_version + i; // 为每个相机生成唯一版本号
        std::string cam_version_param_name = prefix_node + ".version";
        cam_calib_params.emplace_back(cam_version_param_name, static_cast<int64_t>(cam_version));
        LOG_DEBUG("为相机%d添加版本号参数: %s = %lu", cam_calib_info.cam_id, cam_version_param_name.c_str(), cam_version);
        
        LOG_DEBUG("处理第 %d 个相机[cam_id = %d]的标定信息生成了 %d 个参数", static_cast<int>(i + 1), static_cast<int>(cam_calib_info.cam_id), static_cast<int>(cam_calib_params.size()));
        for (const auto& param : cam_calib_params) // 将参数向量中的所有参数添加到输出向量中
        {
            ros_paras.push_back(param);
            LOG_DEBUG("参数: %s = %s", param.get_name().c_str(), param.value_to_string().c_str());
        }
    }
    logsys::Level log_level = logsys::Level::INFO;
    if (LOG_ON(log_level))
    {
        logsys::Color color = logsys::Color::BLUE;
        LOG_OUT(log_level, "完成相机列表 %s 的标定信息列表到参数服务器格式, 共生成 %d 个参数：", cam_ids_str.c_str(), static_cast<int>(ros_paras.size()));
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
        basros::printLog_rosParam(ros_paras, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    LOG_INFO("===== 完成转换相机标定信息列表到参数服务器格式===== \n");
    return true;
}

/**
 * @brief 将CamCalibInfoList转换为参数服务器可存储的格式，并直接声明和设置参数
 * 该函数用于将相机标定信息列表转换为参数并向参数服务器声明和设置这些参数。
 * @param node ROS节点指针，用于访问参数服务器
 * @param para_infos 相机标定信息列表引用
 * @param param_prefix 参数前缀字符串
 * @return 操作是否成功
 */
bool setCamCalibInfoListToServer(rclcpp::Node::SharedPtr node, 
    const handeyecalib::CamCalibInfoList& para_infos, const std::string& param_prefix)
{
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    LOG_INFO("===== 开始声明系统相机标定信息列表所有参数并设置参数到参数服务器===== ");
    std::vector<uint8_t> cam_ids = handeyecalib::getCamIds(para_infos);// 获取相机ID列表
    std::string cam_ids_str = basmodule::get_list_string(cam_ids);
    LOG_DEBUG("标定相机配置个数: %d %s, 参数前缀: %s", static_cast<int>(cam_ids.size()), cam_ids_str.c_str(), param_prefix.c_str());
    try 
    {
        std::vector<rclcpp::Parameter> ros_paras;// 转换标定信息为参数格式
        bool result = camCalibInfoListToRos(para_infos, param_prefix, ros_paras);
        if (!result) 
        {
            LOG_ERROR("相机标定信息列表参数转换失败");
            return false;
        }
        LOG_DEBUG("系统相机标定信息列表参数转换完成，共生成 %d 个参数，开始声明和设置参数", static_cast<int>(ros_paras.size()));
        bool bRet = basros::paraInfoToServer(node.get(), ros_paras);
        LOG_INFO("===== 完成声明系统相机标定信息列表所有参数并设置参数到参数服务器===== \n");
        return bRet;
    } catch (const std::exception& e) {
        LOG_ERROR("设置相机标定信息列表到参数服务器时发生异常: %s", e.what());
        return false;
    }
}

/**
 * @brief 从参数服务器读取CalibRes数据
 * 该函数使用反射机制自动读取CalibRes的所有参数，避免手动逐个读取。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀字符串
 * @param[out] calib_res 读取到的标定结果信息结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool getCalibResFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    const std::string& param_prefix, handeyecalib::CalibRes& calib_res)
{
    calib_res.Init(); // 初始化标定结果对象并设置默认值
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    try 
    {
        LOG_INFO("===== 开始从参数服务器读取标定结果信息===== ");
        LOG_DEBUG("参数前缀: %s", param_prefix.c_str());
        handeyecalib::CalibResReflector reflector(calib_res);// 使用反射器自动处理参数
        auto para_infos = reflector.getParams();
        LOG_DEBUG("反射器获取到 %d 个参数", static_cast<int>(para_infos.size()));
        bool bRet = basros::paraInfoFromServer(client, para_infos, param_prefix);// 从参数服务器获取参数信息的函数实现
        if (!bRet)
        {
            LOG_ERROR("从参数服务器读取标定结果信息时出错");
            return false;
        }
        bRet = reflector.tranInfoFromParamInfos(para_infos);
        if (!bRet)
        {
            LOG_ERROR("从反射器读取相机参数信息转换失败");
            return false;
        }
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "完成标定结果信息从服务器的读取, 共读取到 %d 个参数：", static_cast<int>(para_infos.size()));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
            basros::printLog_paraInfo(para_infos, project_name, (int)log_level, param_prefix, 
                (int)color, __FILE__, __FUNCTION__, __LINE__);
            LOG_OUT(log_level, "完成标定结果信息从服务器的读取转换, 参数信息如下：");
            printLog_CalibRes(calib_res, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        LOG_INFO("===== 结束从参数服务器读取标定结果信息 ===== \n");
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("从参数服务器读取标定结果参数时出错: %s", e.what());
        return false;  // 读取失败
    }
}

/**
 * @brief 从参数服务器读取单个ArmCalibInfo
 * 该函数使用反射机制自动读取ArmCalibInfo的所有参数，避免手动逐个读取。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀字符串
 * @param arm_id 机械臂ID
 * @param arm_calib_info[out] 读取到的机械臂标定信息结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool getArmCalibInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    const std::string& param_prefix, uint8_t arm_id, handeyecalib::ArmCalibInfo& arm_calib_info)
{
    arm_calib_info.Init(); // 初始化机械臂标定信息对象并设置默认值
    arm_calib_info.arm_id = arm_id;
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    try 
    {
        LOG_INFO("===== 开始从参数服务器读取机械臂arm_id= %d 的标定信息 ===== ", static_cast<int>(arm_id));
        const std::string prefix = param_prefix + NODE_NAME_ARM_PREFIX + std::to_string(arm_id);// 构建参数前缀
        LOG_DEBUG("参数前缀: %s", prefix.c_str());
        handeyecalib::ArmCalibInfoReflector reflector(arm_calib_info);// 使用反射器自动处理参数
        auto para_infos = reflector.getParams();
        LOG_DEBUG("反射器获取到 %d 个参数", static_cast<int>(para_infos.size()));
        bool bRet = basros::paraInfoFromServer(client, para_infos, prefix);// 从参数服务器获取参数信息的函数实现
        if (!bRet)
        {
            LOG_ERROR("从参数服务器读取机械臂标定信息时出错");
            LOG_INFO("===== 结束从参数服务器读取机械臂arm_id= %d 的标定信息 ===== \n", static_cast<int>(arm_calib_info.arm_id));
            return false;
        }
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "完成机械臂标定信息从服务器的读取转换, 共生成 %d 个参数：", static_cast<int>(para_infos.size()));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
            basros::printLog_paraInfo(para_infos, project_name, (int)log_level, prefix, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        if (arm_calib_info.arm_id != arm_id) // 验证读取的arm_id是否与传入的arm_id一致
        {
            LOG_ERROR("读取到的机械臂arm_id(%d)与传入的机械臂arm_id(%d)不一致", static_cast<int>(arm_calib_info.arm_id), static_cast<int>(arm_id));
            LOG_INFO("===== 结束从参数服务器读取机械臂arm_id= %d 的标定信息 ===== \n", static_cast<int>(arm_calib_info.arm_id));
            return false;
        }

        bRet = getCalibResFromServer(client, prefix, arm_calib_info.calib_info.calib_res);
        if (!bRet)
        {
            LOG_ERROR("从参数服务器读取机械臂标定信息时出错");
            LOG_INFO("===== 结束从参数服务器读取机械臂arm_id= %d 的标定信息 ===== \n", static_cast<int>(arm_calib_info.arm_id));
            return false;
        }

        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "当前机械臂arm_id= %d 的标定信息读取成功", static_cast<int>(arm_calib_info.arm_id));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
            printLog_ArmCalibInfo(arm_calib_info, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);// 调用打印日志函数
        }
        LOG_INFO("===== 结束从参数服务器读取机械臂arm_id= %d 的标定信息 ===== \n", static_cast<int>(arm_calib_info.arm_id));
        std::cout << std::endl;// 输出空行，确保下次终端输出时能空一行再显示
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("从参数服务器读取机械臂参数时出错: %s", e.what());
        return false;  // 读取失败
    }
}

/**
 * @brief 从参数服务器读取单个ArmCalibInfo
 * 该函数用于从参数服务器中读取单个机械臂标定信息。
 * @param client 参数服务器客户端节点
 * @param arm_id 机械臂ID
 * @param arm_calib_info[out] 读取到的机械臂标定信息结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool getArmCalibInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    uint8_t arm_id, handeyecalib::ArmCalibInfo& arm_calib_info)
{
    // 缺少相机信息，不能这样写。。。。。
    const std::string prefix_arm_info = basros::updateNodeName(std::string(SYS_ENABLE_ARM_LIST), NODE_NAME_ARM_INFO);
    return getArmCalibInfoFromServer(client, prefix_arm_info, arm_id, arm_calib_info);
}

bool getArmCalibInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    uint8_t cam_id, uint8_t arm_id, handeyecalib::ArmCalibInfo& arm_calib_info)
{
    std::string prefix_node = std::string(SYS_CAM_CALIB_LIST) + std::string(NODE_NAME_CAM_PREFIX) + std::to_string(cam_id);
    std::string prefix_arm_info = basros::updateNodeName(prefix_node, NODE_NAME_ARM_INFO);
    return getArmCalibInfoFromServer(client, prefix_arm_info, arm_id, arm_calib_info);
}

/**
 * @brief 设置单个ArmCalibInfo参数服务器
 * 该函数用于从参数服务器中读取单个机械臂标定信息。
 * @param client 参数服务器客户端节点
 * @param arm_id 机械臂ID
 * @param arm_calib_info[out] 读取到的机械臂标定信息结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool SetArmCalibInfoToServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    uint8_t cam_id, uint8_t arm_id, handeyecalib::ArmCalibInfo& arm_calib_info)
{
    try 
    {
        std::string prefix_node = std::string(SYS_CAM_CALIB_LIST) + std::string(NODE_NAME_CAM_PREFIX) + std::to_string(cam_id);
        std::string prefix_arm_info = basros::updateNodeName(prefix_node, NODE_NAME_ARM_INFO);
        prefix_arm_info = prefix_arm_info + NODE_NAME_ARM_PREFIX + std::to_string(arm_id);        
        std::vector<rclcpp::Parameter> arm_calib_params;
        
        if (!armCalibInfoToRos(arm_calib_info, prefix_arm_info, arm_calib_params)) {
            LOG_ERROR("转换机械臂标定信息到ROS参数时出错");
            return false;
        }
        
        // 生成新的版本号（时间戳）
        uint64_t new_version = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        // 添加版本号参数
        std::string version_param_name = prefix_node + ".version";
        arm_calib_params.emplace_back(version_param_name, static_cast<int64_t>(new_version));
        
        // 使用原子写入，确保所有参数一次性更新
        auto result_obj = client->set_parameters_atomically(arm_calib_params);
        if (!result_obj.successful) {
            LOG_ERROR("原子设置标定参数失败: %s", result_obj.reason.c_str());
            return false;
        }
        
        LOG_INFO("标定数据已更新: cam=%d, arm=%d, version=%lu", cam_id, arm_id, new_version);
        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("设置标定信息异常: %s", e.what());
        return false;
    }
}

/**
 * @brief 从参数服务器读取QualityMetrics数据
 * 该函数使用反射机制自动读取QualityMetrics的所有参数，避免手动逐个读取。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀字符串
 * @param[out] quality_metrics 读取到的质量评估指标结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool getQualityMetricsFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    const std::string& param_prefix, handeyecalib::QualityMetrics& quality_metrics)
{
    quality_metrics.Init(); // 初始化质量评估指标对象并设置默认值
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    try 
    {
        LOG_INFO("===== 开始从参数服务器读取质量评估指标信息 ===== ");
        LOG_DEBUG("参数前缀: %s", param_prefix.c_str());
        handeyecalib::QualityMetricsReflector reflector(quality_metrics);// 使用反射器自动处理参数
        auto para_infos = reflector.getParams();
        LOG_DEBUG("反射器获取到 %d 个参数", static_cast<int>(para_infos.size()));
        bool bRet = basros::paraInfoFromServer(client, para_infos, param_prefix);// 从参数服务器获取参数信息的函数实现
        if (!bRet)
        {
            LOG_ERROR("从参数服务器读取质量评估指标信息时出错");
            LOG_INFO("===== 结束从参数服务器读取质量评估指标信息 ===== \n");
            return false;
        }
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "完成质量评估指标信息从服务器的读取, 共读取到 %d 个参数：", static_cast<int>(para_infos.size()));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
            basros::printLog_paraInfo(para_infos, project_name, (int)log_level, param_prefix, 
                (int)color, __FILE__, __FUNCTION__, __LINE__);
            LOG_OUT(log_level, "完成质量评估指标信息从服务器的读取转换, 参数信息如下：");
            printLog_QualityMetrics(quality_metrics, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        LOG_INFO("===== 结束从参数服务器读取质量评估指标信息 ===== \n");
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("从参数服务器读取质量评估指标参数时出错: %s", e.what());
        return false;  // 读取失败
    }
}

/**
 * @brief 从参数服务器读取ArmCalibInfoList数据
 * 该函数用于从参数服务器中读取机械臂标定信息列表，并返回读取状态。
 * 函数首先读取arm_ids参数来确定系统中配置的机械臂ID列表，
 * 然后根据机械臂ID列表逐个读取对应的机械臂标定信息。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀
 * @param arm_calib_list[out] 读取到的机械臂标定信息列表
 * @return 读取状态，true表示成功，false表示失败
 */
bool getArmCalibInfoListFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    const std::string& param_prefix, handeyecalib::ArmCalibInfoList& arm_calib_list)
{
    try 
    {
        arm_calib_list.clear(); // 清空输出列表
        if (param_prefix.empty()) // 检查参数前缀是否为空
        {
            LOG_ERROR("参数前缀不能为空");
            return false;
        }
        LOG_INFO("===== 开始从参数服务器读取机械臂标定信息列表 ===== ");
        const std::string arm_ids_param = basros::updateNodeName(param_prefix, NODE_NAME_ARM_IDS);// 构造arm_ids参数名
        std::vector<uint8_t> arm_ids;// 读取arm_ids参数来确定需要读取哪些机械臂的标定信息
        if (!basros::paramFromServer(client, arm_ids_param, arm_ids, ""))
        {
            LOG_ERROR("参数 %s 不存在，无法读取机械臂标定信息", arm_ids_param.c_str());
            LOG_INFO("===== 结束从参数服务器读取机械臂标定信息列表 ===== ");
            return false;
        }
        std::string arm_ids_str = basmodule::get_list_string(arm_ids);
        LOG_DEBUG("系统机械臂数量: %d %s, 参数前缀: %s", static_cast<int>(arm_ids.size()), arm_ids_str.c_str(), param_prefix.c_str());
        const std::string prefix_arm_info = basros::updateNodeName(param_prefix, NODE_NAME_ARM_INFO);// 构造arm_info参数前缀
        for (size_t i = 0; i < arm_ids.size(); ++i) // 根据机械臂ID列表逐个读取机械臂标定信息
        {
            const auto& arm_id = arm_ids[i];
            LOG_DEBUG("读取第 %d 个机械臂arm_id= %d 的标定信息...", static_cast<int>(i + 1), static_cast<int>(arm_id));
            handeyecalib::ArmCalibInfo arm_calib_info; // 创建机械臂标定信息对象并初始化
            if (getArmCalibInfoFromServer(client, prefix_arm_info, arm_id, arm_calib_info)) 
            {
                arm_calib_list.insert({arm_calib_info.arm_id, arm_calib_info});
                LOG_DEBUG("第 %d 个机械臂arm_id= %d 的标定信息读取成功", static_cast<int>(i + 1), static_cast<int>(arm_id));
            } else {
                LOG_WARN("读取机械臂arm_id= %d的参数失败", arm_id);
                LOG_INFO("===== 结束从参数服务器读取机械臂标定信息列表 ===== \n");
                return false;
            }
        }
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "完成机械臂标定信息列表从参数服务器的读取, 共读取到 %d 个机械臂标定", static_cast<int>(arm_calib_list.size()));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__); 
            printLog_ArmCalibInfoList(arm_calib_list, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        LOG_INFO("===== 结束从参数服务器读取机械臂标定信息列表 ===== \n");
        return true; // 成功读取
    } catch (const std::exception& e) {
        LOG_ERROR("从参数服务器读取机械臂标定信息列表时出错: %s", e.what());
        LOG_INFO("===== 结束从参数服务器读取机械臂标定信息列表 ===== \n");
        return false; // 读取失败
    }
}

/**
 * @brief 从参数服务器读取单个CamCalibInfo
 * 该函数使用反射机制自动读取CamCalibInfo的所有参数，避免手动逐个读取。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀字符串
 * @param cam_id 相机ID
 * @param cam_calib_info[out] 读取到的相机标定信息结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool getCamCalibInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    const std::string& param_prefix, uint8_t cam_id, handeyecalib::CamCalibInfo& cam_calib_info)
{
    cam_calib_info.Init(); // 初始化相机标定信息对象并设置默认值
    cam_calib_info.cam_id = cam_id;
    LOG_INFO("===== 开始从参数服务器读取相机cam_id= %d 的标定信息 ===== ", static_cast<int>(cam_id));
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        LOG_INFO("===== 结束从参数服务器读取相机cam_id= %d 的标定信息 ===== \n", static_cast<int>(cam_calib_info.cam_id));
        return false;
    }
    try 
    {
        const std::string prefix = param_prefix + NODE_NAME_CAM_PREFIX + std::to_string(cam_id);// 构建参数前缀
        LOG_DEBUG("参数前缀: %s", prefix.c_str());
        handeyecalib::CamCalibInfoReflector reflector(cam_calib_info);// 使用反射器自动处理参数
        auto para_infos = reflector.getParams();
        LOG_DEBUG("反射器获取到 %d 个参数", static_cast<int>(para_infos.size()));
        bool bRet = basros::paraInfoFromServer(client, para_infos, prefix);// 从参数服务器获取参数信息的函数实现
        if (!bRet)
        {
            LOG_ERROR("从参数服务器读取相机标定信息时出错");
            LOG_INFO("===== 结束从参数服务器读取相机cam_id= %d 的标定信息 ===== \n", static_cast<int>(cam_calib_info.cam_id));
            return false;
        }
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "完成相机标定信息从服务器的读取, 共读取到 %d 个参数：", static_cast<int>(para_infos.size()));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
            basros::printLog_paraInfo(para_infos, project_name, (int)log_level, prefix, 
                (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        if (cam_calib_info.cam_id != cam_id) // 验证读取的cam_id是否与传入的cam_id一致
        {
            LOG_ERROR("读取到的相机cam_id(%d)与传入的相机cam_id(%d)不一致", static_cast<int>(cam_calib_info.cam_id), static_cast<int>(cam_id));
            LOG_INFO("===== 结束从参数服务器读取相机cam_id= %d 的标定信息 ===== \n", static_cast<int>(cam_calib_info.cam_id));
            return false;
        }
        handeyecalib::ArmCalibInfoList arm_calib_list;
        if (getArmCalibInfoListFromServer(client, prefix, arm_calib_list))
        {
            cam_calib_info.arm_calib1D = arm_calib_list;
            LOG_DEBUG("成功从参数服务器读取相机cam_id= %d 的机械臂配置，数量：%d", static_cast<int>(cam_calib_info.cam_id), cam_calib_info.arm_calib1D.size());
            for (const auto& arm_info : arm_calib_list)
            {
                LOG_DEBUG("获取配置的机械臂ID: %d, 机械臂ID: %d", arm_info.first, arm_info.second.arm_id);
            }
        }
        else
        {
            LOG_ERROR("从参数服务器读取相机cam_id= %d 的机械臂配置失败", static_cast<int>(cam_calib_info.cam_id));
            LOG_INFO("===== 结束从参数服务器读取相机cam_id= %d 的标定信息 ===== \n", static_cast<int>(cam_calib_info.cam_id));
            return false;
        }
        LOG_DEBUG("当前相机cam_id= %d 的标定信息读取成功", static_cast<int>(cam_calib_info.cam_id));
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "完成相机标定信息从服务器的转换, 参数信息如下：");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_CamCalibInfo(cam_calib_info, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        LOG_INFO("===== 结束从参数服务器读取相机cam_id= %d 的标定信息 ===== \n", static_cast<int>(cam_calib_info.cam_id));
        std::cout << std::endl;// 输出空行，确保下次终端输出时能空一行再显示
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("从参数服务器读取相机参数时出错: %s", e.what());
        LOG_INFO("===== 结束从参数服务器读取相机cam_id= %d 的标定信息 ===== \n", static_cast<int>(cam_calib_info.cam_id));
        return false;  // 读取失败
    }
}

/**
 * @brief 从参数服务器读取单个CamCalibInfo
 * 该函数用于从参数服务器中读取单个相机标定信息。
 * 该函数会从sys_cam_calib_list.cam_{cam_id}路径读取相机标定信息
 * @param client 参数服务器客户端节点
 * @param cam_id 相机ID
 * @param cam_calib_info[out] 读取到的相机标定信息结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool getCamCalibInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    uint8_t cam_id, handeyecalib::CamCalibInfo& cam_calib_info)
{
    //const std::string prefix_cam_info = basros::updateNodeName(std::string(SYS_CAM_CALIB_LIST), NODE_NAME_CAM_INFO);
    return getCamCalibInfoFromServer(client, SYS_CAM_CALIB_LIST, cam_id, cam_calib_info);
}

/**
 * @brief 从参数服务器读取CamCalibInfoList数据
 * 该函数用于从参数服务器中读取相机标定信息列表，并返回读取状态。
 * 函数首先读取cam_ids参数来确定系统中配置的相机ID列表，
 * 然后根据相机ID列表逐个读取对应的相机标定信息。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀
 * @param cam_calib_list[out] 读取到的相机标定信息列表
 * @return 读取状态，true表示成功，false表示失败
 */
bool getCamCalibInfoListFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    const std::string& param_prefix, handeyecalib::CamCalibInfoList& cam_calib_list)
{
    try 
    {
        cam_calib_list.clear(); // 清空输出列表
        LOG_INFO("===== 开始从参数服务器读取相机标定信息列表 ===== ");
        if (param_prefix.empty()) // 检查参数前缀是否为空
        {
            LOG_ERROR("参数前缀不能为空");
            LOG_INFO("===== 结束从参数服务器读取相机标定信息列表 ===== \n");
            return false;
        }
        const std::string cam_ids_param = basros::updateNodeName(param_prefix, NODE_NAME_CAM_IDS);// 构造cam_ids参数名
        std::vector<uint8_t> cam_ids;// 读取cam_ids参数来确定需要读取哪些相机的标定信息
        if (!basros::paramFromServer(client, cam_ids_param, cam_ids, ""))
        {
            LOG_ERROR("参数 %s 不存在，无法读取相机标定信息", cam_ids_param.c_str());
            LOG_INFO("===== 结束从参数服务器读取相机标定信息列表 ===== \n");
            return false;
        }
        std::string cam_ids_str = basmodule::get_list_string(cam_ids);
        LOG_DEBUG("系统相机个数: %d %s, 参数前缀: %s", static_cast<int>(cam_ids.size()), cam_ids_str.c_str(), param_prefix.c_str());
        const std::string prefix_cam_info = basros::updateNodeName(param_prefix, NODE_NAME_CAM_INFO);// 构造cam_info参数前缀
        for (size_t i = 0; i < cam_ids.size(); ++i) // 根据相机ID列表逐个读取相机标定信息
        {
            const auto& cam_id = cam_ids[i];
            LOG_DEBUG("读取第 %d 个相机cam_id= %d 的标定信息...", static_cast<int>(i + 1), static_cast<int>(cam_id));
            handeyecalib::CamCalibInfo cam_calib_info; // 创建相机标定信息对象并初始化
            if (getCamCalibInfoFromServer(client, prefix_cam_info, cam_id, cam_calib_info)) 
            {
                cam_calib_list.push_back(cam_calib_info);
                LOG_DEBUG("第 %d 个相机cam_id= %d 的标定信息读取成功", static_cast<int>(i + 1), static_cast<int>(cam_id));
            } else {
                LOG_WARN("读取相机cam_id= %d的参数失败", cam_id);
                LOG_INFO("===== 结束从参数服务器读取相机标定信息列表 ===== \n");
                return false;
            }
        }
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "完成相机标定信息列表从参数服务器的读取, 共读取到 %d 个相机标定", static_cast<int>(cam_calib_list.size()));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_CamCalibInfoList(cam_calib_list, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        LOG_INFO("===== 结束从参数服务器读取相机标定信息列表 ===== \n");
        return true; // 成功读取
    } catch (const std::exception& e) {
        LOG_ERROR("从参数服务器读取相机标定信息列表时出错: %s", e.what());
        LOG_INFO("===== 结束从参数服务器读取相机标定信息列表 ===== \n");
        return false; // 读取失败
    }
}

}
// namespace RosComm
