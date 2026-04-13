/**
 * @file param_to_server.cpp
 * @brief ROS通信工具函数实现(将配置写入参数服务器)
 * 
 * 实现系统配置信息与参数服务器格式之间的转换函数，以及通信信息解析功能
 */
#include "bas_operate_ros/param_to_server.hpp"
#include "bas_sys_config/config_reflector.hpp"
#include "hand_eye_calib/calib_reflector.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "bas_operate/file_operate.hpp"
#include "log_system/log_macros.hpp"
#include "bas_operate/bas_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include "bas_operate_ros/param_utils.hpp"
#include "bas_operate_ros/param_from_server.hpp"
#include "bas_sys_config_ros/node_names.h"
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
 * @brief 将ArmConfigInfo转换为参数服务器可存储的格式
 * 该函数使用反射机制自动转换ArmConfigInfo的所有参数，避免手动逐个转换。
 * @param para_info 机械臂配置信息结构体引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool armInfoToRos(const SysConfig::ArmConfigInfo& para_info, const std::string& param_prefix, 
    std::vector<rclcpp::Parameter>& ros_paras)
{
    ros_paras.clear();
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    LOG_DEBUG("===== 开始转换机械臂arm_id= %d 的配置信息到参数服务器格式===== ", static_cast<int>(para_info.arm_id));
    LOG_DEBUG("参数前缀: %s", param_prefix.c_str());
    SysConfig::ArmConfigInfoReflector reflector(const_cast<SysConfig::ArmConfigInfo&>(para_info));// 使用反射器自动处理参数
    const auto& para_infos = reflector.getParams();
    LOG_DEBUG("参数个数: %d", static_cast<int>(para_infos.size()));
    bool bRet = basros::paraInfoToRos(para_infos, param_prefix, ros_paras);
    if (!bRet) 
    {
        LOG_ERROR("转换参数失败");
        return false;
    }
    logsys::Level log_level = logsys::Level::INFO;
    if (LOG_ON(log_level))
    {
        logsys::Color color = logsys::Color::BLUE;
        LOG_OUT(log_level, "完成当前机械臂arm_id= %d 的配置信息转换到服务器, 共生成 %d 个参数：", static_cast<int>(para_info.arm_id), static_cast<int>(ros_paras.size()));
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
        bRet = basros::printLog_paraInfo_rosPara(para_infos, ros_paras, project_name, (int)log_level, "", (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    LOG_INFO("===== 结束转换机械臂arm_id= %d 的配置信息到参数服务器格式 ===== ", static_cast<int>(para_info.arm_id));
    return true;
}

/**
 * @brief 将ArmConfigInfoList转换为参数服务器可存储的格式
 * 该函数用于将机械臂配置信息列表转换为参数向量，便于后续存储到ROS 2参数服务器中。
 * @param para_infos 机械臂配置信息列表引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool armInfoListToRos(const SysConfig::ArmConfigInfoList& para_infos, const std::string& param_prefix, 
    std::vector<rclcpp::Parameter>& ros_paras)
{
    ros_paras.clear();
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    LOG_DEBUG("===== 开始转换机械臂配置信息列表到参数服务器格式===== ");
    std::vector<uint8_t> arm_ids = SysConfig::getArmIds(para_infos);// 获取机械臂ID列表
    std::string arm_ids_str = basmodule::get_list_string(arm_ids);
    LOG_DEBUG("系统机械臂个数: %d %s, 参数前缀: %s", static_cast<int>(arm_ids.size()), arm_ids_str.c_str(), param_prefix.c_str());
    basros::paramToRos(ros_paras, NODE_NAME_ARM_IDS, arm_ids, param_prefix);// 将启用的机械臂ID列表写入参数服务器
    const std::string prefix_arm_info = basros::updateNodeName(param_prefix, NODE_NAME_ARM_INFO);
    for (size_t i = 0; i < para_infos.size(); ++i) // 遍历机械臂配置列表，为每个机械臂生成参数
    {
        const auto& arm_info = para_infos[i];
        const std::string prefix_arm = basros::updateNodeName(prefix_arm_info, NODE_NAME_ARM_PREFIX + std::to_string(arm_info.arm_id));           
        std::vector<rclcpp::Parameter> arm_params;
        if (!armInfoToRos(arm_info, prefix_arm, arm_params)) 
        {
            LOG_ERROR("转换单个机械臂配置信息失败: arm_id= %d", static_cast<int>(arm_info.arm_id));
            return false;
        }
        for (const auto& it : arm_params) // 将arm_params中的所有参数添加到输出参数向量中
        {
            ros_paras.push_back(it);
        }
    }
    logsys::Level log_level = logsys::Level::INFO;
    if (LOG_ON(log_level))
    {
        logsys::Color color = logsys::Color::BLUE;
        LOG_OUT(log_level, "完成机械臂列表 %s 的配置转换, 共生成 %d 个参数：", arm_ids_str.c_str(), static_cast<int>(ros_paras.size()));
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
        basros::printLog_rosParam(ros_paras, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    LOG_DEBUG("===== 完成转换机械臂配置信息列表到参数服务器格式===== \n");
    return true;
}

/**
 * @brief 将ArmConfigInfoList转换为参数服务器可存储的格式，并直接声明和设置参数
 * 该函数用于将机械臂配置信息列表转换为参数并向参数服务器声明和设置这些参数。
 * @param node ROS节点指针，用于访问参数服务器
 * @param param_prefix 参数前缀字符串
 * @param para_infos 机械臂配置信息列表引用
 * @return 操作是否成功
 */
bool setArmInfoListToServer(rclcpp::Node::SharedPtr node, const std::string& param_prefix, 
    const SysConfig::ArmConfigInfoList& para_infos)
{
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    try 
    {
        std::vector<rclcpp::Parameter> ros_paras;
        if (!armInfoListToRos(para_infos, param_prefix, ros_paras)) 
        {
            LOG_ERROR("转换机械臂配置信息列表到参数服务器格式失败");
            return false;
        }
        LOG_INFO("===== 开始声明系统所有机械臂配置信息参数并设置到参数服务器===== ");
        bool bRet = basros::paraInfoToServer(node.get(), ros_paras);
        LOG_INFO("===== 完成声明系统所有机械臂配置信息参数并设置到参数服务器===== ");
        return bRet;
    } catch (const std::exception& e) {
        LOG_ERROR("设置机械臂配置信息列表到参数服务器时发生异常: %s", e.what());
        return false;
    }
}

/**
 * @brief 从参数服务器读取单个ArmConfigInfo
 * 该函数使用反射机制自动读取ArmConfigInfo的所有参数，避免手动逐个读取。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀字符串
 * @param arm_id 机械臂ID
 * @param arm_info[out] 读取到的机械臂配置信息结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool getArmInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    const std::string& param_prefix, uint8_t arm_id, SysConfig::ArmConfigInfo& arm_info)
{
    arm_info.Init(); // 初始化机械臂配置信息对象并设置默认值
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    try 
    {
        LOG_DEBUG("开始从参数服务器读取机械臂arm_id= %d 的配置信息", static_cast<int>(arm_id));
        const std::string prefix = param_prefix + NODE_NAME_ARM_PREFIX + std::to_string(arm_id);// 构建参数前缀
        LOG_DEBUG("参数前缀: %s", prefix.c_str());
        SysConfig::ArmConfigInfoReflector reflector(arm_info);// 使用反射器自动处理参数
        auto para_infos = reflector.getParams();
        LOG_DEBUG("反射器获取到 %d 个参数", static_cast<int>(para_infos.size()));
        bool bRet = basros::paraInfoFromServer(client, para_infos, prefix);// 从参数服务器获取参数信息的函数实现
        if (!bRet)
        {
            LOG_ERROR("从参数服务器读取机械臂配置信息时出错");
            return false;
        }
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "完成机械臂配置信息从服务器的读取, 共读取到 %d 个参数：", static_cast<int>(para_infos.size()));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
            basros::printLog_paraInfo(para_infos, project_name, (int)log_level, prefix, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }     
        if (arm_info.arm_id != arm_id) // 验证读取的arm_id是否与传入的arm_id一致
        {
            LOG_ERROR("读取到的机械臂arm_id(%d)与传入的机械臂arm_id(%d)不一致", static_cast<int>(arm_info.arm_id), static_cast<int>(arm_id));
            return false;
        }
        LOG_DEBUG("当前机械臂arm_id= %d 的配置信息读取成功, 是否使能：%s", static_cast<int>(arm_info.arm_id), arm_info.is_enable ? "是" : "否");
        LOG_DEBUG("===== 结束从参数服务器读取机械臂arm_id= %d 的配置信息 ===== ", static_cast<int>(arm_info.arm_id));
        std::cout << std::endl;// 输出空行，确保下次终端输出时能空一行再显示
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("从参数服务器读取机械臂参数时出错: %s", e.what());
        return false;  // 读取失败
    }
}

/**
 * @brief 从参数服务器读取ArmConfigInfoList数据
 * 该函数用于从参数服务器中读取机械臂配置信息列表，并返回读取状态。
 * 函数首先读取arm_ids参数来确定系统中配置的机械臂ID列表，
 * 然后根据机械臂ID列表逐个读取对应的机械臂配置信息。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀
 * @param arm_info_list[out] 读取到的机械臂配置信息列表
 * @return 读取状态，true表示成功，false表示失败
 */
bool getArmInfoListFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    const std::string& param_prefix, SysConfig::ArmConfigInfoList& arm_info_list)
{
    try 
    {
        arm_info_list.clear(); // 清空输出列表
        if (param_prefix.empty()) // 检查参数前缀是否为空
        {
            LOG_ERROR("参数前缀不能为空");
            return false;
        }
        LOG_DEBUG("===== 开始从参数服务器读取机械臂配置信息列表 ===== ");
        const std::string arm_ids_param = basros::updateNodeName(param_prefix, NODE_NAME_ARM_IDS);// 构造arm_ids参数名
        std::vector<uint8_t> arm_ids;// 读取arm_ids参数来确定需要读取哪些机械臂的配置信息
        if (!basros::paramFromServer(client, arm_ids_param, arm_ids, ""))
        {
            LOG_ERROR("参数 %s 不存在，无法读取机械臂配置信息", arm_ids_param.c_str());
            LOG_INFO("===== 结束从参数服务器读取机械臂配置信息列表 ===== ");
            return false;
        }
        std::string arm_ids_str = basmodule::get_list_string(arm_ids);
        LOG_DEBUG("系统机械臂数量: %d %s, 参数前缀: %s", static_cast<int>(arm_ids.size()), arm_ids_str.c_str(), param_prefix.c_str());
        const std::string prefix_arm_info = basros::updateNodeName(param_prefix, NODE_NAME_ARM_INFO);// 构造arm_info参数前缀
        for (size_t i = 0; i < arm_ids.size(); ++i) // 根据机械臂ID列表逐个读取机械臂配置信息
        {
            const auto& arm_id = arm_ids[i];
            LOG_DEBUG("读取第 %d 个机械臂arm_id= %d 的配置信息...", static_cast<int>(i + 1), static_cast<int>(arm_id));
            SysConfig::ArmConfigInfo arm_info; // 创建机械臂配置信息对象并初始化
            if (getArmInfoFromServer(client, prefix_arm_info, arm_id, arm_info)) 
            {
                arm_info_list.push_back(arm_info);
                LOG_DEBUG("第 %d 个机械臂arm_id= %d 的配置信息读取成功", static_cast<int>(i + 1), static_cast<int>(arm_id));
            } else {
                LOG_WARN("读取机械臂arm_id= %d的参数失败", arm_id);
                return false;
            }
        }
        LOG_DEBUG("完成机械臂配置信息列表读取, 共读取 %d 个机械臂配置", static_cast<int>(arm_info_list.size()));
        LOG_DEBUG("===== 结束从参数服务器读取机械臂配置信息列表 ===== ");
        return true; // 成功读取
    } catch (const std::exception& e) {
        LOG_ERROR("从参数服务器读取机械臂配置信息列表时出错: %s", e.what());
        LOG_INFO("===== 结束从参数服务器读取机械臂配置信息列表 ===== ");
        return false; // 读取失败
    }
}

/**
 * @brief 将CamConfigInfo转换为参数服务器可存储的格式
 * 该函数使用反射机制自动转换CamConfigInfo的所有参数，避免手动逐个转换。
 * @param para_info 相机配置信息结构体引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool camInfoToRos(const SysConfig::CamConfigInfo& para_info, const std::string& param_prefix, 
    std::vector<rclcpp::Parameter>& ros_paras)
{
    ros_paras.clear();
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    LOG_DEBUG("===== 开始转换相机cam_id= %d 的配置信息到参数服务器格式===== ", static_cast<int>(para_info.cam_id));
    LOG_DEBUG("参数前缀: %s", param_prefix.c_str());
    SysConfig::CamConfigInfoReflector reflector(const_cast<SysConfig::CamConfigInfo&>(para_info));// 使用反射器自动处理参数
    const auto& para_infos = reflector.getParams();
    LOG_DEBUG("参数个数: %d", static_cast<int>(para_infos.size()));
    bool bRet = basros::paraInfoToRos(para_infos, param_prefix, ros_paras);
    if (!bRet) {
        LOG_ERROR("转换参数失败");
        return false;
    }
    logsys::Level log_level = logsys::Level::INFO;
    logsys::Color color = logsys::Color::BLUE;
    if (LOG_ON(log_level))
    {
        LOG_OUT(log_level, "完成当前相机cam_id= %d 的配置信息转换, 共生成 %d 个参数", static_cast<int>(para_info.cam_id), static_cast<int>(ros_paras.size()));
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
        bRet = basros::printLog_paraInfo_rosPara(para_infos, ros_paras, project_name, (int)log_level, "", (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    LOG_DEBUG("开始处理相机关联的机械臂配置信息列表，共 %d 个机械臂", static_cast<int>(para_info.armInfoList.size()));
    std::vector<rclcpp::Parameter> arm_ros_paras;
    if (!armInfoListToRos(para_info.armInfoList, param_prefix, arm_ros_paras)) {
        LOG_ERROR("转换相机关联的机械臂配置信息列表失败");
        return false;
    }
    LOG_OUT(log_level, "机械臂配置信息列表转换完成，共生成 %d 个机械臂配置参数", static_cast<int>(arm_ros_paras.size()));
    for (const auto& param : arm_ros_paras) // 将arm_ros_paras中的参数添加到ros_paras中
    {
        ros_paras.push_back(param);
    }
    if (LOG_ON(log_level))
    {
        std::vector<uint8_t> arm_ids = SysConfig::getArmIds(para_info.armInfoList);// 获取机械臂ID列表
        std::string arm_ids_str = basmodule::get_list_string(arm_ids);
        LOG_OUT(log_level, "完成转换相机cam_id= %d （含机械臂%s）的配置信息到参数服务器格式, 共生成 %d 个参数：", 
            static_cast<int>(para_info.cam_id), arm_ids_str.c_str(), static_cast<int>(ros_paras.size()));
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
        bRet = basros::printLog_rosParam(ros_paras, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    LOG_INFO("===== 完成转换相机cam_id= %d 的配置信息到参数服务器格式===== \n", static_cast<int>(para_info.cam_id));
    return true;
}

/**
 * @brief 将CamConfigInfo1D转换为参数服务器可存储的格式
 * 该函数用于将相机配置信息列表转换为参数向量，便于后续存储到ROS 2参数服务器中。
 * @param para_infos 相机配置信息列表引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool camInfoListToRos(const SysConfig::CamConfigInfo1D& para_infos, const std::string& param_prefix, 
    std::vector<rclcpp::Parameter>& ros_paras)
{
    ros_paras.clear();
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    LOG_INFO("===== 开始转换相机配置信息列表到参数服务器格式===== ");
    std::vector<uint8_t> cam_ids = SysConfig::getCamIds(para_infos);// 获取相机ID列表
    std::string cam_ids_str = basmodule::get_list_string(cam_ids);
    LOG_INFO("系统相机配置个数: %d %s, 参数前缀: %s", static_cast<int>(cam_ids.size()), cam_ids_str.c_str(), param_prefix.c_str());
    basros::paramToRos(ros_paras, NODE_NAME_CAM_IDS, cam_ids, param_prefix);// 将启用的相机ID列表写入参数服务器
    const std::string prefix_cam_info = basros::updateNodeName(param_prefix, NODE_NAME_CAM_INFO);
    for (size_t i = 0; i < para_infos.size(); ++i) // 遍历相机配置列表，为每个相机生成参数
    {
        const auto& cam_info = para_infos[i];
        const std::string prefix_cam = basros::updateNodeName(prefix_cam_info, NODE_NAME_CAM_PREFIX + std::to_string(cam_info.cam_id));      
        std::vector<rclcpp::Parameter> cam_paras;
        if (!camInfoToRos(cam_info, prefix_cam, cam_paras)) 
        {
            LOG_ERROR("转换单个相机配置信息失败: cam_id= %d", static_cast<int>(cam_info.cam_id));
            return false;
        }
        for (const auto& it : cam_paras) // 将cam_params中的所有参数添加到输出参数向量中
        {
            ros_paras.push_back(it);
        }
    }
    logsys::Level log_level = logsys::Level::INFO;
    if (LOG_ON(log_level))
    {
        logsys::Color color = logsys::Color::BLUE;
        LOG_OUT(log_level, "完成相机列表 %s 的配置信息到参数服务器格式, 共生成 %d 个参数：", cam_ids_str.c_str(), static_cast<int>(ros_paras.size()));
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
        basros::printLog_rosParam(ros_paras, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
    }
    LOG_INFO("===== 完成转换相机配置信息列表到参数服务器格式===== \n");
    return true;
}

/**
 * @brief 将CamConfigInfo1D转换为参数服务器可存储的格式，并直接声明和设置参数
 * 该函数用于将相机配置信息列表转换为参数并向参数服务器声明和设置这些参数。
 * 此外，该函数还会处理每个相机关联的机械臂配置信息，将具体的机械臂配置也转换为参数。
 * @param node ROS节点指针，用于访问参数服务器
 * @param param_prefix 参数前缀字符串
 * @param para_infos 相机配置信息列表引用
 * @return 操作是否成功
 */
bool setCamInfoListToServer(rclcpp::Node::SharedPtr node, const std::string& param_prefix, 
    const SysConfig::CamConfigInfo1D& para_infos)
{
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    try 
    {
        std::vector<rclcpp::Parameter> ros_paras;
        if (!camInfoListToRos(para_infos, param_prefix, ros_paras)) 
        {
            LOG_ERROR("转换相机配置信息列表失败");
            return false;
        }
        LOG_INFO("===== 开始声明系统相机配置信息列表所有参数并设置参数到参数服务器===== ");
        bool bRet = basros::paraInfoToServer(node.get(), ros_paras);
        LOG_INFO("===== 完成声明系统相机配置信息列表所有参数并设置参数到参数服务器===== ");
        std::vector<rclcpp::Parameter> cam_ros_paras;
        basros::RosCommInfo comm_info;
         std::string key;
        for(int i = 0; i < para_infos.size(); i++)
        {
            comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_CAM_INTRINSICS,i, 0);
            key = comm_info.name;
            cam_ros_paras.push_back(rclcpp::Parameter(key, ""));
            comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_COLOR_IMAGE,i, 0);
            key = comm_info.name;
            cam_ros_paras.push_back(rclcpp::Parameter(key, ""));
            comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_DEPTH_IMAGE,i, 0);
            key = comm_info.name;
            cam_ros_paras.push_back(rclcpp::Parameter(key, ""));
            comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_POINT_CLOUD,i, 0);
            key = comm_info.name;
            cam_ros_paras.push_back(rclcpp::Parameter(key, ""));
            comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_MODULE_INFO_CAM,i, 0);
            key = comm_info.name;
            cam_ros_paras.push_back(rclcpp::Parameter(key, ""));
        }
        basros::paraInfoToServer(node.get(), cam_ros_paras);
        LOG_INFO("===== 完成声明系统相机话题===== ");   
        return bRet;
    } catch (const std::exception& e) {
        LOG_ERROR("设置相机配置信息列表到参数服务器时发生异常: %s", e.what());
        return false;
    }
}

/**
 * @brief 从参数服务器读取单个CamConfigInfo
 * 该函数使用反射机制自动读取CamConfigInfo的所有参数，避免手动逐个读取。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀字符串
 * @param cam_id 相机ID
 * @param cam_info[out] 读取到的相机配置信息结构体
 * @return 读取状态，true表示成功，false表示失败
 */
bool getCamInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    const std::string& param_prefix, uint8_t cam_id, SysConfig::CamConfigInfo& cam_info)
{
    cam_info.Init(); // 初始化相机配置信息对象并设置默认值
    cam_info.cam_id = cam_id;
    if (param_prefix.empty()) // 检查参数前缀是否为空
    {
        LOG_ERROR("参数前缀不能为空");
        return false;
    }
    try 
    {
        LOG_DEBUG("开始从参数服务器读取相机cam_id= %d 的配置信息", static_cast<int>(cam_id));
        const std::string prefix = param_prefix + NODE_NAME_CAM_PREFIX + std::to_string(cam_id);// 构建参数前缀
        LOG_DEBUG("参数前缀: %s", prefix.c_str());
        SysConfig::CamConfigInfoReflector reflector(cam_info);// 使用反射器自动处理参数
        auto para_infos = reflector.getParams();
        LOG_DEBUG("反射器获取到 %d 个参数", static_cast<int>(para_infos.size()));
        bool bRet = basros::paraInfoFromServer(client, para_infos, prefix);// 从参数服务器获取参数信息的函数实现
        if (!bRet)
        {
            LOG_ERROR("从参数服务器读取相机配置信息时出错");
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
            LOG_OUT(log_level, "完成相机配置信息从服务器的读取, 共读取到 %d 个参数：", static_cast<int>(para_infos.size()));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
            basros::printLog_paraInfo(para_infos, project_name, (int)log_level, prefix, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        if (cam_info.cam_id != cam_id) // 验证读取的cam_id是否与传入的cam_id一致
        {
            LOG_ERROR("读取到的相机cam_id(%d)与传入的相机cam_id(%d)不一致", static_cast<int>(cam_info.cam_id), static_cast<int>(cam_id));
            return false;
        }
        LOG_DEBUG("当前相机cam_id= %d 的配置信息读取成功, 序列号：%s", static_cast<int>(cam_info.cam_id), cam_info.serial_number.c_str());
        LOG_DEBUG("===== 结束从参数服务器读取相机cam_id= %d 的配置信息 ===== ", static_cast<int>(cam_info.cam_id));
        std::cout << std::endl;// 输出空行，确保下次终端输出时能空一行再显示

        SysConfig::ArmConfigInfoList arm_info_list;
        if (getArmInfoListFromServer(client, prefix, arm_info_list))
        {
            cam_info.armInfoList = arm_info_list;
            LOG_DEBUG("===== 成功从参数服务器读取相机cam_id= %d 的机械臂配置，数量：%d", static_cast<int>(cam_info.cam_id), cam_info.armInfoList.size());
            for (const auto& arm_info : arm_info_list)
            {
                LOG_DEBUG("获取配置的机械臂ID: %d, 是否启用: %s", arm_info.arm_id, arm_info.is_enable ? "是" : "否");
            }
        }
        else
        {
            LOG_ERROR("===== 从参数服务器读取相机cam_id= %d 的机械臂配置失败", static_cast<int>(cam_info.cam_id));
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("从参数服务器读取相机参数时出错: %s", e.what());
        return false;  // 读取失败
    }
}

bool getCamInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    uint8_t cam_id, SysConfig::CamConfigInfo& cam_info)
{
    const std::string prefix_cam_info = basros::updateNodeName(std::string(SYS_ENABLE_CAM_LIST), NODE_NAME_CAM_INFO);// 构造cam_info参数前缀
    return getCamInfoFromServer(client, prefix_cam_info, cam_id, cam_info);
}

/**
 * @brief 从参数服务器读取CamConfigInfo1D数据
 * 该函数用于从参数服务器中读取相机配置信息列表，并返回读取状态。
 * 函数首先读取cam_ids参数来确定系统中配置的相机ID列表，
 * 然后根据相机ID列表逐个读取对应的相机配置信息。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀
 * @param cam_info_list[out] 读取到的相机配置信息列表
 * @return 读取状态，true表示成功，false表示失败
 */
bool getCamInfoListFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    const std::string& param_prefix, SysConfig::CamConfigInfo1D& cam_info_list)
{
    try 
    {
        cam_info_list.clear(); // 清空输出列表
        if (param_prefix.empty()) // 检查参数前缀是否为空
        {
            LOG_ERROR("参数前缀不能为空");
            return false;
        }
        LOG_DEBUG("===== 开始从参数服务器读取相机配置信息列表 ===== ");
        const std::string cam_ids_param = basros::updateNodeName(param_prefix, NODE_NAME_CAM_IDS);// 构造cam_ids参数名
        std::vector<uint8_t> cam_ids;// 读取cam_ids参数来确定需要读取哪些相机的配置信息
        if (!basros::paramFromServer(client, cam_ids_param, cam_ids, ""))
        {
            LOG_ERROR("参数 %s 不存在，无法读取相机配置信息", cam_ids_param.c_str());
            LOG_DEBUG("===== 结束从参数服务器读取相机配置信息列表 ===== ");
            return false;
        }
        std::string cam_ids_str = basmodule::get_list_string(cam_ids);
        LOG_DEBUG("系统相机个数: %d %s, 参数前缀: %s", static_cast<int>(cam_ids.size()), cam_ids_str.c_str(), param_prefix.c_str());
        const std::string prefix_cam_info = basros::updateNodeName(param_prefix, NODE_NAME_CAM_INFO);// 构造cam_info参数前缀
        for (size_t i = 0; i < cam_ids.size(); ++i) // 根据相机ID列表逐个读取相机配置信息
        {
            const auto& cam_id = cam_ids[i];
            LOG_DEBUG("读取第 %d 个相机cam_id= %d 的配置信息...", static_cast<int>(i + 1), static_cast<int>(cam_id));
            SysConfig::CamConfigInfo cam_info; // 创建相机配置信息对象并初始化
            if (getCamInfoFromServer(client, prefix_cam_info, cam_id, cam_info)) 
            {
                cam_info_list.push_back(cam_info);
                LOG_DEBUG("第 %d 个相机cam_id= %d 的配置信息读取成功", static_cast<int>(i + 1), static_cast<int>(cam_id));
            } else {
                LOG_WARN("读取相机cam_id= %d的参数失败", cam_id);
                return false;
            }
        }
        LOG_DEBUG("完成相机配置信息列表读取, 共读取 %d 个相机配置", static_cast<int>(cam_info_list.size()));
        LOG_DEBUG("===== 结束从参数服务器读取相机配置信息列表 ===== ");
        return true; // 成功读取
    } catch (const std::exception& e) {
        LOG_ERROR("从参数服务器读取相机配置信息列表时出错: %s", e.what());
        LOG_DEBUG("===== 结束从参数服务器读取相机配置信息列表 ===== ");
        return false; // 读取失败
    }
}


/**
 * @brief 从参数服务器读取相机ID列表
 * 该函数用于从参数服务器中读取系统配置的相机ID列表。
 * @param client 参数服务器客户端节点
 * @return 读取到的相机ID列表，如果读取失败则返回空列表
 */
std::vector<uint8_t> getCamIdsFromServer(const rclcpp::SyncParametersClient::SharedPtr& client)
{
    std::vector<uint8_t> cam_ids;// 读取cam_ids参数来确定需要读取哪些相机的配置信息
    try
    {
        const std::string cam_ids_param = basros::updateNodeName(std::string(SYS_ENABLE_CAM_LIST), NODE_NAME_CAM_IDS);// 构造cam_ids参数名
        if (!basros::paramFromServer(client, cam_ids_param, cam_ids, ""))
        {
            LOG_ERROR("参数 %s 不存在，无法读取相机配置信息", cam_ids_param.c_str());
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("从参数服务器读取相机ID列表时出错: %s", e.what());
    }  
    return cam_ids;
}

}  // namespace RosComm
