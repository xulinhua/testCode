/**
 * @file param_to_server.h
 * @brief ROS通信工具函数
 * 
 * 提供系统配置信息与参数服务器格式之间的转换函数，以及通信信息解析功能
 */
#ifndef BAS_SYS_CONFIG_ROS__PARAM_TO_SERVER_H_
#define BAS_SYS_CONFIG_ROS__PARAM_TO_SERVER_H_

#include "bas_sys_config/sys_config_struct.hpp"
#include "bas_operate_ros/ros_comm_info.h"
#include "hand_eye_calib/calib_struct.hpp"
#include "data_handler/param_reflector.hpp"
#include <map>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>

// 前向声明
namespace SysConfig {
    using ParamInfo = ::datahandler::ParamInfo;
}
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
    std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将ArmConfigInfoList转换为参数服务器可存储的格式
 * 该函数使用反射机制辅助转换机械臂配置信息列表，便于后续存储到ROS 2参数服务器中。
 * @param para_infos 机械臂配置信息列表引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool armInfoListToRos(const SysConfig::ArmConfigInfoList& para_infos, 
    const std::string& param_prefix, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将ArmConfigInfoList转换为参数服务器可存储的格式，并直接声明和设置参数
 * 该函数使用反射机制辅助转换机械臂配置信息列表并向参数服务器声明和设置这些参数。
 * @param node ROS节点指针，用于访问参数服务器
 * @param param_prefix 参数前缀字符串
 * @param para_infos 机械臂配置信息列表引用
 * @return 操作是否成功
 */
bool setArmInfoListToServer(rclcpp::Node::SharedPtr node, const std::string& param_prefix, 
    const SysConfig::ArmConfigInfoList& para_infos);

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
    const std::string& param_prefix, uint8_t arm_id, SysConfig::ArmConfigInfo& arm_info);

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
    const std::string& param_prefix, SysConfig::ArmConfigInfoList& arm_info_list);

/**
 * @brief 将CamConfigInfo转换为参数服务器可存储的格式
 * 该函数使用反射机制自动转换CamConfigInfo的所有参数，避免手动逐个转换。
 * @param para_info 相机配置信息结构体引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool camInfoToRos(const SysConfig::CamConfigInfo& para_info, const std::string& param_prefix, 
    std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将CamConfigInfo1D转换为参数服务器可存储的格式
 * 该函数用于将相机配置信息列表转换为参数向量，便于后续存储到ROS 2参数服务器中。
 * @param para_infos 相机配置信息列表引用
 * @param param_prefix 参数前缀字符串
 * @param[out] ros_paras 输出参数向量，用于存储转换后的参数
 * @return 操作是否成功
 */
bool camInfoListToRos(const SysConfig::CamConfigInfo1D& para_infos, const std::string& param_prefix, 
    std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将CamConfigInfo1D转换为参数服务器可存储的格式，并直接声明和设置参数
 * 该函数使用反射机制辅助转换相机配置信息列表并向参数服务器声明和设置这些参数。
 * 此外，该函数还会处理每个相机关联的机械臂配置信息，将具体的机械臂配置也转换为参数。
 * @param node ROS节点指针，用于访问参数服务器
 * @param param_prefix 参数前缀字符串
 * @param para_infos 相机配置信息列表引用
 * @return 操作是否成功
 */
bool setCamInfoListToServer(rclcpp::Node::SharedPtr node, const std::string& param_prefix, 
    const SysConfig::CamConfigInfo1D& para_infos);

/**
 * @brief 从参数服务器读取CamConfigInfo1D数据
 * 该函数使用反射机制辅助读取相机配置信息列表，并返回读取状态。
 * 函数首先尝试读取启用的相机ID列表(param_prefix.sys_enable_cam_list)，
 * 然后根据启用的相机ID列表读取对应的相机配置信息。
 * 如果启用相机列表参数不存在，则回退到传统的遍历方式，
 * 通过检查param_prefix.cam_{index}.is_enable参数是否存在来判断。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀
 * @param cam_list[out] 读取到的相机配置信息列表
 * @return 读取状态，true表示成功，false表示失败
 */
bool getCamInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    const std::string& param_prefix, uint8_t cam_id, SysConfig::CamConfigInfo& cam_info);

bool getCamInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    uint8_t cam_id, SysConfig::CamConfigInfo& cam_info);
    
/**
 * @brief 从参数服务器读取CamConfigInfo1D数据
 * 该函数使用反射机制辅助读取相机配置信息列表，并返回读取状态。
 * 函数首先尝试读取启用的相机ID列表(param_prefix.sys_enable_cam_list)，
 * 然后根据启用的相机ID列表读取对应的相机配置信息。
 * 如果启用相机列表参数不存在，则回退到传统的遍历方式，
 * 通过检查param_prefix.cam_{index}.is_enable参数是否存在来判断。
 * @param client 参数服务器客户端节点
 * @param param_prefix 参数前缀
 * @param cam_list[out] 读取到的相机配置信息列表
 * @return 读取状态，true表示成功，false表示失败
 */
bool getCamInfoListFromServer(const rclcpp::SyncParametersClient::SharedPtr& client, 
    const std::string& param_prefix, SysConfig::CamConfigInfo1D& cam_info_list);

/**
 * @brief 从参数服务器读取相机ID列表
 * 该函数用于从参数服务器中读取系统配置的相机ID列表。
 * @param client 参数服务器客户端节点
 * @return 读取到的相机ID列表，如果读取失败则返回空列表
 */
std::vector<uint8_t> getCamIdsFromServer(const rclcpp::SyncParametersClient::SharedPtr& client);

}
#endif  // BAS_SYS_CONFIG_ROS__PARAM_TO_SERVER_H_