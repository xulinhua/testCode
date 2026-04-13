#ifndef BAS_OPERATE_ROS_PARAM_TO_SERVER_HPP
#define BAS_OPERATE_ROS_PARAM_TO_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <functional>
#include "data_handler/param_reflector.hpp"
#include "bas_operate/bas_utils.hpp"
#include "bas_operate_ros/param_utils.hpp"
#include <iostream>

namespace basros {

/**
 * @brief 将单个参数设置到ROS参数服务器
 * @param node ROS节点指针
 * @param ros_para 要设置的参数
 * @return 是否成功设置参数
 */
bool paraInfoToServer(rclcpp::Node* node, const rclcpp::Parameter& ros_para);

/**
 * @brief 将参数向量原子性设置到ROS参数服务器（一次性设置所有参数）
 * @param node ROS节点指针
 * @param ros_paras 要设置的参数向量
 * @return 是否成功设置所有参数
 */
bool paraInfoToServer(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 通用参数转换函数，将参数信息转换为rclcpp::Parameter并记录日志
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 存储转换后的ROS参数
 * @return 是否成功转换参数
 */
bool paraInfoToRos(const datahandler::ParamInfo& param_info, const std::string& prefix_node, rclcpp::Parameter& ros_para);

    /**
 * @brief 通用参数转换函数，将参数信息向量转换为ROS Parameter向量
 * @param param_infos bas_operate中的参数信息向量
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 存储转换后的ROS参数向量
 * @return 是否成功转换所有参数
 */
bool paraInfoToRos(const std::vector<datahandler::ParamInfo>& param_infos, const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 更新或添加参数到ros_paras
 * @param ros_paras ROS参数列表
 * @param param 要添加的参数
 */
void updateOrAddToRos(std::vector<rclcpp::Parameter>& ros_paras, const rclcpp::Parameter& param);

/**
 * @brief 通用参数创建函数，根据参数名称和值创建rclcpp::Parameter，可选择添加参数前缀
 * @tparam T 参数值的类型
 * @param para_name 参数名称
 * @param value 参数值
 * @param param_prefix 参数前缀（可选，默认为空字符串）
 * @return rclcpp::Parameter对象
 */
template<typename T>
rclcpp::Parameter paramToRos(const std::string& para_name, const T& value, const std::string& param_prefix = "") {
    if (param_prefix.empty()) {
        return rclcpp::Parameter(para_name, value);
    } else {
        return rclcpp::Parameter(updateNodeName(param_prefix, para_name), value);
    }
}
/**
 * @brief 通用参数创建函数，根据参数名称和值创建rclcpp::Parameter并添加到参数列表，可选择添加参数前缀
 * @tparam T 参数值的类型
 * @param ros_paras 参数列表
 * @param para_name 参数名称
 * @param value 参数值
 * @param param_prefix 参数前缀（可选，默认为空字符串）
 */
template<typename T>
void paramToRos(std::vector<rclcpp::Parameter>& ros_paras, const std::string& para_name, const T& value, const std::string& param_prefix = "") 
{
    if (param_prefix.empty()) {
        updateOrAddToRos(ros_paras, rclcpp::Parameter(para_name, value));
    } else {
        updateOrAddToRos(ros_paras, rclcpp::Parameter(updateNodeName(param_prefix, para_name), value));
    }
}

/**
 * @brief 将BOOL类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void boolParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将BOOL类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void boolParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将INT8类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void int8ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将INT8类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void int8ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将UINT8类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void uint8ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将UINT8类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void uint8ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将INT16类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void int16ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将INT16类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void int16ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将UINT16类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void uint16ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将UINT16类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void uint16ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将INT32类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void int32ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将INT32类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void int32ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将UINT32类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void uint32ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将UINT32类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void uint32ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将INT64类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void int64ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将INT64类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void int64ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将UINT64类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void uint64ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将UINT64类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void uint64ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将FLOAT类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void floatParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将FLOAT类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void floatParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将DOUBLE类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void doubleParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将DOUBLE类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void doubleParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将STRING类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void stringParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将STRING类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void stringParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将BOOL_ARRAY类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void boolArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将BOOL_ARRAY类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void boolArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将INT8_ARRAY类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void int8ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将INT8_ARRAY类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void int8ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将UINT8_ARRAY类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void uint8ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将UINT8_ARRAY类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void uint8ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将INT16_ARRAY类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void int16ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将INT16_ARRAY类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void int16ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将UINT16_ARRAY类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void uint16ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将UINT16_ARRAY类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void uint16ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将INT32_ARRAY类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void int32ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将INT32_ARRAY类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void int32ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将UINT32_ARRAY类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void uint32ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将UINT32_ARRAY类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void uint32ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将INT64_ARRAY类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void int64ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将INT64_ARRAY类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void int64ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将UINT64_ARRAY类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void uint64ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将UINT64_ARRAY类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void uint64ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将FLOAT_ARRAY类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void floatArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将FLOAT_ARRAY类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void floatArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将DOUBLE_ARRAY类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void doubleArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将DOUBLE_ARRAY类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void doubleArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

/**
 * @brief 将STRING_ARRAY类型的参数信息转换为rclcpp::Parameter
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 参数向量，用于存储转换后的ROS参数
 */
void stringArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras);

/**
 * @brief 将STRING_ARRAY类型的参数信息转换为rclcpp::Parameter (重载函数)
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 单个参数，用于存储转换后的ROS参数
 */
void stringArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para);

} // namespace basros

#endif // BAS_OPERATE_ROS_PARAM_TO_SERVER_HPP