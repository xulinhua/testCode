#ifndef BAS_OPERATE_ROS_PARAM_FROM_SERVER_HPP
#define BAS_OPERATE_ROS_PARAM_FROM_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <functional>
#include <type_traits>
#include "data_handler/param_reflector.hpp"
#include "bas_operate/bas_utils.hpp"
#include <iostream>

namespace basros {

/**
 * @brief 从ROS参数服务器向量中读取单个参数并设置到参数信息中
 * @param ros_para 单个参数对象
 * @param para_info 参数信息结构体
 * @return 是否成功读取参数
 */
bool paraInfoFromRosType(const rclcpp::Parameter& ros_para, datahandler::ParamInfo& para_info);

/**
 * @brief 从ROS参数服务器向量中读取单个参数并设置到参数信息中
 * @param ros_para 单个参数对象
 * @param prefix_node 参数前缀节点名称
 * @param para_info 参数信息结构体
 * @return 是否成功读取参数
 */
bool paraInfoFromRosType(const rclcpp::Parameter& ros_para,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

/**
 * @brief 从ROS参数服务器映射中读取单个参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool paraInfoFromRosType(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

/**
 * @brief 从ROS参数服务器映射中读取参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param prefix_node 参数前缀节点名称
 * @param para_infos 参数信息向量
 * @return 是否成功读取所有参数
 */
bool paraInfoFromRosType(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, std::vector<datahandler::ParamInfo>& para_infos);

/**
 * @brief 从参数服务器获取参数信息向量
 * @param client 参数服务器客户端
 * @param para_infos 参数信息向量，用于存储从服务器获取的参数
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功获取所有参数
 */
bool paraInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    std::vector<datahandler::ParamInfo>& para_infos, const std::string& prefix_node);

/**
 * @brief 从参数服务器获取单个参数信息
 * @param client 参数服务器客户端
 * @param para_info 参数信息，用于存储从服务器获取的参数
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功获取参数
 */
bool paraInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    datahandler::ParamInfo& para_info, const std::string& prefix_node);

/**
 * @brief 通用模板函数：从ROS节点中读取单个参数值
 * @tparam T 参数类型（支持 bool, int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t, float, double, std::string 及其对应的vector类型）
 * @param node ROS节点指针
 * @param para_name 参数名称
 * @param value 输出参数，存储读取的值
 * @param param_prefix 参数前缀（可选）
 * @return 是否成功读取参数
 * @note 支持的类型包括：bool, 整数类型, 浮点类型, std::string, 及其对应的vector类型
 */
template<typename T>
bool paramFromNode(rclcpp::Node* node, const std::string& para_name, T& value, const std::string& param_prefix = "");

/**
 * @brief 从参数服务器获取单个参数值的通用模板函数
 * @tparam T 参数值类型
 * @param client 参数服务器客户端
 * @param para_name 参数名称
 * @param value 输出参数，存储读取的值
 * @param param_prefix 参数前缀（可选）
 * @return 是否成功读取参数
 */
template<typename T>
bool paramFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    const std::string& para_name, T& value, const std::string& param_prefix = "");

/**
 * @brief 从参数服务器获取参数列表
 * 该函数用于从ROS参数服务器批量获取参数值
 * @param client 参数服务器客户端
 * @param para_names 要获取的参数名称列表
 * @param param_prefix 参数前缀
 * @return 从参数服务器获取的参数列表
 */
std::vector<rclcpp::Parameter> rosParaFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    const std::vector<std::string>& para_names, std::string param_prefix = "");

/**
 * @brief 从参数服务器获取参数列表（基于ParamInfo结构体）
 * 该函数使用ParamInfo结构体列表提取参数名称并从ROS参数服务器批量获取参数值
 * @param client 参数服务器客户端
 * @param para_infos 参数信息结构体向量
 * @param param_prefix 参数前缀
 * @return 从参数服务器获取的参数列表
 */
std::vector<rclcpp::Parameter> rosParaFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    const std::vector<datahandler::ParamInfo>& para_infos, std::string param_prefix);

// 通用模板函数：从参数对象中读取值
template<typename T>
bool paramFromRos(const rclcpp::Parameter& param, T& value);

// 特化版本声明
template<>
bool paramFromRos<bool>(const rclcpp::Parameter& param, bool& value);

template<>
bool paramFromRos<int8_t>(const rclcpp::Parameter& param, int8_t& value);

template<>
bool paramFromRos<uint8_t>(const rclcpp::Parameter& param, uint8_t& value);

template<>
bool paramFromRos<int16_t>(const rclcpp::Parameter& param, int16_t& value);

template<>
bool paramFromRos<uint16_t>(const rclcpp::Parameter& param, uint16_t& value);

template<>
bool paramFromRos<int32_t>(const rclcpp::Parameter& param, int32_t& value);

template<>
bool paramFromRos<uint32_t>(const rclcpp::Parameter& param, uint32_t& value);

template<>
bool paramFromRos<int64_t>(const rclcpp::Parameter& param, int64_t& value);

template<>
bool paramFromRos<uint64_t>(const rclcpp::Parameter& param, uint64_t& value);

template<>
bool paramFromRos<float>(const rclcpp::Parameter& param, float& value);

template<>
bool paramFromRos<double>(const rclcpp::Parameter& param, double& value);

template<>
bool paramFromRos<std::string>(const rclcpp::Parameter& param, std::string& value);

template<>
bool paramFromRos<std::vector<bool>>(const rclcpp::Parameter& param, std::vector<bool>& value);

template<>
bool paramFromRos<std::vector<int8_t>>(const rclcpp::Parameter& param, std::vector<int8_t>& value);

template<>
bool paramFromRos<std::vector<uint8_t>>(const rclcpp::Parameter& param, std::vector<uint8_t>& value);

template<>
bool paramFromRos<std::vector<int16_t>>(const rclcpp::Parameter& param, std::vector<int16_t>& value);

template<>
bool paramFromRos<std::vector<uint16_t>>(const rclcpp::Parameter& param, std::vector<uint16_t>& value);

template<>
bool paramFromRos<std::vector<int32_t>>(const rclcpp::Parameter& param, std::vector<int32_t>& value);

template<>
bool paramFromRos<std::vector<uint32_t>>(const rclcpp::Parameter& param, std::vector<uint32_t>& value);

template<>
bool paramFromRos<std::vector<int64_t>>(const rclcpp::Parameter& param, std::vector<int64_t>& value);

template<>
bool paramFromRos<std::vector<uint64_t>>(const rclcpp::Parameter& param, std::vector<uint64_t>& value);

template<>
bool paramFromRos<std::vector<float>>(const rclcpp::Parameter& param, std::vector<float>& value);

template<>
bool paramFromRos<std::vector<double>>(const rclcpp::Parameter& param, std::vector<double>& value);

template<>
bool paramFromRos<std::vector<std::string>>(const rclcpp::Parameter& param, std::vector<std::string>& value);

/**
 * @brief 通用模板函数：从参数服务器向量中读取单个参数值
 * @tparam T 参数类型（支持 bool, int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t, float, double, std::string 及其对应的vector类型）
 * @param ros_paras 参数服务器向量
 * @param para_name 参数名称
 * @param value 输出参数，存储读取的值
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 * @note 支持的类型包括：bool, 整数类型, 浮点类型, std::string, 及其对应的vector类型
 */
template<typename T>
bool paramFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& para_name, T& value, const std::string& prefix_node = "");

/**
 * @brief 从ROS参数服务器读取BOOL类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool boolParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// bool 类型
bool boolParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool boolParamFromRos(const rclcpp::Parameter& param, bool& value);

/**
 * @brief 从ROS参数服务器读取INT8类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool int8ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// int8_t 类型
bool int8ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool int8ParamFromRos(const rclcpp::Parameter& param, int8_t& value);

/**
 * @brief 从ROS参数服务器读取UINT8类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool uint8ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// uint8_t 类型
bool uint8ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool uint8ParamFromRos(const rclcpp::Parameter& param, uint8_t& value);

/**
 * @brief 从ROS参数服务器读取INT16类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool int16ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// int16_t 类型
bool int16ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool int16ParamFromRos(const rclcpp::Parameter& param, int16_t& value);

/**
 * @brief 从ROS参数服务器读取UINT16类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool uint16ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// uint16_t 类型
bool uint16ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool uint16ParamFromRos(const rclcpp::Parameter& param, uint16_t& value);

/**
 * @brief 从ROS参数服务器读取INT32类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool int32ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// int32_t 类型
bool int32ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool int32ParamFromRos(const rclcpp::Parameter& param, int32_t& value);

/**
 * @brief 从ROS参数服务器读取UINT32类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool uint32ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// uint32_t 类型
bool uint32ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool uint32ParamFromRos(const rclcpp::Parameter& param, uint32_t& value);

/**
 * @brief 从ROS参数服务器读取INT64类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool int64ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// int64_t 类型
bool int64ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool int64ParamFromRos(const rclcpp::Parameter& param, int64_t& value);

/**
 * @brief 从ROS参数服务器读取UINT64类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool uint64ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// uint64_t 类型
bool uint64ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool uint64ParamFromRos(const rclcpp::Parameter& param, uint64_t& value);

/**
 * @brief 从ROS参数服务器读取FLOAT类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool floatParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// float 类型
bool floatParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool floatParamFromRos(const rclcpp::Parameter& param, float& value);

/**
 * @brief 从ROS参数服务器读取DOUBLE类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool doubleParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// double 类型
bool doubleParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool doubleParamFromRos(const rclcpp::Parameter& param, double& value);

/**
 * @brief 从ROS参数服务器读取STRING类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool stringParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// std::string 类型
bool stringParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool stringParamFromRos(const rclcpp::Parameter& param, std::string& value);

/**
 * @brief 从ROS参数服务器读取BOOL_ARRAY类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool boolArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// std::vector<bool> 类型
bool boolArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool boolArrayParamFromRos(const rclcpp::Parameter& param, std::vector<bool>& value);

/**
 * @brief 从ROS参数服务器读取INT8_ARRAY类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool int8ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// std::vector<int8_t> 类型
bool int8ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool int8ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<int8_t>& value);

/**
 * @brief 从ROS参数服务器读取UINT8_ARRAY类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool uint8ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// std::vector<uint8_t> 类型
bool uint8ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool uint8ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<uint8_t>& value);

/**
 * @brief 从ROS参数服务器读取INT16_ARRAY类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool int16ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// std::vector<int16_t> 类型
bool int16ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool int16ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<int16_t>& value);

/**
 * @brief 从ROS参数服务器读取UINT16_ARRAY类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool uint16ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// std::vector<uint16_t> 类型
bool uint16ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool uint16ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<uint16_t>& value);

/**
 * @brief 从ROS参数服务器读取INT32_ARRAY类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool int32ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// std::vector<int32_t> 类型
bool int32ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool int32ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<int32_t>& value);

/**
 * @brief 从ROS参数服务器读取UINT32_ARRAY类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool uint32ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// std::vector<uint32_t> 类型
bool uint32ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool uint32ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<uint32_t>& value);

/**
 * @brief 从ROS参数服务器读取INT64_ARRAY类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool int64ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// std::vector<int64_t> 类型
bool int64ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool int64ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<int64_t>& value);

/**
 * @brief 从ROS参数服务器读取UINT64_ARRAY类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool uint64ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// std::vector<uint64_t> 类型
bool uint64ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool uint64ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<uint64_t>& value);

/**
 * @brief 从ROS参数服务器读取FLOAT_ARRAY类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool floatArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// std::vector<float> 类型
bool floatArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool floatArrayParamFromRos(const rclcpp::Parameter& param, std::vector<float>& value);

/**
 * @brief 从ROS参数服务器读取DOUBLE_ARRAY类型参数并设置到参数信息中
 * @param ros_paras 参数服务器向量
 * @param para_info 参数信息结构体
 * @param prefix_node 参数前缀节点名称
 * @return 是否成功读取参数
 */
bool doubleArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info);

// std::vector<double> 类型
bool doubleArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool doubleArrayParamFromRos(const rclcpp::Parameter& param, std::vector<double>& value);

// std::vector<std::string> 类型
bool stringArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info);
bool stringArrayParamFromRos(const rclcpp::Parameter& param, std::vector<std::string>& value);

} // namespace basros

#endif // BAS_OPERATE_ROS_PARAM_FROM_SERVER_HPP