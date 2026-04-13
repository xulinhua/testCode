#include "bas_operate_ros/param_from_server.hpp"
#include "bas_operate_ros/param_utils.hpp"
#include "log_system/log_macros.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <functional>
#include <type_traits>
#include <iostream>
#include <memory>
#include "data_handler/param_reflector.hpp"
#include "bas_operate/bas_utils.hpp"

namespace basros {

bool paraInfoFromRosType(const rclcpp::Parameter& ros_para, datahandler::ParamInfo& para_info)
{
    bool success = false;
    switch (para_info.type) 
    {
        case datahandler::ParamType::BOOL:
            success = boolParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::INT8:
            success = int8ParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::UINT8:
            success = uint8ParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::INT16:
            success = int16ParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::UINT16:
            success = uint16ParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::INT32:
            success = int32ParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::UINT32:
            success = uint32ParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::INT64:
            success = int64ParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::UINT64:
            success = uint64ParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::FLOAT:
            success = floatParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::DOUBLE:
            success = doubleParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::STRING:
            success = stringParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::BOOL_ARRAY:
            success = boolArrayParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::INT8_ARRAY:
            success = int8ArrayParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::UINT8_ARRAY:
            success = uint8ArrayParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::INT16_ARRAY:
            success = int16ArrayParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::UINT16_ARRAY:
            success = uint16ArrayParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::INT32_ARRAY:
            success = int32ArrayParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::UINT32_ARRAY:
            success = uint32ArrayParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::INT64_ARRAY:
            success = int64ArrayParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::UINT64_ARRAY:
            success = uint64ArrayParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::FLOAT_ARRAY:
            success = floatArrayParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::DOUBLE_ARRAY:
            success = doubleArrayParamFromRos(ros_para, para_info);
            break;
        case datahandler::ParamType::STRING_ARRAY:
            success = stringArrayParamFromRos(ros_para, para_info);
            break;
        default:
            LOG_ERROR("不支持的参数类型: %d", static_cast<int>(para_info.type));
            success = false;
            break;
    }
    return success;
}

bool paraInfoFromRosType(const rclcpp::Parameter& ros_para,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    if (prefix_node.empty()) 
    {
        LOG_ERROR("参数前缀节点名称不能为空");
        return false;
    }
    const std::string node_name = updateNodeName(prefix_node, para_info.name); 
    if (ros_para.get_name() != node_name) // 检查参数名称是否匹配
    {
        LOG_ERROR("参数名称不匹配: 期望 %s, 实际 %s", node_name.c_str(), ros_para.get_name().c_str());
        return false;
    }
    LOG_DEBUG("参数名称: %s, 参数类型: %s", node_name.c_str(), para_info.getTypeString().c_str());
    // 根据参数类型进行转换
    bool success = paraInfoFromRosType(ros_para, para_info);
    return success;
}

bool paraInfoFromRosType(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    if (prefix_node.empty()) 
    {
        LOG_ERROR("参数前缀节点名称不能为空");
        return false;
    }
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    LOG_DEBUG("参数名称: %s, 参数类型: %s", node_name.c_str(), para_info.getTypeString().c_str()); 
    // 在参数列表中查找匹配的参数
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    return paraInfoFromRosType(*it, prefix_node, para_info);
}

bool paraInfoFromRosType(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, std::vector<datahandler::ParamInfo>& para_infos)
{
    if (prefix_node.empty()) 
    {
        LOG_ERROR("参数前缀节点名称不能为空");
        return false;
    }
    bool overall_success = true;
    for (auto& para_info : para_infos) 
    {
        bool success = paraInfoFromRosType(ros_paras, prefix_node, para_info);
        overall_success = overall_success && success;
    }
    return overall_success;
}

// 从参数服务器获取参数信息的函数实现
bool paraInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    std::vector<datahandler::ParamInfo>& para_infos, const std::string& prefix_node)
{
    if (!client) 
    {
        LOG_ERROR("参数服务器客户端为空");
        return false;
    }
    if (prefix_node.empty()) 
    {
        LOG_ERROR("参数前缀节点名称不能为空");
        return false;
    }
    std::vector<std::string> para_names = updateNodeName(prefix_node, para_infos); // 获取参数名称列表
    std::vector<rclcpp::Parameter> ros_paras;
    try 
    {
        ros_paras = client->get_parameters(para_names);// 从参数服务器获取参数
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("从参数服务器获取参数列表时出错: %s", e.what());
        return false;
    }
    catch (...) 
    {
        LOG_ERROR("从参数服务器获取参数列表时发生未知错误");
        return false;
    }
    if (ros_paras.empty()) 
    {
        LOG_ERROR("从参数服务器未获取到任何参数");
        return false;
    }
    return paraInfoFromRosType(ros_paras, prefix_node, para_infos);
}

bool paraInfoFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    datahandler::ParamInfo& para_info, const std::string& prefix_node)
{
    if (!client) 
    {
        LOG_ERROR("参数服务器客户端为空");
        return false;
    }
    if (prefix_node.empty()) 
    {
        LOG_ERROR("参数前缀节点名称不能为空");
        return false;
    }
    std::string full_param_name = updateNodeName(prefix_node, para_info.name);// 构建完整的参数名称
    try 
    {
        // 检查参数是否存在
        if (client->has_parameter(full_param_name)) 
        {   
            // 使用get_parameters方法获取参数，这是ROS Humble兼容的方法
            std::vector<rclcpp::Parameter> params = client->get_parameters({full_param_name});
            if (!params.empty() && params[0].get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) 
            {
                return paraInfoFromRosType(params[0], prefix_node, para_info); // 调用具体的类型转换函数
            } 
            else 
            {
                LOG_ERROR("参数 %s 不存在或类型未设置", full_param_name.c_str());
                return false;
            }
        } 
        else 
        {
            LOG_ERROR("参数 %s 不存在", full_param_name.c_str());
            return false;
        }
    } 
    catch (const std::exception& e) 
    {
        LOG_ERROR("从参数服务器获取参数 %s 时出错: %s", full_param_name.c_str(), e.what());
        return false;
    }
    catch (...) 
    {
        LOG_ERROR("从参数服务器获取参数 %s 时发生未知错误", full_param_name.c_str());
        return false;
    }
}

// 从ROS节点获取参数的函数实现
template<typename T>
bool paramFromNode(rclcpp::Node* node, const std::string& para_name, T& value, const std::string& param_prefix)
{
    if (!node) {
        LOG_ERROR("ROS节点指针为空");
        return false;
    }
    const std::string node_name = updateNodeName(param_prefix, para_name); 
    try 
    {
        if (node->has_parameter(node_name)) 
        {
            value = node->get_parameter(node_name).get_value<T>();
            return true;
        } 
        else 
        {
            LOG_ERROR("节点参数 %s 不存在", node_name.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("读取节点参数 %s 时出错: %s", node_name.c_str(), e.what());
        return false;
    }
}

// 从参数服务器获取单个参数值的函数实现
template<typename T>
bool paramFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    const std::string& para_name, T& value, const std::string& param_prefix)
{
    if (!client) 
    {
        LOG_ERROR("参数服务器客户端为空");
        return false;
    }
    const std::string node_name = updateNodeName(param_prefix, para_name); 
    try 
    {
        // 检查参数是否存在
        if (client->has_parameter(node_name)) 
        {   
            // 使用get_parameters方法获取参数，这是ROS Humble兼容的方法
            try 
            {
                std::vector<rclcpp::Parameter> params = client->get_parameters({node_name});
                if (!params.empty() && params[0].get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) 
                {
                    rclcpp::Parameter param = params[0];
                    return paramFromRos(param, value); // 调用具体的类型转换函数
                } 
                else 
                {
                    LOG_ERROR("参数 %s 不存在或类型未设置", node_name.c_str());
                    return false;
                }
            } 
            catch (const std::exception& e) 
            {
                LOG_ERROR("从参数服务器获取参数 %s 时出错: %s", node_name.c_str(), e.what());
                return false;
            }
            catch (...) 
            {
                LOG_ERROR("从参数服务器获取参数 %s 时发生未知错误", node_name.c_str());
                return false;
            }
        } 
        else 
        {
            LOG_ERROR("参数 %s 不存在", node_name.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("读取参数 %s 时出错: %s", node_name.c_str(), e.what());
        return false;
    }
}

std::vector<rclcpp::Parameter> rosParaFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    const std::vector<std::string>& para_names, std::string param_prefix)
{
    std::vector<rclcpp::Parameter> result;
    if (!client) 
    {
        LOG_ERROR("参数服务器客户端为空");
        return result;
    }
    if (para_names.empty()) 
    {
        LOG_WARN("参数名称列表为空");
        return result;
    }
    try 
    {
        const std::vector<std::string> node_names = updateNodeName(param_prefix, para_names); 
        result = client->get_parameters(node_names);// 使用get_parameters方法获取所有参数
        // 过滤掉未找到的参数
        auto it = std::remove_if(result.begin(), result.end(),
            [](const rclcpp::Parameter& param) {
                return param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET;
            });
        result.erase(it, result.end());
        LOG_INFO("成功从参数服务器获取 %zu 个参数，请求 %zu 个", result.size(), node_names.size());
        return result;
    } catch (const std::exception& e) {
        LOG_ERROR("从参数服务器获取参数时出错: %s", e.what());
        return result;
    }
}

std::vector<rclcpp::Parameter> rosParaFromServer(const rclcpp::SyncParametersClient::SharedPtr& client,
    const std::vector<datahandler::ParamInfo>& para_infos, std::string param_prefix)
{
    if (param_prefix.empty()) 
    {
        LOG_ERROR("参数前缀不能为空");
        std::vector<rclcpp::Parameter> result;
        return result;
    }
    std::vector<std::string> para_names = updateNodeName(param_prefix, para_infos); // 获取参数名称列表
    return rosParaFromServer(client, para_names, "");
}

// 通用模板函数：从参数对象中读取值
template<typename T>
bool paramFromRos(const rclcpp::Parameter& param, T& value)
{
    // 这个函数需要根据T的具体类型来调用相应的转换函数
    // 由于无法在模板中使用if-constexpr，我们使用SFINAE或其他方法
    // 但为了简化，这里将通过特化实现
    LOG_ERROR("paramFromRos未定义类型: %s", typeid(T).name());
    return false;
}

// 特化版本实现
template<>
bool paramFromRos<bool>(const rclcpp::Parameter& param, bool& value)
{
    return boolParamFromRos(param, value);
}

template<>
bool paramFromRos<int8_t>(const rclcpp::Parameter& param, int8_t& value)
{
    return int8ParamFromRos(param, value);
}

template<>
bool paramFromRos<uint8_t>(const rclcpp::Parameter& param, uint8_t& value)
{
    return uint8ParamFromRos(param, value);
}

template<>
bool paramFromRos<int16_t>(const rclcpp::Parameter& param, int16_t& value)
{
    return int16ParamFromRos(param, value);
}

template<>
bool paramFromRos<uint16_t>(const rclcpp::Parameter& param, uint16_t& value)
{
    return uint16ParamFromRos(param, value);
}

template<>
bool paramFromRos<int32_t>(const rclcpp::Parameter& param, int32_t& value)
{
    return int32ParamFromRos(param, value);
}

template<>
bool paramFromRos<uint32_t>(const rclcpp::Parameter& param, uint32_t& value)
{
    return uint32ParamFromRos(param, value);
}

template<>
bool paramFromRos<int64_t>(const rclcpp::Parameter& param, int64_t& value)
{
    return int64ParamFromRos(param, value);
}

template<>
bool paramFromRos<uint64_t>(const rclcpp::Parameter& param, uint64_t& value)
{
    return uint64ParamFromRos(param, value);
}

template<>
bool paramFromRos<float>(const rclcpp::Parameter& param, float& value)
{
    return floatParamFromRos(param, value);
}

template<>
bool paramFromRos<double>(const rclcpp::Parameter& param, double& value)
{
    return doubleParamFromRos(param, value);
}

template<>
bool paramFromRos<std::string>(const rclcpp::Parameter& param, std::string& value)
{
    return stringParamFromRos(param, value);
}

template<>
bool paramFromRos<std::vector<bool>>(const rclcpp::Parameter& param, std::vector<bool>& value)
{
    return boolArrayParamFromRos(param, value);
}

template<>
bool paramFromRos<std::vector<int8_t>>(const rclcpp::Parameter& param, std::vector<int8_t>& value)
{
    return int8ArrayParamFromRos(param, value);
}

template<>
bool paramFromRos<std::vector<uint8_t>>(const rclcpp::Parameter& param, std::vector<uint8_t>& value)
{
    return uint8ArrayParamFromRos(param, value);
}

template<>
bool paramFromRos<std::vector<int16_t>>(const rclcpp::Parameter& param, std::vector<int16_t>& value)
{
    return int16ArrayParamFromRos(param, value);
}

template<>
bool paramFromRos<std::vector<uint16_t>>(const rclcpp::Parameter& param, std::vector<uint16_t>& value)
{
    return uint16ArrayParamFromRos(param, value);
}

template<>
bool paramFromRos<std::vector<int32_t>>(const rclcpp::Parameter& param, std::vector<int32_t>& value)
{
    return int32ArrayParamFromRos(param, value);
}

template<>
bool paramFromRos<std::vector<uint32_t>>(const rclcpp::Parameter& param, std::vector<uint32_t>& value)
{
    return uint32ArrayParamFromRos(param, value);
}

template<>
bool paramFromRos<std::vector<int64_t>>(const rclcpp::Parameter& param, std::vector<int64_t>& value)
{
    return int64ArrayParamFromRos(param, value);
}

template<>
bool paramFromRos<std::vector<uint64_t>>(const rclcpp::Parameter& param, std::vector<uint64_t>& value)
{
    return uint64ArrayParamFromRos(param, value);
}

template<>
bool paramFromRos<std::vector<float>>(const rclcpp::Parameter& param, std::vector<float>& value)
{
    return floatArrayParamFromRos(param, value);
}

template<>
bool paramFromRos<std::vector<double>>(const rclcpp::Parameter& param, std::vector<double>& value)
{
    return doubleArrayParamFromRos(param, value);
}

template<>
bool paramFromRos<std::vector<std::string>>(const rclcpp::Parameter& param, std::vector<std::string>& value)
{
    return stringArrayParamFromRos(param, value);
}

// 从参数向量获取参数的函数实现
template<typename T>
bool paramFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& para_name, T& value, const std::string& prefix_node)
{
    const std::string node_name = updateNodeName(prefix_node, para_name); 
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it != ros_paras.end()) 
    {
        return paramFromRos(*it, value);
    } 
    else 
    {
        LOG_ERROR("参数向量中不存在参数 %s", node_name.c_str());
        return false;
    }
}

// 各种类型参数转换函数的实现
bool boolParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name); 
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    } 
    bool success = boolParamFromRos(*it, para_info);
    return success;
}

// 从参数对象提取值的函数实现
bool boolParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    bool value = false;
    bool success = boolParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}

// 从参数对象提取值的函数实现
bool boolParamFromRos(const rclcpp::Parameter& param, bool& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) 
        {
            value = param.as_bool();
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value ? "true" : "false", 
                datahandler::getTypeString(datahandler::ParamType::BOOL).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::BOOL).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::BOOL).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool int8ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    bool success = int8ParamFromRos(*it, para_info);
    return success;
}

bool int8ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    int8_t value;
    bool success = int8ParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}

bool int8ParamFromRos(const rclcpp::Parameter& param, int8_t& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) 
        {
            auto temp_value = param.as_int();
            if (temp_value >= std::numeric_limits<int8_t>::min() && temp_value <= std::numeric_limits<int8_t>::max()) 
            {
                value = static_cast<int8_t>(temp_value);
                LOG_DEBUG("参数: %s = %d  [类型: %s]", 
                    param.get_name().c_str(), static_cast<int>(value), 
                    datahandler::getTypeString(datahandler::ParamType::INT8).c_str());
                return true;
            } 
            else
            {
                LOG_ERROR("参数 %s 的值 %ld 超出int8_t范围 [%d, %d]", param.get_name().c_str(), temp_value, 
                    std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max());
                return false;
            }
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::INT8).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s", 
            datahandler::getTypeString(datahandler::ParamType::INT8).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool uint8ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    } 
    bool success = uint8ParamFromRos(*it, para_info);
    return success;
}

bool uint8ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    uint8_t value;
    bool success = uint8ParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}

bool uint8ParamFromRos(const rclcpp::Parameter& param, uint8_t& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) 
        {
            auto temp_value = param.as_int();
            if (temp_value >= std::numeric_limits<uint8_t>::min() && temp_value <= std::numeric_limits<uint8_t>::max()) 
            {
                value = static_cast<uint8_t>(temp_value);
                LOG_DEBUG("参数: %s = %u  [类型: %s]", param.get_name().c_str(), static_cast<unsigned int>(value), 
                    datahandler::getTypeString(datahandler::ParamType::UINT8).c_str());
                return true;
            } 
            else 
            {
                LOG_ERROR("参数 %s 的值 %ld 超出uint8_t范围 [%u, %u]", param.get_name().c_str(), temp_value, 
                    std::numeric_limits<uint8_t>::min(), std::numeric_limits<uint8_t>::max());
                return false;
            }
        } 
        else {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT8).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::UINT8).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool int16ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    bool success = int16ParamFromRos(*it, para_info);
    return success;
}

bool int16ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    int16_t value;
    bool success = int16ParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}

bool int16ParamFromRos(const rclcpp::Parameter& param, int16_t& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) 
        {
            auto temp_value = param.as_int();
            if (temp_value >= std::numeric_limits<int16_t>::min() && temp_value <= std::numeric_limits<int16_t>::max()) 
            {
                value = static_cast<int16_t>(temp_value);
                LOG_DEBUG("参数: %s = %d  [类型: %s]", param.get_name().c_str(), static_cast<int>(value), 
                    datahandler::getTypeString(datahandler::ParamType::INT16).c_str());
                return true;
            } 
            else 
            {
                LOG_ERROR("参数 %s 的值 %ld 超出int16_t范围 [%d, %d]", param.get_name().c_str(), temp_value, 
                    std::numeric_limits<int16_t>::min(), std::numeric_limits<int16_t>::max());
                return false;
            }
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::INT16).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::INT16).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool uint16ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);  
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    bool success = uint16ParamFromRos(*it, para_info);
    return success;
}

bool uint16ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    uint16_t value;
    bool success = uint16ParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}

bool uint16ParamFromRos(const rclcpp::Parameter& param, uint16_t& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) 
        {
            auto temp_value = param.as_int();
            if (temp_value >= std::numeric_limits<uint16_t>::min() && temp_value <= std::numeric_limits<uint16_t>::max()) 
            {
                value = static_cast<uint16_t>(temp_value);
                LOG_DEBUG("参数: %s = %u  [类型: %s]", param.get_name().c_str(), static_cast<unsigned int>(value), 
                    datahandler::getTypeString(datahandler::ParamType::UINT16).c_str());
                return true;
            } 
            else 
            {
                LOG_ERROR("参数 %s 的值 %ld 超出uint16_t范围 [%u, %u]", param.get_name().c_str(), temp_value, 
                    std::numeric_limits<uint16_t>::min(), std::numeric_limits<uint16_t>::max());
                return false;
            }
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT16).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::UINT16).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool int32ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    bool success = int32ParamFromRos(*it, para_info);
    return success;
}

bool int32ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    int32_t value;
    bool success = int32ParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}

bool int32ParamFromRos(const rclcpp::Parameter& param, int32_t& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) 
        {
            auto temp_value = param.as_int();
            if (temp_value >= std::numeric_limits<int32_t>::min() && temp_value <= std::numeric_limits<int32_t>::max()) {
                value = static_cast<int32_t>(temp_value);
                LOG_DEBUG("参数: %s = %d  [类型: %s]", param.get_name().c_str(), static_cast<int>(value), 
                    datahandler::getTypeString(datahandler::ParamType::INT32).c_str());
                return true;
            } 
            else 
            {
                LOG_ERROR("参数 %s 的值 %ld 超出int32_t范围 [%d, %d]", param.get_name().c_str(), temp_value, 
                    std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max());
                return false;
            }
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::INT32).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::INT32).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool uint32ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    bool success = uint32ParamFromRos(*it, para_info);
    return success;
}

bool uint32ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    uint32_t value;
    bool success = uint32ParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}

bool uint32ParamFromRos(const rclcpp::Parameter& param, uint32_t& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) 
        {
            auto temp_value = param.as_int();
            if (temp_value >= std::numeric_limits<uint32_t>::min() && temp_value <= std::numeric_limits<uint32_t>::max()) {
                value = static_cast<uint32_t>(temp_value);
                LOG_DEBUG("参数: %s = %u  [类型: %s]", param.get_name().c_str(), static_cast<unsigned int>(value), 
                    datahandler::getTypeString(datahandler::ParamType::UINT32).c_str());
                return true;
            } 
            else 
            {
                LOG_ERROR("参数 %s 的值 %ld 超出uint32_t范围 [%u, %u]", param.get_name().c_str(), temp_value, 
                    std::numeric_limits<uint32_t>::min(), std::numeric_limits<uint32_t>::max());
                return false;
            }
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT32).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::UINT32).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool int64ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    } 
    bool success = int64ParamFromRos(*it, para_info);
    return success;
}

bool int64ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    int64_t value;
    bool success = int64ParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}

bool int64ParamFromRos(const rclcpp::Parameter& param, int64_t& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) 
        {
            value = param.as_int();
            LOG_DEBUG("参数: %s = %ld  [类型: %s]", param.get_name().c_str(), value, 
                datahandler::getTypeString(datahandler::ParamType::INT64).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::INT64).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::INT64).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool uint64ParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    bool success = uint64ParamFromRos(*it, para_info);
    return success;
}

bool uint64ParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    uint64_t value;
    bool success = uint64ParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}

bool uint64ParamFromRos(const rclcpp::Parameter& param, uint64_t& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) 
        {
            auto temp_value = param.as_int();
            if (temp_value >= 0) 
            { // 确保值为非负数
                value = static_cast<uint64_t>(temp_value);
                LOG_DEBUG("参数: %s = %llu  [类型: %s]", param.get_name().c_str(), value, 
                    datahandler::getTypeString(datahandler::ParamType::UINT64).c_str());
                return true;
            } 
            else 
            {
                LOG_ERROR("参数 %s 的值 %ld 为负数，无法转换为uint64_t", param.get_name().c_str(), temp_value);
                return false;
            }
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT64).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::UINT64).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool floatParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    bool success = floatParamFromRos(*it, para_info);
    return success;
}

bool floatParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    float value;
    bool success = floatParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}

bool floatParamFromRos(const rclcpp::Parameter& param, float& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) 
        {
            auto temp_value = param.as_double();
            if (temp_value >= std::numeric_limits<float>::min() && temp_value <= std::numeric_limits<float>::max()) 
            {
                value = static_cast<float>(temp_value);
                LOG_DEBUG("参数: %s = %f  [类型: %s]", param.get_name().c_str(), value, 
                    datahandler::getTypeString(datahandler::ParamType::FLOAT).c_str());
                return true;
            } 
            else 
            {
                LOG_ERROR("参数 %s 的值 %lf 超出float范围", param.get_name().c_str(), temp_value);
                return false;
            }
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::FLOAT).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::FLOAT).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool doubleParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    bool success = doubleParamFromRos(*it, para_info);
    return success;
}

bool doubleParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    double value;
    bool success = doubleParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}   

bool doubleParamFromRos(const rclcpp::Parameter& param, double& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) 
        {
            value = param.as_double();
            LOG_DEBUG("参数: %s = %lf  [类型: %s]", param.get_name().c_str(), value, 
                datahandler::getTypeString(datahandler::ParamType::DOUBLE).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::DOUBLE).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::DOUBLE).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool stringParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::string value;
    bool success = stringParamFromRos(*it, para_info);
    return success;
}
bool stringParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::string value;
    bool success = stringParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}

bool stringParamFromRos(const rclcpp::Parameter& param, std::string& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) 
        {
            value = param.as_string();
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::STRING).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::STRING).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::STRING).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool boolArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::vector<bool> value;
    bool success = boolArrayParamFromRos(*it, para_info);
    return success;
}

bool boolArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::vector<bool> value;
    bool success = boolArrayParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    }
    return success;
}

bool boolArrayParamFromRos(const rclcpp::Parameter& param, std::vector<bool>& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) 
        {
            auto temp_value = param.as_bool_array();
            value.clear();
            value.reserve(temp_value.size());
            for (bool b : temp_value) 
            {
                value.push_back(b);
            }
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::BOOL_ARRAY).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::BOOL_ARRAY).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::BOOL_ARRAY).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool int8ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::vector<int8_t> value;
    bool success = int8ArrayParamFromRos(*it, para_info);
    return success;
}

bool int8ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::vector<int8_t> value;
    bool success = int8ArrayParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    } 
    return success;
}

bool int8ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<int8_t>& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) 
        {
            auto temp_value = param.as_integer_array();
            value.clear();
            for (const auto& v : temp_value) 
            {
                if (v >= std::numeric_limits<int8_t>::min() && v <= std::numeric_limits<int8_t>::max()) 
                {
                    value.push_back(static_cast<int8_t>(v));
                } 
                else 
                {
                    LOG_ERROR("数组中存在超出int8_t范围的值: %ld", v);
                    return false;
                }
            }
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::INT8_ARRAY).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::INT8_ARRAY).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::INT8_ARRAY).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool uint8ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::vector<uint8_t> value;
    bool success = uint8ArrayParamFromRos(*it, para_info);
    return success;
}

bool uint8ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::vector<uint8_t> value;
    bool success = uint8ArrayParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    } 
    return success;
}

bool uint8ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<uint8_t>& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) 
        {
            auto temp_value = param.as_integer_array();
            value.clear();
            for (const auto& v : temp_value) 
            {
                if (v >= std::numeric_limits<uint8_t>::min() && v <= std::numeric_limits<uint8_t>::max()) 
                {
                    value.push_back(static_cast<uint8_t>(v));
                } 
                else 
                {
                    LOG_ERROR("数组中存在超出uint8_t范围的值: %ld", v);
                    return false;
                }
            }
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT8_ARRAY).c_str());
            return true;
        } 
        else if (param.get_type() == rclcpp::ParameterType::PARAMETER_BYTE_ARRAY)
        {
            // 处理 PARAMETER_BYTE_ARRAY 类型
            auto temp_value = param.as_byte_array();
            value.clear();
            for (const auto& v : temp_value) 
            {
                // byte 类型本身就在 uint8_t 范围内，无需额外检查
                value.push_back(v);
            }
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT8_ARRAY).c_str());
            return true;
        }
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT8_ARRAY).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::UINT8_ARRAY).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool int16ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::vector<int16_t> value;
    bool success = int16ArrayParamFromRos(*it, para_info);
    return success;
}

bool int16ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::vector<int16_t> value;
    bool success = int16ArrayParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    } 
    return success;
}

bool int16ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<int16_t>& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) 
        {
            auto temp_value = param.as_integer_array();
            value.clear();
            for (const auto& v : temp_value) 
            {
                if (v >= std::numeric_limits<int16_t>::min() && v <= std::numeric_limits<int16_t>::max()) 
                {
                    value.push_back(static_cast<int16_t>(v));
                } 
                else 
                {
                    LOG_ERROR("数组中存在超出int16_t范围的值: %ld", v);
                    return false;
                }
            }
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::INT16_ARRAY).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::INT16_ARRAY).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::INT16_ARRAY).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool uint16ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name); 
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::vector<uint16_t> value;
    bool success = uint16ArrayParamFromRos(*it, para_info);
    return success;
}

bool uint16ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::vector<uint16_t> value;
    bool success = uint16ArrayParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    } 
    return success;
}

bool uint16ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<uint16_t>& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) 
        {
            auto temp_value = param.as_integer_array();
            value.clear();
            for (const auto& v : temp_value) 
            {
                if (v >= std::numeric_limits<uint16_t>::min() && v <= std::numeric_limits<uint16_t>::max())
                {
                    value.push_back(static_cast<uint16_t>(v));
                } 
                else 
                {
                    LOG_ERROR("数组中存在超出uint16_t范围的值: %ld", v);
                    return false;
                }
            }
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT16_ARRAY).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT16_ARRAY).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s",
            datahandler::getTypeString(datahandler::ParamType::UINT16_ARRAY).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool int32ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::vector<int32_t> value;
    bool success = int32ArrayParamFromRos(*it, para_info);
    return success;
}

bool int32ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::vector<int32_t> value;
    bool success = int32ArrayParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    } 
    return success;
}

bool int32ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<int32_t>& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) 
        {
            auto temp_value = param.as_integer_array();
            value.clear();
            for (const auto& v : temp_value) 
            {
                if (v >= std::numeric_limits<int32_t>::min() && v <= std::numeric_limits<int32_t>::max()) 
                {
                    value.push_back(static_cast<int32_t>(v));
                } 
                else 
                {
                    LOG_ERROR("数组中存在超出int32_t范围的值: %ld", v);
                    return false;
                }
            }
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::INT32_ARRAY).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::INT32_ARRAY).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s", 
            datahandler::getTypeString(datahandler::ParamType::INT32_ARRAY).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool uint32ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::vector<uint32_t> value;
    bool success = uint32ArrayParamFromRos(*it, para_info);
    return success;
}

bool uint32ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::vector<uint32_t> value;
    bool success = uint32ArrayParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    } 
    return success;
}

bool uint32ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<uint32_t>& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) 
        {
            auto temp_value = param.as_integer_array();
            value.clear();
            for (const auto& v : temp_value) 
            {
                if (v >= 0 && v <= std::numeric_limits<uint32_t>::max()) 
                { // 确保值为非负数
                    value.push_back(static_cast<uint32_t>(v));
                } 
                else 
                {
                    LOG_ERROR("数组中存在超出uint32_t范围的值: %ld", v);
                    return false;
                }
            }
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT32_ARRAY).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT32_ARRAY).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s", 
            datahandler::getTypeString(datahandler::ParamType::UINT32_ARRAY).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool int64ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::vector<int64_t> value;
    bool success = int64ArrayParamFromRos(*it, para_info);
    return success;
}

bool int64ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::vector<int64_t> value;
    bool success = int64ArrayParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    } 
    return success;
}

bool int64ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<int64_t>& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) 
        {
            value = param.as_integer_array();
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::INT64_ARRAY).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::INT64_ARRAY).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s", 
            datahandler::getTypeString(datahandler::ParamType::INT64_ARRAY).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool uint64ArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::vector<uint64_t> value;
    bool success = uint64ArrayParamFromRos(*it, para_info);
    return success;
}

bool uint64ArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::vector<uint64_t> value;
    bool success = uint64ArrayParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    } 
    return success;
}

bool uint64ArrayParamFromRos(const rclcpp::Parameter& param, std::vector<uint64_t>& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) 
        {
            auto temp_value = param.as_integer_array();
            value.clear();
            for (const auto& v : temp_value) 
            {
                if (v >= 0) 
                { // 确保值为非负数
                    value.push_back(static_cast<uint64_t>(v));
                } 
                else 
                {
                    LOG_ERROR("数组中存在负数，无法转换为uint64_t: %ld", v);
                    return false;
                }
            }
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT64_ARRAY).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::UINT64_ARRAY).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s", 
            datahandler::getTypeString(datahandler::ParamType::UINT64_ARRAY).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool floatArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::vector<float> value;
    bool success = floatArrayParamFromRos(*it, para_info);
    return success;
}

bool floatArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::vector<float> value;
    bool success = floatArrayParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    } 
    return success;
}

bool floatArrayParamFromRos(const rclcpp::Parameter& param, std::vector<float>& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) 
        {
            auto temp_value = param.as_double_array();
            value.clear();
            for (const auto& v : temp_value) 
            {
                if (v >= std::numeric_limits<float>::min() && v <= std::numeric_limits<float>::max()) 
                {
                    value.push_back(static_cast<float>(v));
                } 
                else 
                {
                    LOG_ERROR("数组中存在超出float范围的值: %lf", v);
                    return false;
                }
            }
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", 
                param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::FLOAT_ARRAY).c_str());
            return true;
        } else {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::FLOAT_ARRAY).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s", 
            datahandler::getTypeString(datahandler::ParamType::FLOAT_ARRAY).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool doubleArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name);
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::vector<double> value;
    bool success = doubleArrayParamFromRos(*it, para_info);
    return success;
}

bool doubleArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::vector<double> value;
    bool success = doubleArrayParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    } 
    return success;
}

bool doubleArrayParamFromRos(const rclcpp::Parameter& param, std::vector<double>& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) 
        {
            value = param.as_double_array();
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::DOUBLE_ARRAY).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::DOUBLE_ARRAY).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s", 
            datahandler::getTypeString(datahandler::ParamType::DOUBLE_ARRAY).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

bool stringArrayParamFromRos(const std::vector<rclcpp::Parameter>& ros_paras,
    const std::string& prefix_node, datahandler::ParamInfo& para_info)
{
    const std::string node_name = updateNodeName(prefix_node, para_info.name); 
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&node_name](const rclcpp::Parameter& param) {
            return param.get_name() == node_name;
        });
    if (it == ros_paras.end()) 
    {
        LOG_ERROR("参数 %s 不存在", node_name.c_str());
        return false;
    }
    std::vector<std::string> value;
    bool success = stringArrayParamFromRos(*it, para_info);
    return success;
}

bool stringArrayParamFromRos(const rclcpp::Parameter& param, datahandler::ParamInfo& para_info)
{
    std::vector<std::string> value;
    bool success = stringArrayParamFromRos(param, value);
    if (success) 
    {
        success = para_info.setValue(value);
    } 
    return success;
}

bool stringArrayParamFromRos(const rclcpp::Parameter& param, std::vector<std::string>& value)
{
    try 
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) 
        {
            value = param.as_string_array();
            std::string value_str = basmodule::get_list_string(value);
            LOG_DEBUG("参数: %s = %s  [类型: %s]", param.get_name().c_str(), value_str.c_str(), 
                datahandler::getTypeString(datahandler::ParamType::STRING_ARRAY).c_str());
            return true;
        } 
        else 
        {
            LOG_ERROR("参数 %s 的类型不匹配: 期望类型：%s，实际类型：%s", param.get_name().c_str(), 
                datahandler::getTypeString(datahandler::ParamType::STRING_ARRAY).c_str(), rosTypeName(param).c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换 %s 类型的参数：%s 时出错: %s", 
            datahandler::getTypeString(datahandler::ParamType::STRING_ARRAY).c_str(), param.get_name().c_str(), e.what());
        return false;
    }
}

// 为常用的vector<unsigned char>类型提供显式实例化
// 由于uint8_t在某些平台上被定义为unsigned char，需要显式实例化
template bool paramFromServer<std::vector<unsigned char>>(const rclcpp::SyncParametersClient::SharedPtr& client,
    const std::string& para_name, std::vector<unsigned char>& value, const std::string& param_prefix);

} // namespace basros
