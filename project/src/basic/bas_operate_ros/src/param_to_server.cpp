#include "bas_operate_ros/param_to_server.hpp"
#include "log_system/log_macros.hpp"
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

namespace basros {

bool paraInfoToServer(rclcpp::Node* node, const rclcpp::Parameter& ros_para)
{
    try 
    {
        if (node->has_parameter(ros_para.get_name())) // 检查参数是否已存在，如果不存在则需要先声明
        {
            auto result = node->set_parameter(ros_para);// 参数已存在，直接设置
            if (!result.successful) 
            {
                LOG_WARN("设置参数失败: %s, 原因: %s", ros_para.get_name().c_str(), result.reason.c_str());
                return false;
            }
        } 
        else 
        {
            node->declare_parameter(ros_para.get_name(), ros_para.get_parameter_value());// 参数不存在，需要先声明再设置
            auto result = node->set_parameter(ros_para);// 重新设置参数值以确保值被正确应用
            if (!result.successful) 
            {
                LOG_WARN("设置参数失败: %s, 原因: %s", ros_para.get_name().c_str(), result.reason.c_str());
                return false;
            }
        }
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("设置参数时发生异常: %s, 参数: %s", e.what(), ros_para.get_name().c_str());
        return false;
    }
}

bool paraInfoToServer(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& ros_paras)
{
    try 
    {
        int declared_count = 0;
        std::vector<rclcpp::Parameter> params_to_set;// 对于原子性操作，我们需要先确保所有参数都被声明
        for (const auto& param : ros_paras) 
        {
            if (!node->has_parameter(param.get_name())) 
            {
                node->declare_parameter(param.get_name(), param.get_parameter_value());// 如果参数未声明，先声明它
                declared_count++;
                LOG_DEBUG("声明参数: %s = %s", param.get_name().c_str(), param.value_to_string().c_str());
            }
            else {
                LOG_DEBUG("参数已存在，跳过声明: %s = %s", param.get_name().c_str(), param.value_to_string().c_str());
            }
            params_to_set.push_back(param);// 将参数添加到待设置列表
        }
        LOG_DEBUG("总共 %d 个参数声明完成，新声明了 %d 个参数，开始设置参数值", ros_paras.size(), declared_count);
        auto results = node->set_parameters(params_to_set);// 原子性设置所有参数
        bool all_success = true;// 检查是否有任何参数设置失败
        for (size_t i = 0; i < results.size(); ++i) 
        {
            if (!results[i].successful) 
            {
                LOG_WARN("原子性设置参数失败: %s, 原因: %s", params_to_set[i].get_name().c_str(), results[i].reason.c_str());
                all_success = false;
            }
        }
        if (all_success) {
            LOG_DEBUG("声明所有参数并原子性设置参数到参数服务器成功！");
        }
        return all_success;
    } catch (const std::exception& e) {
        LOG_ERROR("原子性设置参数时发生异常: %s", e.what());
        return false;
    }
}

/**
 * @brief 通用参数转换函数，将参数信息转换为rclcpp::Parameter并记录日志
 * @param param_info bas_operate中的参数信息
 * @param prefix_node 参数前缀节点名称
 * @param ros_para 存储转换后的ROS参数
 * @return 是否成功转换参数
 */
bool paraInfoToRos(const datahandler::ParamInfo& param_info, const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    try 
    {
        switch (param_info.type) 
        {
            case datahandler::ParamType::BOOL: 
                boolParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::INT8:
                int8ParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::UINT8:
                uint8ParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::INT16: 
                int16ParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::UINT16: 
                uint16ParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::INT32: 
                int32ParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::UINT32: 
                uint32ParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::INT64: 
                int64ParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::UINT64: 
                uint64ParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::FLOAT: 
                floatParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::DOUBLE: 
                doubleParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::STRING: 
                stringParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::BOOL_ARRAY: 
                boolArrayParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::INT8_ARRAY: 
                int8ArrayParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::UINT8_ARRAY: 
                uint8ArrayParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::INT16_ARRAY: 
                int16ArrayParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::UINT16_ARRAY: 
                uint16ArrayParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::INT32_ARRAY: 
                int32ArrayParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::UINT32_ARRAY: 
                uint32ArrayParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::INT64_ARRAY: 
                int64ArrayParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::UINT64_ARRAY: 
                uint64ArrayParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::FLOAT_ARRAY: 
                floatArrayParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::DOUBLE_ARRAY: 
                doubleArrayParamToRos(param_info, prefix_node, ros_para);
                return true;
            case datahandler::ParamType::STRING_ARRAY:
                stringArrayParamToRos(param_info, prefix_node, ros_para);
                return true;
            default:
                LOG_WARN("未处理的参数类型: %s, 参数名称: %s", param_info.getTypeString().c_str(), param_info.name.c_str());
                return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数: %s", e.what(), updateNodeName(prefix_node, param_info.name).c_str());
        return false;
    }
}

/**
 * @brief 通用参数转换函数，将参数信息向量转换为ROS Parameter向量
 * @param param_infos bas_operate中的参数信息向量
 * @param prefix_node 参数前缀节点名称
 * @param ros_paras 存储转换后的ROS参数向量
 * @return 是否成功转换所有参数
 */
bool paraInfoToRos(const std::vector<datahandler::ParamInfo>& param_infos, const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    try 
    {
        ros_paras.clear();
        ros_paras.reserve(param_infos.size());
        for (const auto& it : param_infos) 
        {
            rclcpp::Parameter ros_para;
            if (paraInfoToRos(it, prefix_node, ros_para)) 
            {
                ros_paras.push_back(ros_para);
            }
            else 
            {
                LOG_ERROR("参数转换失败: %s", updateNodeName(prefix_node, it.name).c_str());
                return false;
            }
        }
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("批量转换参数时发生异常: %s", e.what());
        return false;
    }
}

/**
 * @brief 更新或添加参数到ros_paras
 * @param ros_paras ROS参数列表
 * @param param 要添加的参数
 */
void updateOrAddToRos(std::vector<rclcpp::Parameter>& ros_paras, const rclcpp::Parameter& param)
{
    // 查找是否存在同名参数
    auto it = std::find_if(ros_paras.begin(), ros_paras.end(),
        [&param](const rclcpp::Parameter& p) {
            return p.get_name() == param.get_name();
        });

    if (it != ros_paras.end()){
        *it = param;// 存在则原地替换（避免移动后续元素）
    }else{
        ros_paras.push_back(param);// 不存在则添加
    }
}

void boolParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try
    {
        if (param_info.ptr == nullptr)
        {
            bool value = std::any_cast<bool>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, value));
        }
        else
        {
            bool* ptr = static_cast<bool*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, *ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void boolParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr)
        {
            bool value = std::any_cast<bool>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, value);
        }
        else 
        {
            bool* ptr = static_cast<bool*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, *ptr);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int8ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try
    {
        if (param_info.ptr == nullptr)
        {
            int8_t value = std::any_cast<int8_t>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, static_cast<int>(value)));
        }
        else
        {
            int8_t* ptr = static_cast<int8_t*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, static_cast<int>(*ptr)));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int8ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            int8_t value = std::any_cast<int8_t>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, static_cast<int>(value));
        }
        else 
        {
            int8_t* ptr = static_cast<int8_t*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, static_cast<int>(*ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint8ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try
    {
        if (param_info.ptr == nullptr)
        {
            uint8_t value = std::any_cast<uint8_t>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, static_cast<int>(value)));
        }
        else
        {
            uint8_t* ptr = static_cast<uint8_t*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, static_cast<int>(*ptr)));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint8ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {    
        if (param_info.ptr == nullptr) 
        {
            uint8_t value = std::any_cast<uint8_t>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, static_cast<int>(value));
        }
        else 
        {
            uint8_t* ptr = static_cast<uint8_t*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, static_cast<int>(*ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int16ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try
    {
        if (param_info.ptr == nullptr)
        {
            int16_t value = std::any_cast<int16_t>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, static_cast<int>(value)));
        }
        else
        {
            int16_t* ptr = static_cast<int16_t*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, static_cast<int>(*ptr)));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int16ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            int16_t value = std::any_cast<int16_t>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, static_cast<int>(value));
        }
        else 
        {
            int16_t* ptr = static_cast<int16_t*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, static_cast<int>(*ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint16ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try
    {
        if (param_info.ptr == nullptr)
        {
            uint16_t value = std::any_cast<uint16_t>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, static_cast<int>(value)));
        }
        else
        {
            uint16_t* ptr = static_cast<uint16_t*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, static_cast<int>(*ptr)));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint16ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            uint16_t value = std::any_cast<uint16_t>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, static_cast<int>(value));
        }
        else 
        {
            uint16_t* ptr = static_cast<uint16_t*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, static_cast<int>(*ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int32ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try
    {
        if (param_info.ptr == nullptr) 
        {
            int32_t value = std::any_cast<int32_t>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, value));
        }
        else 
        {
            int32_t* ptr = static_cast<int32_t*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, *ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int32ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try
    {
        if (param_info.ptr == nullptr) 
        {
            int32_t value = std::any_cast<int32_t>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, value);
        }
        else 
        {
            int32_t* ptr = static_cast<int32_t*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, *ptr);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint32ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try
    {
        if (param_info.ptr == nullptr)
        {
            uint32_t value = std::any_cast<uint32_t>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, static_cast<int>(value)));
        }
        else
        {
            uint32_t* ptr = static_cast<uint32_t*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, static_cast<int>(*ptr)));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint32ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr)
        {
            uint32_t value = std::any_cast<uint32_t>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, static_cast<int>(value));
        }
        else
        {
            uint32_t* ptr = static_cast<uint32_t*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, static_cast<int>(*ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int64ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            int64_t value = std::any_cast<int64_t>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, value));
        }
        else 
        {
            int64_t* ptr = static_cast<int64_t*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, *ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int64ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            int64_t value = std::any_cast<int64_t>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, value);
        }
        else 
        {
            int64_t* ptr = static_cast<int64_t*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, *ptr);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint64ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try
    {
        if (param_info.ptr == nullptr)
        {
            uint64_t value = std::any_cast<uint64_t>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, static_cast<int64_t>(value)));
        }
        else
        {
            uint64_t* ptr = static_cast<uint64_t*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, static_cast<int64_t>(*ptr)));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint64ParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            uint64_t value = std::any_cast<uint64_t>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, static_cast<int64_t>(value));
        }
        else 
        {
            uint64_t* ptr = static_cast<uint64_t*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, static_cast<int64_t>(*ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void floatParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            float value = std::any_cast<float>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, value));
        }
        else 
        {
            float* ptr = static_cast<float*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, *ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void floatParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            float value = std::any_cast<float>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, value);
        }
        else 
        {
            float* ptr = static_cast<float*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, *ptr);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void doubleParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            double value = std::any_cast<double>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, value));
        }
        else 
        {
            double* ptr = static_cast<double*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, *ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void doubleParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            double value = std::any_cast<double>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, value);
        }
        else 
        {
            double* ptr = static_cast<double*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, *ptr);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void stringParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::string value = std::any_cast<std::string>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, value));
        }
        else 
        {
            std::string* ptr = static_cast<std::string*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, *ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void stringParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::string value = std::any_cast<std::string>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, value);
        }
        else 
        {
            std::string* ptr = static_cast<std::string*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, *ptr);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void boolArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<bool> value = std::any_cast<std::vector<bool>>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, value));
        }
        else 
        {
            std::vector<bool>* ptr = static_cast<std::vector<bool>*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, *ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void boolArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<bool> value = std::any_cast<std::vector<bool>>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, value);
        }
        else 
        {
            std::vector<bool>* ptr = static_cast<std::vector<bool>*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, *ptr);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int8ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<int8_t> value = std::any_cast<std::vector<int8_t>>(param_info.value);
            // 将int8_t数组转换为uint8_t数组以兼容ROS参数服务器
            std::vector<uint8_t> uint8_array;
            std::transform(value.begin(), value.end(), std::back_inserter(uint8_array), 
                            [](int8_t val) { return static_cast<uint8_t>(val); });
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, uint8_array));
        }
        else 
        {
            std::vector<int8_t>* ptr = static_cast<std::vector<int8_t>*>(param_info.ptr);
            // 将int8_t数组转换为uint8_t数组以兼容ROS参数服务器
            std::vector<uint8_t> uint8_array;
            std::transform(ptr->begin(), ptr->end(), std::back_inserter(uint8_array), 
                            [](int8_t val) { return static_cast<uint8_t>(val); });
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, uint8_array));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int8ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<int8_t> value = std::any_cast<std::vector<int8_t>>(param_info.value);
            // 将int8_t数组转换为uint8_t数组以兼容ROS参数服务器
            std::vector<uint8_t> uint8_array;
            std::transform(value.begin(), value.end(), std::back_inserter(uint8_array), 
                            [](int8_t val) { return static_cast<uint8_t>(val); });
            ros_para = rclcpp::Parameter(node_name, uint8_array);
        }
        else 
        {
            std::vector<int8_t>* ptr = static_cast<std::vector<int8_t>*>(param_info.ptr);
            // 将int8_t数组转换为uint8_t数组以兼容ROS参数服务器
            std::vector<uint8_t> uint8_array;
            std::transform(ptr->begin(), ptr->end(), std::back_inserter(uint8_array), 
                            [](int8_t val) { return static_cast<uint8_t>(val); });
            ros_para = rclcpp::Parameter(node_name, uint8_array);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint8ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<uint8_t> value = std::any_cast<std::vector<uint8_t>>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, value));
        }
        else 
        {
            std::vector<uint8_t>* ptr = static_cast<std::vector<uint8_t>*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, *ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint8ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<uint8_t> value = std::any_cast<std::vector<uint8_t>>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, value);
        }
        else 
        {
            std::vector<uint8_t>* ptr = static_cast<std::vector<uint8_t>*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, *ptr);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int16ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<int16_t> value = std::any_cast<std::vector<int16_t>>(param_info.value);
            // 将int16_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(value.begin(), value.end(), std::back_inserter(int_array), 
                            [](int16_t val) { return static_cast<int>(val); });
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, int_array));
        }
        else 
        {
            std::vector<int16_t>* ptr = static_cast<std::vector<int16_t>*>(param_info.ptr);
            // 将int16_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(ptr->begin(), ptr->end(), std::back_inserter(int_array), 
                            [](int16_t val) { return static_cast<int>(val); });
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, int_array));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int16ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<int16_t> value = std::any_cast<std::vector<int16_t>>(param_info.value);
            // 将int16_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(value.begin(), value.end(), std::back_inserter(int_array), 
                            [](int16_t val) { return static_cast<int>(val); });
            ros_para = rclcpp::Parameter(node_name, int_array);
        }
        else 
        {
            std::vector<int16_t>* ptr = static_cast<std::vector<int16_t>*>(param_info.ptr);
            // 将int16_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(ptr->begin(), ptr->end(), std::back_inserter(int_array), 
                            [](int16_t val) { return static_cast<int>(val); });
            ros_para = rclcpp::Parameter(node_name, int_array);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint16ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<uint16_t> value = std::any_cast<std::vector<uint16_t>>(param_info.value);
            // 将uint16_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(value.begin(), value.end(), std::back_inserter(int_array), 
                            [](uint16_t val) { return static_cast<int>(val); });
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, int_array));
        }
        else 
        {
            std::vector<uint16_t>* ptr = static_cast<std::vector<uint16_t>*>(param_info.ptr);
            // 将uint16_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(ptr->begin(), ptr->end(), std::back_inserter(int_array), 
                            [](uint16_t val) { return static_cast<int>(val); });
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, int_array));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint16ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<uint16_t> value = std::any_cast<std::vector<uint16_t>>(param_info.value);
            // 将uint16_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(value.begin(), value.end(), std::back_inserter(int_array), 
                            [](uint16_t val) { return static_cast<int>(val); });
            ros_para = rclcpp::Parameter(node_name, int_array);
        }
        else 
        {
            std::vector<uint16_t>* ptr = static_cast<std::vector<uint16_t>*>(param_info.ptr);
            // 将uint16_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(ptr->begin(), ptr->end(), std::back_inserter(int_array), 
                            [](uint16_t val) { return static_cast<int>(val); });
            ros_para = rclcpp::Parameter(node_name, int_array);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int32ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<int32_t> value = std::any_cast<std::vector<int32_t>>(param_info.value);
            // 将int32_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(value.begin(), value.end(), std::back_inserter(int_array), 
                            [](int32_t val) { return static_cast<int>(val); });
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, int_array));
        }
        else 
        {
            std::vector<int32_t>* ptr = static_cast<std::vector<int32_t>*>(param_info.ptr);
            // 将int32_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(ptr->begin(), ptr->end(), std::back_inserter(int_array), 
                            [](int32_t val) { return static_cast<int>(val); });
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, int_array));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int32ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<int32_t> value = std::any_cast<std::vector<int32_t>>(param_info.value);
            // 将int32_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(value.begin(), value.end(), std::back_inserter(int_array), 
                            [](int32_t val) { return static_cast<int>(val); });
            ros_para = rclcpp::Parameter(node_name, int_array);
        }
        else 
        {
            std::vector<int32_t>* ptr = static_cast<std::vector<int32_t>*>(param_info.ptr);
            // 将int32_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(ptr->begin(), ptr->end(), std::back_inserter(int_array), 
                            [](int32_t val) { return static_cast<int>(val); });
            ros_para = rclcpp::Parameter(node_name, int_array);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint32ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<uint32_t> value = std::any_cast<std::vector<uint32_t>>(param_info.value);
            // 将uint32_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(value.begin(), value.end(), std::back_inserter(int_array), 
                            [](uint32_t val) { return static_cast<int>(val); });
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, int_array));
        }
        else 
        {
            std::vector<uint32_t>* ptr = static_cast<std::vector<uint32_t>*>(param_info.ptr);
            // 将uint32_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(ptr->begin(), ptr->end(), std::back_inserter(int_array), 
                            [](uint32_t val) { return static_cast<int>(val); });
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, int_array));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint32ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<uint32_t> value = std::any_cast<std::vector<uint32_t>>(param_info.value);
            // 将uint32_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(value.begin(), value.end(), std::back_inserter(int_array), 
                            [](uint32_t val) { return static_cast<int>(val); });
            ros_para = rclcpp::Parameter(node_name, int_array);
        }
        else 
        {
            std::vector<uint32_t>* ptr = static_cast<std::vector<uint32_t>*>(param_info.ptr);
            // 将uint32_t数组转换为int数组以兼容ROS参数服务器
            std::vector<int> int_array;
            std::transform(ptr->begin(), ptr->end(), std::back_inserter(int_array), 
                            [](uint32_t val) { return static_cast<int>(val); });
            ros_para = rclcpp::Parameter(node_name, int_array);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int64ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<int64_t> value = std::any_cast<std::vector<int64_t>>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, value));
        }
        else 
        {
            std::vector<int64_t>* ptr = static_cast<std::vector<int64_t>*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, *ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void int64ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<int64_t> value = std::any_cast<std::vector<int64_t>>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, value);
        }
        else 
        {
            std::vector<int64_t>* ptr = static_cast<std::vector<int64_t>*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, *ptr);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint64ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<uint64_t> value = std::any_cast<std::vector<uint64_t>>(param_info.value);
            // 将uint64_t数组转换为int64_t数组以兼容ROS参数服务器
            std::vector<int64_t> int64_array;
            std::transform(value.begin(), value.end(), std::back_inserter(int64_array), 
                            [](uint64_t val) { return static_cast<int64_t>(val); });
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, int64_array));
        }
        else 
        {
            std::vector<uint64_t>* ptr = static_cast<std::vector<uint64_t>*>(param_info.ptr);
            // 将uint64_t数组转换为int64_t数组以兼容ROS参数服务器
            std::vector<int64_t> int64_array;
            std::transform(ptr->begin(), ptr->end(), std::back_inserter(int64_array), 
                        [](uint64_t val) { return static_cast<int64_t>(val); });
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, int64_array));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void uint64ArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<uint64_t> value = std::any_cast<std::vector<uint64_t>>(param_info.value);
            // 将uint64_t数组转换为int64_t数组以兼容ROS参数服务器
            std::vector<int64_t> int64_array;
            std::transform(value.begin(), value.end(), std::back_inserter(int64_array), 
                            [](uint64_t val) { return static_cast<int64_t>(val); });
            ros_para = rclcpp::Parameter(node_name, int64_array);
        }
        else 
        {
            std::vector<uint64_t>* ptr = static_cast<std::vector<uint64_t>*>(param_info.ptr);
            // 将uint64_t数组转换为int64_t数组以兼容ROS参数服务器
            std::vector<int64_t> int64_array;
            std::transform(ptr->begin(), ptr->end(), std::back_inserter(int64_array), 
                        [](uint64_t val) { return static_cast<int64_t>(val); });
            ros_para = rclcpp::Parameter(node_name, int64_array);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void floatArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<float> value = std::any_cast<std::vector<float>>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, value));
        }
        else 
        {
            std::vector<float>* ptr = static_cast<std::vector<float>*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, *ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void floatArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<float> value = std::any_cast<std::vector<float>>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, value);
        }
        else 
        {
            std::vector<float>* ptr = static_cast<std::vector<float>*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, *ptr);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void doubleArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<double> value = std::any_cast<std::vector<double>>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, value));
        }
        else 
        {
            std::vector<double>* ptr = static_cast<std::vector<double>*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, *ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void doubleArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<double> value = std::any_cast<std::vector<double>>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, value);
        }
        else 
        {
            std::vector<double>* ptr = static_cast<std::vector<double>*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, *ptr);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void stringArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, std::vector<rclcpp::Parameter>& ros_paras)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<std::string> value = std::any_cast<std::vector<std::string>>(param_info.value);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, value));
        }
        else 
        {
            std::vector<std::string>* ptr = static_cast<std::vector<std::string>*>(param_info.ptr);
            updateOrAddToRos(ros_paras, rclcpp::Parameter(node_name, *ptr));
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

void stringArrayParamToRos(const datahandler::ParamInfo& param_info,
    const std::string& prefix_node, rclcpp::Parameter& ros_para)
{
    const std::string node_name = updateNodeName(prefix_node, param_info.name);
    try 
    {
        if (param_info.ptr == nullptr) 
        {
            std::vector<std::string> value = std::any_cast<std::vector<std::string>>(param_info.value);
            ros_para = rclcpp::Parameter(node_name, value);
        }
        else 
        {
            std::vector<std::string>* ptr = static_cast<std::vector<std::string>*>(param_info.ptr);
            ros_para = rclcpp::Parameter(node_name, *ptr);
        }
    } catch (const std::bad_any_cast& e) {
        LOG_ERROR("参数类型转换失败: %s, 参数名称: %s", e.what(), node_name.c_str());
    } catch (const std::exception& e) {
        LOG_ERROR("转换参数时发生异常: %s, 参数名称: %s", e.what(), node_name.c_str());
    }
}

} // namespace basros