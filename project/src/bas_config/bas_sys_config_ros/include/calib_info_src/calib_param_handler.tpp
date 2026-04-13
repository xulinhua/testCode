/**
 * @file calib_param_handler.tpp
 * @brief 统一标定参数处理器模板实现
 */

#include "bas_sys_config_ros/calib_param_handler.h"
#include "bas_sys_config_ros/calib_info_server.h" 
#include "log_system/log_macros.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <string>
#include <stdexcept>

namespace RosComm {

template<typename DataType>
CalibParamHandler<DataType>::CalibParamHandler(rclcpp::Node::SharedPtr node)
    : node_(node) 
{
    // 注册参数回调处理器
    param_callback_handle_ = node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            return this->onParameterChange(parameters);
        });
}

template<typename DataType>
CalibParamHandler<DataType>::~CalibParamHandler() 
{
    // 析构函数
}

template<typename DataType>
void CalibParamHandler<DataType>::registerCalibParamCallback(std::function<void(const DataType&)> callback) 
{
    callback_ = callback;
}

template<typename DataType>
bool CalibParamHandler<DataType>::getCalibDatFromServer(const std::string& param_prefix, DataType& calib_data) 
{
    // 这里需要特化实现，因为不同类型的数据读取方式不同
    // 对于CamCalibInfoList类型
    std::string sys_config_node_name = "sys_config_ros_node"; // 参数服务器节点名称
    if constexpr (std::is_same_v<DataType, handeyecalib::CamCalibInfoList>) 
    {
        // 创建参数客户端
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_, sys_config_node_name);
        return RosComm::getCamCalibInfoListFromServer(
            parameters_client, 
            param_prefix, 
            calib_data);
    }
    // 对于ArmCalibInfo类型
    else if constexpr (std::is_same_v<DataType, handeyecalib::ArmCalibInfo>) 
    {
        // 对于ArmCalibInfo，param_prefix应为arm_id的字符串形式
        // 尝试将param_prefix转换为arm_id
        try 
        {
            uint8_t arm_id = static_cast<uint8_t>(std::stoi(param_prefix));
            // 创建参数客户端
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_, sys_config_node_name);
            return RosComm::getArmCalibInfoFromServer(parameters_client, arm_id, calib_data);
        } catch (const std::exception& e) {
            LOG_ERROR("转换参数前缀为arm_id失败: %s, param_prefix: %s", e.what(), param_prefix.c_str());
            return false;
        }
    }
    return false;
}

template<typename DataType>
const DataType& CalibParamHandler<DataType>::getLatestCalibData() const 
{
    return latest_data_;
}

template<typename DataType>
rcl_interfaces::msg::SetParametersResult CalibParamHandler<DataType>::onParameterChange(
    const std::vector<rclcpp::Parameter>& parameters) 
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    // 检查是否有标定参数发生变化
    bool calib_params_changed = false;
    std::string param_prefix = "calib";  // 根据实际参数前缀调整
    for (const auto& param : parameters) 
    {
        if (param.get_name().find(param_prefix) != std::string::npos) 
        {
            calib_params_changed = true;
            LOG_DEBUG("检测到标定参数变化: %s", param.get_name().c_str());
            break;
        }
    }
    
    // 如果标定参数发生变化，读取完整的标定数据并调用回调
    if (calib_params_changed && callback_) 
    {
        LOG_INFO("标定参数发生变化，正在从参数服务器读取完整数据");
        DataType calib_data;
        if (getCalibDatFromServer(param_prefix, calib_data)) 
        {
            LOG_INFO("成功从参数服务器读取标定数据");
            latest_data_ = calib_data;
            callback_(calib_data);
        } 
        else 
        {
            LOG_WARN("无法从参数服务器读取完整的标定数据");
            result.successful = false;
            result.reason = "Failed to read calibration data from parameter server";
        }
    } 
    return result;
}

// 显式实例化模板类
template class CalibParamHandler<handeyecalib::CamCalibInfoList>;
template class CalibParamHandler<handeyecalib::ArmCalibInfo>;

}  // namespace RosComm