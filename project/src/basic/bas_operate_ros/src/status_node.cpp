/**
 * @file status_node.cpp
 * @brief 状态节点基类实现
 * @author bas_operate_ros Team
 * @date 2026-03-31
 */

#include "bas_operate_ros/status_node.h"
#include <iostream>

namespace basros
{

StatusNodeBase::StatusNodeBase(
    const std::string& node_name,
    const std::string& module_name,
    const rclcpp::NodeOptions& options)
    : rclcpp::Node(node_name, options),
      module_name_(module_name),
      current_status_(basros::ModuleStatus::UNKNOWN)
{
    // 初始化模块状态发布器
    initModuleStatusPublisher();
}

StatusNodeBase::~StatusNodeBase()
{
    // 析构时发布停止状态
    if (module_status_pub_) {
        publishModuleStatus(basros::ModuleStatus::STOPPED, "节点已停止");
    }
}

void StatusNodeBase::initModuleStatusPublisher()
{
    // 话题名称格式: /{module_name}/mdl_status_info
    std::string topic_name = "/" + module_name_ + "/mdl_status_info";
    
    // 创建发布器，使用可靠QoS确保消息送达
    module_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    
    RCLCPP_INFO(this->get_logger(), "模块状态发布器初始化完成，话题: %s", topic_name.c_str());
}

void StatusNodeBase::publishModuleStatus(basros::ModuleStatus status, const std::string& message)
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    // 更新当前状态
    current_status_ = status;
    
    // 构建ModuleStatusInfo结构体
    basros::ModuleStatusInfo status_info(module_name_, -1, status, message);
    
    // 序列化为JSON字符串
    std::string json_str = basros::moduleStatusInfoToJson(status_info);
    
    // 创建并发布消息
    auto msg = std_msgs::msg::String();
    msg.data = json_str;
    module_status_pub_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), "发布模块状态: %s - %s", 
                basros::moduleStatusToString(status).c_str(), 
                message.c_str());
}

basros::ModuleStatus StatusNodeBase::getCurrentStatus() const
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_;
}

std::string StatusNodeBase::getModuleName() const
{
    return module_name_;
}

}  // namespace bas_control
