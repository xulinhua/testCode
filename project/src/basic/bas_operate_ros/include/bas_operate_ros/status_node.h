/**
 * @file status_node.h
 * @brief 状态节点基类，提供模块状态发布功能
 * @author bas_control Team
 * @date 2026-03-31
 */

#ifndef BAS_OPERATE_ROS_STATUS_NODE_H
#define BAS_OPERATE_ROS_STATUS_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <string>
#include "bas_operate_ros/module_status.hpp"

namespace basros
{

/**
 * @brief 状态节点基类
 * 
 * 继承自 rclcpp::Node，提供模块状态发布功能。
 * 所有需要发布模块状态的节点可以继承此类。
 * 
 * @example
 * class MyNode : public bas_control::StatusNodeBase
 * {
 * public:
 *     MyNode(const rclcpp::NodeOptions& options)
 *         : StatusNodeBase("my_node", "my_module_name", options)
 *     {
 *         // 发布运行状态
 *         publishModuleStatus(basros::ModuleStatus::RUNNING, "节点初始化完成");
 *     }
 * };
 */
class StatusNodeBase : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param node_name 节点名称
     * @param module_name 模块名称（用于状态话题命名）
     * @param options 节点选项
     */
    StatusNodeBase(
        const std::string& node_name,
        const std::string& module_name,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    virtual ~StatusNodeBase();

    /**
     * @brief 初始化模块状态发布器
     * 
     * 按照统一的话题命名规范: /{module_name}/mdl_status_info
     */
    void initModuleStatusPublisher();

    /**
     * @brief 发布模块状态
     * @param status 模块状态
     * @param message 状态信息文本
     */
    void publishModuleStatus(basros::ModuleStatus status, const std::string& message);

    /**
     * @brief 获取当前模块状态
     * @return 当前模块状态
     */
    basros::ModuleStatus getCurrentStatus() const;

    /**
     * @brief 获取模块名称
     * @return 模块名称
     */
    std::string getModuleName() const;

protected:
    // 模块状态发布相关变量
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr module_status_pub_;  ///< 模块状态发布器
    std::string module_name_;                           ///< 本模块名称
    basros::ModuleStatus current_status_;               ///< 当前模块状态
    mutable std::mutex status_mutex_;                   ///< 状态互斥锁
};

}  // namespace bas_control

#endif  // BAS_OPERATE_ROS_STATUS_NODE_H
