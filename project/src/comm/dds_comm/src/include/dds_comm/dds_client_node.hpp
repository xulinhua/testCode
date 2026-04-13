#ifndef DDS_COMMUNICATION__DDS_CLIENT_NODE_HPP_
#define DDS_COMMUNICATION__DDS_CLIENT_NODE_HPP_

#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include "dds_service.hpp"
#include "command_manager.hpp"
#include "command_types.hpp"

namespace dds_comm {

/**
 * @class DdsClientNode
 * @brief DDS通信客户端节点类（x86端）
 * 
 * 该类运行在x86主板上，作为DDS通信的客户端，向Jetson主板发送命令
 * 并接收状态反馈
 */
class DdsClientNode {
public:
    /**
     * @brief 构造函数
     */
    DdsClientNode();
    
    /**
     * @brief 析构函数
     */
    ~DdsClientNode();

private:
    /**
     * @brief 初始化参数
     */
    void initializeParameters();
    
    /**
     * @brief 初始化DDS服务
     */
    void initializeServices();
    
    /**
     * @brief 状态消息回调函数
     * @param status 接收到的状态
     */
    void handleStatus(const StatusMessage& status);
    
    /**
     * @brief 发送启动命令
     * @param project_type 项目类型
     */
    void sendStartCommand(ProjectType project_type);
    
    /**
     * @brief 发送停止命令
     * @param project_type 项目类型
     */
    void sendStopCommand(ProjectType project_type);
    
    /**
     * @brief 发送状态查询命令
     * @param project_type 项目类型
     */
    void sendStatusQuery(ProjectType project_type);
    
    /**
     * @brief 定时器回调函数（用于示例命令发送）
     */
    void timerCallback();
    
    std::shared_ptr<DdsService> dds_service_;  ///< DDS通信服务
    
    // 参数
    std::string node_name_;
    int command_interval_;
    
    size_t command_counter_;              ///< 命令计数器
    
    // 定时器相关（独立于ROS2）
    std::function<void()> timer_callback_;
    std::chrono::milliseconds timer_interval_;
};

} // namespace dds_comm

#endif // DDS_COMMUNICATION__DDS_CLIENT_NODE_HPP_