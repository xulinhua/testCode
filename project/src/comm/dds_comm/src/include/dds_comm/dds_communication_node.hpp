#ifndef DDS_COMMUNICATION__DDS_COMMUNICATION_NODE_HPP_
#define DDS_COMMUNICATION__DDS_COMMUNICATION_NODE_HPP_

#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include "dds_service.hpp"
#include "command_manager.hpp"
#include "command_types.hpp"

namespace dds_comm {

/**
 * @class DdsCommunicationNode
 * @brief DDS通信主节点类（服务端 - Jetson端）
 * 
 * 该类运行在Jetson主板上，作为DDS通信的服务端，接收来自x86主板的命令
 * 并管理本地项目的启动、停止和状态查询
 */
class DdsCommunicationNode {
public:
    /**
     * @brief 构造函数
     */
    DdsCommunicationNode();
    
    /**
     * @brief 析构函数
     */
    ~DdsCommunicationNode();

private:
    /**
     * @brief 初始化参数
     */
    void initializeParameters();
    
    /**
     * @brief 初始化DDS服务和命令管理器
     */
    void initializeServices();
    
    /**
     * @brief 注册项目命令处理器
     */
    void registerCommandHandlers();
    
    /**
     * @brief 注册项目状态处理器
     */
    void registerStatusHandlers();
    
    /**
     * @brief 命令消息回调函数
     * @param command 接收到的命令
     */
    void handleCommand(const CommandMessage& command);
    
    /**
     * @brief 状态消息回调函数
     * @param status 接收到的状态
     */
    void handleStatus(const StatusMessage& status);
    
    /**
     * @brief 手眼标定项目命令处理器
     * @param command 命令消息
     * @return bool 处理是否成功
     */
    bool handleHandEyeCalibCommand(const CommandMessage& command);
    
    /**
     * @brief 手眼标定项目状态查询处理器
     * @return StatusMessage 状态消息
     */
    StatusMessage queryHandEyeCalibStatus();
    
    /**
     * @brief 相机项目命令处理器
     * @param command 命令消息
     * @return bool 处理是否成功
     */
    bool handleCameraCommand(const CommandMessage& command);
    
    /**
     * @brief 相机项目状态查询处理器
     * @return StatusMessage 状态消息
     */
    StatusMessage queryCameraStatus();
    
    /**
     * @brief 点云转激光扫描项目命令处理器
     * @param command 命令消息
     * @return bool 处理是否成功
     */
    bool handlePcl2LaserScanCommand(const CommandMessage& command);
    
    /**
     * @brief 点云转激光扫描项目状态查询处理器
     * @return StatusMessage 状态消息
     */
    StatusMessage queryPcl2LaserScanStatus();
    
    /**
     * @brief 目标检测项目命令处理器
     * @param command 命令消息
     * @return bool 处理是否成功
     */
    bool handleYoloDetCommand(const CommandMessage& command);
    
    /**
     * @brief 目标检测项目状态查询处理器
     * @return StatusMessage 状态消息
     */
    StatusMessage queryYoloDetStatus();
    
    std::shared_ptr<DdsService> dds_service_;      ///< DDS通信服务
    std::shared_ptr<CommandManager> cmd_manager_; ///< 命令管理器
    
    // 参数
    std::string node_name_;
    int heartbeat_interval_;
    
    // 项目状态跟踪
    bool hand_eye_calib_running_;
    bool camera_running_;
    bool pcl2laserscan_running_;
    bool yolo_det_running_;
};

} // namespace dds_comm

#endif // DDS_COMMUNICATION__DDS_COMMUNICATION_NODE_HPP_