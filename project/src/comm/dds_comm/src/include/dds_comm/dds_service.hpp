/**
 * @file dds_service.hpp
 * @brief DDS通信服务核心头文件
 * 
 * @section 功能作用
 * 本文件定义了DDS通信服务的核心类DdsService，提供以下功能：
 * 1. 命令和状态消息的发布/订阅管理
 * 2. 项目生命周期管理（启动、停止、重启、状态查询）
 * 3. 错误处理和自动恢复机制
 * 4. 服务统计信息和健康检查
 * 5. 批量命令处理和发布间隔控制
 * 6. 数据发布管理
 * 7. 线程安全的服务状态管理
 * 
 * @section 项目耦合关系
 * - 强依赖：command_types.hpp - 命令和状态消息类型定义
 * - 强依赖：project_lifecycle_manager.hpp - 项目生命周期管理
 * - 强依赖：data_publisher.hpp - 数据发布功能
 * - 弱依赖：ROS2框架 - 节点管理和消息通信
 * - 无依赖：手眼标定项目 - 通过接口抽象实现完全解耦
 * - 耦合度：中等 - 内部模块间有明确依赖，但对外接口清晰
 * 
 * @section 设计模式
 * - 观察者模式：通过回调函数处理命令和状态消息
 * - 状态模式：管理服务的不同状态（初始化、运行、停止等）
 * - 工厂模式：创建和管理不同类型的命令处理器
 * - 策略模式：支持不同的命令处理策略
 */

#ifndef DDS_COMMUNICATION__DDS_SERVICE_HPP_
#define DDS_COMMUNICATION__DDS_SERVICE_HPP_

#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <mutex>
#include <functional>
#include <vector>
#include "command_types.hpp"
#include "command_manager.hpp"
#include "project_manager.hpp"
#include "data_publisher.hpp"
#include "log_system/log_macros.hpp"

namespace dds_comm {

/**
 * @class DdsService
 * @brief DDS通信服务
 * 
 * 功能概述：
 * 1. 增强的命令和状态管理
 * 2. 项目生命周期管理集成
 * 3. 错误处理和自动恢复
 * 4. 统计信息和健康检查
 * 5. 批量命令处理
 * 6. 数据发布管理
 * 
 * 服务状态：
 * - INITIALIZING: 初始化中
 * - RUNNING: 运行中
 * - STOPPING: 停止中
 * - STOPPED: 已停止
 * - ERROR: 错误状态
 * - RECOVERING: 恢复中
 */

class DdsService {
public:
    /**
     * @brief 服务状态枚举
     */
    enum class ServiceState {
        INITIALIZING,   ///< 初始化中
        RUNNING,        ///< 运行中
        STOPPING,       ///< 停止中
        STOPPED,        ///< 已停止
        ERROR,          ///< 错误状态
        RECOVERING      ///< 恢复中
    };
    
    /**
     * @brief 服务统计信息结构
     */
    struct ServiceStats {
        std::chrono::system_clock::time_point start_time;        ///< 启动时间
        uint64_t total_commands_received;                        ///< 接收命令总数
        uint64_t total_commands_sent;                            ///< 发送命令总数
        uint64_t total_status_sent;                             ///< 发送状态总数
        uint64_t command_processing_errors;                     ///< 命令处理错误数
        uint64_t connection_errors;                             ///< 连接错误数
        double average_command_rate;                             ///< 平均命令速率
        ServiceState current_state;                             ///< 当前状态
    };
    
    /**
     * @brief 命令回调函数类型
     */
    using CommandCallback = std::function<void(const CommandMessage&)>;
    
    /**
     * @brief 状态回调函数类型
     */
    using StatusCallback = std::function<void(const StatusMessage&)>;
    
    /**
     * @brief 错误回调函数类型
     */
    using ErrorCallback = std::function<void(const std::string&)>;
    
    /**
     * @brief 构造函数
     * @param is_server 是否为服务端模式
     * @param config_file 配置文件路径（可选）
     */
    DdsService(bool is_server = true, 
                      const std::string& config_file = "");
    
    /**
     * @brief 析构函数
     */
    ~DdsService();
    
    /**
     * @brief 初始化服务
     * @return bool 初始化是否成功
     */
    bool initialize();
    
    /**
     * @brief 发送命令
     * @param command 命令消息
     * @param max_retries 最大重试次数
     * @return bool 发送是否成功
     */
    bool sendCommand(const CommandMessage& command, int max_retries = 3);
    
    /**
     * @brief 发送状态
     * @param status 状态消息
     * @return bool 发送是否成功
     */
    bool sendStatus(const StatusMessage& status);
    
    /**
     * @brief 发送批量命令
     * @param commands 命令列表
     * @param interval 命令间隔时间
     * @return bool 发送是否成功
     */
    bool sendBatchCommands(const std::vector<CommandMessage>& commands,
                          std::chrono::milliseconds interval = std::chrono::milliseconds(100));
    
    /**
     * @brief 注册项目生命周期
     * @param project_type 项目类型
     * @param start_handler 启动处理器
     * @param stop_handler 停止处理器
     * @param status_handler 状态处理器
     * @return bool 注册是否成功
     */
    bool registerProjectLifecycle(ProjectType project_type,
                                 std::function<bool()> start_handler,
                                 std::function<bool()> stop_handler,
                                 std::function<StatusMessage()> status_handler);
    
    /**
     * @brief 创建数据发布者
     * @param data_type 数据类型
     * @param config 发布配置
     * @return bool 创建是否成功
     */
    bool createDataPublisher(const std::string& data_type, 
                            const DataPublisher::PublishConfig& config);
    
    /**
     * @brief 发布数据
     * @param data_type 数据类型
     * @param data 数据内容
     * @return bool 发布是否成功
     */
    bool publishData(const std::string& data_type, const std::string& data);
    
    /**
     * @brief 获取服务状态
     * @return ServiceState 服务状态
     */
    ServiceState getServiceState() const;
    
    /**
     * @brief 获取服务统计信息
     * @return ServiceStats 服务统计信息
     */
    ServiceStats getServiceStats() const;
    
    /**
     * @brief 健康检查
     * @return bool 健康状态
     */
    bool healthCheck();
    
    /**
     * @brief 恢复服务
     * @return bool 恢复是否成功
     */
    bool recoverService();
    
    /**
     * @brief 关闭服务
     */
    void shutdown();
    
    /**
     * @brief 注册命令回调
     * @param callback 回调函数
     */
    void registerCommandCallback(CommandCallback callback);
    
    /**
     * @brief 注册状态回调
     * @param callback 回调函数
     */
    void registerStatusCallback(StatusCallback callback);
    
    /**
     * @brief 注册错误回调
     * @param callback 回调函数
     */
    void registerErrorCallback(ErrorCallback callback);

private:
    /**
     * @brief 加载配置文件
     * @param config_file 配置文件路径
     * @return bool 加载是否成功
     */
    bool loadConfiguration(const std::string& config_file);
    
    /**
     * @brief 初始化发布者和订阅者
     * @return bool 初始化是否成功
     */
    bool initializePublishersAndSubscribers();
    
    /**
     * @brief 增强命令回调
     * @param msg 命令消息
     */
    void commandCallback(const std::string& message);
    
    /**
     * @brief 增强状态回调
     * @param msg 状态消息
     */
    void statusCallback(const std::string& message);
    
    /**
     * @brief 处理命令请求
     * @param command 命令消息
     */
    void processCommandRequest(const CommandMessage& command);
    
    /**
     * @brief 处理启动命令
     * @param command 命令消息
     */
    void handleStartCommand(const CommandMessage& command);
    
    /**
     * @brief 处理停止命令
     * @param command 命令消息
     */
    void handleStopCommand(const CommandMessage& command);
    
    /**
     * @brief 处理重启命令
     * @param command 命令消息
     */
    void handleRestartCommand(const CommandMessage& command);
    
    /**
     * @brief 处理状态命令
     * @param command 命令消息
     */
    void handleStatusCommand(const CommandMessage& command);
    
    /**
     * @brief 处理配置命令
     * @param command 命令消息
     */
    void handleConfigureCommand(const CommandMessage& command);
    
    /**
     * @brief 构建命令JSON
     * @param command 命令消息
     * @return std::string JSON字符串
     */
    std::string buildCommandJson(const CommandMessage& command) const;
    
    /**
     * @brief 构建状态JSON
     * @param status 状态消息
     * @return std::string JSON字符串
     */
    std::string buildStatusJson(const StatusMessage& status) const;
    
    /**
     * @brief 解析命令消息
     * @param json_str JSON字符串
     * @param command 命令消息（输出）
     * @return bool 解析是否成功
     */
    bool parseCommandMessage(const std::string& json_str, CommandMessage& command) const;
    
    /**
     * @brief 解析状态消息
     * @param json_str JSON字符串
     * @param status 状态消息（输出）
     * @return bool 解析是否成功
     */
    bool parseStatusMessage(const std::string& json_str, StatusMessage& status) const;
    
    /**
     * @brief 检查命令发布间隔
     * @param command_type 命令类型
     * @return bool 是否允许发布
     */
    bool checkCommandPublishInterval(CommandType command_type);
    
    /**
     * @brief 更新服务状态
     * @param new_state 新状态
     */
    void updateServiceState(ServiceState new_state);
    
    /**
     * @brief 记录服务错误
     * @param error_msg 错误信息
     */
    void logServiceError(const std::string& error_msg);
    
    /**
     * @brief 初始化日志系统
     * @param config_file 配置文件路径
     */
    void initializelog_system(const std::string& config_file);
    
    bool is_server_;                                  ///< 是否为服务端模式
    ServiceState service_state_;                      ///< 服务状态
    ServiceStats service_stats_;                      ///< 服务统计信息
    
    std::string command_topic_;                       ///< 命令话题名称
    std::string status_topic_;                        ///< 状态话题名称
    std::chrono::milliseconds command_publish_interval_; ///< 命令发布间隔
    int max_command_retries_;                         ///< 最大命令重试次数
    
    // 管理器
    std::unique_ptr<ProjectManager> lifecycle_manager_;
    std::unique_ptr<DataPublisher> data_publisher_;
    
    // 命令管理器映射
    std::map<ProjectType, std::shared_ptr<CommandManager>> command_managers_;
    
    // 回调函数
    CommandCallback command_callback_;
    StatusCallback status_callback_;
    ErrorCallback error_callback_;
    
    // 时间记录
    std::map<CommandType, std::chrono::system_clock::time_point> last_command_publish_time_;
    
    mutable std::mutex mutex_;                       ///< 线程安全锁
};

} // namespace dds_comm

#endif // DDS_COMMUNICATION__DDS_SERVICE_HPP_