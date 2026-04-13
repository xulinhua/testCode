#ifndef DDS_COMMUNICATION__COMMAND_MANAGER_HPP_
#define DDS_COMMUNICATION__COMMAND_MANAGER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <map>
#include "command_types.hpp"
#include "log_system/log_macros.hpp"

namespace dds_comm {

/**
 * @class CommandManager
 * @brief 命令管理器类
 * 
 * 负责处理特定项目的命令执行和状态管理
 */
class CommandManager {
public:
    using Ptr = std::shared_ptr<CommandManager>;
    
    /**
     * @brief 默认构造函数
     */
    CommandManager();
    
    /**
     * @brief 构造函数
     * @param project_type 项目类型
     */
    CommandManager(ProjectType project_type);
    
    /**
     * @brief 析构函数
     */
    virtual ~CommandManager() = default;
    
    /**
     * @brief 执行命令
     * @param command 命令消息
     * @return bool 执行是否成功
     */
    virtual bool executeCommand(const CommandMessage& command);
    
    /**
     * @brief 获取项目状态
     * @return StatusMessage 项目状态
     */
    virtual StatusMessage getProjectStatus();
    
    /**
     * @brief 设置启动处理器
     * @param handler 启动处理器函数
     */
    void setStartHandler(std::function<bool()> handler);
    
    /**
     * @brief 设置停止处理器
     * @param handler 停止处理器函数
     */
    void setStopHandler(std::function<bool()> handler);
    
    /**
     * @brief 设置状态处理器
     * @param handler 状态处理器函数
     */
    void setStatusHandler(std::function<StatusMessage()> handler);
    
    /**
     * @brief 注册命令处理器
     * @param project_type 项目类型
     * @param command_type 命令类型
     * @param handler 命令处理函数
     * @return bool 注册是否成功
     */
    bool registerCommandHandler(ProjectType project_type, CommandType command_type,
                               std::function<bool(const CommandMessage&)> handler);
    
    /**
     * @brief 注册状态处理器
     * @param project_type 项目类型
     * @param handler 状态查询函数
     * @return bool 注册是否成功
     */
    bool registerStatusHandler(ProjectType project_type,
                              std::function<StatusMessage()> handler);

protected:
    ProjectType project_type_;
    
    std::function<bool()> start_handler_;
    std::function<bool()> stop_handler_;
    std::function<StatusMessage()> status_handler_;
    
    // 命令处理器映射
    std::map<CommandType, std::function<bool(const CommandMessage&)>> command_handlers_;
    
    // 状态处理器
    std::function<StatusMessage()> status_query_handler_;
};

} // namespace dds_comm

#endif // DDS_COMMUNICATION__COMMAND_MANAGER_HPP_