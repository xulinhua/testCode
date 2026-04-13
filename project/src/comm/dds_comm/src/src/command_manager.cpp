#include "../include/dds_comm/command_manager.hpp"
#include "log_system/log_macros.hpp"
#include <chrono>

namespace dds_comm {

CommandManager::CommandManager(ProjectType project_type)
    : project_type_(project_type) {
    LOG_INFO("创建命令管理器: 项目类型=%d", static_cast<int>(project_type));
}

// 默认构造函数（用于dds_service.cpp中的创建）
CommandManager::CommandManager()
    : project_type_(ProjectType::UNKNOWN) {
}

bool CommandManager::executeCommand(const CommandMessage& command) {
    if (command.project_type != project_type_) {
        LOG_WARN("命令项目类型不匹配: 期望=%d, 实际=%d",
                   static_cast<int>(project_type_), 
                   static_cast<int>(command.project_type));
        return false;
    }
    
    try {
        switch (command.command_type) {
            case CommandType::START:
                if (start_handler_) {
                    return start_handler_();
                }
                LOG_WARN("启动处理器未设置");
                break;
                
            case CommandType::STOP:
                if (stop_handler_) {
                    return stop_handler_();
                }
                LOG_WARN("停止处理器未设置");
                break;
                
            case CommandType::STATUS:
                // 状态命令不需要处理器，直接返回状态
                return true;
                
            case CommandType::RESTART:
                if (stop_handler_ && start_handler_) {
                    if (stop_handler_()) {
                        return start_handler_();
                    }
                }
                LOG_WARN("重启处理器未设置");
                break;
                
            case CommandType::CONFIGURE:
                LOG_INFO("配置命令处理");
                // 配置命令需要具体实现
                break;
                
            default:
                LOG_ERROR("未知命令类型: %d", 
                           static_cast<int>(command.command_type));
                break;
        }
        
        return false;
        
    } catch (const std::exception& e) {
        LOG_ERROR("命令执行异常: %s", e.what());
        return false;
    }
}

StatusMessage CommandManager::getProjectStatus() {
    StatusMessage status;
    status.project_type = project_type_;
    status.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    if (status_handler_) {
        return status_handler_();
    }
    
    // 默认状态
    status.status = "项目状态未知";
    status.is_running = false;
    status.uptime = 0;
    status.last_error = "状态处理器未设置";
    
    return status;
}

void CommandManager::setStartHandler(std::function<bool()> handler) {
    start_handler_ = handler;
}

void CommandManager::setStopHandler(std::function<bool()> handler) {
    stop_handler_ = handler;
}

void CommandManager::setStatusHandler(std::function<StatusMessage()> handler) {
    status_handler_ = handler;
}

bool CommandManager::registerCommandHandler(ProjectType project_type, CommandType command_type,
                                           std::function<bool(const CommandMessage&)> handler) {
    if (project_type != project_type_ && project_type_ != ProjectType::UNKNOWN) {
        LOG_WARN("项目类型不匹配");
        return false;
    }
    
    command_handlers_[command_type] = handler;
    return true;
}

bool CommandManager::registerStatusHandler(ProjectType project_type,
                                          std::function<StatusMessage()> handler) {
    if (project_type != project_type_ && project_type_ != ProjectType::UNKNOWN) {
        LOG_WARN("项目类型不匹配");
        return false;
    }
    
    status_query_handler_ = handler;
    return true;
}

} // namespace dds_communication