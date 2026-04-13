/**
 * @file project_manager.cpp
 * @brief 项目管理器实现 - DDS通讯项目生命周期管理
 * 
 * 功能实现说明：
 * 1. 项目注册和注销管理 - 完整的项目生命周期管理
 * 2. 项目状态监控和更新 - 实时状态跟踪和状态转换验证
 * 3. 错误处理和恢复机制 - 自动错误检测、恢复和重试机制
 * 4. 健康检查和自动恢复 - 定期健康状态检查和自动恢复流程
 * 5. 项目信息管理 - 项目运行时间、错误计数等统计信息
 * 
 * 核心方法说明：
 * - registerProject/unregisterProject: 项目注册和注销管理
 * - startProject/stopProject/restartProject: 项目启动、停止和重启
 * - updateProjectState: 项目状态更新和状态转换验证
 * - healthCheck/autoRecovery: 健康检查和自动恢复
 * - stateToString: 状态枚举值到字符串的转换
 * - isValidStateTransition: 状态转换规则验证
 * 
 * 实现特点：
 * - 线程安全：所有方法都使用std::mutex进行保护
 * - 状态机管理：完整的状态转换规则验证
 * - 错误恢复：支持自动恢复和重试机制
 * - 统计信息：完整的项目运行统计
 * 
 * 使用注意事项：
 * - 状态转换必须符合预定义的转换规则
 * - 健康检查失败会自动触发恢复机制
 * - 项目信息在内存中维护，重启后需要重新注册
 */

#include "../include/dds_comm/project_manager.hpp"
#include "log_system/log_macros.hpp"
#include <chrono>
#include <algorithm>

namespace dds_comm {

ProjectManager::ProjectManager() {
    LOG_INFO("项目管理器初始化完成");
}

ProjectManager::~ProjectManager() {
    std::lock_guard<std::mutex> lock(mutex_);
    projects_.clear();
    LOG_INFO("项目管理器已销毁");
}

bool ProjectManager::registerProject(ProjectType project_type, 
                                            const std::string& project_name,
                                            bool auto_recovery, int max_retries) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (projects_.find(project_type) != projects_.end()) {
        LOG_WARN("项目已注册: %s", project_name.c_str());
        return false;
    }
    
    ProjectInfo info;
    info.project_type = project_type;
    info.project_name = project_name;
    info.state = ProjectState::REGISTERED;
    info.start_time = std::chrono::system_clock::now();
    info.last_update = std::chrono::system_clock::now();
    info.last_error = "";
    info.error_count = 0;
    info.auto_recovery = auto_recovery;
    info.max_retries = max_retries;
    
    projects_[project_type] = info;
    
    LOG_INFO("项目注册成功: %s (类型: %s)", 
                project_name.c_str(),
                CommandTypeConverter::toString(project_type).c_str());
    
    return true;
}

bool ProjectManager::unregisterProject(ProjectType project_type) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = projects_.find(project_type);
    if (it == projects_.end()) {
        LOG_WARN("项目未注册: %s", 
                    CommandTypeConverter::toString(project_type).c_str());
        return false;
    }
    
    // 如果项目正在运行，先停止
    if (it->second.state == ProjectState::RUNNING || 
        it->second.state == ProjectState::STARTING) {
        LOG_WARN("项目正在运行，先停止: %s", it->second.project_name.c_str());
        return false;
    }
    
    projects_.erase(it);
    
    LOG_INFO("项目注销成功: %s", 
                CommandTypeConverter::toString(project_type).c_str());
    
    return true;
}

bool ProjectManager::startProject(ProjectType project_type) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = projects_.find(project_type);
    if (it == projects_.end()) {
        LOG_ERROR("项目未注册: %s", 
                     CommandTypeConverter::toString(project_type).c_str());
        return false;
    }
    
    if (!isValidStateTransition(it->second.state, ProjectState::STARTING)) {
        LOG_ERROR("无效的状态转换: %s -> STARTING", 
                     stateToString(it->second.state).c_str());
        return false;
    }
    
    it->second.state = ProjectState::STARTING;
    it->second.last_update = std::chrono::system_clock::now();
    
    LOG_INFO("项目启动中: %s", it->second.project_name.c_str());
    
    return true;
}

bool ProjectManager::stopProject(ProjectType project_type) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = projects_.find(project_type);
    if (it == projects_.end()) {
        LOG_ERROR("项目未注册: %s", 
                     CommandTypeConverter::toString(project_type).c_str());
        return false;
    }
    
    if (!isValidStateTransition(it->second.state, ProjectState::STOPPING)) {
        LOG_ERROR("无效的状态转换: %s -> STOPPING", 
                     stateToString(it->second.state).c_str());
        return false;
    }
    
    it->second.state = ProjectState::STOPPING;
    it->second.last_update = std::chrono::system_clock::now();
    
    LOG_INFO("项目停止中: %s", it->second.project_name.c_str());
    
    return true;
}

bool ProjectManager::restartProject(ProjectType project_type) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = projects_.find(project_type);
    if (it == projects_.end()) {
        LOG_ERROR("项目未注册: %s", 
                     CommandTypeConverter::toString(project_type).c_str());
        return false;
    }
    
    // 先停止项目
    if (it->second.state == ProjectState::RUNNING || 
        it->second.state == ProjectState::STARTING) {
        it->second.state = ProjectState::STOPPING;
        it->second.last_update = std::chrono::system_clock::now();
        
        LOG_INFO("项目重启: 先停止 %s", it->second.project_name.c_str());
    }
    
    // 然后启动项目
    it->second.state = ProjectState::STARTING;
    it->second.last_update = std::chrono::system_clock::now();
    it->second.error_count = 0;
    it->second.last_error = "";
    
    LOG_INFO("项目重启: 启动 %s", it->second.project_name.c_str());
    
    return true;
}

bool ProjectManager::updateProjectState(ProjectType project_type, 
                                                ProjectState state, 
                                                const std::string& error_msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = projects_.find(project_type);
    if (it == projects_.end()) {
        LOG_ERROR("项目未注册: %s", 
                     CommandTypeConverter::toString(project_type).c_str());
        return false;
    }
    
    if (!isValidStateTransition(it->second.state, state)) {
        LOG_ERROR("无效的状态转换: %s -> %s", 
                     stateToString(it->second.state).c_str(),
                     stateToString(state).c_str());
        return false;
    }
    
    it->second.state = state;
    it->second.last_update = std::chrono::system_clock::now();
    
    if (!error_msg.empty()) {
        it->second.last_error = error_msg;
        it->second.error_count++;
        
        LOG_WARN("项目状态更新: %s -> %s, 错误: %s", 
                    it->second.project_name.c_str(),
                    stateToString(state).c_str(),
                    error_msg.c_str());
    } else {
        LOG_INFO("项目状态更新: %s -> %s", 
                    it->second.project_name.c_str(),
                    stateToString(state).c_str());
    }
    
    return true;
}

ProjectManager::ProjectInfo ProjectManager::getProjectInfo(
    ProjectType project_type) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = projects_.find(project_type);
    if (it == projects_.end()) {
        ProjectInfo empty_info;
        empty_info.project_type = ProjectType::UNKNOWN;
        empty_info.state = ProjectState::UNREGISTERED;
        return empty_info;
    }
    
    return it->second;
}

std::vector<ProjectManager::ProjectInfo> 
ProjectManager::getAllProjectInfo() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<ProjectInfo> infos;
    for (const auto& pair : projects_) {
        infos.push_back(pair.second);
    }
    
    return infos;
}

bool ProjectManager::isProjectRunning(ProjectType project_type) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = projects_.find(project_type);
    if (it == projects_.end()) {
        return false;
    }
    
    return it->second.state == ProjectState::RUNNING;
}

bool ProjectManager::isProjectRegistered(ProjectType project_type) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return projects_.find(project_type) != projects_.end();
}

int ProjectManager::getProjectUptime(ProjectType project_type) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = projects_.find(project_type);
    if (it == projects_.end()) {
        return 0;
    }
    
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(
        now - it->second.start_time);
    
    return duration.count();
}

int ProjectManager::getProjectErrorCount(ProjectType project_type) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = projects_.find(project_type);
    if (it == projects_.end()) {
        return 0;
    }
    
    return it->second.error_count;
}

bool ProjectManager::healthCheck(ProjectType project_type) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = projects_.find(project_type);
    if (it == projects_.end()) {
        return false;
    }
    
    // 检查项目是否长时间没有更新
    auto now = std::chrono::system_clock::now();
    auto time_since_update = std::chrono::duration_cast<std::chrono::seconds>(
        now - it->second.last_update);
    
    if (time_since_update > std::chrono::seconds(300)) { // 5分钟没有更新
        LOG_WARN("项目健康检查失败: %s 长时间没有更新", 
                    it->second.project_name.c_str());
        return false;
    }
    
    // 检查错误计数是否超过阈值
    if (it->second.error_count > 10) {
        LOG_WARN("项目健康检查失败: %s 错误计数过高", 
                    it->second.project_name.c_str());
        return false;
    }
    
    return true;
}

bool ProjectManager::autoRecovery(ProjectType project_type) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = projects_.find(project_type);
    if (it == projects_.end()) {
        return false;
    }
    
    if (!it->second.auto_recovery) {
        LOG_INFO("项目 %s 未启用自动恢复", it->second.project_name.c_str());
        return false;
    }
    
    if (it->second.error_count >= it->second.max_retries) {
        LOG_ERROR("项目 %s 已达到最大重试次数", it->second.project_name.c_str());
        return false;
    }
    
    // 执行恢复操作：重启项目
    it->second.state = ProjectState::RECOVERING;
    it->second.last_update = std::chrono::system_clock::now();
    
    LOG_INFO("项目自动恢复: %s", it->second.project_name.c_str());
    
    return true;
}

int ProjectManager::cleanupExpiredProjects(int timeout_seconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    int cleaned_count = 0;
    auto now = std::chrono::system_clock::now();
    
    for (auto it = projects_.begin(); it != projects_.end(); ) {
        auto time_since_update = std::chrono::duration_cast<std::chrono::seconds>(
            now - it->second.last_update);
        
        if (time_since_update > std::chrono::seconds(timeout_seconds) &&
            it->second.state != ProjectState::RUNNING) {
            
            LOG_INFO("清理过期项目: %s", it->second.project_name.c_str());
            
            it = projects_.erase(it);
            cleaned_count++;
        } else {
            ++it;
        }
    }
    
    return cleaned_count;
}

bool ProjectManager::isValidStateTransition(ProjectState current_state, 
                                                   ProjectState new_state) const {
    // 定义有效的状态转换
    switch (current_state) {
        case ProjectState::UNREGISTERED:
            return new_state == ProjectState::REGISTERED;
            
        case ProjectState::REGISTERED:
            return new_state == ProjectState::STARTING || 
                   new_state == ProjectState::UNREGISTERED;
            
        case ProjectState::STARTING:
            return new_state == ProjectState::RUNNING || 
                   new_state == ProjectState::ERROR;
            
        case ProjectState::RUNNING:
            return new_state == ProjectState::STOPPING || 
                   new_state == ProjectState::ERROR;
            
        case ProjectState::STOPPING:
            return new_state == ProjectState::REGISTERED || 
                   new_state == ProjectState::ERROR;
            
        case ProjectState::ERROR:
            return new_state == ProjectState::RECOVERING || 
                   new_state == ProjectState::STOPPING;
            
        case ProjectState::RECOVERING:
            return new_state == ProjectState::STARTING || 
                   new_state == ProjectState::ERROR;
            
        default:
            return false;
    }
}

std::string ProjectManager::stateToString(ProjectState state) const {
    switch (state) {
        case ProjectState::UNREGISTERED: return "UNREGISTERED";
        case ProjectState::REGISTERED: return "REGISTERED";
        case ProjectState::STARTING: return "STARTING";
        case ProjectState::RUNNING: return "RUNNING";
        case ProjectState::STOPPING: return "STOPPING";
        case ProjectState::ERROR: return "ERROR";
        case ProjectState::RECOVERING: return "RECOVERING";
        default: return "UNKNOWN";
    }
}

} // namespace dds_communication