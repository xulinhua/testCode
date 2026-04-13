#ifndef DDS_COMMUNICATION__PROJECT_MANAGER_HPP_
#define DDS_COMMUNICATION__PROJECT_MANAGER_HPP_

#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <mutex>
#include <vector>
#include <functional>
#include "command_types.hpp"

namespace dds_comm {

/**
 * @class ProjectManager
 * @brief 项目管理类 - DDS通讯项目生命周期管理
 * 
 * 功能概述：
 * 1. 项目注册和注销管理 - 支持项目动态注册和注销
 * 2. 项目状态监控和更新 - 实时监控项目运行状态
 * 3. 错误处理和恢复机制 - 自动错误检测和恢复
 * 4. 项目生命周期状态跟踪 - 完整的状态机管理
 * 5. 健康检查和自动恢复 - 定期健康检查和自动恢复机制
 * 
 * 耦合关系：
 * - 与DataPublisher协同工作，管理数据发布项目
 * - 依赖log_system进行日志记录
 * - 与CommandManager协同处理项目控制命令
 * 
 * 使用说明：
 * 1. 使用registerProject注册新项目
 * 2. 调用startProject启动项目运行
 * 3. 通过updateProjectState更新项目状态
 * 4. 使用getProjectInfo获取项目信息
 * 5. 调用healthCheck进行健康检查
 * 
 * 线程安全：
 * - 使用std::mutex确保多线程环境下的线程安全
 * - 所有公共方法都进行了线程安全保护
 * 
 * 状态转换规则：
 * - UNREGISTERED -> REGISTERED: 项目注册
 * - REGISTERED -> STARTING: 项目启动
 * - STARTING -> RUNNING: 启动完成
 * - RUNNING -> STOPPING: 项目停止
 * - STOPPING -> REGISTERED: 停止完成
 * - 任何状态 -> ERROR: 发生错误
 * - ERROR -> RECOVERING: 开始恢复
 * - RECOVERING -> RUNNING: 恢复完成
 * 
 * 支持的项目状态：
 * - UNREGISTERED: 未注册
 * - REGISTERED: 已注册但未启动
 * - STARTING: 启动中
 * - RUNNING: 运行中
 * - STOPPING: 停止中
 * - ERROR: 错误状态
 * - RECOVERING: 恢复中
 */

class ProjectManager {
public:
    /**
     * @brief 项目状态枚举
     */
    enum class ProjectState {
        UNREGISTERED,   ///< 未注册
        REGISTERED,     ///< 已注册
        STARTING,       ///< 启动中
        RUNNING,        ///< 运行中
        STOPPING,       ///< 停止中
        ERROR,          ///< 错误状态
        RECOVERING      ///< 恢复中
    };
    
    /**
     * @brief 项目信息结构
     */
    struct ProjectInfo {
        ProjectType project_type;          ///< 项目类型
        std::string project_name;          ///< 项目名称
        ProjectState state;                ///< 当前状态
        std::chrono::system_clock::time_point start_time; ///< 启动时间
        std::chrono::system_clock::time_point last_update; ///< 最后更新时间
        std::string last_error;           ///< 最后错误信息
        int error_count;                  ///< 错误计数
        bool auto_recovery;               ///< 是否自动恢复
        int max_retries;                  ///< 最大重试次数
    };
    
    /**
     * @brief 构造函数
     */
    ProjectManager();
    
    /**
     * @brief 析构函数
     */
    ~ProjectManager();
    
    /**
     * @brief 注册项目
     * @param project_type 项目类型
     * @param project_name 项目名称
     * @param auto_recovery 是否自动恢复
     * @param max_retries 最大重试次数
     * @return bool 注册是否成功
     */
    bool registerProject(ProjectType project_type, const std::string& project_name,
                        bool auto_recovery = true, int max_retries = 3);
    
    /**
     * @brief 注销项目
     * @param project_type 项目类型
     * @return bool 注销是否成功
     */
    bool unregisterProject(ProjectType project_type);
    
    /**
     * @brief 启动项目
     * @param project_type 项目类型
     * @return bool 启动是否成功
     */
    bool startProject(ProjectType project_type);
    
    /**
     * @brief 停止项目
     * @param project_type 项目类型
     * @return bool 停止是否成功
     */
    bool stopProject(ProjectType project_type);
    
    /**
     * @brief 重启项目
     * @param project_type 项目类型
     * @return bool 重启是否成功
     */
    bool restartProject(ProjectType project_type);
    
    /**
     * @brief 更新项目状态
     * @param project_type 项目类型
     * @param state 新状态
     * @param error_msg 错误信息（可选）
     * @return bool 更新是否成功
     */
    bool updateProjectState(ProjectType project_type, ProjectState state, 
                           const std::string& error_msg = "");
    
    /**
     * @brief 获取项目信息
     * @param project_type 项目类型
     * @return ProjectInfo 项目信息
     */
    ProjectInfo getProjectInfo(ProjectType project_type) const;
    
    /**
     * @brief 获取所有项目信息
     * @return std::vector<ProjectInfo> 项目信息列表
     */
    std::vector<ProjectInfo> getAllProjectInfo() const;
    
    /**
     * @brief 检查项目是否运行中
     * @param project_type 项目类型
     * @return bool 是否运行中
     */
    bool isProjectRunning(ProjectType project_type) const;
    
    /**
     * @brief 检查项目是否注册
     * @param project_type 项目类型
     * @return bool 是否注册
     */
    bool isProjectRegistered(ProjectType project_type) const;
    
    /**
     * @brief 获取项目运行时间
     * @param project_type 项目类型
     * @return int 运行时间（秒）
     */
    int getProjectUptime(ProjectType project_type) const;
    
    /**
     * @brief 获取项目错误计数
     * @param project_type 项目类型
     * @return int 错误计数
     */
    int getProjectErrorCount(ProjectType project_type) const;
    
    /**
     * @brief 执行健康检查
     * @param project_type 项目类型
     * @return bool 健康状态
     */
    bool healthCheck(ProjectType project_type);
    
    /**
     * @brief 执行自动恢复
     * @param project_type 项目类型
     * @return bool 恢复是否成功
     */
    bool autoRecovery(ProjectType project_type);
    
    /**
     * @brief 清理过期项目
     * @param timeout_seconds 超时时间（秒）
     * @return int 清理的项目数量
     */
    int cleanupExpiredProjects(int timeout_seconds = 3600);

    /**
     * @brief 状态字符串转换
     * @param state 状态
     * @return std::string 状态字符串
     */
    std::string stateToString(ProjectState state) const;

private:
    /**
     * @brief 项目信息映射类型
     */
    using ProjectInfoMap = std::map<ProjectType, ProjectInfo>;
    
    /**
     * @brief 状态转换检查
     * @param current_state 当前状态
     * @param new_state 新状态
     * @return bool 状态转换是否有效
     */
    bool isValidStateTransition(ProjectState current_state, ProjectState new_state) const;
    
    mutable std::mutex mutex_;           ///< 线程安全锁
    ProjectInfoMap projects_;            ///< 项目信息映射
};

} // namespace dds_comm

#endif // DDS_COMMUNICATION__PROJECT_MANAGER_HPP_