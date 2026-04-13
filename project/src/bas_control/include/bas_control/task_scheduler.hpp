#ifndef BAS_CONTROL_TASK_SCHEDULER_HPP
#define BAS_CONTROL_TASK_SCHEDULER_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <mutex>
#include <thread>
#include <chrono>
#include "module_info.hpp"
#include <bas_operate/bas_utils.hpp>
#include "launch_mgr.hpp"
#include "status_monitor.hpp"

namespace bas_control {

/**
 * @brief任务调度器类
 *根据场景需求调度视觉资源
 */
class TaskScheduler {
public:
    /**
     * @brief构造函数
     * @param config配置参数
     */
    explicit TaskScheduler(const ConfigParams& config);
    
    /**
     * @brief析函数
     */
    ~TaskScheduler();
    
    //拷构造和赋值
    TaskScheduler(const TaskScheduler&) = delete;
    TaskScheduler& operator=(const TaskScheduler&) = delete;
    
    /**
     * @brief根据场景激活/停用模块
     * @param scene_type场景类型
     * @return true if successful, false otherwise
     */
    bool switchScene(SceneType scene_type);
    
    /**
     * @brief获取当前活跃场景
     * @return当前场景类型
     */
    SceneType getCurrentScene() const;
    
    /**
     * @brief获取所有可用场景
     * @return场景类型列表
     */
    std::vector<SceneType> getAvailableScenes() const;
    
    /**
     * @brief添加场景配置
     * @param scene_config场景配置
     */
    void addScene(const SceneConfig& scene_config);
    
    /**
     * @brief移除场景配置
     * @param scene_type场景类型
     */
    void removeScene(SceneType scene_type);
    
    /**
     * @brief更新场景配置
     * @param scene_config场景配置
     */
    void updateScene(const SceneConfig& scene_config);
    
    /**
     * @brief获取场景配置
     * @param scene_type场景类型
     * @return场景配置，如果未找到则返回空的SceneConfig
     */
    SceneConfig getSceneConfig(SceneType scene_type) const;
    
    /**
     * @brief注册场景切换回调函数
     * @param callback回调函数
     */
    void registerSceneCallback(std::function<void(const std::string&, const std::vector<std::string>&)> callback);
    
    /**
     * @brief获取当前活跃模块列表
     * @return活跃模块列表
     */
    std::vector<std::string> getActiveModules() const;
    
    /**
     * @brief检查模块是否在当前场景中应该激活
     * @param module_name模块名称
     * @return true if module should be active, false otherwise
     */
    bool isModuleActiveInCurrentScene(const std::string& module_name) const;
    
    /**
     * @brief获取模块在不同场景中的状态
     * @param module_name模块名称
     * @return场景类型到激活状态的映射
     */
    std::map<SceneType, bool> getModuleSceneStatus(const std::string& module_name) const;
    
    /**
     * @brief设置模块状态管理器
     * @param launch_manager启动管理器指针
     */
    void setLaunchManager(LaunchMgr* launch_manager);
    
    /**
     * @brief设置状态监控器
     * @param status_monitor状态监控器指针
     */
    void setStatusMonitor(StatusMonitor* status_monitor);
    
    /**
     * @brief获取场景切换历史
     * @param max_entries最大历史条目数
     * @return场景切换历史记录
     */
    std::vector<std::pair<SceneType, std::chrono::system_clock::time_point>> 
    getSceneSwitchHistory(size_t max_entries = 10) const;
    
    /**
     * @brief获取场景切换统计信息
     * @return场景切换统计
     */
    std::map<SceneType, size_t> getSceneSwitchStats() const;

private:
    /**
     * @brief执行场景切换
     * @param scene_type场景类型
     * @return true if successful, false otherwise
     */
    bool executeSceneSwitch(SceneType scene_type);
    
    /**
     * @brief启动需要激活的模块
     * @param modules_to_start需要启动的模块列表
     * @return成功启动的模块数量
     */
    size_t startRequiredModules(const std::vector<std::string>& modules_to_start);
    
    /**
     * @brief停止需要停用的模块
     * @param modules_to_stop需要停止的模块列表
     * @return成功停止的模块数量
     */
    size_t stopUnnecessaryModules(const std::vector<std::string>& modules_to_stop);
    
    /**
     * @brief获取需要启动的模块
     * @param target_modules目标场景的模块列表
     * @param current_modules当前活跃的模块列表
     * @return需要启动的模块列表
     */
    std::vector<std::string> getModulesToStart(const std::vector<std::string>& target_modules,
        const std::vector<std::string>& current_modules) const;
    
    /**
     * @brief获取需要停止的模块
     * @param target_modules目标场景的模块列表
     * @param current_modules当前活跃的模块列表
     * @return需要停止的模块列表
     */
    std::vector<std::string> getModulesToStop(const std::vector<std::string>& target_modules,
        const std::vector<std::string>& current_modules) const;
    
    /**
     * @brief验证场景配置
     * @param scene_config场景配置
     * @return true if valid, false otherwise
     */
    bool validateSceneConfig(const SceneConfig& scene_config) const;
    
    /**
     * @brief触发场景切换回调
     * @param scene_type场景类型
     * @param active_modules活跃模块列表
     */
    void triggerSceneCallback(SceneType scene_type, const std::vector<std::string>& active_modules);
    
    /**
     * @brief记录场景切换历史
     * @param scene_type场景类型
     */
    void recordSceneSwitch(SceneType scene_type);
    
    /**
     * @brief获取模块依赖关系
     * @param modules模块列表
     * @return模块启动顺序
     */
    std::vector<std::string> getModuleStartupOrder(const std::vector<std::string>& modules) const;

private:
    // 场景配置与状态
    ConfigParams config_;                         ///<配置参数
    mutable std::mutex scenes_mutex_;             ///<场景互斥锁
    std::map<SceneType, SceneConfig> scenes_;     ///<场景配置映射
    SceneType current_scene_;                     ///<当前场景类型
    std::vector<std::string> active_modules_;     ///<当前活跃模块列表

    // **依赖注入的组件指针**：通过setter方法注入，而非直接创建，降低耦合
    LaunchMgr* launch_manager_;           ///<启动管理器指针，用于启动/停止模块
    StatusMonitor* status_monitor_;       ///<状态监控器指针，用于获取健康状态以辅助决策

    std::vector<std::function<void(const std::string&, const std::vector<std::string>&)>> scene_callbacks_; ///<场景回调函数
    mutable std::mutex callbacks_mutex_; ///<回调函数互斥锁
    
    // 历史与统计（用于运维分析）
    std::vector<std::pair<SceneType, std::chrono::system_clock::time_point>> scene_history_; ///<场景切换历史
    std::map<SceneType, size_t> scene_switch_stats_;   ///<各场景切换次数统计
    mutable std::mutex history_mutex_;                   ///<历史记录互斥锁
    
};

} // namespace bas_control

#endif // BAS_CONTROL_TASK_SCHEDULER_HPP