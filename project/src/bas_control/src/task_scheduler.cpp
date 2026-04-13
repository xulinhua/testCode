/**
 * @file task_scheduler.cpp
 * @brief 任务调度器实现文件
 * 
 * 功能概述：
 * TaskScheduler类负责根据场景需求动态调度视觉资源，通过场景切换优化系统资源使用：
 * 1. 场景管理：定义和维护多个场景配置，每个场景包含特定任务所需的活跃模块集合
 * 2. 动态调度：根据任务需求切换到相应场景，自动激活/停用模块，调整系统功能配置
 * 3. 依赖协调：与LaunchMgr协作，按依赖关系安全地启动/停止模块
 * 4. 状态同步：与StatusMonitor同步模块状态，确保场景切换的可靠性
 * 5. 历史记录：跟踪场景切换历史，支持场景使用统计和性能分析
 * 
 * 主要接口说明：
 * 1. 场景控制：
 *    - switchScene()：切换到指定场景，动态调整活跃模块集合
 *    - getCurrentScene()：获取当前活跃场景名称
 *    - getAvailableScenes()：获取所有可用场景名称列表
 * 
 * 2. 配置管理：
 *    - addScene()：添加新的场景配置
 *    - removeScene()：移除指定场景配置（默认idle场景不允许删除）
 *    - updateScene()：更新指定场景的配置
 *    - getSceneConfig()：获取指定场景的详细配置信息
 *    - isModuleActiveInCurrentScene()：检查模块在当前场景中是否应该激活
 *    - getModuleSceneStatus()：获取模块在不同场景中的激活状态映射
 * 
 * 3. 状态查询：
 *    - getActiveModules()：获取当前场景下活跃的模块列表
 *    - getSceneSwitchHistory()：获取场景切换历史记录
 *    - getSceneSwitchStats()：获取场景切换统计信息（各场景切换次数）
 * 
 * 4. 依赖设置：
 *    - setLaunchManager()：设置启动管理器，用于启动/停止模块
 *    - setStatusMonitor()：设置状态监控器，用于同步模块状态
 * 
 * 5. 回调注册：
 *    - registerSceneCallback()：注册场景切换回调函数
 * 
 * 实现机制：
 * 1. 场景配置映射：使用map存储场景配置（SceneConfig），支持快速查找和更新
 * 2. 模块差异计算：比较当前场景和目标场景的模块集合，计算需要启动/停止的模块
 * 3. 安全切换流程：先停止不需要的模块，再启动需要的模块，避免资源冲突
 * 4. 历史记录管理：使用vector存储场景切换历史，支持历史查询和统计
 * 5. 多线程安全：使用scenes_mutex_保护场景数据，history_mutex_保护历史记录
 * 
 * 默认场景配置：
 * 1. idle场景（空闲场景）：仅启动基础服务（bas_sys_config_ros, cam_mgr_ros）
 * 2. navigation场景（导航场景）：启动检测和OCR服务（yolo_det, ppocr等）
 * 3. interaction场景（交互场景）：启动人脸识别和手势识别服务（face_det, hand_gesture_rec）
 * 4. manipulation场景（操作场景）：启动精确检测服务（yolo_det, yolo_obb_det）
 * 5. full场景（完整场景）：启动所有视觉服务，适用于复杂任务
 * 
 * 场景切换流程：
 * 1. 验证目标场景：检查场景名称有效性，确保场景配置存在且有效
 * 2. 计算模块差异：比较当前活跃模块与目标场景活跃模块
 *    - modules_to_stop：当前有但目标没有的模块（需要停止）
 *    - modules_to_start：当前没有但目标有的模块（需要启动）
 * 3. 停止不需要的模块：按依赖逆序安全停止modules_to_stop中的模块
 * 4. 启动需要的模块：按依赖顺序安全启动modules_to_start中的模块
 * 5. 更新状态：更新当前场景和活跃模块列表，记录切换历史
 * 6. 触发回调：通知所有注册的回调函数场景切换已完成
 * 
 * 场景验证规则：
 * 1. 场景名称不能为空
 * 2. 活跃模块列表不能为空（每个场景必须至少有一个活动模块）
 * 3. 模块名称不能为空（每个模块必须有有效名称）
 * 4. 依赖关系检查（通过LaunchMgr验证模块依赖是否满足）
 * 
 * 使用场景：
 * 1. 任务适配：根据不同任务需求（导航、交互、操作等）动态调整视觉功能
 * 2. 资源优化：根据任务复杂度激活相应模块，避免不必要的资源消耗
 * 3. 系统扩展：支持新增场景配置，适应未来功能扩展需求
 * 4. 性能分析：通过场景切换统计，分析不同任务对系统资源的需求
 * 5. 故障隔离：通过场景切换隔离故障模块，提高系统整体可用性
 * 
 * 注意事项：
 * 1. 场景切换涉及模块启动/停止，可能造成短暂的服务中断
 * 2. 默认idle场景应保持最小配置，确保系统基础功能可用
 * 3. 场景配置需考虑模块依赖关系，避免因依赖不满足导致切换失败
 * 4. 回调函数应避免长时间阻塞，否则影响后续场景切换
 * 5. 历史记录大小有限，重要切换记录建议外部存储
 */

#include "bas_control/task_scheduler.hpp"
#include "bas_control/launch_mgr.hpp"
#include "bas_control/status_monitor.hpp"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include "log_system/log_macros.hpp"

namespace bas_control {

TaskScheduler::TaskScheduler(const ConfigParams& config)
    : config_(config), launch_manager_(nullptr), status_monitor_(nullptr) 
{
    
    // 从配置文件加载场景配置
    bool has_config_scenes = !config_.scenes.empty();
    
    if (has_config_scenes) 
    {
        // 从配置文件加载场景
        LOG_INFO("从配置文件加载场景配置，共 %zu 个场景", config_.scenes.size());
        
        for (const auto& pair : config_.scenes) 
        {
            const std::string& scene_name = pair.first;
            const SceneConfig& scene_config = pair.second;
            
            // 将场景添加到 scenes_ 映射中
            scenes_[scene_config.type] = scene_config;
            
            LOG_INFO(false, logsys::Color::BLUE, "场景: %s, 描述: %s, 活动模块数: %zu", 
                scene_name.c_str(), scene_config.description.c_str(), scene_config.active_modules.size());
            
            // 输出场景的活动模块列表
            std::string modules_str;
            for (size_t i = 0; i < scene_config.active_modules.size(); ++i) 
            {
                modules_str += scene_config.active_modules[i];
                if (i < scene_config.active_modules.size() - 1) 
                {
                    modules_str += ", ";
                }
            }
            LOG_INFO(false, logsys::Color::BLUE, "  活动模块: [%s]", modules_str.c_str());
        }
    } 
    else 
    {
        // 使用默认场景配置（硬编码）
        LOG_INFO("配置文件中无场景配置，使用默认场景配置");
        
        SceneConfig idle_scene(SceneType::IDLE, {"bas_sys_config_ros", "cam_mgr_ros"});
        idle_scene.description = "空闲场景，仅启动基础服务";
        scenes_[SceneType::IDLE] = idle_scene;
        
        SceneConfig navigation_scene(SceneType::NAVIGATION, {"bas_sys_config_ros", "cam_mgr_ros", "yolo_det", "ppocr"});
        navigation_scene.description = "导航场景，启动检测和OCR服务";
        scenes_[SceneType::NAVIGATION] = navigation_scene;
        
        SceneConfig interaction_scene(SceneType::INTERACTION, {"bas_sys_config_ros", "cam_mgr_ros", "face_det", "hand_gesture_rec"});
        interaction_scene.description = "交互场景，启动人脸识别和手势识别";
        scenes_[SceneType::INTERACTION] = interaction_scene;
        
        SceneConfig manipulation_scene(SceneType::MANIPULATION, {"bas_sys_config_ros", "cam_mgr_ros", "yolo_det", "yolo_obb_det"});
        manipulation_scene.description = "操作场景，启动精确检测服务";
        scenes_[SceneType::MANIPULATION] = manipulation_scene;
        
        SceneConfig full_scene(SceneType::FULL, {"bas_sys_config_ros", "cam_mgr_ros", "yolo_det", "face_det", "hand_gesture_rec", "ppocr", "yolo_obb_det"});
        full_scene.description = "完整场景，启动所有视觉服务";
        scenes_[SceneType::FULL] = full_scene;
    }
    
    // 设置默认启动场景（从配置文件读取）
    std::string default_scene_name = config_.default_scene.empty() ? "idle" : config_.default_scene;
    SceneType default_scene_type = stringToSceneType(default_scene_name);
    
    // 验证默认场景是否存在
    auto it = scenes_.find(default_scene_type);
    if (it == scenes_.end()) 
    {
        LOG_WARN("配置的默认场景 '%s' 不存在，使用 'idle' 场景", default_scene_name.c_str());
        default_scene_type = SceneType::IDLE;
        it = scenes_.find(default_scene_type);
    }
    
    if (it != scenes_.end()) 
    {
        current_scene_ = default_scene_type;
        active_modules_ = it->second.active_modules;
        LOG_INFO("默认启动场景设置为: %s", sceneTypeToString(default_scene_type).c_str());
    } 
    else 
    {
        // 兜底：使用第一个可用的场景
        if (!scenes_.empty()) 
        {
            current_scene_ = scenes_.begin()->first;
            active_modules_ = scenes_.begin()->second.active_modules;
            LOG_WARN("使用第一个可用场景: %s", sceneTypeToString(current_scene_).c_str());
        } 
        else 
        {
            LOG_ERROR("无可用场景配置！");
        }
    }
    
    LOG_INFO("任务调度器已初始化，共 %zu 个场景，当前场景: %s", 
             scenes_.size(), sceneTypeToString(current_scene_).c_str());
}

TaskScheduler::~TaskScheduler() 
{
    //停止所有模块
    if (launch_manager_) {
        // 不在这里直接调用stop，由外部管理器负责清理
    }
}

bool TaskScheduler::switchScene(SceneType scene_type) 
{
    std::string scene_name = sceneTypeToString(scene_type);
    LOG_INFO("正在切换到场景: %s", scene_name.c_str());
    std::lock_guard<std::mutex> lock(scenes_mutex_);
    auto it = scenes_.find(scene_type);
    if (it == scenes_.end()) 
    {
        LOG_WARN("场景未找到: %s", scene_name.c_str());
        return false;
    } 
    if (scene_type == current_scene_) 
    {
        LOG_INFO("已在当前场景中: %s", scene_name.c_str());
        
        // ========== 新增：检查当前场景下每个模块是否正常启动 ==========
        SceneConfig current_scene_config = it->second;
        std::vector<std::string> modules_to_start;
        
        // 检查每个活动模块是否正常运行
        for (const auto& module_name : current_scene_config.active_modules) 
        {
            if (launch_manager_) 
            {
                bool is_running = launch_manager_->isModuleRunning(module_name);
                if (!is_running) 
                {
                    LOG_WARN("模块未正常运行: %s，将尝试启动", module_name.c_str());
                    modules_to_start.push_back(module_name);
                }
            }
        }
        
        // 如果有模块未正常运行，尝试启动它们
        if (!modules_to_start.empty()) 
        {
            LOG_INFO("当前场景有 %zu 个模块未正常运行，正在重新启动...", modules_to_start.size());
            size_t started_count = startRequiredModules(modules_to_start);
            
            if (started_count == modules_to_start.size()) 
            {
                LOG_INFO("所有未运行的模块已成功启动");
            } 
            else 
            {
                LOG_WARN("部分模块启动失败，成功: %zu / %zu", started_count, modules_to_start.size());
            }
        } 
        else 
        {
            LOG_INFO("当前场景所有模块均正常运行");
        }
        
        return true;
    }
    SceneConfig target_scene = it->second;
    //验证场景配置
    if (!validateSceneConfig(target_scene)) 
    {
        LOG_WARN("无效的场景配置: %s", scene_name.c_str());
        return false;
    }
    //执行场景切换
    if (!executeSceneSwitch(scene_type)) 
    {
        LOG_WARN("场景切换执行失败: %s", scene_name.c_str());
        return false;
    }
    //更新状态
    current_scene_ = scene_type;
    active_modules_ = target_scene.active_modules;
    //记录场景切换历史
    recordSceneSwitch(scene_type);
    //更新统计信息
    scene_switch_stats_[scene_type]++;
    //触发回调
    triggerSceneCallback(scene_type, active_modules_);
    LOG_INFO("场景切换成功: %s", scene_name.c_str());
    return true;
}

SceneType TaskScheduler::getCurrentScene() const 
{
    std::lock_guard<std::mutex> lock(scenes_mutex_);
    return current_scene_;
}

std::vector<SceneType> TaskScheduler::getAvailableScenes() const 
{
    std::lock_guard<std::mutex> lock(scenes_mutex_);
    std::vector<SceneType> scene_types;
    for (const auto& pair : scenes_) {
        scene_types.push_back(pair.first);
    }
    return scene_types;
}

void TaskScheduler::addScene(const SceneConfig& scene_config) 
{
    if (scene_config.name.empty()) 
    {
        LOG_WARN("无法添加空名称的场景");
        return;
    }   
    std::lock_guard<std::mutex> lock(scenes_mutex_); 
    if (!validateSceneConfig(scene_config)) 
    {
        LOG_WARN("无效的场景配置: %s", scene_config.name.c_str());
        return;
    }
    scenes_[scene_config.type] = scene_config;
    LOG_INFO("场景已添加: %s", scene_config.name.c_str());
}

void TaskScheduler::removeScene(SceneType scene_type) 
{
    std::string scene_name = sceneTypeToString(scene_type);
    if (scene_type == SceneType::IDLE) {
        LOG_WARN("无法删除默认的 'idle' 场景");
        return;
    }
    std::lock_guard<std::mutex> lock(scenes_mutex_);
    scenes_.erase(scene_type);
    LOG_INFO("场景已移除: %s", scene_name.c_str());
}

void TaskScheduler::updateScene(const SceneConfig& scene_config) 
{
    if (scene_config.name.empty()) 
    {
        LOG_WARN("无法更新空名称的场景");
        return;
    } 
    std::lock_guard<std::mutex> lock(scenes_mutex_);
    if (!validateSceneConfig(scene_config)) 
    {
        LOG_WARN("无效的场景配置: %s", scene_config.name.c_str());
        return;
    }
    scenes_[scene_config.type] = scene_config;
    LOG_INFO("场景已更新: %s", scene_config.name.c_str());
}

SceneConfig TaskScheduler::getSceneConfig(SceneType scene_type) const 
{
    std::lock_guard<std::mutex> lock(scenes_mutex_);
    auto it = scenes_.find(scene_type);
    if (it != scenes_.end()) {
        return it->second;
    }
    return SceneConfig();
}

void TaskScheduler::registerSceneCallback(std::function<void(const std::string&, const std::vector<std::string>&)> callback) 
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    scene_callbacks_.push_back(callback);
}

std::vector<std::string> TaskScheduler::getActiveModules() const 
{
    std::lock_guard<std::mutex> lock(scenes_mutex_);
    return active_modules_;
}

bool TaskScheduler::isModuleActiveInCurrentScene(const std::string& module_name) const 
{
    std::lock_guard<std::mutex> lock(scenes_mutex_);
    return std::find(active_modules_.begin(), active_modules_.end(), module_name) != active_modules_.end();
}

std::map<SceneType, bool> TaskScheduler::getModuleSceneStatus(const std::string& module_name) const 
{
    std::lock_guard<std::mutex> lock(scenes_mutex_);
    std::map<SceneType, bool> scene_status;
    
    for (const auto& pair : scenes_) 
    {
        const auto& scene_type = pair.first;
        const auto& scene_config = pair.second;
        bool is_active = std::find(scene_config.active_modules.begin(), scene_config.active_modules.end(), module_name) != scene_config.active_modules.end();
        scene_status[scene_type] = is_active;
    }
    
    return scene_status;
}

void TaskScheduler::setLaunchManager(LaunchMgr* launch_manager) 
{
    launch_manager_ = launch_manager;
    LOG_INFO("启动管理器已设置");
}

void TaskScheduler::setStatusMonitor(StatusMonitor* status_monitor) 
{
    status_monitor_ = status_monitor;
    LOG_INFO("状态监控器已设置");
}

std::vector<std::pair<SceneType, std::chrono::system_clock::time_point>> 
TaskScheduler::getSceneSwitchHistory(size_t max_entries) const 
{
    std::lock_guard<std::mutex> lock(history_mutex_);
    std::vector<std::pair<SceneType, std::chrono::system_clock::time_point>> history;
    size_t start_index = (scene_history_.size() > max_entries) ? (scene_history_.size() - max_entries) : 0;
    
    for (size_t i = start_index; i < scene_history_.size(); ++i) {
        history.push_back(scene_history_[i]);
    }
    return history;
}

std::map<SceneType, size_t> TaskScheduler::getSceneSwitchStats() const 
{
    std::lock_guard<std::mutex> lock(history_mutex_);
    return scene_switch_stats_;
}

bool TaskScheduler::executeSceneSwitch(SceneType scene_type) 
{
    // 清空旧场景的模块相机状态，避免状态残留
    if (status_monitor_) {
        status_monitor_->clearAllModuleCamStatus();
    }
    std::vector<std::string> current_modules = active_modules_;
    std::vector<std::string> target_modules;
    {
        std::lock_guard<std::mutex> lock(scenes_mutex_);
        auto it = scenes_.find(scene_type);
        if (it != scenes_.end()) {
            target_modules = it->second.active_modules;
        }
    }
    
    //计算需要启动和停止的模块
    std::vector<std::string> modules_to_start = getModulesToStart(target_modules, current_modules);
    std::vector<std::string> modules_to_stop = getModulesToStop(target_modules, current_modules);
    
    LOG_INFO("需要启动的模块数: %zu", modules_to_start.size());
    LOG_INFO("需要停止的模块数: %zu", modules_to_stop.size());
    
    //停止不需要的模块
    size_t stopped_count = stopUnnecessaryModules(modules_to_stop);
    if (stopped_count < modules_to_stop.size()) {
        LOG_WARN("部分模块停止失败，继续执行场景切换");
    }
    
    //启动需要的模块
    size_t started_count = startRequiredModules(modules_to_start);
    if (started_count < modules_to_start.size()) 
    {
        LOG_WARN("部分模块启动失败");
        return false;
    }
    
    return true;
}

size_t TaskScheduler::startRequiredModules(const std::vector<std::string>& modules_to_start) 
{
    if (modules_to_start.empty() || !launch_manager_) {
        return 0;
    }
    //按依赖关系排序启动顺序
    std::vector<std::string> startup_order = getModuleStartupOrder(modules_to_start);
    size_t success_count = 0;
    for (const auto& module_name : startup_order) 
    {
        if (launch_manager_->startModule(module_name)) 
        {
            success_count++;
            LOG_INFO("模块启动成功: %s", module_name.c_str());
        } else {
            LOG_WARN("模块启动失败: %s", module_name.c_str());
        }
    }
    return success_count;
}

size_t TaskScheduler::stopUnnecessaryModules(const std::vector<std::string>& modules_to_stop) 
{
    if (modules_to_stop.empty() || !launch_manager_) {
        return 0;
    }
    //按逆序停止模块
    std::vector<std::string> stop_order = modules_to_stop;
    std::reverse(stop_order.begin(), stop_order.end());
    size_t success_count = 0;
    for (const auto& module_name : stop_order) 
    {
        if (launch_manager_->stopModule(module_name)) 
        {
            success_count++;
            LOG_INFO("模块停止成功: %s", module_name.c_str());
        } else {
            LOG_WARN("模块停止失败: %s", module_name.c_str());
        }
    } 
    return success_count;
}

std::vector<std::string> TaskScheduler::getModulesToStart(const std::vector<std::string>& target_modules,
    const std::vector<std::string>& current_modules) const 
{
    std::vector<std::string> modules_to_start;
    for (const auto& module : target_modules) 
    {
        if (std::find(current_modules.begin(), current_modules.end(), module) == current_modules.end()) {
            modules_to_start.push_back(module);
        }
    }
    return modules_to_start;
}

std::vector<std::string> TaskScheduler::getModulesToStop(const std::vector<std::string>& target_modules,
    const std::vector<std::string>& current_modules) const 
{
    std::vector<std::string> modules_to_stop;
    for (const auto& module : current_modules) 
    {
        if (std::find(target_modules.begin(), target_modules.end(), module) == target_modules.end()) {
            modules_to_stop.push_back(module);
        }
    }
    
    return modules_to_stop;
}

bool TaskScheduler::validateSceneConfig(const SceneConfig& scene_config) const 
{
    //检查场景名称
    if (scene_config.name.empty()) {
        return false;
    }
    //检查模块列表
    if (scene_config.active_modules.empty()) {
        LOG_WARN("场景必须至少有一个活动模块: %s", scene_config.name.c_str());
        return false;
    }
    //检查模块名称有效性（简单检查）
    for (const auto& module_name : scene_config.active_modules) 
    {
        if (module_name.empty()) {
            LOG_WARN("场景中存在无效的空模块名: %s", scene_config.name.c_str());
            return false;
        }
    }
    return true;
}

void TaskScheduler::triggerSceneCallback(SceneType scene_type, const std::vector<std::string>& active_modules) 
{
    std::string scene_name = sceneTypeToString(scene_type);
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    for (const auto& callback : scene_callbacks_) 
    {
        try {
            callback(scene_name, active_modules);
        } catch (const std::exception& e) {
            LOG_WARN("场景回调异常: %s", e.what());
        }
    }
}

void TaskScheduler::recordSceneSwitch(SceneType scene_type) 
{
    std::lock_guard<std::mutex> lock(history_mutex_);
    scene_history_.emplace_back(scene_type, std::chrono::system_clock::now());
    //限制历史记录大小
    if (scene_history_.size() > 100) {
        scene_history_.erase(scene_history_.begin());
    }
}

std::vector<std::string> TaskScheduler::getModuleStartupOrder(const std::vector<std::string>& modules) const 
{
    //简单实现：返回原始顺序
    //实际应用中应该根据模块依赖关系进行排序
    return modules;
}

} // namespace bas_control
