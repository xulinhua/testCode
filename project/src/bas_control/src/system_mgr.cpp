/**
 * @file system_mgr.cpp
 * @brief 系统管理器实现文件
 * 
 * 功能概述：
 * SystemMgr类是统一部署控制层的核心协调器，负责整合各组件功能，提供统一的系统管理接口：
 * 1. 系统生命周期管理：初始化、启动、停止、重启整个视觉系统
 * 2. 组件协调：管理LaunchMgr、StatusMonitor、TaskScheduler、ConfigHotUpdater等组件
 * 3. 配置管理：加载和解析YAML配置文件，统一管理系统配置
 * 4. 状态聚合：聚合各组件状态，提供统一的系统状态视图
 * 5. 接口整合：提供统一的API接口，简化上层应用调用
 * 
 * 主要接口说明：
 * 1. 系统控制：
 *    - initialize()：初始化系统，创建各组件并建立连接
 *    - start()：启动系统，启动所有核心服务和模块
 *    - stop()：停止系统，安全停止所有服务和模块
 *    - restart()：重启系统（先停止再启动）
 * 
 * 2. 状态查询：
 *    - getSystemStatus()：获取当前系统状态（整体状态、模块状态、资源状态）
 *    - getCurrentScene()：获取当前活跃场景名称
 *    - getAvailableScenes()：获取所有可用场景列表
 *    - getRunningModules()：获取当前运行中的模块列表
 *    - isSystemHealthy()：检查系统是否健康
 *    - getSystemResource()：获取系统资源使用情况
 * 
 * 3. 配置管理：
 *    - switchScene()：切换到指定场景，调整活跃模块集合
 *    - updateModuleConfig()：更新指定模块的参数配置
 *    - updateModuleModel()：更新指定模块的模型文件
 *    - setLogLevel()：设置系统日志级别
 *    - getConfig()：获取当前配置参数
 * 
 * 4. 系统信息：
 *    - getSystemStats()：获取系统统计信息（运行时间、场景、模块数等）
 *    - getSystemInfo()：获取系统详细信息（版本、平台、运行状态等）
 *    - performSelfCheck()：执行系统自检，检查各组件状态
 * 
 * 5. 回调注册：
 *    - registerSystemStatusCallback()：注册系统状态变化回调
 *    - registerSceneCallback()：注册场景切换回调
 *    - registerModuleStatusCallback()：注册模块状态变化回调
 * 
 * 实现机制：
 * 1. 组件管理：使用unique_ptr管理各组件生命周期，确保资源正确释放
 * 2. 配置解析：使用yaml-cpp库解析YAML配置文件，支持复杂配置结构
 * 3. 状态聚合：通过回调函数接收各组件状态更新，聚合为统一系统状态
 * 4. 线程安全：使用mgr_mutex_保护系统状态数据，callbacks_mutex_保护回调函数
 * 5. 错误处理：各组件操作异常时记录错误日志，尽量保持系统可用性
 * 
 * 初始化流程：
 * 1. 加载配置文件（如果提供）：解析YAML配置，填充ConfigParams
 * 2. 初始化组件：创建LaunchMgr、StatusMonitor、TaskScheduler、ConfigHotUpdater
 * 3. 建立组件连接：设置组件间依赖关系，注册回调函数
 * 4. 标记为已初始化，更新系统状态为STOPPED
 * 
 * 启动流程：
 * 1. 检查系统是否已初始化，未初始化则返回失败
 * 2. 启动核心服务：
 *    - 启动StatusMonitor监控线程
 *    - 启动ConfigHotUpdater配置监控
 *    - 通过LaunchMgr启动所有模块
 * 3. 标记为运行状态，更新系统状态为RUNNING
 * 
 * 停止流程：
 * 1. 通过LaunchMgr停止所有模块
 * 2. 停止ConfigHotUpdater配置监控
 * 3. 停止StatusMonitor状态监控
 * 4. 标记为停止状态，更新系统状态为STOPPED
 * 
 * 配置解析：
 * 1. 系统配置：config_path（配置文件路径）
 * 2. 监控配置：health_check_interval, status_report_interval, 各种阈值
 * 3. 日志配置：level, path, max_size
 * 4. 模块配置：startup_order, dependencies
 * 5. 场景配置：scenes（名称、活动模块、描述）
 * 
 * 使用场景：
 * 1. 系统部署：统一管理视觉系统的初始化和启动
 * 2. 运维监控：提供统一的状态查询接口，简化监控系统开发
 * 3. 任务调度：通过场景切换动态调整视觉功能配置
 * 4. 配置管理：集中管理系统配置，支持热更新和回滚
 * 5. 故障处理：通过自检功能快速定位故障组件
 * 
 * 注意事项：
 * 1. 配置文件需符合YAML格式，路径需可访问
 * 2. 组件初始化顺序重要，需确保依赖关系正确
 * 3. 系统启动前需确保所有依赖服务可用
 * 4. 回调函数应避免长时间阻塞，否则影响系统响应性
 * 5. 资源清理需彻底，防止内存泄漏和资源残留
 */

#include "bas_control/system_mgr.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include "log_system/log_macros.hpp"

namespace bas_control {

SystemMgr::SystemMgr(const std::string& config_file)
    : config_file_path_(config_file), initialized_(false), running_(false) 
{
    startup_time_ = std::chrono::system_clock::now();
    //如果提供了配置文件路径，则加载配置
    if (!config_file.empty()) {
        loadConfig(config_file);
    }
    LOG_INFO("系统管理器已创建");
}

SystemMgr::~SystemMgr() 
{
    if (running_) {
        stop();
    }
    LOG_INFO("系统管理器已销毁");
}

bool SystemMgr::initialize() 
{
    if (initialized_) 
    {
        LOG_WARN("系统已初始化");
        return false;
    } 
    LOG_INFO("正在初始化系统...");
    //初始化各组件
    if (!initializeComponents()) 
    {
        LOG_WARN("组件初始化失败");
        return false;
    } 
    //建立组件间连接
    if (!setupComponentConnections()) 
    {
        LOG_WARN("组件连接设置失败");
        return false;
    }  
    initialized_ = true;
    current_status_.overall_status = ModuleStatus::STOPPED;
    current_status_.timestamp = std::chrono::steady_clock::now();
    LOG_INFO("系统初始化成功");
    return true;
}

bool SystemMgr::start() 
{
    if (!initialized_) 
    {
        LOG_WARN("系统未初始化");
        return false;
    }
    if (running_) 
    {
        LOG_WARN("系统已在运行中");
        return false;
    } 
    LOG_INFO("正在启动系统...");
    //启动核心服务
    if (!startCoreServices()) 
    {
        LOG_WARN("核心服务启动失败");
        return false;
    }
    running_ = true;
    current_status_.overall_status = ModuleStatus::RUNNING;
    current_status_.timestamp = std::chrono::steady_clock::now();
    LOG_INFO("系统启动成功");
    return true;
}

bool SystemMgr::stop() 
{
    if (!running_) {
        return true; //已经停止
    }
    LOG_INFO("正在停止系统...");
    //停止核心服务
    if (!stopCoreServices()) {
        LOG_WARN("核心服务停止失败");
        return false;
    }
    running_ = false;
    current_status_.overall_status = ModuleStatus::STOPPED;
    current_status_.timestamp = std::chrono::steady_clock::now();
    LOG_INFO("系统停止成功");
    return true;
}

bool SystemMgr::restart() 
{
    LOG_INFO("正在重启系统...");
    if (!stop()) 
    {
        LOG_WARN("重启时系统停止失败");
        return false;
    }
    basmodule::sleep_ms(1000); //等待1秒
    if (!start()) 
    {
        LOG_WARN("重启后系统启动失败");
        return false;
    } 
    LOG_INFO("系统重启成功");
    return true;
}

SystemStatus SystemMgr::getSystemStatus() const 
{
    std::lock_guard<std::mutex> lock(mgr_mutex_);
    
    if (status_monitor_) {
        return status_monitor_->getCurrentStatus();
    }
    
    return current_status_;
}

SceneType SystemMgr::getCurrentScene() const 
{
    if (task_scheduler_) {
        return task_scheduler_->getCurrentScene();
    }
    return SceneType::UNKNOWN;
}

bool SystemMgr::switchScene(SceneType scene_type) 
{
    if (!task_scheduler_) {
        LOG_WARN("任务调度器不可用");
        return false;
    }
    
    return task_scheduler_->switchScene(scene_type);
}

std::vector<SceneType> SystemMgr::getAvailableScenes() const 
{
    if (task_scheduler_) {
        return task_scheduler_->getAvailableScenes();
    }
    return {};
}

//更新指定模块的参数配置(1.配置热更新 (ConfigHotUpdater) 内部调用 - 当监测到配置文件变化时，自动触发参数更新；2.其他内部模块 直接调用)
bool SystemMgr::updateModuleConfig(const std::string& module_name, const std::map<std::string, std::string>& params) 
{
    if (!config_updater_) 
    {
        LOG_WARN("配置更新器不可用");
        return false;
    }
    return config_updater_->updateParams(module_name, params);
}

bool SystemMgr::updateModuleModel(const std::string& module_name, const std::string& model_path) 
{
    if (!config_updater_) 
    {
        LOG_WARN("配置更新器不可用");
        return false;
    }
    return config_updater_->updateModel(module_name, model_path);
}

std::vector<std::string> SystemMgr::getRunningModules() const 
{
    if (launch_mgr_) {
        return launch_mgr_->getRunningModules();
    }
    return {};
}

bool SystemMgr::isSystemHealthy() const 
{
    if (status_monitor_) {
        return status_monitor_->isSystemHealthy();
    }
    std::lock_guard<std::mutex> lock(mgr_mutex_);
    return current_status_.isHealthy();
}

SystemResource SystemMgr::getSystemResource() const 
{
    if (status_monitor_) {
        return status_monitor_->getSystemResource();
    }
    std::lock_guard<std::mutex> lock(mgr_mutex_);
    return current_status_.resource_usage;
}

void SystemMgr::registerSystemStatusCallback(std::function<void(const SystemStatus&)> callback) 
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    status_callbacks_.push_back(callback);
}

void SystemMgr::registerSceneCallback(std::function<void(const std::string&, const std::vector<std::string>&)> callback) 
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    scene_callbacks_.push_back(callback);
}

void SystemMgr::registerModuleStatusCallback(std::function<void(const ModuleInfo&)> callback) 
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    module_callbacks_.push_back(callback);
}

std::map<std::string, std::string> SystemMgr::getSystemStats() const 
{
    std::map<std::string, std::string> stats;
    stats["uptime"] = getUptime();
    stats["startup_time"] = basmodule::format_timestamp(startup_time_);
    stats["current_scene"] = sceneTypeToString(getCurrentScene());
    stats["running_modules"] = std::to_string(getRunningModules().size());
    stats["system_status"] = getSystemStatus().getStatusString();
    stats["healthy"] = isSystemHealthy() ? "true" : "false";
    if (status_monitor_) 
    {
        const auto& resource = status_monitor_->getSystemResource();
        stats["cpu_usage"] = std::to_string(resource.cpu_usage) + "%";
        stats["memory_usage"] = std::to_string(resource.memory_usage) + "%";
        stats["gpu_usage"] = std::to_string(resource.gpu_usage) + "%";
        stats["temperature"] = std::to_string(resource.temperature) + "°C";
    }
    return stats;
}

std::map<std::string, bool> SystemMgr::performSelfCheck() const 
{
    std::map<std::string, bool> results;
    results["launch_manager"] = (launch_mgr_ != nullptr);
    results["status_monitor"] = (status_monitor_ != nullptr);
    results["task_scheduler"] = (task_scheduler_ != nullptr);
    results["config_updater"] = (config_updater_ != nullptr);
    results["initialized"] = initialized_;
    results["running"] = running_;
    
    //检查各组件是否正常工作
    if (launch_mgr_) {
        results["launch_manager_status"] = (launch_mgr_->getStatus() != ModuleStatus::ERROR);
    }
    if (status_monitor_) {
        results["status_monitor_active"] = true; //假设监控器正常运行
    }
    return results;
}

std::map<std::string, std::string> SystemMgr::getSystemInfo() const 
{
    std::map<std::string, std::string> info;
    info["version"] = "1.0.0";
    info["platform"] = "Jetson Orin NX";
    info["os"] = "Ubuntu 22.04";
    info["ros_version"] = "Humble";
    info["startup_time"] = basmodule::format_timestamp(startup_time_);
    info["uptime"] = getUptime();
    info["current_scene"] = sceneTypeToString(getCurrentScene());
    auto stats = getSystemStats();
    for (const auto& pair : stats) 
    {
        info["stats_" + pair.first] = pair.second;
    }
    return info;
}

ConfigParams SystemMgr::getConfig() const {
    return config_;
}

bool SystemMgr::loadConfig(const std::string& config_file) 
{
    try 
    {
        YAML::Node config = YAML::LoadFile(config_file);
        LOG_INFO("加载配置文件%s成功！", config_file.c_str());
        return parseYamlConfig(config);
    } 
    catch (const YAML::Exception& e) 
    {
        LOG_WARN("加载配置文件失败: %s", e.what());
        return false;
    } 
    catch (const std::exception& e) 
    {
        LOG_WARN("加载配置文件失败: %s", e.what());
        return false;
    }
}

bool SystemMgr::parseYamlConfig(const YAML::Node& yaml_node) 
{
    try 
    {
        //解析系统配置
        if (yaml_node["system"]) 
        {
            const auto& system_node = yaml_node["system"];
            config_.config_file_path = basmodule::get_param_from_yaml<std::string>(system_node, "config_path", "/config/bas_control");
            // 解析默认启动场景
            config_.default_scene = basmodule::get_param_from_yaml<std::string>(system_node, "default_scene", "idle");
            LOG_INFO("  default_scene: %s", config_.default_scene.c_str());
        }
        
        //解析监控配置
        if (yaml_node["monitoring"]) 
        {
            const auto& monitoring_node = yaml_node["monitoring"];
            config_.health_check_interval_ms = basmodule::get_param_from_yaml<int>(monitoring_node, "health_check_interval", 5000);
            config_.status_report_interval_ms = basmodule::get_param_from_yaml<int>(monitoring_node, "status_report_interval", 1000);
            config_.cpu_threshold_warning = basmodule::get_param_from_yaml<float>(monitoring_node, "cpu_threshold_warning", 80.0f);
            config_.memory_threshold_warning = basmodule::get_param_from_yaml<float>(monitoring_node, "memory_threshold_warning", 85.0f);
        }
        
        //解析模块配置
        if (yaml_node["modules"]) 
        {
            const auto& modules_node = yaml_node["modules"];
            //启动顺序
            if (modules_node["startup_order"]) 
            {
                const auto& order_node = modules_node["startup_order"];
                for (const auto& module : order_node) {
                    config_.startup_order.push_back(module.as<std::string>());
                }
            }
            //依赖关系
            if (modules_node["dependencies"]) 
            {
                const auto& deps_node = modules_node["dependencies"];
                for (const auto& module_node : deps_node) 
                {
                    std::string module_name = module_node.first.as<std::string>();
                    std::vector<std::string> dependencies;
                    for (const auto& dep : module_node.second) 
                    {
                        dependencies.push_back(dep.as<std::string>());
                    }
                    config_.dependencies[module_name] = dependencies;
                }
            }
        }
        
        //解析场景配置
        if (yaml_node["scenes"]) 
        {
            const auto& scenes_node = yaml_node["scenes"];
            for (const auto& it_node : scenes_node) 
            {
                std::string scene_name = it_node.first.as<std::string>();
                SceneConfig scene_config;
                scene_config.name = scene_name;
                scene_config.type = stringToSceneType(scene_name);  // ⭐ 关键：设置场景类型枚举
                const auto& scene_data = it_node.second;
                if (scene_data["modules"]) 
                {
                    for (const auto& module : scene_data["modules"]) {
                        scene_config.active_modules.push_back(module.as<std::string>());
                    }
                }
                if (scene_data["description"]) {
                    scene_config.description = scene_data["description"].as<std::string>();
                }
                config_.scenes[scene_name] = scene_config;
            }
        }
        
        // 打印加载后的配置参数信息
        printConfigParams(config_);
        
        return true;
    } catch (const std::exception& e) {
        LOG_WARN("解析YAML配置失败: %s", e.what());
        return false;
    }
}

bool SystemMgr::initializeComponents() 
{
    try 
    {
        //创建启动管理器
        launch_mgr_ = std::make_unique<LaunchMgr>(config_);
        //创建状态监控器
        status_monitor_ = std::make_unique<StatusMonitor>(config_);
        //创建任务调度器
        task_scheduler_ = std::make_unique<TaskScheduler>(config_);
        //创建配置热更新器
        config_updater_ = std::make_unique<ConfigHotUpdater>(config_); 
        return true;
    } catch (const std::exception& e) {
        LOG_WARN("组件初始化失败: %s", e.what());
        return false;
    }
}

bool SystemMgr::setupComponentConnections() 
{
    try 
    {
        //设置任务调度器的依赖
        if (task_scheduler_ && launch_mgr_) {
            task_scheduler_->setLaunchManager(launch_mgr_.get());
        } 
        if (task_scheduler_ && status_monitor_) {
            task_scheduler_->setStatusMonitor(status_monitor_.get());
        }
        
        // 【关键】将 LaunchMgr 的模块状态变化同步到 StatusMonitor
        // 这样 StatusMonitor 才能监控进程是否存活
        if (launch_mgr_ && status_monitor_) {
            launch_mgr_->registerSystemStatusCallback(
                [this](const ModuleInfo& module_info) {
                    if (status_monitor_) {
                        status_monitor_->updateModuleStatus(module_info.name, module_info);
                    }
                });
        }
        
        //注册回调函数
        if (status_monitor_) 
        {
            status_monitor_->registerSystemStatusCallback(
                [this](const SystemStatus& status) {
                    this->onSystemStatusUpdate(status);
                });
            
            status_monitor_->registerModuleStatusCallback(
                [this](const ModuleInfo& module_info) {
                    this->onModuleStatusUpdate(module_info);
                });
            
            status_monitor_->registerResourceCallback(
                [this](const SystemResource& resource) {
                    this->onResourceUpdate(resource);
                });
        }
        if (task_scheduler_) 
        {
            task_scheduler_->registerSceneCallback(
                [this](const std::string& scene_name, const std::vector<std::string>& active_modules) {
                    this->onSceneSwitch(scene_name, active_modules);
                });
        }
        return true;
    } catch (const std::exception& e) {
        LOG_WARN("设置组件连接失败: %s", e.what());
        return false;
    }
}

bool SystemMgr::startCoreServices() 
{
    try 
    {
        // 启动状态监控（优先启动，以便监控后续模块）
        if (status_monitor_ && !status_monitor_->startMonitoring()) {
            LOG_WARN("状态监控器启动失败");
            return false;
        }
        
        // 启动配置监控
        if (config_updater_ && !config_updater_->startMonitoring()) {
            LOG_WARN("配置更新器启动失败");
            return false;
        }
        
        // 根据当前场景启动模块
        if (task_scheduler_ && launch_mgr_) {
            std::vector<std::string> active_modules = task_scheduler_->getActiveModules();
            LOG_INFO("根据当前场景启动模块，共 %zu 个", active_modules.size());
            
            if (!launch_mgr_->startModules(active_modules)) {
                LOG_WARN("启动场景模块失败");
                return false;
            }
        } else {
            // 如果任务调度器不可用，启动所有模块（兜底方案）
            LOG_WARN("任务调度器不可用，启动所有模块");
            if (launch_mgr_ && !launch_mgr_->startAllModules()) {
                LOG_WARN("启动所有模块失败");
                return false;
            }
        }
        
        return true;
    } catch (const std::exception& e) {
        LOG_WARN("核心服务启动失败: %s", e.what());
        return false;
    }
}

bool SystemMgr::stopCoreServices() 
{
    try 
    {
        bool success = true;
        //停止所有模块
        if (launch_mgr_ && !launch_mgr_->stopAllModules()) {
            LOG_WARN("停止所有模块失败");
            success = false;
        }
        //停止配置监控
        if (config_updater_ && !config_updater_->stopMonitoring()) {
            LOG_WARN("停止配置更新器失败");
            success = false;
        }
        //停止状态监控
        if (status_monitor_ && !status_monitor_->stopMonitoring()) {
            LOG_WARN("停止状态监控器失败");
            success = false;
        }
        return success;
    } catch (const std::exception& e) {
        LOG_WARN("核心服务停止失败: %s", e.what());
        return false;
    }
}

void SystemMgr::onSystemStatusUpdate(const SystemStatus& status) 
{
    std::lock_guard<std::mutex> lock(mgr_mutex_);
    current_status_ = status;
    //触发回调
    std::lock_guard<std::mutex> callback_lock(callbacks_mutex_);
    for (const auto& callback : status_callbacks_) 
    {
        try {
            callback(status);
        } catch (const std::exception& e) {
            LOG_WARN("系统状态回调异常: %s", e.what());
        }
    }
}

void SystemMgr::onModuleStatusUpdate(const ModuleInfo& module_info) 
{
    //更新系统状态中的模块信息
    {
        std::lock_guard<std::mutex> lock(mgr_mutex_);
        current_status_.modules[module_info.name] = module_info;
    }
    //触发回调
    std::lock_guard<std::mutex> callback_lock(callbacks_mutex_);
    for (const auto& callback : module_callbacks_) 
    {
        try {
            callback(module_info);
        } catch (const std::exception& e) {
            LOG_WARN("模块状态回调异常: %s", e.what());
        }
    }
}

void SystemMgr::onSceneSwitch(const std::string& scene_name, const std::vector<std::string>& active_modules) 
{
    LOG_INFO("场景已切换到: %s", scene_name.c_str());
    //触发回调
    std::lock_guard<std::mutex> callback_lock(callbacks_mutex_);
    for (const auto& callback : scene_callbacks_) 
    {
        try {
            callback(scene_name, active_modules);
        } catch (const std::exception& e) {
            LOG_WARN("场景回调异常: %s", e.what());
        }
    }
}

void SystemMgr::onResourceUpdate(const SystemResource& resource) 
{
    std::lock_guard<std::mutex> lock(mgr_mutex_);
    current_status_.resource_usage = resource;
}

std::chrono::system_clock::time_point SystemMgr::getStartupTime() const 
{
    return startup_time_;
}

std::string SystemMgr::getUptime() const 
{
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - startup_time_);
    auto seconds = duration.count();
    int hours = seconds / 3600;
    int minutes = (seconds % 3600) / 60;
    int secs = seconds % 60;
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << hours << ":"
        << std::setfill('0') << std::setw(2) << minutes << ":"
        << std::setfill('0') << std::setw(2) << secs;
    return oss.str();
}

void SystemMgr::updateModuleCamStatus(const std::string& module_name, int cam_id, const basros::ModuleStatusInfo& status_info) 
{
    if (status_monitor_) {
        status_monitor_->updateModuleCamStatus(module_name, cam_id, status_info);
    }
}

basros::ModuleStatusInfo SystemMgr::getModuleCamStatus(const std::string& module_name, int cam_id) const 
{
    if (status_monitor_) {
        return status_monitor_->getModuleCamStatus(module_name, cam_id);
    }
    return basros::ModuleStatusInfo(module_name, cam_id, basros::ModuleStatus::UNKNOWN);
}

void printConfigParams(const ConfigParams& config)
{
    LOG_INFO("========== 配置参数加载完成 ==========");
    bool bShowExInfo = false;
    logsys::Color color = logsys::Color::BLUE;
    
    // 系统配置
    LOG_INFO(bShowExInfo, color, "【系统配置】");
    LOG_INFO(bShowExInfo, color, "  config_file_path: %s", config.config_file_path.c_str());
    LOG_INFO(bShowExInfo, color, "  default_scene: %s", config.default_scene.c_str());
    
    // 显示默认场景对应的模块
    auto scene_it = config.scenes.find(config.default_scene);
    if (scene_it != config.scenes.end()) 
    {
        std::string modules_str;
        for (size_t i = 0; i < scene_it->second.active_modules.size(); ++i) 
        {
            modules_str += scene_it->second.active_modules[i];
            if (i < scene_it->second.active_modules.size() - 1) modules_str += ", ";
        }
        LOG_INFO(bShowExInfo, color, "  default_scene 模块: [%s] - %s", 
                 modules_str.c_str(), scene_it->second.description.c_str());
    } 
    else 
    {
        LOG_INFO(bShowExInfo, color, "  default_scene 模块: [场景未配置]");
    }
    
    // 监控配置
    LOG_INFO(bShowExInfo, color, "【监控配置】");
    LOG_INFO(bShowExInfo, color, "  health_check_interval_ms: %d", config.health_check_interval_ms);
    LOG_INFO(bShowExInfo, color, "  status_report_interval_ms: %d", config.status_report_interval_ms);
    LOG_INFO(bShowExInfo, color, "  cpu_threshold_warning: %.1f%%", config.cpu_threshold_warning);
    LOG_INFO(bShowExInfo, color, "  memory_threshold_warning: %.1f%%", config.memory_threshold_warning);
    LOG_INFO(bShowExInfo, color, "  gpu_threshold_warning: %.1f%%", config.gpu_threshold_warning);
    LOG_INFO(bShowExInfo, color, "  temperature_threshold_warning: %.1f°C", config.temperature_threshold_warning);
    
    // 模块配置
    LOG_INFO(bShowExInfo, color, "【模块配置】");
    LOG_INFO(bShowExInfo, color, "  startup_order: 共 %zu 个模块（全局启动顺序，用于依赖解析）", config.startup_order.size());
    std::string startup_order_str;
    for (size_t i = 0; i < config.startup_order.size(); ++i) 
    {
        startup_order_str += config.startup_order[i];
        if (i < config.startup_order.size() - 1) startup_order_str += " -> ";
    }
    LOG_INFO(bShowExInfo, color, "    全局顺序: [%s]", startup_order_str.c_str());
    
    LOG_INFO(bShowExInfo, color, "  dependencies: 共 %zu 个模块有依赖", config.dependencies.size());
    for (const auto& pair : config.dependencies) 
    {
        std::string deps_str;
        for (size_t i = 0; i < pair.second.size(); ++i) 
        {
            deps_str += pair.second[i];
            if (i < pair.second.size() - 1) deps_str += ", ";
        }
        LOG_INFO(bShowExInfo, color, "    %s: [%s]", pair.first.c_str(), deps_str.c_str());
    }
    
    // 场景配置
    LOG_INFO(bShowExInfo, color, "【场景配置】");
    LOG_INFO(bShowExInfo, color, "  scenes: 共 %zu 个场景", config.scenes.size());
    for (const auto& pair : config.scenes) 
    {
        const auto& scene_config = pair.second;
        std::string modules_str;
        for (size_t i = 0; i < scene_config.active_modules.size(); ++i) 
        {
            modules_str += scene_config.active_modules[i];
            if (i < scene_config.active_modules.size() - 1) modules_str += ", ";
        }
        LOG_INFO(bShowExInfo, color, "    %s: [%s] - %s", 
                 pair.first.c_str(), modules_str.c_str(), scene_config.description.c_str());
    }
    
    LOG_INFO("========================================");
}

} // namespace bas_control
