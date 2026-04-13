/**
 * @file status_monitor.cpp
 * @brief 状态监控器实现文件
 * 
 * 功能概述：
 * StatusMonitor类负责实时监控系统状态和资源使用情况，提供健康检查和状态报告功能：
 * 1. 系统状态监控：跟踪整体系统状态（运行、停止、错误等）和各模块运行状态
 * 2. 资源监控：实时监测CPU、内存、GPU使用率和系统温度
 * 3. 健康检查：定期执行系统健康检查，检测故障模块和资源异常
 * 4. 状态报告：定期发布系统状态信息，支持实时监控和历史分析
 * 5. 阈值告警：监控资源使用阈值，超限时发出警告日志
 * 
 * 主要接口说明：
 * 1. 监控控制：
 *    - startMonitoring()：启动监控线程，开始定期检查
 *    - stopMonitoring()：停止监控，释放资源
 * 
 * 2. 状态查询：
 *    - getCurrentStatus()：获取当前系统状态（包含模块状态和资源状态）
 *    - getModuleStatus()：获取指定模块的详细状态信息
 *    - getSystemResource()：获取系统资源使用情况
 *    - isSystemHealthy()：检查系统是否健康（无错误且资源正常）
 *    - getRunningModuleCount()：获取运行中的模块数量
 * 
 * 3. 错误处理：
 *    - hasModuleErrors()：检查是否有模块处于错误状态
 *    - getErrorModules()：获取所有错误模块的列表
 * 
 * 4. 配置管理：
 *    - setHealthCheckInterval()：设置健康检查间隔时间
 *    - setStatusReportInterval()：设置状态报告间隔时间
 * 
 * 5. 回调注册：
 *    - registerSystemStatusCallback()：注册系统状态变化回调函数
 *    - registerModuleStatusCallback()：注册模块状态变化回调函数
 *    - registerResourceCallback()：注册资源状态变化回调函数
 * 
 * 6. 状态更新：
 *    - updateModuleStatus()：手动更新模块状态信息（用于外部状态同步）
 * 
 * 实现机制：
 * 1. 使用独立监控线程定期执行健康检查和资源检查
 * 2. 多级锁机制：status_mutex_保护状态数据，callbacks_mutex_保护回调函数
 * 3. 模块心跳检查：通过最后心跳时间判断模块是否存活
 * 4. 进程资源监控：获取各模块进程的CPU和内存使用情况
 * 5. 阈值管理：配置CPU、内存、GPU、温度等资源的警告阈值
 * 
 * 监控流程（监控线程循环）：
 * 1. 检查是否达到健康检查间隔，是则执行performHealthCheck()
 *    - 检查系统资源：CPU、内存、GPU、温度
 *    - 检查模块状态：进程存活、心跳超时
 *    - 检查资源阈值：超限则记录警告日志
 * 2. 检查是否达到状态报告间隔，是则触发状态回调
 * 3. 睡眠100ms后继续下一轮检查
 * 
 * 健康检查内容：
 * 1. 系统资源检查：
 *    - CPU使用率：通过basmodule::get_cpu_usage()获取
 *    - 内存使用率：通过basmodule::get_memory_usage()获取
 *    - GPU使用率：模拟获取（实际应用中需硬件支持）
 *    - 系统温度：通过basmodule::get_system_temperature()获取
 * 2. 模块状态检查：
 *    - 进程存活：通过basmodule::is_process_alive()检查
 *    - 心跳超时：检查最后心跳时间是否超过5秒阈值
 *    - 资源使用：获取进程的CPU和内存使用率
 * 
 * 使用场景：
 * 1. 系统监控：实时监控系统运行状态，及时发现故障
 * 2. 性能分析：分析资源使用情况，优化系统配置
 * 3. 故障诊断：定位错误模块，辅助故障排除
 * 4. 资源管理：监控资源使用趋势，预测容量需求
 * 5. 告警通知：资源超限时触发告警，及时处理
 * 
 * 注意事项：
 * 1. 监控间隔不宜过短，避免过度消耗系统资源
 * 2. 阈值设置需考虑硬件特性和应用场景
 * 3. 心跳超时时间需根据模块特性调整
 * 4. 回调函数应避免长时间阻塞，否则影响监控及时性
 * 5. 资源监控数据可能有延迟，需注意时效性
 */

#include "bas_control/status_monitor.hpp"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include "log_system/log_macros.hpp"

namespace bas_control {

/**
 * @brief StatusMonitor构造函数
 * 
 * @details 初始化状态监控器，设置配置参数和初始状态。
 * 构造函数会初始化内部状态变量，包括当前系统状态、最后检查时间等。
 * 
 * @param config 配置参数，包含健康检查间隔、状态报告间隔、各种阈值等
 * 
 * @note 构造函数不会启动监控线程，需要调用startMonitoring()来启动监控
 * @note 初始系统状态设置为UNKNOWN，等待后续状态更新
 */
StatusMonitor::StatusMonitor(const ConfigParams& config)
    : config_(config), monitoring_active_(false), 
      health_check_interval_ms_(config.health_check_interval_ms),
      status_report_interval_ms_(config.status_report_interval_ms) 
{
    current_status_.overall_status = ModuleStatus::UNKNOWN;
    current_status_.timestamp = std::chrono::steady_clock::now();
    last_health_check_ = std::chrono::steady_clock::now();
    last_status_report_ = std::chrono::steady_clock::now(); 
    LOG_INFO("状态监控器已初始化");
}

StatusMonitor::~StatusMonitor() 
{
    stopMonitoring();
}

/**
 * @brief 启动状态监控线程
 * 
 * @details 该方法创建并启动一个独立的监控线程，该线程会定期执行健康检查和状态报告。
 * 监控线程会持续运行，直到调用stopMonitoring()方法停止。
 * 
 * @return true 如果监控成功启动
 * @return false 如果监控已经在运行中
 * 
 * @note 此方法是线程安全的，内部使用monitoring_active_标志避免重复启动
 * @note 启动后监控线程会立即开始执行监控循环
 * @note 如果监控启动失败，会记录相应日志但不抛出异常
 */
bool StatusMonitor::startMonitoring() 
{
    if (monitoring_active_) 
    {
        LOG_INFO("监控已处于活动状态");
        return false;
    }
    monitoring_active_ = true;
    monitoring_thread_ = std::make_unique<std::thread>(&StatusMonitor::monitoringLoop, this);
    LOG_INFO("状态监控已启动");
    return true;
}

/**
 * @brief 停止状态监控线程
 * 
 * @details 该方法停止监控线程并释放相关资源。首先设置停止标志，然后等待监控线程正常退出。
 * 如果监控线程已经在停止状态，则直接返回成功。
 * 
 * @return true 停止操作执行成功（无论监控是否已经在停止状态）
 * @return false 停止操作执行失败（通常不会发生）
 * 
 * @note 此方法是线程安全的，通过monitoring_active_标志协调线程停止
 * @note 方法会等待监控线程完全退出，可能需要一定时间
 * @note 停止后可以重新调用startMonitoring()重新启动监控
 */
bool StatusMonitor::stopMonitoring() 
{
    if (!monitoring_active_) {
        return true;
    }
    monitoring_active_ = false;
    if (monitoring_thread_ && monitoring_thread_->joinable()) {
        monitoring_thread_->join();
    }
    monitoring_thread_.reset();
    LOG_INFO("状态监控已停止");
    return true;
}

/**
 * @brief 获取当前系统状态
 * 
 * @details 该方法返回包含系统整体状态、各模块状态和资源使用情况的完整状态信息。
 * 状态数据通过内部锁保护，确保读取时的一致性。
 * 
 * @return SystemStatus 当前系统状态对象
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 返回的状态对象是调用时刻的快照，可能不是实时最新数据
 * @note 如果需要实时状态更新，建议注册状态回调函数
 */
SystemStatus StatusMonitor::getCurrentStatus() const 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_;
}

/**
 * @brief 获取指定模块的状态信息
 * 
 * @details 该方法查询并返回指定模块的详细状态信息，包括模块状态、PID、心跳时间等。
 * 如果模块不存在，则返回一个默认构造的ModuleInfo对象，状态为UNKNOWN。
 * 
 * @param module_name 要查询的模块名称
 * @return ModuleInfo 模块状态信息对象
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 如果模块不存在，返回的ModuleInfo对象会使用传入的模块名进行初始化
 * @note 调用者应检查返回对象的有效性（如status字段是否为UNKNOWN）
 */
ModuleInfo StatusMonitor::getModuleStatus(const std::string& module_name) const 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    auto it = current_status_.modules.find(module_name);
    if (it != current_status_.modules.end()) {
        return it->second;
    }
    return ModuleInfo(module_name);
}

/**
 * @brief 获取系统资源使用情况
 * 
 * @details 该方法返回当前系统的资源使用情况，包括CPU、内存、GPU使用率和系统温度。
 * 资源数据通过内部锁保护，确保读取时的一致性。
 * 
 * @return SystemResource 系统资源使用情况对象
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 返回的资源数据是调用时刻的快照，可能不是实时最新数据
 * @note 资源数据可能有延迟，具体取决于监控间隔和系统负载
 */
SystemResource StatusMonitor::getSystemResource() const 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_.resource_usage;
}

/**
 * @brief 注册系统状态变化回调函数
 * 
 * @details 该方法注册一个回调函数，当系统状态发生变化时会调用该函数。
 * 回调函数会在监控线程中调用，因此应避免长时间阻塞。
 * 
 * @param callback 回调函数，接收SystemStatus参数
 * 
 * @note 此方法是线程安全的，内部使用callbacks_mutex_保护回调列表
 * @note 回调函数会在监控线程中调用，应避免执行耗时操作
 * @note 同一个回调函数可以多次注册，会多次调用
 * @note 目前没有提供取消注册的机制，回调函数会一直有效直到对象销毁
 */
void StatusMonitor::registerSystemStatusCallback(std::function<void(const SystemStatus&)> callback) 
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    status_callbacks_.push_back(callback);
}

/**
 * @brief 注册模块状态变化回调函数
 * 
 * @details 该方法注册一个回调函数，当任意模块状态发生变化时会调用该函数。
 * 回调函数会在监控线程中调用，因此应避免长时间阻塞。
 * 
 * @param callback 回调函数，接收ModuleInfo参数
 * 
 * @note 此方法是线程安全的，内部使用callbacks_mutex_保护回调列表
 * @note 回调函数会在监控线程中调用，应避免执行耗时操作
 * @note 同一个回调函数可以多次注册，会多次调用
 * @note 目前没有提供取消注册的机制，回调函数会一直有效直到对象销毁
 */
void StatusMonitor::registerModuleStatusCallback(std::function<void(const ModuleInfo&)> callback) 
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    module_callbacks_.push_back(callback);
}

/**
 * @brief 注册资源状态变化回调函数
 * 
 * @details 该方法注册一个回调函数，当系统资源状态发生变化时会调用该函数。
 * 回调函数会在监控线程中调用，因此应避免长时间阻塞。
 * 
 * @param callback 回调函数，接收SystemResource参数
 * 
 * @note 此方法是线程安全的，内部使用callbacks_mutex_保护回调列表
 * @note 回调函数会在监控线程中调用，应避免执行耗时操作
 * @note 同一个回调函数可以多次注册，会多次调用
 * @note 目前没有提供取消注册的机制，回调函数会一直有效直到对象销毁
 */
void StatusMonitor::registerResourceCallback(std::function<void(const SystemResource&)> callback) 
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    resource_callbacks_.push_back(callback);
}

/**
 * @brief 更新指定模块的状态信息
 * 
 * @details 此方法用于手动更新模块状态信息，通常由外部模块调用以同步状态。
 * 它会更新模块状态映射，并根据模块运行状态调整活动模块列表。
 * 同时重新计算系统整体状态（UNKNOWN/STARTING/RUNNING/ERROR）。
 * 
 * @param module_name 模块名称，用于标识要更新的模块
 * @param module_info 模块信息，包含模块状态、PID、心跳时间等详细信息
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护状态数据
 * @note 更新后会重新计算系统整体状态，可能触发状态回调
 * @note 如果模块处于运行状态，会被添加到活动模块列表；否则从列表中移除
 */
void StatusMonitor::updateModuleStatus(const std::string& module_name, const ModuleInfo& module_info) 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_.modules[module_name] = module_info;
    // 更新活动模块列表
    auto it = std::find(current_status_.active_modules.begin(), current_status_.active_modules.end(), module_name);
    if (module_info.isRunning()) 
    {
        if (it == current_status_.active_modules.end()) {
            current_status_.active_modules.push_back(module_name);
        }
    } 
    else 
    {
        if (it != current_status_.active_modules.end()) {
            current_status_.active_modules.erase(it);
        }
    }
    
    //更新整体状态
    if (current_status_.modules.empty()) 
    {
        current_status_.overall_status = ModuleStatus::UNKNOWN;
    } 
    else 
    {
        bool all_running = true;
        bool any_error = false;
        
        for (const auto& pair : current_status_.modules) 
        {
            if (pair.second.hasError()) 
            {
                any_error = true;
                break;
            }
            if (!pair.second.isRunning()) 
            {
                all_running = false;
            }
        }
        
        if (any_error) {
            current_status_.overall_status = ModuleStatus::ERROR;
        } else if (all_running) {
            current_status_.overall_status = ModuleStatus::RUNNING;
        } else {
            current_status_.overall_status = ModuleStatus::STARTING;
        }
    }
    
    current_status_.timestamp = std::chrono::steady_clock::now();
}

void StatusMonitor::monitoringLoop() 
{
    LOG_INFO("监控循环已启动");
    
    while (monitoring_active_) 
    {
        auto current_time = std::chrono::steady_clock::now();
        //执行健康检查
        if (basmodule::duration_ms(last_health_check_, current_time) >= health_check_interval_ms_) 
        {
            performHealthCheck();
            last_health_check_ = current_time;
        }
        //状态上报
        if (basmodule::duration_ms(last_status_report_, current_time) >= status_report_interval_ms_) 
        {
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                current_status_.timestamp = current_time;
                triggerStatusCallback(current_status_);
            }
            last_status_report_ = current_time;
        }
        basmodule::sleep_ms(100); // 100ms检查间隔
    }
    LOG_INFO("监控循环已停止");
}

void StatusMonitor::performHealthCheck() 
{
    //检查系统资源
    checkSystemResources();
    //检查模块状态
    checkModuleStatus();
    //检查资源阈值
    if (!checkResourceThresholds()) {
        LOG_WARN("系统资源阈值超限");
    }
}

void StatusMonitor::checkSystemResources() 
{
    SystemResource resource;
    // 获取CPU使用率
    resource.cpu_usage = basmodule::get_cpu_usage(); 
    // 获取内存使用情况
    uint64_t used_memory, total_memory;
    basmodule::get_memory_usage(used_memory, total_memory, resource.memory_usage);
    resource.available_memory = total_memory - used_memory;
    resource.total_memory = total_memory; 
    // 获取系统温度
    resource.temperature = basmodule::get_system_temperature();
    // 模拟GPU使用率（实际应用中需要根据具体硬件实现）
    resource.gpu_usage = resource.cpu_usage * 0.8f; //简单模拟
    // 更新状态
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_.resource_usage = resource;
    }
    //触发资源回调
    triggerResourceCallback(resource);
}

void StatusMonitor::checkModuleStatus() 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    for (auto& pair : current_status_.modules) 
    {
        ModuleInfo& module_info = pair.second;
        
        // 只要有 PID 就检查进程存活（无论当前状态是什么）
        // 这样可以检测到被 kill -9 的进程，即使之前状态是 ERROR
        if (module_info.pid > 0) 
        {
            // 检查进程是否存活
            if (!basmodule::is_process_alive(module_info.pid)) 
            {
                LOG_WARN("模块进程未存活: %s (PID: %d)", module_info.name.c_str(), module_info.pid);
                module_info.status = ModuleStatus::CRASHED;
                module_info.error_message = "Process terminated unexpectedly";
                module_info.pid = -1;  // 清除无效的 PID
            }
            else if (module_info.isRunning())
            {
                // 进程存活且状态为运行中，检查心跳
                if (!checkModuleHeartbeat(module_info)) 
                {
                    LOG_WARN("模块心跳超时: %s", module_info.name.c_str());
                    module_info.status = ModuleStatus::ERROR;
                    module_info.error_message = "心跳超时";
                }
                
                // 获取进程资源使用情况
                float cpu_usage, memory_usage;
                getProcessResourceUsage(module_info.pid, cpu_usage, memory_usage);
            }
        }
        //触发模块状态回调
        triggerModuleStatusCallback(module_info);
    }
}

bool StatusMonitor::checkModuleHeartbeat(const ModuleInfo& module_info) 
{
    //最后心跳时间
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = basmodule::duration_ms(module_info.last_heartbeat, now);
    //心跳超时为5秒
    return elapsed_ms < 5000;
}

bool StatusMonitor::checkResourceThresholds() 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    const auto& resource = current_status_.resource_usage;
    bool within_thresholds = true;
    if (resource.cpu_usage > config_.cpu_threshold_warning) {
        LOG_WARN("CPU使用率阈值超限: %.1f%%", resource.cpu_usage);
        within_thresholds = false;
    }
    if (resource.memory_usage > config_.memory_threshold_warning) {
        LOG_WARN("内存使用率阈值超限: %.1f%%", resource.memory_usage);
        within_thresholds = false;
    }
    if (resource.gpu_usage > config_.gpu_threshold_warning) {
        LOG_WARN("GPU使用率阈值超限: %.1f%%", resource.gpu_usage);
        within_thresholds = false;
    }
    if (resource.temperature > config_.temperature_threshold_warning) {
        LOG_WARN("温度阈值超限: %.1f°C", resource.temperature);
        within_thresholds = false;
    }
    return within_thresholds;
}

/**
 * @brief 检查系统是否健康
 * 
 * @details 该方法检查当前系统是否存在错误状态或资源异常。
 * 系统健康的判定标准：没有模块处于错误状态，且所有资源使用率在正常阈值范围内。
 * 
 * @return true 如果系统健康（无错误，资源正常）
 * @return false 如果系统存在任何问题
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 返回的是调用时刻的健康状态，可能不是实时最新状态
 * @note 健康检查结果会受到监控间隔的影响
 */
bool StatusMonitor::isSystemHealthy() const 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_.isHealthy();
}

/**
 * @brief 获取运行中的模块数量
 * 
 * @details 该方法统计当前处于运行状态（RUNNING）的模块数量。
 * 统计基于最新的模块状态信息，但不包括启动中、停止中或错误状态的模块。
 * 
 * @return size_t 运行中的模块数量
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 返回的是调用时刻的统计结果，可能不是实时最新数据
 * @note 统计结果可以用于监控系统负载和性能分析
 */
size_t StatusMonitor::getRunningModuleCount() const 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_.getRunningModuleCount();
}

/**
 * @brief 检查是否存在模块错误
 * 
 * @details 该方法遍历所有模块状态，检查是否有模块处于错误状态（ERROR或CRASHED）。
 * 只要有一个模块处于错误状态，就返回true。
 * 
 * @return true 如果存在至少一个错误模块
 * @return false 如果所有模块都正常（没有错误状态）
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 返回的是调用时刻的错误状态，可能不是实时最新数据
 * @note 此方法不提供具体错误信息，仅返回布尔结果
 */
bool StatusMonitor::hasModuleErrors() const 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    for (const auto& pair : current_status_.modules) {
        if (pair.second.hasError()) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 获取所有错误模块的名称列表
 * 
 * @details 该方法收集所有处于错误状态（ERROR或CRASHED）的模块名称，并以向量形式返回。
 * 返回的列表按模块名称的自然顺序排列（基于map的迭代顺序）。
 * 
 * @return std::vector<std::string> 错误模块名称列表，如果没有任何错误模块则返回空向量
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 返回的是调用时刻的错误模块列表，可能不是实时最新数据
 * @note 调用者可以结合getModuleStatus()获取具体错误信息
 */
std::vector<std::string> StatusMonitor::getErrorModules() const 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    std::vector<std::string> error_modules;
    for (const auto& pair : current_status_.modules) 
    {
        if (pair.second.hasError()) {
            error_modules.push_back(pair.first);
        }
    }
    return error_modules;
}

/**
 * @brief 设置健康检查间隔时间
 * 
 * @details 该方法设置健康检查的执行间隔时间（毫秒）。监控线程会根据此间隔定期执行健康检查。
 * 间隔时间越短，监控越及时，但也会增加系统开销。
 * 
 * @param interval_ms 健康检查间隔时间（毫秒），必须大于0
 * 
 * @note 此方法不是线程安全的，建议在监控启动前调用或在监控线程外安全调用
 * @note 间隔时间设置过小可能导致监控线程占用过多CPU资源
 * @note 实际执行间隔可能略大于设定值，取决于系统负载和调度
 */
void StatusMonitor::setHealthCheckInterval(int interval_ms) 
{
    health_check_interval_ms_ = interval_ms;
}

/**
 * @brief 设置状态报告间隔时间
 * 
 * @details 该方法设置状态报告的执行间隔时间（毫秒）。监控线程会根据此间隔定期触发状态回调函数。
 * 状态报告间隔通常短于健康检查间隔，以便及时反馈系统状态变化。
 * 
 * @param interval_ms 状态报告间隔时间（毫秒），必须大于0
 * 
 * @note 此方法不是线程安全的，建议在监控启动前调用或在监控线程外安全调用
 * @note 间隔时间设置过小可能导致回调函数频繁调用，增加系统开销
 * @note 实际执行间隔可能略大于设定值，取决于系统负载和调度
 */
void StatusMonitor::setStatusReportInterval(int interval_ms) 
{
    status_report_interval_ms_ = interval_ms;
}

/**
 * @brief 更新模块的相机状态信息
 * 
 * @details 该方法用于更新指定模块的指定相机的状态信息。使用两层映射：模块名->(相机ID->状态信息)。
 * 这是支持多相机模块的关键方法，每个模块可以有多个子节点对应不同的相机。
 * 
 * @param module_name 模块名称，如 "marker_detect_ros"
 * @param cam_id 相机ID，-1表示主节点，>=0表示子节点
 * @param status_info 模块状态信息，包含状态和状态消息
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 更新后会覆盖之前的状态信息
 * @note 调用者应确保模块名称和相机ID的有效性
 */
void StatusMonitor::updateModuleCamStatus(const std::string& module_name, int cam_id, 
                                              const basros::ModuleStatusInfo& status_info) 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    module_cam_status_map_[module_name][cam_id] = status_info;
    LOG_DEBUG("更新模块相机状态: %s, 相机ID: %d, 状态: %s", 
             module_name.c_str(), cam_id, status_info.getStatusString().c_str());
}

/**
 * @brief 获取模块的相机状态信息
 * 
 * @details 该方法查询并返回指定模块的指定相机的状态信息。如果不存在，返回默认构造的对象。
 * 
 * @param module_name 模块名称
 * @param cam_id 相机ID
 * @return basros::ModuleStatusInfo 模块状态信息对象
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 如果状态不存在，返回的对象使用传入的模块名和相机ID初始化
 */
basros::ModuleStatusInfo StatusMonitor::getModuleCamStatus(const std::string& module_name, int cam_id) const 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    auto it_mdl = module_cam_status_map_.find(module_name);
    if (it_mdl != module_cam_status_map_.end()) 
    {
        auto it_cam = it_mdl->second.find(cam_id);
        if (it_cam != it_mdl->second.end()) {
            return it_cam->second;
        }
    }
    return basros::ModuleStatusInfo(module_name, cam_id, basros::ModuleStatus::UNKNOWN);
}

/**
 * @brief 获取模块所有相机的状态信息
 * 
 * @details 该方法查询并返回指定模块所有相机的状态信息。返回一个映射表，键为相机ID，值为状态信息。
 * 
 * @param module_name 模块名称
 * @return std::map<int, basros::ModuleStatusInfo> 相机状态信息映射表
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 如果模块没有任何相机状态，返回空映射表
 */
std::map<int, basros::ModuleStatusInfo> StatusMonitor::getModuleAllCamStatus(
    const std::string& module_name) const 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    auto it = module_cam_status_map_.find(module_name);
    if (it != module_cam_status_map_.end()) {
        return it->second;
    }
    return std::map<int, basros::ModuleStatusInfo>();
}

/**
 * @brief 获取所有模块所有相机的状态信息
 * 
 * @details 该方法返回所有模块所有相机的状态信息映射表。
 * 
 * @return basros::ModuleStatusInfoMap 所有模块所有相机的状态信息映射表
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 返回的是当前状态的快照，可能不是实时最新数据
 */
basros::ModuleStatusInfoMap StatusMonitor::getAllModuleCamStatus() const 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return module_cam_status_map_;
}

/**
 * @brief 清空所有模块相机状态
 * 
 * @details 该方法清空module_cam_status_map_中的所有状态信息。
 * 通常在场景切换时调用，避免旧场景的模块状态残留影响新场景的状态查询。
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 清空后，在新的状态更新之前，所有相机状态查询将返回UNKNOWN
 */
void StatusMonitor::clearAllModuleCamStatus() 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    module_cam_status_map_.clear();
    LOG_INFO("已清空所有模块相机状态");
}

/**
 * @brief 清空指定模块的相机状态
 * 
 * @details 该方法清空指定模块的所有相机状态信息。
 * 用于模块停止或移除时清理该模块的状态缓存。
 * 
 * @param module_name 模块名称
 * 
 * @note 此方法是线程安全的，内部使用status_mutex_保护共享数据
 * @note 如果模块不存在，该方法不会有任何效果
 */
void StatusMonitor::clearModuleCamStatus(const std::string& module_name) 
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    auto it = module_cam_status_map_.find(module_name);
    if (it != module_cam_status_map_.end()) {
        module_cam_status_map_.erase(it);
        LOG_INFO("已清空模块相机状态: %s", module_name.c_str());
    }
}

void StatusMonitor::triggerStatusCallback(const SystemStatus& status) 
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    for (const auto& callback : status_callbacks_) 
    {
        try {
            callback(status);
        } catch (const std::exception& e) {
            LOG_WARN("状态回调异常: %s", e.what());
        }
    }
}

void StatusMonitor::triggerModuleStatusCallback(const ModuleInfo& module_info) 
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    for (const auto& callback : module_callbacks_) 
    {
        try {
            callback(module_info);
        } catch (const std::exception& e) {
            LOG_WARN("模块状态回调异常: %s", e.what());
        }
    }
}

void StatusMonitor::triggerResourceCallback(const SystemResource& resource) 
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    for (const auto& callback : resource_callbacks_) 
    {
        try {
            callback(resource);
        } catch (const std::exception& e) {
            LOG_WARN("资源回调异常: %s", e.what());
        }
    }
}

void StatusMonitor::getProcessResourceUsage(int pid, float& cpu_usage, float& memory_usage) const 
{
    // 模拟进程资源使用情况
    // 实际应用中需要根据具体操作系统实现
    cpu_usage = static_cast<float>(rand() % 50); // 0-50% CPU
    memory_usage = static_cast<float>(rand() % 30); // 0-30%内存
}

} // namespace bas_control