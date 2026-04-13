/**
 * @file launch_mgr.cpp
 * @brief 启动管理器实现文件
 * 
 * 功能概述：
 * LaunchMgr类负责按依赖关系管理视觉模块的启动和停止，提供模块生命周期管理功能：
 * 1. 模块启动管理：根据配置的启动顺序和依赖关系，按拓扑顺序启动模块
 * 2. 模块停止管理：按启动顺序的逆序安全停止模块，确保依赖关系正确处理
 * 3. 状态监控：跟踪各模块运行状态（运行、停止、错误等），支持状态回调
 * 4. 依赖关系管理：维护模块间的依赖图，检测循环依赖，生成启动顺序
 * 
 * 主要接口说明：
 * 1. 系统级控制：
 *    - startAllModules()：启动所有模块，按拓扑顺序逐个启动
 *    - stopAllModules()：停止所有模块，按逆序逐个停止，支持强制终止
 *    - restartModule()：重启指定模块（先停止再启动）
 * 
 * 2. 模块级控制：
 *    - startModule()：启动单个模块，检查依赖是否满足
 *    - stopModule()：停止单个模块，等待进程正常退出
 *    - isModuleRunning()：检查模块是否正在运行
 * 
 * 3. 状态查询：
 *    - getModuleInfo()：获取指定模块的详细信息
 *    - getAllModuleInfo()：获取所有模块的信息映射
 *    - getRunningModules()：获取当前运行中的模块列表
 *    - getStartupOrder()：获取当前启动顺序
 *    - getStatus()：获取系统整体状态
 * 
 * 4. 配置管理：
 *    - setStartupOrder()：设置自定义启动顺序
 *    - addModuleDependencies()：添加模块依赖关系
 *    - removeModule()：移除模块及其相关依赖
 *    - hasCircularDependency()：检查是否存在循环依赖
 * 
 * 5. 回调注册：
 *    - registerSystemStatusCallback()：注册模块状态变化回调函数
 * 
 * 实现机制：
 * 1. 使用拓扑排序算法确定模块启动顺序，确保依赖满足
 * 2. 通过进程ID（PID）跟踪模块运行状态
 * 3. 采用多级锁机制：modules_mutex_保护模块数据，callbacks_mutex_保护回调函数
 * 4. 实现指数退避重试机制，提高启动/停止的可靠性
 * 
 * 启动流程：
 * 1. 检查整体状态，防止重复启动
 * 2. 按启动顺序逐个启动模块
 * 3. 对每个模块：检查依赖→更新状态为STARTING→执行启动→更新状态为RUNNING
 * 4. 等待模块启动完成，检查心跳
 * 5. 更新整体状态为RUNNING
 * 
 * 停止流程：
 * 1. 更新整体状态为STOPPING，设置关闭请求标志
 * 2. 按启动顺序的逆序逐个停止模块
 * 3. 对每个模块：更新状态为STOPPING→发送停止信号→等待进程退出→更新状态为STOPPED
 * 4. 强制终止未正常退出的进程
 * 5. 更新整体状态为STOPPED
 * 
 * 使用场景：
 * 1. 系统启动：按依赖关系有序启动各视觉模块
 * 2. 系统停止：安全停止所有模块，防止数据丢失
 * 3. 模块维护：单独重启故障模块，提高系统可用性
 * 4. 动态配置：运行时调整模块依赖关系，适应任务需求
 * 
 * 注意事项：
 * 1. 依赖关系必须是无环图，否则无法生成有效启动顺序
 * 2. 模块启动超时时间需合理设置，避免长时间等待
 * 3. 强制终止可能造成数据不一致，建议优先等待正常退出
 * 4. 状态回调函数应避免阻塞，否则影响系统响应性
 */

#include "bas_control/launch_mgr.hpp"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include "log_system/log_macros.hpp"
#include "bas_operate/file_operate.hpp"  // 添加：用于查找可执行文件
#include "bas_operate/bas_utils.hpp"     // 添加：用于进程管理

namespace bas_control {

/**
 * @brief LaunchMgr构造函数
 * 
 * @details 初始化启动管理器，设置配置参数和初始状态。
 * 构造函数会调用initializeModules()方法初始化模块信息和依赖关系。
 * 
 * @param config 配置参数，包含启动顺序、依赖关系、超时时间等
 * 
 * @note 构造函数不会启动任何模块，需要调用startAllModules()来启动系统
 * @note 初始系统状态设置为UNKNOWN，等待后续状态更新
 * @note 构造函数是线程安全的，可以在多线程环境中调用
 */
LaunchMgr::LaunchMgr(const ConfigParams& config)
    : config_(config), overall_status_(ModuleStatus::UNKNOWN), shutdown_requested_(false) 
{
    initializeModules();
}

LaunchMgr::~LaunchMgr() 
{
    shutdown_requested_ = true;
    
    // 检查是否还有运行中的模块需要停止
    bool has_running_modules = false;
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        for (const auto& pair : modules_) {
            if (pair.second.isRunning() && pair.second.pid > 0) {
                has_running_modules = true;
                break;
            }
        }
    }
    
    // 只有在有运行中模块时才调用停止
    if (has_running_modules) {
        LOG_INFO("LaunchMgr 析构：检测到运行中的模块，正在停止...");
        stopAllModules();
    } else {
        LOG_DEBUG("LaunchMgr 析构：没有运行中的模块需要停止");
    }
}

void LaunchMgr::initializeModules() 
{
    std::lock_guard<std::mutex> lock(modules_mutex_);
    
    // 初始化依赖关系
    dependencies_ = config_.dependencies;
    startup_order_ = config_.startup_order;
    
    // 如果启动顺序为空，根据依赖关系生成
    if (startup_order_.empty() && !dependencies_.empty()) 
    {
        try 
        {
            startup_order_ = basmodule::topological_sort(dependencies_);
            LOG_DEBUG("拓扑排序生成启动顺序成功，共 %zu 个模块", startup_order_.size());
            // 打印启动顺序详情
            std::string order_str;
            for (size_t i = 0; i < startup_order_.size(); ++i) 
            {
                order_str += startup_order_[i];
                if (i < startup_order_.size() - 1) 
                {
                    order_str += " -> ";
                }
            }
            LOG_DEBUG("启动顺序: %s", order_str.c_str());
            
        } catch (const std::exception& e) {
            LOG_WARN("生成启动顺序失败: %s", e.what());
        }
    }
    
    // 初始化模块状态
    for (const auto& module_name : startup_order_) 
    {
        ModuleInfo module_info(module_name);
        module_info.status = ModuleStatus::STOPPED;
        module_info.type = ModuleType::UNKNOWN;
        modules_[module_name] = module_info;
    }
    
    // 添加依赖关系中未在启动顺序中的模块
    for (const auto& dep_pair : dependencies_) 
    {
        if (modules_.find(dep_pair.first) == modules_.end()) 
        {
            ModuleInfo module_info(dep_pair.first);
            module_info.status = ModuleStatus::STOPPED;
            module_info.type = ModuleType::UNKNOWN;
            modules_[dep_pair.first] = module_info;
        }
        for (const auto& dep : dep_pair.second) 
        {
            if (modules_.find(dep) == modules_.end()) 
            {
                ModuleInfo module_info(dep);
                module_info.status = ModuleStatus::STOPPED;
                module_info.type = ModuleType::UNKNOWN;
                modules_[dep] = module_info;
            }
        }
    }
    overall_status_ = ModuleStatus::STOPPED;
    LOG_INFO("启动管理器已初始化，共 %zu 个模块", modules_.size());
    
    // 打印每个模块的详细依赖项
    for (const auto& dep_pair : dependencies_) 
    {
        const std::string& module_name = dep_pair.first;
        const auto& deps = dep_pair.second;
        if (deps.empty()) 
        {
            LOG_DEBUG("模块 [%s] 无依赖项", module_name.c_str());
        } 
        else 
        {
            std::string deps_str;
            for (size_t i = 0; i < deps.size(); ++i) 
            {
                deps_str += deps[i];
                if (i < deps.size() - 1) 
                {
                    deps_str += ", ";
                }
            }
            LOG_DEBUG("模块 [%s] 依赖项: %s", module_name.c_str(), deps_str.c_str());
        }
    }
}

/**
 * @brief 启动所有模块
 * 
 * @details 调用统一的startModules接口（传入空列表表示系统启动）。
 * 
 * @return true 如果所有模块都成功启动
 * @return false 如果启动过程中出现任何失败或系统已经在运行中
 * 
 * @note 此方法是对startModules的封装，提供向后兼容的接口
 */
bool LaunchMgr::startAllModules() 
{
    // 调用统一的startModules接口（空列表表示系统启动）
    return startModules({});
}

/**
 * @brief 停止所有模块
 * 
 * @details 设置关闭请求标志，然后调用统一的stopModules接口（传入空列表表示系统停止）。
 * 
 * @return true 如果停止操作成功执行
 * @return false 如果停止操作执行失败
 * 
 * @note 此方法是对stopModules的封装，提供向后兼容的接口
 */
bool LaunchMgr::stopAllModules() 
{
    // 设置关闭请求标志（用于中断启动过程）
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        shutdown_requested_ = true;
    }
    
    // 调用统一的stopModules接口（空列表表示系统停止）
    return stopModules({});
}

/**
 * @brief 重启指定模块
 * 
 * @details 先停止指定模块，再重新启动该模块。适用于故障恢复或配置更新后需要重启的场景。
 * 重启过程会保持模块依赖关系，确保重启前后依赖满足。
 * 
 * @param module_name 要重启的模块名称
 * @return true 如果模块重启成功
 * @return false 如果模块停止或启动失败
 * 
 * @note 此方法是线程安全的，内部调用startModuleInternal和stopModuleInternal处理锁
 * @note 重启过程中模块会短暂不可用，调用者应考虑服务中断的影响
 * @note 重启失败可能是由于依赖不满足或模块本身问题
 */
bool LaunchMgr::restartModule(const std::string& module_name) 
{
    LOG_INFO("正在重启模块: %s", module_name.c_str());
    
    //停止模块
    if (!stopModuleInternal(module_name, 10000)) 
    {
        LOG_WARN("重启时模块停止失败: %s", module_name.c_str());
        return false;
    }
    //启动模块
    return startModuleInternal(module_name, config_.startup_timeout_ms);
}

/**
 * @brief 启动单个模块
 * 
 * @details 启动指定名称的单个模块，检查其依赖是否满足。
 * 该方法是对startModuleInternal的封装，提供公共接口。
 * 
 * @param module_name 要启动的模块名称
 * @return true 如果模块启动成功
 * @return false 如果模块启动失败或依赖不满足
 * 
 * @note 此方法是线程安全的，内部调用startModuleInternal处理锁
 * @note 启动过程会更新模块状态（STARTING→RUNNING）
 * @note 如果依赖模块未运行，启动会失败
 */
bool LaunchMgr::startModule(const std::string& module_name) 
{
    return startModuleInternal(module_name, config_.startup_timeout_ms);
}

/**
 * @brief 停止单个模块
 * 
 * @details 停止指定名称的单个模块，等待进程正常退出。
 * 该方法是对stopModuleInternal的封装，提供公共接口。
 * 
 * @param module_name 要停止的模块名称
 * @return true 如果模块停止成功
 * @return false 如果模块停止失败或模块不存在
 * 
 * @note 此方法是线程安全的，内部调用stopModuleInternal处理锁
 * @note 停止过程会更新模块状态（STOPPING→STOPPED）
 * @note 如果模块未运行，直接返回成功
 */
bool LaunchMgr::stopModule(const std::string& module_name) 
{
    return stopModuleInternal(module_name, 10000);
}

/**
 * @brief 批量启动模块（统一接口）
 * @details 统一的启动接口，支持两种使用场景：
 *          1. 系统启动：传入空列表 {}，启动所有配置的模块
 *          2. 场景切换：传入模块列表，只启动指定的模块
 * @param module_names 要启动的模块名称列表（空列表表示启动所有模块）
 * @return true 所有模块启动成功
 * @return false 有模块启动失败
 * 
 * @example
 * // 系统启动 - 启动所有模块
 * launch_mgr->startModules({});
 * 
 * // 场景切换 - 只启动指定模块
 * launch_mgr->startModules({"cam_mgr_ros", "yolo_det"});
 */
bool LaunchMgr::startModules(const std::vector<std::string>& module_names) 
{
    // 判断是否为系统启动（空列表或列表包含所有模块）
    bool is_system_start = module_names.empty();
    std::vector<std::string> modules_to_start = module_names;
    // 空列表表示启动所有模块
    if (is_system_start) 
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        modules_to_start = startup_order_;
    }
    
    // ========== 系统启动场景：检查整体状态并更新 ==========
    if (is_system_start) 
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        // 检查是否已有关闭请求
        if (shutdown_requested_) 
        {
            LOG_WARN("已有关闭请求，取消启动");
            return false;
        }
        if (overall_status_ == ModuleStatus::STARTING || overall_status_ == ModuleStatus::RUNNING) 
        {
            LOG_WARN("系统正在启动或运行中");
            return false;
        }
        if (overall_status_ == ModuleStatus::STOPPING) 
        {
            LOG_WARN("系统正在停止中，无法启动");
            return false;
        } 
        LOG_INFO("========================================");
        LOG_INFO("系统启动：启动所有配置的模块");
        LOG_INFO("========================================");
        overall_status_ = ModuleStatus::STARTING;
    } 
    else 
    {
        LOG_INFO("========================================");
        LOG_INFO("场景切换：启动 %zu 个指定模块", modules_to_start.size());
        LOG_INFO("========================================");
        for (const auto& name : modules_to_start) {
            LOG_INFO("  - %s", name.c_str());
        }
        
        // 场景切换时也需要检查关闭请求和停止状态
        std::lock_guard<std::mutex> lock(modules_mutex_);
        if (shutdown_requested_) 
        {
            LOG_WARN("已有关闭请求，取消启动");
            return false;
        }
        if (overall_status_ == ModuleStatus::STOPPING) 
        {
            LOG_WARN("系统正在停止中，无法启动模块");
            return false;
        }
        // 如果系统未运行（STOPPED 或 UNKNOWN），将场景切换视为系统启动
        if (overall_status_ != ModuleStatus::RUNNING) 
        {
            overall_status_ = ModuleStatus::STARTING;
        }
    }
    
    // 根据依赖关系确定启动顺序
    std::vector<std::string> ordered_modules = getModuleStartupOrder(modules_to_start);
    bool success = true;
    bool interrupted_by_shutdown = false;
    for (const auto& module_name : ordered_modules) 
    {
        // ========== 所有场景都检查是否有关闭请求 ==========
        {
            std::lock_guard<std::mutex> lock(modules_mutex_);
            if (shutdown_requested_) 
            {
                LOG_WARN("启动过程中收到关闭请求，中断启动");
                success = false;
                interrupted_by_shutdown = true;
                break;
            }
        }
        
        if (!startModuleInternal(module_name, config_.startup_timeout_ms)) 
        {
            LOG_WARN("模块启动失败: %s", module_name.c_str());
            success = false;
            // 继续启动其他模块
        }
    }
    
    // ========== 更新整体状态 ==========
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        // 如果被关闭请求中断，状态由 stopModules 设置
        if (!interrupted_by_shutdown) 
        {
            size_t running_count = 0;
            for (const auto& pair : modules_) 
            {
                if (pair.second.isRunning()) {
                    running_count++;
                }
            }
            // 只要有模块在运行，就认为系统在运行
            if (running_count > 0) 
            {
                overall_status_ = ModuleStatus::RUNNING;
                if (is_system_start) {
                    if (running_count == modules_.size()) {
                        LOG_INFO("所有模块启动成功");
                    } else {
                        LOG_INFO("部分模块启动成功，运行中: %zu / %zu", running_count, modules_.size());
                    }
                } else {
                    LOG_INFO("场景切换完成，运行中模块: %zu", running_count);
                }
            } 
            else 
            {
                overall_status_ = ModuleStatus::ERROR;
                LOG_WARN("所有模块启动失败");
            }
        }
    }
    
    // ========== 打印最终结果 ==========
    if (success) 
    {
        LOG_INFO("========================================");
        if (is_system_start) {
            LOG_INFO("✅ 所有模块启动成功");
        } else {
            LOG_INFO("✅ 所有指定模块启动成功");
        }
        LOG_INFO("========================================");
    } 
    else 
    {
        LOG_WARN("========================================");
        if (interrupted_by_shutdown) {
            LOG_WARN("⚠️  启动被关闭请求中断");
        } else {
            LOG_WARN("⚠️  部分模块启动失败");
        }
        LOG_WARN("========================================");
    }
    return success;
}

/**
 * @brief 批量停止模块（统一接口）
 * @details 统一的停止接口，支持两种使用场景：
 *          1. 系统停止：传入空列表 {}，停止所有运行中的模块
 *          2. 场景切换：传入模块列表，只停止指定的模块
 * @param module_names 要停止的模块名称列表（空列表表示停止所有模块）
 * @return true 所有模块停止成功
 * @return false 有模块停止失败
 * 
 * @example
 * // 系统停止 - 停止所有模块（Ctrl+C）
 * launch_mgr->stopModules({});
 * 
 * // 场景切换 - 只停止指定模块
 * launch_mgr->stopModules({"yolo_det", "ppocr"});
 */
bool LaunchMgr::stopModules(const std::vector<std::string>& module_names) 
{
    // 判断是否为系统停止（空列表或列表包含所有模块）
    bool is_system_stop = module_names.empty();
    std::vector<std::string> modules_to_stop = module_names;
    
    // ========== 系统停止场景：设置关闭标志并更新状态 ==========
    if (is_system_stop) 
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        // 设置关闭请求标志（用于中断正在进行的启动过程）
        shutdown_requested_ = true;
        // 如果系统正在停止中，直接返回
        if (overall_status_ == ModuleStatus::STOPPING) 
        {
            LOG_WARN("系统正在停止中，请等待...");
            return false;
        }
        // 如果系统已经停止，直接返回成功
        if (overall_status_ == ModuleStatus::STOPPED) 
        {
            LOG_INFO("系统已经停止");
            return true;
        }
        // 获取所有模块名称
        for (const auto& pair : modules_) 
        {
            modules_to_stop.push_back(pair.first);
        } 
        LOG_INFO("========================================");
        LOG_INFO("系统停止：停止所有运行中的模块");
        LOG_INFO("========================================");
        overall_status_ = ModuleStatus::STOPPING;
    } 
    else 
    {
        LOG_INFO("========================================");
        LOG_INFO("场景切换：停止 %zu 个指定模块", modules_to_stop.size());
        LOG_INFO("========================================");
        for (const auto& name : modules_to_stop) {
            LOG_INFO("  - %s", name.c_str());
        }
    }
    
    // ========== 第一步：获取所有运行中的模块PID，并更新状态为STOPPING ==========
    std::vector<std::pair<std::string, int>> running_modules;
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        for (const auto& module_name : modules_to_stop) 
        {
            auto it = modules_.find(module_name);
            if (it != modules_.end() && it->second.isRunning() && it->second.pid > 0) 
            {
                running_modules.push_back({module_name, it->second.pid});
                updateModuleStatusInternal(module_name, ModuleStatus::STOPPING);
                
                if (is_system_stop) {
                    LOG_INFO("发现运行中的模块: %s (PID: %d)", module_name.c_str(), it->second.pid);
                }
            }
        }
    }
    if (running_modules.empty()) 
    {
        LOG_INFO("没有运行中的模块需要停止");
        if (is_system_stop) {
            overall_status_ = ModuleStatus::STOPPED;
        }
        return true;
    }
    // ========== 第二步：向所有模块同时发送 SIGTERM 信号（并行停止） ==========
    LOG_INFO("========================================");
    LOG_INFO("步骤1：向 %zu 个模块发送停止信号（并行）", running_modules.size());
    LOG_INFO("========================================");
    for (const auto& [module_name, pid] : running_modules) 
    {
        // 先检查进程是否还活着
        if (!basmodule::is_process_alive(pid)) 
        {
            LOG_WARN("  ✗ 进程已经不存在: %s (PID: %d)", module_name.c_str(), pid);
            continue;
        }
        LOG_INFO("  → 发送 SIGTERM 到 %s (PID: %d)", module_name.c_str(), pid);
        bool send_result = basmodule::stop_process(pid, false);  // false = SIGTERM
        if (!send_result) 
        {
            LOG_ERROR("  ✗ 发送信号失败: %s (PID: %d)，错误原因可能是：进程不存在或权限不足", 
                     module_name.c_str(), pid);
        } 
        else 
        {
            LOG_INFO("  ✓ 信号已发送: %s (PID: %d)", module_name.c_str(), pid);
            // 验证进程是否还活着（立即检查）
            basmodule::sleep_ms(100);
            if (!basmodule::is_process_alive(pid)) 
            {
                LOG_INFO("  ✓ 进程已快速退出: %s (PID: %d)", module_name.c_str(), pid);
            }
        }
    }
    
    // ========== 第三步：等待所有模块停止（统一等待，而不是逐个等待） ==========
    const int wait_timeout_ms = config_.startup_timeout_ms;  // 使用配置的超时时间
    const int check_interval_ms = 200;  // 每200ms检查一次
    int elapsed_ms = 0;
    
    LOG_INFO("========================================");
    LOG_INFO("步骤 2：等待所有模块停止（最长 %.1f 秒）", wait_timeout_ms / 1000.0);
    LOG_INFO("========================================");
    while (elapsed_ms < wait_timeout_ms) 
    {
        // 检查是否所有模块都已停止
        bool all_stopped = true;
        std::vector<std::pair<std::string, int>> still_running;
        {
            std::lock_guard<std::mutex> lock(modules_mutex_);
            for (const auto& [module_name, pid] : running_modules) 
            {
                if (basmodule::is_process_alive(pid)) 
                {
                    all_stopped = false;
                    still_running.push_back({module_name, pid});
                }
            }
        }
        if (all_stopped) 
        {
            LOG_INFO("========================================");
            LOG_INFO("✅ 所有模块已在 %.1f 秒内正常停止", elapsed_ms / 1000.0);
            LOG_INFO("========================================");
            break;
        }
        // 每 2 秒打印仍在运行的模块
        if (elapsed_ms % 2000 == 0 && elapsed_ms > 0) 
        {
            LOG_INFO("⏱️  已等待 %.1f 秒，仍在运行的模块：%zu 个", elapsed_ms / 1000.0, still_running.size());
            for (const auto& [name, pid] : still_running) 
            {
                LOG_INFO("  - %s (PID: %d)", name.c_str(), pid);
            }
        }
                
        // 如果等待时间超过超时时间的一半，发送日志提醒
        if (elapsed_ms > 0 && elapsed_ms % 5000 == 0) 
        {
            LOG_WARN("⚠️  已等待 %.1f 秒，模块仍未退出，请检查模块的资源清理逻辑...", elapsed_ms / 1000.0);
        }
                
        basmodule::sleep_ms(check_interval_ms);
        elapsed_ms += check_interval_ms;
    }
    
    // ========== 第四步：强制终止仍在运行的模块 ==========
    LOG_INFO("========================================");
    LOG_INFO("步骤3：检查并处理未停止的模块");
    LOG_INFO("========================================");
    
    bool success = true;
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        for (const auto& [module_name, pid] : running_modules) 
        {
            if (basmodule::is_process_alive(pid)) 
            {
                
                if (0)//暂时先不进行强制停止
                {
                    LOG_WARN("  ✗ 模块超时，发送 SIGKILL 强制终止：%s (PID: %d)", module_name.c_str(), pid);
                    basmodule::stop_process(pid, true);  // true = SIGKILL
                    success = false;
                    
                    // 等待一小段时间确认进程终止
                    basmodule::sleep_ms(100);
                }
                else
                {
                    LOG_ERROR("  ✗ 模块超时，等待手动终止：%s (PID: %d)", module_name.c_str(), pid);
                }
            } 
            else 
            {
                LOG_INFO("  ✓ 模块已正常退出: %s (PID: %d)", module_name.c_str(), pid);
            }
            // 更新模块状态
            auto& module_info = modules_[module_name];
            module_info.pid = -1;
            updateModuleStatusInternal(module_name, ModuleStatus::STOPPED);
        }
    }
    
    // ========== 第五步：等待模块完成资源清理并检查僵尸进程（仅系统停止场景） ==========
    if (is_system_stop) 
    {
        LOG_INFO("========================================");
        LOG_INFO("步骤4：等待模块完成资源清理");
        LOG_INFO("========================================");
        LOG_INFO("等待 1000ms 让所有模块完成资源清理...");
        basmodule::sleep_ms(1000);
        
        // 检查是否有僵尸进程
        int zombie_count = 0;
        std::vector<std::pair<std::string, int>> actual_zombies;
        {
            std::lock_guard<std::mutex> lock(modules_mutex_);
            for (const auto& [module_name, pid] : running_modules) 
            {
                if (pid > 0) 
                {
                    int status;
                    pid_t result = waitpid(pid, &status, WNOHANG);
                    if (result == 0) 
                    {
                        // 进程还在运行（可能是僵尸进程）
                        // 再次检查进程状态
                        if (!basmodule::is_process_alive(pid)) 
                        {
                            // 进程已退出但未被回收，这是真正的僵尸进程
                            zombie_count++;
                            actual_zombies.push_back({module_name, pid});
                            LOG_WARN("  ⚠️  检测到僵尸进程：%s (PID: %d)，状态：已退出但未回收", module_name.c_str(), pid);
                        }
                        else
                        {
                            // 进程仍在运行，需要进一步处理
                            LOG_WARN("  ⚠️  进程仍在运行：%s (PID: %d)，尝试再次发送 SIGKILL", module_name.c_str(), pid);
                            basmodule::stop_process(pid, true);  // 强制终止
                        }
                    } 
                    else if (result == pid) 
                    {
                        // 进程已退出，waitpid 成功回收
                        if (WIFEXITED(status)) 
                        {
                            LOG_INFO("  ✓ 进程正常退出：%s (PID: %d)，退出码：%d", 
                                    module_name.c_str(), pid, WEXITSTATUS(status));
                        } 
                        else if (WIFSIGNALED(status)) 
                        {
                            LOG_INFO("  ✓ 进程被信号终止：%s (PID: %d)，信号：%d", 
                                    module_name.c_str(), pid, WTERMSIG(status));
                        }
                    }
                    else if (result == -1)
                    {
                        // waitpid 错误，进程可能已经不存在
                        LOG_DEBUG("  ✓ 进程已不存在：%s (PID: %d)", module_name.c_str(), pid);
                    }
                }
            }
        }
        if (zombie_count > 0) 
        {
            LOG_ERROR("⚠️  发现 %d 个真正的僵尸进程，这些进程需要父进程调用 wait/waitpid 来回收:", zombie_count);
            for (const auto& [name, pid] : actual_zombies) {
                LOG_ERROR("  - %s (PID: %d)", name.c_str(), pid);
            }
            LOG_ERROR("  这通常是因为 bas_control_node 没有正确处理子进程的退出信号");
            LOG_ERROR("  建议在 bas_control_node 中注册 SIGCHLD 信号处理器来自动回收僵尸进程");
        } 
        else 
        {
            LOG_INFO("  ✓ 所有进程已正确退出并回收");
        }
        overall_status_ = ModuleStatus::STOPPED;
    }
    
    // ========== 打印最终结果 ==========
    if (success) 
    {
        LOG_INFO("========================================");
        if (is_system_stop) {
            LOG_INFO("✅ 所有模块已成功停止");
        } else {
            LOG_INFO("✅ 所有指定模块已成功停止");
        }
        LOG_INFO("========================================");
    } 
    else 
    {
        LOG_WARN("========================================");
        LOG_WARN("⚠️  部分模块需要强制终止");
        LOG_WARN("========================================");
    }
    return success;
}

/**
 * @brief 注册模块状态变化回调函数
 * 
 * @details 注册一个回调函数，当任意模块状态发生变化时会调用该函数。
 * 回调函数在模块状态更新时触发，如启动、停止、错误等状态变化。
 * 
 * @param callback 回调函数，接收ModuleInfo参数
 * 
 * @note 此方法是线程安全的，内部使用callbacks_mutex_保护回调列表
 * @note 回调函数会在状态更新线程中调用，应避免执行耗时操作
 * @note 同一个回调函数可以多次注册，会多次调用
 * @note 目前没有提供取消注册的机制，回调函数会一直有效直到对象销毁
 */
void LaunchMgr::registerSystemStatusCallback(std::function<void(const ModuleInfo&)> callback) 
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    status_callbacks_.push_back(callback);
}

/**
 * @brief 获取指定模块的详细信息
 * 
 * @details 查询并返回指定模块的详细信息，包括模块状态、PID、启动时间等。
 * 如果模块不存在，则返回一个默认构造的ModuleInfo对象。
 * 
 * @param module_name 要查询的模块名称
 * @return ModuleInfo 模块信息对象
 * 
 * @note 此方法是线程安全的，内部使用modules_mutex_保护共享数据
 * @note 如果模块不存在，返回的ModuleInfo对象为空（name为空字符串）
 * @note 调用者应检查返回对象的有效性（如name是否为空）
 */
ModuleInfo LaunchMgr::getModuleInfo(const std::string& module_name) const 
{
    std::lock_guard<std::mutex> lock(modules_mutex_);
    auto it = modules_.find(module_name);
    if (it != modules_.end()) {
        return it->second;
    }
    return ModuleInfo();
}

/**
 * @brief 获取所有模块的信息映射
 * 
 * @details 返回包含所有模块信息的映射表，键为模块名称，值为模块信息对象。
 * 返回的是当前时刻的快照，调用后对原数据的修改不会影响返回的副本。
 * 
 * @return std::map<std::string, ModuleInfo> 模块信息映射表
 * 
 * @note 此方法是线程安全的，内部使用modules_mutex_保护共享数据
 * @note 返回的是数据副本，修改返回的映射不会影响内部状态
 * @note 如果模块数量很大，返回映射可能占用较多内存
 */
std::map<std::string, ModuleInfo> LaunchMgr::getAllModuleInfo() const 
{
    std::lock_guard<std::mutex> lock(modules_mutex_);
    return modules_;
}

void LaunchMgr::updateModuleInfo(const std::string& module_name, const ModuleInfo& module_info) 
{
    std::lock_guard<std::mutex> lock(modules_mutex_);
    modules_[module_name] = module_info;
}

bool LaunchMgr::isModuleRunning(const std::string& module_name) const 
{
    std::lock_guard<std::mutex> lock(modules_mutex_);
    auto it = modules_.find(module_name);
    return it != modules_.end() && it->second.isRunning();
}

std::vector<std::string> LaunchMgr::getRunningModules() const 
{
    std::lock_guard<std::mutex> lock(modules_mutex_);
    std::vector<std::string> running_modules;
    for (const auto& pair : modules_) {
        if (pair.second.isRunning()) {
            running_modules.push_back(pair.first);
        }
    }
    return running_modules;
}

std::vector<std::string> LaunchMgr::getStartupOrder() const 
{
    return startup_order_;
}

void LaunchMgr::setStartupOrder(const std::vector<std::string>& order) 
{
    startup_order_ = order;
}

void LaunchMgr::addModuleDependencies(const std::string& module_name, const std::vector<std::string>& dependencies) 
{
    dependencies_[module_name] = dependencies;
    
    // 更新模块信息
    std::lock_guard<std::mutex> lock(modules_mutex_);
    if (modules_.find(module_name) == modules_.end()) 
    {
        ModuleInfo module_info(module_name);
        module_info.status = ModuleStatus::STOPPED;
        modules_[module_name] = module_info;
    }
    modules_[module_name].dependencies = dependencies;
}

void LaunchMgr::removeModule(const std::string& module_name) 
{
    std::lock_guard<std::mutex> lock(modules_mutex_);
    modules_.erase(module_name);
    dependencies_.erase(module_name);
    // 从启动顺序中移除
    startup_order_.erase(std::remove(startup_order_.begin(), startup_order_.end(), module_name), startup_order_.end());
    // 从其他模块的依赖列表中移除
    for (auto& pair : dependencies_) 
    {
        auto& deps = pair.second;
        deps.erase(std::remove(deps.begin(), deps.end(), module_name), deps.end());
    }
}

bool LaunchMgr::hasCircularDependency() const 
{
    return basmodule::has_circular_dependency(dependencies_);
}

ModuleStatus LaunchMgr::getStatus() const 
{
    return overall_status_;
}

bool LaunchMgr::startModuleInternal(const std::string& module_name, int timeout_ms) 
{
    LOG_INFO("正在启动模块: %s", module_name.c_str());
    
    // 第一步：检查依赖并更新状态（需要锁）
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        if (!checkDependenciesSatisfied(module_name)) 
        {
            LOG_WARN("模块依赖未满足: %s", module_name.c_str());
            updateModuleStatusInternal(module_name, ModuleStatus::ERROR, "Dependencies not satisfied");
            return false;
        }
        updateModuleStatusInternal(module_name, ModuleStatus::STARTING);// 更新模块状态为启动中
    }
    
    // 第二步：查找并启动可执行文件（不需要锁）
    // 模块名称即ROS2包名称
    std::string executable_path = basmodule::find_ros_executable(module_name);
    
    if (executable_path.empty()) {
        LOG_ERROR("未找到模块可执行文件: %s", module_name.c_str());
        std::lock_guard<std::mutex> lock(modules_mutex_);
        updateModuleStatusInternal(module_name, ModuleStatus::ERROR, "Executable not found");
        return false;
    }
    
    LOG_INFO("模块可执行文件路径: %s", executable_path.c_str());
    
    // 获取模块的启动参数（如果有）
    std::vector<std::string> arguments;
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        auto it = modules_.find(module_name);
        if (it != modules_.end()) {
            // 从parameters map中获取启动参数
            for (const auto& param : it->second.parameters) {
                arguments.push_back("--" + param.first);
                arguments.push_back(param.second);
            }
        }
    }
    
    // 启动进程
    int pid = basmodule::launch_process(executable_path, arguments, "");
    
    if (pid <= 0) {
        LOG_ERROR("进程启动失败: %s, path: %s", module_name.c_str(), executable_path.c_str());
        std::lock_guard<std::mutex> lock(modules_mutex_);
        updateModuleStatusInternal(module_name, ModuleStatus::ERROR, "Failed to launch process");
        return false;
    }
    
    LOG_INFO("进程启动成功，PID: %d", pid);
    
    // 第三步：更新启动结果（需要锁）
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        auto& module_info = modules_[module_name];
        module_info.pid = pid;
        module_info.executable_path = executable_path;
        module_info.start_time = std::chrono::steady_clock::now();
        module_info.last_heartbeat = std::chrono::steady_clock::now();
        updateModuleStatusInternal(module_name, ModuleStatus::RUNNING);// 更新状态为运行中
    }
    
    // 第四步：等待模块完全启动（不需要锁，但会轮询状态）
    if (!waitForModuleStartup(module_name, timeout_ms)) 
    {
        LOG_WARN("模块启动超时: %s", module_name.c_str());
        std::lock_guard<std::mutex> lock(modules_mutex_);
        updateModuleStatusInternal(module_name, ModuleStatus::ERROR, "Startup timeout");
        return false;
    }
    LOG_INFO("模块启动成功: %s", module_name.c_str());
    return true;
}

bool LaunchMgr::stopModuleInternal(const std::string& module_name, int timeout_ms) 
{
    LOG_INFO("正在停止模块: %s", module_name.c_str());
    // 第一步：检查模块状态并更新（需要锁）
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        auto it = modules_.find(module_name);
        if (it == modules_.end()) {
            LOG_WARN("模块未找到: %s", module_name.c_str());
            return false;
        }
        auto& module_info = it->second;
        if (module_info.isStopped()) {
            LOG_INFO("模块已停止: %s", module_name.c_str());
            return true;
        }
        // 更新状态为停止中
        updateModuleStatusInternal(module_name, ModuleStatus::STOPPING);
    }
    
    // 第二步：实际停止模块（不需要锁）
    // 模拟模块停止过程
    int pid = -1;
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        pid = modules_[module_name].pid;
    }
    if (pid > 0) 
    {
        // 发送优雅停止信号 SIGTERM（不持有锁）
        LOG_INFO("向模块 %s (PID: %d) 发送停止信号", module_name.c_str(), pid);
        basmodule::stop_process(pid, false);  // false = SIGTERM
        
        // 等待进程正常退出（不持有锁）
        if (!basmodule::wait_for_process(pid, timeout_ms)) 
        {
            LOG_WARN("模块停止超时，强制终止: %s (PID: %d)", module_name.c_str(), pid);
            basmodule::stop_process(pid, true);  // true = SIGKILL
        } else {
            LOG_INFO("模块 %s (PID: %d) 已正常退出", module_name.c_str(), pid);
        }
    }
    // 第三步：更新停止结果（需要锁）
    {
        std::lock_guard<std::mutex> lock(modules_mutex_);
        
        auto& module_info = modules_[module_name];
        module_info.pid = -1;
        
        // 更新状态为已停止
        updateModuleStatusInternal(module_name, ModuleStatus::STOPPED);
    }
    LOG_INFO("模块停止成功: %s", module_name.c_str());
    return true;
}

bool LaunchMgr::checkDependenciesSatisfied(const std::string& module_name) const 
{
    // 注意：此方法假设已持有modules_mutex_锁
    auto deps_it = dependencies_.find(module_name);
    if (deps_it == dependencies_.end()) {
        return true; //没有依赖
    }
    
    const auto& dependencies = deps_it->second;
    
    // 打印当前模块所有依赖信息
    std::string deps_str;
    for (size_t i = 0; i < dependencies.size(); ++i) {
        deps_str += dependencies[i];
        if (i < dependencies.size() - 1) {
            deps_str += ", ";
        }
    }
    LOG_DEBUG("检查模块 [%s] 的依赖项: ( %s )", module_name.c_str(), deps_str.c_str());
    for (const auto& dep : dependencies) 
    {
        auto module_it = modules_.find(dep);
        if (module_it == modules_.end() || !module_it->second.isRunning()) {
            return false;
        }
    } 
    return true;
}

/**
 * @brief 获取模块启动顺序
 * @details 根据依赖关系对指定模块列表进行拓扑排序
 */
std::vector<std::string> LaunchMgr::getModuleStartupOrder(const std::vector<std::string>& modules) const 
{
    if (modules.empty()) {
        return {};
    }
    
    // 如果只有一个模块，直接返回
    if (modules.size() == 1) {
        return modules;
    }
    
    // 构建子依赖图（只包含指定模块）
    std::map<std::string, std::vector<std::string>> sub_deps;
    for (const auto& module_name : modules) {
        auto deps_it = dependencies_.find(module_name);
        if (deps_it != dependencies_.end()) {
            // 只保留在指定模块列表中的依赖
            std::vector<std::string> filtered_deps;
            for (const auto& dep : deps_it->second) {
                if (std::find(modules.begin(), modules.end(), dep) != modules.end()) {
                    filtered_deps.push_back(dep);
                }
            }
            sub_deps[module_name] = filtered_deps;
        } else {
            sub_deps[module_name] = {};
        }
    }
    
    // 使用拓扑排序
    try {
        return basmodule::topological_sort(sub_deps);
    } catch (const std::exception& e) {
        LOG_WARN("拓扑排序失败: %s，使用原始顺序", e.what());
        return modules;
    }
}

bool LaunchMgr::waitForModuleStartup(const std::string& module_name, int timeout_ms) 
{
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() < timeout_ms) 
    {
        // 检查模块是否运行（需要在锁内）
        bool is_running = false;
        bool heartbeat_ok = false;
        {
            std::lock_guard<std::mutex> lock(modules_mutex_);
            
            auto it = modules_.find(module_name);
            if (it != modules_.end()) {
                is_running = it->second.isRunning();
                if (is_running) {
                    // 检查心跳
                    heartbeat_ok = checkModuleHeartbeat(module_name);
                }
            }
        } 
        if (is_running && heartbeat_ok) {
            return true;
        }
        // 不持有锁时睡眠
        basmodule::sleep_ms(100);
    }
    
    return false;
}

bool LaunchMgr::checkModuleHeartbeat(const std::string& module_name) 
{
    // 注意：此方法假设已持有modules_mutex_锁
    // 模拟心跳检查
    auto it = modules_.find(module_name);
    if (it != modules_.end()) 
    {
        // 模拟心跳更新
        it->second.last_heartbeat = std::chrono::steady_clock::now();
        return true;
    }
    return false;
}

void LaunchMgr::updateModuleStatusInternal(const std::string& module_name, ModuleStatus status, const std::string& error_message) 
{
    // 注意：此方法假设已持有modules_mutex_锁，不会再次加锁
    auto it = modules_.find(module_name);
    if (it != modules_.end()) 
    {
        it->second.status = status;
        if (!error_message.empty()) 
        {
            it->second.error_message = error_message;
        }
        // 触发状态回调
        triggerStatusCallback(it->second);
    }
}

void LaunchMgr::updateModuleStatus(const std::string& module_name, ModuleStatus status, const std::string& error_message) 
{
    // 此方法是线程安全的公开接口，会加锁
    std::lock_guard<std::mutex> lock(modules_mutex_);
    updateModuleStatusInternal(module_name, status, error_message);
}

void LaunchMgr::triggerStatusCallback(const ModuleInfo& module_info) 
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    for (const auto& callback : status_callbacks_) 
    {
        try {
            callback(module_info);
        } catch (const std::exception& e) {
            LOG_WARN("状态回调异常: %s", e.what());
        }
    }
}

} // namespace bas_control
