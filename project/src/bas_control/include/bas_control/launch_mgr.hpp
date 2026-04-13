#ifndef BAS_CONTROL_LAUNCH_MGR_HPP
#define BAS_CONTROL_LAUNCH_MGR_HPP

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

namespace bas_control {

/**
 * @brief 启动管理器类
 * @details 负责按依赖顺序启动/停止视觉模块，管理模块生命周期
 * 
 * 主要功能：
 * 1. 模块启动/停止管理：按拓扑顺序启动模块，逆序停止模块
 * 2. 依赖关系管理：维护模块间的依赖关系，确保启动顺序正确
 * 3. 状态监控：跟踪各模块运行状态，支持状态回调
 * 4. 异常处理：处理启动失败、超时等异常情况
 */
class LaunchMgr {
public:
    /**
     * @brief 构造函数
     * @param config 配置参数，包含启动顺序、依赖关系等
     */
    explicit LaunchMgr(const ConfigParams& config);
    
    /**
     * @brief 析构函数
     * @details 停止所有运行中的模块，释放资源
     */
    ~LaunchMgr();
    
    // 禁用拷贝构造和赋值（管理独占资源）
    LaunchMgr(const LaunchMgr&) = delete;
    LaunchMgr& operator=(const LaunchMgr&) = delete;
    
    /**
     * @brief 启动所有模块
     * @details 调用startModules({})来启动所有模块。
     *          此方法是对startModules的封装，提供向后兼容的接口。
     * @return true 所有模块启动成功
     * @return false 有模块启动失败或系统已经在运行中
     * 
     * @note 内部调用 startModules({})
     */
    bool startAllModules();
    
    /**
     * @brief 停止所有模块
     * @details 调用stopModules({})来停止所有模块。
     *          此方法是对stopModules的封装，提供向后兼容的接口。
     * @return true 所有模块停止成功
     * @return false 有模块停止失败
     * 
     * @note 内部调用 stopModules({})
     */
    bool stopAllModules();
    
    /**
     * @brief 重启指定模块
     * @details 先停止模块，再重新启动
     * @param module_name 模块名称
     * @return true 重启成功
     * @return false 重启失败
     */
    bool restartModule(const std::string& module_name);
    
    /**
     * @brief 启动指定模块
     * @details 启动单个模块，会检查依赖是否满足
     * @param module_name 模块名称
     * @return true 启动成功
     * @return false 启动失败
     */
    bool startModule(const std::string& module_name);
    
    /**
     * @brief 停止指定模块
     * @details 停止单个模块，会等待进程正常退出
     * @param module_name 模块名称
     * @return true 停止成功
     * @return false 停止失败
     */
    bool stopModule(const std::string& module_name);
    
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
    bool startModules(const std::vector<std::string>& module_names);
    
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
    bool stopModules(const std::vector<std::string>& module_names);
    
    /**
     * @brief 注册模块状态回调函数
     * @details 当模块状态发生变化时会调用注册的回调函数
     * @param callback 回调函数，参数为模块信息
     */
    void registerSystemStatusCallback(std::function<void(const ModuleInfo&)> callback);
    
    /**
     * @brief 获取模块信息
     * @param module_name 模块名称
     * @return ModuleInfo 模块信息，如果未找到则返回空的ModuleInfo
     */
    ModuleInfo getModuleInfo(const std::string& module_name) const;
    
    /**
     * @brief 获取所有模块信息
     * @return std::map<std::string, ModuleInfo> 模块名称到模块信息的映射
     */
    std::map<std::string, ModuleInfo> getAllModuleInfo() const;
    
    /**
     * @brief 更新模块信息
     * @details 手动更新指定模块的信息
     * @param module_name 模块名称
     * @param module_info 新的模块信息
     */
    void updateModuleInfo(const std::string& module_name, const ModuleInfo& module_info);
    
    /**
     * @brief 检查模块是否正在运行
     * @param module_name 模块名称
     * @return true 模块正在运行
     * @return false 模块未运行或不存在
     */
    bool isModuleRunning(const std::string& module_name) const;
    
    /**
     * @brief 获取运行中的模块列表
     * @return std::vector<std::string> 运行中的模块名称列表
     */
    std::vector<std::string> getRunningModules() const;
    
    /**
     * @brief 获取启动顺序
     * @return std::vector<std::string> 模块启动顺序列表
     */
    std::vector<std::string> getStartupOrder() const;
    
    /**
     * @brief 设置启动顺序
     * @param order 新的启动顺序
     */
    void setStartupOrder(const std::vector<std::string>& order);
    
    /**
     * @brief 添加模块依赖关系
     * @details 设置指定模块依赖的其他模块
     * @param module_name 模块名称
     * @param dependencies 依赖模块列表
     */
    void addModuleDependencies(const std::string& module_name, const std::vector<std::string>& dependencies);
    
    /**
     * @brief 移除模块
     * @details 从管理器中移除指定模块，并清理相关依赖关系
     * @param module_name 模块名称
     */
    void removeModule(const std::string& module_name);
    
    /**
     * @brief 检查是否存在循环依赖
     * @return true 存在循环依赖
     * @return false 不存在循环依赖
     */
    bool hasCircularDependency() const;
    
    /**
     * @brief 获取当前整体状态
     * @return ModuleStatus 系统整体状态
     */
    ModuleStatus getStatus() const;

private:
    /**
     * @brief 初始化模块信息
     * @details 根据配置参数初始化模块信息、依赖关系和启动顺序
     */
    void initializeModules();
    
    /**
     * @brief 启动单个模块（内部方法，不加锁）
     * @details 实际执行模块启动逻辑，调用前需确保已持有modules_mutex_
     * @param module_name 模块名称
     * @param timeout_ms 超时时间(毫秒)
     * @return true 启动成功
     * @return false 启动失败
     */
    bool startModuleInternal(const std::string& module_name, int timeout_ms = 30000);
    
    /**
     * @brief 停止单个模块（内部方法，不加锁）
     * @details 实际执行模块停止逻辑，调用前需确保已持有modules_mutex_
     * @param module_name 模块名称
     * @param timeout_ms 超时时间(毫秒)
     * @return true 停止成功
     * @return false 停止失败
     */
    bool stopModuleInternal(const std::string& module_name, int timeout_ms = 10000);
    
    /**
     * @brief 检查模块依赖是否满足（内部方法，不加锁）
     * @details 检查指定模块的所有依赖模块是否都在运行，调用前需确保已持有modules_mutex_
     * @param module_name 模块名称
     * @return true 依赖已满足
     * @return false 依赖未满足
     */
    bool checkDependenciesSatisfied(const std::string& module_name) const;
    
    /**
     * @brief 等待模块启动完成（内部方法，不加锁）
     * @details 轮询检查模块状态直到启动成功或超时，调用前需确保已持有modules_mutex_
     * @param module_name 模块名称
     * @param timeout_ms 超时时间(毫秒)
     * @return true 模块启动成功
     * @return false 模块启动超时
     */
    bool waitForModuleStartup(const std::string& module_name, int timeout_ms);
    
    /**
     * @brief 检查模块心跳（内部方法，不加锁）
     * @details 检查模块是否存活，调用前需确保已持有modules_mutex_
     * @param module_name 模块名称
     * @return true 模块存活
     * @return false 模块已死亡
     */
    bool checkModuleHeartbeat(const std::string& module_name);
    
    /**
     * @brief 获取模块启动顺序
     * @details 根据依赖关系对指定模块列表进行拓扑排序，确保依赖满足
     * @param modules 需要启动的模块列表
     * @return 排序后的模块列表
     */
    std::vector<std::string> getModuleStartupOrder(const std::vector<std::string>& modules) const;
    
    /**
     * @brief 更新模块状态（内部方法，不加锁）
     * @details 直接修改模块状态并触发回调，调用前需确保已持有modules_mutex_
     * @param module_name 模块名称
     * @param status 新状态
     * @param error_message 错误信息(可选)
     */
    void updateModuleStatusInternal(const std::string& module_name, ModuleStatus status, const std::string& error_message = "");
    
    /**
     * @brief 更新模块状态（公共方法，加锁）
     * @details 线程安全的模块状态更新方法
     * @param module_name 模块名称
     * @param status 新状态
     * @param error_message 错误信息(可选)
     */
    void updateModuleStatus(const std::string& module_name, ModuleStatus status, const std::string& error_message = "");
    
    /**
     * @brief 触发状态回调（内部方法，不加锁）
     * @details 调用所有注册的状态回调函数，调用前需确保已持有callbacks_mutex_
     * @param module_info 模块信息
     */
    void triggerStatusCallback(const ModuleInfo& module_info);
 
    /**
     * @brief 重试执行函数（带指数退避）
     * @tparam Func 函数类型
     * @param func 要执行的函数
     * @param max_attempts 最大尝试次数
     * @param delay_ms 初始重试间隔(毫秒)，后续每次翻倍
     * @return decltype(func()) 函数执行结果
     * @throws std::runtime_error 所有重试均失败
     */
    template<typename Func>
    auto retryWithBackoff(Func func, int max_attempts, int delay_ms = 1000) -> decltype(func()) {
        std::exception_ptr last_exception;
        
        for (int attempt = 0; attempt < max_attempts; ++attempt) 
        {
            try {
                return func();
            } catch (...) {
                last_exception = std::current_exception();
                if (attempt < max_attempts - 1) {
                    basmodule::sleep_ms(delay_ms * (attempt + 1)); // 指数退避
                }
            }
        }
        
        if (last_exception) {
            std::rethrow_exception(last_exception);
        }
        
        throw std::runtime_error("All retry attempts failed");
    }

private:
    // 配置与模块图
    ConfigParams config_;                                    ///< 配置参数（启动超时、重试次数等）
    std::map<std::string, ModuleInfo> modules_;///< 模块信息映射：模块名 -> 模块信息，包括模块状态、PID、启动时间等
    std::map<std::string, std::vector<std::string>> dependencies_;///邻接表形式的依赖关系图（依赖模块列表）
    std::vector<std::string> startup_order_;                 ///<启动顺序：按拓扑排序的模块名列表
    
    // 控制状态
    ModuleStatus overall_status_;                            ///< 整体状态（RUNNING/STOPPED/ERROR等）
    bool shutdown_requested_;                                ///< 关闭请求标志（用于中断启动过程）

    // 观察者模式支持
    std::vector<std::function<void(const ModuleInfo&)>> status_callbacks_; ///<模块状态变更回调
    
    mutable std::mutex modules_mutex_;///<模块信息互斥锁
    ///< 保护数据: modules_, dependencies_, 
    ///<           startup_order_, overall_status_,
    ///<           shutdown_requested_
    
    mutable std::mutex callbacks_mutex_;///< 回调函数互斥锁，保护数据: status_callbacks_
    
};

} // namespace bas_control

#endif // BAS_CONTROL_LAUNCH_MGR_HPP