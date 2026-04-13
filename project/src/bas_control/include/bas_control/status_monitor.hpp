#ifndef BAS_CONTROL_STATUS_MONITOR_HPP
#define BAS_CONTROL_STATUS_MONITOR_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <mutex>
#include <thread>
#include <chrono>
#include "module_info.hpp"
#include "bas_operate_ros/module_status.hpp"
#include <bas_operate/bas_utils.hpp>

namespace bas_control {

/**
 * @brief状态监控器类
 * 实时监控各模块运行状态和系统资源占用
 */
class StatusMonitor {
public:
    /**
     * @brief构造函数
     * @param config配置参数
     */
    explicit StatusMonitor(const ConfigParams& config);
    
    /**
     * @brief析函数
     */
    ~StatusMonitor();
    
    //拷构造和赋值
    StatusMonitor(const StatusMonitor&) = delete;
    StatusMonitor& operator=(const StatusMonitor&) = delete;
    
    /**
     * @brief启动监控
     * @return true if successful, false otherwise
     */
    bool startMonitoring();
    
    /**
     * @brief停止监控
     * @return true if successful, false otherwise
     */
    bool stopMonitoring();
    
    /**
     * @brief获取当前系统状态
     * @return系统状态
     */
    SystemStatus getCurrentStatus() const;
    
    /**
     * @brief获取指定模块状态
     * @param module_name 模块名称
     * @return模块状态
     */
    ModuleInfo getModuleStatus(const std::string& module_name) const;
    
    /**
     * @brief 获取系统资源状态
     * @return 系统资源状态
     */
    SystemResource getSystemResource() const;
    
    /**
     * @brief注册状态变化回调函数
     * @param callback回调函数
     */
    void registerSystemStatusCallback(std::function<void(const SystemStatus&)> callback);
    
    /**
     * @brief 注册模块状态变化回调函数
     * @param callback回调函数
     */
    void registerModuleStatusCallback(std::function<void(const ModuleInfo&)> callback);
    
    /**
     * @brief 注册资源状态变化回调函数
     * @param callback回调函数
     */
    void registerResourceCallback(std::function<void(const SystemResource&)> callback);
    
    /**
     * @brief 更新模块状态信息
     * @param module_name 模块名称
     * @param module_info模块信息
     */
    void updateModuleStatus(const std::string& module_name, const ModuleInfo& module_info);
    
    /**
     * @brief检查系统健康状态
     * @return true if system is healthy, false otherwise
     */
    bool isSystemHealthy() const;
    
    /**
     * @brief获取运行中的模块数量
     * @return运行中的模块数
     */
    size_t getRunningModuleCount() const;
    
    /**
     * @brief检查是否有模块错误
     * @return true if any module has error, false otherwise
     */
    bool hasModuleErrors() const;
    
    /**
     * @brief获取有错误的模块列表
     * @return错误模块列表
     */
    std::vector<std::string> getErrorModules() const;
    
    /**
     * @brief设置健康检查间隔
     * @param interval_ms间隔时间(毫秒)
     */
    void setHealthCheckInterval(int interval_ms);
    
    /**
     * @brief 设置状态上报间隔
     * @param interval_ms 间隔时间(毫秒)
     */
    void setStatusReportInterval(int interval_ms);
    
    /**
     * @brief 更新模块的相机状态信息
     * @param module_name 模块名称
     * @param cam_id 相机ID
     * @param status_info 模块状态信息
     */
    void updateModuleCamStatus(const std::string& module_name, int cam_id, const basros::ModuleStatusInfo& status_info);
    
    /**
     * @brief 获取模块的相机状态信息
     * @param module_name 模块名称
     * @param cam_id 相机ID
     * @return 模块状态信息，如果不存在返回默认构造的对象
     */
    basros::ModuleStatusInfo getModuleCamStatus(const std::string& module_name, int cam_id) const;
    
    /**
     * @brief 获取模块所有相机的状态信息
     * @param module_name 模块名称
     * @return 相机状态信息映射表
     */
    std::map<int, basros::ModuleStatusInfo> getModuleAllCamStatus(const std::string& module_name) const;
    
    /**
     * @brief 获取所有模块所有相机的状态信息
     * @return 所有模块所有相机的状态信息映射表
     */
    basros::ModuleStatusInfoMap getAllModuleCamStatus() const;
    
    /**
     * @brief 清空所有模块相机状态
     * @note 切换场景时调用，避免旧状态残留
     */
    void clearAllModuleCamStatus();
    
    /**
     * @brief 清空指定模块的相机状态
     * @param module_name 模块名称
     */
    void clearModuleCamStatus(const std::string& module_name);

private:
    /**
     * @brief监控主循环
     */
    void monitoringLoop();
    
    /**
     * @brief执行健康检查
     */
    void performHealthCheck();
    
    /**
     * @brief检查系统资源状态
     */
    void checkSystemResources();
    
    /**
     * @brief检查模块状态
     */
    void checkModuleStatus();
    
    /**
     * @brief检查模块心跳
     * @param module_info模块信息
     * @return true if module is alive, false otherwise
     */
    bool checkModuleHeartbeat(const ModuleInfo& module_info);
    
    /**
     * @brief检查系统资源阈值
     * @return true if resources are within normal range, false otherwise
     */
    bool checkResourceThresholds();
    
    /**
     * @brief触发状态回调
     * @param status系统状态
     */
    void triggerStatusCallback(const SystemStatus& status);
    
    /**
     * @brief触发模块状态回调
     * @param module_info模块信息
     */
    void triggerModuleStatusCallback(const ModuleInfo& module_info);
    
    /**
     * @brief触发资源状态回调
     * @param resource系统资源状态
     */
    void triggerResourceCallback(const SystemResource& resource);
  
    /**
     * @brief 获取进程资源使用情况
     * @param pid进程ID
     * @param cpu_usage[out] CPU使用率
     * @param memory_usage[out] 内存使用率
     */
    void getProcessResourceUsage(int pid, float& cpu_usage, float& memory_usage) const;

private:
    // 状态缓存
    ConfigParams config_;                    ///<配置参数
    SystemStatus current_status_;            ///<当前系统状态
    mutable std::mutex status_mutex_;        ///<状态互斥锁
    
    // 模块相机状态缓存
    basros::ModuleStatusInfoMap module_cam_status_map_; ///<所有模块的所有相机下的子节点状态映射表
    
    // 定时控制
    int health_check_interval_ms_;                                ///<健检查康检查间隔
    int status_report_interval_ms_;                               ///<状态上报间隔
    std::chrono::steady_clock::time_point last_health_check_;     ///<上次健康检查时间
    std::chrono::steady_clock::time_point last_status_report_;    ///<上次状态上报时间

    // **多类型回调容器**：允许不同组件订阅不同类型的状态变化
    std::vector<std::function<void(const SystemStatus&)>> status_callbacks_; ///<状态回调函数
    std::vector<std::function<void(const ModuleInfo&)>> module_callbacks_; ///<模块状态回调函数
    std::vector<std::function<void(const SystemResource&)>> resource_callbacks_; ///<状态回调函数
    mutable std::mutex callbacks_mutex_;                ///<回调函数互斥锁

    // 线程控制
    std::unique_ptr<std::thread> monitoring_thread_;    ///<独立监控线程
    std::atomic<bool> monitoring_active_;               ///<监控活动状态，原子标志，用于安全地启停线程
    
};

} // namespace bas_control

#endif // BAS_CONTROL_STATUS_MONITOR_HPP