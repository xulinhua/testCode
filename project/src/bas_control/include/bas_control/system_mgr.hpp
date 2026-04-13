#ifndef BAS_CONTROL_SYSTEM_MGR_HPP
#define BAS_CONTROL_SYSTEM_MGR_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <mutex>
#include <thread>
#include <chrono>
#include "module_info.hpp"
#include "launch_mgr.hpp"
#include "status_monitor.hpp"
#include "task_scheduler.hpp"
#include "config_hot_updater.hpp"
#include <bas_operate/bas_utils.hpp>

//前向声明
namespace YAML {
    class Node;
}

namespace bas_control {

/**
 * @brief系统管理器类
 *统一部署控制层的核心协调类
 */
class SystemMgr {
public:
    /**
     * @brief构造函数
     * @param config_file配置文件路径
     */
    explicit SystemMgr(const std::string& config_file = "");
    
    /**
     * @brief析函数
     */
    ~SystemMgr();
    
    //拷贝构造和赋值
    SystemMgr(const SystemMgr&) = delete;
    SystemMgr& operator=(const SystemMgr&) = delete;
    
    /**
     * @brief初始化系统
     * @return true if successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief启动系统
     * @return true if successful, false otherwise
     */
    bool start();
    
    /**
     * @brief停止系统
     * @return true if successful, false otherwise
     */
    bool stop();
    
    /**
     * @brief重启系统
     * @return true if successful, false otherwise
     */
    bool restart();
    
    /**
     * @brief获取系统状态
     * @return系统状态
     */
    SystemStatus getSystemStatus() const;
    
    /**
     * @brief获取当前场景
     * @return当前场景类型
     */
    SceneType getCurrentScene() const;
    
    /**
     * @brief切换场景
     * @param scene_type场景类型
     * @return true if successful, false otherwise
     */
    bool switchScene(SceneType scene_type);
    
    /**
     * @brief获取所有可用场景
     * @return场景名称列表
     */
    /**
     * @brief获取所有可用场景
     * @return场景类型列表
     */
    std::vector<SceneType> getAvailableScenes() const;
    
    /**
     * @brief更新模块配置
     * @param module_name模块名称
     * @param params新参数
     * @return true if successful, false otherwise
     */
    bool updateModuleConfig(const std::string& module_name, const std::map<std::string, std::string>& params);
    
    /**
     * @brief更新模块模型
     * @param module_name模块名称
     * @param model_path新模型路径
     * @return true if successful, false otherwise
     */
    bool updateModuleModel(const std::string& module_name, const std::string& model_path);
    
    /**
     * @brief获取运行中的模块列表
     * @return运行中的模块名称列表
     */
    std::vector<std::string> getRunningModules() const;
    
    /**
     * @brief检查系统健康状态
     * @return true if system is healthy, false otherwise
     */
    bool isSystemHealthy() const;
    
    /**
     * @brief获取系统资源使用情况
     * @return系统资源状态
     */
    SystemResource getSystemResource() const;
    
    /**
     * @brief注册系统状态回调函数
     * @param callback回调函数
     */
    void registerSystemStatusCallback(std::function<void(const SystemStatus&)> callback);
    
    /**
     * @brief注册场景切换回调函数
     * @param callback回调函数
     */
    void registerSceneCallback(std::function<void(const std::string&, const std::vector<std::string>&)> callback);
    
    /**
     * @brief注册模块状态回调函数
     * @param callback回调函数
     */
    void registerModuleStatusCallback(std::function<void(const ModuleInfo&)> callback);
    
    /**
     * @brief获取系统统计信息
     * @return统计信息映射
     */
    std::map<std::string, std::string> getSystemStats() const;
    
    /**
     * @brief执行系统自检
     * @return自检结果
     */
    std::map<std::string, bool> performSelfCheck() const;
    
    /**
     * @brief获取系统信息
     * @return系统信息
     */
    std::map<std::string, std::string> getSystemInfo() const;

    /**
     * @brief获取配置参数
     * @return配置参数
     */
    ConfigParams getConfig() const;
    
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
     * @return 模块状态信息
     */
    basros::ModuleStatusInfo getModuleCamStatus(const std::string& module_name, int cam_id) const;

private:
    /**
     * @brief加载配置文件
     * @param config_file配置文件路径
     * @return true if successful, false otherwise
     */
    bool loadConfig(const std::string& config_file);
    
    /**
     * @brief解析YAML配置
     * @param yaml_node YAML节点
     * @return true if successful, false otherwise
     */
    bool parseYamlConfig(const YAML::Node& yaml_node);
    
    /**
     * @brief初始化各组件
     * @return true if successful, false otherwise
     */
    bool initializeComponents();
    
    /**
     * @brief建立组件间连接
     * @return true if successful, false otherwise
     */
    bool setupComponentConnections();
    
    /**
     * @brief启动核心服务
     * @return true if successful, false otherwise
     */
    bool startCoreServices();
    
    /**
     * @brief停止核心服务
     * @return true if successful, false otherwise
     */
    bool stopCoreServices();
    
    /**
     * @brief系统状态回调处理
     * @param status系统状态
     */
    void onSystemStatusUpdate(const SystemStatus& status);
    
    /**
     * @brief模块状态回调处理
     * @param module_info模块信息
     */
    void onModuleStatusUpdate(const ModuleInfo& module_info);
    
    /**
     * @brief场景切换回调处理
     * @param scene_name场景名称
     * @param active_modules活跃模块列表
     */
    void onSceneSwitch(const std::string& scene_name, const std::vector<std::string>& active_modules);
    
    /**
     * @brief资源状态回调处理
     * @param resource系统资源状态
     */
    void onResourceUpdate(const SystemResource& resource);
 
    /**
     * @brief获取系统启动时间
     * @return启动时间
     */
    std::chrono::system_clock::time_point getStartupTime() const;
    
    /**
     * @brief计算系统运行时间
     * @return运行时间字符串
     */
    std::string getUptime() const;

private:
    // 配置与组件实例（生命周期由智能指针管理）
    ConfigParams config_;                           ///< 当前生效的完整配置参数（系统配置）
    std::string config_file_path_;                  ///< 配置文件路径
    std::unique_ptr<LaunchMgr> launch_mgr_;             ///< 启动管理器实例
    std::unique_ptr<StatusMonitor> status_monitor_;     ///<状态监控器实例
    std::unique_ptr<TaskScheduler> task_scheduler_;     ///<任务调度器实例
    std::unique_ptr<ConfigHotUpdater> config_updater_;  ///<配置热更新器实例

    // 运行时状态
    SystemStatus current_status_;                                      ///<当前系统状态
    std::chrono::system_clock::time_point startup_time_;               ///<系统启动时间

    std::vector<std::function<void(const SystemStatus&)>> status_callbacks_; ///<状态回调函数
    std::vector<std::function<void(const std::string&, const std::vector<std::string>&)>> scene_callbacks_; ///<场景回调函数
    std::vector<std::function<void(const ModuleInfo&)>> module_callbacks_; ///<模块状态回调函数
    mutable std::mutex callbacks_mutex_;                               ///<回调函数互斥锁
    bool initialized_;                                                 ///<初始化状态
    bool running_;                                                     ///<运行状态
    mutable std::mutex mgr_mutex_;///< 管理器互斥锁，保护数据: current_status_, initialized_, running_
};

} // namespace bas_control

#endif // BAS_CONTROL_SYSTEM_MGR_HPP