#ifndef BAS_CONTROL_CONFIG_HOT_UPDATER_HPP
#define BAS_CONTROL_CONFIG_HOT_UPDATER_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <mutex>
#include <thread>
#include <chrono>
#include <filesystem>
#include "module_info.hpp"
#include <bas_operate/bas_utils.hpp>

//前向声明
namespace YAML {
    class Node;
}

namespace bas_control {

/**
 * @brief配置热更新器类
 *支持运行时参数调整和模型切换
 */
class ConfigHotUpdater {
public:
    /**
     * @brief构造函数
     * @param config配置参数
     */
    explicit ConfigHotUpdater(const ConfigParams& config);
    
    /**
     * @brief析函数
     */
    ~ConfigHotUpdater();
    
    //拷贝构造和赋值
    ConfigHotUpdater(const ConfigHotUpdater&) = delete;
    ConfigHotUpdater& operator=(const ConfigHotUpdater&) = delete;
    
    /**
     * @brief启动配置监控
     * @return true if successful, false otherwise
     */
    bool startMonitoring();
    
    /**
     * @brief停止配置监控
     * @return true if successful, false otherwise
     */
    bool stopMonitoring();
    
    /**
     * @brief热更新模型文件
     * @param module_name模块名称
     * @param model_path新模型路径
     * @return true if successful, false otherwise
     */
    bool updateModel(const std::string& module_name, const std::string& model_path);
    
    /**
     * @brief热更新参数配置
     * @param module_name模块名称
     * @param params新参数配置
     * @return true if successful, false otherwise
     */
    bool updateParams(const std::string& module_name, const std::map<std::string, std::string>& params);
    
    /**
     * @brief从配置文件热更新配置
     * @param module_name模块名称
     * @param config_file配置文件路径
     * @return true if successful, false otherwise
     */
    bool updateFromFile(const std::string& module_name, const std::string& config_file);
    
    /**
     * @brief A/B测试切换
     * @param module_name模块名称
     * @param version版本标识
     * @return true if successful, false otherwise
     */
    bool switchABTest(const std::string& module_name, const std::string& version);
    
    /**
     * @brief版本回滚
     * @param module_name模块名称
     * @param target_version目标版本
     * @return true if successful, false otherwise
     */
    bool rollback(const std::string& module_name, const std::string& target_version = "");
    
    /**
     * @brief获取当前配置版本
     * @param module_name模块名称
     * @return当前版本信息
     */
    std::string getCurrentVersion(const std::string& module_name) const;
    
    /**
     * @brief获取可用版本列表
     * @param module_name模块名称
     * @return版本列表
     */
    std::vector<std::string> getAvailableVersions(const std::string& module_name) const;
    
    /**
     * @brief注册配置更新回调函数
     * @param module_name模块名称
     * @param callback回调函数
     */
    void registerUpdateCallback(const std::string& module_name, std::function<void(const std::string&, const std::map<std::string, std::string>&)> callback);
    
    /**
     * @brief注册模型更新回调函数
     * @param module_name模块名称
     * @param callback回调函数
     */
    void registerModelCallback(const std::string& module_name, std::function<void(const std::string&, const std::string&)> callback);
    
    /**
     * @brief检查配置文件是否发生变化
     * @param config_file配置文件路径
     * @return true if file has changed, false otherwise
     */
    bool isConfigFileChanged(const std::string& config_file);
    
    /**
     * @brief获取配置更新历史
     * @param module_name模块名称
     * @param max_entries最大历史条目数
     * @return更新历史记录
     */
    std::vector<std::tuple<std::string, std::string, std::chrono::system_clock::time_point>> 
    getUpdateHistory(const std::string& module_name, size_t max_entries = 10) const;
    
    /**
     * @brief验证配置文件格式
     * @param config_file配置文件路径
     * @return true if valid, false otherwise
     */
    bool validateConfigFile(const std::string& config_file) const;
    
    /**
     * @brief备份当前配置
     * @param module_name模块名称
     * @param backup_path备份路径
     * @return true if successful, false otherwise
     */
    bool backupConfig(const std::string& module_name, const std::string& backup_path = "");
    
    /**
     * @brief从备份恢复配置
     * @param module_name模块名称
     * @param backup_path备份路径
     * @return true if successful, false otherwise
     */
    bool restoreFromBackup(const std::string& module_name, const std::string& backup_path);

private:
    /**
     * @brief配置监控主循环
     */
    void monitoringLoop();
    
    /**
     * @brief检查配置文件变化
     */
    void checkConfigFileChanges();
    
    /**
     * @brief执行配置更新
     * @param module_name模块名称
     * @param new_params新参数
     * @return true if successful, false otherwise
     */
    bool executeConfigUpdate(const std::string& module_name, const std::map<std::string, std::string>& new_params);
    
    /**
     * @brief执行模型更新
     * @param module_name模块名称
     * @param new_model_path新模型路径
     * @return true if successful, false otherwise
     */
    bool executeModelUpdate(const std::string& module_name, const std::string& new_model_path);
    
    /**
     * @brief加载配置文件
     * @param config_file配置文件路径
     * @return配置参数映射
     */
    std::map<std::string, std::string> loadConfigFile(const std::string& config_file) const;
    
    /**
     * @brief保存配置文件
     * @param config_file配置文件路径
     * @param params参数映射
     * @return true if successful, false otherwise
     */
    bool saveConfigFile(const std::string& config_file, const std::map<std::string, std::string>& params) const;
    
    /**
     * @brief触发配置更新回调
     * @param module_name模块名称
     * @param params更新后的参数
     */
    void triggerConfigCallback(const std::string& module_name, const std::map<std::string, std::string>& params);
    
    /**
     * @brief触发模型更新回调
     * @param module_name模块名称
     * @param model_path新模型路径
     */
    void triggerModelCallback(const std::string& module_name, const std::string& model_path);
    
    /**
     * @brief记录配置更新历史
     * @param module_name模块名称
     * @param update_type更新类型
     * @param details更新详情
     */
    void recordUpdateHistory(const std::string& module_name, const std::string& update_type, const std::string& details);
    
    /**
     * @brief获取文件修改时间
     * @param file_path文件路径
     * @return文件修改时间
     */
    std::chrono::system_clock::time_point getFileModificationTime(const std::string& file_path) const;
  
    /**
     * @brief生成版本标识
     * @return版本标识字符串
     */
    std::string generateVersionId() const;

private:
    // 当前配置与版本快照
    ConfigParams config_;                                              ///<配置参数
    mutable std::mutex updater_mutex_;                                ///<更新器互斥锁
    std::map<std::string, std::map<std::string, std::string>> current_configs_; ///<当前配置：模块名->参数键值映射
    std::map<std::string, std::string> current_models_;                ///<当前模型路径：模型名->文件路径映射
    std::map<std::string, std::string> current_versions_;              ///<当前版本：配置/模型版本标识
    std::map<std::string, std::vector<std::string>> version_history_;  ///<版本历史记录，支持回滚

    // 文件监控
    std::map<std::string, std::chrono::system_clock::time_point> file_timestamps_; ///<文件时间戳

    // **嵌套的回调映射**：实现模块/配置项的精准通知
    // 第一层键：模块名，第二层键：参数名，值：回调函数
    std::map<std::string, std::map<std::string, std::function<void(const std::string&, const std::map<std::string, std::string>&)>>> config_callbacks_; ///<配置回调函数
    std::map<std::string, std::map<std::string, std::function<void(const std::string&, const std::string&)>>> model_callbacks_; ///<模型回调函数
    
    // 线程控制
    std::unique_ptr<std::thread> monitoring_thread_;                    ///<监控线程
    std::atomic<bool> monitoring_active_;                             ///<监控活动状态，原子标志，用于安全地启停线程

    std::map<std::string, std::vector<std::tuple<std::string, std::string, std::chrono::system_clock::time_point>>> update_history_; ///<更新历史
    mutable std::mutex history_mutex_;                                 ///<历史记录互斥锁
    int config_check_interval_ms_;                                     ///<配置检查间隔
};

} // namespace bas_control

#endif // BAS_CONTROL_CONFIG_HOT_UPDATER_HPP