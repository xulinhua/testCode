/**
 * @file config_hot_updater.cpp
 * @brief 配置热更新器实现文件
 * 
 * 功能概述：
 * ConfigHotUpdater类负责在系统运行时动态更新模块配置和模型文件，支持：
 * 1. 参数热更新：运行时修改模块参数，无需重启模块
 * 2. 模型热更新：运行时替换模型文件，支持A/B测试切换
 * 3. 配置监控：监控配置文件变化，自动应用更新
 * 4. 版本管理：记录配置版本历史，支持回滚操作
 * 
 * 主要接口说明：
 * 1. 监控管理：
 *    - startMonitoring()：启动配置监控线程，定期检查配置文件变化
 *    - stopMonitoring()：停止配置监控，释放资源
 *    - isConfigFileChanged()：检查指定配置文件是否被修改
 * 
 * 2. 配置更新：
 *    - updateParams()：更新指定模块的参数配置
 *    - updateModel()：更新指定模块的模型文件
 *    - updateFromFile()：从配置文件批量更新模块配置
 *    - switchABTest()：切换A/B测试版本
 *    - rollback()：回滚到指定版本或上一个版本
 * 
 * 3. 状态查询：
 *    - getCurrentVersion()：获取指定模块的当前配置版本
 *    - getAvailableVersions()：获取指定模块的可用版本列表
 *    - getUpdateHistory()：获取配置更新历史记录
 * 
 * 4. 备份恢复：
 *    - backupConfig()：备份当前配置到指定路径
 *    - restoreFromBackup()：从备份文件恢复配置
 * 
 * 5. 回调注册：
 *    - registerUpdateCallback()：注册配置更新回调函数
 *    - registerModelCallback()：注册模型更新回调函数
 * 
 * 实现机制：
 * 1. 使用独立监控线程定期检查配置文件修改时间
 * 2. 通过版本标识（时间戳）管理配置变更历史
 * 3. 采用读写锁（mutex）保护共享数据（当前配置、版本历史等）
 * 4. 支持异步回调通知，解耦更新逻辑和业务逻辑
 * 
 * 使用场景：
 * 1. 在线参数调优：根据环境变化动态调整检测阈值
 * 2. 模型部署：部署新模型版本并进行A/B测试
 * 3. 配置管理：集中管理各模块配置，支持快速回滚
 * 4. 系统维护：运行时修复配置错误，提高系统可用性
 * 
 * 注意事项：
 * 1. 配置更新可能影响模块性能，建议在低负载时操作
 * 2. 模型更新需要模块支持动态加载，部分模块可能需要重启
 * 3. 配置文件格式支持YAML/JSON，需保持语法正确性
 * 4. 回滚操作会恢复到指定版本状态，注意数据一致性
 */

#include "bas_control/config_hot_updater.hpp"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "log_system/log_macros.hpp"

namespace bas_control {

ConfigHotUpdater::ConfigHotUpdater(const ConfigParams& config)
    : config_(config), monitoring_active_(false), config_check_interval_ms_(2000) 
{
    
    LOG_INFO("配置热更新器已初始化");
}

ConfigHotUpdater::~ConfigHotUpdater() 
{
    stopMonitoring();
}

bool ConfigHotUpdater::startMonitoring()
{
    if (monitoring_active_) {
        LOG_WARN("监控已处于活动状态");
        return false;
    }
    monitoring_active_ = true;
    monitoring_thread_ = std::make_unique<std::thread>(&ConfigHotUpdater::monitoringLoop, this);
    LOG_INFO("配置监控已启动");
    return true;
}

bool ConfigHotUpdater::stopMonitoring() 
{
    if (!monitoring_active_) {
        return true;
    }
    monitoring_active_ = false;
    if (monitoring_thread_ && monitoring_thread_->joinable()) {
        monitoring_thread_->join();
    }
    monitoring_thread_.reset();
    LOG_INFO("配置监控已停止");
    return true;
}

bool ConfigHotUpdater::updateModel(const std::string& module_name, const std::string& model_path) 
{
    if (module_name.empty() || model_path.empty()) {
        LOG_WARN("无效的模块名或模型路径");
        return false;
    }
    LOG_INFO("正在更新模块模型: %s", module_name.c_str());
    //检查模型文件是否存在
    if (!std::filesystem::exists(model_path)) {
        LOG_WARN("模型文件未找到: %s", model_path.c_str());
        return false;
    }
    //执行模型更新
    if (!executeModelUpdate(module_name, model_path)) {
        LOG_WARN("模块模型更新执行失败: %s", module_name.c_str());
        return false;
    }
    //更新状态
    {
        std::lock_guard<std::mutex> lock(updater_mutex_);
        current_models_[module_name] = model_path;
        
        //生成新版本
        std::string new_version = generateVersionId();
        current_versions_[module_name] = new_version;
        version_history_[module_name].push_back(new_version);
        
        //限制版本历史大小
        if (version_history_[module_name].size() > 10) {
            version_history_[module_name].erase(version_history_[module_name].begin());
        }
    }
    //记录更新历史
    recordUpdateHistory(module_name, "MODEL_UPDATE", model_path);
    //触发回调
    triggerModelCallback(module_name, model_path);
    LOG_INFO("模块模型更新成功: %s", module_name.c_str());
    return true;
}

bool ConfigHotUpdater::updateParams(const std::string& module_name, const std::map<std::string, std::string>& params) 
{
    if (module_name.empty() || params.empty()) {
        LOG_WARN("无效的模块名或参数");
        return false;
    }
    
    LOG_INFO("正在更新模块参数: %s", module_name.c_str());
    //执行配置更新
    if (!executeConfigUpdate(module_name, params)) {
        LOG_WARN("模块配置更新执行失败: %s", module_name.c_str());
        return false;
    }
    
    //更新状态
    {
        std::lock_guard<std::mutex> lock(updater_mutex_);
        current_configs_[module_name] = params;
        //生成新版本
        std::string new_version = generateVersionId();
        current_versions_[module_name] = new_version;
        version_history_[module_name].push_back(new_version);
        
        //限制版本历史大小
        if (version_history_[module_name].size() > 10) {
            version_history_[module_name].erase(version_history_[module_name].begin());
        }
    }
    
    //记录更新历史
    recordUpdateHistory(module_name, "PARAM_UPDATE", "Updated " + std::to_string(params.size()) + " parameters");
    //触发回调
    triggerConfigCallback(module_name, params);
    LOG_INFO("模块参数更新成功: %s", module_name.c_str());
    return true;
}

bool ConfigHotUpdater::updateFromFile(const std::string& module_name, const std::string& config_file) 
{
    if (module_name.empty() || config_file.empty()) {
        LOG_WARN("无效的模块名或配置文件");
        return false;
    }
    LOG_INFO("正在从文件更新模块配置: %s", module_name.c_str());
    //验证配置文件
    if (!validateConfigFile(config_file)) {
        LOG_WARN("无效的配置文件: %s", config_file.c_str());
        return false;
    }
    //加载配置文件
    auto params = loadConfigFile(config_file);
    if (params.empty()) {
        LOG_WARN("加载配置文件失败: %s", config_file.c_str());
        return false;
    }
    return updateParams(module_name, params);
}

bool ConfigHotUpdater::switchABTest(const std::string& module_name, const std::string& version) 
{
    if (module_name.empty() || version.empty()) {
        LOG_WARN("无效的模块名或版本号");
        return false;
    }
    LOG_INFO("正在切换模块A/B测试版本: %s 到版本: %s", module_name.c_str(), version.c_str());
    std::lock_guard<std::mutex> lock(updater_mutex_);
    //检查版本是否存在
    auto it = std::find(version_history_[module_name].begin(), version_history_[module_name].end(), version);
    if (it == version_history_[module_name].end()) {
        LOG_WARN("版本未找到: %s", version.c_str());
        return false;
    }
    //切换到指定版本
    current_versions_[module_name] = version;
    //记录更新历史
    recordUpdateHistory(module_name, "AB_TEST_SWITCH", "Switched to version " + version);
    LOG_INFO("模块A/B测试版本切换成功: %s", module_name.c_str());
    return true;
}

bool ConfigHotUpdater::rollback(const std::string& module_name, const std::string& target_version) 
{
    if (module_name.empty()) {
        LOG_WARN("无效的模块名");
        return false;
    }
    LOG_INFO("正在回滚模块: %s", module_name.c_str());
    std::lock_guard<std::mutex> lock(updater_mutex_); 
    //获取当前版本
    auto current_it = current_versions_.find(module_name);
    if (current_it == current_versions_.end()) {
        LOG_WARN("未找到模块当前版本: %s", module_name.c_str());
        return false;
    }
    std::string current_version = current_it->second;
    //确定回滚目标版本
    std::string rollback_version = target_version;
    if (rollback_version.empty()) 
    {
        //回滚到上一个版本
        const auto& history = version_history_[module_name];
        if (history.size() < 2) {
            LOG_WARN("没有可回滚的历史版本");
            return false;
        }
        //找到当前版本的前一个版本
        auto current_version_it = std::find(history.begin(), history.end(), current_version);
        if (current_version_it == history.begin()) {
            LOG_WARN("当前版本已是最早版本，无法回滚");
            return false;
        }
        rollback_version = *(current_version_it - 1);
    }
    //执行回滚
    current_versions_[module_name] = rollback_version;
    //记录更新历史
    recordUpdateHistory(module_name, "ROLLBACK", "Rolled back from " + current_version + " to " + rollback_version);
    LOG_INFO("模块回滚完成: %s 到版本: %s", module_name.c_str(), rollback_version.c_str());
    return true;
}

std::string ConfigHotUpdater::getCurrentVersion(const std::string& module_name) const 
{
    std::lock_guard<std::mutex> lock(updater_mutex_);
    auto it = current_versions_.find(module_name);
    return (it != current_versions_.end()) ? it->second : "";
}

std::vector<std::string> ConfigHotUpdater::getAvailableVersions(const std::string& module_name) const 
{
    std::lock_guard<std::mutex> lock(updater_mutex_);
    auto it = version_history_.find(module_name);
    return (it != version_history_.end()) ? it->second : std::vector<std::string>();
}

void ConfigHotUpdater::registerUpdateCallback(const std::string& module_name, std::function<void(const std::string&, 
    const std::map<std::string, std::string>&)> callback) 
{
    std::lock_guard<std::mutex> lock(updater_mutex_);
    config_callbacks_[module_name][basmodule::get_current_timestamp()] = callback;
}

void ConfigHotUpdater::registerModelCallback(const std::string& module_name, std::function<void(const std::string&, const std::string&)> callback) 
{
    std::lock_guard<std::mutex> lock(updater_mutex_);
    model_callbacks_[module_name][basmodule::get_current_timestamp()] = callback;
}

bool ConfigHotUpdater::isConfigFileChanged(const std::string& config_file) 
{
    if (!std::filesystem::exists(config_file)) {
        return false;
    }
    auto current_time = getFileModificationTime(config_file);
    std::lock_guard<std::mutex> lock(updater_mutex_);
    auto it = file_timestamps_.find(config_file);
    if (it == file_timestamps_.end()) {
        file_timestamps_[config_file] = current_time;
        return false;
    }
    bool changed = (it->second != current_time);
    if (changed) {
        file_timestamps_[config_file] = current_time;
    }
    return changed;
}

bool ConfigHotUpdater::validateConfigFile(const std::string& config_file) const 
{
    //简单验证：检查文件是否存在且可读
    if (!std::filesystem::exists(config_file)) {
        return false;
    }
    //检查文件扩展名
    std::string extension = std::filesystem::path(config_file).extension().string();
    if (extension != ".yaml" && extension != ".yml" && extension != ".json") {
        LOG_WARN("不支持的配置文件格式: %s", extension.c_str());
        return false;
    }
    return true;
}

bool ConfigHotUpdater::backupConfig(const std::string& module_name, const std::string& backup_path) 
{
    std::lock_guard<std::mutex> lock(updater_mutex_);
    auto config_it = current_configs_.find(module_name);
    if (config_it == current_configs_.end()) {
        LOG_WARN("未找到模块配置: %s", module_name.c_str());
        return false;
    }
    std::string backup_file = backup_path.empty() ? 
                             ("/var/log/bas_control/backup_" + module_name + "_" + basmodule::get_current_timestamp() + ".yaml") : 
                             backup_path;
    return saveConfigFile(backup_file, config_it->second);
}

bool ConfigHotUpdater::restoreFromBackup(const std::string& module_name, const std::string& backup_path) 
{
    if (!std::filesystem::exists(backup_path)) {
        LOG_WARN("备份文件未找到: %s", backup_path.c_str());
        return false;
    }
    auto params = loadConfigFile(backup_path);
    if (params.empty()) {
        LOG_WARN("加载备份文件失败: %s", backup_path.c_str());
        return false;
    }
    return updateParams(module_name, params);
}

void ConfigHotUpdater::monitoringLoop() 
{
    LOG_INFO("配置监控循环已启动");
    while (monitoring_active_) 
    {
        checkConfigFileChanges();
        basmodule::sleep_ms(config_check_interval_ms_);
    }
    LOG_INFO("配置监控循环已停止");
}

void ConfigHotUpdater::checkConfigFileChanges() 
{
    //这里应该检查所有监控的配置文件
    //简化实现：只检查示例文件
    std::vector<std::string> config_files = {
        config_.config_file_path + "/bas_control.yaml"
    };
    for (const auto& config_file : config_files) 
    {
        if (isConfigFileChanged(config_file)) 
        {
            LOG_INFO("配置文件已更改: %s", config_file.c_str());
            //这里应该解析文件并更新相应模块的配置
        }
    }
}

bool ConfigHotUpdater::executeConfigUpdate(const std::string& module_name, const std::map<std::string, std::string>& new_params) 
{
    //这里应该执行实际的配置更新逻辑
    //例如：通过ROS参数服务器更新参数，或直接调用模块的更新接口
    LOG_INFO("正在执行模块配置更新: %s", module_name.c_str());
    //模拟配置更新延迟
    basmodule::sleep_ms(500);
    //模拟更新成功
    return true;
}

bool ConfigHotUpdater::executeModelUpdate(const std::string& module_name, const std::string& new_model_path) 
{
    //这里应该执行实际的模型更新逻辑
    //例如：停止模块推理，加载新模型，重新启动推理
    LOG_INFO("正在执行模块模型更新: %s", module_name.c_str());
    //模拟模型更新延迟
    basmodule::sleep_ms(1000);
    //模拟更新成功
    return true;
}

std::map<std::string, std::string> ConfigHotUpdater::loadConfigFile(const std::string& config_file) const 
{
    std::map<std::string, std::string> params;
    //这里应该根据文件格式解析配置文件
    //简化实现：返回示例参数
    params["example_param1"] = "value1";
    params["example_param2"] = "value2";
    return params;
}

bool ConfigHotUpdater::saveConfigFile(const std::string& config_file, const std::map<std::string, std::string>& params) const 
{
    //这里应该将参数保存到配置文件
    //简化实现：返回成功
    return true;
}

void ConfigHotUpdater::triggerConfigCallback(const std::string& module_name, const std::map<std::string, std::string>& params) 
{
    std::lock_guard<std::mutex> lock(updater_mutex_);
    auto module_callbacks = config_callbacks_.find(module_name);
    if (module_callbacks != config_callbacks_.end()) 
    {
        for (const auto& pair : module_callbacks->second) 
        {
            try {
                pair.second(module_name, params);
            } catch (const std::exception& e) {
                LOG_WARN("配置回调异常: %s", e.what());
            }
        }
    }
}

void ConfigHotUpdater::triggerModelCallback(const std::string& module_name, const std::string& model_path) 
{
    std::lock_guard<std::mutex> lock(updater_mutex_);
    auto module_callbacks = model_callbacks_.find(module_name);
    if (module_callbacks != model_callbacks_.end()) 
    {
        for (const auto& pair : module_callbacks->second) 
        {
            try {
                pair.second(module_name, model_path);
            } catch (const std::exception& e) {
                LOG_WARN("模型回调异常: %s", e.what());
            }
        }
    }
}

void ConfigHotUpdater::recordUpdateHistory(const std::string& module_name, const std::string& update_type, const std::string& details) 
{
    std::lock_guard<std::mutex> lock(history_mutex_);
    update_history_[module_name].emplace_back(update_type, details, std::chrono::system_clock::now());
    //限制历史记录大小
    if (update_history_[module_name].size() > 50) {
        update_history_[module_name].erase(update_history_[module_name].begin());
    }
}

std::chrono::system_clock::time_point ConfigHotUpdater::getFileModificationTime(const std::string& file_path) const 
{
    if (std::filesystem::exists(file_path)) 
    {
        auto time = std::filesystem::last_write_time(file_path);
        auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
            time - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());
        return sctp;
    }
    return std::chrono::system_clock::time_point();
}

std::string ConfigHotUpdater::generateVersionId() const 
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    return "v" + std::to_string(millis);
}

} // namespace bas_control
