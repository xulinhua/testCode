#ifndef PCL_CONFIG_MANAGER_HPP_
#define PCL_CONFIG_MANAGER_HPP_

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <pcl2laserscan_trans/pcl_config_struct.hpp>
#include <mutex>
#include <shared_mutex>
#include <memory>

namespace PclConfig
{
    class PclConfigMgr
    {
    public:
        // 构造函数和析构函数
        PclConfigMgr();
        PclConfigMgr(const std::string& config_file_path);
        PclConfigMgr(const PclConfigMgr& other);  // 拷贝构造函数
        PclConfigMgr(PclConfigMgr&& other) noexcept;  // 移动构造函数
        ~PclConfigMgr();
        
        // 赋值运算符
        PclConfigMgr& operator=(const PclConfigMgr& other);  // 拷贝赋值运算符
        PclConfigMgr& operator=(PclConfigMgr&& other) noexcept;  // 移动赋值运算符
        
        // 初始化和加载配置
        bool initialize(const std::string& config_file_path = "");
        bool load_config();
        
        // 保存配置
        bool save_config();
        
        // 获取配置
        const PclConfig& get_config() const;
        
        // 获取特定模块的配置
        const SystemConfig& get_system_config() const;
        const LaunchConfig& get_launch_config() const;
        const CameraConfig& get_camera_config() const;
        const LidarConfig& get_lidar_config() const;
        const TFConfig& get_tf_config() const;
        
        // 设置配置
        void set_config(const PclConfig& config);
        
        // 获取配置文件路径
        std::string get_config_file_path() const;
        
    public:
        const char* project_name_ = "pcl_config_mgr"; // 项目名称
        
    private:
        // 配置文件路径
        std::string config_file_path_;
        
        // 配置数据
        PclConfig config_;
        
        // 读写锁，用于保护配置数据
        mutable std::shared_mutex config_mutex_;
        
        // 从YAML加载配置
        bool load_from_yaml(const YAML::Node& node);
        
        // 保存配置到YAML
        bool save_to_yaml(YAML::Emitter& emitter) const;
        
        // 创建默认配置
        void create_default_config();
    };
}  // namespace PclConfig

#endif  // PCL_CONFIG_MANAGER_HPP_