#ifndef CAM_CONFIG_MGR__CAMERA_CONFIG_MANAGER_HPP_
#define CAM_CONFIG_MGR__CAMERA_CONFIG_MANAGER_HPP_
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <cam_config_mgr/cam_com_struct.hpp>
#include <log_system/log_macros.hpp>
// 添加线程同步所需的头文件
#include <mutex>
#include <shared_mutex>

namespace CamMgr
{
// 获取当前时间戳字符串
std::string get_current_time_str();

class CamConfigMgr
{
public:
    // 构造函数和析构函数
    CamConfigMgr();
    CamConfigMgr(const CamConfigMgr& other);  // 拷贝构造函数
    CamConfigMgr(CamConfigMgr&& other) noexcept;  // 移动构造函数
    ~CamConfigMgr();
    
    // 赋值运算符
    CamConfigMgr& operator=(const CamConfigMgr& other);  // 拷贝赋值运算符
    CamConfigMgr& operator=(CamConfigMgr&& other) noexcept;  // 移动赋值运算符
    
    bool initialize_camera_folders();

    bool create_default_config();//默认配置文件
    bool create_default_cam_sys();// 创建默认相机配置基础参数
    bool create_default_cam_info(int cam_id);
    bool create_default_sence_info(int cam_id, int sence_id); // 创建默认场景信息
    
    bool load_config(int mode = 0);//加载所有配置
    bool load_cam_config();// 加载相机配置基础参数
    bool load_cam_info(int cam_id); // 加载单个相机信息
    bool load_sence_info(int cam_id, int sence_id); // 加载某个相机下单个场景信息

    bool save_config();//保存所有配置
    bool save_cam_config();// 保存相机配置基础参数
    bool save_cam_info(int cam_id);// 保存单个相机信息
    bool save_sence_info(int cam_id, int sence_id);// 保存单个场景信息
    
    // 获取文件路径接口
    std::string get_cam_info_file(int cam_id); // 获取相机信息文件路径
    std::string get_sence_info_file(int cam_id, int sence_id); // 获取场景信息文件路径

    // 系统配置参数获取接口
    int get_cam_num() const { return cam_num_; }
    // 通过相机ID获取或设置相机配置
    bool get_camera_config(int cam_id, CamInfo& cam_info);//获取相机配置
    bool set_camera_config(int cam_id, const CamInfo& cam_info);//设置相机配置
    // 通过自定义相机名称获取或设置相机配置
    bool get_camera_config_by_name(const std::string& cam_user_name, CamInfo& cam_info);//获取相机配置
    bool set_camera_config_by_name(const std::string& cam_user_name, const CamInfo& cam_info);//设置相机配置
    // 通过序列号获取或设置相机配置
    bool get_camera_config_by_serial(const std::string& serial_number, CamInfo& cam_info);//获取相机配置
    bool set_camera_config_by_serial(const std::string& serial_number, const CamInfo& cam_info);//设置相机配置
    
public:
    const char* project_name_ = "cam_config_mgr"; // 项目名称
    
    std::string cam_cfg_dir_;//配置文件路径 
    std::string cam_sys_file_;//所有相机配置文件路径(包含相机数量、机械臂数量等系统配置)
    
    // 系统/相机配置参数
    int cam_num_;           // 相机数量
    int arm_num_;           // 手臂数量
    bool is_create_default_file_; // 加载时如果文件不存在是否创建默认参数
    std::map<int, CamInfo> camera_configs_;//cam_id对应的相机配置

private:
    // 为每个cam_id添加读写锁，用于保护文件写入操作
    mutable std::map<int, std::shared_mutex> cam_file_mutexes_;
    // 用于保护cam_file_mutexes_本身的锁
    mutable std::shared_mutex mutex_map_mutex_;
    
    // 获取指定cam_id的读写锁
    std::shared_mutex& get_cam_file_mutex(int cam_id) const;
};
}  // namespace CamMgr

#endif  // CAM_CONFIG_MGR__CAMERA_CONFIG_MANAGER_HPP_