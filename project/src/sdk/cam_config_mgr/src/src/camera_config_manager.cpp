#include "cam_config_mgr/camera_config_manager.hpp"
#include "bas_operate/file_operate.hpp"
#include "bas_operate/bas_utils.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <exception>
#include <nlohmann/json.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cctype>

namespace CamMgr
{
    CamConfigMgr::CamConfigMgr()
        : cam_num_(1),
          arm_num_(2),
          is_create_default_file_(true) // 默认为true，文件不存在时创建默认参数
    {
        // 默认相机配置文件路径设置
        std::string install_path = basmodule::get_install_dir();
        cam_cfg_dir_ = install_path + "/bas_config_data/cam_config/";
        cam_sys_file_ = cam_cfg_dir_ + "sys_cam_config.yaml"; // 全局配置文件路径

        // 确保cam_config目录存在
        if (!std::filesystem::exists(cam_cfg_dir_))
        {
            std::filesystem::create_directories(cam_cfg_dir_);
        }
        LOG_INFO( "相机配置文件路径: %s", cam_cfg_dir_.c_str()); // 添加日志
        LOG_INFO( "相机配置文件: %s", cam_sys_file_.c_str());    // 添加日志
    }
    
    // 拷贝构造函数
    CamConfigMgr::CamConfigMgr(const CamConfigMgr& other)
        : cam_num_(other.cam_num_),
          arm_num_(other.arm_num_),
          is_create_default_file_(other.is_create_default_file_),
          cam_cfg_dir_(other.cam_cfg_dir_),
          cam_sys_file_(other.cam_sys_file_),
          camera_configs_(other.camera_configs_)
    {
        LOG_INFO( "拷贝构造函数被调用");
    }

    // 移动构造函数
    CamConfigMgr::CamConfigMgr(CamConfigMgr&& other) noexcept
        : cam_num_(other.cam_num_),
          arm_num_(other.arm_num_),
          is_create_default_file_(other.is_create_default_file_),
          cam_cfg_dir_(std::move(other.cam_cfg_dir_)),
          cam_sys_file_(std::move(other.cam_sys_file_)),
          camera_configs_(std::move(other.camera_configs_)),
          cam_file_mutexes_(std::move(other.cam_file_mutexes_))
    {
        LOG_INFO( "移动构造函数被调用");
    }

    CamConfigMgr::~CamConfigMgr()
    {
        std::unique_lock<std::shared_mutex> lock(mutex_map_mutex_);
        cam_file_mutexes_.clear();
    }

    // 拷贝赋值运算符
    CamConfigMgr& CamConfigMgr::operator=(const CamConfigMgr& other)
    {
        if (this != &other)
        {
            // 先销毁当前对象的状态
            {
                std::unique_lock<std::shared_mutex> lock(mutex_map_mutex_);
                cam_file_mutexes_.clear();
            }

            // 复制其他对象的状态
            cam_num_ = other.cam_num_;
            arm_num_ = other.arm_num_;
            is_create_default_file_ = other.is_create_default_file_;
            cam_cfg_dir_ = other.cam_cfg_dir_;
            cam_sys_file_ = other.cam_sys_file_;
            camera_configs_ = other.camera_configs_;

            LOG_INFO( "拷贝赋值运算符被调用");
        }
        return *this;
    }

    // 移动赋值运算符
    CamConfigMgr& CamConfigMgr::operator=(CamConfigMgr&& other) noexcept
    {
        if (this != &other)
        {
            // 先销毁当前对象的状态
            {
                std::unique_lock<std::shared_mutex> lock(mutex_map_mutex_);
                cam_file_mutexes_.clear();
            }

            // 移动其他对象的状态
            cam_num_ = other.cam_num_;
            arm_num_ = other.arm_num_;
            is_create_default_file_ = other.is_create_default_file_;
            cam_cfg_dir_ = std::move(other.cam_cfg_dir_);
            cam_sys_file_ = std::move(other.cam_sys_file_);
            camera_configs_ = std::move(other.camera_configs_);
            cam_file_mutexes_ = std::move(other.cam_file_mutexes_);

            LOG_INFO( "移动赋值运算符被调用");
        }
        return *this;
    }

    // 实现获取指定cam_id的读写锁
    std::shared_mutex &CamConfigMgr::get_cam_file_mutex(int cam_id) const
    {
        // 获取mutex_map_mutex_的写锁，因为我们可能需要修改cam_file_mutexes_
        std::unique_lock<std::shared_mutex> lock(mutex_map_mutex_);
        // 使用operator[]查找或创建对应cam_id的mutex
        return cam_file_mutexes_[cam_id];
    }

    bool CamConfigMgr::load_config(int mode)
    {
        try
        {
            // 当mode!=1时加载全局配置
            if (mode != 1)
            {
                if (!load_cam_config())
                {
                    LOG_ERROR( "加载全局配置失败");
                    return false;
                }
            }
            // 初始化子目录文件
            initialize_camera_folders();
            // 加载各个相机配置（遍历已加载的相机 ID，而不是假设从 0 开始连续）
            for (const auto& pair : camera_configs_)
            {
                int cam_id = pair.first;
                if (!load_cam_info(cam_id))
                {
                    LOG_ERROR( "加载相机配置 %d 失败", cam_id);
                }
                // 加载各个场景配置
                auto it = camera_configs_.find(cam_id);
                if (it != camera_configs_.end())
                {
                    CamInfo& cam_info = it->second;
                    for (int j = 0; j < cam_info.sence_num; j++)
                    {
                        if (!load_sence_info(cam_id, j))
                        {
                            LOG_ERROR( "加载相机 %d 场景 %d 配置失败", cam_id, j);
                        }
                    }
                }
            }
            
            LOG_INFO( "配置加载完成");
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "加载配置时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamConfigMgr::load_cam_config()
    {
        try
        {
            std::string global_config_path = cam_cfg_dir_ + "sys_cam_config.yaml";
            if (!std::filesystem::exists(global_config_path))
            {
                LOG_ERROR( "全局配置文件不存在: %s", global_config_path.c_str());
                return false;
            }

            YAML::Node config = YAML::LoadFile(global_config_path);
            if (!config["sys_cam_config"])
            {
                LOG_ERROR( "全局配置文件格式错误，缺少sys_cam_config节点");
                return false;
            }

            YAML::Node sys_config_node = config["sys_cam_config"];
            cam_num_ = sys_config_node["cam_num"].as<int>(1);
            // arm_num_ 不再从sys_cam_config中读取，默认为1
            arm_num_ = 1;
            
            // 读取各个相机的配置信息
            for (int i = 0; i < cam_num_; i++) {
                std::string cam_key = "cam_" + std::to_string(i);
                if (config[cam_key]) {
                    YAML::Node cam_node = config[cam_key];
                    
                    // 初始化或获取相机配置
                    CamMgr::CamInfo cam_info;
                    auto it = camera_configs_.find(i);
                    if (it != camera_configs_.end()) {
                        cam_info = it->second;
                    } else {
                        // 设置默认值
                        cam_info.cam_id = i;
                        cam_info.cam_usr_name = "";
                        cam_info.serial_number = "";
                        cam_info.cam_type = CamMgr::CamType::CAM_TYPE_RS;
                        cam_info.cam_model = "";
                        cam_info.cam_index = 0;
                        cam_info.enable = true;
                        cam_info.show_topic_image = true;
                        cam_info.sence_num = 1;
                    }
                    
                    // 从全局配置中读取相机信息
                    if (cam_node["is_enable"]) {
                        cam_info.enable = cam_node["is_enable"].as<bool>(true);
                    }
                    if (cam_node["serial_number"]) {
                        cam_info.serial_number = cam_node["serial_number"].as<std::string>("");
                    }
                    if (cam_node["username"]) {
                        cam_info.cam_usr_name = cam_node["username"].as<std::string>("");
                    }
                    
                    // 存储到camera_configs_
                    camera_configs_[i] = cam_info;
                }
            }
            
            LOG_INFO( "成功加载全局配置");
            return true;
        }
        catch (const YAML::Exception &e)
        {
            LOG_ERROR( "YAML解析错误: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "加载全局配置时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamConfigMgr::load_cam_info(int cam_id)
    {
        try
        {
            std::string camera_config_path = get_cam_info_file(cam_id);
            if (!std::filesystem::exists(camera_config_path))
            {
                LOG_WARN( "相机配置文件不存在: %s", camera_config_path.c_str());
                // 如果设置了创建默认文件标志，则创建默认配置
                if (is_create_default_file_)
                {
                    LOG_INFO( "创建默认相机配置文件: %s", camera_config_path.c_str());
                    create_default_cam_info(cam_id);
                }
            }
            YAML::Node config = YAML::LoadFile(camera_config_path);

            CamMgr::CamInfo &cam_info=camera_configs_[cam_id];
            cam_info.cam_id = cam_id;
            // 优化相机类型解析，支持多种字符串表示形式
            CamMgr::CamType cam_type = CamMgr::CamType::CAM_TYPE_RS; // 默认为RealSense类型
            try
            { // 尝试直接解析为整数
                cam_type = static_cast<CamMgr::CamType>(config["cam_type"].as<int>(0));
            }
            catch (...)
            {
                // 如果解析为整数失败，尝试解析为字符串
                std::string cam_type_str = config["cam_type"].as<std::string>("0");

                // 转换为小写以便比较
                std::transform(cam_type_str.begin(), cam_type_str.end(), cam_type_str.begin(), ::tolower);

                // 根据字符串值确定相机类型
                if (cam_type_str == "0" || cam_type_str == "realsense" || cam_type_str == "rs")
                {
                    cam_type = CamMgr::CamType::CAM_TYPE_RS; // 0
                }
                else if (cam_type_str == "1" || cam_type_str == "orbbec" || cam_type_str == "ob")
                {
                    cam_type = CamMgr::CamType::CAM_TYPE_OB; // 1
                }
                else if (cam_type_str == "2" || cam_type_str == "CSI" || cam_type_str == "csi")
                {
                    cam_type = CamMgr::CamType::CAM_TYPE_CSI; // 2
                }
                else
                {
                    cam_type = CamMgr::CamType::CAM_TYPE_RS; // 默认为RealSense类型
                }
            }
            cam_info.cam_type = cam_type;
            cam_info.cam_model = config["cam_model"].as<std::string>("");
            cam_info.cam_index = config["cam_index"].as<int>(0);
            cam_info.sence_num = config["sence_num"].as<int>(1);
            cam_info.show_topic_image = config["show_topic_image"].as<bool>(true);
            
            LOG_INFO("成功加载相机cam_id= %s 的配置参数", std::to_string(cam_info.cam_id).c_str());
            return true;
        }
        catch (const YAML::Exception &e)
        {
            LOG_ERROR("YAML解析错误: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("加载相机信息时发生错误: %s", e.what());
            return false;
        }
    }


    bool CamConfigMgr::load_sence_info(int cam_id, int sence_id)
    {
        try
        {
            std::string sence_config_path = get_sence_info_file(cam_id, sence_id);
            if (!std::filesystem::exists(sence_config_path))
            {
                LOG_WARN( "场景配置文件不存在: %s", sence_config_path.c_str());
                // 如果设置了创建默认文件标志，则创建默认配置
                if (is_create_default_file_)
                {
                    LOG_INFO( "创建默认场景配置文件: %s", sence_config_path.c_str());
                    create_default_sence_info(cam_id, sence_id);
                }
            }

            YAML::Node config = YAML::LoadFile(sence_config_path);

            // 获取相机配置
            auto cam_it = camera_configs_.find(cam_id);
            if (cam_it == camera_configs_.end())
            {
                LOG_ERROR( "未找到相机配置，相机ID: %s", std::to_string(cam_id).c_str());
                return false;
            }
            
            CamMgr::CamInfo& cam_info = cam_it->second;
            CamMgr::CamSencePara sence_para;

            sence_para.enable_depth_stream = config["enable_depth_stream"].as<bool>(false);
            sence_para.enable_color_stream = config["enable_color_stream"].as<bool>(true);
            sence_para.enable_ir_stream = config["enable_ir_stream"].as<bool>(false);
            sence_para.enable_cloud_stream = config["enable_cloud_stream"].as<bool>(false);
            sence_para.enable_publish_intrinsics = config["enable_publish_intrinsics"].as<bool>(true);
            sence_para.enable_save_intrinsics = config["enable_save_intrinsics"].as<bool>(true);
            
            // 解析彩图流参数
            if (config["color_para"])
            {
                YAML::Node color_para = config["color_para"];
                sence_para.color_para.auto_exposure = color_para["auto_exposure"].as<bool>(true);
                sence_para.color_para.auto_white_balance = color_para["auto_white_balance"].as<bool>(true);
                sence_para.color_para.width = color_para["width"].as<int>(640);
                sence_para.color_para.height = color_para["height"].as<int>(480);
                sence_para.color_para.fps = color_para["fps"].as<int>(30);
                sence_para.color_para.exposure = color_para["exposure"].as<int>(200);
                sence_para.color_para.gain = color_para["gain"].as<int>(0);
            }
            
            // 解析深度流参数
            if (config["depth_para"])
            {
                YAML::Node depth_para = config["depth_para"];
                sence_para.depth_para.auto_exposure = depth_para["auto_exposure"].as<bool>(true);
                sence_para.depth_para.width = depth_para["width"].as<int>(640);
                sence_para.depth_para.height = depth_para["height"].as<int>(480);
                sence_para.depth_para.fps = depth_para["fps"].as<int>(30);
                sence_para.depth_para.exposure = depth_para["exposure"].as<int>(200);
                sence_para.depth_para.gain = depth_para["gain"].as<int>(0);
            }

            // 存储场景参数
            cam_info.sence_para[sence_id] = sence_para;

            LOG_INFO( "成功加载场景配置，相机ID: %s, 场景ID: %s", std::to_string(cam_id).c_str(), std::to_string(sence_id).c_str());
            return true;
        }
        catch (const YAML::Exception &e)
        {
            LOG_ERROR( "YAML解析错误: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "加载场景信息时发生错误: %s", e.what());
            return false;
        }
    }


    bool CamConfigMgr::save_config()
    {
        try
        {
            // 保存全局配置
            if (!save_cam_config())
            {
                LOG_ERROR( "保存全局配置失败");
                return false;
            }

            // 保存各个相机配置
            for (const auto &pair : camera_configs_)
            {
                int cam_id = pair.first;
                if (!save_cam_info(cam_id))
                {
                    LOG_ERROR( "保存相机配置 %s 失败", std::to_string(cam_id).c_str());
                    return false;
                }
                
                // 保存各个场景配置
                const CamInfo& cam_info = pair.second;
                for (const auto& sence_pair : cam_info.sence_para)
                {
                    int sence_id = sence_pair.first;
                    if (!save_sence_info(cam_id, sence_id))
                    {
                        LOG_ERROR( "保存相机 %s 场景 %s 配置失败", std::to_string(cam_id).c_str(), std::to_string(sence_id).c_str());
                        return false;
                    }
                }
            }

            LOG_INFO( "配置保存完成");
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "保存配置时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamConfigMgr::save_cam_config()
    {
        try
        {
            // 创建cam_config目录（如果不存在）
            if (!std::filesystem::exists(cam_cfg_dir_))
            {
                std::filesystem::create_directories(cam_cfg_dir_);
            }

            // 保存全局配置文件 (sys_cam_config.yaml)，参考 install/sys_config/bas_sys_config/cam_config/sys_cam_config.yaml
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            
            // 系统配置部分
            emitter << YAML::Key << "sys_cam_config" << YAML::Value;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "cam_num" << YAML::Value << cam_num_;
            emitter << YAML::EndMap;
            
            // 相机配置部分（示例）
            for (const auto& pair : camera_configs_)
            {
                int cam_id = pair.first;
                const CamMgr::CamInfo& cam_info = pair.second;
                
                emitter << YAML::Key << "cam_" + std::to_string(cam_id) << YAML::Value;
                emitter << YAML::BeginMap;
                emitter << YAML::Key << "is_enable" << YAML::Value << cam_info.enable;
                emitter << YAML::Key << "serial_number" << YAML::Value << cam_info.serial_number;
                emitter << YAML::Key << "username" << YAML::Value << cam_info.cam_usr_name;
                emitter << YAML::EndMap;
            }
            
            emitter << YAML::EndMap;

            std::string global_config_path = cam_cfg_dir_ + "sys_cam_config.yaml";
            std::ofstream fout(global_config_path);
            fout << emitter.c_str();
            fout.close();

            LOG_INFO( "全局配置已保存到: %s", global_config_path.c_str());
            return true;
        }
        catch (const YAML::Exception &e)
        {
            LOG_ERROR( "YAML保存错误: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "保存全局配置时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamConfigMgr::save_cam_info(int cam_id)
    {
        try
        {
            auto it = camera_configs_.find(cam_id);
            if (it == camera_configs_.end())
            {
                LOG_ERROR( "未找到相机ID %s 的配置", std::to_string(cam_id).c_str());
                return false;
            }

            const CamMgr::CamInfo &cam_info = it->second;

            // 创建相机目录（如果不存在）
            std::string camera_folder_path = cam_cfg_dir_ + "cam_" + std::to_string(cam_id);
            if (!std::filesystem::exists(camera_folder_path))
            {
                std::filesystem::create_directories(camera_folder_path);
            }

            // 保存相机配置
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "cam_id" << YAML::Value << cam_info.cam_id;
            emitter << YAML::Key << "cam_usr_name" << YAML::Value << cam_info.cam_usr_name;
            emitter << YAML::Key << "serial_number" << YAML::Value << cam_info.serial_number;
            emitter << YAML::Key << "cam_type" << YAML::Value << static_cast<int>(cam_info.cam_type);
            emitter << YAML::Key << "cam_model" << YAML::Value << cam_info.cam_model;
            emitter << YAML::Key << "cam_index" << YAML::Value << cam_info.cam_index;
            emitter << YAML::Key << "enable" << YAML::Value << cam_info.enable;
            emitter << YAML::Key << "show_topic_image" << YAML::Value << cam_info.show_topic_image;
            emitter << YAML::Key << "sence_num" << YAML::Value << cam_info.sence_num;
            emitter << YAML::EndMap;

            std::string camera_config_path = get_cam_info_file(cam_id);
            std::ofstream fout(camera_config_path);
            fout << emitter.c_str();
            fout.close();

            LOG_INFO( "相机 %s 配置已保存到: %s", std::to_string(cam_id).c_str(), camera_config_path.c_str());
            return true;
        }
        catch (const YAML::Exception &e)
        {
            LOG_ERROR( "YAML保存错误: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "保存相机配置时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamConfigMgr::save_sence_info(int cam_id, int sence_id)
    {
        try
        {
            auto cam_it = camera_configs_.find(cam_id);
            if (cam_it == camera_configs_.end())
            {
                LOG_ERROR( "未找到相机ID %s 的配置", std::to_string(cam_id).c_str());
                return false;
            }

            const CamMgr::CamInfo &cam_info = cam_it->second;
            auto sence_it = cam_info.sence_para.find(sence_id);
            if (sence_it == cam_info.sence_para.end())
            {
                LOG_ERROR( "未找到相机 %s 场景 %s 的配置", std::to_string(cam_id).c_str(), std::to_string(sence_id).c_str());
                return false;
            }

            const CamMgr::CamSencePara &sence_para = sence_it->second;

            // 创建场景目录（如果不存在）
            std::string sence_folder_path = cam_cfg_dir_ + "cam_" + std::to_string(cam_id) + "/sence_" + std::to_string(sence_id);
            if (!std::filesystem::exists(sence_folder_path))
            {
                std::filesystem::create_directories(sence_folder_path);
            }

            // 保存场景配置
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "enable_depth_stream" << YAML::Value << sence_para.enable_depth_stream;
            emitter << YAML::Key << "enable_color_stream" << YAML::Value << sence_para.enable_color_stream;
            emitter << YAML::Key << "enable_ir_stream" << YAML::Value << sence_para.enable_ir_stream;
            emitter << YAML::Key << "enable_cloud_stream" << YAML::Value << sence_para.enable_cloud_stream;
            emitter << YAML::Key << "enable_publish_intrinsics" << YAML::Value << sence_para.enable_publish_intrinsics;
            emitter << YAML::Key << "enable_save_intrinsics" << YAML::Value << sence_para.enable_save_intrinsics;

            emitter << YAML::Key << "color_para" << YAML::Value;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "auto_exposure" << YAML::Value << sence_para.color_para.auto_exposure;
            emitter << YAML::Key << "auto_white_balance" << YAML::Value << sence_para.color_para.auto_white_balance;
            emitter << YAML::Key << "width" << YAML::Value << sence_para.color_para.width;
            emitter << YAML::Key << "height" << YAML::Value << sence_para.color_para.height;
            emitter << YAML::Key << "fps" << YAML::Value << sence_para.color_para.fps;
            emitter << YAML::Key << "exposure" << YAML::Value << static_cast<int>(sence_para.color_para.exposure);
            emitter << YAML::Key << "gain" << YAML::Value << static_cast<int>(sence_para.color_para.gain);
            emitter << YAML::EndMap;

            emitter << YAML::Key << "depth_para" << YAML::Value;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "auto_exposure" << YAML::Value << sence_para.depth_para.auto_exposure;
            emitter << YAML::Key << "width" << YAML::Value << sence_para.depth_para.width;
            emitter << YAML::Key << "height" << YAML::Value << sence_para.depth_para.height;
            emitter << YAML::Key << "fps" << YAML::Value << sence_para.depth_para.fps;
            emitter << YAML::Key << "exposure" << YAML::Value << static_cast<int>(sence_para.depth_para.exposure);
            emitter << YAML::Key << "gain" << YAML::Value << static_cast<int>(sence_para.depth_para.gain);
            emitter << YAML::EndMap;

            emitter << YAML::EndMap;

            std::string sence_config_path = get_sence_info_file(cam_id, sence_id);
            std::ofstream fout(sence_config_path);
            fout << emitter.c_str();
            fout.close();

            LOG_INFO( "相机 %s 场景 %s 配置已保存到: %s", std::to_string(cam_id).c_str(), std::to_string(sence_id).c_str(), sence_config_path.c_str());
            return true;
        }
        catch (const YAML::Exception &e)
        {
            LOG_ERROR( "YAML保存错误: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "保存场景配置时发生错误: %s", e.what());
            return false;
        }
    }

    // 在 camera_config_manager.cpp 中实现


    bool CamConfigMgr::create_default_config()
    {
        try
        {
            // 创建默认全局配置
            if (!create_default_cam_sys())
            {
                LOG_ERROR( "创建默认全局配置失败");
                return false;
            }
            
            // 创建默认相机配置（根据cam_num_创建对应数量的相机配置）
            for (int i = 0; i < cam_num_; i++)
            {
                if (!create_default_cam_info(i))
                {
                            LOG_ERROR( "创建默认相机配置 %s 失败", std::to_string(i).c_str());
                    return false;
                }
                
                // 创建默认场景配置（根据sence_num创建对应数量的场景配置）
                auto it = camera_configs_.find(i);
                if (it != camera_configs_.end())
                {
                    const CamInfo& cam_info = it->second;
                    for (int j = 0; j < cam_info.sence_num; j++)
                    {
                        if (!create_default_sence_info(i, j))
                        {
                            LOG_ERROR( "创建默认场景配置 相机%s 场景%s 失败", std::to_string(i).c_str(), std::to_string(j).c_str());
                            return false;
                        }
                    }
                }
            }

            LOG_INFO( "默认配置创建成功");
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "创建默认配置时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamConfigMgr::create_default_cam_sys()
    {
        try
        {
            // 创建cam_config目录（如果不存在）
            if (!std::filesystem::exists(cam_cfg_dir_))
            {
                std::filesystem::create_directories(cam_cfg_dir_);
            }

            // 创建默认全局配置，参考 install/sys_config/bas_sys_config/cam_config/sys_cam_config.yaml
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            
            // 系统配置部分
            emitter << YAML::Key << "sys_cam_config" << YAML::Value;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "cam_num" << YAML::Value << cam_num_;
            emitter << YAML::EndMap;
            
            // 创建默认相机配置（3个相机示例）
            for (int i = 0; i < std::min(cam_num_, 3); i++)
            {
                emitter << YAML::Key << "cam_" + std::to_string(i) << YAML::Value;
                emitter << YAML::BeginMap;
                emitter << YAML::Key << "is_enable" << YAML::Value << (i > 0); // 只有cam_1和cam_2默认启用
                emitter << YAML::Key << "serial_number" << YAML::Value << "33622207229" + std::to_string(i);
                emitter << YAML::Key << "username" << YAML::Value << "camera_" + std::to_string(i);
                
                // arm_info数组
                emitter << YAML::Key << "arm_info" << YAML::Value;
                emitter << YAML::Flow;
                emitter << YAML::BeginSeq;
                if (i == 0)
                {
                    emitter << 0 << 1; // head_camera关联两个机械臂
                }
                else
                {
                    emitter << (i - 1); // 其他相机关联对应的机械臂
                }
                emitter << YAML::EndSeq;
                emitter << YAML::EndMap;
            }
            
            emitter << YAML::EndMap;

            std::string global_config_path = cam_cfg_dir_ + "sys_cam_config.yaml";
            std::ofstream fout(global_config_path);
            fout << emitter.c_str();
            fout.close();

            LOG_INFO( "默认全局配置已创建到: %s", global_config_path.c_str());
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "创建默认全局配置时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamConfigMgr::create_default_cam_info(int cam_id)
    {
        try
        {
            // 创建相机目录（如果不存在）
            std::string camera_folder_path = cam_cfg_dir_ + "cam_" + std::to_string(cam_id);
            if (!std::filesystem::exists(camera_folder_path))
            {
                std::filesystem::create_directories(camera_folder_path);
            }

            // 创建默认相机配置
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "cam_id" << YAML::Value << cam_id;
            emitter << YAML::Key << "enable" << YAML::Value << true;
            emitter << YAML::Key << "show_topic_image" << YAML::Value << true;
            emitter << YAML::Key << "cam_usr_name" << YAML::Value << "";
            emitter << YAML::Key << "serial_number" << YAML::Value << "";
            emitter << YAML::Key << "cam_type" << YAML::Value << 0;
            emitter << YAML::Key << "cam_model" << YAML::Value << "";
            emitter << YAML::Key << "cam_index" << YAML::Value << 0;
            emitter << YAML::Key << "sence_num" << YAML::Value << 1;
            emitter << YAML::EndMap;
            std::string camera_config_path = get_cam_info_file(cam_id);
            std::ofstream fout(camera_config_path);
            fout << emitter.c_str();
            fout.close();

            // 更新内存中的配置
            CamMgr::CamInfo cam_info;
            cam_info.cam_id = cam_id;
            cam_info.enable = true;
            cam_info.show_topic_image = true;
            cam_info.cam_usr_name = "";
            cam_info.serial_number = "";
            cam_info.cam_type = CamMgr::CamType::CAM_TYPE_RS;
            cam_info.cam_model = "";
            cam_info.cam_index = 0;
            cam_info.sence_num = 1;
            
            camera_configs_[cam_id] = cam_info;            LOG_INFO( "默认相机配置 %s 已创建到: %s", std::to_string(cam_id).c_str(), camera_config_path.c_str());
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "创建默认相机配置时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamConfigMgr::create_default_sence_info(int cam_id, int sence_id)
    {
        try
        {
            // 创建场景目录（如果不存在）
            std::string sence_folder_path = cam_cfg_dir_ + "cam_" + std::to_string(cam_id) + "/sence_" + std::to_string(sence_id);
            if (!std::filesystem::exists(sence_folder_path))
            {
                std::filesystem::create_directories(sence_folder_path);
            }

            // 创建默认场景配置
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "enable_depth_stream" << YAML::Value << false;
            emitter << YAML::Key << "enable_color_stream" << YAML::Value << true;
            emitter << YAML::Key << "enable_ir_stream" << YAML::Value << false;
            emitter << YAML::Key << "enable_cloud_stream" << YAML::Value << false;
            emitter << YAML::Key << "enable_publish_intrinsics" << YAML::Value << true;
            emitter << YAML::Key << "enable_save_intrinsics" << YAML::Value << true;

            emitter << YAML::Key << "color_para" << YAML::Value;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "auto_exposure" << YAML::Value << true;
            emitter << YAML::Key << "auto_white_balance" << YAML::Value << true;
            emitter << YAML::Key << "width" << YAML::Value << 640;
            emitter << YAML::Key << "height" << YAML::Value << 480;
            emitter << YAML::Key << "fps" << YAML::Value << 30;
            emitter << YAML::EndMap;

            emitter << YAML::Key << "depth_para" << YAML::Value;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "auto_exposure" << YAML::Value << true;
            emitter << YAML::Key << "width" << YAML::Value << 640;
            emitter << YAML::Key << "height" << YAML::Value << 480;
            emitter << YAML::Key << "fps" << YAML::Value << 30;
            emitter << YAML::EndMap;

            emitter << YAML::EndMap;

            std::string sence_config_path = get_sence_info_file(cam_id, sence_id);
            std::ofstream fout(sence_config_path);
            fout << emitter.c_str();
            fout.close();

            // 更新内存中的配置
            auto cam_it = camera_configs_.find(cam_id);
            if (cam_it != camera_configs_.end())
            {
                CamMgr::CamSencePara sence_para;
                sence_para.enable_depth_stream = false;
                sence_para.enable_color_stream = true;
                sence_para.enable_ir_stream = false;
                sence_para.enable_cloud_stream = false;
                sence_para.enable_publish_intrinsics = true;
                sence_para.enable_save_intrinsics = true;
                
                // 设置默认参数
                sence_para.color_para.auto_exposure = true;
                sence_para.color_para.auto_white_balance = true;
                sence_para.color_para.width = 640;
                sence_para.color_para.height = 480;
                sence_para.color_para.fps = 30;
                
                sence_para.depth_para.auto_exposure = true;
                sence_para.depth_para.width = 640;
                sence_para.depth_para.height = 480;
                sence_para.depth_para.fps = 30;
                
                cam_it->second.sence_para[sence_id] = sence_para;
            }

            LOG_INFO( "默认场景配置 相机%s 场景%s 已创建到: %s", std::to_string(cam_id).c_str(), std::to_string(sence_id).c_str(), sence_config_path.c_str());
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "创建默认场景配置时发生错误: %s", e.what());
            return false;
        }
    }

    
    std::string CamConfigMgr::get_cam_info_file(int cam_id)
    {
        return cam_cfg_dir_ + "cam_" + std::to_string(cam_id) + "/cam_" + std::to_string(cam_id) + "_info.yaml";
    }
    
    std::string CamConfigMgr::get_sence_info_file(int cam_id, int sence_id)
    {
        return cam_cfg_dir_ + "cam_" + std::to_string(cam_id) + "/sence_" + std::to_string(sence_id) + "/cam_" + std::to_string(cam_id) + "_sence_" + std::to_string(sence_id) + "_info.yaml";
    }

    bool CamConfigMgr::initialize_camera_folders()
    {
        try
        {
            // 根据相机数量创建对应的文件夹
            for (int i = 0; i < cam_num_; i++)
            {
                std::string camera_folder_path = cam_cfg_dir_ + "cam_" + std::to_string(i);
                if (!std::filesystem::exists(camera_folder_path))
                {
                    std::filesystem::create_directory(camera_folder_path);
                    LOG_INFO( "创建相机文件夹: %s", camera_folder_path.c_str());
                }
                for (int j = 0; j < arm_num_; j++)
                {
                    std::string arm1_path = camera_folder_path + "/cam" + std::to_string(i) + "_arm" + std::to_string(j);
                    if (!std::filesystem::exists(arm1_path))
                    {
                        std::filesystem::create_directory(arm1_path);
                        LOG_INFO( "创建数据文件夹: %s", arm1_path.c_str());
                    }
                    std::string arm1_path_test = camera_folder_path + "/cam" + std::to_string(i) + "_arm" + std::to_string(j) + "/test_ouput_calib_data";
                    if (!std::filesystem::exists(arm1_path_test))
                    {
                        std::filesystem::create_directory(arm1_path_test);
                        LOG_INFO( "创建数据文件夹: %s", arm1_path_test.c_str());
                    }
                }
            }
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "创建相机文件夹时发生错误: %s", e.what());
            return false;
        }
    }

    // 通过相机ID获取或设置相机配置
    bool CamConfigMgr::get_camera_config(int cam_id, CamInfo &cam_info)
    {
        try
        {
            auto it = camera_configs_.find(cam_id);
            if (it != camera_configs_.end())
            {
                cam_info = it->second;
                return true;
            }
            LOG_WARN( "未找到相机配置，相机ID: %s", std::to_string(cam_id).c_str());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "获取相机配置时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamConfigMgr::set_camera_config(int cam_id, const CamInfo &cam_info)
    {
        try
        {
            camera_configs_[cam_id] = cam_info;
            LOG_INFO( "成功设置相机配置，相机ID: %s", std::to_string(cam_id).c_str());
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "设置相机配置时发生错误: %s", e.what());
            return false;
        }
    }

    // 通过自定义相机名称获取或设置相机配置
    bool CamConfigMgr::get_camera_config_by_name(const std::string &cam_user_name, CamInfo &cam_info)
    {
        try
        {
            for (const auto &pair : camera_configs_)
            {
                if (pair.second.cam_usr_name == cam_user_name)
                {
                    cam_info = pair.second;
                    LOG_INFO( "成功通过名称获取相机配置: %s", cam_user_name.c_str());
                    return true;
                }
            }
            LOG_WARN( "未找到相机配置，相机名称: %s", cam_user_name.c_str());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "通过名称获取相机配置时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamConfigMgr::set_camera_config_by_name(const std::string &cam_user_name, const CamInfo &cam_info)
    {
        try
        {
            for (auto &pair : camera_configs_)
            {
                if (pair.second.cam_usr_name == cam_user_name)
                {
                    pair.second = cam_info;
                    LOG_INFO( "成功通过名称设置相机配置: %s", cam_user_name.c_str());
                    return true;
                }
            }
            LOG_WARN( "未找到相机配置，相机名称: %s", cam_user_name.c_str());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "通过名称设置相机配置时发生错误: %s", e.what());
            return false;
        }
    }

    // 通过序列号获取或设置相机配置
    bool CamConfigMgr::get_camera_config_by_serial(const std::string &serial_number, CamInfo &cam_info)
    {
        try
        {
            for (const auto &pair : camera_configs_)
            {
                if (pair.second.serial_number == serial_number)
                {
                    cam_info = pair.second;
                    LOG_INFO( "成功通过序列号获取相机配置: %s", serial_number.c_str());
                    return true;
                }
            }
            LOG_WARN( "未找到相机配置，序列号: %s", serial_number.c_str());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "通过序列号获取相机配置时发生错误: %s", e.what());
            return false;
        }
    }

    bool CamConfigMgr::set_camera_config_by_serial(const std::string &serial_number, const CamInfo &cam_info)
    {
        try
        {
            for (auto &pair : camera_configs_)
            {
                if (pair.second.serial_number == serial_number)
                {
                    pair.second = cam_info;
                    LOG_INFO( "成功通过序列号设置相机配置: %s", serial_number.c_str());
                    return true;
                }
            }
            LOG_WARN( "未找到相机配置，序列号: %s", serial_number.c_str());
            return false;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR( "通过序列号设置相机配置时发生错误: %s", e.what());
            return false;
        }
    }

} // namespace CamMgr