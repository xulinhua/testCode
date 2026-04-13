#include <pcl2laserscan_trans/pcl_config_manager.hpp>
#include <fstream>
#include <iostream>
#include <log_system/log_macros.hpp>

namespace PclConfig
{
    PclConfigMgr::PclConfigMgr()
    {
        create_default_config();
    }
    
    PclConfigMgr::PclConfigMgr(const std::string& config_file_path)
    {
        config_file_path_ = config_file_path;
        create_default_config();
        load_config();
    }
    
    PclConfigMgr::PclConfigMgr(const PclConfigMgr& other)
    {
        std::shared_lock<std::shared_mutex> lock(other.config_mutex_);
        config_file_path_ = other.config_file_path_;
        config_ = other.config_;
    }
    
    PclConfigMgr::PclConfigMgr(PclConfigMgr&& other) noexcept
    {
        std::shared_lock<std::shared_mutex> lock(other.config_mutex_);
        config_file_path_ = std::move(other.config_file_path_);
        config_ = std::move(other.config_);
    }
    
    PclConfigMgr::~PclConfigMgr()
    {
    }
    
    PclConfigMgr& PclConfigMgr::operator=(const PclConfigMgr& other)
    {
        if (this != &other)
        {
            std::unique_lock<std::shared_mutex> write_lock(config_mutex_);
            std::shared_lock<std::shared_mutex> read_lock(other.config_mutex_);
            config_file_path_ = other.config_file_path_;
            config_ = other.config_;
        }
        return *this;
    }
    
    PclConfigMgr& PclConfigMgr::operator=(PclConfigMgr&& other) noexcept
    {
        if (this != &other)
        {
            std::unique_lock<std::shared_mutex> write_lock(config_mutex_);
            std::shared_lock<std::shared_mutex> read_lock(other.config_mutex_);
            config_file_path_ = std::move(other.config_file_path_);
            config_ = std::move(other.config_);
        }
        return *this;
    }
    
    bool PclConfigMgr::initialize(const std::string& config_file_path)
    {
        if (!config_file_path.empty())
        {
            config_file_path_ = config_file_path;
        }
        
        if (config_file_path_.empty())
        {
            LOG_ERROR("配置文件路径为空");
            return false;
        }
        
        return load_config();
    }
    
    bool PclConfigMgr::load_config()
    {
        if (config_file_path_.empty())
        {
            LOG_ERROR("配置文件路径为空");
            return false;
        }
        
        try
        {
            YAML::Node node = YAML::LoadFile(config_file_path_);
            if (load_from_yaml(node))
            {
                LOG_INFO("配置成功从 %s 加载", config_file_path_.c_str());
                return true;
            }
            else
            {
                LOG_ERROR("从 YAML 加载配置失败");
                return false;
            }
        }
        catch (const YAML::Exception& e)
        {
            LOG_ERROR("YAML 异常：%s", e.what());
            // 如果加载失败，使用默认配置
            create_default_config();
            return false;
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("异常：%s", e.what());
            // 如果加载失败，使用默认配置
            create_default_config();
            return false;
        }
    }
    
    bool PclConfigMgr::save_config()
    {
        if (config_file_path_.empty())
        {
            LOG_ERROR("配置文件路径为空");
            return false;
        }
        
        try
        {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            
            if (save_to_yaml(emitter))
            {
                emitter << YAML::EndMap;
                
                std::ofstream file(config_file_path_);
                if (file.is_open())
                {
                    file << emitter.c_str();
                    file.close();
                    LOG_INFO("配置成功保存到 %s", config_file_path_.c_str());
                    return true;
                }
                else
                {
                    LOG_ERROR("无法打开配置文件进行写入");
                    return false;
                }
            }
            else
            {
                LOG_ERROR("保存配置到 YAML 失败");
                return false;
            }
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("异常：%s", e.what());
            return false;
        }
    }
    
    const PclConfig& PclConfigMgr::get_config() const
    {
        std::shared_lock<std::shared_mutex> lock(config_mutex_);
        return config_;
    }
    
    const SystemConfig& PclConfigMgr::get_system_config() const
    {
        std::shared_lock<std::shared_mutex> lock(config_mutex_);
        return config_.system;
    }
    
    const LaunchConfig& PclConfigMgr::get_launch_config() const
    {
        std::shared_lock<std::shared_mutex> lock(config_mutex_);
        return config_.launch;
    }
    
    const CameraConfig& PclConfigMgr::get_camera_config() const
    {
        std::shared_lock<std::shared_mutex> lock(config_mutex_);
        return config_.camera;
    }
    
    const LidarConfig& PclConfigMgr::get_lidar_config() const
    {
        std::shared_lock<std::shared_mutex> lock(config_mutex_);
        return config_.lidar;
    }
    
    const TFConfig& PclConfigMgr::get_tf_config() const
    {
        std::shared_lock<std::shared_mutex> lock(config_mutex_);
        return config_.tf;
    }
    
    void PclConfigMgr::set_config(const PclConfig& config)
    {
        std::unique_lock<std::shared_mutex> lock(config_mutex_);
        config_ = config;
    }
    
    std::string PclConfigMgr::get_config_file_path() const
    {
        std::shared_lock<std::shared_mutex> lock(config_mutex_);
        return config_file_path_;
    }
    
    bool PclConfigMgr::load_from_yaml(const YAML::Node& node)
    {
        try
        {
            // 加载系统参数
            if (node["system"])
            {
                if (node["system"]["log_dir"])
                {
                    config_.system.log_dir = node["system"]["log_dir"].as<std::string>();
                }
            }
            
            // 加载启动参数
            if (node["launch"])
            {
                if (node["launch"]["default_mode"])
                {
                    config_.launch.default_mode = node["launch"]["default_mode"].as<std::string>();
                }
                if (node["launch"]["camera_input_topic"])
                {
                    config_.launch.camera_input_topic = node["launch"]["camera_input_topic"].as<std::string>();
                }
                if (node["launch"]["camera_output_topic"])
                {
                    config_.launch.camera_output_topic = node["launch"]["camera_output_topic"].as<std::string>();
                }
                if (node["launch"]["lidar_input_topic"])
                {
                    config_.launch.lidar_input_topic = node["launch"]["lidar_input_topic"].as<std::string>();
                }
                if (node["launch"]["lidar_output_topic"])
                {
                    config_.launch.lidar_output_topic = node["launch"]["lidar_output_topic"].as<std::string>();
                }
            }
            
            // 加载相机参数
            if (node["camera"])
            {
                if (node["camera"]["target_frame"])
                {
                    config_.camera.target_frame = node["camera"]["target_frame"].as<std::string>();
                }
                if (node["camera"]["min_height"])
                {
                    config_.camera.min_height = node["camera"]["min_height"].as<float>();
                }
                if (node["camera"]["max_height"])
                {
                    config_.camera.max_height = node["camera"]["max_height"].as<float>();
                }
                if (node["camera"]["bSaveLogInfo2Files"])
                {
                    config_.camera.bSaveLogInfo2Files = node["camera"]["bSaveLogInfo2Files"].as<bool>();
                }
                if (node["camera"]["bOutputToTerminal"])
                {
                    config_.camera.bOutputToTerminal = node["camera"]["bOutputToTerminal"].as<bool>();
                }
                if (node["camera"]["range_min"])
                {
                    config_.camera.range_min = node["camera"]["range_min"].as<float>();
                }
                if (node["camera"]["range_max"])
                {
                    config_.camera.range_max = node["camera"]["range_max"].as<float>();
                }
                if (node["camera"]["angle_min"])
                {
                    config_.camera.angle_min = node["camera"]["angle_min"].as<float>();
                }
                if (node["camera"]["angle_max"])
                {
                    config_.camera.angle_max = node["camera"]["angle_max"].as<float>();
                }
                if (node["camera"]["angle_increment"])
                {
                    config_.camera.angle_increment = node["camera"]["angle_increment"].as<float>();
                }
                
                // 加载x86_64架构参数
                if (node["camera"]["x86_64"])
                {
                    if (node["camera"]["x86_64"]["voxel_leaf_size"])
                    {
                    }
                    if (node["camera"]["x86_64"]["sor_mean_k"])
                    {
                    }
                    if (node["camera"]["x86_64"]["sor_stddev_mul_thresh"])
                    {
                    }
                }
                
                // 加载aarch64架构参数
                if (node["camera"]["aarch64"])
                {
                    if (node["camera"]["aarch64"]["voxel_leaf_size"])
                    {
                    }
                    if (node["camera"]["aarch64"]["sor_mean_k"])
                    {
                    }
                    if (node["camera"]["aarch64"]["sor_stddev_mul_thresh"])
                    {
                    }
                }
            }
            
            // 加载激光雷达参数
            if (node["lidar"])
            {
                if (node["lidar"]["target_frame"])
                {
                    config_.lidar.target_frame = node["lidar"]["target_frame"].as<std::string>();
                }
                if (node["lidar"]["min_height"])
                {
                    config_.lidar.min_height = node["lidar"]["min_height"].as<float>();
                }
                if (node["lidar"]["max_height"])
                {
                    config_.lidar.max_height = node["lidar"]["max_height"].as<float>();
                }
                if (node["lidar"]["range_min"])
                {
                    config_.lidar.range_min = node["lidar"]["range_min"].as<float>();
                }
                if (node["lidar"]["tilt_compensation_angle"])
                {
                    config_.lidar.tilt_compensation_angle = node["lidar"]["tilt_compensation_angle"].as<float>();
                }
                if (node["lidar"]["tilt_axis"])
                {
                    config_.lidar.tilt_axis = node["lidar"]["tilt_axis"].as<std::string>();
                }
                if (node["lidar"]["debug"])
                {
                    config_.lidar.debug = node["lidar"]["debug"].as<bool>();
                }
                if (node["lidar"]["filter_mean_k"])
                {
                    config_.lidar.filter_mean_k = node["lidar"]["filter_mean_k"].as<int>();
                }
                if (node["lidar"]["filter_stddev"])
                {
                    config_.lidar.filter_stddev = node["lidar"]["filter_stddev"].as<float>();
                }
                if (node["lidar"]["voxel_leaf_size"])
                {
                    config_.lidar.voxel_leaf_size = node["lidar"]["voxel_leaf_size"].as<float>();
                }
            }
            
            // 加载TF变换参数
            if (node["tf"])
            {
                // 加载相机TF
                if (node["tf"]["camera"])
                {
                    if (node["tf"]["camera"]["x"])
                    {
                        config_.tf.camera_x = node["tf"]["camera"]["x"].as<float>();
                    }
                    if (node["tf"]["camera"]["y"])
                    {
                        config_.tf.camera_y = node["tf"]["camera"]["y"].as<float>();
                    }
                    if (node["tf"]["camera"]["z"])
                    {
                        config_.tf.camera_z = node["tf"]["camera"]["z"].as<float>();
                    }
                    if (node["tf"]["camera"]["roll"])
                    {
                        config_.tf.camera_roll = node["tf"]["camera"]["roll"].as<float>();
                    }
                    if (node["tf"]["camera"]["pitch"])
                    {
                        config_.tf.camera_pitch = node["tf"]["camera"]["pitch"].as<float>();
                    }
                    if (node["tf"]["camera"]["yaw"])
                    {
                        config_.tf.camera_yaw = node["tf"]["camera"]["yaw"].as<float>();
                    }
                    if (node["tf"]["camera"]["parent_frame"])
                    {
                        config_.tf.camera_parent_frame = node["tf"]["camera"]["parent_frame"].as<std::string>();
                    }
                    if (node["tf"]["camera"]["child_frame"])
                    {
                        config_.tf.camera_child_frame = node["tf"]["camera"]["child_frame"].as<std::string>();
                    }
                }
                
                // 加载雷达TF
                if (node["tf"]["lidar"])
                {
                    if (node["tf"]["lidar"]["x"])
                    {
                        config_.tf.lidar_x = node["tf"]["lidar"]["x"].as<float>();
                    }
                    if (node["tf"]["lidar"]["y"])
                    {
                        config_.tf.lidar_y = node["tf"]["lidar"]["y"].as<float>();
                    }
                    if (node["tf"]["lidar"]["z"])
                    {
                        config_.tf.lidar_z = node["tf"]["lidar"]["z"].as<float>();
                    }
                    if (node["tf"]["lidar"]["roll"])
                    {
                        config_.tf.lidar_roll = node["tf"]["lidar"]["roll"].as<float>();
                    }
                    if (node["tf"]["lidar"]["pitch"])
                    {
                        config_.tf.lidar_pitch = node["tf"]["lidar"]["pitch"].as<float>();
                    }
                    if (node["tf"]["lidar"]["yaw"])
                    {
                        config_.tf.lidar_yaw = node["tf"]["lidar"]["yaw"].as<float>();
                    }
                    if (node["tf"]["lidar"]["parent_frame"])
                    {
                        config_.tf.lidar_parent_frame = node["tf"]["lidar"]["parent_frame"].as<std::string>();
                    }
                    if (node["tf"]["lidar"]["child_frame"])
                    {
                        config_.tf.lidar_child_frame = node["tf"]["lidar"]["child_frame"].as<std::string>();
                    }
                }
            }
            
            return true;
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("加载配置时发生异常：%s", e.what());
            return false;
        }
    }
    
    bool PclConfigMgr::save_to_yaml(YAML::Emitter& emitter) const
    {
        try
        {
            // 保存系统参数
            emitter << YAML::Key << "system" << YAML::Value;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "log_dir" << YAML::Value << config_.system.log_dir;
            emitter << YAML::EndMap;
            
            // 保存启动参数
            emitter << YAML::Key << "launch" << YAML::Value;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "default_mode" << YAML::Value << config_.launch.default_mode;
            emitter << YAML::Key << "camera_input_topic" << YAML::Value << config_.launch.camera_input_topic;
            emitter << YAML::Key << "camera_output_topic" << YAML::Value << config_.launch.camera_output_topic;
            emitter << YAML::Key << "lidar_input_topic" << YAML::Value << config_.launch.lidar_input_topic;
            emitter << YAML::Key << "lidar_output_topic" << YAML::Value << config_.launch.lidar_output_topic;
            emitter << YAML::EndMap;
            
            // 保存相机参数
            emitter << YAML::Key << "camera" << YAML::Value;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "target_frame" << YAML::Value << config_.camera.target_frame;
            emitter << YAML::Key << "min_height" << YAML::Value << config_.camera.min_height;
            emitter << YAML::Key << "max_height" << YAML::Value << config_.camera.max_height;
            emitter << YAML::Key << "bSaveLogInfo2Files" << YAML::Value << config_.camera.bSaveLogInfo2Files;
            emitter << YAML::Key << "bOutputToTerminal" << YAML::Value << config_.camera.bOutputToTerminal;
            emitter << YAML::Key << "range_min" << YAML::Value << config_.camera.range_min;
            emitter << YAML::Key << "range_max" << YAML::Value << config_.camera.range_max;
            emitter << YAML::Key << "angle_min" << YAML::Value << config_.camera.angle_min;
            emitter << YAML::Key << "angle_max" << YAML::Value << config_.camera.angle_max;
            emitter << YAML::Key << "angle_increment" << YAML::Value << config_.camera.angle_increment;
            emitter << YAML::Key << "voxel_leaf_size" << YAML::Value << config_.camera.voxel_leaf_size;
            emitter << YAML::Key << "sor_mean_k" << YAML::Value << config_.camera.sor_mean_k;
            emitter << YAML::Key << "sor_stddev_mul_thresh" << YAML::Value << config_.camera.sor_stddev_mul_thresh;
            emitter << YAML::EndMap;
            
            // 保存激光雷达参数
            emitter << YAML::Key << "lidar" << YAML::Value;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "target_frame" << YAML::Value << config_.lidar.target_frame;
            emitter << YAML::Key << "min_height" << YAML::Value << config_.lidar.min_height;
            emitter << YAML::Key << "max_height" << YAML::Value << config_.lidar.max_height;
            emitter << YAML::Key << "range_min" << YAML::Value << config_.lidar.range_min;
            emitter << YAML::Key << "tilt_compensation_angle" << YAML::Value << config_.lidar.tilt_compensation_angle;
            emitter << YAML::Key << "tilt_axis" << YAML::Value << config_.lidar.tilt_axis;
            emitter << YAML::Key << "debug" << YAML::Value << config_.lidar.debug;
            emitter << YAML::Key << "filter_mean_k" << YAML::Value << config_.lidar.filter_mean_k;
            emitter << YAML::Key << "filter_stddev" << YAML::Value << config_.lidar.filter_stddev;
            emitter << YAML::Key << "voxel_leaf_size" << YAML::Value << config_.lidar.voxel_leaf_size;
            emitter << YAML::EndMap;
            
            // 保存激光扫描到点云转换参数
            emitter << YAML::Key << "laserscan_to_pointcloud" << YAML::Value;
            // 保存TF变换参数
            emitter << YAML::Key << "tf" << YAML::Value;
            emitter << YAML::BeginMap;
            
            // 保存相机TF
            emitter << YAML::Key << "camera" << YAML::Value;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "x" << YAML::Value << config_.tf.camera_x;
            emitter << YAML::Key << "y" << YAML::Value << config_.tf.camera_y;
            emitter << YAML::Key << "z" << YAML::Value << config_.tf.camera_z;
            emitter << YAML::Key << "roll" << YAML::Value << config_.tf.camera_roll;
            emitter << YAML::Key << "pitch" << YAML::Value << config_.tf.camera_pitch;
            emitter << YAML::Key << "yaw" << YAML::Value << config_.tf.camera_yaw;
            emitter << YAML::Key << "parent_frame" << YAML::Value << config_.tf.camera_parent_frame;
            emitter << YAML::Key << "child_frame" << YAML::Value << config_.tf.camera_child_frame;
            emitter << YAML::EndMap;
            
            // 保存雷达TF
            emitter << YAML::Key << "lidar" << YAML::Value;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "x" << YAML::Value << config_.tf.lidar_x;
            emitter << YAML::Key << "y" << YAML::Value << config_.tf.lidar_y;
            emitter << YAML::Key << "z" << YAML::Value << config_.tf.lidar_z;
            emitter << YAML::Key << "roll" << YAML::Value << config_.tf.lidar_roll;
            emitter << YAML::Key << "pitch" << YAML::Value << config_.tf.lidar_pitch;
            emitter << YAML::Key << "yaw" << YAML::Value << config_.tf.lidar_yaw;
            emitter << YAML::Key << "parent_frame" << YAML::Value << config_.tf.lidar_parent_frame;
            emitter << YAML::Key << "child_frame" << YAML::Value << config_.tf.lidar_child_frame;
            emitter << YAML::EndMap;
            
            emitter << YAML::EndMap;
            
            return true;
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("保存配置时发生异常：%s", e.what());
            return false;
        }
    }
    
    void PclConfigMgr::create_default_config()
    {
        // 使用默认构造函数创建默认配置
        config_ = PclConfig();
    }
}
