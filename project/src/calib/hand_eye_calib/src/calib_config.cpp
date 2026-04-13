#include "hand_eye_calib/calib_config.hpp"
#include "internal_constants.h"
#include "log_system/log_macros.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

namespace handeyecalib {

CalibConfig::CalibConfig() 
    : min_calibration_points_(4),
      source_data_dir_("sys_config/source_calib_data"),
      output_data_dir_("output_calib_data")
{
}

CalibConfig::~CalibConfig() {
}

bool CalibConfig::loadConfig(const std::string& config_file) 
{
    try 
    {
        // 检查配置文件是否存在
        if (!fs::exists(config_file)) 
        {
            LOG_ERROR("❌ 配置文件不存在: %s", config_file.c_str());
            return false;
        }
        
        // 加载YAML配置文件
        config_node_ = YAML::LoadFile(config_file);
        
        // 加载最小标定点数量
        if (config_node_["calibration"] && config_node_["calibration"]["min_points"]) {
            min_calibration_points_ = config_node_["calibration"]["min_points"].as<int>();
        }
        
        // 加载源数据目录
        if (config_node_["calibration"] && config_node_["calibration"]["source_data_dir"]) {
            source_data_dir_ = config_node_["calibration"]["source_data_dir"].as<std::string>();
        }
        
        // 加载输出数据目录
        if (config_node_["calibration"] && config_node_["calibration"]["output_data_dir"]) {
            output_data_dir_ = config_node_["calibration"]["output_data_dir"].as<std::string>();
        }
        
        LOG_INFO("✅ 配置文件加载成功: %s", config_file.c_str());
        return true;
    } catch (const YAML::Exception& e) {
        LOG_ERROR("❌ 加载配置文件失败: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR("❌ 加载配置文件失败: %s", e.what());
        return false;
    }
}

bool CalibConfig::saveConfig(const std::string& config_file) 
{
    try 
    {
        // 创建保存目录
        fs::path config_path(config_file);
        if (!fs::exists(config_path.parent_path())) {
            fs::create_directories(config_path.parent_path());
        }
        
        // 更新YAML节点
        config_node_["calibration"]["min_points"] = min_calibration_points_;
        config_node_["calibration"]["source_data_dir"] = source_data_dir_;
        config_node_["calibration"]["output_data_dir"] = output_data_dir_;
        
        // 保存到文件
        std::ofstream fout(config_file);
        if (fout.is_open()) 
        {
            fout << config_node_;
            fout.close();
            LOG_INFO("✅ 配置文件保存成功: %s", config_file.c_str());
            return true;
        } else {
            LOG_ERROR("❌ 无法打开配置文件进行写入: %s", config_file.c_str());
            return false;
        }
    } catch (const YAML::Exception& e) 
    {
        LOG_ERROR("❌ 保存配置文件失败: %s", e.what());
        return false;
    } catch (const std::exception& e) 
    {
        LOG_ERROR("❌ 保存配置文件失败: %s", e.what());
        return false;
    }
}

int CalibConfig::getMinCalibrationPoints() const 
{
    return min_calibration_points_;
}

void CalibConfig::setMinCalibrationPoints(int min_points) 
{
    min_calibration_points_ = min_points;
}

std::string CalibConfig::getSourceDataDir() const 
{
    return source_data_dir_;
}

void CalibConfig::setSourceDataDir(const std::string& dir) 
{
    source_data_dir_ = dir;
}

std::string CalibConfig::getOutputDataDir() const 
{
    return output_data_dir_;
}

void CalibConfig::setOutputDataDir(const std::string& dir) 
{
    output_data_dir_ = dir;
}

} // namespace handeyecalib