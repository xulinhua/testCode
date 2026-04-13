#include "hand_eye_calib/calib_data_collector.hpp"
#include "internal_constants.h"
#include "log_system/log_macros.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;
using json = nlohmann::json;

// 将所有实现放在命名空间中
namespace handeyecalib {

CalibDataCollector::CalibDataCollector() 
{

}

CalibDataCollector::~CalibDataCollector() 
{

}

bool CalibDataCollector::addCalibrationPoint(const std::vector<double>& robot_pose, const std::vector<double>& marker_position) 
{
    // 检查输入参数
    if (robot_pose.size() != 6) 
    {
        LOG_ERROR("❌ 机械臂位姿数据维度错误，应为6维");
        return false;
    }
    
    if (marker_position.size() != 3) 
    {
        LOG_ERROR("❌ 标记位置数据维度错误，应为3维");
        return false;
    }
    
    // 创建标定点
    CalibrationPoint point;
    point.index = static_cast<int>(calibration_points_.size());
    point.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    point.robot_pose = robot_pose;
    point.marker_position = marker_position;
    
    // 添加到标定点列表
    std::lock_guard<std::mutex> lock(data_mutex_);
    calibration_points_.push_back(point);
    
    LOG_INFO("✅ 成功添加标定点 %d", point.index);
    return true;
}

bool CalibDataCollector::setCalibrationPoint(int index, const std::vector<double>& robot_pose, const std::vector<double>& marker_position) 
{
    // 检查输入参数
    if (robot_pose.size() != 6) 
    {
        LOG_ERROR("❌ 机械臂位姿数据维度错误，应为6维");
        return false;
    }
    
    if (marker_position.size() != 3) 
    {
        LOG_ERROR("❌ 标记位置数据维度错误，应为3维");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 如果索引超出当前范围，扩展向量
    if (index >= static_cast<int>(calibration_points_.size())) 
    {
        calibration_points_.resize(index + 1);
    }
    
    // 设置标定点数据
    calibration_points_[index].index = index;
    calibration_points_[index].timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    calibration_points_[index].robot_pose = robot_pose;
    calibration_points_[index].marker_position = marker_position;
    
    LOG_INFO("✅ 成功设置标定点 %d", index);
    return true;
}

void CalibDataCollector::addCalibrationPoint(const CalibrationPoint& point) 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    calibration_points_.push_back(point);
}

bool CalibDataCollector::setCalibrationPoint(int index, const CalibrationPoint& point) 
{
    // 检查索引是否有效
    if (index < 0) 
    {
        LOG_ERROR("❌ 标定点索引不能为负数");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 如果索引超出当前范围，扩展向量
    if (index >= static_cast<int>(calibration_points_.size())) 
    {
        calibration_points_.resize(index + 1);
    }
    
    // 设置标定点数据
    calibration_points_[index] = point;
    
    LOG_INFO("✅ 成功设置标定点 %d", index);
    return true;
}

bool CalibDataCollector::loadCalibrationData(const std::string& data_dir) 
{
    // 检查目录是否存在
    if (!fs::exists(data_dir)) 
    {
        LOG_ERROR("❌ 数据目录不存在: %s", data_dir.c_str());
        return false;
    }
    LOG_INFO("✅ 数据目录存在: %s", data_dir.c_str());
    
    // 清除现有数据
    clearCalibrationPoints();
    
    // 查找所有JSON文件
    std::vector<std::string> json_files;
    for (const auto& entry : fs::directory_iterator(data_dir)) 
    {
        if (entry.path().extension() == ".json") 
        {
            json_files.push_back(entry.path().string());
        }
    }
    
    // 按文件名排序
    std::sort(json_files.begin(), json_files.end());
    
    // 加载所有数据点
    for (const auto& file_path : json_files) 
    {
        try 
        {
            std::ifstream file(file_path);
            if (!file.is_open()) {
                LOG_WARN("⚠️  无法打开文件: %s", file_path.c_str());
                continue;
            }
            
            json data;
            file >> data;
            
            CalibrationPoint point;
            point.index = data.value("index", 0);
            
            // 解析时间戳
            std::string timestamp_str = data.value("timestamp", "");
            // 简化处理，实际应用中可能需要解析ISO时间格式
            point.timestamp = std::stod(timestamp_str.empty() ? "0" : timestamp_str);
            
            // 解析机器人位姿
            auto robot_pos = data["robot_position"];
            point.robot_pose = {
                robot_pos.value("x", 0.0),
                robot_pos.value("y", 0.0),
                robot_pos.value("z", 0.0),
                robot_pos.value("rx", 0.0),
                robot_pos.value("ry", 0.0),
                robot_pos.value("rz", 0.0)
            };
            
            // 解析标记位置
            auto marker_pos = data["marker_position"];
            point.marker_position = {
                marker_pos.value("x", 0.0),
                marker_pos.value("y", 0.0),
                marker_pos.value("z", 0.0),
                marker_pos.value("rx", 0.0),
                marker_pos.value("ry", 0.0),
                marker_pos.value("rz", 0.0)
            };
            
            std::lock_guard<std::mutex> lock(data_mutex_);
            // 确保向量大小足够
            if (point.index >= static_cast<int>(calibration_points_.size())) 
            {
                calibration_points_.resize(point.index + 1);
            }
            calibration_points_[point.index] = point;
            
        } 
        catch (const std::exception& e) 
        {
            LOG_WARN("⚠️  加载文件 %s 失败: %s", file_path.c_str(), e.what());
            continue;
        }
    }
    
    LOG_INFO("✅ 成功加载 %d 个标定点数据", static_cast<int>(calibration_points_.size()));
    return !calibration_points_.empty();
}

bool CalibDataCollector::saveCalibrationData(const std::string& save_dir) 
{
    // 创建保存目录
    if (!fs::exists(save_dir)) 
    {
        if (!fs::create_directories(save_dir)) 
        {
            LOG_ERROR("❌ 创建保存目录失败: %s", save_dir.c_str());
            return false;
        }
    }
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 保存每个标定点
    for (const auto& point : calibration_points_) 
    {
        // 跳过空的标定点
        if (point.robot_pose.empty() && point.marker_position.empty()) 
        {
            continue;
        }
        
        try 
        {
            // 构造文件名
            std::stringstream filename;
            filename << "data_point_" << std::setfill('0') << std::setw(3) << point.index << ".json";
            std::string file_path = (fs::path(save_dir) / filename.str()).string();
            
            // 手动构造JSON字符串，确保严格按照用户要求的格式和顺序
            std::stringstream json_stream;
            json_stream << "{\n";
            json_stream << "  \"index\": " << point.index << ",\n";
            
            // 格式化时间戳为ISO格式，不包含时区信息，精确到微秒
            auto time_point = std::chrono::system_clock::time_point(
                std::chrono::milliseconds(static_cast<long long>(point.timestamp * 1000)));
            auto time_t = std::chrono::system_clock::to_time_t(time_point);
            auto ms = static_cast<long long>(point.timestamp * 1000000) % 1000000;
            
            std::stringstream timestamp_ss;
            timestamp_ss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%S");
            if (ms > 0) {
                timestamp_ss << "." << std::setfill('0') << std::setw(6) << ms;
            }
            json_stream << "  \"timestamp\": \"" << timestamp_ss.str() << "\",\n";
            
            // 机器人位置 (按照用户要求的顺序: x, y, z, rx, ry, rz)
            json_stream << "  \"robot_position\": {\n";
            json_stream << "    \"x\": " << std::fixed << std::setprecision(6) << point.robot_pose[0] << ",\n";
            json_stream << "    \"y\": " << std::fixed << std::setprecision(6) << point.robot_pose[1] << ",\n";
            json_stream << "    \"z\": " << std::fixed << std::setprecision(6) << point.robot_pose[2] << ",\n";
            json_stream << "    \"rx\": " << std::fixed << std::setprecision(6) << point.robot_pose[3] << ",\n";
            json_stream << "    \"ry\": " << std::fixed << std::setprecision(6) << point.robot_pose[4] << ",\n";
            json_stream << "    \"rz\": " << std::fixed << std::setprecision(6) << point.robot_pose[5] << "\n";
            json_stream << "  },\n";
            
            // 标记位置 (按照要求的顺序: x, y, z, rx, ry, rz)
            json_stream << "  \"marker_position\": {\n";
            json_stream << "    \"x\": " << std::fixed << std::setprecision(12) << point.marker_position[0] << ",\n";
            json_stream << "    \"y\": " << std::fixed << std::setprecision(12) << point.marker_position[1] << ",\n";
            json_stream << "    \"z\": " << std::fixed << std::setprecision(12) << point.marker_position[2] << ",\n";
            json_stream << "    \"rx\": " << std::fixed << std::setprecision(12) << point.marker_position[3] << ",\n";
            json_stream << "    \"ry\": " << std::fixed << std::setprecision(12) << point.marker_position[4] << ",\n";
            json_stream << "    \"rz\": " << std::fixed << std::setprecision(12) << point.marker_position[5] << "\n";
            json_stream << "  }\n";
            json_stream << "}";
            
            // 写入文件
            std::ofstream file(file_path);
            if (file.is_open()) {
                file << json_stream.str();
                file.close();
            } else {
                LOG_WARN("⚠️  无法写入文件: %s", file_path.c_str());
                continue;
            }
            
        } 
        catch (const std::exception& e)
        {
            LOG_WARN("⚠️  保存标定点 %d 失败: %s", point.index, e.what());
            continue;
        }
    }
    
    return true;
}

const std::vector<CalibrationPoint>& CalibDataCollector::getCalibrationPoints() const 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return calibration_points_;
}

const CalibrationPoint* CalibDataCollector::getCalibrationPoint(int index) const 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (index < 0 || index >= static_cast<int>(calibration_points_.size())) {
        return nullptr;
    }
    return &calibration_points_[index];
}

std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> 
CalibDataCollector::getCalibrationData() const 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    std::vector<std::vector<double>> robot_poses;
    std::vector<std::vector<double>> marker_positions;
    
    for (const auto& point : calibration_points_) {
        // 跳过空的标定点
        if (point.robot_pose.empty() || point.marker_position.empty()) 
        {
            continue;
        }
        robot_poses.push_back(point.robot_pose);
        marker_positions.push_back(point.marker_position);
    }
    
    return std::make_pair(robot_poses, marker_positions);
}

void CalibDataCollector::clearCalibrationPoints() 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    calibration_points_.clear();
}

bool CalibDataCollector::hasEnoughPoints(size_t min_points) const 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 计算非空的标定点数量
    size_t valid_points = 0;
    for (const auto& point : calibration_points_) 
    {
        if (!point.robot_pose.empty() || !point.marker_position.empty()) 
        {
            valid_points++;
        }
    }
    return valid_points >= min_points;
}

size_t CalibDataCollector::getPointCount() const 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return calibration_points_.size();
}

} // namespace handeyecalib