#include "hand_eye_calib/calib_robot_pos_mgr.hpp"
#include "internal_constants.h"
#include "log_system/log_macros.hpp"
#include <iostream>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>
#include "bas_operate/file_operate.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace handeyecalib {

// 全局变量，控制是否加载预保存的点数据
// true: 加载预保存的点数据 false: 使用现有的方式生成移动用的标定点
bool g_load_saved_points = false;
bool g_use_angle = true;

CalibRobotPosMgr::CalibRobotPosMgr() 
{
    // 初始化标准位置为默认值
    //standard_pose_ = {0, 0, 0, 0, 0, 0};
    //standard_pose_ = {-117.13,-344.74,415.82,-179.98,0.01,90.07}; // nova左臂
    standard_pose_ = {111.860, -341.080, 415.82, -179.98, 0.01, -89.990}; // nova右臂
    
    // 参照nova_data_collector.py中第58行到第75行的实现
    // XY平面偏移点配置
    xy_offsets_ = {
        // 中心点
        {0, 0},
        // 四角点（±50mm）
        {50, 50}, {-50, 50}, {-50, -50}, {50, -50},
        // 边中点（±50mm）
        {50, 0}, {0, 50}, {-50, 0}, {0, -50},
        // 内圈点（±30mm）
        {30, 30}, {-30, 30}, {-30, -30}, {30, -30},
        // 内圈边中点（±30mm）
        {30, 0}, {0, 30}, {-30, 0}, {0, -30}
    };
    
    // Z轴偏移：范围±150mm，间距30mm
    z_vals_ = {-150, -90, -60, -30, 0, 30};
    if (g_use_angle)
    {
        xy_offsets_.clear();
        z_vals_.clear();
        // xy_offsets_ = {
        //     // 中心点
        //     {0, 0},
        //     // 内圈点（±30mm）
        //     {10, 10}, {-10, -10}
        // };
    
        // // Z轴偏移：范围±150mm，间距30mm
        // z_vals_ = {-10, 30};
        // ang_vals_ = {{-20, 0, 20},
        //          {15, 0, 0},
        //          {0, 15, 15}};

        xy_offsets_ = {
            // 内圈点（±30mm）
            {-30, -30}, {60, 60},
            // 中心点
            {0, 0}
        };
    
        // Z轴偏移：范围±150mm，间距30mm
        z_vals_ = {50, 0};
        ang_vals_ = {{-20, 0, -15},
                     {0, -10, 10},
                    {0, 0, 0}};
    }
}

CalibRobotPosMgr::~CalibRobotPosMgr() 
{
}

std::vector<RobotPoseData> CalibRobotPosMgr::generateDataPoints() const 
{
    // 如果设置了加载预保存的点数据，则加载并返回这些点
    if (g_load_saved_points) {
        return loadSavedDataPoints();
    }
    
    // 否则使用现有的方式生成移动用的标定点
    std::vector<RobotPoseData> data_points;

    // 生成数据采集点列表
    for (const auto& z : z_vals_) 
    {
        for (const auto& offset : xy_offsets_) 
        {
            if (g_use_angle) 
            {
                for (const auto& ang : ang_vals_) 
                {
                    RobotPoseData point;
                    point.x = standard_pose_.x + offset.first;
                    point.y = standard_pose_.y + offset.second;
                    point.z = standard_pose_.z + z;
                    point.rx = standard_pose_.rx + ang[0];
                    point.ry = standard_pose_.ry + ang[1];
                    point.rz = standard_pose_.rz + ang[2];
                    
                    data_points.push_back(point);
                }
            } 
            else 
            {
                RobotPoseData point;
                point.x = standard_pose_.x + offset.first;
                point.y = standard_pose_.y + offset.second;
                point.z = standard_pose_.z + z;
                point.rx = standard_pose_.rx;
                point.ry = standard_pose_.ry;
                point.rz = standard_pose_.rz;
                data_points.push_back(point);
            }
        }
    }
    
    return data_points;
}

void CalibRobotPosMgr::initializeCachedDataPoints()
{
    // 初始化缓存数据点列表
    cached_data_points_ = generateDataPoints();
}

RobotPoseData CalibRobotPosMgr::getDataPoint(std::size_t index) const 
{
    // 检查索引是否有效
    if (index >= cached_data_points_.size()) 
    {
        LOG_ERROR("❌ 索引超出范围: %d", static_cast<int>(index));
        return standard_pose_; // 返回标准位置作为默认值
    }
    
    RobotPoseData point = cached_data_points_[index];
    
    // 检查数据点是否有效
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        !std::isfinite(point.rx) || !std::isfinite(point.ry) || !std::isfinite(point.rz)) {
        LOG_ERROR("❌ 数据点无效，索引: %d, 位姿: x=%.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f", 
            static_cast<int>(index), point.x, point.y, point.z, point.rx, point.ry, point.rz);
        return standard_pose_; // 返回标准位置作为默认值
    }
    
    return point;
}

std::size_t CalibRobotPosMgr::getPointCount() const 
{
    return cached_data_points_.size();
}

void CalibRobotPosMgr::setStandardPose(double x, double y, double z, double rx, double ry, double rz) 
{
    standard_pose_.x = x;
    standard_pose_.y = y;
    standard_pose_.z = z;
    standard_pose_.rx = rx;
    standard_pose_.ry = ry;
    standard_pose_.rz = rz;
    
    // 注意：标准位置改变后不再自动清除缓存
    // 缓存的坐标点数据不会因标准位置的变化而变化
}

RobotPoseData CalibRobotPosMgr::getStandardPose() const 
{
    return standard_pose_;
}

void CalibRobotPosMgr::setCachedDataPoints(const std::vector<RobotPoseData>& data_points)
{
    cached_data_points_ = data_points;
}

std::vector<RobotPoseData> CalibRobotPosMgr::loadSavedDataPoints() const 
{
    std::vector<RobotPoseData> data_points;

    // 使用通用接口获取当前项目所在的install目录的绝对路径
    std::string install_path_str = basmodule::get_install_dir();
    if (!install_path_str.empty()) 
    {
        LOG_INFO("当前项目install目录: %s", install_path_str.c_str());
        
        // 基于install目录配置正确的collected_data_dir路径
        // 正确的路径是 /home/user/testCode/project/install/sys_config/collected_data
        fs::path install_path(install_path_str);
        fs::path collected_data_dir_path = install_path.parent_path() / "sys_config" / "collected_data";
        std::string install_collected_data_dir = collected_data_dir_path.string();
        LOG_INFO("配置的collected_data_dir路径: %s", install_collected_data_dir.c_str());
        
        // 检查该路径是否存在
        if (fs::exists(install_collected_data_dir)) 
        {
            //LOG_INFO("找到了install目录下的collected_data目录");
            std::string collected_data_dir = install_collected_data_dir;
            
            try {
                // 检查目录是否存在
                if (!fs::exists(collected_data_dir)) {
                    LOG_WARN("⚠️  预保存点数据目录不存在: %s", collected_data_dir.c_str());
                    return data_points;
                }
                
                // 遍历目录中的所有JSON文件
                // 收集所有JSON文件路径
                std::vector<fs::path> json_files;
                for (const auto& entry : fs::directory_iterator(collected_data_dir)) {
                    if (entry.path().extension() == ".json") {
                        json_files.push_back(entry.path());
                    }
                }
                
                // 按文件名中的数字索引排序
                std::sort(json_files.begin(), json_files.end(), [](const fs::path& a, const fs::path& b) {
                    // 提取文件名中的数字索引
                    std::string filename_a = a.stem().string();
                    std::string filename_b = b.stem().string();
                    
                    // 查找最后一个下划线后的内容作为索引
                    size_t pos_a = filename_a.find_last_of('_');
                    size_t pos_b = filename_b.find_last_of('_');
                    
                    if (pos_a != std::string::npos && pos_b != std::string::npos) {
                        std::string index_str_a = filename_a.substr(pos_a + 1);
                        std::string index_str_b = filename_b.substr(pos_b + 1);
                        
                        // 转换为整数进行比较
                        try {
                            int index_a = std::stoi(index_str_a);
                            int index_b = std::stoi(index_str_b);
                            return index_a < index_b;
                        } catch (const std::exception&) {
                            // 如果转换失败，按字典序排序
                            return filename_a < filename_b;
                        }
                    }
                    // 如果没有找到下划线，按字典序排序
                    return filename_a < filename_b;
                });
                
                // 按排序后的顺序加载JSON文件
                for (const auto& file_path : json_files) {
                    try {
                        // 读取JSON文件
                        std::ifstream file(file_path);
                        if (!file.is_open()) {
                            LOG_WARN("⚠️  无法打开文件: %s", file_path.string().c_str());
                            continue;
                        }
                        
                        json j;
                        file >> j;
                        file.close();
                        
                        // 提取机械臂位置数据
                        RobotPoseData point;
                        point.x = j.value("robot_position", json::object()).value("x", 0.0); 
                        point.y = j.value("robot_position", json::object()).value("y", 0.0); 
                        point.z = j.value("robot_position", json::object()).value("z", 0.0); 
                        point.rx = j.value("robot_position", json::object()).value("rx", 0.0);       
                        point.ry = j.value("robot_position", json::object()).value("ry", 0.0);       
                        point.rz = j.value("robot_position", json::object()).value("rz", 0.0);  
                        
                        data_points.push_back(point);

                    } catch (const std::exception& e) {
                        LOG_WARN("⚠️  解析JSON文件失败: %s, 错误: %s", 
                                    file_path.string().c_str(), e.what());
                        continue;
                    }
                }
                
                //LOG_INFO("✅ 成功加载 %d 个预保存的点数据", static_cast<int>(data_points.size()));
                bool bPrintPosInfo = true;
                if  (bPrintPosInfo) {
                    // 打印每个标定点的详细数据，保留6位小数
                    for (size_t i = 0; i < data_points.size(); ++i) 
                    {
                        const auto& point = data_points[i];
                        LOG_INFO("标定点[%d]: x=%.6f, y=%.6f, z=%.6f, rx=%.6f, ry=%.6f, rz=%.6f", 
                            static_cast<int>(i), point.x, point.y, point.z, point.rx, point.ry, point.rz);
                    }
                }
 
                // 当使用加载数据模式时，设置手臂的标准位置
                if (!data_points.empty()) {
                    // 设置标准位置为指定值
                    const_cast<CalibRobotPosMgr*>(this)->standard_pose_.x = 323.2776951609348;  
                    const_cast<CalibRobotPosMgr*>(this)->standard_pose_.y = 1.4260709574537844; 
                    const_cast<CalibRobotPosMgr*>(this)->standard_pose_.z = 1040.5693680220445; 
                    const_cast<CalibRobotPosMgr*>(this)->standard_pose_.rx = 167.6793290697525;
                    const_cast<CalibRobotPosMgr*>(this)->standard_pose_.ry = -89.15535543043285;
                    const_cast<CalibRobotPosMgr*>(this)->standard_pose_.rz = -165.4546984452126;
                }
                return data_points; // 成功加载数据后直接返回
            } catch (const std::exception& e) {
                LOG_ERROR("❌ 遍历预保存点数据目录失败: %s", e.what());
                return data_points; // 出现异常时也要返回
            }
        } else {
            LOG_INFO("install目录下的collected_data目录不存在: %s", install_collected_data_dir.c_str());
        }
    }
    
    // 如果通过install路径找不到，则使用原来的相对路径方式
    // 预保存点数据的目录路径
    std::string collected_data_dir = "../../../sys_config/collected_data";
    
    try {
        // 检查目录是否存在
        if (!fs::exists(collected_data_dir)) {
            LOG_WARN("⚠️  预保存点数据目录不存在: %s", collected_data_dir.c_str());
            return data_points;
        }
        
        // 遍历目录中的所有JSON文件
        // 收集所有JSON文件路径
        std::vector<fs::path> json_files;
        for (const auto& entry : fs::directory_iterator(collected_data_dir)) 
        {
            if (entry.path().extension() == ".json") {
                json_files.push_back(entry.path());
            }
        }
        
        // 按文件名中的数字索引排序
        std::sort(json_files.begin(), json_files.end(), [](const fs::path& a, const fs::path& b) 
        {
            // 提取文件名中的数字索引
            std::string filename_a = a.stem().string();
            std::string filename_b = b.stem().string();
            
            // 查找最后一个下划线后的内容作为索引
            size_t pos_a = filename_a.find_last_of('_');
            size_t pos_b = filename_b.find_last_of('_');
            
            if (pos_a != std::string::npos && pos_b != std::string::npos) {
                std::string index_str_a = filename_a.substr(pos_a + 1);
                std::string index_str_b = filename_b.substr(pos_b + 1);
                
                // 转换为整数进行比较
                try {
                    int index_a = std::stoi(index_str_a);
                    int index_b = std::stoi(index_str_b);
                    return index_a < index_b;
                } catch (const std::exception&) {
                    // 如果转换失败，按字典序排序
                    return filename_a < filename_b;
                }
            }
            // 如果没有找到下划线，按字典序排序
            return filename_a < filename_b;
        });
        
        // 按排序后的顺序加载JSON文件
        for (const auto& file_path : json_files) 
        {
            try {
                // 读取JSON文件
                std::ifstream file(file_path);
                if (!file.is_open()) {
                    LOG_WARN("⚠️  无法打开文件: %s", file_path.string().c_str());
                    continue;
                }
                
                json j;
                file >> j;
                file.close();
                
                // 提取机械臂位置数据
                RobotPoseData point;
                point.x = j.value("robot_position", json::object()).value("x", 0.0);  
                point.y = j.value("robot_position", json::object()).value("y", 0.0); 
                point.z = j.value("robot_position", json::object()).value("z", 0.0); 
                point.rx = j.value("robot_position", json::object()).value("rx", 0.0);         
                point.ry = j.value("robot_position", json::object()).value("ry", 0.0);         
                point.rz = j.value("robot_position", json::object()).value("rz", 0.0);    
                
                data_points.push_back(point);
                
            } catch (const std::exception& e) {
                LOG_WARN("⚠️  解析JSON文件失败: %s, 错误: %s", 
                            file_path.string().c_str(), e.what());
                continue;
            }
        }
        
        LOG_INFO("✅ 成功加载 %d 个预保存的点数据", static_cast<int>(data_points.size()));

        bool bPrintPosInfo = true;
        if  (bPrintPosInfo) {
            // 打印每个标定点的详细数据，保留6位小数
            for (size_t i = 0; i < data_points.size(); ++i) {
                const auto& point = data_points[i];
                LOG_INFO("标定点[%d]: x=%.6f, y=%.6f, z=%.6f, rx=%.6f, ry=%.6f, rz=%.6f", 
                    static_cast<int>(i), point.x, point.y, point.z, point.rx, point.ry, point.rz);
            }
        }
    } catch (const std::exception& e) {
        LOG_ERROR("❌ 遍历预保存点数据目录失败: %s", e.what());
    }
    
    // 当使用加载数据模式时，设置手臂的标准位置
    if (!data_points.empty()) {
        // 设置标准位置为指定值
        const_cast<CalibRobotPosMgr*>(this)->standard_pose_.x = 323.2776951609348;  
        const_cast<CalibRobotPosMgr*>(this)->standard_pose_.y = 1.4260709574537844;  
        const_cast<CalibRobotPosMgr*>(this)->standard_pose_.z = 1040.5693680220445;  
        const_cast<CalibRobotPosMgr*>(this)->standard_pose_.rx = 167.6793290697525;
        const_cast<CalibRobotPosMgr*>(this)->standard_pose_.ry = -89.15535543043285;
        const_cast<CalibRobotPosMgr*>(this)->standard_pose_.rz = -165.4546984452126;
    }
    
    return data_points;
}

} // namespace handeyecalib