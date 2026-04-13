#include "nova_robot_ctrl/calib_robot_pos_mgr.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace nova_robot_ctrl {

// 全局变量，控制是否加载预保存的点数据
// true: 加载预保存的点数据 false: 使用现有的方式生成移动用的标定点
static bool g_load_saved_points = false;

CalibRobotPosMgr::CalibRobotPosMgr(int robot_id)
    : robot_id_(robot_id) 
{
    // 初始化标准位置为默认值
    standard_pose_ = {0, 0, 0, 0, 0, 0};

    // 参照hand_eye_calib项目中的实现
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
    //z_vals_ = {-30, 0, 30};
}

CalibRobotPosMgr::~CalibRobotPosMgr() {}

std::vector<RobotPoseData> CalibRobotPosMgr::generateDataPoints() const 
{
    // 如果设置了加载预保存的点数据，则加载并返回这些点
    if (g_load_saved_points) 
    {
        return loadSavedDataPoints();
    }

    // 否则使用现有的方式生成移动用的标定点
    std::vector<RobotPoseData> data_points;

    // 生成数据采集点列表
    for (const auto& z : z_vals_) 
    {
        for (const auto& offset : xy_offsets_) 
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
    if (index >= cached_data_points_.size()) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << " - ❌ 索引超出范围: " << static_cast<int>(index) << std::endl;
        return standard_pose_; // 返回标准位置作为默认值
    }
    
    RobotPoseData point = cached_data_points_[index];
    
    // 检查数据点是否有效
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        !std::isfinite(point.rx) || !std::isfinite(point.ry) || !std::isfinite(point.rz)) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << " - ❌ 数据点无效，索引: " << static_cast<int>(index) << ", 位姿: x=" << point.x << ", y=" << point.y << ", z=" << point.z << ", rx=" << point.rx << ", ry=" << point.ry << ", rz=" << point.rz << std::endl;
        return standard_pose_; // 返回标准位置作为默认值
    }
    
    return point;
}

std::size_t CalibRobotPosMgr::getPointCount() const 
{
    return cached_data_points_.size();
}

void CalibRobotPosMgr::setStandardPose(double x, double y, double z, double rx, double ry, double rz) {
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

void CalibRobotPosMgr::setRobotId(int robot_id) 
{
    robot_id_ = robot_id;
}

int CalibRobotPosMgr::getRobotId() const 
{
    return robot_id_;
}

std::vector<RobotPoseData> CalibRobotPosMgr::loadSavedDataPoints() const 
{
    std::vector<RobotPoseData> data_points;

    try {
        // 获取标定数据目录
        std::string data_dir = getCalibDataDirectory();
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << " - 标定数据目录: " << data_dir << std::endl;

        // 检查目录是否存在
        if (!fs::exists(data_dir)) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << " - 标定数据目录不存在: " << data_dir << std::endl;
            return data_points;
        }

        // 遍历目录中的所有JSON文件
        for (const auto& entry : fs::directory_iterator(data_dir)) {
            if (entry.path().extension() == ".json") {
                std::string file_path = entry.path().string();
                std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << " - 处理文件: " << file_path << std::endl;

                try {
                    // 读取JSON文件
                    std::ifstream file(file_path);
                    if (!file.is_open()) {
                        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << " - 无法打开文件: " << file_path << std::endl;
                        continue;
                    }

                    json j;
                    file >> j;

                    // 解析机械臂位置数据
                    RobotPoseData point;
                    point.x = j.value("robot_position", json::object()).value("x", 0.0);
                    point.y = j.value("robot_position", json::object()).value("y", 0.0);
                    point.z = j.value("robot_position", json::object()).value("z", 0.0);
                    point.rx = j.value("robot_position", json::object()).value("rx", 0.0);
                    point.ry = j.value("robot_position", json::object()).value("ry", 0.0);
                    point.rz = j.value("robot_position", json::object()).value("rz", 0.0);

                    data_points.push_back(point);
                } catch (const std::exception& e) {
                    std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << " - ⚠️  解析JSON文件失败: " << file_path << ", 错误: " << e.what() << std::endl;
                    continue;
                }
            }
        }

        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << " - ✅ 成功加载 " << static_cast<int>(data_points.size()) << " 个预保存的点数据" << std::endl;

        // 打印每个标定点的详细数据
        for (size_t i = 0; i < data_points.size(); ++i) {
            const auto& point = data_points[i];
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << " - 标定点[" << static_cast<int>(i) << "]: x=" << point.x << ", y=" << point.y << ", z=" << point.z << ", rx=" << point.rx << ", ry=" << point.ry << ", rz=" << point.rz << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << " - ❌ 遍历预保存点数据目录失败: " << e.what() << std::endl;
    }

    return data_points;
}

std::string CalibRobotPosMgr::getCalibDataDirectory() const 
{
    // 根据机械手ID确定数据目录
    std::string base_dir = "../sys_config/collected_data"; // 相对于可执行文件的路径
    
    // 如果有多个机械手，可以在路径中加入机械手ID
    if (robot_id_ > 0) {
        base_dir += "_robot_" + std::to_string(robot_id_);
    }
    
    return base_dir;
}

} // namespace nova_robot_ctrl