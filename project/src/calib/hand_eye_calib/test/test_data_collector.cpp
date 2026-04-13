#include <iostream>
#include <vector>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include "hand_eye_calib/calib_data_collector.hpp"
#include "hand_eye_calib/calib_config.hpp"
#include "internal_constants.h"
#include "log_system/log_macros.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
// 添加unistd.h头文件以使用readlink函数
#include <unistd.h>
#include "bas_operate/file_operate.hpp"  

using namespace handeyecalib;
namespace fs = std::filesystem;
using json = nlohmann::json;

/**
 * @brief 从JSON文件加载标定点数据
 * @param file_path JSON文件路径
 * @param robot_pose 机械臂位姿（输出）
 * @param marker_position 标记位置（输出）
 * @return 是否加载成功
 */
bool loadCalibrationPointFromJson(const std::string& file_path, 
                                 std::vector<double>& robot_pose,
                                 std::vector<double>& marker_position) {
    try {
        // 读取JSON文件
        std::ifstream file(file_path);
        if (!file.is_open()) {
            LOG_ERROR("❌ 无法打开文件: %s", file_path.c_str());
            return false;
        }
        
        json j;
        file >> j;
        file.close();
        
        // 提取机械臂位姿
        robot_pose.resize(6);
        robot_pose[0] = j.value("robot_pose", json::object()).value("x", 0.0);
        robot_pose[1] = j.value("robot_pose", json::object()).value("y", 0.0);
        robot_pose[2] = j.value("robot_pose", json::object()).value("z", 0.0);
        robot_pose[3] = j.value("robot_pose", json::object()).value("rx", 0.0);
        robot_pose[4] = j.value("robot_pose", json::object()).value("ry", 0.0);
        robot_pose[5] = j.value("robot_pose", json::object()).value("rz", 0.0);
        
        // 提取标记位置
        marker_position.resize(6);
        marker_position[0] = j.value("marker_position", json::object()).value("x", 0.0);
        marker_position[1] = j.value("marker_position", json::object()).value("y", 0.0);
        marker_position[2] = j.value("marker_position", json::object()).value("z", 0.0);
        marker_position[3] = j.value("marker_position", json::object()).value("rx", 0.0);
        marker_position[4] = j.value("marker_position", json::object()).value("ry", 0.0);
        marker_position[5] = j.value("marker_position", json::object()).value("rz", 0.0);
        
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("❌ 解析JSON文件失败: %s", e.what());
        return false;
    }
}

/**
 * @brief 保存标定点数据到JSON文件
 * @param file_path JSON文件路径
 * @param index 标定点索引
 * @param robot_pose 机械臂位姿
 * @param marker_position 标记位置
 * @return 是否保存成功
 */
bool saveCalibrationPointToJson(const std::string& file_path,
                               int index,
                               const std::vector<double>& robot_pose,
                               const std::vector<double>& marker_position) {
    try {
        // 检查输入参数
        if (robot_pose.size() != 6) {
            LOG_ERROR("❌ 机械臂位姿数据维度错误，应为6维: %d", static_cast<int>(robot_pose.size()));
            return false;
        }
        
        if (marker_position.size() != 6) {
            LOG_ERROR("❌ 标记位置数据维度错误，应为6维: %d", static_cast<int>(marker_position.size()));
            return false;
        }
        
        // 创建JSON对象
        json j;
        j["index"] = index;
        
        // 添加机械臂位姿
        j["robot_pose"]["x"] = robot_pose[0];
        j["robot_pose"]["y"] = robot_pose[1];
        j["robot_pose"]["z"] = robot_pose[2];
        j["robot_pose"]["rx"] = robot_pose[3];
        j["robot_pose"]["ry"] = robot_pose[4];
        j["robot_pose"]["rz"] = robot_pose[5];
        
        // 添加标记位置
        j["marker_position"]["x"] = marker_position[0];
        j["marker_position"]["y"] = marker_position[1];
        j["marker_position"]["z"] = marker_position[2];
        j["marker_position"]["rx"] = marker_position[3];
        j["marker_position"]["ry"] = marker_position[4];
        j["marker_position"]["rz"] = marker_position[5];
        
        // 创建目录
        fs::path path(file_path);
        std::error_code ec;
        if (!fs::exists(path.parent_path(), ec)) {
            if (!fs::create_directories(path.parent_path(), ec)) {
                LOG_ERROR("❌ 创建目录失败: %s, 错误: %s", 
                              path.parent_path().string().c_str(), ec.message().c_str());
                return false;
            }
        }
        
        // 写入文件
        std::ofstream file(file_path);
        if (file.is_open()) {
            file << j.dump(2);
            file.close();
            return true;
        } else {
            LOG_ERROR("❌ 无法创建文件: %s", file_path.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("❌ 保存JSON文件失败: %s", e.what());
        return false;
    }
}

/**
 * @brief 从源数据目录加载所有标定点数据
 * @param source_dir 源数据目录
 * @param collector 数据收集器
 */
void loadAllCalibrationPoints(const std::string& source_dir, CalibDataCollector& collector) {
    // 输出当前项目被安装的绝对路径
    std::error_code ec;
    std::string current_path = fs::current_path(ec).string();
    if (!ec) {
        LOG_INFO("当前工作目录: %s", current_path.c_str());
    }
    
    // 使用通用接口获取当前可执行文件的路径
    std::string exe_path = basmodule::get_executable_path();
    if (!exe_path.empty()) {
        LOG_INFO("可执行文件路径: %s", exe_path.c_str());
        
        // 使用通用接口获取当前项目所在的install目录的绝对路径
        std::string install_path_str = basmodule::get_install_dir();
        if (!install_path_str.empty()) {
            LOG_INFO("当前项目install目录: %s", install_path_str.c_str());
            
            // 基于install目录配置正确的coordinates_dir路径
            // 正确的路径是 /home/user/testCode/project/install/sys_config/source_calib_data/coordinates
            fs::path install_path(install_path_str);
            fs::path coordinates_dir_path = install_path.parent_path() / "sys_config" / "source_calib_data" / "coordinates";
            std::string install_coordinates_dir = coordinates_dir_path.string();
            LOG_INFO("配置的coordinates_dir路径: %s", install_coordinates_dir.c_str());
            
            // 检查该路径是否存在
            if (fs::exists(install_coordinates_dir)) 
            {
                LOG_INFO("找到了install目录下的coordinates目录");
                // 直接使用install目录下的路径
                std::string coordinates_dir = install_coordinates_dir;
                
                try {
                    // 检查目录是否存在
                    if (!fs::exists(coordinates_dir)) {
                        LOG_WARN("⚠️  源数据目录不存在: %s", coordinates_dir.c_str());
                        return;
                    }
                    
                    // 遍历目录中的所有JSON文件
                    int loaded_count = 0;
                    for (const auto& entry : fs::directory_iterator(coordinates_dir)) {
                        if (entry.path().extension() == ".json") {
                            std::vector<double> robot_pose, marker_position;
                            if (loadCalibrationPointFromJson(entry.path().string(), robot_pose, marker_position)) {
                                // 从文件名提取索引
                                std::string filename = entry.path().stem().string();
                                int index = 0;
                                if (filename.find("data_point_") != std::string::npos) {
                                    std::string index_str = filename.substr(11); // "data_point_" 后面的部分
                                    try {
                                        index = std::stoi(index_str);
                                    } catch (...) {
                                        // 如果无法解析索引，使用默认值
                                        index = loaded_count + 1;
                                    }
                                } else {
                                    index = loaded_count + 1;
                                }
                                
                                // 添加到收集器
                                if (collector.setCalibrationPoint(index, robot_pose, marker_position)) {
                                    LOG_INFO("✅ 成功加载标定点 %d", index);
                                    loaded_count++;
                                } else {
                                    LOG_ERROR("❌ 加载标定点 %d 失败", index);
                                }
                            } else {
                                LOG_ERROR("❌ 加载文件失败: %s", entry.path().string().c_str());
                            }
                        }
                    }
                    
                    LOG_INFO("✅ 成功加载 %d 个标定点", loaded_count);
                    return; // 成功加载数据后直接返回
                } catch (const std::exception& e) {
                    LOG_ERROR("❌ 遍历目录失败: %s", e.what());
                    return; // 出现异常时也要返回
                }
            } else {
                LOG_INFO("install目录下的coordinates目录不存在: %s", install_coordinates_dir.c_str());
            }
        }
    }
    
    // 输出准确的source_dir路径
    LOG_INFO("源数据目录: %s", source_dir.c_str());
    
    // 构造标定点数据目录路径
    std::string coordinates_dir = source_dir + "/coordinates";
    // 输出准确的coordinates_dir路径
    LOG_INFO("标定点数据目录: %s", coordinates_dir.c_str());
    
    // 输出加载标定点数据的绝对路径
    std::string absolute_path = fs::absolute(coordinates_dir, ec).string();
    if (ec) {
        LOG_INFO("从目录加载标定点数据: %s (获取绝对路径失败: %s)", coordinates_dir.c_str(), ec.message().c_str());
    } else {
        LOG_INFO("从目录加载标定点数据: %s (绝对路径: %s)", coordinates_dir.c_str(), absolute_path.c_str());
    }
    
    // 如果目录不存在，尝试通过 install 路径查找
    if (!fs::exists(coordinates_dir)) {
        // 实际源数据目录应该为/home/user/testCode/project/install/sys_config/source_calib_data/coordinates
        // 当前hand_eye_calib项目被安装的路径为/home/user/testCode/project/install/hand_eye_calib
        // 从hand_eye_calib/lib/hand_eye_calib/到sys_config的相对路径应该是 ../../../sys_config
        std::string install_coordinates_dir = "../../../sys_config/source_calib_data/coordinates";
        
        // 输出尝试的绝对路径
        std::string install_absolute_path = fs::absolute(install_coordinates_dir, ec).string();
        if (ec) {
            LOG_INFO("尝试使用 install 路径: %s (获取绝对路径失败: %s)", install_coordinates_dir.c_str(), ec.message().c_str());
        } else {
            LOG_INFO("尝试使用 install 路径: %s (绝对路径: %s)", install_coordinates_dir.c_str(), install_absolute_path.c_str());
        }
        
        if (fs::exists(install_coordinates_dir)) {
            coordinates_dir = install_coordinates_dir;
            LOG_INFO("使用 install 路径加载标定点数据: %s", coordinates_dir.c_str());
        } else {
            // 再次尝试另一种可能的路径
            install_coordinates_dir = "../../../../sys_config/source_calib_data/coordinates";
            if (fs::exists(install_coordinates_dir)) {
                coordinates_dir = install_coordinates_dir;
                LOG_INFO("使用 install 路径加载标定点数据: %s", coordinates_dir.c_str());
            } else {
                // 再次尝试另一种可能的路径（相对于可执行文件的路径）
                install_coordinates_dir = "../../sys_config/source_calib_data/coordinates";
                if (fs::exists(install_coordinates_dir)) {
                    coordinates_dir = install_coordinates_dir;
                    LOG_INFO("使用 install 路径加载标定点数据: %s", coordinates_dir.c_str());
                } else {
                    // 尝试另一种相对路径
                    install_coordinates_dir = "../sys_config/source_calib_data/coordinates";
                    if (fs::exists(install_coordinates_dir)) {
                        coordinates_dir = install_coordinates_dir;
                        LOG_INFO("使用 install 路径加载标定点数据: %s", coordinates_dir.c_str());
                    } else {
                        // 尝试直接使用相对于项目根目录的路径
                        install_coordinates_dir = "../../../install/sys_config/source_calib_data/coordinates";
                        if (fs::exists(install_coordinates_dir)) {
                            coordinates_dir = install_coordinates_dir;
                            LOG_INFO("使用 install 路径加载标定点数据: %s", coordinates_dir.c_str());
                        } else {
                            LOG_WARN("⚠️  源数据目录不存在: %s", coordinates_dir.c_str());
                            return;
                        }
                    }
                }
            }
        }
    }
    
    try {
        // 检查目录是否存在
        if (!fs::exists(coordinates_dir)) {
            LOG_WARN("⚠️  源数据目录不存在: %s", coordinates_dir.c_str());
            return;
        }
        
        // 遍历目录中的所有JSON文件
        int loaded_count = 0;
        for (const auto& entry : fs::directory_iterator(coordinates_dir)) {
            if (entry.path().extension() == ".json") {
                std::vector<double> robot_pose, marker_position;
                if (loadCalibrationPointFromJson(entry.path().string(), robot_pose, marker_position)) {
                    // 从文件名提取索引
                    std::string filename = entry.path().stem().string();
                    int index = 0;
                    if (filename.find("data_point_") != std::string::npos) {
                        std::string index_str = filename.substr(11); // "data_point_" 后面的部分
                        try {
                            index = std::stoi(index_str);
                        } catch (...) {
                            // 如果无法解析索引，使用默认值
                            index = loaded_count + 1;
                        }
                    } else {
                        index = loaded_count + 1;
                    }
                    
                    // 添加到收集器
                    if (collector.setCalibrationPoint(index, robot_pose, marker_position)) {
                        LOG_INFO("✅ 成功加载标定点 %d", index);
                        loaded_count++;
                    } else {
                        LOG_ERROR("❌ 加载标定点 %d 失败", index);
                    }
                } else {
                    LOG_ERROR("❌ 加载文件失败: %s", entry.path().string().c_str());
                }
            }
        }
        
        LOG_INFO("✅ 成功加载 %d 个标定点", loaded_count);
    } catch (const std::exception& e) {
        LOG_ERROR("❌ 遍历目录失败: %s", e.what());
    }
}

/**
 * @brief 执行数据采集功能
 * @param config 配置对象
 */
void runDataCollection(const CalibConfig& config) {
    // 创建数据管理器
    CalibDataCollector collector;
    
    // 从源数据目录加载已有标定点数据
    loadAllCalibrationPoints(config.getSourceDataDir(), collector);
    
    // 模拟接收外部模块传入的标定数据
    // 在实际应用中，这些数据将来自其他功能模块
    std::vector<std::pair<std::vector<double>, std::vector<double>>> sample_data = {
        {{0.1, 0.2, 0.3, 0.0, 0.0, 0.0}, {0.05, 0.1, 0.15}},
        {{0.2, 0.3, 0.4, 0.0, 0.0, 0.0}, {0.1, 0.15, 0.2}},
        {{0.3, 0.4, 0.5, 0.0, 0.0, 0.0}, {0.15, 0.2, 0.25}},
        {{0.4, 0.5, 0.6, 0.0, 0.0, 0.0}, {0.2, 0.25, 0.3}},
        {{0.5, 0.6, 0.7, 0.0, 0.0, 0.0}, {0.25, 0.3, 0.35}}
    };
    
    LOG_INFO("开始添加标定点数据...");
    for (std::size_t i = 0; i < sample_data.size(); ++i) {
        const auto& data = sample_data[i];
        LOG_INFO("添加第 %d 个标定点...", static_cast<int>(i + 1));
        
        if (collector.addCalibrationPoint(data.first, data.second)) {
            LOG_INFO("✅ 成功添加第 %d 个标定点", static_cast<int>(i + 1));
        } else {
            LOG_ERROR("❌ 添加第 %d 个标定点失败", static_cast<int>(i + 1));
        }
    }
    
    // 演示使用setCalibrationPoint接口设置指定索引的标定点
    LOG_INFO("\n演示使用setCalibrationPoint接口设置指定索引的标定点...");
    std::vector<double> robot_pose = {0.6, 0.7, 0.8, 0.0, 0.0, 0.0};
    std::vector<double> marker_position = {0.3, 0.35, 0.4};
    
    // 设置索引为10的标定点
    if (collector.setCalibrationPoint(10, robot_pose, marker_position)) {
        LOG_INFO("✅ 成功设置索引为10的标定点");
        // 验证设置的数据
        const CalibrationPoint* point = collector.getCalibrationPoint(10);
        if (point) {
            LOG_INFO("   机械臂位姿: [%f, %f, %f, %f, %f, %f]", 
                         point->robot_pose[0], point->robot_pose[1], point->robot_pose[2], 
                         point->robot_pose[3], point->robot_pose[4], point->robot_pose[5]);
            LOG_INFO("   标记位置: [%f, %f, %f]", 
                         point->marker_position[0], point->marker_position[1], point->marker_position[2]);
        }
    } else {
        LOG_ERROR("❌ 设置索引为10的标定点失败");
    }
    
    // 保存数据到输出目录
    std::string output_coordinates_dir = config.getOutputDataDir() + "/coordinates";
    
    // 确保输出目录存在
    if (!fs::exists(output_coordinates_dir)) {
        try {
            fs::create_directories(output_coordinates_dir);
            LOG_INFO("创建输出目录: %s", output_coordinates_dir.c_str());
        } catch (const std::exception& e) {
            LOG_ERROR("❌ 创建输出目录失败: %s", e.what());
            
            // 尝试在项目根目录下创建输出目录
            try {
                std::string project_output_dir = "../../../output_calib_data/coordinates";
                fs::create_directories(project_output_dir);
                output_coordinates_dir = project_output_dir;
                LOG_INFO("使用项目根路径创建输出目录: %s", output_coordinates_dir.c_str());
            } catch (const std::exception& e2) {
                LOG_ERROR("❌ 使用项目根路径创建输出目录也失败: %s", e2.what());
                // 如果仍然失败，则使用当前目录
                output_coordinates_dir = "./output_calib_data/coordinates";
                try {
                    fs::create_directories(output_coordinates_dir);
                    LOG_INFO("使用当前目录创建输出目录: %s", output_coordinates_dir.c_str());
                } catch (const std::exception& e3) {
                    LOG_ERROR("❌ 使用当前目录创建输出目录也失败: %s", e3.what());
                    return; // 如果所有方法都失败，则返回
                }
            }
        }
    }
    
    // 保存所有标定点数据
    const auto& points = collector.getCalibrationPoints();
    int saved_count = 0;
    for (const auto& point : points) {
        // 跳过空的标定点
        if (point.robot_pose.empty() && point.marker_position.empty()) {
            continue;
        }
        
        // 生成文件名
        std::stringstream ss;
        ss << "data_point_" << std::setfill('0') << std::setw(3) << point.index << ".json";
        std::string filename = ss.str();
        std::string filepath = output_coordinates_dir + "/" + filename;
        
        // 保存到输出目录
        if (saveCalibrationPointToJson(filepath, point.index, point.robot_pose, point.marker_position)) {
            LOG_INFO("✅ 保存标定点 %d 到 %s", point.index, filepath.c_str());
            saved_count++;
        } else {
            LOG_ERROR("❌ 保存标定点 %d 失败", point.index);
        }
    }
    
    LOG_INFO("\n✅ 数据采集完成，共保存 %d 个标定点到 %s", 
                 saved_count, output_coordinates_dir.c_str());
}

/**
 * @brief 打印使用说明
 */
void printUsage() {
    LOG_INFO("手眼标定数据采集工具使用说明:");
    LOG_INFO("执行数据采集: test_data_collector");
    LOG_INFO("显示帮助: test_data_collector --help");
}

int main(int argc, char* argv[]) {
    LOG_INFO("=== 数据采集模块测试 ===");
    
    // 创建配置管理器并加载配置
    CalibConfig config;
    std::string config_file = "config/calib_config.yaml";
    
    // 检查当前目录下是否存在配置文件
    if (!fs::exists(config_file)) {
        // 尝试在上一级目录查找
        config_file = "../config/calib_config.yaml";
        if (!fs::exists(config_file)) {
            // 尝试在上两级目录查找
            config_file = "../../config/calib_config.yaml";
            if (!fs::exists(config_file)) {
                // 尝试通过 ROS 包路径查找
                try {
                    std::string package_path = ament_index_cpp::get_package_share_directory("hand_eye_calib");
                    config_file = package_path + "/config/calib_config.yaml";
                    if (!fs::exists(config_file)) {
                        config_file = "";
                    }
                } catch (const std::exception& e) {
                    config_file = "";
                }
            }
        }
    }
    
    if (!config_file.empty() && config.loadConfig(config_file)) {
        LOG_INFO("✅ 成功加载配置文件: %s", config_file.c_str());
    } else {
        LOG_WARN("⚠️  加载配置文件失败，使用默认配置");
    }
    
    // 检查参数
    if (argc >= 2) {
        std::string mode = argv[1];
        if (mode == "--help") {
            printUsage();
            return 0;
        } else {
            LOG_ERROR("❌ 未知模式: %s", mode.c_str());
            printUsage();
            return 1;
        }
    }
    
    // 默认执行数据采集功能
    runDataCollection(config);
    
    return 0;
}
