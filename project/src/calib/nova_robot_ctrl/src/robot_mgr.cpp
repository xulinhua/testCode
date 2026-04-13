#include "nova_robot_ctrl/robot_mgr.h"
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
namespace nova_robot_ctrl
{

    RobotMgr::RobotMgr()
    {
        // 初始化配置文件路径
        
        std::string install_path = ament_index_cpp::get_package_share_directory("nova_robot_ctrl")+"/../../../";
        robot_sys_config_path_ = install_path + "/bas_config_data/cam_config/sys_arm_config.yaml";
        
        // 初始化机器人参数
        init_default_param();
        
        // 初始化USB扫描结果相关变量
        usb_scan_performed_ = false;
        
        // 初始化自动创建配置文件标志
        is_create_default_file_ = true;
    }

    RobotMgr::~RobotMgr()
    {
        // 释放所有机器人控制对象
        for (auto& pair : robot_ctrls_) {
            if (pair.second) {
                delete pair.second;
                pair.second = nullptr;
            }
        }
        
        // 释放所有夹爪控制对象
        for (auto& pair : gripper_ctrls_) {
            if (pair.second) {
                delete pair.second;
                pair.second = nullptr;
            }
        }
        
        // 清空maps
        robot_ctrls_.clear();
        gripper_ctrls_.clear();
        robot_configs_.clear();
        gripper_configs_.clear();
        robot_enables_.clear();
    }
    
    bool RobotMgr::init()
    {
        bool res = true;
        // 初始化所有启用的机器人
        for (const auto& enable_pair : robot_enables_) {
            if (enable_pair.second) {  // 如果机器人启用
                res &= enable_robot(enable_pair.first);
                // 只有当夹爪启用时才初始化夹爪
                if (gripper_configs_.count(enable_pair.first) && 
                    gripper_configs_[enable_pair.first].enable) {
                       enable_gripper(enable_pair.first);//夹爪初始化是否成功不影响机械臂运行
                }
            }
        }
        return res;
    }

    bool RobotMgr::release()
    {
        bool res = true;
        // 释放所有启用的机器人
        for (const auto& enable_pair : robot_enables_) {
            if (enable_pair.second) {  // 如果机器人启用
                res &= disable_robot(enable_pair.first);
                // 只有当夹爪启用时才释放夹爪
                if (gripper_configs_.count(enable_pair.first) && 
                    gripper_configs_[enable_pair.first].enable) {
                    res &= disable_gripper(enable_pair.first);
                }
            }
        }
        return res;
    }

    bool RobotMgr::create_robot_sys_config()
    {
        try
        {
            YAML::Node config;
            
            // 创建系统配置节点
            YAML::Node sys_config_node;
            sys_config_node["arm_num"] = robot_configs_.size();
            config["sys_arm_config"] = sys_config_node;
            
            // 为每个机器人创建配置
            for (const auto& robot_pair : robot_configs_) {
                int robot_id = robot_pair.first;
                const RobotConfig& robot_config = robot_pair.second;
                
                YAML::Node robot_node;
                
                // 添加enable节点
                robot_node["is_enable"] = robot_enables_[robot_id];
                
                // 添加IP地址
                robot_node["robot_arm_ip"] = robot_config.ip;
                
                // 添加用户名
                if (!robot_config.user_name.empty()) {
                    robot_node["username"] = robot_config.user_name;
                } else {
                    robot_node["username"] = "robot_arm_" + std::to_string(robot_id);
                }
                
                // 将机器人节点添加到主配置中
                config["arm_" + std::to_string(robot_id)] = robot_node;
            }

            // 检查并创建目录
            std::filesystem::path config_path(robot_sys_config_path_);
            std::filesystem::path config_dir = config_path.parent_path();
            if (!std::filesystem::exists(config_dir)) {
                std::error_code ec;
                if (!std::filesystem::create_directories(config_dir, ec)) {
                    std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无法创建目录: " << config_dir << ", 错误: " << ec.message() << std::endl;
                    return false;
                }
                std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 成功创建目录: " << config_dir << std::endl;
            }
            
            // 写入文件
            std::ofstream fout(robot_sys_config_path_);
            if (!fout.is_open())
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无法打开配置文件进行写入: " << robot_sys_config_path_ << std::endl;
                return false;
            }

            fout << config;
            fout.close();

            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人系统配置文件创建成功: " << robot_sys_config_path_ << std::endl;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 创建机器人系统配置时发生异常: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool RobotMgr::create_arm_sys_config(int robot_id)
    {
        try
        {
            // 检查机器人是否存在
            if (robot_configs_.find(robot_id) == robot_configs_.end()) {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 未找到ID为 " << robot_id << " 的机器人" << std::endl;
                return false;
            }
            
            // 生成配置文件路径
            std::string config_path = get_arm_config_path(robot_id);
            
            YAML::Node config;
            
            // 创建机器人节点名
            std::string robot_key = "robot_" + std::to_string(robot_id);
            
            YAML::Node robot_node;
            
            // 添加enable节点
            robot_node["enable"] = robot_enables_[robot_id];
            
            // 创建机器人配置
            YAML::Node robot_config_node;
            robot_config_node["ip"] = robot_configs_[robot_id].ip;
            if (!robot_configs_[robot_id].user_name.empty()) {
                robot_config_node["user_name"] = robot_configs_[robot_id].user_name;
            }
            
            // 为每个工具坐标系ID创建移动范围参数
            for (const auto &range_pair : robot_configs_[robot_id].ranges)
            {
                int tool_id = range_pair.first;
                const RobotCtrlRange &range_values = range_pair.second;

                YAML::Node range_node;
                range_node["x_min"] = range_values.x_min;
                range_node["x_max"] = range_values.x_max;
                range_node["y_min"] = range_values.y_min;
                range_node["y_max"] = range_values.y_max;
                range_node["z_min"] = range_values.z_min;
                range_node["z_max"] = range_values.z_max;

                robot_config_node["tool_" + std::to_string(tool_id)] = range_node;
            }
            
            robot_node["robot_config"] = robot_config_node;
            
            // 创建夹爪配置（如果存在）
            if (gripper_configs_.count(robot_id)) {
                const GripperConfig& gripper_config = gripper_configs_[robot_id];
                
                YAML::Node gripper_config_node;
                gripper_config_node["enable"] = gripper_config.enable;  // 添加enable配置项
                gripper_config_node["serial_port"] = gripper_config.serial_port;
                gripper_config_node["id"] = gripper_config.id;
                if (!gripper_config.user_name.empty()) {
                    gripper_config_node["user_name"] = gripper_config.user_name;
                }
                gripper_config_node["baudrate"] = gripper_config.baudrate;
                
                // 创建夹爪参数（只有一组）
                YAML::Node servo_node;
                servo_node["pos_min"] = gripper_config.servo_ranges.pos_min;
                servo_node["pos_max"] = gripper_config.servo_ranges.pos_max;
                servo_node["speed_min"] = gripper_config.servo_ranges.speed_min;
                servo_node["speed_max"] = gripper_config.servo_ranges.speed_max;
                servo_node["force_min"] = gripper_config.servo_ranges.force_min;
                servo_node["force_max"] = gripper_config.servo_ranges.force_max;

                gripper_config_node["servo_params"] = servo_node;
                robot_node["gripper_config"] = gripper_config_node;
            }
            
            // 将机器人节点添加到主配置中
            config[robot_key] = robot_node;

            // 检查并创建目录
            std::filesystem::path arm_config_path(config_path);
            std::filesystem::path arm_config_dir = arm_config_path.parent_path();
            if (!std::filesystem::exists(arm_config_dir)) {
                std::error_code ec;
                if (!std::filesystem::create_directories(arm_config_dir, ec)) {
                    std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无法创建目录: " << arm_config_dir.c_str() << ", 错误: " << ec.message().c_str() << std::endl;
                    return false;
                }
                std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 成功创建目录: " << arm_config_dir.c_str() << std::endl;
            }
            
            // 写入文件
            std::ofstream fout(config_path);
            if (!fout.is_open())
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无法打开配置文件进行写入: " << config_path.c_str() << std::endl;
                return false;
            }

            fout << config;
            fout.close();

            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机械臂配置文件创建成功: " << config_path.c_str() << std::endl;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 创建机械臂配置时发生异常: " << e.what() << std::endl;
            return false;
        }
    }

    bool RobotMgr::load_robot_sys_config()
    {
        try
        {
            // 检查配置文件是否存在
            std::ifstream fin(robot_sys_config_path_);
            if (!fin.is_open())
            {
                if (is_create_default_file_) {
                    std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 找不到机器人系统配置文件: " << robot_sys_config_path_.c_str() << "，正在创建默认配置" << std::endl;
                    return create_robot_sys_config();
                } else {
                    std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 找不到机器人系统配置文件: " << robot_sys_config_path_.c_str() << "，且未启用自动创建功能" << std::endl;
                    return false;
                }
            }
            fin.close();

            // 加载配置文件
            YAML::Node config = YAML::LoadFile(robot_sys_config_path_);
            
            // 清空现有配置
            robot_configs_.clear();
            robot_enables_.clear();
            
            // 加载系统配置
            if (config["sys_arm_config"]) {
                YAML::Node sys_cfg = config["sys_arm_config"];
                // 系统配置中的arm_num暂时不使用，因为我们直接从arm_x节点获取机械臂数量
            }
            
            // 遍历所有机器人配置节点
            for (const auto& robot_node : config) {
                std::string robot_key = robot_node.first.as<std::string>();
                
                // 检查是否是机器人配置节点 (arm_x格式)
                if (robot_key.find("arm_") == 0) {
                    try {
                        std::string robot_id_str = robot_key.substr(4); // 去掉"arm_"前缀
                        int robot_id = std::stoi(robot_id_str);
                        
                        YAML::Node robot_cfg_node = robot_node.second;
                        
                        // 默认启用机器人
                        robot_enables_[robot_id] = true;
                        if (robot_cfg_node["is_enable"]) {
                            robot_enables_[robot_id] = robot_cfg_node["is_enable"].as<bool>();
                        }
                        
                        // 加载机器人配置
                        RobotConfig robot_config;
                        if (robot_cfg_node["robot_arm_ip"]) {
                            robot_config.ip = robot_cfg_node["robot_arm_ip"].as<std::string>();
                        }
                        if (robot_cfg_node["username"]) {
                            robot_config.user_name = robot_cfg_node["username"].as<std::string>();
                        }
                        
                        robot_configs_[robot_id] = robot_config;
                    } catch (const std::exception &e) {
                        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 解析配置中的机器人ID失败: " << robot_key.c_str() << "，错误: " << e.what() << std::endl;
                    }
                }
            }

            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人系统配置已成功加载自: " << robot_sys_config_path_.c_str() << std::endl;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 加载机器人系统配置时发生异常: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool RobotMgr::load_arm_sys_config(int robot_id)
    {
        try
        {
            // 生成配置文件路径
            std::string config_path = get_arm_config_path(robot_id);
            
            // 检查配置文件是否存在
            std::ifstream fin(config_path);
            if (!fin.is_open())
            {
                if (is_create_default_file_) {
                    std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 找不到机械臂配置文件: " << config_path.c_str() << "，正在创建默认配置" << std::endl;
                    return create_arm_sys_config(robot_id);
                } else {
                    std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 找不到机械臂配置文件: " << config_path.c_str() << "，且未启用自动创建功能" << std::endl;
                    return false;
                }
            }
            fin.close();

            // 加载配置文件
            YAML::Node config = YAML::LoadFile(config_path);
            
            // 遍历所有机器人配置节点
            for (const auto& robot_node : config) {
                std::string robot_key = robot_node.first.as<std::string>();
                
                // 检查是否是机器人配置节点 (robot_x格式)
                if (robot_key.find("robot_") == 0) {
                    try {
                        std::string robot_id_str = robot_key.substr(6); // 去掉"robot_"前缀
                        int cfg_robot_id = std::stoi(robot_id_str);
                        
                        // 只处理指定ID的机器人配置
                        if (cfg_robot_id != robot_id) {
                            continue;
                        }
                        
                        YAML::Node robot_cfg_node = robot_node.second;
                        
                        // 只有当启用状态未设置时才从配置文件读取
                        if (!robot_enables_.count(robot_id) && robot_cfg_node["enable"]) {
                            robot_enables_[robot_id] = robot_cfg_node["enable"].as<bool>();
                        }
                        
                        // 加载机器人配置
                        if (robot_cfg_node["robot_config"])
                        {
                            RobotConfig robot_config;
                            YAML::Node robot_cfg = robot_cfg_node["robot_config"];
                            // 只有当IP地址为空时才从配置文件读取
                            if (robot_config.ip.empty() && robot_cfg["ip"])
                                robot_config.ip = robot_cfg["ip"].as<std::string>();
                            // 只有当用户名为空时才从配置文件读取
                            if (robot_config.user_name.empty() && robot_cfg["user_name"])
                                robot_config.user_name = robot_cfg["user_name"].as<std::string>();

                            // 加载每个工具坐标系的移动范围参数
                            for (const auto &tool_node : robot_cfg)
                            {
                                if (tool_node.first.as<std::string>().find("tool_") == 0)
                                {
                                    try {
                                        std::string tool_str = tool_node.first.as<std::string>().substr(5); // 去掉"tool_"前缀
                                        int tool_id = std::stoi(tool_str);

                                        YAML::Node range_cfg = tool_node.second;
                                        RobotCtrlRange range_values;
                                        if (range_cfg["x_min"])
                                            range_values.x_min = range_cfg["x_min"].as<double>();
                                        if (range_cfg["x_max"])
                                            range_values.x_max = range_cfg["x_max"].as<double>();
                                        if (range_cfg["y_min"])
                                            range_values.y_min = range_cfg["y_min"].as<double>();
                                        if (range_cfg["y_max"])
                                            range_values.y_max = range_cfg["y_max"].as<double>();
                                        if (range_cfg["z_min"])
                                            range_values.z_min = range_cfg["z_min"].as<double>();
                                        if (range_cfg["z_max"])
                                            range_values.z_max = range_cfg["z_max"].as<double>();

                                        robot_config.ranges[tool_id] = range_values;
                                    } catch (const std::exception &e) {
                                        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 解析机器人 " << robot_id << " 配置中的工具ID失败: " << e.what() << std::endl;
                                    }
                                }
                            }
                            
                            robot_configs_[robot_id] = robot_config;
                        }

                        // 加载夹爪配置
                        if (robot_cfg_node["gripper_config"])
                        {
                            GripperConfig gripper_config;
                            YAML::Node gripper_cfg = robot_cfg_node["gripper_config"];
                            if (gripper_cfg["id"])
                                gripper_config.id = gripper_cfg["id"].as<int>();
                    
                            if (gripper_cfg["serial_port"])
                                gripper_config.serial_port = gripper_cfg["serial_port"].as<std::string>();
                                
                            if (gripper_cfg["user_name"])
                                gripper_config.user_name = gripper_cfg["user_name"].as<std::string>();
                                
                            if (gripper_cfg["baudrate"])
                                gripper_config.baudrate = gripper_cfg["baudrate"].as<int>();
                                
                            if (gripper_cfg["enable"])
                                gripper_config.enable = gripper_cfg["enable"].as<bool>();
                            else
                                gripper_config.enable = true;  // 默认启用

                            // 加载夹爪参数（只有一组）
                            if (gripper_cfg["servo_params"])
                            {
                                YAML::Node servo_cfg = gripper_cfg["servo_params"];
                                ServoCtrlRange servo_range;
                                if (servo_cfg["pos_min"])
                                    servo_range.pos_min = servo_cfg["pos_min"].as<int>();
                                if (servo_cfg["pos_max"])
                                    servo_range.pos_max = servo_cfg["pos_max"].as<int>();
                                if (servo_cfg["speed_min"])
                                    servo_range.speed_min = servo_cfg["speed_min"].as<int>();
                                if (servo_cfg["speed_max"])
                                    servo_range.speed_max = servo_cfg["speed_max"].as<int>();
                                if (servo_cfg["force_min"])
                                    servo_range.force_min = servo_cfg["force_min"].as<int>();
                                if (servo_cfg["force_max"])
                                    servo_range.force_max = servo_cfg["force_max"].as<int>();
                                gripper_config.servo_ranges = servo_range;
                            }
                            
                            gripper_configs_[robot_id] = gripper_config;
                        }
                    } catch (const std::exception &e) {
                        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 解析配置中的机器人ID失败: " << robot_key.c_str() << "，错误: " << e.what() << std::endl;
                    }
                }
            }

            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机械臂配置已成功加载自: " << config_path.c_str() << std::endl;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 加载机械臂配置时发生异常: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool RobotMgr::load_config(int mode)
    {
        if (mode == 0) {
            // 从文件加载配置
            if (!load_robot_sys_config()) {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 从文件加载机器人系统配置失败" << std::endl;
                return false;
            }
            
            // 加载所有启用的机器人配置
            for (const auto& pair : robot_enables_) {
                int robot_id = pair.first;
                if (pair.second) {  // 如果机器人已启用
                    if (!load_arm_sys_config(robot_id)) {
                        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 从文件加载机器人 " << robot_id << " 配置失败" << std::endl;
                        return false;
                    }
                }
            }
            
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 成功从文件加载所有配置" << std::endl;
        } else if (mode == 1) {
            // 从参数服务器加载配置
            // if (!load_robot_sys_srv()) {
            //     std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 从参数服务器加载机器人系统配置失败" << std::endl;
            //     return false;
            // }
            
            // 加载所有启用的机器人配置
            for (const auto& pair : robot_enables_) {
                int robot_id = pair.first;
                if (pair.second) {  // 如果机器人已启用
                    if (!load_arm_sys_config(robot_id)) {
                        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 加载机器人 " << robot_id << " 配置失败" << std::endl;
                        return false;
                    }
                }
            }
            
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 成功从参数服务器加载所有配置" << std::endl;
        } else {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无效的加载模式: " << mode << std::endl;
            return false;
        }
        
        // 打印所有机器人和夹爪的配置信息
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - ==== 打印所有机器人配置信息 ====" << std::endl;
        for (const auto& pair : robot_configs_) {
            int robot_id = pair.first;
            const RobotConfig& config = pair.second;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID: " << robot_id << ", 启用状态: " << (robot_enables_.at(robot_id) ? "是" : "否") << std::endl;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   IP地址: " << config.ip.c_str() << std::endl;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   用户名: " << config.user_name.c_str() << std::endl;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   移动范围参数数量: " << config.ranges.size() << std::endl;
            for (const auto& range_pair : config.ranges) {
                int tool_id = range_pair.first;
                const RobotCtrlRange& range = range_pair.second;
                std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -     工具坐标系ID " << tool_id << ": x_min=" << std::fixed << std::setprecision(2) << range.x_min << ", x_max=" << range.x_max << ", y_min=" << range.y_min << ", y_max=" << range.y_max << ", z_min=" << range.z_min << ", z_max=" << range.z_max << std::endl;
            }
        }
        
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - ==== 打印所有夹爪配置信息 ====" << std::endl;
        for (const auto& pair : gripper_configs_) {
            int robot_id = pair.first;
            const GripperConfig& config = pair.second;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID: " << robot_id << ", 夹爪ID: " << config.id << std::endl;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   串口路径: " << config.serial_port.c_str() << std::endl;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   用户名: " << config.user_name.c_str() << std::endl;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   波特率: " << config.baudrate << std::endl;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   启用状态: " << (config.enable ? "是" : "否") << std::endl;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   舵机参数: pos_min=" << config.servo_ranges.pos_min << ", pos_max=" << config.servo_ranges.pos_max << ", speed_min=" << config.servo_ranges.speed_min << ", speed_max=" << config.servo_ranges.speed_max << ", force_min=" << config.servo_ranges.force_min << ", force_max=" << config.servo_ranges.force_max << std::endl;
        }
        
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - ==== 配置信息打印完毕 ====" << std::endl;
        
        return true;
    }

    // 机械手控制接口
    bool RobotMgr::enable_robot(int robot_id)
    {
        // 检查机器人是否存在
        if (robot_configs_.find(robot_id) == robot_configs_.end()) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 未找到ID为 " << robot_id << " 的机器人" << std::endl;
            return false;
        }
        
        // 如果机器人控制对象不存在，则创建
        if (robot_ctrls_.find(robot_id) == robot_ctrls_.end()) {
            robot_ctrls_[robot_id] = new NovaRobotCtrl(0, 0);
        }
        
        // 连接机器人
        if (!robot_ctrls_[robot_id]->open(robot_configs_[robot_id].ip))
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无法连接到IP为 " << robot_configs_[robot_id].ip.c_str() << " 的机器人" << std::endl;
            return false;
        }
        
        // 使能机器人
        if (!robot_ctrls_[robot_id]->enable_robot())
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 启用机器人失败" << std::endl;
            robot_ctrls_[robot_id]->close();
            return false;
        }
        
        // 更新机器人状态
        robot_states_[robot_id].robot_open = true;
        
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人系统已成功连接并启用，机器人ID: " << robot_id << std::endl;
        return true;
    }

    bool RobotMgr::disable_robot(int robot_id)
    {
        // 检查机器人是否存在
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            robot_ctrls_[robot_id]->close();
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人系统已禁用并断开连接，机器人ID: " << robot_id << std::endl;
        }
        
        // 更新机器人状态
        if (robot_states_.find(robot_id) != robot_states_.end()) {
            robot_states_[robot_id].robot_open = false;
        }
        
        return true;
    }

    bool RobotMgr::servo_j_robot(const std::vector<double> &joint, float t, float aheadtime, float gain, int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->servo_j(joint, t, aheadtime, gain);
        }
        return false;
    }

    bool RobotMgr::servo_p_robot(const std::vector<double> &joint, float t, float aheadtime, float gain, int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->servo_p(joint, t, aheadtime, gain);
        }
        return false;
    }

    bool RobotMgr::get_angle_robot(std::vector<double> &joint, int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->get_angle(joint);
        }
        return false;
    }

    bool RobotMgr::get_current_pose_robot(Pose &pose, int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->get_current_pose(pose);
        }
        return false;
    }

    bool RobotMgr::set_user_coordinate_robot(int user_id, int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->set_user_coordinate(user_id);
        }
        return false;
    }

    bool RobotMgr::set_tool_coordinate_robot(int tool_id, int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->set_tool_coordinate(tool_id);
        }
        return false;
    }

    bool RobotMgr::is_position_valid_robot(double x, double y, double z, int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->is_position_valid(x, y, z);
        }
        return false;
    }

    bool RobotMgr::move_l_robot(double x, double y, double z, double rx, double ry, double rz, int speed, int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->move_l(x, y, z, rx, ry, rz, speed);
        }
        return false;
    }
    
    bool RobotMgr::jog_move_robot(char axis, double distance, int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->jog_move(axis, distance);
        }
        return false;
    }

    bool RobotMgr::sync(int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->sync();
        }
        return false;
    }

    bool RobotMgr::is_connected_robot(int robot_id) const
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_.at(robot_id)->is_connected();
        }
        return false;
    }

    bool RobotMgr::get_standard_pose_robot(Pose &pose, int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->get_standard_pose(pose);
        }
        return false;
    }

    bool RobotMgr::move_to_standard_pose_robot(int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->move_to_standard_pose();
        }
        return false;
    }

    void RobotMgr::set_robot_id_robot(int robot_id, int robot_index)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->set_robot_id(robot_index);
        }
    }
    
    nova_robot_ctrl::NovaRobotCtrl::pushed_info RobotMgr::get_pushed_info_robot(int robot_id)
    {
        if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
            return robot_ctrls_[robot_id]->get_pushed_info();
        }
        // 返回默认构造的pushed_info
        return nova_robot_ctrl::NovaRobotCtrl::pushed_info();
    }
    
    // nova_robot_ctrl::CalibRobotPosMgr* RobotMgr::get_calib_robot_pos_mgr_robot(int robot_id)
    // {
    //     if (robot_ctrls_.find(robot_id) != robot_ctrls_.end()) {
    //         return robot_ctrls_[robot_id]->get_calib_robot_pos_mgr();
    //     }
    //     return nullptr;
    // }
    
    // 夹爪控制接口
    bool RobotMgr::enable_gripper(int robot_id)
    {
        // 检查机器人是否存在
        if (gripper_configs_.find(robot_id) == gripper_configs_.end()) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 未找到机器人ID " << robot_id << " 的夹爪配置" << std::endl;
            return true;  // 没有夹爪配置不算错误
        }
        
        // 检查夹爪是否启用
        if(!gripper_configs_[robot_id].enable)
        {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未启用" << std::endl;
            return true;
        }
        
        // 如果还没有执行过USB端口扫描，则执行一次扫描
        if (!usb_scan_performed_) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 首次执行USB端口扫描" << std::endl;
            usb_scan_results_ = NovaGripperCtrl::scan_usb_ports_and_servos();
            usb_scan_performed_ = true;
            
            // 打印扫描结果
            if (usb_scan_results_.empty()) {
                std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 扫描期间未找到带有舵机的USB端口" << std::endl;
            } else {
                std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 找到 " << usb_scan_results_.size() << " 个带有舵机的USB端口:" << std::endl;
                for (size_t i = 0; i < usb_scan_results_.size(); ++i) {
                    const auto& port_servos = usb_scan_results_[i];
                    for (const auto& [port, servo_id] : port_servos) {
                        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -   端口 " << i+1 << ": " << port.c_str() << ", 舵机ID: " << servo_id << std::endl;
                    }
                }
            }
        }
        
        // 检查扫描结果，如果gripper_configs_[robot_id].id与扫描结果中的ID相同，
        // 但gripper_configs_[robot_id].serial_port与扫描结果中的端口不一致，则自动修改serial_port
        for (const auto& port_servo_map : usb_scan_results_) {
            for (const auto& [port, servo_id] : port_servo_map) {
                if (servo_id == gripper_configs_[robot_id].id) {
                    if (gripper_configs_[robot_id].serial_port != port) {
                        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在为机器人ID " << robot_id << " 更新串口，从 " << gripper_configs_[robot_id].serial_port.c_str() << " 到 " << port.c_str() << " (发现舵机ID " << servo_id << ")" << std::endl;
                        gripper_configs_[robot_id].serial_port = port;
                    }
                    break;
                }
            }
        }
        
        // 如果夹爪控制对象不存在，则创建
        if (gripper_ctrls_.find(robot_id) == gripper_ctrls_.end()) {
            // 将ServoCtrlRange转换为std::vector<int>
            std::vector<int> servo_pos = {
                gripper_configs_[robot_id].servo_ranges.pos_min,
                gripper_configs_[robot_id].servo_ranges.pos_max,
                gripper_configs_[robot_id].servo_ranges.speed_min,
                gripper_configs_[robot_id].servo_ranges.speed_max,
                gripper_configs_[robot_id].servo_ranges.force_min,
                gripper_configs_[robot_id].servo_ranges.force_max
            };
            gripper_ctrls_[robot_id] = new NovaGripperCtrl(
                gripper_configs_[robot_id].serial_port, 
                gripper_configs_[robot_id].id, 
                servo_pos);
        } else {
            // 更新夹爪配置
            gripper_ctrls_[robot_id]->set_port_and_id(
                gripper_configs_[robot_id].serial_port, 
                gripper_configs_[robot_id].id);
        }
        
        // 连接夹爪设备
        if (!gripper_ctrls_[robot_id]->connect())
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无法连接到机器人ID " << robot_id << " 的夹爪" << std::endl;
            return false;
        }

        // Ping夹爪设备以检查连接
        if (!gripper_ctrls_[robot_id]->ping())
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无法ping通机器人ID " << robot_id << " 的夹爪" << std::endl;
            gripper_ctrls_[robot_id]->disconnect();
            return false;
        }
        
        // 更新夹爪状态
        if (robot_states_.find(robot_id) != robot_states_.end()) {
            robot_states_[robot_id].gripper_open = true;
        }
        
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪已成功启用" << std::endl;
        return true;
    }

    bool RobotMgr::disable_gripper(int robot_id)
    {
        // 检查夹爪是否存在
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end()) {
            // 断开夹爪设备连接
            gripper_ctrls_[robot_id]->disconnect();
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪已成功禁用" << std::endl;
        }
        
        // 更新夹爪状态
        if (robot_states_.find(robot_id) != robot_states_.end()) {
            robot_states_[robot_id].gripper_open = false;
        }
        
        return true;
    }

    bool RobotMgr::gripper_open(int robot_id)
    {
        // 检查机器人是否存在且夹爪启用
        if (gripper_configs_.find(robot_id) == gripper_configs_.end() || 
            !gripper_configs_[robot_id].enable) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未启用" << std::endl;
            return true;
        }
         
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end() && 
            !gripper_ctrls_[robot_id]->is_connected())
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未连接" << std::endl;
            return false;
        }

        // 张开夹爪动作
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end()) {
            auto result = gripper_ctrls_[robot_id]->move(gripper_ctrls_[robot_id]->get_open_position(), 50, 0);
            if (!result.first)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无法打开机器人ID " << robot_id << " 的夹爪，错误代码: " << result.second << std::endl;
                return false;
            }

            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪已成功打开" << std::endl;
        }
        return true;
    }

    bool RobotMgr::gripper_close(int robot_id)
    {
        // 检查机器人是否存在且夹爪启用
        if (gripper_configs_.find(robot_id) == gripper_configs_.end() || 
            !gripper_configs_[robot_id].enable) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未启用" << std::endl;
            return true;
        }
        
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end() && 
            !gripper_ctrls_[robot_id]->is_connected())
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未连接" << std::endl;
            return false;
        }

        // 闭合夹爪动作
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end()) {
            auto result = gripper_ctrls_[robot_id]->move(gripper_ctrls_[robot_id]->get_closed_position(), 50, 0);
            if (!result.first)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无法关闭机器人ID " << robot_id << " 的夹爪，错误代码: " << result.second << std::endl;
                return false;
            }

            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪已成功关闭" << std::endl;
        }
        return true;
    }

    bool RobotMgr::is_open_gripper(int robot_id) const
    {
        // 检查机器人是否存在且夹爪启用
        if (gripper_configs_.find(robot_id) == gripper_configs_.end() || 
            !gripper_configs_.at(robot_id).enable) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未启用" << std::endl;
            return false;
        }
        
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end()) {
            return gripper_ctrls_.at(robot_id)->is_open();
        }
        return false;
    }

    bool RobotMgr::is_closed_gripper(int robot_id) const
    {
        // 检查机器人是否存在且夹爪启用
        if (gripper_configs_.find(robot_id) == gripper_configs_.end() || 
            !gripper_configs_.at(robot_id).enable) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未启用" << std::endl;
            return true;
        }
        
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end()) {
            return gripper_ctrls_.at(robot_id)->is_closed();
        }
        return true;
    }

    int RobotMgr::get_current_position_gripper(int robot_id) const
    {
        // 检查机器人是否存在且夹爪启用
        if (gripper_configs_.find(robot_id) == gripper_configs_.end() || 
            !gripper_configs_.at(robot_id).enable) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未启用" << std::endl;
            return -1;
        }
        
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end()) {
            return gripper_ctrls_.at(robot_id)->get_current_position();
        }
        return -1;
    }

    int RobotMgr::get_current_speed_gripper(int robot_id) const
    {
        // 检查机器人是否存在且夹爪启用
        if (gripper_configs_.find(robot_id) == gripper_configs_.end() || 
            !gripper_configs_.at(robot_id).enable) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未启用" << std::endl;
            return -1;
        }
        
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end()) {
            return gripper_ctrls_.at(robot_id)->get_current_speed();
        }
        return -1;
    }

    std::pair<bool, int> RobotMgr::move_gripper(int position, int speed, int force, int robot_id)
    {
        // 检查机器人是否存在且夹爪启用
        if (gripper_configs_.find(robot_id) == gripper_configs_.end() || 
            !gripper_configs_[robot_id].enable) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未启用" << std::endl;
            return std::pair<bool, int>(false, -1);
        }
        
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end()) {
            return gripper_ctrls_[robot_id]->move(position, speed, force);
        }
        return std::pair<bool, int>(false, -1);
    }

    std::pair<bool, int> RobotMgr::reset_position_gripper(int speed, int force, int robot_id)
    {
        // 检查机器人是否存在且夹爪启用
        if (gripper_configs_.find(robot_id) == gripper_configs_.end() || 
            !gripper_configs_[robot_id].enable) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未启用" << std::endl;
            return std::pair<bool, int>(false, -1);
        }
        
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end()) {
            return gripper_ctrls_[robot_id]->reset_position(speed, force);
        }
        return std::pair<bool, int>(false, -1);
    }

    std::pair<bool, int> RobotMgr::move_by_abs_pos_gripper(int position, int speed, int force, int robot_id)
    {
        // 检查机器人是否存在且夹爪启用
        if (gripper_configs_.find(robot_id) == gripper_configs_.end() || 
            !gripper_configs_[robot_id].enable) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未启用" << std::endl;
            return std::pair<bool, int>(false, -1);
        }
        
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end()) {
            return gripper_ctrls_[robot_id]->move_by_abs_pos(position, speed, force);
        }
        return std::pair<bool, int>(false, -1);
    }

    bool RobotMgr::set_gripper_port_and_id(const std::string& port, int id, int robot_id)
    {
        // 更新夹爪配置
        if (gripper_configs_.find(robot_id) != gripper_configs_.end()) {
            gripper_configs_[robot_id].serial_port = port;
            gripper_configs_[robot_id].id = id;
        }
        
        // 调用夹爪控制器的设置方法
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end()) {
            return gripper_ctrls_[robot_id]->set_port_and_id(port, id);
        }
        return false;
    }
    
    bool RobotMgr::ping_gripper(int robot_id)
    {
        // 检查机器人是否存在且夹爪启用
        if (gripper_configs_.find(robot_id) == gripper_configs_.end() || 
            !gripper_configs_[robot_id].enable) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未启用" << std::endl;
            return false;
        }
        
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end() && 
            !gripper_ctrls_[robot_id]->is_connected())
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪未连接" << std::endl;
            return false;
        }
        
        // Ping夹爪设备以检查连接
        if (gripper_ctrls_.find(robot_id) != gripper_ctrls_.end()) {
            bool result = gripper_ctrls_[robot_id]->ping();
            if (!result)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无法ping通机器人ID " << robot_id << " 的夹爪" << std::endl;
                return false;
            }
            
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人ID " << robot_id << " 的夹爪ping成功" << std::endl;
        }
        return true;
    }
    
    std::vector<int> RobotMgr::get_all_enable_robots_id() const
    {
        std::vector<int> enable_robots;
        for (const auto& enable_pair : robot_enables_) {
            if (enable_pair.second) {  // 如果机器人启用
                enable_robots.push_back(enable_pair.first);
            }
        }
        return enable_robots;
    }
    
    bool RobotMgr::load_robot_sys_srv()
    {
        // TODO: 需要重构此函数以正确获取节点引用
        // 当前RobotMgr类不是从rclcpp::Node派生的，因此无法直接调用shared_from_this()
        // 需要通过其他方式传递节点引用或重构此类
        //LOG_WARN(PROJECT_NAME, __FILE__, __FUNCTION__, __LINE__, "从参数服务器加载机器人系统配置功能尚未完全实现");
        return true;
        
        /*
        try
        {
            LOG_INFO(PROJECT_NAME, __FILE__, __FUNCTION__, __LINE__, "开始从参数服务器加载机器人系统配置信息...");
            
            // 创建连接到系统配置节点的参数客户端
            // 注意：需要使用参数客户端来获取远程节点的参数
            // auto node = rclcpp::Node::shared_from_this(); // 这行代码有问题，因为RobotMgr不是从rclcpp::Node派生的
            // auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "sys_config_ros_node");
            
            // 等待参数服务可用
            // if (!parameters_client->wait_for_service(std::chrono::seconds(3)))
            // {
            //     LOG_ERROR(PROJECT_NAME, __FILE__, __FUNCTION__, __LINE__, 
            //                  "无法连接到系统配置参数服务");
            //     return false;
            // }
            
            // 使用正确的接口从参数服务器读取机械臂配置信息列表
            // SysConfig::ArmConfigInfoList sys_arm_list;
            // if (!RosComm::armConfigInfoListFromServer(parameters_client, SYS_ENABLE_ARM_LIST, sys_arm_list))
            // {
            //     std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -
            //                   "从参数服务器读取机械臂配置信息列表失败");
            //     return false;
            // }

            // std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -
            //              "成功从参数服务器获取到 %zu 个机械臂配置", sys_arm_list.size());
            
            // 打印所有获取到的机械臂配置信息
            // LOG_INFO(PROJECT_NAME, __FILE__, __FUNCTION__, __LINE__, "=== 从参数服务器获取的机械臂配置信息 ===");
            
            // 遍历获取到的机械臂配置信息
            // for (size_t i = 0; i < sys_arm_list.size(); ++i)
            // {
            //     const auto &sys_arm_info = sys_arm_list[i];
            //     
            //     std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -
            //                  "机械臂配置 %zu:", i);
            //     std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -
            //                  "  - 机械臂ID: %d", static_cast<int>(sys_arm_info.arm_id));
            //     std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -
            //                  "  - 是否启用: %s", sys_arm_info.is_enable ? "是" : "否");
            //     std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -
            //                  "  - IP地址: %s", sys_arm_info.robot_arm_ip.c_str());
            //     std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -
            //                  "  - 用户名称: %s", sys_arm_info.user_name.c_str());
            //     
            //     // 更新机器人配置信息
            //     if (robot_configs_.find(sys_arm_info.arm_id) != robot_configs_.end()) {
            //         robot_configs_[sys_arm_info.arm_id].ip = sys_arm_info.robot_arm_ip;
            //         robot_configs_[sys_arm_info.arm_id].user_name = sys_arm_info.user_name;
            //         robot_enables_[sys_arm_info.arm_id] = sys_arm_info.is_enable;
            //         
            //         std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -
            //                      "成功更新机器人 %d 配置", static_cast<int>(sys_arm_info.arm_id));
            //     } else {
            //         LOG_WARN(PROJECT_NAME, __FILE__, __FUNCTION__, __LINE__,
            //                      "机器人 %d 不存在于配置中，跳过更新", static_cast<int>(sys_arm_info.arm_id));
            //     }
            //     
            //     // 加载单个机器人配置文件
            //     if (load_arm_sys_config(sys_arm_info.arm_id))
            //     {
            //         std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -
            //                      "成功加载机器人 %d 配置文件", static_cast<int>(sys_arm_info.arm_id));
            //     }
            //     else
            //     {
            //         std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -
            //                       "加载机器人 %d 配置文件失败", static_cast<int>(sys_arm_info.arm_id));
            //     }
            // }
            
            // LOG_INFO(PROJECT_NAME, __FILE__, __FUNCTION__, __LINE__, "=== 机械臂配置信息加载完成 ===");
            // return true;
        }
        catch (const std::exception &e)
        {
            // std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " -
            //               "从参数服务器加载机器人系统配置时发生错误: %s", e.what());
            // return false;
        }
        */
    }
    
    bool RobotMgr::get_robot_config(int robot_id, RobotConfig& config) const
    {
        auto it = robot_configs_.find(robot_id);
        if (it != robot_configs_.end()) {
            config = it->second;
            return true;
        }
        return false;
    }
    
    bool RobotMgr::set_robot_config(int robot_id, const RobotConfig& config)
    {
        robot_configs_[robot_id] = config;
        return true;
    }
    
    const std::map<int, RobotConfig>& RobotMgr::get_all_robot_configs() const
    {
        return robot_configs_;
    }
    
    void RobotMgr::set_all_robot_configs(const std::map<int, RobotConfig>& configs)
    {
        robot_configs_ = configs;
    }
    
    bool RobotMgr::get_robot_enabled(int robot_id, bool& enabled) const
    {
        auto it = robot_enables_.find(robot_id);
        if (it != robot_enables_.end()) {
            enabled = it->second;
            return true;
        }
        return false;
    }
    
    bool RobotMgr::set_robot_enabled(int robot_id, bool enabled)
    {
        robot_enables_[robot_id] = enabled;
        return true;
    }
    
    const std::map<int, bool>& RobotMgr::get_all_robot_enables() const
    {
        return robot_enables_;
    }
    
    void RobotMgr::set_all_robot_enables(const std::map<int, bool>& enables)
    {
        robot_enables_ = enables;
    }
    
    bool RobotMgr::init_default_param()
    {
        // 初始化默认配置 - 机器人0
        RobotConfig default_robot_config;
        default_robot_config.ip = "192.168.5.1";
        default_robot_config.user_name = "robot_arm_0";
        
        // 初始化三组机器人移动范围参数，对应工具坐标系ID 0, 1, 2
        RobotCtrlRange range0 = {-400.00, 275.00, -500.00, -165.00, 240.00, 525.00};
        default_robot_config.ranges[0] = range0;
        RobotCtrlRange range1 = {-390.00, 270.00, -510.00, -195.00, 44.00, 350.00};
        default_robot_config.ranges[1] = range1;
        RobotCtrlRange range2 = {-390.00, 270.00, -510.00, -195.00, 44.00, 350.00};
        default_robot_config.ranges[2] = range2;
        
        robot_configs_[1] = default_robot_config;
        robot_enables_[1] = true;
        
        // 初始化默认夹爪配置 - 机器人0
        GripperConfig default_gripper_config;
        default_gripper_config.serial_port = "/dev/ttyUSB0";  // 默认串口路径
        default_gripper_config.id = 21;
        default_gripper_config.user_name = "gripper_0";  // 默认用户名
        default_gripper_config.baudrate = 1000000;  // 默认波特率
        default_gripper_config.enable = true;  // 默认启用夹爪
        // 初始化夹爪参数（只有一组）
        ServoCtrlRange servo_range = {2000, 3000, 0, 4096, 0, 100};
        default_gripper_config.servo_ranges = servo_range;
        
        gripper_configs_[1] = default_gripper_config;
        
        // 初始化第二组机器人配置 - 机器人1
        RobotConfig second_robot_config;
        second_robot_config.ip = "192.168.5.2";
        second_robot_config.user_name = "robot_arm_1";
        
        // 初始化三组机器人移动范围参数，对应工具坐标系ID 0, 1, 2
        RobotCtrlRange range0_second = {-400.00, 275.00, -500.00, -165.00, 240.00, 525.00};
        second_robot_config.ranges[0] = range0_second;
        RobotCtrlRange range1_second = {-390.00, 270.00, -510.00, -195.00, 44.00, 350.00};
        second_robot_config.ranges[1] = range1_second;
        RobotCtrlRange range2_second = {-390.00, 270.00, -510.00, -195.00, 44.00, 350.00};
        second_robot_config.ranges[2] = range2_second;
        
        robot_configs_[2] = second_robot_config;
        robot_enables_[2] = true;
        
        // 初始化第二组夹爪配置 - 机器人1
        GripperConfig second_gripper_config;
        second_gripper_config.serial_port = "/dev/ttyUSB1";  // 串口路径
        second_gripper_config.id = 22;  // 不同的ID
        second_gripper_config.user_name = "gripper_1";  // 默认用户名
        second_gripper_config.baudrate = 1000000;  // 默认波特率
        second_gripper_config.enable = true;  // 启用夹爪
        // 初始化夹爪参数（只有一组）
        ServoCtrlRange servo_range_second = {2000, 3000, 0, 4096, 0, 100};
        second_gripper_config.servo_ranges = servo_range_second;
        
        gripper_configs_[2] = second_gripper_config;
        
        return true;
    }
    
    std::string RobotMgr::get_arm_config_path(int robot_id) const
    {
        std::string install_path = ament_index_cpp::get_package_share_directory("nova_robot_ctrl")+"/../../../";
        std::string config_path = install_path + "/bas_config_data/robot_config/arm_{}/arm_{}_config.yaml";
        
        // 替换第一个{}占位符
        size_t pos1 = config_path.find("{}");
        if (pos1 != std::string::npos) {
            config_path.replace(pos1, 2, std::to_string(robot_id));
        }
        
        // 替换第二个{}占位符
        size_t pos2 = config_path.find("{}", pos1);
        if (pos2 != std::string::npos) {
            config_path.replace(pos2, 2, std::to_string(robot_id));
        }
        
        return config_path;
    }
}