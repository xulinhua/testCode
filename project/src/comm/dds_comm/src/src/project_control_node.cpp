/**
 * @file project_control_node.cpp
 * @brief 项目控制节点 - 使用log_system进行日志记录
 * 
 * 功能概述：
 * 1. 提供项目生命周期管理功能
 * 2. 使用log_system进行所有日志记录
 * 3. 支持项目启动、停止、重启等操作
 */

#include "../include/dds_comm/project_manager.hpp"
#include "../include/dds_comm/command_manager.hpp"
#include "log_system/log_macros.hpp"
#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <csignal>

using namespace dds_comm;

// 定义当前项目类型
constexpr ProjectType CURRENT_PROJECT = ProjectType::DDS_COMMUNICATION;
const std::string CURRENT_PROJECT_NAME = getProjectLogName(CURRENT_PROJECT);

/**
 * @brief 信号处理函数
 */
void signalHandler(int signum) {
    LOG_INFO("接收到终止信号: %d", signum);
}

/**
 * @brief 打印使用帮助
 */
void printHelp() {
    std::cout << "项目控制节点使用说明:" << std::endl;
    std::cout << "  --help             显示此帮助信息" << std::endl;
    std::cout << "  --start <项目类型>  启动指定项目" << std::endl;
    std::cout << "  --stop <项目类型>   停止指定项目" << std::endl;
    std::cout << "  --restart <项目类型> 重启指定项目" << std::endl;
    std::cout << "  --status <项目类型> 查看项目状态" << std::endl;
    std::cout << "  --list             列出所有项目" << std::endl;
    std::cout << std::endl;
    std::cout << "可用项目类型:" << std::endl;
    std::cout << "  dds_comm - DDS通信项目" << std::endl;
    std::cout << "  hand_eye_calibration - 手眼标定项目" << std::endl;
    std::cout << "  camera_calibration - 相机标定项目" << std::endl;
    std::cout << "  point_cloud_processing - 点云处理项目" << std::endl;
}

/**
 * @brief 项目启动处理器
 */
bool startProjectHandler() {
    LOG_INFO("开始启动项目...");
    
    // 模拟项目启动过程
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    LOG_INFO("项目启动成功");
    return true;
}

/**
 * @brief 项目停止处理器
 */
bool stopProjectHandler() {
    LOG_INFO("开始停止项目...");
    
    // 模拟项目停止过程
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    LOG_INFO("项目停止成功");
    return true;
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    LOG_INFO("项目控制节点启动");
    
    // 解析命令行参数
    if (argc < 2) {
        LOG_ERROR("参数不足，使用 --help 查看使用说明");
        printHelp();
        return 1;
    }
    
    std::string command = argv[1];
    
    if (command == "--help") {
        printHelp();
        return 0;
    }
    
    // 创建项目管理器
    auto project_manager = std::make_shared<ProjectManager>();
    
    // 创建命令管理器
    auto command_manager = std::make_shared<CommandManager>(CURRENT_PROJECT);
    
    // 设置命令处理器
    command_manager->setStartHandler(startProjectHandler);
    command_manager->setStopHandler(stopProjectHandler);
    
    // 注册当前项目
    if (!project_manager->registerProject(CURRENT_PROJECT, CURRENT_PROJECT_NAME, true, 3)) {
        LOG_ERROR("项目注册失败: %s", CURRENT_PROJECT_NAME.c_str());
        return 1;
    }
    
    LOG_INFO("项目注册成功: %s", CURRENT_PROJECT_NAME.c_str());
    
    // 处理不同命令
    if (command == "--start") {
        if (argc < 3) {
            LOG_ERROR("缺少项目类型参数");
            return 1;
        }
        
        std::string project_type = argv[2];
        LOG_INFO("启动项目: %s", project_type.c_str());
        
        // 执行启动命令
        CommandMessage cmd;
        cmd.project_type = CURRENT_PROJECT;
        cmd.command_type = CommandType::START;
        
        if (command_manager->executeCommand(cmd)) {
            LOG_INFO("项目启动成功: %s", project_type.c_str());
        } else {
            LOG_ERROR("项目启动失败: %s", project_type.c_str());
            return 1;
        }
        
    } else if (command == "--stop") {
        if (argc < 3) {
            LOG_ERROR("缺少项目类型参数");
            return 1;
        }
        
        std::string project_type = argv[2];
        LOG_INFO("停止项目: %s", project_type.c_str());
        
        // 执行停止命令
        CommandMessage cmd;
        cmd.project_type = CURRENT_PROJECT;
        cmd.command_type = CommandType::STOP;
        
        if (command_manager->executeCommand(cmd)) {
            LOG_INFO("项目停止成功: %s", project_type.c_str());
        } else {
            LOG_ERROR("项目停止失败: %s", project_type.c_str());
            return 1;
        }
        
    } else if (command == "--restart") {
        if (argc < 3) {
            LOG_ERROR("缺少项目类型参数");
            return 1;
        }
        
        std::string project_type = argv[2];
        LOG_INFO("重启项目: %s", project_type.c_str());
        
        // 执行重启命令（先停止再启动）
        CommandMessage stop_cmd;
        stop_cmd.project_type = CURRENT_PROJECT;
        stop_cmd.command_type = CommandType::STOP;
        
        CommandMessage start_cmd;
        start_cmd.project_type = CURRENT_PROJECT;
        start_cmd.command_type = CommandType::START;
        
        if (command_manager->executeCommand(stop_cmd) && 
            command_manager->executeCommand(start_cmd)) {
            LOG_INFO("项目重启成功: %s", project_type.c_str());
        } else {
            LOG_ERROR("项目重启失败: %s", project_type.c_str());
            return 1;
        }
        
    } else if (command == "--status") {
        if (argc < 3) {
            LOG_ERROR("缺少项目类型参数");
            return 1;
        }
        
        std::string project_type = argv[2];
        LOG_INFO("查看项目状态: %s", project_type.c_str());
        
        // 执行状态查询命令
        CommandMessage cmd;
        cmd.project_type = CURRENT_PROJECT;
        cmd.command_type = CommandType::STATUS;
        
        if (command_manager->executeCommand(cmd)) {
            auto project_info = project_manager->getProjectInfo(CURRENT_PROJECT);
            LOG_INFO("项目状态: %s", project_info.project_name.c_str());
        } else {
            LOG_ERROR("状态查询失败: %s", project_type.c_str());
            return 1;
        }
        
    } else if (command == "--list") {
        LOG_INFO("列出所有项目");
        
        auto all_projects = project_manager->getAllProjectInfo();
        for (const auto& project : all_projects) {
            LOG_INFO("项目: %s", project.project_name.c_str());
        }
        
    } else {
        LOG_ERROR("未知命令: %s", command.c_str());
        printHelp();
        return 1;
    }
    
    LOG_INFO("项目控制节点正常退出");
    return 0;
}