/**
 * @file main.cpp
 * @brief DDS通信服务主程序入口文件
 * 
 * @section 功能作用
 * 本文件是DDS通信项目的核心入口点，负责：
 * 1. 初始化ROS2节点和DDS通信服务
 * 2. 解析命令行参数（服务端/客户端模式、配置文件路径）
 * 3. 注册信号处理函数，实现优雅关闭
 * 4. 创建并管理DDS服务实例
 * 5. 注册回调函数处理命令、状态和错误信息
 * 6. 在服务端模式下注册项目生命周期管理
 * 
 * @section 项目耦合关系
 * - 强依赖：dds_service.hpp, command_types.hpp - 核心服务接口和命令类型定义
 * - 弱依赖：ROS2框架 - 用于节点管理和消息通信
 * - 无依赖：其他项目模块 - 完全独立于手眼标定项目
 * - 耦合度：低 - 通过接口抽象实现松耦合设计
 * 
 * @section 使用说明
 * 服务端模式：./dds_service --server
 * 客户端模式：./dds_service --client
 * 指定配置：./dds_service --config /path/to/config.yaml
 */

#include <memory>
#include <signal.h>
#include <iostream>
#include <thread>
#include <string>
#include <chrono>
#include "../include/dds_comm/dds_service.hpp"
#include "dds_comm/command_types.hpp"
#include "log_system/log_macros.hpp"

using namespace dds_comm;
/**
 * @brief 信号处理函数
 * @param signum 信号编号
 */
void signalHandler(int signum) {
    LOG_INFO("接收到信号: %d", signum);
}

/**
 * @brief DDS服务主函数
 * @param argc 参数个数
 * @param argv 参数数组
 * @return int 退出码
 */
int main(int argc, char** argv) {
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // 解析命令行参数
        bool is_server = true;
        std::string config_file = "";
        
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--client") {
                is_server = false;
            } else if (arg == "--config" && i + 1 < argc) {
                config_file = argv[++i];
            }
        }
        
        LOG_INFO("启动DDS服务，模式: %s", is_server ? "服务端" : "客户端");
        
        if (!config_file.empty()) {
            LOG_INFO("使用配置文件: %s", config_file.c_str());
        }
        
        // 创建DDS服务
        auto dds_service = std::make_shared<DdsService>(is_server, config_file);
        
        // 初始化服务
        if (!dds_service->initialize()) {
            LOG_ERROR("DDS服务初始化失败");
            return -1;
        }
        
        LOG_INFO("DDS服务启动成功");
        
        // 注册回调函数示例
        dds_service->registerCommandCallback([](const CommandMessage& command) {
            LOG_INFO("接收到命令: 类型=%d, 项目=%d", 
                        command.command_type, command.project_type);
        });
        
        dds_service->registerStatusCallback([](const StatusMessage& status) {
            LOG_INFO("接收到状态: 项目=%d, 状态=%s", 
                        status.project_type, status.status.c_str());
        });
        
        dds_service->registerErrorCallback([](const std::string& error_msg) {
            LOG_ERROR("服务错误: %s", error_msg.c_str());
        });
        
        // 服务端模式下注册示例项目
        if (is_server) {
            // 示例：注册手眼标定项目
            dds_service->registerProjectLifecycle(
                ProjectType::HAND_EYE_CALIB,
                []() -> bool {
                    LOG_INFO("hand_eye_calib", "手眼标定项目启动...");
                    // 这里添加实际的启动逻辑
                    return true;
                },
                []() -> bool {
                    LOG_INFO("hand_eye_calib", "手眼标定项目停止...");
                    // 这里添加实际的停止逻辑
                    return true;
                },
                []() -> StatusMessage {
                    StatusMessage status;
                    status.project_type = ProjectType::HAND_EYE_CALIB;
                    status.status = "手眼标定项目运行中";
                    status.is_running = true;
                    status.uptime = 100;
                    status.last_error = "";
                    status.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
                    return status;
                }
            );
            
            LOG_INFO("示例项目注册完成");
        }
        
        // 运行服务（这里需要根据实际架构调整）
        // 例如：使用事件循环或线程池
        LOG_INFO("DDS服务运行中，按Ctrl+C退出...");
        
        // 简单的等待循环
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        // 关闭服务
        dds_service->shutdown();
        
        LOG_INFO("DDS服务已正常关闭");
        
    } catch (const std::exception& e) {
        LOG_ERROR("DDS服务异常: %s", e.what());
        return -1;
    }
    return 0;
}