#include "hand_eye_calib_ros/calib_test_node.hpp"
#include <memory>
#include <thread>
#include <chrono>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <atomic>

// 全局变量用于存储原始终端设置
static struct termios g_original_settings;
static bool g_terminal_settings_saved = false;
static std::atomic<bool> g_running_flag{true};

// 信号处理函数，确保终端设置被正确恢复
void signal_handler(int signal) {
    std::cout << "[SIGNAL] 收到信号: " << signal << std::endl;
    g_running_flag = false;
    
    if (g_terminal_settings_saved) {
        // 恢复终端设置
        tcsetattr(0, TCSANOW, &g_original_settings);
        // 清空输入缓冲区
        tcflush(0, TCIFLUSH);
    }
    // 退出程序
    exit(signal);
}

int main(int argc, char * argv[]) 
{
    // 注册信号处理函数，处理Ctrl+C等信号
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 保存原始终端设置
    tcgetattr(0, &g_original_settings);
    g_terminal_settings_saved = true;
    
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<handeyecalib_ros::CalibTestNode>();
    
    // 使用单独的线程运行ROS事件循环
    bool ros_running = true;
    std::thread ros_thread([&node, &ros_running]() {
        rclcpp::spin(node);
        ros_running = false;
    });
    
    // 在主线程中处理键盘输入（模仿nova_robot_ctrl的方式）
    node->runKeyboardLoop();
    
    // 停止ROS事件循环
    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }
    
    // 恢复终端设置
    if (g_terminal_settings_saved) {
        tcsetattr(0, TCSANOW, &g_original_settings);
        tcflush(0, TCIFLUSH);
    }
    
    return 0;
}