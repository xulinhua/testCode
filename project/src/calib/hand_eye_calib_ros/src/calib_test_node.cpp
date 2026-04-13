#include "hand_eye_calib_ros/calib_test_node.hpp"
#include "hand_eye_calib_ros/keyboard_handler.hpp"
#include "log_system/log_macros.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <cmath>

// 全局变量用于存储原始终端设置
static struct termios g_original_settings;
static bool g_terminal_settings_saved = false;

// 信号处理函数，确保终端设置被正确恢复
void signal_handler(int signal) {
    if (g_terminal_settings_saved) {
        // 恢复终端设置
        tcsetattr(0, TCSANOW, &g_original_settings);
        // 清空输入缓冲区
        tcflush(0, TCIFLUSH);
    }
    // 退出程序
    exit(signal);
}

namespace handeyecalib_ros {

/**
 * @brief 构造函数
 * @param options ROS节点选项
 */
CalibTestNode::CalibTestNode(const rclcpp::NodeOptions & options)
: Node("calib_test_node", options),
  robot_connected_(false),
  robot_enabled_(false),
  aruco_detection_received_(false),
  running_(true),
  waiting_for_user_input_(false),
  aruco_target_pose_recorded_(false)
{
    // 注册信号处理函数，处理Ctrl+C等信号
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 保存原始终端设置
    tcgetattr(0, &g_original_settings);
    g_terminal_settings_saved = true;
    
    LOG_INFO("开始 CalibTestNode 构造函数");
    
    // 初始化参数
    initParameters();
    
    // 初始化订阅器
    initSubscribers();
    
    // 初始化发布器
    initPublishers();
    
    // 初始化键盘处理器
    initKeyboardHandler();
    
    LOG_INFO("CalibTestNode 构造函数完成");
}

/**
 * @brief 析构函数
 */
CalibTestNode::~CalibTestNode() 
{
    LOG_INFO("CalibTestNode 析构");
}

/**
 * @brief 运行键盘输入循环
 */
void CalibTestNode::runKeyboardLoop()
{
    LOG_INFO("开始键盘输入循环");
    
    LOG_INFO("测试节点启动完成。按以下键位操作:");
    LOG_INFO("  S: 移动到Aruco检测位置");
    LOG_INFO("  P: 显示当前位置");
    LOG_INFO("  Q: 退出程序");
    
    while (running_) {
        processKeyboardInput();
        // 短暂休眠以避免过度占用CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    LOG_INFO("键盘输入循环结束");
}

/**
 * @brief 初始化参数
 */
void CalibTestNode::initParameters() 
{
    // 声明并获取参数
    this->declare_parameter("robot_pose_topic", "/right_arm_cartesian_pose");
    this->declare_parameter("calib_result_topic", "/aruco_detection/calib_result");
    this->declare_parameter("robot_target_topic", "/robot_target");
    this->declare_parameter("timer_period_ms", 100);
    
    robot_pose_topic_ = this->get_parameter("robot_pose_topic").as_string();
    calib_result_topic_ = this->get_parameter("calib_result_topic").as_string();
    robot_target_topic_ = this->get_parameter("robot_target_topic").as_string();
    timer_period_ms_ = this->get_parameter("timer_period_ms").as_int();
    
    LOG_INFO("参数初始化完成，机械臂位姿话题: %s, 标定结果话题: %s, 机械臂目标话题: %s, 定时器周期: %d ms", 
        robot_pose_topic_.c_str(), calib_result_topic_.c_str(), robot_target_topic_.c_str(), timer_period_ms_);
}

/**
 * @brief 初始化订阅器
 */
void CalibTestNode::initSubscribers() 
{
    // 创建机械臂位姿订阅器
    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        robot_pose_topic_, 10,
        std::bind(&CalibTestNode::robotPoseCallback, this, std::placeholders::_1));
    
    // 创建Aruco标定结果订阅器
    aruco_calib_result_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        calib_result_topic_, 10,
        std::bind(&CalibTestNode::arucoCalibResultCallback, this, std::placeholders::_1));
    
    LOG_INFO("订阅器初始化完成，机械臂位姿话题: %s, 标定结果话题: %s", robot_pose_topic_.c_str(), calib_result_topic_.c_str());
}

/**
 * @brief 初始化发布器
 */
void CalibTestNode::initPublishers() 
{
    // 创建机械臂目标位姿发布器
    robot_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(robot_target_topic_, 10);
    
    LOG_INFO("发布器初始化完成，机械臂目标话题: %s", robot_target_topic_.c_str());
}

/**
 * @brief 初始化键盘处理器
 */
void CalibTestNode::initKeyboardHandler() 
{
    keyboard_handler_ = std::make_unique<KeyboardHandler>();
}

/**
 * @brief 机械臂位姿回调函数
 * @param msg 机械臂位姿消息
 */
void CalibTestNode::robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
    std::lock_guard<std::mutex> lock(robot_pose_mutex_);
    current_robot_pose_ = *msg;
    robot_connected_ = true;
    LOG_DEBUG("收到机械臂位姿: x=%.3f, y=%.3f, z=%.3f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

/**
 * @brief Aruco标定结果回调函数
 * @param msg Aruco标定结果消息
 */
void CalibTestNode::arucoCalibResultCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
    std::lock_guard<std::mutex> lock(aruco_detection_mutex_);
    
    aruco_detection_received_ = true;
    aruco_detection_pose_.position.x = msg->pose.position.x;
    aruco_detection_pose_.position.y = msg->pose.position.y;
    aruco_detection_pose_.position.z = msg->pose.position.z;
    aruco_detection_pose_.orientation = msg->pose.orientation;
    LOG_DEBUG("收到Aruco标定结果: x=%.3f, y=%.3f, z=%.3f", 
        aruco_detection_pose_.position.x, aruco_detection_pose_.position.y, aruco_detection_pose_.position.z);
     aruco_detection_pose_.orientation = current_robot_pose_.pose.orientation;
    // 如果正在等待用户输入，显示提示信息
    if (waiting_for_user_input_) {
        LOG_INFO("检测到Aruco标记位置: x=%.3f, y=%.3f, z=%.3f", 
            aruco_detection_pose_.position.x, aruco_detection_pose_.position.y, aruco_detection_pose_.position.z);
        LOG_INFO("按 S 键移动机械臂到此位置，或按其他键跳过");
    }
}

/**
 * @brief 读取键盘输入
 * @return 按键字符，如果没有按键则返回-1
 */
int CalibTestNode::readKeyboardInput() 
{
    if (keyboard_handler_) {
        // 首先尝试读取第一个字节
        int key = keyboard_handler_->readOne();
        if (key != -1) {
            // 检查是否是转义序列的开始（通常是方向键）
            if (key == 27) { // ESC键
                // 尝试读取接下来的两个字节
                int key2 = keyboard_handler_->readOne();
                if (key2 != -1) {
                    int key3 = keyboard_handler_->readOne();
                    if (key3 != -1) {
                        // 如果是方向键序列：27 91 65-68
                        if (key2 == 91 && key3 >= 65 && key3 <= 68) {
                            return key3; // 返回方向键代码
                        }
                    }
                }
            }
            return key;
        }
    }
    return -1;
}

/**
 * @brief 处理键盘输入
 */
void CalibTestNode::processKeyboardInput() 
{
    int key = readKeyboardInput();
    LOG_DEBUG("readKeyboardInput返回值: %d", key);
    if (key != -1) {
        LOG_INFO("检测到按键: %c (%d)", (char)key, key);
        switch (key) {
            case 65: // 上箭头
                LOG_INFO("上箭头 - 向前移动");
                // 这里添加实际的移动逻辑
                break;
            case 66: // 下箭头
                LOG_INFO("下箭头 - 向后移动");
                // 这里添加实际的移动逻辑
                break;
            case 67: // 右箭头
                LOG_INFO("右箭头 - 向右移动");
                // 这里添加实际的移动逻辑
                break;
            case 68: // 左箭头
                LOG_INFO("左箭头 - 向左移动");
                // 这里添加实际的移动逻辑
                break;
            case 'i':
            case 'I':
                LOG_INFO("I键 - 向上移动");
                // 这里添加实际的移动逻辑
                break;
            case 'k':
            case 'K':
                LOG_INFO("K键 - 向下移动");
                // 这里添加实际的移动逻辑
                break;
            case 's':
            case 'S':
                LOG_INFO("S键 - 移动到Aruco检测位置");
                moveToArucoPosition();
                break;
            case 'p':
            case 'P':
                LOG_INFO("P键 - 显示当前位置");
                {
                    std::lock_guard<std::mutex> lock(robot_pose_mutex_);
                    if (robot_connected_) {
                        LOG_INFO("当前机械臂位置: x=%.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f", 
                            current_robot_pose_.pose.position.x, current_robot_pose_.pose.position.y, current_robot_pose_.pose.position.z,
                            current_robot_pose_.pose.orientation.x, current_robot_pose_.pose.orientation.y, current_robot_pose_.pose.orientation.z);
                    } else {
                        LOG_WARN("机械臂未连接");
                    }
                }
                break;
            case 'q':
            case 'Q':
                LOG_INFO("Q键 - 退出程序");
                running_ = false;
                rclcpp::shutdown();
                break;
            default:
                LOG_DEBUG("未知按键: %c (%d)", (char)key, key);
                break;
        }
        // 重置等待用户输入标志
        waiting_for_user_input_ = false;
    } else {
        LOG_DEBUG("未检测到按键输入");
    }
}

/**
 * @brief 移动到Aruco检测位置
 */
void CalibTestNode::moveToArucoPosition() 
{
    // 检查是否收到Aruco检测结果
    {
        std::lock_guard<std::mutex> lock(aruco_detection_mutex_);
        if (!aruco_detection_received_) {
            LOG_WARN("尚未收到Aruco检测结果");
            return;
        }
    }
    
    // 记录移动前的位置
    {
        std::lock_guard<std::mutex> lock(robot_pose_mutex_);
        before_move_pose_ = current_robot_pose_;
    }
    
    // 记录Aruco检测位置作为目标位置
    geometry_msgs::msg::Pose aruco_pose;
    {
        std::lock_guard<std::mutex> lock(aruco_detection_mutex_);
        aruco_pose = aruco_detection_pose_;
    }
    
    LOG_INFO("准备移动到Aruco位置: x=%.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f", 
        aruco_pose.position.x, aruco_pose.position.y, aruco_pose.position.z, aruco_pose.orientation.x, aruco_pose.orientation.y, aruco_pose.orientation.z);
    
    // 发布目标位置到机械臂控制节点
    auto target_msg = geometry_msgs::msg::PoseStamped();
    target_msg.header.stamp = this->now(); 
    target_msg.header.frame_id = "base_link";
    target_msg.pose = aruco_pose;
    robot_target_pub_->publish(target_msg);
    
    LOG_INFO("已发布目标位置到机械臂控制节点");
    
    // 等待一段时间模拟机械臂移动
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 记录移动后的位置
    {
        std::lock_guard<std::mutex> lock(robot_pose_mutex_);
        after_move_pose_ = current_robot_pose_;
    }
    
    // 输出标定精度偏差
    printCalibrationDeviation();
}

/**
 * @brief 输出标定精度偏差
 */
void CalibTestNode::printCalibrationDeviation() 
{
    // 计算偏差
    double x_diff = after_move_pose_.pose.position.x - aruco_detection_pose_.position.x;
    double y_diff = after_move_pose_.pose.position.y - aruco_detection_pose_.position.y;
    double z_diff = after_move_pose_.pose.position.z - aruco_detection_pose_.position.z;
    
    LOG_INFO("标定精度偏差计算结果:");
    LOG_INFO("  移动前位置: x=%.6f, y=%.6f, z=%.6f", 
        before_move_pose_.pose.position.x, before_move_pose_.pose.position.y, before_move_pose_.pose.position.z);
    LOG_INFO("  Aruco目标位置: x=%.6f, y=%.6f, z=%.6f", 
        aruco_detection_pose_.position.x, aruco_detection_pose_.position.y, aruco_detection_pose_.position.z);
    LOG_INFO("  移动后位置: x=%.6f, y=%.6f, z=%.6f", 
        after_move_pose_.pose.position.x, after_move_pose_.pose.position.y, after_move_pose_.pose.position.z);
    LOG_INFO("  X轴偏差: %.6f m", x_diff);
    LOG_INFO("  Y轴偏差: %.6f m", y_diff);
    LOG_INFO("  Z轴偏差: %.6f m", z_diff);
    LOG_INFO("  总体偏差: %.6f m", std::sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff));
}

}  // namespace handeyecalib_ros