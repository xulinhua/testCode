#include "nova_robot_ctrl_ros/nova_robot_ctrl_node.hpp"
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>
#include "custom_msgs_comm/msg/robot_std_pose.hpp"
#include "bas_sys_config_ros/sys_info_server.h"
#include "bas_operate_ros/param_utils.hpp"

// 添加系统配置相关头文件
#include "bas_sys_config/sys_config_struct.hpp"
#include "bas_sys_config_ros/sys_info_server.h"

namespace nova_robot_ctrl_ros {

    constexpr int ROBOT_LOAD_FROM_FILE = 0;             // 从文件加载机器人配置
    constexpr int ROBOT_LOAD_FROM_PARAM_SERVER = 1;     // 从参数服务器加载机器人配置

/**
 * @brief 构造函数
 * @param options ROS节点选项
 */
NovaRobotCtrlNode::NovaRobotCtrlNode(const rclcpp::NodeOptions & options)
: Node("nova_robot_ctrl_node", options),
  robot_connected_(false),
  robot_enabled_(false),
  robot_moving_(false),
  arm_id_(1),
  gripper_position_(0),
  gripper_speed_(50),
  gripper_force_(0),
  gripper_abs_position_(2000),
  current_x_(0.0),
  current_y_(0.0),
  current_z_(0.0),
  step_size_(1.0)  // 默认步长1mm
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 开始 NovaRobotCtrlNode 构造函数" << std::endl;
    
    // 初始化参数
    initParameters();
    
    // 初始化订阅器
    initSubscribers();
    
    // 初始化发布器
    initPublishers();
    
    // 初始化服务服务器
    initServiceServers();
    
    // 初始化定时器
    initTimer();
    
    // 初始化夹爪控制
    initGripperControl();
    
    // 配置加载
    load_config(ROBOT_LOAD_FROM_PARAM_SERVER);

    // 启用RobotMgr系统
    if (!robot_mgr_.init())
    {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - RobotMgr系统初始化失败" << std::endl;
    }
    else
    {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - RobotMgr系统初始化成功" << std::endl;
        robot_connected_ = true;
        robot_enabled_ = true;
        
        // 设置默认的用户坐标系和工具坐标系
        // 这里可以根据需要调整坐标系ID
        int user_coordinate_id = 0;
        int tool_coordinate_id = 0;
        // 获取所有已启用的机器人ID，并为每个机器人设置坐标系
        std::vector<int> enable_robot_ids = robot_mgr_.get_all_enable_robots_id();
        if (!enable_robot_ids.empty()) {
            // 如果存在多个启用的机器人，为每个机器人设置坐标系
            for (int robot_id : enable_robot_ids) {
                robot_mgr_.set_user_coordinate_robot(user_coordinate_id, robot_id);
                robot_mgr_.set_tool_coordinate_robot(tool_coordinate_id, robot_id);
                std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 为机器人ID " << robot_id << " 设置坐标系: 用户坐标系=" << user_coordinate_id << ", 工具坐标系=" << tool_coordinate_id << std::endl;
            }
        } else {
            // 如果没有启用的机器人ID列表（向后兼容），则使用当前的arm_id_
            robot_mgr_.set_user_coordinate_robot(user_coordinate_id, arm_id_);
            robot_mgr_.set_tool_coordinate_robot(tool_coordinate_id, arm_id_);
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 使用默认arm_id设置坐标系: 用户坐标系=" << user_coordinate_id << ", 工具坐标系=" << tool_coordinate_id << ", arm_id=" << arm_id_ << std::endl;
        }
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 设置坐标系: 用户坐标系=" << user_coordinate_id << ", 工具坐标系=" << tool_coordinate_id << std::endl;
        
        // 获取机械臂当前位置
        geometry_msgs::msg::PoseStamped current_pose;
        if (getCurrentPose(current_pose, arm_id_)) {
            current_x_ = current_pose.pose.position.x;
            current_y_ = current_pose.pose.position.y;
            current_z_ = current_pose.pose.position.z;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂当前位置: x=" << std::fixed << std::setprecision(3) << current_x_ << ", y=" << current_y_ << ", z=" << current_z_ << std::endl;
        } else {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法获取机械臂当前位置" << std::endl;
        }
    }
    
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - NovaRobotCtrlNode 初始化完成" << std::endl;
    
    // 打印按键操作说明
    print_key_operate();
    
    // 添加初始化完成后服务状态检查
    // if (get_calib_points_service_server_) {
    //     std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 标定点数据服务服务器已就绪" << std::endl;
    // } else {
    //     std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 标定点数据服务服务器未就绪" << std::endl;
    // }
}

/**
 * @brief 析构函数
 */
NovaRobotCtrlNode::~NovaRobotCtrlNode() 
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - NovaRobotCtrlNode 析构" << std::endl;
}

/**
 * @brief 初始化参数
 */
void NovaRobotCtrlNode::initParameters() 
{
    // 声明并获取参数
    this->declare_parameter("robot_ip", "192.168.5.1");
    robot_ip_ = this->get_parameter("robot_ip").as_string();
    
    this->declare_parameter("robot_id", 0);
    arm_id_ = this->get_parameter("robot_id").as_int();
    
    this->declare_parameter("robot_num", 1);  // 默认只有一个机械臂
    

    this->declare_parameter("robot_target_pos_topic", "/robot_target_pose");
    this->declare_parameter("robot_pose_topic", "/right_arm_cartesian_pose");
    this->declare_parameter("robot_run_state_topic", "/robot_run_state");
    this->declare_parameter("robot_ready_state_topic", "/robot_ready_state");  // 添加机械臂准备状态话题参数
    this->declare_parameter("robot_control_service", "/robot_control");
    this->declare_parameter("get_calib_points_service", "/get_calibration_points");
    this->declare_parameter("get_robot_pose_service", "/get_robot_pose");  // 添加获取机械臂位姿服务参数
    this->declare_parameter("timer_period_ms", 100);
}

/**
 * @brief 初始化订阅器
 */
void NovaRobotCtrlNode::initSubscribers()
{
    // 获取参数
    std::string robot_target_pos_topic = this->get_parameter("robot_target_pos_topic").as_string();

    // 创建订阅器
    robot_target_sub_ = this->create_subscription<custom_msgs_comm::msg::RobotStdPose>(
        robot_target_pos_topic, 10,
        std::bind(&NovaRobotCtrlNode::robotTargetPoseCallback, this, std::placeholders::_1));

    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 订阅器初始化完成，订阅主题: " << robot_target_pos_topic << std::endl;
}

/**
 * @brief 初始化发布器
 */
void NovaRobotCtrlNode::initPublishers() 
{
    // 获取参数
    std::string robot_run_state_topic = this->get_parameter("robot_run_state_topic").as_string();
    std::string robot_ready_state_topic = this->get_parameter("robot_ready_state_topic").as_string();  // 获取机械臂准备状态话题参数
    
    // 从robot_mgr_获取机械臂数量
    int robot_num = robot_mgr_.get_all_enable_robots_id().size();
    
    // 获取启用的机械臂ID列表
    std::vector<int> enabled_arm_ids = robot_mgr_.get_all_enable_robots_id();
    
    // 为每个启用的机械臂创建位姿发布器
    basros::RosCommInfo comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_ARM_CURRENT_POSE, 0, 0);
    std::string robot_pose_topic = comm_info.name;   
    // 创建发布器
    robot_pose_pub_ = this->create_publisher<custom_msgs_comm::msg::RobotStdPose>(robot_pose_topic, 10);
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << " 创建位姿发布器，话题名称: " << robot_pose_topic << std::endl;
    
    // 保存机械臂数量和ID列表
    robot_num_ = enabled_arm_ids.size();
    arm_id_list_ = enabled_arm_ids;
    
    // 创建共享的状态发布器
    robot_status_pub_ = this->create_publisher<custom_msgs_comm::msg::BoolStamped>(robot_run_state_topic, 10);
    robot_ready_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(robot_ready_state_topic, 10);  // 创建机械臂准备状态发布器
    
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 发布器初始化完成，状态发布主题: " << robot_run_state_topic << ", " << robot_ready_state_topic << std::endl;
}

/**
 * @brief 初始化服务服务器
 */
void NovaRobotCtrlNode::initServiceServers() 
{
    // 获取参数
    std::string robot_control_service = this->get_parameter("robot_control_service").as_string();
    std::string get_calib_points_service = this->get_parameter("get_calib_points_service").as_string();
    std::string get_robot_pose_service = this->get_parameter("get_robot_pose_service").as_string();

    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 准备创建服务服务器，机械臂控制服务名称: " << robot_control_service << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 准备创建服务服务器，获取标定点数据服务名称: " << get_calib_points_service << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 准备创建服务服务器，获取机械臂位姿服务名称: " << get_robot_pose_service << std::endl;

    // 创建服务服务器
    robot_control_service_server_ = this->create_service<std_srvs::srv::Trigger>(
        robot_control_service,
        std::bind(&NovaRobotCtrlNode::robotControlServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 创建获取标定点数据服务服务器
    // get_calib_points_service_server_ = this->create_service<custom_msgs_comm::srv::GetCalibrationPoints>(
    //     get_calib_points_service,
    //     std::bind(&NovaRobotCtrlNode::getCalibrationPointsServiceCallback, this,
    //               std::placeholders::_1, std::placeholders::_2));
    
    // 创建获取机械臂位姿服务服务器
    get_robot_pose_service_server_ = this->create_service<custom_msgs_comm::srv::GetRobotPose>(
        get_robot_pose_service,
        std::bind(&NovaRobotCtrlNode::getRobotPoseServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 创建键盘控制服务服务器
    keyboard_control_service_server_ = this->create_service<custom_msgs_comm::srv::KeyboardControl>(
        "/keyboard_control",
        std::bind(&NovaRobotCtrlNode::keyboardControlServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 创建移动到指定位姿服务服务器
    move_to_pose_service_server_ = this->create_service<custom_msgs_comm::srv::MoveToPose>(
        "/move_to_pose",
        std::bind(&NovaRobotCtrlNode::moveToPoseServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
            
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 服务服务器初始化完成，服务名称: " << robot_control_service << ", " << get_calib_points_service << ", " << get_robot_pose_service << ", /keyboard_control, /move_to_pose" << std::endl;
    
    // 添加服务可用性检查
    // if (get_calib_points_service_server_) {
    //     std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 标定点数据服务服务器创建成功" << std::endl;
    // } else {
    //     std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 标定点数据服务服务器创建失败" << std::endl;
    // }
    
    if (get_robot_pose_service_server_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂位姿服务服务器创建成功" << std::endl;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂位姿服务服务器创建失败" << std::endl;
    }
}

/**
 * @brief 初始化定时器
 */
void NovaRobotCtrlNode::initTimer() 
{
    // 获取参数
    int timer_period_ms = this->get_parameter("timer_period_ms").as_int();
    
    // 创建定时器
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        std::bind(&NovaRobotCtrlNode::timerCallback, this));
    
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 定时器初始化完成，周期: " << timer_period_ms << " ms" << std::endl;
    
    // 不再需要定时器加载参数服务器配置，因为在构造函数中已经直接调用了
}

/**
 * @brief 连接机械臂
 * @return 是否连接成功
 */
bool NovaRobotCtrlNode::connectRobot() 
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 尝试连接机械臂，IP地址: " << robot_ip_ << std::endl;
    
    // 不再直接连接机器人，而是通过RobotMgr来处理
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机器人连接由RobotMgr处理" << std::endl;
    
    return true;
}

/**
 * @brief 使能机械臂
 * @return 是否使能成功
 */
bool NovaRobotCtrlNode::enableRobot() 
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 尝试使能机械臂" << std::endl;
    
    // 不再直接使能机器人，而是通过RobotMgr来处理
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机器人使能由RobotMgr处理" << std::endl;
    
    return true;
}

/**
 * @brief 获取机械臂当前位姿
 * @param pose 输出的位姿信息
 * @return 是否获取成功
 */
bool NovaRobotCtrlNode::getCurrentPose(geometry_msgs::msg::PoseStamped& pose, int arm_id) 
{
    // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 获取机械臂当前位姿" << std::endl;
    
    // 获取Nova机械臂的位姿，通过RobotMgr来访问已连接的机器人
    nova_robot_ctrl::Pose nova_pose;
    bool result = robot_mgr_.get_current_pose_robot(nova_pose, arm_id);
    
    if (result) 
    {
        // 转换为ROS消息格式
        pose.header.stamp = this->now();
        //pose.header.frame_id = "base_link";
        pose.header.frame_id = std::to_string(arm_id); // 临时处理，使用机械臂ID作为frame_id

        pose.pose.position.x = nova_pose.x;  
        pose.pose.position.y = nova_pose.y; 
        pose.pose.position.z = nova_pose.z; 
        
        // 这里简化处理，实际应用中应该使用正确的欧拉角到四元数转换
        pose.pose.orientation.x = nova_pose.rx;
        pose.pose.orientation.y = nova_pose.ry;
        pose.pose.orientation.z = nova_pose.rz;
        pose.pose.orientation.w = 0.0;
        
        // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 当前位姿: x=" << std::fixed << std::setprecision(3) << pose.pose.position.x << ", y=" << pose.pose.position.y << ", z=" << pose.pose.position.z << ", rx=" << nova_pose.rx << ", ry=" << nova_pose.ry << ", rz=" << nova_pose.rz << std::endl;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() arm_ID" << arm_id << " - 获取机械臂位姿失败" << std::endl;
    }
    
    return result;
}

/**
 * @brief 移动机械臂到指定位置
 * @param target_pose 目标位姿
 * @param speed 运动速度(%)，默认为30(较慢的速度用于手眼标定)
 * @return 是否移动成功
 */
bool NovaRobotCtrlNode::moveRobotToPose(const geometry_msgs::msg::PoseStamped& target_pose, int speed /* = 30 */) 
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 移动机械臂到指定位置: x=" << std::fixed << std::setprecision(3) << target_pose.pose.position.x << ", y=" << target_pose.pose.position.y << ", z=" << target_pose.pose.position.z << ", rx=" << target_pose.pose.orientation.x << ", ry=" << target_pose.pose.orientation.y << ", rz=" << target_pose.pose.orientation.z << ", 速度=" << speed << "%%" << std::endl;
    
    // 设置机械臂正在移动状态
    robot_moving_ = true;

    // 调用Nova机械臂的直线运动接口，带速度参数，通过RobotMgr来访问已连接的机器人
    bool result = robot_mgr_.move_l_robot(
        target_pose.pose.position.x,   
        target_pose.pose.position.y,   
        target_pose.pose.position.z,
        target_pose.pose.orientation.x,         // rx
        target_pose.pose.orientation.y,         // ry
        target_pose.pose.orientation.z,         // rz
        speed,                                  // 速度参数
        arm_id_                               // 机器人ID
    );
    
    if (result) 
    {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂移动命令发送成功" << std::endl;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂移动命令发送失败" << std::endl;
    }
    
    // 等待机械臂实际完成移动
    waitForRobotToStop();
    
    // 等待一小段时间确保机械臂完全稳定，避免读取到空的坐标值
    // 根据经验，移动完成后需要至少500ms延迟再获取位置
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    // 获取并打印当前机械臂实际坐标位置
    geometry_msgs::msg::PoseStamped current_pose;
    if (getCurrentPose(current_pose, arm_id_)) 
    {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 移动后机械臂实际位置: x=" << std::fixed << std::setprecision(3) << current_pose.pose.position.x << ", y=" << current_pose.pose.position.y << ", z=" << current_pose.pose.position.z << ", rx=" << current_pose.pose.orientation.x << ", ry=" << current_pose.pose.orientation.y << ", rz=" << current_pose.pose.orientation.z << std::endl;
    } 
    else 
    {
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法获取移动后机械臂实际位置" << std::endl;
    }
    
    
    // 验证机械臂是否真的到达了目标位置
    double tolerance = 2.0; // 2mm容差
    if (verifyRobotAtPosition(target_pose, tolerance)) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂已成功到达目标位置" << std::endl;
    } else {
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂未能准确到达目标位置" << std::endl;
    }
    
    // 移动完成，更新状态
    robot_moving_ = false;
    publishRobotStatus(result);  // 发布机械臂空闲状态
    
    return result;
}

/**
 * @brief 移动机械臂到指定位置（指定机械臂ID）
 * @param target_pose 目标位姿
 * @param speed 运动速度(%)
 * @param arm_id 机械臂ID
 * @return 是否移动成功
 */
bool NovaRobotCtrlNode::moveRobotToPoseWithArmId(const geometry_msgs::msg::PoseStamped& target_pose, int speed, int arm_id) 
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 移动机械臂到指定位置: x=" << std::fixed << std::setprecision(3) << target_pose.pose.position.x << ", y=" << target_pose.pose.position.y << ", z=" << target_pose.pose.position.z << ", rx=" << target_pose.pose.orientation.x << ", ry=" << target_pose.pose.orientation.y << ", rz=" << target_pose.pose.orientation.z << ", 速度=" << speed << "%%, 机械臂ID=" << arm_id << std::endl;
    
    // 设置机械臂正在移动状态
    robot_moving_ = true;

    // 调用Nova机械臂的直线运动接口，带速度参数，通过RobotMgr来访问指定的机器人
    bool result = robot_mgr_.move_l_robot(
        target_pose.pose.position.x,   
        target_pose.pose.position.y,   
        target_pose.pose.position.z,
        target_pose.pose.orientation.x,         // rx
        target_pose.pose.orientation.y,         // ry
        target_pose.pose.orientation.z,         // rz
        speed,                                  // 速度参数
        arm_id                               // 机器人ID
    );
    
    if (result) 
    {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂移动命令发送成功" << std::endl;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂移动命令发送失败" << std::endl;
    }
    
    // 等待机械臂实际完成移动
    waitForRobotToStop();
    
    // 等待一小段时间确保机械臂完全稳定，避免读取到空的坐标值
    // 根据经验，移动完成后需要至少500ms延迟再获取位置
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    // 获取并打印当前机械臂实际坐标位置
    geometry_msgs::msg::PoseStamped current_pose;
    if (getCurrentPose(current_pose, arm_id)) 
    {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 移动后机械臂实际位置: x=" << std::fixed << std::setprecision(3) << current_pose.pose.position.x << ", y=" << current_pose.pose.position.y << ", z=" << current_pose.pose.position.z << ", rx=" << current_pose.pose.orientation.x << ", ry=" << current_pose.pose.orientation.y << ", rz=" << current_pose.pose.orientation.z << std::endl;
    } 
    else 
    {
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法获取移动后机械臂实际位置" << std::endl;
    }
    
    
    // 验证机械臂是否真的到达了目标位置
    double tolerance = 2.0; // 2mm容差
    if (verifyRobotAtPositionWithArmId(target_pose, tolerance, arm_id)) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂已成功到达目标位置" << std::endl;
    } else {
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂未能准确到达目标位置" << std::endl;
    }
    
    // 移动完成，更新状态
    robot_moving_ = false;
    publishRobotStatus(result, target_pose.header.frame_id);  // 发布机械臂空闲状态
    
    return result;
}

/**
 * @brief 验证机械臂是否在指定位置
 * @param target_pose 目标位置
 * @param tolerance 位置容差（毫米），默认为1.0
 * @return 是否在指定位置
 */
bool NovaRobotCtrlNode::verifyRobotAtPosition(const geometry_msgs::msg::PoseStamped& target_pose, double tolerance)
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 验证机械臂是否在指定位置" << std::endl;
    
    // 获取机械臂当前位姿，直接调用getCurrentPose接口
    geometry_msgs::msg::PoseStamped current_pose_msg;
    if (!getCurrentPose(current_pose_msg, arm_id_)) 
    {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法获取机械臂当前位姿" << std::endl;
        return false;
    }
    // 计算位置差值
    double dx = std::abs(current_pose_msg.pose.position.x - target_pose.pose.position.x);
    double dy = std::abs(current_pose_msg.pose.position.y - target_pose.pose.position.y);
    double dz = std::abs(current_pose_msg.pose.position.z - target_pose.pose.position.z);
    
    // 检查位置是否在容差范围内
    bool position_match = (dx <= tolerance) && 
                         (dy <= tolerance) && 
                         (dz <= tolerance);
    
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 当前位置: x=" << std::fixed << std::setprecision(3) << current_pose_msg.pose.position.x << ", y=" << current_pose_msg.pose.position.y << ", z=" << current_pose_msg.pose.position.z << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 目标位置: x=" << target_pose.pose.position.x << ", y=" << target_pose.pose.position.y << ", z=" << target_pose.pose.position.z << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 位置差值: dx=" << std::fixed << std::setprecision(3) << dx << ", dy=" << dy << ", dz=" << dz << ", 容差: " << std::fixed << tolerance << std::endl;
    
    if (position_match) 
    {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂位置验证通过" << std::endl;
        return true;
    } else {
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂位置验证失败" << std::endl;
        // 记录具体的偏差信息
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 位置偏差详情 - X轴偏差: " << std::fixed << std::setprecision(3) << dx << "mm, Y轴偏差: " << dy << "mm, Z轴偏差: " << dz << "mm" << std::endl;
        // 判断哪个轴的偏差最大
        if (dx > tolerance && dx >= dy && dx >= dz) 
        {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - X轴偏差最大，可能存在问题" << std::endl;
        } else if (dy > tolerance && dy >= dx && dy >= dz) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - Y轴偏差最大，可能存在问题" << std::endl;
        } else if (dz > tolerance && dz >= dx && dz >= dy) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - Z轴偏差最大，可能存在问题" << std::endl;
        }
        return false;
    }
}



/**
 * @brief 移动机械臂到标准位置
 * @param speed 运动速度(%)，默认为30(较慢的速度用于手眼标定)
 * @return 是否移动成功
 */
bool NovaRobotCtrlNode::moveRobotToStandardPose(int speed /* = 30 */) 
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 移动机械臂到标准位置，速度=" << speed << "%%" << std::endl;
    
    // 设置机械臂正在移动状态
    robot_moving_ = true;

    // 获取标准位置，通过RobotMgr来访问已连接的机器人
    nova_robot_ctrl::Pose target_pose;
    if (!robot_mgr_.get_standard_pose_robot(target_pose,arm_id_)) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 获取标准位置失败" << std::endl;
        robot_moving_ = false;
        return false;
    }
    
    // 打印要移动的位置坐标值
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 移动到标准位置: x=" << std::fixed << std::setprecision(3) << target_pose.x << ", y=" << target_pose.y << ", z=" << target_pose.z << ", rx=" << target_pose.rx << ", ry=" << target_pose.ry << ", rz=" << target_pose.rz << ", 速度=" << speed << "%%" << std::endl;
    
    // 调用Nova机械臂的直线运动接口，带速度参数，通过RobotMgr来访问已连接的机器人
    bool result = robot_mgr_.move_l_robot(
        target_pose.x,   
        target_pose.y,   
        target_pose.z,
        target_pose.rx,         // rx
        target_pose.ry,         // ry
        target_pose.rz,         // rz
        speed,                     // 速度参数
        arm_id_
    );
    
    if (result) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂移动到标准位置命令发送成功" << std::endl;
        
        // 等待机械臂实际完成移动
        waitForRobotToStop();
        
        // 等待一小段时间确保机械臂完全稳定，避免读取到空的坐标值
        // 根据经验，移动完成后需要至少500ms延迟再获取位置
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 获取并打印当前机械臂实际坐标位置
        geometry_msgs::msg::PoseStamped current_pose;
        if (getCurrentPose(current_pose, arm_id_)) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 移动后机械臂实际位置: x=" << std::fixed << std::setprecision(3) << current_pose.pose.position.x << ", y=" << current_pose.pose.position.y << ", z=" << current_pose.pose.position.z << ", rx=" << current_pose.pose.orientation.x << ", ry=" << current_pose.pose.orientation.y << ", rz=" << current_pose.pose.orientation.z << std::endl;
        } else {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法获取移动后机械臂实际位置" << std::endl;
        }
        
        // 验证机械臂是否真的到达了目标位置
        double tolerance = 2.0; // 2mm容差
        // 将nova_robot_ctrl::Pose转换为geometry_msgs::msg::PoseStamped
        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.pose.position.x = target_pose.x;
        target_pose_msg.pose.position.y = target_pose.y;
        target_pose_msg.pose.position.z = target_pose.z;
        target_pose_msg.pose.orientation.x = target_pose.rx;
        target_pose_msg.pose.orientation.y = target_pose.ry;
        target_pose_msg.pose.orientation.z = target_pose.rz;
        if (verifyRobotAtPosition(target_pose_msg, tolerance)) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂已成功到达目标位置" << std::endl;
        } else {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂未能准确到达目标位置" << std::endl;
        }
        
        // 移动完成，更新状态
        robot_moving_ = false;
        publishRobotStatus(true);  // 发布机械臂空闲状态
        
        return true;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂移动到标准位置命令发送失败" << std::endl;
        
        // 移动完成，更新状态
        robot_moving_ = false;
        publishRobotStatus(false);  // 发布机械臂空闲状态
        
        return false;
    }
}

/**
 * @brief 点动移动机械臂
 * @param axis 移动轴（'x', 'y', 'z'）
 * @param distance 移动距离（毫米）
 * @return 是否移动成功
 */
bool NovaRobotCtrlNode::jogMoveRobot(char axis, double distance)
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 点动移动机械臂: axis=" << axis << ", distance=" << std::fixed << std::setprecision(3) << distance << std::endl;
    
    // 检查机械臂是否已连接和使能
    if (!robot_connected_ || !robot_enabled_) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂未连接或未使能" << std::endl;
        return false;
    }
    
    // 调用RobotMgr的点动接口
    bool result = robot_mgr_.jog_move_robot(axis, distance,arm_id_);
    
    if (result) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂点动移动成功: axis=" << axis << ", distance=" << std::fixed << std::setprecision(3) << distance << std::endl;
        
        // 更新当前位置
        switch (axis) {
            case 'x':
            case 'X':
                current_x_ += distance;
                break;
            case 'y':
            case 'Y':
                current_y_ += distance;
                break;
            case 'z':
            case 'Z':
                current_z_ += distance;
                break;
        }
        
        // 等待机械臂实际完成移动
        waitForRobotToStop();
        
        // 等待一小段时间确保机械臂完全稳定，避免读取到空的坐标值
        // 根据经验，移动完成后需要至少500ms延迟再获取位置
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 获取并打印当前机械臂实际坐标位置
        geometry_msgs::msg::PoseStamped current_pose;
        if (getCurrentPose(current_pose, arm_id_)) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 移动后机械臂实际位置: x=" << std::fixed << std::setprecision(3) << current_pose.pose.position.x << ", y=" << current_pose.pose.position.y << ", z=" << current_pose.pose.position.z << ", rx=" << current_pose.pose.orientation.x << ", ry=" << current_pose.pose.orientation.y << ", rz=" << current_pose.pose.orientation.z << std::endl;
        } else {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法获取移动后机械臂实际位置" << std::endl;
        }
        
        // 点动移动不需要验证目标位置，因为这是一个相对移动
        
        return true;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂点动移动失败: axis=" << axis << ", distance=" << std::fixed << std::setprecision(3) << distance << std::endl;
        return false;
    }
}

/**
 * @brief 发布机械臂状态
 * @param is_ready 机械臂是否准备就绪
 */
void NovaRobotCtrlNode::publishRobotStatus(bool is_ready, std::string frame_id) 
{
    auto status_msg = custom_msgs_comm::msg::BoolStamped();
    status_msg.data = is_ready;
    status_msg.header.frame_id = frame_id;
    status_msg.header.stamp = this->now();
    
    robot_status_pub_->publish(status_msg);
    
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 发布机械臂状态: " << (is_ready ? "准备就绪" : "正在移动") << std::endl;
}

/**
 * @brief 发布机械臂准备状态
 * @param is_ready 机械臂是否准备就绪
 */
void NovaRobotCtrlNode::publishRobotReadyState(bool is_ready) 
{
    auto ready_state_msg = std_msgs::msg::Bool();
    ready_state_msg.data = is_ready;
    
    robot_ready_state_pub_->publish(ready_state_msg);
    
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 发布机械臂准备状态: " << (is_ready ? "准备就绪" : "未准备") << std::endl;
}

/**
 * @brief 发布机械臂位姿
 */
void NovaRobotCtrlNode::publishRobotPose() 
{
    // 为每个启用的机械臂发布位姿
    for (int arm_id : arm_id_list_) {
        geometry_msgs::msg::PoseStamped pose;
        if (getCurrentPose(pose, arm_id)) 
        {
            auto current_pose = custom_msgs_comm::msg::RobotStdPose();
            current_pose.arm_id = arm_id;
            current_pose.std_pose = pose;
            robot_pose_pub_->publish(current_pose);
            //std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 发布机械臂位置:  " << arm_id << current_pose.std_pose.pose.position.x << current_pose.std_pose.pose.position.y << current_pose.std_pose.pose.position.z << std::endl;
        }
    }
}

/**
 * @brief 等待机械臂停止移动
 */
void NovaRobotCtrlNode::waitForRobotToStop() 
{
    robot_mgr_.sync(arm_id_);
    // std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 等待机械臂停止移动" << std::endl;
    
    // // 获取初始推送信息，通过RobotMgr来访问已连接的机器人
    // auto initial_info = robot_mgr_.get_pushed_info_robot();
    
    // // 等待机械臂停止移动
    // const double velocity_threshold = 0.01;  // 关节速度阈值 (度/秒)
    // const int max_wait_time = 10000;  // 最大等待时间 (毫秒)
    // const int check_interval = 100;   // 检查间隔 (毫秒)
    // int waited_time = 0;
    
    // while (waited_time < max_wait_time) {
    //     // 获取当前推送信息，通过RobotMgr来访问已连接的机器人
    //     auto current_info = robot_mgr_.get_pushed_info_robot();
        
    //     // 检查关节速度是否都接近0
    //     bool stopped = true;
    //     for (const auto& velocity : current_info.q_d_actual) {
    //         if (std::abs(velocity) > velocity_threshold) {
    //             stopped = false;
    //             break;
    //         }
    //     }
        
    //     // 如果机械臂已停止，退出循环
    //     if (stopped) {
    //         std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂已停止移动" << std::endl;
    //         // 再延时2000ms，确保机械臂完全停稳并且位置信息已更新
    //         std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //         std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂已完全停稳" << std::endl;
    //         break;
    //     }
        
    //     // 等待一段时间再检查
    //     std::this_thread::sleep_for(std::chrono::milliseconds(check_interval));
    //     waited_time += check_interval;
    // }
    
    // if (waited_time >= max_wait_time) {
    //     std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 等待机械臂停止超时" << std::endl;
    // }
}

/**
 * @brief 验证指定机械臂是否在指定位置
 * @param target_pose 目标位置
 * @param tolerance 位置容差（毫米），默认为1.0
 * @param arm_id 机械臂ID
 * @return 是否在指定位置
 */
bool NovaRobotCtrlNode::verifyRobotAtPositionWithArmId(const geometry_msgs::msg::PoseStamped& target_pose, double tolerance, int arm_id)
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 验证机械臂是否在指定位置，机械臂ID: " << arm_id << std::endl;
    
    // 获取机械臂当前位姿，直接调用getCurrentPose接口
    geometry_msgs::msg::PoseStamped current_pose_msg;
    if (!getCurrentPose(current_pose_msg, arm_id)) 
    {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法获取机械臂ID为 " << arm_id << " 的当前位姿" << std::endl;
        return false;
    }
    // 计算位置差值
    double dx = std::abs(current_pose_msg.pose.position.x - target_pose.pose.position.x);
    double dy = std::abs(current_pose_msg.pose.position.y - target_pose.pose.position.y);
    double dz = std::abs(current_pose_msg.pose.position.z - target_pose.pose.position.z);
    
    // 检查位置是否在容差范围内
    bool position_match = (dx <= tolerance) && 
                         (dy <= tolerance) && 
                         (dz <= tolerance);
    
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 当前位置: x=" << std::fixed << std::setprecision(3) << current_pose_msg.pose.position.x << ", y=" << current_pose_msg.pose.position.y << ", z=" << current_pose_msg.pose.position.z << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 目标位置: x=" << target_pose.pose.position.x << ", y=" << target_pose.pose.position.y << ", z=" << target_pose.pose.position.z << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 位置差值: dx=" << std::fixed << std::setprecision(3) << dx << ", dy=" << dy << ", dz=" << dz << ", 容差: " << std::fixed << tolerance << std::endl;
    
    if (position_match) 
    {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂ID " << arm_id << " 位置验证通过" << std::endl;
        return true;
    } else {
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂ID " << arm_id << " 位置验证失败" << std::endl;
        // 记录具体的偏差信息
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 位置偏差详情 - X轴偏差: " << std::fixed << std::setprecision(3) << dx << "mm, Y轴偏差: " << dy << "mm, Z轴偏差: " << dz << "mm" << std::endl;
        // 判断哪个轴的偏差最大
        if (dx > tolerance && dx >= dy && dx >= dz) 
        {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - X轴偏差最大，可能存在问题" << std::endl;
        } else if (dy > tolerance && dy >= dx && dy >= dz) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - Y轴偏差最大，可能存在问题" << std::endl;
        } else if (dz > tolerance && dz >= dx && dz >= dy) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - Z轴偏差最大，可能存在问题" << std::endl;
        }
        return false;
    }
}



// 回调函数
/**
 * @brief 机械臂目标位姿回调函数
 * @param msg 机械臂目标位姿消息
 */
void NovaRobotCtrlNode::robotTargetPoseCallback(const custom_msgs_comm::msg::RobotStdPose::SharedPtr msg)
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 收到机械臂目标位姿: x=" << std::fixed << std::setprecision(3) << msg->std_pose.pose.position.x << ", y=" << msg->std_pose.pose.position.y << ", z=" << msg->std_pose.pose.position.z << ", rx=" << msg->std_pose.pose.orientation.x << ", ry=" << msg->std_pose.pose.orientation.y << ", rz=" << msg->std_pose.pose.orientation.z << ", 机械臂ID=" << msg->arm_id << std::endl;
    
    // 从消息中获取机械臂ID
    int target_arm_id = msg->arm_id;
    
    // 检查机械臂是否已连接和使能
    if (!robot_connected_ || !robot_enabled_) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂未连接或未使能" << std::endl;
        return;
    }
    
    // 添加位置有效性检查
    if (!robot_mgr_.is_position_valid_robot(msg->std_pose.pose.position.x, msg->std_pose.pose.position.y, msg->std_pose.pose.position.z, target_arm_id)) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 目标位置超出允许范围，拒绝移动。目标位置：x=" << std::fixed << std::setprecision(3) << msg->std_pose.pose.position.x << "，y=" << msg->std_pose.pose.position.y << "，z=" << msg->std_pose.pose.position.z << "，机械臂ID=" << target_arm_id << std::endl;
        return;
    }
    
    // 记录移动前的当前位置
    geometry_msgs::msg::PoseStamped current_pose_before;
    if (getCurrentPose(current_pose_before, target_arm_id)) 
    {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 移动前机械臂位置: x=" << std::fixed << std::setprecision(3) << current_pose_before.pose.position.x << ", y=" << current_pose_before.pose.position.y << ", z=" << current_pose_before.pose.position.z << ", 机械臂ID=" << target_arm_id << std::endl;
    } else {
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法获取移动前机械臂位置，机械臂ID=" << target_arm_id << std::endl;
    }
    
    // 移动机械臂到目标位置，使用较慢的速度(30%)进行手眼标定
    if (moveRobotToPoseWithArmId(msg->std_pose, 30, target_arm_id)) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂移动到目标位置成功，机械臂ID=" << target_arm_id << std::endl;
        
        // 记录移动后的当前位置
        geometry_msgs::msg::PoseStamped current_pose_after;
        if (getCurrentPose(current_pose_after, target_arm_id)) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 移动后机械臂位置: x=" << std::fixed << std::setprecision(3) << current_pose_after.pose.position.x << ", y=" << current_pose_after.pose.position.y << ", z=" << current_pose_after.pose.position.z << ", 机械臂ID=" << target_arm_id << std::endl;
                    
            // 计算实际移动距离
            double moved_x = std::abs(current_pose_after.pose.position.x - current_pose_before.pose.position.x);
            double moved_y = std::abs(current_pose_after.pose.position.y - current_pose_before.pose.position.y);
            double moved_z = std::abs(current_pose_after.pose.position.z - current_pose_before.pose.position.z);
                    
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 实际移动距离: x=" << std::fixed << std::setprecision(3) << moved_x << "mm, y=" << moved_y << "mm, z=" << moved_z << "mm, 机械臂ID=" << target_arm_id << std::endl;
        } else {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法获取移动后机械臂位置，机械臂ID=" << target_arm_id << std::endl;
        }
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂移动到目标位置失败，机械臂ID=" << target_arm_id << std::endl;
    }
}

/**
 * @brief 机械臂控制服务回调函数
 * @param request 请求
 * @param response 响应
 */
void NovaRobotCtrlNode::robotControlServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 收到机械臂控制服务请求" << std::endl;
    
    // 检查机械臂是否已连接和使能
    if (!robot_connected_ || !robot_enabled_) {
        response->success = false;
        response->message = "机械臂未连接或未使能";
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - " << response->message << std::endl;
        
        // 发布机械臂准备状态（失败）
        publishRobotReadyState(false);
        return;
    }
    
    // 发布机械臂准备状态（准备开始移动）
    publishRobotReadyState(false);
    
    // 移动机械臂到标准位置，使用较慢的速度(30%)进行手眼标定
    if (1 || moveRobotToStandardPose(30)) {
        response->success = true;
        response->message = "机械臂已移动到标准位置";
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - " << response->message << std::endl;
        
        // 发布机械臂准备状态（移动完成）
        publishRobotReadyState(true);
    } else {
        response->success = false;
        response->message = "机械臂移动到标准位置失败";
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - " << response->message << std::endl;
        
        // 发布机械臂准备状态（移动失败）
        publishRobotReadyState(false);
    }
}

/**
 * @brief 定时器回调函数
 */
void NovaRobotCtrlNode::timerCallback() 
{
    // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 定时器回调函数被调用" << std::endl;
    
    // 定期发布机械臂位姿
    if (robot_connected_ && robot_enabled_) {
        publishRobotPose();
    }
}

/**
 * @brief 启动手眼标定回调函数
 */
void NovaRobotCtrlNode::startCalibrationCallback()
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 收到启动标定命令" << std::endl;
    
    // 检查机械臂是否已连接和使能
    if (!robot_connected_ || !robot_enabled_) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂未连接或未使能" << std::endl;
        return;
    }
    
    // 移动机械臂到标准位置，使用较慢的速度(30%)进行手眼标定
    if (1 || moveRobotToStandardPose(30)) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂移动到标准位置成功" << std::endl;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂移动到标准位置失败" << std::endl;
    }
}

/**
 * @brief 获取标定点数据服务回调函数
 * @param request 请求
 * @param response 响应
 */
// void NovaRobotCtrlNode::getCalibrationPointsServiceCallback(
//     const std::shared_ptr<custom_msgs_comm::srv::GetCalibrationPoints::Request> request,
//     std::shared_ptr<custom_msgs_comm::srv::GetCalibrationPoints::Response> response)
// {
//     std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 收到获取标定点数据服务请求" << std::endl;
//     std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 请求ID: " << request->request_id.c_str() << std::endl;
    
//     // 获取标定点管理器，通过RobotMgr来访问已连接的机器人
//     nova_robot_ctrl::CalibRobotPosMgr* calib_mgr = robot_mgr_.get_calib_robot_pos_mgr_robot();
    
//     if (calib_mgr) 
//     {
//         // 初始化缓存数据点
//         calib_mgr->initializeCachedDataPoints();
        
//         // 获取数据点数量
//         size_t point_count = calib_mgr->getPointCount();
//         std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 标定点数量: " << static_cast<int>(point_count) << std::endl;
        
//         if (point_count > 0) 
//         {
//             // 填充响应数据
//             response->success = true;
//             response->message = "成功获取标定点数据";
            
//             // 遍历所有数据点并填充到响应中
//             for (size_t i = 0; i < point_count; ++i) 
//             {
//                 nova_robot_ctrl::RobotPoseData point = calib_mgr->getDataPoint(i);
//                 response->x.push_back(point.x);
//                 response->y.push_back(point.y);
//                 response->z.push_back(point.z);
//                 response->rx.push_back(point.rx);
//                 response->ry.push_back(point.ry);
//                 response->rz.push_back(point.rz);
//             }
            
//             std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 成功填充 " << static_cast<int>(point_count) << " 个标定点数据到响应中" << std::endl;
//         } 
//         else 
//         {
//             response->success = false;
//             response->message = "标定点数据为空";
//             std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 标定点数据为空" << std::endl;
//         }
//     } 
//     else 
//     {
//         response->success = false;
//         response->message = "无法获取标定点管理器";
//         std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法获取标定点管理器" << std::endl;
//     }
// }

/**
 * @brief 获取机械臂位姿服务回调函数
 * @param request 请求
 * @param response 响应
 */
void NovaRobotCtrlNode::getRobotPoseServiceCallback(
    const std::shared_ptr<custom_msgs_comm::srv::GetRobotPose::Request> request,
    std::shared_ptr<custom_msgs_comm::srv::GetRobotPose::Response> response)
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 收到获取机械臂位姿服务请求" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 请求ID: " << request->request_id.c_str() << std::endl;
    
    // 从请求ID中解析机械臂ID
    int target_arm_id = 0;
    try {
        target_arm_id = std::stoi(request->request_id);
    } catch (const std::invalid_argument& e) {
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法解析请求ID为机械臂ID，默认使用0: " << request->request_id.c_str() << std::endl;
        target_arm_id = 0;  // 默认使用ID为0的机械臂
    } catch (const std::out_of_range& e) {
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 请求ID超出机械臂ID范围，默认使用0: " << request->request_id.c_str() << std::endl;
        target_arm_id = 0;  // 默认使用ID为0的机械臂
    }
    
    // 获取机械臂当前位姿
    geometry_msgs::msg::PoseStamped current_pose;
    if (getCurrentPose(current_pose, target_arm_id)) 
    {
        // 填充响应数据
        response->success = true;
        response->message = "成功获取机械臂位姿";
        response->x = current_pose.pose.position.x;
        response->y = current_pose.pose.position.y;
        response->z = current_pose.pose.position.z;
        response->rx = current_pose.pose.orientation.x;
        response->ry = current_pose.pose.orientation.y;
        response->rz = current_pose.pose.orientation.z;
        response->stamp = current_pose.header.stamp;
        response->arm_id = target_arm_id;
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 成功获取机械臂位姿: x=" << std::fixed << std::setprecision(3) << response->x << ", y=" << response->y << ", z=" << response->z << ", rx=" << response->rx << ", ry=" << response->ry << ", rz=" << response->rz << ", arm_id=" << response->arm_id << std::endl;
    } 
    else 
    {
        response->success = false;
        response->message = "获取机械臂位姿失败";
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 获取机械臂位姿失败" << std::endl;
    }
}

/**
 * @brief 初始化夹爪控制
 */
void NovaRobotCtrlNode::initGripperControl()
{
    // 初始化夹爪控制相关变量
    gripper_position_ = 0;
    gripper_speed_ = 50;
    gripper_force_ = 0;
    gripper_abs_position_ = 2000;

    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪控制初始化完成" << std::endl;
}

/**
 * @brief 键盘控制服务回调函数
 * @param request 服务请求
 * @param response 服务响应
 */
void NovaRobotCtrlNode::keyboardControlServiceCallback(
    const std::shared_ptr<custom_msgs_comm::srv::KeyboardControl::Request> request,
    std::shared_ptr<custom_msgs_comm::srv::KeyboardControl::Response> response)
{
    const std::string& key = request->key_name;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 收到键盘控制服务请求: " << key << std::endl;

    // 方向键处理
    if (key == "UP") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 上箭头 - 向前移动" << std::endl;
        response->success = jogMoveRobot('y', step_size_);
        response->message = "向前移动";
    }
    else if (key == "DOWN") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 下箭头 - 向后移动" << std::endl;
        response->success = jogMoveRobot('y', -step_size_);
        response->message = "向后移动";
    }
    else if (key == "RIGHT") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 右箭头 - 向右移动" << std::endl;
        response->success = jogMoveRobot('x', step_size_);
        response->message = "向右移动";
    }
    else if (key == "LEFT") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 左箭头 - 向左移动" << std::endl;
        response->success = jogMoveRobot('x', -step_size_);
        response->message = "向左移动";
    }
    // 数字键处理
    else if (key == "1") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 1键 - 向左移动" << std::endl;
        response->success = jogMoveRobot('x', -step_size_*10);
        response->message = "向左移动(大步)";
    }
    else if (key == "2") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 2键 - 向前移动" << std::endl;
        response->success = jogMoveRobot('y', -step_size_*10);
        response->message = "向前移动(大步)";
    }
    else if (key == "3") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 3键 - 向下移动" << std::endl;
        response->success = jogMoveRobot('z', -step_size_*10);
        response->message = "向下移动(大步)";
    }
    else if (key == "4") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 4键 - 向右移动" << std::endl;
        response->success = jogMoveRobot('x', step_size_*10);
        response->message = "向右移动(大步)";
    }
    else if (key == "5") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 5键 - 向后移动" << std::endl;
        response->success = jogMoveRobot('y', step_size_*10);
        response->message = "向后移动(大步)";
    }
    else if (key == "6") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 6键 - 向上移动" << std::endl;
        response->success = jogMoveRobot('z', step_size_*10);
        response->message = "向上移动(大步)";
    }
    // 字母键处理 - Z轴控制
    else if (key == "I") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - I键 - 向上移动" << std::endl;
        response->success = jogMoveRobot('z', step_size_);
        response->message = "向上移动";
    }
    else if (key == "K") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - K键 - 向下移动" << std::endl;
        response->success = jogMoveRobot('z', -step_size_);
        response->message = "向下移动";
    }
    // S键处理 - 切换robotid
    else if (key == "S") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - S键 - 切换robotid" << std::endl;
        // 切换robotid（假设在0和1之间切换，可根据需要调整）
        if (arm_id_ == 1) {
            arm_id_ = 2;
        } else {
            arm_id_ = 1;
        }
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - robotid已切换到: " << arm_id_ << std::endl;
        response->success = true;
        response->message = "robotid已切换到: " + std::to_string(arm_id_);
    }
    // 夹爪控制
    else if (key == "E") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - E键 - 启用夹爪" << std::endl;
        enableGripper();
        response->success = true;
        response->message = "启用夹爪";
    }
    else if (key == "D") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - D键 - 禁用夹爪" << std::endl;
        disableGripper();
        response->success = true;
        response->message = "禁用夹爪";
    }
    else if (key == "G") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - G键 - 打开夹爪" << std::endl;
        openGripper();
        response->success = true;
        response->message = "打开夹爪";
    }
    else if (key == "H") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - H键 - 闭合夹爪" << std::endl;
        closeGripper();
        response->success = true;
        response->message = "闭合夹爪";
    }
    else if (key == "J") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - J键 - Ping夹爪连接" << std::endl;
        pingGripper();
        response->success = true;
        response->message = "Ping夹爪连接";
    }
    else if (key == "U") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - U键 - 减小夹爪位置" << std::endl;
        adjustGripperPosition(-1);
        response->success = true;
        response->message = "减小夹爪位置";
    }
    else if (key == "O") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - O键 - 增大夹爪位置" << std::endl;
        adjustGripperPosition(1);
        response->success = true;
        response->message = "增大夹爪位置";
    }
    else if (key == "V") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - V键 - 减小夹爪绝对位置" << std::endl;
        adjustGripperAbsPosition(-100);
        response->success = true;
        response->message = "减小夹爪绝对位置";
    }
    else if (key == "N") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - N键 - 增大夹爪绝对位置" << std::endl;
        adjustGripperAbsPosition(100);
        response->success = true;
        response->message = "增大夹爪绝对位置";
    }
    // 位置显示
    else if (key == "P") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - P键 - 显示当前位置" << std::endl;
        geometry_msgs::msg::PoseStamped pose;
        if (getCurrentPose(pose, arm_id_)) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 当前机械臂位置: x=" << std::fixed << std::setprecision(3) << pose.pose.position.x << ", y=" << pose.pose.position.y << ", z=" << pose.pose.position.z << ", rx=" << pose.pose.orientation.x << ", ry=" << pose.pose.orientation.y << ", rz=" << pose.pose.orientation.z << std::endl;
            response->success = true;
            response->message = "当前位置: x=" + std::to_string(pose.pose.position.x) + ", y=" + std::to_string(pose.pose.position.y) + ", z=" + std::to_string(pose.pose.position.z);
        } else {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法获取机械臂位置" << std::endl;
            response->success = false;
            response->message = "无法获取机械臂位置";
        }
    }
    else if (key == "Q") {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - Q键 - 退出程序" << std::endl;
        response->success = true;
        response->message = "退出程序";
        rclcpp::shutdown();
    }
    else {
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 未处理的按键: " << key << std::endl;
        response->success = false;
        response->message = "未处理的按键: " + key;
    }
}

/**
 * @brief 移动到指定位姿服务回调函数
 * @param request 服务请求
 * @param response 服务响应
 */
void NovaRobotCtrlNode::moveToPoseServiceCallback(
    const std::shared_ptr<custom_msgs_comm::srv::MoveToPose::Request> request,
    std::shared_ptr<custom_msgs_comm::srv::MoveToPose::Response> response)
{
    const auto& target_pose = request->target_pose;
    int speed = request->speed;
    int arm_id = request->arm_id;

    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 收到移动到指定位姿服务请求" << std::endl;
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 目标位置: x=" << target_pose.pose.position.x
              << ", y=" << target_pose.pose.position.y << ", z=" << target_pose.pose.position.z << ", 速度=" << speed << "%, 机械臂ID=" << arm_id << std::endl;

    // 检查机械臂是否已连接和使能
    if (!robot_connected_) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂未连接" << std::endl;
        response->success = false;
        response->message = "机械臂未连接";
        return;
    }

    if (!robot_enabled_) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂未使能" << std::endl;
        response->success = false;
        response->message = "机械臂未使能";
        return;
    }
    
    // 调用移动到指定位姿函数，这里需要传递arm_id
    if (moveRobotToPoseWithArmId(target_pose, speed, arm_id)) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 移动到指定位姿成功" << std::endl;
        response->success = true;
        response->message = "移动到指定位姿成功";
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 移动到指定位姿失败" << std::endl;
        response->success = false;
        response->message = "移动到指定位姿失败";
    }
}

/**
 * @brief 打开夹爪
 */
void NovaRobotCtrlNode::openGripper() 
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 打开夹爪" << std::endl;
    
    // 调用RobotMgr的夹爪控制接口
    auto result = robot_mgr_.gripper_open(arm_id_);
    if (result) {
        // 更新夹爪位置
        gripper_position_ = 100;  // 假设100为完全打开位置
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪位置更新为: " << gripper_position_ << std::endl;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪控制失败" << std::endl;
    }
}

/**
 * @brief 闭合夹爪
 */
void NovaRobotCtrlNode::closeGripper() 
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 闭合夹爪" << std::endl;
    
    // 调用RobotMgr的夹爪控制接口
    auto result = robot_mgr_.gripper_close(arm_id_);
    if (result) {
        // 更新夹爪位置
        gripper_position_ = 0;  // 假设0为完全闭合位置
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪位置更新为: " << gripper_position_ << std::endl;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪控制失败" << std::endl;
    }
}

/**
 * @brief 启用夹爪
 */
void NovaRobotCtrlNode::enableGripper() 
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 启用夹爪" << std::endl;
    
    // 调用RobotMgr的夹爪控制接口
    bool result = robot_mgr_.enable_gripper(arm_id_);
    if (result) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪启用成功" << std::endl;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪启用失败" << std::endl;
    }
}

/**
 * @brief 禁用夹爪
 */
void NovaRobotCtrlNode::disableGripper() 
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 禁用夹爪" << std::endl;
    
    // 调用RobotMgr的夹爪控制接口
    bool result = robot_mgr_.disable_gripper(arm_id_);
    if (result) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪禁用成功" << std::endl;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪禁用失败" << std::endl;
    }
}

void NovaRobotCtrlNode::pingGripper() 
{
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - Ping夹爪连接" << std::endl;
    
    // 调用RobotMgr的夹爪控制接口
    bool result = robot_mgr_.ping_gripper(arm_id_);
    if (result) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪连接正常" << std::endl;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪连接异常" << std::endl;
    }
}

void NovaRobotCtrlNode::adjustGripperPosition(int delta_position) 
{
    // 先获取最新的夹爪位置
    // 尝试获取当前夹爪位置，如果获取失败则保持原有位置
    gripper_position_ = robot_mgr_.get_current_position_gripper(arm_id_);
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 当前夹爪位置: " << gripper_position_ << std::endl;
    
    // 更新夹爪位置
    gripper_position_ += delta_position;
    // 限制在合理范围内
    gripper_position_ = std::max(0, std::min(100, gripper_position_));
    
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 调整夹爪位置: " << gripper_position_ << " (delta: " << delta_position << ")" << std::endl;
    
    // 调用RobotMgr的夹爪控制接口
    auto result = robot_mgr_.move_gripper(gripper_position_, gripper_speed_, gripper_force_, arm_id_);
    if (result.first) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪位置调整成功: " << gripper_position_ << std::endl;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪位置调整失败，错误码: " << result.second << std::endl;
    }
}

void NovaRobotCtrlNode::adjustGripperAbsPosition(int delta_position) 
{
    gripper_abs_position_ += delta_position;
    // 限制在合理范围内
    gripper_abs_position_ = std::max(0, std::min(100000, gripper_abs_position_));
    
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 调整夹爪绝对位置: " << gripper_abs_position_ << " (delta: " << delta_position << ")" << std::endl;
    
    // 调用RobotMgr的夹爪控制接口
    auto result = robot_mgr_.move_by_abs_pos_gripper(gripper_abs_position_, 1500, 0, arm_id_);
    if (result.first) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪绝对位置调整成功: " << gripper_abs_position_ << std::endl;
    } else {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 夹爪绝对位置调整失败，错误码: " << result.second << std::endl;
    }
}

bool NovaRobotCtrlNode::load_robot_sys_srv()
{
    try
    {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 开始从参数服务器加载机器人系统配置信息..." << std::endl;
        
        // 创建连接到系统配置节点的参数客户端
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "sys_config_ros_node");
        
        // 等待参数服务可用
        if (!parameters_client->wait_for_service(std::chrono::seconds(3)))
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 无法连接到系统配置参数服务" << std::endl;
            return false;
        }
        
        // 使用正确的接口从参数服务器读取机械臂配置信息列表
        SysConfig::ArmConfigInfoList sys_arm_list;
        if (!RosComm::getArmInfoListFromServer(parameters_client, SYS_ENABLE_ARM_LIST, sys_arm_list))
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 从参数服务器读取机械臂配置信息列表失败" << std::endl;
            return false;
        }

        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 成功从参数服务器获取到 " << sys_arm_list.size() << " 个机械臂配置" << std::endl;
        
        // 打印所有获取到的机械臂配置信息
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - === 从参数服务器获取的机械臂配置信息 ===" << std::endl;
        
        // 创建新的配置映射
        std::map<int, nova_robot_ctrl::RobotConfig> new_robot_configs;
        std::map<int, bool> new_robot_enables;
        
        // 遍历获取到的机械臂配置信息
        for (size_t i = 0; i < sys_arm_list.size(); ++i)
        {
            const auto &sys_arm_info = sys_arm_list[i];
            
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 机械臂配置 " << i << ":" << std::endl;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() -   - 机械臂ID: " << static_cast<int>(sys_arm_info.arm_id) << std::endl;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() -   - 是否启用: " << (sys_arm_info.is_enable ? "是" : "否") << std::endl;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() -   - IP地址: " << sys_arm_info.robot_arm_ip << std::endl;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() -   - 用户名称: " << sys_arm_info.user_name << std::endl;
            
            // 创建机器人配置
            nova_robot_ctrl::RobotConfig robot_config;
            robot_config.ip = sys_arm_info.robot_arm_ip;
            robot_config.user_name = sys_arm_info.user_name;
            
            // 更新配置映射
            new_robot_configs[sys_arm_info.arm_id] = robot_config;
            new_robot_enables[sys_arm_info.arm_id] = sys_arm_info.is_enable;
            
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 成功更新机器人 " << static_cast<int>(sys_arm_info.arm_id) << " 配置" << std::endl;
            
            // 加载单个机器人配置文件
            if (robot_mgr_.load_arm_sys_config(sys_arm_info.arm_id))
            {
                std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 成功加载机器人 " << static_cast<int>(sys_arm_info.arm_id) << " 配置文件" << std::endl;
            }
            else
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 加载机器人 " << static_cast<int>(sys_arm_info.arm_id) << " 配置文件失败" << std::endl;
            }
        }
        
        // 使用getter/setter接口更新RobotMgr中的配置
        robot_mgr_.set_all_robot_configs(new_robot_configs);
        robot_mgr_.set_all_robot_enables(new_robot_enables);
        
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - === 机械臂配置信息加载完成 ===" << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 从参数服务器加载机器人系统配置时发生错误: " << e.what() << std::endl;
        return false;
    }
    // 移除多余的 return true;
}

bool NovaRobotCtrlNode::load_config(int mode)
{
    bool config_loaded = false;
    
    if (mode == 1) {
        // 尝试从参数服务器加载配置
        if (load_robot_sys_srv()) 
        {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 从参数服务器成功加载RobotMgr配置" << std::endl;
            config_loaded = true;
        }
        else
        {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 从参数服务器加载RobotMgr配置失败" << std::endl;
            // 如果从参数服务器加载失败，则从文件加载
            if (!robot_mgr_.load_config(0))  // mode=0 表示从文件加载
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 从文件加载RobotMgr配置失败" << std::endl;
            }
            else
            {
                std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 从文件成功加载RobotMgr配置" << std::endl;
                config_loaded = true;
            }
        }
    }
    else {
        // 直接从文件加载
        if (!robot_mgr_.load_config(0))  // mode=0 表示从文件加载
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 从文件加载RobotMgr配置失败" << std::endl;
        }
        else
        {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __func__ << "() - 从文件成功加载RobotMgr配置" << std::endl;
            config_loaded = true;
        }
    }
    
    return config_loaded;
}

/**
 * @brief 打印按键操作说明
 */
void NovaRobotCtrlNode::print_key_operate() {
    std::cout << "==================================================" << std::endl;
    std::cout << "按键控制说明:" << std::endl;
    std::cout << "  方向键 UP/DOWN/LEFT/RIGHT - 控制机械臂XY方向移动" << std::endl;
    std::cout << "  I/K - Z轴上下移动" << std::endl;
    std::cout << "  数字键 1-6 - 大步距移动" << std::endl;
    std::cout << "  E/D - 启用/禁用夹爪" << std::endl;
    std::cout << "  G/H - 打开/闭合夹爪" << std::endl;
    std::cout << "  J - Ping夹爪连接" << std::endl;
    std::cout << "  U/O - 调整夹爪相对位置" << std::endl;
    std::cout << "  V/N - 调整夹爪绝对位置" << std::endl;
    std::cout << "  P - 显示当前位置" << std::endl;
    std::cout << "  S - 切换机械臂ID" << std::endl;
    std::cout << "  Q - 退出程序" << std::endl;
    std::cout << "==================================================" << std::endl;
}

}  // namespace nova_robot_ctrl_ros