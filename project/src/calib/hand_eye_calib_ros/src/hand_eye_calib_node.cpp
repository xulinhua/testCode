#include "hand_eye_calib_ros/hand_eye_calib_node.hpp"
#include "log_system/log_macros.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "vision_msgs/msg/detection2_d.hpp"  // 添加Detection2D消息头文件
#include "custom_msgs_comm/srv/project_control.hpp"  // 添加ProjectControl服务头文件
#include "custom_msgs_comm/msg/robot_std_pose.hpp"
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <filesystem>  // 用于文件系统操作
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "hand_eye_calib/sys_calib_data_mgr.hpp"  // 添加系统标定数据管理器头文件
#include "bas_operate/file_operate.hpp"  // 添加文件操作头文件，包含basmodule::get_install_dir函数
//#include <tf2/LinearMath/Quaternion.h>

// 控制是否发送测试标定点的变量，设为true即可发送测试点
static bool SEND_TEST_POINT = false;
std::string calib_target_frame_ = "calib_target_frame";

namespace handeyecalib_ros {

/**
 * @brief 构造函数
 * @param options ROS节点选项
 */
HandEyeCalibNode::HandEyeCalibNode(const rclcpp::NodeOptions & options)
: StatusNodeBase("hand_eye_calib_node", "hand_eye_calib_ros", options),
  calib_started_(false),
  robot_ready_(false),
  robot_moved_(false),
  current_point_index_(0),
  successful_points_count_(0),
  use_service_for_points_(true),   // 默认通过服务通讯获取标定点
  calibration_points_load_success_(false)  // 标定点数据加载未成功
{
    LOG_INFO("开始 HandEyeCalibNode 构造函数");
    initParameters();// 初始化参数
    initSubscribers();// 初始化订阅器
    initHeadMotorAngleSubscriber();// 初始化头电机角度订阅器
    initPublishers();// 初始化发布器
    initServiceClients();// 初始化服务客户端
    initServiceServers();// 初始化服务服务器
    initParametersClient();// 初始化参数服务器客户端
    initTimer();// 初始化定时器

    // 初始化标定参数处理器
    calib_param_handler_ = std::make_unique<RosComm::ArmCalibParamHandler>(parameters_node_);
    // 注册标定参数回调
    calib_param_handler_->registerCalibParamCallback(
        [this](const handeyecalib::ArmCalibInfo& calib_data) {
            this->calibDatChangedCallback(calib_data);
        });
    // 从参数服务器读取初始标定数据
    handeyecalib::ArmCalibInfo initial_calib_data;
    if (RosComm::getArmCalibInfoFromServer(sys_config_client_, camID_, armID_, initial_calib_data)) 
    {
        calibDatChangedCallback(initial_calib_data);// 处理初始标定数据
    }
    // 发送测试标定点（如果启用）
    if (SEND_TEST_POINT) 
    {
        LOG_INFO("开始测试发送标定点");
        auto test_msg = custom_msgs_comm::msg::RobotStdPose();
        test_msg.arm_id = armID_;
        test_msg.std_pose.header.stamp = this->now();
        test_msg.std_pose.header.frame_id = "base_link";
        test_msg.std_pose.pose.position.x = 0.3;
        test_msg.std_pose.pose.position.y = 0.0;
        test_msg.std_pose.pose.position.z = 0.2;
        test_msg.std_pose.pose.orientation.x = 0.0;
        test_msg.std_pose.pose.orientation.y = 0.0;
        test_msg.std_pose.pose.orientation.z = 0.0;
        test_msg.std_pose.pose.orientation.w = 1.0;
        
        robot_target_pub_->publish(test_msg);
        LOG_INFO("发送测试标定点: x=0.3, y=0.0, z=0.2");
    }
    
    // 加载配置
    // 直接使用hand_eye_calib包中的配置文件
    std::string package_path = ament_index_cpp::get_package_share_directory("hand_eye_calib");
    std::string config_file = package_path + "/config/calib_config.yaml";
    if (!calib_config_.loadConfig(config_file)) {
        LOG_WARN("无法加载配置文件: %s", config_file.c_str());
    } else {
        LOG_INFO("成功加载配置文件: %s", config_file.c_str());
    }

    // 初始化标定点数据
    if (!initCalibrationPoints()) 
    {
        LOG_ERROR("初始化标定点数据失败");
    }
    // 节点启动后主动发送服务请求给机械臂项目，通知机械臂进行初始化准备移动
    LOG_INFO("节点启动完成，主动发送服务请求通知机械臂进行初始化准备移动");
    sendStartCalibCommandToRobot();
    LOG_INFO("已发送机械臂控制服务请求，等待机械臂准备OK信号启动标定流程");
    LOG_INFO("HandEyeCalibNode 初始化完成");    
    
    // 发布运行中状态
    publishModuleStatus(basros::ModuleStatus::RUNNING, "模块初始化完成，正常运行中");
}

/**
 * @brief 析构函数
 */
HandEyeCalibNode::~HandEyeCalibNode() 
{
    LOG_INFO("HandEyeCalibNode 析构");
}

/**
 * @brief 初始化参数
 */
void HandEyeCalibNode::initParameters() 
{
    camID_ = 0;
    armID_ = 0;
    eye_on_hand_ = false;

    // 声明并获取参数
    //this->declare_parameter("robot_pose_topic", "/right_arm_cartesian_pose");
    this->declare_parameter("robot_run_state_topic", "/robot_run_state");
    this->declare_parameter("robot_ready_state_topic", "/robot_ready_state");  // 添加机械臂准备状态话题参数
    this->declare_parameter("robot_target_pos_topic", "/robot_target_pose");
    this->declare_parameter("timer_period_ms", 100);
    this->declare_parameter("robot_control_service", "/robot_control");
    //this->declare_parameter("detect_res_service", "/aruco_detection/get_result");  // 修改为获取标定marker检测结果的服务
    //this->declare_parameter("calib_src_img_service", "/aruco_detection/get_aruco_src_img");  // 添加图像请求服务
    this->declare_parameter("start_calib_service", "/start_calibration");
    this->declare_parameter("use_service_for_points", true);   // 默认通过服务通讯获取标定点
    this->declare_parameter("get_calibration_points_service", "/get_calibration_points");  // 获取标定点数据服务
    this->declare_parameter("get_robot_pose_service", "/get_robot_pose");  // 获取机械臂位姿服务
    this->declare_parameter("camID", camID_);  // 声明相机ID参数
    this->declare_parameter("armID", armID_);  // 声明机械臂ID参数
    this->declare_parameter("eye_on_hand", eye_on_hand_);  // 声明眼在手上参数
    // 获取参数值
    use_service_for_points_ = this->get_parameter("use_service_for_points").as_bool();
    camID_ = this->get_parameter("camID").as_int();  // 获取相机ID参数
    armID_ = this->get_parameter("armID").as_int();  // 获取机械臂ID参数
    eye_on_hand_ = this->get_parameter("eye_on_hand").as_bool();
    LOG_INFO("标定点获取方式: %s", 
                use_service_for_points_ ? "服务通讯获取" : "本地文件加载");
    
    LOG_INFO("相机：%d, 机械臂：%d, 标定模式: %s", camID_, armID_, eye_on_hand_ ? "眼在手上" : "眼在手外");
}

/**
 * @brief 初始化订阅器
 */
void HandEyeCalibNode::initSubscribers() 
{
    // 获取参数
    std::string robot_ready_state_topic = this->get_parameter("robot_ready_state_topic").as_string();  // 获取机械臂准备状态话题参数
    // std::string robot_pose_topic = this->get_parameter("robot_pose_topic").as_string();
    basros::RosCommInfo comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_ARM_CURRENT_POSE, camID_, armID_);
	std::string robot_pose_topic = comm_info.name;
    std::string robot_run_state_topic = this->get_parameter("robot_run_state_topic").as_string();

    robot_ready_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(robot_ready_state_topic, 10,
        std::bind(&HandEyeCalibNode::robotReadyStateCallback, this, std::placeholders::_1));

    robot_pose_sub_ = this->create_subscription<custom_msgs_comm::msg::RobotStdPose>(robot_pose_topic, 10,
        std::bind(&HandEyeCalibNode::robotPoseCallback, this, std::placeholders::_1));
        
    robot_run_state_topic_sub_ = this->create_subscription<custom_msgs_comm::msg::BoolStamped>(robot_run_state_topic, 10,
        std::bind(&HandEyeCalibNode::robotStatusCallback, this, std::placeholders::_1));
    
    LOG_INFO("订阅器初始化完成");
    // LOG_INFO("  - Aruco标记话题: %s", aruco_marker_topic.c_str());
    LOG_INFO("  - 机械臂位姿话题: %s", robot_pose_topic.c_str());
    LOG_INFO("  - 机械臂运行状态话题: %s", robot_run_state_topic.c_str());
    LOG_INFO("  - 机械臂准备状态话题: %s", robot_ready_state_topic.c_str());
}

/**
 * @brief 初始化发布器
 */
void HandEyeCalibNode::initPublishers() 
{
    // 获取参数
    std::string robot_target_pos_topic = this->get_parameter("robot_target_pos_topic").as_string();
    
    // 创建发布器
    robot_target_pub_ = this->create_publisher<custom_msgs_comm::msg::RobotStdPose>(robot_target_pos_topic, 10);
        
    LOG_INFO("发布器初始化完成");
    LOG_INFO("  - 机械臂目标话题: %s", robot_target_pos_topic.c_str());
}

/**
 * @brief 初始化服务客户端
 */
void HandEyeCalibNode::initServiceClients() 
{
    LOG_INFO("开始初始化服务客户端");

    basros::RosCommInfo comm_info;
    std::string detect_res_service = "/marker_detection/get_result";
    std::string calib_src_img_service = "/marker_detection/get_marker_src_img";
    comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_MARKER_RESULTS, camID_, armID_);
    detect_res_service = comm_info.name;
    comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_MARKER_SRC_IMAGE, camID_, armID_);
    calib_src_img_service = comm_info.name;
    // 获取参数
    std::string robot_control_service = this->get_parameter("robot_control_service").as_string();
    std::string get_calibration_points_service = this->get_parameter("get_calibration_points_service").as_string();  // 获取标定点数据服务
    // std::string detect_res_service = this->get_parameter("detect_res_service").as_string();  // 修改为获取标定marker检测结果的服务
    // std::string calib_src_img_service = this->get_parameter("calib_src_img_service").as_string();  // 添加图像请求服务
    std::string get_robot_pose_service = this->get_parameter("get_robot_pose_service").as_string();  // 获取机械臂位姿服务
    
    LOG_INFO("机器人控制服务名称: %s", robot_control_service.c_str());
    LOG_INFO("获取标定点数据服务名称: %s", get_calibration_points_service.c_str());
    LOG_INFO("标定marker检测服务名称: %s", detect_res_service.c_str());
    LOG_INFO("图像请求服务名称: %s", calib_src_img_service.c_str());
    LOG_INFO("获取机械臂位姿服务名称: %s", get_robot_pose_service.c_str());
    
    // 创建服务客户端 (使用 std_srvs::srv::Trigger 而不是 ProjectControl)
    robot_control_client_ = this->create_client<std_srvs::srv::Trigger>(robot_control_service);
    get_calibration_points_client_ = this->create_client<custom_msgs_comm::srv::GetCalibrationPoints>(get_calibration_points_service);  // 获取标定点数据服务客户端
    detect_res_service_client_ = this->create_client<custom_msgs_comm::srv::GetMarkerDetection>(detect_res_service);  // 修改为获取标定marker检测结果的服务
    calib_src_img_client_ = this->create_client<custom_msgs_comm::srv::GetMarkerSrcImage>(calib_src_img_service);  // 添加标定源图像请求服务客户端
    get_robot_pose_client_ = this->create_client<custom_msgs_comm::srv::GetRobotPose>(get_robot_pose_service);  // 获取机械臂位姿服务客户端
    
    // 添加服务客户端创建检查
    if (get_calibration_points_client_) {
        LOG_INFO("标定点数据服务客户端创建成功");
    } else {
        LOG_ERROR("标定点数据服务客户端创建失败");
    }
    
    if (get_robot_pose_client_) {
        LOG_INFO("机械臂位姿服务客户端创建成功");
    } else {
        LOG_ERROR("机械臂位姿服务客户端创建失败");
    }
    
    LOG_INFO("服务客户端创建完成");    
    LOG_INFO("服务客户端初始化完成");
    LOG_INFO("  - 机械臂控制服务: %s", robot_control_service.c_str());
    LOG_INFO("  - 获取标定点数据服务: %s", get_calibration_points_service.c_str());
    LOG_INFO("  - 标定marker检测服务: %s", detect_res_service.c_str());
    LOG_INFO("  - 图像请求服务: %s", calib_src_img_service.c_str());
    LOG_INFO("  - 获取机械臂位姿服务: %s", get_robot_pose_service.c_str());
}

/**
 * @brief 初始化服务服务器
 */
void HandEyeCalibNode::initServiceServers() 
{
    LOG_INFO("开始初始化服务服务器");
    
    // 获取参数
    std::string start_calib_service = this->get_parameter("start_calib_service").as_string();
    
    LOG_INFO("启动标定服务名称: %s", start_calib_service.c_str());
    
    // 注释掉服务服务器创建，因为不再使用startCalibrationServiceCallback
    // start_calib_service_server_ = this->create_service<std_srvs::srv::Trigger>(start_calib_service,
    //     std::bind(&HandEyeCalibNode::startCalibrationServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
        
    LOG_INFO("服务服务器创建完成");    LOG_INFO("  - 启动标定服务: %s", start_calib_service.c_str());
}

/**
 * @brief 初始化系统配置客户端
 */
void HandEyeCalibNode::initParametersClient() 
{
    LOG_INFO("开始初始化系统配置客户端");

    // 避免当前节点多次添加到executor，Node '/aruco_detect_node_1' has already been added to an executor.
	parameters_node_ = std::make_shared<rclcpp::Node>("hand_eye_calib_params_node");
    
    sys_config_node_name_ = "sys_config_ros_node";
	sys_config_client_ = std::make_shared<rclcpp::SyncParametersClient>(
		parameters_node_,  // 当前节点
		sys_config_node_name_  // 目标节点名称
		);

    LOG_INFO("系统配置客户端初始化完成");
    // 等待参数服务可用
	if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
	{
		LOG_ERROR("无法连接到系统配置参数服务");
	}
}

/**
 * @brief 初始化定时器
 */
void HandEyeCalibNode::initTimer() 
{
    // 获取参数
    int timer_period_ms = this->get_parameter("timer_period_ms").as_int();
    
    // 创建定时器
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms),
        std::bind(&HandEyeCalibNode::timerCallback, this));
        
    LOG_INFO("定时器初始化完成，周期: %d ms", timer_period_ms);
}

/**
 * @brief 机械臂位姿回调函数
 * @param msg 机械臂位姿消息
 */
void HandEyeCalibNode::robotPoseCallback(const custom_msgs_comm::msg::RobotStdPose::SharedPtr msg) 
{
    // 保存当前机械臂位姿
    if (msg != nullptr && msg->arm_id == armID_) {
        current_robot_pose_ = msg->std_pose;
    }
}

/**
 * @brief 机械臂状态回调函数
 * @param msg 机械臂状态消息
 */
void HandEyeCalibNode::robotStatusCallback(const custom_msgs_comm::msg::BoolStamped::SharedPtr msg) 
{
    // 使用Bool消息的data字段来判断状态
    // true表示机械臂已移动到位，false表示机械臂未移动到位
    bool is_move_ready = msg->data;
    LOG_INFO("机械臂状态变化: is_move_ready=%s, calib_started_=%s, robot_ready_=%s, current_point_index_=%d", 
                  is_move_ready ? "true" : "false", 
                  calib_started_ ? "true" : "false", 
                  robot_ready_ ? "true" : "false", 
                  current_point_index_);
    if (!robot_ready_)    
    {
        LOG_ERROR("机械臂还未初始化准备好！");
        return;
    }
    if (!calib_started_) 
    {
        LOG_ERROR("还未启动标定流程！");
        return;
    }
    if (is_move_ready) 
    {
        if ( msg->header.frame_id != calib_target_frame_)
        {
            LOG_WARN("标定点目标框架 %s 与当前框架 %s 不匹配！", calib_target_frame_.c_str(), msg->header.frame_id.c_str());
            return;
        }
        LOG_INFO("机械臂移动到位，当前标定点索引: %d", current_point_index_);
        robot_moved_ = true;// 机械臂移动到位
        requestArucoDetectionResult();// 请求标定marker检测结果
    } else {
        LOG_WARN("机械臂状态未匹配任何条件: is_move_ready=%s, calib_started_=%s, robot_ready_=%s", 
                      is_move_ready ? "true" : "false", 
                      calib_started_ ? "true" : "false", 
                      robot_ready_ ? "true" : "false");
        moveToNextPoint();// 如果机械臂移动不到位这种情况，直接跳过当前点，继续发送下一个标定点
    }
}

/**
 * @brief 机械臂准备状态回调函数
 * @param msg 机械臂准备状态消息
 */
void HandEyeCalibNode::robotReadyStateCallback(const std_msgs::msg::Bool::SharedPtr msg) 
{
    // 使用Bool消息的data字段来判断状态
    // true表示机械臂已准备好，false表示机械臂未准备好
    bool is_ready = msg->data;
    LOG_INFO("机械臂准备状态变化: is_ready=%s, calib_started_=%s, robot_ready_=%s, current_point_index_=%d", 
                  is_ready ? "true" : "false", 
                  calib_started_ ? "true" : "false", 
                  robot_ready_ ? "true" : "false", 
                  current_point_index_);
    robot_ready_ = is_ready;  //重置robot_ready_状态 
    if (is_ready) 
    {
        if (!calib_started_) 
        {
            LOG_INFO("收到机械臂准备OK信号，开始启动标定流程...");
            // 启动标定流程
            robot_poses_.clear();// 清空之前的数据
            marker_positions_.clear();
            calib_started_ = true;
            robot_moved_ = false;  // 重置robot_moved_状态
            current_point_index_ = 0;  // 重置当前标定点索引
            successful_points_count_ = 0;  // 重置成功标定点计数器
            // 发送第一个标定点数据
            LOG_INFO("发送第一个标定点数据，索引: %d", current_point_index_);
            handeyecalib::RobotPoseData target_pose = robot_pos_mgr_.getDataPoint(current_point_index_);
            sendRobotTargetPose(target_pose);
        }else{
            LOG_ERROR("机械臂已准备好，但标定流程状态异常！");
        }
    }else {
        LOG_WARN("机械臂还未准备好，无法进入标定流程！");
    }
}

/**
 * @brief 初始化头部电机角度订阅器
 */
void HandEyeCalibNode::initHeadMotorAngleSubscriber() 
{
  // 初始化当前头部电机角度
  current_head_angles_.resize(2, 0.0);  // 默认为0
  
  // 创建订阅器
  head_motor_angles_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/head_motor_angles", 10,
      std::bind(&HandEyeCalibNode::headMotorAngleCallback, this, std::placeholders::_1));
  
  LOG_INFO("头部电机角度订阅器初始化完成");
}

/**
 * @brief 头部电机角度回调函数
 * @param msg 头部电机角度消息
 */
void HandEyeCalibNode::headMotorAngleCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
{
  std::lock_guard<std::mutex> lock(head_angles_mutex_);
  
  // 检查数组大小是否足够
  if (msg->data.size() < 2) {
      LOG_WARN("头部电机角度数据不足，需要2个值，实际收到%zu个", msg->data.size());
      return;
  }
  
  // 更新当前头部电机角度
  current_head_angles_.resize(2);
  current_head_angles_[0] = msg->data[0];  // 俯仰角
  current_head_angles_[1] = msg->data[1];  // 偏航角
  
  LOG_INFO("接收到头部电机角度 - 俯仰角: %.2f°, 偏航角: %.2f°", current_head_angles_[0], current_head_angles_[1]);
}

/**
 * @brief 定时器回调函数
 */
void HandEyeCalibNode::timerCallback() 
{
    LOG_DEBUG("定时器回调函数被调用");
    // 定时器主要用于触发状态检查，实际处理在回调函数中完成
    // 这里可以添加一些周期性任务
}

/**
 * @brief 发送启动标定命令到机械臂
 */
void HandEyeCalibNode::sendStartCalibCommandToRobot() 
{
    calib_started_ = false;
    robot_ready_ = false;
    // 创建服务请求 (使用 std_srvs::srv::Trigger 而不是 ProjectControl)
    auto service_request = std::make_shared<std_srvs::srv::Trigger::Request>(); 
    LOG_INFO("准备发送机械臂控制服务请求");
    // 异步调用机械臂控制服务，提高可靠性
    robot_control_client_->async_send_request(
        service_request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            try {
                LOG_INFO("收到机械臂控制服务响应");
                auto response = future.get();
                if (response->success) 
                {
                    LOG_INFO("机械臂标定准备成功");
                    // 机械臂准备成功，等待robotReadyStateCallback中接收到准备OK信号后再启动标定流程
                    LOG_INFO("等待机械臂准备OK信号启动标定流程...");

                } else {
                    LOG_ERROR("机械臂准备标定失败: %s", response->message.c_str());
                }
            } catch (const std::exception& e) {
                LOG_ERROR("调用机械臂控制服务时发生异常: %s", e.what());
            }
        });
    LOG_INFO("异步机械臂控制服务请求已发送");
}

/**
 * @brief 发送机械臂目标位姿
 * @param pose 目标位姿（单位：毫米）
 */
void HandEyeCalibNode::sendRobotTargetPose(const handeyecalib::RobotPoseData& pose) 
{
    // 检查位姿数据是否有效
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z) ||
        !std::isfinite(pose.rx) || !std::isfinite(pose.ry) || !std::isfinite(pose.rz)) {
        LOG_ERROR("发送的机械臂目标位姿数据无效: x=%.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f", 
            pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
        return;
    }
    
    auto msg = custom_msgs_comm::msg::RobotStdPose();
    msg.arm_id = armID_;
    msg.std_pose.header.stamp = this->now();
    msg.std_pose.header.frame_id = calib_target_frame_;
    
    // 设置位置（单位：mm）
    msg.std_pose.pose.position.x = pose.x;  
    msg.std_pose.pose.position.y = pose.y;
    msg.std_pose.pose.position.z = pose.z;
    
    // 将欧拉角(rx, ry, rz)转换为四元数
#if 0 //先临时屏蔽，需要时再开放配置
    tf2::Quaternion quaternion;
    quaternion.setRPY(pose.rx * M_PI / 180.0, pose.ry * M_PI / 180.0, pose.rz * M_PI / 180.0);  // 将角度转换为弧度
    msg.pose.orientation.x = quaternion.x();
    msg.pose.orientation.y = quaternion.y();
    msg.pose.orientation.z = quaternion.z();
    msg.pose.orientation.w = quaternion.w();
#endif
    // 直接使用欧拉角作为姿态表示
    msg.std_pose.pose.orientation.x = pose.rx;
    msg.std_pose.pose.orientation.y = pose.ry;
    msg.std_pose.pose.orientation.z = pose.rz;
    msg.std_pose.pose.orientation.w = 0.0;

    robot_target_pub_->publish(msg);
    
    LOG_INFO("发送机械臂目标位姿: x=%.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f",
        pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
}

/**
 * @brief 发送标定结束命令
 */
void HandEyeCalibNode::sendEndCalibCommand() 
{
    calib_started_ = false;
    LOG_WARN("标定结束");
}

/**
 * @brief 请求标定marker检测结果
 */
void HandEyeCalibNode::requestArucoDetectionResult() 
{
    LOG_INFO("请求标定marker检测结果，当前标定点索引: %d", current_point_index_);
    
    // 检查服务客户端是否可用
    LOG_INFO("检查标定marker检测服务是否可用...");
    if (!detect_res_service_client_->wait_for_service(std::chrono::seconds(1))) 
    {
        LOG_ERROR("无法连接到标定marker检测服务");
        return;
    }
    LOG_INFO("标定marker检测服务可用");
    
    // 创建服务请求
    auto service_request = std::make_shared<custom_msgs_comm::srv::GetMarkerDetection::Request>();
    service_request->request_id = "calib_point_" + std::to_string(current_point_index_);
    
    LOG_INFO("发送Marker检测服务请求，请求ID: %s", service_request->request_id.c_str());
    
    // 调用Marker检测服务
    detect_res_service_client_->async_send_request(
        service_request,
        [this](rclcpp::Client<custom_msgs_comm::srv::GetMarkerDetection>::SharedFuture future) {
            try 
            {
                LOG_INFO("收到Marker检测服务响应");
                auto response = future.get();
                LOG_INFO("服务响应内容 - success: %s, message: %s", response->success ? "true" : "false", response->message.c_str());
                if (response->success) 
                {
                    LOG_INFO("成功获取Aruco标记检测结果");
                    
                    // 更新当前Aruco标记数据
                    if (response->position.size() >= 3) 
                    {
                        current_aruco_marker_.header.stamp = response->stamp;
                        current_aruco_marker_.header.frame_id = response->frame_id;
                        current_aruco_marker_.pose.position.x = response->position[0];
                        current_aruco_marker_.pose.position.y = response->position[1];
                        current_aruco_marker_.pose.position.z = response->position[2];
                        
                        // 如果响应中包含方向信息，则也保存方向信息
                        if (response->orientation.size() >= 4) 
                        {
                            current_aruco_marker_.pose.orientation.x = response->orientation[0];
                            current_aruco_marker_.pose.orientation.y = response->orientation[1];
                            current_aruco_marker_.pose.orientation.z = response->orientation[2];
                            current_aruco_marker_.pose.orientation.w = response->orientation[3];
                        } 
                        else 
                        {
                            // 如果没有方向信息，则设置为默认值
                            current_aruco_marker_.pose.orientation.x = 0.0;
                            current_aruco_marker_.pose.orientation.y = 0.0;
                            current_aruco_marker_.pose.orientation.z = 0.0;
                            current_aruco_marker_.pose.orientation.w = 1.0;
                        }
                        
                        LOG_INFO("Aruco标记位置: x=%.3f, y=%.3f, z=%.3f", 
                            current_aruco_marker_.pose.position.x, current_aruco_marker_.pose.position.y, current_aruco_marker_.pose.position.z);
                        
                        // 保存图像数据
                        current_calib_src_img_ = response->image;
                        current_calib_render_img_ = response->result_image;
                        
                        // 请求机械臂位姿数据
                        requestRobotPose();
                    } else {
                        LOG_ERROR("标定marker检测结果数据不完整，位置数据大小: %d", static_cast<int>(response->position.size()));
                        // 即使Aruco检测失败，也移动到下一个标定点
                        moveToNextPoint();
                    }
                } else {
                    LOG_ERROR("获取标定marker检测结果失败: %s", response->message.c_str());
                    // 即使Aruco检测失败，也移动到下一个标定点
                    moveToNextPoint();
                }
            } catch (const std::exception& e) {
                LOG_ERROR("处理标定marker检测结果时发生异常: %s", e.what());
                // 发生异常时也移动到下一个标定点
                moveToNextPoint();
            }
        });
}

/**
 * @brief 请求机械臂位姿数据
 */
void HandEyeCalibNode::requestRobotPose()
{
    LOG_INFO("请求机械臂位姿数据，当前标定点索引: %d", current_point_index_);
    
    // 检查服务客户端是否可用
    LOG_INFO("检查机械臂位姿服务是否可用...");
    if (!get_robot_pose_client_->wait_for_service(std::chrono::seconds(1))) 
    {
        LOG_ERROR("无法连接到机械臂位姿服务");
        // 即使无法连接到服务，也移动到下一个标定点
        moveToNextPoint();
        return;
    }
    LOG_INFO("机械臂位姿服务可用");
    
    // 创建服务请求
    auto service_request = std::make_shared<custom_msgs_comm::srv::GetRobotPose::Request>();
    service_request->request_id = std::to_string(armID_);
    
    LOG_INFO("发送机械臂位姿服务请求，请求ID: %s", service_request->request_id.c_str());
    
    // 调用机械臂位姿服务
    get_robot_pose_client_->async_send_request(
        service_request,
        [this](rclcpp::Client<custom_msgs_comm::srv::GetRobotPose>::SharedFuture future) {
            try 
            {
                LOG_INFO("收到机械臂位姿服务响应");
                auto response = future.get();
                LOG_INFO("服务响应内容 - success: %s, message: %s", response->success ? "true" : "false", response->message.c_str());
                if (response->success) 
                {
                    LOG_INFO("成功获取机械臂位姿数据");
                    
                    // 更新当前机械臂位姿数据
                    current_robot_pose_.header.stamp = response->stamp;
                    current_robot_pose_.header.frame_id = "base_link";
                    current_robot_pose_.pose.position.x = response->x;
                    current_robot_pose_.pose.position.y = response->y;
                    current_robot_pose_.pose.position.z = response->z;
                    current_robot_pose_.pose.orientation.x = response->rx;
                    current_robot_pose_.pose.orientation.y = response->ry;
                    current_robot_pose_.pose.orientation.z = response->rz;
                    current_robot_pose_.pose.orientation.w = 0.0;
                    
                    LOG_INFO("机械臂位姿: x=%.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f", 
                        current_robot_pose_.pose.position.x, current_robot_pose_.pose.position.y, current_robot_pose_.pose.position.z,
                        current_robot_pose_.pose.orientation.x, current_robot_pose_.pose.orientation.y, current_robot_pose_.pose.orientation.z);
                    
                    // 记录Aruco检测结果和机械臂位姿的时间戳
                    auto aruco_timestamp = current_aruco_marker_.header.stamp;
                    auto robot_pose_timestamp = current_robot_pose_.header.stamp;
                    
                    // 计算时间差
                    // 使用rclcpp的时间计算方法
                    auto time_diff = rclcpp::Time(robot_pose_timestamp) - rclcpp::Time(aruco_timestamp);
                    auto time_diff_ms = time_diff.nanoseconds() / 1000000; // 转换为毫秒
                    
                    LOG_INFO("Aruco检测时间戳: %d.%09d, 机械臂位姿时间戳: %d.%09d, 时间差: %ld ms",
                        aruco_timestamp.sec, aruco_timestamp.nanosec,
                        robot_pose_timestamp.sec, robot_pose_timestamp.nanosec, time_diff_ms);
                    
                    // 保存标定数据点（包含Aruco检测结果和机械臂位姿）
                    saveCalibrationDataPoint();
                } else {
                    LOG_ERROR("获取机械臂位姿数据失败: %s", response->message.c_str());
                }
                
                // 无论成功与否，都移动到下一个标定点
                LOG_INFO("准备移动到下一个标定点");
                moveToNextPoint();
            } catch (const std::exception& e) {
                LOG_ERROR("处理机械臂位姿数据时发生异常: %s", e.what());
                // 发生异常时也移动到下一个标定点
                moveToNextPoint();
            }
        });
}

/**
 * @brief 请求标定源图像
 */
void HandEyeCalibNode::requestImage() 
{
    LOG_INFO("请求图像，当前标定点索引: %d", current_point_index_);
    
    // 检查服务客户端是否可用
    LOG_INFO("检查图像请求服务是否可用...");
    if (!calib_src_img_client_->wait_for_service(std::chrono::seconds(1))) 
    {
        LOG_ERROR("无法连接到图像请求服务");
        return;
    }
    LOG_INFO("图像请求服务可用");
    
    // 创建服务请求
    auto service_request = std::make_shared<custom_msgs_comm::srv::GetMarkerSrcImage::Request>();
    service_request->request_id = "calib_point_" + std::to_string(current_point_index_);
    
    LOG_INFO("发送标定源图像请求服务，请求ID: %s", service_request->request_id.c_str());
    
    // 调用图像请求服务
    calib_src_img_client_->async_send_request(
        service_request,
        [this](rclcpp::Client<custom_msgs_comm::srv::GetMarkerSrcImage>::SharedFuture future) {
            try 
            {
                LOG_INFO("收到标定源图像请求服务响应");
                auto response = future.get();
                LOG_INFO("服务响应内容 - success: %s, message: %s", response->success ? "true" : "false", response->message.c_str());
                if (response->success) 
                {
                    LOG_INFO("成功获取标定源图像");
                    // 打印图像信息
                    LOG_INFO("图像信息: 尺寸 %dx%d, 编码 %s", response->image.width, response->image.height, response->image.encoding.c_str());
                    
                    // 保存图像到文件
                    try 
                    {
                        // 将ROS图像消息转换为OpenCV图像
                        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(response->image, response->image.encoding);
                        
                        // 保存图像
                        saveCalibSourceImage(cv_ptr, current_point_index_);
                    } catch (const cv_bridge::Exception& e) {
                        LOG_ERROR("图像转换失败: %s", e.what());
                    } catch (const std::exception& e) {
                        LOG_ERROR("保存图像时发生异常: %s", e.what());
                    }
                } else {
                    LOG_ERROR("获取标定源图像失败: %s", response->message.c_str());
                }
            } catch (const std::exception& e) {
                LOG_ERROR("处理标定源图像请求时发生异常: %s", e.what());
            }
        });
}

/**
 * @brief 检查机械臂位姿数据是否有效
 * @param pose 机械臂位姿数据
 * @return 是否有效
 */
bool HandEyeCalibNode::isRobotPoseValid(const geometry_msgs::msg::PoseStamped& pose) 
{
    // 检查位置数据是否有效（非NaN和非无穷大）
    if (!std::isfinite(pose.pose.position.x) || 
        !std::isfinite(pose.pose.position.y) || 
        !std::isfinite(pose.pose.position.z)) {
        return false;
    }
    
    // 检查姿态数据是否有效（非NaN和非无穷大）
    if (!std::isfinite(pose.pose.orientation.x) || 
        !std::isfinite(pose.pose.orientation.y) || 
        !std::isfinite(pose.pose.orientation.z) || 
        !std::isfinite(pose.pose.orientation.w)) {
        return false;
    }
    
    return true;
}

/**
 * @brief 执行标定计算
 */
void HandEyeCalibNode::performCalibration() 
{
    LOG_INFO("开始执行标定计算，使用 %d 个数据点", successful_points_count_);
    
    // 检查标定点数量是否足够
    if (successful_points_count_ < 4) {
        LOG_ERROR("成功采集的标定点数量不足，无法执行标定计算: %d < 4", successful_points_count_);
        return;
    }
    
    try 
    {
        for (size_t i = 0; i < robot_poses_.size(); ++i) {
            LOG_INFO("标定数据点 %d: 机器人位姿, x=%.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f", 
                i, robot_poses_[i][0], robot_poses_[i][1], robot_poses_[i][2],
                robot_poses_[i][3], robot_poses_[i][4], robot_poses_[i][5]);
        }
        for (size_t i = 0; i < marker_positions_.size(); ++i) {
            LOG_INFO("标定数据点 %d: 标记位置, x=%.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f", 
                i, marker_positions_[i][0], marker_positions_[i][1], marker_positions_[i][2],
                marker_positions_[i][3], marker_positions_[i][4], marker_positions_[i][5]);
        }
        // 计算手眼标定矩阵
        handeyecalib::CalibRes result = handeyecalib::computeHandEyeCalibrationByCv(
                robot_poses_, marker_positions_, eye_on_hand_);
        
        // 将当前头部电机角度添加到标定结果中
        {
            std::lock_guard<std::mutex> lock(head_angles_mutex_);
            result.head_motor_angles = {current_head_angles_[0], current_head_angles_[1]};
            LOG_INFO("标定结果中记录头部电机角度 - 俯仰角: %.2f°, 偏航角: %.2f°", 
                current_head_angles_[0], current_head_angles_[1]);
        }
        // 保存标定结果到系统配置
        //std::string bas_config_data_path = calib_config_.getOutputDataDir() + "/../.."; // 回退到bas_config_data根目录
        std::string install_path_str = basmodule::get_install_dir();
	    std::filesystem::path install_path(install_path_str);
	    std::filesystem::path config_path = install_path / "bas_config_data";
        std::string bas_config_data_path = config_path.string();
        uint8_t cam_id = camID_; // 需要根据实际情况设置相机ID
        uint8_t arm_id = armID_; // 需要根据实际情况设置机械臂ID

        // 分析标定质量
        handeyecalib::QualityMetrics metrics = handeyecalib::analyzeCalibrationQuality(
            robot_poses_, marker_positions_, result, eye_on_hand_);
        
        // 创建完整的ArmCalibInfo对象并填充所有必需的参数
        handeyecalib::ArmCalibInfo arm_calib_info;
        RosComm::getArmCalibInfoFromServer(sys_config_client_, cam_id, arm_id, arm_calib_info);
        arm_calib_info.arm_id = arm_id;
        
        // 设置标定信息
        handeyecalib::CalibInfo calib_info;
        result.offset_compensation = arm_calib_info.calib_info.calib_res.offset_compensation;
        calib_info.calib_method = "Park-Martin";  // 或其他适当的标定方法
        calib_info.calib_res = result;
        calib_info.quality_metrics = metrics;
        calib_info.timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
        arm_calib_info.setCalibInfo(calib_info);
        
        // 使用系统标定数据管理器保存标定结果
        if (handeyecalib::SysCalibMgr::getInstance().saveArmCalibInfo(bas_config_data_path, cam_id, arm_calib_info)) {
            LOG_INFO("标定结果已保存到: 相机%d_机械臂%d", static_cast<int>(cam_id), static_cast<int>(arm_id));
            if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
		        LOG_ERROR("无法连接到系统配置参数服务");
            else
            {
                if (RosComm::SetArmCalibInfoToServer(sys_config_client_, camID_, armID_, arm_calib_info))
                    LOG_INFO("设置标定结果到参数服务器成功");
                else
                    LOG_ERROR("设置标定结果到参数服务器失败");
            }
        } 
        else 
        {
            LOG_ERROR("保存标定结果失败");
        }
        LOG_INFO("标定质量分析结果:");
        LOG_INFO("  - 重投影误差: %.6f mm", metrics.reprojection_error);
        LOG_INFO("  - 平移误差: %.6f mm", metrics.translation_error);
        LOG_INFO("  - 旋转误差: %.6f degrees", metrics.rotation_error);
        LOG_INFO("  - 矩阵条件数: %.6f", metrics.condition_number);
        LOG_INFO("  - 数据点数量: %d points", metrics.data_point_count);
        
        // 分析每个标定点的重投影误差
        // analyzePerPointReprojectionError(robot_poses_, marker_positions_, result.cam_to_base_transform);
        
    } catch (const std::exception& e) {
        LOG_ERROR("标定计算失败: %s", e.what());
    }
}

/**
 * @brief 分析每个标定点的重投影误差 * @param robot_poses 机器人位姿列表
 * @param marker_positions 标记位置列表
 * @param cam_to_base_transform 相机到基座的变换矩阵
 */
void HandEyeCalibNode::analyzePerPointReprojectionError(
    const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions,
    const cv::Mat& cam_to_base_transform) 
{
    LOG_INFO("开始分析每个标定点的重投影误差");
    
    if (robot_poses.size() != marker_positions.size()) 
    {
        LOG_ERROR("机器人位姿和标记位置数量不匹配");
        return;
    }
    
    int n = robot_poses.size();
    if (n == 0) {
        LOG_WARN("没有标定点数据");
        return;
    }
    
    LOG_INFO("标定点详细分析:");
    LOG_INFO("序号\t预测坐标(X,Y,Z)\t\t实际坐标(X,Y,Z)\t\t误差(mm)");
    LOG_INFO("----\t------------------\t\t------------------\t\t--------");
    
    double total_error = 0.0;
    std::vector<double> errors;
    errors.reserve(n);
    
    for (int i = 0; i < n; i++) 
    {
        // 将机器人位姿转换为齐次变换矩阵
        cv::Mat T_base2end = handeyecalib::poseToHomogeneousMatrix(robot_poses[i]);
        
        // 标记在相机坐标系下的位置
        cv::Mat marker_cam = (cv::Mat_<double>(4, 1) << 
            marker_positions[i][0], marker_positions[i][1], marker_positions[i][2], 1.0);
        
        // 通过标定矩阵变换到基座坐标系（理论值）
        cv::Mat marker_base_predicted = cam_to_base_transform * marker_cam;
        
        // 通过机器人位姿直接计算标记在基座坐标系下的位置（实际值）
        cv::Mat marker_base_actual = T_base2end * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
        
        // 提取坐标值
        double pred_x = marker_base_predicted.at<double>(0, 0);
        double pred_y = marker_base_predicted.at<double>(1, 0);
        double pred_z = marker_base_predicted.at<double>(2, 0);
        
        double actual_x = marker_base_actual.at<double>(0, 0);
        double actual_y = marker_base_actual.at<double>(1, 0);
        double actual_z = marker_base_actual.at<double>(2, 0);
        
        // 计算位置误差
        double dx = pred_x - actual_x;
        double dy = pred_y - actual_y;
        double dz = pred_z - actual_z;
        
        double error = sqrt(dx*dx + dy*dy + dz*dz);
        errors.push_back(error);
        total_error += error;
        
        // 输出每个点的详细信息
        LOG_INFO("%d\t(%.3f, %.3f, %.3f)\t\t(%.3f, %.3f, %.3f)\t\t%.6f", 
            i+1, pred_x, pred_y, pred_z, actual_x, actual_y, actual_z, error);
    }
    
    // 计算统计信息
    double mean_error = total_error / n;
    double max_error = *std::max_element(errors.begin(), errors.end());
    double min_error = *std::min_element(errors.begin(), errors.end());
    
    // 计算标准差
    double sum_square_diff = 0.0;
    for (double err : errors) {
        sum_square_diff += (err - mean_error) * (err - mean_error);
    }
    double std_error = sqrt(sum_square_diff / n);
    
    LOG_INFO("----\t------------------\t\t------------------\t\t--------");
    LOG_INFO("统计信息:");
    LOG_INFO("平均误差: %.6f mm", mean_error);
    LOG_INFO("最大误差: %.6f mm", max_error);
    LOG_INFO("最小误差: %.6f mm", min_error);
    LOG_INFO("标准差:   %.6f mm", std_error);
    LOG_INFO("每个标定点的重投影误差分析完成");
}
/**
 * @brief 保存标定数据点
 */
void HandEyeCalibNode::saveCalibrationDataPoint() 
{
    // 检查机械臂位姿数据是否有效
    if (!isRobotPoseValid(current_robot_pose_)) 
    {
        LOG_WARN("机械臂位姿数据无效，舍弃该数据点");
        return;
    }
    if (0)
    {
        // 检查机械臂位姿数据的时间戳，确保它是最新的
        auto now = this->now();
        auto time_diff = now - current_robot_pose_.header.stamp;
        auto time_diff_ms = time_diff.nanoseconds() / 1000000; // 转换为毫秒
        
        // 如果时间差超过1秒，认为位姿数据可能不是最新的
        if (time_diff_ms > 1000) {
            LOG_WARN("机械臂位姿数据可能不是最新的，时间差: %ld ms", time_diff_ms);
        }
    }
    #if 0
        // 在保存坐标点数据时，先发送服务通讯请求拿到并保存该坐标点处的标定源图像
        requestImage();
        LOG_INFO("已请求标定源图像，等待图像数据..."); 
    #endif
       
    // 从标定marker检测结果中获取图像数据并保存
    try 
    {
        // 将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImagePtr cv_src_ptr = cv_bridge::toCvCopy(current_calib_src_img_, current_calib_src_img_.encoding);
        cv_bridge::CvImagePtr cv_render_ptr = cv_bridge::toCvCopy(current_calib_render_img_, current_calib_render_img_.encoding);
        
        // 保存源图像和渲染图像
        saveCalibSourceImage(cv_src_ptr, current_point_index_);
        saveCalibRenderImage(cv_render_ptr, current_point_index_);
    } catch (const cv_bridge::Exception& e) {
        LOG_ERROR("图像转换失败: %s", e.what());
    } catch (const std::exception& e) {
        LOG_ERROR("保存图像时发生异常: %s", e.what());
    }

    // 保存机器人位姿数据 [x, y, z, rx, ry, rz]
    std::vector<double> robot_pose = {
        current_robot_pose_.pose.position.x,  // 根据单位配置进行转换
        current_robot_pose_.pose.position.y,
        current_robot_pose_.pose.position.z,
        current_robot_pose_.pose.orientation.x,  // rx (简化处理)
        current_robot_pose_.pose.orientation.y,  // ry (简化处理)
        current_robot_pose_.pose.orientation.z   // rz (简化处理)
    };

     // 打印原始机械臂坐标数据
    LOG_INFO("机械臂位姿数据 (mm): 位置(x=%.6f, y=%.6f, z=%.6f), 姿态(rx=%.6f, ry=%.6f, rz=%.6f, w=%.6f)",
        current_robot_pose_.pose.position.x, current_robot_pose_.pose.position.y, current_robot_pose_.pose.position.z,
        current_robot_pose_.pose.orientation.x, current_robot_pose_.pose.orientation.y, 
        current_robot_pose_.pose.orientation.z, current_robot_pose_.pose.orientation.w);

    // 打印原始Aruco标记坐标数据
    LOG_INFO("Aruco标记位置数据 (mm): (x=%.6f, y=%.6f, z=%.6f, rx=%.6f, ry=%.6f, rz=%.6f)",
        current_aruco_marker_.pose.position.x, current_aruco_marker_.pose.position.y, current_aruco_marker_.pose.position.z,
        current_aruco_marker_.pose.orientation.x, current_aruco_marker_.pose.orientation.y, current_aruco_marker_.pose.orientation.z);
    
    // 保存标记位置数据 [x, y, z]
    std::vector<double> marker_position = {
        current_aruco_marker_.pose.position.x,  
        current_aruco_marker_.pose.position.y,
        current_aruco_marker_.pose.position.z,
        current_aruco_marker_.pose.orientation.x,
        current_aruco_marker_.pose.orientation.y,
        current_aruco_marker_.pose.orientation.z
    };
 
    // 添加到数据集合
    robot_poses_.push_back(robot_pose);
    marker_positions_.push_back(marker_position);
    successful_points_count_++;
    
    LOG_INFO("保存标定数据点 %d: 机器人位姿(%.3f, %.3f, %.3f), 标记位置(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
        current_point_index_, robot_pose[0], robot_pose[1], robot_pose[2],
        marker_position[0], marker_position[1], marker_position[2], marker_position[3], marker_position[4], marker_position[5]);
    
    // 保存坐标点数据到文件
    try {
        // 创建标定点对象
        handeyecalib::CalibrationPoint point;
        point.index = current_point_index_;
        point.timestamp = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;
        point.robot_pose = robot_pose;
        point.marker_position = marker_position;
        
        // 添加到数据收集器
        data_collector_.addCalibrationPoint(point);
        
        // 获取输出目录
        std::string output_dir = calib_config_.getOutputDataDir();
        std::string coordinates_dir = output_dir + "/coordinates";
        
        // 保存坐标点数据
        if (data_collector_.saveCalibrationData(coordinates_dir)) {
            LOG_INFO("坐标点数据已保存到: %s", coordinates_dir.c_str());
        } else {
            LOG_ERROR("保存坐标点数据失败: %s", coordinates_dir.c_str());
        }
    } catch (const std::exception& e) {
        LOG_ERROR("保存坐标点数据时发生异常: %s", e.what());
    }
}

/**
 * @brief 移动到下一个标定点
 */
void HandEyeCalibNode::moveToNextPoint() 
{
    LOG_INFO("当前标定点索引: %d，准备移动到下一个标定点", current_point_index_);
    // 注意：标定点只在启动时获取一次，这里不再重复获取
    // 检查标定点管理器是否已初始化
    size_t point_count = robot_pos_mgr_.getPointCount();
    if (point_count == 0) 
    {
        LOG_ERROR("标定点管理器未初始化或标定点数量为0");
        sendEndCalibCommand();
        return;
    }
    
    current_point_index_++;// 递增标定点索引
    
    // 检查是否已完成所有标定点采集
    if (current_point_index_ >= static_cast<int>(point_count)) 
    {
        LOG_INFO("所有标定点采集完成，共 %d 个点", point_count);
        
        // 检查标定点数量是否足够
        if (point_count < 4) 
        {
            LOG_ERROR("标定点数量不足，无法执行标定计算: %d < 4", static_cast<int>(point_count));
            // 发送标定结束命令
            sendEndCalibCommand();
            return;
        }
        
        // 所有标定点采集完成，发送标定结束命令
        sendEndCalibCommand();
        
        // 如果成功采集的标定点数量满足要求，则执行标定计算
        if (successful_points_count_ >= calib_config_.getMinCalibrationPoints()) 
        {
            LOG_INFO("成功采集的标定点数量满足要求: %d >= %d", successful_points_count_, calib_config_.getMinCalibrationPoints());
            performCalibration();
        } else {
            LOG_WARN("成功采集的标定点数量不足，无法执行标定计算: %d < %d", successful_points_count_, calib_config_.getMinCalibrationPoints());
        }
    } 
    else 
    {
        // 还有更多标定点需要采集
        LOG_INFO("还有更多标定点需要采集，当前索引: %d / %d", current_point_index_, point_count);
        
        // 获取下一个标定点的目标位姿并发送
        handeyecalib::RobotPoseData target_pose = robot_pos_mgr_.getDataPoint(current_point_index_);
        sendRobotTargetPose(target_pose);
        robot_moved_ = false;// 重置机械臂移动状态
    }
}

/**
 * @brief 保存标定源图像到文件
 * @param cv_ptr OpenCV图像指针
 * @param point_index 标定点索引
 */
void HandEyeCalibNode::saveCalibSourceImage(const cv_bridge::CvImagePtr& cv_ptr, int point_index) 
{
    try 
    {
        // 创建文件名
        std::string filename = "calib_point_" + std::to_string(point_index) + ".png";
        
        // 获取输出目录
        std::string output_dir = calib_config_.getOutputDataDir();
        std::string image_dir = output_dir + "/source_img";
        std::string filepath = image_dir + "/" + filename;
        
        // 确保输出目录存在
        std::filesystem::create_directories(image_dir);
        
        // 保存图像
        bool saved = cv::imwrite(filepath, cv_ptr->image);
        if (saved) {
            LOG_INFO("标定源图像已保存到: %s", filepath.c_str());
        } else {
            LOG_ERROR("保存标定源图像失败: %s", filepath.c_str());
        }
    } catch (const std::exception& e) {
        LOG_ERROR("保存图像时发生异常: %s", e.what());
    }
}

/**
 * @brief 保存标定渲染图像到文件
 * @param cv_ptr OpenCV图像指针
 * @param point_index 标定点索引
 */
void HandEyeCalibNode::saveCalibRenderImage(const cv_bridge::CvImagePtr& cv_ptr, int point_index) 
{
    try 
    {
        // 创建文件名
        std::string filename = "calib_point_" + std::to_string(point_index) + ".png";
        
        // 获取输出目录
        std::string output_dir = calib_config_.getOutputDataDir();
        std::string image_dir = output_dir + "/render_img";
        std::string filepath = image_dir + "/" + filename;
        
        // 确保输出目录存在
        std::filesystem::create_directories(image_dir);
        
        // 保存图像
        bool saved = cv::imwrite(filepath, cv_ptr->image);
        if (saved) {
            LOG_INFO("标定渲染图像已保存到: %s", filepath.c_str());
        } else {
            LOG_ERROR("保存标定渲染图像失败: %s", filepath.c_str());
        }
    } catch (const std::exception& e) {
        LOG_ERROR("保存图像时发生异常: %s", e.what());
    }
}

/**
 * @brief 请求标定点数据
 */
void HandEyeCalibNode::requestCalibrationPoints() 
{
    LOG_INFO("请求标定点数据");
    
    // 检查服务客户端是否可用
    if (!get_calibration_points_client_->wait_for_service(std::chrono::seconds(5))) 
    {
        LOG_ERROR("无法连接到获取标定点数据服务");
        return;
    }
    
    // 创建服务请求
    auto calib_points_request = std::make_shared<custom_msgs_comm::srv::GetCalibrationPoints::Request>();
    // calib_points_request->request_id = "hand_eye_calib_request_" + std::to_string(this->now().nanoseconds());
    calib_points_request->request_id = std::to_string(armID_);
    
    LOG_INFO("发送获取标定点数据服务请求，请求ID: %s", calib_points_request->request_id.c_str());
    
    // 异步调用获取标定点数据服务
    get_calibration_points_client_->async_send_request(
        calib_points_request,
        [this](rclcpp::Client<custom_msgs_comm::srv::GetCalibrationPoints>::SharedFuture future) {
            try {
                LOG_INFO("收到获取标定点数据服务响应");
                auto response = future.get();
                LOG_INFO("服务响应结果: success=%s, message=%s", response->success ? "true" : "false", response->message.c_str());
                if (response->success) {
                    LOG_INFO("获取标定点数据成功: %s", response->message.c_str());
                    
                    // 解析服务响应中的标定点数据并刷新到缓存中
                    std::vector<handeyecalib::RobotPoseData> service_data_points;
                    
                    // 从服务响应中获取标定点数据
                    size_t point_count = response->x.size();
                    LOG_INFO("接收到 %d 个标定点数据", static_cast<int>(point_count));
                    
                    for (size_t i = 0; i < point_count; ++i) {
                        handeyecalib::RobotPoseData point;
                        point.x = response->x[i];
                        point.y = response->y[i];
                        point.z = response->z[i];
                        point.rx = response->rx[i];
                        point.ry = response->ry[i];
                        point.rz = response->rz[i];
                        service_data_points.push_back(point);
                    }
                    
                    // 将服务获取到的数据点刷新到CalibRobotPosMgr中
                    robot_pos_mgr_.setCachedDataPoints(service_data_points);
                    
                    LOG_INFO("已将服务获取的 %d 个标定点数据刷新到缓存中", static_cast<int>(service_data_points.size()));
                } else {
                    LOG_ERROR("获取标定点数据失败: %s", response->message.c_str());
                }
            } catch (const std::exception& e) {
                LOG_ERROR("调用获取标定点数据服务时发生异常: %s", e.what());
            }
        });
    
    LOG_INFO("异步获取标定点数据服务请求已发送");
}

/**
 * @brief 检查标定点数量是否满足要求
 * @param point_count 标定点数量
 * @return 是否满足要求
 */
bool HandEyeCalibNode::checkCalibrationPointsCount(size_t point_count) 
{
    // 检查标定点数量是否满足最小要求
    if (point_count < static_cast<size_t>(calib_config_.getMinCalibrationPoints())) 
    {
        LOG_WARN("⚠️ 标定点数量不足，当前点数: %d, 最小需要点数: %d", static_cast<int>(point_count), calib_config_.getMinCalibrationPoints());
        return false;
    }
    return true;
}

/**
 * @brief 初始化标定点数据
 */
bool HandEyeCalibNode::initCalibrationPoints() 
{
    // 如果标定点已经成功加载，则直接返回成功状态
    if (calibration_points_load_success_) {
        LOG_INFO("标定点数据已成功加载，无需重复加载");
        size_t point_count = robot_pos_mgr_.getPointCount();
        LOG_INFO("当前缓存中标定点数量: %d", static_cast<int>(point_count));
        return true;
    }
    
    bool bRet = false;
    // 根据配置参数决定是否通过服务获取标定点
    if (use_service_for_points_) 
    {
        LOG_INFO("使用服务通讯方式获取标定点");
        
        // 如果使用服务方式，需要先调用服务获取标定点数据
        // 获取标定点数据服务
        if (!get_calibration_points_client_->wait_for_service(std::chrono::seconds(5))) 
        {
            LOG_ERROR("无法连接到获取标定点数据服务");
            bRet = false;
        } 
        else 
        {
            requestCalibrationPoints();// 调用封装的接口请求标定点数据
            
            // 对于初始化过程，我们需要等待一段时间确保服务调用完成
            // 增加等待时间到5秒以确保服务响应处理完成
            std::this_thread::sleep_for(std::chrono::seconds(7));
            LOG_INFO("获取标定点数据服务请求已发送，等待服务处理完成");
            
            // 等待一段时间后检查robot_pos_mgr_中保存的标定点个数来判断是否成功
            size_t point_count = robot_pos_mgr_.getPointCount();
            LOG_INFO("当前缓存中标定点数量: %d", static_cast<int>(point_count));
            
            if (point_count > 0) 
            {
                LOG_INFO("成功获取 %d 个标定点数据", static_cast<int>(point_count));
                // 复用相同的检查逻辑
                bRet = checkCalibrationPointsCount(point_count);
            } else 
            {
                bRet = false;
                LOG_ERROR("未能成功获取标定点数据");
                
                // 添加额外的诊断信息
                LOG_INFO("检查服务是否可用...");
                if (get_calibration_points_client_->service_is_ready()) {
                    LOG_INFO("服务客户端报告服务可用");
                } else {
                    LOG_ERROR("服务客户端报告服务不可用");
                }
            }
        }
    } 
    else 
    {
        // 初始化缓存数据点（在调用getPointCount或getDataPoint之前必须调用）
        LOG_INFO("初始化缓存数据点...");
        robot_pos_mgr_.initializeCachedDataPoints();
        size_t point_count = robot_pos_mgr_.getPointCount();
        LOG_INFO("缓存数据点初始化完成，数据点数量: %d", static_cast<int>(point_count));
        
        // 复用相同的检查逻辑
        bRet = checkCalibrationPointsCount(point_count);
    }
    
    // 更新标定点加载状态
    calibration_points_load_success_ = bRet;
    
    return bRet;
}

/**
 * @brief 标定数据更新回调函数
 * @param calib_data 更新的标定数据
 */
void HandEyeCalibNode::calibDatChangedCallback(const handeyecalib::ArmCalibInfo& calib_data)
{
    LOG_INFO("接收到标定数据更新，机械臂ID: %d", static_cast<int>(calib_data.arm_id));
    
    // 在这里处理标定数据更新
    // 可以更新内部状态或触发重新标定等操作
    const handeyecalib::ArmCalibInfo& arm_calib_info = calib_data;
    LOG_INFO("  机械臂ID: %d, 标定方法: %s", static_cast<int>(arm_calib_info.arm_id), arm_calib_info.calib_info.calib_method.c_str());

    cv::Mat cam_to_base_transform = arm_calib_info.calib_info.calib_res.cam_to_base_transform;

	LOG_INFO("相机到基座的变换矩阵: (%.3f, %.3f, %.3f, %.3f)",
		cam_to_base_transform.at<double>(0, 0), cam_to_base_transform.at<double>(1, 0), cam_to_base_transform.at<double>(2, 0), cam_to_base_transform.at<double>(3, 0));
	LOG_INFO("相机到基座的变换矩阵: (%.3f, %.3f, %.3f, %.3f)",
		cam_to_base_transform.at<double>(0, 1), cam_to_base_transform.at<double>(1, 1), cam_to_base_transform.at<double>(2, 1), cam_to_base_transform.at<double>(3, 1));
	LOG_INFO("相机到基座的变换矩阵: (%.3f, %.3f, %.3f, %.3f)",
		cam_to_base_transform.at<double>(0, 2), cam_to_base_transform.at<double>(1, 2), cam_to_base_transform.at<double>(2, 2), cam_to_base_transform.at<double>(3, 2));
	LOG_INFO("相机到基座的变换矩阵: (%.3f, %.3f, %.3f, %.3f)",
		cam_to_base_transform.at<double>(0, 3), cam_to_base_transform.at<double>(1, 3), cam_to_base_transform.at<double>(2, 3), cam_to_base_transform.at<double>(3, 3));
}

}  // namespace handeyecalib_ros

/**
 * @brief 主函数
 * @param argc 参数数量
 * @param argv 参数数组
 * @return 程序退出码
 */
int main(int argc, char * argv[]) 
{
    // 设置ROS 2日志级别
    rcpputils::fs::path log_config_path = rcpputils::fs::current_path();
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<handeyecalib_ros::HandEyeCalibNode>());
    rclcpp::shutdown();
    return 0;
}