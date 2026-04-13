#include "hand_gesture_rec_node.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <chrono>
#include "bas_operate/file_operate.hpp"
#include <yaml-cpp/yaml.h>

HandGestureRecNode::HandGestureRecNode(const rclcpp::NodeOptions &options, int cam_id)
    : Node("gesture_rec_node_" + std::to_string(cam_id), options)
      , camera_id_(cam_id)
      , camera_intrinsics_initialized_(false)
{
  // 根据camera_id生成日志项目路径
  const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
  log_project_path_ = project_name + "_cam_" + std::to_string(camera_id_);

  // 预先声明参数（使用 declare_parameter 只声明未设置过的参数）
  this->declare_parameter("camera_type", "realsense");
  this->declare_parameter("arm_id", 0);
  this->declare_parameter("engine_path", "install/hand_gesture_rec/models/rtmdet-hand.engine");
  this->declare_parameter("pose_engine_path", "install/hand_gesture_rec/models/rtmpose-hand.engine");
  this->declare_parameter("color_image_topic", "");
  this->declare_parameter("depth_image_topic", "");
  this->declare_parameter("camera_info_topic", "");
  this->declare_parameter("usecalib", true);

  // 相机类型参数
  camera_type_ = this->get_parameter("camera_type").as_string();

  // 机械臂ID参数 - 用于获取对应的标定矩阵
  arm_id_ = this->get_parameter("arm_id").as_int();

  // 初始化arm_id_list_
  arm_id_list_.clear();

  // 创建参数服务器客户端
  sys_config_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "sys_config_ros_node");
  if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
  {
    LOG_INFO(log_project_path_, "无法连接到系统配置参数服务");
  }

  // 从参数服务器获取系统配置（相机对应的机械臂列表）
  if (!getSysDat()) {
    LOG_WARN(log_project_path_, "获取系统配置数据失败，但节点将继续运行");
  }

  // 如果arm_id_list_为空，使用默认arm_id
  if (arm_id_list_.size() == 0)
  {
    arm_id_ = 0;
    arm_id_list_.push_back(arm_id_);
    LOG_WARN(log_project_path_, "No arm ID configured in sys_cam_config, using default arm ID: %d", arm_id_);
  }
  // 默认使用第一个机械臂ID
  arm_id_ = arm_id_list_[0];

  LOG_INFO(log_project_path_, "Arm ID: %d, Arm ID List: [%s]", arm_id_,
    [this]() {
      std::string s;
      for (size_t i = 0; i < arm_id_list_.size(); i++) {
        s += std::to_string(arm_id_list_[i]);
        if (i < arm_id_list_.size() - 1) s += ", ";
      }
      return s;
    }().c_str());

  // 获取引擎路径参数
  std::string engine_path = this->get_parameter("engine_path").as_string();
  std::string pose_engine_path = this->get_parameter("pose_engine_path").as_string();
  LOG_INFO(log_project_path_, "Detection Engine Path: %s", engine_path.c_str());
  LOG_INFO(log_project_path_, "Pose Engine Path: %s", pose_engine_path.c_str());

  // 尝试从launch文件获取话题名称
  std::string launch_color_topic = this->get_parameter("color_image_topic").as_string();
  std::string launch_depth_topic = this->get_parameter("depth_image_topic").as_string();
  std::string launch_camera_info_topic = this->get_parameter("camera_info_topic").as_string();

  // 初始化话题名（从参数服务器获取）
  initTopicNames();
  if(color_image_topic_.empty() || depth_image_topic_.empty() || camera_info_topic_.empty())
  {
    LOG_WARN(log_project_path_, "未从参数服务器正确获取话题名");
    // 如果launch文件提供了话题名，直接使用；
    if (!launch_color_topic.empty() && !launch_depth_topic.empty() && !launch_camera_info_topic.empty())
    {
        color_image_topic_ = launch_color_topic;
        depth_image_topic_ = launch_depth_topic;
        camera_info_topic_ = launch_camera_info_topic;
        LOG_INFO(log_project_path_, "使用launch文件提供的话题名称");
    }
    else
    {
        LOG_ERROR(log_project_path_, "话题名称未设置，请通过launch文件或参数服务器提供");
    }
  }

  // 初始化相机内参状态
  fx_ = 0.0f;
  fy_ = 0.0f;
  cx_ = 0.0f;
  cy_ = 0.0f;

  // 初始化标定参数处理器
  initCalibParamHandler();

  // 获取usecalib参数
  usecalib_ = this->get_parameter("usecalib").as_bool();
  LOG_INFO(log_project_path_, "标定模式: %s", usecalib_ ? "开启" : "关闭");

  LOG_INFO(log_project_path_, "Camera ID: %d", camera_id_);
  LOG_INFO(log_project_path_, "Camera Type: %s", camera_type_.c_str());
  LOG_INFO(log_project_path_, "Arm ID: %d", arm_id_);
  LOG_INFO(log_project_path_, "Color Image Topic: %s", color_image_topic_.c_str());
  LOG_INFO(log_project_path_, "Depth Image Topic: %s", depth_image_topic_.c_str());
  LOG_INFO(log_project_path_, "Camera Info Topic: %s", camera_info_topic_.c_str());

  // 获取模型路径
  try {
      std::string package_share_dir = ament_index_cpp::get_package_share_directory("hand_gesture_rec");
      detection_engine_path_ = package_share_dir + "/../../models/rtmdet-hand.engine";
      pose_engine_path_ = package_share_dir + "/../../models/rtmpose-hand.engine";
  } catch (const std::exception& e) {
      LOG_WARN(log_project_path_, "Failed to get package share directory: %s", e.what());
      detection_engine_path_ = "models/rtmdet-hand.engine";
      pose_engine_path_ = "models/rtmpose-hand.engine";
  }

  // 初始化新增功能 - 解耦合设计
  setup_camera_intrinsics();  // 设置相机内参

  // 初始化手部检测管道
  try {
      hand_pipeline_ = std::make_unique<HandPipeline>();

      // 设置检测配置
      HandDetectionConfig det_config;
      det_config.input_size = 320;
      det_config.conf_thresh = 0.5f;
      det_config.nms_thresh = 0.2f;
      det_config.crop_enlargement = 1.25f;
      det_config.use_gpu_preprocess = false;
      hand_pipeline_->set_detection_config(det_config);

      // 设置姿态配置
      HandPoseConfig pose_config;
      pose_config.input_size = 256;
      pose_config.simcc_split_ratio = 2.0f;
      hand_pipeline_->set_pose_config(pose_config);

      // 加载模型
  hand_pipeline_->load_models(detection_engine_path_, pose_engine_path_);
  LOG_INFO(log_project_path_, "Hand pipeline models loaded successfully");

  // 订阅深度图像话题
  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_image_topic_, 10,
      std::bind(&HandGestureRecNode::depth_callback, this, std::placeholders::_1));

  } catch (const std::exception& e) {
      LOG_ERROR(log_project_path_, "Failed to initialize hand pipeline: %s", e.what());
      throw;
  }

  // 初始化手势检测器
  gesture_detector_ = std::make_unique<GestureDetector>();

  // 生成话题前缀：根据camera_id
  std::string topic_prefix = "/cam_" + std::to_string(camera_id_);

  // 创建发布者 - 使用相机前缀
  std::string gesture_result_topic = topic_prefix + "/gesture_result";
  std::string visualization_topic = topic_prefix + "/gesture_visualization";
  std::string det_res_topic = topic_prefix + "/hand_det_res";

  gesture_result_topic_ = this->declare_parameter("gesture_result_topic", "");
  visualization_topic_ = this->declare_parameter("visualization_topic", "");

  // 如果参数为空，使用代码生成的默认值
  if (gesture_result_topic_.empty()) {
      gesture_result_topic_ = gesture_result_topic;
  }
  if (visualization_topic_.empty()) {
      visualization_topic_ = visualization_topic;
  }

  // 获取参数（使用 get_parameter_or 避免与 NodeOptions 冲突）
  enable_visualization_ = this->get_parameter_or("enable_visualization", true);
  min_wave_pairs_ = this->get_parameter_or("min_wave_pairs", 1);
  use_real_3d_ = this->get_parameter_or("use_real_3d", true);

  // 根据 use_real_3d 设置不同的挥手阈值
  float max_wave_y_movement;
  float min_wave_amplitude;
  if (use_real_3d_) {
      // 3D模式：使用毫米单位
      max_wave_y_movement = this->get_parameter_or("max_wave_y_movement", 30.0f);
      min_wave_amplitude = this->get_parameter_or("min_wave_amplitude", 30.0f);
  } else {
      // 2D模式：使用像素单位（默认值会根据图像尺寸动态调整）
      max_wave_y_movement = this->get_parameter_or("max_wave_y_movement", 50.0f);
      min_wave_amplitude = this->get_parameter_or("min_wave_amplitude", 80.0f);
  }

  gesture_detector_->set_min_wave_pairs(min_wave_pairs_);
  gesture_detector_->set_max_wave_y_movement(max_wave_y_movement);
  gesture_detector_->set_min_wave_amplitude(min_wave_amplitude);
  gesture_detector_->set_use_real_3d(use_real_3d_);

  // 保存初始阈值参数（用于2D模式动态计算）
  initial_max_wave_y_movement_ = max_wave_y_movement;
  initial_min_wave_amplitude_ = min_wave_amplitude;
  wave_thresholds_updated_ = false;  // 标记阈值是否已根据图像尺寸更新

  // 初始化Wave手势指令相关变量
  wave_triggered_ = false;
  wave_cooldown_frames_ = 30;  // 约1秒
  wave_cooldown_counter_ = 0;

  gesture_result_pub_ = this->create_publisher<std_msgs::msg::String>(gesture_result_topic_, 10);
  det_res_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(det_res_topic, 10);

  // 创建Wave手势指令发布者
  std::string gesture_command_topic = topic_prefix + "/gesture_command";
  gesture_command_pub_ = this->create_publisher<std_msgs::msg::String>(gesture_command_topic, 10);
  
  if (enable_visualization_) {
      visualization_pub_ = this->create_publisher<sensor_msgs::msg::Image>(visualization_topic_, 10);
  }

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      color_image_topic_, 10,
      std::bind(&HandGestureRecNode::image_callback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      std::bind(&HandGestureRecNode::camera_info_callback, this, std::placeholders::_1));

  LOG_INFO(log_project_path_, "Publishing to: %s", gesture_result_topic_.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", det_res_topic.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", gesture_command_pub_->get_topic_name());
  if (enable_visualization_) {
      LOG_INFO(log_project_path_, "Publishing to: %s", visualization_topic_.c_str());
  }
  LOG_INFO(log_project_path_, "使用3D手势识别: %s", use_real_3d_ ? "是" : "否");
}

// 使用parseCommInfo初始化话题名
void HandGestureRecNode::initTopicNames()
{
  basros::RosCommInfo comm_info;
  comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_COLOR_IMAGE, camera_id_, 0);
  if (sys_config_client_->has_parameter(comm_info.name)) {
      // 安全地获取参数值
      color_image_topic_ = sys_config_client_->get_parameter<std::string>(comm_info.name);
  }

  comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_DEPTH_IMAGE, camera_id_, 0);
  if (sys_config_client_->has_parameter(comm_info.name)) {
      // 安全地获取参数值
      depth_image_topic_ = sys_config_client_->get_parameter<std::string>(comm_info.name);
  }

  comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_CAM_INTRINSICS, camera_id_, 0);

  if (sys_config_client_->has_parameter(comm_info.name)) {
      // 安全地获取参数值
      camera_info_topic_ = sys_config_client_->get_parameter<std::string>(comm_info.name);
  }
}

// 从参数服务器获取系统配置（包括相机对应的机械臂列表）
bool HandGestureRecNode::getSysDat()
{
    // 等待参数服务可用
    if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
    {
        LOG_ERROR(log_project_path_, "无法连接到系统配置参数服务");
        return false;
    }

    try
    {
        SysConfig::CamConfigInfo cam_info;
        bool success = getCamConfigInfo(cam_info);
        if (!success)
        {
            LOG_WARN(log_project_path_, "参数服务器未找到系统参数");
        }
        return success;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(log_project_path_, "读取系统参数失败: %s", e.what());
        return false;
    }
}

// 获取相机配置信息
bool HandGestureRecNode::getCamConfigInfo(SysConfig::CamConfigInfo& cam_info)
{
    bool bRet = RosComm::getCamInfoFromServer(sys_config_client_, camera_id_, cam_info);
    if (bRet)
    {
        LOG_INFO(log_project_path_, "读取到与当前相机ID匹配的配置，ID: %d, 机械臂列表: %zu", cam_info.cam_id, cam_info.armInfoList.size());
        const SysConfig::ArmConfigInfoList& arm_info_list = cam_info.armInfoList;
        if (!arm_info_list.empty())
        {
            for (const auto& arm_info : arm_info_list)
            {
                LOG_INFO(log_project_path_, "获取配置的机械臂ID: %d, 是否启用: %s", arm_info.arm_id, arm_info.is_enable ? "是" : "否");
                if (arm_info.is_enable)
                {
                    arm_id_list_.push_back(arm_info.arm_id);
                }
            }
        }
    }
    return bRet;
}

// 初始化标定参数处理器
void HandGestureRecNode::initCalibParamHandler()
{
  try
  {
    // 等待参数服务器上线（最多等待10秒）
    if (sys_config_client_->wait_for_service(std::chrono::seconds(3)))
    {
      LOG_INFO(log_project_path_, "参数服务器已连接");

      // 初始化标定结果
      calib_result_ = std::make_unique<handeyecalib::CalibRes>();

      // 从参数服务器直接读取标定矩阵 - 根据camera_id和arm_id动态构建参数路径
      std::string param_prefix = "sys_cam_calib_list.cam_" + std::to_string(camera_id_) + ".arm_info.arm_" + std::to_string(arm_id_);
      LOG_INFO(log_project_path_, "标定矩阵参数前缀: %s", param_prefix.c_str());

      // 读取 cam_to_base_transform
      std::string cam_to_base_param = param_prefix + ".cam_to_base_transform";
      try {
        std::string cam_to_base_str = sys_config_client_->get_parameter<std::string>(cam_to_base_param);
        calib_result_->cam_to_base_transform = handeyecalib::stringToMat(cam_to_base_str);
        LOG_INFO(log_project_path_, "成功读取 cam_to_base_transform");
      } catch (const std::exception& e) {
        LOG_WARN(log_project_path_, "读取 cam_to_base_transform 失败: %s", e.what());
      }

      // 读取 base_to_cam_transform
      std::string base_to_cam_param = param_prefix + ".base_to_cam_transform";
      try {
        std::string base_to_cam_str = sys_config_client_->get_parameter<std::string>(base_to_cam_param);
        calib_result_->base_to_cam_transform = handeyecalib::stringToMat(base_to_cam_str);
        LOG_INFO(log_project_path_, "成功读取 base_to_cam_transform");
      } catch (const std::exception& e) {
        LOG_WARN(log_project_path_, "读取 base_to_cam_transform 失败: %s", e.what());
      }

      // 读取 offset_compensation
      std::string offset_param = param_prefix + ".offset_compensation";
      try {
        calib_result_->offset_compensation = sys_config_client_->get_parameter<std::vector<double>>(offset_param);
        LOG_INFO(log_project_path_, "成功读取 offset_compensation, size: %zu", calib_result_->offset_compensation.size());
        if (!calib_result_->offset_compensation.empty()) {
          LOG_INFO(log_project_path_, "  offset_compensation: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
                      calib_result_->offset_compensation[0], calib_result_->offset_compensation[1],
                      calib_result_->offset_compensation[2], calib_result_->offset_compensation[3],
                      calib_result_->offset_compensation[4], calib_result_->offset_compensation[5]);
        }
      } catch (const std::exception& e) {
        LOG_WARN(log_project_path_, "读取 offset_compensation 失败: %s", e.what());
      }

      // 打印标定矩阵信息
      LOG_INFO(log_project_path_, "标定结果已加载");
      if (!calib_result_->cam_to_base_transform.empty())
      {
        const auto& m = calib_result_->cam_to_base_transform;
        LOG_INFO(log_project_path_, "cam_to_base_transform (%d x %d):",
                    m.rows, m.cols);
        for (int i = 0; i < m.rows; i++) {
          LOG_INFO(log_project_path_, "  [%.6f, %.6f, %.6f, %.6f]",
                      m.at<double>(i, 0), m.at<double>(i, 1),
                      m.at<double>(i, 2), m.at<double>(i, 3));
        }
      }
    }
    else
    {
      LOG_WARN(log_project_path_, "参数服务器未上线，将在后台继续等待...");
    }
  }
  catch (const std::exception& e)
  {
    LOG_WARN(log_project_path_, "初始化标定参数处理器失败: %s", e.what());
  }
}

// 设置相机内参 - 通过API获取
void HandGestureRecNode::setup_camera_intrinsics()
{
    // 等待相机信息话题发布内参数据
    LOG_INFO(log_project_path_, "等待相机内参数据...");
    LOG_INFO(log_project_path_, "订阅相机信息话题: %s", camera_info_topic_.c_str());
}

// 相机信息回调函数 - 通过API获取真实的内参数据
void HandGestureRecNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (camera_intrinsics_initialized_)
    {
        return;  // 已经初始化过，避免重复处理
    }

    // 从CameraInfo消息中提取内参
    if (msg->k.size() >= 9)
    {
        // 内参矩阵 K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
        fx_ = msg->k[0];  // 焦距x
        fy_ = msg->k[4];  // 焦距y
        cx_ = msg->k[2];  // 主点x
        cy_ = msg->k[5];  // 主点y

        camera_intrinsics_initialized_ = true;

        LOG_INFO(log_project_path_, "成功获取相机内参:");
        LOG_INFO(log_project_path_, "  fx=%.2f, fy=%.2f", fx_, fy_);
        LOG_INFO(log_project_path_, "  cx=%.2f, cy=%.2f", cx_, cy_);
        LOG_INFO(log_project_path_, "  图像尺寸: %dx%d", msg->width, msg->height);

        // 检查内参是否合理
        if (fx_ <= 0 || fy_ <= 0)
        {
            LOG_WARN(log_project_path_, "相机内参异常，使用默认值");
            fx_ = 615.0f;
            fy_ = 615.0f;
            cx_ = msg->width / 2.0f;
            cy_ = msg->height / 2.0f;
        }
    }
    else
    {
        LOG_WARN(log_project_path_, "CameraInfo消息格式错误，使用默认内参");
        fx_ = 615.0f;
        fy_ = 615.0f;
        cx_ = 320.0f;
        cy_ = 240.0f;
        camera_intrinsics_initialized_ = true;
    }
}

// 计算camera_link下的3D坐标
geometry_msgs::msg::Point HandGestureRecNode::calculate_3d_position(const HandResult& /* hand_result */, const GestureInfo& gesture_info)
{
    geometry_msgs::msg::Point camera_point;

    // 检查相机内参是否已初始化
    if (!camera_intrinsics_initialized_)
    {
        LOG_WARN(log_project_path_, "相机内参未初始化，无法计算3D坐标");
        return camera_point;
    }

    // 检查内参是否有效
    if (fx_ <= 0 || fy_ <= 0)
    {
        LOG_WARN(log_project_path_, "相机内参无效，无法计算3D坐标");
        return camera_point;
    }

    // 使用手腕关键点作为手部中心位置
    if (gesture_info.keypoints_2d.empty())
    {
        return camera_point;
    }

    // 手腕点（关键点0）作为参考点
    float center_x = gesture_info.keypoints_2d[0].x;
    float center_y = gesture_info.keypoints_2d[0].y;

    float depth_mm = 500.0f;  // 默认深度500mm

    // 如果启用了3D模式且深度图可用，从深度图获取真实深度
    if (use_real_3d_ && !depth_frame_.empty())
    {
        // 检查像素坐标是否在图像范围内
        int pixel_x = static_cast<int>(center_x);
        int pixel_y = static_cast<int>(center_y);

        if (pixel_x >= 0 && pixel_x < depth_frame_.cols &&
            pixel_y >= 0 && pixel_y < depth_frame_.rows)
        {
            // 获取深度值
            uint16_t depth_value = depth_frame_.at<uint16_t>(pixel_y, pixel_x);

            if (depth_value > 0)
            {
                depth_mm = static_cast<float>(depth_value);  // 输出为毫米
            }
            else
            {
                LOG_DEBUG(log_project_path_, "深度值为0，使用默认深度500mm");
            }
        }
        else
        {
            LOG_DEBUG(log_project_path_, "像素坐标超出深度图范围，使用默认深度500mm");
        }
    }
    else if (use_real_3d_)
    {
        LOG_DEBUG(log_project_path_, "深度图不可用，使用默认深度500mm");
    }

    // 计算3D坐标（相机坐标系）
    camera_point.x = (center_x - cx_) * depth_mm / fx_;
    camera_point.y = (center_y - cy_) * depth_mm / fy_;
    camera_point.z = depth_mm;

    return camera_point;
}

// 深度图像回调函数
void HandGestureRecNode::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    depth_frame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  }
  catch (const cv_bridge::Exception& e)
  {
    LOG_ERROR(log_project_path_, "转换深度图像失败: %s", e.what());
  }
}

// 根据2D图像尺寸更新挥手阈值
void HandGestureRecNode::update_wave_thresholds_2d(int image_width, int image_height)
{
  if (!use_real_3d_ && !wave_thresholds_updated_)
  {
    // 2D模式：根据图像尺寸动态计算（像素单位）
    float max_wave_y_movement_2d = image_height * 0.20f;
    float min_wave_amplitude_2d = image_width * 0.10f;

    gesture_detector_->set_max_wave_y_movement(max_wave_y_movement_2d);
    gesture_detector_->set_min_wave_amplitude(min_wave_amplitude_2d);

    LOG_INFO(log_project_path_, "2D模式挥手阈值已根据图像尺寸更新:");
    LOG_INFO(log_project_path_, "  图像尺寸: %dx%d", image_width, image_height);
    LOG_INFO(log_project_path_, "  max_wave_y_movement: %.2f px ", max_wave_y_movement_2d);
    LOG_INFO(log_project_path_, "  min_wave_amplitude: %.2f px ", min_wave_amplitude_2d);

    wave_thresholds_updated_ = true;
  }
}

void HandGestureRecNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // 转换ROS图像到OpenCV格式
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;

        auto start = std::chrono::high_resolution_clock::now();

        // 2D模式：根据实际图像尺寸动态更新挥手阈值
        if (!use_real_3d_ && !wave_thresholds_updated_)
        {
            update_wave_thresholds_2d(image.cols, image.rows);
        }

        // 检测手部和姿态
        std::vector<HandResult> hand_results = hand_pipeline_->process(image);

        // 准备可视化图像（始终克隆，无论是否有手部检测到）
        cv::Mat vis_image;
        if (enable_visualization_) {
            vis_image = image.clone();
        }

        // 对每只手进行手势识别
        for (const auto& hand_result : hand_results) {
            GestureInfo gesture_info;

            // 根据use_real_3d参数选择模式
            if (use_real_3d_ && camera_intrinsics_initialized_ && !depth_frame_.empty()) {
                // 3D模式：使用真实深度图（直接传入深度图参数）
                std::vector<cv::Point3f> keypoints_3d_real =
                    gesture_detector_->convert_to_real_3d(hand_result.keypoints, depth_frame_, fx_, fy_, cx_, cy_);

                gesture_info = gesture_detector_->detect_gesture_3d(hand_result, keypoints_3d_real);
            } else {
                // 2D模式
                gesture_info = gesture_detector_->detect_gesture(hand_result, image.cols, image.rows);
            }

            // 计算手部3D坐标(手腕关键点)
            geometry_msgs::msg::Point camera_3d = calculate_3d_position(hand_result, gesture_info);

            // 发布手势结果
            if (!gesture_info.gesture_name.empty()) {
                publish_gesture_result(gesture_info, msg->header, hand_result, camera_3d);
            }
        }

        // 可视化：绘制手部关键点和骨架（在循环外）
        if (enable_visualization_) {
            for (const auto& hand_result : hand_results) {
                GestureInfo gesture_info;
                if (use_real_3d_ && camera_intrinsics_initialized_ && !depth_frame_.empty()) {
                    // 3D模式：直接传入深度图
                    std::vector<cv::Point3f> keypoints_3d_real =
                        gesture_detector_->convert_to_real_3d(hand_result.keypoints, depth_frame_, fx_, fy_, cx_, cy_);
                    gesture_info = gesture_detector_->detect_gesture_3d(hand_result, keypoints_3d_real);
                } else {
                    gesture_info = gesture_detector_->detect_gesture(hand_result, image.cols, image.rows);
                }
                geometry_msgs::msg::Point camera_3d = calculate_3d_position(hand_result, gesture_info);
                visualize_hand(vis_image, hand_result, gesture_info, camera_3d);
            }
        }

        // 发布可视化图像（始终发布，即使没有检测到手部）
        if (enable_visualization_) {
            sensor_msgs::msg::Image::SharedPtr vis_msg =
                cv_bridge::CvImage(msg->header, "bgr8", vis_image).toImageMsg();
            visualization_pub_->publish(*vis_msg);
        }

        // Wave指令冷却处理
        if (wave_triggered_) {
            wave_cooldown_counter_--;
            if (wave_cooldown_counter_ <= 0) {
                wave_triggered_ = false;
                wave_cooldown_counter_ = 0;
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        LOG_DEBUG(log_project_path_, "Gesture recognition took %ld ms", duration.count());

    } catch (const cv_bridge::Exception& e) {
        LOG_ERROR(log_project_path_, "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        LOG_ERROR(log_project_path_, "Exception in image callback: %s", e.what());
    }
}

void HandGestureRecNode::publish_gesture_result(const GestureInfo& gesture_info,
                                          const std_msgs::msg::Header& header,
                                          const HandResult& hand_result,
                                          const geometry_msgs::msg::Point& camera_3d)
{
    // 发布手势名称
    std_msgs::msg::String result_msg;
    result_msg.data = gesture_info.gesture_name;
    gesture_result_pub_->publish(result_msg);

    if(gesture_info.gesture_name == "Wave")
    {
        LOG_INFO(log_project_path_, "Detected wave gesture: %s (confidence: %.2f, hand: %s)",
                gesture_info.gesture_name.c_str(), gesture_info.confidence,
                gesture_info.hand_type == HandType::LEFT ? "Left" :
                gesture_info.hand_type == HandType::RIGHT ? "Right" : "Unknown");

        // Wave手势特殊处理：触发指令
        if (!wave_triggered_) {
            std_msgs::msg::String wave_command_msg;
            wave_command_msg.data = "wave_greeting";  // 挥手问好指令
            gesture_command_pub_->publish(wave_command_msg);
            LOG_INFO(log_project_path_, "Wave gesture detected: Sending wave_greeting command");
            wave_triggered_ = true;
            wave_cooldown_counter_ = wave_cooldown_frames_;
        }
    }
    else if (gesture_info.gesture_name != "Wave") {
        // 其他手势重置Wave触发状态
        wave_triggered_ = false;
    }

    // 发布检测框结果（兼容现有系统），包含手部检测中心点的3D位置
    vision_msgs::msg::Detection2DArray det_array;
    det_array.header = header;

    vision_msgs::msg::Detection2D det;
    det.bbox.center.position.x = (hand_result.bbox.x1 + hand_result.bbox.x2) / 2.0f;
    det.bbox.center.position.y = (hand_result.bbox.y1 + hand_result.bbox.y2) / 2.0f;
    det.bbox.size_x = hand_result.bbox.x2 - hand_result.bbox.x1;
    det.bbox.size_y = hand_result.bbox.y2 - hand_result.bbox.y1;

    vision_msgs::msg::ObjectHypothesisWithPose obj_hypo;
    obj_hypo.hypothesis.class_id = gesture_info.gesture_name;
    obj_hypo.hypothesis.score = gesture_info.confidence;

    // 根据 usecalib_ 参数决定使用哪个坐标系的3D位置
    geometry_msgs::msg::Point position_3d = camera_3d;
    
    // 如果启用标定模式，将相机坐标系转换为base坐标系
    if (usecalib_ && calib_result_ && !calib_result_->cam_to_base_transform.empty())
    {
        try {
            // 准备标记位姿（x, y, z, roll, pitch, yaw），对于手势识别不需要旋转角
            std::vector<double> marker_pose = {
                camera_3d.x, camera_3d.y, camera_3d.z,
                0.0, 0.0, 0.0  // 手势识别不需要旋转角，设为0
            };
            std::vector<double> offset_compensation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            if (!calib_result_->offset_compensation.empty() && calib_result_->offset_compensation.size() >= 6) {
                offset_compensation = calib_result_->offset_compensation;
            }

            // 使用标定库计算机器人位姿
            auto trans_result = handeyecalib::computeRobotPoseFromMarker(
                marker_pose, calib_result_->cam_to_base_transform, &offset_compensation);

            if (!trans_result.transformed_pose.empty())
            {
                position_3d.x = trans_result.transformed_pose[0] + calib_result_->offset_compensation[0];
                position_3d.y = trans_result.transformed_pose[1] + calib_result_->offset_compensation[1];
                position_3d.z = trans_result.transformed_pose[2] + calib_result_->offset_compensation[2];

                LOG_DEBUG(log_project_path_, "3D位置已转换到base坐标系: (%.3f, %.3f, %.3f)",
                            position_3d.x, position_3d.y, position_3d.z);
            }
        }
        catch (const std::exception& e) {
            LOG_WARN(log_project_path_, "坐标转换失败: %s", e.what());
            // 转换失败，继续使用相机坐标系
        }
    }

    // 在pose中添加3D位置信息（根据usecalib_决定是相机坐标系还是base坐标系）
    obj_hypo.pose.pose.position.x = position_3d.x;
    obj_hypo.pose.pose.position.y = position_3d.y;
    obj_hypo.pose.pose.position.z = position_3d.z;
    obj_hypo.pose.pose.orientation.w = 1.0;  // 单位四元数

    det.results.push_back(obj_hypo);

    det_array.detections.push_back(det);
    det_res_pub_->publish(det_array);
}

void HandGestureRecNode::visualize_hand(cv::Mat& image, const HandResult& hand_result,
                                  const GestureInfo& gesture_info,
                                  const geometry_msgs::msg::Point& camera_3d) {
    // 设置绘制颜色（根据左右手）
    cv::Scalar hand_color = (gesture_info.hand_type == HandType::LEFT) ? cv::Scalar(0, 255, 0) :
                          (gesture_info.hand_type == HandType::RIGHT) ? cv::Scalar(255, 0, 0) :
                          cv::Scalar(128, 128, 128);

    // 绘制手部标签
    std::string hand_label = gesture_info.hand_type == HandType::LEFT ? "Left" :
                           gesture_info.hand_type == HandType::RIGHT ? "Right" : "Unknown";
    cv::putText(image, hand_label,
                cv::Point(hand_result.bbox.x1, hand_result.bbox.y1 - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, hand_color, 2);

    // 如果识别出手势，绘制手势名称和置信度
    if (!gesture_info.gesture_name.empty()) {
        cv::putText(image, gesture_info.gesture_name,
                    cv::Point(hand_result.bbox.x1, hand_result.bbox.y2 + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        std::string conf_text = "Conf: " + std::to_string(gesture_info.confidence).substr(0, 4);
        cv::putText(image, conf_text,
                    cv::Point(hand_result.bbox.x1, hand_result.bbox.y2 + 55),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
    }

    // 绘制3D坐标
    std::string pos_text = "3D: [" + std::to_string(camera_3d.x).substr(0, 5) + ", " +
                         std::to_string(camera_3d.y).substr(0, 5) + ", " +
                         std::to_string(camera_3d.z).substr(0, 5) + "]";
    cv::putText(image, pos_text,
                cv::Point(hand_result.bbox.x1, hand_result.bbox.y2 + 80),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);

    // 绘制关键点（使用2D像素坐标）
    for (size_t i = 0; i < gesture_info.keypoints_2d.size(); i++) {
        cv::Point pt(static_cast<int>(gesture_info.keypoints_2d[i].x),
                    static_cast<int>(gesture_info.keypoints_2d[i].y));
        cv::circle(image, pt, 4, hand_color, -1);
    }

    // 绘制连接线（使用2D像素坐标）
    const auto& connections = gesture_detector_->get_hand_connections();
    for (const auto& conn : connections) {
        if (static_cast<size_t>(conn.first) < gesture_info.keypoints_2d.size() &&
            static_cast<size_t>(conn.second) < gesture_info.keypoints_2d.size()) {
            cv::Point pt1(static_cast<int>(gesture_info.keypoints_2d[conn.first].x),
                        static_cast<int>(gesture_info.keypoints_2d[conn.first].y));
            cv::Point pt2(static_cast<int>(gesture_info.keypoints_2d[conn.second].x),
                        static_cast<int>(gesture_info.keypoints_2d[conn.second].y));
            cv::line(image, pt1, pt2, hand_color, 2);
        }
    }
}

// Hand Gesture Recognition 主节点 - 用于管理多个相机节点
class GestureRecMainNode : public rclcpp::Node
{
public:
  explicit GestureRecMainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("gesture_rec_main_node", options)
  {
    log_project_path_ = basmodule::get_project_name_by_file_path(__FILE__);
    std::string sys_config_node_name = "sys_config_ros_node";
    sys_config_client_ = std::make_shared<rclcpp::SyncParametersClient>(
      this,
      sys_config_node_name
    );
  }

  std::vector<int> getActiveCameraIds()
  {
    cam_active_ids_.clear();
    try
    {
      // 从配置文件读取相机ID列表
      std::string config_file_path = ament_index_cpp::get_package_share_directory("hand_gesture_rec")
        + "/config" + "/hand_gesture_rec_params.yaml";
      YAML::Node config = YAML::LoadFile(config_file_path);

      if (config["ros__parameters"] && config["ros__parameters"]["camera_id"])
      {
        auto cam_id_node = config["ros__parameters"]["camera_id"];
        if (cam_id_node.IsSequence())
        {
          for (const auto& id : cam_id_node)
          {
            cam_active_ids_.push_back(id.as<int>());
          }
        }
        else
        {
          cam_active_ids_.push_back(cam_id_node.as<int>());
        }
      }
    }
    catch (const YAML::Exception& e)
    {
      LOG_ERROR(log_project_path_, "解析配置文件失败: %s", e.what());
    }

    std::string cam_ids_str;
    for (const auto& id : cam_active_ids_) {
      cam_ids_str += std::to_string(id) + " ";
    }
    LOG_INFO(log_project_path_, "有效配置的相机ID列表: %s", cam_ids_str.c_str());
    return cam_active_ids_;
  }

private:
  rclcpp::SyncParametersClient::SharedPtr sys_config_client_;
  std::vector<int> cam_active_ids_;
  std::string log_project_path_;
};

void test()
{
  std::string detection_engine_path = "install/hand_gesture_rec/models/rtmdet-hand.engine";
  std::string pose_engine_path = "install/hand_gesture_rec/models/rtmpose-hand.engine";
  std::string input_folder = "src/perception/hand_gesture_rec/test_imgs";
  std::string output_folder = "output_results";

  if (access(output_folder.c_str(), 0) == -1)
  {
      mkdir(output_folder.c_str(), 0755);
      std::cout << "创建输出文件夹: " << output_folder << std::endl;
  }

  auto hand_pipeline = std::make_unique<HandPipeline>();

  HandDetectionConfig det_config;
  det_config.input_size = 320;
  det_config.conf_thresh = 0.5f;
  det_config.nms_thresh = 0.2f;
  det_config.crop_enlargement = 1.25f;
  det_config.use_gpu_preprocess = false;
  hand_pipeline->set_detection_config(det_config);

  HandPoseConfig pose_config;
  pose_config.input_size = 256;
  pose_config.simcc_split_ratio = 2.0f;
  hand_pipeline->set_pose_config(pose_config);

  hand_pipeline->load_models(detection_engine_path, pose_engine_path);
  std::cout << "模型加载成功！" << std::endl;

  auto gesture_detector = std::make_unique<GestureDetector>();
  gesture_detector->set_min_wave_pairs(1);
  gesture_detector->set_max_wave_y_movement(50.0f);
  gesture_detector->set_min_wave_amplitude(80.0f);
  gesture_detector->set_use_real_3d(false);

  std::vector<std::string> image_extensions = {".jpg", ".jpeg", ".png", ".bmp"};
  std::vector<std::string> image_files;

  DIR* dir = opendir(input_folder.c_str());
  if (dir == nullptr)
  {
      throw std::runtime_error("无法打开输入文件夹: " + input_folder);
  }

  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr)
  {
      std::string filename = entry->d_name;
      if (filename == "." || filename == "..") continue;
      std::string extension = filename.substr(filename.find_last_of('.'));
      std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

      for (const auto& ext : image_extensions)
      {
          if (extension == ext)
          {
              image_files.push_back(input_folder + "/" + filename);
              break;
          }
      }
  }
  closedir(dir);

  if (image_files.empty())
  {
      std::cout << "输入文件夹中未找到图片文件" << std::endl;
      return;
  }

  std::cout << "找到 " << image_files.size() << " 张图片" << std::endl;

  int success_count = 0;
  for (const auto& image_path : image_files)
  {
      std::string filename = image_path.substr(image_path.find_last_of('/') + 1);
      std::string output_path = output_folder + "/" + filename;

      cv::Mat img = cv::imread(image_path);
      if (img.empty())
      {
          std::cerr << "无法加载图像: " << image_path << std::endl;
          continue;
      }

      std::cout << "\n处理: " << filename << std::endl;

      auto start = std::chrono::high_resolution_clock::now();
      std::vector<HandResult> hand_results;
      try {
          hand_results = hand_pipeline->process(img);
      }
      catch (const std::exception& e) {
          std::cerr << "检测异常: " << e.what() << std::endl;
          continue;
      }

      auto end = std::chrono::high_resolution_clock::now();
      double inference_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

      std::cout << "检测到手部数量: " << hand_results.size() << ", 耗时: " << inference_time << " ms" << std::endl;

      cv::Mat result_img = img.clone();

      for (const auto& hand_result : hand_results)
      {
          GestureInfo gesture_info;
          gesture_info = gesture_detector->detect_gesture(hand_result, img.cols, img.rows);

          std::cout << "手势: " << gesture_info.gesture_name
                    << " (置信度: " << gesture_info.confidence << ")" << std::endl;

          cv::Scalar hand_color = (gesture_info.hand_type == HandType::LEFT) ? cv::Scalar(0, 255, 0) :
                                (gesture_info.hand_type == HandType::RIGHT) ? cv::Scalar(255, 0, 0) :
                                cv::Scalar(128, 128, 128);

          cv::rectangle(result_img,
                       cv::Point(hand_result.bbox.x1, hand_result.bbox.y1),
                       cv::Point(hand_result.bbox.x2, hand_result.bbox.y2),
                       hand_color, 2);

          std::string hand_label = gesture_info.hand_type == HandType::LEFT ? "Left" :
                                 gesture_info.hand_type == HandType::RIGHT ? "Right" : "Unknown";
          cv::putText(result_img, hand_label,
                      cv::Point(hand_result.bbox.x1, hand_result.bbox.y1 - 10),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, hand_color, 2);

          if (!gesture_info.gesture_name.empty())
          {
              cv::putText(result_img, gesture_info.gesture_name,
                          cv::Point(hand_result.bbox.x1, hand_result.bbox.y2 + 25),
                          cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
          }

          for (size_t i = 0; i < gesture_info.keypoints_2d.size(); i++)
          {
              cv::Point pt(static_cast<int>(gesture_info.keypoints_2d[i].x),
                          static_cast<int>(gesture_info.keypoints_2d[i].y));
              cv::circle(result_img, pt, 4, hand_color, -1);
          }

          const auto& connections = gesture_detector->get_hand_connections();
          for (const auto& conn : connections)
          {
              if (static_cast<size_t>(conn.first) < gesture_info.keypoints_2d.size() &&
                  static_cast<size_t>(conn.second) < gesture_info.keypoints_2d.size())
              {
                  cv::Point pt1(static_cast<int>(gesture_info.keypoints_2d[conn.first].x),
                              static_cast<int>(gesture_info.keypoints_2d[conn.first].y));
                  cv::Point pt2(static_cast<int>(gesture_info.keypoints_2d[conn.second].x),
                              static_cast<int>(gesture_info.keypoints_2d[conn.second].y));
                  cv::line(result_img, pt1, pt2, hand_color, 2);
              }
          }
      }

      if (cv::imwrite(output_path, result_img))
      {
          std::cout << "结果已保存: " << output_path << std::endl;
          success_count++;
      }
      else
      {
          std::cerr << "保存失败: " << output_path << std::endl;
      }
  }

  std::cout << "\n==== 处理完成 ====" << std::endl;
  std::cout << "成功处理: " << success_count << "/" << image_files.size() << " 张图片" << std::endl;
  std::cout << "结果保存在: " << output_folder << std::endl;
  _exit(0);
}

int main(int argc, char** argv) {
  if(0)
  {
    test();
  }
  else
  {
    // 构造命令行参数，包含配置文件路径
    std::vector<std::string> new_argv_strings;
    std::vector<char*> new_argv;

    for (int i = 1; i < argc; i++)
    {
      new_argv_strings.push_back(std::string(argv[i]));
    }
    std::string default_config_file_path = ament_index_cpp::get_package_share_directory("hand_gesture_rec")
      + "/config" + "/hand_gesture_rec_params.yaml";
    if (argc <= 1)
      new_argv_strings.push_back("--ros-args");
    new_argv_strings.push_back("--params-file");
    new_argv_strings.push_back(default_config_file_path);
    std::cout << "使用默认配置文件: " << default_config_file_path << std::endl;

    new_argv.reserve(new_argv_strings.size() + 1);
    for (const auto& arg : new_argv_strings) {
      std::cout << "参数配置: " << arg << std::endl;
      new_argv.push_back(const_cast<char*>(arg.c_str()));
    }
    new_argv.push_back(nullptr);

    rclcpp::init(argc, argv);
    const std::string log_project_path = basmodule::get_project_name_by_file_path(__FILE__);
    LOG_INFO(log_project_path, "hand_gesture_recognition节点启动");
    rclcpp::executors::MultiThreadedExecutor executor;

    auto main_node = std::make_shared<GestureRecMainNode>();
    std::vector<int> act_cam_ids = main_node->getActiveCameraIds();

    if (act_cam_ids.empty())
    {
      LOG_ERROR(log_project_path, "有效配置的相机ID列表为空,默认使用相机ID 0");
      act_cam_ids = {0};
    }

    std::vector<std::shared_ptr<HandGestureRecNode>> nodes;
    for (int camera_id : act_cam_ids)
    {
      LOG_INFO(log_project_path, "创建 HandGestureRecNode 节点，camera_id: %d", camera_id);

      // 通过NodeOptions传递camera_id参数
      rclcpp::NodeOptions node_options;
      node_options.append_parameter_override("camera_id", rclcpp::ParameterValue(camera_id));

      // 从配置文件中读取对应相机的独立参数
      try
      {
        std::string config_file_path = ament_index_cpp::get_package_share_directory("hand_gesture_rec")
          + "/config" + "/hand_gesture_rec_params.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);

        std::string cam_param_key = "cam_" + std::to_string(camera_id) + "_parameters";
        if (config[cam_param_key])
        {
          YAML::Node cam_config = config[cam_param_key];
          LOG_INFO(log_project_path, "读取相机%d的独立配置", camera_id);

          // 传递相机独立配置参数
          if (cam_config["engine_path"])
            node_options.append_parameter_override("engine_path", rclcpp::ParameterValue(cam_config["engine_path"].as<std::string>()));
          if (cam_config["pose_engine_path"])
            node_options.append_parameter_override("pose_engine_path", rclcpp::ParameterValue(cam_config["pose_engine_path"].as<std::string>()));
          if (cam_config["usecalib"])
            node_options.append_parameter_override("usecalib", rclcpp::ParameterValue(cam_config["usecalib"].as<bool>()));
          if (cam_config["enable_visualization"])
            node_options.append_parameter_override("enable_visualization", rclcpp::ParameterValue(cam_config["enable_visualization"].as<bool>()));
          if (cam_config["min_wave_pairs"])
            node_options.append_parameter_override("min_wave_pairs", rclcpp::ParameterValue(cam_config["min_wave_pairs"].as<int>()));
          if (cam_config["use_real_3d"])
            node_options.append_parameter_override("use_real_3d", rclcpp::ParameterValue(cam_config["use_real_3d"].as<bool>()));
          if (cam_config["max_wave_y_movement"])
            node_options.append_parameter_override("max_wave_y_movement", rclcpp::ParameterValue(cam_config["max_wave_y_movement"].as<float>()));
          if (cam_config["min_wave_amplitude"])
            node_options.append_parameter_override("min_wave_amplitude", rclcpp::ParameterValue(cam_config["min_wave_amplitude"].as<float>()));
        }
        else
        {
          LOG_INFO(log_project_path, "相机%d无独立配置", camera_id);
        }
      }
      catch (const YAML::Exception& e)
      {
        LOG_ERROR(log_project_path, "解析相机配置失败: %s", e.what());
      }

      auto node = std::make_shared<HandGestureRecNode>(node_options, camera_id);
      nodes.push_back(node);
      executor.add_node(node);
      std::cout << "创建 HandGestureRecNode 节点，camera_id: " << camera_id << std::endl;
    }

    executor.spin();
    rclcpp::shutdown();
    return 0;
  }
}