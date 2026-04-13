#include "elevator_rec_node.h"
#include "bas_operate/file_operate.hpp"
#include "sys_info_src/sys_info_server.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

// ---------- 工具：从模型路径推导并加载 .names 文件 ----------
std::vector<std::string> Elevator_Rec_Node::loadClassNamesFromModel(const std::string& model_path)
{
    std::string names_path = model_path;
    size_t dot_pos = names_path.rfind('.');
    if (dot_pos != std::string::npos) {
        names_path = names_path.substr(0, dot_pos) + ".names";
    } else {
        names_path += ".names";
    }

    LOG_INFO(log_project_path_, "Loading class names from: %s", names_path.c_str());

    std::vector<std::string> names;
    std::ifstream f(names_path);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open class names file: " + names_path +
                                 ". Each model must have a corresponding .names file.");
    }

    std::string line;
    while (std::getline(f, line)) {
        size_t end = line.find_last_not_of(" \t\r\n");
        if (end != std::string::npos) {
            line = line.substr(0, end + 1);
        }
        if (!line.empty()) {
            names.push_back(line);
        }
    }

    if (names.empty()) {
        throw std::runtime_error("Class names file is empty: " + names_path);
    }

    LOG_INFO(log_project_path_, "Loaded %zu class names from %s", names.size(), names_path.c_str());
    return names;
}

// ---------- 工具：Mat -> PNG -> base64 ----------
static std::string mat2png_base64(const cv::Mat& img)
{
  std::vector<uchar> buf;
  cv::imencode(".png", img, buf);   // PNG 压缩
  static const char* tbl =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(buf.size() * 4 / 3 + 4);
  for (size_t i = 0; i < buf.size(); ) {
    uint32_t b = (buf[i++] & 0xFF) << 16;
    if (i < buf.size()) b |= (buf[i++] & 0xFF) << 8;
    if (i < buf.size()) b |= (buf[i++] & 0xFF);
    out.push_back(tbl[(b >> 18) & 0x3F]);
    out.push_back(tbl[(b >> 12) & 0x3F]);
    out.push_back(i > buf.size()     ? '=' : tbl[(b >> 6) & 0x3F]);
    out.push_back(i > buf.size() + 1 ? '=' : tbl[b & 0x3F]);
  }
  return out;
}

Elevator_Rec_Node::Elevator_Rec_Node(const rclcpp::NodeOptions &options, int cam_id)
  : Node("elevator_rec_node_" + std::to_string(cam_id), options), camera_id_(cam_id)
{
  // 预先声明所有可能被NodeOptions覆盖的参数
  this->declare_parameter("engine_path", "install/elevator_rec/models/ele_det.engine");
  this->declare_parameter("rec_engine_path", "install/elevator_rec/models/rec_int8.engine");
  this->declare_parameter("dict_path", "install/elevator_rec/dict/dict.txt");
  this->declare_parameter("usecalib", true);

  // 根据camera_id生成日志项目路径
  const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
  log_project_path_ = project_name + "_cam_" + std::to_string(camera_id_);

  // 新增相机类型参数
  camera_type_ = this->declare_parameter("camera_type", "realsense");

  // 机械臂ID参数 - 用于获取对应的标定矩阵
  arm_id_ = this->declare_parameter("arm_id", 0);

  // 初始化arm_id_list_
  arm_id_list_.clear();

  // 获取引擎路径参数
  engine_name_ = this->get_parameter("engine_path").as_string();
  rec_engine_name_ = this->get_parameter("rec_engine_path").as_string();
  dict_path_ = this->get_parameter("dict_path").as_string();
  LOG_INFO(log_project_path_, "Engine path: %s", engine_name_.c_str());

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

  // 尝试从launch文件获取话题名称（优先级最高）
  std::string launch_color_topic = this->declare_parameter("color_image_topic", "");
  std::string launch_depth_topic = this->declare_parameter("depth_image_topic", "");
  std::string launch_camera_info_topic = this->declare_parameter("camera_info_topic", "");

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
  camera_intrinsics_initialized_ = false;
  fx_ = 0.0f;
  fy_ = 0.0f;
  cx_ = 0.0f;
  cy_ = 0.0f;

  // 初始化标定参数处理器
  initCalibParamHandler();

  // 声明usecalib参数
  usecalib_ = this->get_parameter("usecalib").as_bool();
  LOG_INFO(log_project_path_, "标定模式: %s", usecalib_ ? "开启" : "关闭");

  LOG_INFO(log_project_path_, "Camera ID: %d", camera_id_);
  LOG_INFO(log_project_path_, "Arm ID: %d", arm_id_);
  LOG_INFO(log_project_path_, "Color Image Topic: %s", color_image_topic_.c_str());
  LOG_INFO(log_project_path_, "Depth Image Topic: %s", depth_image_topic_.c_str());
  LOG_INFO(log_project_path_, "Camera Info Topic: %s", camera_info_topic_.c_str());

  // 初始化新增功能 - 解耦合设计
  setup_camera_intrinsics();  // 设置相机内参
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  //publish_static_tf();        // 发布静态TF

  elevator_rec_.load_engine(engine_name_, rec_engine_name_, dict_path_);
  class_names_ = loadClassNamesFromModel(engine_name_);
  color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(color_image_topic_, 10, std::bind(&Elevator_Rec_Node::Color_Callback, this, std::placeholders::_1));
  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(depth_image_topic_, 10, std::bind(&Elevator_Rec_Node::Depth_Callback, this, std::placeholders::_1));
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10, std::bind(&Elevator_Rec_Node::CameraInfo_Callback, this, std::placeholders::_1));

  // 生成话题前缀：根据camera_id
  std::string topic_prefix = "/cam_" + std::to_string(camera_id_);

  // 创建发布者 - 使用相机前缀
  std::string box_image_topic = topic_prefix + "/ele_det_image";
  std::string res_image_topic = topic_prefix + "/ele_det_image_png";
  std::string det_res_topic = topic_prefix + "/ele_det_res";
  std::string camera_command_topic = topic_prefix + "/camera_command";
  std::string elevator_status_topic = topic_prefix + "/camera_result";
  std::string detection_topic = topic_prefix + "/ele_det_result";

  box_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(box_image_topic, 1);
  res_image_pub_ = this->create_publisher<std_msgs::msg::String>(res_image_topic, 1);
  det_res_pub_ = this->create_publisher<elevator_rec_msgs::msg::ElevatorRecognitionArray>(det_res_topic, 1);
  camera_command_sub_ = this->create_subscription<custom_msgs_comm::msg::ElevatorCommand>(camera_command_topic, 1, std::bind(&Elevator_Rec_Node::Camera_Command_Callback, this, std::placeholders::_1));
  elevator_status_pub_ = this->create_publisher<std_msgs::msg::String>(elevator_status_topic, 10);
  detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2D>(detection_topic, 10);

  LOG_INFO(log_project_path_, "Publishing to: %s", box_image_topic.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", res_image_topic.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", det_res_topic.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", camera_command_topic.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", elevator_status_topic.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", detection_topic.c_str());

  // 初始化按钮状态相关变量
  button_pressed_ = false;
  monitoring_floor_arrival_ = false;
  button_detection_saved_ = false;
}

void Elevator_Rec_Node::Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  
  try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        std::vector<ButtonRecognition> buttonResults;
        buttonResults.clear();
        buttonResults = elevator_rec_.infer(frame, class_names_);
        Intrinsics intrinsics;
        intrinsics.fx = fx_;
        intrinsics.fy = fy_;
        intrinsics.cx = cx_;
        intrinsics.cy = cy_;
        
        // 线程安全地保存最新检测结果和当前帧图像
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            latest_button_results_ = buttonResults;
            latest_header_ = msg->header;
            image_ = frame.clone();  // 保存当前帧图像
        }
        
        publish(buttonResults, msg->header);

        elevator_rec_.draw_results(frame, buttonResults, depth_frame_, intrinsics, class_names_);
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        box_image_pub_->publish(*img_msg);
        cv::Size target(160, 120); 
        cv::Mat resized;
        cv::resize(frame, resized, target, 0, 0, cv::INTER_LINEAR);    
        auto png_base64 = mat2png_base64(resized);
        auto out = std_msgs::msg::String();
        out.data = std::move(png_base64);
        res_image_pub_->publish(out);
        
        // 检查是否有待处理的命令
        std::string command_to_process;
        int current_floor_to_process = 0;
        int target_floor_to_process = 0;
        bool has_command = false;
        
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (!camera_command_.empty()) {
                command_to_process = camera_command_;
                current_floor_to_process = current_floor_;
                target_floor_to_process = target_floor_;
                
                // 清空命令，避免重复处理
                camera_command_.clear();
                has_command = true;
            }
        }
        
        // 在锁外处理命令，避免死锁
        if (has_command) {
            processCameraCommand(command_to_process, current_floor_to_process, target_floor_to_process);
        }
        
        // 处理持续等待的命令
        processWaitingCommand(buttonResults);        
      } 
      catch (const cv_bridge::Exception& e) {
        LOG_ERROR(log_project_path_, "trans image error: %s", e.what());
      }
}
  
void Elevator_Rec_Node::Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try 
  {
    depth_frame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image; 
  } 
  catch (const cv_bridge::Exception& e) 
  {
    LOG_ERROR(log_project_path_, "trans depth image error: %s", e.what());
  }
}

// 使用parseCommInfo初始化话题名
void Elevator_Rec_Node::initTopicNames()
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
bool Elevator_Rec_Node::getSysDat()
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
bool Elevator_Rec_Node::getCamConfigInfo(SysConfig::CamConfigInfo& cam_info)
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
void Elevator_Rec_Node::initCalibParamHandler()
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
          std::string cam_to_base_str;
          std::string cam_to_base_param = param_prefix + ".cam_to_base_transform";
          try {
              cam_to_base_str = sys_config_client_->get_parameter<std::string>(cam_to_base_param);
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
void Elevator_Rec_Node::setup_camera_intrinsics()
{
    // 等待相机信息话题发布内参数据
    // 这里不设置硬编码值，而是等待CameraInfo回调函数获取真实的内参数据
    LOG_INFO(log_project_path_, "等待相机内参数据...");
    LOG_INFO(log_project_path_, "订阅相机信息话题: %s", camera_info_topic_.c_str());
}

// 相机信息回调函数 - 通过API获取真实的内参数据
void Elevator_Rec_Node::CameraInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
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
            // 使用合理的默认值
            if (camera_type_ == "Gemini") 
            {
                fx_ = 610.0f;
                fy_ = 610.0f;
                cx_ = msg->width / 2.0f;
                cy_ = msg->height / 2.0f;
            }
            else 
            {
                fx_ = 615.0f;
                fy_ = 615.0f;
                cx_ = msg->width / 2.0f;
                cy_ = msg->height / 2.0f;
            }
        }
    }
    else 
    {
        LOG_WARN(log_project_path_, "CameraInfo消息格式错误，使用默认内参");
        // 使用默认内参
        if (camera_type_ == "Gemini") 
        {
            fx_ = 610.0f;
            fy_ = 610.0f;
            cx_ = 320.0f;
            cy_ = 240.0f;
        }
        else 
        {
            fx_ = 615.0f;
            fy_ = 615.0f;
            cx_ = 320.0f;
            cy_ = 240.0f;
        }
        camera_intrinsics_initialized_ = true;
    }
}

void Elevator_Rec_Node::Camera_Command_Callback(const custom_msgs_comm::msg::ElevatorCommand::SharedPtr msg)
{
    // 线程安全地记录命令和参数
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        
        if (msg->elevator_command == "wait_elevator_door_open" ||
            msg->elevator_command == "wait_floor_arrived") {
            // 等待类指令设置为持续状态
            waiting_command_ = msg->elevator_command;
            LOG_INFO(log_project_path_, "开始持续等待指令: %s", msg->elevator_command.c_str());
        } else {
            // 其他指令即时处理
            camera_command_ = msg->elevator_command;
            current_floor_ = msg->current_floor;
            target_floor_ = msg->target_floor;
            LOG_INFO(log_project_path_, "接收到即时指令: %s (当前楼层: %d, 目标楼层: %d)", 
                        msg->elevator_command.c_str(), msg->current_floor, msg->target_floor);
        }
    }
}

// 发布相机到base_link的静态TF
void Elevator_Rec_Node::publish_static_tf()
{
    camera_to_base_tf_.header.stamp = this->now();
    camera_to_base_tf_.header.frame_id = "base_link";
    camera_to_base_tf_.child_frame_id = "camera_link";
    
    // 从参数服务器获取相机位置（支持动态配置）
    double camera_x = this->declare_parameter("cam_x", 0.1);   // x方向偏移
    double camera_y = this->declare_parameter("cam_y", 0.0);   // y方向偏移
    double camera_z = this->declare_parameter("cam_z", 1.1);   // z方向偏移
    
    // 从参数服务器获取相机旋转角度
    double camera_roll = this->declare_parameter("cam_roll", 0.0);   // roll角度
    double camera_pitch = this->declare_parameter("cam_pitch", 0.23); // pitch角度
    double camera_yaw = this->declare_parameter("cam_yaw", 0.0);     // yaw角度
    
    // 设置平移
    camera_to_base_tf_.transform.translation.x = camera_x;
    camera_to_base_tf_.transform.translation.y = camera_y;
    camera_to_base_tf_.transform.translation.z = camera_z;
    
    // 设置旋转（欧拉角：roll, pitch, yaw）
    tf2::Quaternion q;
    q.setRPY(camera_roll, camera_pitch, camera_yaw);
    camera_to_base_tf_.transform.rotation = tf2::toMsg(q);
    
    static_tf_broadcaster_->sendTransform(camera_to_base_tf_);
    LOG_INFO(log_project_path_, "发布静态TF: base_link -> camera_link, 位置: (%.2f, %.2f, %.2f), 旋转: (%.2f, %.2f, %.2f)", 
                camera_x, camera_y, camera_z, camera_roll, camera_pitch, camera_yaw);
}

// 计算camera_link下的3D坐标
geometry_msgs::msg::Point Elevator_Rec_Node::calculate_3d_position(const Detection& det)
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

    // 获取边界框中心点
    float center_x = det.bbox[0];  // 边界框中心x
    float center_y = det.bbox[1];  // 边界框中心y

    // 确保深度图像有效
    if (depth_frame_.empty())
    {
        LOG_WARN(log_project_path_, "深度图像为空，无法计算3D坐标");
        return camera_point;
    }

    // 检查像素坐标是否在图像范围内
    int pixel_x = static_cast<int>(center_x);
    int pixel_y = static_cast<int>(center_y);

    if (pixel_x < 0 || pixel_x >= depth_frame_.cols ||
        pixel_y < 0 || pixel_y >= depth_frame_.rows)
    {
        LOG_WARN(log_project_path_, "像素坐标超出图像范围: (%d, %d)", pixel_x, pixel_y);
        return camera_point;
    }

    // 获取深度值
    uint16_t depth_value = depth_frame_.at<uint16_t>(pixel_y, pixel_x);

    if (depth_value == 0)
    {
        LOG_WARN(log_project_path_, "深度值为0，无效深度");
        return camera_point;
    }

    // 输出为毫米
    float depth_mm = depth_value;
    camera_point.x = (center_x - cx_) * depth_mm / fx_; // X坐标
    camera_point.y = (center_y - cy_) * depth_mm / fy_; // Y坐标
    camera_point.z = depth_mm;

    return camera_point;
}

// 将camera_link坐标转换为base_link坐标
geometry_msgs::msg::Point Elevator_Rec_Node::transform_to_base_link(const geometry_msgs::msg::Point& camera_point)
{
    geometry_msgs::msg::Point base_point;
    
    // 使用TF变换将camera_link坐标转换到base_link
    tf2::Vector3 camera_vec(camera_point.x, camera_point.y, camera_point.z);
    tf2::Transform base_to_camera;
    
    // 从TransformStamped中提取变换（注意：camera_to_base_tf_表示从base_link到camera_link的变换）
    tf2::fromMsg(camera_to_base_tf_.transform, base_to_camera);
    
    // 获取从camera_link到base_link的逆变换
    tf2::Transform camera_to_base = base_to_camera.inverse();
    
    // 应用变换：camera_link -> base_link
    tf2::Vector3 base_vec = camera_to_base * camera_vec;
    
    base_point.x = base_vec.x();
    base_point.y = base_vec.y();
    base_point.z = base_vec.z();
    
    return base_point;
}

void Elevator_Rec_Node::publish(const std::vector<ButtonRecognition>& buttonResults,
                const std_msgs::msg::Header& header)   // 把图像时间戳/坐标系带过来
{
    (void)header;  // 消息类型不支持header，暂时忽略
    elevator_rec_msgs::msg::ElevatorRecognitionArray arr;

    for (const auto& res : buttonResults)
    {
        elevator_rec_msgs::msg::ElevatorRecognition elevator_rec;
        elevator_rec.text = res.rec.text;
        elevator_rec.score = res.rec.score;
        elevator_rec.det.bbox.center.position.x = res.det.bbox[0];
        elevator_rec.det.bbox.center.position.y = res.det.bbox[1];
        elevator_rec.det.bbox.size_x              = res.det.bbox[2];
        elevator_rec.det.bbox.size_y              = res.det.bbox[3];

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = class_names_[res.det.class_id];
        hyp.hypothesis.score = res.det.conf;

        // 新增：使用深度图像计算3D坐标并转换到base_link坐标系
        if (!depth_frame_.empty())
        {
            // 计算camera_link下的3D坐标
            geometry_msgs::msg::Point camera_point = calculate_3d_position(res.det);
            // std::cout << "camera_point:" << camera_point.x << "," << camera_point.y << "," << camera_point.z << std::endl;
            // 转换为base_link坐标系
            geometry_msgs::msg::Point base_point = camera_point;

            // 如果启用标定模式，将相机坐标系转换为base坐标系
            if (usecalib_ && calib_result_ && !calib_result_->cam_to_base_transform.empty())
            {
                try {
                    std::vector<double> marker_pose = {camera_point.x, camera_point.y, camera_point.z, 0.0, 0.0, 0.0};
                    std::vector<double> offset_compensation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    if (!calib_result_->offset_compensation.empty() && calib_result_->offset_compensation.size() >= 6) {
                        offset_compensation = calib_result_->offset_compensation;
                    }
                    auto trans_result = handeyecalib::computeRobotPoseFromMarker(
                        marker_pose, calib_result_->cam_to_base_transform, &offset_compensation);
                    if (!trans_result.transformed_pose.empty())
                    {
                        base_point.x = trans_result.transformed_pose[0] + calib_result_->offset_compensation[0];
                        base_point.y = trans_result.transformed_pose[1] + calib_result_->offset_compensation[1];
                        base_point.z = trans_result.transformed_pose[2] + calib_result_->offset_compensation[2];

                        LOG_DEBUG(log_project_path_, "3D位置已转换到base坐标系: (%.3f, %.3f, %.3f)",
                                    base_point.x, base_point.y, base_point.z);
                    }
                }
                catch (const std::exception& e) {
                    LOG_WARN(log_project_path_, "坐标转换失败: %s", e.what());
                }
            }
            hyp.pose.pose.position = base_point;

            // 设置姿态（默认朝向）
            hyp.pose.pose.orientation.x = 0.0;
            hyp.pose.pose.orientation.y = 0.0;
            hyp.pose.pose.orientation.z = 0.0;
            hyp.pose.pose.orientation.w = 1.0;

            LOG_DEBUG(log_project_path_, "检测目标3D坐标 - camera_link: (%.3f, %.3f, %.3f), base_link: (%.3f, %.3f, %.3f)",
                        camera_point.x, camera_point.y, camera_point.z,
                        base_point.x, base_point.y, base_point.z);
        }
        else
        {
            // 如果没有深度图像，使用原有的3D坐标
            hyp.pose.pose.position.x = 0.0;
            hyp.pose.pose.position.y = 0.0;
            hyp.pose.pose.position.z = 0.0;
            hyp.pose.pose.orientation.x = 0.0;
            hyp.pose.pose.orientation.y = 0.0;
            hyp.pose.pose.orientation.z = 0.0;
            hyp.pose.pose.orientation.w = 1.0;
        }
        elevator_rec.det.results.push_back(hyp);
        arr.elevator_recognitions.push_back(elevator_rec);
    }
    det_res_pub_->publish(arr);
}

void Elevator_Rec_Node::publish(const Detection& buttonResult, const std_msgs::msg::Header& header)
{
    vision_msgs::msg::Detection2D det;
    det.bbox.center.position.x = buttonResult.bbox[0];
    det.bbox.center.position.y = buttonResult.bbox[1];
    det.bbox.size_x              = buttonResult.bbox[2];
    det.bbox.size_y              = buttonResult.bbox[3];

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = class_names_[buttonResult.class_id];
    hyp.hypothesis.score = buttonResult.conf;

    // 新增：使用深度图像计算3D坐标并转换到base_link坐标系
    if (!depth_frame_.empty())
    {
        // 计算camera_link下的3D坐标
        geometry_msgs::msg::Point camera_point = calculate_3d_position(buttonResult);
        // 转换为base_link坐标系
        geometry_msgs::msg::Point base_point = camera_point;

        // 如果启用标定模式，将相机坐标系转换为base坐标系
        if (usecalib_ && calib_result_ && !calib_result_->cam_to_base_transform.empty())
        {
            try {
                std::vector<double> marker_pose = {camera_point.x, camera_point.y, camera_point.z, 0.0, 0.0, 0.0};
                std::vector<double> offset_compensation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                if (!calib_result_->offset_compensation.empty() && calib_result_->offset_compensation.size() >= 6) {
                    offset_compensation = calib_result_->offset_compensation;
                }
                auto trans_result = handeyecalib::computeRobotPoseFromMarker(
                    marker_pose, calib_result_->cam_to_base_transform, &offset_compensation);
                if (!trans_result.transformed_pose.empty())
                {
                    base_point.x = trans_result.transformed_pose[0] + calib_result_->offset_compensation[0];
                    base_point.y = trans_result.transformed_pose[1] + calib_result_->offset_compensation[1];
                    base_point.z = trans_result.transformed_pose[2] + calib_result_->offset_compensation[2];
                }
            }
            catch (const std::exception& e) {
                LOG_WARN(log_project_path_, "坐标转换失败: %s", e.what());
            }
        }
        hyp.pose.pose.position = base_point;
        hyp.pose.pose.orientation.x = 0.0;
        hyp.pose.pose.orientation.y = 0.0;
        hyp.pose.pose.orientation.z = 0.0;
        hyp.pose.pose.orientation.w = 1.0;
    }
    else
    {
        hyp.pose.pose.position.x = 0.0;
        hyp.pose.pose.position.y = 0.0;
        hyp.pose.pose.position.z = 0.0;
        hyp.pose.pose.orientation.x = 0.0;
        hyp.pose.pose.orientation.y = 0.0;
        hyp.pose.pose.orientation.z = 0.0;
        hyp.pose.pose.orientation.w = 1.0;
    }
    det.results.push_back(hyp);
    detection_pub_->publish(det);
}

void Elevator_Rec_Node::saveButtonDetection(const ButtonRecognition& button_result, const cv::Mat& current_image)
{
    if (current_image.empty()) {
        LOG_WARN(log_project_path_, "当前图像为空，无法保存按钮检测");
        return;
    }
    
    // 裁剪按钮图像
    int x = static_cast<int>(button_result.det.bbox[0] - button_result.det.bbox[2] / 2);
    int y = static_cast<int>(button_result.det.bbox[1] - button_result.det.bbox[3] / 2);
    int w = static_cast<int>(button_result.det.bbox[2]);
    int h = static_cast<int>(button_result.det.bbox[3]);
    
    // 确保坐标在图像范围内
    x = std::max(0, std::min(x, current_image.cols - 1));
    y = std::max(0, std::min(y, current_image.rows - 1));
    w = std::min(w, current_image.cols - x);
    h = std::min(h, current_image.rows - y);
    
    if (w <= 0 || h <= 0) {
        LOG_WARN(log_project_path_, "边界框无效，无法保存按钮检测");
        return;
    }
    
    cv::Rect button_rect(x, y, w, h);
    
    // 线程安全地保存按钮检测数据和图像
    {
        std::lock_guard<std::mutex> lock(button_save_mutex_);
        saved_button_detection_ = button_result;
        saved_button_image_ = current_image(button_rect).clone();
        button_detection_saved_ = true;
    }
    
    LOG_INFO(log_project_path_, "保存按钮检测: 类别=%s, 文本=%s", 
                class_names_[button_result.det.class_id].c_str(), 
                button_result.rec.text.c_str());
}

void Elevator_Rec_Node::processCameraCommand(const std::string& command, int current_floor, int target_floor)
{
    auto status_msg = std_msgs::msg::String();
    
    if (command == "find_floor_button")
    {
        // 线程安全地获取最新检测结果
        std::vector<ButtonRecognition> current_results;
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            current_results = latest_button_results_;
        }
        
        // 查找目标楼层按钮
        std::string target_text = std::to_string(target_floor);
        bool found = false;
        
        for (const auto& res : current_results)
        {
            if (res.rec.text == target_text)
            {
                // 找到目标楼层按钮
                publish(res.det, latest_header_);
                status_msg.data = "button_found:" + target_text;
                found = true;
                
                // 保存按钮检测，用于后续状态判断
                cv::Mat current_image;
                {
                    std::lock_guard<std::mutex> img_lock(detection_mutex_);
                    current_image = image_.clone();
                }
                saveButtonDetection(res, current_image);
                
                // 设置按钮按下监控
                {
                    std::lock_guard<std::mutex> cmd_lock(command_mutex_);
                    button_pressed_ = false;
                    monitoring_floor_arrival_ = true;
                }
                
                LOG_INFO(log_project_path_, "找到目标楼层按钮: %s", target_text.c_str());
                break;
            }
        }
        
        if (!found)
        {
            status_msg.data = "button_not_found:" + target_text;
            LOG_WARN(log_project_path_, "未找到目标楼层按钮: %s", target_text.c_str());
        }
        
        elevator_status_pub_->publish(status_msg);
    }
    else if (command == "find_open_button" || command == "find_close_button")
    {
        // 线程安全地获取最新检测结果
        std::vector<ButtonRecognition> current_results;
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            current_results = latest_button_results_;
        }
        
        std::string button_name = (command == "find_open_button") ? "open" : "close";
        bool found = false;
        
        for (const auto& res : current_results)
        {
            if (class_names_[res.det.class_id] == button_name)
            {
                // 找到目标按钮
                publish(res.det, latest_header_);
                status_msg.data = button_name + "_button_found";
                found = true;
                LOG_INFO(log_project_path_, "找到%s按钮", button_name.c_str());
                break;
            }
        }
        
        if (!found)
        {
            status_msg.data = button_name + "_button_not_found";
            LOG_WARN(log_project_path_, "未找到%s按钮", button_name.c_str());
        }
        
        elevator_status_pub_->publish(status_msg);
    }
    else
    {
        status_msg.data = "未知指令: " + command;
        elevator_status_pub_->publish(status_msg);
    }
    
    if (!status_msg.data.empty()) {
        LOG_INFO(log_project_path_, "回复: %s", status_msg.data.c_str());
    }
}

void Elevator_Rec_Node::processWaitingCommand(const std::vector<ButtonRecognition>& current_results)
{
    std::string waiting_command;
    bool button_pressed;
    bool monitoring_floor_arrival;
    cv::Mat current_image;
    
    // 线程安全地获取等待的命令和状态
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        waiting_command = waiting_command_;
        button_pressed = button_pressed_;
        monitoring_floor_arrival = monitoring_floor_arrival_;
        
        // 获取当前图像用于按钮状态检测
        if (!image_.empty()) {
            current_image = image_.clone();
        }
    }
    
    auto status_msg = std_msgs::msg::String();
    bool command_completed = false;
    
    // 处理按钮按下后的楼层到达检测
    if (monitoring_floor_arrival && button_pressed && !current_image.empty())
    {
        bool is_currently_pressed = isButtonPressed(current_image);
        
        // 如果按钮从按下状态变为未按下状态，说明楼层已到达
        if (!is_currently_pressed)
        {
            status_msg.data = "floor_arrived";
            command_completed = true;
            
            // 重置按钮状态
            {
                std::lock_guard<std::mutex> lock(command_mutex_);
                button_pressed_ = false;
                monitoring_floor_arrival_ = false;
            }
            
            elevator_status_pub_->publish(status_msg);
            LOG_INFO(log_project_path_, "楼层到达检测完成，按钮状态已重置");
            return;  // 楼层到达检测完成，直接返回
        }
    }
    
    if (waiting_command.empty()) {
        return;  // 没有等待的命令
    }
    
    if (waiting_command == "wait_elevator_door_open")
    {
        for(int i = 0; i < current_results.size(); i++)
        {
            if(class_names_[current_results[i].det.class_id] == "door_open")
            {
                status_msg.data = "door_open";
                command_completed = true;
                break;
            }
        }
    }
    else if (waiting_command == "wait_floor_arrived")
    {
        for(int i = 0; i < current_results.size(); i++)
        {
            if(class_names_[current_results[i].det.class_id] == "door_open")
            {
                status_msg.data = "floor_arrived";
                command_completed = true;
                break;
            }
        }
    }
    
    // 如果命令完成，清空等待状态并发布结果
    if (command_completed)
    {
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            waiting_command_.clear();
        }
        
        elevator_status_pub_->publish(status_msg);
        LOG_INFO(log_project_path_, "等待指令完成: %s, 回复: %s", 
                    waiting_command.c_str(), status_msg.data.c_str());
    }
}

bool Elevator_Rec_Node::isButtonPressed(const cv::Mat& current_image)
{
    // 检查是否已保存按钮检测数据
    bool has_saved_data = false;
    ButtonRecognition saved_detection;
    cv::Mat saved_image;
    
    {
        std::lock_guard<std::mutex> lock(button_save_mutex_);
        has_saved_data = button_detection_saved_;
        if (has_saved_data) {
            saved_detection = saved_button_detection_;
            saved_image = saved_button_image_.clone();
        }
    }
    
    if (!has_saved_data) {
        LOG_WARN(log_project_path_, "没有保存的按钮数据，无法判断状态");
        return false;
    }
    
    if (current_image.empty()) {
        LOG_WARN(log_project_path_, "当前图像为空，无法判断状态");
        return false;
    }
    
    // 直接使用保存的检测结果裁剪当前图像
    int x = static_cast<int>(saved_detection.det.bbox[0] - saved_detection.det.bbox[2] / 2);
    int y = static_cast<int>(saved_detection.det.bbox[1] - saved_detection.det.bbox[3] / 2);
    int w = static_cast<int>(saved_detection.det.bbox[2]);
    int h = static_cast<int>(saved_detection.det.bbox[3]);
    
    // 确保坐标在图像范围内
    x = std::max(0, std::min(x, current_image.cols - 1));
    y = std::max(0, std::min(y, current_image.rows - 1));
    w = std::min(w, current_image.cols - x);
    h = std::min(h, current_image.rows - y);
    
    if (w <= 0 || h <= 0) {
        LOG_WARN(log_project_path_, "保存的边界框无效或超出图像范围");
        return false;
    }
    
    cv::Rect button_rect(x, y, w, h);
    cv::Mat current_button_image = current_image(button_rect).clone();
    
    // 检查图像有效性
    if (saved_image.empty() || current_button_image.empty()) {
        LOG_WARN(log_project_path_, "保存的图像或当前按钮图像为空");
        return false;
    }
    
    // 判断对比度是否提升（阈值设为1.1，即提升10%）
    bool is_pressed = isContrastImproved(saved_image, current_button_image, 1.1);
    
    if (is_pressed) {
        LOG_INFO(log_project_path_, "按钮已被按下");
    } else {
        LOG_INFO(log_project_path_, "按钮未被按下");
    }
    
    return is_pressed;
}

 void Elevator_Rec_Node::calcMeanContrast(const cv::Mat& img, double& mean, double& contrast)
{
    if (img.empty())
        throw std::invalid_argument("input image is empty");

    // 保证单通道
    cv::Mat gray;
    if (img.channels() == 1)
        gray = img;
    else if (img.channels() == 3)
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else if (img.channels() == 4)
        cv::cvtColor(img, gray, cv::COLOR_BGRA2GRAY);
    else
        throw std::invalid_argument("unsupported channel count");

    cv::Scalar m = cv::mean(gray);
    mean = m[0];

    cv::Mat dev;
    cv::meanStdDev(gray, cv::noArray(), dev);
    contrast = dev.at<double>(0);
}

bool Elevator_Rec_Node::isContrastImproved(const cv::Mat& img1, const cv::Mat& img2, double thr)
{
    double mean1, contrast1;
    double mean2, contrast2;

    calcMeanContrast(img1, mean1, contrast1);
    calcMeanContrast(img2, mean2, contrast2);

    // 避免除以 0
    if (contrast1 < 1e-6 || contrast2 < 1e-6)
        return false;

    // double ratio1 = mean1 / contrast1;
    // double ratio2 = mean2 / contrast2;
    double ratio1 = mean1;
    double ratio2 = mean2;

    return ratio2 > ratio1 * thr;
}



void test()
{
  std::string det_engine_path = "install/elevator_rec/models/ele_det.engine";
  std::string rec_engine_path = "install/elevator_rec/models/rec_int8.engine";
  std::string dict_path = "install/elevator_rec/dict/dict.txt";
  std::string input_folder = "src/perception/elevator_rec/elevator_rec/test_imgs";
  std::string output_folder = "output_results";

  std::string names_path = det_engine_path;
  size_t dot_pos = names_path.rfind('.');
  if (dot_pos != std::string::npos) {
    names_path = names_path.substr(0, dot_pos) + ".names";
  } else {
    names_path += ".names";
  }
  std::vector<std::string> class_names;
  std::ifstream f(names_path);
  if (f.is_open()) {
    std::string line;
    while (std::getline(f, line)) {
      size_t end = line.find_last_not_of(" \t\r\n");
      if (end != std::string::npos) line = line.substr(0, end + 1);
      if (!line.empty()) class_names.push_back(line);
    }
  }
  if (class_names.empty()) {
    std::cerr << "Warning: Failed to load class names from " << names_path << std::endl;
  }

  if (access(output_folder.c_str(), 0) == -1)
  {
      mkdir(output_folder.c_str(), 0755);
      std::cout << "创建输出文件夹: " << output_folder << std::endl;
  }

  ElevatorButtonRec buttonRec;
  buttonRec.load_engine(det_engine_path, rec_engine_path, dict_path);

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
      std::vector<ButtonRecognition> buttonResults;
      try {
          buttonResults = buttonRec.infer(img, class_names);
      }
      catch (const std::exception& e) {
          std::cerr << "检测异常: " << e.what() << std::endl;
          continue;
      }

      auto end = std::chrono::high_resolution_clock::now();
      double inference_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

      std::cout << "识别到 " << buttonResults.size() << " 个目标, 耗时: " << inference_time << " ms" << std::endl;

      if (!buttonResults.empty())
      {
          std::cout << "==== 识别结果 ====" << std::endl;
          for (const auto& res : buttonResults)
          {
              std::cout << "Box: [" << std::fixed << std::setprecision(2)
                      << res.det.bbox[0] << "," << res.det.bbox[1] << ","
                      << res.det.bbox[2] << "," << res.det.bbox[3] << "]"
                      << " Score: " << res.det.conf << " Text: " << res.rec.text << std::endl;
          }
      }

      cv::Mat result_img = img.clone();
      buttonRec.draw_results(result_img, buttonResults, class_names);

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

// Elevator Recognition主节点 - 用于管理多个相机节点
class ElevatorRecMainNode : public rclcpp::Node
{
public:
  explicit ElevatorRecMainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("elevator_rec_main_node", options)
  {
    log_project_path_ = basmodule::get_project_name_by_file_path(__FILE__);
    std::string sys_config_node_name = "sys_config_ros_node";
    sys_config_client_ = std::make_shared<rclcpp::SyncParametersClient>(
      this,
      sys_config_node_name
    );
  }

  std::vector<int> getServerCameraIds()
  {
    cam_server_ids_.clear();
    try
    {
      if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
      {
        LOG_ERROR(log_project_path_, "无法连接到系统配置参数服务");
        return cam_server_ids_;
      }
      std::vector<uint8_t> temp_camera_ids = RosComm::getCamIdsFromServer(sys_config_client_);
      std::string cam_ids_str;
      for (const auto& id : temp_camera_ids) {
        cam_server_ids_.push_back(static_cast<int>(id));
        cam_ids_str += std::to_string(id) + " ";
      }
      LOG_INFO(log_project_path_, "从系统配置参数服务获取相机ID列表: %s", cam_ids_str.c_str());
    }
    catch (const YAML::Exception& e) {
      LOG_ERROR(log_project_path_, "解析配置文件出错: %s", e.what());
    }
    return cam_server_ids_;
  }

  std::vector<int> getConfigCameraIds()
  {
    try
    {
      cam_config_ids_.clear();
      std::string config_file_path = ament_index_cpp::get_package_share_directory("elevator_rec")
        + "/config" + "/elevator_rec_params.yaml";
      YAML::Node config = YAML::LoadFile(config_file_path);
      std::string cam_ids_str;
      if (config["ros__parameters"] && config["ros__parameters"]["camera_id"])
      {
        YAML::Node camera_id_node = config["ros__parameters"]["camera_id"];
        if (camera_id_node.IsSequence()) {
          for (const auto& id : camera_id_node) {
            cam_config_ids_.push_back(id.as<int>());
            cam_ids_str += std::to_string(id.as<int>()) + " ";
          }
        }
      }
      LOG_INFO(log_project_path_, "从配置文件获取相机ID列表: %s", cam_ids_str.c_str());
    }
    catch (const YAML::Exception& e)
    {
      LOG_ERROR(log_project_path_, "解析配置文件出错: %s", e.what());
    }
    return cam_config_ids_;
  }

  std::vector<int> getActiveCameraIds()
  {
    std::vector<int> act_cam_ids;
    cam_server_ids_ = getServerCameraIds();
    cam_config_ids_ = getConfigCameraIds();
    std::string cam_ids_str;
    for (int camera_id : cam_config_ids_)
    {
      if (std::find(cam_server_ids_.begin(), cam_server_ids_.end(), camera_id) != cam_server_ids_.end())
      {
        act_cam_ids.push_back(camera_id);
        cam_ids_str += std::to_string(camera_id) + " ";
      }
      else
        LOG_ERROR(log_project_path_, "参数服务器没有配置该相机ID，camera_id: %d", camera_id);
    }
    LOG_INFO(log_project_path_, "有效配置的相机ID列表: %s", cam_ids_str.c_str());
    cam_active_ids_ = act_cam_ids;
    return act_cam_ids;
  }

private:
  rclcpp::SyncParametersClient::SharedPtr sys_config_client_;
  std::vector<int> cam_server_ids_;
  std::vector<int> cam_config_ids_;
  std::vector<int> cam_active_ids_;
  std::string log_project_path_;
};

int main(int argc, char **argv)
{
  if(0)
  {
    test();
  }
  else
  {
    // 构造新的命令行参数，包含配置文件路径
    std::vector<std::string> new_argv_strings;
    std::vector<char*> new_argv;

    for (int i = 1; i < argc; i++)
    {
      new_argv_strings.push_back(std::string(argv[i]));
    }
    std::string default_config_file_path = ament_index_cpp::get_package_share_directory("elevator_rec")
      + "/config" + "/elevator_rec_params.yaml";
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
    LOG_INFO(log_project_path, "elevator_recognition节点启动");
    rclcpp::executors::MultiThreadedExecutor executor;

    auto main_node = std::make_shared<ElevatorRecMainNode>();
    std::vector<int> act_cam_ids = main_node->getActiveCameraIds();

    if (act_cam_ids.empty())
    {
      LOG_ERROR(log_project_path, "有效配置的相机ID列表为空,默认使用相机ID 0");
      act_cam_ids = {0};
    }

    std::vector<std::shared_ptr<Elevator_Rec_Node>> nodes;
    for (int camera_id : act_cam_ids)
    {
      LOG_INFO(log_project_path, "创建 Elevator_Rec_Node 节点，camera_id: %d", camera_id);

      // 通过NodeOptions传递camera_id参数
      rclcpp::NodeOptions node_options;
      node_options.append_parameter_override("camera_id", rclcpp::ParameterValue(camera_id));

      // 从配置文件中读取对应相机的独立参数
      try
      {
        std::string config_file_path = ament_index_cpp::get_package_share_directory("elevator_rec")
          + "/config" + "/elevator_rec_params.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);

        std::string cam_param_key = "cam_" + std::to_string(camera_id) + "_parameters";
        if (config[cam_param_key])
        {
          YAML::Node cam_config = config[cam_param_key];
          LOG_INFO(log_project_path, "读取相机%d的独立配置", camera_id);

          // 传递相机独立配置参数
          if (cam_config["engine_path"])
            node_options.append_parameter_override("engine_path", rclcpp::ParameterValue(cam_config["engine_path"].as<std::string>()));
          if (cam_config["rec_engine_path"])
            node_options.append_parameter_override("rec_engine_path", rclcpp::ParameterValue(cam_config["rec_engine_path"].as<std::string>()));
          if (cam_config["dict_path"])
            node_options.append_parameter_override("dict_path", rclcpp::ParameterValue(cam_config["dict_path"].as<std::string>()));
          if (cam_config["usecalib"])
            node_options.append_parameter_override("usecalib", rclcpp::ParameterValue(cam_config["usecalib"].as<bool>()));
        }
        else
        {
          LOG_INFO(log_project_path, "相机%d无独立配置", camera_id);
        }
      }
      catch (const YAML::Exception& e)
      {
        LOG_ERROR(log_project_path, "解析配置文件失败: %s", e.what());
      }

      auto node = std::make_shared<Elevator_Rec_Node>(node_options, camera_id);
      nodes.push_back(node);
      executor.add_node(node);
      std::cout << "创建 Elevator_Rec_Node 节点，camera_id: " << camera_id << std::endl;
    }

    executor.spin();
    rclcpp::shutdown();
  }
  return 0;
}
