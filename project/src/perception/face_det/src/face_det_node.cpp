#include "face_det_node.h"
#include <sstream>
#include <iomanip>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "bas_operate/file_operate.hpp"
#include "sys_info_src/sys_info_server.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

// ---------- 工具：Mat -> PNG -> base64 ----------
static std::string mat2png_base64(const cv::Mat& img)
{
  std::vector<uchar> buf;
  cv::imencode(".png", img, buf);
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

FaceDet_Node::FaceDet_Node(const rclcpp::NodeOptions &options, int cam_id)
  : Node("face_detection_node_" + std::to_string(cam_id), options), camera_id_(cam_id)
{
  // 预先声明所有可能被NodeOptions覆盖的参数
  this->declare_parameter("engine_path", "install/face_det/models/scrfd_500m_bnkps_shape640x640.engine");
  this->declare_parameter("usecalib", true);
  this->declare_parameter("multi_face_threshold", 3);
  this->declare_parameter("multi_face_distance", 1.5);

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

  // 多人互动参数
  multi_face_threshold_ = this->get_parameter("multi_face_threshold").as_int();
  multi_face_distance_ = this->get_parameter("multi_face_distance").as_double();

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

  // 初始化人脸数量检测状态
  previous_face_count_ = 0;
  previous_nearby_face_count_ = 0;
  previous_multi_face_count_ = 0;
  multi_face_triggered_ = false;
  greeting_triggered_ = false;

  // 初始化新增功能 - 解耦合设计
  setup_camera_intrinsics();  // 设置相机内参

  // 加载模型
  try {
    scrfd_.load_engine(engine_name_);
    LOG_INFO(log_project_path_, "SCRFD face detection model loaded successfully");
  } catch (const std::exception& e) {
    LOG_ERROR(log_project_path_, "Failed to load SCRFD model: %s", e.what());
    return;
  }

  LOG_INFO(log_project_path_, "Camera ID: %d", camera_id_);
  LOG_INFO(log_project_path_, "Arm ID: %d", arm_id_);
  LOG_INFO(log_project_path_, "Color Image Topic: %s", color_image_topic_.c_str());
  LOG_INFO(log_project_path_, "Depth Image Topic: %s", depth_image_topic_.c_str());
  LOG_INFO(log_project_path_, "Camera Info Topic: %s", camera_info_topic_.c_str());

  // 创建订阅者
  color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
  color_image_topic_, 10, std::bind(&FaceDet_Node::Color_Callback, this, std::placeholders::_1));
  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
  depth_image_topic_, 10, std::bind(&FaceDet_Node::Depth_Callback, this, std::placeholders::_1));
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
  camera_info_topic_, 10, std::bind(&FaceDet_Node::CameraInfo_Callback, this, std::placeholders::_1));

  // 生成话题前缀：根据camera_id
  std::string topic_prefix = "/cam_" + std::to_string(camera_id_);

  // 创建发布者 - 使用相机前缀
  std::string face_res_topic = topic_prefix + "/face_res";
  std::string face_image_topic = topic_prefix + "/face_image";
  std::string face_text_topic = topic_prefix + "/face_text";
  std::string interaction_topic = topic_prefix + "/face_interaction";

  face_res_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(face_res_topic, 1);
  face_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(face_image_topic, 1);
  face_text_pub_ = this->create_publisher<std_msgs::msg::String>(face_text_topic, 1);
  interaction_pub_ = this->create_publisher<std_msgs::msg::String>(interaction_topic, 1);

  LOG_INFO(log_project_path_, "Publishing to: %s", face_res_topic.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", face_image_topic.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", face_text_topic.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", interaction_topic.c_str());

  LOG_INFO(log_project_path_, "Face Detection Node initialized");
}

void FaceDet_Node::Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    color_frame_ = frame.clone();

    // 执行人脸检测
    std::vector<FaceDetection> face_results;
    face_results = scrfd_.detect(frame);

    // 检测人脸数量变化
    detect_face_count_change(face_results);

    // 检测多人互动
    detect_multi_face_interaction(face_results);

    Intrinsics intrinsics;
    intrinsics.fx = fx_;
    intrinsics.fy = fy_;
    intrinsics.cx = cx_;
    intrinsics.cy = cy_;
    // 绘制结果
    // scrfd_.draw_results(frame, face_results, true, true);
    scrfd_.draw_results(frame, face_results, depth_frame_, intrinsics, true, true);

    // 发布图像
    auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    face_image_pub_->publish(*img_msg);

    // 发布缩放后的图像(base64)
    cv::Size target(160, 120);
    cv::Mat resized;
    cv::resize(frame, resized, target, 0, 0, cv::INTER_LINEAR);
    auto png_base64 = mat2png_base64(resized);
    auto out = std_msgs::msg::String();
    out.data = std::move(png_base64);
    face_text_pub_->publish(out);

    // 发布检测结果
    publish(face_results, msg->header);
  }
  catch (const cv_bridge::Exception& e) {
    LOG_ERROR(log_project_path_, "cv_bridge exception: %s", e.what());
  }
  catch (const std::exception& e) {
    LOG_ERROR(log_project_path_, "Exception: %s", e.what());
  }
}

void FaceDet_Node::Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
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
void FaceDet_Node::initTopicNames()
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
bool FaceDet_Node::getSysDat()
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
bool FaceDet_Node::getCamConfigInfo(SysConfig::CamConfigInfo& cam_info)
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
void FaceDet_Node::initCalibParamHandler()
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
void FaceDet_Node::setup_camera_intrinsics()
{
    // 等待相机信息话题发布内参数据
    // 这里不设置硬编码值，而是等待CameraInfo回调函数获取真实的内参数据
    LOG_INFO(log_project_path_, "等待相机内参数据...");
    LOG_INFO(log_project_path_, "订阅相机信息话题: %s", camera_info_topic_.c_str());
}

// 相机信息回调函数 - 通过API获取真实的内参数据
void FaceDet_Node::CameraInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
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

// 计算camera_link下的3D坐标
geometry_msgs::msg::Point FaceDet_Node::calculate_3d_position(const FaceDetection& det)
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
    float center_x = det.bbox.x + det.bbox.width / 2.0f;  // 边界框中心x
    float center_y = det.bbox.y + det.bbox.height / 2.0f;  // 边界框中心y
    
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

void FaceDet_Node::publish(const std::vector<FaceDetection>& detections,
                         const std_msgs::msg::Header& header)
{
  vision_msgs::msg::Detection2DArray arr;
  arr.header = header;

  for (const auto& result : detections)
  {
    vision_msgs::msg::Detection2D det;

    // 边界框
    det.bbox.center.position.x = result.bbox.x + result.bbox.width / 2.0;
    det.bbox.center.position.y = result.bbox.y + result.bbox.height / 2.0;
    det.bbox.size_x = result.bbox.width;
    det.bbox.size_y = result.bbox.height;

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = "face";
    hyp.hypothesis.score = result.score;

    // 使用深度图像计算3D坐标
    if (!depth_frame_.empty()) 
    {
        // 计算camera_link下的3D坐标
        geometry_msgs::msg::Point camera_point = calculate_3d_position(result);

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

        // 填充到hyp.pose.pose.position
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
        // 如果没有深度图像，使用原有的3D坐标 (暂设为0)
        hyp.pose.pose.position.x = 0.0;
        hyp.pose.pose.position.y = 0.0;
        hyp.pose.pose.position.z = 0.0;
        hyp.pose.pose.orientation.x = 0.0;
        hyp.pose.pose.orientation.y = 0.0;
        hyp.pose.pose.orientation.z = 0.0;
        hyp.pose.pose.orientation.w = 1.0;
    }

    det.results.push_back(hyp);
    arr.detections.push_back(det);

    LOG_DEBUG(log_project_path_, "Detected face: score=%.3f bbox=[%d,%d,%d,%d]",
                 result.score, result.bbox.x, result.bbox.y, result.bbox.width, result.bbox.height);
  }

  face_res_pub_->publish(arr);
}

// 检测人脸数量变化
void FaceDet_Node::detect_face_count_change(const std::vector<FaceDetection>& face_results)
{
    int current_face_count = face_results.size();

    // 计算在1.5米范围内的人脸数量（且必须是正脸）
    int current_nearby_face_count = 0;
    const float MAX_DISTANCE = 1.0f;  // 1.5米范围

    for (const auto& result : face_results)
    {
        // 首先判断是否为正脸
        if (!scrfd_.is_frontal_face(result, 0.3f)) {
            LOG_DEBUG(log_project_path_, "跳过侧脸，朝向程度 >= 0.3");
            continue;
        }

        // 计算当前人脸的3D坐标
        geometry_msgs::msg::Point camera_point = calculate_3d_position(result);
        float distance = camera_point.z;

        // 检查深度是否有效且在范围内
        if (distance > 0 && distance <= MAX_DISTANCE)
        {
            current_nearby_face_count++;
            LOG_DEBUG(log_project_path_, "正脸深度 %.3f 米（在1.5米范围内）", distance);
        }
        else if (distance > 0)
        {
            LOG_DEBUG(log_project_path_, "正脸深度 %.3f 米（超出1.5米范围）", distance);
        }
    }

    // 在范围内的人脸从0变1：有人靠近进入触发区，且之前未触发过greeting
    if (previous_nearby_face_count_ == 0 && current_nearby_face_count > 0 && !greeting_triggered_)
    {
        LOG_INFO(log_project_path_, "检测到正脸进入1.5米范围 - 触发greeting");

        // 发布打招呼指令
        auto msg = std_msgs::msg::String();
        msg.data = "greeting";
        interaction_pub_->publish(msg);
        LOG_INFO(log_project_path_, "发布指令: greeting");

        // 标记为已触发，避免重复触发
        greeting_triggered_ = true;
    }
    // 人脸数量从1变0：所有人脸都消失（不管是否在范围内），重置触发状态
    else if (previous_face_count_ > 0 && current_face_count == 0)
    {
        LOG_INFO(log_project_path_, "检测到人脸消失 - 人脸数量从%d变为0", previous_face_count_);

        // 重置greeting触发状态，下次有人来时可以再次触发
        if (greeting_triggered_)
        {
            greeting_triggered_ = false;
            LOG_INFO(log_project_path_, "重置greeting触发状态");
        }

        // 发布挥手离别指令
        auto msg = std_msgs::msg::String();
        msg.data = "goodbye";
        // interaction_pub_->publish(msg);
        LOG_INFO(log_project_path_, "发布指令: goodbye");
    }

    // 更新上次状态
    previous_face_count_ = current_face_count;
    previous_nearby_face_count_ = current_nearby_face_count;
}

// 检测多人互动
void FaceDet_Node::detect_multi_face_interaction(const std::vector<FaceDetection>& face_results)
{
    // 计算在指定距离范围内的人脸数量（且必须是正脸）
    int current_multi_face_count = 0;
    int current_nearby_face_count = 0;  // 任意人脸（不限制正脸侧脸）在范围内的数量

    // 先计算任意人脸的数量（用于判断是否有人离开）
    for (const auto& result : face_results)
    {
        geometry_msgs::msg::Point camera_point = calculate_3d_position(result);
        float distance = camera_point.z;

        if (distance > 0 && distance <= multi_face_distance_)
        {
            current_nearby_face_count++;
        }
    }

    // 再计算正脸的数量（用于触发判断）
    for (const auto& result : face_results)
    {
        // 首先判断是否为正脸
        if (!scrfd_.is_frontal_face(result, 0.3f)) {
            continue;
        }

        // 计算当前人脸的3D坐标
        geometry_msgs::msg::Point camera_point = calculate_3d_position(result);
        float distance = camera_point.z;

        // 检查深度是否有效且在范围内
        if (distance > 0 && distance <= multi_face_distance_)
        {
            current_multi_face_count++;
        }
    }

    // 当范围内所有人脸都消失时，重置触发标志
    if (current_nearby_face_count == 0 && previous_multi_face_count_ > 0)
    {
        if (multi_face_triggered_)
        {
            LOG_INFO(log_project_path_, "多人互动结束，重置触发状态");
            multi_face_triggered_ = false;
        }
        previous_multi_face_count_ = current_multi_face_count;
        return;
    }

    // 当正脸数量小于阈值时，不重置触发标志（因为还有人存在，只是变成侧脸）
    // 这样可以避免正脸侧脸切换时重复触发

    // 首次达到或超过阈值，且之前未触发过，则触发多人互动指令
    if (current_multi_face_count >= multi_face_threshold_ && !multi_face_triggered_)
    {
        LOG_INFO(log_project_path_, "检测到%d人进入%.1f米范围 - 触发multi_interaction", current_multi_face_count, multi_face_distance_);

        // 发布多人互动指令
        auto msg = std_msgs::msg::String();
        msg.data = "multi_interaction";
        interaction_pub_->publish(msg);
        LOG_INFO(log_project_path_, "发布指令: multi_interaction");

        // 标记为已触发，避免重复触发
        multi_face_triggered_ = true;
    }

    // 更新上次状态
    previous_multi_face_count_ = current_nearby_face_count;
}

void test()
{
    std::string detection_model_path = "install/face_det/models/scrfd_500m_bnkps_shape640x640.engine";
    std::string input_folder = "src/perception/face_det/test_imgs";
    std::string output_folder = "output_results";

    if (access(output_folder.c_str(), 0) == -1)
    {
        mkdir(output_folder.c_str(), 0755);
        std::cout << "创建输出文件夹: " << output_folder << std::endl;
    }

    auto face_detector = std::make_unique<SCRFD>();

    try
    {
        face_detector->load_engine(detection_model_path);
        std::cout << "模型加载成功！" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "模型加载失败: " << e.what() << std::endl;
        return;
    }

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

        cv::Mat image = cv::imread(image_path);
        if (image.empty())
        {
            std::cerr << "无法加载图片: " << image_path << std::endl;
            continue;
        }

        std::cout << "\n处理: " << filename << std::endl;

        auto start = std::chrono::high_resolution_clock::now();
        std::vector<FaceDetection> results;
        try {
            results = face_detector->detect(image);
        }
        catch (const std::exception& e) {
            std::cerr << "检测异常: " << e.what() << std::endl;
            continue;
        }

        auto end = std::chrono::high_resolution_clock::now();
        double inference_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        std::cout << "检测到人脸数量: " << results.size() << ", 耗时: " << inference_time << " ms" << std::endl;

        if (!results.empty())
        {
            std::cout << "==== 检测结果 ====" << std::endl;
            for (size_t i = 0; i < results.size(); ++i)
            {
                const auto& result = results[i];
                std::cout << "人脸 " << i + 1 << ": 置信度=" << result.score
                          << " 边界框=[" << result.bbox.x << "," << result.bbox.y
                          << "," << result.bbox.width << "," << result.bbox.height << "]" << std::endl;
            }
        }

        cv::Mat output_image = image.clone();
        face_detector->draw_results(output_image, results, true, true);

        if (cv::imwrite(output_path, output_image))
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

// Face Detection主节点 - 用于管理多个相机节点
class FaceDetMainNode : public rclcpp::Node
{
public:
  explicit FaceDetMainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("face_det_main_node", options)
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
      std::string config_file_path = ament_index_cpp::get_package_share_directory("face_det")
        + "/config" + "/face_det_params.yaml";
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
    std::string default_config_file_path = ament_index_cpp::get_package_share_directory("face_det")
      + "/config" + "/face_det_params.yaml";
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
    LOG_INFO(log_project_path, "face_detection节点启动");
    rclcpp::executors::MultiThreadedExecutor executor;

    auto main_node = std::make_shared<FaceDetMainNode>();
    std::vector<int> act_cam_ids = main_node->getActiveCameraIds();

    if (act_cam_ids.empty())
    {
      LOG_ERROR(log_project_path, "有效配置的相机ID列表为空,默认使用相机ID 0");
      act_cam_ids = {0};
    }

    std::vector<std::shared_ptr<FaceDet_Node>> nodes;
    for (int camera_id : act_cam_ids)
    {
      LOG_INFO(log_project_path, "创建 FaceDet_Node 节点，camera_id: %d", camera_id);

      // 通过NodeOptions传递camera_id参数
      rclcpp::NodeOptions node_options;
      node_options.append_parameter_override("camera_id", rclcpp::ParameterValue(camera_id));

      // 从配置文件中读取对应相机的独立参数
      try
      {
        std::string config_file_path = ament_index_cpp::get_package_share_directory("face_det")
          + "/config" + "/face_det_params.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);

        std::string cam_param_key = "cam_" + std::to_string(camera_id) + "_parameters";
        if (config[cam_param_key])
        {
          YAML::Node cam_config = config[cam_param_key];
          LOG_INFO(log_project_path, "读取相机%d的独立配置", camera_id);

          // 传递相机独立配置参数
          if (cam_config["engine_path"])
            node_options.append_parameter_override("engine_path", rclcpp::ParameterValue(cam_config["engine_path"].as<std::string>()));
          if (cam_config["usecalib"])
            node_options.append_parameter_override("usecalib", rclcpp::ParameterValue(cam_config["usecalib"].as<bool>()));
          if (cam_config["multi_face_threshold"])
            node_options.append_parameter_override("multi_face_threshold", rclcpp::ParameterValue(cam_config["multi_face_threshold"].as<int>()));
          if (cam_config["multi_face_distance"])
            node_options.append_parameter_override("multi_face_distance", rclcpp::ParameterValue(cam_config["multi_face_distance"].as<double>()));
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

      auto node = std::make_shared<FaceDet_Node>(node_options, camera_id);
      nodes.push_back(node);
      executor.add_node(node);
      std::cout << "创建 FaceDet_Node 节点，camera_id: " << camera_id << std::endl;
    }

    executor.spin();
    rclcpp::shutdown();
  }
  return 0;
}
