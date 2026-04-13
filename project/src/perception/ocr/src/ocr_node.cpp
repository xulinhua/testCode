#include "ocr_node.h"
#include <sstream>
#include <iomanip>
#include "bas_operate/file_operate.hpp"
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

OCR_Node::OCR_Node(const rclcpp::NodeOptions &options, int cam_id)
  : Node("ocr_node_" + std::to_string(cam_id), options),
    camera_id_(cam_id),
    camera_info_received_(false),
    depth_info_received_(false),
    usecalib_(true),
    ft_initialized_(false),
    font_loaded_(false)
{
  // 初始化相机内参矩阵和畸变系数
  camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
  dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

  // 根据camera_id生成日志项目路径
  const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
  log_project_path_ = project_name + "_cam_" + std::to_string(camera_id_);

  // 预先声明参数
  this->declare_parameter("detection_engine", "install/ocr/models/v5_det_mobile.engine");
  this->declare_parameter("recognition_engine", "install/ocr/models/v5_rec_mobile.engine");
  this->declare_parameter("character_dict", "install/ocr/dict/ppocr_keys.txt");
  this->declare_parameter("arm_id", 0);
  this->declare_parameter("color_image_topic", "");
  this->declare_parameter("depth_image_topic", "");
  this->declare_parameter("camera_info_topic", "");
  this->declare_parameter("usecalib", true);

  // 获取参数
  detection_engine_name_ = this->get_parameter("detection_engine").as_string();
  recognition_engine_name_ = this->get_parameter("recognition_engine").as_string();
  character_dict_name_ = this->get_parameter("character_dict").as_string();

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

  // 尝试从launch文件获取话题名称（优先级最高）
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

  // 初始化标定参数
  initCalibParamHandler();

  // 获取usecalib参数
  usecalib_ = this->get_parameter("usecalib").as_bool();
  LOG_INFO(log_project_path_, "标定模式: %s", usecalib_ ? "开启" : "关闭");

  LOG_INFO(log_project_path_, "Camera ID: %d", camera_id_);
  LOG_INFO(log_project_path_, "Arm ID: %d", arm_id_);
  LOG_INFO(log_project_path_, "Color Image Topic: %s", color_image_topic_.c_str());
  LOG_INFO(log_project_path_, "Depth Image Topic: %s", depth_image_topic_.c_str());
  LOG_INFO(log_project_path_, "Camera Info Topic: %s", camera_info_topic_.c_str());

  // 加载模型
  try {
    ocr_.load_models(detection_engine_name_, recognition_engine_name_, character_dict_name_);
    LOG_INFO(log_project_path_, "OCR models loaded successfully");
  } catch (const std::exception& e) {
    LOG_ERROR(log_project_path_, "Failed to load OCR models: %s", e.what());
    return;
  }

  // 创建订阅者
  color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      color_image_topic_, 10, std::bind(&OCR_Node::Color_Callback, this, std::placeholders::_1));
  
  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_image_topic_, 10, std::bind(&OCR_Node::Depth_Callback, this, std::placeholders::_1));
  
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10, std::bind(&OCR_Node::cameraInfoCallback, this, std::placeholders::_1));

  // 生成话题前缀：根据camera_id
  std::string topic_prefix = "/cam_" + std::to_string(camera_id_);

  // 创建发布者 - 使用相机前缀
  std::string ocr_res_topic = topic_prefix + "/ocr_results";
  std::string ocr_image_topic = topic_prefix + "/ocr_image";
  std::string ocr_text_topic = topic_prefix + "/ocr_text";

  ocr_res_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(ocr_res_topic, 1);
  ocr_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(ocr_image_topic, 1);
  ocr_text_pub_ = this->create_publisher<std_msgs::msg::String>(ocr_text_topic, 1);

  LOG_INFO(log_project_path_, "OCR Node initialized");
  LOG_INFO(log_project_path_, "Subscribing to: %s", color_image_topic_.c_str());
  LOG_INFO(log_project_path_, "Subscribing to: %s", depth_image_topic_.c_str());
  LOG_INFO(log_project_path_, "Subscribing to: %s", camera_info_topic_.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", ocr_res_topic.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", ocr_image_topic.c_str());
  LOG_INFO(log_project_path_, "Publishing to: %s", ocr_text_topic.c_str());
  
  // 从参数服务器获取相机内参
  getCameraIntrinsicsFromServer();

  // 初始化FreeType支持中文
  FT_Error error = FT_Init_FreeType(&ft_library_);
  if (error) {
      LOG_WARN(log_project_path_, "初始化FreeType失败");
      ft_initialized_ = false;
  } else {
      ft_initialized_ = true;
      // 尝试加载字体文件（使用opentype路径，与debug_viewer一致）
      const char* font_paths[] = {
          "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
          "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
          "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
          nullptr
      };
      for (int i = 0; font_paths[i] != nullptr; ++i) {
          error = FT_New_Face(ft_library_, font_paths[i], 0, &ft_face_);
          if (error == 0) {
              font_loaded_ = true;
              LOG_INFO(log_project_path_, "成功加载字体: %s", font_paths[i]);
              break;
          }
      }
      if (!font_loaded_) {
          LOG_WARN(log_project_path_, "未找到中文字体文件");
      }
  }
}

void OCR_Node::initTopicNames()
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
bool OCR_Node::getSysDat()
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
bool OCR_Node::getCamConfigInfo(SysConfig::CamConfigInfo& cam_info)
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

void OCR_Node::initCalibParamHandler()
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
          if (!calib_result_->base_to_cam_transform.empty())
          {
              const auto& m = calib_result_->base_to_cam_transform;
              LOG_INFO(log_project_path_, "base_to_cam_transform (%d x %d):",
                          m.rows, m.cols);
              for (int i = 0; i < m.rows; i++) {
                  LOG_INFO(log_project_path_, "  [%.6f, %.6f, %.6f, %.6f]",
                              m.at<double>(i, 0), m.at<double>(i, 1),
                              m.at<double>(i, 2), m.at<double>(i, 3));
              }
          }
          if (!calib_result_->offset_compensation.empty())
          {
              const auto& offset = calib_result_->offset_compensation;
              LOG_INFO(log_project_path_, "offset_compensation [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
                          offset[0], offset[1], offset[2], offset[3], offset[4], offset[5]);
          }
      }
      else
      {
          LOG_WARN(log_project_path_, "参数服务器未上线，将在后台继续等待...");
          // 不禁用标定模式，在第一次使用时重试
      }
  }
  catch (const std::exception& e)
  {
      LOG_WARN(log_project_path_, "初始化标定参数处理器失败: %s", e.what());
  }
}

void OCR_Node::calibDatChangedCallback(const handeyecalib::ArmCalibInfo& calib_data)
{
  *calib_result_ = calib_data.calib_info.calib_res;
  LOG_INFO(log_project_path_, "标定数据已更新");
}

bool OCR_Node::getCameraIntrinsicsFromServer()
{
  // 相机内参将通过camera_info话题接收，无需从参数服务器获取
  LOG_INFO(log_project_path_, "相机内参将通过camera_info话题接收");
  return true;
}

void OCR_Node::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  
  // 更新相机内参矩阵
  camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      msg->k[0], msg->k[1], msg->k[2],
      msg->k[3], msg->k[4], msg->k[5],
      msg->k[6], msg->k[7], msg->k[8]);
  
  // 更新畸变系数
  for (size_t i = 0; i < msg->d.size() && i < 5; i++) {
      dist_coeffs_.at<double>(0, i) = msg->d[i];
  }
  
  camera_info_received_ = true;
  
  LOG_DEBUG(log_project_path_, "接收到相机内参 - fx: %.6f, fy: %.6f, cx: %.6f, cy: %.6f",
              camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
              camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
}

void OCR_Node::Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);

  static int frame_count = 0;
  frame_count++;

  try {
    depth_frame_ = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    depth_info_received_ = true;
  }
  catch (const cv_bridge::Exception& e) {
    LOG_ERROR(log_project_path_, "cv_bridge exception in depth callback: %s", e.what());
  }
}

void OCR_Node::Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    color_frame_ = frame.clone();

    // 执行OCR检测和识别
    std::vector<PPOCRResult> ocr_results;
    ocr_results = ocr_.process_image(frame);

    // if (!ocr_results.empty())
    // {
    //     std::cout << "\n识别结果详情: \n";
    //     for (int i = 0; i < ocr_results.size(); ++i)
    //     {
    //         const auto& result = ocr_results[i];
    //         std::cout << "文本 " << i + 1 << ": " << result.recognition.text 
    //                   << " (置信度: " << result.recognition.score << ")" 
    //                   << " (方向: " << result.detection.orientation << ")" << std::endl;
    //     }
    // }
    // 绘制结果
    // ocr_.draw_results(frame, ocr_results, true, true);

    // 绘制结果和3D位置
    drawResultsWith3D(frame, ocr_results, depth_frame_, camera_matrix_);

    // 发布图像
    auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    ocr_image_pub_->publish(*img_msg);

    // 发布缩放后的图像(base64)
    cv::Size target(160, 120);
    cv::Mat resized;
    cv::resize(frame, resized, target, 0, 0, cv::INTER_LINEAR);
    auto png_base64 = mat2png_base64(resized);
    auto out = std_msgs::msg::String();
    out.data = std::move(png_base64);
    ocr_text_pub_->publish(out);

    // 发布识别结果
    publish(ocr_results, msg->header);
  }
  catch (const cv_bridge::Exception& e) {
    LOG_ERROR(log_project_path_, "cv_bridge exception: %s", e.what());
  }
  catch (const std::exception& e) {
    LOG_ERROR(log_project_path_, "Exception: %s", e.what());
  }
}

void OCR_Node::publish(const std::vector<PPOCRResult>& results,
                         const std_msgs::msg::Header& header)
{
  vision_msgs::msg::Detection2DArray arr;
  arr.header = header;

  for (const auto& result : results)
  {
    vision_msgs::msg::Detection2D det;

    // 计算边界框中心
    cv::Point2f center = result.detection.rrect.center;
    cv::Size2f size = result.detection.rrect.size;

    det.bbox.center.position.x = center.x;
    det.bbox.center.position.y = center.y;
    det.bbox.size_x = size.width;
    det.bbox.size_y = size.height;

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = result.recognition.text;
    hyp.hypothesis.score = result.recognition.score;

    // 计算3D位置
    cv::Point3d pos3d(0.0, 0.0, 0.0);

    if (camera_info_received_ && depth_info_received_)
    {
        std::lock_guard<std::mutex> depth_lock(depth_mutex_);

        // 使用封装的函数计算3D坐标
        pos3d = calculate3DPosition(center, depth_frame_, camera_matrix_);

        // 如果3D坐标有效且启用标定模式，将相机坐标系转换为base坐标系
        if (pos3d.z > 0 && usecalib_ && calib_result_ && !calib_result_->cam_to_base_transform.empty())
        {
            try {
                std::vector<double> marker_pose = {pos3d.x, pos3d.y, pos3d.z, 0.0, 0.0, 0.0};
                std::vector<double> offset_compensation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                auto trans_result = handeyecalib::computeRobotPoseFromMarker(
                    marker_pose, calib_result_->cam_to_base_transform, &offset_compensation);
                if (!trans_result.transformed_pose.empty())
                {
                    pos3d.x = trans_result.transformed_pose[0] + calib_result_->offset_compensation[0];
                    pos3d.y = trans_result.transformed_pose[1] + calib_result_->offset_compensation[1];
                    pos3d.z = trans_result.transformed_pose[2] + calib_result_->offset_compensation[2];

                    LOG_DEBUG(log_project_path_, "3D位置已转换到base坐标系: (%.3f, %.3f, %.3f)",
                                pos3d.x, pos3d.y, pos3d.z);
                }
            }
            catch (const std::exception& e) {
                LOG_WARN(log_project_path_, "坐标转换失败: %s", e.what());
            }
        }
    }
    else
    {
        LOG_WARN(log_project_path_, "相机内参或深度信息未就绪 - camera_info: %d, depth_info: %d",
                   camera_info_received_, depth_info_received_);
    }

    // 设置3D位置
    hyp.pose.pose.position.x = pos3d.x;
    hyp.pose.pose.position.y = pos3d.y;
    hyp.pose.pose.position.z = pos3d.z;
    hyp.pose.pose.orientation.x = 0.0;
    hyp.pose.pose.orientation.y = 0.0;
    hyp.pose.pose.orientation.z = 0.0;
    hyp.pose.pose.orientation.w = 1.0;

    det.results.push_back(hyp);
    arr.detections.push_back(det);

    LOG_DEBUG(log_project_path_, "Detected text: %s (score: %.3f)", 
                 result.recognition.text.c_str(), result.recognition.score);
  }

  ocr_res_pub_->publish(arr);
}

// 绘制旋转矩形框
void OCR_Node::drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rrect, const cv::Scalar& color)
{
    cv::Point2f vertices[4];
    rrect.points(vertices);
    for (int i = 0; i < 4; i++) {
        cv::line(img, vertices[i], vertices[(i + 1) % 4], color, 2);
    }
}

// 计算像素点的3D坐标（相机坐标系）
cv::Point3d OCR_Node::calculate3DPosition(const cv::Point2f& center,
                                             const cv::Mat& depth_img,
                                             const cv::Mat& intrinsics)
{
    cv::Point3d pos(0.0, 0.0, 0.0);

    if (!depth_img.empty() &&
        center.x >= 0 && center.x < depth_img.cols &&
        center.y >= 0 && center.y < depth_img.rows)
    {
        float depth = depth_img.at<uint16_t>(static_cast<int>(center.y), static_cast<int>(center.x));
        if (depth > 0 && depth < 10000.0)  // 深度单位为毫米
        {
            double depth_mm = depth;  //
            double fx = intrinsics.at<double>(0, 0);
            double fy = intrinsics.at<double>(1, 1);
            double cx = intrinsics.at<double>(0, 2);
            double cy = intrinsics.at<double>(1, 2);

            pos.x = (center.x - cx) * depth_mm / fx;
            pos.y = (center.y - cy) * depth_mm / fy;
            pos.z = depth_mm;
        }
    }

    return pos;
}

// 使用FreeType绘制中文文本
void OCR_Node::drawChineseText(cv::Mat& img, const std::string& text,
                                  const cv::Point& text_center, const cv::Scalar& color)
{
    if (!ft_initialized_ || !font_loaded_) {
        return;
    }

    FT_Set_Pixel_Sizes(ft_face_, 24, 0);

    // UTF-8到Unicode转换
    std::vector<uint32_t> unicode_chars;
    for (size_t i = 0; i < text.size(); )
    {
        uint32_t ch = 0;
        unsigned char c = text[i];
        if (c < 0x80) {
            ch = static_cast<uint32_t>(c);
            i++;
        } else if ((c & 0xE0) == 0xC0) {
            if (i + 1 < text.size()) {
                ch = ((c & 0x1F) << 6) | (text[i + 1] & 0x3F);
                i += 2;
            } else {
                i++;
            }
        } else if ((c & 0xF0) == 0xE0) {
            if (i + 2 < text.size()) {
                ch = ((c & 0x0F) << 12) | ((text[i + 1] & 0x3F) << 6) | (text[i + 2] & 0x3F);
                i += 3;
            } else {
                i++;
            }
        } else {
            i++;
            continue;
        }
        unicode_chars.push_back(ch);
    }

    // 计算文本总宽度
    int total_width = 0;
    for (uint32_t char_code : unicode_chars)
    {
        FT_UInt glyph_index = FT_Get_Char_Index(ft_face_, char_code);
        if (glyph_index == 0) continue;

        FT_Error error = FT_Load_Glyph(ft_face_, glyph_index, FT_LOAD_DEFAULT);
        if (error) continue;
        total_width += (ft_face_->glyph->advance.x >> 6);
    }

    // 起始位置为居中
    int pen_x = text_center.x - total_width / 2;
    int pen_y = text_center.y + 16;

    // 绘制每个字符
    for (uint32_t char_code : unicode_chars)
    {
        FT_UInt glyph_index = FT_Get_Char_Index(ft_face_, char_code);
        if (glyph_index == 0) continue;

        FT_Error error = FT_Load_Glyph(ft_face_, glyph_index, FT_LOAD_DEFAULT);
        if (error) continue;

        error = FT_Render_Glyph(ft_face_->glyph, FT_RENDER_MODE_NORMAL);
        if (error) continue;

        FT_GlyphSlot slot = ft_face_->glyph;
        FT_Bitmap bitmap = slot->bitmap;

        for (unsigned int y = 0; y < bitmap.rows; y++)
        {
            for (unsigned int x = 0; x < bitmap.width; x++)
            {
                int img_x = pen_x + slot->bitmap_left + x;
                int img_y = pen_y - slot->bitmap_top + y;
                if (img_x >= 0 && img_x < img.cols && img_y >= 0 && img_y < img.rows)
                {
                    float alpha = bitmap.buffer[y * bitmap.width + x] / 255.0f;
                    cv::Vec3b& pixel = img.at<cv::Vec3b>(img_y, img_x);
                    pixel[0] = cv::saturate_cast<uchar>(pixel[0] * (1 - alpha) + color[0] * alpha);
                    pixel[1] = cv::saturate_cast<uchar>(pixel[1] * (1 - alpha) + color[1] * alpha);
                    pixel[2] = cv::saturate_cast<uchar>(pixel[2] * (1 - alpha) + color[2] * alpha);
                }
            }
        }
        pen_x += slot->advance.x >> 6;
    }
}

// 使用OpenCV绘制英文文本
void OCR_Node::drawEnglishText(cv::Mat& img, const std::string& text,
                                  const cv::Point& text_pos, const cv::Scalar& color)
{
    int baseline;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
    cv::Point pos(text_pos.x - text_size.width / 2, text_pos.y + text_size.height / 2);
    cv::putText(img, text, pos, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
}

// 绘制3D坐标文本
void OCR_Node::draw3DPositionText(cv::Mat& img, const cv::Point3d& pos,
                                    const cv::Point& center, const cv::Scalar& color)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3)
       << "(" << pos.x << "," << pos.y << "," << pos.z << ")";
    std::string pos_text = ss.str();
    int baseline;
    cv::Size pos_size = cv::getTextSize(pos_text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    cv::Point pos_pos(center.x - pos_size.width / 2, center.y - 8);
    cv::putText(img, pos_text, pos_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
}

// 绘制识别结果文本
void OCR_Node::drawResultText(cv::Mat& img, const std::string& text,
                                 const cv::RotatedRect& rrect, const cv::Scalar& color)
{
    cv::Rect bbox = rrect.boundingRect();
    cv::Point text_center(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);

    // 检查是否包含非ASCII字符
    bool has_non_ascii = false;
    for (char c : text) {
        if ((unsigned char)c > 127) {
            has_non_ascii = true;
            break;
        }
    }

    if (has_non_ascii) {
        drawChineseText(img, text, text_center, color);
    } else {
        drawEnglishText(img, text, text_center, color);
    }
}

// 打印识别结果详情
void OCR_Node::printRecognitionResult(const PPOCRResult& result, const cv::Point3d& pos3d, int idx)
{
    LOG_INFO(log_project_path_, "识别结果 [%d] - 文本: '%s' | 置信度: %.3f | 方向: %d | 3D坐标: (%.3f, %.3f, %.3f)m", 
        idx, result.recognition.text.c_str(), result.recognition.score, result.detection.orientation, pos3d.x, pos3d.y, pos3d.z);
}

// 主绘制函数
void OCR_Node::drawResultsWith3D(cv::Mat& img, const std::vector<PPOCRResult>& results,
                                   const cv::Mat& depth_img, const cv::Mat& intrinsics)
{
    static const cv::Scalar colors[] = {
        cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255)
    };
    LOG_INFO(log_project_path_, "=== OCR识别结果 === 检测到 %zu 个文本区域", results.size());
    for (size_t idx = 0; idx < results.size(); ++idx)
    {
        const auto& result = results[idx];
        const cv::RotatedRect& rrect = result.detection.rrect;
        cv::Scalar color = colors[idx % 6];

        // 绘制旋转矩形框
        drawRotatedRect(img, rrect, color);

        // 计算3D位置
        cv::Point2f center = rrect.center;
        cv::Point3d pos3d = calculate3DPosition(center, depth_img, intrinsics);

        // 打印识别结果详情
        printRecognitionResult(result, pos3d, idx);

        // 绘制识别文本
        drawResultText(img, result.recognition.text, rrect, color);

        // 绘制3D坐标
        cv::Rect bbox = rrect.boundingRect();
        cv::Point pos3d_center(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
        draw3DPositionText(img, pos3d, pos3d_center, color);
    }
}

void test()
{
    std::string detection_model_path = "install/ocr/models/v5_det_mobile.engine";
    std::string recognition_model_path = "install/ocr/models/v4_rec_mobile.engine";
    std::string character_dict_path = "install/ocr/dict/ppocr_keys_v1.txt";
    std::string input_folder = "src/perception/ocr/test_imgs";
    std::string output_folder = "output_results";

    if (access(output_folder.c_str(), 0) == -1)
    {
        mkdir(output_folder.c_str(), 0755);
        std::cout << "创建输出文件夹: " << output_folder << std::endl;
    }

    auto ocr_system = std::make_unique<PPOCR>();

    try
    {
        ocr_system->load_models(detection_model_path, recognition_model_path, character_dict_path);
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
        std::vector<PPOCRResult> results;
        try {
            results = ocr_system->process_image(image);
        }
        catch (const std::exception& e) {
            std::cerr << "处理异常: " << e.what() << std::endl;
            continue;
        }

        auto end = std::chrono::high_resolution_clock::now();
        double inference_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        std::cout << "检测到文本区域数量: " << results.size() << ", 耗时: " << inference_time << " ms" << std::endl;

        if (!results.empty())
        {
            std::cout << "==== 识别结果 ====" << std::endl;
            for (size_t i = 0; i < results.size(); ++i)
            {
                const auto& result = results[i];
                std::cout << "文本 " << i + 1 << ": " << result.recognition.text
                          << " (置信度: " << result.recognition.score << ")"
                          << " (方向: " << result.detection.orientation << ")" << std::endl;
            }
        }

        cv::Mat output_image = image.clone();
        ocr_system->draw_results(output_image, results, true, true);

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

// OCR主节点 - 用于管理多个相机节点
class OCRMainNode : public rclcpp::Node
{
public:
  explicit OCRMainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("ocr_main_node", options)
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
      std::string config_file_path = ament_index_cpp::get_package_share_directory("ocr")
        + "/config" + "/ocr_params.yaml";
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

int main(int argc, char **argv)
{
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
    std::string default_config_file_path = ament_index_cpp::get_package_share_directory("ocr")
      + "/config" + "/ocr_params.yaml";
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
    LOG_INFO(log_project_path, "OCR节点启动");
    rclcpp::executors::MultiThreadedExecutor executor;

    auto main_node = std::make_shared<OCRMainNode>();
    std::vector<int> act_cam_ids = main_node->getActiveCameraIds();

    if (act_cam_ids.empty())
    {
      LOG_ERROR(log_project_path, "有效配置的相机ID列表为空,默认使用相机ID 0");
      act_cam_ids = {0};
    }

    std::vector<std::shared_ptr<OCR_Node>> nodes;
    for (int camera_id : act_cam_ids)
    {
      LOG_INFO(log_project_path, "创建 OCR_Node 节点，camera_id: %d", camera_id);

      // 通过NodeOptions传递camera_id参数
      rclcpp::NodeOptions node_options;
      node_options.append_parameter_override("camera_id", rclcpp::ParameterValue(camera_id));

      // 从配置文件中读取对应相机的独立参数
      try
      {
        std::string config_file_path = ament_index_cpp::get_package_share_directory("ocr")
          + "/config" + "/ocr_params.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);

        std::string cam_param_key = "cam_" + std::to_string(camera_id) + "_parameters";
        if (config[cam_param_key])
        {
          YAML::Node cam_config = config[cam_param_key];
          LOG_INFO(log_project_path, "读取相机%d的独立配置", camera_id);

          // 传递相机独立配置参数
          if (cam_config["detection_engine"])
            node_options.append_parameter_override("detection_engine", rclcpp::ParameterValue(cam_config["detection_engine"].as<std::string>()));
          if (cam_config["recognition_engine"])
            node_options.append_parameter_override("recognition_engine", rclcpp::ParameterValue(cam_config["recognition_engine"].as<std::string>()));
          if (cam_config["character_dict"])
            node_options.append_parameter_override("character_dict", rclcpp::ParameterValue(cam_config["character_dict"].as<std::string>()));
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
        LOG_ERROR(log_project_path, "解析相机配置失败: %s", e.what());
      }

      auto node = std::make_shared<OCR_Node>(node_options, camera_id);
      nodes.push_back(node);
      executor.add_node(node);
      std::cout << "创建 OCR_Node 节点，camera_id: " << camera_id << std::endl;
    }

    executor.spin();
    rclcpp::shutdown();
  }
  return 0;
}
