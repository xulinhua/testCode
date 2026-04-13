#include "button_det_node.h"
#include "bas_operate/file_operate.hpp"
#include "sys_info_src/sys_info_server.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

// ---------- 工具：从模型路径推导并加载 .names 文件 ----------
std::vector<std::string> Button_Det_Node::loadClassNamesFromModel(const std::string& model_path)
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

Button_Det_Node::Button_Det_Node(const rclcpp::NodeOptions &options, int cam_id)
  : Node("button_det_node_" + std::to_string(cam_id), options), camera_id_(cam_id)
{
  // 预先声明所有可能被NodeOptions覆盖的参数
  this->declare_parameter("engine_path", "install/button_det/models/switch_button.engine");
  this->declare_parameter("usecalib", true);

  // 根据camera_id生成日志项目路径
  const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
  log_project_path_ = project_name + "_cam_" + std::to_string(camera_id_);

  camera_type_ = this->declare_parameter("camera_type", "realsense");
  arm_id_ = this->declare_parameter("arm_id", 0);

  // 初始化arm_id_list_
  arm_id_list_.clear();

  // 获取引擎路径参数
  engine_name_ = this->get_parameter("engine_path").as_string();

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

  // 尝试从launch文件获取话题名称
  std::string launch_color_topic = this->declare_parameter("color_image_topic", "");
  std::string launch_depth_topic = this->declare_parameter("depth_image_topic", "");
  std::string launch_camera_info_topic = this->declare_parameter("camera_info_topic", "");

  // 声明usecalib参数
  usecalib_ = this->get_parameter("usecalib").as_bool();
  LOG_INFO(log_project_path_, "标定模式: %s", usecalib_ ? "开启" : "关闭");

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

  camera_intrinsics_initialized_ = false;
  fx_ = 0.0f;
  fy_ = 0.0f;
  cx_ = 0.0f;
  cy_ = 0.0f;

  setup_camera_intrinsics();

  if (usecalib_) {
      initCalibParamHandler();
  }

  yolo_det_.load_engine(engine_name_);
  class_names_ = loadClassNamesFromModel(engine_name_);

  color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(color_image_topic_, 10, std::bind(&Button_Det_Node::Color_Callback, this, std::placeholders::_1));
  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(depth_image_topic_, 10, std::bind(&Button_Det_Node::Depth_Callback, this, std::placeholders::_1));
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10, std::bind(&Button_Det_Node::CameraInfo_Callback, this, std::placeholders::_1));

  std::string topic_prefix = "/cam_" + std::to_string(camera_id_);
  box_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_prefix + "/button_det_image", 1);
  res_image_pub_ = this->create_publisher<std_msgs::msg::String>(topic_prefix + "/button_det_image_png", 1);
  det_res_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(topic_prefix + "/button_det_res", 1);

  LOG_INFO(log_project_path_, "Button Detection Node 初始化完成");
  LOG_INFO(log_project_path_, "  Camera ID: %d", camera_id_);
  LOG_INFO(log_project_path_, "  Camera Type: %s", camera_type_.c_str());
  LOG_INFO(log_project_path_, "  Arm ID: %d", arm_id_);
  LOG_INFO(log_project_path_, "  Color Topic: %s", color_image_topic_.c_str());
  LOG_INFO(log_project_path_, "  Depth Topic: %s", depth_image_topic_.c_str());
  LOG_INFO(log_project_path_, "  Camera Info Topic: %s", camera_info_topic_.c_str());
}

void Button_Det_Node::Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        std::vector<Detection> detections;
        detections.clear();
        detections = yolo_det_.infer(frame);

        Intrinsics intrinsics;
        intrinsics.fx = fx_;
        intrinsics.fy = fy_;
        intrinsics.cx = cx_;
        intrinsics.cy = cy_;

        publish(detections, msg->header);
        yolo_det_.draw_results(frame, detections, depth_frame_, intrinsics, class_names_);

        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        box_image_pub_->publish(*img_msg);
      }
      catch (const cv_bridge::Exception& e) {
        LOG_ERROR(log_project_path_, "trans image error: %s", e.what());
      }
}
  
void Button_Det_Node::Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
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

void Button_Det_Node::initTopicNames()
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
bool Button_Det_Node::getSysDat()
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
bool Button_Det_Node::getCamConfigInfo(SysConfig::CamConfigInfo& cam_info)
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

void Button_Det_Node::initCalibParamHandler()
{
  try
  {
      if (sys_config_client_->wait_for_service(std::chrono::seconds(3)))
      {
          LOG_INFO(log_project_path_, "参数服务器已连接");

          calib_result_ = std::make_unique<handeyecalib::CalibRes>();

          std::string param_prefix = "sys_cam_calib_list.cam_" + std::to_string(camera_id_) + ".arm_info.arm_" + std::to_string(arm_id_);
          LOG_INFO(log_project_path_, "标定矩阵参数前缀: %s", param_prefix.c_str());

          std::string cam_to_base_str;
          std::string cam_to_base_param = param_prefix + ".cam_to_base_transform";
          try {
              cam_to_base_str = sys_config_client_->get_parameter<std::string>(cam_to_base_param);
              calib_result_->cam_to_base_transform = handeyecalib::stringToMat(cam_to_base_str);
              LOG_INFO(log_project_path_, "成功读取 cam_to_base_transform");
          } catch (const std::exception& e) {
              LOG_WARN(log_project_path_, "读取 cam_to_base_transform 失败: %s", e.what());
          }

          std::string base_to_cam_param = param_prefix + ".base_to_cam_transform";
          try {
              std::string base_to_cam_str = sys_config_client_->get_parameter<std::string>(base_to_cam_param);
              calib_result_->base_to_cam_transform = handeyecalib::stringToMat(base_to_cam_str);
              LOG_INFO(log_project_path_, "成功读取 base_to_cam_transform");
          } catch (const std::exception& e) {
              LOG_WARN(log_project_path_, "读取 base_to_cam_transform 失败: %s", e.what());
          }

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

void Button_Det_Node::setup_camera_intrinsics()
{
    LOG_INFO(log_project_path_, "等待相机内参数据...");
    LOG_INFO(log_project_path_, "订阅相机信息话题: %s", camera_info_topic_.c_str());
}

void Button_Det_Node::CameraInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (camera_intrinsics_initialized_)
    {
        return;
    }

    if (msg->k.size() >= 9)
    {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];

        camera_intrinsics_initialized_ = true;

        LOG_INFO(log_project_path_, "成功获取相机内参:");
        LOG_INFO(log_project_path_, "  fx=%.2f, fy=%.2f", fx_, fy_);
        LOG_INFO(log_project_path_, "  cx=%.2f, cy=%.2f", cx_, cy_);
        LOG_INFO(log_project_path_, "  图像尺寸: %dx%d", msg->width, msg->height);

        if (fx_ <= 0 || fy_ <= 0)
        {
            LOG_WARN(log_project_path_, "相机内参异常，使用默认值");
            if (camera_type_ == "Gemini") {
                fx_ = 610.0f;
                fy_ = 610.0f;
                cx_ = 320.0f;
                cy_ = 240.0f;
            } else {
                fx_ = 615.0f;
                fy_ = 615.0f;
                cx_ = 320.0f;
                cy_ = 240.0f;
            }
            LOG_INFO(log_project_path_, "使用默认内参: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
        }
    }
    else
    {
        LOG_ERROR(log_project_path_, "相机内参数据无效");
    }
}

geometry_msgs::msg::Point Button_Det_Node::calculate_3d_position(const Detection& det)
{
    geometry_msgs::msg::Point camera_point;

    if (!camera_intrinsics_initialized_)
    {
        LOG_WARN(log_project_path_, "相机内参未初始化，无法计算3D坐标");
        return camera_point;
    }

    if (fx_ <= 0 || fy_ <= 0)
    {
        LOG_WARN(log_project_path_, "相机内参无效，无法计算3D坐标");
        return camera_point;
    }

    float center_x = det.bbox[0];
    float center_y = det.bbox[1];

    if (depth_frame_.empty())
    {
        LOG_WARN(log_project_path_, "深度图像为空，无法计算3D坐标");
        return camera_point;
    }

    int pixel_x = static_cast<int>(center_x);
    int pixel_y = static_cast<int>(center_y);

    if (pixel_x < 0 || pixel_x >= depth_frame_.cols ||
        pixel_y < 0 || pixel_y >= depth_frame_.rows)
    {
        LOG_WARN(log_project_path_, "像素坐标超出图像范围: (%d, %d)", pixel_x, pixel_y);
        return camera_point;
    }

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

void Button_Det_Node::publish(const std::vector<Detection>& dets,
                const std_msgs::msg::Header& header)
{
    vision_msgs::msg::Detection2DArray arr;
    arr.header = header;

    for (const auto& d : dets)
    {
        vision_msgs::msg::Detection2D det;
        det.bbox.center.position.x = d.bbox[0];
        det.bbox.center.position.y = d.bbox[1];
        det.bbox.size_x              = d.bbox[2];
        det.bbox.size_y              = d.bbox[3];

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        std::ostringstream ss;
        ss << d.class_id;
        hyp.hypothesis.class_id = ss.str();
        hyp.hypothesis.score     = d.conf;

        if (!depth_frame_.empty())
        {
            geometry_msgs::msg::Point camera_point = calculate_3d_position(d);

            geometry_msgs::msg::Point base_point = camera_point;

            if (usecalib_ && calib_result_ && !calib_result_->cam_to_base_transform.empty())
            {
                try 
                {
                    std::vector<double> marker_pose = {camera_point.x, camera_point.y, camera_point.z, 0.0, 0.0, 0.0};
                    std::vector<double> offset_compensation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    if (!calib_result_->offset_compensation.empty() && calib_result_->offset_compensation.size() >= 6) {
                        offset_compensation = calib_result_->offset_compensation;
                    }
                    auto trans_result = handeyecalib::computeRobotPoseFromMarker(marker_pose, calib_result_->cam_to_base_transform, &offset_compensation);
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
            hyp.pose.pose.position.x = d.x3d;
            hyp.pose.pose.position.y = d.y3d;
            hyp.pose.pose.position.z = d.z3d;
            hyp.pose.pose.orientation.x = 0.0;
            hyp.pose.pose.orientation.y = 0.0;
            hyp.pose.pose.orientation.z = 0.0;
            hyp.pose.pose.orientation.w = 1.0;
        }

        det.results.push_back(hyp);
        arr.detections.push_back(det);
    }
    det_res_pub_->publish(arr);
}

void test()
{
  std::string engine_path = "install/button_det/models/switch_button.engine";
  std::string input_folder = "src/perception/button_det/test_imgs";
  std::string output_folder = "output_results";

  std::string names_path = engine_path;
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

  YoloDet detector;
  detector.load_engine(engine_path);

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
      std::vector<Detection> detections;
      try {
          detections = detector.infer(img);
      }
      catch (const std::exception& e) {
          std::cerr << "检测异常: " << e.what() << std::endl;
          continue;
      }

      auto end = std::chrono::high_resolution_clock::now();
      double inference_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

      std::cout << "检测到 " << detections.size() << " 个目标, 耗时: " << inference_time << " ms" << std::endl;

      if (!detections.empty())
      {
          std::cout << "==== 检测结果 ====" << std::endl;
          for (const auto& d : detections)
          {
              std::cout << "Box: [" << std::fixed << std::setprecision(2)
                      << d.bbox[0] << "," << d.bbox[1] << ","
                      << d.bbox[2] << "," << d.bbox[3] << "] Conf= " << d.conf << std::endl;
          }
      }

      cv::Mat result_img = img.clone();
      detector.draw_results(result_img, detections, class_names);

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

// Button Detection主节点 - 用于管理多个相机节点
class ButtonDetMainNode : public rclcpp::Node
{
public:
  explicit ButtonDetMainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("button_det_main_node", options)
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
      std::string config_file_path = ament_index_cpp::get_package_share_directory("button_det")
        + "/config" + "/button_det_params.yaml";
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
    std::string default_config_file_path = ament_index_cpp::get_package_share_directory("button_det")
      + "/config" + "/button_det_params.yaml";
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
    LOG_INFO(log_project_path, "button_detection节点启动");
    rclcpp::executors::MultiThreadedExecutor executor;

    auto main_node = std::make_shared<ButtonDetMainNode>();
    std::vector<int> act_cam_ids = main_node->getActiveCameraIds();

    if (act_cam_ids.empty())
    {
      LOG_ERROR(log_project_path, "有效配置的相机ID列表为空,默认使用相机ID 0");
      act_cam_ids = {0};
    }

    std::vector<std::shared_ptr<Button_Det_Node>> nodes;
    for (int camera_id : act_cam_ids)
    {
      LOG_INFO(log_project_path, "创建 Button_Det_Node 节点，camera_id: %d", camera_id);

      // 通过NodeOptions传递camera_id参数
      rclcpp::NodeOptions node_options;
      node_options.append_parameter_override("camera_id", rclcpp::ParameterValue(camera_id));

      // 从配置文件中读取对应相机的独立参数
      try
      {
        std::string config_file_path = ament_index_cpp::get_package_share_directory("button_det")
          + "/config" + "/button_det_params.yaml";
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

      auto node = std::make_shared<Button_Det_Node>(node_options, camera_id);
      nodes.push_back(node);
      executor.add_node(node);
      std::cout << "创建 Button_Det_Node 节点，camera_id: " << camera_id << std::endl;
    }

    executor.spin();
    rclcpp::shutdown();
  }
  return 0;
}
