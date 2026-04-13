#include "object_det_node.h"
#include "bas_operate/file_operate.hpp"
#include "task_factory.h"
#include "bas_operate_ros/ros_comm_info.h"
#include "sys_info_src/sys_info_server.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

// ---------- 工具：从模型路径推导并加载 .names 文件 ----------
std::vector<std::string> Object_Det_Node::loadClassNamesFromModel(const std::string& model_path)
{
    // model_path: "install/object_det/models/coco_vlm.engine"
    // .names文件:  "install/object_det/models/coco_vlm.names"
    std::string names_path = model_path;
    size_t dot_pos = names_path.rfind('.');
    if (dot_pos != std::string::npos) {
        names_path = names_path.substr(0, dot_pos) + ".names";
    } else {
        names_path += ".names";
    }

    LOG_INFO(log_path_, "Loading class names from: %s", names_path.c_str());

    std::vector<std::string> names;
    std::ifstream f(names_path);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open class names file: " + names_path +
                                 ". Each model must have a corresponding .names file.");
    }

    std::string line;
    while (std::getline(f, line)) {
        // 去除行尾空白
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

    LOG_INFO(log_path_, "Loaded %zu class names from %s", names.size(), names_path.c_str());
    return names;
}

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

// 初始化检测任务 - 使用工厂模式创建
// 运行时动态切换算法
void Object_Det_Node::switchAlgorithm(const std::string& algorithm,
                                   const std::string& model_path,
                                   const std::string& config_path,
                                   const InferenceEngineConfig& engine_config)
{
    std::lock_guard<std::mutex> lock(detection_task_mutex_);

    publishModuleStatus(basros::ModuleStatus::RELOADING, "正在切换算法: " + algorithm);

    LOG_INFO(log_path_, "Switching detection algorithm:");
    LOG_INFO(log_path_, "  Algorithm: %s", algorithm.c_str());
    LOG_INFO(log_path_, "  Model Path: %s", model_path.c_str());

    try {
        // 创建新的检测任务
        auto new_task = TaskFactory::createDetectionTask(algorithm, model_path, config_path, engine_config);

        if (!new_task || !new_task->isInitialized()) {
            throw std::runtime_error("Failed to create new detection task");
        }

        // 获取当前阈值
        float conf_thresh = this->get_parameter("conf_threshold").as_double();
        float nms_thresh = this->get_parameter("nms_threshold").as_double();
        new_task->setThreshold(conf_thresh, nms_thresh);
        
        // 从模型同名 .names 文件加载类别名称
        std::vector<std::string> new_class_names = loadClassNamesFromModel(model_path);
        class_names_ = new_class_names;
        new_task->setClassNames(class_names_);

        // 原子替换（新任务创建成功后才替换）
        detection_task_ = std::move(new_task);

        LOG_INFO(log_path_, "Detection task switched successfully:");
        LOG_INFO(log_path_, "  Algorithm: %s", detection_task_->getAlgorithmName().c_str());
        LOG_INFO(log_path_, "  Engine: %s", detection_task_->getEngineName().c_str());
        LOG_INFO(log_path_, "  Class Names: %zu classes loaded", class_names_.size());

        publishModuleStatus(basros::ModuleStatus::RUNNING, "算法切换成功: " + algorithm);
    } catch (const std::exception& e) {
        // 切换失败，发布ERROR状态
        publishModuleStatus(basros::ModuleStatus::ERROR, "算法切换失败: " + std::string(e.what()));
        LOG_ERROR(log_path_, "Algorithm switch failed: %s", e.what());
        throw;
    }
}

// 解析算法配置字符串（格式: "algorithm:model_path:engine_type"）
bool Object_Det_Node::parseAlgorithmConfig(const std::string& config_str,
                                       std::string& algorithm,
                                       std::string& model_path,
                                       std::string& engine_type)
{
    if (config_str.empty()) {
        return false;
    }

    // 分割参数（使用冒号分隔）
    std::vector<std::string> parts;
    std::stringstream ss(config_str);
    std::string part;
    while (std::getline(ss, part, ':')) {
        parts.push_back(part);
    }

    if (parts.size() >= 1) {
        algorithm = parts[0];
    }
    if (parts.size() >= 2) {
        model_path = parts[1];
    }
    if (parts.size() >= 3) {
        engine_type = parts[2];
    }

    LOG_INFO(log_path_, "解析算法配置: algorithm=%s, model_path=%s, engine_type=%s",
             algorithm.c_str(), model_path.c_str(), engine_type.c_str());

    return !algorithm.empty();
}

// 算法切换服务回调
void Object_Det_Node::handleSwitchAlgorithmService(
    const std::shared_ptr<custom_msgs_comm::srv::SwitchAlgorithm::Request> request,
    std::shared_ptr<custom_msgs_comm::srv::SwitchAlgorithm::Response> response)
{
    // 从请求中获取算法配置
    std::string algorithm = request->algorithm;
    std::string model_path = request->model_path;
    std::string engine_type = request->engine_type;

    LOG_INFO(log_path_, "Switch algorithm request received: algorithm=%s, model_path=%s, engine_type=%s",
             algorithm.c_str(), model_path.c_str(), engine_type.c_str());

    // 验证参数
    if (algorithm.empty() || model_path.empty() || engine_type.empty()) {
        response->success = false;
        response->message = "Invalid parameters: algorithm, model_path, and engine_type are required";
        LOG_WARN(log_path_, "Invalid switch algorithm request: missing parameters");
        return;
    }

    // 获取其他配置
    std::string config_path = this->get_parameter("config_path").as_string();
    int device_id = this->get_parameter("device_id").as_int();
    bool enable_fp16 = this->get_parameter("enable_fp16").as_bool();
    bool enable_int8 = this->get_parameter("enable_int8").as_bool();
    int batch_size = this->get_parameter("batch_size").as_int();

    // 构建引擎配置
    InferenceEngineConfig engine_config;
    auto [engine_type_enum, is_valid] = stringToEngineType(engine_type);
    engine_config.engine_type = engine_type_enum;
    if (!is_valid) {
        LOG_WARN(log_path_, "Unknown engine type: %s, using TensorRT", engine_type.c_str());
    }
    engine_config.device_id = device_id;
    engine_config.enable_fp16 = enable_fp16;
    engine_config.enable_int8 = enable_int8;
    engine_config.batch_size = batch_size;

    // 切换算法
    try {
        switchAlgorithm(algorithm, model_path, config_path, engine_config);
        response->success = true;
        response->message = "Algorithm switched successfully to: " + algorithm;
        response->current_algorithm = algorithm;
        response->current_model_path = model_path;
        response->current_engine_type = engine_type;

        // 更新参数服务器中的算法参数
        this->set_parameter(rclcpp::Parameter("algorithm", algorithm));
        this->set_parameter(rclcpp::Parameter("model_path", model_path));
        this->set_parameter(rclcpp::Parameter("engine_type", engine_type));

        LOG_INFO(log_path_, "Algorithm switched via service to: %s (model: %s, engine: %s)",
                 algorithm.c_str(), model_path.c_str(), engine_type.c_str());
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Failed to switch algorithm: " + std::string(e.what());
        LOG_ERROR(log_path_, "Failed to switch algorithm: %s", e.what());
    }
}

// ROS2 参数回调 - 监听参数变化并触发算法切换
rcl_interfaces::msg::SetParametersResult
Object_Det_Node::onParameterChange(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    LOG_INFO(log_path_, "onParameterChange called with %zu parameters", parameters.size());

    // 参数验证
    for (const auto& param : parameters) {
        LOG_INFO(log_path_, "  Parameter: %s", param.get_name().c_str());
        
        // 验证 conf_threshold 范围
        if (param.get_name() == "conf_threshold") {
            double value = param.as_double();
            if (value < 0.0 || value > 1.0) {
                result.successful = false;
                result.reason = "conf_threshold must be in range [0.0, 1.0], got " + 
                               std::to_string(value);
                LOG_ERROR(log_path_, "%s", result.reason.c_str());
                return result;
            }
        }
        
        // 验证 nms_threshold 范围
        if (param.get_name() == "nms_threshold") {
            double value = param.as_double();
            if (value < 0.0 || value > 1.0) {
                result.successful = false;
                result.reason = "nms_threshold must be in range [0.0, 1.0], got " + 
                               std::to_string(value);
                LOG_ERROR(log_path_, "%s", result.reason.c_str());
                return result;
            }
        }
        
        // 验证 max_detections 范围
        if (param.get_name() == "max_detections") {
            int value = param.as_int();
            if (value < 1 || value > 10000) {
                result.successful = false;
                result.reason = "max_detections must be in range [1, 10000], got " + 
                               std::to_string(value);
                LOG_ERROR(log_path_, "%s", result.reason.c_str());
                return result;
            }
        }
        
        // 不支持动态更新的参数提示
        if (param.get_name() == "device_id" || 
            param.get_name() == "enable_fp16" ||
            param.get_name() == "enable_int8" ||
            param.get_name() == "batch_size") {
            LOG_WARN(log_path_, "Parameter '%s' change ignored - requires node restart", 
                     param.get_name().c_str());
            // 允许修改参数值，但不会生效（仅更新参数服务器）
        }
    }

    // 检查算法参数变化
    std::string new_algorithm;
    std::string new_model_path;
    std::string new_engine_type_str;
    bool algo_params_changed = false;

    for (const auto& param : parameters) {
        LOG_INFO(log_path_, "  Parameter changed: %s", param.get_name().c_str());
        if (param.get_name() == "algorithm") {
            new_algorithm = param.as_string();
            algo_params_changed = true;
        } else if (param.get_name() == "model_path") {
            new_model_path = param.as_string();
            algo_params_changed = true;
        } else if (param.get_name() == "engine_type") {
            new_engine_type_str = param.as_string();
            algo_params_changed = true;
        }
    }

    // 如果只是检测参数变化，不需要切换算法
    if (!algo_params_changed) {
        std::lock_guard<std::mutex> lock(detection_task_mutex_);
        if (detection_task_) {
            // 🔥 使用 setConfig 统一更新所有检测参数
            // 获取当前配置
            DetectionConfig config = detection_task_->getConfig();
            
            // 从回调参数中更新配置
            for (const auto& param : parameters) {
                if (param.get_name() == "conf_threshold") {
                    config.conf_threshold = param.as_double();
                } else if (param.get_name() == "nms_threshold") {
                    config.nms_threshold = param.as_double();
                } else if (param.get_name() == "max_detections") {
                    config.max_detections = param.as_int();
                }
                // 其他参数如 use_gpu_preprocess 不建议动态修改，此处不处理
            }
            
            LOG_INFO(log_path_, "Updating detection config: conf=%.2f, nms=%.2f, max_det=%d",
                     config.conf_threshold, config.nms_threshold, config.max_detections);
            
            // 统一设置配置
            detection_task_->setConfig(config);
            
            LOG_INFO(log_path_, "Detection config updated successfully");
        }
        return result;
    }

    // 获取未变化的参数值（从当前参数服务器）
    if (new_algorithm.empty()) {
        new_algorithm = this->get_parameter("algorithm").as_string();
    }
    if (new_model_path.empty()) {
        new_model_path = this->get_parameter("model_path").as_string();
    }
    if (new_engine_type_str.empty()) {
        new_engine_type_str = this->get_parameter("engine_type").as_string();
    }

    std::string new_config_path = this->get_parameter("config_path").as_string();

    // 获取其他引擎配置参数
    int device_id = this->get_parameter("device_id").as_int();
    bool enable_fp16 = this->get_parameter("enable_fp16").as_bool();
    bool enable_int8 = this->get_parameter("enable_int8").as_bool();
    int batch_size = this->get_parameter("batch_size").as_int();

    // 转换推理引擎类型
    InferenceEngineConfig inference_engine_config;
    auto [engine_type, is_valid] = stringToEngineType(new_engine_type_str);
    inference_engine_config.engine_type = engine_type;
    if (!is_valid) {
        LOG_WARN(log_path_, "未知引擎类型: %s, 使用TensorRT", new_engine_type_str.c_str());
    }

    // 设置引擎配置参数
    inference_engine_config.device_id = device_id;
    inference_engine_config.enable_fp16 = enable_fp16;
    inference_engine_config.enable_int8 = enable_int8;
    inference_engine_config.batch_size = batch_size;

    LOG_INFO(log_path_, "Parameter change detected, switching algorithm...");
    LOG_INFO(log_path_, "  Switching to algorithm=%s, model=%s, config=%s",
             new_algorithm.c_str(), new_model_path.c_str(), new_config_path.c_str());

    // 调用切换算法方法
    switchAlgorithm(new_algorithm, new_model_path, new_config_path, inference_engine_config);

    LOG_INFO(log_path_, "Algorithm switch completed");

    return result;
}

void Object_Det_Node::initDetectionTask()
{
  try {
    // 获取算法参数（已在构造函数中声明）
    std::string algorithm = this->get_parameter("algorithm").as_string();
    std::string model_path = this->get_parameter("model_path").as_string();
    std::string engine_type_str = this->get_parameter("engine_type").as_string();

    LOG_INFO(log_path_, "算法配置: algorithm=%s, model_path=%s, engine_type=%s",
             algorithm.c_str(), model_path.c_str(), engine_type_str.c_str());

    std::string config_path = this->get_parameter("config_path").as_string();

    // 获取推理引擎配置参数
    int device_id = this->get_parameter("device_id").as_int();
    bool enable_fp16 = this->get_parameter("enable_fp16").as_bool();
    bool enable_int8 = this->get_parameter("enable_int8").as_bool();
    int batch_size = this->get_parameter("batch_size").as_int();

    // 获取检测阈值参数
    float conf_threshold = this->get_parameter("conf_threshold").as_double();
    float nms_threshold = this->get_parameter("nms_threshold").as_double();
    int max_detections = this->get_parameter("max_detections").as_int();
    bool use_gpu_preprocess = this->get_parameter("use_gpu_preprocess").as_bool();
    bool use_gpu_postprocess = this->get_parameter("use_gpu_postprocess").as_bool();
    
    // 从模型同名 .names 文件加载类别名称
    class_names_ = loadClassNamesFromModel(model_path);

    // 转换推理引擎类型
    InferenceEngineConfig engine_config;
    auto [engine_type, is_valid] = stringToEngineType(engine_type_str);
    engine_config.engine_type = engine_type;
    if (!is_valid) {
        LOG_WARN(log_path_, "未知引擎类型: %s, 使用TensorRT", engine_type_str.c_str());
    }

    // 设置引擎配置参数
    engine_config.device_id = device_id;
    engine_config.enable_fp16 = enable_fp16;
    engine_config.enable_int8 = enable_int8;
    engine_config.batch_size = batch_size;

    // 构建检测配置
    DetectionConfig det_config;
    det_config.conf_threshold = conf_threshold;
    det_config.nms_threshold = nms_threshold;
    det_config.max_detections = max_detections;
    det_config.use_gpu_preprocess = use_gpu_preprocess;
    det_config.use_gpu_postprocess = use_gpu_postprocess;
    det_config.engine_config = engine_config;

    LOG_INFO(log_path_, "Initializing detection task via factory:");
    LOG_INFO(log_path_, "  Algorithm: %s", algorithm.c_str());
    LOG_INFO(log_path_, "  Model Path: %s", model_path.c_str());
    LOG_INFO(log_path_, "  Engine Type: %s", engine_type_str.c_str());
    LOG_INFO(log_path_, "  Device ID: %d", device_id);
    LOG_INFO(log_path_, "  FP16: %s", enable_fp16 ? "enabled" : "disabled");
    LOG_INFO(log_path_, "  INT8: %s", enable_int8 ? "enabled" : "disabled");
    LOG_INFO(log_path_, "  Conf Threshold: %.2f", conf_threshold);
    LOG_INFO(log_path_, "  NMS Threshold: %.2f", nms_threshold);
    LOG_INFO(log_path_, "  Class Names: %zu classes", class_names_.size());

    // 使用工厂创建检测任务（传递引擎配置）
    detection_task_ = TaskFactory::createDetectionTask(algorithm, model_path, config_path, engine_config);

    if (!detection_task_ || !detection_task_->isInitialized()) {
      throw std::runtime_error("Failed to create detection task");
    }

    // 设置完整检测配置
    detection_task_->setConfig(det_config);
    
    // 设置类别名称（从模型同名 .names 文件加载）
    detection_task_->setClassNames(class_names_);

    LOG_INFO(log_path_, "Detection task initialized successfully:");
    LOG_INFO(log_path_, "  Algorithm: %s", detection_task_->getAlgorithmName().c_str());
    LOG_INFO(log_path_, "  Engine: %s", detection_task_->getEngineName().c_str());

  } catch (const std::exception& e) {
    LOG_ERROR(log_path_, "Failed to initialize detection task: %s", e.what());
    throw;
  }
}

Object_Det_Node::Object_Det_Node(const rclcpp::NodeOptions &options, int cam_id)
  : Node("object_det_node_" + std::to_string(cam_id), options), cam_id_(cam_id),
    module_name_("object_det"), current_status_(basros::ModuleStatus::UNKNOWN)
{
  // 预先声明所有可能被NodeOptions覆盖的参数
  this->declare_parameter("algorithm", "yolo");
  this->declare_parameter("model_path", "install/object_det/models/coco_vlm.engine");
  this->declare_parameter("engine_type", "tensorrt");
  this->declare_parameter("usecalib", true);
  this->declare_parameter("config_path", "");

  // 推理引擎配置参数
  this->declare_parameter("device_id", 0);
  this->declare_parameter("enable_fp16", false);
  this->declare_parameter("enable_int8", false);
  this->declare_parameter("batch_size", 1);

  // 检测参数
  this->declare_parameter("conf_threshold", 0.5);
  this->declare_parameter("nms_threshold", 0.45);
  this->declare_parameter("max_detections", 100);
  this->declare_parameter("use_gpu_preprocess", true);
  this->declare_parameter("use_gpu_postprocess", false);

  // 🔥 注册参数变化回调（必须在参数声明之后，并保存句柄）
  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&Object_Det_Node::onParameterChange, this, std::placeholders::_1));

  // 初始化模块状态发布器并发布启动状态
  initModuleStatusPublisher();
  publishModuleStatus(basros::ModuleStatus::STARTING, "模块正在启动");
  
  // 初始化数据流监控
  data_stream_active_ = false;
  first_frame_received_ = false;
  last_frame_time_ = this->now();
  // 创建数据流监控定时器（每2秒检查一次）
  data_stream_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&Object_Det_Node::checkDataStreamStatus, this));

  // 根据camera_id生成日志项目路径
  const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
  log_path_ = project_name + "_cam_" + std::to_string(cam_id_);

  // 新增相机类型参数
  cam_type_ = this->declare_parameter("camera_type", "Realsense");

  // 机械臂ID参数 - 用于获取对应的标定矩阵
  arm_id_ = this->declare_parameter("arm_id", 0);

  // 初始化arm_id_list_
  arm_id_list_.clear();

  // 创建参数服务器客户端
  sys_config_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "sys_config_ros_node");
  if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
  {
    LOG_INFO(log_path_, "无法连接到系统配置参数服务");
  }

  // 从参数服务器获取系统配置（相机对应的机械臂列表）
  if (!getSysDat()) {
    LOG_WARN(log_path_, "获取系统配置数据失败，但节点将继续运行");
  }

  // 如果arm_id_list_为空，使用默认arm_id
  if (arm_id_list_.size() == 0)
  {
    arm_id_ = 0;
    arm_id_list_.push_back(arm_id_);
    LOG_WARN(log_path_, "No arm ID configured in sys_cam_config, using default arm ID: %d", arm_id_);
  }
  // 默认使用第一个机械臂ID
  arm_id_ = arm_id_list_[0];

  LOG_INFO(log_path_, "Arm ID: %d, Arm ID List: [%s]", arm_id_,
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
  if(color_image_topic_.empty() || depth_image_topic_.empty() || cam_info_topic_.empty())
  {
    LOG_WARN(log_path_, "未从参数服务器正确获取话题名");
    // 如果launch文件提供了话题名，直接使用；
    if (!launch_color_topic.empty() && !launch_depth_topic.empty() && !launch_camera_info_topic.empty())
    {
        color_image_topic_ = launch_color_topic;
        depth_image_topic_ = launch_depth_topic;
        cam_info_topic_ = launch_camera_info_topic;
        LOG_INFO(log_path_, "使用launch文件提供的话题名称");
    }
    else
    {
        LOG_ERROR(log_path_, "话题名称未设置，请通过launch文件或参数服务器提供");
    }
  }

  // 初始化相机内参状态
  cam_intrinsics_initialized_ = false;
  fx_ = 0.0f;
  fy_ = 0.0f;
  cx_ = 0.0f;
  cy_ = 0.0f;

  // 初始化标定参数处理器
  initCalibParamHandler();

  // 声明usecalib参数
  usecalib_ = this->get_parameter_or("usecalib", true);
  LOG_INFO(log_path_, "标定模式: %s", usecalib_ ? "开启" : "关闭");

  LOG_INFO(log_path_, "Camera ID: %d", cam_id_);
  LOG_INFO(log_path_, "Arm ID: %d", arm_id_);
  LOG_INFO(log_path_, "Color Image Topic: %s", color_image_topic_.c_str());
  LOG_INFO(log_path_, "Depth Image Topic: %s", depth_image_topic_.c_str());
  LOG_INFO(log_path_, "Camera Info Topic: %s", cam_info_topic_.c_str());

  // 初始化新增功能 - 解耦合设计
  setup_cam_intrinsics();  // 设置相机内参

  // 【重构核心】使用抽象层初始化检测任务
  try {
    initDetectionTask();
    publishModuleStatus(basros::ModuleStatus::RUNNING, "检测任务初始化完成，等待图像数据");
  } catch (const std::exception& e) {
    publishModuleStatus(basros::ModuleStatus::ERROR, "初始化检测任务失败: " + std::string(e.what()));
    throw;
  }

  // 创建算法切换服务（服务名包含camera_id以区分不同相机节点）
  std::string switch_service_name = "/object_det_node_" + std::to_string(cam_id_) + "/switch_algorithm";
  switch_algorithm_service_ = this->create_service<custom_msgs_comm::srv::SwitchAlgorithm>(
      switch_service_name,
      std::bind(&Object_Det_Node::handleSwitchAlgorithmService, this,
                std::placeholders::_1, std::placeholders::_2));

  LOG_INFO(log_path_, "Algorithm switch service created: %s", switch_service_name.c_str());

  color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(color_image_topic_, 10, std::bind(&Object_Det_Node::Color_Callback, this, std::placeholders::_1));
  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(depth_image_topic_, 10, std::bind(&Object_Det_Node::Depth_Callback, this, std::placeholders::_1));
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(cam_info_topic_, 10, std::bind(&Object_Det_Node::CamInfo_Callback, this, std::placeholders::_1));

  // 生成话题前缀：根据camera_id
  std::string topic_prefix = "/cam_" + std::to_string(cam_id_);

  // 创建发布者 - 使用相机前缀
  std::string box_image_topic = topic_prefix + "/det_image";
  std::string res_image_topic = topic_prefix + "/det_image_png";
  std::string det_res_topic = topic_prefix + "/det_res";

  box_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(box_image_topic, 1);
  res_image_pub_ = this->create_publisher<std_msgs::msg::String>(res_image_topic, 1);
  det_res_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(det_res_topic, 1);

  LOG_INFO(log_path_, "Publishing to: %s", box_image_topic.c_str());
  LOG_INFO(log_path_, "Publishing to: %s", res_image_topic.c_str());
  LOG_INFO(log_path_, "Publishing to: %s", det_res_topic.c_str());
}

void Object_Det_Node::Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // 更新数据流状态
  last_frame_time_ = this->now();
  if (!data_stream_active_) {
    data_stream_active_ = true;
    if (!first_frame_received_) {
      first_frame_received_ = true;
      publishModuleStatus(basros::ModuleStatus::RUNNING, "图像数据流已建立，正常运行中");
    } else {
      publishModuleStatus(basros::ModuleStatus::RUNNING, "数据流已恢复");
    }
  }

  try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        
        // 检查图像是否为空
        if (frame.empty()) {
          LOG_ERROR(log_path_, "接收到空图像数据");
          publishModuleStatus(basros::ModuleStatus::ERROR, "接收到空图像数据");
          return;
        }

        // 使用互斥锁保护检测任务访问
        std::lock_guard<std::mutex> lock(detection_task_mutex_);

        // 【重构】使用抽象接口进行检测
        std::vector<DetectionResult> results = detection_task_->detect(frame);

        // 使用新接口绘制结果
        CameraIntrinsics intrinsics;
        intrinsics.fx = fx_;
        intrinsics.fy = fy_;
        intrinsics.cx = cx_;
        intrinsics.cy = cy_;
        intrinsics.width = frame.cols;
        intrinsics.height = frame.rows;

        detection_task_->drawResultsWithDepth(frame, results, depth_frame_, intrinsics);

        // 发布检测结果
        publish(results, msg->header);

        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        box_image_pub_->publish(*img_msg);
        cv::Size target(160, 120);
        cv::Mat resized;
        cv::resize(frame, resized, target, 0, 0, cv::INTER_LINEAR);
        auto png_base64 = mat2png_base64(resized);
        auto out = std_msgs::msg::String();
        out.data = std::move(png_base64);
        res_image_pub_->publish(out);
      }
      catch (const cv_bridge::Exception& e) {
        LOG_ERROR(log_path_, "trans image error: %s", e.what());
        publishModuleStatus(basros::ModuleStatus::ERROR, "图像转换异常: " + std::string(e.what()));
      }
      catch (const std::exception& e) {
        LOG_ERROR(log_path_, "Detection error: %s", e.what());
        publishModuleStatus(basros::ModuleStatus::ERROR, "检测过程异常: " + std::string(e.what()));
      }
}

void Object_Det_Node::Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    depth_frame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  }
  catch (const cv_bridge::Exception& e)
  {
    LOG_ERROR(log_path_, "trans depth image error: %s", e.what());
  }
}

// 使用parseCommInfo初始化话题名
void Object_Det_Node::initTopicNames()
{
    basros::RosCommInfo comm_info;
    comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_COLOR_IMAGE, cam_id_, 0);
    if (sys_config_client_->has_parameter(comm_info.name)) {
        color_image_topic_ = sys_config_client_->get_parameter<std::string>(comm_info.name);
    }

    comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_DEPTH_IMAGE, cam_id_, 0);
    if (sys_config_client_->has_parameter(comm_info.name)) {
        depth_image_topic_ = sys_config_client_->get_parameter<std::string>(comm_info.name);
    }

    comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_CAM_INTRINSICS, cam_id_, 0);

    if (sys_config_client_->has_parameter(comm_info.name)) {
        cam_info_topic_ = sys_config_client_->get_parameter<std::string>(comm_info.name);
    }
}

// 从参数服务器获取系统配置（包括相机对应的机械臂列表）
bool Object_Det_Node::getSysDat()
{
    // 等待参数服务可用
    if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
    {
        LOG_ERROR(log_path_, "无法连接到系统配置参数服务");
        return false;
    }

    try
    {
        SysConfig::CamConfigInfo cam_info;
        bool success = getCamConfigInfo(cam_info);
        if (!success)
        {
            LOG_WARN(log_path_, "参数服务器未找到系统参数");
        }
        return success;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(log_path_, "读取系统参数失败: %s", e.what());
        return false;
    }
}

// 获取相机配置信息
bool Object_Det_Node::getCamConfigInfo(SysConfig::CamConfigInfo& cam_info)
{
    bool bRet = RosComm::getCamInfoFromServer(sys_config_client_, cam_id_, cam_info);
    if (bRet)
    {
        LOG_INFO(log_path_, "读取到与当前相机ID匹配的配置，ID: %d, 机械臂列表: %zu", cam_info.cam_id, cam_info.armInfoList.size());
        const SysConfig::ArmConfigInfoList& arm_info_list = cam_info.armInfoList;
        if (!arm_info_list.empty())
        {
            for (const auto& arm_info : arm_info_list)
            {
                LOG_INFO(log_path_, "获取配置的机械臂ID: %d, 是否启用: %s", arm_info.arm_id, arm_info.is_enable ? "是" : "否");
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
void Object_Det_Node::initCalibParamHandler()
{
  try
  {
      // 等待参数服务器上线（最多等待10秒）
      if (sys_config_client_->wait_for_service(std::chrono::seconds(3)))
      {
          LOG_INFO(log_path_, "参数服务器已连接");

          // 初始化标定结果
          calib_result_ = std::make_unique<handeyecalib::CalibRes>();

          // 从参数服务器直接读取标定矩阵 - 根据camera_id和arm_id动态构建参数路径
          std::string param_prefix = "sys_cam_calib_list.cam_" + std::to_string(cam_id_) + ".arm_info.arm_" + std::to_string(arm_id_);
          LOG_INFO(log_path_, "标定矩阵参数前缀: %s", param_prefix.c_str());

          // 读取 cam_to_base_transform
          std::string cam_to_base_str;
          std::string cam_to_base_param = param_prefix + ".cam_to_base_transform";
          try {
              cam_to_base_str = sys_config_client_->get_parameter<std::string>(cam_to_base_param);
              calib_result_->cam_to_base_transform = handeyecalib::stringToMat(cam_to_base_str);
              LOG_INFO(log_path_, "成功读取 cam_to_base_transform");
          } catch (const std::exception& e) {
              LOG_WARN(log_path_, "读取 cam_to_base_transform 失败: %s", e.what());
          }

          // 读取 base_to_cam_transform
          std::string base_to_cam_param = param_prefix + ".base_to_cam_transform";
          try {
              std::string base_to_cam_str = sys_config_client_->get_parameter<std::string>(base_to_cam_param);
              calib_result_->base_to_cam_transform = handeyecalib::stringToMat(base_to_cam_str);
              LOG_INFO(log_path_, "成功读取 base_to_cam_transform");
          } catch (const std::exception& e) {
              LOG_WARN(log_path_, "读取 base_to_cam_transform 失败: %s", e.what());
          }

          // 读取 offset_compensation
          std::string offset_param = param_prefix + ".offset_compensation";
          try {
              calib_result_->offset_compensation = sys_config_client_->get_parameter<std::vector<double>>(offset_param);
              LOG_INFO(log_path_, "成功读取 offset_compensation, size: %zu", calib_result_->offset_compensation.size());
              if (!calib_result_->offset_compensation.empty()) {
                  LOG_INFO(log_path_, "  offset_compensation: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
                    calib_result_->offset_compensation[0], calib_result_->offset_compensation[1],
                    calib_result_->offset_compensation[2], calib_result_->offset_compensation[3],
                    calib_result_->offset_compensation[4], calib_result_->offset_compensation[5]);
              }
          } catch (const std::exception& e) {
              LOG_WARN(log_path_, "读取 offset_compensation 失败: %s", e.what());
          }

          // 打印标定矩阵信息
          LOG_INFO(log_path_, "标定结果已加载");
          if (!calib_result_->cam_to_base_transform.empty())
          {
              const auto& m = calib_result_->cam_to_base_transform;
              LOG_INFO(log_path_, "cam_to_base_transform (%d x %d):", m.rows, m.cols);
              for (int i = 0; i < m.rows; i++)
              {
                  LOG_INFO(log_path_, "  [%.6f, %.6f, %.6f, %.6f]",
                    m.at<double>(i, 0), m.at<double>(i, 1), m.at<double>(i, 2), m.at<double>(i, 3));
              }
          }
      }
      else
      {
          LOG_WARN(log_path_, "参数服务器未上线，将在后台继续等待...");
      }
  }
  catch (const std::exception& e)
  {
      LOG_WARN(log_path_, "初始化标定参数处理器失败: %s", e.what());
  }
}

// 设置相机内参 - 通过API获取
void Object_Det_Node::setup_cam_intrinsics()
{
    // 等待相机信息话题发布内参数据
    // 这里不设置硬编码值，而是等待CameraInfo回调函数获取真实的内参数据
    LOG_INFO(log_path_, "等待相机内参数据...");
    LOG_INFO(log_path_, "订阅相机信息话题: %s", cam_info_topic_.c_str());
}

// 相机信息回调函数 - 通过API获取真实的内参数据
void Object_Det_Node::CamInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (cam_intrinsics_initialized_)
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

        cam_intrinsics_initialized_ = true;

        LOG_INFO(log_path_, "成功获取相机内参:");
        LOG_INFO(log_path_, "  fx=%.2f, fy=%.2f", fx_, fy_);
        LOG_INFO(log_path_, "  cx=%.2f, cy=%.2f", cx_, cy_);
        LOG_INFO(log_path_, "  图像尺寸: %dx%d", msg->width, msg->height);

        // 检查内参是否合理
        if (fx_ <= 0 || fy_ <= 0)
        {
            LOG_WARN(log_path_, "相机内参异常，使用默认值");
            // 使用合理的默认值
            if (cam_type_ == "Gemini")
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
        LOG_WARN(log_path_, "CameraInfo消息格式错误，使用默认内参");
        // 使用默认内参
        if (cam_type_ == "Gemini")
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
        cam_intrinsics_initialized_ = true;
    }
}

// 计算camera_link下的3D坐标
geometry_msgs::msg::Point Object_Det_Node::calculate_3d_position(const DetectionResult& det)
{
    geometry_msgs::msg::Point cam_point;

    // 检查相机内参是否已初始化
    if (!cam_intrinsics_initialized_)
    {
        LOG_WARN(log_path_, "相机内参未初始化，无法计算3D坐标");
        return cam_point;
    }

    // 检查内参是否有效
    if (fx_ <= 0 || fy_ <= 0)
    {
        LOG_WARN(log_path_, "相机内参无效，无法计算3D坐标");
        return cam_point;
    }

    // 获取边界框中心点
    float center_x = det.center.x;  // 边界框中心x
    float center_y = det.center.y;  // 边界框中心y

    // 确保深度图像有效
    if (depth_frame_.empty())
    {
        LOG_WARN(log_path_, "深度图像为空，无法计算3D坐标");
        return cam_point;
    }

    // 检查像素坐标是否在图像范围内
    int pixel_x = static_cast<int>(center_x);
    int pixel_y = static_cast<int>(center_y);

    if (pixel_x < 0 || pixel_x >= depth_frame_.cols ||
        pixel_y < 0 || pixel_y >= depth_frame_.rows)
    {
        LOG_WARN(log_path_, "像素坐标超出图像范围: (%d, %d)", pixel_x, pixel_y);
        return cam_point;
    }

    // 获取深度值
    uint16_t depth_value = depth_frame_.at<uint16_t>(pixel_y, pixel_x);

    if (depth_value == 0)
    {
        LOG_WARN(log_path_, "深度值为0，无效深度");
        return cam_point;
    }

   // 输出为毫米
    float depth_mm = depth_value;
    cam_point.x = (center_x - cx_) * depth_mm / fx_; // X坐标
    cam_point.y = (center_y - cy_) * depth_mm / fy_; // Y坐标
    cam_point.z = depth_mm;                          // Z坐标（深度）

    return cam_point;
}

// 把图像时间戳/坐标系带过来
void Object_Det_Node::publish(const std::vector<DetectionResult>& dets, const std_msgs::msg::Header& header)
{
    vision_msgs::msg::Detection2DArray arr;
    arr.header = header;
    if(dets.size()!=0)
        LOG_INFO(log_path_, "检测个数: %zu", dets.size());
    for (const auto& d : dets)
    {
        vision_msgs::msg::Detection2D det;
        det.bbox.center.position.x = d.center.x;
        det.bbox.center.position.y = d.center.y;
        det.bbox.size_x            = d.width;
        det.bbox.size_y            = d.height;

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = d.class_name;
        hyp.hypothesis.score = d.confidence;

        // 新增：使用深度图像计算3D坐标并转换到base_link坐标系
        if (!depth_frame_.empty())
        {
            // 计算camera_link下的3D坐标
            geometry_msgs::msg::Point cam_point = calculate_3d_position(d);
            LOG_INFO(log_path_, "相机坐标系: (%.3f, %.3f, %.3f) class_name: %s, confidence: %.2f",
                                    cam_point.x, cam_point.y, cam_point.z, d.class_name.c_str(), d.confidence);
            // 转换为base_link坐标系
            geometry_msgs::msg::Point base_point = cam_point;

            // 如果启用标定模式，将相机坐标系转换为base坐标系
            if (usecalib_ && calib_result_ && !calib_result_->cam_to_base_transform.empty())
            {
                try {
                    std::vector<double> marker_pose = {cam_point.x, cam_point.y, cam_point.z, 0.0, 0.0, 0.0};
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

                        LOG_INFO(log_path_, "base坐标系: (%.3f, %.3f, %.3f)",
                                    base_point.x, base_point.y, base_point.z);
                    }
                }
                catch (const std::exception& e) {
                    LOG_WARN(log_path_, "坐标转换失败: %s", e.what());
                }
            }

            // 填充到hyp.pose.pose.position
            hyp.pose.pose.position = base_point;

            // 设置姿态（默认朝向）
            hyp.pose.pose.orientation.x = 0.0;
            hyp.pose.pose.orientation.y = 0.0;
            hyp.pose.pose.orientation.z = 0.0;
            hyp.pose.pose.orientation.w = 1.0;

            LOG_DEBUG(log_path_, "检测目标3D坐标 - camera_link: (%.3f, %.3f, %.3f), base_link: (%.3f, %.3f, %.3f)",
                cam_point.x, cam_point.y, cam_point.z, base_point.x, base_point.y, base_point.z);
        }
        else
        {
            // 如果没有深度图像，使用DetectionResult中的3D坐标
            hyp.pose.pose.position.x = d.position_3d.x;                  // 单位：米
            hyp.pose.pose.position.y = d.position_3d.y;
            hyp.pose.pose.position.z = d.position_3d.z;
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
  std::string algorithm = "yolo";
  std::string model_path = "install/object_det/models/coco_vlm.engine";
  std::string input_folder = "src/perception/object_det/test_imgs";
  std::string output_folder = "output_results";

  std::string names_path = model_path;
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

  InferenceEngineConfig engine_config;
  engine_config.engine_type = InferenceEngineType::TENSORRT;
  engine_config.device_id = 0;
  engine_config.enable_fp16 = false;
  engine_config.enable_int8 = false;
  engine_config.batch_size = 1;
  engine_config.num_threads = 4;

  DetectionConfig det_config;
  det_config.conf_threshold = 0.5f;
  det_config.nms_threshold = 0.45f;
  det_config.use_gpu_preprocess = true;
  det_config.use_gpu_postprocess = true;
  det_config.max_detections = 100;
  det_config.engine_config = engine_config;

  auto detector = TaskFactory::createDetectionTask(algorithm, model_path, "", engine_config);
  if (!detector || !detector->isInitialized()) {
      throw std::runtime_error("Failed to create detection task");
  }
  detector->setConfig(det_config);
  detector->setClassNames(class_names);

  std::cout << "Algorithm: " << detector->getAlgorithmName()
            << ", Engine: " << detector->getEngineName() << std::endl;
  std::cout << "GPU预处理: " << (det_config.use_gpu_preprocess ? "ON" : "OFF")
            << ", GPU后处理: " << (det_config.use_gpu_postprocess ? "ON" : "OFF") << std::endl;

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
      std::vector<DetectionResult> detections;
      try {
          detections = detector->detect(img);
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
              std::cout << "Center: (" << d.center.x << ", " << d.center.y
                        << "), Size: " << (int)d.width << "x" << (int)d.height
                        << ", Conf=" << d.confidence
                        << ", Class=" << d.class_name << std::endl;
          }
      }

      cv::Mat result_img = img.clone();
      detector->drawResults(result_img, detections);

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

void Object_Det_Node::initModuleStatusPublisher()
{
    // 话题名称格式: /cam{cam_id}/object_det/mdl_status_info
    std::string topic_name = "/cam" + std::to_string(cam_id_) + "/" + module_name_ + "/mdl_status_info";
    module_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    LOG_INFO(log_path_, "模块状态发布器初始化完成，话题: %s", topic_name.c_str());
}

// 检查数据流状态
void Object_Det_Node::checkDataStreamStatus()
{
    auto now = this->now();
    auto elapsed = (now - last_frame_time_).seconds();
    
    // 启动阶段：尚未收到首帧，检测是否超时
    if (!first_frame_received_) {
        if (elapsed > 5.0) {  // 启动后5秒仍未收到数据
            publishModuleStatus(basros::ModuleStatus::RUNNING_PAUSED, 
                               "等待图像数据超时，请检查相机状态");
            LOG_WARN(log_path_, "启动后 %.1f 秒未收到图像数据，请检查相机是否开启", elapsed);
        }
        return;
    }
    
    // 运行阶段：数据流停止检测
    if (elapsed > 5.0 && data_stream_active_) {
        data_stream_active_ = false;
        publishModuleStatus(basros::ModuleStatus::RUNNING_PAUSED, 
                           "数据流已停止，等待图像数据");
        LOG_WARN(log_path_, "数据流已停止，超过 %.1f 秒无图像数据", elapsed);
    }
}

void Object_Det_Node::publishModuleStatus(basros::ModuleStatus status, const std::string& status_msg)
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_ = status;
    basros::ModuleStatusInfo status_info(module_name_, cam_id_, status, status_msg);
    std::string json_str = basros::moduleStatusInfoToJson(status_info);
    auto msg = std_msgs::msg::String();
    msg.data = json_str;
    module_status_pub_->publish(msg);
    LOG_INFO(log_path_, "发布模块状态: %s - %s", basros::moduleStatusToString(status).c_str(), status_msg.c_str());
}

// 目标检测主节点 - 用于管理多个相机节点
class ObjectDetMainNode : public rclcpp::Node
{
public:
  explicit ObjectDetMainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("object_det_main_node", options)
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
      std::string config_file_path = ament_index_cpp::get_package_share_directory("object_det")
        + "/config" + "/object_det_params.yaml";
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
    std::string default_config_file_path = ament_index_cpp::get_package_share_directory("object_det")
      + "/config" + "/object_det_params.yaml";
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
    LOG_INFO(log_project_path, "object_detection节点启动");
    rclcpp::executors::MultiThreadedExecutor executor;

    auto main_node = std::make_shared<ObjectDetMainNode>();
    std::vector<int> act_cam_ids = main_node->getActiveCameraIds();

    if (act_cam_ids.empty())
    {
      LOG_ERROR(log_project_path, "有效配置的相机ID列表为空,默认使用相机ID 0");
      act_cam_ids = {0};
    }

    std::vector<std::shared_ptr<Object_Det_Node>> nodes;
    for (int camera_id : act_cam_ids)
    {
      LOG_INFO(log_project_path, "创建 Object_Det_Node 节点，camera_id: %d", camera_id);

      // 通过NodeOptions传递camera_id参数
      rclcpp::NodeOptions node_options;
      node_options.append_parameter_override("camera_id", rclcpp::ParameterValue(camera_id));

      // 从配置文件中读取对应相机的独立参数
      try
      {
        std::string config_file_path = ament_index_cpp::get_package_share_directory("object_det")
          + "/config" + "/object_det_params.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);

        std::string cam_param_key = "cam_" + std::to_string(camera_id) + "_parameters";
        if (config[cam_param_key])
        {
          YAML::Node cam_config = config[cam_param_key];
          LOG_INFO(log_project_path, "读取相机%d的独立配置", camera_id);

          // 传递相机独立配置参数
          if (cam_config["algorithm"])
            node_options.append_parameter_override("algorithm", rclcpp::ParameterValue(cam_config["algorithm"].as<std::string>()));
          if (cam_config["model_path"])
            node_options.append_parameter_override("model_path", rclcpp::ParameterValue(cam_config["model_path"].as<std::string>()));
          if (cam_config["engine_type"])
            node_options.append_parameter_override("engine_type", rclcpp::ParameterValue(cam_config["engine_type"].as<std::string>()));
          if (cam_config["usecalib"])
            node_options.append_parameter_override("usecalib", rclcpp::ParameterValue(cam_config["usecalib"].as<bool>()));

          // 推理引擎配置参数
          if (cam_config["device_id"])
            node_options.append_parameter_override("device_id", rclcpp::ParameterValue(cam_config["device_id"].as<int>()));
          if (cam_config["enable_fp16"])
            node_options.append_parameter_override("enable_fp16", rclcpp::ParameterValue(cam_config["enable_fp16"].as<bool>()));
          if (cam_config["enable_int8"])
            node_options.append_parameter_override("enable_int8", rclcpp::ParameterValue(cam_config["enable_int8"].as<bool>()));
          if (cam_config["batch_size"])
            node_options.append_parameter_override("batch_size", rclcpp::ParameterValue(cam_config["batch_size"].as<int>()));

          // 检测参数
          if (cam_config["conf_threshold"])
            node_options.append_parameter_override("conf_threshold", rclcpp::ParameterValue(cam_config["conf_threshold"].as<double>()));
          if (cam_config["nms_threshold"])
            node_options.append_parameter_override("nms_threshold", rclcpp::ParameterValue(cam_config["nms_threshold"].as<double>()));
          if (cam_config["max_detections"])
            node_options.append_parameter_override("max_detections", rclcpp::ParameterValue(cam_config["max_detections"].as<int>()));
          if (cam_config["use_gpu_preprocess"])
            node_options.append_parameter_override("use_gpu_preprocess", rclcpp::ParameterValue(cam_config["use_gpu_preprocess"].as<bool>()));
          if (cam_config["use_gpu_postprocess"])
            node_options.append_parameter_override("use_gpu_postprocess", rclcpp::ParameterValue(cam_config["use_gpu_postprocess"].as<bool>()));
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

      auto node = std::make_shared<Object_Det_Node>(node_options, camera_id);
      nodes.push_back(node);
      executor.add_node(node);
    }

    executor.spin();

    // 正常退出前发布停止状态
    for (auto& node : nodes) {
        node->publishModuleStatus(basros::ModuleStatus::STOPPED, "模块正常停止");
    }
    rclcpp::shutdown();
  }
  return 0;
}