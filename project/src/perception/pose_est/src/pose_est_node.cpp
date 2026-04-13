#include "pose_est_node.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include "bas_operate/file_operate.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace detection_6d;
using namespace inference_core;

// ============================================================================
// 工具：从模型路径推导并加载 .names 文件
// ============================================================================
std::vector<std::string> PoseEstNode::loadClassNamesFromModel(const std::string& model_path)
{
    std::string names_path = model_path;
    size_t dot_pos = names_path.rfind('.');
    if (dot_pos != std::string::npos) {
        names_path = names_path.substr(0, dot_pos) + ".names";
    } else {
        names_path += ".names";
    }

    std::vector<std::string> names;
    std::ifstream f(names_path);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open class names file: " + names_path +
                                 ". Each seg model must have a corresponding .names file.");
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

    return names;
}

// ============================================================================
// Helper functions (unchanged from original - used by test())
// ============================================================================

// 辅助函数：读取相机内参
Eigen::Matrix3f ReadCamK(const std::string& cam_k_path)
{
    Eigen::Matrix3f intrinsic = Eigen::Matrix3f::Identity();
    if (cam_k_path.empty())
    {
        return intrinsic;
    }
    
    std::ifstream file(cam_k_path);
    if (!file.is_open())
    {
        std::cerr << "Failed to open camera intrinsics file: " << cam_k_path << std::endl;
        return intrinsic;
    }

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            file >> intrinsic(i, j);
        }
    }
    
    std::cout << "Loaded camera intrinsics:\n"
              << intrinsic(0, 0) << " " << intrinsic(0, 1) << " " << intrinsic(0, 2) << "\n"
              << intrinsic(1, 0) << " " << intrinsic(1, 1) << " " << intrinsic(1, 2) << "\n"
              << intrinsic(2, 0) << " " << intrinsic(2, 1) << " " << intrinsic(2, 2) << std::endl;
    return intrinsic;
}

std::vector<std::filesystem::path> get_files_in_directory(const std::string& dir_path)
{
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(dir_path))
    {
        if (entry.is_regular_file())
        {
            files.push_back(entry.path());
        }
    }
    return files;
}

// 辅助函数：读取RGB和Depth图像
std::tuple<cv::Mat, cv::Mat> ReadRgbDepth(const std::string& rgb_path, const std::string& depth_path)
{
    cv::Mat rgb = cv::imread(rgb_path, cv::IMREAD_COLOR);
    cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
    
    cv::Mat depth = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
    depth.convertTo(depth, CV_32F);
    depth = depth / 1000.0f;
    
    return {rgb, depth};
}

// 辅助函数：读取RGB、Depth和Mask图像
std::tuple<cv::Mat, cv::Mat, cv::Mat> ReadRgbDepthMask(const std::string& rgb_path, 
                                                        const std::string& depth_path,
                                                        const std::string& mask_path)
{
    auto [rgb, depth] = ReadRgbDepth(rgb_path, depth_path);
    
    cv::Mat mask;
    if (!mask_path.empty())
    {
        mask = cv::imread(mask_path, cv::IMREAD_GRAYSCALE);
    }
    else
    {
        mask = (depth > 0);
    }
    
    return {rgb, depth, mask};
}

// 辅助函数：绘制3D边界框（包含RGB三轴）
void draw3DBoundingBox(const Eigen::Matrix3f& intrinsic, const Eigen::Matrix4f& pose,
                       int height, int width, const Eigen::Vector3f& dimension, cv::Mat& image)
{
    // 获取物体尺寸
    float l = dimension(0) / 2;
    float w = dimension(1) / 2;
    float h = dimension(2) / 2;

    // 定义3D边界框的8个顶点
    Eigen::Vector3f points[8] = {
        {-l, -w, h},  {l, -w, h},   {l, w, h},   {-l, w, h},
        {-l, -w, -h}, {l, -w, -h},  {l, w, -h},  {-l, w, -h}
    };

    // 变换到世界坐标系
    Eigen::Vector4f transformed_points[8];
    for (int i = 0; i < 8; ++i)
    {
        transformed_points[i] = pose * Eigen::Vector4f(points[i](0), points[i](1), points[i](2), 1);
    }

    // 投影到图像平面
    std::vector<cv::Point2f> image_points;
    for (int i = 0; i < 8; ++i)
    {
        float x = transformed_points[i](0) / transformed_points[i](2);
        float y = transformed_points[i](1) / transformed_points[i](2);

        float u = intrinsic(0, 0) * x + intrinsic(0, 2);
        float v = intrinsic(1, 1) * y + intrinsic(1, 2);

        image_points.emplace_back(static_cast<float>(u), static_cast<float>(v));
    }

    // 绘制边框（连接顶点）
    std::vector<std::pair<int, int>> edges = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, // 底面
        {4, 5}, {5, 6}, {6, 7}, {7, 4}, // 顶面
        {0, 4}, {1, 5}, {2, 6}, {3, 7}  // 侧面
    };

    for (const auto& edge : edges)
    {
        if (edge.first < image_points.size() && edge.second < image_points.size())
        {
            cv::line(image, image_points[edge.first], image_points[edge.second], cv::Scalar(0, 255, 0), 2); // 绿色边框
        }
    }

    // 计算物体中心在世界坐标系中的位置
    Eigen::Vector4f center_obj(0, 0, 0, 1);  // 物体坐标系中心原点
    Eigen::Vector4f center_world = pose * center_obj;

    // 定义三个轴的终点
    float axis_length = (dimension(0) + dimension(1) + dimension(2)) / 6;
    Eigen::Vector4f x_end_obj(axis_length, 0, 0, 1);
    Eigen::Vector4f y_end_obj(0, axis_length, 0, 1);
    Eigen::Vector4f z_end_obj(0, 0, axis_length, 1);

    Eigen::Vector4f x_end_world = pose * x_end_obj;
    Eigen::Vector4f y_end_world = pose * y_end_obj;
    Eigen::Vector4f z_end_world = pose * z_end_obj;

    std::vector<Eigen::Vector4f> axis_end_points = {x_end_world, y_end_world, z_end_world};
    std::vector<cv::Scalar> axis_colors = {
        cv::Scalar(0, 0, 255),     // X轴: 红色
        cv::Scalar(0, 255, 0),     // Y轴: 绿色
        cv::Scalar(255, 0, 0)      // Z轴: 蓝色
    };

    float cx = center_world(0) / center_world(2);
    float cy = center_world(1) / center_world(2);
    float cu = intrinsic(0, 0) * cx + intrinsic(0, 2);
    float cv_val = intrinsic(1, 1) * cy + intrinsic(1, 2);
    cv::Point center_pt(cu, cv_val);

    for (int k = 0; k < 3; ++k)
    {
        float ex = axis_end_points[k](0) / axis_end_points[k](2);
        float ey = axis_end_points[k](1) / axis_end_points[k](2);
        float eu = intrinsic(0, 0) * ex + intrinsic(0, 2);
        float ev = intrinsic(1, 1) * ey + intrinsic(1, 2);

        cv::line(image, center_pt, cv::Point(eu, ev), axis_colors[k], 3);
    }
}

// ============================================================================
// PoseEstMainNode implementation
// ============================================================================

PoseEstMainNode::PoseEstMainNode(const rclcpp::NodeOptions& options)
    : Node("pose_est_main_node", options)
{
    log_project_path_ = basmodule::get_project_name_by_file_path(__FILE__);
    sys_config_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "sys_config_ros_node");
}

std::vector<int> PoseEstMainNode::getServerCameraIds()
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
        for (const auto& id : temp_camera_ids)
        {
            cam_server_ids_.push_back(static_cast<int>(id));
            cam_ids_str += std::to_string(id) + " ";
        }
        LOG_INFO(log_project_path_, "从系统配置参数服务获取相机ID列表: %s", cam_ids_str.c_str());
    }
    catch (const YAML::Exception& e)
    {
        LOG_ERROR(log_project_path_, "解析配置文件出错: %s", e.what());
    }
    return cam_server_ids_;
}

std::vector<int> PoseEstMainNode::getConfigCameraIds()
{
    try
    {
        cam_config_ids_.clear();
        std::string config_file_path = ament_index_cpp::get_package_share_directory("pose_est")
            + "/config/pose_est_params.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);
        std::string cam_ids_str;
        if (config["ros__parameters"] && config["ros__parameters"]["camera_id"])
        {
            YAML::Node camera_id_node = config["ros__parameters"]["camera_id"];
            if (camera_id_node.IsSequence())
            {
                for (const auto& id : camera_id_node)
                {
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

std::vector<int> PoseEstMainNode::getActiveCameraIds()
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
        {
            LOG_WARN(log_project_path_, "参数服务器没有配置该相机ID，camera_id: %d", camera_id);
        }
    }
    LOG_INFO(log_project_path_, "有效配置的相机ID列表: %s", cam_ids_str.c_str());
    cam_active_ids_ = act_cam_ids;
    return act_cam_ids;
}

// ============================================================================
// PoseEstNode implementation
// ============================================================================

PoseEstNode::PoseEstNode(const rclcpp::NodeOptions& options, int cam_id)
    : Node("pose_est_node_" + std::to_string(cam_id), options), state_(State::IDLE), camera_id_(cam_id), processing_(false)
{
    // Logging setup
    const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
    log_project_path_ = project_name + "_cam_" + std::to_string(camera_id_);

    // Declare parameters
    this->declare_parameter("camera_type", "realsense");
    this->declare_parameter("arm_id", 0);
    this->declare_parameter("usecalib", true);
    this->declare_parameter("color_image_topic", "");
    this->declare_parameter("depth_image_topic", "");
    this->declare_parameter("camera_info_topic", "");

    // FoundationPose model parameters
    this->declare_parameter("refiner_engine_path", "install/pose_est/models/refiner_hwc_dynamic_fp16.engine");
    this->declare_parameter("scorer_engine_path", "install/pose_est/models/scorer_hwc_dynamic_fp16.engine");
    this->declare_parameter("mesh_path", "");
    this->declare_parameter("target_name", "object");

    // YOLO segmentation parameters
    this->declare_parameter("seg_engine_path", "install/pose_est/models/yolo11n-seg.engine");
    this->declare_parameter("target_class_name", "");    // 空字符串表示自动选择置信度最高的检测
    this->declare_parameter("seg_conf_threshold", 0.5);
    this->declare_parameter("seg_iou_threshold", 0.3);

    // Read parameter values
    camera_type_ = this->get_parameter("camera_type").as_string();
    arm_id_ = this->get_parameter("arm_id").as_int();
    arm_id_list_.clear();

    refiner_engine_path_ = this->get_parameter("refiner_engine_path").as_string();
    scorer_engine_path_ = this->get_parameter("scorer_engine_path").as_string();
    mesh_path_ = this->get_parameter("mesh_path").as_string();
    target_name_ = this->get_parameter("target_name").as_string();

    seg_engine_path_ = this->get_parameter("seg_engine_path").as_string();
    target_class_name_ = this->get_parameter("target_class_name").as_string();
    seg_conf_threshold_ = this->get_parameter("seg_conf_threshold").as_double();
    seg_iou_threshold_ = this->get_parameter("seg_iou_threshold").as_double();

    // 从分割模型同名 .names 文件加载类别名称
    try {
        seg_class_names_ = loadClassNamesFromModel(seg_engine_path_);
    } catch (const std::exception& e) {
        LOG_ERROR(log_project_path_, "加载分割模型类别名称失败: %s", e.what());
    }

    usecalib_ = this->get_parameter("usecalib").as_bool();

    // Track validation parameters
    track_max_depth_error_ = this->get_parameter_or("track_max_depth_error", rclcpp::ParameterValue(0.15)).get<double>();
    track_fail_reset_count_ = this->get_parameter_or("track_fail_reset_count", rclcpp::ParameterValue(10)).get<int>();
    reinit_period_ms_ = this->get_parameter_or("reinit_period_ms", rclcpp::ParameterValue(0)).get<int>();
    track_min_visible_ratio_ = this->get_parameter_or("track_min_visible_ratio", rclcpp::ParameterValue(0.3)).get<double>();

    track_fail_count_ = 0;

    // Connect to parameter server
    sys_config_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "sys_config_ros_node");
    if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
    {
        LOG_INFO(log_project_path_, "无法连接到系统配置参数服务");
    }

    // Get system config (arm ID list)
    if (!getSysDat())
    {
        LOG_WARN(log_project_path_, "获取系统配置数据失败，但节点将继续运行");
    }

    if (arm_id_list_.empty())
    {
        arm_id_ = 0;
        arm_id_list_.push_back(arm_id_);
        LOG_WARN(log_project_path_, "No arm ID configured, using default arm ID: %d", arm_id_);
    }
    arm_id_ = arm_id_list_[0];

    LOG_INFO(log_project_path_, "Arm ID: %d", arm_id_);

    // Initialize topic names from parameter server
    initTopicNames();
    std::string launch_color = this->get_parameter("color_image_topic").as_string();
    std::string launch_depth = this->get_parameter("depth_image_topic").as_string();
    std::string launch_cam_info = this->get_parameter("camera_info_topic").as_string();

    if (color_image_topic_.empty() || depth_image_topic_.empty() || camera_info_topic_.empty())
    {
        LOG_WARN(log_project_path_, "未从参数服务器正确获取话题名");
        if (!launch_color.empty() && !launch_depth.empty() && !launch_cam_info.empty())
        {
            color_image_topic_ = launch_color;
            depth_image_topic_ = launch_depth;
            camera_info_topic_ = launch_cam_info;
            LOG_INFO(log_project_path_, "使用launch文件提供的话题名称");
        }
        else
        {
            LOG_ERROR(log_project_path_, "话题名称未设置，请通过launch文件或参数服务器提供");
        }
    }

    // Initialize camera intrinsics state
    camera_intrinsics_initialized_ = false;
    fx_ = fy_ = cx_ = cy_ = 0.0f;
    intrinsic_ = Eigen::Matrix3f::Identity();

    // Initialize calibration
    initCalibParamHandler();

    LOG_INFO(log_project_path_, "Camera ID: %d", camera_id_);
    LOG_INFO(log_project_path_, "Arm ID: %d", arm_id_);
    LOG_INFO(log_project_path_, "Use Calib: %s", usecalib_ ? "true" : "false");
    LOG_INFO(log_project_path_, "Color Topic: %s", color_image_topic_.c_str());
    LOG_INFO(log_project_path_, "Depth Topic: %s", depth_image_topic_.c_str());
    LOG_INFO(log_project_path_, "CameraInfo Topic: %s", camera_info_topic_.c_str());
    LOG_INFO(log_project_path_, "Refiner Engine: %s", refiner_engine_path_.c_str());
    LOG_INFO(log_project_path_, "Scorer Engine: %s", scorer_engine_path_.c_str());
    LOG_INFO(log_project_path_, "Mesh Path: %s", mesh_path_.c_str());
    LOG_INFO(log_project_path_, "Target Name: %s", target_name_.c_str());
    LOG_INFO(log_project_path_, "Seg Engine: %s", seg_engine_path_.c_str());
    LOG_INFO(log_project_path_, "Target Class Name: %s", target_class_name_.empty() ? "(auto)" : target_class_name_.c_str());
    LOG_INFO(log_project_path_, "Seg Class Names: %zu classes loaded", seg_class_names_.size());
    LOG_INFO(log_project_path_, "Seg Conf Threshold: %.2f", seg_conf_threshold_);
    LOG_INFO(log_project_path_, "Seg IoU Threshold: %.2f", seg_iou_threshold_);

    // === YOLO分割模型延迟加载 ===
    // 不在构造函数中加载YoloSeg，而是进入IDLE状态后自动加载。
    // 原因：Register需要refiner context(~2.4GB) + scorer context(~1GB)的连续显存，
    // 如果YoloSeg(~1GB)先加载再释放，会导致CUDA内存碎片化，Register阶段scorer分配失败。
    // 延迟加载确保YoloSeg和TRT context不会同时存在于显存中。
    // seg_detector_ 默认为 nullptr，processFrame的IDLE状态会自动加载

    // === Load FoundationPose models ===
    if (mesh_path_.empty())
    {
        LOG_ERROR(log_project_path_, "mesh_path未配置，FoundationPose无法初始化");
    }
    else
    {
        try
        {
            // 检查引擎文件是否存在
            if (!std::filesystem::exists(refiner_engine_path_))
            {
                LOG_ERROR(log_project_path_, "Refiner引擎文件不存在: %s", refiner_engine_path_.c_str());
            }
            if (!std::filesystem::exists(scorer_engine_path_))
            {
                LOG_ERROR(log_project_path_, "Scorer引擎文件不存在: %s", scorer_engine_path_.c_str());
            }

            const int batch_size = 252;
            const int render_h = 160;
            const int render_w = 160;

            refiner_core_ = CreateTrtInferCore(refiner_engine_path_,
                {{"transf_input", {batch_size, render_h, render_w, 6}},
                 {"render_input", {batch_size, render_h, render_w, 6}}},
                {{"trans", {batch_size, 3}}, {"rot", {batch_size, 3}}}, 1);

            scorer_core_ = CreateTrtInferCore(scorer_engine_path_,
                {{"transf_input", {batch_size, render_h, render_w, 6}},
                 {"render_input", {batch_size, render_h, render_w, 6}}},
                {{"scores", {batch_size, 1}}}, 1);

            // Load mesh
            mesh_loader_ = CreateAssimpMeshLoader(target_name_, mesh_path_);
            if (!mesh_loader_)
            {
                LOG_ERROR(log_project_path_, "Mesh加载失败: %s", mesh_path_.c_str());
            }
            else
            {
                LOG_INFO(log_project_path_, "Mesh加载成功: vertices=%zu, faces=%zu, diameter=%.4f",
                    mesh_loader_->GetMeshNumVertices(), mesh_loader_->GetMeshNumFaces(),
                    mesh_loader_->GetMeshDiameter());
            }

            // FoundationPose model will be created after camera intrinsics are received
            state_ = State::IDLE;
            current_pose_ = Eigen::Matrix4f::Identity();
            LOG_INFO(log_project_path_, "FoundationPose推理核心加载成功，等待相机内参...");
        }
        catch (const std::exception& e)
        {
            LOG_ERROR(log_project_path_, "FoundationPose模型加载失败: %s", e.what());
            state_ = State::IDLE;
        }
    }

    // Create subscribers
    color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(color_image_topic_, 10,
        std::bind(&PoseEstNode::Color_Callback, this, std::placeholders::_1));
    depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(depth_image_topic_, 10,
        std::bind(&PoseEstNode::Depth_Callback, this, std::placeholders::_1));
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10,
        std::bind(&PoseEstNode::CameraInfo_Callback, this, std::placeholders::_1));

    // Create publishers
    std::string topic_prefix = "/cam_" + std::to_string(camera_id_);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix + "/pose_est_pose", 1);
    pose_base_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix + "/pose_est_pose_base", 1);
    result_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_prefix + "/pose_est_image", 1);

    LOG_INFO(log_project_path_, "位姿估计节点初始化完成");
    LOG_INFO(log_project_path_, "Publishing pose to: %s", (topic_prefix + "/pose_est_pose").c_str());
    LOG_INFO(log_project_path_, "Publishing pose_base to: %s", (topic_prefix + "/pose_est_pose_base").c_str());
    LOG_INFO(log_project_path_, "Publishing result image to: %s", (topic_prefix + "/pose_est_image").c_str());
}

PoseEstNode::~PoseEstNode()
{
    LOG_INFO(log_project_path_, "位姿估计节点析构，释放资源...");
    // 先释放foundation_pose（内部持有refiner/scorer的shared_ptr引用）
    foundation_pose_.reset();
    refiner_core_.reset();
    scorer_core_.reset();
    seg_detector_.reset();
    LOG_INFO(log_project_path_, "资源已释放");
}

void PoseEstNode::Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        depth_frame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    }
    catch (const cv_bridge::Exception& e)
    {
        LOG_ERROR(log_project_path_, "深度图像转换错误: %s", e.what());
    }
}

void PoseEstNode::CameraInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
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

        if (fx_ <= 0 || fy_ <= 0)
        {
            LOG_WARN(log_project_path_, "相机内参异常(fx=%.2f, fy=%.2f)，使用默认值", fx_, fy_);
            fx_ = 615.0f;
            fy_ = 615.0f;
            cx_ = msg->width / 2.0f;
            cy_ = msg->height / 2.0f;
        }

        intrinsic_ << fx_, 0, cx_,
                       0, fy_, cy_,
                       0, 0, 1;

        camera_intrinsics_initialized_ = true;
        LOG_INFO(log_project_path_, "相机内参获取成功: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
        LOG_INFO(log_project_path_, "图像尺寸: %dx%d", msg->width, msg->height);

        // Now that we have intrinsics, create the FoundationPose model
        if (refiner_core_ && scorer_core_ && mesh_loader_ && !foundation_pose_)
        {
            try
            {
                foundation_pose_ = CreateFoundationPoseModel(refiner_core_, scorer_core_, {mesh_loader_}, intrinsic_);
                LOG_INFO(log_project_path_, "FoundationPose模型创建成功");
            }
            catch (const std::exception& e)
            {
                LOG_ERROR(log_project_path_, "FoundationPose模型创建失败: %s", e.what());
            }
        }
    }
    else
    {
        LOG_WARN(log_project_path_, "CameraInfo格式错误，使用默认内参");
        fx_ = 615.0f;
        fy_ = 615.0f;
        cx_ = 320.0f;
        cy_ = 240.0f;
        intrinsic_ << fx_, 0, cx_,
                       0, fy_, cy_,
                       0, 0, 1;
        camera_intrinsics_initialized_ = true;
    }
}

void PoseEstNode::initTopicNames()
{
    if (!sys_config_client_->wait_for_service(std::chrono::seconds(1)))
    {
        return;
    }

    basros::RosCommInfo comm_info;
    comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_COLOR_IMAGE, camera_id_, 0);
    if (sys_config_client_->has_parameter(comm_info.name))
    {
        color_image_topic_ = sys_config_client_->get_parameter<std::string>(comm_info.name);
    }

    comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_DEPTH_IMAGE, camera_id_, 0);
    if (sys_config_client_->has_parameter(comm_info.name))
    {
        depth_image_topic_ = sys_config_client_->get_parameter<std::string>(comm_info.name);
    }

    comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_CAM_INTRINSICS, camera_id_, 0);
    if (sys_config_client_->has_parameter(comm_info.name))
    {
        camera_info_topic_ = sys_config_client_->get_parameter<std::string>(comm_info.name);
    }
}

bool PoseEstNode::getSysDat()
{
    if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
    {
        LOG_ERROR(log_project_path_, "无法连接到系统配置参数服务");
        return false;
    }

    try
    {
        SysConfig::CamConfigInfo cam_info;
        return getCamConfigInfo(cam_info);
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(log_project_path_, "读取系统参数失败: %s", e.what());
        return false;
    }
}

bool PoseEstNode::getCamConfigInfo(SysConfig::CamConfigInfo& cam_info)
{
    bool bRet = RosComm::getCamInfoFromServer(sys_config_client_, camera_id_, cam_info);
    if (bRet)
    {
        LOG_INFO(log_project_path_, "读取到相机配置，ID: %d, 机械臂列表: %zu",
            cam_info.cam_id, cam_info.armInfoList.size());
        for (const auto& arm_info : cam_info.armInfoList)
        {
            LOG_INFO(log_project_path_, "机械臂ID: %d, 启用: %s",
                arm_info.arm_id, arm_info.is_enable ? "是" : "否");
            if (arm_info.is_enable)
            {
                arm_id_list_.push_back(arm_info.arm_id);
            }
        }
    }
    return bRet;
}

void PoseEstNode::initCalibParamHandler()
{
    try
    {
        if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
        {
            LOG_WARN(log_project_path_, "参数服务器未上线");
            return;
        }

        calib_result_ = std::make_unique<handeyecalib::CalibRes>();

        std::string param_prefix = "sys_cam_calib_list.cam_" + std::to_string(camera_id_)
            + ".arm_info.arm_" + std::to_string(arm_id_);
        LOG_INFO(log_project_path_, "标定参数前缀: %s", param_prefix.c_str());

        // Read cam_to_base_transform
        std::string cam_to_base_param = param_prefix + ".cam_to_base_transform";
        try
        {
            std::string cam_to_base_str = sys_config_client_->get_parameter<std::string>(cam_to_base_param);
            calib_result_->cam_to_base_transform = handeyecalib::stringToMat(cam_to_base_str);
            LOG_INFO(log_project_path_, "成功读取 cam_to_base_transform");
        }
        catch (const std::exception& e)
        {
            LOG_WARN(log_project_path_, "读取 cam_to_base_transform 失败: %s", e.what());
        }

        // Read base_to_cam_transform
        std::string base_to_cam_param = param_prefix + ".base_to_cam_transform";
        try
        {
            std::string base_to_cam_str = sys_config_client_->get_parameter<std::string>(base_to_cam_param);
            calib_result_->base_to_cam_transform = handeyecalib::stringToMat(base_to_cam_str);
            LOG_INFO(log_project_path_, "成功读取 base_to_cam_transform");
        }
        catch (const std::exception& e)
        {
            LOG_WARN(log_project_path_, "读取 base_to_cam_transform 失败: %s", e.what());
        }

        // Read offset_compensation
        std::string offset_param = param_prefix + ".offset_compensation";
        try
        {
            calib_result_->offset_compensation =
                sys_config_client_->get_parameter<std::vector<double>>(offset_param);
            LOG_INFO(log_project_path_, "成功读取 offset_compensation");
        }
        catch (const std::exception& e)
        {
            LOG_WARN(log_project_path_, "读取 offset_compensation 失败: %s", e.what());
        }
    }
    catch (const std::exception& e)
    {
        LOG_WARN(log_project_path_, "初始化标定参数处理器失败: %s", e.what());
    }
}

void PoseEstNode::Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat color_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;       
        cv::Mat vis_image = color_frame.clone();
        processFrame(color_frame, vis_image);
    
        // Publish result image
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", vis_image).toImageMsg();
        result_image_pub_->publish(*img_msg);
    }
    catch (const cv_bridge::Exception& e)
    {
        LOG_ERROR(log_project_path_, "彩色图像转换错误: %s", e.what());
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(log_project_path_, "处理帧错误: %s", e.what());
    }
}

void PoseEstNode::loadSegDetector()
{
    try
    {
        seg_detector_ = std::make_unique<YoloSeg>();
        seg_detector_->load_engine(seg_engine_path_);
        seg_detector_->set_thresholds(seg_conf_threshold_, seg_iou_threshold_);
        LOG_INFO(log_project_path_, "YOLO分割模型加载成功");
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(log_project_path_, "YOLO分割模型加载失败: %s", e.what());
        seg_detector_.reset();
    }
}

void PoseEstNode::releaseSegDetector()
{
    if (seg_detector_)
    {
        LOG_INFO(log_project_path_, "释放YOLO分割模型以节省显存");
        seg_detector_.reset();
    }
}

void PoseEstNode::loadScorerCore()
{
    if (scorer_core_)
    {
        return;  // 已加载
    }
    try
    {
        const int batch_size = 252;
        const int render_h = 160;
        const int render_w = 160;
        scorer_core_ = CreateTrtInferCore(scorer_engine_path_,
            {{"transf_input", {batch_size, render_h, render_w, 6}},
             {"render_input", {batch_size, render_h, render_w, 6}}},
            {{"scores", {batch_size, 1}}}, 1);
        LOG_INFO(log_project_path_, "Scorer推理核心重新加载成功");
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(log_project_path_, "Scorer推理核心加载失败: %s", e.what());
        scorer_core_.reset();
    }
}

void PoseEstNode::releaseScorerCore()
{
    if (scorer_core_)
    {
        LOG_INFO(log_project_path_, "释放Scorer推理核心以节省显存");
        scorer_core_.reset();
    }
}

void PoseEstNode::rebuildForTracking()
{
    LOG_INFO(log_project_path_, "重建推理核心以释放Register阶段的大batch execution context...");
    try
    {
        // 1. 保存scorer_core（重建foundation_pose需要传入）
        auto saved_scorer = scorer_core_;

        // 2. 先释放foundation_pose模型（内部持有refiner/scorer的shared_ptr）
        foundation_pose_.reset();

        // 3. 释放旧的refiner核心（含Register创建的大execution context ~2.4GB）
        refiner_core_.reset();

        // 4. 释放YOLO分割模型（Register已完成，Track不需要YOLO，节省~1GB）
        releaseSegDetector();

        // 5. 重新创建refiner核心（Track用，会创建小context）
        const int batch_size = 252;
        const int render_h = 160;
        const int render_w = 160;
        refiner_core_ = CreateTrtInferCore(refiner_engine_path_,
            {{"transf_input", {batch_size, render_h, render_w, 6}},
             {"render_input", {batch_size, render_h, render_w, 6}}},
            {{"trans", {batch_size, 3}}, {"rot", {batch_size, 3}}}, 1);

        // 6. 重建foundation_pose模型（传入新的refiner_core + 保留的scorer_core）
        scorer_core_ = saved_scorer;
        foundation_pose_ = CreateFoundationPoseModel(refiner_core_, scorer_core_, {mesh_loader_}, intrinsic_);

        LOG_INFO(log_project_path_, "推理核心重建完成，进入Track模式（YOLO已释放，scorer保留）");
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(log_project_path_, "重建推理核心失败: %s", e.what());
        state_ = State::IDLE;
        loadSegDetector();
        loadScorerCore();
    }
}

void PoseEstNode::processFrame(const cv::Mat& color_frame, cv::Mat& vis_image)
{
    // 防止多线程并发处理：如果正在处理则跳过本帧
    bool expected = false;
    if (!processing_.compare_exchange_strong(expected, true))
    {
        return;
    }

    // RAII scope guard: 任何退出路径（return/break/异常）都会自动重置 processing_
    struct ScopeGuard {
        std::atomic<bool>& flag;
        ~ScopeGuard() { flag.store(false); }
    } guard{processing_};

    // Check prerequisites
    if (!camera_intrinsics_initialized_ || !foundation_pose_)
    {
        return;
    }

    cv::Mat depth_for_processing;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (depth_frame_.empty())
        {
            return;
        }
        depth_for_processing = depth_frame_.clone();
    }

    // Convert depth from mm uint16 to meters float32 (FoundationPose expects CV_32FC1 in meters)
    cv::Mat depth_float;
    depth_for_processing.convertTo(depth_float, CV_32F);
    depth_float = depth_float / 1000.0f;

    // Convert color from BGR to RGB (FoundationPose expects RGB)
    cv::Mat color_rgb;
    cv::cvtColor(color_frame, color_rgb, cv::COLOR_BGR2RGB);

    bool pose_valid = false;
    Eigen::Matrix4f pose_out;

    switch (state_)
    {
    case State::IDLE:
    {
        // Run YOLO segmentation to find target object mask
        if (!seg_detector_)
        {
            // 延迟加载YOLO模型（构造函数中不加载，避免Register阶段内存碎片化）
            loadSegDetector();
            if (!seg_detector_)
            {
                return;  // 加载失败
            }
        }

        auto seg_start = std::chrono::high_resolution_clock::now();
        cv::Mat seg_input = color_frame.clone();
        std::vector<Segmentation> segs = seg_detector_->infer(seg_input);
        auto seg_end = std::chrono::high_resolution_clock::now();
        double seg_time = std::chrono::duration_cast<std::chrono::milliseconds>(seg_end - seg_start).count();
        LOG_DEBUG(log_project_path_, "YOLO分割: 检测到%zu个目标, 耗时%.1fms", segs.size(), seg_time);

        // Find target segmentation
        const Segmentation* target_seg = nullptr;
        if (!target_class_name_.empty())
        {
            // Find by class name: look up index in seg_class_names_, then match class_id
            int target_id = -1;
            for (size_t i = 0; i < seg_class_names_.size(); ++i) {
                if (seg_class_names_[i] == target_class_name_) {
                    target_id = static_cast<int>(i);
                    break;
                }
            }
            if (target_id < 0) {
                LOG_WARN(log_project_path_, "目标类别 '%s' 未在 %s 中找到",
                    target_class_name_.c_str(), seg_class_names_.empty() ? "class list" : "class names file");
            } else {
                for (const auto& seg : segs) {
                    if (seg.class_id == target_id) {
                        target_seg = &seg;
                        break;
                    }
                }
            }
        }
        else
        {
            // No specific class: use the most confident detection
            float best_score = 0.0f;
            for (const auto& seg : segs)
            {
                if (seg.conf > best_score)
                {
                    best_score = seg.conf;
                    target_seg = &seg;
                }
            }
        }

        if (!target_seg || target_seg->mask_matrix.empty())
        {
            if (!target_class_name_.empty() && !target_seg) {
                LOG_DEBUG(log_project_path_, "未检测到目标类别 '%s' (共%zu个检测结果)",
                    target_class_name_.c_str(), segs.size());
            } else if (!target_seg) {
                LOG_DEBUG(log_project_path_, "未检测到任何目标");
            }
            return;
        }

        // Log detected target with class name
        std::string detected_class = (target_seg->class_id >= 0 &&
            target_seg->class_id < static_cast<int>(seg_class_names_.size()))
            ? seg_class_names_[target_seg->class_id]
            : "unknown(id=" + std::to_string(target_seg->class_id) + ")";
        LOG_INFO(log_project_path_, "检测到目标物体: class=%s, class_id=%d, conf=%.3f",
            detected_class.c_str(), target_seg->class_id, target_seg->conf);

        // Convert mask_matrix to binary mask (CV_8UC1)
        cv::Mat mask_float(target_seg->mask_matrix);
        if (mask_float.rows == color_frame.cols * color_frame.rows)
        {
            // mask_matrix is flat, reshape to image dimensions
            mask_float = mask_float.reshape(1, color_frame.rows);
        }
        cv::Mat mask = (mask_float > 0.5f);

        // 释放YOLO模型以腾出显存供Register的scorer context使用（~1GB）
        // 必须在Register前释放，否则YOLO(~1GB) + refiner context(~2.4GB) + scorer context(~1GB) > 可用显存
        releaseSegDetector();

        // 检查GPU可用显存是否足够Register（refiner context ~2.5GB + scorer context ~1GB = ~3.5GB）
        size_t free_mem = 0, total_mem = 0;
        cudaMemGetInfo(&free_mem, &total_mem);
        size_t required_mem = 3800UL * 1024 * 1024;  // 3.8GB safety margin
        if (free_mem < required_mem)
        {
            LOG_WARN(log_project_path_, "GPU可用显存不足: %.1fGB < %.1fGB，跳过Register等待显存释放",
                free_mem / (1024.0 * 1024.0 * 1024.0), required_mem / (1024.0 * 1024.0 * 1024.0));
            return;
        }

        // Register
        LOG_INFO(log_project_path_, "开始Register...");
        state_ = State::REGISTERING;
        auto reg_start = std::chrono::high_resolution_clock::now();

        try
        {
            pose_valid = foundation_pose_->Register(color_rgb, depth_float, mask, target_name_, pose_out, 1);
        }
        catch (const std::bad_alloc& e)
        {
            LOG_ERROR(log_project_path_, "Register显存不足(GPU OOM): %s，请确认GPU显存是否足够（建议>=8GB）", e.what());
            pose_valid = false;
        }
        catch (const std::exception& e)
        {
            LOG_ERROR(log_project_path_, "Register异常: %s", e.what());
            pose_valid = false;
        }

        auto reg_end = std::chrono::high_resolution_clock::now();
        double reg_time = std::chrono::duration_cast<std::chrono::milliseconds>(reg_end - reg_start).count();

        if (pose_valid)
        {
            state_ = State::TRACKING;
            current_pose_ = pose_out;
            last_valid_pose_ = pose_out;
            track_fail_count_ = 0;
            tracking_start_time_ = std::chrono::steady_clock::now();
            LOG_INFO(log_project_path_, "Register成功, 耗时%.1fms", reg_time);
            // Register成功后重建推理核心：释放Register阶段的大batch context，切换到Track模式
            rebuildForTracking();
            // rebuildForTracking失败时state会被重置为IDLE
        }
        else
        {
            state_ = State::IDLE;
            LOG_WARN(log_project_path_, "Register失败, 耗时%.1fms", reg_time);
            // Register失败需要重新加载YOLO以再次检测
            loadSegDetector();
        }
        break;
    }

    case State::REGISTERING:
    {
        // Should not stay in this state; fall through to IDLE
        state_ = State::IDLE;
        break;
    }

    case State::TRACKING:
    {
        auto track_start = std::chrono::high_resolution_clock::now();

        try
        {
            pose_valid = foundation_pose_->Track(color_rgb, depth_float, current_pose_, target_name_, pose_out, 1);
        }
        catch (const std::bad_alloc& e)
        {
            LOG_ERROR(log_project_path_, "Track显存不足(GPU OOM): %s", e.what());
            pose_valid = false;
        }
        catch (const std::exception& e)
        {
            LOG_ERROR(log_project_path_, "Track异常: %s", e.what());
            pose_valid = false;
        }

        auto track_end = std::chrono::high_resolution_clock::now();
        double track_time = std::chrono::duration_cast<std::chrono::milliseconds>(track_end - track_start).count();

        if (pose_valid)
        {
            // Track后验校验：防止物体离开视野后仍输出无意义的pose
            if (validateTrackedPose(pose_out, depth_float, track_max_depth_error_, track_min_visible_ratio_))
            {
                current_pose_ = pose_out;
                last_valid_pose_ = pose_out;
                track_fail_count_ = 0;
                LOG_DEBUG(log_project_path_, "Track成功, 耗时%.1fms", track_time);

                // 防漂移：定时重新Register（借鉴Isaac ROS的reset_period设计）
                if (reinit_period_ms_ > 0)
                {
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - tracking_start_time_).count();
                    if (elapsed_ms > reinit_period_ms_)
                    {
                        LOG_INFO(log_project_path_, "Track已运行%lldms超过reinit_period(%dms)，重新Register防止漂移",
                            (long long)elapsed_ms, reinit_period_ms_);
                        state_ = State::IDLE;
                        loadSegDetector();
                        loadScorerCore();
                    }
                }
            }
            else
            {
                // 校验失败，不计为有效pose
                pose_valid = false;
                track_fail_count_++;
                LOG_WARN(log_project_path_, "Track校验失败(连续第%d次), 耗时%.1fms", track_fail_count_, track_time);
            }
        }
        else
        {
            // Track()返回false（内部失败），递增失败计数
            track_fail_count_++;
            LOG_WARN(log_project_path_, "Track返回失败(连续第%d次), 耗时%.1fms", track_fail_count_, track_time);
        }

        if (!pose_valid)
        {
            if (track_fail_count_ >= track_fail_reset_count_)
            {
                LOG_WARN(log_project_path_, "连续%d次Track失败, 切换到IDLE等待重新检测", track_fail_count_);
                state_ = State::IDLE;
                track_fail_count_ = 0;
                loadSegDetector();
                loadScorerCore();
            }
        }
        break;
    }
    }
    
    if (pose_valid)
    {
        // Draw visualization
        auto draw_pose = ConvertPoseMesh2BBox(pose_out, mesh_loader_);
        draw3DBoundingBox(intrinsic_, draw_pose, color_frame.rows, color_frame.cols,
            mesh_loader_->GetObjectDimension(), vis_image);

        // Add state text
        std::string state_text = (state_ == State::TRACKING) ? "TRACKING" : "REGISTERED";
        cv::putText(vis_image, state_text, cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        // Publish pose
        std_msgs::msg::Header header;
        header.stamp = this->now();
        publishPose(pose_out, header);
    }

}

bool PoseEstNode::validateTrackedPose(const Eigen::Matrix4f& pose, const cv::Mat& depth_float,
                                                float max_depth_error, float min_visible_ratio)
{
    const int img_h = depth_float.rows;
    const int img_w = depth_float.cols;
    float z3d = pose(2, 3);

    // 基本有效性检查：深度必须为正
    if (z3d <= 0.01f)
    {
        LOG_WARN(log_project_path_, "Track校验失败: 深度为非正值(%.4f)", z3d);
        return false;
    }

    // 获取物体尺寸（用于生成边界框采样点）
    Eigen::Vector3f dimension = mesh_loader_->GetObjectDimension();
    float half_x = dimension(0) * 0.5f;
    float half_y = dimension(1) * 0.5f;
    float half_z = dimension(2) * 0.5f;

    // 在物体3D边界框上生成采样点（中心 + 8个角点 + 6个面中心 = 15个点）
    // 这些点在物体坐标系下，需要用pose变换到相机坐标系
    std::vector<Eigen::Vector3f> local_points = {
        {0, 0, 0},                              // 中心
        {-half_x, -half_y, -half_z},             // 8个角点
        { half_x, -half_y, -half_z},
        {-half_x,  half_y, -half_z},
        { half_x,  half_y, -half_z},
        {-half_x, -half_y,  half_z},
        { half_x, -half_y,  half_z},
        {-half_x,  half_y,  half_z},
        { half_x,  half_y,  half_z},
        {0, 0, -half_z},                          // 6个面中心
        {0, 0,  half_z},
        {0, -half_y, 0},
        {0,  half_y, 0},
        {-half_x, 0, 0},
        { half_x, 0, 0},
    };

    Eigen::Matrix3f rotation = pose.block<3, 3>(0, 0);
    Eigen::Vector3f translation = pose.block<3, 1>(0, 3);

    int total_in_image = 0;
    int depth_consistent = 0;

    for (const auto& pt_local : local_points)
    {
        // 变换到相机坐标系
        Eigen::Vector3f pt_cam = rotation * pt_local + translation;
        float pz = pt_cam(2);

        if (pz <= 0.01f)
            continue;  // 在相机后面，跳过

        // 投影到像素坐标
        int pu = static_cast<int>(fx_ * pt_cam(0) / pz + cx_);
        int pv = static_cast<int>(fy_ * pt_cam(1) / pz + cy_);

        // 检查是否在图像范围内
        if (pu < 0 || pu >= img_w || pv < 0 || pv >= img_h)
            continue;  // 超出图像范围，不算有效也不算失败

        total_in_image++;

        // 读取深度图中的实测深度
        float measured_depth = depth_float.at<float>(pv, pu);
        if (measured_depth <= 0.01f)
            continue;  // 深度无效（传感器盲区），跳过

        // 比较3D点的深度与实测深度
        float depth_diff = std::abs(pz - measured_depth);
        if (depth_diff <= max_depth_error)
        {
            depth_consistent++;
        }
    }

    // 如果没有足够的点在图像内（物体大部分在画面外），直接失败
    if (total_in_image < 3)
    {
        LOG_WARN(log_project_path_, "Track校验失败: 仅%d个采样点在图像内(需要>=3)", total_in_image);
        return false;
    }

    // 计算深度一致的比例
    float visible_ratio = static_cast<float>(depth_consistent) / static_cast<float>(total_in_image);

    if (visible_ratio < min_visible_ratio)
    {
        LOG_WARN(log_project_path_, "Track校验失败: 深度一致率%.1f%%(%.0f/%.0f)低于阈值%.0f%%",
            visible_ratio * 100.0f, (float)depth_consistent, (float)total_in_image, min_visible_ratio * 100.0f);
        return false;
    }

    return true;
}

void PoseEstNode::publishPose(const Eigen::Matrix4f& pose_cam, const std_msgs::msg::Header& header)
{
    // Publish pose in camera frame
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = header;
    pose_msg.header.frame_id = "camera_link";
    pose_msg.pose.position.x = pose_cam(0, 3);
    pose_msg.pose.position.y = pose_cam(1, 3);
    pose_msg.pose.position.z = pose_cam(2, 3);

    Eigen::Matrix3f rot = pose_cam.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rot);
    pose_msg.pose.orientation.w = quat.w();
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();

    pose_pub_->publish(pose_msg);

    LOG_DEBUG(log_project_path_, "位姿(camera): pos=(%.4f, %.4f, %.4f), quat=(%.4f, %.4f, %.4f, %.4f)",
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
        pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y, pose_msg.pose.orientation.z);

    // Publish pose in base frame (if calibration is enabled)
    if (usecalib_ && calib_result_ && !calib_result_->cam_to_base_transform.empty())
    {
        try
        {
            // Convert Eigen::Matrix4f to cv::Mat
            cv::Mat pose_cam_cv(4, 4, CV_64F);
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    pose_cam_cv.at<double>(i, j) = static_cast<double>(pose_cam(i, j));
                }
            }

            // Transform: pose_base = cam_to_base * pose_cam
            cv::Mat pose_base_cv = calib_result_->cam_to_base_transform * pose_cam_cv;

            geometry_msgs::msg::PoseStamped pose_base_msg;
            pose_base_msg.header = header;
            pose_base_msg.header.frame_id = "base_link";
            pose_base_msg.pose.position.x = pose_base_cv.at<double>(0, 3);
            pose_base_msg.pose.position.y = pose_base_cv.at<double>(1, 3);
            pose_base_msg.pose.position.z = pose_base_cv.at<double>(2, 3);

            // Apply offset compensation
            if (!calib_result_->offset_compensation.empty() && calib_result_->offset_compensation.size() >= 6)
            {
                pose_base_msg.pose.position.x += calib_result_->offset_compensation[0];
                pose_base_msg.pose.position.y += calib_result_->offset_compensation[1];
                pose_base_msg.pose.position.z += calib_result_->offset_compensation[2];
            }

            // Extract rotation from base pose
            cv::Mat rot_base_cv = pose_base_cv(cv::Rect(0, 0, 3, 3));
            Eigen::Matrix3d rot_base_eigen;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    rot_base_eigen(i, j) = pose_base_cv.at<double>(i, j);
                }
            }
            Eigen::Quaterniond quat_base(rot_base_eigen);
            pose_base_msg.pose.orientation.w = quat_base.w();
            pose_base_msg.pose.orientation.x = quat_base.x();
            pose_base_msg.pose.orientation.y = quat_base.y();
            pose_base_msg.pose.orientation.z = quat_base.z();

            pose_base_pub_->publish(pose_base_msg);

            LOG_DEBUG(log_project_path_, "位姿(base): pos=(%.4f, %.4f, %.4f)",
                pose_base_msg.pose.position.x, pose_base_msg.pose.position.y, pose_base_msg.pose.position.z);
        }
        catch (const std::exception& e)
        {
            LOG_WARN(log_project_path_, "坐标转换失败: %s", e.what());
        }
    }
}

// ============================================================================
// test() function (unchanged - used for offline testing)
// ============================================================================

void test()
{
    std::cout << "==== PoseEst Test ====" << std::endl;
    
    // 设置模型和数据路径 - 与 simple_tests 保持一致
    std::string model_base_path = "install/pose_est/models";
    std::string test_data_path = "install/pose_est/test_data/jetson-box";
    std::string mesh_path = test_data_path + "/mesh/Texture_1.obj";
    std::string cam_k_path = test_data_path + "/cam_K.txt";
    std::string target_name = "jetson-box";
    std::string refiner_model_path;
    std::string scorer_model_path;
    
    // 检查模型文件路径 - 使用 fp16 引擎(与 simple_tests 一致)
    // 使用 fp16 引擎(与 simple_tests 一致)
    std::string refiner_engine_path = model_base_path + "/refiner_hwc_dynamic_fp16.engine";
    std::string scorer_engine_path = model_base_path + "/scorer_hwc_dynamic_fp16.engine";
    
    if (std::filesystem::exists(refiner_engine_path) && std::filesystem::exists(scorer_engine_path))
    {
        refiner_model_path = refiner_engine_path;
        scorer_model_path = scorer_engine_path;
    }
    else
    {
        std::cerr << "No valid TensorRT engine files found!" << std::endl;
            return;
    }
    
    if (!std::filesystem::exists(mesh_path))
    {
        std::cerr << "Mesh file not found: " << mesh_path << std::endl;
        return;
    }
    
    std::cout << "Loading models..." << std::endl;
    
    // 创建推理核心 - 只支持TensorRT
    std::shared_ptr<BaseInferCore> refiner_core;
    std::shared_ptr<BaseInferCore> scorer_core;
    
    // 使用TensorRT引擎 - 与 simple_tests 保持一致
    const int batch_size = 252;
    const int render_h = 160;
    const int render_w = 160;
    
    refiner_core = CreateTrtInferCore(refiner_model_path,
        {{"transf_input", {batch_size, render_h, render_w, 6}}, {"render_input", {batch_size, render_h, render_w, 6}}},
        {{"trans", {batch_size, 3}}, {"rot", {batch_size, 3}}}, 1);

    // 使用正确的 shape 配置
    scorer_core = CreateTrtInferCore(scorer_model_path,
        {{"transf_input", {batch_size, render_h, render_w, 6}}, {"render_input", {batch_size, render_h, render_w, 6}}},
        {{"scores", {batch_size, 1}}}, 1);

    // 加载相机内参
    Eigen::Matrix3f intrinsic = ReadCamK(cam_k_path);

    // 创建mesh loader
    auto mesh_loader = CreateAssimpMeshLoader(target_name, mesh_path);
    if (!mesh_loader)
    {
        std::cerr << "Failed to create mesh loader" << std::endl;
        return;
    }
    
    std::cout << "Mesh loaded: vertices=" << mesh_loader->GetMeshNumVertices()
              << ", faces=" << mesh_loader->GetMeshNumFaces()
              << ", diameter=" << mesh_loader->GetMeshDiameter() << std::endl;

    // 创建FoundationPose模型
    auto foundation_pose = CreateFoundationPoseModel(refiner_core, scorer_core, {mesh_loader}, intrinsic);
    std::cout << "FoundationPose model initialized successfully!" << std::endl;

    // 测试数据
    std::string ext = ".png";
    std::string frame_id = "1774493435636744841";
    std::string rgb_path = test_data_path + "/rgb/" + frame_id + ext;
    std::string depth_path = test_data_path + "/depth/" + frame_id + ext;
    std::string mask_path = test_data_path + "/masks/" + frame_id + ext;
    
    // 读取测试图像
    auto [rgb, depth, mask] = ReadRgbDepthMask(rgb_path, depth_path, mask_path);
    
    std::cout << "Image loaded: " << rgb.cols << "x" << rgb.rows << std::endl;
    
    // 执行注册
    std::cout << "\n==== Testing Register ====" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    
    Eigen::Matrix4f out_pose;
    bool success = foundation_pose->Register(rgb, depth, mask, target_name, out_pose, 1);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    if (success)
    {
        std::cout << "Register successful! Time: " << duration.count() << " ms" << std::endl;
        std::cout << "Pose:\n"
                  << out_pose(0, 0) << " " << out_pose(0, 1) << " " << out_pose(0, 2) << " " << out_pose(0, 3) << "\n"
                  << out_pose(1, 0) << " " << out_pose(1, 1) << " " << out_pose(1, 2) << " " << out_pose(1, 3) << "\n"
                  << out_pose(2, 0) << " " << out_pose(2, 1) << " " << out_pose(2, 2) << " " << out_pose(2, 3) << "\n"
                  << out_pose(3, 0) << " " << out_pose(3, 1) << " " << out_pose(3, 2) << " " << out_pose(3, 3) << std::endl;
        
        // 绘制结果
        cv::Mat result_image = rgb.clone();
        cv::cvtColor(result_image, result_image, cv::COLOR_RGB2BGR);
        
        auto draw_pose = ConvertPoseMesh2BBox(out_pose, mesh_loader);
        Eigen::Vector3f dimension = mesh_loader->GetObjectDimension();
        draw3DBoundingBox(intrinsic, draw_pose, rgb.rows, rgb.cols, dimension, result_image);
        
        cv::imwrite("pose_est_register_result.png", result_image);
        std::cout << "Result saved to: pose_est_register_result.png" << std::endl;
    }
    else
    {
        std::cerr << "Register failed!" << std::endl;
        return;
    }
    
    // 测试跟踪
    std::cout << "\n==== Testing Track ====" << std::endl;
    
    // 获取所有测试帧
    auto rgb_paths = get_files_in_directory(test_data_path + "/rgb/");
    std::sort(rgb_paths.begin(), rgb_paths.end());
    
    std::vector<std::string> frame_ids;
    for (const auto& p : rgb_paths)
    {
        frame_ids.push_back(p.stem().string());
    }
    
    int track_count = 0;
    int track_success = 0;
    std::vector<cv::Mat> result_sequence;
    result_sequence.push_back(cv::imread("pose_est_register_result.png"));
    
    // 测试多帧跟踪 - 限制帧数以避免显存耗尽
    int test_frames = std::min(700, static_cast<int>(frame_ids.size()));
    
    std::cout << "Testing track on " << test_frames << " frames..." << std::endl;
    
    start = std::chrono::high_resolution_clock::now();
    
    for (int i = 1; i < test_frames; ++i)
    {
        std::string cur_rgb_path = test_data_path + "/rgb/" + frame_ids[i] + ext;
        std::string cur_depth_path = test_data_path + "/depth/" + frame_ids[i] + ext;
        
        auto [cur_rgb, cur_depth] = ReadRgbDepth(cur_rgb_path, cur_depth_path);
        
        Eigen::Matrix4f track_pose;
        bool track_ok = false;
        try {
            track_ok = foundation_pose->Track(cur_rgb, cur_depth, out_pose, target_name, track_pose);
        } catch (const std::exception& e) {
            std::cerr << "Track error at frame " << i << ": " << e.what() << std::endl;
            // 继续处理下一帧
        }
        
        if (track_ok)
        {
            track_success++;
            out_pose = track_pose;
            
            cv::Mat track_image = cur_rgb.clone();
            cv::cvtColor(track_image, track_image, cv::COLOR_RGB2BGR);
            
            auto draw_pose = ConvertPoseMesh2BBox(track_pose, mesh_loader);
            draw3DBoundingBox(intrinsic, draw_pose, cur_rgb.rows, cur_rgb.cols, 
                            mesh_loader->GetObjectDimension(), track_image);
            cv::imshow("test_pose_est_result", track_image);
            cv::waitKey(20);
            result_sequence.push_back(track_image);
        }
        else
        {
            std::cerr << "Track failed at frame " << i << std::endl;
        }
        track_count++;
    }
    
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "Track success: " << track_success << "/" << track_count << std::endl;
    std::cout << "Average track time: " << static_cast<double>(duration.count()) / track_count << " ms" << std::endl;
    
    if (!result_sequence.empty())
    {
        // 保存结果视频
        std::string video_path = "pose_est_track_result.mp4";
        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        cv::VideoWriter writer(video_path, fourcc, 10, result_sequence[0].size());
        
        for (const auto& frame : result_sequence)
        {
            writer.write(frame);
        }
        writer.release();
        std::cout << "Track video saved to: " << video_path << std::endl;
    }
    
    // 性能测试
    std::cout << "\n==== Performance Test ====" << std::endl;
    
    // Register性能测试
    int perf_test_count = 10;
    start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < perf_test_count; ++i)
    {
        Eigen::Matrix4f tmp_pose;
        foundation_pose->Register(rgb.clone(), depth, mask, target_name, tmp_pose, 1);
    }
    
    end = std::chrono::high_resolution_clock::now();
    double avg_register_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() 
                               / static_cast<double>(perf_test_count);
    std::cout << "Average Register time: " << avg_register_time << " ms" << std::endl;
    
    // Track性能测试 - 减少次数避免显存耗尽
    perf_test_count = 20;
    start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < perf_test_count; ++i)
    {
        Eigen::Matrix4f tmp_pose;
        foundation_pose->Track(rgb.clone(), depth, out_pose, target_name, tmp_pose, 1);
    }
    
    end = std::chrono::high_resolution_clock::now();
    double avg_track_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() 
                            / static_cast<double>(perf_test_count);
    std::cout << "Average Track time: " << avg_track_time << " ms" << std::endl;
    
    std::cout << "\n==== Test Complete ====" << std::endl;
    
    // 显式释放 CUDA 资源
    std::cout << "Releasing CUDA resources..." << std::endl;
    foundation_pose.reset();
    refiner_core->Release();
    scorer_core->Release();
    std::cout << "CUDA resources released." << std::endl;
    _exit(0);  // 使用 _exit 避免调用全局析构函数导致的 CUDA 资源释放问题
}

// ============================================================================
// main()
// ============================================================================

int main(int argc, char** argv)
{
    // 设置glog
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;

    // 运行测试
    if (0)  // 设置为1则运行离线测试
    {
        test();
    }
    else
    {
        const std::string log_project_path = basmodule::get_project_name_by_file_path(__FILE__);

        // Build command line args with default config file
        std::vector<std::string> new_argv_strings;
        std::vector<char*> new_argv;

        for (int i = 1; i < argc; i++)
        {
            new_argv_strings.push_back(std::string(argv[i]));
        }

        std::string default_config_file_path = ament_index_cpp::get_package_share_directory("pose_est")
            + "/config/pose_est_params.yaml";
        if (argc <= 1)
            new_argv_strings.push_back("--ros-args");
        new_argv_strings.push_back("--params-file");
        new_argv_strings.push_back(default_config_file_path);
        std::cout << "使用默认配置文件: " << default_config_file_path << std::endl;

        new_argv.reserve(new_argv_strings.size() + 1);
        for (const auto& arg : new_argv_strings)
        {
            std::cout << "参数配置: " << arg << std::endl;
            new_argv.push_back(const_cast<char*>(arg.c_str()));
        }
        new_argv.push_back(nullptr);

        rclcpp::init(argc, argv);
        LOG_INFO(log_project_path, "pose_est节点启动");
        // 使用单线程executor：FoundationPose的TRT engine不支持多线程并发推理
        // （多线程会导致TRT为每个线程创建独立的execution context，触发OOM）
        // Register耗时约3秒，但processFrame内部有跳帧保护，单线程足够
        rclcpp::executors::SingleThreadedExecutor executor;

        // Create main node to determine active cameras
        auto main_node = std::make_shared<PoseEstMainNode>();
        std::vector<int> act_cam_ids = main_node->getActiveCameraIds();

        if (act_cam_ids.empty())
        {
            LOG_WARN(log_project_path, "有效配置的相机ID列表为空, 默认使用相机ID 0");
            act_cam_ids = {0};
        }

        // Create per-camera sub-nodes
        std::vector<std::shared_ptr<PoseEstNode>> nodes;
        for (int camera_id : act_cam_ids)
        {
            LOG_INFO(log_project_path, "创建 PoseEstNode, camera_id: %d", camera_id);

            rclcpp::NodeOptions node_options;
            node_options.append_parameter_override("camera_id", rclcpp::ParameterValue(camera_id));

            // Read per-camera config from YAML
            try
            {
                std::string config_file_path = ament_index_cpp::get_package_share_directory("pose_est")
                    + "/config/pose_est_params.yaml";
                YAML::Node config = YAML::LoadFile(config_file_path);

                std::string cam_param_key = "cam_" + std::to_string(camera_id) + "_parameters";
                if (config[cam_param_key])
                {
                    YAML::Node cam_config = config[cam_param_key];
                    LOG_INFO(log_project_path, "读取相机%d的独立配置", camera_id);

                    if (cam_config["refiner_engine_path"])
                        node_options.append_parameter_override("refiner_engine_path",
                            rclcpp::ParameterValue(cam_config["refiner_engine_path"].as<std::string>()));
                    if (cam_config["scorer_engine_path"])
                        node_options.append_parameter_override("scorer_engine_path",
                            rclcpp::ParameterValue(cam_config["scorer_engine_path"].as<std::string>()));
                    if (cam_config["mesh_path"])
                        node_options.append_parameter_override("mesh_path",
                            rclcpp::ParameterValue(cam_config["mesh_path"].as<std::string>()));
                    if (cam_config["target_name"])
                        node_options.append_parameter_override("target_name",
                            rclcpp::ParameterValue(cam_config["target_name"].as<std::string>()));
                    if (cam_config["seg_engine_path"])
                        node_options.append_parameter_override("seg_engine_path",
                            rclcpp::ParameterValue(cam_config["seg_engine_path"].as<std::string>()));
                    if (cam_config["target_class_name"])
                        node_options.append_parameter_override("target_class_name",
                            rclcpp::ParameterValue(cam_config["target_class_name"].as<std::string>()));
                    if (cam_config["seg_conf_threshold"])
                        node_options.append_parameter_override("seg_conf_threshold",
                            rclcpp::ParameterValue(cam_config["seg_conf_threshold"].as<double>()));
                    if (cam_config["seg_iou_threshold"])
                        node_options.append_parameter_override("seg_iou_threshold",
                            rclcpp::ParameterValue(cam_config["seg_iou_threshold"].as<double>()));
                    if (cam_config["usecalib"])
                        node_options.append_parameter_override("usecalib",
                            rclcpp::ParameterValue(cam_config["usecalib"].as<bool>()));
                    if (cam_config["camera_type"])
                        node_options.append_parameter_override("camera_type",
                            rclcpp::ParameterValue(cam_config["camera_type"].as<std::string>()));
                    if (cam_config["track_max_depth_error"])
                        node_options.append_parameter_override("track_max_depth_error",
                            rclcpp::ParameterValue(cam_config["track_max_depth_error"].as<double>()));
                    if (cam_config["track_min_visible_ratio"])
                        node_options.append_parameter_override("track_min_visible_ratio",
                            rclcpp::ParameterValue(cam_config["track_min_visible_ratio"].as<double>()));
                    if (cam_config["track_fail_reset_count"])
                        node_options.append_parameter_override("track_fail_reset_count",
                            rclcpp::ParameterValue(cam_config["track_fail_reset_count"].as<int>()));
                    if (cam_config["reinit_period_ms"])
                        node_options.append_parameter_override("reinit_period_ms",
                            rclcpp::ParameterValue(cam_config["reinit_period_ms"].as<int>()));
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

            auto node = std::make_shared<PoseEstNode>(node_options, camera_id);
            nodes.push_back(node);
            executor.add_node(node);
            std::cout << "创建 PoseEstNode, camera_id: " << camera_id << std::endl;
        }

        executor.spin();
        rclcpp::shutdown();

        // 使用_exit避免CUDA/TensorRT资源在全局析构时产生double free
        _exit(0);
    }

    google::ShutdownGoogleLogging();
    return 0;
}
