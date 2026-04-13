#include "cam_sdk_aruco_test/cam_sdk_aruco_test.hpp"
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <sstream>
#include <vector>
#include <thread>
#include <chrono>
//#include "log_system/log_macros.hpp"

// 包含相关头文件
#include "visualization/visualization_mgr.hpp"
#include "aruco_alg/aruco_detector.hpp"
#include "comm_alg/marker_structs.hpp"
#include "cam_manage/cam_manage.hpp"
#include "cam_manage/cam_com_struct.hpp"

// ROS环境下的包目录查找
#ifdef __ROS_ENV__
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace fs = std::filesystem;

/**
 * @brief 四元数转欧拉角 (XYZ顺序)
*/
std::vector<double> quaternionToEulerXYZ(double qw, double qx, double qy, double qz) 
{
    double norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    double w = qw / norm;
    double x = qx / norm;
    double y = qy / norm;
    double z = qz / norm;
    double roll, pitch, yaw;
        
    // XYZ顺序
    double sinp = -2.0 * (y * z - w * x);
    if (std::abs(sinp) >= 1.0) 
    {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } 
    else 
    {
        pitch = std::asin(sinp);
    }
        
    // 检查是否处于万向锁
    if (std::abs(sinp) > 0.9999) 
    {
        roll = 0.0;
        yaw = 2.0 * std::atan2(x, w);
    } 
    else 
    {
        roll = std::atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y));
        yaw = std::atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z));
    }
        
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;
        
    return {roll, pitch, yaw};
}

/**
 * @brief 四元数转欧拉角 (ZYX顺序 - 更常用)
 * @return [roll, pitch, yaw] 但旋转顺序不同
 * 
 * ZYX顺序（外旋）：
 * 1. 先绕Z轴旋转 (yaw)
 * 2. 再绕Y轴旋转 (pitch)
 * 3. 最后绕X轴旋转 (roll)
 */
std::vector<double> quaternionToEulerZYX(double qw, double qx, double qy, double qz) 
{
    double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    double w = qw / norm;
    double x = qx / norm;
    double y = qy / norm;
    double z = qz / norm;
    
    double roll, pitch, yaw;
    
    // ZYX顺序
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }
    
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
    
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;
    
    return {roll, pitch, yaw};
}

CamSdkArucoTest::CamSdkArucoTest()
    : rclcpp::Node("cam_sdk_aruco_test"),
      camera_manager_(nullptr),
      camera_id_(-1),
      current_image_index_(0),
      is_running_(false),
      reservice_thread_running_(false) {  // 初始化新添加的成员变量
#ifdef __ROS_ENV__
    // 创建发布器和订阅器
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("cam_0/color/image_raw", 10);
    
    image_topic_ = "/camera/camera/color/image_rect_raw";
    
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, 10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            this->imageCallback(msg);
        });
        
    result_sub_ = this->create_subscription<vision_msgs::msg::Detection2D>(
        "cam_0/aruco_detection/results", 10,
        [this](const vision_msgs::msg::Detection2D::SharedPtr msg) {
            this->resultCallback(msg);
        });
        
    // 订阅标定结果话题
    calib_result_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/aruco_detection/calib_result", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->calibResultCallback(msg);
        });
        
    aruco_detection_client_ = this->create_client<custom_msgs_comm::srv::GetMarkerDetection>("cam_0/aruco_detection/results");  // 修改为获取Aruco检测结果的服务
    aruco_makers_info_client_ = this->create_client<custom_msgs_comm::srv::GetMarkersInfo>("/aruco_detection/get_makers_info");  // 添加获取Aruco标记信息服务的客户端
#endif
    is_result_sub = false; ///< 是否订阅
    is_result_service_ = false; ///< 是否提供服务
    base_save_dir_ = "save_parms";
    // 设置保存目录路径
    coords_save_dir_ = base_save_dir_ + "/coordinates";
    images_save_dir_ = base_save_dir_ + "/images";
    aruco_images_save_dir_ = base_save_dir_ + "/aruco_rendered";
    current_file_index_ = 0; ///< 当前文件索引
}

CamSdkArucoTest::~CamSdkArucoTest() {
    shutdown();
}

bool CamSdkArucoTest::initialize(const std::string& config_file) 
{
	// 获取正确的配置文件路径
    std::string actual_config_file = getConfigFilePath(config_file);
    
    std::cout << "Trying to load config file: " << actual_config_file << std::endl;
    
    // 检查文件是否存在
    if (!std::filesystem::exists(actual_config_file)) {
        std::cerr << "Config file does not exist: " << actual_config_file << std::endl;
        return false;
    }
    
    // 加载配置文件
    if (!loadConfig(actual_config_file)) {
        std::cerr << "Failed to load configuration file: " << actual_config_file << std::endl;
        return false;
    }
    
    // 将config_file路径中的"aruco_show_config.yaml"替换为"camera_intrinsics.json"
    std::string Intrinsics_config_file = actual_config_file; // 相机内参配置文件路径
    size_t pos = Intrinsics_config_file.find("aruco_show_config.yaml");
    if (pos != std::string::npos) {
        Intrinsics_config_file.replace(pos, strlen("aruco_show_config.yaml"), "camera_intrinsics.json");
    	initCameraIntrinsics(Intrinsics_config_file); // 初始化相机内参
        std::cout << "Camera Matrix: " << camera_matrix_ << std::endl;
    }

    std::cout << "Successfully loaded config file: " << actual_config_file << std::endl;
    
    // 打印加载的主配置信息
    std::cout << "Main configuration loaded:" << std::endl;
    std::cout << "  Work mode: " << work_mode_ << std::endl;
    if (work_mode_ == "camera") {
        std::cout << "  Camera config file: " << camera_config_file_ << std::endl;
    } else if (work_mode_ == "file") {
        std::cout << "  Image folder: " << image_folder_ << std::endl;
        std::cout << "  Image extensions: ";
        for (const auto& ext : image_extensions_) {
            std::cout << ext << " ";
        }
        std::cout << std::endl;
    }

    // 初始化Aruco检测器
    aruco_detector_ = std::make_unique<aruco_alg::ArucoDetector>(std::vector<double>{0.1}, std::vector<int>{5});
    aruco_detector_->setCameraIntrinsics(camera_matrix_, dist_coeffs_);

    // 初始化可视化管理器
    visualizer_ = std::make_unique<visualization::VisualizationMgr>();
    if (!visualizer_->initialize("Aruco Detection Test")) {
        std::cerr << "Failed to initialize visualization manager" << std::endl;
        return false;
    }
    
    // 根据工作模式初始化
    if (work_mode_ == "camera") {
        return initCameraMode();
    } else if (work_mode_ == "file") {
        return initFileMode();
    } else {
        std::cerr << "Invalid work mode: " << work_mode_ << std::endl;
        return false;
    }
}

bool CamSdkArucoTest::run() {
    is_running_ = true;
    reservice_thread_running_ = true;  // 启动reservice线程
    
    // 启动reservice线程
    reservice_thread_ = std::thread([this]() {
        this->reserviceThread();
    });
    
    // 将run函数的执行放在线程中
    worker_thread_ = std::thread([this]() {
        if (work_mode_ == "camera") {
            processCameraMode();
        } else if (work_mode_ == "file") {
            processFileMode();
        }
    });
    
    return true;
}

void CamSdkArucoTest::shutdown() {
    is_running_ = false;
    reservice_thread_running_ = false;  // 停止reservice线程
    
    // 等待reservice线程结束
    if (reservice_thread_.joinable()) {
        reservice_thread_.join();
    }
    
    // 等待工作线程结束
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    
    if (visualizer_) {
        visualizer_->closeWindows();
    }
    
    if (camera_manager_ && camera_id_ >= 0) {
        // 这里可以添加相机关闭逻辑
    }
}

std::string CamSdkArucoTest::getConfigFilePath(const std::string& default_config_path) {
    // 首先检查环境变量，这在ROS环境中会指向install目录
    const char* ament_prefix_path = std::getenv("AMENT_PREFIX_PATH");
    if (ament_prefix_path) {
        std::cout << "AMENT_PREFIX_PATH: " << ament_prefix_path << std::endl;
        
        // 分割路径字符串
        std::stringstream ss(ament_prefix_path);
        std::string path;
        std::vector<std::string> paths;
        
        while (std::getline(ss, path, ':')) {
            paths.push_back(path);
        }
        
        // 遍历每个路径，查找配置文件
        for (const auto& base_path : paths) {
            // 在ROS环境中，配置文件应该在install/share目录下
            std::filesystem::path ros_config_path = 
                std::filesystem::path(base_path) / "share" / "cam_sdk_aruco_test" / "config" / "aruco_show_config.yaml";
            
            std::cout << "Checking ROS config path: " << ros_config_path << std::endl;
            
            if (std::filesystem::exists(ros_config_path)) {
                std::cout << "Found config file in ROS install directory" << std::endl;
                return ros_config_path.string();
            }
        }
        
        std::cout << "Config file not found in any ROS install directory" << std::endl;
    } else {
        std::cout << "AMENT_PREFIX_PATH not set" << std::endl;
    }
    
    // 如果在ROS环境中没找到，或者不在ROS环境中，使用默认路径
    std::cout << "Using default config path: " << default_config_path << std::endl;
    return default_config_path;
}

bool CamSdkArucoTest::loadConfig(const std::string& config_file) {
    try {
        YAML::Node config = YAML::LoadFile(config_file);
        
        std::cout << "Parsing configuration file: " << config_file << std::endl;
        
        // 检查必要的配置节
        if (!config["work_mode"]) {
            std::cerr << "Configuration file missing 'work_mode' field" << std::endl;
            return false;
        }
        
        // 获取工作模式
        work_mode_ = config["work_mode"].as<std::string>();
        std::cout << "Work mode: " << work_mode_ << std::endl;
        
        if (work_mode_ == "camera") {
            // 检查相机配置节
            if (!config["camera"]) {
                std::cerr << "Configuration file missing 'camera' section for camera mode" << std::endl;
                return false;
            }
            
            if (!config["camera"]["config_file"]) {
                std::cerr << "Configuration file missing 'camera.config_file' field" << std::endl;
                return false;
            }
            
            // 获取相机配置
            camera_config_file_ = config["camera"]["config_file"].as<std::string>();
            std::cout << "Camera config file path: " << camera_config_file_ << std::endl;
        } else if (work_mode_ == "file") {
            // 检查文件配置节
            if (!config["file"]) {
                std::cerr << "Configuration file missing 'file' section for file mode" << std::endl;
                return false;
            }
            
            if (!config["file"]["image_folder"]) {
                std::cerr << "Configuration file missing 'file.image_folder' field" << std::endl;
                return false;
            }
            
            if (!config["file"]["image_extensions"]) {
                std::cerr << "Configuration file missing 'file.image_extensions' field" << std::endl;
                return false;
            }
            
            // 获取文件配置
            image_folder_ = config["file"]["image_folder"].as<std::string>();
            image_extensions_ = config["file"]["image_extensions"].as<std::vector<std::string>>();
            std::cout << "Image folder: " << image_folder_ << std::endl;
            std::cout << "Image extensions: ";
            for (const auto& ext : image_extensions_) {
                std::cout << ext << " ";
            }
            std::cout << std::endl;
        } else {
            std::cerr << "Invalid work mode: " << work_mode_ << ". Supported modes are 'camera' and 'file'." << std::endl;
            return false;
        }
        
        std::cout << "Configuration loaded successfully" << std::endl;
        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error in config file: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config file: " << e.what() << std::endl;
        return false;
    }
}

bool CamSdkArucoTest::initCameraIntrinsics(const std::string& Intrinsics_config_file) {
    try {
        // 从配置文件中指定的路径加载相机内参
        std::string intrinsics_json = Intrinsics_config_file;
        
        if (!fs::exists(intrinsics_json)) {
            std::cerr << "相机内参文件不存在: " << intrinsics_json << std::endl;
            std::cerr << "使用默认相机内参" << std::endl;
            // 使用默认的相机内参
            camera_matrix_ = (cv::Mat_<double>(3, 3) << 
                651.56, 0, 639.46,
                0, 650.79, 365.41,
                0, 0, 1);
            dist_coeffs_ = (cv::Mat_<double>(1, 5) << 
                -0.0547, 0.0623, -0.000624, 0.0000543, -0.0212);
            return true;
        }
        
        // 读取JSON文件
        std::ifstream file(intrinsics_json);
        if (!file.is_open()) {
            std::cerr << "无法打开相机内参文件: " << intrinsics_json << std::endl;
            return false;
        }
        
        // 解析JSON
        std::stringstream buffer;
        buffer << file.rdbuf();
        file.close();
        
        // 简单的JSON解析（提取fx, fy, ppx, ppy和畸变系数）
        std::string json_str = buffer.str();
        
        // 提取fx, fy, ppx, ppy
        double fx, fy, ppx, ppy;
        size_t pos;
        
        pos = json_str.find("\"fx\":");
        if (pos != std::string::npos) {
            sscanf(json_str.c_str() + pos + 5, "%lf", &fx);
        }
        
        pos = json_str.find("\"fy\":");
        if (pos != std::string::npos) {
            sscanf(json_str.c_str() + pos + 5, "%lf", &fy);
        }
        
        pos = json_str.find("\"ppx\":");
        if (pos != std::string::npos) {
            sscanf(json_str.c_str() + pos + 6, "%lf", &ppx);
        }
        
        pos = json_str.find("\"ppy\":");
        if (pos != std::string::npos) {
            sscanf(json_str.c_str() + pos + 6, "%lf", &ppy);
        }
        
        // 构造相机内参矩阵
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 
            fx, 0, ppx,
            0, fy, ppy,
            0, 0, 1);
        
        // 提取畸变系数
        pos = json_str.find("\"coefficients\":");
        if (pos != std::string::npos) {
            double k1, k2, p1, p2, k3;
            size_t start = json_str.find("[", pos);
            if (start != std::string::npos) {
                sscanf(json_str.c_str() + start + 1, "%lf, %lf, %lf, %lf, %lf", 
                       &k1, &k2, &p1, &p2, &k3);
                dist_coeffs_ = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
            }
        } else {
            // 如果没有畸变系数，使用零
            dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
        }
        
        std::cout << "✅ 从配置文件加载相机内参: " << intrinsics_json << std::endl;
        std::cout << "   fx=" << fx << ", fy=" << fy << ", cx=" << ppx << ", cy=" << ppy << std::endl;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load camera intrinsics: " << e.what() << std::endl;
        return false;
    }
}

bool CamSdkArucoTest::initCameraMode() {
    try {
        // 获取相机管理器实例
        camera_manager_ = &CameraManager::get_instance();
        
        // 检查相机配置文件路径
        std::string actual_camera_config = camera_config_file_;
        
        // 如果是相对路径，尝试在不同位置查找
        if (camera_config_file_.substr(0, 2) == "..") {
            // 首先检查环境变量，这在ROS环境中会指向install目录
            const char* ament_prefix_path = std::getenv("AMENT_PREFIX_PATH");
            if (ament_prefix_path) {
#ifdef __ROS_ENV__
                // 在ROS环境中，尝试多种方式查找相机配置文件
                // 方法1: 在install目录下的sys_config中查找
                std::filesystem::path install_config_path = 
                    std::filesystem::path(ament_prefix_path) / ".." / ".." / "install" / "sys_config" / "cam_config.yaml";
                if (std::filesystem::exists(install_config_path)) {
                    actual_camera_config = install_config_path.string();
                    std::cout << "Found camera config file in install directory: " << actual_camera_config << std::endl;
                } else {
                    // 方法2: 在源码目录下的sys_config中查找
                    std::filesystem::path src_config_path = 
                        std::filesystem::path(ament_prefix_path) / ".." / ".." / "src" / "sys_config" / "cam_config.yaml";
                    if (std::filesystem::exists(src_config_path)) {
                        actual_camera_config = src_config_path.string();
                        std::cout << "Found camera config file in src directory: " << actual_camera_config << std::endl;
                    } else {
                        // 方法3: 使用ament_index_cpp获取包共享目录
                        std::string package_share_dir = ament_index_cpp::get_package_share_directory("cam_sdk_aruco_test");
                        std::string ros_config_path = package_share_dir + "/../../../sys_config/cam_config.yaml";
                        
                        if (std::filesystem::exists(ros_config_path)) {
                            actual_camera_config = ros_config_path;
                            std::cout << "Found camera config file in ROS package directory: " << ros_config_path << std::endl;
                        } else {
                            std::cout << "Camera config file not found in ROS package directory: " << ros_config_path << std::endl;
                        }
                    }
                }
#else
                // 非ROS环境下也尝试查找sys_config目录
                std::filesystem::path local_config_path = std::filesystem::current_path() / ".." / ".." / "src" / "sys_config" / "cam_config.yaml";
                if (std::filesystem::exists(local_config_path)) {
                    actual_camera_config = local_config_path.string();
                    std::cout << "Found camera config file in local src directory: " << actual_camera_config << std::endl;
                }
#endif
            } else {
                // 非ROS环境下也尝试查找sys_config目录
                std::filesystem::path local_config_path = std::filesystem::current_path() / ".." / ".." / "src" / "sys_config" / "cam_config.yaml";
                if (std::filesystem::exists(local_config_path)) {
                    actual_camera_config = local_config_path.string();
                    std::cout << "Found camera config file in local src directory: " << actual_camera_config << std::endl;
                }
            }
        }
        
        std::cout << "Trying to load camera config file: " << actual_camera_config << std::endl;
        
        // 检查文件是否存在
        if (!std::filesystem::exists(actual_camera_config)) {
            std::cerr << "Camera config file does not exist: " << actual_camera_config << std::endl;
            // 如果配置文件不存在，尝试使用默认路径
            std::filesystem::path default_config_path = std::filesystem::current_path() / "sys_config" / "cam_config.yaml";
            if (std::filesystem::exists(default_config_path)) {
                actual_camera_config = default_config_path.string();
                std::cout << "Using default camera config file: " << actual_camera_config << std::endl;
            } else {
                return false;
            }
        }
        
        // 加载相机配置文件
        YAML::Node cam_config;
        try {
            cam_config = YAML::LoadFile(actual_camera_config);
            std::cout << "Successfully loaded camera config file: " << actual_camera_config << std::endl;
        } catch (const YAML::Exception& e) {
            std::cerr << "Failed to load camera config file: " << actual_camera_config << std::endl;
            std::cerr << "YAML error: " << e.what() << std::endl;
            return false;
        }
        
        // 检查配置文件是否包含必要的字段
        if (!cam_config["default_camera"]) {
            std::cerr << "Camera config file missing 'default_camera' section" << std::endl;
            return false;
        }
        
        // 获取默认相机配置
        std::string camera_type = cam_config["default_camera"]["camera_type"].as<std::string>();
        std::string device_id = cam_config["default_camera"]["device_id"].as<std::string>();
        std::string resolution = cam_config["default_camera"]["resolution"].as<std::string>();
        int fps = cam_config["default_camera"]["fps"].as<int>();
        
        // 解析分辨率
        int width, height;
        sscanf(resolution.c_str(), "%dx%d", &width, &height);
        
        // 创建相机配置
        CamConfigInfo1D cam_configs;
        CamConfigInfo cam_config_info;
        cam_config_info.cam_id = 0;
        cam_config_info.cam_usr_name = "aruco_test_camera";
        cam_config_info.cam_index = 0;  // 添加缺失的cam_index字段
        cam_config_info.serial_number = device_id;  // 设置序列号
        
        // 设置相机类型
        if (camera_type == "realsense") {
            cam_config_info.cam_type = CamType::CAM_TYPE_RS;
            cam_config_info.cam_model = "D435";  // 默认型号
        } else if (camera_type == "orbbec") {
            cam_config_info.cam_type = CamType::CAM_TYPE_OB;
            cam_config_info.cam_model = "Gemini 335";  // 默认型号
        } else {
            cam_config_info.cam_type = CamType::CAM_TYPE_NONE;
        }
        
        // 设置流参数
        cam_config_info.enable_color_stream = true;
        cam_config_info.enable_depth_stream = false;
        cam_config_info.enable_ir_stream = false;
        cam_config_info.enable_cloud_stream = false;
        
        cam_config_info.color_para.width = width;
        cam_config_info.color_para.height = height;
        cam_config_info.color_para.fps = fps;
        cam_config_info.color_para.auto_exposure = true;
        cam_config_info.color_para.auto_white_balance = true;
        
        cam_config_info.depth_para.width = width;
        cam_config_info.depth_para.height = height;
        cam_config_info.depth_para.fps = fps;
        cam_config_info.depth_para.auto_exposure = true;

        // 打印加载的相机配置信息
        std::cout << "Camera configuration loaded:" << std::endl;
        std::cout << "  Camera type: " << camera_type << std::endl;
        std::cout << "  Device ID: " << device_id << std::endl;
        std::cout << "  Resolution: " << resolution << " (" << width << "x" << height << ")" << std::endl;
        std::cout << "  FPS: " << fps << std::endl;
        
        cam_configs.push_back(cam_config_info);
        
        // 初始化相机
        RtnType result = camera_manager_->init_all_camera(cam_configs);
        if (result != RtnType::RTN_SUCCESS) {
            std::cerr << "Failed to initialize camera" << std::endl;
            return false;
        }
        
        camera_id_ = 0;
        return true;
    } catch (const YAML::BadFile& e) {
        std::cerr << "Failed to initialize camera mode: bad file: " << camera_config_file_ << std::endl;
        std::cerr << "YAML error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize camera mode: " << e.what() << std::endl;
        return false;
    }
}

bool CamSdkArucoTest::initFileMode() {
    try {
        // 获取图片文件列表
        image_files_ = getImageFiles(image_folder_, image_extensions_);
        
        if (image_files_.empty()) {
            std::cerr << "No image files found in folder: " << image_folder_ << std::endl;
            return false;
        }
        
        std::sort(image_files_.begin(), image_files_.end());
        current_image_index_ = 0;
        
        //cleanupPreviousData();
        createDirectories();

        std::cout << "Found " << image_files_.size() << " image files" << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize file mode: " << e.what() << std::endl;
        return false;
    }
}

bool CamSdkArucoTest::processCameraMode() {
    if (!camera_manager_ || camera_id_ < 0) {
        std::cerr << "Camera not initialized" << std::endl;
        return false;
    }
    
    while (is_running_) {
        // 检查窗口是否打开（如果有可视化管理器）
        bool window_open = visualizer_ ? visualizer_->isWindowOpen() : true;
        if (!window_open) {
            std::cerr << "Visualization window is closed, exiting..." << std::endl;
            is_running_ = false;
            break;
        }
        
        // 获取一帧图像
        cv::Mat* img = nullptr;
        RtnType result = camera_manager_->get_one_frame_color(camera_id_, img);
        
        if (result == RtnType::RTN_SUCCESS && img) {
            // 处理Aruco检测
            processArucoDetection(*img);
            
            // 显示图像（如果有可视化管理器）
            // if (visualizer_) {
            //     visualizer_->showImage(*img);
            // }
        }
        
        // 处理键盘输入（如果有可视化管理器）
        if (visualizer_) {
            int key = cv::waitKey(1) & 0xFF;
            if (key == 27) {  // ESC键退出
                is_running_ = false;
                break;
            }
        } else {
            // 无GUI模式下短暂延迟避免CPU占用过高
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
        }
    }
    
    return true;
}

bool CamSdkArucoTest::processFileMode() {
    if (image_files_.empty()) {
        std::cerr << "No image files to process" << std::endl;
        return false;
    }
    
    while (is_running_) {
        // 检查窗口是否打开（如果有可视化管理器）
        bool window_open = visualizer_ ? visualizer_->isWindowOpen() : true;
        if (!window_open && current_image_index_ < image_files_.size()) {
            std::cerr << "Visualization window is closed, exiting..." << std::endl;
            is_running_ = false;
            break;
        }
        
        if (current_image_index_ >= image_files_.size()) {
            std::cout << "Finished processing all images" << std::endl;
            is_running_ = false;
            break;
        }
        
        // 加载当前图片
        cv::Mat image = cv::imread(image_files_[current_image_index_]);
        bool is_image_empty = image.empty();

        //std::string prefix = "calib_point_";
        std::string prefix = "data_point_";
        std::string suffix = ".png";
        std::string filepath = image_files_[current_image_index_];
        size_t pos1 = filepath.find(prefix);
        if (pos1 == std::string::npos) is_image_empty = true;
        
        size_t pos2 = filepath.find(suffix, pos1 + prefix.length());
        if (pos2 == std::string::npos) is_image_empty = true;
        
        std::string num_str = filepath.substr(
            pos1 + prefix.length(), 
            pos2 - (pos1 + prefix.length())
        );

        current_file_index_ = std::stoi(num_str);

        if (is_image_empty) {
            std::cerr << "Failed to load image: " << image_files_[current_image_index_] << std::endl;
            current_image_index_++;
            continue;
        }
        
        is_result_sub = false;
        // 处理Aruco检测
        processArucoDetection(image);
        
        // 显示图像（如果有可视化管理器）
        if (visualizer_) {
            visualizer_->showImage(image);
        }
        
        // 处理键盘输入（如果有可视化管理器）
        if (0 && visualizer_) {
            int key = cv::waitKey(0) & 0xFF;
            switch (key) {
                case 27:  // ESC键退出
                    is_running_ = false;
                    break;
                case 81:  // ←键
                case 82:  // ↑键
                    if (current_image_index_ > 0) {
                        current_image_index_--;
                    }
                    break;
                case 83:  // →键
                case 84:  // ↓键
                    if (current_image_index_ < image_files_.size() - 1) {
                        current_image_index_++;
                    }
                    break;
                default:
                    current_image_index_++;
                    break;
            }
        } else {
            // 无GUI模式下自动处理下一张图片
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 1秒间隔
            // while (!is_result_sub)
            // {
            //     std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 1秒间隔
            // }
        }
        if (is_result_service_)
        {
            is_result_service_ = false;
            current_image_index_++;
            std::cerr << "服务结束，下一张图片Index: " << current_image_index_ << std::endl;
        }
        //current_image_index_++;
    }
    
    return true;
}

void CamSdkArucoTest::processArucoDetection(const cv::Mat& image) {
    if (!aruco_detector_) {
        return;
    }
    
#ifdef __ROS_ENV__
    // 发布图像到ROS话题
    if (image_pub_) {
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        image_msg->header.stamp = this->now();
        image_msg->header.frame_id = "camera_frame_" + std::to_string(current_file_index_);
        image_pub_->publish(*image_msg);
        std::cout << "Published image to /aruco_detection/image : " << image_msg->header.frame_id << std::endl;
    }
#endif
    
    // // 执行Aruco检测
    // std::cerr << "执行Aruco检测" << std::endl;
    // auto result = aruco_detector_->detectAndProcessMarkers(image, nullptr, true, true);
    
    // // 可以在这里添加额外的处理逻辑
    // std::cerr << "执行Aruco数据保存" << std::endl;
    // saveDataPoint(current_file_index_ + 1, result.markers_info[0], image, result.processed_frame);
    // std::cerr << "执行Aruco数据保存结束" << std::endl;
}

void CamSdkArucoTest::reserviceThread() {
    // 持续调用reservice函数的线程
    bool useMakersInfoService = false;  // 标记使用哪个服务
    while (reservice_thread_running_) {
        if (is_result_sub)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 100毫秒间隔
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 100毫秒间隔
        // 交替调用不同的服务
        if (useMakersInfoService) {
            // 调用Aruco标记信息服务
            requestArucoMakersInfo();
        } else {
            // 调用原来的Aruco检测服务
            reservice();
        }
        
        // 切换服务标记
        //useMakersInfoService = !useMakersInfoService;
        
        // 等待一段时间再继续，避免过于频繁的调用
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 100毫秒间隔
    }
}

void CamSdkArucoTest::reservice()
{
    auto service_request = std::make_shared<custom_msgs_comm::srv::GetMarkerDetection::Request>();
    service_request->request_id = "calib_point_" + std::to_string(current_file_index_);
    
    //LOG_INFO(PROJECT_NAME, "发送Aruco检测服务请求，请求ID: %s", service_request->request_id.c_str());
    // 调用Aruco检测服务
    aruco_detection_client_->async_send_request(
        service_request,
        [this](rclcpp::Client<custom_msgs_comm::srv::GetMarkerDetection>::SharedFuture future) {
            try 
            {
                //LOG_INFO(PROJECT_NAME, "收到Aruco检测服务响应");
                auto response = future.get();
                //LOG_INFO(PROJECT_NAME, "服务响应内容 - success: %s, message: %s", 
                //             response->success ? "true" : "false", response->message.c_str());
                if (response->success) 
                {
                    //LOG_INFO(PROJECT_NAME, "成功获取Aruco标记检测结果");
                    
                    // 更新当前Aruco标记数据
                    if (response->position.size() >= 3) {
                        comm_alg::MarkerInfo marker_info;
                        marker_info.position.x = response->position[0];
                        marker_info.position.y = response->position[1];
                        marker_info.position.z = response->position[2];
                        std::vector<double> rotation = quaternionToEulerZYX(response->orientation[3], response->orientation[0], response->orientation[1], response->orientation[2]);
                        marker_info.rotation[0] = rotation[0];
                        marker_info.rotation[1] = rotation[1];
                        marker_info.rotation[2] = rotation[2]; // Fixed index access
                        // LOG_INFO(PROJECT_NAME, "Aruco标记位置: x=%.3f, y=%.3f, z=%.3f", current_aruco_marker_.x, current_aruco_marker_.y, current_aruco_marker_.z);
                        
                        std::cerr << "转换图像数据" << std::endl;
                        cv::Mat srcImg;
                        try 
                        {
                            std::cout << "1. 图像参数:" << std::endl;
                            std::cout << "   编码: '" << response->image.encoding << "'" << std::endl;
                            std::cout << "   宽度: " << response->image.width << std::endl;
                            std::cout << "   高度: " << response->image.height << std::endl;
                            std::cout << "   步长: " << response->image.step << " 字节" << std::endl;
                            std::cout << "   数据大小: " << response->image.data.size() << " 字节" << std::endl;
                            std::cout << "   是否为大数据: " << (response->image.is_bigendian ? "是" : "否") << std::endl;
                            
                            // 检查图像数据是否有效
                            if (response->image.data.size() == 0) {
                                std::cerr << "警告: 图像数据为空" << std::endl;
                            } else {
                                // 将ROS图像消息转换为OpenCV图像
                                std::string encoding = response->image.encoding.empty() ? 
                                    sensor_msgs::image_encodings::BGR8 : 
                                    response->image.encoding;
                                
                                // 确保使用有效的编码格式
                                if (encoding.empty()) {
                                    encoding = sensor_msgs::image_encodings::BGR8;
                                    std::cerr << "警告: 图像编码为空，使用默认编码: " << encoding << std::endl;
                                }
                                
                                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(response->image, encoding);
                                srcImg = cv_ptr->image.clone();
                            }
                        } 
                        catch (const cv_bridge::Exception& e) 
                        {
                            std::cerr << "图像转换失败: " << e.what() << std::endl;
                            // 即使图像转换失败，也要继续保存其他数据
                        } 
                        catch (const std::exception& e) 
                        {
                            std::cerr << "图像转换失败2: " << e.what() << std::endl;
                            // 即使图像转换失败，也要继续保存其他数据
                        }
                        
                        // 保存标定数据点
                        std::cerr << "保存标定数据点" << std::endl;
                        saveDataPoint(current_file_index_, marker_info, srcImg, cv::Mat());
                    } else {
                        // LOG_ERROR(PROJECT_NAME, 
                        //              "Aruco检测结果数据不完整，位置数据大小: %d", static_cast<int>(response->position.size()));
                    }
                } else {
                    //LOG_ERROR(PROJECT_NAME, "获取Aruco标记检测结果失败: %s", response->message.c_str());
                }
                
                // 无论成功与否，都移动到下一个标定点
                
            } catch (const std::exception& e) {
                // 发生异常时也移动到下一个标定点
               
            }
            is_result_service_ = true;
        });
}

void CamSdkArucoTest::requestArucoMakersInfo()
{
    auto service_request = std::make_shared<custom_msgs_comm::srv::GetMarkersInfo::Request>();
    service_request->request_id = "calib_point_" + std::to_string(current_file_index_);
    
    // 调用Aruco标记信息服务
    aruco_makers_info_client_->async_send_request(
        service_request,
        [this](rclcpp::Client<custom_msgs_comm::srv::GetMarkersInfo>::SharedFuture future) {
            try 
            {
                auto response = future.get();
                if (response->success) 
                {
                    std::cout << "成功获取Aruco标记详细信息" << std::endl;
                    std::cout << "标记数量: " << response->marker_num << std::endl;
                    std::cout << "对象类别: " << response->object_class << std::endl;
                    
                    // 处理每个标记的信息
                    for (int i = 0; i < response->marker_num; i++) {
                        const auto& marker = response->marker_info[i];
                        std::cout << "标记 " << i << ":" << std::endl;
                        std::cout << "  ID: " << marker.marker_id << std::endl;
                        std::cout << "  2D中心点: (" << marker.center_2d.x << ", " << marker.center_2d.y << ")" << std::endl;
                        std::cout << "  3D位置: (" << marker.position.x << ", " << marker.position.y << ", " << marker.position.z << ")" << std::endl;
                        std::cout << "  距离: " << marker.distance << std::endl;
                        
                        // 如果需要，可以将这些信息保存到文件或其他处理
                    }
                    
                    // 保存图像数据
                    std::cerr << "转换图像数据" << std::endl;
                    cv::Mat srcImg;
                    try 
                    {
                        std::cout << "1. 图像参数:" << std::endl;
                        std::cout << "   编码: '" << response->image.encoding << "'" << std::endl;
                        std::cout << "   宽度: " << response->image.width << std::endl;
                        std::cout << "   高度: " << response->image.height << std::endl;
                        std::cout << "   步长: " << response->image.step << " 字节" << std::endl;
                        std::cout << "   数据大小: " << response->image.data.size() << " 字节" << std::endl;
                        std::cout << "   是否为大数据: " << (response->image.is_bigendian ? "是" : "否") << std::endl;
                        
                        // 检查图像数据是否有效
                        if (response->image.data.size() == 0) {
                            std::cerr << "警告: 图像数据为空" << std::endl;
                        } else {
                            // 将ROS图像消息转换为OpenCV图像
                            std::string encoding = response->image.encoding.empty() ? 
                                sensor_msgs::image_encodings::BGR8 : 
                                response->image.encoding;
                            
                            // 确保使用有效的编码格式
                            if (encoding.empty()) {
                                encoding = sensor_msgs::image_encodings::BGR8;
                                std::cerr << "警告: 图像编码为空，使用默认编码: " << encoding << std::endl;
                            }
                            
                            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(response->image, encoding);
                            srcImg = cv_ptr->image.clone();
                        }
                    } 
                    catch (const cv_bridge::Exception& e) 
                    {
                        std::cerr << "图像转换失败: " << e.what() << std::endl;
                    } 
                    catch (const std::exception& e) 
                    {
                        std::cerr << "图像转换失败2: " << e.what() << std::endl;
                    }
                    
                    // 如果有标记信息，保存第一个标记的数据点
                    if (response->marker_num > 0) {
                        comm_alg::MarkerInfo marker_info;
                        marker_info.position.x = response->marker_info[0].position.x;
                        marker_info.position.y = response->marker_info[0].position.y;
                        marker_info.position.z = response->marker_info[0].position.z;
                        marker_info.rotation[0] = response->marker_info[0].rotation.x;
                        marker_info.rotation[1] = response->marker_info[0].rotation.y;
                        marker_info.rotation[2] = response->marker_info[0].rotation.z;
                        marker_info.marker_id = response->marker_info[0].marker_id;
                        
                        // 保存标定数据点
                        std::cerr << "保存标定数据点" << std::endl;
                        saveDataPoint(current_file_index_, marker_info, srcImg, cv::Mat());
                    }
                } else {
                    std::cerr << "获取Aruco标记信息服务失败: " << response->message << std::endl;
                }
                
            } catch (const std::exception& e) {
                std::cerr << "处理Aruco标记信息服务响应时发生异常: " << e.what() << std::endl;
            }
            is_result_sub = true;
        });
}

std::vector<std::string> CamSdkArucoTest::getImageFiles(const std::string& folder_path, 
                                                       const std::vector<std::string>& extensions) {
    std::vector<std::string> files;
    
    try {
        if (!fs::exists(folder_path)) {
            std::cerr << "Folder does not exist: " << folder_path << std::endl;
            return files;
        }
        
        for (const auto& entry : fs::directory_iterator(folder_path)) {
            if (entry.is_regular_file()) {
                std::string filename = entry.path().filename().string();
                if (isSupportedExtension(filename, extensions)) {
                    files.push_back(entry.path().string());
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error reading folder: " << e.what() << std::endl;
    }
    
    return files;
}

bool CamSdkArucoTest::isSupportedExtension(const std::string& filename, 
                                          const std::vector<std::string>& extensions) {
    std::string lower_filename = filename;
    std::transform(lower_filename.begin(), lower_filename.end(), lower_filename.begin(), ::tolower);
    
    for (const auto& ext : extensions) {
        std::string lower_ext = ext;
        std::transform(lower_ext.begin(), lower_ext.end(), lower_ext.begin(), ::tolower);
        
        if (lower_filename.size() >= lower_ext.size() && 
            lower_filename.compare(lower_filename.size() - lower_ext.size(), lower_ext.size(), lower_ext) == 0) {
            return true;
        }
    }
    
    return false;
}

void CamSdkArucoTest::createDirectories() {
    try {
        std::cerr << "创建目录: " << base_save_dir_ << std::endl;
        fs::create_directories(base_save_dir_);
        fs::create_directories(coords_save_dir_);
        fs::create_directories(images_save_dir_);
        fs::create_directories(aruco_images_save_dir_);
    } catch (const std::exception& e) {
        std::cerr << "创建目录失败: " << e.what() << std::endl;
    }
}

void CamSdkArucoTest::cleanupPreviousData() {
    std::cout << "清理之前的数据..." << std::endl;
    
    clearDirectory(coords_save_dir_, "*.json");
    clearDirectory(images_save_dir_, "*.png");
    clearDirectory(aruco_images_save_dir_, "*.png");
    
    std::cout << "✅ 已清理上一次采集的数据" << std::endl;
}

void CamSdkArucoTest::resultCallback(const vision_msgs::msg::Detection2D::SharedPtr msg) {
    std::cout << "Received detection result:" << std::endl;
    std::cout << "  Frame ID: " << msg->header.frame_id << std::endl;
    if (msg->results.size() > 0)
    {
        std::cout << "  Object class: " << msg->results[0].hypothesis.class_id << std::endl;
        std::cout << "  Position: [" 
        << msg->results[0].pose.pose.position.x << ", "
        << msg->results[0].pose.pose.position.y << ", " 
        << msg->results[0].pose.pose.position.z 
        << "]" << std::endl;

        std::cout << "  Orientation: [" 
        << msg->results[0].pose.pose.orientation.x << ", "
        << msg->results[0].pose.pose.orientation.y << ", "
        << msg->results[0].pose.pose.orientation.z << ", "
        << msg->results[0].pose.pose.orientation.w
        << "]" << std::endl;
        
        comm_alg::MarkerInfo marker_info;
        marker_info.position.x = msg->results[0].pose.pose.position.x;
        marker_info.position.y = msg->results[0].pose.pose.position.y;
        marker_info.position.z= msg->results[0].pose.pose.position.z;
        std::vector<double> rotation = quaternionToEulerZYX(msg->results[0].pose.pose.orientation.w, msg->results[0].pose.pose.orientation.x, msg->results[0].pose.pose.orientation.y, msg->results[0].pose.pose.orientation.z);
        marker_info.rotation[0] = rotation[0];
        marker_info.rotation[1] = rotation[1];
        marker_info.rotation[2] = rotation[2]; // Fixed index access
        //saveDataPoint(current_file_index_, marker_info, cv::Mat(), cv::Mat());
    }
    else
    {
        std::cout << " No detection result" << std::endl;
    }
    is_result_sub = true;
    //reservice();
}

void CamSdkArucoTest::calibResultCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::cout << "Received calibration result:" << std::endl;
    std::cout << "  Frame ID: " << msg->header.frame_id << std::endl;
    std::cout << "  Position: [" 
        << msg->pose.position.x << ", "
        << msg->pose.position.y << ", " 
        << msg->pose.position.z 
        << "]" << std::endl;

    std::cout << "  Orientation: [" 
        << msg->pose.orientation.x << ", "
        << msg->pose.orientation.y << ", "
        << msg->pose.orientation.z << ", "
        << msg->pose.orientation.w
        << "]" << std::endl;
        
    // 这里可以添加处理标定结果的逻辑
    // 例如保存到文件或更新内部状态
}

#ifdef __ROS_ENV__
void CamSdkArucoTest::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // 将ROS图像消息转换为OpenCV格式
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        // 显示图像（如果有可视化管理器）
        if (visualizer_) {
            visualizer_->showImage(image);
        }
        
        // 发布处理后的图像
        if (image_pub_) {
            auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
            image_pub_->publish(*out_msg);
        }
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}
#endif

void CamSdkArucoTest::clearDirectory(const std::string& dir_path, 
                                       const std::string& pattern) {
    try {
        if (!fs::exists(dir_path)) {
            return;
        }
        
        for (const auto& entry : fs::directory_iterator(dir_path)) {
            if (entry.is_regular_file()) {
                // 简单的通配符匹配
                std::string filename = entry.path().filename().string();
                std::string ext = entry.path().extension().string();
                
                // 检查扩展名是否匹配
                if (pattern.find(ext) != std::string::npos) {
                    fs::remove(entry.path());
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "清理目录失败: " << e.what() << std::endl;
    }
}

bool CamSdkArucoTest::saveDataPoint(int index, 
                                      const comm_alg::MarkerInfo& marker_info,
                                      const cv::Mat& image,
                                      const cv::Mat& rendered_image) {
    try {
        // 获取当前时间戳
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::stringstream timestamp_ss;
        timestamp_ss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S");
        
        // 创建JSON数据
        // std::stringstream json_ss;
        // json_ss << "{\n";
        // json_ss << "  \"index\": " << index << ",\n";
        // json_ss << "  \"timestamp\": \"" << timestamp_ss.str() << "\",\n";
        // json_ss << "  \"marker_position\": {\n";
        // json_ss << "    \"x\": " << std::fixed << std::setprecision(6) << marker_info.position.x << ",\n";
        // json_ss << "    \"y\": " << std::fixed << std::setprecision(6) << marker_info.position.y << ",\n";
        // json_ss << "    \"z\": " << std::fixed << std::setprecision(6) << marker_info.position.z << "\n";
        // json_ss << "  }\n";
        // json_ss << "}\n";
        
        // 保存JSON文件
        std::stringstream filename_ss;
        filename_ss << "data_point_" << std::setw(3) << std::setfill('0') << index << ".json";
        std::string json_filepath = coords_save_dir_ + "/" + filename_ss.str();
        
        // std::ofstream json_file(json_filepath);
        // if (!json_file.is_open()) {
        //     std::cerr << "无法创建JSON文件: " << json_filepath << std::endl;
        //     return false;
        // }
        // json_file << json_ss.str();
        // json_file.close();



        
    // 2. 读取现有数据（如果有）
    std::string existing_robot_position = "";
    std::string existing_robot_pose = "";
    
    std::ifstream existing_file(json_filepath);
    if (existing_file.is_open()) {
        std::string line;
        while (std::getline(existing_file, line)) {
            // 简单提取 robot_position 和 robot_pose
            if (line.find("\"robot_position\"") != std::string::npos) {
                // 读取多行直到遇到 }
                existing_robot_position = line;
                std::string next_line;
                while (std::getline(existing_file, next_line)) {
                    existing_robot_position += "\n" + next_line;
                    if (next_line.find("}") != std::string::npos) {
                        break;
                    }
                }
            }
            // if (line.find("\"robot_pose\"") != std::string::npos) {
            //     // 读取多行直到遇到 }
            //     existing_robot_pose = line;
            //     std::string next_line;
            //     while (std::getline(existing_file, next_line)) {
            //         existing_robot_pose += "\n" + next_line;
            //         if (next_line.find("}") != std::string::npos) {
            //             break;
            //         }
            //     }
            // }
        }
        existing_file.close();
    }
    
    // 3. 生成时间戳
    auto current_time = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(current_time);
    std::stringstream time_str;
    time_str << std::put_time(std::localtime(&in_time_t), "%Y-%m-%dT%H:%M:%S");
    
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(current_time);
    auto ms = now_ms.time_since_epoch().count() % 1000;
    time_str << "." << std::setw(3) << std::setfill('0') << ms;
    
    // 4. 生成新的JSON
    std::stringstream json_ss;
    json_ss << "{\n";
    json_ss << "  \"index\": " << index << ",\n";
    json_ss << "  \"timestamp\": \"" << time_str.str() << "\",\n";
    
    // 5. 使用现有的robot_position或创建默认
    if (!existing_robot_position.empty()) {
        json_ss << existing_robot_position << "\n";
    } else {
        json_ss << "  \"robot_position\": {\n";
        json_ss << "    \"x\": 0.0,\n";
        json_ss << "    \"y\": 0.0,\n";
        json_ss << "    \"z\": 0.0,\n";
        json_ss << "    \"rx\": 0.0,\n";
        json_ss << "    \"ry\": 0.0,\n";
        json_ss << "    \"rz\": 0.0\n";
        json_ss << "  },\n";
    }
    
    // 6. 新的marker_position
    json_ss << "  \"marker_position\": {\n";
    json_ss << "    \"x\": " << std::fixed << std::setprecision(15) << marker_info.position.x << ",\n";
    json_ss << "    \"y\": " << std::fixed << std::setprecision(15) << marker_info.position.y << ",\n";
    json_ss << "    \"z\": " << std::fixed << std::setprecision(15) << marker_info.position.z << ",\n";
    json_ss << "    \"rx\": " << std::fixed << std::setprecision(15) << marker_info.rotation[0] << ",\n";
    json_ss << "    \"ry\": " << std::fixed << std::setprecision(15) << marker_info.rotation[1] << ",\n";
    json_ss << "    \"rz\": " << std::fixed << std::setprecision(15) << marker_info.rotation[2] << "\n";
    json_ss << "  }\n";
    
    // 7. 使用现有的robot_pose或创建默认
    // if (!existing_robot_pose.empty()) {
    //     json_ss << existing_robot_pose << "\n";
    // } else {
    //     json_ss << "  \"robot_pose\": {\n";
    //     json_ss << "    \"x\": 0.0,\n";
    //     json_ss << "    \"y\": 0.0,\n";
    //     json_ss << "    \"z\": 0.0,\n";
    //     json_ss << "    \"rx\": 0.0,\n";
    //     json_ss << "    \"ry\": 0.0,\n";
    //     json_ss << "    \"rz\": 0.0\n";
    //     json_ss << "  }\n";
    // }
    
    json_ss << "}\n";
    
    // 8. 保存文件
    std::ofstream json_file(json_filepath);
    if (!json_file.is_open()) {
        std::cerr << "无法创建JSON文件: " << json_filepath << std::endl;
        return false;
    }
    json_file << json_ss.str();
    json_file.close();

        
        // 保存源图像
        if (!image.empty()) {
            std::stringstream img_filename_ss;
            img_filename_ss << "data_point_" << std::setw(3) << std::setfill('0') << index << ".png";
            std::string img_filepath = images_save_dir_ + "/" + img_filename_ss.str();
            
            if (!cv::imwrite(img_filepath, image)) {
                std::cerr << "图像保存失败: " << img_filepath << std::endl;
            }
        }
        
        // 保存Aruco渲染后的图像
        if (!rendered_image.empty()) {
            std::stringstream rendered_filename_ss;
            rendered_filename_ss << "data_point_" << std::setw(3) << std::setfill('0') 
                                << index << "_aruco.png";
            std::string rendered_filepath = aruco_images_save_dir_ + "/" + rendered_filename_ss.str();
            cv::Mat rendered_image_temp = rendered_image.clone();
            cv::rectangle(rendered_image_temp, cv::Rect(marker_info.corners[0].x - 10, marker_info.corners[0].y - 10, 20, 20), cv::Scalar(128, 128, 128), 2);
            if (!cv::imwrite(rendered_filepath, rendered_image_temp)) {
                std::cerr << "Aruco渲染图像保存失败: " << rendered_filepath << std::endl;
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "保存数据点失败: " << e.what() << std::endl;
        return false;
    }
}