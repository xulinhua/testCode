#include "marker_detect_ros/marker_detect_node.hpp"
#include "aruco_alg/aruco_detector.hpp"
#include "chessboard_alg/chessboard_pose_detector.hpp"
#include "visualization/visualization_mgr.hpp"
#include "log_system/log_macros.hpp"
#include "bas_operate/file_operate.hpp"
#include "bas_operate/bas_utils.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
// 包含hand_eye_calib头文件
#include "hand_eye_calib/calib_utils.hpp"
#include "hand_eye_calib/head_utils.hpp"

// 添加bas_sys_config_ros头文件
#include "sys_info_src/sys_info_server.h"
#include "bas_sys_config_ros/calib_info_server.h"

// 自定义消息类型
#include "custom_msgs_comm/msg/ai_coordinate_data.hpp"
#include "custom_msgs_comm/srv/get_marker_detection.hpp"
#include <std_srvs/srv/set_bool.hpp>

#include <filesystem>  // 用于文件系统操作
#include "bas_operate_ros/param_utils.hpp"  // 添加param_utils头文件

// 命名空间别名
namespace fs = std::filesystem;
using json = nlohmann::json;

std::string g_str_src_img_path_ = "/home/user/code/Dev/Src_Image";

// 关节偏移（单位：毫米，URDF 中为米，这里转换为毫米保持一致）
double p2y_tx = 5.0;      // 5 mm
double p2y_ty = 0.0;      // 0 mm
double p2y_tz = 173.0;    // 173 mm

namespace marker_detect_ros
{

MarkerDetectNode::MarkerDetectNode(const rclcpp::NodeOptions & options, int cam_id)
	: StatusNodeBase("marker_detect_node_" + std::to_string(cam_id), "marker_detect_ros_" + std::to_string(cam_id), options),
	cam_id_(cam_id), arm_id_(0), 
	arm_id_list_(), sys_config_loaded_(false), init_timer_(nullptr), init_show_timer_(nullptr)
{
	LOG_INFO("Marker检测节点启动，ID : %d", cam_id_);
	arm_id_list_.clear();
	visualizer_src_ = nullptr;
	visualizer_res_ = nullptr;
	keyboard_handler_ = std::make_unique<KeyboardHandler>();// 初始化键盘处理器
	ctrl_l_pressed_ = false;
	calib_thread_running_ = false;

    callback_group_image_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);  //创建两个独立的回调组 可重入，允许并行
    callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);  //创建服务回调组 可重入，允许并行

	initParameterClient();  // 初始化参数客户端
	// 使用新的接口获取系统配置数据
	if (!getSysDat()) {
		LOG_WARN("获取系统配置数据失败，但节点将继续运行");
	}

	// 根据配置机械臂ID列表，初始化位姿/标定矩阵等参数
	if (arm_id_list_.size() == 0)
	{
		arm_id_ = 0;
		arm_id_list_.push_back(arm_id_);
		LOG_WARN("No arm ID configured, using default arm ID: %d", arm_id_);
	}
	arm_id_ = arm_id_list_[0];
	for (size_t i = 0; i < arm_id_list_.size(); ++i)
	{
		int arm_id = arm_id_list_[i];
		calib_result_map_[arm_id] = std::make_unique<handeyecalib::CalibRes>();
		current_robot_pose_map_[arm_id] = geometry_msgs::msg::Pose();
	}
	initParameters();// 初始化参数

	// 创建ArUco检测器
	if (aruco_dict_type_List_.size() == 0 || aruco_length_List_.size() == 0 || aruco_dict_type_List_.size() != aruco_length_List_.size())
	{
		LOG_ERROR( "ArUco检测器创建失败，Marker长度或字典类型配置错误");
		return;
	}

	if (marker_type_ == comm_alg::MarkerType::Aruco)
		base_detector_ = std::make_unique<aruco_alg::ArucoDetector>(aruco_length_List_, aruco_dict_type_List_);
	else if (marker_type_ == comm_alg::MarkerType::Chessboard)
		base_detector_ = std::make_unique<chessboard_alg::ChessboardPoseDetector>(chessboard_size_, chessboard_square_size_);
	else
		LOG_ERROR( "Marker类型配置错误，仅支持Aruco和Chessboard");
	base_detector_->setTransMarkerMM(trans_marker_mm_);
	base_detector_->setCameraIntrinsics(camera_matrix_, dist_coeffs_);
	detection_res_ = std::make_unique<comm_alg::DetectionResult>();  // 检测结果

	for (size_t i = 0; i < aruco_dict_type_List_.size(); ++i)
	{
		int dict_type = aruco_dict_type_List_[i];
		double marker_length = aruco_length_List_[i];
		LOG_INFO("ArUco检测器创建成功 - Marker长度: %.3f, 字典类型: %d", marker_length, dict_type);
	}

	initializeVisualizationManagers();// 初始化可视化管理器
	LOG_INFO("ArUco检测节点初始化完成");
	if (usecalib_)
		LOG_INFO("标定模式已启用");
	else
		LOG_INFO("标定模式已禁用");

	initTopicNames();
	initPubSub();// 初始化发布器和订阅器
	initServices();// 初始化服务服务器
	initHeadMotorAngleSubscriber();// 初始化头部电机角度订阅器

	// 启动ArUco检测线程
	thread_running_ = true;
	detection_thread_ = std::thread(&MarkerDetectNode::detectionThreadFunc, this);

	// 创建键盘监听定时器，每100ms检查一次键盘输入
	keyboard_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
		keyboardTimerCallback();
	});

	initCalibParams();
	initializeTimer();// 初始化显示定时器

	auto init_timer_period = std::chrono::milliseconds(1000); // 100毫秒后初始化
	init_timer_ = this->create_wall_timer(init_timer_period, [this]() {
		initExtraData(); // 初始化额外数据
		LOG_INFO("定时器初始化完成"); // 调试信息
		init_timer_->cancel();
	});

	// 发布运行中状态
    publishModuleStatus(basros::ModuleStatus::RUNNING, "模块初始化完成，正常运行中");
}

MarkerDetectNode::~MarkerDetectNode()
{
	if (visualizer_src_) {
		visualizer_src_->closeWindows();
	}
	if (visualizer_res_) {
		visualizer_res_->closeWindows();
	}
	
	// 停止检测线程
	stopDetectionThread();
	
	// 停止标定线程
	if (calib_thread_.joinable()) {
		calib_thread_running_ = false;
		calib_thread_.join();
	}
	
	LOG_INFO("Aruco检测节点关闭");
}

void MarkerDetectNode::initializeVisualizationManagers()
{
	if (show_src_image_)
	{
		visualizer_src_ = std::make_unique<visualization::VisualizationMgr>();
		if (!visualizer_src_->initialize("Marker Src Cam_" + std::to_string(cam_id_))) {
			std::cerr << "Failed to initialize visualization manager" << std::endl;
		}
	}
	if (show_result_image_)
	{
		visualizer_res_ = std::make_unique<visualization::VisualizationMgr>();
		if (!visualizer_res_->initialize("Marker Result Cam_" + std::to_string(cam_id_))) {
			std::cerr << "Failed to initialize Result visualization manager" << std::endl;
		}
	}
}

void MarkerDetectNode::initializeTimer()
{
	if (show_src_image_ || show_result_image_)
	{
		auto init_timer_period = std::chrono::milliseconds(1000); // 100毫秒后初始化
		init_show_timer_ = this->create_wall_timer(init_timer_period, [this]() {
			static int init_count = 0;
			cv::Mat image_show = cv::Mat(480, 640, CV_8UC3, cv::Scalar(100, 150, 200));  // 自定义颜色
			cv::putText(image_show, "Aruco Detection : Wait for Image!",
				cv::Point(200, 20), cv::FONT_HERSHEY_SIMPLEX, 0.7,
				cv::Scalar(255, 250, 250), 2);
			if (visualizer_src_ && !image_callback_invoked_)
			{
				//cv::imwrite("init_image.png", image_show);
				visualizer_src_->showImage(image_show);
			}
			if (visualizer_res_ && !image_callback_invoked_)
			{
				//cv::imwrite("init_image.png", image_show);
				visualizer_res_->showImage(image_show);
			}
			init_count++;
			// 刷新5次再取消定时器，确保图像刷新
			if (init_count > 30 || image_callback_invoked_)
			{
				LOG_INFO("初始化完成图像显示"); // 调试信息
				init_show_timer_->cancel();
			}
		});
	}
	auto init_timer_period = std::chrono::milliseconds(5000); // 5秒刷新一次
	parameters_timer_ = this->create_wall_timer(init_timer_period, [this]() {
		if (parameters_changed_)
		{
			initCalibParams();
			std::lock_guard<std::mutex> lock(parameters_mutex_);
			parameters_changed_ = false;
		}
	});
	
}

void MarkerDetectNode::initParameters()
{
	// 声明并获取参数
	image_callback_invoked_ = false;  // 是否进入图像回调函数
	marker_result_type_ = "center";  // 可选值: "center" 或 "corner"，默认为 "center"
	image_topic_ = "/camera/color/image_raw";
	depth_image_topic_ = "/camera/depth/image_raw";
	detection_result_topic_ = "/marker_detection/result";
	camera_info_topic_ = "/camera/camera_info";  // 默认相机内参话题
	camera_info_service_name_ = "/camera/camera_info";
	detection_service_name_ = "/marker_detection/result";
	markers_info_service_name_ = "/marker_detection/markers_info";
	image_result_topic_ = "/marker_detection/result_image";
	marker_src_img_service_name_ = "/marker_detection/src_image";
	marker_clear_service_name_ = "/marker_detection/clear";
	show_src_image_ = true;
	show_result_image_ = true;
	usecalib_ = true; //默认先打开
	trans_marker_mm_ = true; //默认使用mm
	marker_type_ = comm_alg::MarkerType::Aruco;
	aruco_dict_type_List_ = {5};
	aruco_length_List_ = {1.0};
	chessboard_size_ = cv::Size(10, 7);
	chessboard_square_size_ = 0.024f;
	result_is_ready_ = false;  // 检测结果是否就绪
	image_proc_is_ready_ = false;  // 图像处理是否就绪
	camera_info_received_ = false;
	std::string cam_id_str = std::to_string(cam_id_);
	std::string cam_dir_str = "cam_" + cam_id_str;

	std::string default_config_file_path = ament_index_cpp::get_package_share_directory("marker_detect_ros") + "/config" + "/marker_detect_params.yaml";
	LOG_INFO("  参数文件: %s", default_config_file_path.c_str());

	if (std::filesystem::exists(default_config_file_path))
	{
		YAML::Node config = YAML::LoadFile(default_config_file_path);
		std::string key_str = cam_dir_str + "_parameters";
		if (config[key_str] && config[key_str]["marker_result_type"])
			marker_result_type_ = config[key_str]["marker_result_type"].as<std::string>();
		else
			LOG_WARN("配置文件中缺少%s.marker_result_type配置项", key_str.c_str());
		if (config[key_str] && config[key_str]["show_src_image"])
			show_src_image_ = config[key_str]["show_src_image"].as<bool>();
		else
			LOG_WARN("配置文件中缺少%s.show_src_image配置项", key_str.c_str());
		if (config[key_str] && config[key_str]["show_result_image"])
			show_result_image_ = config[key_str]["show_result_image"].as<bool>();
		else
			LOG_WARN("配置文件中缺少%s.show_result_image配置项", key_str.c_str());
		if (config[key_str] && config[key_str]["trans_marker_mm"])
			trans_marker_mm_ = config[key_str]["trans_marker_mm"].as<bool>();
		else
			LOG_WARN("配置文件中缺少%s.trans_marker_mm配置项", key_str.c_str());

		if (config[key_str] && config[key_str]["marker_type"])
			marker_type_ = (comm_alg::MarkerType)config[key_str]["marker_type"].as<int>();
		else
			LOG_WARN("配置文件中缺少%s.marker_type配置项", key_str.c_str());
		
		// 检查是否有新的字典类型和标记长度列表配置
		if (config[key_str] && config[key_str]["aruco_dict_type"])
		{
			// 解析配置文件中的字典类型和标记长度列表
			YAML::Node aruco_config_list = config[key_str]["aruco_dict_type"];
			if (aruco_config_list.IsSequence())
			{
				aruco_length_List_.clear();
				aruco_dict_type_List_.clear();
				
				for (const auto& item : aruco_config_list)
				{
					if (item.size() == 2)
					{
						int dict_type = item[0].as<int>();
						double marker_length = item[1].as<double>();
						aruco_dict_type_List_.push_back(dict_type);
						aruco_length_List_.push_back(marker_length);
					}
				}

				// 如果列表不为空，使用第一个值作为默认值
				if (aruco_dict_type_List_.empty() || aruco_length_List_.empty())
				{
					LOG_ERROR("配置文件中缺少%s.aruco_dict_type配置项", key_str.c_str());
				}
			}
			else
			{
				// 保持向后兼容性，处理旧的单一值配置
				LOG_WARN("配置文件中缺少%s.aruco_dict_type配置项", key_str.c_str());
				int aruco_dict_type = 5;
				double marker_length = 1.0;
				aruco_dict_type_List_.clear();
				aruco_length_List_.clear();

				if (config[key_str] && config[key_str]["aruco_dict_type"])
					aruco_dict_type = config[key_str]["aruco_dict_type"].as<int>();				
				// 检查是否有单独的marker_length配置
				if (config[key_str] && config[key_str]["marker_length"])
					marker_length = config[key_str]["marker_length"].as<double>();
				aruco_dict_type_List_.push_back(aruco_dict_type);
				aruco_length_List_.push_back(marker_length);
			}
		}
		else
			LOG_WARN("配置文件中缺少%s.aruco_dict_type配置项", key_str.c_str());

		if (config[key_str] && config[key_str]["chessboard_size"])
			chessboard_size_ = cv::Size(config[key_str]["chessboard_size"][0].as<int>(), config[key_str]["chessboard_size"][1].as<int>());
		else
			LOG_WARN("配置文件中缺少%s.chessboard_size配置项", key_str.c_str());
		if (config[key_str] && config[key_str]["chessboard_square_size"])
			chessboard_square_size_ = config[key_str]["chessboard_square_size"].as<double>();
		else
			LOG_WARN("配置文件中缺少%s.chessboard_square_size配置项", key_str.c_str());
	}
	else
		LOG_WARN("配置文件%s不存在", default_config_file_path.c_str());

	LOG_INFO("参数配置:");
	// LOG_INFO("  Marker长度: %.3f m", marker_length_);
	// LOG_INFO("  字典类型: %d", aruco_dict_type_);
	LOG_INFO("  标记类型: %s", marker_type_ == comm_alg::MarkerType::Aruco ? "Aruco" : "Chessboard");
	LOG_INFO("  标定模式: %s", usecalib_ ? "开启" : "关闭");
	LOG_INFO("  显示源图像: %s", show_src_image_ ? "开启" : "关闭");
	LOG_INFO("  显示结果图像: %s", show_result_image_ ? "开启" : "关闭");
}

void MarkerDetectNode::initTopicNames()
{
	// 使用bas_sys_config_ros模块的parseCommInfo函数获取话题名称
	basros::RosCommInfo comm_info;
	comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_COLOR_IMAGE, cam_id_, 0);
	//image_topic_ = comm_info.name;
	if (sys_config_client_->has_parameter(comm_info.name)) {
        // 安全地获取参数值
		std::string param = sys_config_client_->get_parameter<std::string>(comm_info.name);
		if (param != "")
			image_topic_ = param;
	}


	comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_SRC_DEPTH_IMAGE, cam_id_, 0);
	//depth_image_topic_ = comm_info.name;
	if (sys_config_client_->has_parameter(comm_info.name)) {
        // 安全地获取参数值
		std::string param = sys_config_client_->get_parameter<std::string>(comm_info.name);
        if (param != "")
			depth_image_topic_ = param;
	}

	comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_CAM_INTRINSICS, cam_id_, 0);
	//camera_info_topic_ = comm_info.name;
	camera_info_service_name_ = comm_info.name;
	if (sys_config_client_->has_parameter(comm_info.name)) {
        // 安全地获取参数值
        std::string param = sys_config_client_->get_parameter<std::string>(comm_info.name);
        if (param != "")
			camera_info_topic_ = param;
	}


	comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_MARKER_RESULTS, cam_id_, 0);
	detection_result_topic_ = comm_info.name;
	detection_service_name_ = comm_info.name;

	comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_MARKER_MARKERS_INFO, cam_id_, 0);
	markers_info_service_name_ = comm_info.name;

	for (size_t i = 0; i < arm_id_list_.size(); ++i)
	{
		comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_MARKER_CALIB_RESULTS, cam_id_, arm_id_list_[i]);
		calib_result_topic_map_[arm_id_list_[i]] = comm_info.name;
	}

	comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_ARM_CURRENT_POSE, cam_id_, 0);
	robot_pose_topic_ = comm_info.name;

	comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_MARKER_RESULTS_IMAGE, cam_id_, 0);
	image_result_topic_ = comm_info.name;
	comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_MARKER_SRC_IMAGE, cam_id_, 0);
	marker_src_img_service_name_ = comm_info.name;
	comm_info = basros::parseCommInfo(basros::RosCommMsgType::COMM_MARKER_CLEAR, cam_id_, 0);
	marker_clear_service_name_ = comm_info.name;
}

void MarkerDetectNode::initParameterClient()
{
	// 避免当前节点多次添加到executor，Node '/marker_detect_node_1' has already been added to an executor.
	parameters_node_ = std::make_shared<rclcpp::Node>("marker_detect_params_node_" + std::to_string(cam_id_));

	// 创建连接到系统配置节点的参数客户端
	sys_config_node_name_ = "sys_config_ros_node";
	//sys_config_node_name_ = "cam_sdk_aruco_test";
	sys_config_client_ = std::make_shared<rclcpp::SyncParametersClient>(
		parameters_node_,  // 当前节点
		sys_config_node_name_  // 目标节点名称
		);

	// ===== 1. 订阅参数事件 =====
	initCalibSyncMechanism();

	LOG_INFO("初始化参数客户端，连接节点: sys_config_ros_node");
}

bool MarkerDetectNode::getSysDat()
{
	// 等待参数服务可用
	if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
	{
		LOG_ERROR("无法连接到系统配置参数服务");
		return false; // 返回失败状态，但不终止节点
	}
	bool success = true; // 标记整体操作是否成功
	try
	{
		SysConfig::CamConfigInfo cam_info;
		success = getCamConfigInfo(cam_info);
		if (!success)
		{
			LOG_WARN("参数服务器未找到系统参数");
		}
		sys_config_loaded_ = success; // 更新系统配置加载状态
		return success; // 返回操作结果
	}
	catch (const std::exception& e)
	{
		LOG_ERROR("读取系统参数失败: %s", e.what());
		sys_config_loaded_ = false; // 更新系统配置加载状态
		return false; // 返回失败状态
	}
}

bool MarkerDetectNode::getCamConfigInfo(SysConfig::CamConfigInfo& cam_info)
{
	bool bRet = RosComm::getCamInfoFromServer(sys_config_client_, cam_id_, cam_info);
	if (bRet)
	{
		LOG_INFO("读取到与当前相机ID匹配的配置，ID: %d, 机械臂列表: %zu", cam_info.cam_id, cam_info.armInfoList.size());
		const SysConfig::ArmConfigInfoList& arm_info_list = cam_info.armInfoList;
		if (!arm_info_list.empty())
		{
			for (const auto& arm_info : arm_info_list)
			{
				LOG_INFO("获取配置的机械臂ID: %d, 是否启用: %s", arm_info.arm_id, arm_info.is_enable ? "是" : "否");
				if (arm_info.is_enable)
				{
					arm_id_list_.push_back(arm_info.arm_id);
				}
			}
		}
	}
	return bRet;
}

//初始化标定参数
void MarkerDetectNode::initCalibParams()
{
	// 从参数服务器读取初始标定数据
	handeyecalib::CamCalibInfo cam_calib_info;
	if (RosComm::getCamCalibInfoFromServer(sys_config_client_, cam_id_, cam_calib_info))
	{
		for (auto& arm_info : cam_calib_info.arm_calib1D)
		{
			int arm_id = arm_info.first;
			LOG_INFO("标定数据更新，机械臂ID: %d", arm_id);
			auto it = find(arm_id_list_.begin(), arm_id_list_.end(), arm_id);
			if (it != arm_id_list_.end())
			{
				calibDatChangedCallback(arm_info.second);
			}
		}
	} 
	else 
	{
		LOG_WARN("从参数服务器读取初始标定数据失败");
	}
	LOG_INFO("标定参数处理器初始化完成");
}

void MarkerDetectNode::calibDatChangedCallback(const handeyecalib::ArmCalibInfo& calib_data)
{
	LOG_INFO("接收到标定数据更新，机械臂ID: %d", static_cast<int>(calib_data.arm_id));
	
	const handeyecalib::ArmCalibInfo& arm_calib_info = calib_data;
	int arm_id = arm_calib_info.arm_id;

	auto it = std::find(arm_id_list_.begin(), arm_id_list_.end(), arm_id);
	if (it == arm_id_list_.end()) {
		return;
	}
	
	LOG_INFO("更新机械臂标定数据，机械臂ID: %d, 标定类型: %s", 
		static_cast<int>(arm_id), handeyecalib::CalibRes::getCalibTypeString(arm_calib_info.calib_info.calib_res.calib_type));

	// 检查是否已存在该机械臂的标定数据
	if (calib_result_map_.find(arm_id) == calib_result_map_.end()) {
		calib_result_map_[arm_id] = std::make_unique<handeyecalib::CalibRes>();
	}
	
	// 更新标定结果
	*calib_result_map_[arm_id] = arm_calib_info.calib_info.calib_res;

	cv::Mat cam_to_base_transform = arm_calib_info.calib_info.calib_res.cam_to_base_transform;

	LOG_INFO("相机到基座的变换矩阵: (%.3f, %.3f, %.3f, %.3f)",
		cam_to_base_transform.at<double>(0, 0), cam_to_base_transform.at<double>(1, 0), cam_to_base_transform.at<double>(2, 0), cam_to_base_transform.at<double>(3, 0));
	LOG_INFO("相机到基座的变换矩阵: (%.3f, %.3f, %.3f, %.3f)",
		cam_to_base_transform.at<double>(0, 1), cam_to_base_transform.at<double>(1, 1), cam_to_base_transform.at<double>(2, 1), cam_to_base_transform.at<double>(3, 1));
	LOG_INFO("相机到基座的变换矩阵: (%.3f, %.3f, %.3f, %.3f)",
		cam_to_base_transform.at<double>(0, 2), cam_to_base_transform.at<double>(1, 2), cam_to_base_transform.at<double>(2, 2), cam_to_base_transform.at<double>(3, 2));
	LOG_INFO("相机到基座的变换矩阵: (%.3f, %.3f, %.3f, %.3f)",
		cam_to_base_transform.at<double>(0, 3), cam_to_base_transform.at<double>(1, 3), cam_to_base_transform.at<double>(2, 3), cam_to_base_transform.at<double>(3, 3));

	LOG_INFO("机械臂%d标定数据更新完成，偏移补偿: x=%.3f, y=%.3f, z=%.3f", arm_id,
		calib_result_map_[arm_id]->offset_compensation[0],
		calib_result_map_[arm_id]->offset_compensation[1],
		calib_result_map_[arm_id]->offset_compensation[2]);
}

void MarkerDetectNode::initPubSub()
{
	// 创建图像订阅器
	rclcpp::SubscriptionOptions image_options;
	image_options.callback_group = callback_group_image_;
	image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(image_topic_, 10,
		std::bind(&MarkerDetectNode::imageCallback, this, std::placeholders::_1), image_options);
	
	depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(depth_image_topic_, 10,
		std::bind(&MarkerDetectNode::depthImageCallback, this, std::placeholders::_1), image_options);

	// 创建相机内参订阅器
	camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10,
		std::bind(&MarkerDetectNode::cameraInfoCallback, this, std::placeholders::_1));

	// 创建机器人位姿订阅器  
	robot_pose_sub_ = this->create_subscription<custom_msgs_comm::msg::RobotStdPose>(robot_pose_topic_, 10,
			std::bind(&MarkerDetectNode::robotPoseCallback, this, std::placeholders::_1));
		LOG_INFO("订阅机器人位姿话题: %s", robot_pose_topic_.c_str());

	// 创建检测结果发布器 (改为Detection2D类型)
	detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2D>(detection_result_topic_, 10);

	// 创建标定结果发布器
	for (auto it : calib_result_topic_map_)
	{
		calib_result_pub_map_[it.first] = nullptr;
		if (it.second.empty())
			continue;
		calib_result_pub_map_[it.first] = this->create_publisher<geometry_msgs::msg::PoseStamped>(it.second, 10);
		LOG_INFO("发布标定结果话题: %s", it.second.c_str());
	}

	// 创建标定结果发布器
	image_result_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_result_topic_, 10);

	LOG_INFO("订阅话题: %s", image_topic_.c_str());
	LOG_INFO("订阅相机内参话题: %s", camera_info_topic_.c_str());  // 添加日志
	LOG_INFO("发布话题: %s", detection_result_topic_.c_str());
	LOG_INFO("发布结果图像话题: %s", image_result_topic_.c_str());
}

void MarkerDetectNode::initServices()
{
	// 创建Marker检测服务服务器
	marker_detection_service_ = this->create_service<custom_msgs_comm::srv::GetMarkerDetection>(
		detection_service_name_,
		std::bind(&MarkerDetectNode::markerDetectionService, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_service_);

	// 创建Marker源图像服务服务器
	marker_source_image_service_ = this->create_service<custom_msgs_comm::srv::GetMarkerSrcImage>(
		marker_src_img_service_name_,
		std::bind(&MarkerDetectNode::getMarkerSrcImgService, this, std::placeholders::_1, std::placeholders::_2));

	// 创建Marker标记信息服务服务器
	markers_info_service_ = this->create_service<custom_msgs_comm::srv::GetMarkersInfo>(
		markers_info_service_name_,
		std::bind(&MarkerDetectNode::getMarkersInfoService, this, std::placeholders::_1, std::placeholders::_2));

	// 创建清除检测结果服务服务器
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clear_detection_result_service_;
	clear_detection_result_service_ = this->create_service<std_srvs::srv::SetBool>(
		marker_clear_service_name_,
		std::bind(&MarkerDetectNode::clearDetectionResultService, this, std::placeholders::_1, std::placeholders::_2));

	// 创建获取相机内参服务客户端
	get_cam_intr_client_ = this->create_client<custom_msgs_comm::srv::GetCamIntr>(camera_info_service_name_);

	LOG_INFO("Marker检测服务服务器已创建: %s", detection_service_name_.c_str());
	LOG_INFO("Marker源图像服务服务器已创建: %s", marker_src_img_service_name_.c_str());
	LOG_INFO("Marker标记信息服务服务器已创建: %s", markers_info_service_name_.c_str());
	LOG_INFO("清除检测结果服务服务器已创建: %s", marker_clear_service_name_.c_str());
	LOG_INFO("获取相机内参服务客户端已创建: %s", camera_info_service_name_.c_str());
}

bool MarkerDetectNode::initExtraData()
{
	LOG_INFO("初始化额外数据");
	int count = 0;
	while (!getCameraIntrinsics(cam_id_, 3)) {
		if (count > 2) {
			LOG_ERROR("获取相机内参超时");
			break; // Exit the loop if timeout occurs
		}
		count++;
	}
	return true;
}

void MarkerDetectNode::onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
	// 只处理来自系统配置节点的事件
	std::string myParameterNodeName = std::string("/" + sys_config_node_name_);
	LOG_DEBUG("收到系统配置节点参数事件: %s， 当前节点名: %s", event->node.c_str(), myParameterNodeName.c_str());
	if (event->node != myParameterNodeName) // 只响应系统配置节点的事件
		return;

	handeyecalib::ArmCalibInfo arm_calib_info;
	std::vector<std::string> arm_info_prefixs;
	for (auto it : arm_id_list_)
	{
		std::string arm_info_prefix = RosComm::getArmCalibInfoParamPrefix(cam_id_, it);
		arm_info_prefixs.push_back(arm_info_prefix);
		LOG_DEBUG("参数前缀: %s", arm_info_prefix.c_str());
	}

	// 检查 cam_Num 是否变更
	for (size_t i = 0; i < event->changed_parameters.size(); i++)
	{
		LOG_DEBUG("参数名称: %s", event->changed_parameters[i].name.c_str());
		bool bChanged = false;
		for (auto it : arm_info_prefixs)
		{
			if (event->changed_parameters[i].name.find(it) != std::string::npos)
			{
				bChanged = true;
				break;
			}
		}
		if (bChanged)
		{
			std::lock_guard<std::mutex> lock(parameters_mutex_);
			parameters_changed_ = true;
			LOG_INFO("参数发生改变");
			break;
		}
	}

	onParameterEventDebounce(event);
}

bool MarkerDetectNode::markerDetect()
{
	if (!image_proc_is_ready_)
		return false;
	LOG_DEBUG("进行Marker检测");
	bool res = true;
	try
	{
		cv::Mat cur_image;
		cv::Mat res_image;
		std::string frame_id;

		{ //拷贝检测数据
			std::lock_guard<std::mutex> lock(image_mutex_);
			
			if (!image_.empty())
				image_.copyTo(cur_image);
			else
			{
				LOG_ERROR("图像为空");
				return false;
			}
			frame_id = frame_id_;
			image_proc_is_ready_ = false;
		}

		LOG_DEBUG("图像帧ID: %s", frame_id.c_str());
		// 显示图像（如果有可视化管理器）
		if (show_src_image_ && visualizer_src_)
		{
			LOG_DEBUG("显示源图");
			visualizer_src_->showImage(cur_image);
		}

		// 创建Detection2D消息 (替换原来的AICoordinateData消息)
		auto detection_msg = vision_msgs::msg::Detection2D();
		detection_msg.header.stamp = this->now();
		detection_msg.header.frame_id = frame_id;

		// 使用maker检测器检测标记
		comm_alg::DetectionResult result = base_detector_->detectAndProcessMarkers(cur_image, nullptr, true, false);
		if (result.processed_frame.empty())
			cur_image.copyTo(res_image);
		else
			result.processed_frame.copyTo(res_image);
		if (!res_image.empty())
		{
			cv::putText(res_image, "frame_id : " + frame_id,
				cv::Point(200, 20), cv::FONT_HERSHEY_SIMPLEX, 0.7,
				cv::Scalar(0, 255, 255), 2);
		}

		{ //拷贝检测数据
			std::lock_guard<std::mutex> lock(image_mutex_);
			*detection_res_ = result;
			cur_image.copyTo(image_proc_);
			if (!result.processed_frame.empty())
				result.processed_frame.copyTo(detection_res_->processed_frame);
			result_is_ready_ = true;
		}

		// 如果检测到标记，发布结果
		if (result.found && !result.markers_info.empty())
		{
			// 发布第一个检测到的标记的中心位置
			LOG_DEBUG("检测到 %zu 个Marker标记", result.markers_info.size());

			// 设置边界框中心位置（2D位置，没有z坐标）
			detection_msg.bbox.center.position.x = result.markers_info[0].position.x;
			detection_msg.bbox.center.position.y = result.markers_info[0].position.y;
			// 注意：Point2D结构没有z坐标

			// 设置边界框大小（假设为一个点，大小为0）
			detection_msg.bbox.size_x = 0.0;
			detection_msg.bbox.size_y = 0.0;

			for (size_t i = 0; i < result.markers_info.size(); i++)
			{
				// 设置检测结果
				auto detect_res = vision_msgs::msg::ObjectHypothesisWithPose();
				detect_res.hypothesis.class_id = "marker";
				detect_res.hypothesis.score = 1.0;

				if (marker_result_type_ == "corner")
				{
					detect_res.pose.pose.position.x = result.markers_info[i].world_corners[0].x;
					detect_res.pose.pose.position.y = result.markers_info[i].world_corners[0].y;
					detect_res.pose.pose.position.z = result.markers_info[i].world_corners[0].z;

				LOG_DEBUG("第一个角点标记位置: (%.3f, %.3f, %.3f)",
					detect_res.pose.pose.position.x, detect_res.pose.pose.position.y, detect_res.pose.pose.position.z);
				}
				else 
				{
					detect_res.pose.pose.position.x = result.markers_info[i].position.x;
					detect_res.pose.pose.position.y = result.markers_info[i].position.y;
					detect_res.pose.pose.position.z = result.markers_info[i].position.z;

					LOG_DEBUG("第一个中心标记位置: (%.3f, %.3f, %.3f)",
						detect_res.pose.pose.position.x, detect_res.pose.pose.position.y, detect_res.pose.pose.position.z);
				}
				detect_res.pose.pose.orientation.w = 1; // w
      			detect_res.pose.pose.orientation.x = result.markers_info[i].rotation[0]; // x
      			detect_res.pose.pose.orientation.y = result.markers_info[i].rotation[1]; // y
      			detect_res.pose.pose.orientation.z = result.markers_info[i].rotation[2]; // z

				detection_msg.results.push_back(detect_res);
			}

			if (usecalib_)
				transMarkerToCalibRes(result, res_image);
		}
		else
		{
			LOG_DEBUG("未检测到Marker标记");
		}

		detection_pub_->publish(detection_msg);

		if (!res_image.empty())
			drawExtraDataResultImage(res_image);
		if (show_result_image_ && visualizer_res_ && !res_image.empty())
			visualizer_res_->showImage(res_image);
		if (image_result_pub_ && !res_image.empty())
		{
			auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", res_image).toImageMsg();
			image_msg->header.stamp = this->now();
			image_msg->header.frame_id = frame_id;
			image_result_pub_->publish(*image_msg);
			LOG_DEBUG("发布结果图像");
		}
	}
	catch (const cv_bridge::Exception& e) {
		LOG_ERROR("图像转换失败: %s", e.what());
		res = false;
	}
	catch (const std::exception& e) {
		LOG_DEBUG("检测异常: %s", e.what());
		res = false;
	}
	return res;
}

void MarkerDetectNode::transMarkerToCalibRes(const comm_alg::DetectionResult &result, cv::Mat &res_image)
{
	if (!usecalib_)
		return;
	if (result.found && !result.markers_info.empty())
	{
		for (size_t i = 0; i < result.markers_info.size(); i++)
		{
			for (auto it : calib_result_pub_map_)
			{
				int arm_id = it.first;
				if (it.second == nullptr || calib_result_map_[arm_id] == nullptr || calib_result_map_[arm_id]->cam_to_base_transform.empty())
					continue;
				std::vector<double> position;
				if (marker_result_type_ == "corner")
				{
					position = {
						static_cast<double>(result.markers_info[i].world_corners[0].x),
						static_cast<double>(result.markers_info[i].world_corners[0].y),
						static_cast<double>(result.markers_info[i].world_corners[0].z),
						static_cast<double>(result.markers_info[i].rotation[0]),
						static_cast<double>(result.markers_info[i].rotation[1]),
						static_cast<double>(result.markers_info[i].rotation[2])
					};
				}
				else 
				{
					position = {
						static_cast<double>(result.markers_info[i].position.x),
						static_cast<double>(result.markers_info[i].position.y),
						static_cast<double>(result.markers_info[i].position.z),
						static_cast<double>(result.markers_info[i].rotation[0]),
						static_cast<double>(result.markers_info[i].rotation[1]),
						static_cast<double>(result.markers_info[i].rotation[2])
					};
				}

				std::vector<double> orientation(4, 0.0f);
				orientation[3] = 1.0f;
				handeyecalib::TransformResult trans_result;
				try
				{
					if (calib_result_map_[arm_id]->calib_type == handeyecalib::CalibRes::CalibType::EYE_IN_HAND)
          			{
            			// current_robot_pose_.orientation.x当rx处理，外部发送的是rx
            			std::vector<double> endPos = {
          					static_cast<double>(current_robot_pose_map_[arm_id].position.x),
          					static_cast<double>(current_robot_pose_map_[arm_id].position.y),
          					static_cast<double>(current_robot_pose_map_[arm_id].position.z),
              				static_cast<double>(current_robot_pose_map_[arm_id].orientation.x),
              				static_cast<double>(current_robot_pose_map_[arm_id].orientation.y),
              				static_cast<double>(current_robot_pose_map_[arm_id].orientation.z)
            			};
						trans_result = handeyecalib::computeRobotPoseFromEndMarker(
							position, endPos, calib_result_map_[arm_id]->cam_to_base_transform);
					}
					else
					{
						std::vector<double> offset_compensation = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  // 初始化偏移补偿值
						trans_result = handeyecalib::computeRobotPoseFromMarker(
							position, calib_result_map_[arm_id]->cam_to_base_transform, &offset_compensation);
					}		
				}
				catch (const std::exception& e)
				{
					LOG_ERROR("标定结果异常: %s", e.what());
				}
				if (!trans_result.transformed_pose.empty())
				{
					// 打印补偿前的位置和补偿值
					LOG_DEBUG("补偿前位置: (%.3f, %.3f, %.3f), 补偿值: (%.3f, %.3f, %.3f)",
						position[0], position[1], position[2],
						calib_result_map_[arm_id]->offset_compensation[0], calib_result_map_[arm_id]->offset_compensation[1], calib_result_map_[arm_id]->offset_compensation[2]);

					// 更新位置为机器人位置
					position[0] = static_cast<double>(trans_result.transformed_pose[0]);
					position[1] = static_cast<double>(trans_result.transformed_pose[1]);
					position[2] = static_cast<double>(trans_result.transformed_pose[2]);
					position[0] += calib_result_map_[arm_id]->offset_compensation[0]; //加上偏移补偿
					position[1] += calib_result_map_[arm_id]->offset_compensation[1];
					position[2] += calib_result_map_[arm_id]->offset_compensation[2];
					LOG_DEBUG("机器人位置: (%.3f, %.3f, %.3f)",
						position[0], position[1], position[2]);
					if (!res_image.empty()) {
						std::string result_point = "calib_result_point_arm_" + std::to_string(arm_id) + ": (" +
							std::to_string(position[0]) + ", " +
							std::to_string(position[1]) + ", " +
							std::to_string(position[2]) + ", " +
							std::to_string(trans_result.transformed_pose[3]) + ", " +
							std::to_string(trans_result.transformed_pose[4]) + ", " +
							std::to_string(trans_result.transformed_pose[5]) + ", " +
							std::to_string(1.0) + ")";
						cv::putText(res_image, result_point,
							cv::Point(20, res_image.rows - 80 - arm_id * 20 - i*20), cv::FONT_HERSHEY_SIMPLEX, 0.4,
							cv::Scalar(0, 255, 255), 1);
					}
					// 发布标定结果
					auto calib_pose_msg = geometry_msgs::msg::PoseStamped();
					calib_pose_msg.header.stamp = this->now();
					calib_pose_msg.header.frame_id = std::to_string(arm_id);
					calib_pose_msg.pose.position.x = position[0];
					calib_pose_msg.pose.position.y = position[1];
					calib_pose_msg.pose.position.z = position[2];
					calib_pose_msg.pose.orientation.x = trans_result.transformed_pose[3];
					calib_pose_msg.pose.orientation.y = trans_result.transformed_pose[4];
					calib_pose_msg.pose.orientation.z = trans_result.transformed_pose[5];
					calib_pose_msg.pose.orientation.w = 1.0;

					calib_result_pub_map_[arm_id]->publish(calib_pose_msg);
				}
				else
					LOG_ERROR("无法计算机器人位置");

				if (0) //计算头部电机角度偏移
				{
					std::vector<double> offset_compensation = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  // 初始化偏移补偿值
					position = {
						static_cast<double>(result.markers_info[i].position.x),
						static_cast<double>(result.markers_info[i].position.y),
						static_cast<double>(result.markers_info[i].position.z),
						static_cast<double>(result.markers_info[i].rotation[0]),
						static_cast<double>(result.markers_info[i].rotation[1]),
						static_cast<double>(result.markers_info[i].rotation[2])
					};

					cv::Mat yaw_end = handeyecalib::YawAngleToTransform(current_head_angles_[1]);
					std::vector<double> endPos = handeyecalib::homogeneousMatrixToPose(yaw_end);
					trans_result = handeyecalib::computeRobotPoseFromEndMarker(
							position, endPos, head_transform_);
					
					// 更新位置为机器人位置
					position[0] = static_cast<double>(trans_result.transformed_pose[0]);
					position[1] = static_cast<double>(trans_result.transformed_pose[1]);
					position[2] = static_cast<double>(trans_result.transformed_pose[2]);
					LOG_DEBUG("头部电机角度偏移后机器人位置为: (%.3f, %.3f, %.3f)",
						position[0], position[1], position[2]);
					if (!res_image.empty()) {
						std::string result_point = "head_angle: pitch: " + std::to_string(current_head_angles_[0]) + 
						", yaw: " + std::to_string(current_head_angles_[1]) + 
						", position: " + std::to_string(arm_id) + ": (" +
							std::to_string(position[0]) + ", " +
							std::to_string(position[1]) + ", " +
							std::to_string(position[2]) + ")";
						cv::putText(res_image, result_point,
							cv::Point(20, res_image.rows - 100 - arm_id * 20 - i*20), cv::FONT_HERSHEY_SIMPLEX, 0.6,
							cv::Scalar(255, 0, 255), 1);
					}
				}
				if (0) //计算头部电机角度偏移
				{
					std::vector<double> offset_compensation = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  // 初始化偏移补偿值
					position = {
						static_cast<double>(result.markers_info[i].position.x),
						static_cast<double>(result.markers_info[i].position.y),
						static_cast<double>(result.markers_info[i].position.z),
						static_cast<double>(result.markers_info[i].rotation[0]),
						static_cast<double>(result.markers_info[i].rotation[1]),
						static_cast<double>(result.markers_info[i].rotation[2])
					};

					cv::Mat T_p_y = handeyecalib::computeHeadPoseTransform(current_head_angles_[0], current_head_angles_[1], p2y_tx, p2y_ty, p2y_tz);
					cv::Mat T_end = head_base_transform_ * T_p_y;
					std::vector<double> endPos = handeyecalib::homogeneousMatrixToPose(T_end);
					
					trans_result = handeyecalib::computeRobotPoseFromEndMarker(
							position, endPos, head_transform_);
					
					// 更新位置为机器人位置
					position[0] = static_cast<double>(trans_result.transformed_pose[0]);
					position[1] = static_cast<double>(trans_result.transformed_pose[1]);
					position[2] = static_cast<double>(trans_result.transformed_pose[2]);
					LOG_DEBUG("头部电机角度偏移后机器人位置为: (%.3f, %.3f, %.3f)",
						position[0], position[1], position[2]);
					if (!res_image.empty()) {
						std::string result_point = "head_angle: pitch: " + std::to_string(current_head_angles_[0]) + 
						", yaw: " + std::to_string(current_head_angles_[1]) + 
						", position: " + std::to_string(arm_id) + ": (" +
							std::to_string(position[0]) + ", " +
							std::to_string(position[1]) + ", " +
							std::to_string(position[2]) + ")";
						cv::putText(res_image, result_point,
							cv::Point(20, res_image.rows - 100 - arm_id * 20 - i*20), cv::FONT_HERSHEY_SIMPLEX, 0.6,
							cv::Scalar(255, 0, 255), 1);
					}
				}
			}
		}	
	}
}

void MarkerDetectNode::detectionThreadFunc()
{
	LOG_INFO("Marker检测线程启动");
	
	while (thread_running_) {
		try {
			markerDetect();
			
			// 短暂休眠以控制处理频率
			rclcpp::sleep_for(std::chrono::milliseconds(20)); // 每10ms处理一次
		} catch (const std::exception& e) {
			LOG_ERROR("检测线程异常: %s", e.what());
		}
	}
	
	LOG_INFO("Marker检测线程结束");
}

void MarkerDetectNode::stopDetectionThread()
{
	if (thread_running_) {
		LOG_INFO("正在停止检测线程...");
		thread_running_ = false;
		
		// 等待线程结束，最多等待5秒
		if (detection_thread_.joinable()) {
			detection_thread_.join();
			LOG_INFO("检测线程已停止");
		} else {
			LOG_WARN("检测线程不可连接");
		}
	}
}

void MarkerDetectNode::markerDetectionService(
	const std::shared_ptr<custom_msgs_comm::srv::GetMarkerDetection::Request> request,
	std::shared_ptr<custom_msgs_comm::srv::GetMarkerDetection::Response> response)
{
	LOG_INFO("收到Marker检测服务请求");

	try
	{
		// 转换ROS图像消息为OpenCV图像
		std::string frame_id = request->request_id;

		LOG_INFO("图像帧ID: %s", frame_id.c_str());

		{ //清除上一次结果
			std::lock_guard<std::mutex> lock(image_mutex_);
			result_is_ready_ = false;
			*detection_res_ = comm_alg::DetectionResult(); //清除识别结果
		}

		// 最多等500ms刷新结果
		int count = 0;
		while (!result_is_ready_ && count < 25) {
			rclcpp::sleep_for(std::chrono::milliseconds(20));
			// LOG_INFO("等待检测结果... %d", count);
			count++;
		}

		// 设置响应基本信息
		response->stamp = this->now();
		response->frame_id = frame_id;
		response->success = false;
		response->message = "未检测到Marker标记";
		response->object_class = "";
		response->confidence = 0.0;
		response->position.clear();
		response->orientation.clear();

		comm_alg::DetectionResult result;
		if (result_is_ready_)
		{ //拷贝检测数据
			std::lock_guard<std::mutex> lock(image_mutex_);
			result = *detection_res_;
			image_proc_.copyTo(service_request_image_);
			*detection_res_ = comm_alg::DetectionResult(); //清除识别结果
		}
		else
		{
			response->message = "未检测到Marker标记, 检测超时";
			LOG_ERROR("未检测到Marker标记, 检测超时"); 
			return;
		}

		cv::Mat image = service_request_image_.clone();
		cv::Mat res_image = result.processed_frame.clone();

		if (image.empty())
		{
			LOG_INFO("图像为空");
			response->message = "图像为空";
			return;
		}

		// 转换OpenCV图像为ROS图像消息
		cv_bridge::CvImage cv_image;
		cv_image.header.stamp = this->now();
		cv_image.header.frame_id = frame_id;
		// 确保图像编码正确设置，如果图像为空则使用默认编码
		cv_image.encoding = sensor_msgs::image_encodings::BGR8;  // 默认编码
		cv_image.image = image;

		// 将cv_bridge图像转换为ROS图像消息
		cv_image.toImageMsg(response->image);

		// 显示图像（如果有可视化管理器）
		if (show_src_image_ && visualizer_src_) {
			LOG_INFO("显示源图");
			visualizer_src_->showImage(image);
		}

		// 使用Marker检测器检测标记
		// auto result = marker_detector_->detectAndProcessMarkers(image, nullptr, true, true);

		if (res_image.empty())
			image.copyTo(res_image);
		if (!res_image.empty()) {
			cv::putText(res_image, "frame_id : " + frame_id,
				cv::Point(200, 20), cv::FONT_HERSHEY_SIMPLEX, 0.7,
				cv::Scalar(0, 255, 255), 2);
		}

		// 如果检测到标记，填充结果
		if (result.found && !result.markers_info.empty()) {
			LOG_INFO("检测到 %zu 个Marker标记", result.markers_info.size());

			response->success = true;
			response->message = "成功检测到Marker标记";
			response->object_class = "marker";
			response->confidence = 1.0;

			// 设置位置坐标
			response->position.resize(3);
			response->orientation.resize(4);
			if (marker_result_type_ == "corner")
			{
				response->position[0] = result.markers_info[0].world_corners[0].x;
				response->position[1] = result.markers_info[0].world_corners[0].y;
				response->position[2] = result.markers_info[0].world_corners[0].z;
				LOG_INFO("第一个角点标记位置: (%.3f, %.3f, %.3f)",
					response->position[0], response->position[1], response->position[2]);
			}
			else {
				response->position[0] = result.markers_info[0].position.x;
				response->position[1] = result.markers_info[0].position.y;
				response->position[2] = result.markers_info[0].position.z;

				LOG_INFO("第一个中心标记位置: (%.3f, %.3f, %.3f)",
					response->position[0], response->position[1], response->position[2]);
			}

			response->orientation[3] = 1; // w
			response->orientation[0] = result.markers_info[0].rotation[0]; // x
			response->orientation[1] = result.markers_info[0].rotation[1]; // y
			response->orientation[2] = result.markers_info[0].rotation[2]; // z

																	// std::string imagefile = src_image_dir_ + frame_id + ".png";
																	// if (!cv::imwrite(imagefile, image)) {
																	//     std::cerr << "图像保存失败: " << imagefile << std::endl;
																	//   }
		}
		else
		{
			LOG_INFO("未检测到Marker标记");
			response->message = "未检测到Marker标记";
		}
		if (!res_image.empty())
		{
			drawExtraDataResultImage(res_image);
		}

		// 转换OpenCV图像为ROS图像消息
		cv_bridge::CvImage cv_image_res;
		cv_image_res.header.stamp = this->now();
		cv_image_res.header.frame_id = frame_id;
		// 确保图像编码正确设置，如果图像为空则使用默认编码
		cv_image_res.encoding = sensor_msgs::image_encodings::BGR8;  // 默认编码
		cv_image_res.image = res_image;
		// 将cv_bridge图像转换为ROS图像消息
		cv_image_res.toImageMsg(response->result_image);

		if (show_result_image_ && visualizer_res_ && !res_image.empty())
		{
			LOG_DEBUG("显示结果渲染图");
			visualizer_res_->showImage(res_image);
		}
	}
	catch (const cv_bridge::Exception& e) {
		LOG_ERROR("图像转换失败: %s", e.what());
		response->success = false;
		response->message = "图像转换失败: " + std::string(e.what());
	}
	catch (const std::exception& e) {
		LOG_ERROR("检测异常: %s", e.what());
		response->success = false;
		response->message = "检测异常: " + std::string(e.what());
	}

	// 确保总是设置response的success字段
	if (!response->success) {
		LOG_INFO("Marker检测服务响应: %s", response->message.c_str());
	}
}

bool MarkerDetectNode::getImage(cv::Mat & image)
{
	std::lock_guard<std::mutex> lock(image_mutex_);  // 自动加锁
	if (image_.empty())
	{
		image_.release();
		return false;
	}

	image_.copyTo(image);
	return true;
}

void MarkerDetectNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try
	{
		// 转换ROS图像消息为OpenCV图像
		LOG_DEBUG("进入图像回调");
		image_callback_invoked_ = true;  // 设置图像回调函数已调用
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		{ //拷贝检测数据
			std::lock_guard<std::mutex> lock(image_mutex_);
			if (!cv_ptr->image.empty())
				cv_ptr->image.copyTo(image_);
			else
			{
				LOG_ERROR("图像为空");
				return;
			}
			frame_id_ = msg->header.frame_id;
			image_proc_is_ready_ = true;
		}

		LOG_DEBUG("图像帧ID: %s", frame_id_.c_str());
		
		// 图像处理现在在检测线程中完成，这里不再直接调用markerDetect()
	}
	catch (const cv_bridge::Exception& e) {
		LOG_ERROR("图像转换失败: %s", e.what());
	}
	catch (const std::exception& e) {
		LOG_ERROR("检测异常: %s", e.what());
	}
}

void MarkerDetectNode::depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try 
  {
    // 转换ROS图像消息为OpenCV图像
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    
    LOG_DEBUG("接收到深度图像: %dx%d, 编码: %s", 
                 cv_ptr->image.cols, cv_ptr->image.rows, msg->encoding.c_str());
    
    // 存储深度图像
    cv_ptr->image.copyTo(depth_image_);

    // 显示图像（如果有可视化管理器）
    if (show_src_image_ && visualizer_src_) 
    {
      LOG_DEBUG("显示深度源图");
      visualizer_src_->showImage(depth_image_);
    }
    // 这里可以对深度图像进行处理
    // 例如：与检测到的Marker标记位置进行深度值提取
    
    // 你可以在这里添加深度图像处理逻辑
    // 比如获取Marker标记中心位置的深度值等
    
  } catch (const cv_bridge::Exception& e) {
    LOG_ERROR("深度图像转换失败: %s", e.what());
  } catch (const std::exception& e) {
    LOG_ERROR("深度图像处理异常: %s", e.what());
  }
}

void MarkerDetectNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(camera_info_mutex_);

	// 更新相机内参矩阵
	camera_matrix_ = (cv::Mat_<double>(3, 3) <<
		msg->k[0], msg->k[1], msg->k[2],
		msg->k[3], msg->k[4], msg->k[5],
		msg->k[6], msg->k[7], msg->k[8]);

	// 更新畸变系数
	dist_coeffs_ = cv::Mat::zeros(1, msg->d.size(), CV_64F);
	for (size_t i = 0; i < msg->d.size(); i++) {
		dist_coeffs_.at<double>(0, i) = msg->d[i];
	}

	// // 相机内参矩阵:
	// camera_matrix_ = (cv::Mat_<double>(3, 3) << 
	// 	717.7244963883575, 0, 639.7124789653936,
	// 	0, 718.6032326483363, 381.8491245277353,
	// 	0, 0, 1);

	// // 畸变系数:
	// dist_coeffs_ = (cv::Mat_<double>(1, 5) << 
	// 	-0.07036155648129329, 0.9168588734019, -0.001286742166097405, 0.002838800244360746, -2.57546054681674);

	// 更新Marker检测器的相机内参
	if (!camera_info_received_)
	{
		base_detector_->setCameraIntrinsics(camera_matrix_, dist_coeffs_);
		camera_info_received_ = true;
		LOG_INFO("接收到相机内参 - fx: %.6f, fy: %.6f, cx: %.6f, cy: %.6f",
			msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
	}

}

void MarkerDetectNode::robotPoseCallback(const custom_msgs_comm::msg::RobotStdPose::SharedPtr msg)
{
	int arm_id = msg->arm_id;
	if (current_robot_pose_map_.find(arm_id) != current_robot_pose_map_.end())
	{
		current_robot_pose_map_[arm_id] = msg->std_pose.pose;
		//LOG_INFO("收到机械臂ID %d 的位置", arm_id);
	}
	// else
	// 	LOG_INFO("没有配置机械臂ID %d", arm_id);

		
}

void MarkerDetectNode::keyboardTimerCallback()
{
	// 检查是否有键盘输入
	int ch = keyboard_handler_->readOne();

	// 检查是否按下了Ctrl+L (12是Ctrl+L的ASCII码)
	if (ch == 12) {
	  LOG_INFO("检测到Ctrl+L按键，重新加载机器人位姿偏移值");
	  loadOffsetRobotPose();
	}
	// 检查是否按下了Ctrl+D (4是Ctrl+D的ASCII码)
	else if (ch == 4) 
	{
	  LOG_INFO("检测到Ctrl+D按键，禁用机器人位姿偏移值");
	  // 将机器人位姿偏移值置零
	  calib_result_map_[arm_id_]->offset_compensation[0] = 0.0;
	  calib_result_map_[arm_id_]->offset_compensation[1] = 0.0;
	  calib_result_map_[arm_id_]->offset_compensation[2] = 0.0;
	  LOG_INFO("机器人位姿偏移值已置零: (%.3f, %.3f, %.3f)", 
			calib_result_map_[arm_id_]->offset_compensation[0], 
			calib_result_map_[arm_id_]->offset_compensation[1], 
			calib_result_map_[arm_id_]->offset_compensation[2]);
	}
	// 检查是否按下了Ctrl+E (5是Ctrl+E的ASCII码)
	else if (ch == 5) 
	{
	  LOG_INFO("检测到Ctrl+E按键，启用机器人位姿偏移值");
	  loadOffsetRobotPose();
	}
	// 检查是否按下了Ctrl+I (9是Ctrl+I的ASCII码)
	else if (ch == 9) 
	{
	  LOG_INFO("检测到Ctrl+I按键，保存当前图像到路径: %s", g_str_src_img_path_.c_str());
	  
	  // 确保保存路径存在
	  if (!std::filesystem::exists(g_str_src_img_path_)) {
		std::filesystem::create_directories(g_str_src_img_path_);
		LOG_INFO("创建保存路径: %s", g_str_src_img_path_.c_str());
	  }
	  
	  // 获取当前时间作为文件名
	  auto now = std::chrono::system_clock::now();
	  auto now_c = std::chrono::system_clock::to_time_t(now);
	  std::tm tm_now;
	  localtime_r(&now_c, &tm_now);
	  char filename[100];
	  sprintf(filename, "image_%04d%02d%02d_%02d%02d%02d.jpg", 
		  tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
		  tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
	  
	  std::string save_path = g_str_src_img_path_ + "/" + filename;
	  
	  // 锁定图像互斥量并保存图像
	  { 
		std::lock_guard<std::mutex> lock(image_mutex_);
		if (!image_.empty()) {
		  if (cv::imwrite(save_path, image_)) {
			LOG_INFO("图像保存成功: %s", save_path.c_str());
		  } else {
			LOG_ERROR("图像保存失败: %s", save_path.c_str());
		  }
		} else {
		  LOG_WARN("当前没有可用图像，无法保存");
		}
	  }
	}
	// 检查是否按下了Ctrl+H (8是Ctrl+H的ASCII码)
	else if (ch == 8)
	{
	  LOG_INFO("检测到Ctrl+H按键，启动标定线程");
	  
	  if (!calib_thread_running_) {
		calib_thread_running_ = true;
		if (calib_thread_.joinable()) {
		  calib_thread_.join();
		}
		calib_thread_ = std::thread(&MarkerDetectNode::calibThreadFunc, this);
		LOG_INFO("标定线程已启动");
	  } else {
		LOG_WARN("标定线程已在运行中");
	  }
	}
	else if (ch == 25) // 检查是否按下了Ctrl+y (ASCII: 25) current_head_angles_[1]加5度
	{
		auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.push_back(current_head_angles_[0]);
        msg.data.push_back(current_head_angles_[1] + 5.0);
        msg.data.push_back(0.0);
        head_motor_angles_pub_->publish(msg);
		LOG_INFO("检测到Ctrl+y按键，头部电机角度偏航加5度， 现在角度 %f", current_head_angles_[1] + 5.0);
	}
	else if (ch == 16) // 检查是否按下了Ctrl+p (ASCII: 16) current_head_angles_[0]加5度
	{
		auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.push_back(current_head_angles_[0] + 5.0);
        msg.data.push_back(current_head_angles_[1]);
        msg.data.push_back(0.0);
        head_motor_angles_pub_->publish(msg);
		LOG_INFO("检测到Ctrl+p按键，头部电机角度俯仰加5度， 现在角度 %f", current_head_angles_[0] + 5.0);
	}
	else if (ch == 20) // 检查是否按下了Ctrl+t (ASCII: 20) current_head_angles_[1]减5度
	{
		auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.push_back(current_head_angles_[0]);
        msg.data.push_back(current_head_angles_[1] - 5.0);
        msg.data.push_back(0.0);
        head_motor_angles_pub_->publish(msg);
		LOG_INFO("检测到Ctrl+t按键，头部电机角度偏航减5度， 现在角度 %f", current_head_angles_[1] - 5.0);
	}
	else if (ch == 15) // 检查是否按下了Ctrl+o (ASCII: 15) current_head_angles_[0]减5度
	{
		auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.push_back(current_head_angles_[0] - 5.0);
        msg.data.push_back(current_head_angles_[1]);
        msg.data.push_back(0.0);
        head_motor_angles_pub_->publish(msg);
		LOG_INFO("检测到Ctrl+o按键，头部电机角度俯仰减5度， 现在角度 %f", current_head_angles_[0] - 5.0);
	}

}

void MarkerDetectNode::loadOffsetRobotPose()
{
	// 使用通用接口获取当前项目所在的install目录的绝对路径
	std::string install_path_str = basmodule::get_install_dir();
	fs::path install_path(install_path_str);

	// 读取marker_to_arm_offset文件配置
	fs::path config_path = install_path.parent_path() / "bas_config_data" / "cam_config";
	std::string strCamArm = "cam" + std::to_string(cam_id_) + "_arm" + std::to_string(arm_id_);
	std::string marker_tool_offset_file_path = config_path.string() + "/cam_" + std::to_string(cam_id_) + "/" + strCamArm + "/tcp_offset_" + strCamArm + ".yaml";
	LOG_INFO("重新从配置文件读取marker_tool_offset文件: %s", marker_tool_offset_file_path.c_str());

	// 读取现有配置
	if (std::filesystem::exists(marker_tool_offset_file_path))
	{
		YAML::Node config = YAML::LoadFile(marker_tool_offset_file_path);
		bool bEnable = config["enable"].as<bool>();
		if (bEnable)
		{
			calib_result_map_[arm_id_]->offset_compensation[0] = config["offset_x"].as<double>();
			calib_result_map_[arm_id_]->offset_compensation[1] = config["offset_y"].as<double>();
			calib_result_map_[arm_id_]->offset_compensation[2] = config["offset_z"].as<double>();
		}
		else
		{
			calib_result_map_[arm_id_]->offset_compensation[0] = 0.0;
			calib_result_map_[arm_id_]->offset_compensation[1] = 0.0;
			calib_result_map_[arm_id_]->offset_compensation[2] = 0.0;
		}
		// 设置topic_name
		

		LOG_INFO("%s 重新从配置文件读取机器人位姿偏移: %.3f, %.3f, %.3f", bEnable ? "使能" : "禁止", 
			calib_result_map_[arm_id_]->offset_compensation[0], calib_result_map_[arm_id_]->offset_compensation[1], calib_result_map_[arm_id_]->offset_compensation[2]);
	}
	else
	{
		LOG_WARN("marker_tool_offset文件不存在: 使用默认机器人位姿偏移: %.3f, %.3f, %.3f",
			calib_result_map_[arm_id_]->offset_compensation[0], calib_result_map_[arm_id_]->offset_compensation[1], calib_result_map_[arm_id_]->offset_compensation[2]);
	}
}

bool MarkerDetectNode::initCameraIntrinsics(const std::string& intrinsics_config_file)
{
	try
	{
		// 从配置文件中指定的路径加载相机内参
		std::string intrinsics_json = intrinsics_config_file;

		if (!fs::exists(intrinsics_json)) {
			LOG_ERROR("相机内参文件不存在: %s", intrinsics_json.c_str());
			LOG_ERROR("无法加载相机内参，程序将无法正常运行");
			return false;
		}

		// 读取JSON文件
		std::ifstream file(intrinsics_json);
		if (!file.is_open()) {
			LOG_ERROR("无法打开相机内参文件: %s", intrinsics_json.c_str());
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
		if (pos != std::string::npos)
		{
			double k1, k2, p1, p2, k3;
			size_t start = json_str.find("[", pos);
			if (start != std::string::npos) {
				sscanf(json_str.c_str() + start + 1, "%lf, %lf, %lf, %lf, %lf",
					&k1, &k2, &p1, &p2, &k3);
				dist_coeffs_ = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
			}
		}
		else {
			// 如果没有畸变系数，使用零
			dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
		}

		LOG_INFO("✅ 从配置文件加载相机内参: %s", intrinsics_json.c_str());
		LOG_INFO("   fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f", fx, fy, ppx, ppy);

		return true;
	}
	catch (const std::exception& e) {
		LOG_ERROR("Failed to load camera intrinsics: %s", e.what());
		return false;
	}
}

// ========== 标定参数同步机制（推拉结合） ==========

void MarkerDetectNode::initCalibSyncMechanism()
{
    LOG_INFO("初始化标定参数同步机制...");
    
    // 1. 订阅参数事件（推模式）
    param_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        rclcpp::QoS(100).reliable(),
        std::bind(&MarkerDetectNode::onParameterEvent, this, std::placeholders::_1)
    );
    
    // 2. 启动版本号检查定时器（拉模式-兜底）
    version_check_timer_ = this->create_wall_timer(
        VERSION_CHECK_INTERVAL,
        std::bind(&MarkerDetectNode::versionCheckTimerCallback, this)
    );
    
    // 3. 立即同步一次（确保启动时数据最新）
    if (!syncCalibDataFromServer()) {
        LOG_ERROR("初始标定数据同步失败，将在后台重试");
    }
    
    LOG_INFO("标定参数同步机制初始化完成");
}

bool MarkerDetectNode::syncCalibDataFromServer()
{
    std::lock_guard<std::mutex> lock(calib_sync_mutex_);
    
    if (!sys_config_client_ || !sys_config_client_->service_is_ready()) {
        LOG_ERROR("系统配置客户端未连接");
        return false;
    }
    
    LOG_INFO("开始从参数服务器同步标定数据...");
    
    // 读取完整的标定数据
    handeyecalib::CamCalibInfo cam_calib_info;
    if (!RosComm::getCamCalibInfoFromServer(sys_config_client_, cam_id_, cam_calib_info)) {
        LOG_ERROR("读取标定数据失败");
        return false;
    }
    
    // 更新本地数据
    updateLocalCalibData(cam_calib_info);
    
    // 更新版本号
    local_calib_version_ = getServerCalibVersion();
    
    LOG_INFO("标定数据同步完成，版本号=%ld", local_calib_version_.load());
    return true;
}

void MarkerDetectNode::onParameterEventDebounce(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    // 只处理系统配置节点的事件
    if (event->node != "/" + sys_config_node_name_) {
        return;
    }
    
    // 检查是否包含标定相关参数
    bool is_calib_change = false;
    for (const auto& param : event->changed_parameters) {
        // 检查是否是本相机的标定参数
        std::string cam_prefix = "cam_calib_list.cam_" + std::to_string(cam_id_);
        if (param.name.find(cam_prefix) != std::string::npos ||
            param.name.find(".version") != std::string::npos) {
            is_calib_change = true;
            LOG_DEBUG("检测到标定参数变化: %s", param.name.c_str());
            break;
        }
    }
    
    if (!is_calib_change) {
        return;
    }
    
    // 防抖处理：多个参数变化时只触发一次更新
    pending_update_ = true;
    
    // 取消之前的定时器
    if (debounce_timer_) {
        debounce_timer_->cancel();
    }
    
    // 创建新的防抖定时器
    debounce_timer_ = this->create_wall_timer(
        DEBOUNCE_DELAY,
        [this]() {
            if (pending_update_.exchange(false)) {
                LOG_INFO("防抖触发，执行标定数据同步");
                syncCalibDataFromServer();
            }
        }
    );
}

void MarkerDetectNode::versionCheckTimerCallback()
{
    // 如果正在处理更新，跳过本次检查
    if (pending_update_.load()) {
        return;
    }
    
    int64_t server_version = getServerCalibVersion();
    if (server_version < 0) {
        LOG_DEBUG("获取服务器版本号失败");
        return;
    }
    
    int64_t local_version = local_calib_version_.load();
    
    // 版本号不一致，需要更新
    if (server_version != local_version) {
        LOG_WARN("版本号不匹配: 本地=%ld, 服务器=%ld，执行同步", 
                 local_version, server_version);
        syncCalibDataFromServer();
    } else {
        LOG_DEBUG("版本号一致: %ld", local_version);
    }
}

int64_t MarkerDetectNode::getServerCalibVersion()
{
    if (!sys_config_client_ || !sys_config_client_->service_is_ready()) {
        return -1;
    }
    
    try {
        std::string version_param = "cam_calib_list.cam_" + 
                                    std::to_string(cam_id_) + ".version";
        
		if (sys_config_client_->has_parameter(version_param))
		{
			auto param = sys_config_client_->get_parameter<int64_t>(version_param);
            return param;
		}
        return -1; // 参数不存在时返回-1
    }
    catch (const std::exception& e) {
        LOG_DEBUG("获取版本号参数失败: %s", e.what());
        return -1;
    }
}

void MarkerDetectNode::updateLocalCalibData(const handeyecalib::CamCalibInfo& cam_calib_info)
{
    // 更新每个机械臂的标定数据
    for (const auto& arm_pair : cam_calib_info.arm_calib1D) {
        int arm_id = arm_pair.first;
        const auto& arm_calib = arm_pair.second;
        
        auto it = std::find(arm_id_list_.begin(), arm_id_list_.end(), arm_id);
        if (it != arm_id_list_.end()) {
            calibDatChangedCallback(arm_calib);
            LOG_INFO("机械臂%d标定数据已更新", arm_id);
        }
    }
}

void MarkerDetectNode::getMarkerSrcImgService(
	const std::shared_ptr<custom_msgs_comm::srv::GetMarkerSrcImage::Request> request,
	std::shared_ptr<custom_msgs_comm::srv::GetMarkerSrcImage::Response> response)
{
	LOG_INFO("收到Marker源图像服务请求");

	try
	{
		// 获取请求ID
		std::string frame_id = request->request_id;
		LOG_INFO("图像帧ID: %s", frame_id.c_str());
		cv::Mat image;
		// 在服务请求时刷新图像
		{
			std::lock_guard<std::mutex> lock(image_mutex_);
			service_request_image_.copyTo(image);
		}

		if (image.empty())
		{
			LOG_INFO("图像为空");
			response->success = false;
			response->message = "图像为空";
			return;
		}

		// 设置响应基本信息
		response->stamp = this->now();
		response->frame_id = frame_id;
		response->success = true;
		response->message = "成功获取Marker源图像";

		// 转换OpenCV图像为ROS图像消息
		cv_bridge::CvImage cv_image;
		cv_image.header.stamp = this->now();
		cv_image.header.frame_id = frame_id;
		cv_image.encoding = sensor_msgs::image_encodings::BGR8;
		cv_image.image = image;

		// 将cv_bridge图像转换为ROS图像消息
		cv_image.toImageMsg(response->image);

		LOG_INFO("成功获取Marker源图像，尺寸: %dx%d", image.cols, image.rows);

	}
	catch (const cv_bridge::Exception& e) {
		LOG_ERROR("图像转换失败: %s", e.what());
		response->success = false;
		response->message = "图像转换失败: " + std::string(e.what());
	}
	catch (const std::exception& e) {
		LOG_ERROR("获取图像异常: %s", e.what());
		response->success = false;
		response->message = "获取图像异常: " + std::string(e.what());
	}
}

void MarkerDetectNode::getMarkersInfoService(
	const std::shared_ptr<custom_msgs_comm::srv::GetMarkersInfo::Request> request,
	std::shared_ptr<custom_msgs_comm::srv::GetMarkersInfo::Response> response)
{
	LOG_INFO("收到Marker标记信息服务请求");

	try
	{
		// 转换ROS图像消息为OpenCV图像
		std::string frame_id = request->request_id;

		LOG_INFO("图像帧ID: %s", frame_id.c_str());

		// 在服务请求时刷新图像
		cv::Mat image;
		comm_alg::DetectionResult result;
		{ //拷贝检测数据
			std::lock_guard<std::mutex> lock(image_mutex_);
			result = *detection_res_;
			image_.copyTo(image);
		}

		cv::Mat res_image = result.processed_frame.clone();

		// 设置响应基本信息
		response->stamp = this->now();
		response->frame_id = frame_id;
		response->success = false;
		response->message = "未检测到Marker标记";
		response->object_class = "";
		response->marker_num = 0;

		if (image.empty())
		{
			LOG_INFO("图像为空");
			response->message = "图像为空";
			return;
		}

		// 转换OpenCV图像为ROS图像消息
		cv_bridge::CvImage cv_image;
		cv_image.header.stamp = this->now();
		cv_image.header.frame_id = frame_id;
		// 确保图像编码正确设置，如果图像为空则使用默认编码
		cv_image.encoding = sensor_msgs::image_encodings::BGR8;  // 默认编码
		cv_image.image = image;

		// 将cv_bridge图像转换为ROS图像消息
		cv_image.toImageMsg(response->image);

		// 显示图像（如果有可视化管理器）
		if (show_src_image_ && visualizer_src_) {
			LOG_INFO("显示源图");
			visualizer_src_->showImage(image);
		}

		if (res_image.empty())
			image.copyTo(res_image);
		if (!res_image.empty()) {
			cv::putText(res_image, "frame_id : " + frame_id,
				cv::Point(200, 20), cv::FONT_HERSHEY_SIMPLEX, 0.7,
				cv::Scalar(0, 255, 255), 2);
		}

		// 如果检测到标记，填充结果
		if (result.found && !result.markers_info.empty()) {
			LOG_INFO("检测到 %zu 个Marker标记", result.markers_info.size());

			response->success = true;
			response->message = "成功检测到Marker标记";
			response->object_class = "marker";
			response->marker_num = static_cast<int32_t>(result.markers_info.size());

			// 调整marker_info数组大小
			response->marker_info.resize(result.markers_info.size());

			// 填充每个标记的信息
			for (size_t i = 0; i < result.markers_info.size(); ++i) {
				const auto& marker = result.markers_info[i];

				// 设置标记ID
				response->marker_info[i].marker_id = marker.marker_id;

				// 设置2D中心点
				response->marker_info[i].center_2d.x = marker.center_2d.x;
				response->marker_info[i].center_2d.y = marker.center_2d.y;
				response->marker_info[i].center_2d.z = 0.0;  // 2D点没有z坐标

															 // 设置3D位置
				response->marker_info[i].position.x = marker.position.x;
				response->marker_info[i].position.y = marker.position.y;
				response->marker_info[i].position.z = marker.position.z;

				// 设置旋转（欧拉角）
				response->marker_info[i].rotation.x = marker.rotation[0];
				response->marker_info[i].rotation.y = marker.rotation[1];
				response->marker_info[i].rotation.z = marker.rotation[2];

				// 设置旋转矩阵（3x3矩阵转为9个元素的数组）
				response->marker_info[i].rotation_matrix.resize(9);
				response->marker_info[i].rotation_matrix[0] = marker.rotation_matrix(0, 0);
				response->marker_info[i].rotation_matrix[1] = marker.rotation_matrix(0, 1);
				response->marker_info[i].rotation_matrix[2] = marker.rotation_matrix(0, 2);
				response->marker_info[i].rotation_matrix[3] = marker.rotation_matrix(1, 0);
				response->marker_info[i].rotation_matrix[4] = marker.rotation_matrix(1, 1);
				response->marker_info[i].rotation_matrix[5] = marker.rotation_matrix(1, 2);
				response->marker_info[i].rotation_matrix[6] = marker.rotation_matrix(2, 0);
				response->marker_info[i].rotation_matrix[7] = marker.rotation_matrix(2, 1);
				response->marker_info[i].rotation_matrix[8] = marker.rotation_matrix(2, 2);

				// 设置旋转向量
				response->marker_info[i].rvec.resize(3);
				response->marker_info[i].rvec[0] = marker.rvec[0];
				response->marker_info[i].rvec[1] = marker.rvec[1];
				response->marker_info[i].rvec[2] = marker.rvec[2];

				// 设置平移向量
				response->marker_info[i].tvec.resize(3);
				response->marker_info[i].tvec[0] = marker.tvec[0];
				response->marker_info[i].tvec[1] = marker.tvec[1];
				response->marker_info[i].tvec[2] = marker.tvec[2];

				// 设置角点（像素坐标）
				response->marker_info[i].corners.resize(marker.corners.size());
				for (size_t j = 0; j < marker.corners.size(); ++j) {
					response->marker_info[i].corners[j].x = marker.corners[j].x;
					response->marker_info[i].corners[j].y = marker.corners[j].y;
					response->marker_info[i].corners[j].z = 0.0;  // 2D点没有z坐标
				}

				// 设置世界坐标系下的角点
				response->marker_info[i].world_corners.resize(marker.world_corners.size());
				for (size_t j = 0; j < marker.world_corners.size(); ++j) {
					response->marker_info[i].world_corners[j].x = marker.world_corners[j].x;
					response->marker_info[i].world_corners[j].y = marker.world_corners[j].y;
					response->marker_info[i].world_corners[j].z = marker.world_corners[j].z;
				}

				// 设置距离
				response->marker_info[i].distance = marker.distance;

				// 设置置信度
				response->marker_info[i].confidence = 1.0;  // 默认置信度为1.0
			}

		}
		else {
			LOG_INFO("未检测到Marker标记");
			response->message = "未检测到Marker标记";
		}

		// 显示结果图像（如果有可视化管理器）
		if (show_result_image_ && visualizer_res_ && !res_image.empty())
		{
			drawExtraDataResultImage(res_image);
			LOG_DEBUG("显示结果渲染图");
			visualizer_res_->showImage(res_image);
		}
	}
	catch (const cv_bridge::Exception& e) {
		LOG_ERROR("图像转换失败: %s", e.what());
		response->success = false;
		response->message = "图像转换失败: " + std::string(e.what());
	}
	catch (const std::exception& e) {
		LOG_ERROR("检测异常: %s", e.what());
		response->success = false;
		response->message = "检测异常: " + std::string(e.what());
	}

	// 确保总是设置response的success字段
	if (!response->success) {
		LOG_INFO("Marker标记信息服务响应: %s", response->message.c_str());
	}
}

void MarkerDetectNode::clearDetectionResultService(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  LOG_INFO("收到清除检测结果服务请求");
  
  try {
    // 设置响应基本信息
    response->success = false;
    response->message = "清除检测结果失败";
    
    // 获取请求参数
    bool clear_flag = request->data;
    LOG_INFO("清除标志: %s", clear_flag ? "true" : "false");
    
    // 如果清除标志为true，则清除检测结果
    if (clear_flag) {
      std::lock_guard<std::mutex> lock(image_mutex_);
      *detection_res_ = comm_alg::DetectionResult(); // 清除识别结果
      result_is_ready_ = false;
      
      response->success = true;
      response->message = "成功清除检测结果";
      LOG_INFO("成功清除检测结果");
    } else {
      response->success = true;
      response->message = "未执行清除操作，清除标志为false";
      LOG_INFO("未执行清除操作，清除标志为false");
    }
  }
  catch (const std::exception& e) {
    response->success = false;
    response->message = "清除检测结果异常: " + std::string(e.what());
    LOG_ERROR("清除检测结果异常: %s", e.what());
  }
}

bool MarkerDetectNode::getCameraIntrinsics(int cam_id, int timeout_sec)
{
	// 等待服务可用
	if (!get_cam_intr_client_->wait_for_service(std::chrono::seconds(timeout_sec))) {
		LOG_ERROR("等待获取相机内参服务超时(%d秒)", timeout_sec);
		sensor_msgs::msg::CameraInfo empty_info;
		return false;
	}
	// 创建请求
	auto request = std::make_shared<custom_msgs_comm::srv::GetCamIntr::Request>();
	request->cam_id = cam_id;

	// 发送异步请求
	auto result_future = get_cam_intr_client_->async_send_request(request);

	// 等待结果
	auto status = result_future.wait_for(std::chrono::seconds(timeout_sec));
	if (status == std::future_status::ready) {
		// 获取结果
		auto result = result_future.get();
		if (result->success)
		{
			LOG_INFO("成功获取相机%d的内参", cam_id);
			//return result->camera_info;
			std::lock_guard<std::mutex> lock(camera_info_mutex_);

			// 更新相机内参矩阵
			camera_matrix_ = (cv::Mat_<double>(3, 3) <<
				result->camera_info.k[0], result->camera_info.k[1], result->camera_info.k[2],
				result->camera_info.k[3], result->camera_info.k[4], result->camera_info.k[5],
				result->camera_info.k[6], result->camera_info.k[7], result->camera_info.k[8]);

			// 更新畸变系数
			dist_coeffs_ = cv::Mat::zeros(1, result->camera_info.d.size(), CV_64F);
			for (size_t i = 0; i < result->camera_info.d.size(); i++)
			{
				dist_coeffs_.at<double>(0, i) = result->camera_info.d[i];
			}

			// 更新Marker检测器的相机内参
			base_detector_->setCameraIntrinsics(camera_matrix_, dist_coeffs_);
			camera_info_received_ = true;
			LOG_INFO("接收到相机内参 - fx: %.6f, fy: %.6f, cx: %.6f, cy: %.6f",
				result->camera_info.k[0], result->camera_info.k[4], result->camera_info.k[2], result->camera_info.k[5]);
			return true;
		}
		else
		{
			LOG_ERROR("获取相机%d内参失败: %s", cam_id, result->message.c_str());
			sensor_msgs::msg::CameraInfo empty_info;
			return false;
		}
	}
	else
	{
		LOG_ERROR("获取相机%d内参超时", cam_id);
		sensor_msgs::msg::CameraInfo empty_info;
		return false;
	}
}

void MarkerDetectNode::drawExtraDataResultImage(cv::Mat& res_image)
{
	if (res_image.empty())
		return;
	for (auto it : current_robot_pose_map_)
	{
		int arm_id = it.first;
		std::string robot_pose = "robot_pose_arm_" + std::to_string(arm_id) + ": (" +
			std::to_string(it.second.position.x) + ", " +
			std::to_string(it.second.position.y) + ", " +
			std::to_string(it.second.position.z) + ", " +
			std::to_string(it.second.orientation.x) + ", " +
			std::to_string(it.second.orientation.y) + ", " +
			std::to_string(it.second.orientation.z) + ", " +
			std::to_string(it.second.orientation.w) + ")";
		cv::putText(res_image, robot_pose,
			cv::Point(20, res_image.rows - 20 - arm_id * 20), cv::FONT_HERSHEY_SIMPLEX, 0.4,
			cv::Scalar(0, 255, 255), 1);
	}
}

/**
 * @brief 初始化头部电机角度订阅器
 */
void MarkerDetectNode::initHeadMotorAngleSubscriber() 
{
  // 初始化当前头部电机角度
  current_head_angles_.resize(2, 0.0);  // 默认为0

  head_base_transform_ = head_transform_ = handeyecalib::YawAngleToTransform(0.0);
  
  // 创建订阅器
  head_motor_angles_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/head_motor_angles", 10,
      std::bind(&MarkerDetectNode::headMotorAngleCallback, this, std::placeholders::_1));
  head_motor_angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("head_drv", 10);

  LOG_INFO("头部电机角度订阅器初始化完成");
}

/**
 * @brief 头部电机角度回调函数
 * @param msg 头部电机角度消息
 */
void MarkerDetectNode::headMotorAngleCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
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
  
  LOG_DEBUG("接收到头部电机角度 - 俯仰角: %.2f°, 偏航角: %.2f°", current_head_angles_[0], current_head_angles_[1]);
}

void MarkerDetectNode::calibThreadFunc()
{
	calib_thread_running_ = false;
	LOG_INFO("标定线程启动");
	//std::vector<double> yaw_angles{-20.0, 0.0, 20.0};
	//std::vector<double> pitch_angles{-20.0, -10.0, 0.0, 10.0, 20.0};
	//std::vector<double> pitch_angles{10.0, 0.0, 10.0};
	std::vector<std::vector<double>> head_angles{
		{-10.0, -10.0}, {0.0, -10.0}, {10.0, -10.0},  
		{10.0, 0.0}, {0.0, 0.0}, {-10.0, 0.0},  
		{-10.0, 10.0}, {0.0, 10.0}, {10.0, 10.0},
		{0.0, 0.0}};
	std::vector<std::vector<double>> marker_positions;
	std::vector<std::vector<double>> head_positions;
	std::vector<double> position;
	std::vector<double> head_position;
	for (size_t i = 0; i < head_angles.size(); i++)
	{
		rclcpp::sleep_for(std::chrono::milliseconds(100)); // 每100ms处理一次
		auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.push_back(head_angles[i][0]);
        msg.data.push_back(head_angles[i][1]);
        msg.data.push_back(0.0);
        head_motor_angles_pub_->publish(msg);
		rclcpp::sleep_for(std::chrono::seconds(5)); // 每5秒处理一次
		if (i == 0)
			rclcpp::sleep_for(std::chrono::seconds(5)); // 每5秒处理一次

		{ //拷贝检测数据
			std::lock_guard<std::mutex> lock(image_mutex_);
			position = {
						static_cast<double>(detection_res_->markers_info[0].position.x),
						static_cast<double>(detection_res_->markers_info[0].position.y),
						static_cast<double>(detection_res_->markers_info[0].position.z),
						static_cast<double>(detection_res_->markers_info[0].rotation[0]),
						static_cast<double>(detection_res_->markers_info[0].rotation[1]),
						static_cast<double>(detection_res_->markers_info[0].rotation[2])
					};
		}
		cv::Mat head_transform = handeyecalib::computeHeadPoseTransform(head_angles[i][0], head_angles[i][1], p2y_tx, p2y_ty, p2y_tz);
		head_position = handeyecalib::homogeneousMatrixToPose(head_transform);
		LOG_INFO("marker position: %f %f %f %f %f %f", position[0], position[1], position[2], position[3], position[4], position[5]);
		LOG_INFO("head position: %f %f %f %f %f %f", head_position[0], head_position[1], head_position[2], head_position[3], head_position[4], head_position[5]);
		marker_positions.push_back(position);
		head_positions.push_back(head_position);
	}
	// 计算偏航变换矩阵
	//head_transform_ = handeyecalib::computeYawTransform(marker_positions, yaw_angles);	
	//head_transform_.at<double>(2, 3) = 230.0;

	// 计算手眼标定矩阵
    handeyecalib::CalibRes result = handeyecalib::computeHandEyeCalibrationByCv(
                head_positions, marker_positions, true);
	head_transform_ = result.cam_to_base_transform;
	for (auto it = calib_result_map_.begin(); it != calib_result_map_.end(); ++it)
	{
		int arm_id = it->first;
		cv::Mat T_p_y = handeyecalib::computeHeadPoseTransform(0.0, 0.0, p2y_tx, p2y_ty, p2y_tz);
		cv::Mat cam_head = T_p_y * head_transform_;
		head_base_transform_ = calib_result_map_[arm_id]->cam_to_base_transform * cam_head.inv();
		break;
	}
	std::cout << "head_transform_: " << head_transform_ << std::endl;
	std::cout << "head_base_transform_: " << head_base_transform_ << std::endl;

	LOG_INFO("标定线程结束");
}

}  // namespace marker_detect_ros
