#ifndef ARUCO_DETECT_ROS__ARUCO_DETECT_NODE_HPP_
#define ARUCO_DETECT_ROS__ARUCO_DETECT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>  // 添加CameraInfo消息类型
#include <geometry_msgs/msg/pose_stamped.hpp>  // 添加PoseStamped消息类型（保留用于兼容性）
#include <vision_msgs/msg/detection2_d.hpp>  // 添加Detection2D消息类型
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
// 添加互斥锁头文件
#include <mutex>
#include <thread>  // 添加线程支持
#include <atomic>  // 添加原子操作支持

// 添加服务消息类型
#include "custom_msgs_comm/srv/get_marker_detection.hpp"
#include "marker_detect_ros/keyboard_handler.hpp"  // 添加键盘处理头文件
#include "custom_msgs_comm/srv/get_marker_src_image.hpp"
#include "custom_msgs_comm/srv/get_markers_info.hpp"  // 添加GetMarkersInfo服务
#include "custom_msgs_comm/srv/get_cam_intr.hpp"  // 添加GetCamIntr服务
#include "custom_msgs_comm/msg/robot_std_pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "hand_eye_calib/calib_struct.hpp"  // 包含CamCalibInfoList定义
#include <std_srvs/srv/set_bool.hpp>  // 添加SetBool服务消息类型
#include "bas_sys_config_ros/calib_param_handler.h"  // 使用统一标定参数处理器头文件
#include "bas_operate_ros/ros_comm_info.h"  // 添加RosCommInfo结构体定义

// 包含共用数据结构和基类
#include "comm_alg/marker_detect_base.hpp"

// 添加模块状态发布所需头文件
#include "bas_operate_ros/status_node.h"

// 前向声明
namespace aruco_alg {
class ArucoDetector;
}
namespace chessboard_alg {
class ChessboardPoseDetector;
}

namespace visualization {
class VisualizationMgr;
}

namespace handeyecalib {
class CalibRes;
}

namespace marker_detect_ros
{

/**
 * @brief 标记检测ROS2节点
 * 
 * 该节点订阅图像话题，使用marker_alg模块检测标记，
 * 并发布检测结果
 */
class MarkerDetectNode : public basros::StatusNodeBase
{
public:
  /**
   * @brief 构造函数
   */
  explicit MarkerDetectNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions(), int ID = 0);

  /**
   * @brief 析构函数
   */
  ~MarkerDetectNode();

private:
  /**
   * @brief 标记检测
   * @return 是否成功
   */
  bool markerDetect();

  /**
   * @brief 检测线程主函数
   */
  void detectionThreadFunc();
  
  /**
   * @brief 停止检测线程
   */
  void stopDetectionThread();

  /**
   * @brief 获取图像
   * @param image 图像消息
   */
  bool getImage(cv::Mat & image);

  /**
   * @brief 图像消息回调函数
   * @param msg 图像消息
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief 深度图像消息回调函数
   * @param msg 深度图像消息
   */
  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief 相机内参消息回调函数
   * @param msg 相机内参消息
   */
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  void robotPoseCallback(const custom_msgs_comm::msg::RobotStdPose::SharedPtr msg);

  /**
   * @brief Aruco检测服务回调函数
   * @param request 服务请求
   * @param response 服务响应
   */
  void markerDetectionService(
    const std::shared_ptr<custom_msgs_comm::srv::GetMarkerDetection::Request> request,
    std::shared_ptr<custom_msgs_comm::srv::GetMarkerDetection::Response> response);
    
  /**
   * @brief Aruco源图像服务回调函数
   * @param request 服务请求
   * @param response 服务响应
   */
  void getMarkerSrcImgService(
    const std::shared_ptr<custom_msgs_comm::srv::GetMarkerSrcImage::Request> request,
    std::shared_ptr<custom_msgs_comm::srv::GetMarkerSrcImage::Response> response);

  /**
   * @brief Marker标记信息服务回调函数
   * @param request 服务请求
   * @param response 服务响应
   */
  void getMarkersInfoService(
    const std::shared_ptr<custom_msgs_comm::srv::GetMarkersInfo::Request> request,
    std::shared_ptr<custom_msgs_comm::srv::GetMarkersInfo::Response> response);

  /**
   * @brief 清除检测结果服务回调函数
   * @param request 服务请求
   * @param response 服务响应
   */
  void clearDetectionResultService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief 获取相机内参服务请求函数
   * @param cam_id 相机ID
   * @param timeout_sec 超时时间（秒）
   * @return bool 是否成功获取相机内参
   */
  bool getCameraIntrinsics(int cam_id, int timeout_sec = 3);  ///< 获取相机内参信息

  /**
   * @brief 初始化参数
   */
  void initParameters();

  /**
   * @brief 初始化话题名称
   */
  void initTopicNames();

  /**
   * @brief 初始化发布器和订阅器
   */
  void initPubSub();

  /**
   * @brief 初始化服务服务器
   */
  void initServices();

  /**
  * @brief 初始化相机内参
  * @return 是否初始化成功
  */
  bool initCameraIntrinsics(const std::string& Intrinsics_config_file);

  /**
   * @brief 加载标定结果文件
   */
   #if 0
  bool loadCalibrationResult(const std::string& load_file) ;
   #endif

  /**
   * @brief 初始化额外数据
   * @return 是否初始化成功
   */
  bool initExtraData();

  /**
   * @brief 绘制额外数据结果图像
   * @return void
   */
  void drawExtraDataResultImage(cv::Mat& res_image);
  
  /**
   * @brief 初始化可视化管理器
   */
  void initializeVisualizationManagers();
  
  /**
   * @brief 初始化显示定时器
   */
  void initializeTimer();

  void transMarkerToCalibRes(const comm_alg::DetectionResult& result, cv::Mat& res_image);

/**
 * @brief 初始化头部电机角度订阅器
 */
void initHeadMotorAngleSubscriber();

/**
 * @brief 头部电机角度回调函数
 * @param msg 头部电机角度消息
 */
void headMotorAngleCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  /**
   * @brief 标定线程主函数
   */
  void calibThreadFunc();

private: 
    /**
   * @brief 键盘监听定时器回调函数
   */
  void keyboardTimerCallback();
  
  /**
   * @brief 重新加载机器人位姿偏移值
   */
  void loadOffsetRobotPose();
  rclcpp::TimerBase::SharedPtr keyboard_timer_; // 用于键盘监听的定时器
  std::unique_ptr<KeyboardHandler> keyboard_handler_; // 键盘处理器
  std::atomic<bool> ctrl_l_pressed_; // Ctrl+L按键状态
private:
  
  void initParameterClient();//初始化参数客户端   
  bool getSysDat();//读取系统配置参数
  bool getCamConfigInfo(SysConfig::CamConfigInfo& cam_info);//根据cam_id获取指定相机的配置信息

  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  void initCalibParams();//初始化标定参数
  void calibDatChangedCallback(const handeyecalib::ArmCalibInfo& calib_data);
  
  rclcpp::SyncParametersClient::SharedPtr sys_config_client_; // 系统配置客户端
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;  // 参数事件订阅器
  std::string sys_config_node_name_;
  rclcpp::Node::SharedPtr parameters_node_;
  bool parameters_changed_;
  std::mutex parameters_mutex_;                   ///< 保护parameters_changed_的互斥锁
  rclcpp::TimerBase::SharedPtr parameters_timer_; // 用于参数刷新的定时器
private:
  // ========== 标定参数同步机制（推拉结合） ==========
  
  /**
   * @brief 初始化标定参数同步机制
   */
  void initCalibSyncMechanism();
  
  /**
   * @brief 强制从参数服务器同步标定数据（启动时调用）
   * @return true-成功, false-失败
   */
  bool syncCalibDataFromServer();
  
  /**
   * @brief 参数事件回调（推模式）
   */
  void onParameterEventDebounce(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
  
  /**
   * @brief 版本号检查定时器回调（拉模式-兜底）
   */
  void versionCheckTimerCallback();
  
  /**
   * @brief 获取参数服务器上的标定数据版本号
   * @return 版本号，-1表示获取失败
   */
  int64_t getServerCalibVersion();
  
  /**
   * @brief 更新本地标定数据
   * @param cam_calib_info 从服务器获取的完整标定数据
   */
  void updateLocalCalibData(const handeyecalib::CamCalibInfo& cam_calib_info);
  
  // 标定同步相关成员
  rclcpp::TimerBase::SharedPtr version_check_timer_;  // 版本号检查定时器
  std::atomic<int64_t> local_calib_version_{-1};      // 本地标定数据版本号（-1表示未初始化）
  std::mutex calib_sync_mutex_;                       // 标定同步互斥锁
  const std::chrono::seconds VERSION_CHECK_INTERVAL{3}; // 版本号检查间隔（3秒）
  
  // 防抖机制
  rclcpp::TimerBase::SharedPtr debounce_timer_;       // 防抖定时器
  std::atomic<bool> pending_update_{false};           // 待更新标志
  const std::chrono::milliseconds DEBOUNCE_DELAY{100}; // 防抖延迟100ms
private:
  // ROS订阅器和发布器
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;  // 深度图像订阅器
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;  // 添加相机内参订阅器
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_result_pub_; // 添加结果图像发布器映射
  rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr detection_pub_;  // 修改为Detection2D类型
  std::map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> calib_result_pub_map_;  // 添加标定结果发布器映射;
  rclcpp::Subscription<custom_msgs_comm::msg::RobotStdPose>::SharedPtr robot_pose_sub_;  // 添加机器人位姿订阅器

  // ROS服务服务器
  rclcpp::Service<custom_msgs_comm::srv::GetMarkerDetection>::SharedPtr marker_detection_service_;
  rclcpp::Service<custom_msgs_comm::srv::GetMarkerSrcImage>::SharedPtr marker_source_image_service_;  ///< 标记源图像服务服务器
  rclcpp::Service<custom_msgs_comm::srv::GetMarkersInfo>::SharedPtr markers_info_service_;  ///< 标记标记信息服务服务器
  
  // ROS服务客户端
  rclcpp::Client<custom_msgs_comm::srv::GetCamIntr>::SharedPtr get_cam_intr_client_;  ///< 获取相机内参服务客户端

  // 回调组
  rclcpp::CallbackGroup::SharedPtr callback_group_image_;
  rclcpp::CallbackGroup::SharedPtr callback_group_service_;
 
private:
  // marker检测器
  std::unique_ptr<comm_alg::MarkerDetectorBase> base_detector_;  ///< 标记检测器
  std::unique_ptr<comm_alg::DetectionResult> detection_res_;  ///< 检测结果
  comm_alg::MarkerType marker_type_; ///< 标记类型
  std::vector<double> aruco_length_List_;       ///< ArUco标记边长
  std::vector<int> aruco_dict_type_List_;        ///< ArUco字典类型
  cv::Size chessboard_size_;                     ///< 棋盘格尺寸（内角点数量）
  float chessboard_square_size_;                ///< 棋盘格方格尺寸（米）
  std::string marker_result_type_;                ///< 标记检测结果类型
  
  // 可视化相关
  std::unique_ptr<visualization::VisualizationMgr> visualizer_src_;  ///< 可视化管理器源图像窗口
  std::unique_ptr<visualization::VisualizationMgr> visualizer_res_;  ///< 可视化管理器结果图像窗口

  int cam_id_;                              ///< marker_detect ID，支持多相机检测
  int arm_id_;                              ///< 机械臂ID，支持多机械臂检测
  std::vector<int> arm_id_list_;            ///< 机械臂ID列表，支持多机械臂检测
  bool sys_config_loaded_;                  ///< 系统配置是否加载成功
  rclcpp::TimerBase::SharedPtr init_timer_; // 用于延迟初始化的定时器
  rclcpp::TimerBase::SharedPtr init_show_timer_; // 用于显示延迟初始化的定时器

  cv::Mat image_;                          ///< 输入图像
  cv::Mat depth_image_;                    ///< 输入深度图像
  cv::Mat service_request_image_;          ///< 服务请求时的图像
  cv::Mat image_proc_;                     ///< 处理的图像
  std::mutex image_mutex_;                 ///< 保护image_的互斥锁
  bool result_is_ready_;                   ///< 检测结果是否就绪
  bool image_proc_is_ready_;               ///< 处理图像是否就绪
  cv::Mat camera_matrix_;                  ///< 相机内参矩阵
  cv::Mat dist_coeffs_;                    ///< 畸变系数
  bool camera_info_received_;              ///< 是否已接收到相机内参
  std::mutex camera_info_mutex_;           ///< 保护相机内参的互斥锁

  // 线程相关成员
  std::thread detection_thread_;           ///< ArUco检测线程
  std::atomic<bool> thread_running_;       ///< 检测线程运行标志

  // 参数
  std::string image_topic_;                       ///< 图像话题名称
  std::string depth_image_topic_;                 ///< 深度图像话题名称
  std::string detection_result_topic_;            ///< ArUco检测结果话题名称
  std::string camera_info_topic_;                 ///< 相机内参话题名称
  std::string camera_info_service_name_;          ///< 相机内参服务名称
  std::string detection_service_name_;            ///< marker检测服务话题名称
  std::string markers_info_service_name_;    ///< marker标记信息服务话题名称
  std::string image_result_topic_;                ///< 结果图像话题名称
  std::string marker_src_img_service_name_;        ///< 标记源图像服务话题名称
  std::string marker_clear_service_name_;          ///< 清除缓存服务话题名称
  
  std::map<int, std::string> calib_result_topic_map_;///< 标定结果话题名称
  std::map<int, std::unique_ptr<handeyecalib::CalibRes>> calib_result_map_; ///< 标定结果指针映射
  // 机器人位姿
  std::string robot_pose_topic_; ///< 机器人位姿话题名称
  std::map<int, geometry_msgs::msg::Pose> current_robot_pose_map_; ///< 当前机械臂位姿
  
private:
  // 头部电机角度相关变量
  std::vector<double> current_head_angles_;           ///< 当前头部电机角度 [pitch, yaw]
  std::mutex head_angles_mutex_;                      ///< 头部角度互斥锁
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_motor_angles_sub_;  ///< 头部电机角度订阅器
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_motor_angles_pub_;  ///< 头部电机角度订阅器
  std::thread calib_thread_;           ///< 标定线程
  std::atomic<bool> calib_thread_running_;  ///< 标定线程运行标志
  cv::Mat head_transform_;                      ///< 头部变换矩阵
  cv::Mat head_base_transform_;                      ///< 头部到机器人变换矩阵

private: 
  std::string frame_id_;                         ///< 图像帧ID
  bool usecalib_;                                ///< 是否使用标定模式，默认false
  bool trans_marker_mm_;                          ///< 是否将marker世界坐标转换为mm
  bool show_src_image_;                          ///< 是否显示源图像
  bool show_result_image_;                       ///< 是否显示结果图像
  bool image_callback_invoked_;                  ///< 是否进入图像回调函数
  std::string src_image_dir_; /// 图像保存路径
};

}  // namespace marker_detect_ros

#endif  // MARKER_DETECT_ROS__MARKER_DETECT_NODE_HPP_
