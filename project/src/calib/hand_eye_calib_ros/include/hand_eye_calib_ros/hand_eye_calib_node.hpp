#ifndef HAND_EYE_CALIB_ROS__HAND_EYE_CALIB_NODE_HPP_
#define HAND_EYE_CALIB_ROS__HAND_EYE_CALIB_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>  // 添加Bool消息头文件
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>  // 添加服务头文件
#include "vision_msgs/msg/detection2_d.hpp"  // 添加Detection2D消息头文件
#include "custom_msgs_comm/srv/project_control.hpp"  // 添加自定义服务头文件
#include "custom_msgs_comm/srv/get_marker_detection.hpp"  // 添加获取Aruco检测结果的服务头文件
#include "custom_msgs_comm/srv/get_marker_src_image.hpp"  // 添加获取标定源图像的服务头文件
#include "custom_msgs_comm/srv/get_calibration_points.hpp"  // 添加获取标定点数据的服务头文件
#include "custom_msgs_comm/srv/get_robot_pose.hpp"  // 添加获取机械臂位姿的服务头文件
#include "custom_msgs_comm/msg/robot_std_pose.hpp"
#include "sensor_msgs/msg/image.hpp"  // 添加图像消息头文件
#include "std_msgs/msg/float64_multi_array.hpp"  // 添加Float64MultiArray消息头文件
#include "custom_msgs_comm/msg/bool_stamped.hpp"  // 添加BoolStamped消息头文件

// 添加cv_bridge头文件以支持CvImagePtr类型
#include <cv_bridge/cv_bridge.h>

// 包含hand_eye_calib库的头文件
#include "hand_eye_calib/calib_robot_pos_mgr.hpp"
#include "hand_eye_calib/calib_config.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "hand_eye_calib/calib_data_collector.hpp"

// 包含标定参数处理器头文件
#include "bas_operate_ros/param_utils.hpp"
#include "bas_sys_config_ros/calib_param_handler.h"

// 添加模块状态发布所需头文件
#include "bas_operate_ros/status_node.h"

namespace handeyecalib_ros {

/**
 * @class HandEyeCalibNode
 * @brief ROS节点类，用于封装hand_eye_calib功能
 * 
 * 该类实现了ROS接口，用于与marker_detect_ros和机械臂项目进行通信，
 * 控制标定流程并调用hand_eye_calib库的功能。
 */
class HandEyeCalibNode : public basros::StatusNodeBase {
public:
    /**
     * @brief 构造函数
     * @param options ROS节点选项
     */
    explicit HandEyeCalibNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~HandEyeCalibNode();

private:
    // 订阅器
    rclcpp::Subscription<custom_msgs_comm::msg::RobotStdPose>::SharedPtr robot_pose_sub_;    ///< 机械臂位姿订阅器
    rclcpp::Subscription<custom_msgs_comm::msg::BoolStamped>::SharedPtr robot_run_state_topic_sub_;            ///< 机械臂状态订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_ready_state_sub_;       ///< 机械臂准备状态订阅器

    // 发布器
    rclcpp::Publisher<custom_msgs_comm::msg::RobotStdPose>::SharedPtr robot_target_pub_;     ///< 机械臂目标位姿发布器

    // 服务客户端
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr robot_control_client_;   ///< 机械臂控制服务客户端
    rclcpp::Client<custom_msgs_comm::srv::GetCalibrationPoints>::SharedPtr get_calibration_points_client_;   ///< 获取标定点数据服务客户端
    rclcpp::Client<custom_msgs_comm::srv::GetMarkerDetection>::SharedPtr detect_res_service_client_;   ///< Marker检测结果获取服务客户端
    rclcpp::Client<custom_msgs_comm::srv::GetMarkerSrcImage>::SharedPtr calib_src_img_client_;   ///< 标定源图像请求服务客户端
    rclcpp::Client<custom_msgs_comm::srv::GetRobotPose>::SharedPtr get_robot_pose_client_;   ///< 获取机械臂位姿服务客户端
    
    // 服务服务器 (注释掉不再使用的启动标定服务服务器)
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_calib_service_server_;       ///< 启动标定服务服务器
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;  ///< 定时器，用于周期性检查状态
    rclcpp::TimerBase::SharedPtr auto_start_timer_;  ///< 自动启动标定定时器

    // hand_eye_calib库对象
    handeyecalib::CalibRobotPosMgr robot_pos_mgr_;  ///< 机械臂坐标数据管理器
    handeyecalib::CalibConfig calib_config_;        ///< 标定配置管理器

    // 配置参数
    bool use_service_for_points_;                  ///< 是否通过服务通讯获取标定点(true)或从本地文件加载(false)

    // 状态变量
    bool calib_started_;              ///< 标定是否已启动
    bool robot_ready_;                ///< 机械臂是否准备就绪
    bool robot_moved_;                ///< 机械臂是否移动到位
    int current_point_index_;         ///< 当前标定点索引
    int successful_points_count_;     ///< 成功采集的标定点数量
    geometry_msgs::msg::PoseStamped current_robot_pose_;     ///< 当前机械臂位姿
    geometry_msgs::msg::PoseStamped current_aruco_marker_;   ///< 当前Aruco标记数据
    sensor_msgs::msg::Image current_calib_src_img_;      ///< 当前标定源图像数据
    sensor_msgs::msg::Image current_calib_render_img_;   ///< 当前标定渲染图像数据
    
    // 标定点数据加载状态
    bool calibration_points_load_success_;  ///< 标定点数据加载是否成功

    // 数据存储
    std::vector<std::vector<double>> robot_poses_;      ///< 收集的机器人位姿数据
    std::vector<std::vector<double>> marker_positions_; ///< 收集的标记位置数据
    
    // 新增的私有成员变量
    handeyecalib::CalibDataCollector data_collector_;   ///< 数据收集器
    bool is_calibrated_;                                ///< 是否已完成标定
    cv::Mat camera_to_base_transform_;                  ///< 相机到基座的变换矩阵
    cv::Mat base_to_camera_transform_;                  ///< 基座到相机的变换矩阵
    std::string config_file_;                           ///< 配置文件路径
    std::string source_data_dir_;                       ///< 源数据目录
    std::string output_data_dir_;                       ///< 输出数据目录
    int camID_;                                         ///< 相机ID
    int armID_;                                         ///< 机械臂ID 
    bool eye_on_hand_;                                  ///< 是否为眼在手上
    
    // 头部电机角度相关变量
    std::vector<double> current_head_angles_;           ///< 当前头部电机角度 [pitch, yaw]
    std::mutex head_angles_mutex_;                      ///< 头部角度互斥锁
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_motor_angles_sub_;  ///< 头部电机角度订阅器
    
    // 标定参数处理器
    std::unique_ptr<RosComm::ArmCalibParamHandler> calib_param_handler_;  ///< 标定参数处理器
    rclcpp::SyncParametersClient::SharedPtr sys_config_client_; // 系统配置客户端
    std::string sys_config_node_name_;
    rclcpp::Node::SharedPtr parameters_node_;

private:

    /**
     * @brief 初始化参数
     */
    void initParameters();

    /**
     * @brief 初始化订阅器
     */
    void initSubscribers();

    /**
     * @brief 初始化发布器
     */
    void initPublishers();

    /**
     * @brief 初始化服务客户端
     */
    void initServiceClients();

    /**
     * @brief 初始化服务服务器
     */
    void initServiceServers();

    /**
     * @brief 初始化参数服务器客户端
     */
    void initParametersClient();

    /**
     * @brief 初始化定时器
     */
    void initTimer();
    
    /**
     * @brief 初始化标定点数据
     */
    bool initCalibrationPoints();
    
    /**
     * @brief 检查标定点数量是否满足要求
     * @param point_count 标定点数量
     * @return 是否满足要求
     */
    bool checkCalibrationPointsCount(size_t point_count);

    /**
     * @brief 请求标定点数据
     */
    void requestCalibrationPoints();

    /**
     * @brief 自动启动标定流程
     */
    void autoStartCalibration();

    // /**
    //  * @brief Aruco标记回调函数
    //  * @param msg Aruco标记消息
    //  */
    // void arucoMarkerCallback(const vision_msgs::msg::Detection2D::SharedPtr msg);

    /**
     * @brief 机械臂位姿回调函数
     * @param msg 机械臂位姿消息
     */
    void robotPoseCallback(const custom_msgs_comm::msg::RobotStdPose::SharedPtr msg);

    /**
     * @brief 机械臂状态回调函数
     * @param msg 机械臂状态消息
     */
    void robotStatusCallback(const custom_msgs_comm::msg::BoolStamped::SharedPtr msg);

    /**
     * @brief 机械臂准备状态回调函数
     * @param msg 机械臂准备状态消息
     */
    void robotReadyStateCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief 定时器回调函数
     */
    void timerCallback();

    /**
     * @brief 发送启动标定命令到机械臂
     */
    void sendStartCalibCommandToRobot();
    
    /**
     * @brief 发送机械臂目标位姿
     * @param pose 目标位姿
     */
    void sendRobotTargetPose(const handeyecalib::RobotPoseData& pose);

    /**
     * @brief 发送标定结束命令
     */
    void sendEndCalibCommand();

    /**
     * @brief 请求Aruco检测结果
     */
    void requestArucoDetectionResult();
    
    /**
     * @brief 请求机械臂位姿数据
     */
    void requestRobotPose();
    
    /**
     * @brief 请求图像
     */
    void requestImage();

    /**
     * @brief 检查机械臂位姿数据是否有效
     * @param pose 机械臂位姿数据
     * @return 是否有效
     */
    bool isRobotPoseValid(const geometry_msgs::msg::PoseStamped& pose);

    /**
     * @brief 执行标定计算
     */
    void performCalibration();
    
    /**
     * @brief 分析每个标定点的重投影误差
     * @param robot_poses 机器人位姿列表
     * @param marker_positions 标记位置列表
     * @param cam_to_base_transform 相机到基座的变换矩阵
     */
    void analyzePerPointReprojectionError(
        const std::vector<std::vector<double>>& robot_poses,
        const std::vector<std::vector<double>>& marker_positions,
        const cv::Mat& cam_to_base_transform);

    /**
     * @brief 保存标定数据点
     */
    void saveCalibrationDataPoint();    
    /**
     * @brief 移动到下一个标定点
     */
    void moveToNextPoint();
    
    /**
     * @brief 保存标定源图像到文件
     * @param cv_ptr OpenCV图像指针
     * @param point_index 标定点索引
     */
    void saveCalibSourceImage(const cv_bridge::CvImagePtr& cv_ptr, int point_index);
    
    /**
     * @brief 保存标定渲染图像到文件
     * @param cv_ptr OpenCV图像指针
     * @param point_index 标定点索引
     */
    void saveCalibRenderImage(const cv_bridge::CvImagePtr& cv_ptr, int point_index);
    
    /**
     * @brief 标定数据更新回调函数
     * @param calib_data 更新的标定数据
     */
    void calibDatChangedCallback(const handeyecalib::ArmCalibInfo& calib_data);
    
    /**
     * @brief 初始化头部电机角度订阅器
     */
    void initHeadMotorAngleSubscriber();
    
    /**
     * @brief 头部电机角度回调函数
     * @param msg 头部电机角度消息
     */
    void headMotorAngleCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace handeyecalib_ros

#endif  // HAND_EYE_CALIB_ROS__HAND_EYE_CALIB_NODE_HPP_