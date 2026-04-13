#include "elevator_rec.h"
#include "utils.h"
#include <iostream>
#include <chrono>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <elevator_rec_msgs/msg/elevator_recognition.hpp>
#include <elevator_rec_msgs/msg/elevator_recognition_array.hpp>
#include <custom_msgs_comm/msg/elevator_command.hpp>
#include <random>
#include <nav_msgs/msg/path.hpp>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iomanip>
#include <string>
#include <algorithm>
#include <vector>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Transform.h>
#include "opencv2/highgui.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include "bas_operate_ros/ros_comm_info.h"
#include "bas_operate_ros/param_utils.hpp"
#include "hand_eye_calib/calib_struct.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "calib_info_src/calib_info_server.h"
#include "log_system/log_macros.hpp"

class Elevator_Rec_Node : public rclcpp::Node
{
public:
    Elevator_Rec_Node(const rclcpp::NodeOptions &options, int cam_id);

private:
    void Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void CameraInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);  // 新增相机信息回调
    void Camera_Command_Callback(const custom_msgs_comm::msg::ElevatorCommand::SharedPtr msg);
    void publish(const std::vector<ButtonRecognition>& buttonResults, const std_msgs::msg::Header& header);
    void publish(const Detection& buttonResult, const std_msgs::msg::Header& header);

    // 从模型同名 .names 文件加载类别名称
    std::vector<std::string> loadClassNamesFromModel(const std::string& model_path);

    // 新增功能方法 - 解耦合设计
    void setup_camera_intrinsics();  // 设置相机内参
    void publish_static_tf();        // 发布静态TF
    geometry_msgs::msg::Point calculate_3d_position(const Detection& det);  // 计算3D坐标
    geometry_msgs::msg::Point transform_to_base_link(const geometry_msgs::msg::Point& camera_point);  // 坐标转换
    void initTopicNames();  // 使用parseCommInfo初始化话题名
    void initCalibParamHandler();  // 初始化标定参数处理器
    void calcMeanContrast(const cv::Mat& img, double& mean, double& contrast);//计算图像对比度
    bool isContrastImproved(const cv::Mat& img1, const cv::Mat& img2, double thr = 1.2);//判断两张图像对比度是否提升
    void saveButtonDetection(const ButtonRecognition& button_result, const cv::Mat& current_image);//保存按钮检测结果和图像
    bool isButtonPressed(const cv::Mat& current_image); //判断按钮是否被按下（基于灰度对比度）
    void processCameraCommand(const std::string& command, int current_floor, int target_floor); //处理相机指令
    void processWaitingCommand(const std::vector<ButtonRecognition>& current_results); //处理持续等待的指令
    bool getSysDat();  // 从参数服务器获取系统配置（包括相机对应的机械臂列表）
    bool getCamConfigInfo(SysConfig::CamConfigInfo& cam_info);  // 获取相机配置信息

private:
    std::string camera_type_;      // 相机类型
    int camera_id_;               // 相机ID，支持多相机配置
    int arm_id_;                 // 机械臂ID，用于获取对应的标定矩阵
    std::vector<int> arm_id_list_;  // 机械臂ID列表，支持多机械臂检测
    bool usecalib_;               // 是否使用标定数据变换位置坐标
    std::string color_image_topic_;      // rgb图像话题名称
    std::string depth_image_topic_;      // 深度图像话题名称
    std::string camera_info_topic_;     // 相机信息话题名称
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    bool camera_intrinsics_initialized_;  // 相机内参是否已初始化

    // 标定参数
    std::unique_ptr<handeyecalib::CalibRes> calib_result_;  // 标定结果
    rclcpp::SyncParametersClient::SharedPtr sys_config_client_;  // 参数服务器客户端

    ElevatorButtonRec elevator_rec_;
    cv::Mat depth_frame_;
    std::string engine_name_;
    std::string rec_engine_name_;
    std::string dict_path_;

    // 类别名称列表（从模型同名 .names 文件动态加载）
    std::vector<std::string> class_names_;

    // 新增成员变量
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped camera_to_base_tf_;
    
    rclcpp::Publisher<elevator_rec_msgs::msg::ElevatorRecognitionArray>::SharedPtr det_res_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr box_image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr res_image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;  // 新增相机信息订阅

    rclcpp::Subscription<custom_msgs_comm::msg::ElevatorCommand>::SharedPtr camera_command_sub_;//电梯指令消息
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_status_pub_;//状态消息
    rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr detection_pub_;//按钮坐标结果

    std::string camera_command_;
    int current_floor_;
    int target_floor_;
    std::string waiting_command_;  // 持续等待的命令
    std::mutex command_mutex_;  // 保护camera_command_和相关参数的互斥锁
    
    cv::Mat image_;
    // 新增：线程安全的检测结果存储
    std::vector<ButtonRecognition> latest_button_results_;
    std::mutex detection_mutex_;
    std_msgs::msg::Header latest_header_;
    
    // 新增：按钮状态检测相关成员变量
    ButtonRecognition saved_button_detection_;  // 保存的按钮检测结果
    cv::Mat saved_button_image_;               // 保存的按钮图像
    bool button_detection_saved_;              // 是否已保存按钮检测
    std::mutex button_save_mutex_;             // 按钮保存数据的互斥锁
    
    // 新增：按钮按下后的楼层到达检测
    bool button_pressed_;                      // 按钮是否已被按下
    bool monitoring_floor_arrival_;            // 是否正在监控楼层到达

    // 日志相关
    std::string log_project_path_;  // 根据camera_id生成的日志项目路径
};
