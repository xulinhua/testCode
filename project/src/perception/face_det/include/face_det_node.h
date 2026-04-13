#pragma once

#include "scrfd.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <message_filters/subscriber.h>
#include "bas_operate_ros/ros_comm_info.h"
#include "bas_operate_ros/param_utils.hpp"
#include "hand_eye_calib/calib_struct.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "calib_info_src/calib_info_server.h"
#include "log_system/log_macros.hpp"

class FaceDet_Node : public rclcpp::Node
{
public:
    FaceDet_Node(const rclcpp::NodeOptions &options, int cam_id);

private:
    void Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void CameraInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void publish(const std::vector<FaceDetection>& detections, const std_msgs::msg::Header& header);
    
    // 新增功能方法
    void setup_camera_intrinsics();  // 设置相机内参
    geometry_msgs::msg::Point calculate_3d_position(const FaceDetection& det);  // 计算3D坐标
    void initTopicNames();  // 使用parseCommInfo初始化话题名
    void initCalibParamHandler();  // 初始化标定参数处理器
    void detect_face_count_change(const std::vector<FaceDetection>& face_results);  // 检测人脸数量变化
    void detect_multi_face_interaction(const std::vector<FaceDetection>& face_results);  // 检测多人互动
    bool getSysDat();  // 从参数服务器获取系统配置（包括相机对应的机械臂列表）
    bool getCamConfigInfo(SysConfig::CamConfigInfo& cam_info);  // 获取相机配置信息

private:
    std::string camera_type_;      // 相机类型
    int camera_id_;                 // 相机ID，支持多相机配置
    int arm_id_;                   // 机械臂ID，用于获取对应的标定矩阵
    std::vector<int> arm_id_list_;  // 机械臂ID列表，支持多机械臂检测
    std::string color_image_topic_;      // rgb图像话题名称
    std::string depth_image_topic_;      // 深度图像话题名称
    std::string camera_info_topic_;     // 相机信息话题名称
    
    SCRFD scrfd_;
    cv::Mat color_frame_;
    cv::Mat depth_frame_;
    std::string engine_name_;
    
    // 相机内参相关
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    bool camera_intrinsics_initialized_;

    // 标定参数
    bool usecalib_;                       // 是否使用标定模式
    std::unique_ptr<handeyecalib::CalibRes> calib_result_;  // 标定结果
    rclcpp::SyncParametersClient::SharedPtr sys_config_client_;  // 参数服务器客户端
    
    // 人脸数量检测相关
    int previous_face_count_;
    int previous_nearby_face_count_;  // 在1.5米范围内的人脸数量
    int previous_multi_face_count_;  // 达到多人互动阈值的人脸数量
    int multi_face_threshold_;       // 多人互动触发阈值（默认3）
    float multi_face_distance_;      // 多人互动距离范围（默认3米）
    bool multi_face_triggered_;      // 是否已触发多人互动
    bool greeting_triggered_;         // 是否已触发greeting指令
    
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr face_res_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr face_image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr face_text_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr interaction_pub_;  // 交互指令发布者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // 日志相关
    std::string log_project_path_;  // 根据camera_id生成的日志项目路径
};
