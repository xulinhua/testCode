#pragma once

#include "hand_pipeline.h"
#include "gesture_detector.h"
#include "hand_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include <mutex>
#include "log_system/log_macros.hpp"
#include "bas_operate_ros/ros_comm_info.h"
#include "bas_operate_ros/param_utils.hpp"
#include "hand_eye_calib/calib_struct.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "calib_info_src/calib_info_server.h"
#include "sys_info_src/sys_info_server.h"

struct Intrinsics {
    float fx;
    float fy;
    float cx;
    float cy;
};

class HandGestureRecNode : public rclcpp::Node
{
public:
    HandGestureRecNode(const rclcpp::NodeOptions &options, int cam_id);

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void publish_gesture_result(const GestureInfo& gesture_info, const std_msgs::msg::Header& header, const HandResult& hand_result, const geometry_msgs::msg::Point& camera_3d);
    void visualize_hand(cv::Mat& image, const HandResult& hand_result, const GestureInfo& gesture_info, const geometry_msgs::msg::Point& camera_3d);

    // 新增功能方法 - 解耦合设计
    void setup_camera_intrinsics();  // 设置相机内参
    geometry_msgs::msg::Point calculate_3d_position(const HandResult& hand_result, const GestureInfo& gesture_info);  // 计算3D坐标
    void initTopicNames();  // 使用parseCommInfo初始化话题名
    void initCalibParamHandler();  // 初始化标定参数处理器
    void update_wave_thresholds_2d(int image_width, int image_height);  // 根据2D图像尺寸更新挥手阈值
    bool getSysDat();  // 从参数服务器获取系统配置（包括相机对应的机械臂列表）
    bool getCamConfigInfo(SysConfig::CamConfigInfo& cam_info);  // 获取相机配置信息

private:
    // 日志相关
    std::string log_project_path_;

    // 相机相关
    std::string camera_type_;      // 相机类型
    int camera_id_;               // 相机ID，支持多相机配置
    int arm_id_;                 // 机械臂ID，用于获取对应的标定矩阵
    std::vector<int> arm_id_list_;  // 机械臂ID列表，支持多机械臂检测
    std::string color_image_topic_;      // rgb图像话题名称
    std::string depth_image_topic_;      // 深度图像话题名称
    std::string camera_info_topic_;     // 相机信息话题名称
    std::string gesture_result_topic_;   // 手势结果话题
    std::string visualization_topic_;     // 可视化话题

    // 模型路径
    std::string detection_engine_path_;
    std::string pose_engine_path_;

    // 相机内参
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    bool camera_intrinsics_initialized_;  // 相机内参是否已初始化

    // 标定参数
    bool usecalib_;                       // 是否使用标定模式
    std::unique_ptr<handeyecalib::CalibRes> calib_result_;  // 标定结果
    rclcpp::SyncParametersClient::SharedPtr sys_config_client_;  // 参数服务器客户端

    // 手势检测器
    std::unique_ptr<HandPipeline> hand_pipeline_;
    std::unique_ptr<GestureDetector> gesture_detector_;

    // ROS通信
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_result_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_res_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // 参数
    bool enable_visualization_;
    int min_wave_pairs_;
    bool use_real_3d_;  // 是否使用真实3D坐标进行手势识别

    // 2D模式挥手阈值动态更新
    float initial_max_wave_y_movement_;  // 初始Y轴最大允许波动范围（像素）
    float initial_min_wave_amplitude_;   // 初始最小挥手幅度（像素）
    bool wave_thresholds_updated_;       // 标记阈值是否已根据图像尺寸更新

    // 深度图像
    cv::Mat depth_frame_;

    // Wave手势指令相关
    bool wave_triggered_;               // 标记Wave指令是否已触发
    int wave_cooldown_frames_;         // Wave指令冷却帧数（约1秒）
    int wave_cooldown_counter_;         // 当前冷却计数器
};
