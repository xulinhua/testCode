#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "yolo_seg.h"
#include "foundationpose_alg/foundationpose.hpp"
#include "foundationpose_alg/mesh_loader.hpp"
#include "trt_core/trt_core.h"
#include "bas_operate_ros/ros_comm_info.h"
#include "bas_operate_ros/param_utils.hpp"
#include "hand_eye_calib/calib_struct.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "log_system/log_macros.hpp"
#include "sys_info_src/sys_info_server.h"

class PoseEstNode : public rclcpp::Node
{
public:
    PoseEstNode(const rclcpp::NodeOptions& options, int cam_id);
    ~PoseEstNode();

private:
    // === Callbacks ===
    void Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void CameraInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    // === Initialization ===
    void initTopicNames();
    void initCalibParamHandler();
    bool getSysDat();
    bool getCamConfigInfo(SysConfig::CamConfigInfo& cam_info);

    // 从模型同名 .names 文件加载类别名称
    static std::vector<std::string> loadClassNamesFromModel(const std::string& model_path);

    // === Processing ===
    void processFrame(const cv::Mat& color_frame, cv::Mat& vis_image);
    void publishPose(const Eigen::Matrix4f& pose_cam, const std_msgs::msg::Header& header);

    // === Track后验校验 ===
    bool validateTrackedPose(const Eigen::Matrix4f& pose, const cv::Mat& depth_float,
                             float max_depth_error, float min_visible_ratio);

    // === YOLO segmentation lifecycle (显存管理) ===
    void loadSegDetector();
    void releaseSegDetector();
    bool isSegDetectorLoaded() const { return seg_detector_ != nullptr; }

    // === Scorer core lifecycle (显存管理: Track不需要scorer) ===
    void loadScorerCore();
    void releaseScorerCore();

    // === 模型重建 (Register后重建以释放大batch context) ===
    void rebuildForTracking();

    // === State machine ===
    enum class State { IDLE, REGISTERING, TRACKING };
    State state_;

    // === Parameters ===
    int camera_id_;
    int arm_id_;
    std::vector<int> arm_id_list_;
    std::string camera_type_;
    std::string color_image_topic_;
    std::string depth_image_topic_;
    std::string camera_info_topic_;
    bool usecalib_;

    // FoundationPose model parameters
    std::string refiner_engine_path_;
    std::string scorer_engine_path_;
    std::string mesh_path_;
    std::string target_name_;

    // YOLO segmentation parameters
    std::string seg_engine_path_;
    std::string target_class_name_;      // 目标类别名称（空字符串表示自动选择置信度最高的）
    std::vector<std::string> seg_class_names_;  // 分割模型的类别名称列表（从 .names 文件加载）
    float seg_conf_threshold_;
    float seg_iou_threshold_;

    // Track validation parameters
    float track_max_depth_error_;    // 最大允许深度偏差(m)
    int track_fail_reset_count_;     // 连续Track失败多少次后回到IDLE
    int reinit_period_ms_;           // Track多久后强制重新Register防漂移(ms)，0=禁用
    float track_min_visible_ratio_;  // 投影点中深度一致的比例阈值(0~1)

    // Track state
    int track_fail_count_;          // 连续Track失败计数
    Eigen::Matrix4f last_valid_pose_; // 上一次有效位姿（用于平移量校验）
    std::chrono::steady_clock::time_point tracking_start_time_; // Track开始时间（防漂移计时）

    // === Camera intrinsics ===
    float fx_, fy_, cx_, cy_;
    bool camera_intrinsics_initialized_;
    Eigen::Matrix3f intrinsic_;

    // === Calibration ===
    std::unique_ptr<handeyecalib::CalibRes> calib_result_;
    rclcpp::SyncParametersClient::SharedPtr sys_config_client_;

    // === Algorithm instances ===
    std::unique_ptr<YoloSeg> seg_detector_;
    std::shared_ptr<detection_6d::Base6DofDetectionModel> foundation_pose_;
    std::shared_ptr<detection_6d::BaseMeshLoader> mesh_loader_;
    std::shared_ptr<inference_core::BaseInferCore> refiner_core_;
    std::shared_ptr<inference_core::BaseInferCore> scorer_core_;

    // === Frame data ===
    cv::Mat depth_frame_;
    Eigen::Matrix4f current_pose_;
    std::mutex data_mutex_;
    std::atomic<bool> processing_;  // 防止多线程并发处理帧

    // === ROS subscribers ===
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // === ROS publishers ===
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_base_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_image_pub_;

    // === Logging ===
    std::string log_project_path_;
};

// === Main node for multi-camera management ===
class PoseEstMainNode : public rclcpp::Node
{
public:
    explicit PoseEstMainNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    std::vector<int> getServerCameraIds();
    std::vector<int> getConfigCameraIds();
    std::vector<int> getActiveCameraIds();

private:
    rclcpp::SyncParametersClient::SharedPtr sys_config_client_;
    std::vector<int> cam_server_ids_;
    std::vector<int> cam_config_ids_;
    std::vector<int> cam_active_ids_;
    std::string log_project_path_;
};
