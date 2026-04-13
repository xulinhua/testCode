// 使用新的抽象层接口
#include "det_task_base.h"

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
#include <random>
#include <nav_msgs/msg/path.hpp>
#include <custom_msgs_comm/srv/switch_algorithm.hpp>
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
#include "bas_operate_ros/ros_comm_info.h"
#include "bas_operate_ros/param_utils.hpp"
#include "bas_operate_ros/module_status.hpp"
#include "hand_eye_calib/calib_struct.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "calib_info_src/calib_info_server.h"
#include "log_system/log_macros.hpp"

class Object_Det_Node : public rclcpp::Node
{
public:
    Object_Det_Node(const rclcpp::NodeOptions &options, int cam_id);

    // 模块状态发布
    void publishModuleStatus(basros::ModuleStatus status, const std::string& status_msg);

private:
    void Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void CamInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void publish(const std::vector<DetectionResult>& dets, const std_msgs::msg::Header& header);

    // 模块状态发布器初始化
    void initModuleStatusPublisher();

    // 核心重构方法 - 使用抽象接口初始化检测任务
    void initDetectionTask();

    // 运行时动态切换算法
    void switchAlgorithm(const std::string& algorithm,
                       const std::string& model_path,
                       const std::string& config_path,
                       const InferenceEngineConfig& engine_config);

    // ROS2 参数回调
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter>& parameters);

    // 解析算法配置字符串（格式: "algorithm:model_path:engine_type"）
    bool parseAlgorithmConfig(const std::string& config_str,
                           std::string& algorithm,
                           std::string& model_path,
                           std::string& engine_type);

    // 算法切换服务回调
    void handleSwitchAlgorithmService(
        const std::shared_ptr<custom_msgs_comm::srv::SwitchAlgorithm::Request> request,
        std::shared_ptr<custom_msgs_comm::srv::SwitchAlgorithm::Response> response);

    // 辅助方法
    void setup_cam_intrinsics();
    geometry_msgs::msg::Point calculate_3d_position(const DetectionResult& det);
    void initTopicNames();
    void initCalibParamHandler();
    bool getSysDat();  // 从参数服务器获取系统配置（包括相机对应的机械臂列表）
    bool getCamConfigInfo(SysConfig::CamConfigInfo& cam_info);  // 获取相机配置信息

    // 从模型同名 .names 文件加载类别名称
    std::vector<std::string> loadClassNamesFromModel(const std::string& model_path);

private:
    std::string cam_type_;
    int cam_id_;
    int arm_id_;
    std::vector<int> arm_id_list_;  // 机械臂ID列表，支持多机械臂检测
    std::string color_image_topic_;
    std::string depth_image_topic_;
    std::string cam_info_topic_;
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    bool cam_intrinsics_initialized_;

    // 使用抽象接口替代直接依赖YoloDet
    std::unique_ptr<IDetectionTask> detection_task_;
    
    // 类别名称列表（由应用层从配置加载）
    std::vector<std::string> class_names_;

    // 互斥锁保护检测任务的读写
    std::mutex detection_task_mutex_;

    cv::Mat depth_frame_;
    std::string engine_name_;

    // 标定参数
    bool usecalib_;
    std::unique_ptr<handeyecalib::CalibRes> calib_result_;
    rclcpp::SyncParametersClient::SharedPtr sys_config_client_;

    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_res_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr box_image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr res_image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

    // 算法切换服务
    rclcpp::Service<custom_msgs_comm::srv::SwitchAlgorithm>::SharedPtr switch_algorithm_service_;

    // 模块状态发布
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr module_status_pub_;
    std::string module_name_;
    basros::ModuleStatus current_status_;
    std::mutex status_mutex_;
    
    // 数据流监控
    bool data_stream_active_;                          // 数据流是否活跃
    bool first_frame_received_;                        // 是否已收到首帧
    rclcpp::Time last_frame_time_;                     // 上次收到帧的时间
    rclcpp::TimerBase::SharedPtr data_stream_timer_;   // 数据流监控定时器
    void checkDataStreamStatus();                      // 检查数据流状态

    // 参数变化回调句柄（必须保存，否则会被自动销毁）
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 日志相关
    std::string log_path_;
};
