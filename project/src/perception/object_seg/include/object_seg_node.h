#include "seg_task_base.h"
#include "task_factory.h"
#include "algorithm_loader.h"
#include <iostream>
#include <chrono>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <random>
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
#include "sys_info_src/sys_info_server.h"
#include "log_system/log_macros.hpp"
#include <custom_msgs_comm/srv/switch_algorithm.hpp>
#include <mutex>

class Object_Seg_Node : public rclcpp::Node
{
public:
    Object_Seg_Node(const rclcpp::NodeOptions &options, int cam_id);

    // 模块状态发布
    void publishModuleStatus(basros::ModuleStatus status, const std::string& status_msg);

private:
    void Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void CameraInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void publish(const std::vector<SegmentationResult>& segs, const std_msgs::msg::Header& header);

    // 从模型同名 .names 文件加载类别名称
    std::vector<std::string> loadClassNamesFromModel(const std::string& model_path);

    // 新增功能方法 - 解耦合设计
    void setup_camera_intrinsics();
    geometry_msgs::msg::Point calculate_3d_position(const SegmentationResult& seg);
    void initTopicNames();
    void initCalibParamHandler();
    bool getSysDat();
    bool getCamConfigInfo(SysConfig::CamConfigInfo& cam_info);

    // 核心重构方法 - 使用抽象接口初始化分割任务
    void initSegmentationTask();

    // 运行时动态切换算法
    void switchAlgorithm(const std::string& algorithm,
                       const std::string& model_path,
                       const std::string& config_path,
                       const InferenceEngineConfig& engine_config);

    // ROS2 参数回调
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter>& parameters);

    // 算法切换服务回调
    void handleSwitchAlgorithmService(
        const std::shared_ptr<custom_msgs_comm::srv::SwitchAlgorithm::Request> request,
        std::shared_ptr<custom_msgs_comm::srv::SwitchAlgorithm::Response> response);

    // 模块状态发布器初始化
    void initModuleStatusPublisher();

private:
    // 参数
    std::string engine_path_;
    std::string algorithm_name_;  // 算法名称
    std::string color_image_topic_;
    std::string depth_image_topic_;
    std::string camera_info_topic_;
    std::string output_image_topic_;
    std::string detection_topic_;

    // 相机和标定相关
    std::string camera_type_;
    int camera_id_;
    int arm_id_;
    std::vector<int> arm_id_list_;
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    bool camera_intrinsics_initialized_;
    cv::Mat depth_frame_;

    // 模型配置
    float conf_threshold_;
    float iou_threshold_;
    bool usecalib_;

    // 分割任务抽象接口（替代原有的 YoloSeg 具体类）
    std::unique_ptr<ISegmentationTask> seg_task_;
    std::mutex seg_task_mutex_;  // 保护 seg_task_ 的互斥锁

    // 类别名称列表（从模型同名 .names 文件动态加载）
    std::vector<std::string> class_names_;

    // 标定参数
    std::unique_ptr<handeyecalib::CalibRes> calib_result_;
    rclcpp::SyncParametersClient::SharedPtr sys_config_client_;

    // ROS订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // ROS发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr box_image_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_res_pub_;

    // 算法切换服务
    rclcpp::Service<custom_msgs_comm::srv::SwitchAlgorithm>::SharedPtr switch_algorithm_service_;

    // 模块状态发布
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr module_status_pub_;
    std::string module_name_;
    basros::ModuleStatus current_status_;
    std::mutex status_mutex_;
    
    // 数据流监控
    bool data_stream_active_;
    bool first_frame_received_;
    rclcpp::Time last_frame_time_;
    rclcpp::TimerBase::SharedPtr data_stream_timer_;
    void checkDataStreamStatus();

    // 参数变化回调句柄（必须保存，否则会被自动销毁）
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 互斥锁
    std::mutex detection_mutex_;
    cv::Mat service_color_frame_;
    std_msgs::msg::Header last_header_;
    std::string log_project_path_;
};
