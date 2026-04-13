#include "obb_task_base.h"
#include "task_factory.h"
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
#include "sys_info_src/sys_info_server.h"
#include "log_system/log_macros.hpp"
#include "custom_msgs_comm/srv/get_obb_detection.hpp"
#include <custom_msgs_comm/srv/switch_algorithm.hpp>
#include <mutex>

class Obb_Det_Node : public rclcpp::Node
{
public:
    Obb_Det_Node(const rclcpp::NodeOptions &options, int cam_id);

    // 模块状态发布
    void publishModuleStatus(basros::ModuleStatus status, const std::string& status_msg);

private:
    void Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void CameraInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);  // 新增相机信息回调
    void publish(const std::vector<OBBDetectionResult>& dets, const std_msgs::msg::Header& header);

    // 从模型同名 .names 文件加载类别名称
    std::vector<std::string> loadClassNamesFromModel(const std::string& model_path);

    // 新增功能方法 - 解耦合设计
    void setup_camera_intrinsics();  // 设置相机内参
    geometry_msgs::msg::Point calculate_3d_position(const OBBDetectionResult& det);  // 计算3D坐标（考虑OBB旋转角）
    void initTopicNames();  // 使用parseCommInfo初始化话题名
    void initCalibParamHandler();  // 初始化标定参数处理器
    bool getSysDat();  // 从参数服务器获取系统配置（包括相机对应的机械臂列表）
    bool getCamConfigInfo(SysConfig::CamConfigInfo& cam_info);  // 获取相机配置信息

    // 新增服务回调
    void obbDetectionService(
        const std::shared_ptr<custom_msgs_comm::srv::GetOBBDetection::Request> request,
        std::shared_ptr<custom_msgs_comm::srv::GetOBBDetection::Response> response);

    // 核心重构方法 - 使用抽象接口初始化检测任务
    void initOBDetectionTask();

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
    std::string camera_type_;      // 相机类型
    int camera_id_;                 // 相机ID，支持多相机配置
    int arm_id_;                    // 机械臂ID，用于获取对应的标定矩阵
    std::vector<int> arm_id_list_;  // 机械臂ID列表，支持多机械臂检测
    std::string color_image_topic_;      // rgb图像话题名称
    std::string depth_image_topic_;      // 深度图像话题名称
    std::string camera_info_topic_;     // 相机信息话题名称
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    bool camera_intrinsics_initialized_;  // 相机内参是否已初始化
    
    // 使用抽象接口（工厂模式创建）
    std::unique_ptr<IOBBDetectionTask> obb_det_;
    std::mutex detection_task_mutex_;  // 保护检测任务的读写
    
    cv::Mat depth_frame_;
    std::string engine_name_;  // 引擎模型路径，从参数服务器获取

    // 类别名称列表（从模型同名 .names 文件动态加载）
    std::vector<std::string> class_names_;

    // 标定参数
    bool usecalib_;                       // 是否使用标定模式
    std::unique_ptr<handeyecalib::CalibRes> calib_result_;  // 标定结果
    rclcpp::SyncParametersClient::SharedPtr sys_config_client_;  // 参数服务器客户端

    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_res_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr box_image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;  // 新增相机信息订阅

    // 服务相关
    rclcpp::Service<custom_msgs_comm::srv::GetOBBDetection>::SharedPtr obb_detection_service_;
    
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

    std::mutex detection_mutex_;  // 保护图像数据的互斥锁
    cv::Mat service_color_frame_;  // 存储供服务使用的彩色图像
    std_msgs::msg::Header last_header_;  // 存储最后一次图像的header

    // 日志相关
    std::string log_project_path_;  // 根据camera_id生成的日志项目路径
};
