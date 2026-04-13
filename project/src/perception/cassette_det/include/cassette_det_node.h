#include "yolo_det.h"
#include <iostream>
#include <chrono>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <iostream>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <custom_msgs_comm/msg/cassette_with_holes.hpp>
#include <custom_msgs_comm/msg/cassette_with_holes_array.hpp>
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
#include <sensor_msgs/msg/camera_info.hpp>
#include "bas_operate_ros/ros_comm_info.h"
#include "bas_operate_ros/param_utils.hpp"
#include "hand_eye_calib/calib_struct.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "calib_info_src/calib_info_server.h"
#include "log_system/log_macros.hpp"

// 数据结构定义
struct CassetteWithHoles
{
    Detection cassette;
    std::vector<Detection> holes;
};

// 函数声明
void render_cassette_with_holes(cv::Mat& img, const std::vector<CassetteWithHoles>& cassette_results, 
                                const std::vector<std::string>& cassette_class_names, 
                                const std::vector<std::string>& hole_class_names);
void crop_image(const cv::Mat& src_img, const Detection& detection, cv::Mat& crop_img);

class Cassette_Det_Node : public rclcpp::Node
{
public:
    Cassette_Det_Node(const rclcpp::NodeOptions &options, int cam_id);

private:
    void Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void CameraInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    // 从模型同名 .names 文件加载类别名称
    std::vector<std::string> loadClassNamesFromModel(const std::string& model_path);

    void setup_camera_intrinsics();  // 设置相机内参
    geometry_msgs::msg::Point calculate_3d_position(const Detection& det);  // 计算3D坐标
    void initTopicNames();  // 使用parseCommInfo初始化话题名
    void initCalibParamHandler();  // 初始化标定参数处理器
    void publish(const std::vector<CassetteWithHoles>& cassette_res, const std_msgs::msg::Header& header);
    bool getSysDat();  // 从参数服务器获取系统配置（包括相机对应的机械臂列表）
    bool getCamConfigInfo(SysConfig::CamConfigInfo& cam_info);  // 获取相机配置信息

private:
    std::string camera_type_;      // 相机类型
    int camera_id_;                 // 相机ID，支持多相机配置
    int arm_id_;                    // 机械臂ID，用于获取对应的标定矩阵
    std::vector<int> arm_id_list_;  // 机械臂ID列表，支持多机械臂检测
    std::string color_image_topic_;      // rgb图像话题名称
    std::string depth_image_topic_;      // 深度图像话题名称
    std::string camera_info_topic_;     // 相机信息话题名称
    bool camera_intrinsics_initialized_;   // 相机内参是否已初始化
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    YoloDet cassette_det_;
    YoloDet hole_det_;
    cv::Mat depth_frame_;
    std::string cassete_engine_name_;
    std::string hole_engine_name_;

    // 类别名称列表（从模型同名 .names 文件动态加载）
    std::vector<std::string> class_names_;
    std::vector<std::string> class1_names_;

    // 标定参数
    bool usecalib_;                       // 是否使用标定模式
    std::unique_ptr<handeyecalib::CalibRes> calib_result_;  // 标定结果
    rclcpp::SyncParametersClient::SharedPtr sys_config_client_;  // 参数服务器客户端

    rclcpp::Publisher<custom_msgs_comm::msg::CassetteWithHolesArray>::SharedPtr cassette_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr box_image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // 日志相关
    std::string log_project_path_;  // 根据camera_id生成的日志项目路径
};
