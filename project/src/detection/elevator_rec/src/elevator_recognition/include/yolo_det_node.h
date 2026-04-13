#include "yolo_det.h"
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
// #include <librealsense2/rs.hpp>
// #include "ob.hpp"
#define USE_ORBBEC 1
#define TEST 0
const std::vector<std::string> class_names = 
{
  "standard_button", "alarm", "close", "down", "open", "stop", "up", "updown"
};

class Yolo_Det_Node : public rclcpp::Node
{
public:
    Yolo_Det_Node(const rclcpp::NodeOptions &options);

private:
    void Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void CameraInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);  // 新增相机信息回调
    void publish(const std::vector<Detection>& dets, const std_msgs::msg::Header& header);
    
    // 新增功能方法 - 解耦合设计
    void setup_camera_intrinsics();  // 设置相机内参
    void publish_static_tf();        // 发布静态TF
    geometry_msgs::msg::Point calculate_3d_position(const Detection& det);  // 计算3D坐标
    geometry_msgs::msg::Point transform_to_base_link(const geometry_msgs::msg::Point& camera_point);  // 坐标转换
    
private:
    std::string camera_type_;      // 相机类型
    std::string color_image_topic_;      // rgb图像话题名称
    std::string depth_image_topic_;      // 深度图像话题名称
    std::string camera_info_topic_;     // 相机信息话题名称
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    bool camera_intrinsics_initialized_;  // 相机内参是否已初始化
    YoloDet yolo_det_;
    cv::Mat depth_frame_;
    std::string engine_name_ = "install/elevator_recognition/models/det_rt10.engine";
    
    // 新增成员变量
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped camera_to_base_tf_;
    
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_res_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr box_image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr res_image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;  // 新增相机信息订阅
};

