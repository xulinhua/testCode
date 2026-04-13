#ifndef CAMERA_PCL2LASERSCAN_H_
#define CAMERA_PCL2LASERSCAN_H_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <Eigen/Eigen>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <vector>
#include <cmath>
#include <memory>
#include <string>
#include "log_system/log_macros.hpp"

typedef pcl::PointXYZ PointType;
class Camera_Pcl2laserscan
{
public:
    Camera_Pcl2laserscan() = default;
    ~Camera_Pcl2laserscan() = default;
    void init(rclcpp::Node::SharedPtr nh);

private:
    rclcpp::Node::SharedPtr node_; // 存储节点指针
    std::string cloud_topic_;      // 点云话题名称
    std::string target_frame_;     // 目标坐标系
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_filtHeight; // 单独发布过滤后的点云
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // RosLoggerPtr logger_; // 移除RosLoggerPtr日志工具

    void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);

    double min_height_;        // 有效点云最小高度（过滤地面）
    double max_height_;        // 有效点云最大高度（过滤障碍物）
    double angle_min_;         // 扫描起始角度
    double angle_max_;         // 扫描结束角度
    double angle_increment_;   // 角度分辨率
    double scan_time_;         // 扫描周期
    double range_min_;         // 最小有效距离
    double range_max_;         // 最大有效距离
    bool use_inf_;             // 是否用inf表示无效值
    double inf_epsilon_;       // 非inf模式下无效值的偏移量
};
#endif