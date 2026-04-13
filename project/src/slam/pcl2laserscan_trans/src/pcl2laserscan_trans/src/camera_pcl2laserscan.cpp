#include "pcl2laserscan_trans/camera_pcl2laserscan.h"
#include <pcl/filters/passthrough.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <vector>
#include <limits>
#include <chrono>
#include <thread>
#include <stdexcept>

// 定义项目名称常量，用于log_system
static const std::string PROJECT_NAME = "camera_pcl2laserscan";

void Camera_Pcl2laserscan::init(rclcpp::Node::SharedPtr nh)
{
    node_ = nh;
    // 点云话题从参数读取，默认值可由launch/config覆盖
    cloud_topic_ = nh->declare_parameter("cloud_topic", "/camera/depth/color/points");
    target_frame_ = nh->declare_parameter("target_frame", "camera_link");
    min_height_ = nh->declare_parameter("min_height", 0.7);
    max_height_ = nh->declare_parameter("max_height", 1.5);
    angle_min_ = nh->declare_parameter("angle_min", -1.5708);
    angle_max_ = nh->declare_parameter("angle_max", 1.5708);
    angle_increment_ = nh->declare_parameter("angle_increment", 0.0087);
    scan_time_ = nh->declare_parameter("scan_time", 0.3333);
    range_min_ = nh->declare_parameter("range_min", 0.1);
    range_max_ = nh->declare_parameter("range_max", 4.0);
    inf_epsilon_ = nh->declare_parameter("inf_epsilon", 1.0);
    use_inf_ = nh->declare_parameter("use_inf", true);

    LOG_INFO(PROJECT_NAME, "相机节点启动，目标坐标系: %s", target_frame_.c_str());
    LOG_INFO(PROJECT_NAME, "订阅点云话题: %s", cloud_topic_.c_str());
    LOG_INFO(PROJECT_NAME, "参数: target_frame=%s, min_height=%.2f, max_height=%.2f",
                 target_frame_.c_str(), min_height_, max_height_);

    // 初始化TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 使用更大的队列大小和合适的QoS配置来订阅点云
    rclcpp::QoS cloud_qos_profile = rclcpp::QoS(10);  // 增加队列大小到10
    cloud_qos_profile.best_effort();  // 使用best effort可靠性
    
    cloud_sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, cloud_qos_profile,
        std::bind(&Camera_Pcl2laserscan::CloudCallback, this, std::placeholders::_1));

    cloud_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("trans_cloud", 1);

    cloud_pub_filtHeight = nh->create_publisher<sensor_msgs::msg::PointCloud2>("trans_cloud_heightfilted", 1);
    // 在发布器创建时使用与RViz2兼容的QoS配置
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_profile.deadline(rclcpp::Duration(0, 100000000)); // 100ms deadline
    pub_ = nh->create_publisher<sensor_msgs::msg::LaserScan>("camera_scan", qos_profile /*使用优化的QoS配置*/);
}

void Camera_Pcl2laserscan::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
    try {
        LOG_DEBUG(PROJECT_NAME, "Received point cloud message");
        auto start_time = std::chrono::high_resolution_clock::now();

        std::string source_frame_id = laserCloudMsg->header.frame_id;

        // 坐标系处理逻辑
        if (source_frame_id.empty() || source_frame_id == "camera_depth_optical_frame")
        {
            // 如果是源帧是camera_depth_optical_frame，尝试查找可用的源帧
            if (!tf_buffer_->canTransform(target_frame_, "camera_depth_optical_frame",
                                          tf2::TimePointZero, tf2::durationFromSec(0.1)))
            {
                // 尝试其他可能的坐标系名称
                std::vector<std::string> possible_source_frames = {
                    "camera_depth_frame",
                    "camera_aligned_depth_to_color_frame",
                    "camera_rgb_frame",
                    "camera_link"
                };
                
                bool found_valid_frame = false;
                for (const auto& frame : possible_source_frames)
                {
                    if (tf_buffer_->canTransform(target_frame_, frame,
                                                 tf2::TimePointZero, tf2::durationFromSec(0.1)))
                    {
                        source_frame_id = frame;
                        LOG_INFO(PROJECT_NAME,
                                     "找到可用的源坐标系: %s", frame.c_str());
                        found_valid_frame = true;
                        break;
                    }
                }
                
                if (!found_valid_frame)
                {
                    LOG_WARN(PROJECT_NAME,
                                 "未找到有效的源坐标系，当前可用的源坐标系: %s", laserCloudMsg->header.frame_id.c_str());
                    return;
                }
            }
        }

        // 检查目标坐标系变换
        // 使用点云数据的时间戳来查询TF变换，避免时间戳不匹配问题
        rclcpp::Time cloud_time(laserCloudMsg->header.stamp);
        tf2::TimePoint transform_time = tf2_ros::fromMsg(cloud_time);
        
        if (!tf_buffer_->canTransform(target_frame_, source_frame_id,
                                      transform_time, tf2::durationFromSec(0.1)))
        {
            LOG_WARN(PROJECT_NAME,
                         "无法获取从 %s 到 %s 的坐标变换，正在检查可用的TF树", 
                         source_frame_id.c_str(), target_frame_.c_str());
            
            // 尝试获取TF缓冲区中的详细信息
            try {
                auto all_frames = tf_buffer_->getAllFrameNames();
                std::string frames_str;
                for (size_t i = 0; i < all_frames.size(); ++i) {
                    frames_str += all_frames[i];
                    if (i < all_frames.size() - 1) {
                        frames_str += ", ";
                    }
                }
                LOG_INFO(PROJECT_NAME,
                            "当前可用的坐标系: %s", frames_str.c_str());
            } catch (...) {
                LOG_WARN(PROJECT_NAME,
                            "无法获取TF树信息");
            }
            
            LOG_DEBUG(PROJECT_NAME,
                          "旧版坐标变换检查（已弃用）");
            // 移除旧的调试日志

            return;
        }

        // 转换点云坐标系
        sensor_msgs::msg::PointCloud2 cloud_transformed;
        try
        {
            auto transform = tf_buffer_->lookupTransform(
                target_frame_, source_frame_id, transform_time);

            tf2::doTransform(*laserCloudMsg, cloud_transformed, transform);
            cloud_transformed.header.frame_id = target_frame_;
        }
        catch (tf2::TransformException &ex)
        {
            LOG_ERROR(PROJECT_NAME,
                          "点云转换异常: %s", ex.what());

            return;
        }

        // 转换为PCL格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_transformed, *cloud_raw);
        LOG_DEBUG(PROJECT_NAME,
                      "原始点云点数cloud_raw->points.size(): %zu", cloud_raw->points.size());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // 应用体素滤波降采样
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud_raw);
    #if 0
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
    #else
            pcl::PointXYZ min_pt, max_pt;
            min_pt = cloud_raw->points[0];
            max_pt = cloud_raw->points[0];
        
        for (const auto& point : cloud_raw->points) {
            min_pt.x = std::min(min_pt.x, point.x);
            min_pt.y = std::min(min_pt.y, point.y);
            min_pt.z = std::min(min_pt.z, point.z);
            max_pt.x = std::max(max_pt.x, point.x);
            max_pt.y = std::max(max_pt.y, point.y);
            max_pt.z = std::max(max_pt.z, point.z);
        }
            
            float range_x = max_pt.x - min_pt.x;
            float range_y = max_pt.y - min_pt.y;
            float range_z = max_pt.z - min_pt.z;
            
            LOG_DEBUG(PROJECT_NAME,
                          "点云范围: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
                          min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);
            
            // 自动调整leaf_size
            float dynamic_leaf_size = std::max(0.05f, std::min(range_x, std::min(range_y, range_z)) / 100.0f);
            dynamic_leaf_size = std::min(dynamic_leaf_size, 0.2f);  // 最大0.2米
            
            voxel_filter.setLeafSize(dynamic_leaf_size, dynamic_leaf_size, dynamic_leaf_size);
            LOG_DEBUG(PROJECT_NAME,
                          "使用动态leaf_size: %.3f米", dynamic_leaf_size);
    #endif
            voxel_filter.filter(*cloud_filtered);
            LOG_DEBUG(PROJECT_NAME,
                      "体素滤波降采样点云点数cloud_filtered->points.size(): %zu", cloud_filtered->points.size());

        // 应用统计滤波去除离群点
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_filtered);
        sor.setMeanK(20);
        sor.setStddevMulThresh(0.5);
        sor.filter(*cloud_filtered);
        LOG_DEBUG(PROJECT_NAME,
                      "统计滤波点云点数cloud_filtered->points.size(): %zu", cloud_filtered->points.size());

        // 发布滤波后点云（调试用）
        sensor_msgs::msg::PointCloud2 filtered_cloud;
        pcl::toROSMsg(*cloud_filtered, filtered_cloud);
        filtered_cloud.header.frame_id = target_frame_;
        filtered_cloud.header.stamp = laserCloudMsg->header.stamp;
        cloud_pub->publish(filtered_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFilted = nullptr;
        bool bFilterHeight = true; // 高度过滤
        if (bFilterHeight)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_height(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto &point : *cloud_filtered)
            {
                if (point.z >= min_height_ && point.z <= max_height_)
                {
                    cloud_filtered_height->push_back(point);
                }
            }
            LOG_DEBUG(PROJECT_NAME,
                          "高度过滤后的点云点数cloud_raw->points.size(): %zu", cloud_filtered_height->points.size());
            // 发布高度过滤后的点云
            sensor_msgs::msg::PointCloud2 filtered_cloud_height;
            pcl::toROSMsg(*cloud_filtered_height, filtered_cloud_height);
            filtered_cloud_height.header.frame_id = target_frame_;
            filtered_cloud_height.header.stamp = laserCloudMsg->header.stamp;
            cloud_pub_filtHeight->publish(filtered_cloud_height);
            pCloudFilted = cloud_filtered_height;
        }
        else
        {
            // 先不考虑高度过滤
            pCloudFilted = cloud_filtered;
        }

        // 准备LaserScan消息
        auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        scan_msg->header.stamp = laserCloudMsg->header.stamp;
        scan_msg->header.frame_id = target_frame_;
        scan_msg->angle_min = angle_min_;
        scan_msg->angle_max = angle_max_;
        scan_msg->angle_increment = angle_increment_;
        scan_msg->scan_time = scan_time_;
        scan_msg->range_min = range_min_;
        scan_msg->range_max = range_max_;

        const size_t ranges_size = std::ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
        scan_msg->ranges.assign(ranges_size, use_inf_ ? std::numeric_limits<double>::infinity() : (range_max_ + inf_epsilon_));

        // 转换为2D激光数据
        for (const auto &point : *pCloudFilted)
        {
            // 计算距离与角度 (基于base_link坐标系)
            const double range = std::hypot(point.x, point.y);
            const double angle = std::atan2(point.y, point.x);

            // 检查距离/角度有效性
            if (range < range_min_ || range > range_max_)
                continue;
            if (angle < scan_msg->angle_min || angle > scan_msg->angle_max)
                continue;

            // 更新对应射线的距离（取最小值）
            const int index = static_cast<int>((angle - scan_msg->angle_min) / scan_msg->angle_increment);
            if (index >= 0 && index < static_cast<int>(ranges_size) && range < scan_msg->ranges[index])
            {
                scan_msg->ranges[index] = range;
            }
        }

        pub_->publish(std::move(scan_msg));
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        if (duration.count() > 33)
        { // 超过33ms警告（30FPS）
            LOG_WARN(PROJECT_NAME,
                         "点云处理耗时 %ld ms", duration.count());
        }
        LOG_DEBUG(PROJECT_NAME, "发布激光扫描消息");
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(PROJECT_NAME,
                      "CloudCallback中发生异常: %s", e.what());
    }
    catch (...)
    {
        LOG_ERROR(PROJECT_NAME,
                      "CloudCallback中发生未知异常");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nodePtr = rclcpp::Node::make_shared("camera_pcl2laserscan_node");
    Camera_Pcl2laserscan camera_pcl2laserscan;
    camera_pcl2laserscan.init(nodePtr);
    if (0)
    {
        rclcpp::spin(nodePtr);
    }
    else
    {
        // 使用多线程执行器提高性能
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(nodePtr);
        executor.spin();
    }
    rclcpp::shutdown();
    return 0;
}