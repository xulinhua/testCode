/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pcl2laserscan_trans/pointcloud_to_laserscan_node.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <cmath>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"
#include <log_system/log_macros.hpp>

// 新增：PCL 滤波头文件
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

// 新增：ModuleInfo 发布相关
#include <nlohmann/json.hpp>

namespace pointcloud_to_laserscan
{

  PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("pointcloud_to_laserscan", options)
  {
    target_frame_ = this->declare_parameter("target_frame", "");
    tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
    // TODO(hidmic): adjust default input queue size based on actual concurrency levels
    // achievable by the associated executor
    input_queue_size_ = this->declare_parameter(
        "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
    min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
    max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
    angle_min_ = this->declare_parameter("angle_min", -M_PI);
    angle_max_ = this->declare_parameter("angle_max", M_PI);
    angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
    scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
    range_min_ = this->declare_parameter("range_min", 0.0);
    range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
    inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
    use_inf_ = this->declare_parameter("use_inf", true);

    filter_mean_k_ = this->declare_parameter("filter_mean_k", 50);       // 统计滤波邻域点数
    filter_stddev_ = this->declare_parameter("filter_stddev", 1.0);      // 统计滤波标准差阈值
    voxel_leaf_size_ = this->declare_parameter("voxel_leaf_size", 0.05); // 体素网格大小（米）

    // 激光雷达安装倾斜补偿参数
    tilt_compensation_angle_ = this->declare_parameter("tilt_compensation_angle", 0.0);
    // tilt_axis_ = this->declare_parameter("tilt_axis", "y");

    pub_FiltedCloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_heightfilted", 1);

    // 使用 RELIABLE 以兼容 RViz2 等默认订阅端（SensorDataQoS 为 BEST_EFFORT 会触发 QoS 不兼容）
    rclcpp::QoS scan_qos(rclcpp::KeepLast(10));
    scan_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    scan_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", scan_qos);

    using std::placeholders::_1;
    // if pointcloud target frame specified, we need to filter by transform availability
    if (!target_frame_.empty())
    {
      tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
          this->get_node_base_interface(), this->get_node_timers_interface());
      tf2_->setCreateTimerInterface(timer_interface);
      tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
      message_filter_ = std::make_unique<MessageFilter>(
          sub_, *tf2_, target_frame_, input_queue_size_,
          this->get_node_logging_interface(),
          this->get_node_clock_interface());
      message_filter_->registerCallback(
          std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
    }
    else
    { // otherwise setup direct subscription
      sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
    }

    // 创建 ModuleInfo 刷新定时器，步长 200ms
    module_info_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&PointCloudToLaserScanNode::refreshModuleInfo, this));
    RCLCPP_INFO(this->get_logger(), "ModuleInfo 刷新定时器已创建");

    subscription_listener_thread_ = std::thread(
        std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
  }

  PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
  {
    alive_.store(false);
    subscription_listener_thread_.join();
  }

  void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
  {
    rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

    const std::chrono::milliseconds timeout(100);
    while (rclcpp::ok(context) && alive_.load())
    {
      int subscription_count = pub_->get_subscription_count() +
                               pub_->get_intra_process_subscription_count();
      if (subscription_count > 0)
      {
        if (!sub_.getSubscriber())
        {
          LOG_INFO("收到激光扫描订阅者，启动点云订阅器");
          // 使用 RELIABLE 以匹配仿真/laserscan_to_pointcloud 等常见点云发布端，避免收不到数据
          rclcpp::QoS qos{rclcpp::KeepLast(input_queue_size_)};
          qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
          qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
          sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
        }
      }
      else if (sub_.getSubscriber())
      {
        LOG_INFO("无激光扫描订阅者，关闭点云订阅器");
        sub_.unsubscribe();
      }
      rclcpp::Event::SharedPtr event = this->get_graph_event();
      this->wait_for_graph_change(event, timeout);
    }
    sub_.unsubscribe();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudToLaserScanNode::filterPointCloud(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_sor(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(pcl_cloud);
    sor.setMeanK(filter_mean_k_);           // 邻域点数量
    sor.setStddevMulThresh(filter_stddev_); // 标准差阈值（小于阈值的点保留）
    sor.filter(*filtered_sor);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(filtered_sor);
    vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_); // 体素大小
    vg.filter(*filtered_voxel);

    LOG_DEBUG("原始点数量：%zu, 滤波后点数量：%zu",
                 pcl_cloud->size(), filtered_voxel->size());
    return filtered_voxel;
  }

  void PointCloudToLaserScanNode::cloudCallback(
      sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {
    LOG_INFO("pointcloud_to_laserscan: cloudCallback 进入，frame_id=%s, width=%u, height=%u",
      cloud_msg->header.frame_id.c_str(), cloud_msg->width, cloud_msg->height);

    if (0)
    {
      auto filtered_pcl_cloud = filterPointCloud(cloud_msg);
      sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
      pcl::toROSMsg(*filtered_pcl_cloud, filtered_cloud_msg);
      filtered_cloud_msg.header = cloud_msg->header; // 保持原坐标帧和时间戳

      cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(filtered_cloud_msg);
    }

    // 创建用于存储高度过滤后点云的消息
    sensor_msgs::msg::PointCloud2 filtered_cloud_height;

    // 复制原始点云的基本信息（header、height、width、fields等）
    filtered_cloud_height.header = cloud_msg->header;
    filtered_cloud_height.height = cloud_msg->height;
    filtered_cloud_height.width = cloud_msg->width;
    filtered_cloud_height.fields = cloud_msg->fields;
    filtered_cloud_height.is_bigendian = cloud_msg->is_bigendian;
    filtered_cloud_height.point_step = cloud_msg->point_step;
    filtered_cloud_height.row_step = cloud_msg->row_step;
    filtered_cloud_height.is_dense = cloud_msg->is_dense;

    // 预留空间，但实际数据将在后面填充
    filtered_cloud_height.data.reserve(cloud_msg->data.size());

    // build laserscan output
    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan_msg->header = cloud_msg->header;
    if (!target_frame_.empty())
    {
      scan_msg->header.frame_id = target_frame_;
    }

    scan_msg->angle_min = angle_min_;
    scan_msg->angle_max = angle_max_;
    scan_msg->angle_increment = angle_increment_;
    scan_msg->time_increment = 0.0;
    scan_msg->scan_time = scan_time_;
    scan_msg->range_min = range_min_;
    scan_msg->range_max = range_max_;

    // determine amount of rays to create
    uint32_t ranges_size = std::ceil(
        (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    if (use_inf_)
    {
      scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    }
    else
    {
      scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
    }

    // Transform cloud if necessary
    if (scan_msg->header.frame_id != cloud_msg->header.frame_id)
    {
      try
      {
        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
        cloud_msg = cloud;
        // std::cout<< "Transform success" << std::endl;
      }
      catch (tf2::TransformException &ex)
      {
        LOG_ERROR("坐标变换失败：%s", ex.what());
        return;
      }
    }

    // 创建用于存储高度过滤后点云数据的缓冲区
    std::vector<uint8_t> filtered_data;
    filtered_data.reserve(cloud_msg->data.size());

    // 获取点云字段的偏移量
    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (size_t i = 0; i < cloud_msg->fields.size(); ++i)
    {
      if (cloud_msg->fields[i].name == "x")
        x_offset = cloud_msg->fields[i].offset;
      if (cloud_msg->fields[i].name == "y")
        y_offset = cloud_msg->fields[i].offset;
      if (cloud_msg->fields[i].name == "z")
        z_offset = cloud_msg->fields[i].offset;
    }

    if (x_offset == -1 || y_offset == -1 || z_offset == -1)
    {
      LOG_ERROR("在点云中找不到 x, y, z 字段！");
      return;
    }

// 添加倾斜补偿
// 计算旋转矩阵（绕X轴旋转）
#if 1
    const double cos_tilt = cos(tilt_compensation_angle_);
    const double sin_tilt = sin(tilt_compensation_angle_);
#endif
    // 用于调试的计数器
    size_t points_compensated = 0;

    // Iterate through pointcloud
    // for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    //   iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    //   iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    // std::cout<< "倾斜角度:" << tilt_compensation_angle_ << std::endl;

    for (size_t i = 0; i < cloud_msg->width * cloud_msg->height; ++i)
    {
      // 获取当前点的数据指针
      const uint8_t *point_ptr = &cloud_msg->data[i * cloud_msg->point_step];

      // 提取点的坐标
      float x = *reinterpret_cast<const float *>(point_ptr + x_offset);
      float y = *reinterpret_cast<const float *>(point_ptr + y_offset);
      float z = *reinterpret_cast<const float *>(point_ptr + z_offset);

      // 应用倾斜补偿
      // 对点云应用绕Y轴的旋转补偿
      if (std::abs(tilt_compensation_angle_) > 1e-6)
      {
        if (0)
        { // 绕X轴旋转
          // 保存原始坐标用于调试
          const float orig_y = y;
          const float orig_z = z;

          // 应用旋转矩阵 (绕X轴旋转)
          y = orig_y * cos_tilt - orig_z * sin_tilt;
          z = orig_y * sin_tilt + orig_z * cos_tilt;
        }
        else
        {
          // 保存原始坐标用于调试
          const float orig_x = x;
          const float orig_z = z;

          // 应用旋转矩阵 (绕Y轴旋转)
          x = orig_x * cos_tilt - orig_z * sin_tilt;
          z = orig_x * sin_tilt + orig_z * cos_tilt;
        }
        points_compensated++;
      }

      if (std::isnan(x) || std::isnan(y) || std::isnan(z))
      {
        LOG_DEBUG(
            "因 NaN 拒绝点 (%f, %f, %f)\n",
            x, y, z);
        continue;
      }

      // 高度过滤
      if (z > max_height_ || z < min_height_)
      {
        LOG_DEBUG(
            "因高度 %f 不在范围 (%f, %f) 内拒绝\n",
            z, min_height_, max_height_);
        continue;
      }

      // 计算距离
      double range = hypot(x, y);
      if (range < range_min_)
      {
        LOG_DEBUG(
            "因距离 %f 小于最小值 %f 拒绝。点：(%f, %f, %f)",
            range, range_min_, x, y, z);
        continue;
      }
      if (range > range_max_)
      {
        LOG_DEBUG(
            "因距离 %f 大于最大值 %f 拒绝。点：(%f, %f, %f)",
            range, range_max_, x, y, z);
        continue;
      }

      // 计算角度
      double angle = atan2(y, x);
      if (angle < scan_msg->angle_min || angle > scan_msg->angle_max)
      {
        LOG_DEBUG(
            "因角度 %f 不在范围 (%f, %f) 内拒绝\n",
            angle, scan_msg->angle_min, scan_msg->angle_max);
        continue;
      }
      // 将通过高度过滤的点添加到过滤后的数据中
      filtered_data.insert(filtered_data.end(),
                           point_ptr,
                           point_ptr + cloud_msg->point_step);

      // overwrite range at laserscan ray if new range is smaller
      int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
      if (range < scan_msg->ranges[index])
      {
        scan_msg->ranges[index] = range;
      }
    }
    // 调试输出
    if (points_compensated > 0)
    {
      LOG_DEBUG(
                   "对 %zu 个点应用 Y 轴倾斜补偿 (角度：%.4f 弧度，%.1f 度)",
                   points_compensated,
                   tilt_compensation_angle_,
                   tilt_compensation_angle_ * 180.0 / M_PI);
    }

    cloud_received_.store(true);
    pub_->publish(std::move(scan_msg));

    if (1)
    {
      // 更新过滤后点云的实际数据
      filtered_cloud_height.data = std::move(filtered_data);
      filtered_cloud_height.width = filtered_cloud_height.data.size() / filtered_cloud_height.point_step;
      filtered_cloud_height.row_step = filtered_cloud_height.width * filtered_cloud_height.point_step;
      // 发布高度过滤后的点云
      if (!target_frame_.empty())
      {
        filtered_cloud_height.header.frame_id = target_frame_;
      }
      pub_FiltedCloud_->publish(filtered_cloud_height);
    }
    else
    {
      // 发布高度过滤后的点云
      sensor_msgs::msg::PointCloud2 filtered_cloud_height;
      // 直接复制点云数据，而不是使用toROSMsg转换
      filtered_cloud_height = *cloud_msg;
      filtered_cloud_height.header.frame_id = target_frame_;
      filtered_cloud_height.header.stamp = cloud_msg->header.stamp; // 使用正确的变量名
      pub_FiltedCloud_->publish(filtered_cloud_height);
    }
  }
} // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)

// 实现 ModuleInfo 相关方法
namespace pointcloud_to_laserscan
{

  void PointCloudToLaserScanNode::initModuleStatusInfo()
  {
    if (module_info_initialized_) {
      return;
    }

    // 初始化 ModuleStatusInfo
    module_status_info_.module_name = "pcl2laserscan_trans";
    module_status_info_.cam_id = 0;
    module_status_info_.status = basros::ModuleStatus::RUNNING;
    module_status_info_.status_msg = "点云转激光节点正常运行";

    // 创建发布器
    basros::RosCommInfo comm_info = basros::parseCommInfo(
        basros::RosCommMsgType::COMM_MODULE_INFO_PCL, 0, 0);
    module_info_publisher_ = this->create_publisher<std_msgs::msg::String>(
        comm_info.name, rclcpp::QoS(10).reliable());
    RCLCPP_INFO(this->get_logger(),
        "ModuleStatusInfo 发布器已创建：%s", comm_info.name.c_str());

    module_info_initialized_ = true;
  }

  void PointCloudToLaserScanNode::refreshModuleInfo()
  {
    if (!module_info_initialized_) {
      initModuleStatusInfo();
    }

    // 根据是否收到点云数据判断运行状态
    if (cloud_received_.load()) {
      module_status_info_.status = basros::ModuleStatus::RUNNING;
      module_status_info_.status_msg = "";
    } else {
      module_status_info_.status = basros::ModuleStatus::STARTING;
      module_status_info_.status_msg = "等待点云数据...";
    }

    publishModuleInfo();
  }

  void PointCloudToLaserScanNode::publishModuleInfo()
  {
    if (!module_info_initialized_ || !module_info_publisher_) {
      return;
    }

    std_msgs::msg::String ros_msg;
    ros_msg.data = basros::moduleStatusInfoToJson(module_status_info_);
    module_info_publisher_->publish(ros_msg);
    RCLCPP_DEBUG(this->get_logger(),
        "发布 ModuleStatusInfo: %s", ros_msg.data.c_str());
  }

} // namespace pointcloud_to_laserscan
