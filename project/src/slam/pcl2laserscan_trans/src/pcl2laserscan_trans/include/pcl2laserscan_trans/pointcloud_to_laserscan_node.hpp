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

#ifndef POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_
#define POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include "bas_operate_ros/param_utils.hpp"
#include "bas_operate_ros/module_status.hpp"

#include "pcl2laserscan_trans/visibility_control.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloud_to_laserscan
{
  typedef tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> MessageFilter;

  /**
   * Class to process incoming pointclouds into laserscans.
   * Some initial code was pulled from the defunct turtlebot pointcloud_to_laserscan implementation.
   */
  class PointCloudToLaserScanNode : public rclcpp::Node
  {
  public:
    POINTCLOUD_TO_LASERSCAN_PUBLIC
    explicit PointCloudToLaserScanNode(const rclcpp::NodeOptions &options);

    ~PointCloudToLaserScanNode() override;

  private:
    void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);

    void subscriptionListenerThreadLoop();

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg);

    // ModuleInfo 发布相关
    void initModuleStatusInfo();
    void publishModuleInfo();
    void refreshModuleInfo();

    std::unique_ptr<tf2_ros::Buffer> tf2_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_FiltedCloud_;
    std::unique_ptr<MessageFilter> message_filter_;

    std::thread subscription_listener_thread_;
    std::atomic_bool alive_{true};

    // ModuleInfo 定时器和发布器
    rclcpp::TimerBase::SharedPtr module_info_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr module_info_publisher_;
    basros::ModuleStatusInfo module_status_info_;
    bool module_info_initialized_{false};
    std::atomic_bool cloud_received_{false};  ///< 是否收到过点云数据

    // ROS Parameters
    int input_queue_size_;
    std::string target_frame_;
    double tolerance_;
    double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_,
        range_max_;
    bool use_inf_;
    double inf_epsilon_;
    int filter_mean_k_;
    double filter_stddev_;
    double voxel_leaf_size_;
    // 激光雷达安装的倾斜补偿参数
    std::string tilt_axis_;
    double tilt_compensation_angle_;
  };

} // namespace pointcloud_to_laserscan

#endif // POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_
