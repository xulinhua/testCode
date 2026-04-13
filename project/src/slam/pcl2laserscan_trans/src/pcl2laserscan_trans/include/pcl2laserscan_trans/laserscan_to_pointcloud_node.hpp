#ifndef PCL2LASERSCAN_TRANS__LASERSCAN_TO_POINTCLOUD_NODE_HPP_
#define PCL2LASERSCAN_TRANS__LASERSCAN_TO_POINTCLOUD_NODE_HPP_

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

#include "pcl2laserscan_trans/visibility_control.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <laser_geometry/laser_geometry.hpp>

namespace pointcloud_to_laserscan
{
  typedef tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> MessageFilter;

  /**
   * Class to process incoming laserscans into pointclouds.
   */
  class LaserScanToPointCloudNode : public rclcpp::Node
  {
  public:
    POINTCLOUD_TO_LASERSCAN_PUBLIC
    explicit LaserScanToPointCloudNode(const rclcpp::NodeOptions &options);

    ~LaserScanToPointCloudNode() override;

  private:
    void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    void subscriptionListenerThreadLoop();

    std::unique_ptr<tf2_ros::Buffer> tf2_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    laser_geometry::LaserProjection projector_;
    std::unique_ptr<MessageFilter> message_filter_;

    std::thread subscription_listener_thread_;
    std::atomic_bool alive_{true};

    // ROS Parameters
    int input_queue_size_;
    std::string target_frame_;
    double tolerance_;
  };

} // namespace pointcloud_to_laserscan

#endif // PCL2LASERSCAN_TRANS__LASERSCAN_TO_POINTCLOUD_NODE_HPP_