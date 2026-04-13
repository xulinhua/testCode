#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Configuration parameters for easy debugging and modification
const std::string INPUT_TOPIC_NAME = "/camera/depth_pcl";
const std::string OUTPUT_TOPIC_NAME = "/camera/pt_transformed_out";
const std::string OUTPUT_FRAME_ID = "camera_link";//chassis odom camera_link

class PointCloudTransformer : public rclcpp::Node
{
public:
  PointCloudTransformer() : Node("point_cloud_transformer")
  {
    // 订阅原始点云话题
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      INPUT_TOPIC_NAME, 10, std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

    // 发布转换后的点云话题
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      OUTPUT_TOPIC_NAME, 10);  

    // 初始化TF2缓冲区和监听器
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 计算变换矩阵：rpy=(-1.57, 0, -1.57) 对应于绕X轴旋转-90度，绕Z轴旋转-90度
    tf2::Quaternion q;
    // 注意：tf2的setRPY参数顺序是(roll, pitch, yaw)，对应绕X轴、Y轴、Z轴的旋转
    q.setRPY(-1.57, 0, -1.57);
    transform_.setRotation(q);
    transform_.setOrigin(tf2::Vector3(0, 0, 0));
    
    // 打印变换信息，用于调试
    RCLCPP_INFO(this->get_logger(), "Transform initialized with rotation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
                q.x(), q.y(), q.z(), q.w());
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      // 检查消息是否为空
      if (!msg) {
        RCLCPP_WARN(this->get_logger(), "Received null point cloud message");
        return;
      }

      // 检查消息数据是否为空
      if (msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud message");
        return;
      }

      // 打印输入点云信息
      RCLCPP_INFO(this->get_logger(), "Received point cloud with %u points, frame_id: %s", 
                  static_cast<unsigned int>(msg->width * msg->height), msg->header.frame_id.c_str());

      // 将ROS点云消息转换为PCL点云
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::fromROSMsg(*msg, pcl_cloud);

      // 检查转换后的点云是否为空
      if (pcl_cloud.points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty point cloud after conversion");
        return;
      }

      // 打印转换前的点云信息
      RCLCPP_INFO(this->get_logger(), "Converted to PCL cloud with %zu points", pcl_cloud.points.size());
      if (!pcl_cloud.points.empty()) {
        RCLCPP_INFO(this->get_logger(), "First point before transform: x=%.3f, y=%.3f, z=%.3f", 
                    pcl_cloud.points[0].x, pcl_cloud.points[0].y, pcl_cloud.points[0].z);
      }

      // 创建转换后的点云
      pcl::PointCloud<pcl::PointXYZ> transformed_pcl_cloud;
      transformed_pcl_cloud.header = pcl_cloud.header;
      transformed_pcl_cloud.width = pcl_cloud.width;
      transformed_pcl_cloud.height = pcl_cloud.height;
      transformed_pcl_cloud.is_dense = pcl_cloud.is_dense;
      transformed_pcl_cloud.points.resize(pcl_cloud.points.size());

      // 对每个点应用变换
      for (size_t i = 0; i < pcl_cloud.points.size(); ++i)
      {
        const auto& point = pcl_cloud.points[i];
        tf2::Vector3 p(point.x, point.y, point.z);
        tf2::Vector3 transformed_p = transform_ * p;
        transformed_pcl_cloud.points[i].x = transformed_p.x();
        transformed_pcl_cloud.points[i].y = transformed_p.y();
        transformed_pcl_cloud.points[i].z = transformed_p.z();
      }

      // 打印转换后的点云信息
      if (!transformed_pcl_cloud.points.empty()) {
        RCLCPP_INFO(this->get_logger(), "First point after transform: x=%.3f, y=%.3f, z=%.3f", 
                    transformed_pcl_cloud.points[0].x, transformed_pcl_cloud.points[0].y, transformed_pcl_cloud.points[0].z);
      }

      // 将PCL点云转换回ROS点云消息
      sensor_msgs::msg::PointCloud2 transformed_cloud;
      pcl::toROSMsg(transformed_pcl_cloud, transformed_cloud);
      // 先复制header，然后修改frame_id
      transformed_cloud.header = msg->header;
      transformed_cloud.header.frame_id = OUTPUT_FRAME_ID;

      // 再次确认frame_id设置
      RCLCPP_INFO(this->get_logger(), "Setting frame_id to: %s", OUTPUT_FRAME_ID.c_str());

      // 打印输出点云信息
      RCLCPP_INFO(this->get_logger(), "Publishing transformed point cloud with %u points, frame_id: %s", 
                  static_cast<unsigned int>(transformed_cloud.width * transformed_cloud.height), transformed_cloud.header.frame_id.c_str());

      // 发布转换后的点云
      publisher_->publish(transformed_cloud);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown error processing point cloud");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2::Transform transform_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}