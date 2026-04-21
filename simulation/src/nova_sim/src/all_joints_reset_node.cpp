// 订阅 Empty 触发：向 /arm_controller/commands 发布全零向量，用于仿真中一键回零。
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{
class AllJointsResetNode final : public rclcpp::Node
{
public:
  AllJointsResetNode()
  : Node("all_joints_reset_node")
  {
    this->declare_parameter("reset_sub_topic", std::string("/nova_sim/reset_all_joints"));
    this->declare_parameter("commands_pub_topic", std::string("/arm_controller/commands"));
    this->declare_parameter("joint_count", 28);

    const std::string sub_topic = this->get_parameter("reset_sub_topic").as_string();
    const std::string pub_topic = this->get_parameter("commands_pub_topic").as_string();
    const int joint_count = this->get_parameter("joint_count").as_int();

    cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(pub_topic, 10);
    sub_ = this->create_subscription<std_msgs::msg::Empty>(
      sub_topic, 10,
      [this, joint_count, pub_topic](std_msgs::msg::Empty::SharedPtr /*msg*/) {
        if (joint_count <= 0) {
          return;
        }
        std_msgs::msg::Float64MultiArray out;
        out.data.assign(static_cast<std::size_t>(joint_count), 0.0);
        cmd_pub_->publish(out);
        RCLCPP_INFO(
          this->get_logger(), "All joints reset: published %d zeros to %s",
          joint_count, pub_topic.c_str());
      });

    RCLCPP_INFO(
      this->get_logger(), "Subscribing %s, publishing zeros to %s (joint_count=%d)",
      sub_topic.c_str(), pub_topic.c_str(), joint_count);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AllJointsResetNode>());
  rclcpp::shutdown();
  return 0;
}
