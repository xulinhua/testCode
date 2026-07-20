#include <memory>

#include "nova_grasp_moveit/grasp_arm_executor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nova_grasp_moveit::GraspArmExecutor>();
  // IK 已改为 fully async；单线程 executor 即可，避免回调阻塞导致 /compute_ik 无响应
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
