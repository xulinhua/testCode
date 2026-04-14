#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nova_sim/moveit2_arm_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveIt2ArmExecutorCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
