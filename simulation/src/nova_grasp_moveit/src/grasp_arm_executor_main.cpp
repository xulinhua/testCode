#include <memory>

#include "nova_grasp_moveit/grasp_arm_executor.hpp"
#include "rclcpp/rclcpp.hpp"

// 独立进程运行 IK executor，使 Qt 节点只负责规划编排；即使 UI 线程执行
// 候选筛选，姿态命令的 /compute_ik 响应仍可由本节点持续处理。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nova_grasp_moveit::GraspArmExecutor>();
  // IK 已改为 fully async；单线程 executor 即可，避免回调阻塞导致 /compute_ik 无响应
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
