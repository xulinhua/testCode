#include <thread>

#include "nova_grasp_moveit/grasp_qt_ui.hpp"
#include "nova_grasp_moveit/grasp_ros_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto ros_node = std::make_shared<nova_grasp_moveit::GraspRosNode>(options);
  ros_node->init_tf_listener();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(ros_node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  auto shutdown_hook = [&]() {
      ros_node->prepare_shutdown();
      exec.cancel();
    };

  const int rc = nova_grasp_moveit::RunGraspQtUiApp(ros_node, argc, argv, shutdown_hook);

  shutdown_hook();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
