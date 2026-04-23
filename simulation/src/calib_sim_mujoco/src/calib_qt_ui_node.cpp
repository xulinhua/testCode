// 仅启动 Qt UI + CalibQtUiRosNode（标定节点需另行启动或由 launch 组合）。
#include <thread>
#include "calib_sim/calib_qt_ui.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto ros_node = std::make_shared<calib_sim::CalibQtUiRosNode>(options);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(ros_node);
  std::thread spin_thread([&exec]() { exec.spin(); });
  const int rc = calib_sim::RunCalibQtUiApp(ros_node, argc, argv);

  exec.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return rc;
}
