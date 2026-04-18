#include "calib_sim/calib_node.hpp"
#include "calib_sim/calib_qt_ui.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include <thread>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto pkg_share = ament_index_cpp::get_package_share_directory("calib_sim");
  rclcpp::NodeOptions calib_options;
  calib_options.arguments({"--ros-args", "--params-file", pkg_share + "/config/eye_to_hand.yaml"});
  auto calib_node = std::make_shared<calib_sim::CalibNode>("calib_eye_to_hand_node", false, calib_options);
  calib_node->initAfterSharedPtr(calib_node);
  auto ui_node = std::make_shared<calib_sim::CalibQtUiRosNode>(rclcpp::NodeOptions{});

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(calib_node);
  exec.add_node(ui_node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  const int rc = calib_sim::RunCalibQtUiApp(ui_node, argc, argv);
  (void)rc;
  exec.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
