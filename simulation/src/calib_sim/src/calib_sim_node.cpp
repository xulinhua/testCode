#include "calib_sim/calib_node.hpp"
#include "calib_sim/calib_qt_ui.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include <thread>

/// 统一标定节点 + Qt：眼在手外(eth0/eth1) 与眼在手上(eih0/eih1) 通过界面下拉框 set_mode 切换，
/// 参数见 share/calib_sim/config/calib_unified.yaml。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto pkg_share = ament_index_cpp::get_package_share_directory("calib_sim");
  rclcpp::NodeOptions calib_options;
  calib_options.arguments({"--ros-args", "--params-file", pkg_share + "/config/calib_unified.yaml"});
  auto calib_node = std::make_shared<calib_sim::CalibNode>("calib_sim", calib_options);
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
