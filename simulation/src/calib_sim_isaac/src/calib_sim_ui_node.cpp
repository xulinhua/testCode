// OpenCV 高窗 UI 入口：从 share/calib_sim_isaac/config/ui.yaml 读订阅话题名。
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "calib_sim_isaac/calib_ui_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto pkg_share = ament_index_cpp::get_package_share_directory("calib_sim");
  const auto config_path = pkg_share + "/config/ui.yaml";
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", config_path});
  auto node = std::make_shared<calib_sim_isaac::CalibSimUiNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
