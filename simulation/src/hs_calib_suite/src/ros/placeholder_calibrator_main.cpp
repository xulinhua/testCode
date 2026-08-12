#include "hs_calib_suite/ros/placeholder_calibrator_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

/// \brief 命令行是否已显式指定 --params-file
bool argv_has_params_file(int argc, char **argv) {
  // 扫描 argv 中的 --params-file / --params-file=...
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--params-file" || arg.rfind("--params-file=", 0) == 0) {
      return true;
    }
  }
  return false;
}

/// \brief 未指定 params-file 时，自动加载包内默认 YAML
rclcpp::NodeOptions make_node_options(int argc, char **argv) {
  rclcpp::NodeOptions options;
  // 已显式指定则保持默认 NodeOptions
  if (argv_has_params_file(argc, argv)) {
    return options;
  }

  // 加载包 share/config 下的默认参数文件
  const std::string share =
      ament_index_cpp::get_package_share_directory("hs_calib_suite");
  const std::string params_file = share + "/config/placeholder_calibrator.yaml";
  options.arguments({
      "--ros-args",
      "--params-file",
      params_file,
  });
  return options;
}

}  // namespace

/// \brief 占位标定节点入口：加载配置并 spin
int main(int argc, char **argv) {
  // 初始化 ROS 并构造占位节点
  rclcpp::init(argc, argv);
  RCLCPP_INFO(
      rclcpp::get_logger("placeholder_calibrator"),
      "placeholder calibrator starting");
  auto node = std::make_shared<hs_calib::ros_wrap::PlaceholderCalibratorNode>(
      make_node_options(argc, argv));
  // 阻塞 spin 直至关闭
  rclcpp::spin(node);
  RCLCPP_INFO(
      rclcpp::get_logger("placeholder_calibrator"),
      "placeholder calibrator shutting down");
  rclcpp::shutdown();
  return 0;
}
