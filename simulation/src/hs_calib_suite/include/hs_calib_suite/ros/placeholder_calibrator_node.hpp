#pragma once

#include <memory>

#include "hs_calib_suite/srv/calibrate.hpp"
#include "hs_calib_suite/srv/get_calibrator_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hs_calib {
namespace ros_wrap {

/// \brief P0 占位标定节点（ROS 薄封装）
/// 暴露 Calibrate / GetCalibratorInfo 服务；真正求解后续接到 core/。
class PlaceholderCalibratorNode : public rclcpp::Node {
public:
  /// \brief 声明参数并注册 ~/calibrate、~/get_calibrator_info 服务
  explicit PlaceholderCalibratorNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  /// \brief 标定服务回调（当前返回未实现）
  void on_calibrate(
      const std::shared_ptr<hs_calib_suite::srv::Calibrate::Request> request,
      std::shared_ptr<hs_calib_suite::srv::Calibrate::Response> response);

  /// \brief 查询标定器信息
  void on_info(
      const std::shared_ptr<hs_calib_suite::srv::GetCalibratorInfo::Request> request,
      std::shared_ptr<hs_calib_suite::srv::GetCalibratorInfo::Response> response);

  rclcpp::Service<hs_calib_suite::srv::Calibrate>::SharedPtr calibrate_srv_;  // ~/calibrate
  rclcpp::Service<hs_calib_suite::srv::GetCalibratorInfo>::SharedPtr info_srv_;  // ~/get_calibrator_info
};

}  // namespace ros_wrap
}  // namespace hs_calib
