#include "hs_calib_suite/ros/placeholder_calibrator_node.hpp"

#include <functional>

#include "hs_calib_suite/msg/calibration_result.hpp"

namespace hs_calib {
namespace ros_wrap {

/// \brief 声明参数、注册服务；参数默认值可被 config/*.yaml 覆盖
PlaceholderCalibratorNode::PlaceholderCalibratorNode(
    const rclcpp::NodeOptions &options)
    : Node("hs_calib_placeholder_calibrator", options) {
  // 声明节点参数（可被 YAML / 启动参数覆盖）
  declare_parameter<std::string>("calibrator_id", "placeholder");
  declare_parameter<std::string>("display_name", "Placeholder Calibrator");
  declare_parameter<std::string>("category", "dev");

  // 注册标定与信息查询服务
  calibrate_srv_ = create_service<hs_calib_suite::srv::Calibrate>(
      "~/calibrate",
      std::bind(
          &PlaceholderCalibratorNode::on_calibrate, this,
          std::placeholders::_1, std::placeholders::_2));

  info_srv_ = create_service<hs_calib_suite::srv::GetCalibratorInfo>(
      "~/get_calibrator_info",
      std::bind(
          &PlaceholderCalibratorNode::on_info, this,
          std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
      get_logger(),
      "Placeholder calibrator ready | id=%s | name=%s | category=%s",
      get_parameter("calibrator_id").as_string().c_str(),
      get_parameter("display_name").as_string().c_str(),
      get_parameter("category").as_string().c_str());
}

/// \brief 标定服务回调（当前返回未实现）
void PlaceholderCalibratorNode::on_calibrate(
    const std::shared_ptr<hs_calib_suite::srv::Calibrate::Request> /*request*/,
    std::shared_ptr<hs_calib_suite::srv::Calibrate::Response> response) {
  // 组装占位失败结果（P0 尚未接入 core 求解）
  hs_calib_suite::msg::CalibrationResult result;
  result.success = false;
  result.score = 0.0f;
  result.message =
      "P0 placeholder: implement solver in core/ and call it from ros/.";
  response->success = false;
  response->message = result.message;
  response->results = {result};
  RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
}

/// \brief 查询标定器信息并回填支持的靶标类型
void PlaceholderCalibratorNode::on_info(
    const std::shared_ptr<hs_calib_suite::srv::GetCalibratorInfo::Request> request,
    std::shared_ptr<hs_calib_suite::srv::GetCalibratorInfo::Response> response) {
  // 填充标定器元信息与支持靶标列表
  response->success = true;
  response->message = "ok";
  response->calibrator_id = request->calibrator_id.empty()
      ? get_parameter("calibrator_id").as_string()
      : request->calibrator_id;
  response->display_name = get_parameter("display_name").as_string();
  response->category = get_parameter("category").as_string();
  response->required_frames.clear();
  response->supported_targets = {
      "chessboard", "charuco", "aruco_grid", "trihedral_chess", "trihedral_charuco"};
  RCLCPP_INFO(
      get_logger(),
      "get_calibrator_info ok | id=%s | name=%s | category=%s",
      response->calibrator_id.c_str(), response->display_name.c_str(),
      response->category.c_str());
}

}  // namespace ros_wrap
}  // namespace hs_calib
