#include "hs_calib_suite/gui/bridges/tf_pose_bridge.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

namespace hs_calib {
namespace gui {

struct TfPoseBridge::Impl {
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
};

/// \brief 构造：创建 tf2 Buffer/Listener 节点
TfPoseBridge::TfPoseBridge(QObject *parent)
    : QObject(parent), impl_(std::make_unique<Impl>()) {
  if (!rclcpp::ok()) {
    RCLCPP_WARN(
        rclcpp::get_logger("hs_calib_gui"),
        "TF bridge: rclcpp not ok, online hand-eye TF unavailable");
    return;
  }
  impl_->node = std::make_shared<rclcpp::Node>("hs_calib_gui_tf_bridge");
  impl_->buffer = std::make_unique<tf2_ros::Buffer>(impl_->node->get_clock());
  impl_->listener =
      std::make_unique<tf2_ros::TransformListener>(*impl_->buffer, impl_->node, false);
  RCLCPP_INFO(impl_->node->get_logger(), "TF bridge node ready");
}

/// \brief 析构
TfPoseBridge::~TfPoseBridge() = default;

/// \brief TF 桥与 rclcpp 是否可用
bool TfPoseBridge::is_ready() const {
  return impl_ && impl_->node && rclcpp::ok() && impl_->buffer;
}

/// \brief 设置 base/gripper 坐标系
void TfPoseBridge::set_frames(const QString &base_frame, const QString &gripper_frame) {
  base_frame_ = base_frame;
  gripper_frame_ = gripper_frame;
  if (impl_ && impl_->node) {
    RCLCPP_INFO(
        impl_->node->get_logger(), "TF frames: %s -> %s",
        base_frame_.toStdString().c_str(), gripper_frame_.toStdString().c_str());
  }
}

/// \brief 泵一次 TF 节点回调
void TfPoseBridge::spin_some() {
  if (impl_ && impl_->node && rclcpp::ok()) {
    rclcpp::spin_some(impl_->node);
  }
}

/// \brief 查询最新 base→gripper 变换
bool TfPoseBridge::lookup(Eigen::Matrix4d *T_base_gripper, QString *error_out) {
  if (T_base_gripper == nullptr) {
    return false;
  }
  if (!is_ready()) {
    if (error_out) {
      *error_out = QStringLiteral("TF 桥未就绪");
    }
    RCLCPP_WARN(
        rclcpp::get_logger("hs_calib_gui"), "TF lookup skipped: bridge not ready");
    return false;
  }
  try {
    // —— 查最新变换并写成 4x4 ——
    const auto tf = impl_->buffer->lookupTransform(
        base_frame_.toStdString(), gripper_frame_.toStdString(), tf2::TimePointZero);
    const auto &t = tf.transform.translation;
    const auto &q = tf.transform.rotation;
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    *T_base_gripper = Eigen::Matrix4d::Identity();
    T_base_gripper->block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
    (*T_base_gripper)(0, 3) = t.x;
    (*T_base_gripper)(1, 3) = t.y;
    (*T_base_gripper)(2, 3) = t.z;
    return true;
  } catch (const std::exception &ex) {
    if (error_out) {
      *error_out = QString::fromUtf8(ex.what());
    }
    if (impl_ && impl_->node) {
      RCLCPP_DEBUG_THROTTLE(
          impl_->node->get_logger(), *impl_->node->get_clock(), 2000,
          "TF lookup failed %s -> %s: %s",
          base_frame_.toStdString().c_str(), gripper_frame_.toStdString().c_str(),
          ex.what());
    }
    return false;
  }
}

}  // namespace gui
}  // namespace hs_calib
