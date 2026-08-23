#pragma once

#include <memory>
#include <string>

#include <QObject>
#include <QString>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

namespace hs_calib {
namespace gui {

/// \brief GUI 内嵌 tf2：查询 base→gripper（T_base_gripper）
class TfPoseBridge : public QObject {
  Q_OBJECT

public:
  /// \brief 构造：创建 tf2 Buffer/Listener 节点
  explicit TfPoseBridge(QObject *parent = nullptr);
  /// \brief 析构
  ~TfPoseBridge() override;

  /// \brief TF 桥与 rclcpp 是否可用
  bool is_ready() const;

  /// \brief 设置 base/gripper 坐标系
  void set_frames(const QString &base_frame, const QString &gripper_frame);

  /// \brief base 坐标系名
  QString base_frame() const { return base_frame_; }
  /// \brief gripper 坐标系名
  QString gripper_frame() const { return gripper_frame_; }

  /// \brief 查最新 TF；成功写出 T_base_gripper
  bool lookup(Eigen::Matrix4d *T_base_gripper, QString *error_out = nullptr);

  /// \brief 泵一次 TF 节点回调
  void spin_some();

  rclcpp::Node::SharedPtr ros_node() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  QString base_frame_ = QStringLiteral("base");
  QString gripper_frame_ = QStringLiteral("tool0");
};

}  // namespace gui
}  // namespace hs_calib
