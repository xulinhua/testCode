#include "ros_robot_assist_tools/module/tf_viewer_module.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <queue>
#include <set>
#include <thread>
#include <unordered_set>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <QStringList>

namespace ros_robot_assist_tools::ui
{
namespace
{
struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

double Clamp(double v) { return std::max(-1.0, std::min(1.0, v)); }

QString FormatAligned(double value, int width = 11, int precision = 6)
{
  return QString("%1").arg(value, width, 'f', precision);
}

QString FormatMatrix4x4(
  const std::array<std::array<double, 3>, 3> & r,
  double tx, double ty, double tz)
{
  return QString(
    "[%1 %2 %3 %4\n"
    " %5 %6 %7 %8\n"
    " %9 %10 %11 %12\n"
    " %13 %14 %15 %16 ]")
    .arg(FormatAligned(r[0][0])).arg(FormatAligned(r[0][1])).arg(FormatAligned(r[0][2])).arg(FormatAligned(tx))
    .arg(FormatAligned(r[1][0])).arg(FormatAligned(r[1][1])).arg(FormatAligned(r[1][2])).arg(FormatAligned(ty))
    .arg(FormatAligned(r[2][0])).arg(FormatAligned(r[2][1])).arg(FormatAligned(r[2][2])).arg(FormatAligned(tz))
    .arg(FormatAligned(0.0)).arg(FormatAligned(0.0)).arg(FormatAligned(0.0)).arg(FormatAligned(1.0));
}

Quaternion Normalize(const Quaternion & q)
{
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (n < 1e-12) { return {}; }
  return {q.x / n, q.y / n, q.z / n, q.w / n};
}

std::array<std::array<double, 3>, 3> RotationFromQuaternion(const Quaternion & q_in)
{
  const Quaternion q = Normalize(q_in);
  const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
  const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
  return {{
      {{1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)}},
      {{2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)}},
      {{2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)}}
    }};
}
}  // namespace

class TfViewerBackend::Impl
{
public:
  Impl()
  {
    node_ = std::make_shared<rclcpp::Node>("ros_robot_assist_tools_tf_viewer");
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    buffer_->setUsingDedicatedThread(true);
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node_, false);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }
  ~Impl()
  {
    if (executor_ != nullptr) { executor_->cancel(); }
    if (spin_thread_.joinable()) { spin_thread_.join(); }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TfViewerBackend::TfViewerBackend() : impl_(new Impl()) {}
TfViewerBackend::~TfViewerBackend() { delete impl_; }

bool TfViewerBackend::BuildSnapshot(TfViewerSnapshot * out, QString * err_msg) const
{
  if (out == nullptr) {
    if (err_msg != nullptr) { *err_msg = "internal snapshot is null"; }
    return false;
  }
  try {
    const YAML::Node root = YAML::Load(impl_->buffer_->allFramesAsYAML());
    if (!root || !root.IsMap()) {
      if (err_msg != nullptr) { *err_msg = "TF tree is empty, waiting for /tf and /tf_static"; }
      return false;
    }
    TfViewerSnapshot snapshot;
    std::set<std::string> all_frames;
    for (auto it = root.begin(); it != root.end(); ++it) {
      const std::string child = it->first.as<std::string>();
      all_frames.insert(child);
      const double rate_hz = it->second["rate"] ? it->second["rate"].as<double>() : 0.0;
      const std::string broadcaster = it->second["broadcaster"] ? it->second["broadcaster"].as<std::string>() : "";
      const QString broadcaster_q = QString::fromStdString(broadcaster).toLower();
      const bool is_static = (rate_hz <= 0.01) || broadcaster_q.contains("static");
      snapshot.rate_hz_by_frame[child] = rate_hz;
      snapshot.is_static_by_frame[child] = is_static;
      if (!it->second["parent"]) { continue; }
      const std::string parent = it->second["parent"].as<std::string>();
      if (parent.empty() || parent == "NO_PARENT") { continue; }
      all_frames.insert(parent);
      snapshot.parent_by_frame[child] = parent;
      snapshot.adjacency[child].push_back(parent);
      snapshot.adjacency[parent].push_back(child);
    }
    for (const auto & frame : all_frames) {
      snapshot.frames.push_back(QString::fromStdString(frame));
    }
    std::sort(snapshot.frames.begin(), snapshot.frames.end());
    *out = std::move(snapshot);
    return true;
  } catch (const std::exception & e) {
    if (err_msg != nullptr) { *err_msg = QString("TF tree parse failed: %1").arg(e.what()); }
    return false;
  }
}

std::vector<QString> TfViewerBackend::ConnectedFrames(const TfViewerSnapshot & snapshot, const QString & frame) const
{
  std::vector<QString> rows;
  const std::string start = frame.toStdString();
  if (start.empty() || snapshot.adjacency.find(start) == snapshot.adjacency.end()) { return rows; }
  std::unordered_set<std::string> visited;
  std::queue<std::string> q;
  visited.insert(start);
  q.push(start);
  while (!q.empty()) {
    const std::string curr = q.front();
    q.pop();
    if (curr != start) { rows.push_back(QString::fromStdString(curr)); }
    for (const auto & next : snapshot.adjacency.at(curr)) {
      if (visited.insert(next).second) { q.push(next); }
    }
  }
  std::sort(rows.begin(), rows.end());
  return rows;
}

QString TfViewerBackend::FrameTypeText(const TfViewerSnapshot & snapshot, const QString & frame) const
{
  const std::string key = frame.toStdString();
  auto static_it = snapshot.is_static_by_frame.find(key);
  if (static_it == snapshot.is_static_by_frame.end()) return "-";
  if (static_it->second) return "静态TF";
  auto rate_it = snapshot.rate_hz_by_frame.find(key);
  if (rate_it == snapshot.rate_hz_by_frame.end()) return "动态TF";
  return QString("动态TF (%1 Hz)").arg(rate_it->second, 0, 'f', 2);
}

bool TfViewerBackend::BuildTransformReport(
  const QString & from_frame, const QString & to_frame,
  QString * matrix_text, QString * meaning_text, QString * err_msg) const
{
  if (from_frame.trimmed().isEmpty() || to_frame.trimmed().isEmpty()) {
    if (err_msg != nullptr) { *err_msg = "请选择两个 frame"; }
    return false;
  }
  try {
    const auto tf = impl_->buffer_->lookupTransform(
      to_frame.toStdString(), from_frame.toStdString(), tf2::TimePointZero, tf2::durationFromSec(0.2));
    const double tx = tf.transform.translation.x, ty = tf.transform.translation.y, tz = tf.transform.translation.z;
    const Quaternion qn = Normalize({tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w});
    const auto r = RotationFromQuaternion(qn);
    const double roll = std::atan2(r[2][1], r[2][2]) * 180.0 / M_PI;
    const double pitch = std::asin(-Clamp(r[2][0])) * 180.0 / M_PI;
    const double yaw = std::atan2(r[1][0], r[0][0]) * 180.0 / M_PI;
    const double trans_norm = std::sqrt(tx * tx + ty * ty + tz * tz);
    const double rot_deg = 2.0 * std::acos(Clamp(qn.w)) * 180.0 / M_PI;
    if (matrix_text != nullptr) {
      *matrix_text = FormatMatrix4x4(r, tx, ty, tz);
    }
    if (meaning_text != nullptr) {
      *meaning_text = QString(
        "变换方向: %1 -> %2\n"
        "实际意义: 将 %1 坐标系下的点，转换到 %2 坐标系。\n\n"
        "位置偏差 (m):\n  dx=%3, dy=%4, dz=%5\n  平移模长=%6\n\n"
        "角度偏差 (deg, ZYX):\n  roll=%7, pitch=%8, yaw=%9\n  总旋转角=%10")
        .arg(from_frame).arg(to_frame)
        .arg(tx, 0, 'f', 6).arg(ty, 0, 'f', 6).arg(tz, 0, 'f', 6).arg(trans_norm, 0, 'f', 6)
        .arg(roll, 0, 'f', 3).arg(pitch, 0, 'f', 3).arg(yaw, 0, 'f', 3).arg(rot_deg, 0, 'f', 3);
    }
    return true;
  } catch (const tf2::TransformException & e) {
    if (err_msg != nullptr) { *err_msg = QString("查询失败: %1").arg(e.what()); }
    return false;
  }
}

}  // namespace ros_robot_assist_tools::ui
