#include "ros_robot_workbench/module/tf_viewer_module.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <queue>
#include <set>
#include <unordered_set>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include <QStringList>

#include "ros_robot_workbench/ui/shared_ui_executor.hpp"

namespace ros_robot_workbench::ui
{
namespace
{
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

}  // namespace

class TfViewerBackend::Impl
{
public:
  Impl()
  {
    node_ = std::make_shared<rclcpp::Node>("ros_robot_workbench_tf_viewer");
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    buffer_->setUsingDedicatedThread(true);
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node_, false);
    SharedUiExecutor::instance().add_node(node_);
  }
  ~Impl()
  {
    listener_.reset();
    buffer_.reset();
    if (node_) {
      SharedUiExecutor::instance().remove_node(node_);
      node_.reset();
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
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
    tf2::Quaternion tq(
      tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w);
    tq.normalize();
    tf2::Matrix3x3 m(tq);
    double roll_r = 0.0;
    double pitch_r = 0.0;
    double yaw_r = 0.0;
    m.getRPY(roll_r, pitch_r, yaw_r);
    std::array<std::array<double, 3>, 3> r;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        r[i][j] = m[i][j];
      }
    }
    if (matrix_text != nullptr) {
      *matrix_text = FormatMatrix4x4(r, tx, ty, tz);
    }
    const double roll = roll_r * 180.0 / M_PI;
    const double pitch = pitch_r * 180.0 / M_PI;
    const double yaw = yaw_r * 180.0 / M_PI;
    if (meaning_text != nullptr) {
      *meaning_text = QString(
        "变换方向: %1 -> %2\n"
        "实际意义: 将 %1 坐标系下的点，转换到 %2 坐标系。\n"
        "对应查询: lookupTransform(%2, %1)\n\n"
        "位置偏移 (m):\n"
        "  dx=%3, dy=%4, dz=%5\n\n"
        "角度偏移 (deg, RPY):\n"
        "  roll=%6, pitch=%7, yaw=%8\n\n"
        "终端指令:\n"
        "  ros2 run tf2_ros tf2_echo %2 %1\n\n"
        "At time 0.0\n"
        "- Translation: [%9, %10, %11]\n"
        "- Rotation: in Quaternion (xyzw) [%12, %13, %14, %15]\n"
        "- Rotation: in RPY (radian) [%16, %17, %18]\n"
        "- Rotation: in RPY (degree) [%19, %20, %21]\n")
        .arg(from_frame).arg(to_frame)
        .arg(tx, 0, 'f', 6).arg(ty, 0, 'f', 6).arg(tz, 0, 'f', 6)
        .arg(roll, 0, 'f', 3).arg(pitch, 0, 'f', 3).arg(yaw, 0, 'f', 3)
        .arg(tx, 0, 'f', 3).arg(ty, 0, 'f', 3).arg(tz, 0, 'f', 3)
        .arg(tq.x(), 0, 'f', 3).arg(tq.y(), 0, 'f', 3).arg(tq.z(), 0, 'f', 3).arg(tq.w(), 0, 'f', 3)
        .arg(roll_r, 0, 'f', 3).arg(pitch_r, 0, 'f', 3).arg(yaw_r, 0, 'f', 3)
        .arg(roll, 0, 'f', 3).arg(pitch, 0, 'f', 3).arg(yaw, 0, 'f', 3);
    }
    return true;
  } catch (const tf2::TransformException & e) {
    if (err_msg != nullptr) { *err_msg = QString("查询失败: %1").arg(e.what()); }
    return false;
  }
}

}  // namespace ros_robot_workbench::ui
