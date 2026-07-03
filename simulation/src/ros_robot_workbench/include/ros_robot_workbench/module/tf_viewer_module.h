#ifndef ROS_ROBOT_WORKBENCH__MODULE__TF_VIEWER_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__TF_VIEWER_MODULE_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <QString>

namespace ros_robot_workbench::ui
{

struct TfViewerSnapshot
{
  std::vector<QString> frames;
  std::unordered_map<std::string, std::string> parent_by_frame;
  std::unordered_map<std::string, std::vector<std::string>> adjacency;
  std::unordered_map<std::string, double> rate_hz_by_frame;
  std::unordered_map<std::string, bool> is_static_by_frame;
};

class TfViewerBackend
{
public:
  TfViewerBackend();
  ~TfViewerBackend();

  bool BuildSnapshot(TfViewerSnapshot * out, QString * err_msg) const;
  std::vector<QString> ConnectedFrames(const TfViewerSnapshot & snapshot, const QString & frame) const;
  QString FrameTypeText(const TfViewerSnapshot & snapshot, const QString & frame) const;
  bool BuildTransformReport(
    const QString & from_frame, const QString & to_frame,
    QString * matrix_text, QString * meaning_text, QString * err_msg) const;

private:
  class Impl;
  Impl * impl_;
};

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__MODULE__TF_VIEWER_MODULE_H_
