#ifndef ROS_ROBOT_WORKBENCH__UI__ROSBAG_WORKBENCH_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__ROSBAG_WORKBENCH_WIDGET_H_

#include <QWidget>

class QProcess;

namespace ros_robot_workbench::ui
{

class RosbagWorkbenchWidget : public QWidget
{
public:
  explicit RosbagWorkbenchWidget(QWidget * parent = nullptr);
  ~RosbagWorkbenchWidget() override;

private:
  QProcess * record_proc_ = nullptr;
  QProcess * play_proc_ = nullptr;
};

}  // namespace ros_robot_workbench::ui

#endif
