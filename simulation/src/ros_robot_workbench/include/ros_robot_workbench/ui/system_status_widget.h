#ifndef ROS_ROBOT_WORKBENCH__UI__SYSTEM_STATUS_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__SYSTEM_STATUS_WIDGET_H_

#include <functional>
#include <QWidget>

namespace ros_robot_workbench::ui
{

class SystemStatusWidget : public QWidget
{
public:
  explicit SystemStatusWidget(QWidget * parent = nullptr);
  void SetActive(bool active);

private:
  std::function<void(bool)> set_active_;
};

}  // namespace ros_robot_workbench::ui

#endif
