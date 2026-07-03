#ifndef ROS_ROBOT_WORKBENCH__UI__IMAGE_VIEWER_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__IMAGE_VIEWER_WIDGET_H_

#include <functional>

#include <QWidget>

namespace ros_robot_workbench::ui
{

class ImageViewerWidget : public QWidget
{
public:
  explicit ImageViewerWidget(QWidget * parent = nullptr);
  void SetActive(bool active);

private:
  std::function<void(bool)> set_active_;
};

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__UI__IMAGE_VIEWER_WIDGET_H_
