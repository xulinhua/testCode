#ifndef ROS_ROBOT_WORKBENCH__UI__INTRINSIC_CALIBRATION_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__INTRINSIC_CALIBRATION_WIDGET_H_

#include <QWidget>

namespace ros_robot_workbench::ui
{

class IntrinsicCalibrationWidget : public QWidget
{
public:
  explicit IntrinsicCalibrationWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
