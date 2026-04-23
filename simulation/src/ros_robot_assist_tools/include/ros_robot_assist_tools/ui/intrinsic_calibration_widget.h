#ifndef ROS_ROBOT_ASSIST_TOOLS__UI__INTRINSIC_CALIBRATION_WIDGET_H_
#define ROS_ROBOT_ASSIST_TOOLS__UI__INTRINSIC_CALIBRATION_WIDGET_H_

#include <QWidget>

namespace ros_robot_assist_tools::ui
{

class IntrinsicCalibrationWidget : public QWidget
{
public:
  explicit IntrinsicCalibrationWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_assist_tools::ui

#endif
