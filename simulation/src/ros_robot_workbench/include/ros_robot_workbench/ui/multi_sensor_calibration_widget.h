#ifndef ROS_ROBOT_WORKBENCH__UI__MULTI_SENSOR_CALIBRATION_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__MULTI_SENSOR_CALIBRATION_WIDGET_H_

#include <QWidget>

namespace ros_robot_workbench::ui
{

class MultiSensorCalibrationWidget : public QWidget
{
public:
  explicit MultiSensorCalibrationWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
