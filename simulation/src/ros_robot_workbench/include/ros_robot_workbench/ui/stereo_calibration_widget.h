#ifndef ROS_ROBOT_WORKBENCH__UI__STEREO_CALIBRATION_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__STEREO_CALIBRATION_WIDGET_H_

#include <QWidget>

namespace ros_robot_workbench::ui
{

class StereoCalibrationWidget : public QWidget
{
public:
  explicit StereoCalibrationWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_workbench::ui

#endif
