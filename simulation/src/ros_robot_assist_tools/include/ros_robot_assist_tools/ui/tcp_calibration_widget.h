#ifndef ROS_ROBOT_ASSIST_TOOLS__UI__TCP_CALIBRATION_WIDGET_H_
#define ROS_ROBOT_ASSIST_TOOLS__UI__TCP_CALIBRATION_WIDGET_H_

#include <QWidget>

#include "ros_robot_assist_tools/manage/tcp_calibration_data_manager.hpp"
#include "ros_robot_assist_tools/module/tcp_calibration_module.h"

namespace ros_robot_assist_tools::ui
{

class TcpCalibrationWidget : public QWidget
{
public:
  explicit TcpCalibrationWidget(QWidget * parent = nullptr);

private:
  manage::TcpCalibrationDataManager dm_;
  TcpCalibrationResult last_result_{};
  bool has_result_ = false;
};

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__UI__TCP_CALIBRATION_WIDGET_H_
