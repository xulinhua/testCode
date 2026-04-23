#ifndef ROS_ROBOT_ASSIST_TOOLS__UI__KINEMATICS_SOLVER_WIDGET_H_
#define ROS_ROBOT_ASSIST_TOOLS__UI__KINEMATICS_SOLVER_WIDGET_H_

#include <QWidget>

namespace ros_robot_assist_tools::ui
{

class KinematicsSolverWidget : public QWidget
{
public:
  explicit KinematicsSolverWidget(QWidget * parent = nullptr);
};

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__UI__KINEMATICS_SOLVER_WIDGET_H_
