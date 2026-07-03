#ifndef ROS_ROBOT_WORKBENCH__UI__KINEMATICS_SOLVER_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__KINEMATICS_SOLVER_WIDGET_H_

#include <memory>
#include <vector>

#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include "ros_robot_workbench/manage/kinematics_solver_data_manager.hpp"
#include "ros_robot_workbench/module/kinematics_solver_module.h"

class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QGroupBox;
class QLineEdit;
class QPlainTextEdit;
class QLabel;
class QTextEdit;
class QTableWidget;

namespace ros_robot_workbench::ui
{

class KinematicsSolverWidget : public QWidget
{
public:
  explicit KinematicsSolverWidget(QWidget * parent = nullptr);

private:
  void appendLog(const QString & s);
  void storeFields();
  void storeDiff();
  void storeAck();
  void applyConfiguration();
  void runFk();
  void runIk();
  void refreshUrdfLinkOptions();
  void refreshFkJointInputs();
  void refreshMdhTable();

  manage::KinematicsSolverDataManager dm_;
  rclcpp::Node::SharedPtr ros_node_;
  ArmKdlCache kdl_cache_;

  QComboBox * backend_{nullptr};
  QTextEdit * log_{nullptr};
  QLineEdit * urdf_path_{nullptr};
  QComboBox * base_link_{nullptr};
  QComboBox * tip_link_{nullptr};
  QGroupBox * moveit_block_{nullptr};
  QLineEdit * moveit_group_edit_{nullptr};
  QLineEdit * moveit_iklink_edit_{nullptr};
  QLineEdit * moveit_service_edit_{nullptr};
  QLineEdit * moveit_frame_edit_{nullptr};
  QLineEdit * moveit_node_edit_{nullptr};
  QLabel * moveit_plugin_status_{nullptr};
  QPlainTextEdit * seed_text_{nullptr};
  QFormLayout * fk_joints_form_{nullptr};
  std::vector<QDoubleSpinBox *> fk_joints_spins_;
  QComboBox * dh_mode_combo_{nullptr};
  QComboBox * mdh_angle_unit_combo_{nullptr};
  QComboBox * mdh_length_unit_combo_{nullptr};
  QTableWidget * mdh_table_{nullptr};
  QLineEdit * seed_line_{nullptr};
  QDoubleSpinBox * px_{nullptr};
  QDoubleSpinBox * py_{nullptr};
  QDoubleSpinBox * pz_{nullptr};
  QDoubleSpinBox * qx_{nullptr};
  QDoubleSpinBox * qy_{nullptr};
  QDoubleSpinBox * qz_{nullptr};
  QDoubleSpinBox * qw_{nullptr};
  QDoubleSpinBox * diff_L_{nullptr};
  QDoubleSpinBox * diff_r_{nullptr};
  QDoubleSpinBox * diff_wl_{nullptr};
  QDoubleSpinBox * diff_wr_{nullptr};
  QDoubleSpinBox * diff_v_{nullptr};
  QDoubleSpinBox * diff_omega_{nullptr};
  QDoubleSpinBox * ack_L_{nullptr};
  QDoubleSpinBox * ack_delta_{nullptr};
  QComboBox * ack_model_{nullptr};
  QDoubleSpinBox * ack_W_{nullptr};
  QDoubleSpinBox * ack_delta_in_{nullptr};
  QDoubleSpinBox * ack_delta_out_{nullptr};
  QDoubleSpinBox * ack_k_target_{nullptr};
  QDoubleSpinBox * ack_v_ref_fwd_{nullptr};
  QDoubleSpinBox * ack_v_ref_inv_{nullptr};
  QDoubleSpinBox * ack_omega_target_{nullptr};
};

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__UI__KINEMATICS_SOLVER_WIDGET_H_
