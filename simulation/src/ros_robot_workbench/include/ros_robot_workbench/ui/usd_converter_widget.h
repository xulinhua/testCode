#ifndef ROS_ROBOT_WORKBENCH__UI__USD_CONVERTER_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__USD_CONVERTER_WIDGET_H_

#include <QWidget>

#include "ros_robot_workbench/manage/usd_converter_data_manager.hpp"
#include "ros_robot_workbench/module/usd_converter_module.h"

class QCheckBox;
class QComboBox;
class QLineEdit;
class QProcess;

namespace ros_robot_workbench::ui
{

class UsdConverterWidget : public QWidget
{
public:
  explicit UsdConverterWidget(QWidget * parent = nullptr);
  ~UsdConverterWidget() override;

private:
  void updateBackendOptions();
  UsdConvertRequest buildRequest() const;

  manage::UsdConverterDataManager dm_;
  QProcess * convert_proc_ = nullptr;
  QComboBox * input_kind_ = nullptr;
  QComboBox * backend_ = nullptr;
  QLineEdit * input_path_ = nullptr;
  QLineEdit * output_usd_ = nullptr;
  QLineEdit * mesh_root_prim_ = nullptr;
  QLineEdit * isaac_python_ = nullptr;
  QLineEdit * python_venv_ = nullptr;
  QCheckBox * merge_fixed_ = nullptr;
  QCheckBox * fix_base_ = nullptr;
  QCheckBox * expand_xacro_ = nullptr;
  QWidget * urdf_opts_ = nullptr;
  QWidget * mesh_opts_ = nullptr;
};

}  // namespace ros_robot_workbench::ui

#endif
