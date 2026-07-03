#ifndef ROS_ROBOT_WORKBENCH__UI__WORKBENCH_MODULE_REGISTRY_HPP_
#define ROS_ROBOT_WORKBENCH__UI__WORKBENCH_MODULE_REGISTRY_HPP_

#include <functional>
#include <vector>

#include <QStackedWidget>
#include <QString>
#include <QStringList>
#include <QWidget>

namespace ros_robot_workbench::ui
{

using WorkbenchPageFactory = std::function<QWidget *(QWidget * parent)>;

struct WorkbenchModuleSpec
{
  QString kit;
  QString title;
  WorkbenchPageFactory factory;
  bool lazy = true;
};

std::vector<WorkbenchModuleSpec> GetWorkbenchModuleSpecs();

/// 构建 Kit 分组导航，向 stack 注册全部页面，返回扁平页面名列表（供菜单/跳转）。
QWidget * BuildKitNavigationPanel(
  QStackedWidget * stack,
  QStringList * flat_page_names,
  int * image_viewer_stack_index);

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__UI__WORKBENCH_MODULE_REGISTRY_HPP_
