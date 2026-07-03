#ifndef ROS_ROBOT_WORKBENCH__UI__TOPIC_LAB_WIDGET_H_
#define ROS_ROBOT_WORKBENCH__UI__TOPIC_LAB_WIDGET_H_

#include <vector>

#include <QWidget>

#include "ros_robot_workbench/manage/topic_lab_data_manager.hpp"
#include "ros_robot_workbench/module/topic_lab_module.h"

class QLineEdit;
class QTableWidget;

namespace ros_robot_workbench::ui
{

class TopicLabWidget : public QWidget
{
public:
  explicit TopicLabWidget(QWidget * parent = nullptr);

private:
  void reloadTopics();

  manage::TopicLabDataManager dm_;
  std::vector<TopicInfoRow> cached_topics_;
  QTableWidget * topic_table_ = nullptr;
  QLineEdit * filter_edit_ = nullptr;
};

}  // namespace ros_robot_workbench::ui

#endif
