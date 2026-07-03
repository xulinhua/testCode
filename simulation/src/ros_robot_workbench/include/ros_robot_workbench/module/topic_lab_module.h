#ifndef ROS_ROBOT_WORKBENCH__MODULE__TOPIC_LAB_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__TOPIC_LAB_MODULE_H_

#include <QString>
#include <vector>

namespace ros_robot_workbench::ui
{

struct TopicInfoRow
{
  QString name;
  QString type;
};

QString TopicLabModuleSummary();

bool ListOnlineTopics(std::vector<TopicInfoRow> * topics, QString * err_msg);

bool EchoTopicOnce(const QString & topic, QString * output, QString * err_msg, int timeout_sec = 5);

bool QueryTopicHz(const QString & topic, double * hz, QString * err_msg, int sample_sec = 2);

bool PublishStringOnce(const QString & topic, const QString & data, QString * err_msg);

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__MODULE__TOPIC_LAB_MODULE_H_
