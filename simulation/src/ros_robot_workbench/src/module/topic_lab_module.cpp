#include "ros_robot_workbench/module/topic_lab_module.h"

#include <array>
#include <cstdio>

#include <QRegularExpression>

namespace ros_robot_workbench::ui
{
namespace
{

QString RunBashRos2Command(const QString & cmd, int * exit_code)
{
  const QString wrapped = QString(
    "bash -lc 'source /opt/ros/humble/setup.bash >/dev/null 2>&1 && %1'")
                            .arg(cmd);
  std::array<char, 4096> buf{};
  QString out;
  FILE * pipe = popen(wrapped.toUtf8().constData(), "r");
  if (pipe == nullptr) {
    if (exit_code) {
      *exit_code = -1;
    }
    return out;
  }
  while (fgets(buf.data(), static_cast<int>(buf.size()), pipe) != nullptr) {
    out += QString::fromUtf8(buf.data());
  }
  const int code = pclose(pipe);
  if (exit_code) {
    *exit_code = code;
  }
  return out;
}

}  // namespace

QString TopicLabModuleSummary()
{
  return QStringLiteral("浏览在线 topic、Echo 单条消息、估算 Hz，并支持 std_msgs/String 单次发布。");
}

bool ListOnlineTopics(std::vector<TopicInfoRow> * topics, QString * err_msg)
{
  if (!topics) {
    if (err_msg) {
      *err_msg = "topics is null";
    }
    return false;
  }
  topics->clear();
  int code = 0;
  const QString raw = RunBashRos2Command("ros2 topic list -t 2>/dev/null", &code);
  if (raw.trimmed().isEmpty()) {
    if (err_msg) {
      *err_msg = "未获取到 topic 列表，请确认 ROS 环境已 source 且有节点在运行";
    }
    return false;
  }
  const QRegularExpression re("^\\s*([^\\s\\[]+)\\s*\\[([^\\]]+)\\]\\s*$");
  for (const QString & line : raw.split('\n', Qt::SkipEmptyParts)) {
    const QRegularExpressionMatch m = re.match(line.trimmed());
    if (!m.hasMatch()) {
      continue;
    }
    TopicInfoRow row;
    row.name = m.captured(1).trimmed();
    row.type = m.captured(2).trimmed();
    topics->push_back(row);
  }
  if (topics->empty()) {
    if (err_msg) {
      *err_msg = "topic 列表为空";
    }
    return false;
  }
  return true;
}

bool EchoTopicOnce(const QString & topic, QString * output, QString * err_msg, int timeout_sec)
{
  if (topic.trimmed().isEmpty()) {
    if (err_msg) {
      *err_msg = "topic 为空";
    }
    return false;
  }
  const QString safe_topic = topic.trimmed();
  const QString cmd = QString("timeout %1 ros2 topic echo --once %2 2>&1")
                        .arg(timeout_sec)
                        .arg(safe_topic);
  int code = 0;
  const QString raw = RunBashRos2Command(cmd, &code);
  if (output) {
    *output = raw.trimmed();
  }
  if (raw.trimmed().isEmpty()) {
    if (err_msg) {
      *err_msg = QString("Echo 超时或无数据: %1").arg(safe_topic);
    }
    return false;
  }
  (void)code;
  return true;
}

bool QueryTopicHz(const QString & topic, double * hz, QString * err_msg, int sample_sec)
{
  if (!hz) {
    if (err_msg) {
      *err_msg = "hz is null";
    }
    return false;
  }
  if (topic.trimmed().isEmpty()) {
    if (err_msg) {
      *err_msg = "topic 为空";
    }
    return false;
  }
  const QString cmd = QString("timeout %1 ros2 topic hz %2 2>&1")
                        .arg(sample_sec + 1)
                        .arg(topic.trimmed());
  int code = 0;
  const QString raw = RunBashRos2Command(cmd, &code);
  const QRegularExpression re("average rate:\\s*([0-9.eE+-]+)");
  double best = 0.0;
  bool found = false;
  QRegularExpressionMatchIterator it = re.globalMatch(raw);
  while (it.hasNext()) {
    const QRegularExpressionMatch m = it.next();
    bool ok = false;
    const double v = m.captured(1).toDouble(&ok);
    if (ok) {
      best = v;
      found = true;
    }
  }
  if (!found) {
    if (err_msg) {
      *err_msg = QString("未测到 Hz（可能无发布者）: %1").arg(topic);
    }
    return false;
  }
  *hz = best;
  return true;
}

bool PublishStringOnce(const QString & topic, const QString & data, QString * err_msg)
{
  if (topic.trimmed().isEmpty()) {
    if (err_msg) {
      *err_msg = "topic 为空";
    }
    return false;
  }
  const QString escaped = data;
  const QString cmd = QString("ros2 topic pub --once %1 std_msgs/msg/String \"{data: '%2}'\" 2>&1")
                        .arg(topic.trimmed(), QString(escaped).replace("'", "'\\''"));
  int code = 0;
  const QString raw = RunBashRos2Command(cmd, &code);
  if (code != 0 && !raw.contains("Publishing")) {
    if (err_msg) {
      *err_msg = raw.trimmed().isEmpty() ? "发布失败" : raw.trimmed();
    }
    return false;
  }
  return true;
}

}  // namespace ros_robot_workbench::ui
