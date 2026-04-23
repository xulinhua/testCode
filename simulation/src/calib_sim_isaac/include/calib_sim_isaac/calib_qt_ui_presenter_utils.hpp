#ifndef CALIB_SIM__CALIB_QT_UI_PRESENTER_UTILS_HPP_
#define CALIB_SIM__CALIB_QT_UI_PRESENTER_UTILS_HPP_

#include <string>
#include <vector>

class QTextEdit;
class QString;

namespace calib_sim_isaac
{

/// 判断一行日志是否应按错误色显示。
bool IsErrorLogLine(const std::string & line);
/// 拼接 UI 本地日志与 ROS 实时日志，形成当前应显示的日志快照。
std::vector<std::string> BuildDisplayLogs(
  const std::vector<std::string> & ui_extra_logs,
  const std::vector<std::string> & ros_logs);
/// 向 QTextEdit 追加一行并按错误/普通文本着色。
void AppendLogLineColored(QTextEdit * text_edit, const QString & line);

}  // namespace calib_sim_isaac

#endif  // CALIB_SIM__CALIB_QT_UI_PRESENTER_UTILS_HPP_
