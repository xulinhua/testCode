#include "calib_sim_isaac/calib_qt_ui_presenter_utils.hpp"

#include <QBrush>
#include <QColor>
#include <QPalette>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QTextEdit>

namespace calib_sim_isaac
{

bool IsErrorLogLine(const std::string & line)
{
  // 与旧 UI 行为保持一致：错误前缀或标定失败关键字即视为错误日志。
  return line.rfind("[ERROR]", 0) == 0 || line.find("calibration_failed") != std::string::npos;
}

std::vector<std::string> BuildDisplayLogs(
  const std::vector<std::string> & ui_extra_logs,
  const std::vector<std::string> & ros_logs)
{
  // 展示顺序：先 UI 本地提示，再 ROS 实时日志，便于阅读上下文。
  std::vector<std::string> out = ui_extra_logs;
  out.insert(out.end(), ros_logs.begin(), ros_logs.end());
  return out;
}

void AppendLogLineColored(QTextEdit * text_edit, const QString & line)
{
  // 统一文本渲染规则，避免 UI 主流程里重复处理颜色格式。
  QTextCursor cursor(text_edit->document());
  cursor.movePosition(QTextCursor::End);
  QTextCharFormat fmt;
  if (IsErrorLogLine(line.toStdString())) {
    fmt.setForeground(QBrush(QColor(200, 40, 40)));
  } else {
    fmt.setForeground(QBrush(text_edit->palette().color(QPalette::WindowText)));
  }
  cursor.setCharFormat(fmt);
  cursor.insertText(line + QLatin1Char('\n'));
}

}  // namespace calib_sim_isaac
