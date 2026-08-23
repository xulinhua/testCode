#pragma once

#include <QString>

namespace hs_calib {
namespace gui {

/// \brief 工程标定任务流（各步骤独立页面）
enum class TaskFlowKind {
  MonoIntrinsics = 0,
  StereoIntrinsics = 1,
  StereoExtrinsics = 2,
  Trihedral = 3,
  HandEye = 4,
};

/// \brief 标定器 ID → 任务流
TaskFlowKind task_flow_from_calibrator_id(const QString &calibrator_id);

/// \brief 任务流显示名
QString task_flow_display_name(TaskFlowKind kind);

/// \brief 步骤页标题（如「单目内参 · 数据源设置」）
QString task_flow_step_title(TaskFlowKind kind, const QString &step_name);

}  // namespace gui
}  // namespace hs_calib
