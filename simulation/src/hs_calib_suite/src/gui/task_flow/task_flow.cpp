#include "hs_calib_suite/gui/task_flow/task_flow.hpp"

namespace hs_calib {
namespace gui {

TaskFlowKind task_flow_from_calibrator_id(const QString &calibrator_id) {
  if (calibrator_id == QStringLiteral("stereo_intrinsics")) {
    return TaskFlowKind::StereoIntrinsics;
  }
  if (calibrator_id == QStringLiteral("stereo_extrinsics")) {
    return TaskFlowKind::StereoExtrinsics;
  }
  if (calibrator_id == QStringLiteral("trihedral_oneshot")) {
    return TaskFlowKind::Trihedral;
  }
  if (calibrator_id == QStringLiteral("eye_in_hand") ||
      calibrator_id == QStringLiteral("eye_to_hand")) {
    return TaskFlowKind::HandEye;
  }
  return TaskFlowKind::MonoIntrinsics;
}

QString task_flow_display_name(TaskFlowKind kind) {
  switch (kind) {
    case TaskFlowKind::MonoIntrinsics:
      return QStringLiteral("单目内参");
    case TaskFlowKind::StereoIntrinsics:
      return QStringLiteral("双目内参");
    case TaskFlowKind::StereoExtrinsics:
      return QStringLiteral("双目外参");
    case TaskFlowKind::Trihedral:
      return QStringLiteral("直角三面");
    case TaskFlowKind::HandEye:
      return QStringLiteral("手眼标定");
  }
  return QStringLiteral("标定");
}

QString task_flow_step_title(TaskFlowKind kind, const QString &step_name) {
  return QStringLiteral("%1 · %2").arg(task_flow_display_name(kind), step_name);
}

}  // namespace gui
}  // namespace hs_calib
