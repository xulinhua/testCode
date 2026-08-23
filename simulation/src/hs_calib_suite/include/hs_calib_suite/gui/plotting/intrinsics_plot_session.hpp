#pragma once

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_plot_statistics.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"

namespace hs_calib {
namespace gui {

/// \brief 从会话组装 Tier4 统计图输入
bool build_intrinsics_plot_input(
    const SessionController &session, core::IntrinsicsPlotInput *out);

/// \brief 双目分侧统计图输入（side: "left" / "right"）
bool build_intrinsics_plot_input_for_side(
    const SessionController &session,
    const QString &side,
    core::IntrinsicsPlotInput *out);

}  // namespace gui
}  // namespace hs_calib
