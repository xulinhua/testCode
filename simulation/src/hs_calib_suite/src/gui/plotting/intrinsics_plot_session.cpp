#include "hs_calib_suite/gui/plotting/intrinsics_plot_session.hpp"

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"

namespace hs_calib {
namespace gui {

bool build_intrinsics_plot_input(
    const SessionController &session, core::IntrinsicsPlotInput *out) {
  if (out == nullptr || !session.is_intrinsics()) {
    return false;
  }
  const auto &state = session.intrinsics_state();
  out->collector = &state.collector();
  out->profile = core::profile_from_config_map(session.solve_options());
  out->collector_params = state.collector().params();
  out->extras = core::calibration_extras_from_config(session.solve_options());
  out->solve_config = session.solve_options();
  const auto train_batch = state.training_batch();
  if (!train_batch.items.empty()) {
    out->image_width = train_batch.items.front().image_width;
    out->image_height = train_batch.items.front().image_height;
  }
  out->has_calibrated = state.has_calibrated_model();
  out->has_singleshot =
      state.has_singleshot_model() || state.provisional_model().valid;
  if (out->has_calibrated) {
    out->calibrated_K = state.calibrated_K();
    out->calibrated_D = state.calibrated_D();
  }
  if (state.has_singleshot_model()) {
    out->singleshot_K = state.singleshot_model().camera_matrix;
    out->singleshot_D = state.singleshot_model().dist_coeffs;
  } else if (out->has_singleshot) {
    out->singleshot_K = state.provisional_model().camera_matrix;
    out->singleshot_D = state.provisional_model().dist_coeffs;
  }
  return out->image_width > 0 && out->image_height > 0;
}

}  // namespace gui
}  // namespace hs_calib
