#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_profile.hpp"

#include <algorithm>
#include <cctype>

namespace hs_calib {
namespace core {
namespace {

std::string lower_copy(std::string s) {
  for (char &c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

IntrinsicsProfile make_classic() {
  IntrinsicsProfile p;
  p.id = "classic";
  p.display_name = "Classic";
  p.solver = IntrinsicsSolverKind::OpenCV;
  p.radial_coeffs = 2;
  p.rational_coeffs = 0;
  p.use_ransac_pre_rejection = false;
  p.use_entropy_subsampling = false;
  p.use_post_rejection = false;
  p.filter_capture_by_reproj = false;
  return p;
}

IntrinsicsProfile make_general() {
  IntrinsicsProfile p;
  p.id = "general";
  p.display_name = "General";
  p.solver = IntrinsicsSolverKind::OpenCV;
  p.radial_coeffs = 2;
  p.rational_coeffs = 0;
  p.pre_rejection_max_rms_error = 0.5;
  p.post_rejection_max_rms_error = 0.5;
  p.capture_max_reproj_error = 2.0;
  p.capture_max_rms_reproj_error = 0.5;
  return p;
}

IntrinsicsProfile make_c1() {
  IntrinsicsProfile p = make_general();
  p.id = "c1";
  p.display_name = "C1";
  p.radial_coeffs = 3;
  p.pre_rejection_max_rms_error = 0.35;
  p.post_rejection_max_rms_error = 0.35;
  p.capture_max_reproj_error = 0.5;
  p.capture_max_rms_reproj_error = 0.3;
  return p;
}

IntrinsicsProfile make_ceres() {
  IntrinsicsProfile p = make_general();
  p.id = "ceres";
  p.display_name = "Ceres";
  p.solver = IntrinsicsSolverKind::Ceres;
  p.radial_coeffs = 3;
  p.rational_coeffs = 3;
  p.coeffs_regularization_weight = 0.2;
  p.fov_regularization_weight = 0.0;
  return p;
}

IntrinsicsProfile make_c2() {
  IntrinsicsProfile p = make_ceres();
  p.id = "c2";
  p.display_name = "C2";
  p.fov_regularization_weight = 0.05;
  return p;
}

}  // namespace

bool is_tier4_intrinsics_profile(const std::string &profile_id) {
  const std::string key = lower_copy(profile_id);
  return key == "general" || key == "c1" || key == "ceres" || key == "c2";
}

IntrinsicsProfile intrinsics_profile_from_id(const std::string &id) {
  const std::string key = lower_copy(id);
  if (key == "classic" || key == "legacy" || key == "opencv") {
    return make_classic();
  }
  if (key == "c1") {
    return make_c1();
  }
  if (key == "ceres") {
    return make_ceres();
  }
  if (key == "c2") {
    return make_c2();
  }
  if (key == "general") {
    return make_general();
  }
  return make_classic();
}

IntrinsicsProfile intrinsics_profile_from_config(
    const std::map<std::string, std::string> &config) {
  const auto it = config.find("intrinsics_profile");
  if (it == config.end()) {
    return make_classic();
  }
  return intrinsics_profile_from_id(it->second);
}

bool tier4_intrinsics_enabled(const std::map<std::string, std::string> &config) {
  const auto it = config.find("intrinsics_profile");
  if (it == config.end()) {
    return false;
  }
  return is_tier4_intrinsics_profile(it->second);
}

}  // namespace core
}  // namespace hs_calib
