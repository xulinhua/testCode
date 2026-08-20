#include "hs_calib_suite/core/calibrators/stereo_intrinsics_calibrator.hpp"

#include <sstream>

#include "hs_calib_suite/core/calibrators/cam_intrinsics_calibrator.hpp"
#include "hs_calib_suite/core/registry/registry.hpp"

namespace hs_calib {
namespace core {
namespace {

ObservationBatch filter_side(const ObservationBatch &in, const std::string &side) {
  ObservationBatch out;
  out.notes = in.notes;
  for (const auto &obs : in.items) {
    std::string tag = obs.frame_id;
    // 兼容 source_path 前缀 "left:" / "right:"
    if (tag != "left" && tag != "right") {
      if (obs.source_path.rfind("left:", 0) == 0) {
        tag = "left";
      } else if (obs.source_path.rfind("right:", 0) == 0) {
        tag = "right";
      }
    }
    if (tag == side) {
      out.items.push_back(obs);
    }
  }
  return out;
}

void copy_meta_prefixed(
    const CalibrationResult &src, const std::string &prefix, CalibrationResult *dst) {
  for (const auto &kv : src.intrinsics_meta) {
    dst->intrinsics_meta[prefix + kv.first] = kv.second;
  }
  for (const auto &kv : src.metrics) {
    dst->metrics[prefix + kv.first] = kv.second;
  }
}

}  // namespace

CalibratorInfo StereoIntrinsicsCalibrator::calibrator_info() const {
  CalibratorInfo info;
  info.calibrator_id = "stereo_intrinsics";
  info.display_name = "双目各自内参";
  info.category = "intrinsics";
  info.supported_targets = {
      "chessboard", "charuco", "aruco_grid", "aprilgrid", "circles_symmetric",
      "circles_asymmetric"};
  return info;
}

CalibrationResult StereoIntrinsicsCalibrator::calibrate(
    const ObservationBatch &observations,
    const std::map<std::string, std::string> &config) const {
  CalibrationResult result;
  const ObservationBatch left = filter_side(observations, "left");
  const ObservationBatch right = filter_side(observations, "right");

  CamIntrinsicsCalibrator mono;
  CalibrationResult left_r;
  CalibrationResult right_r;
  std::ostringstream msg;

  if (left.items.empty() && right.items.empty()) {
    result.success = false;
    result.message =
        "没有带左右标记的观测。请在会话里选择「当前采集侧」left/right 后再采集。";
    return result;
  }

  if (!left.items.empty()) {
    left_r = mono.calibrate(left, config);
    if (left_r.success) {
      copy_meta_prefixed(left_r, "left_", &result);
      msg << "left ok (n=" << static_cast<int>(left.items.size()) << ")";
    } else {
      msg << "left fail: " << left_r.message;
    }
  } else {
    msg << "left: no views";
  }
  msg << " | ";
  if (!right.items.empty()) {
    right_r = mono.calibrate(right, config);
    if (right_r.success) {
      copy_meta_prefixed(right_r, "right_", &result);
      msg << "right ok (n=" << static_cast<int>(right.items.size()) << ")";
    } else {
      msg << "right fail: " << right_r.message;
    }
  } else {
    msg << "right: no views";
  }

  result.metrics["num_views_left"] = static_cast<double>(left.items.size());
  result.metrics["num_views_right"] = static_cast<double>(right.items.size());
  if (left_r.success) {
    result.metrics["left_reprojection_rmse"] =
        left_r.metrics.count("reprojection_rmse") ? left_r.metrics.at("reprojection_rmse")
                                                    : 0.0;
  }
  if (right_r.success) {
    result.metrics["right_reprojection_rmse"] =
        right_r.metrics.count("reprojection_rmse") ? right_r.metrics.at("reprojection_rmse")
                                                     : 0.0;
  }

  result.success = left_r.success || right_r.success;
  result.intrinsics_meta["model"] =
      left_r.success ? (left_r.intrinsics_meta.count("model") ? left_r.intrinsics_meta.at("model")
                                                              : "brown_conrady")
                     : (right_r.intrinsics_meta.count("model")
                            ? right_r.intrinsics_meta.at("model")
                            : "brown_conrady");
  result.intrinsics_meta["stereo_mode"] = "separate";
  // 兼容单文件导出：优先写左侧到默认字段，并保留 left_/right_ 前缀
  if (left_r.success) {
    for (const auto &kv : left_r.intrinsics_meta) {
      if (result.intrinsics_meta.count(kv.first) == 0) {
        result.intrinsics_meta[kv.first] = kv.second;
      }
    }
    if (left_r.metrics.count("reprojection_rmse")) {
      result.metrics["reprojection_rmse"] = left_r.metrics.at("reprojection_rmse");
    }
  } else if (right_r.success) {
    for (const auto &kv : right_r.intrinsics_meta) {
      if (result.intrinsics_meta.count(kv.first) == 0) {
        result.intrinsics_meta[kv.first] = kv.second;
      }
    }
    if (right_r.metrics.count("reprojection_rmse")) {
      result.metrics["reprojection_rmse"] = right_r.metrics.at("reprojection_rmse");
    }
  }

  if (left_r.success && right_r.success) {
    result.score = 0.5f * (left_r.score + right_r.score);
    result.message = "stereo_intrinsics: " + msg.str();
  } else if (result.success) {
    result.score = left_r.success ? left_r.score : right_r.score;
    result.message = "stereo_intrinsics (partial): " + msg.str();
  } else {
    result.message = "stereo_intrinsics failed: " + msg.str();
  }
  return result;
}

HS_CALIB_REGISTER("stereo_intrinsics", StereoIntrinsicsCalibrator);

}  // namespace core
}  // namespace hs_calib
