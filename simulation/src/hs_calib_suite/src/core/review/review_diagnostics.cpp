#include "hs_calib_suite/core/review/review_diagnostics.hpp"

#include <cmath>
#include <sstream>

#include <opencv2/calib3d.hpp>

#include "hs_calib_suite/core/util/camera_models.hpp"

namespace hs_calib {
namespace core {
namespace {

bool intrinsics_from_result_prefixed(
    const CalibrationResult &result, const std::string &prefix, CameraIntrinsics *out) {
  if (out == nullptr || !result.success) {
    return false;
  }
  if (prefix.empty()) {
    return camera_intrinsics_from_result(result, out);
  }
  CalibrationResult remapped;
  remapped.success = true;
  const std::vector<std::string> keys = {
      "fx", "fy", "cx", "cy", "xi", "model", "image_width", "image_height", "dist_n",
      "k1", "k2", "k3", "k4", "p1", "p2", "rms"};
  bool any = false;
  for (const auto &k : keys) {
    const auto it = result.intrinsics_meta.find(prefix + k);
    if (it != result.intrinsics_meta.end()) {
      remapped.intrinsics_meta[k] = it->second;
      any = true;
    }
  }
  if (!any) {
    // 回退：无前缀字段时用默认
    return camera_intrinsics_from_result(result, out);
  }
  return camera_intrinsics_from_result(remapped, out);
}

std::string basename_of(const std::string &path) {
  const auto pos = path.find_last_of("/\\");
  if (pos == std::string::npos) {
    return path;
  }
  return path.substr(pos + 1);
}

bool obs_to_points(
    const Observation &obs,
    std::vector<cv::Point3f> *obj,
    std::vector<cv::Point2f> *img) {
  if (obj == nullptr || img == nullptr || obs.correspondences.empty()) {
    return false;
  }
  const auto &c = obs.correspondences.front();
  if (c.image_points.rows() < 4 ||
      c.object_points.rows() != c.image_points.rows()) {
    return false;
  }
  obj->clear();
  img->clear();
  obj->reserve(static_cast<size_t>(c.image_points.rows()));
  img->reserve(static_cast<size_t>(c.image_points.rows()));
  for (int i = 0; i < c.image_points.rows(); ++i) {
    obj->emplace_back(
        static_cast<float>(c.object_points(i, 0)),
        static_cast<float>(c.object_points(i, 1)),
        static_cast<float>(c.object_points(i, 2)));
    img->emplace_back(
        static_cast<float>(c.image_points(i, 0)),
        static_cast<float>(c.image_points(i, 1)));
  }
  return !obj->empty();
}

void append_view_diagnostics(
    const ObservationBatch &batch,
    const CameraIntrinsics &intr,
    const std::string &side_filter,
    ReviewDiagnostics *diag) {
  if (diag == nullptr || !intr.valid) {
    return;
  }
  const CameraModelId mid = parse_camera_model(intr.model);
  diag->image_width = std::max(diag->image_width, intr.image_width);
  diag->image_height = std::max(diag->image_height, intr.image_height);

  double sum_sq = 0.0;
  int n_all = 0;

  for (int vi = 0; vi < static_cast<int>(batch.items.size()); ++vi) {
    const auto &obs = batch.items[static_cast<size_t>(vi)];
    std::string side = obs.frame_id;
    if (side != "left" && side != "right") {
      const std::string &p = obs.source_path;
      if (p.rfind("left:", 0) == 0) {
        side = "left";
      } else if (p.rfind("right:", 0) == 0) {
        side = "right";
      } else {
        side.clear();
      }
    }
    if (!side_filter.empty() && side != side_filter) {
      continue;
    }

    ViewResidual vr;
    vr.index = vi;
    vr.side = side;
    std::string name = basename_of(obs.source_path);
    if (name.rfind("left:", 0) == 0) {
      name = name.substr(5);
    } else if (name.rfind("right:", 0) == 0) {
      name = name.substr(6);
    }
    if (name.empty()) {
      name = "view_" + std::to_string(vi);
    }
    if (!side.empty()) {
      vr.label = (side == "left" ? "[L] " : "[R] ") + name;
    } else {
      vr.label = name;
    }

    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;
    if (!obs_to_points(obs, &obj, &img)) {
      vr.ok = false;
      diag->views.push_back(vr);
      continue;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    if (!solve_pnp_model(mid, obj, img, intr.K, intr.D, intr.xi, &rvec, &tvec, false)) {
      vr.ok = false;
      vr.num_points = static_cast<int>(img.size());
      diag->views.push_back(vr);
      continue;
    }

    std::vector<cv::Point2f> proj;
    if (!project_points_model(mid, obj, rvec, tvec, intr.K, intr.D, intr.xi, &proj) ||
        proj.size() != img.size()) {
      vr.ok = false;
      vr.num_points = static_cast<int>(img.size());
      diag->views.push_back(vr);
      continue;
    }

    double sq = 0.0;
    for (size_t i = 0; i < img.size(); ++i) {
      const float du = img[i].x - proj[i].x;
      const float dv = img[i].y - proj[i].y;
      const float e = std::sqrt(du * du + dv * dv);
      sq += static_cast<double>(e) * static_cast<double>(e);
      ResidualPoint rp;
      rp.u = img[i].x;
      rp.v = img[i].y;
      rp.err_px = e;
      rp.view_index = vi;
      diag->points.push_back(rp);
    }
    vr.num_points = static_cast<int>(img.size());
    vr.rms_px = std::sqrt(sq / static_cast<double>(img.size()));
    vr.ok = true;
    sum_sq += sq;
    n_all += vr.num_points;
    diag->views.push_back(vr);
  }

  if (n_all > 0) {
    const double g = std::sqrt(sum_sq / static_cast<double>(n_all));
    if (diag->global_rms_px < 0.0) {
      diag->global_rms_px = g;
    } else {
      // 合并左右时加权近似：取较大全局即可；精确值在外层重算
      diag->global_rms_px =
          std::sqrt(0.5 * (diag->global_rms_px * diag->global_rms_px + g * g));
    }
  }
}

}  // namespace

ReviewDiagnostics compute_review_diagnostics(
    const CalibrationResult &result,
    const ObservationBatch &batch,
    const std::string &meta_prefix) {
  ReviewDiagnostics diag;
  if (!result.success) {
    diag.message = result.message.empty() ? "尚无成功求解结果" : result.message;
    return diag;
  }
  CameraIntrinsics intr;
  if (!intrinsics_from_result_prefixed(result, meta_prefix, &intr) || !intr.valid) {
    diag.message = "无法从结果解析内参，跳过残差诊断";
    return diag;
  }
  if (batch.items.empty()) {
    diag.message = "无观测";
    return diag;
  }

  const std::string side =
      meta_prefix == "left_" ? "left" : (meta_prefix == "right_" ? "right" : "");
  append_view_diagnostics(batch, intr, side, &diag);

  int ok_n = 0;
  for (const auto &v : diag.views) {
    if (v.ok) {
      ++ok_n;
    }
  }
  diag.valid = ok_n > 0 || !diag.points.empty();
  if (!diag.valid) {
    diag.message = "未能计算重投影残差（观测无对应点或 PnP 失败）";
  } else {
    std::ostringstream oss;
    oss << "views=" << ok_n << "/" << diag.views.size();
    if (diag.global_rms_px >= 0.0) {
      oss << " rms=" << diag.global_rms_px << "px";
    }
    diag.message = oss.str();
  }
  return diag;
}

ReviewDiagnostics compute_stereo_review_diagnostics(
    const CalibrationResult &result, const ObservationBatch &batch) {
  ReviewDiagnostics left = compute_review_diagnostics(result, batch, "left_");
  ReviewDiagnostics right = compute_review_diagnostics(result, batch, "right_");

  ReviewDiagnostics merged;
  merged.image_width = std::max(left.image_width, right.image_width);
  merged.image_height = std::max(left.image_height, right.image_height);
  merged.views = left.views;
  merged.views.insert(merged.views.end(), right.views.begin(), right.views.end());
  merged.points = left.points;
  merged.points.insert(merged.points.end(), right.points.begin(), right.points.end());

  double sum_sq = 0.0;
  int n = 0;
  for (const auto &p : merged.points) {
    sum_sq += static_cast<double>(p.err_px) * static_cast<double>(p.err_px);
    ++n;
  }
  if (n > 0) {
    merged.global_rms_px = std::sqrt(sum_sq / static_cast<double>(n));
  }
  merged.valid = !merged.points.empty();
  std::ostringstream oss;
  oss << "L " << left.message << " | R " << right.message;
  merged.message = oss.str();
  return merged;
}

}  // namespace core
}  // namespace hs_calib
