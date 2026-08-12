#include "hs_calib_suite/core/calibrators/trihedral_oneshot_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "hs_calib_suite/core/registry/registry.hpp"

namespace hs_calib {
namespace core {

namespace {

/// \brief 从 config 解析整型参数
int parse_int(const std::map<std::string, std::string> &config, const char *key, int def) {
  const auto it = config.find(key);
  if (it == config.end()) {
    return def;
  }
  try {
    return std::stoi(it->second);
  } catch (...) {
    return def;
  }
}

/// \brief 将 double 格式化为高精度字符串
std::string to_str(double v) {
  std::ostringstream oss;
  oss.precision(12);
  oss << v;
  return oss.str();
}

/// \brief 统计对应点覆盖的不同面数量
int count_faces(const Correspondence &c) {
  bool seen[3] = {false, false, false};
  for (int id : c.ids) {
    const int f = id / 1000;
    if (f >= 0 && f < 3) {
      seen[f] = true;
    }
  }
  return static_cast<int>(seen[0]) + static_cast<int>(seen[1]) + static_cast<int>(seen[2]);
}

constexpr int kMinPtsDlt = 6;  ///< OpenCV cvFindExtrinsicCameraParams2 / DLT

/// \brief 检查二维像点坐标是否有限
bool finite2(const cv::Point2f &p) {
  return std::isfinite(p.x) && std::isfinite(p.y);
}
/// \brief 检查三维物点坐标是否有限
bool finite3(const cv::Point3f &p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

/// \brief 去除亚像素吸附后重合的像点对
void dedupe_pairs(
    std::vector<cv::Point3f> *op, std::vector<cv::Point2f> *ip, float min_px) {
  if (op == nullptr || ip == nullptr || op->size() != ip->size()) {
    return;
  }
  std::vector<cv::Point3f> o2;
  std::vector<cv::Point2f> i2;
  o2.reserve(op->size());
  i2.reserve(ip->size());
  const float min2 = min_px * min_px;
  for (size_t i = 0; i < ip->size(); ++i) {
    if (!finite2((*ip)[i]) || !finite3((*op)[i])) {
      continue;
    }
    bool dup = false;
    for (const auto &q : i2) {
      const float dx = (*ip)[i].x - q.x;
      const float dy = (*ip)[i].y - q.y;
      if (dx * dx + dy * dy < min2) {
        dup = true;
        break;
      }
    }
    if (dup) {
      continue;
    }
    o2.push_back((*op)[i]);
    i2.push_back((*ip)[i]);
  }
  *op = std::move(o2);
  *ip = std::move(i2);
}

/// \brief 像点包围盒是否足够大（避免退化）
bool image_span_ok(const std::vector<cv::Point2f> &ip) {
  if (ip.size() < static_cast<size_t>(kMinPtsDlt)) {
    return false;
  }
  const cv::Rect box = cv::boundingRect(ip);
  return box.width >= 16 && box.height >= 16;
}

/// \brief 快速 EPNP 试探位姿是否可解
bool pose_guess_ok(
    const std::vector<cv::Point3f> &op, const std::vector<cv::Point2f> &ip, const cv::Mat &K) {
  if (op.size() < static_cast<size_t>(kMinPtsDlt) || op.size() != ip.size() || K.empty()) {
    return false;
  }
  cv::Mat rvec;
  cv::Mat tvec;
  cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
  try {
    return cv::solvePnP(op, ip, K, D, rvec, tvec, false, cv::SOLVEPNP_EPNP);
  } catch (const cv::Exception &) {
    return false;
  }
}

/// \brief 构造 fx=fy、主点给定的 3×3 内参矩阵
cv::Mat make_K(double fx, double cx, double cy) {
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = fx;
  K.at<double>(1, 1) = fx;
  K.at<double>(0, 2) = cx;
  K.at<double>(1, 2) = cy;
  return K;
}

/// \brief 给定外参计算平均重投影误差（像素）
double reproj_rms(
    const std::vector<cv::Point3f> &op, const std::vector<cv::Point2f> &ip, const cv::Mat &K,
    const cv::Mat &rvec, const cv::Mat &tvec) {
  if (op.empty() || op.size() != ip.size()) {
    return std::numeric_limits<double>::infinity();
  }
  std::vector<cv::Point2f> proj;
  cv::projectPoints(op, rvec, tvec, K, cv::Mat::zeros(5, 1, CV_64F), proj);
  double acc = 0.0;
  for (size_t i = 0; i < ip.size(); ++i) {
    acc += cv::norm(proj[i] - ip[i]);
  }
  return acc / static_cast<double>(ip.size());
}

/// \brief 固定焦距 PnP 并返回重投影 RMSE
bool pnp_rms(
    double fx, double cx, double cy, const std::vector<cv::Point3f> &op,
    const std::vector<cv::Point2f> &ip, cv::Mat *rvec, cv::Mat *tvec, double *rms) {
  const cv::Mat K = make_K(fx, cx, cy);
  const cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
  cv::Mat rv;
  cv::Mat tv;
  try {
    if (!cv::solvePnP(op, ip, K, D, rv, tv, false, cv::SOLVEPNP_EPNP)) {
      return false;
    }
    cv::solvePnP(op, ip, K, D, rv, tv, true, cv::SOLVEPNP_ITERATIVE);
  } catch (const cv::Exception &) {
    return false;
  }
  const double e = reproj_rms(op, ip, K, rv, tv);
  if (!std::isfinite(e)) {
    return false;
  }
  if (rvec) {
    *rvec = rv;
  }
  if (tvec) {
    *tvec = tv;
  }
  if (rms) {
    *rms = e;
  }
  return true;
}

/// \brief 单帧一维搜索焦距 + PnP（单视图内参欠定，不用 calibrateCamera）
bool solve_oneshot_fx(
    const std::vector<cv::Point3f> &op, const std::vector<cv::Point2f> &ip, int width,
    int height, cv::Mat *K_out, cv::Mat *rvec, cv::Mat *tvec, double *rms_out) {
  const double cx = width * 0.5;
  const double cy = height * 0.5;
  const double fx_lo = 0.5 * static_cast<double>(width) / std::tan(55.0 * CV_PI / 180.0);  // ~110°
  const double fx_hi = 0.5 * static_cast<double>(width) / std::tan(12.0 * CV_PI / 180.0);  // ~24°
  double best_fx = 0.5 * static_cast<double>(width) / std::tan(35.0 * CV_PI / 180.0);
  double best_rms = std::numeric_limits<double>::infinity();
  cv::Mat best_r;
  cv::Mat best_t;

  // —— 对数空间粗搜焦距 ——
  const int n_grid = 28;
  for (int i = 0; i <= n_grid; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n_grid);
    const double fx = fx_lo * std::pow(fx_hi / fx_lo, t);  // log-space
    double rms = 0.0;
    cv::Mat rv;
    cv::Mat tv;
    if (!pnp_rms(fx, cx, cy, op, ip, &rv, &tv, &rms)) {
      continue;
    }
    if (rms < best_rms) {
      best_rms = rms;
      best_fx = fx;
      best_r = rv;
      best_t = tv;
    }
  }
  if (!std::isfinite(best_rms)) {
    return false;
  }

  // —— 黄金分割精搜 ——
  double a = std::max(fx_lo, best_fx / 1.25);
  double b = std::min(fx_hi, best_fx * 1.25);
  for (int iter = 0; iter < 18; ++iter) {
    const double m1 = a + (b - a) * 0.382;
    const double m2 = a + (b - a) * 0.618;
    double e1 = 0.0;
    double e2 = 0.0;
    cv::Mat r1;
    cv::Mat t1;
    cv::Mat r2;
    cv::Mat t2;
    const bool ok1 = pnp_rms(m1, cx, cy, op, ip, &r1, &t1, &e1);
    const bool ok2 = pnp_rms(m2, cx, cy, op, ip, &r2, &t2, &e2);
    if (!ok1 && !ok2) {
      break;
    }
    if (!ok1 || (ok2 && e2 < e1)) {
      a = m1;
      if (ok2 && e2 < best_rms) {
        best_rms = e2;
        best_fx = m2;
        best_r = r2;
        best_t = t2;
      }
    } else {
      b = m2;
      if (e1 < best_rms) {
        best_rms = e1;
        best_fx = m1;
        best_r = r1;
        best_t = t1;
      }
    }
  }

  if (K_out) {
    *K_out = make_K(best_fx, cx, cy);
  }
  if (rvec) {
    *rvec = best_r;
  }
  if (tvec) {
    *tvec = best_t;
  }
  if (rms_out) {
    *rms_out = best_rms;
  }
  return best_fx > 1.0 && std::isfinite(best_rms);
}

/// \brief 判断焦距是否在合理 HFOV 范围内
bool fx_plausible(double fx, int width) {
  if (!std::isfinite(fx) || width < 16) {
    return false;
  }
  const double lo = 0.5 * static_cast<double>(width) / std::tan(55.0 * CV_PI / 180.0);
  const double hi = 0.5 * static_cast<double>(width) / std::tan(12.0 * CV_PI / 180.0);
  return fx >= lo * 0.85 && fx <= hi * 1.15;
}

/// \brief 计算向量中位数
double median_of(std::vector<double> v) {
  if (v.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
  std::nth_element(v.begin(), mid, v.end());
  return *mid;
}

/// \brief 多帧：单帧筛帧 → calibrateCamera，发散则中位焦距回退
bool solve_multiview(
    const std::vector<std::vector<cv::Point3f>> &obj_all,
    const std::vector<std::vector<cv::Point2f>> &img_all, int width, int height, cv::Mat *K_out,
    cv::Mat *D_out, std::vector<cv::Mat> *rvecs, std::vector<cv::Mat> *tvecs, double *rms_out,
    int *used_views, int *dropped_views, std::string *note) {
  std::vector<std::vector<cv::Point3f>> obj;
  std::vector<std::vector<cv::Point2f>> img;
  std::vector<double> fxs;
  int dropped = 0;
  // —— 逐帧单帧 PnP 筛帧 ——
  for (size_t i = 0; i < obj_all.size(); ++i) {
    cv::Mat Ki;
    cv::Mat rv;
    cv::Mat tv;
    double e = 0.0;
    if (!solve_oneshot_fx(obj_all[i], img_all[i], width, height, &Ki, &rv, &tv, &e) ||
        e > 5.0 || !fx_plausible(Ki.at<double>(0, 0), width)) {
      ++dropped;
      continue;
    }
    obj.push_back(obj_all[i]);
    img.push_back(img_all[i]);
    fxs.push_back(Ki.at<double>(0, 0));
  }
  if (used_views) {
    *used_views = static_cast<int>(obj.size());
  }
  if (dropped_views) {
    *dropped_views = dropped;
  }
  if (obj.size() < 3) {
    if (note) {
      *note = "多帧中仅 " + std::to_string(obj.size()) +
              " 帧单帧 PnP 合格（RMSE<12px 且焦距合理），无法联合标定。角点与物点对应可能仍有问题。";
    }
    return false;
  }

  const double med_fx = median_of(fxs);
  const double cx = width * 0.5;
  const double cy = height * 0.5;
  cv::Mat K = make_K(med_fx, cx, cy);
  cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
  std::vector<cv::Mat> rv;
  std::vector<cv::Mat> tv;
  double calib_rms = std::numeric_limits<double>::infinity();
  bool calib_ok = false;
  int flags = cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_K3 |
              cv::CALIB_ZERO_TANGENT_DIST;
  if (obj.size() < 8) {
    flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
  }
  // —— 联合 calibrateCamera ——
  try {
    calib_rms = cv::calibrateCamera(
        obj, img, cv::Size(width, height), K, D, rv, tv, flags);
    calib_ok = std::isfinite(calib_rms) && fx_plausible(K.at<double>(0, 0), width) &&
               calib_rms < 3.0;
  } catch (const cv::Exception &) {
    calib_ok = false;
  }

  std::string fallback_note;
  // —— 发散时中位焦距 + 逐帧 PnP 回退 ——
  if (!calib_ok) {
    K = make_K(med_fx, cx, cy);
    D = cv::Mat::zeros(5, 1, CV_64F);
    rv.clear();
    tv.clear();
    double acc = 0.0;
    int n = 0;
    for (size_t i = 0; i < obj.size(); ++i) {
      cv::Mat rvi;
      cv::Mat tvi;
      double e = 0.0;
      if (!pnp_rms(med_fx, cx, cy, obj[i], img[i], &rvi, &tvi, &e)) {
        continue;
      }
      rv.push_back(rvi);
      tv.push_back(tvi);
      acc += e * static_cast<double>(obj[i].size());
      n += static_cast<int>(obj[i].size());
    }
    calib_rms = (n > 0) ? acc / static_cast<double>(n) : std::numeric_limits<double>::infinity();
    if (!std::isfinite(calib_rms) || calib_rms > 12.0 || rv.empty()) {
      if (note) {
        *note =
            "calibrateCamera 发散，单帧焦距中位数回退后 RMSE 仍为 " + to_str(calib_rms) +
            " px。请检查三面 ChArUco 物点对应（面 ID / 字典 / 方格数是否与 Isaac 一致）。";
      }
      return false;
    }
    fallback_note = "calibrateCamera 发散，已用单帧焦距中位数回退";
  }

  if (K_out) {
    *K_out = K;
  }
  if (D_out) {
    *D_out = D;
  }
  if (rvecs) {
    *rvecs = std::move(rv);
  }
  if (tvecs) {
    *tvecs = std::move(tv);
  }
  if (rms_out) {
    *rms_out = calib_rms;
  }
  if (note) {
    *note = fallback_note;
  }
  return true;
}

/// \brief 从观测批次收集有效 3D-2D 对应并校验尺寸
bool collect_points(
    const ObservationBatch &observations,
    const cv::Mat &K_guess,
    std::vector<std::vector<cv::Point3f>> *obj_pts,
    std::vector<std::vector<cv::Point2f>> *img_pts,
    int *image_width,
    int *image_height,
    int *min_faces_out,
    int *skipped_out) {
  obj_pts->clear();
  img_pts->clear();
  *image_width = 0;
  *image_height = 0;
  int min_faces = 99;
  int skipped = 0;
  for (const auto &obs : observations.items) {
    if (obs.correspondences.empty()) {
      ++skipped;
      continue;
    }
    const auto &c = obs.correspondences.front();
    if (c.image_points.rows() < kMinPtsDlt ||
        c.object_points.rows() != c.image_points.rows()) {
      ++skipped;
      continue;
    }
    std::vector<cv::Point3f> op;
    std::vector<cv::Point2f> ip;
    op.reserve(static_cast<size_t>(c.image_points.rows()));
    ip.reserve(static_cast<size_t>(c.image_points.rows()));
    for (int i = 0; i < c.image_points.rows(); ++i) {
      op.emplace_back(
          static_cast<float>(c.object_points(i, 0)),
          static_cast<float>(c.object_points(i, 1)),
          static_cast<float>(c.object_points(i, 2)));
      ip.emplace_back(
          static_cast<float>(c.image_points(i, 0)),
          static_cast<float>(c.image_points(i, 1)));
    }
    dedupe_pairs(&op, &ip, 0.75f);
    if (op.size() < static_cast<size_t>(kMinPtsDlt) || !image_span_ok(ip) ||
        !pose_guess_ok(op, ip, K_guess)) {
      ++skipped;
      continue;
    }
    const int faces = count_faces(c);
    min_faces = std::min(min_faces, faces);
    obj_pts->push_back(std::move(op));
    img_pts->push_back(std::move(ip));
    if (obs.image_width > 0 && obs.image_height > 0) {
      if (*image_width <= 0) {
        *image_width = obs.image_width;
        *image_height = obs.image_height;
      } else if (obs.image_width != *image_width || obs.image_height != *image_height) {
        return false;
      }
    }
  }
  *min_faces_out = (min_faces == 99) ? 0 : min_faces;
  if (skipped_out) {
    *skipped_out = skipped;
  }
  return true;
}

}  // namespace

/// \brief 返回标定器元信息（三面单帧/多帧内参）
CalibratorInfo TrihedralOneshotCalibrator::calibrator_info() const {
  CalibratorInfo info;
  info.calibrator_id = "trihedral_oneshot";
  info.display_name = "直角三面单帧/多帧内参";
  info.category = "extrinsics";
  info.supported_targets = {"trihedral_chess", "trihedral_charuco", "trihedral_aruco"};
  return info;
}

/// \brief 三面靶单帧或多帧内参标定（焦距搜索 / calibrateCamera）
CalibrationResult TrihedralOneshotCalibrator::calibrate(
    const ObservationBatch &observations,
    const std::map<std::string, std::string> &config) const {
  CalibrationResult result;
  (void)parse_int(config, "squares_x", 9);
  (void)parse_int(config, "squares_y", 6);

  // —— 获取图像尺寸 ——
  int image_width = 0;
  int image_height = 0;
  for (const auto &obs : observations.items) {
    if (obs.image_width > 0 && obs.image_height > 0) {
      image_width = obs.image_width;
      image_height = obs.image_height;
      break;
    }
  }
  if (image_width <= 0 || image_height <= 0) {
    result.success = false;
    result.message = "缺少图像尺寸";
    return result;
  }

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = 0.5 * static_cast<double>(image_width) / std::tan(35.0 * CV_PI / 180.0);
  K.at<double>(1, 1) = K.at<double>(0, 0);
  K.at<double>(0, 2) = image_width * 0.5;
  K.at<double>(1, 2) = image_height * 0.5;

  // —— 收集并校验有效观测 ——
  std::vector<std::vector<cv::Point3f>> obj_pts;
  std::vector<std::vector<cv::Point2f>> img_pts;
  int min_faces = 0;
  int skipped = 0;
  if (!collect_points(
          observations, K, &obj_pts, &img_pts, &image_width, &image_height, &min_faces,
          &skipped)) {
    result.success = false;
    result.message = "观测图像尺寸不一致";
    return result;
  }

  const int n_raw = static_cast<int>(observations.items.size());
  if (obj_pts.empty()) {
    result.success = false;
    result.message =
        "无有效三面靶观测（每帧至少 6 个不重合角点；OpenCV DLT 要求）。已跳过 " +
        std::to_string(skipped) + "/" + std::to_string(n_raw) + " 帧";
    return result;
  }

  // 单帧需要非共面结构（≥2 面）；面内可为部分角点，但总点数须足够
  const bool oneshot = obj_pts.size() == 1;
  if (oneshot && min_faces < 2) {
    result.success = false;
    result.message = "单帧标定至少需要检测到 2 个面（可部分角点）";
    return result;
  }
  if (oneshot && obj_pts.front().size() < 12) {
    result.success = false;
    result.message = "单帧有效角点过少（部分检测也需足够 3D 约束）";
    return result;
  }
  const int min_views = parse_int(config, "min_views", oneshot ? 1 : 3);
  if (!oneshot && static_cast<int>(obj_pts.size()) < std::max(3, min_views)) {
    result.success = false;
    result.message = "有效观测不足：可用 " + std::to_string(obj_pts.size()) + " 帧（点数≥6），至少 " +
                     std::to_string(std::max(3, min_views)) + " 帧；已跳过 " +
                     std::to_string(skipped) + "/" + std::to_string(n_raw);
    return result;
  }

  cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  double rms = 0.0;

  // —— 单帧或多帧求解 ——
  if (oneshot) {
    cv::Mat rvec;
    cv::Mat tvec;
    if (!solve_oneshot_fx(
            obj_pts.front(), img_pts.front(), image_width, image_height, &K, &rvec, &tvec,
            &rms) ||
        !fx_plausible(K.at<double>(0, 0), image_width)) {
      result.success = false;
      result.message = "单帧焦距搜索失败（请确认至少两面、角点对应正确）";
      return result;
    }
    rvecs.push_back(rvec);
    tvecs.push_back(tvec);
    if (rms > 25.0) {
      result.success = false;
      result.message =
          "单帧重投影误差过大 (" + to_str(rms) +
          " px)。常见原因：面 ID / 物点对应错误，或只看到近乎共面的点。请换姿态或改用多帧。";
      result.metrics["reprojection_rmse"] = rms;
      return result;
    }
  } else {
    int used = 0;
    int dropped_pnp = 0;
    std::string note;
    if (!solve_multiview(
            obj_pts, img_pts, image_width, image_height, &K, &D, &rvecs, &tvecs, &rms, &used,
            &dropped_pnp, &note)) {
      result.success = false;
      result.message = note.empty() ? "多帧标定失败" : note;
      result.metrics["skipped_views"] = static_cast<double>(skipped + dropped_pnp);
      return result;
    }
    skipped += dropped_pnp;
    if (!note.empty()) {
      result.intrinsics_meta["solver_note"] = note;
    }
  }

  // —— 首帧外参写入 transforms ——
  if (!rvecs.empty()) {
    cv::Mat R;
    cv::Rodrigues(rvecs[0], R);
    result.transforms["camera"]["board"] = Eigen::Matrix4d::Identity();
    auto &T = result.transforms["camera"]["board"];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        T(r, c) = R.at<double>(r, c);
      }
      T(r, 3) = tvecs[0].at<double>(r, 0);
    }
  }

  result.success = true;
  result.score = static_cast<float>(1.0 / (1.0 + rms));
  if (oneshot) {
    result.message = rms <= 5.0 ? "trihedral oneshot ok"
                                : ("trihedral oneshot ok，RMSE=" + to_str(rms) +
                                   " px（略高，建议再采几帧精化）");
  } else {
    const auto note_it = result.intrinsics_meta.find("solver_note");
    result.message = (note_it != result.intrinsics_meta.end() && !note_it->second.empty())
                         ? ("trihedral multi-view ok（" + note_it->second + "）")
                         : "trihedral multi-view ok";
  }
  result.metrics["reprojection_rmse"] = rms;
  result.metrics["num_views"] = static_cast<double>(obj_pts.size());
  result.metrics["skipped_views"] = static_cast<double>(skipped);
  result.metrics["min_faces"] = static_cast<double>(min_faces);

  result.intrinsics_meta["fx"] = to_str(K.at<double>(0, 0));
  result.intrinsics_meta["fy"] = to_str(K.at<double>(1, 1));
  result.intrinsics_meta["cx"] = to_str(K.at<double>(0, 2));
  result.intrinsics_meta["cy"] = to_str(K.at<double>(1, 2));
  result.intrinsics_meta["k1"] = to_str(D.at<double>(0, 0));
  result.intrinsics_meta["k2"] = to_str(D.at<double>(1, 0));
  result.intrinsics_meta["p1"] = to_str(D.at<double>(2, 0));
  result.intrinsics_meta["p2"] = to_str(D.at<double>(3, 0));
  result.intrinsics_meta["k3"] = to_str(D.rows > 4 ? D.at<double>(4, 0) : 0.0);
  result.intrinsics_meta["image_width"] = std::to_string(image_width);
  result.intrinsics_meta["image_height"] = std::to_string(image_height);
  result.intrinsics_meta["rms"] = to_str(rms);
  result.intrinsics_meta["model"] = "pinhole";
  result.intrinsics_meta["mode"] = oneshot ? "oneshot" : "multi";
  result.intrinsics_meta["parent_frame"] = "camera";
  result.intrinsics_meta["child_frame"] = "board";
  return result;
}

HS_CALIB_REGISTER("trihedral_oneshot", TrihedralOneshotCalibrator);

}  // namespace core
}  // namespace hs_calib
