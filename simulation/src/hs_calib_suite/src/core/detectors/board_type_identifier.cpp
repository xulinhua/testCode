#include "hs_calib_suite/core/detectors/board_type_identifier.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include "hs_calib_suite/core/detectors/aprilgrid_detector.hpp"
#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/detectors/aruco_grid_detector.hpp"
#include "hs_calib_suite/core/detectors/charuco_detector.hpp"
#include "hs_calib_suite/core/detectors/chessboard_detector.hpp"
#include "hs_calib_suite/core/detectors/circle_grid_detector.hpp"
#include "hs_calib_suite/core/detectors/trihedral_charuco_detector.hpp"
#include "hs_calib_suite/core/detectors/trihedral_chess_detector.hpp"
#include "hs_calib_suite/core/targets/aprilgrid_target.hpp"
#include "hs_calib_suite/core/targets/aruco_grid_target.hpp"
#include "hs_calib_suite/core/targets/charuco_target.hpp"
#include "hs_calib_suite/core/targets/chessboard_target.hpp"
#include "hs_calib_suite/core/targets/circle_grid_target.hpp"
#include "hs_calib_suite/core/targets/trihedral_target.hpp"
#include "hs_calib_suite/core/util/cv_bridge_local.hpp"

namespace hs_calib {
namespace core {
namespace {

constexpr double kPlaceholderLen = 0.025;
constexpr double kPlaceholderMarker = 0.018;

bool type_enabled(
    const std::vector<std::string> &candidates, const std::string &id) {
  if (candidates.empty()) {
    return true;
  }
  return std::find(candidates.begin(), candidates.end(), id) != candidates.end();
}

double area_ratio_of_points(
    const std::vector<cv::Point2f> &pts, int width, int height) {
  if (pts.size() < 3 || width <= 0 || height <= 0) {
    return 0.0;
  }
  const cv::Rect box = cv::boundingRect(pts);
  return static_cast<double>(box.area()) /
         (static_cast<double>(width) * static_cast<double>(height));
}

double clamp01(double v) {
  return std::max(0.0, std::min(1.0, v));
}

/// \brief 类型先验：码类靶优先于棋盘/圆点（圆点阵极易在 Charuco/棋盘上误检）
int type_priority(const std::string &type_id) {
  if (type_id == "trihedral_charuco") {
    return 100;
  }
  if (type_id == "trihedral_chess") {
    return 95;
  }
  if (type_id == "charuco") {
    return 90;
  }
  if (type_id == "aprilgrid") {
    return 85;
  }
  if (type_id == "aruco_grid") {
    return 80;
  }
  if (type_id == "aruco") {
    return 55;
  }
  if (type_id == "chessboard") {
    return 35;
  }
  if (type_id == "circles_asymmetric" || type_id == "circles_symmetric") {
    return 10;
  }
  return 0;
}

bool is_circle_type(const std::string &type_id) {
  return type_id == "circles_symmetric" || type_id == "circles_asymmetric";
}

void upsert_hypothesis(
    std::vector<BoardTypeHypothesis> *out, BoardTypeHypothesis hyp) {
  if (out == nullptr || hyp.score <= 0.0) {
    return;
  }
  for (auto &existing : *out) {
    if (existing.type_id == hyp.type_id) {
      if (hyp.score > existing.score) {
        existing = std::move(hyp);
      }
      return;
    }
  }
  out->push_back(std::move(hyp));
}

/// \brief 有明确 ArUco/AprilTag 证据时，压低无码靶假阳性
void apply_marker_evidence(
    int marker_count, std::vector<BoardTypeHypothesis> *ranked) {
  if (ranked == nullptr || marker_count < 3) {
    return;
  }
  for (auto &h : *ranked) {
    if (is_circle_type(h.type_id)) {
      // 圆点阵不含编码码；多码命中时几乎可排除
      h.score *= 0.08;
      if (h.note.find("有码") == std::string::npos) {
        h.note += (h.note.empty() ? "" : " · ");
        h.note += "有码检出，圆点阵可能性极低";
      }
    } else if (h.type_id == "chessboard") {
      h.score *= 0.2;
      if (h.note.find("有码") == std::string::npos) {
        h.note += (h.note.empty() ? "" : " · ");
        h.note += "有码检出，纯棋盘可能性低";
      }
    } else if (h.type_id == "charuco") {
      h.score = clamp01(h.score + 0.12 + 0.008 * static_cast<double>(marker_count));
    } else if (h.type_id == "aruco_grid" || h.type_id == "aprilgrid") {
      h.score = clamp01(h.score + 0.06 + 0.004 * static_cast<double>(marker_count));
    } else if (h.type_id == "aruco") {
      h.score = clamp01(h.score + 0.03);
    }
  }
}

bool hypothesis_better(
    const BoardTypeHypothesis &a, const BoardTypeHypothesis &b) {
  if (std::abs(a.score - b.score) > 1e-4) {
    return a.score > b.score;
  }
  const int pa = type_priority(a.type_id);
  const int pb = type_priority(b.type_id);
  if (pa != pb) {
    return pa > pb;
  }
  return a.feature_count > b.feature_count;
}

std::vector<std::string> default_dicts(
    const std::vector<std::string> &hints, int max_scan) {
  static const char *kBuiltin[] = {
      "DICT_4X4_50",
      "DICT_4X4_100",
      "DICT_4X4_250",
      "DICT_5X5_100",
      "DICT_5X5_250",
      "DICT_6X6_250",
      "DICT_6X6_1000",
      "DICT_APRILTAG_36h11",
  };
  std::vector<std::string> out;
  std::unordered_set<std::string> seen;
  auto push = [&](const std::string &d) {
    if (d.empty() || !seen.insert(d).second) {
      return;
    }
    out.push_back(d);
  };
  for (const auto &h : hints) {
    push(h);
  }
  for (const char *d : kBuiltin) {
    push(d);
    if (max_scan > 0 && static_cast<int>(out.size()) >= max_scan) {
      break;
    }
  }
  return out;
}

/// \brief 在常见字典上扫 ArUco，返回命中最多的一组
bool scan_best_markers(
    const cv::Mat &bgr, const std::vector<std::string> &dicts,
    DetectedMarkers *best) {
  if (best == nullptr || bgr.empty()) {
    return false;
  }
  best->corners.clear();
  best->ids.clear();
  best->dictionary_name.clear();
  auto params = make_aruco_detector_params();
  params.minMarkerPerimeterRate = 0.02;
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  cv::Mat eq;
  cv::createCLAHE(2.0, cv::Size(8, 8))->apply(gray, eq);

  for (const auto &name : dicts) {
    DetectedMarkers cur;
    try {
      const auto dict = make_aruco_dictionary(name);
      aruco_detect_markers(eq, dict, cur.corners, cur.ids, params);
    } catch (...) {
      continue;
    }
    if (cur.ids.size() > best->ids.size()) {
      cur.dictionary_name = name;
      *best = std::move(cur);
    }
    // 已足够多，提前停
    if (best->ids.size() >= 12) {
      break;
    }
  }
  return !best->empty();
}

void try_charuco(
    const ImageFrame &frame, const cv::Mat &bgr, const DetectedMarkers &markers,
    const std::vector<std::string> &candidates,
    std::vector<BoardTypeHypothesis> *out) {
  if (!type_enabled(candidates, "charuco") || markers.empty()) {
    return;
  }
  static const cv::Size kBoards[] = {
      {5, 7}, {7, 5}, {8, 6}, {6, 8}, {5, 5}, {12, 9}, {4, 6}, {6, 4},
  };
  int best_corners = 0;
  std::string best_note;
  for (const auto &sz : kBoards) {
    CharucoTarget target(
        sz.width, sz.height, kPlaceholderLen, kPlaceholderMarker,
        markers.dictionary_name.empty() ? "DICT_4X4_50" : markers.dictionary_name);
    DetectedMarkers tmp;
    const auto corrs = CharucoDetector(target).detect(frame, &tmp);
    if (corrs.empty()) {
      continue;
    }
    const int n = static_cast<int>(corrs.front().image_points.rows());
    if (n > best_corners) {
      best_corners = n;
      best_note = "网格试探 " + std::to_string(sz.width) + "x" +
                  std::to_string(sz.height);
    }
  }
  if (best_corners < 4) {
    return;
  }
  std::vector<cv::Point2f> marker_pts;
  for (const auto &c : markers.corners) {
    marker_pts.insert(marker_pts.end(), c.begin(), c.end());
  }
  BoardTypeHypothesis h;
  h.type_id = "charuco";
  h.feature_count = best_corners;
  h.dict_hint = markers.dictionary_name;
  h.note = best_note;
  const double cover =
      area_ratio_of_points(marker_pts, bgr.cols, bgr.rows);
  // Charuco = 棋盘角点 + 码；码数与角点都计入，饱和上限低于「假圆点阵满分」
  h.score = clamp01(
      0.42 + 0.018 * static_cast<double>(best_corners) +
      0.012 * static_cast<double>(markers.ids.size()) + 0.28 * cover);
  upsert_hypothesis(out, std::move(h));
}

void try_aruco_family(
    const ImageFrame &frame, const cv::Mat &bgr, const DetectedMarkers &markers,
    const std::vector<std::string> &candidates,
    std::vector<BoardTypeHypothesis> *out) {
  if (markers.empty()) {
    return;
  }
  const int n_markers = static_cast<int>(markers.ids.size());
  std::vector<cv::Point2f> all_pts;
  for (const auto &c : markers.corners) {
    all_pts.insert(all_pts.end(), c.begin(), c.end());
  }
  const double cover = area_ratio_of_points(all_pts, bgr.cols, bgr.rows);

  // AprilTag 字典更偏向 aprilgrid
  const bool april_dict =
      markers.dictionary_name.find("APRILTAG") != std::string::npos;
  if (april_dict && type_enabled(candidates, "aprilgrid")) {
    static const cv::Size kGrids[] = {{6, 6}, {4, 3}, {5, 4}, {8, 6}, {3, 2}};
    int best_pts = 0;
    std::string note;
    for (const auto &sz : kGrids) {
      AprilgridTarget target(sz.width, sz.height, 0.088, 0.3);
      DetectedMarkers tmp;
      const auto corrs = AprilgridDetector(target).detect(frame, &tmp);
      if (corrs.empty()) {
        continue;
      }
      const int n = static_cast<int>(corrs.front().image_points.rows());
      if (n > best_pts) {
        best_pts = n;
        note = "tagCols×tagRows " + std::to_string(sz.width) + "x" +
               std::to_string(sz.height);
      }
    }
    if (best_pts >= 4) {
      BoardTypeHypothesis h;
      h.type_id = "aprilgrid";
      h.feature_count = best_pts;
      h.dict_hint = "DICT_APRILTAG_36h11";
      h.note = note;
      h.score = clamp01(0.4 + 0.03 * static_cast<double>(best_pts / 4) + 0.3 * cover);
      upsert_hypothesis(out, std::move(h));
    }
  }

  if (type_enabled(candidates, "aruco_grid") && n_markers >= 4) {
    static const cv::Size kGrids[] = {{5, 7}, {4, 6}, {3, 4}, {6, 6}, {2, 3}};
    int best_pts = 0;
    std::string note;
    const std::string dict =
        markers.dictionary_name.empty() ? "DICT_6X6_1000" : markers.dictionary_name;
    for (const auto &sz : kGrids) {
      ArucoGridTarget target(sz.width, sz.height, 0.04, 0.01, dict);
      DetectedMarkers tmp;
      const auto corrs = ArucoGridDetector(target).detect(frame, &tmp);
      if (corrs.empty()) {
        continue;
      }
      const int n = static_cast<int>(corrs.front().image_points.rows());
      if (n > best_pts) {
        best_pts = n;
        note = "markers " + std::to_string(sz.width) + "x" + std::to_string(sz.height);
      }
    }
    BoardTypeHypothesis h;
    h.type_id = "aruco_grid";
    h.feature_count = n_markers;
    h.dict_hint = dict;
    h.note = best_pts > 0 ? note : "多码命中，网格布局待确认";
    h.score = clamp01(
        (best_pts > 0 ? 0.40 : 0.26) + 0.018 * static_cast<double>(n_markers) +
        0.22 * cover);
    upsert_hypothesis(out, std::move(h));
  }

  if (type_enabled(candidates, "aruco")) {
    BoardTypeHypothesis h;
    h.type_id = "aruco";
    h.feature_count = n_markers;
    h.dict_hint = markers.dictionary_name;
    h.note = "检出 " + std::to_string(n_markers) + " 个码";
    // 单码/少码更偏向 aruco；多码时分数略低，让 grid/charuco 优先
    h.score = clamp01(
        0.25 + 0.08 * std::min(n_markers, 6) + 0.2 * cover -
        (n_markers >= 6 ? 0.08 : 0.0));
    upsert_hypothesis(out, std::move(h));
  }
}

void try_chessboard(
    const ImageFrame &frame, const cv::Mat &bgr,
    const std::vector<std::string> &candidates,
    std::vector<BoardTypeHypothesis> *out) {
  if (!type_enabled(candidates, "chessboard")) {
    return;
  }
  static const cv::Size kPatterns[] = {
      {9, 6}, {8, 6}, {7, 5}, {11, 8}, {6, 4}, {8, 5}, {10, 7}, {5, 4}, {7, 7},
  };
  ChessboardDetectOptions dopts;
  dopts.fast_check = false;
  dopts.allow_partial = true;
  dopts.thorough = false;
  int best_n = 0;
  cv::Size best_sz;
  std::vector<cv::Point2f> best_pts;
  for (const auto &sz : kPatterns) {
    ChessboardTarget target(sz.width, sz.height, kPlaceholderLen);
    const auto corrs = ChessboardDetector(target, dopts).detect(frame);
    if (corrs.empty()) {
      continue;
    }
    const int n = static_cast<int>(corrs.front().image_points.rows());
    if (n <= best_n) {
      continue;
    }
    best_n = n;
    best_sz = sz;
    best_pts.clear();
    best_pts.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      best_pts.emplace_back(
          static_cast<float>(corrs.front().image_points(i, 0)),
          static_cast<float>(corrs.front().image_points(i, 1)));
    }
  }
  if (best_n < 9) {
    return;
  }
  BoardTypeHypothesis h;
  h.type_id = "chessboard";
  h.feature_count = best_n;
  h.note = "内角点试探 " + std::to_string(best_sz.width) + "x" +
           std::to_string(best_sz.height);
  const double cover = area_ratio_of_points(best_pts, bgr.cols, bgr.rows);
  h.score = clamp01(0.3 + 0.015 * static_cast<double>(best_n) + 0.35 * cover);
  upsert_hypothesis(out, std::move(h));
}

void try_circles(
    const ImageFrame &frame, const cv::Mat &bgr,
    const std::vector<std::string> &candidates, int marker_count,
    std::vector<BoardTypeHypothesis> *out) {
  // 已有多个编码码时，圆点阵假阳性极高（Charuco 白格/码心会被当成圆）
  if (marker_count >= 3) {
    return;
  }
  auto run = [&](const char *type_id, CircleGridPattern pattern,
                 const std::vector<cv::Size> &sizes) {
    if (!type_enabled(candidates, type_id)) {
      return;
    }
    int best_n = 0;
    cv::Size best_sz;
    std::vector<cv::Point2f> best_pts;
    for (const auto &sz : sizes) {
      CircleGridTarget target(sz.width, sz.height, kPlaceholderLen, pattern);
      const auto corrs = CircleGridDetector(target).detect(frame);
      if (corrs.empty()) {
        continue;
      }
      const int n = static_cast<int>(corrs.front().image_points.rows());
      // 要求接近完整阵列，避免零星误检抬高分
      const int expect = sz.width * sz.height;
      if (n < std::max(12, expect * 3 / 4)) {
        continue;
      }
      if (n <= best_n) {
        continue;
      }
      best_n = n;
      best_sz = sz;
      best_pts.clear();
      for (int i = 0; i < n; ++i) {
        best_pts.emplace_back(
            static_cast<float>(corrs.front().image_points(i, 0)),
            static_cast<float>(corrs.front().image_points(i, 1)));
      }
    }
    if (best_n < 12) {
      return;
    }
    BoardTypeHypothesis h;
    h.type_id = type_id;
    h.feature_count = best_n;
    h.note = "阵列 " + std::to_string(best_sz.width) + "x" +
             std::to_string(best_sz.height);
    const double cover = area_ratio_of_points(best_pts, bgr.cols, bgr.rows);
    // 圆点阵基分更保守，避免轻易顶到满分压过码类靶
    h.score = clamp01(0.22 + 0.012 * static_cast<double>(best_n) + 0.30 * cover);
    upsert_hypothesis(out, std::move(h));
  };

  run("circles_symmetric", CircleGridPattern::Symmetric,
      {{7, 7}, {8, 8}, {9, 9}, {6, 6}, {5, 5}});
  run("circles_asymmetric", CircleGridPattern::Asymmetric,
      {{4, 11}, {3, 9}, {5, 13}, {4, 9}});
}

void try_trihedral(
    const ImageFrame &frame, const DetectedMarkers &markers,
    const std::vector<std::string> &candidates,
    std::vector<BoardTypeHypothesis> *out) {
  TrihedralTarget geom(8, 8, kPlaceholderLen);
  if (type_enabled(candidates, "trihedral_chess")) {
    TrihedralChessDetector detector(geom);
    int faces = 0;
    const Correspondence corr = detector.detect_merged(
        frame, &faces, TrihedralChessDetectSpeed::Fast);
    const int n = static_cast<int>(corr.image_points.rows());
    if (faces >= 2 && n >= 12) {
      BoardTypeHypothesis h;
      h.type_id = "trihedral_chess";
      h.feature_count = n;
      h.note = "检出约 " + std::to_string(faces) + " 面";
      h.score = clamp01(0.42 + 0.12 * static_cast<double>(faces - 1) +
                        0.01 * static_cast<double>(std::min(n, 80)));
      upsert_hypothesis(out, std::move(h));
    }
  }
  if (type_enabled(candidates, "trihedral_charuco") && !markers.empty()) {
    const std::string dict =
        markers.dictionary_name.empty() ? "DICT_4X4_50" : markers.dictionary_name;
    TrihedralCharucoDetector detector(geom, kPlaceholderMarker, dict);
    int faces = 0;
    DetectedMarkers tmp;
    const Correspondence corr =
        detector.detect_merged(frame, &faces, &tmp, true);
    const int n = static_cast<int>(corr.image_points.rows());
    if (faces >= 2 && n >= 8) {
      BoardTypeHypothesis h;
      h.type_id = "trihedral_charuco";
      h.feature_count = n;
      h.dict_hint = dict;
      h.note = "检出约 " + std::to_string(faces) + " 面 · 码 " +
               std::to_string(static_cast<int>(markers.ids.size()));
      h.score = clamp01(0.48 + 0.12 * static_cast<double>(faces - 1) +
                        0.015 * static_cast<double>(std::min(n, 60)));
      upsert_hypothesis(out, std::move(h));
    }
  }
}

void draw_top_overlay(
    cv::Mat *bgr, const BoardTypeHypothesis &top, const DetectedMarkers *markers) {
  if (bgr == nullptr || bgr->empty()) {
    return;
  }
  // 仅当 Top-1 是码类靶时画码框，避免「圆点阵第一却叠着 ArUco id」造成误导
  const bool marker_top =
      top.type_id == "charuco" || top.type_id == "aruco" ||
      top.type_id == "aruco_grid" || top.type_id == "aprilgrid" ||
      top.type_id == "trihedral_charuco";
  if (marker_top && markers != nullptr && !markers->empty()) {
    // 细边框 + 小号数字 ID（与检测台叠加风格一致）
    static const cv::Scalar kPalette[] = {
        cv::Scalar(0, 255, 255), cv::Scalar(0, 200, 0), cv::Scalar(255, 128, 0),
        cv::Scalar(0, 128, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 255, 0),
    };
    constexpr int kPaletteN =
        static_cast<int>(sizeof(kPalette) / sizeof(kPalette[0]));
    for (size_t i = 0; i < markers->ids.size() && i < markers->corners.size();
         ++i) {
      const auto &corners = markers->corners[i];
      if (corners.size() < 4) {
        continue;
      }
      const cv::Scalar &col = kPalette[static_cast<int>(i) % kPaletteN];
      std::vector<cv::Point> poly;
      cv::Point2f center(0.f, 0.f);
      for (const auto &p : corners) {
        poly.emplace_back(cvRound(p.x), cvRound(p.y));
        center += p;
      }
      center *= 1.f / static_cast<float>(corners.size());
      const cv::Point *pts = poly.data();
      const int npts = static_cast<int>(poly.size());
      cv::polylines(*bgr, &pts, &npts, 1, true, col, 1, cv::LINE_AA);
      const std::string label = std::to_string(markers->ids[i]);
      int baseline = 0;
      const double font_scale = 0.35;
      const cv::Size tsz = cv::getTextSize(
          label, cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, &baseline);
      const cv::Point org(
          cvRound(center.x) - tsz.width / 2, cvRound(center.y) + tsz.height / 2);
      cv::putText(
          *bgr, label, org, cv::FONT_HERSHEY_SIMPLEX, font_scale,
          cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
      cv::putText(
          *bgr, label, org, cv::FONT_HERSHEY_SIMPLEX, font_scale, col, 1,
          cv::LINE_AA);
    }
  }
  const std::string line =
      "type=" + top.type_id + " score=" +
      cv::format("%.2f", top.score) + " feats=" + std::to_string(top.feature_count);
  cv::putText(
      *bgr, line, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7,
      cv::Scalar(40, 220, 80), 2, cv::LINE_AA);
  if (!top.dict_hint.empty()) {
    cv::putText(
        *bgr, "dict=" + top.dict_hint, cv::Point(12, 56), cv::FONT_HERSHEY_SIMPLEX,
        0.55, cv::Scalar(40, 200, 255), 1, cv::LINE_AA);
  }
}

}  // namespace

BoardTypeIdentifyResult BoardTypeIdentifier::identify(
    const cv::Mat &bgr, const BoardTypeIdentifyOptions &options) const {
  BoardTypeIdentifyResult result;
  if (bgr.empty()) {
    result.message = "empty image";
    return result;
  }
  cv::Mat work = bgr;
  if (bgr.channels() == 1) {
    cv::cvtColor(bgr, work, cv::COLOR_GRAY2BGR);
  } else if (bgr.channels() == 4) {
    cv::cvtColor(bgr, work, cv::COLOR_BGRA2BGR);
  } else {
    work = bgr.clone();
  }

  const ImageFrame frame = mat_as_image_frame(work, "bgr8");
  const auto dicts =
      default_dicts(options.dictionary_hints, options.max_dictionary_scan);
  DetectedMarkers markers;
  const bool has_markers = scan_best_markers(work, dicts, &markers);

  if (has_markers) {
    try_charuco(frame, work, markers, options.candidate_types, &result.ranked);
    try_aruco_family(frame, work, markers, options.candidate_types, &result.ranked);
  }
  try_chessboard(frame, work, options.candidate_types, &result.ranked);
  const int marker_n = has_markers ? static_cast<int>(markers.ids.size()) : 0;
  try_circles(frame, work, options.candidate_types, marker_n, &result.ranked);
  try_trihedral(
      frame, has_markers ? markers : DetectedMarkers{}, options.candidate_types,
      &result.ranked);

  apply_marker_evidence(marker_n, &result.ranked);

  std::sort(result.ranked.begin(), result.ranked.end(), hypothesis_better);

  // 相对归一仅用于展示，不改变排序：保留峰值差距
  if (!result.ranked.empty()) {
    const double peak = std::max(1e-6, result.ranked.front().score);
    for (auto &h : result.ranked) {
      h.score = clamp01(h.score / peak);
    }
  }

  if (options.draw_overlay && !result.ranked.empty()) {
    result.overlay_bgr = work.clone();
    draw_top_overlay(&result.overlay_bgr, result.ranked.front(),
                     has_markers ? &markers : nullptr);
  } else if (options.draw_overlay) {
    result.overlay_bgr = work.clone();
    cv::putText(
        result.overlay_bgr, "no board type matched", cv::Point(12, 28),
        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 80, 255), 2, cv::LINE_AA);
  }

  if (result.ranked.empty()) {
    result.message = "未识别到常见标定板类型";
  } else {
    result.message = "top=" + result.ranked.front().type_id;
  }
  return result;
}

}  // namespace core
}  // namespace hs_calib
