#include "hs_calib_suite/core/detectors/chessboard_detector.hpp"

#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "hs_calib_suite/core/util/cv_bridge_local.hpp"
#include "hs_calib_suite/core/util/cv_image_ops.hpp"

namespace hs_calib {
namespace core {
namespace {

/// \brief 经典 findChessboardCorners（可选去掉 FAST_CHECK 重试）
bool find_full_classic(
    const cv::Mat &gray, cv::Size pattern, int flags,
    std::vector<cv::Point2f> *corners) {
  corners->clear();
  if (cv::findChessboardCorners(gray, pattern, *corners, flags) && !corners->empty()) {
    return true;
  }
  corners->clear();
  int retry = flags & ~cv::CALIB_CB_FAST_CHECK;
  if (retry == 0) {
    retry = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  }
  return cv::findChessboardCorners(gray, pattern, *corners, retry) && !corners->empty();
}

/// \brief SB 满格（Thorough 用）
bool find_full_sb(
    const cv::Mat &gray, cv::Size pattern, std::vector<cv::Point2f> *corners) {
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 5)
  corners->clear();
  return cv::findChessboardCornersSB(
             gray, pattern, *corners, cv::CALIB_CB_NORMALIZE_IMAGE) &&
         !corners->empty() &&
         static_cast<int>(corners->size()) == pattern.width * pattern.height;
#else
  (void)gray;
  (void)pattern;
  (void)corners;
  return false;
#endif
}

bool pattern_plausible(cv::Size found, int sx, int sy) {
  if (found.width < 3 || found.height < 3) {
    return false;
  }
  if (found.width > sx || found.height > sy) {
    return false;
  }
  // 拒极端细长假条
  const double a =
      static_cast<double>(std::max(found.width, found.height)) /
      static_cast<double>(std::max(1, std::min(found.width, found.height)));
  return a <= 3.5;
}

/// \brief 由大到小试子网格（局部模式）
bool try_find_subgrid(
    const cv::Mat &gray, int sx, int sy, int max_try, bool allow_sb,
    cv::Size *found_out, std::vector<cv::Point2f> *corners_out) {
  struct Cand {
    int w;
    int h;
  };
  std::vector<Cand> cands;
  auto push = [&](int w, int h) {
    if (w >= 3 && h >= 3 && w <= sx && h <= sy) {
      cands.push_back({w, h});
    }
    if (w != h && h >= 3 && w >= 3 && h <= sx && w <= sy) {
      cands.push_back({h, w});
    }
  };
  push(sx - 1, sy);
  push(sx, sy - 1);
  push(sx - 1, sy - 1);
  push(sx - 2, sy - 1);
  push(sx - 1, sy - 2);
  push(sx - 2, sy - 2);
  push(6, 6);
  push(5, 5);
  push(5, 4);
  push(4, 4);
  push(4, 3);
  push(3, 3);
  std::sort(cands.begin(), cands.end(), [](const Cand &a, const Cand &b) {
    if (a.w * a.h != b.w * b.h) {
      return a.w * a.h > b.w * b.h;
    }
    return a.w > b.w;
  });
  cands.erase(
      std::unique(
          cands.begin(), cands.end(),
          [](const Cand &a, const Cand &b) { return a.w == b.w && a.h == b.h; }),
      cands.end());

  const int flags =
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  int tried = 0;
  for (const Cand &c : cands) {
    if (tried >= max_try) {
      break;
    }
    if (!pattern_plausible(cv::Size(c.w, c.h), sx, sy)) {
      continue;
    }
    ++tried;
    std::vector<cv::Point2f> corners;
    const bool sb = allow_sb && tried <= 2;
    bool ok = find_full_classic(gray, cv::Size(c.w, c.h), flags, &corners);
    if (!ok && sb) {
      ok = find_full_sb(gray, cv::Size(c.w, c.h), &corners);
    }
    if (ok) {
      *found_out = cv::Size(c.w, c.h);
      *corners_out = std::move(corners);
      return true;
    }
  }
  return false;
}

/// \brief SB LARGER：接受任意可装进板面的局部（≥3×3）
bool try_find_partial_sb(
    const cv::Mat &gray, int sx, int sy, cv::Size *found_out,
    std::vector<cv::Point2f> *corners_out) {
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 5)
  if (sx < 3 || sy < 3 || gray.cols < 48 || gray.rows < 48) {
    return false;
  }
  std::vector<cv::Point2f> corners;
  cv::Mat meta;
  const int flags = cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_LARGER;
  if (!cv::findChessboardCornersSB(gray, cv::Size(3, 3), corners, flags, meta)) {
    return false;
  }
  if (meta.empty() || corners.empty()) {
    return false;
  }
  cv::Size found(meta.cols, meta.rows);
  if (static_cast<int>(corners.size()) != found.area()) {
    return false;
  }
  if (!pattern_plausible(found, sx, sy)) {
    return false;
  }
  *found_out = found;
  *corners_out = std::move(corners);
  return true;
#else
  (void)gray;
  (void)sx;
  (void)sy;
  (void)found_out;
  (void)corners_out;
  return false;
#endif
}

}  // namespace

/// \brief 仅用靶标构造，检测选项取默认值
ChessboardDetector::ChessboardDetector(ChessboardTarget target)
    : target_(std::move(target)) {}

/// \brief 靶标 + OpenCV 标志 / 亚像素窗口
ChessboardDetector::ChessboardDetector(
    ChessboardTarget target, ChessboardDetectOptions options)
    : target_(std::move(target)), options_(options) {}

/// \brief 将选项折叠为 findChessboardCorners 的 flags 位掩码
int ChessboardDetector::opencv_flags() const {
  int flags = 0;
  if (options_.adaptive_thresh) {
    flags |= cv::CALIB_CB_ADAPTIVE_THRESH;
  }
  if (options_.normalize_image) {
    flags |= cv::CALIB_CB_NORMALIZE_IMAGE;
  }
  if (options_.filter_quads) {
    flags |= cv::CALIB_CB_FILTER_QUADS;
  }
  if (options_.fast_check) {
    flags |= cv::CALIB_CB_FAST_CHECK;
  }
  return flags;
}

/// \brief 使用构造时绑定的棋盘几何检测
std::vector<Correspondence> ChessboardDetector::detect(const ImageFrame &frame) const {
  return detect(frame, target_);
}

/// \brief 平面棋盘：完整格优先；局部模式再挖子网格 / SB LARGER
std::vector<Correspondence> ChessboardDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  std::vector<Correspondence> out;
  cv::Mat mat = image_frame_as_mat(frame);
  if (mat.empty()) {
    return out;
  }

  const cv::Mat gray = to_gray(mat);
  const int sx = target_.squares_x();
  const int sy = target_.squares_y();
  const cv::Size pattern(sx, sy);
  std::vector<cv::Point2f> corners;
  cv::Size found = pattern;

  const int flags = opencv_flags();
  bool ok = find_full_classic(gray, pattern, flags, &corners);
  if (!ok && options_.allow_partial && options_.thorough) {
    ok = find_full_sb(gray, pattern, &corners);
  }

  if (!ok && options_.allow_partial) {
    const bool large = gray.total() > static_cast<size_t>(720 * 540);
    const int sub_try = large ? (options_.thorough ? 8 : 5) : (options_.thorough ? 12 : 7);
    const bool sb = options_.thorough && !large;
    if (try_find_subgrid(gray, sx, sy, sub_try, sb, &found, &corners)) {
      ok = true;
    } else {
      const size_t lim = options_.thorough ? static_cast<size_t>(800 * 600)
                                           : static_cast<size_t>(640 * 480);
      if (gray.total() <= lim &&
          try_find_partial_sb(gray, sx, sy, &found, &corners)) {
        ok = true;
      }
    }
  }

  if (!ok || corners.empty()) {
    return out;
  }

  refine_corners_subpix(gray, &corners, options_.subpix_win);

  Correspondence c;
  const int n = static_cast<int>(corners.size());
  c.image_points.resize(n, 2);
  c.ids.resize(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    c.image_points(i, 0) = corners[static_cast<size_t>(i)].x;
    c.image_points(i, 1) = corners[static_cast<size_t>(i)].y;
    c.ids[static_cast<size_t>(i)] = i;
  }
  if (found.width == sx && found.height == sy) {
    c.object_points = target_.all_object_points();
  } else {
    // 局部子网格：用同方格边长的临时板，便于调试可视化
    ChessboardTarget sub(found.width, found.height, target_.square_length_m());
    c.object_points = sub.all_object_points();
  }
  out.push_back(std::move(c));
  return out;
}

}  // namespace core
}  // namespace hs_calib
