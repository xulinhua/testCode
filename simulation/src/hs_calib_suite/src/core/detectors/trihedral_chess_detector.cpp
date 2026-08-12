#include "hs_calib_suite/core/detectors/trihedral_chess_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "hs_calib_suite/core/util/cv_bridge_local.hpp"
#include "hs_calib_suite/core/util/cv_image_ops.hpp"

// =============================================================================
// 三面棋盘格：把「三张共点拼接、仅角点」的板从一张图里拆开
//
// 直觉：每一面 = 一张带白边的 chessboard，别人只找角点也能标定。
// OpenCV 一次 findChessboardCorners 通常只返回一个网格，因此必须：
//   找到一面 → 掩膜抹掉 → 再找 → 再抹 → 最多三面；
//   再把每个网格用 PnP 残差贴到模型 XY / XZ / YZ。
//
// 入口：TrihedralChessDetector::detect_merged
// 通用图像工具见 cv_image_ops（to_gray / enhance_clahe / guess_K）
// =============================================================================

namespace hs_calib {
namespace core {

namespace {

/// 一次检测到的矩形棋盘网格（完整面或子网格 / 部分面）
struct FaceHit {
  std::vector<cv::Point2f> corners;  ///< 行优先，pattern.width × pattern.height
  cv::Size pattern;
};

/// 已贴到已知模型面上的检测结果
struct MappedHit {
  int face_id = -1;
  std::vector<cv::Point2f> corners;
  std::vector<int> local_indices;
};

/// \brief 邻域最近距离中位数，估计格点间距（像素）
float median_neighbor_spacing(const std::vector<cv::Point2f> &pts) {
  if (pts.size() < 2) {
    return 8.f;
  }
  std::vector<float> d;
  d.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    float best = std::numeric_limits<float>::infinity();
    for (size_t j = 0; j < pts.size(); ++j) {
      if (i == j) {
        continue;
      }
      const float n = static_cast<float>(cv::norm(pts[i] - pts[j]));
      if (n > 1.f && n < best) {
        best = n;
      }
    }
    if (std::isfinite(best)) {
      d.push_back(best);
    }
  }
  if (d.empty()) {
    return 8.f;
  }
  std::nth_element(d.begin(), d.begin() + static_cast<int>(d.size() / 2), d.end());
  return std::max(4.f, d[d.size() / 2]);
}

/// \brief 两角点集是否属同一面（重合比例，非 bbox IoU）
bool corners_same_face(
    const std::vector<cv::Point2f> &a, const std::vector<cv::Point2f> &b) {
  if (a.empty() || b.empty()) {
    return false;
  }
  // 三面 bbox 常相交，不能靠 IoU；用角点重合比例
  const auto &small = a.size() <= b.size() ? a : b;
  const auto &large = a.size() <= b.size() ? b : a;
  const float cell = 0.45f * median_neighbor_spacing(large);
  const float cell2 = cell * cell;
  int hits = 0;
  for (const auto &p : small) {
    for (const auto &q : large) {
      const cv::Point2f d = p - q;
      if (d.x * d.x + d.y * d.y <= cell2) {
        ++hits;
        break;
      }
    }
  }
  const int need = std::max(4, static_cast<int>(0.4 * static_cast<double>(small.size())));
  return hits >= need;
}

/// \brief 前向声明：凸包掩膜抹掉已检出区域
void paint_out_board(cv::Mat *gray, const std::vector<cv::Point2f> &corners, int pad_px);

/// \brief 沿棋盘轴轻度外扩 + 凸包，掩膜抹掉一面；外扩要克制，否则会抹到邻面
void paint_out_grid_face(
    cv::Mat *gray, const FaceHit &hit, int sx, int sy) {
  if (gray == nullptr || hit.corners.size() < 4) {
    return;
  }
  const int cols = hit.pattern.width;
  const int rows = hit.pattern.height;
  if (cols < 2 || rows < 2 ||
      static_cast<int>(hit.corners.size()) != cols * rows) {
    return;
  }

  // 先按凸包抹已检出区域（不会沿轴射向折缝邻面）
  const float cell = median_neighbor_spacing(hit.corners);
  paint_out_board(gray, hit.corners, std::max(2, cvRound(0.75f * cell)));

  cv::Point2f du(0.f, 0.f);
  cv::Point2f dv(0.f, 0.f);
  int nu = 0;
  int nv = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c + 1 < cols; ++c) {
      du += hit.corners[static_cast<size_t>(r * cols + c + 1)] -
            hit.corners[static_cast<size_t>(r * cols + c)];
      ++nu;
    }
  }
  for (int r = 0; r + 1 < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      dv += hit.corners[static_cast<size_t>((r + 1) * cols + c)] -
            hit.corners[static_cast<size_t>(r * cols + c)];
      ++nv;
    }
  }
  if (nu == 0 || nv == 0) {
    return;
  }
  du *= (1.f / static_cast<float>(nu));
  dv *= (1.f / static_cast<float>(nv));
  // 仅轻度外扩清同面残留角点；大幅外扩会抹掉正交邻面（根因）
  const float extra_u =
      std::min(0.9f, 0.25f + 0.18f * static_cast<float>(std::max(0, sx - cols)));
  const float extra_v =
      std::min(0.9f, 0.25f + 0.18f * static_cast<float>(std::max(0, sy - rows)));
  const cv::Point2f o = hit.corners.front();
  const std::vector<cv::Point> poly = {
      cv::Point(cvRound((o - extra_u * du - extra_v * dv).x),
                cvRound((o - extra_u * du - extra_v * dv).y)),
      cv::Point(cvRound((o + (static_cast<float>(cols - 1) + extra_u) * du - extra_v * dv).x),
                cvRound((o + (static_cast<float>(cols - 1) + extra_u) * du - extra_v * dv).y)),
      cv::Point(cvRound((o + (static_cast<float>(cols - 1) + extra_u) * du +
                         (static_cast<float>(rows - 1) + extra_v) * dv)
                            .x),
                cvRound((o + (static_cast<float>(cols - 1) + extra_u) * du +
                         (static_cast<float>(rows - 1) + extra_v) * dv)
                            .y)),
      cv::Point(cvRound((o - extra_u * du + (static_cast<float>(rows - 1) + extra_v) * dv).x),
                cvRound((o - extra_u * du + (static_cast<float>(rows - 1) + extra_v) * dv).y)),
  };
  cv::Mat mask = cv::Mat::zeros(gray->size(), CV_8UC1);
  cv::fillConvexPoly(mask, poly, 255);
  cv::Mat blurred;
  cv::blur(*gray, blurred, cv::Size(21, 21));
  blurred.copyTo(*gray, mask);
}

/// \brief 凸包收缩后模糊填充，抹掉已用角点区域
void paint_out_board(cv::Mat *gray, const std::vector<cv::Point2f> &corners, int pad_px) {
  if (gray == nullptr || corners.empty()) {
    return;
  }
  std::vector<cv::Point> poly;
  poly.reserve(corners.size());
  for (const auto &p : corners) {
    poly.emplace_back(cvRound(p.x), cvRound(p.y));
  }
  std::vector<cv::Point> hull;
  cv::convexHull(poly, hull);
  // 向内收缩，避免把相邻面公共棱边一起抹掉
  cv::Point2f c(0, 0);
  for (const auto &p : hull) {
    c.x += static_cast<float>(p.x);
    c.y += static_cast<float>(p.y);
  }
  c.x /= static_cast<float>(std::max<size_t>(1, hull.size()));
  c.y /= static_cast<float>(std::max<size_t>(1, hull.size()));
  std::vector<cv::Point> shrunk;
  shrunk.reserve(hull.size());
  for (const auto &p : hull) {
    const float vx = static_cast<float>(p.x) - c.x;
    const float vy = static_cast<float>(p.y) - c.y;
    shrunk.emplace_back(
        cvRound(c.x + vx * 0.88f), cvRound(c.y + vy * 0.88f));
  }
  cv::Mat mask = cv::Mat::zeros(gray->size(), CV_8UC1);
  cv::fillConvexPoly(mask, shrunk, 255);
  if (pad_px > 0) {
    const int k = pad_px * 2 + 1;
    cv::dilate(
        mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k)));
  }
  // 用局部均值，减少掩膜边缘假轮廓
  cv::Mat blurred;
  cv::blur(*gray, blurred, cv::Size(31, 31));
  blurred.copyTo(*gray, mask);
}

/// \brief 检出网格尺寸是否能装进一面（含转置）
bool accept_pattern_size(cv::Size found, int sx, int sy) {
  if (found.width < 3 || found.height < 3) {
    return false;
  }
  // 检测到的连续网格须能放入某一面（或转置后放入）
  return (found.width <= sx && found.height <= sy) ||
         (found.width <= sy && found.height <= sx);
}

/// \brief 三面允许局部网格：≥3×3 即可贴面；只拒极端细长假条
bool pattern_plausible(cv::Size found, int sx, int sy) {
  if (!accept_pattern_size(found, sx, sy)) {
    return false;
  }
  const int lo = std::min(found.width, found.height);
  const int hi = std::max(found.width, found.height);
  if (lo < 3 || found.area() < 9) {
    return false;
  }
  // 细长条（如 8×3 假网格）易误检；正常斜视局部仍接近方形
  if (static_cast<double>(hi) / static_cast<double>(lo) > 2.6) {
    return false;
  }
  return true;
}

/// \brief 在灰度图上找指定尺寸完整棋盘角点（经典 + 可选 SB）
bool try_find_full(
    const cv::Mat &gray, cv::Size pattern, std::vector<cv::Point2f> *corners_out,
    bool allow_sb) {
  corners_out->clear();
  if (pattern.width < 3 || pattern.height < 3) {
    return false;
  }
  if (gray.cols < pattern.width * 5 || gray.rows < pattern.height * 5) {
    return false;
  }
  // 三面同框时 FAST_CHECK 常直接判无，去掉
  const int flags_fast =
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  const int flags_full =
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE |
      cv::CALIB_CB_FILTER_QUADS;
  std::vector<cv::Point2f> corners;
  bool ok = cv::findChessboardCorners(gray, pattern, corners, flags_fast);
  if (!ok) {
    corners.clear();
    ok = cv::findChessboardCorners(gray, pattern, corners, flags_full);
  }
  if (ok && corners.size() == static_cast<size_t>(pattern.area())) {
    *corners_out = std::move(corners);
    return true;
  }
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 5)
  // SB 很慢：仅在明确需要时开，且限制图幅
  if (allow_sb && gray.total() <= static_cast<size_t>(960 * 720)) {
    corners.clear();
    if (cv::findChessboardCornersSB(
            gray, pattern, corners, cv::CALIB_CB_NORMALIZE_IMAGE) &&
        corners.size() == static_cast<size_t>(pattern.area())) {
      *corners_out = std::move(corners);
      return true;
    }
  }
#else
  (void)allow_sb;
#endif
  return false;
}

/// \brief 局部检出后尝试扩 ROI 再找更大网格（失败则保留局部）
///
/// 扩 ROI 时可能扫到邻面；候选必须与种子角点同面。
bool try_upgrade_to_full(
    const cv::Mat &gray, int sx, int sy, FaceHit *hit,
    TrihedralChessDetectSpeed speed) {
  if (hit == nullptr || hit->corners.size() < 9) {
    return false;
  }
  if ((hit->pattern.width == sx && hit->pattern.height == sy) ||
      (hit->pattern.width == sy && hit->pattern.height == sx)) {
    return true;
  }
  const FaceHit seed = *hit;
  const float cell = median_neighbor_spacing(hit->corners);
  cv::Rect box = cv::boundingRect(hit->corners);
  const int miss_u = std::max(0, std::max(sx, sy) - hit->pattern.width);
  const int miss_v = std::max(0, std::max(sx, sy) - hit->pattern.height);
  const int pad_x = cvRound(cell * static_cast<float>(miss_u + 2));
  const int pad_y = cvRound(cell * static_cast<float>(miss_v + 2));
  box.x = std::max(0, box.x - pad_x);
  box.y = std::max(0, box.y - pad_y);
  box.width = std::min(gray.cols - box.x, box.width + 2 * pad_x);
  box.height = std::min(gray.rows - box.y, box.height + 2 * pad_y);
  if (box.width < sx * 4 || box.height < sy * 4) {
    return false;
  }

  cv::Mat patch = gray(box);
  const bool fast = (speed == TrihedralChessDetectSpeed::Fast);
  // Fast：单 CLAHE；Thorough：CLAHE + 原图（去掉双 CLAHE）
  std::vector<cv::Mat> views;
  views.push_back(enhance_clahe(patch, 3.0));
  if (!fast) {
    views.push_back(patch);
  }
  std::vector<cv::Size> targets;
  targets.emplace_back(sx, sy);
  if (sx != sy) {
    targets.emplace_back(sy, sx);
  }
  if (!fast) {
    targets.emplace_back(sx - 1, sy);
    targets.emplace_back(sx, sy - 1);
    targets.emplace_back(sx - 1, sy - 1);
  }

  std::vector<cv::Point2f> corners;
  FaceHit best = *hit;
  const std::vector<double> scales = fast ? std::vector<double>{1.0}
                                          : std::vector<double>{1.0, 1.25};
  for (const cv::Mat &v : views) {
    for (double scale : scales) {
      cv::Mat work = v;
      if (std::abs(scale - 1.0) > 1e-6) {
        cv::resize(v, work, cv::Size(), scale, scale, cv::INTER_LINEAR);
      }
      for (const cv::Size &tg : targets) {
        if (tg.width < hit->pattern.width && tg.height < hit->pattern.height) {
          continue;
        }
        if (tg.width < 3 || tg.height < 3 || !pattern_plausible(tg, sx, sy)) {
          continue;
        }
        const bool want_full =
            (tg.width == sx && tg.height == sy) ||
            (tg.width == sy && tg.height == sx);
        // 非满格用经典角点即可；满格才开一次 SB
        if (!try_find_full(work, tg, &corners, want_full && !fast)) {
          continue;
        }
        for (auto &p : corners) {
          p.x = static_cast<float>(p.x / scale + box.x);
          p.y = static_cast<float>(p.y / scale + box.y);
        }
        if (!corners_same_face(seed.corners, corners)) {
          continue;
        }
        if (static_cast<int>(corners.size()) >
            static_cast<int>(best.corners.size())) {
          best.corners = corners;
          best.pattern = tg;
        }
        if (want_full) {
          *hit = std::move(best);
          return true;
        }
      }
    }
  }
  if (best.corners.size() > hit->corners.size()) {
    *hit = std::move(best);
    return true;
  }
  return false;
}

/// \brief 估计网格轴向量 du（列向）、dv（行向）
bool grid_axes(
    const FaceHit &hit, cv::Point2f *du_out, cv::Point2f *dv_out) {
  const int cols = hit.pattern.width;
  const int rows = hit.pattern.height;
  if (cols < 2 || rows < 2 ||
      static_cast<int>(hit.corners.size()) != cols * rows) {
    return false;
  }
  cv::Point2f du(0.f, 0.f);
  cv::Point2f dv(0.f, 0.f);
  int nu = 0;
  int nv = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c + 1 < cols; ++c) {
      du += hit.corners[static_cast<size_t>(r * cols + c + 1)] -
            hit.corners[static_cast<size_t>(r * cols + c)];
      ++nu;
    }
  }
  for (int r = 0; r + 1 < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      dv += hit.corners[static_cast<size_t>((r + 1) * cols + c)] -
            hit.corners[static_cast<size_t>(r * cols + c)];
      ++nv;
    }
  }
  if (nu == 0 || nv == 0) {
    return false;
  }
  *du_out = du * (1.f / static_cast<float>(nu));
  *dv_out = dv * (1.f / static_cast<float>(nv));
  return true;
}

/// \brief 预测点经亚像素后是否像棋盘角（局部反差够）
bool accept_grown_corner(
    const cv::Mat &gray, cv::Point2f pred, cv::Point2f refined, float cell) {
  const float max_shift = 0.55f * cell;
  if (cv::norm(refined - pred) > max_shift) {
    return false;
  }
  const int x = cvRound(refined.x);
  const int y = cvRound(refined.y);
  const int r = std::max(2, cvRound(cell * 0.28f));
  if (x - r < 0 || y - r < 0 || x + r >= gray.cols || y + r >= gray.rows) {
    return false;
  }
  // 四对角邻域均值差：鞍点两侧黑白对比
  auto mean_roi = [&](int x0, int y0) {
    cv::Scalar m = cv::mean(gray(cv::Rect(x0, y0, r, r)));
    return static_cast<float>(m[0]);
  };
  const float a = mean_roi(x - r, y - r);
  const float b = mean_roi(x, y - r);
  const float c = mean_roi(x - r, y);
  const float d = mean_roi(x, y);
  const float contrast = std::fabs((a + d) - (b + c));
  return contrast > 8.f;
}

/// \brief 沿四边向外长一圈角点（预测 + cornerSubPix），尽量补到 sx×sy
/// \param work 已增强的灰度（避免每次全图 CLAHE）
bool try_grow_to_target(
    const cv::Mat &work, int sx, int sy, FaceHit *hit) {
  if (hit == nullptr || work.empty() || hit->corners.size() < 9) {
    return false;
  }
  auto is_full = [&](const FaceHit &h) {
    return (h.pattern.width == sx && h.pattern.height == sy) ||
           (h.pattern.width == sy && h.pattern.height == sx);
  };
  if (is_full(*hit)) {
    return true;
  }

  bool any = false;
  const int max_iters = std::min(sx + sy, 8);
  for (int iter = 0; iter < max_iters; ++iter) {
    if (is_full(*hit)) {
      break;
    }
    cv::Point2f du;
    cv::Point2f dv;
    if (!grid_axes(*hit, &du, &dv)) {
      break;
    }
    const float cell = std::max(4.f, 0.5f * (static_cast<float>(cv::norm(du)) +
                                             static_cast<float>(cv::norm(dv))));
    const int cols = hit->pattern.width;
    const int rows = hit->pattern.height;
    const std::vector<cv::Point2f> cur = hit->corners;

    struct Cand {
      int new_cols;
      int new_rows;
      std::vector<cv::Point2f> pts;
      int score;
    };
    std::vector<Cand> cands;

    auto try_grow = [&](int side) {
      // side: 0=右 +du, 1=左 -du, 2=下 +dv, 3=上 -dv
      Cand cand;
      cand.score = 0;
      if (side == 0 || side == 1) {
        if (cols >= sx && cols >= sy) {
          return;
        }
        cand.new_cols = cols + 1;
        cand.new_rows = rows;
        if (cand.new_cols > sx && cand.new_cols > sy) {
          return;
        }
        if (!((cand.new_cols <= sx && cand.new_rows <= sy) ||
              (cand.new_cols <= sy && cand.new_rows <= sx))) {
          return;
        }
        cand.pts.resize(static_cast<size_t>(cand.new_cols * cand.new_rows));
        const cv::Point2f step = (side == 0) ? du : (du * -1.f);
        std::vector<cv::Point2f> grown(static_cast<size_t>(rows));
        std::vector<char> ok(static_cast<size_t>(rows), 0);
        int ok_n = 0;
        for (int r = 0; r < rows; ++r) {
          for (int c = 0; c < cols; ++c) {
            const int dst_c = (side == 0) ? c : c + 1;
            cand.pts[static_cast<size_t>(r * cand.new_cols + dst_c)] =
                cur[static_cast<size_t>(r * cols + c)];
          }
          const int edge_c = (side == 0) ? cols - 1 : 0;
          cv::Point2f pred =
              cur[static_cast<size_t>(r * cols + edge_c)] + step;
          std::vector<cv::Point2f> one{pred};
          cv::cornerSubPix(
              work, one, cv::Size(5, 5), cv::Size(-1, -1),
              cv::TermCriteria(
                  cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.03));
          if (accept_grown_corner(work, pred, one[0], cell)) {
            grown[static_cast<size_t>(r)] = one[0];
            ok[static_cast<size_t>(r)] = 1;
            ++ok_n;
          } else {
            grown[static_cast<size_t>(r)] = pred;
          }
        }
        if (ok_n < std::max(2, (rows * 3 + 3) / 4)) {
          return;
        }
        for (int r = 0; r < rows; ++r) {
          if (ok[static_cast<size_t>(r)]) {
            continue;
          }
          int a = r - 1;
          int b = r + 1;
          while (a >= 0 && !ok[static_cast<size_t>(a)]) {
            --a;
          }
          while (b < rows && !ok[static_cast<size_t>(b)]) {
            ++b;
          }
          if (a >= 0 && b < rows) {
            const float t = static_cast<float>(r - a) / static_cast<float>(b - a);
            grown[static_cast<size_t>(r)] =
                grown[static_cast<size_t>(a)] * (1.f - t) +
                grown[static_cast<size_t>(b)] * t;
          } else if (a >= 0) {
            grown[static_cast<size_t>(r)] = grown[static_cast<size_t>(a)] + step;
          } else if (b < rows) {
            grown[static_cast<size_t>(r)] = grown[static_cast<size_t>(b)] - step;
          }
        }
        const int dst_edge = (side == 0) ? cols : 0;
        for (int r = 0; r < rows; ++r) {
          cand.pts[static_cast<size_t>(r * cand.new_cols + dst_edge)] =
              grown[static_cast<size_t>(r)];
        }
        cand.score = ok_n;
      } else {
        if (rows >= sx && rows >= sy) {
          return;
        }
        cand.new_cols = cols;
        cand.new_rows = rows + 1;
        if (cand.new_rows > sx && cand.new_rows > sy) {
          return;
        }
        if (!((cand.new_cols <= sx && cand.new_rows <= sy) ||
              (cand.new_cols <= sy && cand.new_rows <= sx))) {
          return;
        }
        cand.pts.resize(static_cast<size_t>(cand.new_cols * cand.new_rows));
        const cv::Point2f step = (side == 2) ? dv : (dv * -1.f);
        std::vector<cv::Point2f> grown(static_cast<size_t>(cols));
        std::vector<char> ok(static_cast<size_t>(cols), 0);
        int ok_n = 0;
        for (int c = 0; c < cols; ++c) {
          for (int r = 0; r < rows; ++r) {
            const int dst_r = (side == 2) ? r : r + 1;
            cand.pts[static_cast<size_t>(dst_r * cand.new_cols + c)] =
                cur[static_cast<size_t>(r * cols + c)];
          }
          const int edge_r = (side == 2) ? rows - 1 : 0;
          cv::Point2f pred =
              cur[static_cast<size_t>(edge_r * cols + c)] + step;
          std::vector<cv::Point2f> one{pred};
          cv::cornerSubPix(
              work, one, cv::Size(5, 5), cv::Size(-1, -1),
              cv::TermCriteria(
                  cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.03));
          if (accept_grown_corner(work, pred, one[0], cell)) {
            grown[static_cast<size_t>(c)] = one[0];
            ok[static_cast<size_t>(c)] = 1;
            ++ok_n;
          } else {
            grown[static_cast<size_t>(c)] = pred;
          }
        }
        if (ok_n < std::max(2, (cols * 3 + 3) / 4)) {
          return;
        }
        for (int c = 0; c < cols; ++c) {
          if (ok[static_cast<size_t>(c)]) {
            continue;
          }
          int a = c - 1;
          int b = c + 1;
          while (a >= 0 && !ok[static_cast<size_t>(a)]) {
            --a;
          }
          while (b < cols && !ok[static_cast<size_t>(b)]) {
            ++b;
          }
          if (a >= 0 && b < cols) {
            const float t = static_cast<float>(c - a) / static_cast<float>(b - a);
            grown[static_cast<size_t>(c)] =
                grown[static_cast<size_t>(a)] * (1.f - t) +
                grown[static_cast<size_t>(b)] * t;
          } else if (a >= 0) {
            grown[static_cast<size_t>(c)] = grown[static_cast<size_t>(a)] + step;
          } else if (b < cols) {
            grown[static_cast<size_t>(c)] = grown[static_cast<size_t>(b)] - step;
          }
        }
        const int dst_edge = (side == 2) ? rows : 0;
        for (int c = 0; c < cols; ++c) {
          cand.pts[static_cast<size_t>(dst_edge * cand.new_cols + c)] =
              grown[static_cast<size_t>(c)];
        }
        cand.score = ok_n;
      }
      if (cand.score > 0) {
        cands.push_back(std::move(cand));
      }
    };

    for (int side = 0; side < 4; ++side) {
      try_grow(side);
    }
    if (cands.empty()) {
      break;
    }
    std::sort(cands.begin(), cands.end(), [](const Cand &a, const Cand &b) {
      const int aa = a.new_cols * a.new_rows;
      const int bb = b.new_cols * b.new_rows;
      if (aa != bb) {
        return aa > bb;
      }
      return a.score > b.score;
    });
    hit->corners = std::move(cands.front().pts);
    hit->pattern = cv::Size(cands.front().new_cols, cands.front().new_rows);
    any = true;
  }
  return any || is_full(*hit);
}

/// \brief 补全流水线：局部 ROI 上生长 + 有限次重检（避免整图 CLAHE）
void complete_face_hit(
    const cv::Mat &gray, int sx, int sy, FaceHit *hit,
    TrihedralChessDetectSpeed speed) {
  if (hit == nullptr || hit->corners.size() < 9) {
    return;
  }
  auto is_full = [&](const FaceHit &h) {
    return (h.pattern.width == sx && h.pattern.height == sy) ||
           (h.pattern.width == sy && h.pattern.height == sx);
  };
  if (is_full(*hit)) {
    refine_corners_subpix(gray, &hit->corners, 7);
    return;
  }

  const float cell = median_neighbor_spacing(hit->corners);
  cv::Rect box = cv::boundingRect(hit->corners);
  const int pad = cvRound(cell * static_cast<float>(
      2 + std::max(0, std::max(sx, sy) - std::min(hit->pattern.width,
                                                  hit->pattern.height))));
  box.x = std::max(0, box.x - pad);
  box.y = std::max(0, box.y - pad);
  box.width = std::min(gray.cols - box.x, box.width + 2 * pad);
  box.height = std::min(gray.rows - box.y, box.height + 2 * pad);
  if (box.width < 16 || box.height < 16) {
    return;
  }

  cv::Mat local = gray(box).clone();
  // 把角点挪到 local 坐标
  FaceHit local_hit = *hit;
  for (auto &p : local_hit.corners) {
    p.x -= static_cast<float>(box.x);
    p.y -= static_cast<float>(box.y);
  }
  const cv::Mat work = enhance_clahe(local, 3.0);
  try_grow_to_target(work, sx, sy, &local_hit);
  if (!is_full(local_hit)) {
    try_upgrade_to_full(local, sx, sy, &local_hit, speed);
  }
  if (!is_full(local_hit) && speed == TrihedralChessDetectSpeed::Thorough) {
    try_grow_to_target(work, sx, sy, &local_hit);
  }
  for (auto &p : local_hit.corners) {
    p.x += static_cast<float>(box.x);
    p.y += static_cast<float>(box.y);
  }
  *hit = std::move(local_hit);
  refine_corners_subpix(gray, &hit->corners, 7);
}

/// \brief 按面积从大到小试子网格（含 3×3）；三面允许局部
bool try_find_subgrid(
    const cv::Mat &gray, int sx, int sy, FaceHit *hit_out, int max_try,
    bool allow_sb) {
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
  push(sx, sy);
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
    // 子网格扫描默认关 SB；仅最大两档开一次
    const bool sb =
        allow_sb && tried <= 2 && (c.w * c.h >= (sx - 1) * (sy - 1));
    if (try_find_full(gray, cv::Size(c.w, c.h), &corners, sb)) {
      hit_out->corners = std::move(corners);
      hit_out->pattern = cv::Size(c.w, c.h);
      return true;
    }
  }
  return false;
}

/// \brief SB LARGER：接受任意能装进一面的局部（≥3×3，非细长）
bool try_find_partial_sb(
    const cv::Mat &gray, int sx, int sy, FaceHit *hit_out) {
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
  hit_out->corners = std::move(corners);
  hit_out->pattern = found;
  return true;
#else
  (void)gray;
  (void)sx;
  (void)sy;
  (void)hit_out;
  return false;
#endif
}

/// \brief 优先完整面，失败则接受局部子网格（三面标定核心：网格不必拍全）
bool try_find_any(
    const cv::Mat &gray, int sx, int sy, FaceHit *hit_out,
    TrihedralChessDetectSpeed speed) {
  hit_out->corners.clear();
  std::vector<cv::Point2f> corners;
  const bool fast = (speed == TrihedralChessDetectSpeed::Fast);
  const bool large = gray.total() > static_cast<size_t>(720 * 540);
  // 满格：大图 Fast 不开 SB；小图 / Thorough 可开
  const bool sb_full = !fast && !large;
  if (try_find_full(gray, cv::Size(sx, sy), &corners, sb_full)) {
    hit_out->corners = std::move(corners);
    hit_out->pattern = cv::Size(sx, sy);
    return true;
  }
  if (sx != sy && try_find_full(gray, cv::Size(sy, sx), &corners, sb_full)) {
    hit_out->corners = std::move(corners);
    hit_out->pattern = cv::Size(sy, sx);
    return true;
  }
  // 大图少试几档子网格，避免掩膜全图阶段拖死
  const int sub_try = large ? (fast ? 5 : 7) : (fast ? 8 : 12);
  if (try_find_subgrid(gray, sx, sy, hit_out, sub_try, sb_full)) {
    return true;
  }
  // SB LARGER 只在中小 ROI 上
  const size_t lim = fast ? static_cast<size_t>(640 * 480)
                          : static_cast<size_t>(800 * 600);
  if (gray.total() <= lim) {
    return try_find_partial_sb(gray, sx, sy, hit_out);
  }
  return false;
}

/// \brief 全图 / 四象限重叠 / 中心块搜索 ROI
std::vector<cv::Rect> make_search_rois(cv::Size sz, bool include_full) {
  std::vector<cv::Rect> rois;
  if (include_full) {
    rois.emplace_back(0, 0, sz.width, sz.height);  // 全图
  }
  const int tw = static_cast<int>(sz.width * 0.62);
  const int th = static_cast<int>(sz.height * 0.62);
  const int ox = static_cast<int>(sz.width * 0.38);
  const int oy = static_cast<int>(sz.height * 0.38);
  // 2×2 重叠块：每面常落在不同象限
  const int xs[2] = {0, ox};
  const int ys[2] = {0, oy};
  for (int iy = 0; iy < 2; ++iy) {
    for (int ix = 0; ix < 2; ++ix) {
      rois.emplace_back(
          xs[ix], ys[iy], std::min(tw, sz.width - xs[ix]),
          std::min(th, sz.height - ys[iy]));
    }
  }
  // 中心块
  const int cx = static_cast<int>(sz.width * 0.15);
  const int cy = static_cast<int>(sz.height * 0.15);
  rois.emplace_back(cx, cy, sz.width - 2 * cx, sz.height - 2 * cy);
  return rois;
}

/// \brief 按方格朝向+邻域聚类，把三面拆成独立 ROI（整图多棋盘会互相干扰）
std::vector<cv::Rect> find_checker_face_rois(const cv::Mat &gray) {
  std::vector<cv::Rect> rois;
  if (gray.empty() || gray.cols < 48 || gray.rows < 48) {
    return rois;
  }
  cv::Mat bw;
  cv::adaptiveThreshold(
      gray, bw, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 15, 2);
  std::vector<cv::RotatedRect> quads;
  auto collect = [&](const cv::Mat &bin) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    for (const auto &c : contours) {
      if (c.size() < 4) {
        continue;
      }
      const double area = cv::contourArea(c);
      if (area < 20.0 || area > 0.12 * gray.cols * gray.rows) {
        continue;
      }
      std::vector<cv::Point> approx;
      cv::approxPolyDP(c, approx, 0.06 * cv::arcLength(c, true), true);
      if (approx.size() != 4 || !cv::isContourConvex(approx)) {
        continue;
      }
      const cv::RotatedRect rr = cv::minAreaRect(approx);
      const float w = std::max(rr.size.width, 1.f);
      const float h = std::max(rr.size.height, 1.f);
      if (std::min(w, h) < 4.f) {
        continue;
      }
      if (std::max(w, h) / std::min(w, h) > 2.3f) {
        continue;
      }
      quads.push_back(rr);
    }
  };
  collect(bw);
  cv::Mat inv;
  cv::bitwise_not(bw, inv);
  collect(inv);
  if (quads.size() < 8) {
    return rois;
  }

  auto rect_angle = [](const cv::RotatedRect &r) {
    float a = r.angle;
    if (r.size.width < r.size.height) {
      a += 90.f;
    }
    while (a < 0.f) {
      a += 180.f;
    }
    while (a >= 180.f) {
      a -= 180.f;
    }
    return a;
  };
  auto ang_diff = [](float a, float b) {
    const float d = std::fabs(a - b);
    return std::min(d, 180.f - d);
  };

  struct Cluster {
    std::vector<int> idx;
    cv::Point2f mean{0.f, 0.f};
    float angle = 0.f;
    float size = 0.f;
  };
  std::vector<char> used(quads.size(), 0);
  std::vector<Cluster> clusters;
  for (size_t i = 0; i < quads.size(); ++i) {
    if (used[i]) {
      continue;
    }
    Cluster cl;
    cl.idx.push_back(static_cast<int>(i));
    cl.mean = quads[i].center;
    cl.angle = rect_angle(quads[i]);
    cl.size = std::max(quads[i].size.width, quads[i].size.height);
    used[i] = 1;
    bool grew = true;
    while (grew) {
      grew = false;
      for (size_t j = 0; j < quads.size(); ++j) {
        if (used[j]) {
          continue;
        }
        const float ad = ang_diff(rect_angle(quads[j]), cl.angle);
        const float sz = std::max(quads[j].size.width, quads[j].size.height);
        const float szr =
            std::max(sz, cl.size) / std::max(1.f, std::min(sz, cl.size));
        if (ad > 28.f || szr > 2.0f) {
          continue;
        }
        if (cv::norm(quads[j].center - cl.mean) > 3.4f * cl.size) {
          continue;
        }
        used[j] = 1;
        cl.idx.push_back(static_cast<int>(j));
        const float n = static_cast<float>(cl.idx.size());
        cl.mean = (cl.mean * (n - 1.f) + quads[j].center) / n;
        cl.size = (cl.size * (n - 1.f) + sz) / n;
        grew = true;
      }
    }
    if (static_cast<int>(cl.idx.size()) >= 6) {
      clusters.push_back(std::move(cl));
    }
  }
  std::sort(clusters.begin(), clusters.end(), [](const Cluster &a, const Cluster &b) {
    return a.idx.size() > b.idx.size();
  });
  if (clusters.size() > 3) {
    clusters.resize(3);
  }

  for (const auto &cl : clusters) {
    cv::Rect box;
    bool first = true;
    for (int id : cl.idx) {
      const cv::Rect r = quads[static_cast<size_t>(id)].boundingRect();
      box = first ? r : (box | r);
      first = false;
    }
    const int pad = std::max(10, static_cast<int>(cl.size * 1.35f));
    box.x = std::max(0, box.x - pad);
    box.y = std::max(0, box.y - pad);
    box.width = std::min(gray.cols - box.x, box.width + 2 * pad);
    box.height = std::min(gray.rows - box.y, box.height + 2 * pad);
    if (box.width >= 48 && box.height >= 48) {
      rois.push_back(box);
    }
  }
  return rois;
}

/// \brief 亚像素细化后并入结果；同面则保留更大网格
bool append_hit_if_new(
    std::vector<FaceHit> *hits, FaceHit hit, const cv::Mat &gray_ref,
    bool refine) {
  if (hit.corners.size() < 9) {
    return false;
  }
  if (refine) {
    cv::cornerSubPix(
        gray_ref, hit.corners, cv::Size(7, 7), cv::Size(-1, -1),
        cv::TermCriteria(
            cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.03));
  }

  for (auto &prev : *hits) {
    if (corners_same_face(prev.corners, hit.corners)) {
      if (hit.corners.size() > prev.corners.size()) {
        prev = std::move(hit);
      }
      return false;  // 同面：已合并，不算新增
    }
  }
  hits->push_back(std::move(hit));
  return true;
}

/// \brief 剥面检出最多 max_faces 个网格；**允许局部**，越大越好
std::vector<FaceHit> find_all_boards(
    const cv::Mat &gray_in, int sx, int sy, int max_faces,
    TrihedralChessDetectSpeed speed) {
  std::vector<FaceHit> hits;
  if (sx < 2 || sy < 2 || gray_in.empty()) {
    return hits;
  }

  const bool fast = (speed == TrihedralChessDetectSpeed::Fast);
  const auto t0 = std::chrono::steady_clock::now();
  // 搜索预算；补全另计但已瘦身
  const int budget_ms = fast ? 900 : 2000;
  auto timed_out = [&]() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - t0)
               .count() >= budget_ms;
  };

  cv::Mat search = gray_in;
  double inv_scale = 1.0;
  const int max_w = fast ? 960 : 1280;
  if (gray_in.cols > max_w) {
    inv_scale = static_cast<double>(gray_in.cols) / static_cast<double>(max_w);
    cv::resize(
        gray_in, search,
        cv::Size(max_w, std::max(1, cvRound(gray_in.rows / inv_scale))), 0, 0,
        cv::INTER_AREA);
  }

  std::vector<cv::Mat> bases;
  bases.push_back(enhance_clahe(search));
  if (!fast) {
    bases.push_back(search);
  }
  const std::vector<double> scales =
      fast ? std::vector<double>{1.0} : std::vector<double>{1.0, 1.25};

  const std::vector<cv::Rect> face_rois = find_checker_face_rois(search);
  // 跳过整图 ROI：大图上反复 findChessboard 极慢；象限 + 掩膜足够
  const std::vector<cv::Rect> grid_rois =
      make_search_rois(search.size(), /*include_full=*/false);

  auto commit_hit = [&](FaceHit hit) -> bool {
    if (!pattern_plausible(hit.pattern, sx, sy)) {
      return false;
    }
    const size_t before = hits.size();
    append_hit_if_new(&hits, std::move(hit), gray_in, false);
    return hits.size() > before;
  };

  auto try_on = [&](const cv::Mat &img, double scale, int x0, int y0) -> bool {
    if (static_cast<int>(hits.size()) >= max_faces || timed_out()) {
      return false;
    }
    FaceHit hit;
    if (!try_find_any(img, sx, sy, &hit, speed)) {
      return false;
    }
    for (auto &p : hit.corners) {
      p.x = static_cast<float>((p.x / scale + x0) * inv_scale);
      p.y = static_cast<float>((p.y / scale + y0) * inv_scale);
    }
    return commit_hit(std::move(hit));
  };

  auto scan_rois = [&](const std::vector<cv::Rect> &rois, bool one_hit_per_roi) {
    for (const cv::Rect &roi : rois) {
      if (timed_out() || static_cast<int>(hits.size()) >= max_faces) {
        break;
      }
      if (roi.width < 40 || roi.height < 40) {
        continue;
      }
      bool got = false;
      for (const cv::Mat &base : bases) {
        cv::Mat patch = base(roi);
        for (double scale : scales) {
          cv::Mat work;
          if (std::abs(scale - 1.0) < 1e-6) {
            work = patch;
          } else {
            cv::resize(patch, work, cv::Size(), scale, scale, cv::INTER_LINEAR);
          }
          if (try_on(work, scale, roi.x, roi.y)) {
            got = true;
            break;
          }
        }
        if (got) {
          break;
        }
      }
      (void)one_hit_per_roi;
    }
  };

  // —— A. 面聚类 ROI ——
  scan_rois(face_rois, true);
  // —— B. 全图/象限补漏 ——
  if (!timed_out() && static_cast<int>(hits.size()) < max_faces) {
    scan_rois(grid_rois, false);
  }

  // —— C. 掩膜再挖 ——
  if (!timed_out() && static_cast<int>(hits.size()) < max_faces) {
    std::vector<cv::Mat> masked_bases;
    for (const cv::Mat &b : bases) {
      masked_bases.push_back(b.clone());
    }
    auto paint_search_hit = [&](cv::Mat *img, const FaceHit &hit) {
      FaceHit local = hit;
      for (auto &p : local.corners) {
        p.x = static_cast<float>(p.x / inv_scale);
        p.y = static_cast<float>(p.y / inv_scale);
      }
      paint_out_grid_face(img, local, sx, sy);
    };
    for (const auto &h : hits) {
      for (cv::Mat &b : masked_bases) {
        paint_search_hit(&b, h);
      }
    }
    const int max_mask_try = fast ? max_faces * 3 : max_faces * 5;
    const std::vector<cv::Rect> peel_rois =
        make_search_rois(search.size(), /*include_full=*/false);
    for (int attempt = 0;
         attempt < max_mask_try && static_cast<int>(hits.size()) < max_faces &&
         !timed_out();
         ++attempt) {
      bool progress = false;
      for (cv::Mat &base : masked_bases) {
        bool got = false;
        for (const cv::Rect &roi : peel_rois) {
          if (roi.width < 40 || roi.height < 40) {
            continue;
          }
          FaceHit hit;
          if (!try_find_any(base(roi), sx, sy, &hit, speed)) {
            continue;
          }
          for (auto &p : hit.corners) {
            p.x = static_cast<float>((p.x + roi.x) * inv_scale);
            p.y = static_cast<float>((p.y + roi.y) * inv_scale);
          }
          FaceHit painted = hit;
          const bool is_new = commit_hit(std::move(hit));
          paint_search_hit(&base, painted);
          for (cv::Mat &ob : masked_bases) {
            if (&ob != &base) {
              paint_search_hit(&ob, painted);
            }
          }
          progress = true;
          got = true;
          if (is_new) {
            break;
          }
        }
        if (got) {
          break;
        }
      }
      if (!progress) {
        break;
      }
    }
  }

  // —— D. Thorough 漏面扇区（Fast 跳过）——
  if (!fast && !timed_out() && static_cast<int>(hits.size()) < max_faces &&
      !hits.empty()) {
    std::vector<cv::Point2f> centers;
    for (const auto &h : hits) {
      cv::Point2f c(0, 0);
      for (const auto &p : h.corners) {
        c += p;
      }
      c *= 1.f / static_cast<float>(h.corners.size());
      centers.push_back(cv::Point2f(
          static_cast<float>(c.x / inv_scale),
          static_cast<float>(c.y / inv_scale)));
    }
    std::vector<cv::Rect> gap_rois;
    const int W = search.cols;
    const int H = search.rows;
    auto push_unique = [&](cv::Rect r) {
      if (r.width < 64 || r.height < 64) {
        return;
      }
      for (const cv::Rect &e : gap_rois) {
        const cv::Rect inter = e & r;
        if (inter.area() > 0.7 * std::min(e.area(), r.area())) {
          return;
        }
      }
      gap_rois.push_back(r);
    };
    for (const cv::Point2f &c : centers) {
      const int cx = cvRound(c.x);
      const int cy = cvRound(c.y);
      const int xs[2] = {0, std::min(W - 1, std::max(0, cx))};
      const int xe[2] = {std::max(1, cx), W};
      const int ys[2] = {0, std::min(H - 1, std::max(0, cy))};
      const int ye[2] = {std::max(1, cy), H};
      for (int iy = 0; iy < 2; ++iy) {
        for (int ix = 0; ix < 2; ++ix) {
          push_unique(
              cv::Rect(xs[ix], ys[iy], xe[ix] - xs[ix], ye[iy] - ys[iy]));
        }
      }
    }
    for (const cv::Rect &roi : gap_rois) {
      if (timed_out() || static_cast<int>(hits.size()) >= max_faces) {
        break;
      }
      cv::Mat patch = bases.front()(roi).clone();
      for (const auto &h : hits) {
        std::vector<cv::Point2f> inside;
        for (const auto &p : h.corners) {
          const float x = static_cast<float>(p.x / inv_scale - roi.x);
          const float y = static_cast<float>(p.y / inv_scale - roi.y);
          if (x >= 0 && y >= 0 && x < patch.cols && y < patch.rows) {
            inside.emplace_back(x, y);
          }
        }
        if (inside.size() >= 4) {
          paint_out_board(&patch, inside, 8);
        }
      }
      FaceHit hit;
      if (!try_find_any(patch, sx, sy, &hit, speed)) {
        continue;
      }
      for (auto &p : hit.corners) {
        p.x = static_cast<float>((p.x + roi.x) * inv_scale);
        p.y = static_cast<float>((p.y + roi.y) * inv_scale);
      }
      if (commit_hit(std::move(hit))) {
        break;
      }
    }
  }

  // 各面局部补全：搜索已超时则只做廉价生长，避免预算外再穷举
  const auto complete_speed =
      (fast || timed_out()) ? TrihedralChessDetectSpeed::Fast : speed;
  for (size_t i = 0; i < hits.size(); ++i) {
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0)
            .count() >= budget_ms + (fast ? 250 : 600)) {
      break;  // 硬顶：补全总时间也封顶
    }
    const FaceHit before = hits[i];
    complete_face_hit(gray_in, sx, sy, &hits[i], complete_speed);
    if (!corners_same_face(before.corners, hits[i].corners) ||
        hits[i].corners.size() < before.corners.size()) {
      hits[i] = before;
    }
  }

  std::sort(hits.begin(), hits.end(), [](const FaceHit &a, const FaceHit &b) {
    return a.corners.size() > b.corners.size();
  });
  if (static_cast<int>(hits.size()) > max_faces) {
    hits.resize(static_cast<size_t>(max_faces));
  }
  return hits;
}

/// \brief 网格翻转/转置重排，消解棋盘原点二义性
std::vector<cv::Point2f> reorder_grid(
    const std::vector<cv::Point2f> &in, int cols, int rows, int mode) {
  // mode：0=原样，1=水平翻转，2=垂直翻转，3=旋转 180°，4..7=先转置再 0..3
  const bool transpose = mode >= 4;
  const int flip = mode % 4;
  const int in_cols = cols;
  const int in_rows = rows;
  int out_cols = transpose ? rows : cols;
  int out_rows = transpose ? cols : rows;
  std::vector<cv::Point2f> tmp(in.size());
  for (int r = 0; r < in_rows; ++r) {
    for (int c = 0; c < in_cols; ++c) {
      const cv::Point2f &p = in[static_cast<size_t>(r * in_cols + c)];
      if (transpose) {
        tmp[static_cast<size_t>(c * out_cols + r)] = p;
      } else {
        tmp[static_cast<size_t>(r * out_cols + c)] = p;
      }
    }
  }
  std::vector<cv::Point2f> out(tmp.size());
  for (int r = 0; r < out_rows; ++r) {
    for (int c = 0; c < out_cols; ++c) {
      int rr = r;
      int cc = c;
      if (flip == 1 || flip == 3) {
        cc = out_cols - 1 - c;
      }
      if (flip == 2 || flip == 3) {
        rr = out_rows - 1 - r;
      }
      out[static_cast<size_t>(r * out_cols + c)] =
          tmp[static_cast<size_t>(rr * out_cols + cc)];
    }
  }
  return out;
}

/// \brief 枚举局部网格在完整面上的摆放（原点二义性 + 偏移）
bool best_placement_on_face(
    const FaceHit &hit,
    const TrihedralFaceModel &face,
    int sx,
    int sy,
    const cv::Mat &K,
    const cv::Mat &D,
    MappedHit *mapped_out,
    double *err_out) {
  const int dw0 = hit.pattern.width;
  const int dh0 = hit.pattern.height;
  if (static_cast<int>(hit.corners.size()) != dw0 * dh0 || dw0 < 3 || dh0 < 3) {
    return false;
  }

  double best_err = std::numeric_limits<double>::infinity();
  MappedHit best;

  auto consider = [&](const std::vector<cv::Point2f> &ordered, int cols, int rows) {
    if (cols > sx || rows > sy) {
      return;
    }
    for (int j0 = 0; j0 <= sy - rows; ++j0) {
      for (int i0 = 0; i0 <= sx - cols; ++i0) {
        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> img;
        std::vector<int> locals;
        obj.reserve(static_cast<size_t>(cols * rows));
        img.reserve(static_cast<size_t>(cols * rows));
        locals.reserve(static_cast<size_t>(cols * rows));
        for (int r = 0; r < rows; ++r) {
          for (int c = 0; c < cols; ++c) {
            const int local = (j0 + r) * sx + (i0 + c);
            if (local < 0 || local >= face.object_points.rows()) {
              continue;
            }
            locals.push_back(local);
            obj.emplace_back(
                static_cast<float>(face.object_points(local, 0)),
                static_cast<float>(face.object_points(local, 1)),
                static_cast<float>(face.object_points(local, 2)));
            img.push_back(ordered[static_cast<size_t>(r * cols + c)]);
          }
        }
        if (obj.size() < 4) {
          continue;
        }
        cv::Mat rvec, tvec;
        if (!cv::solvePnP(obj, img, K, D, rvec, tvec, false, cv::SOLVEPNP_EPNP)) {
          continue;
        }
        cv::solvePnP(obj, img, K, D, rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
        std::vector<cv::Point2f> proj;
        cv::projectPoints(obj, rvec, tvec, K, D, proj);
        double err = 0.0;
        for (size_t i = 0; i < proj.size(); ++i) {
          err += cv::norm(proj[i] - img[i]);
        }
        err /= static_cast<double>(proj.size());
        // 误差接近时优先点数更多的覆盖
        err -= 0.002 * static_cast<double>(obj.size());
        if (err < best_err) {
          best_err = err;
          best.corners = img;
          best.local_indices = std::move(locals);
        }
      }
    }
  };

  // 4 种翻转 × 可选转置 = 8 种朝向（覆盖棋盘原点二义性）
  for (int mode = 0; mode < 8; ++mode) {
    const bool transpose = mode >= 4;
    const int cols = transpose ? dh0 : dw0;
    const int rows = transpose ? dw0 : dh0;
    if (cols > sx || rows > sy) {
      continue;
    }
    auto ordered = reorder_grid(hit.corners, dw0, dh0, mode);
    consider(ordered, cols, rows);
  }

  if (!std::isfinite(best_err) || best.local_indices.size() < 4) {
    return false;
  }
  *mapped_out = std::move(best);
  if (err_out) {
    *err_out = best_err;
  }
  return true;
}

/// \brief 已贴多面的联合重投影误差
double joint_reproj_error(
    const std::vector<MappedHit> &mapped, const TrihedralTarget &target, const cv::Mat &K,
    const cv::Mat &D) {
  std::vector<cv::Point3f> obj;
  std::vector<cv::Point2f> img;
  for (const auto &m : mapped) {
    if (m.face_id < 0 || m.face_id >= static_cast<int>(target.faces().size())) {
      continue;
    }
    const auto &f = target.faces()[static_cast<size_t>(m.face_id)];
    for (size_t i = 0; i < m.local_indices.size(); ++i) {
      const int local = m.local_indices[i];
      if (local < 0 || local >= f.object_points.rows()) {
        continue;
      }
      obj.emplace_back(
          static_cast<float>(f.object_points(local, 0)),
          static_cast<float>(f.object_points(local, 1)),
          static_cast<float>(f.object_points(local, 2)));
      img.push_back(m.corners[i]);
    }
  }
  if (obj.size() < 6) {
    return std::numeric_limits<double>::infinity();
  }
  cv::Mat rvec, tvec;
  if (!cv::solvePnP(obj, img, K, D, rvec, tvec, false, cv::SOLVEPNP_EPNP)) {
    return std::numeric_limits<double>::infinity();
  }
  cv::solvePnP(obj, img, K, D, rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
  std::vector<cv::Point2f> proj;
  cv::projectPoints(obj, rvec, tvec, K, D, proj);
  double err = 0.0;
  for (size_t i = 0; i < proj.size(); ++i) {
    err += cv::norm(proj[i] - img[i]);
  }
  return err / static_cast<double>(proj.size());
}

/// \brief 多面 MappedHit 合并为单条 Correspondence
Correspondence merge_mapped(const std::vector<MappedHit> &mapped, const TrihedralTarget &target) {
  Correspondence c;
  int total = 0;
  for (const auto &m : mapped) {
    total += static_cast<int>(m.corners.size());
  }
  c.image_points.resize(total, 2);
  c.object_points.resize(total, 3);
  c.ids.resize(static_cast<size_t>(total));
  int row = 0;
  for (const auto &m : mapped) {
    const auto &f = target.faces()[static_cast<size_t>(m.face_id)];
    for (size_t i = 0; i < m.corners.size(); ++i) {
      const int local = m.local_indices[i];
      c.image_points(row, 0) = m.corners[i].x;
      c.image_points(row, 1) = m.corners[i].y;
      c.object_points.row(row) = f.object_points.row(local);
      c.ids[static_cast<size_t>(row)] = TrihedralTarget::point_id(m.face_id, local);
      ++row;
    }
  }
  return c;
}

/// \brief 枚举面置换，选联合重投影误差最小的贴面方案
std::vector<MappedHit> best_face_assignment(
    const std::vector<FaceHit> &hits,
    const TrihedralTarget &target,
    int width,
    int height,
    double *best_err_out) {
  cv::Mat K = guess_K(width, height);
  cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
  const int sx = target.squares_x();
  const int sy = target.squares_y();

  std::vector<MappedHit> best_mapped;
  double best_err = std::numeric_limits<double>::infinity();

  auto consider_perm = [&](const std::vector<int> &perm) {
    std::vector<MappedHit> mapped;
    mapped.reserve(hits.size());
    for (size_t h = 0; h < hits.size(); ++h) {
      const int face = perm[h];
      if (face < 0 || face >= static_cast<int>(target.faces().size())) {
        return;
      }
      MappedHit m;
      double err = 0.0;
      if (!best_placement_on_face(
              hits[h], target.faces()[static_cast<size_t>(face)], sx, sy, K, D, &m,
              &err)) {
        return;
      }
      m.face_id = face;
      mapped.push_back(std::move(m));
    }
    const double joint = joint_reproj_error(mapped, target, K, D);
    if (joint < best_err) {
      best_err = joint;
      best_mapped = std::move(mapped);
    }
  };

  if (hits.size() == 1) {
    for (int f = 0; f < 3; ++f) {
      consider_perm({f});
    }
  } else if (hits.size() == 2) {
    for (int a = 0; a < 3; ++a) {
      for (int b = 0; b < 3; ++b) {
        if (a != b) {
          consider_perm({a, b});
        }
      }
    }
  } else {
    std::vector<int> perm = {0, 1, 2};
    do {
      consider_perm(perm);
    } while (std::next_permutation(perm.begin(), perm.end()));
  }

  if (best_err_out) {
    *best_err_out = best_err;
  }
  return best_mapped;
}

}  // namespace

/// \brief 绑定三面棋盘几何模型
TrihedralChessDetector::TrihedralChessDetector(TrihedralTarget target)
    : target_(std::move(target)) {}

/// \brief DetectorBase：Thorough 预算下合并三面后包装为单元素列表
std::vector<Correspondence> TrihedralChessDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  Correspondence merged = detect_merged(frame, nullptr, TrihedralChessDetectSpeed::Thorough);
  if (merged.image_points.rows() < 4) {
    return {};
  }
  return {std::move(merged)};
}

/// \brief 使用构造时靶标检测
std::vector<Correspondence> TrihedralChessDetector::detect(const ImageFrame &frame) const {
  return detect(frame, target_);
}

/// \brief 剥面检出 → 贴 XY/XZ/YZ → 合并 Correspondence
/// \param faces_found 成功贴面数；\param speed Fast=预览 / Thorough=手动检测
Correspondence TrihedralChessDetector::detect_merged(
    const ImageFrame &frame, int *faces_found, TrihedralChessDetectSpeed speed) const {
  Correspondence empty;
  cv::Mat mat = image_frame_as_mat(frame);
  if (mat.empty()) {
    if (faces_found) {
      *faces_found = 0;
    }
    return empty;
  }
  // 统一走通用灰度转换（支持 BGR / BGRA / mono）
  const cv::Mat gray = to_gray(mat);

  // ① 最多挖出 3 个互不重合网格
  auto hits = find_all_boards(gray, target_.squares_x(), target_.squares_y(), 3, speed);
  if (hits.empty()) {
    if (faces_found) {
      *faces_found = 0;
    }
    return empty;
  }

  // ② 联合枚举面分配（失败则贪心）
  double best_err = 0.0;
  std::vector<MappedHit> mapped =
      best_face_assignment(hits, target_, frame.width, frame.height, &best_err);

  // 联合分配失败时：逐面贪心贴物点，仍可显示/参与多帧
  if (mapped.empty() || !std::isfinite(best_err)) {
    cv::Mat K = guess_K(frame.width, frame.height);
    cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
    bool used[3] = {false, false, false};
    mapped.clear();
    for (const auto &hit : hits) {
      MappedHit best_m;
      double best_e = std::numeric_limits<double>::infinity();
      int best_f = -1;
      for (int f = 0; f < 3; ++f) {
        if (used[f]) {
          continue;
        }
        MappedHit m;
        double e = 0.0;
        if (!best_placement_on_face(
                hit, target_.faces()[static_cast<size_t>(f)], target_.squares_x(),
                target_.squares_y(), K, D, &m, &e)) {
          continue;
        }
        if (e < best_e) {
          best_e = e;
          best_m = std::move(m);
          best_f = f;
        }
      }
      if (best_f >= 0) {
        used[best_f] = true;
        best_m.face_id = best_f;
        mapped.push_back(std::move(best_m));
      }
    }
  }

  if (faces_found) {
    *faces_found = static_cast<int>(mapped.size());
  }
  if (mapped.empty()) {
    return empty;
  }
  return merge_mapped(mapped, target_);
}

}  // namespace core
}  // namespace hs_calib
