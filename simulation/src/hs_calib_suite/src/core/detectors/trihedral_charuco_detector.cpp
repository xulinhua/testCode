#include "hs_calib_suite/core/detectors/trihedral_charuco_detector.hpp"

#include <algorithm>
#include <utility>
#include <cmath>
#include <map>
#include <numeric>
#include <random>
#include <unordered_set>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/util/cv_bridge_local.hpp"
#include "hs_calib_suite/core/util/cv_image_ops.hpp"

// =============================================================================
// 三面 ChArUco 分面：把「三张共点拼接的标定板」从一张图里拆开
//
// 直觉：三面 = 三张带白边的板，分别做 Charuco 检测即可。
// 实现难点（也是拖了很久的根因）：
//
//  A. 观测层（先有码，才谈分面）
//     - 侧脸在仿真里常呈水平镜像 → OpenCV detectMarkers 原图解不出；
//       必须翻转补检或修 Isaac 码位。码都没有，分面再漂亮也是 0 面。
//     - 斜视 / 小目标 → 需要 CLAHE + 上采样。
//
//  B. 分面层（同 ID 可印在三面 → 不能靠 ID）
//     - 白边只是视觉分隔，OpenCV 不返回「面标签」。
//     - 正确做法：用「板局部坐标 → 图像」的单应 H。同一张板上的码中心
//       应落在同一 H 上（残差 ~1px）；正交另一面残差通常 >> 几十 px。
//     - 折缝附近若阈值太松，会把两面码吃进同一 H → 跨面污染。
//
//  C. 建板层（每面当一张 CharucoBoard）
//     - interpolateCornersCharuco 要够多码；PnP 给出法向，再贴到 XY/XZ/YZ。
//
// 流程入口：TrihedralCharucoDetector::detect_merged
// =============================================================================

namespace hs_calib {
namespace core {

namespace {

/// \brief ArUco 码边长中位数（像素）
float median_marker_side(const std::vector<std::vector<cv::Point2f>> &corners) {
  std::vector<float> sides;
  sides.reserve(corners.size() * 4);
  for (const auto &c : corners) {
    if (c.size() < 4) {
      continue;
    }
    for (int i = 0; i < 4; ++i) {
      sides.push_back(static_cast<float>(cv::norm(c[static_cast<size_t>(i)] -
                                                  c[static_cast<size_t>((i + 1) % 4)])));
    }
  }
  if (sides.empty()) {
    return 24.f;
  }
  std::nth_element(sides.begin(), sides.begin() + static_cast<int>(sides.size() / 2), sides.end());
  return std::max(10.f, sides[sides.size() / 2]);
}

/// \brief 由板局部 ID 取码在板上的中心 (x,y)
bool marker_center_on_board(
    const cv::Ptr<cv::aruco::CharucoBoard> &board, int local_id, cv::Point2f *xy) {
  if (!board) {
    return false;
  }
  for (size_t i = 0; i < board->ids.size(); ++i) {
    if (board->ids[i] != local_id || board->objPoints[i].size() < 4) {
      continue;
    }
    cv::Point3f s(0.f, 0.f, 0.f);
    for (const auto &p : board->objPoints[i]) {
      s += p;
    }
    s *= 0.25f;
    *xy = cv::Point2f(s.x, s.y);
    return true;
  }
  return false;
}

/// \brief 兼容旧 Isaac 面 ID 段 0/100/200：映射到单板局部 ID
int to_board_local_id(const cv::Ptr<cv::aruco::CharucoBoard> &board, int raw_id) {
  if (!board || board->ids.empty()) {
    return raw_id;
  }
  cv::Point2f tmp;
  if (marker_center_on_board(board, raw_id, &tmp)) {
    return raw_id;
  }
  const int n = static_cast<int>(board->ids.size());
  // 常见：面偏移 100；也容忍任意 mod n（同板重复印刷）
  const int candidates[] = {raw_id % 100, raw_id % std::max(1, n), raw_id};
  for (int c : candidates) {
    if (c >= 0 && marker_center_on_board(board, c, &tmp)) {
      return c;
    }
  }
  return -1;
}

/// \brief 码四角图像坐标均值
cv::Point2f marker_image_center(const std::vector<cv::Point2f> &c) {
  cv::Point2f s(0.f, 0.f);
  for (const auto &p : c) {
    s += p;
  }
  return s * (1.f / static_cast<float>(std::max<size_t>(1, c.size())));
}

/// \brief 用单应 H 映射一点
cv::Point2f apply_H(const cv::Mat &H, const cv::Point2f &p) {
  cv::Matx33d M;
  H.convertTo(M, CV_64F);
  const double x = M(0, 0) * p.x + M(0, 1) * p.y + M(0, 2);
  const double y = M(1, 0) * p.x + M(1, 1) * p.y + M(1, 2);
  const double w = M(2, 0) * p.x + M(2, 1) * p.y + M(2, 2);
  if (std::abs(w) < 1e-12) {
    return {1e6f, 1e6f};
  }
  return {static_cast<float>(x / w), static_cast<float>(y / w)};
}

/// \brief 将棋盘角点吸到局部鞍点亚像素位置
void snap_to_chess_saddles(
    const cv::Mat &gray, const std::vector<std::vector<cv::Point2f>> &marker_corners,
    float marker_side_px, std::vector<cv::Point2f> *corners, std::vector<int> *ids) {
  if (gray.empty() || corners == nullptr || ids == nullptr || corners->empty() ||
      corners->size() != ids->size()) {
    return;
  }
  cv::Mat work = gray.clone();
  for (const auto &mc : marker_corners) {
    if (mc.size() < 4) {
      continue;
    }
    cv::Point2f c = marker_image_center(mc);
    std::vector<cv::Point> poly;
    poly.reserve(mc.size());
    for (const auto &p : mc) {
      const cv::Point2f q = c + (p - c) * 0.90f;
      poly.emplace_back(cvRound(q.x), cvRound(q.y));
    }
    cv::fillConvexPoly(work, poly, 128);
  }

  const float sq_px = marker_side_px / 0.72f;
  const float gap_px = std::max(3.f, 0.5f * (sq_px - marker_side_px));
  int win = cvRound(std::min(gap_px * 1.1f, sq_px * 0.16f));
  if (win % 2 == 0) {
    ++win;
  }
  win = std::max(3, std::min(9, win));
  cv::cornerSubPix(
      work, *corners, cv::Size(win, win), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.01));

  cv::Mat eig;
  cv::cornerMinEigenVal(work, eig, 5);
  std::vector<float> scores;
  scores.reserve(corners->size());
  for (const auto &p : *corners) {
    const int x = std::max(0, std::min(eig.cols - 1, cvRound(p.x)));
    const int y = std::max(0, std::min(eig.rows - 1, cvRound(p.y)));
    scores.push_back(eig.at<float>(y, x));
  }
  std::vector<float> sorted = scores;
  std::nth_element(
      sorted.begin(), sorted.begin() + static_cast<int>(sorted.size() / 2), sorted.end());
  const float thr = 0.15f * std::max(1e-8f, sorted[sorted.size() / 2]);

  std::vector<cv::Point2f> kc;
  std::vector<int> ki;
  kc.reserve(corners->size());
  ki.reserve(ids->size());
  for (size_t i = 0; i < corners->size(); ++i) {
    if (scores[i] < thr) {
      continue;
    }
    kc.push_back((*corners)[i]);
    ki.push_back((*ids)[i]);
  }
  // 过滤过狠则回退原插值结果
  if (kc.size() >= 4) {
    *corners = std::move(kc);
    *ids = std::move(ki);
  }
}

/// \brief 构造 ArUco 检测参数（亚像素细化等）
cv::Ptr<cv::aruco::DetectorParameters> make_params() {
  auto p = cv::aruco::DetectorParameters::create();
  p->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  p->cornerRefinementWinSize = 5;
  p->cornerRefinementMaxIterations = 50;
  p->cornerRefinementMinAccuracy = 0.01;
  p->adaptiveThreshWinSizeMin = 3;
  p->adaptiveThreshWinSizeMax = 35;
  p->adaptiveThreshWinSizeStep = 6;
  p->adaptiveThreshConstant = 7;
  p->minMarkerPerimeterRate = 0.012;
  p->maxMarkerPerimeterRate = 4.5;
  p->polygonalApproxAccuracyRate = 0.08;
  p->minCornerDistanceRate = 0.03;
  p->minDistanceToBorder = 1;
  p->markerBorderBits = 1;
  p->perspectiveRemovePixelPerCell = 8;
  p->perspectiveRemoveIgnoredMarginPerCell = 0.13;
  p->maxErroneousBitsInBorderRate = 0.45;
  p->errorCorrectionRate = 0.7;
  return p;
}

/// \brief 合并两批码并按中心距离去重
void merge_markers(
    std::vector<std::vector<cv::Point2f>> *corners, std::vector<int> *ids,
    const std::vector<std::vector<cv::Point2f>> &add_c, const std::vector<int> &add_id,
    float dedupe_px) {
  if (corners == nullptr || ids == nullptr) {
    return;
  }
  for (size_t i = 0; i < add_id.size(); ++i) {
    if (add_c[i].size() < 4) {
      continue;
    }
    const cv::Point2f c = marker_image_center(add_c[i]);
    bool dup = false;
    for (size_t j = 0; j < ids->size(); ++j) {
      if ((*ids)[j] != add_id[i]) {
        continue;
      }
      if (cv::norm(marker_image_center((*corners)[j]) - c) < dedupe_px) {
        dup = true;
        break;
      }
    }
    if (!dup) {
      corners->push_back(add_c[i]);
      ids->push_back(add_id[i]);
    }
  }
}

/// ① 码检测：尽量检出三面的 ArUco（尚未分面）
///
/// 步骤：
///   原图 → CLAHE → 1.6×（必要时 2.2×）合并去重；
///   若「同 ID 多实例」仍少（侧脸多半没解出）→ 水平翻转图再检，
///   坐标映回后用空间门控合并（距已有码太近的丢掉，防主面假码）。
void detect_markers_robust(
    const cv::Mat &bgr, const cv::Mat &gray, const cv::Ptr<cv::aruco::Dictionary> &dict,
    std::vector<std::vector<cv::Point2f>> *corners, std::vector<int> *ids, bool fast) {
  corners->clear();
  ids->clear();
  auto params = make_params();

  auto run_detect = [&](const cv::Mat &img) {
    std::vector<std::vector<cv::Point2f>> c;
    std::vector<int> i;
    cv::aruco::detectMarkers(img, dict, c, i, params);
    return std::make_pair(std::move(c), std::move(i));
  };

  auto add_scaled = [&](const cv::Mat &img_bgr, double scale, float dedupe_px) {
    cv::Mat work = img_bgr;
    cv::Mat big;
    if (std::abs(scale - 1.0) > 1e-6) {
      cv::resize(img_bgr, big, cv::Size(), scale, scale, cv::INTER_LINEAR);
      work = big;
    }
    auto [c, i] = run_detect(work);
    if (std::abs(scale - 1.0) > 1e-6) {
      for (auto &poly : c) {
        for (auto &p : poly) {
          p.x = static_cast<float>(p.x / scale);
          p.y = static_cast<float>(p.y / scale);
        }
      }
    }
    merge_markers(corners, ids, c, i, dedupe_px);
  };

  auto count_bases = [&]() {
    // 同 ID 三面：用「同 ID 多实例」判断侧脸是否已解出
    std::map<int, int> id_count;
    for (int id : *ids) {
      ++id_count[id];
    }
    int multi = 0;
    for (const auto &kv : id_count) {
      if (kv.second > 1) {
        ++multi;
      }
    }
    return multi;
  };

  // 空间门控合并：只收下与已有检测中心距离足够远的码（避免主面假码）
  auto merge_gated = [&](std::vector<std::vector<cv::Point2f>> &c, std::vector<int> &i,
                         float min_dist_px) {
    std::vector<std::vector<cv::Point2f>> keep_c;
    std::vector<int> keep_i;
    keep_c.reserve(c.size());
    keep_i.reserve(i.size());
    for (size_t k = 0; k < c.size(); ++k) {
      if (c[k].size() < 4) {
        continue;
      }
      cv::Point2f ctr(0, 0);
      for (const auto &p : c[k]) {
        ctr += p;
      }
      ctr *= 1.f / static_cast<float>(c[k].size());
      bool near = false;
      for (const auto &poly : *corners) {
        if (poly.size() < 4) {
          continue;
        }
        cv::Point2f ec(0, 0);
        for (const auto &p : poly) {
          ec += p;
        }
        ec *= 1.f / static_cast<float>(poly.size());
        if (cv::norm(ctr - ec) < min_dist_px) {
          near = true;
          break;
        }
      }
      if (near) {
        continue;
      }
      keep_c.push_back(std::move(c[k]));
      keep_i.push_back(i[k]);
    }
    merge_markers(corners, ids, keep_c, keep_i, 10.f);
  };

  add_scaled(bgr, 1.0, 6.f);

  // CLAHE 增强后再检（斜视 / 暗面）
  const cv::Mat clahe_img = enhance_clahe(gray);
  const cv::Mat clahe_bgr = to_bgr(clahe_img);
  add_scaled(clahe_bgr, 1.0, 6.f);
  add_scaled(clahe_bgr, 1.6, 8.f);
  if (!fast && corners->size() < 48) {
    add_scaled(clahe_bgr, 2.2, 10.f);
  }

  // 侧脸镜像时原图解不出；同 ID 多实例不足则翻转补检（空间门控防主面假码）
  if (count_bases() < 8 && static_cast<int>(corners->size()) < 90) {
    const int W = clahe_bgr.cols;
    auto unflip_h = [&](std::vector<std::vector<cv::Point2f>> &c) {
      for (auto &poly : c) {
        for (auto &p : poly) {
          p.x = static_cast<float>(W - 1) - p.x;
        }
        if (poly.size() >= 4) {
          std::swap(poly[0], poly[1]);
          std::swap(poly[2], poly[3]);
        }
      }
    };
    {
      cv::Mat flipped;
      cv::flip(clahe_bgr, flipped, 1);
      auto [c, i] = run_detect(flipped);
      unflip_h(c);
      merge_gated(c, i, 14.f);
    }
    if (count_bases() < 8 && static_cast<int>(corners->size()) < 90) {
      cv::Mat flipped_big;
      cv::Mat flipped;
      cv::flip(clahe_bgr, flipped, 1);
      cv::resize(flipped, flipped_big, cv::Size(), 1.6, 1.6, cv::INTER_LINEAR);
      auto [c2, i2] = run_detect(flipped_big);
      for (auto &poly : c2) {
        for (auto &p : poly) {
          p.x = static_cast<float>(W - 1) - static_cast<float>(p.x / 1.6);
          p.y = static_cast<float>(p.y / 1.6);
        }
        if (poly.size() >= 4) {
          std::swap(poly[0], poly[1]);
          std::swap(poly[2], poly[3]);
        }
      }
      merge_gated(c2, i2, 16.f);
    }
  }
}

/// 单个 ArUco 观测：既有图像位置，也有「若它属于标准板」时的板面坐标
struct MarkerObs {
  int index = -1;   ///< 在 detectMarkers 输出数组中的下标
  int id = -1;      ///< 映射到 local_board_ 的局部 ID（三面可重复同一 id）
  int raw_id = -1;  ///< 检测器原始 ID
  cv::Point2f img_c;  ///< 图像中心
  cv::Point2f obj_c;  ///< 板平面上该码中心 (x,y)，z=0
  float side_px = 20.f;
  std::vector<cv::Point2f> corners;
};

/// \brief 码中心相对板→图单应 H 的平均残差
double mean_h_residual(
    const cv::Mat &H, const std::vector<MarkerObs> &pool, const std::vector<int> &idx) {
  if (H.empty() || idx.empty()) {
    return 1e9;
  }
  double sum = 0.0;
  for (int ci : idx) {
    const auto &m = pool[static_cast<size_t>(ci)];
    sum += cv::norm(apply_H(H, m.obj_c) - m.img_c);
  }
  return sum / static_cast<double>(idx.size());
}

/// \brief 种子码集合的图像直径
float seed_set_diameter(const std::vector<MarkerObs> &pool, const std::vector<int> &seed_ci) {
  float diam = 0.f;
  for (size_t i = 0; i < seed_ci.size(); ++i) {
    for (size_t j = i + 1; j < seed_ci.size(); ++j) {
      diam = std::max(
          diam, static_cast<float>(cv::norm(
                    pool[static_cast<size_t>(seed_ci[i])].img_c -
                    pool[static_cast<size_t>(seed_ci[j])].img_c)));
    }
  }
  return diam;
}

/// ③a 在候选码里抽出「一张共面标定板」
///
/// 原理（为何等价于识别一张板）：
///   标准板上每个码 id 有固定板坐标 obj_c。若一组码真的来自同一张板，
///   存在单应 H：img ≈ H · obj。正交另一面的码套用同一 H 会残差很大。
///
/// 算法：
///   1) 随机取空间紧凑的 4 个不同 id 作种子，估 H；
///   2) 用 H 预测其余 id 的图像位置，收残差 < ransac_px 的为内点；
///   3) RANSAC 精炼 + 中心残差再卡紧（防折缝跨面）。
///
/// 注意：三面同 id 时 by_id[id] 可有多个实例，预测时取距 pred 最近的那个。
std::vector<int> fit_board_in_indices(
    const cv::Ptr<cv::aruco::CharucoBoard> &board, const std::vector<MarkerObs> &pool,
    const std::vector<int> &subset, double ransac_px, int min_inliers) {
  if (static_cast<int>(subset.size()) < min_inliers) {
    return {};
  }
  std::map<int, std::vector<int>> by_id;
  for (int ci : subset) {
    by_id[pool[static_cast<size_t>(ci)].id].push_back(ci);
  }
  if (static_cast<int>(by_id.size()) < 4) {
    return {};
  }

  std::vector<int> unique_ids;
  unique_ids.reserve(by_id.size());
  for (const auto &kv : by_id) {
    unique_ids.push_back(kv.first);
  }

  float med_side = 20.f;
  {
    std::vector<float> sides;
    sides.reserve(subset.size());
    for (int ci : subset) {
      sides.push_back(pool[static_cast<size_t>(ci)].side_px);
    }
    std::nth_element(
        sides.begin(), sides.begin() + static_cast<int>(sides.size() / 2), sides.end());
    med_side = std::max(8.f, sides[sides.size() / 2]);
  }
  const float max_seed_diam = std::max(90.f, med_side * 5.0f);

  std::mt19937 rng(0xC0FFEEu ^ static_cast<unsigned>(subset.size() * 17));
  const int iters = std::min(300, std::max(120, static_cast<int>(unique_ids.size()) * 12));
  std::vector<int> best;
  double best_score = -1.0;

  auto score_of = [](size_t n, double mean_err) {
    const double e = mean_err + 0.25;
    return static_cast<double>(n) / (e * e);
  };

  for (int it = 0; it < iters; ++it) {
    std::shuffle(unique_ids.begin(), unique_ids.end(), rng);

    const auto &opts0 = by_id[unique_ids[0]];
    std::uniform_int_distribution<int> pick0(0, static_cast<int>(opts0.size()) - 1);
    std::vector<int> seed_ci = {opts0[static_cast<size_t>(pick0(rng))]};
    const cv::Point2f anchor = pool[static_cast<size_t>(seed_ci[0])].img_c;

    std::vector<std::pair<float, int>> near_ids;
    near_ids.reserve(unique_ids.size());
    for (size_t u = 1; u < unique_ids.size(); ++u) {
      const auto &opts = by_id[unique_ids[u]];
      int best_ci = -1;
      float best_d = 1e9f;
      for (int ci : opts) {
        const float d =
            static_cast<float>(cv::norm(pool[static_cast<size_t>(ci)].img_c - anchor));
        if (d < best_d) {
          best_d = d;
          best_ci = ci;
        }
      }
      if (best_ci >= 0) {
        near_ids.emplace_back(best_d, best_ci);
      }
    }
    std::sort(near_ids.begin(), near_ids.end());
    for (const auto &nd : near_ids) {
      if (seed_ci.size() >= 4) {
        break;
      }
      if (nd.first > max_seed_diam) {
        break;
      }
      seed_ci.push_back(nd.second);
    }
    if (seed_ci.size() < 4 || seed_set_diameter(pool, seed_ci) > max_seed_diam) {
      continue;
    }

    std::vector<cv::Point2f> src;
    std::vector<cv::Point2f> dst;
    for (int ci : seed_ci) {
      src.push_back(pool[static_cast<size_t>(ci)].obj_c);
      dst.push_back(pool[static_cast<size_t>(ci)].img_c);
    }

    cv::Mat H;
    try {
      H = cv::findHomography(src, dst, 0);
    } catch (const cv::Exception &) {
      continue;
    }
    if (H.empty()) {
      continue;
    }
    // 跨面种子会导致种子本身残差偏大
    if (mean_h_residual(H, pool, seed_ci) > 1.8) {
      continue;
    }

    std::vector<int> inliers = seed_ci;
    std::unordered_set<int> taken;
    for (int ci : seed_ci) {
      taken.insert(pool[static_cast<size_t>(ci)].id);
    }

    for (const auto &kv : by_id) {
      if (taken.count(kv.first)) {
        continue;
      }
      cv::Point2f obj;
      if (!marker_center_on_board(board, kv.first, &obj)) {
        continue;
      }
      const cv::Point2f pred = apply_H(H, obj);
      int best_ci = -1;
      float best_d = static_cast<float>(ransac_px);
      for (int ci : kv.second) {
        const float d =
            static_cast<float>(cv::norm(pool[static_cast<size_t>(ci)].img_c - pred));
        if (d < best_d) {
          best_d = d;
          best_ci = ci;
        }
      }
      if (best_ci >= 0) {
        inliers.push_back(best_ci);
        taken.insert(kv.first);
      }
    }
    if (static_cast<int>(inliers.size()) < min_inliers) {
      continue;
    }

    src.clear();
    dst.clear();
    for (int ci : inliers) {
      src.push_back(pool[static_cast<size_t>(ci)].obj_c);
      dst.push_back(pool[static_cast<size_t>(ci)].img_c);
    }
    cv::Mat mask;
    cv::Mat H2;
    try {
      H2 = cv::findHomography(src, dst, cv::RANSAC, ransac_px, mask);
    } catch (const cv::Exception &) {
      continue;
    }
    if (H2.empty() || mask.empty()) {
      continue;
    }
    std::vector<int> refined;
    refined.reserve(inliers.size());
    for (size_t i = 0; i < inliers.size(); ++i) {
      if (!mask.at<uchar>(static_cast<int>(i))) {
        continue;
      }
      const auto &m = pool[static_cast<size_t>(inliers[i])];
      const double e = cv::norm(apply_H(H2, m.obj_c) - m.img_c);
      if (e > 3.0) {
        continue;
      }
      refined.push_back(inliers[i]);
    }
    if (static_cast<int>(refined.size()) < min_inliers) {
      continue;
    }
    const double mean_err = mean_h_residual(H2, pool, refined);
    if (mean_err > 1.25) {
      continue;
    }
    const double sc = score_of(refined.size(), mean_err);
    if (sc > best_score) {
      best_score = sc;
      best = std::move(refined);
    }
  }
  return best;
}

struct FaceHyp {
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  std::vector<int> marker_pool_index;  ///< 回写 markers->face_ids 用
  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;
  cv::Vec3d normal{0, 0, 1};  ///< 相机系下面法向（PnP），用于贴 XY/XZ/YZ
  bool ok = false;
};

/// \brief PnP 旋转矩阵第三列 = 板法向（相机系）
cv::Vec3d plane_normal_from_pnp(const cv::Mat &rvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  return cv::Vec3d(R.at<double>(0, 2), R.at<double>(1, 2), R.at<double>(2, 2));
}

/// \brief 把各 hyp 法向贴到模型 XY/XZ/YZ（对齐+正交；与码 ID 无关）
std::vector<int> assign_faces_by_normals(const std::vector<cv::Vec3d> &normals) {
  const int n = static_cast<int>(normals.size());
  std::vector<int> assign(static_cast<size_t>(n), 0);
  if (n <= 0) {
    return assign;
  }
  if (n == 1) {
    assign[0] = 0;
    return assign;
  }

  const cv::Vec3d expected[3] = {
      cv::Vec3d(0, 0, 1), cv::Vec3d(0, 1, 0), cv::Vec3d(1, 0, 0)};

  auto normed = [](cv::Vec3d v) {
    const double nrm = cv::norm(v);
    return (nrm < 1e-12) ? cv::Vec3d(0, 0, 1) : (v * (1.0 / nrm));
  };

  double best_score = -1e100;
  std::vector<int> best_assign = assign;
  std::vector<int> faces(3);
  std::iota(faces.begin(), faces.end(), 0);
  do {
    std::vector<int> trial(static_cast<size_t>(n), -1);
    cv::Matx33d A(0, 0, 0, 0, 0, 0, 0, 0, 0);
    double ortho = 0.0;
    bool ok = true;
    for (int i = 0; i < n; ++i) {
      trial[static_cast<size_t>(i)] = faces[static_cast<size_t>(i)];
      const cv::Vec3d ni = normed(normals[static_cast<size_t>(i)]);
      const cv::Vec3d ei = expected[faces[static_cast<size_t>(i)]];
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          A(r, c) += ni[r] * ei[c];
        }
      }
    }
    for (int i = 0; i < n; ++i) {
      for (int j = i + 1; j < n; ++j) {
        const double d = std::abs(
            normed(normals[static_cast<size_t>(i)])
                .dot(normed(normals[static_cast<size_t>(j)])));
        ortho += 1.0 - d;
        if (d > 0.55) {
          ok = false;
        }
      }
    }
    if (!ok) {
      ortho *= 0.25;
    }
    cv::Mat Aa(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        Aa.at<double>(r, c) = A(r, c);
      }
    }
    cv::Mat w, u, vt;
    cv::SVD::compute(Aa, w, u, vt, cv::SVD::FULL_UV);
    cv::Mat R = vt.t() * u.t();
    if (cv::determinant(R) < 0) {
      vt.row(2) *= -1;
      R = vt.t() * u.t();
    }
    double align = 0.0;
    for (int i = 0; i < n; ++i) {
      const cv::Vec3d ni = normed(normals[static_cast<size_t>(i)]);
      const cv::Vec3d ei = expected[faces[static_cast<size_t>(i)]];
      cv::Mat eim = (cv::Mat_<double>(3, 1) << ei[0], ei[1], ei[2]);
      cv::Mat rim = R * eim;
      const cv::Vec3d re(rim.at<double>(0), rim.at<double>(1), rim.at<double>(2));
      align += std::abs(ni.dot(normed(re)));
    }
    const double score = align * 2.0 + ortho;
    if (score > best_score) {
      best_score = score;
      best_assign = trial;
    }
  } while (std::next_permutation(faces.begin(), faces.end()));

  return best_assign;
}

}  // namespace

TrihedralCharucoDetector::TrihedralCharucoDetector(
    TrihedralTarget target, double marker_length_m, const std::string &dictionary)
    : target_(std::move(target)),
      marker_length_m_(marker_length_m),
      dict_(make_aruco_dictionary(dictionary)) {
  const int cells = std::max(4, target_.squares_x() + 1);
  const float sq = static_cast<float>(target_.square_length_m());
  float mk = static_cast<float>(marker_length_m_);
  if (mk <= 0.f || mk >= sq) {
    mk = sq * 0.72f;
  }
  local_board_ = cv::aruco::CharucoBoard::create(cells, cells, sq, mk, dict_);
}

std::vector<Correspondence> TrihedralCharucoDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  Correspondence merged = detect_merged(frame, nullptr);
  if (merged.image_points.rows() < 4) {
    return {};
  }
  return {std::move(merged)};
}

std::vector<Correspondence> TrihedralCharucoDetector::detect(const ImageFrame &frame) const {
  return detect(frame, target_);
}

Correspondence TrihedralCharucoDetector::detect_merged(
    const ImageFrame &frame, int *faces_found, DetectedMarkers *markers, bool fast) const {
  // -------------------------------------------------------------------------
  // detect_merged：三面分割主流程
  //
  // 输入：一帧图。输出：合并的 Charuco 角点对应 + faces_found + 每码 face_id。
  // 模型：local_board_ = 单张 CharucoBoard（与「每一面印刷内容」一致）。
  // 世界：TrihedralTarget 三张正交面，把局部角点变到各面坐标系后输出。
  // -------------------------------------------------------------------------
  Correspondence empty;
  if (faces_found) {
    *faces_found = 0;
  }
  if (markers) {
    markers->corners.clear();
    markers->ids.clear();
    markers->face_ids.clear();
  }
  if (!local_board_ || !dict_) {
    return empty;
  }
  cv::Mat mat = image_frame_as_mat(frame);
  if (mat.empty()) {
    return empty;
  }
  const cv::Mat gray = to_gray(mat);
  // Charuco / PnP 暂用标称内参；标定收敛前足够做分面，不必等真实 K
  const cv::Mat K = guess_K(mat.size());
  const cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);

  // ----- ① 检出尽可能多的 ArUco（三面混在一个列表里）-----
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  detect_markers_robust(mat, gray, dict_, &marker_corners, &marker_ids, fast);
  if (markers) {
    markers->corners = marker_corners;
    markers->ids = marker_ids;
    markers->face_ids.assign(marker_ids.size(), -1);
  }
  if (marker_ids.empty()) {
    return empty;
  }

  // ----- ② 建成观测池：图像中心 + 板局部中心（分面用这对点估 H）-----
  std::vector<MarkerObs> pool;
  pool.reserve(marker_ids.size());
  for (size_t i = 0; i < marker_ids.size(); ++i) {
    if (marker_corners[i].size() < 4) {
      continue;
    }
    const int raw = marker_ids[i];
    const int lid = to_board_local_id(local_board_, raw);
    if (lid < 0) {
      continue;
    }
    cv::Point2f obj;
    if (!marker_center_on_board(local_board_, lid, &obj)) {
      continue;
    }
    MarkerObs m;
    m.index = static_cast<int>(i);
    m.id = lid;
    m.raw_id = raw;
    m.corners = marker_corners[i];
    m.img_c = marker_image_center(marker_corners[i]);
    m.obj_c = obj;
    float side = 0.f;
    for (int k = 0; k < 4; ++k) {
      side += static_cast<float>(
          cv::norm(marker_corners[i][static_cast<size_t>(k)] -
                   marker_corners[i][static_cast<size_t>((k + 1) % 4)]));
    }
    m.side_px = std::max(8.f, side * 0.25f);
    pool.push_back(std::move(m));
  }
  if (pool.size() < 3) {
    return empty;
  }

  std::vector<char> used(pool.size(), 0);
  std::vector<FaceHyp> hyps;

  // ----- ③b 把一簇共面码建成「一面」：Charuco 角点 + PnP 法向 -----
  auto try_build_hyp = [&](const std::vector<int> &cluster_idx) -> bool {
    if (cluster_idx.size() < 3) {
      return false;
    }
    FaceHyp hyp;
    for (int ci : cluster_idx) {
      hyp.marker_corners.push_back(pool[static_cast<size_t>(ci)].corners);
      hyp.marker_ids.push_back(pool[static_cast<size_t>(ci)].id);
      hyp.marker_pool_index.push_back(pool[static_cast<size_t>(ci)].index);
    }

    const int min_m = hyp.marker_ids.size() >= 6 ? 2 : 1;
    cv::aruco::interpolateCornersCharuco(
        hyp.marker_corners, hyp.marker_ids, mat, local_board_, hyp.charuco_corners,
        hyp.charuco_ids, K, D, min_m);
    if (hyp.charuco_ids.size() < 4 ||
        hyp.charuco_corners.size() != hyp.charuco_ids.size()) {
      return false;
    }
    const auto before_c = hyp.charuco_corners;
    const auto before_i = hyp.charuco_ids;
    snap_to_chess_saddles(
        gray, hyp.marker_corners, median_marker_side(hyp.marker_corners),
        &hyp.charuco_corners, &hyp.charuco_ids);
    if (hyp.charuco_ids.size() < 4) {
      hyp.charuco_corners = before_c;
      hyp.charuco_ids = before_i;
    }

    auto pnp_ok = [&](const std::vector<cv::Point2f> &img_pts,
                      const std::vector<cv::Point3f> &obj_pts, cv::Mat *rvec,
                      cv::Mat *tvec, double *reproj_out) -> bool {
      if (obj_pts.size() < 4) {
        return false;
      }
      if (!cv::solvePnP(obj_pts, img_pts, K, D, *rvec, *tvec, false, cv::SOLVEPNP_ITERATIVE) &&
          !cv::solvePnP(obj_pts, img_pts, K, D, *rvec, *tvec, false, cv::SOLVEPNP_EPNP)) {
        return false;
      }
      std::vector<cv::Point2f> proj;
      cv::projectPoints(obj_pts, *rvec, *tvec, K, D, proj);
      double reproj = 0.0;
      for (size_t i = 0; i < proj.size(); ++i) {
        reproj += cv::norm(proj[i] - img_pts[i]);
      }
      reproj /= static_cast<double>(proj.size());
      *reproj_out = reproj;
      return reproj <= 8.0;
    };

    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;
    for (size_t i = 0; i < hyp.charuco_ids.size(); ++i) {
      const int lid = hyp.charuco_ids[i];
      if (lid < 0 || lid >= static_cast<int>(local_board_->chessboardCorners.size())) {
        continue;
      }
      obj.push_back(local_board_->chessboardCorners[static_cast<size_t>(lid)]);
      img.push_back(hyp.charuco_corners[i]);
    }
    cv::Mat rvec;
    cv::Mat tvec;
    double reproj = 1e9;
    if (!pnp_ok(img, obj, &rvec, &tvec, &reproj)) {
      // snap 可能过狠：回退未 snap 角点再试
      hyp.charuco_corners = before_c;
      hyp.charuco_ids = before_i;
      obj.clear();
      img.clear();
      for (size_t i = 0; i < hyp.charuco_ids.size(); ++i) {
        const int lid = hyp.charuco_ids[i];
        if (lid < 0 || lid >= static_cast<int>(local_board_->chessboardCorners.size())) {
          continue;
        }
        obj.push_back(local_board_->chessboardCorners[static_cast<size_t>(lid)]);
        img.push_back(hyp.charuco_corners[i]);
      }
      if (!pnp_ok(img, obj, &rvec, &tvec, &reproj)) {
        return false;
      }
    }
    hyp.normal = plane_normal_from_pnp(rvec);
    const double nrm = cv::norm(hyp.normal);
    if (nrm > 1e-12) {
      hyp.normal *= 1.0 / nrm;
    }
    if (hyp.normal[2] > 0) {
      hyp.normal *= -1.0;
    }
    hyp.ok = true;

    // 新 hyp 若与已有面法向近平行，且落在对方 H 内 → 视为同面碎块，并入而非新开面
    int merge_into = -1;
    for (size_t hi = 0; hi < hyps.size(); ++hi) {
      const double d = std::abs(hyps[hi].normal.dot(hyp.normal));
      if (d < 0.88) {
        continue;
      }
      std::vector<cv::Point2f> src;
      std::vector<cv::Point2f> dst;
      for (size_t k = 0; k < hyps[hi].marker_ids.size(); ++k) {
        cv::Point2f obj;
        if (!marker_center_on_board(local_board_, hyps[hi].marker_ids[k], &obj)) {
          continue;
        }
        src.push_back(obj);
        dst.push_back(marker_image_center(hyps[hi].marker_corners[k]));
      }
      if (src.size() < 4) {
        continue;
      }
      cv::Mat Hm;
      try {
        Hm = cv::findHomography(src, dst, cv::RANSAC, 3.0);
      } catch (const cv::Exception &) {
        continue;
      }
      if (Hm.empty()) {
        continue;
      }
      int ok = 0;
      int total = 0;
      for (size_t k = 0; k < hyp.marker_ids.size(); ++k) {
        cv::Point2f obj;
        if (!marker_center_on_board(local_board_, hyp.marker_ids[k], &obj)) {
          continue;
        }
        ++total;
        if (cv::norm(apply_H(Hm, obj) - marker_image_center(hyp.marker_corners[k])) <= 4.0) {
          ++ok;
        }
      }
      if (total > 0 && ok * 4 >= total * 3) {
        merge_into = static_cast<int>(hi);
        break;
      }
    }

    auto append_markers = [&](FaceHyp &dst, const FaceHyp &src) {
      std::unordered_set<int> have(dst.marker_ids.begin(), dst.marker_ids.end());
      // 同 ID 已在 dst：保留更靠近 dst 已有点云中心的实例
      cv::Point2f center(0, 0);
      int nc = 0;
      for (const auto &poly : dst.marker_corners) {
        center += marker_image_center(poly);
        ++nc;
      }
      if (nc > 0) {
        center *= 1.f / static_cast<float>(nc);
      }
      for (size_t k = 0; k < src.marker_ids.size(); ++k) {
        if (have.count(src.marker_ids[k])) {
          continue;
        }
        have.insert(src.marker_ids[k]);
        dst.marker_ids.push_back(src.marker_ids[k]);
        dst.marker_corners.push_back(src.marker_corners[k]);
        dst.marker_pool_index.push_back(src.marker_pool_index[k]);
      }
      std::unordered_set<int> have_c(dst.charuco_ids.begin(), dst.charuco_ids.end());
      for (size_t k = 0; k < src.charuco_ids.size(); ++k) {
        if (have_c.count(src.charuco_ids[k])) {
          continue;
        }
        have_c.insert(src.charuco_ids[k]);
        dst.charuco_ids.push_back(src.charuco_ids[k]);
        dst.charuco_corners.push_back(src.charuco_corners[k]);
      }
      (void)center;
    };

    for (int ci : cluster_idx) {
      used[static_cast<size_t>(ci)] = 1;
    }
    if (merge_into >= 0) {
      append_markers(hyps[static_cast<size_t>(merge_into)], hyp);
      return true;
    }
    hyps.push_back(std::move(hyp));
    return true;
  };

  // 用已建面的 H 吸收「同面残留码」（id 尚未在该面出现、且落在 H 上）
  auto absorb_with_h = [&](size_t hi, std::vector<int> *pending) {
    if (hi >= hyps.size() || pending == nullptr) {
      return;
    }
    FaceHyp &hyp = hyps[hi];
    std::vector<cv::Point2f> src;
    std::vector<cv::Point2f> dst;
    std::unordered_set<int> have(hyp.marker_ids.begin(), hyp.marker_ids.end());
    for (size_t k = 0; k < hyp.marker_ids.size(); ++k) {
      cv::Point2f obj;
      if (!marker_center_on_board(local_board_, hyp.marker_ids[k], &obj)) {
        continue;
      }
      src.push_back(obj);
      dst.push_back(marker_image_center(hyp.marker_corners[k]));
    }
    if (src.size() < 4) {
      return;
    }
    cv::Mat H;
    try {
      H = cv::findHomography(src, dst, cv::RANSAC, 3.0);
    } catch (const cv::Exception &) {
      return;
    }
    if (H.empty()) {
      return;
    }
    std::vector<int> next;
    next.reserve(pending->size());
    for (int ci : *pending) {
      if (used[static_cast<size_t>(ci)]) {
        continue;
      }
      const auto &m = pool[static_cast<size_t>(ci)];
      if (have.count(m.id)) {
        next.push_back(ci);
        continue;
      }
      const double e = cv::norm(apply_H(H, m.obj_c) - m.img_c);
      if (e <= 5.5) {
        hyp.marker_ids.push_back(m.id);
        hyp.marker_corners.push_back(m.corners);
        hyp.marker_pool_index.push_back(m.index);
        have.insert(m.id);
        used[static_cast<size_t>(ci)] = 1;
      } else {
        next.push_back(ci);
      }
    }
    pending->swap(next);
  };

  // ----- ③ 逐面几何剥离（目标：最多 3 张「共面板」）-----
  // pending = 尚未归属任何面的码下标。
  // 每轮：从 pending 里 fit 出一簇共面码 → try_build_hyp。
  //   · 若这簇其实落在已有面 H 上 → 只 absorb，不开新面（防一板多色）；
  //   · 若建成新面但法向与已有面不正交 → 当作碎块并回最平行面；
  //   · 否则接受为新面，再用 H 吸收同面残留。
  {
    std::vector<int> pending(pool.size());
    std::iota(pending.begin(), pending.end(), 0);

    int guard = 0;
    int fail_streak = 0;
    while (hyps.size() < 3 && pending.size() >= 4 && guard++ < 10) {
      auto inliers = fit_board_in_indices(local_board_, pool, pending, 3.0, 4);
      if (inliers.size() < 4) {
        inliers = fit_board_in_indices(local_board_, pool, pending, 4.5, 4);
      }
      if (inliers.size() < 4 && !hyps.empty()) {
        inliers = fit_board_in_indices(local_board_, pool, pending, 5.5, 3);
      }
      if (inliers.size() < 4) {
        break;
      }

      // 与已有面共面 → 只吸收，不开新面（避免一板多色）
      bool coplanar_existing = false;
      for (size_t hi = 0; hi < hyps.size(); ++hi) {
        std::vector<cv::Point2f> src;
        std::vector<cv::Point2f> dst;
        for (size_t k = 0; k < hyps[hi].marker_ids.size(); ++k) {
          cv::Point2f obj;
          if (!marker_center_on_board(local_board_, hyps[hi].marker_ids[k], &obj)) {
            continue;
          }
          src.push_back(obj);
          dst.push_back(marker_image_center(hyps[hi].marker_corners[k]));
        }
        if (src.size() < 4) {
          continue;
        }
        cv::Mat H;
        try {
          H = cv::findHomography(src, dst, cv::RANSAC, 3.0);
        } catch (const cv::Exception &) {
          continue;
        }
        if (H.empty()) {
          continue;
        }
        int ok = 0;
        for (int ci : inliers) {
          const auto &m = pool[static_cast<size_t>(ci)];
          if (cv::norm(apply_H(H, m.obj_c) - m.img_c) <= 6.0) {
            ++ok;
          }
        }
        if (ok * 2 >= static_cast<int>(inliers.size())) {
          absorb_with_h(hi, &pending);
          coplanar_existing = true;
          break;
        }
      }
      if (coplanar_existing) {
        fail_streak = 0;
        std::vector<int> next;
        for (int ci : pending) {
          if (!used[static_cast<size_t>(ci)]) {
            next.push_back(ci);
          }
        }
        pending.swap(next);
        continue;
      }

      const size_t before = hyps.size();
      if (!try_build_hyp(inliers)) {
        ++fail_streak;
        const int drop_ci = inliers[0];
        std::vector<int> next;
        for (int ci : pending) {
          if (ci != drop_ci) {
            next.push_back(ci);
          }
        }
        pending.swap(next);
        if (fail_streak >= 4) {
          break;
        }
        continue;
      }
      // 新建面后检查法向正交；不正交则撤销并入最平行面
      if (hyps.size() > before && before > 0) {
        bool ortho_ok = true;
        const cv::Vec3d &nn = hyps.back().normal;
        for (size_t hi = 0; hi < before; ++hi) {
          const double d = std::abs(nn.dot(hyps[hi].normal));
          if (d > 0.55) {
            ortho_ok = false;
            break;
          }
        }
        if (!ortho_ok) {
          FaceHyp doomed = std::move(hyps.back());
          hyps.pop_back();
          int best_hi = 0;
          double best_d = -1.0;
          for (size_t hi = 0; hi < hyps.size(); ++hi) {
            const double d = std::abs(doomed.normal.dot(hyps[hi].normal));
            if (d > best_d) {
              best_d = d;
              best_hi = static_cast<int>(hi);
            }
          }
          std::unordered_set<int> have(
              hyps[static_cast<size_t>(best_hi)].marker_ids.begin(),
              hyps[static_cast<size_t>(best_hi)].marker_ids.end());
          for (size_t k = 0; k < doomed.marker_ids.size(); ++k) {
            if (have.count(doomed.marker_ids[k])) {
              continue;
            }
            have.insert(doomed.marker_ids[k]);
            hyps[static_cast<size_t>(best_hi)].marker_ids.push_back(doomed.marker_ids[k]);
            hyps[static_cast<size_t>(best_hi)].marker_corners.push_back(
                doomed.marker_corners[k]);
            hyps[static_cast<size_t>(best_hi)].marker_pool_index.push_back(
                doomed.marker_pool_index[k]);
          }
          absorb_with_h(static_cast<size_t>(best_hi), &pending);
          fail_streak = 0;
          std::vector<int> next2;
          for (int ci : pending) {
            if (!used[static_cast<size_t>(ci)]) {
              next2.push_back(ci);
            }
          }
          pending.swap(next2);
          continue;
        }
      }
      fail_streak = 0;
      std::unordered_set<int> taken(inliers.begin(), inliers.end());
      std::vector<int> next;
      for (int ci : pending) {
        if (!taken.count(ci) && !used[static_cast<size_t>(ci)]) {
          next.push_back(ci);
        }
      }
      pending.swap(next);
      if (hyps.size() > before) {
        absorb_with_h(hyps.size() - 1, &pending);
      } else {
        for (size_t hi = 0; hi < hyps.size(); ++hi) {
          absorb_with_h(hi, &pending);
        }
      }
    }
    for (int pass = 0; pass < 2; ++pass) {
      for (size_t hi = 0; hi < hyps.size(); ++hi) {
        absorb_with_h(hi, &pending);
      }
    }
  }

  if (hyps.empty()) {
    return empty;
  }

  // 吸收后重算 Charuco 角点（并入的新码要参与插值）
  for (auto &hyp : hyps) {
    if (hyp.marker_ids.size() < 3) {
      continue;
    }
    std::vector<cv::Point2f> cc;
    std::vector<int> ci;
    const int min_m = hyp.marker_ids.size() >= 6 ? 2 : 1;
    cv::aruco::interpolateCornersCharuco(
        hyp.marker_corners, hyp.marker_ids, mat, local_board_, cc, ci, K, D, min_m);
    if (ci.size() >= 4 && ci.size() == cc.size()) {
      hyp.charuco_corners = std::move(cc);
      hyp.charuco_ids = std::move(ci);
    }
  }

  // 后处理：大面 H 回收小碎块；近平行 hyp 再拼一次
  {
    std::vector<char> drop(hyps.size(), 0);
    for (size_t i = 0; i < hyps.size(); ++i) {
      if (drop[i] || hyps[i].marker_ids.size() < 4) {
        continue;
      }
      std::vector<cv::Point2f> src;
      std::vector<cv::Point2f> dst;
      for (size_t k = 0; k < hyps[i].marker_ids.size(); ++k) {
        cv::Point2f obj;
        if (!marker_center_on_board(local_board_, hyps[i].marker_ids[k], &obj)) {
          continue;
        }
        src.push_back(obj);
        dst.push_back(marker_image_center(hyps[i].marker_corners[k]));
      }
      if (src.size() < 4) {
        continue;
      }
      cv::Mat H;
      try {
        H = cv::findHomography(src, dst, cv::RANSAC, 3.0);
      } catch (const cv::Exception &) {
        continue;
      }
      if (H.empty()) {
        continue;
      }
      for (size_t j = 0; j < hyps.size(); ++j) {
        if (i == j || drop[j]) {
          continue;
        }
        if (hyps[j].marker_ids.size() >= hyps[i].marker_ids.size()) {
          continue;
        }
        int ok = 0;
        int total = 0;
        for (size_t k = 0; k < hyps[j].marker_ids.size(); ++k) {
          cv::Point2f obj;
          if (!marker_center_on_board(local_board_, hyps[j].marker_ids[k], &obj)) {
            continue;
          }
          ++total;
          const double e = cv::norm(
              apply_H(H, obj) - marker_image_center(hyps[j].marker_corners[k]));
          if (e <= 6.0) {
            ++ok;
          }
        }
        if (total > 0 && ok * 2 >= total) {
          // 碎块完全落在大面单应下 → 并入
          std::unordered_set<int> have(hyps[i].marker_ids.begin(), hyps[i].marker_ids.end());
          for (size_t k = 0; k < hyps[j].marker_ids.size(); ++k) {
            if (have.count(hyps[j].marker_ids[k])) {
              continue;
            }
            have.insert(hyps[j].marker_ids[k]);
            hyps[i].marker_ids.push_back(hyps[j].marker_ids[k]);
            hyps[i].marker_corners.push_back(hyps[j].marker_corners[k]);
            hyps[i].marker_pool_index.push_back(hyps[j].marker_pool_index[k]);
          }
          drop[j] = 1;
        }
      }
    }
    std::vector<FaceHyp> kept;
    for (size_t i = 0; i < hyps.size(); ++i) {
      if (!drop[i]) {
        kept.push_back(std::move(hyps[i]));
      }
    }
    hyps.swap(kept);
    for (auto &hyp : hyps) {
      if (hyp.marker_ids.size() < 3) {
        continue;
      }
      std::vector<cv::Point2f> cc;
      std::vector<int> ci;
      const int min_m = hyp.marker_ids.size() >= 6 ? 2 : 1;
      cv::aruco::interpolateCornersCharuco(
          hyp.marker_corners, hyp.marker_ids, mat, local_board_, cc, ci, K, D, min_m);
      if (ci.size() >= 4 && ci.size() == cc.size()) {
        hyp.charuco_corners = std::move(cc);
        hyp.charuco_ids = std::move(ci);
      }
    }
  }

  // 合并法向几乎平行的假设（同面被切成多块时拼回）
  {
    std::vector<char> drop(hyps.size(), 0);
    for (size_t i = 0; i < hyps.size(); ++i) {
      if (drop[i]) {
        continue;
      }
      for (size_t j = i + 1; j < hyps.size(); ++j) {
        if (drop[j]) {
          continue;
        }
        const double d = std::abs(
            hyps[i].normal.dot(hyps[j].normal) /
            (cv::norm(hyps[i].normal) * cv::norm(hyps[j].normal) + 1e-12));
        if (d <= 0.72) {
          continue;
        }
        // 法向近似平行 → 同面碎块，拼回
        size_t keep = i;
        size_t kill = j;
        if (hyps[j].charuco_ids.size() > hyps[i].charuco_ids.size()) {
          keep = j;
          kill = i;
        }
        auto &dst = hyps[keep];
        auto &src = hyps[kill];
        std::unordered_set<int> have_mid(dst.marker_ids.begin(), dst.marker_ids.end());
        for (size_t k = 0; k < src.marker_ids.size(); ++k) {
          if (have_mid.count(src.marker_ids[k])) {
            continue;
          }
          have_mid.insert(src.marker_ids[k]);
          dst.marker_ids.push_back(src.marker_ids[k]);
          dst.marker_corners.push_back(src.marker_corners[k]);
          dst.marker_pool_index.push_back(src.marker_pool_index[k]);
        }
        std::unordered_set<int> have_cid(dst.charuco_ids.begin(), dst.charuco_ids.end());
        for (size_t k = 0; k < src.charuco_ids.size(); ++k) {
          if (have_cid.count(src.charuco_ids[k])) {
            continue;
          }
          have_cid.insert(src.charuco_ids[k]);
          dst.charuco_ids.push_back(src.charuco_ids[k]);
          dst.charuco_corners.push_back(src.charuco_corners[k]);
        }
        drop[kill] = 1;
        if (kill == i) {
          break;
        }
      }
    }
    std::vector<FaceHyp> kept;
    for (size_t i = 0; i < hyps.size(); ++i) {
      if (!drop[i]) {
        kept.push_back(std::move(hyps[i]));
      }
    }
    hyps.swap(kept);
  }

  // ----- ④ 法向 → 模型三面 XY / XZ / YZ（与码 ID 无关）-----
  std::vector<cv::Vec3d> normals;
  for (const auto &h : hyps) {
    normals.push_back(h.normal);
  }
  const std::vector<int> face_of_hyp = assign_faces_by_normals(normals);

  // 写回每码的 face_id，供 GUI 分色
  if (markers) {
    for (size_t hi = 0; hi < hyps.size(); ++hi) {
      const int face = face_of_hyp[hi];
      for (int mi : hyps[hi].marker_pool_index) {
        if (mi >= 0 && mi < static_cast<int>(markers->face_ids.size())) {
          markers->face_ids[static_cast<size_t>(mi)] = face;
        }
      }
    }
  }

  // ----- ⑤ 各面角点变到三角靶坐标系，合并输出 -----
  std::vector<cv::Point2f> img_all;
  std::vector<cv::Point3f> obj_all;
  std::vector<int> id_all;
  int n_faces = 0;
  const int n_board_faces = static_cast<int>(target_.faces().size());

  for (size_t hi = 0; hi < hyps.size(); ++hi) {
    const int face = face_of_hyp[hi];
    if (face < 0 || face >= n_board_faces) {
      continue;
    }
    const auto &face_model = target_.faces()[static_cast<size_t>(face)];
    const auto &hyp = hyps[hi];
    int added = 0;
    for (size_t i = 0; i < hyp.charuco_ids.size(); ++i) {
      const int local = hyp.charuco_ids[i];
      if (local < 0 || local >= face_model.object_points.rows()) {
        continue;
      }
      img_all.push_back(hyp.charuco_corners[i]);
      obj_all.emplace_back(
          static_cast<float>(face_model.object_points(local, 0)),
          static_cast<float>(face_model.object_points(local, 1)),
          static_cast<float>(face_model.object_points(local, 2)));
      id_all.push_back(TrihedralTarget::point_id(face, local));
      ++added;
    }
    if (added >= 4) {
      ++n_faces;
    }
  }

  if (faces_found) {
    *faces_found = n_faces;
  }
  if (static_cast<int>(img_all.size()) < 4) {
    return empty;
  }

  Correspondence c;
  const int n = static_cast<int>(img_all.size());
  c.image_points.resize(n, 2);
  c.object_points.resize(n, 3);
  c.ids.resize(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    c.image_points(i, 0) = img_all[static_cast<size_t>(i)].x;
    c.image_points(i, 1) = img_all[static_cast<size_t>(i)].y;
    c.object_points(i, 0) = obj_all[static_cast<size_t>(i)].x;
    c.object_points(i, 1) = obj_all[static_cast<size_t>(i)].y;
    c.object_points(i, 2) = obj_all[static_cast<size_t>(i)].z;
    c.ids[static_cast<size_t>(i)] = id_all[static_cast<size_t>(i)];
  }
  return c;
}

}  // namespace core
}  // namespace hs_calib
