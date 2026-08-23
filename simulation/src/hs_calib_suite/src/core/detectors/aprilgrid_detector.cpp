#include "hs_calib_suite/core/detectors/aprilgrid_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

extern "C" {
#include "apriltag.h"
#include "common/image_u8.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h10.h"
#include "tag36h11.h"
}

#include "hs_calib_suite/core/detectors/aruco_dict.hpp"
#include "hs_calib_suite/core/util/cv_bridge_local.hpp"

namespace hs_calib {
namespace core {
namespace {

/// Kalibr AprilgridOptions；亚像素位移放宽（屏显/模糊时 1.5 过严会全灭）
constexpr double kMinBorderDistancePx = 2.0;
constexpr double kMaxSubpixDisplacement2 = 9.0;  // 3 px
constexpr int kMinTagsForObs = 1;

struct FamilyHandle {
  apriltag_family_t *fam = nullptr;
  void (*destroy)(apriltag_family_t *) = nullptr;
  const char *name = "";
};

FamilyHandle make_family(const std::string &dict_name) {
  FamilyHandle h;
  if (dict_name.find("16h5") != std::string::npos) {
    h.fam = tag16h5_create();
    h.destroy = tag16h5_destroy;
    h.name = "tag16h5";
  } else if (dict_name.find("25h9") != std::string::npos) {
    h.fam = tag25h9_create();
    h.destroy = tag25h9_destroy;
    h.name = "tag25h9";
  } else if (dict_name.find("36h10") != std::string::npos) {
    h.fam = tag36h10_create();
    h.destroy = tag36h10_destroy;
    h.name = "tag36h10";
  } else {
    h.fam = tag36h11_create();
    h.destroy = tag36h11_destroy;
    h.name = "tag36h11";
  }
  return h;
}

/// AprilRobotics：角点逆时针自左下 → Kalibr pIdx {BL,BR,TR,TL}
struct DetectedTag {
  int id = -1;
  std::array<cv::Point2f, 4> corners{};
  float margin = 0.0f;
};

void merge_tag(std::map<int, DetectedTag> *by_id, DetectedTag &&t) {
  if (by_id == nullptr || t.id < 0) {
    return;
  }
  auto it = by_id->find(t.id);
  if (it == by_id->end() || t.margin > it->second.margin) {
    (*by_id)[t.id] = std::move(t);
  }
}

class AprilTagEngine {
public:
  explicit AprilTagEngine(const std::string &dict_name)
      : family_(make_family(dict_name)),
        detector_(apriltag_detector_create()) {
    apriltag_detector_add_family_bits(detector_, family_.fam, 2);
    detector_->nthreads = 2;
    detector_->refine_edges = 1;
    detector_->decode_sharpening = 0.25;
    detector_->debug = 0;
  }

  ~AprilTagEngine() {
    if (detector_ != nullptr) {
      apriltag_detector_destroy(detector_);
      detector_ = nullptr;
    }
    if (family_.fam != nullptr && family_.destroy != nullptr) {
      family_.destroy(family_.fam);
      family_.fam = nullptr;
    }
  }

  AprilTagEngine(const AprilTagEngine &) = delete;
  AprilTagEngine &operator=(const AprilTagEngine &) = delete;

  const char *family_name() const { return family_.name; }

  std::vector<DetectedTag> detect(
      const cv::Mat &gray, float quad_decimate, float quad_sigma,
      int nthreads, bool refine_edges, double decode_sharpening, bool debug) {
    std::vector<DetectedTag> out;
    if (gray.empty() || gray.type() != CV_8UC1 || detector_ == nullptr) {
      return out;
    }
    // 拷到 AprilTag 对齐缓冲，避免 step/ROI/非对齐导致 0 检出
    image_u8_t *im = image_u8_create(
        static_cast<unsigned>(gray.cols), static_cast<unsigned>(gray.rows));
    if (im == nullptr || im->buf == nullptr) {
      return out;
    }
    for (int y = 0; y < gray.rows; ++y) {
      std::memcpy(
          im->buf + y * im->stride, gray.ptr(y),
          static_cast<size_t>(gray.cols));
    }

    std::lock_guard<std::mutex> lock(mu_);
    detector_->nthreads = std::max(1, nthreads);
    detector_->refine_edges = refine_edges ? 1 : 0;
    detector_->decode_sharpening = static_cast<float>(decode_sharpening);
    detector_->debug = debug ? 1 : 0;
    detector_->quad_decimate = quad_decimate;
    detector_->quad_sigma = quad_sigma;
    zarray_t *dets = apriltag_detector_detect(detector_, im);
    image_u8_destroy(im);
    if (dets == nullptr) {
      return out;
    }
    const int n = zarray_size(dets);
    out.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      apriltag_detection_t *det = nullptr;
      zarray_get(dets, i, &det);
      if (det == nullptr) {
        continue;
      }
      DetectedTag t;
      t.id = det->id;
      t.margin = det->decision_margin;
      for (int j = 0; j < 4; ++j) {
        t.corners[static_cast<size_t>(j)] = cv::Point2f(
            static_cast<float>(det->p[j][0]),
            static_cast<float>(det->p[j][1]));
      }
      out.push_back(t);
    }
    apriltag_detections_destroy(dets);
    return out;
  }

private:
  FamilyHandle family_;
  apriltag_detector_t *detector_ = nullptr;
  std::mutex mu_;
};

AprilTagEngine &engine_for(const std::string &dict_name) {
  static std::mutex map_mu;
  static std::map<std::string, std::unique_ptr<AprilTagEngine>> engines;
  std::string key = dict_name;
  if (key.find("APRILTAG") == std::string::npos &&
      key.find("tag") == std::string::npos) {
    key = "DICT_APRILTAG_36h11";
  }
  std::lock_guard<std::mutex> lock(map_mu);
  auto it = engines.find(key);
  if (it == engines.end()) {
    it = engines.emplace(key, std::make_unique<AprilTagEngine>(key)).first;
  }
  return *it->second;
}

/// OpenCV 角序 TL,TR,BR,BL → Kalibr/April BL,BR,TR,TL
void opencv_corners_to_kalibr(
    const std::vector<cv::Point2f> &ocv, std::array<cv::Point2f, 4> *out) {
  if (out == nullptr || ocv.size() < 4) {
    return;
  }
  (*out)[0] = ocv[3];  // BL
  (*out)[1] = ocv[2];  // BR
  (*out)[2] = ocv[1];  // TR
  (*out)[3] = ocv[0];  // TL
}

void detect_opencv_april(
    const cv::Mat &gray, const std::string &dict_name,
    std::map<int, DetectedTag> *by_id) {
  if (gray.empty() || by_id == nullptr) {
    return;
  }
  cv::aruco::Dictionary dict;
  try {
    dict = make_aruco_dictionary(
        dict_name.find("APRILTAG") != std::string::npos
            ? dict_name
            : "DICT_APRILTAG_36h11");
  } catch (...) {
    return;
  }
  for (int border : {1, 2}) {
    cv::aruco::DetectorParameters p;
    p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    p.useAruco3Detection = false;
    p.markerBorderBits = border;
    p.adaptiveThreshWinSizeMin = 3;
    p.adaptiveThreshWinSizeMax = 53;
    p.adaptiveThreshWinSizeStep = 2;
    p.minMarkerPerimeterRate = 0.01;
    p.maxMarkerPerimeterRate = 4.0;
    p.aprilTagQuadDecimate = 1.0f;
    p.aprilTagQuadSigma = 0.8f;
    p.aprilTagMinWhiteBlackDiff = 5;
    try {
      cv::aruco::ArucoDetector detector(dict, p);
      std::vector<std::vector<cv::Point2f>> corners;
      std::vector<int> ids;
      detector.detectMarkers(gray, corners, ids);
      for (size_t i = 0; i < ids.size(); ++i) {
        if (corners[i].size() < 4) {
          continue;
        }
        DetectedTag t;
        t.id = ids[i];
        t.margin = 1.0f;
        opencv_corners_to_kalibr(corners[i], &t.corners);
        merge_tag(by_id, std::move(t));
      }
    } catch (...) {
    }
  }
}

bool near_border(const std::array<cv::Point2f, 4> &c, int cols, int rows) {
  const float m = static_cast<float>(kMinBorderDistancePx);
  for (const auto &p : c) {
    if (p.x < m || p.y < m || p.x > static_cast<float>(cols) - m ||
        p.y > static_cast<float>(rows) - m) {
      return true;
    }
  }
  return false;
}

void refine_kalibr_subpix(
    const cv::Mat &gray, std::array<cv::Point2f, 4> *corners, bool *keep) {
  if (corners == nullptr || keep == nullptr) {
    return;
  }
  std::vector<cv::Point2f> raw(corners->begin(), corners->end());
  std::vector<cv::Point2f> refined = raw;
  cv::cornerSubPix(
      gray, refined, cv::Size(3, 3), cv::Size(-1, -1),
      cv::TermCriteria(
          cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.03));
  int kept = 0;
  for (int j = 0; j < 4; ++j) {
    const double dx = static_cast<double>(refined[static_cast<size_t>(j)].x -
                                          raw[static_cast<size_t>(j)].x);
    const double dy = static_cast<double>(refined[static_cast<size_t>(j)].y -
                                          raw[static_cast<size_t>(j)].y);
    const double d2 = dx * dx + dy * dy;
    if (d2 <= kMaxSubpixDisplacement2) {
      (*corners)[static_cast<size_t>(j)] = refined[static_cast<size_t>(j)];
      keep[j] = true;
      ++kept;
    } else {
      // 位移过大：保留原角点，勿整点丢弃（屏显摩尔纹常见）
      keep[j] = true;
    }
  }
  (void)kept;
}

void run_apriltag_variants(
    const cv::Mat &gray, AprilTagEngine &eng,
    const AprilgridDetectorParams &params, std::map<int, DetectedTag> *by_id) {
  const float decims[] = {
      static_cast<float>(params.quad_decimate),
      static_cast<float>(params.quad_decimate) * 1.5f,
      static_cast<float>(params.quad_decimate) * 2.0f};
  const float sigmas[] = {0.0f, static_cast<float>(params.quad_sigma)};
  for (float d : decims) {
    if (d < 0.5f) {
      continue;
    }
    for (float s : sigmas) {
      for (auto &t : eng.detect(
               gray, d, s, params.nthreads, params.refine_edges,
               params.decode_sharpening, params.debug)) {
        merge_tag(by_id, std::move(t));
      }
      if (by_id->size() >= 12) {
        return;
      }
    }
  }
}

std::map<int, DetectedTag> detect_robust(
    const cv::Mat &gray, const std::string &dict_name,
    const AprilgridDetectorParams &params, std::string *hit_name) {
  std::map<int, DetectedTag> by_id;
  AprilTagEngine &eng = engine_for(dict_name);
  if (hit_name != nullptr) {
    *hit_name = eng.family_name();
  }

  run_apriltag_variants(gray, eng, params, &by_id);
  if (by_id.size() >= 8) {
    return by_id;
  }

  cv::Mat eq;
  cv::createCLAHE(3.0, cv::Size(8, 8))->apply(gray, eq);
  run_apriltag_variants(eq, eng, params, &by_id);
  if (by_id.size() >= 8) {
    return by_id;
  }

  // 仍不足：OpenCV AprilTag 字典兜底（borderBits 1/2）
  detect_opencv_april(gray, dict_name, &by_id);
  detect_opencv_april(eq, dict_name, &by_id);
  return by_id;
}

}  // namespace

AprilgridDetector::AprilgridDetector(
    AprilgridTarget target, std::string dictionary_name,
    AprilgridDetectorParams params)
    : target_(std::move(target)),
      dictionary_name_(
          dictionary_name.empty() ? "DICT_APRILTAG_36h11"
                                  : std::move(dictionary_name)),
      params_(params) {}

std::vector<Correspondence> AprilgridDetector::detect(
    const ImageFrame &frame, const TargetModelBase &target) const {
  (void)target;
  return detect(frame, static_cast<DetectedMarkers *>(nullptr));
}

std::vector<Correspondence> AprilgridDetector::detect(
    const ImageFrame &frame, DetectedMarkers *markers) const {
  std::vector<Correspondence> out;
  cv::Mat mat = image_frame_as_mat(frame);
  if (mat.empty()) {
    return out;
  }
  cv::Mat gray;
  if (mat.channels() == 1) {
    gray = mat;
  } else if (mat.channels() == 4) {
    cv::cvtColor(mat, gray, cv::COLOR_BGRA2GRAY);
  } else {
    cv::cvtColor(mat, gray, cv::COLOR_BGR2GRAY);
  }
  if (!gray.isContinuous()) {
    gray = gray.clone();
  }

  std::string hit_name;
  auto raw = detect_robust(gray, dictionary_name_, params_, &hit_name);

  std::map<int, DetectedTag> by_id;
  for (auto &kv : raw) {
    DetectedTag t = kv.second;
    if (t.id < 0 || t.id >= target_.num_tags()) {
      continue;
    }
    if (near_border(t.corners, gray.cols, gray.rows)) {
      continue;
    }
    if (t.margin < params_.min_margin) {
      continue;
    }
    merge_tag(&by_id, std::move(t));
  }

  const int expected = target_.num_tags();
  if (expected > 0 &&
      static_cast<double>(by_id.size()) <
          params_.min_detection_ratio * static_cast<double>(expected)) {
    return out;
  }

  if (markers != nullptr) {
    markers->corners.clear();
    markers->ids.clear();
    markers->dictionary_name = hit_name.empty() ? dictionary_name_ : hit_name;
    // 预览：显示原始检出（含越界 ID），便于排查字典/网格配置
    for (const auto &kv : raw) {
      markers->ids.push_back(kv.first);
      markers->corners.emplace_back(
          kv.second.corners.begin(), kv.second.corners.end());
    }
  }

  if (static_cast<int>(by_id.size()) < kMinTagsForObs) {
    return out;
  }

  const Eigen::MatrixXd obj_all = target_.all_object_points();
  Correspondence corr;
  corr.image_points.resize(0, 2);
  corr.object_points.resize(0, 3);
  corr.ids.clear();

  for (auto &kv : by_id) {
    const int tag_id = kv.first;
    std::array<cv::Point2f, 4> corners = kv.second.corners;
    bool keep[4] = {true, true, true, true};
    refine_kalibr_subpix(gray, &corners, keep);

    std::array<int, 4> p_idx;
    try {
      p_idx = target_.corner_indices_for_tag(tag_id);
    } catch (...) {
      continue;
    }
    for (int j = 0; j < 4; ++j) {
      if (!keep[j]) {
        continue;
      }
      const int grid_idx = p_idx[static_cast<size_t>(j)];
      if (grid_idx < 0 || grid_idx >= obj_all.rows()) {
        continue;
      }
      const cv::Point2f &pt = corners[static_cast<size_t>(j)];
      const int row = static_cast<int>(corr.image_points.rows());
      corr.image_points.conservativeResize(row + 1, 2);
      corr.object_points.conservativeResize(row + 1, 3);
      corr.image_points(row, 0) = pt.x;
      corr.image_points(row, 1) = pt.y;
      corr.object_points.row(row) = obj_all.row(grid_idx);
      corr.ids.push_back(grid_idx);
    }
  }

  if (corr.image_points.rows() < 4) {
    return out;
  }
  out.push_back(std::move(corr));
  return out;
}

}  // namespace core
}  // namespace hs_calib
