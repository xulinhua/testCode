#include "hs_calib_suite/core/util/camera_models.hpp"

#include <algorithm>
#include <cctype>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#if defined(__has_include)
#  if __has_include(<opencv2/ccalib/omnidir.hpp>)
#    include <opencv2/ccalib/omnidir.hpp>
#    define HS_CALIB_HAS_OMNIDIR 1
#  endif
#endif
#ifndef HS_CALIB_HAS_OMNIDIR
#  define HS_CALIB_HAS_OMNIDIR 0
#endif

namespace hs_calib {
namespace core {
namespace {

std::string lower_copy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

cv::Mat as_row_or_col_64f(const cv::Mat &D, int prefer_cols) {
  if (D.empty()) {
    return cv::Mat::zeros(1, prefer_cols, CV_64F);
  }
  cv::Mat out;
  D.convertTo(out, CV_64F);
  if (out.rows == 1 || out.cols == 1) {
    return out;
  }
  return out.reshape(1, 1);
}

}  // namespace

CameraModelId parse_camera_model(const std::string &name) {
  const std::string n = lower_copy(name);
  if (n.empty() || n == "pinhole" || n == "brown" || n == "brown_conrady" ||
      n == "brown-conrady" || n == "plumb_bob" || n == "rational_polynomial") {
    return CameraModelId::BrownConrady;
  }
  if (n == "fisheye" || n == "kannala" || n == "kannala_brandt" ||
      n == "kannala-brandt" || n == "brandt_kannala" || n == "brandt-kannala" ||
      n == "equidistant") {
    return CameraModelId::KannalaBrandt;
  }
  if (n == "cmei" || n == "mei" || n == "omnidir" || n == "omni") {
    return CameraModelId::CMei;
  }
  return CameraModelId::BrownConrady;
}

std::string camera_model_to_string(CameraModelId id) {
  switch (id) {
    case CameraModelId::KannalaBrandt:
      return "kannala_brandt";
    case CameraModelId::CMei:
      return "cmei";
    case CameraModelId::BrownConrady:
    default:
      return "brown_conrady";
  }
}

std::string camera_model_display_name(CameraModelId id) {
  switch (id) {
    case CameraModelId::KannalaBrandt:
      return "Kannala-Brandt (fisheye)";
    case CameraModelId::CMei:
      return "CMei (omnidir)";
    case CameraModelId::BrownConrady:
    default:
      return "Brown-Conrady (pinhole)";
  }
}

bool project_points_model(
    CameraModelId model,
    const std::vector<cv::Point3f> &object_points,
    const cv::Mat &rvec,
    const cv::Mat &tvec,
    const cv::Mat &K,
    const cv::Mat &D,
    double xi,
    std::vector<cv::Point2f> *image_points) {
  if (image_points == nullptr || object_points.empty() || K.empty()) {
    return false;
  }
  try {
    if (model == CameraModelId::KannalaBrandt) {
      cv::Mat Dd = as_row_or_col_64f(D, 4);
      cv::fisheye::projectPoints(object_points, *image_points, rvec, tvec, K, Dd);
      return image_points->size() == object_points.size();
    }
    if (model == CameraModelId::CMei) {
#if HS_CALIB_HAS_OMNIDIR
      cv::Mat Dd = as_row_or_col_64f(D, 4);
      cv::omnidir::projectPoints(object_points, *image_points, rvec, tvec, K, xi, Dd);
      return image_points->size() == object_points.size();
#else
      (void)xi;
      return false;
#endif
    }
    cv::projectPoints(object_points, rvec, tvec, K, D, *image_points);
    return image_points->size() == object_points.size();
  } catch (const cv::Exception &) {
    return false;
  }
}

bool solve_pnp_model(
    CameraModelId model,
    const std::vector<cv::Point3f> &object_points,
    const std::vector<cv::Point2f> &image_points,
    const cv::Mat &K,
    const cv::Mat &D,
    double xi,
    cv::Mat *rvec,
    cv::Mat *tvec,
    bool use_square) {
  if (rvec == nullptr || tvec == nullptr || object_points.size() < 4 ||
      object_points.size() != image_points.size() || K.empty()) {
    return false;
  }
  const int method = use_square ? cv::SOLVEPNP_IPPE_SQUARE : cv::SOLVEPNP_ITERATIVE;
  auto try_pnp = [&](const std::vector<cv::Point2f> &pts, const cv::Mat &cam_K,
                     const cv::Mat &cam_D) -> bool {
    bool ok = cv::solvePnP(object_points, pts, cam_K, cam_D, *rvec, *tvec, false, method);
    if (!ok && use_square) {
      ok = cv::solvePnP(
          object_points, pts, cam_K, cam_D, *rvec, *tvec, false, cv::SOLVEPNP_ITERATIVE);
    }
    return ok;
  };

  try {
    if (model == CameraModelId::KannalaBrandt) {
      cv::Mat Dd = as_row_or_col_64f(D, 4);
      std::vector<cv::Point2f> und;
      // 去畸变到与 K 一致的针孔像素，再走标准 PnP（兼容 OpenCV 4.5，无 fisheye::solvePnP）
      cv::fisheye::undistortPoints(image_points, und, K, Dd, cv::noArray(), K);
      if (und.size() != object_points.size()) {
        return false;
      }
      return try_pnp(und, K, cv::Mat());
    }
    if (model == CameraModelId::CMei) {
#if HS_CALIB_HAS_OMNIDIR
      cv::Mat Dd = as_row_or_col_64f(D, 4);
      cv::Mat xi_m = (cv::Mat_<double>(1, 1) << xi);
      std::vector<cv::Point2f> und;
      cv::omnidir::undistortPoints(image_points, und, K, Dd, xi_m, cv::Mat());
      if (und.size() != object_points.size()) {
        return false;
      }
      const cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
      return try_pnp(und, I, cv::Mat());
#else
      (void)xi;
      return false;
#endif
    }
    return try_pnp(image_points, K, D);
  } catch (const cv::Exception &) {
    return false;
  }
}

void draw_frame_axes_model(
    cv::Mat &image,
    CameraModelId model,
    const cv::Mat &K,
    const cv::Mat &D,
    double xi,
    const cv::Mat &rvec,
    const cv::Mat &tvec,
    float length,
    int thickness) {
  if (image.empty() || length <= 0.f) {
    return;
  }
  const std::vector<cv::Point3f> axis = {
      {0.f, 0.f, 0.f}, {length, 0.f, 0.f}, {0.f, length, 0.f}, {0.f, 0.f, length}};
  std::vector<cv::Point2f> img_pts;
  if (!project_points_model(model, axis, rvec, tvec, K, D, xi, &img_pts) ||
      img_pts.size() != 4) {
    return;
  }
  const cv::Point o(cvRound(img_pts[0].x), cvRound(img_pts[0].y));
  const cv::Point x(cvRound(img_pts[1].x), cvRound(img_pts[1].y));
  const cv::Point y(cvRound(img_pts[2].x), cvRound(img_pts[2].y));
  const cv::Point z(cvRound(img_pts[3].x), cvRound(img_pts[3].y));
  cv::line(image, o, x, cv::Scalar(0, 0, 255), thickness, cv::LINE_AA);
  cv::line(image, o, y, cv::Scalar(0, 255, 0), thickness, cv::LINE_AA);
  cv::line(image, o, z, cv::Scalar(255, 0, 0), thickness, cv::LINE_AA);
}

}  // namespace core
}  // namespace hs_calib
