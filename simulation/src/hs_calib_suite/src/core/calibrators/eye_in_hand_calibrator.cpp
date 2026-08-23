#include "hs_calib_suite/core/calibrators/eye_in_hand_calibrator.hpp"

#include <cmath>
#include <sstream>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "hs_calib_suite/core/io/board_pose.hpp"
#include "hs_calib_suite/core/io/export_camera_yaml.hpp"
#include "hs_calib_suite/core/registry/registry.hpp"

namespace hs_calib {
namespace core {

namespace {

/// \brief 解析手眼标定算法名称 → OpenCV 枚举
cv::HandEyeCalibrationMethod parse_method(const std::map<std::string, std::string> &config) {
  const auto it = config.find("method");
  if (it == config.end()) {
    return cv::CALIB_HAND_EYE_TSAI;
  }
  const std::string &m = it->second;
  if (m == "park") {
    return cv::CALIB_HAND_EYE_PARK;
  }
  if (m == "horaud") {
    return cv::CALIB_HAND_EYE_HORAUD;
  }
  if (m == "andreff") {
    return cv::CALIB_HAND_EYE_ANDREFF;
  }
  if (m == "daniilidis") {
    return cv::CALIB_HAND_EYE_DANIILIDIS;
  }
  return cv::CALIB_HAND_EYE_TSAI;
}

/// \brief 从 camera_yaml 或 fx/fy 等字段加载相机内参
bool load_K_from_config(
    const std::map<std::string, std::string> &config, CameraIntrinsics *intr) {
  const auto it = config.find("camera_yaml");
  if (it != config.end() && !it->second.empty()) {
    return load_camera_yaml(it->second, intr);
  }
  // 允许直接塞 fx,fy,cx,cy,k1..
  if (!config.count("fx") || !config.count("fy")) {
    return false;
  }
  CalibrationResult fake;
  fake.success = true;
  fake.intrinsics_meta = config;
  return camera_intrinsics_from_result(fake, intr);
}

/// \brief SE(3) 刚体变换求逆
Eigen::Matrix4d invert_se3(const Eigen::Matrix4d &T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Vector3d t = T.block<3, 1>(0, 3);
  Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
  Ti.block<3, 3>(0, 0) = R.transpose();
  Ti.block<3, 1>(0, 3) = -R.transpose() * t;
  return Ti;
}

/// \brief 两旋转矩阵间的测地线角距离（度）
double rotation_geodesic_deg(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B) {
  Eigen::Matrix3d R = A.transpose() * B;
  double c = (R.trace() - 1.0) * 0.5;
  c = std::max(-1.0, std::min(1.0, c));
  return std::acos(c) * 180.0 / CV_PI;
}

}  // namespace

/// \brief 返回标定器元信息（眼在手上模式）
CalibratorInfo EyeInHandCalibrator::calibrator_info() const {
  CalibratorInfo info;
  info.calibrator_id = "eye_in_hand";
  info.display_name = "眼在手上";
  info.category = "handeye";
  info.required_frames = {"base", "gripper", "camera"};
  info.supported_targets = {"chessboard", "aruco", "aruco_grid"};
  return info;
}

/// \brief 眼在手上：calibrateHandEye 求解 T_gripper_camera
CalibrationResult EyeInHandCalibrator::calibrate(
    const ObservationBatch &observations,
    const std::map<std::string, std::string> &config) const {
  CalibrationResult result;
  // —— 加载相机内参 ——
  CameraIntrinsics intr;
  if (!load_K_from_config(config, &intr) || !intr.valid) {
    result.success = false;
    result.message = "需要有效相机内参（config.camera_yaml 或 fx/fy/…）：" +
                     (intr.message.empty() ? std::string("missing") : intr.message);
    return result;
  }

  std::string parent = "gripper";
  std::string child = "camera";
  if (config.count("parent_frame")) {
    parent = config.at("parent_frame");
  }
  if (config.count("child_frame")) {
    child = config.at("child_frame");
  }

  std::vector<cv::Mat> R_g2b;
  std::vector<cv::Mat> t_g2b;
  std::vector<cv::Mat> R_t2c;
  std::vector<cv::Mat> t_t2c;

  // —— 逐帧收集 gripper2base 与 target2cam ——
  for (const auto &obs : observations.items) {
    if (!obs.has_base_gripper || obs.correspondences.empty()) {
      continue;
    }
    Eigen::Matrix4d T_cam_target;
    std::string err;
    if (!estimate_board_pose(obs.correspondences.front(), intr.K, intr.D, &T_cam_target, &err)) {
      continue;
    }
    // OpenCV target2cam = T_cam_target
    cv::Mat R_tc;
    cv::Mat t_tc;
    matrix4d_to_cv_rt(T_cam_target, &R_tc, &t_tc);
    cv::Mat R_gb;
    cv::Mat t_gb;
    matrix4d_to_cv_rt(obs.T_base_gripper, &R_gb, &t_gb);
    R_g2b.push_back(R_gb);
    t_g2b.push_back(t_gb);
    R_t2c.push_back(R_tc);
    t_t2c.push_back(t_tc);
  }

  if (R_g2b.size() < 3) {
    result.success = false;
    result.message = "有效手眼样本不足（需 ≥3 组：检测+位姿）";
    return result;
  }

  cv::Mat R_c2g;
  cv::Mat t_c2g;
  // —— OpenCV 手眼标定 ——
  try {
    cv::calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c, R_c2g, t_c2g, parse_method(config));
  } catch (const cv::Exception &ex) {
    result.success = false;
    result.message = std::string("calibrateHandEye failed: ") + ex.what();
    return result;
  }

  Eigen::Matrix4d T_gripper_camera = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      T_gripper_camera(r, c) = R_c2g.at<double>(r, c);
    }
    T_gripper_camera(r, 3) = t_c2g.at<double>(r, 0);
  }

  // —— AX≈XB 旋转一致性评估 ——
  double rot_err = 0.0;
  int pairs = 0;
  for (size_t i = 0; i + 1 < R_g2b.size(); ++i) {
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Ai = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d B = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Bi = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        A(r, c) = R_g2b[i + 1].at<double>(r, c);
        Ai(r, c) = R_g2b[i].at<double>(r, c);
        B(r, c) = R_t2c[i + 1].at<double>(r, c);
        Bi(r, c) = R_t2c[i].at<double>(r, c);
      }
    }
    Eigen::Matrix4d Am = invert_se3(Ai) * A;
    Eigen::Matrix4d Bm = invert_se3(Bi) * B;
    Eigen::Matrix4d AX = Am * T_gripper_camera;
    Eigen::Matrix4d XB = T_gripper_camera * Bm;
    rot_err += rotation_geodesic_deg(AX.block<3, 3>(0, 0), XB.block<3, 3>(0, 0));
    ++pairs;
  }
  if (pairs > 0) {
    rot_err /= static_cast<double>(pairs);
  }

  result.success = true;
  result.message = "eye_in_hand ok";
  result.score = static_cast<float>(1.0 / (1.0 + rot_err));
  result.metrics["num_pairs"] = static_cast<double>(R_g2b.size());
  result.metrics["handeye_rmse"] = rot_err;
  result.transforms[parent][child] = T_gripper_camera;
  result.intrinsics_meta["parent_frame"] = parent;
  result.intrinsics_meta["child_frame"] = child;
  result.intrinsics_meta["mode"] = "eye_in_hand";
  return result;
}

HS_CALIB_REGISTER("eye_in_hand", EyeInHandCalibrator);

}  // namespace core
}  // namespace hs_calib
