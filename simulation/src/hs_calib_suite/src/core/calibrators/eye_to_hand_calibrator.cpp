#include "hs_calib_suite/core/calibrators/eye_to_hand_calibrator.hpp"

#include <algorithm>
#include <cmath>
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

}  // namespace

/// \brief 返回标定器元信息（眼在手外模式）
CalibratorInfo EyeToHandCalibrator::calibrator_info() const {
  CalibratorInfo info;
  info.calibrator_id = "eye_to_hand";
  info.display_name = "眼在手外";
  info.category = "handeye";
  info.required_frames = {"base", "gripper", "camera"};
  info.supported_targets = {"chessboard", "charuco", "aruco", "aruco_grid", "aprilgrid"};
  return info;
}

/// \brief 眼在手外：calibrateHandEye 求解 T_base_camera
CalibrationResult EyeToHandCalibrator::calibrate(
    const ObservationBatch &observations,
    const std::map<std::string, std::string> &config) const {
  CalibrationResult result;
  // —— 加载相机内参 ——
  CameraIntrinsics intr;
  if (!load_K_from_config(config, &intr) || !intr.valid) {
    result.success = false;
    result.message = "需要有效相机内参（config.camera_yaml）：" +
                     (intr.message.empty() ? std::string("missing") : intr.message);
    return result;
  }

  std::string parent = "base";
  std::string child = "camera";
  if (config.count("parent_frame")) {
    parent = config.at("parent_frame");
  }
  if (config.count("child_frame")) {
    child = config.at("child_frame");
  }

  std::vector<cv::Mat> R_b2g;
  std::vector<cv::Mat> t_b2g;
  std::vector<cv::Mat> R_t2c;
  std::vector<cv::Mat> t_t2c;

  // —— 逐帧收集 base2gripper 与 target2cam ——
  for (const auto &obs : observations.items) {
    if (!obs.has_base_gripper || obs.correspondences.empty()) {
      continue;
    }
    Eigen::Matrix4d T_cam_target;
    std::string err;
    if (!estimate_board_pose(obs.correspondences.front(), intr.K, intr.D, &T_cam_target, &err)) {
      continue;
    }
    // 眼在手外：把 gripper2base 取逆作为 base2gripper 传入 calibrateHandEye 的 gripper2base 槽
    const Eigen::Matrix4d T_gripper_base = invert_se3(obs.T_base_gripper);
    cv::Mat R_bg;
    cv::Mat t_bg;
    matrix4d_to_cv_rt(T_gripper_base, &R_bg, &t_bg);
    cv::Mat R_tc;
    cv::Mat t_tc;
    matrix4d_to_cv_rt(T_cam_target, &R_tc, &t_tc);
    R_b2g.push_back(R_bg);
    t_b2g.push_back(t_bg);
    R_t2c.push_back(R_tc);
    t_t2c.push_back(t_tc);
  }

  if (R_b2g.size() < 3) {
    result.success = false;
    result.message = "有效手眼样本不足（需 ≥3 组：检测+位姿）";
    return result;
  }

  cv::Mat R_c2b;
  cv::Mat t_c2b;
  // —— OpenCV 手眼标定 ——
  try {
    cv::calibrateHandEye(
        R_b2g, t_b2g, R_t2c, t_t2c, R_c2b, t_c2b, parse_method(config));
  } catch (const cv::Exception &ex) {
    result.success = false;
    result.message = std::string("calibrateHandEye (eye_to_hand) failed: ") + ex.what();
    return result;
  }

  Eigen::Matrix4d T_base_camera = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      T_base_camera(r, c) = R_c2b.at<double>(r, c);
    }
    T_base_camera(r, 3) = t_c2b.at<double>(r, 0);
  }

  double rot_err = 0.0;
  double t_err = 0.0;
  int pairs = 0;
  for (size_t i = 0; i + 1 < R_b2g.size(); ++i) {
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Ai = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d B = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Bi = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        A(r, c) = R_b2g[i + 1].at<double>(r, c);
        Ai(r, c) = R_b2g[i].at<double>(r, c);
        B(r, c) = R_t2c[i + 1].at<double>(r, c);
        Bi(r, c) = R_t2c[i].at<double>(r, c);
      }
    }
    Eigen::Matrix4d Am = invert_se3(Ai) * A;
    Eigen::Matrix4d Bm = invert_se3(Bi) * B;
    Eigen::Matrix4d AX = Am * T_base_camera;
    Eigen::Matrix4d XB = T_base_camera * Bm;
    Eigen::Matrix3d R = AX.block<3, 3>(0, 0).transpose() * XB.block<3, 3>(0, 0);
    double c = (R.trace() - 1.0) * 0.5;
    c = std::max(-1.0, std::min(1.0, c));
    rot_err += std::acos(c) * 180.0 / CV_PI;
    t_err += (AX.block<3, 1>(0, 3) - XB.block<3, 1>(0, 3)).norm();
    ++pairs;
  }
  if (pairs > 0) {
    rot_err /= static_cast<double>(pairs);
    t_err /= static_cast<double>(pairs);
  }

  result.success = true;
  result.message = "eye_to_hand ok";
  result.score = static_cast<float>(1.0 / (1.0 + rot_err));
  result.metrics["num_pairs"] = static_cast<double>(R_b2g.size());
  result.metrics["handeye_rmse"] = rot_err;
  result.metrics["handeye_t_rmse"] = t_err;
  result.transforms[parent][child] = T_base_camera;
  result.intrinsics_meta["parent_frame"] = parent;
  result.intrinsics_meta["child_frame"] = child;
  result.intrinsics_meta["mode"] = "eye_to_hand";
  return result;
}

HS_CALIB_REGISTER("eye_to_hand", EyeToHandCalibrator);

}  // namespace core
}  // namespace hs_calib
