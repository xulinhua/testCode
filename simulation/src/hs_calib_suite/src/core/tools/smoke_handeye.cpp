/// \brief 合成手眼样本，验证 eye_in_hand 求解

#include <cmath>
#include <iostream>
#include <map>
#include <string>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "hs_calib_suite/core/targets/chessboard_target.hpp"
#include "hs_calib_suite/core/registry/registry.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace {

Eigen::Matrix4d make_T(const Eigen::Matrix3d &R, const Eigen::Vector3d &t) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

Eigen::Matrix4d invert_se3(const Eigen::Matrix4d &T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Vector3d t = T.block<3, 1>(0, 3);
  Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
  Ti.block<3, 3>(0, 0) = R.transpose();
  Ti.block<3, 1>(0, 3) = -R.transpose() * t;
  return Ti;
}

/// OpenCV/USD-like camera: +Z forward, look from eye to target
Eigen::Matrix4d look_at_cam(const Eigen::Vector3d &eye, const Eigen::Vector3d &target) {
  Eigen::Vector3d z = (target - eye).normalized();
  Eigen::Vector3d up(0, 0, 1);
  Eigen::Vector3d x = up.cross(z);
  if (x.norm() < 1e-6) {
    up = Eigen::Vector3d(0, 1, 0);
    x = up.cross(z);
  }
  x.normalize();
  Eigen::Vector3d y = z.cross(x);
  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  return make_T(R, eye);
}

}  // namespace

int main() {
  using namespace hs_calib::core;

  const Eigen::Matrix3d R_true =
      (Eigen::AngleAxisd(12.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-8.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()))
          .toRotationMatrix();
  const Eigen::Vector3d t_true(0.04, -0.01, 0.06);
  const Eigen::Matrix4d T_gripper_camera_true = make_T(R_true, t_true);

  const double fx = 700.0;
  const double fy = 700.0;
  const double cx = 320.0;
  const double cy = 240.0;

  ChessboardTarget board(9, 6, 0.025);
  const Eigen::MatrixXd obj_e = board.all_object_points();
  const Eigen::Vector3d board_center_local(0.025 * 4.5, 0.025 * 3.0, 0.0);

  const Eigen::Matrix4d T_base_board =
      make_T(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.55, 0.0, 0.20));
  const Eigen::Vector3d board_center =
      T_base_board.block<3, 3>(0, 0) * board_center_local +
      T_base_board.block<3, 1>(0, 3);

  ObservationBatch batch;
  for (int i = 0; i < 12; ++i) {
    const double az = (i * 30.0) * M_PI / 180.0;
    const double elev = (35.0 + 5.0 * (i % 4)) * M_PI / 180.0;
    const double dist = 0.55 + 0.03 * (i % 3);
    Eigen::Vector3d eye =
        board_center + Eigen::Vector3d(
                           dist * std::cos(elev) * std::cos(az),
                           dist * std::cos(elev) * std::sin(az),
                           dist * std::sin(elev));
    const Eigen::Matrix4d T_base_camera = look_at_cam(eye, board_center);
    const Eigen::Matrix4d T_base_gripper =
        T_base_camera * invert_se3(T_gripper_camera_true);
    const Eigen::Matrix4d T_cam_board = invert_se3(T_base_camera) * T_base_board;

    Correspondence corr;
    corr.object_points = obj_e;
    corr.image_points.resize(obj_e.rows(), 2);
    bool ok = true;
    for (int r = 0; r < obj_e.rows(); ++r) {
      Eigen::Vector4d ph(obj_e(r, 0), obj_e(r, 1), obj_e(r, 2), 1.0);
      Eigen::Vector4d pc = T_cam_board * ph;
      if (pc(2) < 0.05) {
        ok = false;
        break;
      }
      corr.image_points(r, 0) = fx * pc(0) / pc(2) + cx;
      corr.image_points(r, 1) = fy * pc(1) / pc(2) + cy;
    }
    if (!ok) {
      continue;
    }

    Observation obs;
    obs.has_base_gripper = true;
    obs.T_base_gripper = T_base_gripper;
    obs.image_width = 640;
    obs.image_height = 480;
    obs.correspondences = {corr};
    obs.source_path = "synthetic_" + std::to_string(i);
    batch.items.push_back(std::move(obs));
  }

  std::cout << "synthetic views: " << batch.items.size() << "\n";
  if (batch.items.size() < 3) {
    std::cerr << "SMOKE FAIL: not enough synthetic views\n";
    return 1;
  }

  std::map<std::string, std::string> cfg = {
      {"fx", std::to_string(fx)},
      {"fy", std::to_string(fy)},
      {"cx", std::to_string(cx)},
      {"cy", std::to_string(cy)},
      {"k1", "0"},
      {"k2", "0"},
      {"p1", "0"},
      {"p2", "0"},
      {"k3", "0"},
      {"method", "tsai"},
      {"parent_frame", "gripper"},
      {"child_frame", "camera"},
  };

  auto calib = CalibratorRegistry::instance().create("eye_in_hand");
  const CalibrationResult res = calib->calibrate(batch, cfg);
  if (!res.success) {
    std::cerr << "SMOKE FAIL: " << res.message << "\n";
    return 1;
  }
  const Eigen::Matrix4d &Test = res.transforms.at("gripper").at("camera");
  const Eigen::Matrix3d dR =
      T_gripper_camera_true.block<3, 3>(0, 0).transpose() * Test.block<3, 3>(0, 0);
  double c = (dR.trace() - 1.0) * 0.5;
  c = std::max(-1.0, std::min(1.0, c));
  const double ang = std::acos(c) * 180.0 / M_PI;
  const double tnorm =
      (T_gripper_camera_true.block<3, 1>(0, 3) - Test.block<3, 1>(0, 3)).norm();

  std::cout << "SMOKE handeye rot_err_deg=" << ang << " t_err_m=" << tnorm << "\n";
  if (ang > 3.0 || tnorm > 0.02) {
    std::cerr << "SMOKE FAIL: error too large\n";
    return 2;
  }
  std::cout << "SMOKE OK\n";
  return 0;
}
