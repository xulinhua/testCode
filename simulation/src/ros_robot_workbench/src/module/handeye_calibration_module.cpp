#include "ros_robot_workbench/module/handeye_calibration_module.h"

#include <fstream>
#include <algorithm>
#include <map>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <vector>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStringList>
#include <QTextStream>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace ros_robot_workbench::ui
{
namespace
{

struct CsvRow
{
  QString raw_image;
  QString intrinsics_file;
  cv::Vec3d t_base_gripper;
  cv::Vec4d q_base_gripper_xyzw;
};

bool ParseCsvRows(const QString & csv_path, std::vector<CsvRow> * rows, QString * err_msg)
{
  if (!rows) return false;
  rows->clear();
  QFile f(csv_path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
    if (err_msg) *err_msg = "无法打开 CSV 文件";
    return false;
  }
  QTextStream in(&f);
  if (in.atEnd()) {
    if (err_msg) *err_msg = "CSV 文件为空";
    return false;
  }
  const QString header = in.readLine().trimmed();
  const QStringList cols = header.split(',', Qt::KeepEmptyParts);
  auto idx_of = [&](const QString & name) { return cols.indexOf(name); };
  const int i_raw = idx_of("raw_image");
  const int i_intr = idx_of("intrinsics_file");
  const int i_px = idx_of("px");
  const int i_py = idx_of("py");
  const int i_pz = idx_of("pz");
  const int i_qx = idx_of("qx");
  const int i_qy = idx_of("qy");
  const int i_qz = idx_of("qz");
  const int i_qw = idx_of("qw");
  if (i_raw < 0 || i_px < 0 || i_py < 0 || i_pz < 0 || i_qx < 0 || i_qy < 0 || i_qz < 0 || i_qw < 0) {
    if (err_msg) *err_msg = "CSV 缺少必须列: raw_image/px/py/pz/qx/qy/qz/qw";
    return false;
  }
  while (!in.atEnd()) {
    const QString line = in.readLine().trimmed();
    if (line.isEmpty()) continue;
    const QStringList c = line.split(',', Qt::KeepEmptyParts);
    if (c.size() <= std::max({i_raw, i_px, i_py, i_pz, i_qx, i_qy, i_qz, i_qw})) continue;
    bool ok = true;
    CsvRow r;
    r.raw_image = c[i_raw].trimmed();
    r.intrinsics_file = (i_intr >= 0 && i_intr < c.size()) ? c[i_intr].trimmed() : QString();
    r.t_base_gripper = cv::Vec3d(c[i_px].toDouble(&ok), 0.0, 0.0);
    if (!ok) continue;
    r.t_base_gripper[1] = c[i_py].toDouble(&ok);
    if (!ok) continue;
    r.t_base_gripper[2] = c[i_pz].toDouble(&ok);
    if (!ok) continue;
    r.q_base_gripper_xyzw = cv::Vec4d(c[i_qx].toDouble(&ok), 0.0, 0.0, 0.0);
    if (!ok) continue;
    r.q_base_gripper_xyzw[1] = c[i_qy].toDouble(&ok);
    if (!ok) continue;
    r.q_base_gripper_xyzw[2] = c[i_qz].toDouble(&ok);
    if (!ok) continue;
    r.q_base_gripper_xyzw[3] = c[i_qw].toDouble(&ok);
    if (!ok) continue;
    rows->push_back(r);
  }
  if (rows->empty()) {
    if (err_msg) *err_msg = "CSV 无有效样本行";
    return false;
  }
  return true;
}

bool LoadIntrinsics(const QString & path, cv::Mat * K, cv::Mat * D)
{
  cv::FileStorage fs(path.toStdString(), cv::FileStorage::READ);
  if (!fs.isOpened()) return false;
  cv::Mat camera_matrix;
  cv::Mat distortion;
  fs["camera_matrix"] >> camera_matrix;
  fs["distortion_coefficients"] >> distortion;
  if (camera_matrix.empty() || distortion.empty()) return false;
  *K = camera_matrix.clone();
  *D = distortion.clone();
  return true;
}

cv::Matx33d QuatToR(const cv::Vec4d & q_xyzw)
{
  const double x = q_xyzw[0];
  const double y = q_xyzw[1];
  const double z = q_xyzw[2];
  const double w = q_xyzw[3];
  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;
  const double xy = x * y;
  const double xz = x * z;
  const double yz = y * z;
  const double wx = w * x;
  const double wy = w * y;
  const double wz = w * z;
  return cv::Matx33d(
    1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
    2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
    2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy));
}

void AppendMatrix4x4(YAML::Emitter & out, const cv::Matx33d & R, const cv::Vec3d & t)
{
  out << YAML::Flow << std::vector<double>{
    R(0,0), R(0,1), R(0,2), t[0],
    R(1,0), R(1,1), R(1,2), t[1],
    R(2,0), R(2,1), R(2,2), t[2],
    0.0, 0.0, 0.0, 1.0
  };
}

double RotationResidualDeg(const cv::Matx33d & r_ref, const cv::Matx33d & r_cur)
{
  constexpr double kRadToDeg = 57.29577951308232;
  const cv::Matx33d d = r_ref * r_cur.t();
  const double tr = d(0, 0) + d(1, 1) + d(2, 2);
  const double c = std::clamp((tr - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(c) * kRadToDeg;
}

}  // namespace


QString HandeyeSetupModeToYamlString(HandeyeSetupMode mode)
{
  return mode == HandeyeSetupMode::EyeInHand ? "eye_in_hand" : "eye_to_hand";
}

QString HandeyeSolverMethodToYamlString(HandeyeSolverMethod method)
{
  switch (method) {
    case HandeyeSolverMethod::Tsai:
      return "TSAI";
    case HandeyeSolverMethod::Park:
      return "PARK";
    case HandeyeSolverMethod::Horaud:
      return "HORAUD";
    case HandeyeSolverMethod::Andreff:
      return "ANDREFF";
    case HandeyeSolverMethod::Daniilidis:
      return "DANIILIDIS";
    default:
      return "PARK";
  }
}

HandeyeSetupMode HandeyeSetupModeFromYamlString(const QString & value)
{
  const QString v = value.trimmed().toLower();
  if (v == "eye_to_hand" || v == "eye-to-hand" || v == "hand_to_eye" || v.contains("手外")) {
    return HandeyeSetupMode::EyeToHand;
  }
  return HandeyeSetupMode::EyeInHand;
}

QString HandeyeThirdFrameYamlKey(HandeyeSetupMode mode)
{
  return mode == HandeyeSetupMode::EyeInHand ? "camera_frame" : "object_frame";
}

QString HandeyeThirdFrameFieldLabel(HandeyeSetupMode mode)
{
  return mode == HandeyeSetupMode::EyeInHand ? QStringLiteral("camera_frame:") : QStringLiteral("object_frame (标定目标):");
}

bool RunOfflineHandeyeCalibrationForSingleAruco(
  const QString & csv_path,
  HandeyeSetupMode setup_mode,
  HandeyeSolverMethod solver_method,
  double marker_length_m,
  int target_marker_id,
  const QString & output_yaml,
  QString * summary,
  QString * detail_log,
  QString * err_msg)
{
  std::vector<CsvRow> rows;
  if (!ParseCsvRows(csv_path, &rows, err_msg)) {
    return false;
  }
  const QFileInfo csv_info(csv_path);
  const QDir csv_dir = csv_info.dir();
  QString intr_file_name;
  for (const auto & r : rows) {
    if (!r.intrinsics_file.isEmpty()) {
      intr_file_name = r.intrinsics_file;
      break;
    }
  }
  if (intr_file_name.isEmpty()) {
    intr_file_name = "camera_intrinsics_used.yaml";
  }
  const QString intr_path = csv_dir.filePath(intr_file_name);
  cv::Mat K, D;
  if (!LoadIntrinsics(intr_path, &K, &D)) {
    if (err_msg) *err_msg = QString("读取内参失败: %1").arg(intr_path);
    return false;
  }

  const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  cv::aruco::DetectorParameters params;
  cv::aruco::ArucoDetector detector(dict, params);
  const float kMarkerLengthM = static_cast<float>(marker_length_m > 0.0 ? marker_length_m : 0.083333);

  std::vector<cv::Mat> R_gripper2base;
  std::vector<cv::Mat> t_gripper2base;
  std::vector<cv::Mat> R_target2cam;
  std::vector<cv::Mat> t_target2cam;
  std::vector<cv::Matx33d> used_R_base_gripper;
  std::vector<cv::Vec3d> used_t_base_gripper;
  std::vector<cv::Matx33d> used_R_target_cam;
  std::vector<cv::Vec3d> used_t_target_cam;
  std::vector<double> per_point_corner_reprojection_error_px;
  QStringList detail_lines;
  int detected = 0;
  int image_read_ok = 0;
  for (const auto & row : rows) {
    const QString image_path = csv_dir.filePath(row.raw_image);
    cv::Mat img = cv::imread(image_path.toStdString(), cv::IMREAD_COLOR);
    if (img.empty()) {
      detail_lines << QString("sample %1: image_load_fail path=%2").arg(row.raw_image).arg(image_path);
      continue;
    }
    ++image_read_ok;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector.detectMarkers(img, corners, ids);
    if (ids.empty() || corners.empty()) {
      detail_lines << QString("sample %1: aruco_not_found").arg(row.raw_image);
      continue;
    }
    int idx = -1;
    for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
      if (ids[i] == target_marker_id) {
        idx = i;
        break;
      }
    }
    if (idx < 0) {
      detail_lines << QString("sample %1: target_id_not_found target=%2 detected=%3")
        .arg(row.raw_image)
        .arg(target_marker_id)
        .arg(ids.empty() ? QString("none") : QString::number(ids.front()));
      continue;
    }
    std::vector<std::vector<cv::Point2f>> one_corner{corners[idx]};
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(one_corner, kMarkerLengthM, K, D, rvecs, tvecs);
    if (rvecs.empty() || tvecs.empty()) {
      detail_lines << QString("sample %1: pose_estimate_fail marker_id=%2").arg(row.raw_image).arg(ids[idx]);
      continue;
    }
    std::vector<cv::Point3f> marker_points = {
      cv::Point3f(-kMarkerLengthM * 0.5f, kMarkerLengthM * 0.5f, 0.0f),
      cv::Point3f(kMarkerLengthM * 0.5f, kMarkerLengthM * 0.5f, 0.0f),
      cv::Point3f(kMarkerLengthM * 0.5f, -kMarkerLengthM * 0.5f, 0.0f),
      cv::Point3f(-kMarkerLengthM * 0.5f, -kMarkerLengthM * 0.5f, 0.0f)
    };
    std::vector<cv::Point2f> proj;
    cv::projectPoints(marker_points, rvecs[0], tvecs[0], K, D, proj);
    double sum2 = 0.0;
    for (int c = 0; c < 4; ++c) {
      const cv::Point2f d = corners[idx][c] - proj[c];
      sum2 += d.dot(d);
    }
    per_point_corner_reprojection_error_px.push_back(std::sqrt(sum2 / 4.0));
    detail_lines << QString("sample %1: ok marker_id=%2 t_target_cam=(%3,%4,%5)")
      .arg(row.raw_image)
      .arg(ids[idx])
      .arg(tvecs[0][0], 0, 'f', 6)
      .arg(tvecs[0][1], 0, 'f', 6)
      .arg(tvecs[0][2], 0, 'f', 6);

    cv::Mat R_bg = cv::Mat(QuatToR(row.q_base_gripper_xyzw));
    cv::Mat t_bg = (cv::Mat_<double>(3,1) << row.t_base_gripper[0], row.t_base_gripper[1], row.t_base_gripper[2]);
    cv::Mat R_b_g = R_bg.t();
    cv::Mat t_b_g = -R_b_g * t_bg;

    cv::Mat R_tc;
    cv::Rodrigues(rvecs[0], R_tc);
    cv::Mat t_tc = (cv::Mat_<double>(3,1) << tvecs[0][0], tvecs[0][1], tvecs[0][2]);

    if (setup_mode == HandeyeSetupMode::EyeInHand) {
      // OpenCV eye-in-hand expects gripper->base.
      R_gripper2base.push_back(R_bg);
      t_gripper2base.push_back(t_bg);
    } else {
      // OpenCV eye-to-hand in this project uses base->gripper to solve cam->base directly.
      R_gripper2base.push_back(R_b_g);
      t_gripper2base.push_back(t_b_g);
    }
    R_target2cam.push_back(R_tc);
    t_target2cam.push_back(t_tc);
    used_R_base_gripper.push_back(cv::Matx33d(R_bg));
    used_t_base_gripper.push_back(row.t_base_gripper);
    used_R_target_cam.push_back(cv::Matx33d(R_tc));
    used_t_target_cam.push_back(cv::Vec3d(t_tc.at<double>(0, 0), t_tc.at<double>(1, 0), t_tc.at<double>(2, 0)));
    ++detected;
  }
  if (detected < 4) {
    if (image_read_ok == 0) {
      if (err_msg) *err_msg = QString("未找到可读取的原始图像（CSV 引用图像与 CSV 同目录）。");
      return false;
    }
    if (err_msg) *err_msg = QString("有效样本不足，至少需要 4 组，当前 %1 组（图像可读 %2 组）")
      .arg(detected)
      .arg(image_read_ok);
    return false;
  }
  cv::HandEyeCalibrationMethod cv_method = cv::CALIB_HAND_EYE_PARK;
  switch (solver_method) {
    case HandeyeSolverMethod::Tsai:
      cv_method = cv::CALIB_HAND_EYE_TSAI;
      break;
    case HandeyeSolverMethod::Park:
      cv_method = cv::CALIB_HAND_EYE_PARK;
      break;
    case HandeyeSolverMethod::Horaud:
      cv_method = cv::CALIB_HAND_EYE_HORAUD;
      break;
    case HandeyeSolverMethod::Andreff:
      cv_method = cv::CALIB_HAND_EYE_ANDREFF;
      break;
    case HandeyeSolverMethod::Daniilidis:
      cv_method = cv::CALIB_HAND_EYE_DANIILIDIS;
      break;
    default:
      break;
  }
  cv::Mat R_cam2x, t_cam2x;
  cv::calibrateHandEye(
    R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
    R_cam2x, t_cam2x, cv_method);

  cv::Matx33d R;
  cv::Vec3d t;
  for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) R(r, c) = R_cam2x.at<double>(r, c);
  t[0] = t_cam2x.at<double>(0, 0);
  t[1] = t_cam2x.at<double>(1, 0);
  t[2] = t_cam2x.at<double>(2, 0);
  cv::Matx33d R_t_cam_base;
  cv::Vec3d t_t_cam_base;
  if (setup_mode == HandeyeSetupMode::EyeToHand) {
    // Eye-to-hand: solved output is cam->base directly.
    R_t_cam_base = R;
    t_t_cam_base = t;
  } else {
    // Eye-in-hand: solved output is cam->gripper, convert to cam->base using first sample pose.
    const cv::Matx33d R_g_b = used_R_base_gripper.front();
    const cv::Vec3d t_g_b = used_t_base_gripper.front();
    R_t_cam_base = R_g_b * R;
    t_t_cam_base = R_g_b * t + t_g_b;
  }
  const cv::Matx33d R_t_base_cam = R_t_cam_base.t();
  const cv::Vec3d t_t_base_cam = -(R_t_base_cam * t_t_cam_base);
  const cv::Matx33d R_inv = R_t_base_cam;
  const cv::Vec3d t_inv = t_t_base_cam;

  const double mean_corner_reprojection_error_px = per_point_corner_reprojection_error_px.empty()
    ? 0.0
    : std::accumulate(
    per_point_corner_reprojection_error_px.begin(),
    per_point_corner_reprojection_error_px.end(), 0.0) /
    static_cast<double>(per_point_corner_reprojection_error_px.size());

  std::vector<double> per_point_handeye_chain_translation_residual_mm;
  std::vector<double> per_point_handeye_chain_rotation_residual_deg;
  cv::Matx33d anchor_R = cv::Matx33d::eye();
  cv::Vec3d anchor_t(0.0, 0.0, 0.0);
  if (!used_R_base_gripper.empty()) {
    if (setup_mode == HandeyeSetupMode::EyeInHand) {
      // anchor = T_target_base
      anchor_R = used_R_base_gripper[0] * R_inv * used_R_target_cam[0];
      anchor_t = used_R_base_gripper[0] * (R_inv * used_t_target_cam[0] + t_inv) + used_t_base_gripper[0];
    } else {
      // anchor = T_target_gripper
      const cv::Matx33d R_g_b0 = used_R_base_gripper[0];
      const cv::Vec3d t_g_b0 = used_t_base_gripper[0];
      const cv::Matx33d R_b_g0 = R_g_b0.t();
      const cv::Vec3d t_b_g0 = -(R_b_g0 * t_g_b0);
      anchor_R = R_b_g0 * R_t_cam_base * used_R_target_cam[0];
      anchor_t = R_b_g0 * (R_t_cam_base * used_t_target_cam[0] + t_t_cam_base) + t_b_g0;
    }
  }
  for (size_t i = 0; i < used_R_base_gripper.size(); ++i) {
    cv::Matx33d R_pred;
    cv::Vec3d t_pred;
    if (setup_mode == HandeyeSetupMode::EyeInHand) {
      // T_t_c_pred = inv(T_g_b * T_c_g) * anchor
      const cv::Matx33d R_c_b = used_R_base_gripper[i] * R_inv;
      const cv::Vec3d t_c_b = used_R_base_gripper[i] * t_inv + used_t_base_gripper[i];
      const cv::Matx33d R_b_c = R_c_b.t();
      const cv::Vec3d t_b_c = -(R_b_c * t_c_b);
      R_pred = R_b_c * anchor_R;
      t_pred = R_b_c * anchor_t + t_b_c;
    } else {
      // T_t_c_pred = inv(T_c_b) * T_g_b * anchor
      R_pred = R_t_base_cam * used_R_base_gripper[i] * anchor_R;
      t_pred = R_t_base_cam * (used_R_base_gripper[i] * anchor_t + used_t_base_gripper[i]) + t_t_base_cam;
    }
    per_point_handeye_chain_translation_residual_mm.push_back(
      cv::norm(t_pred - used_t_target_cam[i]) * 1000.0);
    per_point_handeye_chain_rotation_residual_deg.push_back(
      RotationResidualDeg(R_pred, used_R_target_cam[i]));
  }
  const double mean_handeye_chain_translation_residual_mm =
    std::accumulate(
    per_point_handeye_chain_translation_residual_mm.begin(),
    per_point_handeye_chain_translation_residual_mm.end(), 0.0) /
    static_cast<double>(per_point_handeye_chain_translation_residual_mm.size());
  const double mean_handeye_chain_rotation_residual_deg =
    std::accumulate(
    per_point_handeye_chain_rotation_residual_deg.begin(),
    per_point_handeye_chain_rotation_residual_deg.end(), 0.0) /
    static_cast<double>(per_point_handeye_chain_rotation_residual_deg.size());

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "arm_id" << YAML::Value << 0;
  out << YAML::Key << "handeye_solver" << YAML::Value << HandeyeSolverMethodToYamlString(solver_method).toStdString();
  out << YAML::Key << "target_marker_id" << YAML::Value << target_marker_id;
  out << YAML::Key << "sample_count" << YAML::Value << detected;
  out << YAML::Key << "mean_corner_reprojection_error_px" << YAML::Value << mean_corner_reprojection_error_px;
  out << YAML::Key << "per_point_corner_reprojection_error_px" << YAML::Value << YAML::BeginSeq;
  for (double v : per_point_corner_reprojection_error_px) out << v;
  out << YAML::EndSeq;
  out << YAML::Key << "mean_handeye_chain_translation_residual_mm" << YAML::Value
      << mean_handeye_chain_translation_residual_mm;
  out << YAML::Key << "per_point_handeye_chain_translation_residual_mm" << YAML::Value << YAML::BeginSeq;
  for (double v : per_point_handeye_chain_translation_residual_mm) out << v;
  out << YAML::EndSeq;
  out << YAML::Key << "mean_handeye_chain_rotation_residual_deg" << YAML::Value
      << mean_handeye_chain_rotation_residual_deg;
  out << YAML::Key << "per_point_handeye_chain_rotation_residual_deg" << YAML::Value << YAML::BeginSeq;
  for (double v : per_point_handeye_chain_rotation_residual_deg) out << v;
  out << YAML::EndSeq;
  out << YAML::Key << "T_cam_base" << YAML::Value;
  AppendMatrix4x4(out, R_t_cam_base, t_t_cam_base);
  out << YAML::Key << "T_base_cam" << YAML::Value;
  AppendMatrix4x4(out, R_t_base_cam, t_t_base_cam);
  out << YAML::EndMap;

  std::ofstream fout(output_yaml.toStdString());
  if (!fout.is_open()) {
    if (err_msg) *err_msg = "无法写入输出文件";
    return false;
  }
  fout << out.c_str();
  if (summary) {
    *summary = QString("离线手眼完成：sample_count=%1，输出=%2").arg(detected).arg(output_yaml);
  }
  if (detail_log) {
    QStringList m;
    m << detail_lines;
    m << QString("arm_id: 0");
    m << QString("handeye_solver: %1").arg(HandeyeSolverMethodToYamlString(solver_method));
    m << QString("target_marker_id: %1").arg(target_marker_id);
    m << QString("sample_count: %1").arg(detected);
    m << QString("mean_corner_reprojection_error_px: %1").arg(mean_corner_reprojection_error_px, 0, 'g', 17);
    m << QString("mean_handeye_chain_translation_residual_mm: %1")
      .arg(mean_handeye_chain_translation_residual_mm, 0, 'g', 17);
    m << QString("mean_handeye_chain_rotation_residual_deg: %1")
      .arg(mean_handeye_chain_rotation_residual_deg, 0, 'g', 17);
    m << QString("final: t=(%1,%2,%3)")
      .arg(t_t_cam_base[0], 0, 'f', 6).arg(t_t_cam_base[1], 0, 'f', 6).arg(t_t_cam_base[2], 0, 'f', 6);
    m << QString("final: R=[[%1,%2,%3],[%4,%5,%6],[%7,%8,%9]]")
      .arg(R_t_cam_base(0, 0), 0, 'f', 6).arg(R_t_cam_base(0, 1), 0, 'f', 6).arg(R_t_cam_base(0, 2), 0, 'f', 6)
      .arg(R_t_cam_base(1, 0), 0, 'f', 6).arg(R_t_cam_base(1, 1), 0, 'f', 6).arg(R_t_cam_base(1, 2), 0, 'f', 6)
      .arg(R_t_cam_base(2, 0), 0, 'f', 6).arg(R_t_cam_base(2, 1), 0, 'f', 6).arg(R_t_cam_base(2, 2), 0, 'f', 6);
    m << QString("T_cam_base: [%1, %2, %3, %4, %5, %6, %7, %8, %9, %10, %11, %12, 0, 0, 0, 1]")
      .arg(R_t_cam_base(0, 0), 0, 'g', 17).arg(R_t_cam_base(0, 1), 0, 'g', 17)
      .arg(R_t_cam_base(0, 2), 0, 'g', 17).arg(t_t_cam_base[0], 0, 'g', 17)
      .arg(R_t_cam_base(1, 0), 0, 'g', 17).arg(R_t_cam_base(1, 1), 0, 'g', 17)
      .arg(R_t_cam_base(1, 2), 0, 'g', 17).arg(t_t_cam_base[1], 0, 'g', 17)
      .arg(R_t_cam_base(2, 0), 0, 'g', 17).arg(R_t_cam_base(2, 1), 0, 'g', 17)
      .arg(R_t_cam_base(2, 2), 0, 'g', 17).arg(t_t_cam_base[2], 0, 'g', 17);
    m << QString("T_base_cam: [%1, %2, %3, %4, %5, %6, %7, %8, %9, %10, %11, %12, 0, 0, 0, 1]")
      .arg(R_t_base_cam(0, 0), 0, 'g', 17).arg(R_t_base_cam(0, 1), 0, 'g', 17)
      .arg(R_t_base_cam(0, 2), 0, 'g', 17).arg(t_t_base_cam[0], 0, 'g', 17)
      .arg(R_t_base_cam(1, 0), 0, 'g', 17).arg(R_t_base_cam(1, 1), 0, 'g', 17)
      .arg(R_t_base_cam(1, 2), 0, 'g', 17).arg(t_t_base_cam[1], 0, 'g', 17)
      .arg(R_t_base_cam(2, 0), 0, 'g', 17).arg(R_t_base_cam(2, 1), 0, 'g', 17)
      .arg(R_t_base_cam(2, 2), 0, 'g', 17).arg(t_t_base_cam[2], 0, 'g', 17);
    *detail_log = m.join('\n');
  }
  return true;
}

bool DetectBoardTypeFromImage(
  const QString & image_path,
  QString * board_type,
  int * marker_id,
  QString * err_msg)
{
  if (!board_type || !marker_id) {
    if (err_msg) *err_msg = "输出参数为空";
    return false;
  }
  *board_type = QString();
  *marker_id = -1;
  cv::Mat gray = cv::imread(image_path.toStdString(), cv::IMREAD_GRAYSCALE);
  if (gray.empty()) {
    if (err_msg) *err_msg = QString("无法读取图像: %1").arg(image_path);
    return false;
  }

  const std::vector<int> dict_ids = {
    cv::aruco::DICT_4X4_50, cv::aruco::DICT_4X4_100, cv::aruco::DICT_4X4_250, cv::aruco::DICT_4X4_1000,
    cv::aruco::DICT_5X5_50, cv::aruco::DICT_5X5_100, cv::aruco::DICT_5X5_250, cv::aruco::DICT_5X5_1000,
    cv::aruco::DICT_6X6_50, cv::aruco::DICT_6X6_100, cv::aruco::DICT_6X6_250, cv::aruco::DICT_6X6_1000,
    cv::aruco::DICT_7X7_50, cv::aruco::DICT_7X7_100, cv::aruco::DICT_7X7_250, cv::aruco::DICT_7X7_1000,
    cv::aruco::DICT_ARUCO_ORIGINAL
  };
  int best_count = 0;
  std::vector<int> best_ids;
  for (int did : dict_ids) {
    auto dict = cv::aruco::getPredefinedDictionary(did);
    cv::aruco::DetectorParameters params;
    cv::aruco::ArucoDetector detector(dict, params);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector.detectMarkers(gray, corners, ids);
    if (static_cast<int>(ids.size()) > best_count) {
      best_count = static_cast<int>(ids.size());
      best_ids = ids;
    }
  }

  std::vector<cv::Point2f> corners;
  bool chess_found = false;
  for (const cv::Size & sz : {cv::Size(7, 10), cv::Size(10, 7), cv::Size(6, 9), cv::Size(9, 6), cv::Size(8, 11), cv::Size(11, 8)}) {
    if (cv::findChessboardCorners(gray, sz, corners)) {
      chess_found = true;
      break;
    }
  }

  if (best_count == 1) {
    *board_type = "Aruco Single Marker";
    *marker_id = best_ids.front();
    return true;
  }
  if (best_count >= 2 && chess_found) {
    *board_type = "Charuco";
    return true;
  }
  if (best_count >= 2) {
    *board_type = "Aruco GridBoard";
    return true;
  }
  if (chess_found) {
    *board_type = "Chessboard";
    return true;
  }

  if (err_msg) *err_msg = "未能识别标定板类型，请确认图像清晰且完整包含标定板";
  return false;
}

}  // namespace ros_robot_workbench::ui
