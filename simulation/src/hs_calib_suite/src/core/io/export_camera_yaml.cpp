#include "hs_calib_suite/core/io/export_camera_yaml.hpp"

#include <fstream>
#include <sstream>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace hs_calib {
namespace core {

namespace {

/// \brief 从标定结果 intrinsics_meta 读取字符串字段
std::string meta(const CalibrationResult &r, const char *key, const char *def = "0") {
  const auto it = r.intrinsics_meta.find(key);
  return it == r.intrinsics_meta.end() ? std::string(def) : it->second;
}

/// \brief 从 intrinsics_meta 读取 double 字段
double meta_d(const CalibrationResult &r, const char *key, double def = 0.0) {
  try {
    return std::stod(meta(r, key, "nan"));
  } catch (...) {
    return def;
  }
}

/// \brief 去除行首尾空白
std::string trim(std::string s) {
  while (!s.empty() && (s.back() == '\r' || s.back() == ' ' || s.back() == '\t')) {
    s.pop_back();
  }
  size_t i = 0;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) {
    ++i;
  }
  return s.substr(i);
}

/// \brief 解析 YAML 行内 [a, b, c, ...] 数值列表
bool parse_bracket_list(const std::string &line, std::vector<double> *out) {
  const auto lb = line.find('[');
  const auto rb = line.rfind(']');
  if (lb == std::string::npos || rb == std::string::npos || rb <= lb) {
    return false;
  }
  std::stringstream ss(line.substr(lb + 1, rb - lb - 1));
  std::string tok;
  out->clear();
  while (std::getline(ss, tok, ',')) {
    tok = trim(tok);
    if (tok.empty()) {
      continue;
    }
    out->push_back(std::stod(tok));
  }
  return !out->empty();
}

}  // namespace

/// \brief OpenCV rvec/tvec 转为 4×4 齐次变换矩阵
Eigen::Matrix4d cv_rt_to_matrix4d(const cv::Mat &rvec, const cv::Mat &tvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      T(r, c) = R.at<double>(r, c);
    }
    T(r, 3) = tvec.at<double>(r, 0);
  }
  return T;
}

/// \brief 4×4 齐次变换矩阵拆分为 OpenCV R/t
void matrix4d_to_cv_rt(const Eigen::Matrix4d &T, cv::Mat *R, cv::Mat *t) {
  *R = cv::Mat(3, 3, CV_64F);
  *t = cv::Mat(3, 1, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R->at<double>(r, c) = T(r, c);
    }
    t->at<double>(r, 0) = T(r, 3);
  }
}

/// \brief 从 CalibrationResult 填充 CameraIntrinsics 结构
bool camera_intrinsics_from_result(const CalibrationResult &result, CameraIntrinsics *out) {
  if (out == nullptr || !result.success) {
    return false;
  }
  out->K = cv::Mat::eye(3, 3, CV_64F);
  out->D = cv::Mat::zeros(5, 1, CV_64F);
  out->K.at<double>(0, 0) = meta_d(result, "fx");
  out->K.at<double>(1, 1) = meta_d(result, "fy");
  out->K.at<double>(0, 2) = meta_d(result, "cx");
  out->K.at<double>(1, 2) = meta_d(result, "cy");
  out->D.at<double>(0, 0) = meta_d(result, "k1");
  out->D.at<double>(1, 0) = meta_d(result, "k2");
  out->D.at<double>(2, 0) = meta_d(result, "p1");
  out->D.at<double>(3, 0) = meta_d(result, "p2");
  out->D.at<double>(4, 0) = meta_d(result, "k3");
  out->image_width = static_cast<int>(meta_d(result, "image_width"));
  out->image_height = static_cast<int>(meta_d(result, "image_height"));
  out->valid = out->K.at<double>(0, 0) > 0.0;
  out->message = out->valid ? "ok" : "invalid fx";
  return out->valid;
}

/// \brief 从本工程导出的 YAML 格式加载相机内参
bool load_camera_yaml(const std::string &path, CameraIntrinsics *out) {
  if (out == nullptr) {
    return false;
  }
  *out = {};
  std::ifstream ifs(path);
  if (!ifs) {
    out->message = "cannot open " + path;
    return false;
  }

  // —— 逐行解析 image_width/height 与 K/D data ——
  int width = 0;
  int height = 0;
  std::vector<double> Kdata;
  std::vector<double> Ddata;
  std::string line;
  while (std::getline(ifs, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    if (line.rfind("image_width:", 0) == 0) {
      width = std::stoi(trim(line.substr(12)));
    } else if (line.rfind("image_height:", 0) == 0) {
      height = std::stoi(trim(line.substr(13)));
    } else if (line.find("data:") != std::string::npos && Kdata.empty()) {
      // first data: camera_matrix
      if (parse_bracket_list(line, &Kdata) && Kdata.size() == 9) {
        continue;
      }
      Kdata.clear();
    } else if (line.find("data:") != std::string::npos && Kdata.size() == 9 && Ddata.empty()) {
      parse_bracket_list(line, &Ddata);
    }
  }

  if (Kdata.size() != 9) {
    out->message = "camera_matrix data missing";
    return false;
  }
  out->K = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) {
    out->K.at<double>(i / 3, i % 3) = Kdata[static_cast<size_t>(i)];
  }
  out->D = cv::Mat::zeros(5, 1, CV_64F);
  for (size_t i = 0; i < Ddata.size() && i < 5; ++i) {
    out->D.at<double>(static_cast<int>(i), 0) = Ddata[i];
  }
  out->image_width = width;
  out->image_height = height;
  out->valid = true;
  out->message = "ok";
  return true;
}

/// \brief 将标定结果内参写入 YAML 文件
bool export_camera_yaml(const CalibrationResult &result, const std::string &path) {
  if (!result.success) {
    return false;
  }
  std::ofstream ofs(path);
  if (!ofs) {
    return false;
  }
  const std::string w = meta(result, "image_width", "0");
  const std::string h = meta(result, "image_height", "0");
  const std::string fx = meta(result, "fx");
  const std::string fy = meta(result, "fy");
  const std::string cx = meta(result, "cx");
  const std::string cy = meta(result, "cy");
  const std::string k1 = meta(result, "k1");
  const std::string k2 = meta(result, "k2");
  const std::string p1 = meta(result, "p1");
  const std::string p2 = meta(result, "p2");
  const std::string k3 = meta(result, "k3");
  const std::string rms = meta(result, "rms");

  ofs << "# hs_calib_suite camera intrinsics\n";
  ofs << "image_width: " << w << "\n";
  ofs << "image_height: " << h << "\n";
  ofs << "camera_model: plumb_bob\n";
  ofs << "reprojection_rmse: " << rms << "\n";
  ofs << "camera_matrix:\n";
  ofs << "  rows: 3\n";
  ofs << "  cols: 3\n";
  ofs << "  data: [" << fx << ", 0.0, " << cx << ", 0.0, " << fy << ", " << cy
      << ", 0.0, 0.0, 1.0]\n";
  ofs << "distortion_coefficients:\n";
  ofs << "  rows: 1\n";
  ofs << "  cols: 5\n";
  ofs << "  data: [" << k1 << ", " << k2 << ", " << p1 << ", " << p2 << ", " << k3
      << "]\n";
  return static_cast<bool>(ofs);
}

/// \brief 格式化内参标定结果为可读文本
std::string format_intrinsics_text(const CalibrationResult &result) {
  std::ostringstream oss;
  if (!result.success) {
    oss << "标定失败：" << result.message << "\n";
    return oss.str();
  }
  oss << "reprojection_rmse: " << meta(result, "rms") << "\n";
  if (result.metrics.count("num_views")) {
    oss << "num_views: " << static_cast<int>(result.metrics.at("num_views")) << "\n";
  }
  oss << "image_size: " << meta(result, "image_width") << " x "
      << meta(result, "image_height") << "\n";
  oss << "camera_matrix K:\n";
  oss << "  [" << meta(result, "fx") << ", 0, " << meta(result, "cx") << "]\n";
  oss << "  [0, " << meta(result, "fy") << ", " << meta(result, "cy") << "]\n";
  oss << "  [0, 0, 1]\n";
  oss << "distortion [k1,k2,p1,p2,k3]:\n";
  oss << "  [" << meta(result, "k1") << ", " << meta(result, "k2") << ", "
      << meta(result, "p1") << ", " << meta(result, "p2") << ", " << meta(result, "k3")
      << "]\n";
  oss << "message: " << result.message << "\n";
  return oss.str();
}

/// \brief 将手眼/外参 4×4 变换写入 YAML 文件
bool export_extrinsics_yaml(
    const CalibrationResult &result,
    const std::string &parent_frame,
    const std::string &child_frame,
    const std::string &path) {
  if (!result.success) {
    return false;
  }
  const auto pit = result.transforms.find(parent_frame);
  if (pit == result.transforms.end()) {
    return false;
  }
  const auto cit = pit->second.find(child_frame);
  if (cit == pit->second.end()) {
    return false;
  }
  const Eigen::Matrix4d &T = cit->second;
  std::ofstream ofs(path);
  if (!ofs) {
    return false;
  }
  ofs << "# hs_calib_suite extrinsics\n";
  ofs << "parent_frame: " << parent_frame << "\n";
  ofs << "child_frame: " << child_frame << "\n";
  if (result.metrics.count("handeye_rmse")) {
    ofs << "handeye_rmse: " << result.metrics.at("handeye_rmse") << "\n";
  }
  ofs << "transform:\n";
  ofs << "  rows: 4\n";
  ofs << "  cols: 4\n";
  ofs << "  data: [";
  for (int i = 0; i < 16; ++i) {
    if (i) {
      ofs << ", ";
    }
    ofs.precision(12);
    ofs << T(i / 4, i % 4);
  }
  ofs << "]\n";
  return static_cast<bool>(ofs);
}

/// \brief 格式化外参标定结果为可读文本
std::string format_extrinsics_text(
    const CalibrationResult &result,
    const std::string &parent_frame,
    const std::string &child_frame) {
  std::ostringstream oss;
  if (!result.success) {
    oss << "标定失败：" << result.message << "\n";
    return oss.str();
  }
  oss << "message: " << result.message << "\n";
  if (result.metrics.count("num_pairs")) {
    oss << "num_pairs: " << static_cast<int>(result.metrics.at("num_pairs")) << "\n";
  }
  if (result.metrics.count("handeye_rmse")) {
    oss << "handeye_rmse: " << result.metrics.at("handeye_rmse") << "\n";
  }
  const auto pit = result.transforms.find(parent_frame);
  if (pit == result.transforms.end() || !pit->second.count(child_frame)) {
    oss << "transform missing for " << parent_frame << " -> " << child_frame << "\n";
    return oss.str();
  }
  const Eigen::Matrix4d &T = pit->second.at(child_frame);
  oss << "T_" << parent_frame << "_" << child_frame << " (p_parent = T * p_child):\n";
  for (int r = 0; r < 4; ++r) {
    oss << "  [";
    for (int c = 0; c < 4; ++c) {
      if (c) {
        oss << ", ";
      }
      oss.precision(6);
      oss << std::fixed << T(r, c);
    }
    oss << "]\n";
  }
  return oss.str();
}

}  // namespace core
}  // namespace hs_calib
