#include "hs_calib_suite/core/io/board_config_yaml.hpp"

#include <fstream>
#include <sstream>

#if __has_include(<ament_index_cpp/get_package_share_directory.hpp>)
#include <ament_index_cpp/get_package_share_directory.hpp>
#define HS_CALIB_HAS_AMENT_INDEX 1
#else
#define HS_CALIB_HAS_AMENT_INDEX 0
#endif

namespace hs_calib {
namespace core {

namespace {

/// \brief 去除行首尾空白与回车
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

/// \brief 去掉一层引号
std::string unquote(std::string v) {
  v = trim(std::move(v));
  if (v.size() >= 2 &&
      ((v.front() == '"' && v.back() == '"') || (v.front() == '\'' && v.back() == '\''))) {
    v = v.substr(1, v.size() - 2);
  }
  return v;
}

}  // namespace

/// \brief 通过 ament 查找包内 config 目录下的 YAML 文件路径
std::string resolve_package_config(const std::string &filename) {
#if HS_CALIB_HAS_AMENT_INDEX
  try {
    const std::string share =
        ament_index_cpp::get_package_share_directory("hs_calib_suite");
    return share + "/config/" + filename;
  } catch (...) {
    return {};
  }
#else
  (void)filename;
  return {};
#endif
}

std::string default_config_filename(const std::string &calibrator_id) {
  if (calibrator_id == "eye_in_hand") {
    return "eye_in_hand.yaml";
  }
  if (calibrator_id == "eye_to_hand") {
    return "eye_to_hand.yaml";
  }
  if (calibrator_id == "stereo_intrinsics") {
    return "stereo_intrinsics.yaml";
  }
  if (calibrator_id == "stereo_extrinsics") {
    return "stereo_extrinsics.yaml";
  }
  if (calibrator_id == "trihedral_oneshot") {
    return "trihedral_oneshot.yaml";
  }
  return "cam_intrinsics.yaml";
}

bool load_simple_yaml_map(
    const std::string &path, std::map<std::string, std::string> *out, std::string *error_out) {
  if (out == nullptr) {
    return false;
  }
  out->clear();
  std::ifstream ifs(path);
  if (!ifs) {
    if (error_out) {
      *error_out = "cannot open " + path;
    }
    return false;
  }
  std::string line;
  bool skip_block = false;
  while (std::getline(ifs, line)) {
    const std::string raw = line;
    std::string t = trim(line);
    if (t.empty() || t[0] == '#') {
      continue;
    }
    const bool indented = !raw.empty() && (raw[0] == ' ' || raw[0] == '\t');
    if (skip_block) {
      if (indented) {
        continue;
      }
      skip_block = false;
    }
    if (t[0] == '-') {
      continue;
    }
    const auto colon = t.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim(t.substr(0, colon));
    const std::string val = unquote(t.substr(colon + 1));
    if (key.empty()) {
      continue;
    }
    if (val == "|" || val == ">") {
      skip_block = true;
      continue;
    }
    if (val.empty()) {
      continue;
    }
    (*out)[key] = val;
  }
  return true;
}

/// \brief 从简易 key:value YAML 读取棋盘格参数
bool load_board_config_yaml(const std::string &path, BoardConfigYaml *out) {
  if (out == nullptr) {
    return false;
  }
  *out = {};
  std::map<std::string, std::string> kv;
  std::string err;
  if (!load_simple_yaml_map(path, &kv, &err)) {
    out->message = err.empty() ? ("cannot open " + path) : err;
    return false;
  }
  out->squares_x = 9;
  out->squares_y = 6;
  out->square_length = 0.025;
  try {
    if (kv.count("squares_x")) {
      out->squares_x = std::stoi(kv.at("squares_x"));
    }
    if (kv.count("squares_y")) {
      out->squares_y = std::stoi(kv.at("squares_y"));
    }
    if (kv.count("square_length")) {
      out->square_length = std::stod(kv.at("square_length"));
    }
  } catch (...) {
    // keep defaults for bad numbers
  }
  out->valid = out->squares_x >= 2 && out->squares_y >= 2 && out->square_length > 0.0;
  out->message = out->valid ? "ok" : "invalid board params";
  return out->valid;
}

}  // namespace core
}  // namespace hs_calib
