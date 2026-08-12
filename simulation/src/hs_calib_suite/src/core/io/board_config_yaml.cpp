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

/// \brief 从简易 key:value YAML 读取棋盘格参数
bool load_board_config_yaml(const std::string &path, BoardConfigYaml *out) {
  if (out == nullptr) {
    return false;
  }
  *out = {};
  std::ifstream ifs(path);
  if (!ifs) {
    out->message = "cannot open " + path;
    return false;
  }
  out->squares_x = 9;
  out->squares_y = 6;
  out->square_length = 0.025;
  // —— 逐行解析 key:value ——
  std::string line;
  while (std::getline(ifs, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim(line.substr(0, colon));
    const std::string val = trim(line.substr(colon + 1));
    try {
      if (key == "squares_x") {
        out->squares_x = std::stoi(val);
      } else if (key == "squares_y") {
        out->squares_y = std::stoi(val);
      } else if (key == "square_length") {
        out->square_length = std::stod(val);
      }
    } catch (...) {
      // ignore bad line
    }
  }
  out->valid = out->squares_x >= 2 && out->squares_y >= 2 && out->square_length > 0.0;
  out->message = out->valid ? "ok" : "invalid board params";
  return out->valid;
}

}  // namespace core
}  // namespace hs_calib
