#pragma once

#include <string>

namespace hs_calib {
namespace core {

/// \brief 从简易 YAML 读取棋盘参数（squares_x/y、square_length）
struct BoardConfigYaml {
  bool valid = false;
  int squares_x = 9;
  int squares_y = 6;
  double square_length = 0.025;
  std::string message;
};

/// \brief 从简易 YAML 文件加载棋盘格参数
bool load_board_config_yaml(const std::string &path, BoardConfigYaml *out);

/// \brief 通过 ament 查找 share/hs_calib_suite/config/<name>
std::string resolve_package_config(const std::string &filename);

}  // namespace core
}  // namespace hs_calib
