#pragma once

#include <map>
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

/// \brief 扁平 key:value YAML → 字符串字典（忽略注释、列表项、多行块）
bool load_simple_yaml_map(
    const std::string &path, std::map<std::string, std::string> *out, std::string *error_out = nullptr);

/// \brief 标定器 ID → 包内 config/ 默认文件名
std::string default_config_filename(const std::string &calibrator_id);

/// \brief 通过 ament 查找 share/hs_calib_suite/config/<name>
std::string resolve_package_config(const std::string &filename);

}  // namespace core
}  // namespace hs_calib
