// MIT License
//
// Copyright (c) 2024 Miguel Ángel González Santamarta
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <iostream>
#include <vector>
#include <string>

#include "audio_common/tts_node.hpp"

int main(int argc, char *argv[]) {
  // 构造新的命令行参数，包含配置文件路径
  std::vector<std::string> new_argv_strings;
  std::vector<char*> new_argv;
  
  // 复制原始参数
  for (int i = 0; i < argc; i++) {
    new_argv_strings.push_back(std::string(argv[i]));
  }
  
  // 尝试自动加载默认配置文件（仅在未提供参数文件时）
  bool has_params_file = false;
  for (int i = 0; i < argc - 1; i++) {
    if (std::string(argv[i]) == "--params-file") {
      has_params_file = true;
      break;
    }
  }
  
  if (!has_params_file) {
    try {
      // 获取包共享目录
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("audio_common");
      std::filesystem::path config_path = std::filesystem::path(package_share_directory) / "config" / "device_config.yaml";
      
      // 检查配置文件是否存在
      if (std::filesystem::exists(config_path)) {
        RCLCPP_INFO(rclcpp::get_logger("tts_main"), "找到默认配置文件: %s", config_path.string().c_str());
        
        // 添加参数文件选项
        new_argv_strings.push_back("--ros-args");
        new_argv_strings.push_back("--params-file");
        new_argv_strings.push_back(config_path.string());
      } else {
        RCLCPP_WARN(rclcpp::get_logger("tts_main"), "未找到默认配置文件: %s", config_path.string().c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("tts_main"), "加载默认配置文件时出错: %s", e.what());
    }
  }
  
  // 转换为char*数组
  new_argv.reserve(new_argv_strings.size() + 1);
  for (const auto& arg : new_argv_strings) {
    new_argv.push_back(const_cast<char*>(arg.c_str()));
  }
  new_argv.push_back(nullptr); // 确保以nullptr结尾
  
  // 初始化ROS 2
  rclcpp::init(new_argv.size() - 1, new_argv.data());
  
  auto node = std::make_shared<audio_common::TtsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}