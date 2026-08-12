// Nova Robot Tools UI Node Entry Point
// 参考 calib_sim 架构

#include "ros_robot_assist_tools/ros_robot_assist_tools_ui.hpp"
#include "ros_robot_assist_tools/preferences/app_preferences.hpp"
#include "rclcpp/rclcpp.hpp"
#include <csignal>
#include <QApplication>

// 全局标志
static bool g_shutdown_requested = false;

// SIGINT 处理函数
void signalHandler(int /*signal*/) {
  g_shutdown_requested = true;
  QCoreApplication::quit();  // 退出 Qt 事件循环
}

int main(int argc, char ** argv) {
  ros_robot_assist_tools::ApplyRosEnvironmentFromSavedPreferences();
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  
  // 注册信号处理
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  
  // 运行 Qt UI 应用
  int result = ros_robot_assist_tools::RunRosRobotAssistToolsUiApp(argc, argv);
  
  // 关闭 ROS2
  rclcpp::shutdown();
  
  return result;
}
