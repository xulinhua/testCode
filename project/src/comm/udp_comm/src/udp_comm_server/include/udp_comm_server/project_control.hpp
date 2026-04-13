#ifndef UDP_SERVER__PROJECT_CONTROL_HPP_
#define UDP_SERVER__PROJECT_CONTROL_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs_comm/msg/project_command.hpp"
#include "custom_msgs_comm/msg/project_status.hpp"
#include "log_system/log_macros.hpp"

namespace udp_comm_server
{

// 项目名称定义 - 只需定义一处
static constexpr const char* PROJECT_NAME = "udp_comm_server";

enum class ProjectStatus : uint8_t {
  STATUS_STOPPED = 0,
  STATUS_RUNNING = 1,
  STATUS_STARTING = 2,
  STATUS_STOPPING = 3,
  STATUS_ERROR = 4
};

struct ProjectInfo {
  std::string name;
  ProjectStatus status;
  std::string description;
  std::string message;
  std::chrono::steady_clock::time_point start_time;
  std::thread::id thread_id;
  std::unique_ptr<std::atomic<bool>> should_stop;
  
  ProjectInfo() : should_stop(std::make_unique<std::atomic<bool>>(false)) {}
};

class ProjectControl
{
public:
  ProjectControl(rclcpp::Logger logger, rclcpp::Node* node);
  ~ProjectControl();

  // 项目控制接口
  std::pair<ProjectStatus, std::string> start_project(const std::string& project_name);
  std::pair<ProjectStatus, std::string> stop_project(const std::string& project_name);
  std::tuple<ProjectStatus, std::string, std::string> get_status(const std::string& project_name);
  std::vector<std::string> list_projects();
  
  // 设置相机类型
  void set_camera_type(const std::string& camera_type);

private:
  rclcpp::Logger logger_;
  rclcpp::Node* node_;
  std::string camera_type_;
  
  // 项目管理
  std::unordered_map<std::string, ProjectInfo> projects_;
  std::mutex projects_mutex_;
  
  // 项目线程
  std::unordered_map<std::string, std::thread> project_threads_;
  std::mutex threads_mutex_;

  // 项目执行函数
  void run_camera_project(const std::string& project_name, std::atomic<bool>& should_stop);
  void run_pcl2laser_project(const std::string& project_name, std::atomic<bool>& should_stop);
  void run_detection_project(const std::string& project_name, std::atomic<bool>& should_stop);
  
  // 工具函数
  bool is_project_running(const std::string& project_name);
  void stop_all_projects();
  void wait_for_project_stop(const std::string& project_name, int timeout_seconds = 30);
  
  // 状态更新
  void update_project_status(const std::string& project_name, ProjectStatus status, 
                            const std::string& message = "");
  
  // 项目检查
  bool validate_project_name(const std::string& project_name);
  std::string get_project_executable(const std::string& project_name);
};

}  // namespace udp_comm_server

#endif  // UDP_SERVER__PROJECT_CONTROL_HPP_