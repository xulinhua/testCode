#ifndef UDP_CLIENT__PROJECT_MANAGER_HPP_
#define UDP_CLIENT__PROJECT_MANAGER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs_comm/msg/project_status.hpp"
#include "log_system/log_macros.hpp"

namespace udp_comm_client
{

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
  int process_id{-1};
};

struct ProjectConfig {
  std::string start_cmd;
  std::string stop_cmd;
  std::string check_cmd;
  std::string working_dir;
  std::string setup_script;
  std::string description;
};

class ProjectManager
{
public:
  ProjectManager(rclcpp::Logger logger, rclcpp::Node* node);
  ~ProjectManager();

  // 项目控制接口
  std::pair<bool, std::string> start_project(const std::string& project_name);
  std::pair<bool, std::string> stop_project(const std::string& project_name);
  std::tuple<bool, std::string, std::string> get_status(const std::string& project_name);
  std::vector<std::string> list_projects();
  bool is_project_running(const std::string& project_name);
  bool is_project_process_running(const std::string& project_name);  // 添加这个方法声明
  
  // 设置相机类型
  void set_camera_type(const std::string& camera_type);
  
  // 停止所有项目
  void stop_all_projects();

private:
  rclcpp::Logger logger_;
  rclcpp::Node* node_;
  std::string camera_type_;
  
  // 项目管理
  std::unordered_map<std::string, ProjectInfo> projects_;
  std::mutex projects_mutex_;
  
  // 项目配置
  std::unordered_map<std::string, ProjectConfig> projects_config_;
  
  // 项目线程
  std::unordered_map<std::string, std::thread> project_threads_;
  std::mutex threads_mutex_;

  // 项目执行函数
  void run_project(const std::string& project_name, std::atomic<bool>& should_stop);
  
  // 工具函数
  void wait_for_project_stop(const std::string& project_name, int timeout_seconds = 30);
  
  // 状态更新
  void update_project_status(const std::string& project_name, ProjectStatus status, 
                            const std::string& message = "");
  
  // 项目检查
  bool validate_project_name(const std::string& project_name);
  void update_camera_commands();
};

}  // namespace udp_comm_client

#endif  // UDP_CLIENT__PROJECT_MANAGER_HPP_