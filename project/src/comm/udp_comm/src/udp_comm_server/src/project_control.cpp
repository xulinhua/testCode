#include "udp_comm_server/project_control.hpp"
#include <chrono>
#include <thread>
#include <cstdlib>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <memory>

namespace udp_comm_server
{

ProjectControl::ProjectControl(rclcpp::Logger logger, rclcpp::Node* node)
: logger_(logger), node_(node), camera_type_("Gemini")
{
  RCLCPP_INFO(logger_, "ProjectControl initialized");
}

ProjectControl::~ProjectControl()
{
  stop_all_projects();
  RCLCPP_INFO(logger_, "ProjectControl destroyed");
}

std::pair<ProjectStatus, std::string> ProjectControl::start_project(const std::string& project_name)
{
  std::lock_guard<std::mutex> lock(projects_mutex_);
  
  if (!validate_project_name(project_name)) {
    return {ProjectStatus::STATUS_ERROR, "Invalid project name: " + project_name};
  }

  auto it = projects_.find(project_name);
  if (it != projects_.end() && it->second.status == ProjectStatus::STATUS_RUNNING) {
    return {ProjectStatus::STATUS_RUNNING, "Project is already running"};
  }

  // 更新项目状态为启动中
  update_project_status(project_name, ProjectStatus::STATUS_STARTING, "Starting project");

  // 创建项目信息
  ProjectInfo project_info;
  project_info.name = project_name;
  project_info.status = ProjectStatus::STATUS_STARTING;
  project_info.description = "Project starting up";
  project_info.message = "Starting project";
  project_info.start_time = std::chrono::steady_clock::now();
  project_info.should_stop = std::make_unique<std::atomic<bool>>(false);

  // 使用emplace避免复制
  projects_.emplace(project_name, std::move(project_info));

  // 启动项目线程
  std::thread project_thread;
  if (project_name == "camera") {
    project_thread = std::thread(&ProjectControl::run_camera_project, this, 
                               project_name, std::ref(*projects_[project_name].should_stop));
  } else if (project_name == "pcl2laser") {
    project_thread = std::thread(&ProjectControl::run_pcl2laser_project, this,
                               project_name, std::ref(*projects_[project_name].should_stop));
  } else if (project_name == "detection") {
    project_thread = std::thread(&ProjectControl::run_detection_project, this,
                               project_name, std::ref(*projects_[project_name].should_stop));
  } else {
    update_project_status(project_name, ProjectStatus::STATUS_ERROR, "Unknown project type");
    return {ProjectStatus::STATUS_ERROR, "Unknown project type: " + project_name};
  }

  // 存储线程ID
  projects_[project_name].thread_id = project_thread.get_id();
  
  {
    std::lock_guard<std::mutex> thread_lock(threads_mutex_);
    project_threads_[project_name] = std::move(project_thread);
  }

  RCLCPP_INFO(logger_, "Started project: %s", project_name.c_str());
  return {ProjectStatus::STATUS_RUNNING, "Project started successfully"};
}

std::pair<ProjectStatus, std::string> ProjectControl::stop_project(const std::string& project_name)
{
  std::lock_guard<std::mutex> lock(projects_mutex_);
  
  auto it = projects_.find(project_name);
  if (it == projects_.end()) {
    return {ProjectStatus::STATUS_STOPPED, "Project not found"};
  }

  if (it->second.status != ProjectStatus::STATUS_RUNNING) {
    return {it->second.status, "Project is not running"};
  }

  // 更新项目状态为停止中
  update_project_status(project_name, ProjectStatus::STATUS_STOPPING, "Stopping project");

  // 设置停止标志
  *it->second.should_stop = true;

  // 等待项目停止
  wait_for_project_stop(project_name, 30);

  // 检查是否成功停止
  it = projects_.find(project_name);
  if (it != projects_.end() && it->second.status == ProjectStatus::STATUS_STOPPED) {
    RCLCPP_INFO(logger_, "Stopped project: %s", project_name.c_str());
    return {ProjectStatus::STATUS_STOPPED, "Project stopped successfully"};
  } else {
    RCLCPP_ERROR(logger_, "Failed to stop project: %s", project_name.c_str());
    return {ProjectStatus::STATUS_ERROR, "Failed to stop project"};
  }
}

std::tuple<ProjectStatus, std::string, std::string> ProjectControl::get_status(const std::string& project_name)
{
  std::lock_guard<std::mutex> lock(projects_mutex_);
  
  auto it = projects_.find(project_name);
  if (it == projects_.end()) {
    return {ProjectStatus::STATUS_STOPPED, "Project not found", "Project is not registered"};
  }

  return {it->second.status, it->second.description, it->second.message};
}

std::vector<std::string> ProjectControl::list_projects()
{
  std::lock_guard<std::mutex> lock(projects_mutex_);
  std::vector<std::string> project_list;
  
  for (const auto& pair : projects_) {
    project_list.push_back(pair.first);
  }
  
  return project_list;
}

void ProjectControl::set_camera_type(const std::string& camera_type)
{
  camera_type_ = camera_type;
  RCLCPP_INFO(logger_, "Camera type set to: %s", camera_type.c_str());
}

void ProjectControl::run_camera_project(const std::string& project_name, std::atomic<bool>& should_stop)
{
  RCLCPP_INFO(logger_, "Starting camera project: %s with camera type: %s", 
              project_name.c_str(), camera_type_.c_str());

  update_project_status(project_name, ProjectStatus::STATUS_RUNNING, "Camera project running");

  // 模拟相机项目运行
  while (!should_stop && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 检查项目状态
    if (should_stop) {
      break;
    }
  }

  update_project_status(project_name, ProjectStatus::STATUS_STOPPED, "Camera project stopped");
  RCLCPP_INFO(logger_, "Camera project stopped: %s", project_name.c_str());
}

void ProjectControl::run_pcl2laser_project(const std::string& project_name, std::atomic<bool>& should_stop)
{
  RCLCPP_INFO(logger_, "Starting pcl2laser project: %s", project_name.c_str());

  update_project_status(project_name, ProjectStatus::STATUS_RUNNING, "PCL to laser project running");

  // 模拟点云转激光项目运行
  while (!should_stop && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 检查项目状态
    if (should_stop) {
      break;
    }
  }

  update_project_status(project_name, ProjectStatus::STATUS_STOPPED, "PCL to laser project stopped");
  RCLCPP_INFO(logger_, "PCL to laser project stopped: %s", project_name.c_str());
}

void ProjectControl::run_detection_project(const std::string& project_name, std::atomic<bool>& should_stop)
{
  RCLCPP_INFO(logger_, "Starting detection project: %s", project_name.c_str());

  update_project_status(project_name, ProjectStatus::STATUS_RUNNING, "Detection project running");

  // 模拟检测项目运行
  while (!should_stop && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 检查项目状态
    if (should_stop) {
      break;
    }
  }

  update_project_status(project_name, ProjectStatus::STATUS_STOPPED, "Detection project stopped");
  RCLCPP_INFO(logger_, "Detection project stopped: %s", project_name.c_str());
}

bool ProjectControl::is_project_running(const std::string& project_name)
{
  std::lock_guard<std::mutex> lock(projects_mutex_);
  auto it = projects_.find(project_name);
  return it != projects_.end() && it->second.status == ProjectStatus::STATUS_RUNNING;
}

void ProjectControl::stop_all_projects()
{
  std::lock_guard<std::mutex> lock(projects_mutex_);
  
  for (auto& pair : projects_) {
    if (pair.second.status == ProjectStatus::STATUS_RUNNING) {
      *pair.second.should_stop = true;
    }
  }

  // 等待所有项目停止
  for (auto& pair : projects_) {
    if (pair.second.status == ProjectStatus::STATUS_RUNNING) {
      wait_for_project_stop(pair.first, 10);
    }
  }

  // 等待所有线程结束
  {
    std::lock_guard<std::mutex> thread_lock(threads_mutex_);
    for (auto& pair : project_threads_) {
      if (pair.second.joinable()) {
        pair.second.join();
      }
    }
    project_threads_.clear();
  }

  RCLCPP_INFO(logger_, "All projects stopped");
}

void ProjectControl::wait_for_project_stop(const std::string& project_name, int timeout_seconds)
{
  auto start_time = std::chrono::steady_clock::now();
  
  while (is_project_running(project_name)) {
    if (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time).count() > timeout_seconds) {
      RCLCPP_ERROR(logger_, "Timeout waiting for project to stop: %s", project_name.c_str());
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void ProjectControl::update_project_status(const std::string& project_name, ProjectStatus status, 
                                         const std::string& message)
{
  std::lock_guard<std::mutex> lock(projects_mutex_);
  auto it = projects_.find(project_name);
  if (it != projects_.end()) {
    it->second.status = status;
    it->second.message = message;
    
    // 更新描述信息
    switch (status) {
      case ProjectStatus::STATUS_STOPPED:
        it->second.description = "Project is stopped";
        break;
      case ProjectStatus::STATUS_RUNNING:
        it->second.description = "Project is running normally";
        break;
      case ProjectStatus::STATUS_STARTING:
        it->second.description = "Project is starting up";
        break;
      case ProjectStatus::STATUS_STOPPING:
        it->second.description = "Project is stopping";
        break;
      case ProjectStatus::STATUS_ERROR:
        it->second.description = "Project encountered an error";
        break;
    }
  }
}

bool ProjectControl::validate_project_name(const std::string& project_name)
{
  return project_name == "camera" || project_name == "pcl2laser" || project_name == "detection";
}

std::string ProjectControl::get_project_executable(const std::string& project_name)
{
  // 这里应该返回实际的可执行文件路径
  // 目前返回模拟路径
  if (project_name == "camera") {
    return "/usr/local/bin/camera_node";
  } else if (project_name == "pcl2laser") {
    return "/usr/local/bin/pcl2laser_node";
  } else if (project_name == "detection") {
    return "/usr/local/bin/detection_node";
  }
  return "";
}

}  // namespace udp_comm_server