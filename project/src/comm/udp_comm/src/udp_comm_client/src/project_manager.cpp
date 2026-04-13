#include "udp_comm_client/project_manager.hpp"
#include "custom_msgs_comm/msg/project_status.hpp"
#include "custom_msgs_comm/msg/project_command.hpp"
#include <chrono>
#include <thread>
#include <cstdlib>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <sstream>
#include <iostream>
#include <fstream>

namespace udp_comm_client
{

ProjectManager::ProjectManager(rclcpp::Logger logger, rclcpp::Node* node)
: logger_(logger), node_(node), camera_type_("Gemini")
{
  RCLCPP_INFO(logger_, "ProjectManager initialized");
  update_camera_commands();
}

ProjectManager::~ProjectManager()
{
  stop_all_projects();
  RCLCPP_INFO(logger_, "ProjectManager destroyed");
}

std::pair<bool, std::string> ProjectManager::start_project(const std::string& project_name)
{
  // 先检查项目名称有效性
  if (!validate_project_name(project_name)) {
    return {false, "Invalid project name: " + project_name};
  }

  // 检查项目是否已经在运行
  {
    std::lock_guard<std::mutex> lock(projects_mutex_);
    auto it = projects_.find(project_name);
    if (it != projects_.end() && it->second.status == ProjectStatus::STATUS_RUNNING) {
      return {false, "Project is already running"};
    }
  }

  // 更新项目状态为启动中（在锁外调用）
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
  {
    std::lock_guard<std::mutex> lock(projects_mutex_);
    projects_.emplace(project_name, std::move(project_info));
  }
  // 启动项目线程
  std::thread project_thread(&ProjectManager::run_project, this, project_name, std::ref(*projects_[project_name].should_stop));
  // 存储线程ID
  projects_[project_name].thread_id = project_thread.get_id();
  
  {
    RCLCPP_INFO(logger_, "启动项目线程，加锁前");
    std::lock_guard<std::mutex> thread_lock(threads_mutex_);
    project_threads_[project_name] = std::move(project_thread);
    RCLCPP_INFO(logger_, "启动项目线程，加锁后");
  }

  RCLCPP_INFO(logger_, "Started project thread: %s", project_name.c_str());
  return {true, "Project started successfully"};
}

std::pair<bool, std::string> ProjectManager::stop_project(const std::string& project_name)
{
  // 检查项目是否存在且正在运行
  bool should_stop = false;
  {
    std::lock_guard<std::mutex> lock(projects_mutex_);
    auto it = projects_.find(project_name);
    if (it == projects_.end()) {
      return {false, "Project not found"};
    }

    if (it->second.status != ProjectStatus::STATUS_RUNNING) {
      return {false, "Project is not running"};
    }

    // 设置停止标志
    if (it->second.should_stop) {
      *it->second.should_stop = true;
      should_stop = true;
    }
  }

  if (should_stop) {
    // 更新项目状态为停止中（在锁外调用）
    update_project_status(project_name, ProjectStatus::STATUS_STOPPING, "Stopping project");

    // 等待项目停止
    wait_for_project_stop(project_name, 30);

    // 检查是否成功停止
    {
      std::lock_guard<std::mutex> lock(projects_mutex_);
      auto it = projects_.find(project_name);
      if (it != projects_.end() && it->second.status == ProjectStatus::STATUS_STOPPED) {
        RCLCPP_INFO(logger_, "Stopped project: %s", project_name.c_str());
        return {true, "Project stopped successfully"};
      } else {
        RCLCPP_ERROR(logger_, "Failed to stop project: %s", project_name.c_str());
        return {false, "Failed to stop project"};
      }
    }
  }
  
  return {false, "Failed to set stop flag"};
}

std::tuple<bool, std::string, std::string> ProjectManager::get_status(const std::string& project_name)
{
  std::lock_guard<std::mutex> lock(projects_mutex_);
  
  auto it = projects_.find(project_name);
  if (it == projects_.end()) {
    return {false, "Project not found", "Project is not registered"};
  }

  return {it->second.status == ProjectStatus::STATUS_RUNNING, it->second.description, it->second.message};
}

std::vector<std::string> ProjectManager::list_projects()
{
  std::lock_guard<std::mutex> lock(projects_mutex_);
  std::vector<std::string> project_list;
  
  for (const auto& pair : projects_) {
    project_list.push_back(pair.first);
  }
  
  return project_list;
}

void ProjectManager::set_camera_type(const std::string& camera_type)
{
  camera_type_ = camera_type;
  RCLCPP_INFO(logger_, "Camera type set to: %s", camera_type.c_str());
  update_camera_commands();
}

void ProjectManager::run_project(const std::string& project_name, std::atomic<bool>& should_stop)
{
  RCLCPP_INFO(logger_, "Starting project: %s with camera type: %s", 
              project_name.c_str(), camera_type_.c_str());

  // 获取项目配置
  auto it = projects_config_.find(project_name);
  if (it == projects_config_.end()) {
    RCLCPP_ERROR(logger_, "Project configuration not found: %s", project_name.c_str());
    update_project_status(project_name, ProjectStatus::STATUS_ERROR, "Project configuration not found");
    return;
  }

  ProjectConfig config = it->second;
  std::string start_cmd = config.start_cmd;
  std::string working_dir = config.working_dir;
  std::string setup_script = config.setup_script;

  // 如果有工作目录和环境设置脚本，构建完整的命令
  if (!working_dir.empty() && !setup_script.empty()) {
    start_cmd = "cd " + working_dir + " && source " + setup_script + " && " + start_cmd;
  } else if (!working_dir.empty()) {
    start_cmd = "cd " + working_dir + " && " + start_cmd;
  } else if (!setup_script.empty()) {
    start_cmd = "source " + setup_script + " && " + start_cmd;
  }

  RCLCPP_INFO(logger_, "Starting project %s with command: %s", project_name.c_str(), start_cmd.c_str());

  // 在后台启动进程，使用nohup和bash -c确保shell命令正确执行
  std::string background_cmd = "nohup bash -c '" + start_cmd + "' > /tmp/" + project_name + ".log 2>&1 &";
  RCLCPP_INFO(logger_, "Executing command: %s", background_cmd.c_str());
  int result = std::system(background_cmd.c_str());
  
  if (WIFEXITED(result) && WEXITSTATUS(result) == 0) {
    RCLCPP_INFO(logger_, "Project %s command executed successfully", project_name.c_str());
  } else {
    RCLCPP_ERROR(logger_, "Failed to execute project %s command with result: %d", project_name.c_str(), result);
    update_project_status(project_name, ProjectStatus::STATUS_ERROR, "Failed to start project");
    return;
  }

  // 等待一段时间，检查进程是否启动成功
  // 根据项目类型设置不同的等待时间
  int startup_wait_time = 3; // 默认3秒
  
  if (project_name == "camera") {
    startup_wait_time = 8; // 相机项目需要更长时间启动
  } else if (project_name == "pcl2laser") {
    startup_wait_time = 10; // pcl2laser项目需要更长时间初始化
  } else if (project_name == "detection") {
    startup_wait_time = 15; // 检测项目需要最长时间启动
  }
  
  RCLCPP_INFO(logger_, "Waiting %d seconds for %s project to start up", startup_wait_time, project_name.c_str());
  std::this_thread::sleep_for(std::chrono::seconds(startup_wait_time));

  // 改进的项目启动检查逻辑：基于ROS2系统状态而不是进程检查
  bool is_running = false;
  
  // 给ROS2进程额外的时间来完全启动
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  // 根据项目类型使用不同的检查策略
  if (project_name == "camera") {
    // 对于camera项目，使用更可靠的ROS2系统状态检查
    std::string topic_check_cmd = "ros2 topic list | grep -q 'camera' > /dev/null 2>&1";
    int topic_result = std::system(topic_check_cmd.c_str());
    
    if (topic_result == 0) {
      is_running = true;
      RCLCPP_INFO(logger_, "Camera project detected via ROS2 topic check - running successfully");
    } else {
      // 如果话题检查失败，尝试节点检查
      std::string node_check_cmd = "ros2 node list | grep -q 'camera' > /dev/null 2>&1";
      int node_result = std::system(node_check_cmd.c_str());
      
      if (node_result == 0) {
        is_running = true;
        RCLCPP_INFO(logger_, "Camera project detected via ROS2 node check - running successfully");
      } else {
        // 最后尝试进程检查作为备选
        is_running = is_project_process_running(project_name);
        if (is_running) {
          RCLCPP_INFO(logger_, "Camera project detected via process check - running successfully");
        }
      }
    }
  } else if (project_name == "pcl2laser") {
    // 对于pcl2laser项目，检查特定的ROS2话题
    std::string topic_check_cmd = "ros2 topic list | grep -q 'camera_scan' > /dev/null 2>&1";
    int topic_result = std::system(topic_check_cmd.c_str());
    
    if (topic_result == 0) {
      is_running = true;
      RCLCPP_INFO(logger_, "Pcl2laser project detected via ROS2 topic check - running successfully");
    } else {
      // 如果话题检查失败，使用进程检查
      is_running = is_project_process_running(project_name);
      if (is_running) {
        RCLCPP_INFO(logger_, "Pcl2laser project detected via process check - running successfully");
      } else {
        // 给pcl2laser更多时间启动，再检查一次
        std::this_thread::sleep_for(std::chrono::seconds(5));
        is_running = is_project_process_running(project_name);
        if (is_running) {
          RCLCPP_INFO(logger_, "Pcl2laser project detected after additional wait - running successfully");
        }
      }
    }
  } else if (project_name == "detection") {
    // 对于detection项目，检查特定的ROS2话题
    std::string topic_check_cmd = "ros2 topic list | grep -q 'ai_detection' > /dev/null 2>&1";
    int topic_result = std::system(topic_check_cmd.c_str());
    
    if (topic_result == 0) {
      is_running = true;
      RCLCPP_INFO(logger_, "Detection project detected via ROS2 topic check - running successfully");
    } else {
      // 如果话题检查失败，使用进程检查
      is_running = is_project_process_running(project_name);
      if (is_running) {
        RCLCPP_INFO(logger_, "Detection project detected via process check - running successfully");
      } else {
        // 给detection更多时间启动，再检查一次
        std::this_thread::sleep_for(std::chrono::seconds(10));
        is_running = is_project_process_running(project_name);
        if (is_running) {
          RCLCPP_INFO(logger_, "Detection project detected after additional wait - running successfully");
        }
      }
    }
  } else {
    // 对于其他项目，使用进程检查
    is_running = is_project_process_running(project_name);
    if (is_running) {
      RCLCPP_INFO(logger_, "Project %s is running successfully", project_name.c_str());
    }
  }

  if (is_running) {
    // 更新项目状态为运行中
    update_project_status(project_name, ProjectStatus::STATUS_RUNNING, "Project running");
    RCLCPP_INFO(logger_, "Project %s started successfully", project_name.c_str());
  } else {
    RCLCPP_ERROR(logger_, "Project %s failed to start properly", project_name.c_str());
    update_project_status(project_name, ProjectStatus::STATUS_ERROR, "Project failed to start properly");
    return;
  }

  // 项目运行循环
  int check_counter = 0;
  while (!should_stop && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 检查项目状态
    if (should_stop) {
      break;
    }
    
    // 每10次循环（约5秒）检查一次进程状态，避免过于频繁的检查
    check_counter++;
    if (check_counter >= 10) {
      check_counter = 0;
      
      if (!is_project_process_running(project_name)) {
        RCLCPP_ERROR(logger_, "Project %s process has stopped unexpectedly", project_name.c_str());
        update_project_status(project_name, ProjectStatus::STATUS_ERROR, "Project process stopped unexpectedly");
        return;
      } else {
        RCLCPP_DEBUG(logger_, "Project %s is still running normally", project_name.c_str());
      }
    }
  }

  // 如果是因为需要停止而退出循环
  if (should_stop) {
    update_project_status(project_name, ProjectStatus::STATUS_STOPPING, "Project stopping");
    
    // 尝试优雅地停止项目
    std::string stop_cmd = config.stop_cmd;
    if (!stop_cmd.empty()) {
      int stop_result = std::system((stop_cmd + " > /dev/null 2>&1").c_str());
      if (WIFEXITED(stop_result) && WEXITSTATUS(stop_result) == 0) {
        RCLCPP_INFO(logger_, "Project %s stopped successfully", project_name.c_str());
      } else {
        RCLCPP_WARN(logger_, "Failed to stop project %s gracefully, force killing", project_name.c_str());
        // 强制杀死进程
        std::string force_kill_cmd = "pkill -9 -f '" + project_name + "'";
        std::system((force_kill_cmd + " > /dev/null 2>&1").c_str());
      }
    }
  }

  update_project_status(project_name, ProjectStatus::STATUS_STOPPED, "Project stopped");
  RCLCPP_INFO(logger_, "Project stopped: %s", project_name.c_str());
}

bool ProjectManager::is_project_running(const std::string& project_name)
{
  std::lock_guard<std::mutex> lock(projects_mutex_);
  auto it = projects_.find(project_name);
  return it != projects_.end() && it->second.status == ProjectStatus::STATUS_RUNNING;
}

// 添加一个新的公共方法来检查项目是否真的在运行（基于进程检查）
bool ProjectManager::is_project_process_running(const std::string& project_name)
{
  // 获取项目配置
  auto it = projects_config_.find(project_name);
  if (it == projects_config_.end()) {
    return false;
  }

  ProjectConfig config = it->second;
  std::string check_cmd = config.check_cmd;

  if (check_cmd.empty()) {
    // 如果没有检查命令，回退到状态检查
    return is_project_running(project_name);
  }

  // 使用简单的进程检查方法
  std::string full_check_cmd = check_cmd + " > /dev/null 2>&1";
  int result = std::system(full_check_cmd.c_str());
  
  // system命令返回0表示进程存在，非0表示进程不存在
  bool is_running = (result == 0);
  
  // 对于camera项目，如果进程检查失败但项目实际上在运行（根据用户反馈），
  // 采用更智能的检查策略
  if (!is_running && project_name == "camera") {
    // 方法1：检查ROS2节点
    std::string node_check_cmd = "ros2 node list | grep -q 'camera' > /dev/null 2>&1";
    int node_result = std::system(node_check_cmd.c_str());
    if (node_result == 0) {
      is_running = true;
      RCLCPP_INFO(logger_, "Camera project detected via ROS2 node check");
    } else {
      // 方法2：检查ROS2话题（更可靠的方法）
      std::string topic_check_cmd = "ros2 topic list | grep -q 'camera' > /dev/null 2>&1";
      int topic_result = std::system(topic_check_cmd.c_str());
      if (topic_result == 0) {
        is_running = true;
        RCLCPP_INFO(logger_, "Camera project detected via ROS2 topic check");
      } else {
        // 方法3：检查Realsense相关进程（更宽松的匹配）
        std::string realsense_check_cmd = "pgrep -f 'realsense' > /dev/null 2>&1";
        int realsense_result = std::system(realsense_check_cmd.c_str());
        if (realsense_result == 0) {
          is_running = true;
          RCLCPP_INFO(logger_, "Camera project detected via realsense process check");
        }
      }
    }
  }
  
  RCLCPP_INFO(logger_, "Process check for %s: command='%s', result=%d, running=%s", 
               project_name.c_str(), check_cmd.c_str(), result, is_running ? "true" : "false");
  
  return is_running;
}

void ProjectManager::stop_all_projects()
{
  std::lock_guard<std::mutex> lock(projects_mutex_);
  
  for (auto& pair : projects_) {
    if (pair.second.status == ProjectStatus::STATUS_RUNNING && pair.second.should_stop) {
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

void ProjectManager::wait_for_project_stop(const std::string& project_name, int timeout_seconds)
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

void ProjectManager::update_project_status(const std::string& project_name, ProjectStatus status, 
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
      default:
        it->second.description = "Unknown status";
        break;
    }
  }
}

bool ProjectManager::validate_project_name(const std::string& project_name)
{
  return project_name == "camera" || project_name == "pcl2laser" || project_name == "detection";
}

void ProjectManager::update_camera_commands()
{
  // 根据相机类型获取相应的命令配置
  std::string camera_start_cmd, camera_stop_cmd, camera_check_cmd, camera_description;
  
  if (camera_type_ == "Realsense") {
    camera_start_cmd = "source /home/user/testCode/project/install/setup.bash && ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_sync:=true depth_width:=640 depth_height:=480 depth_fps:=15 color_width:=640 color_height:=480 color_fps:=15 decimation_filter.enable:=true";
    camera_stop_cmd = "pkill -f 'realsense2_camera'; pkill -f 'rs_launch.py'";
    // 改进的检查命令，包含更多可能的进程名称和更宽松的匹配
    camera_check_cmd = "pgrep -f 'realsense2_camera\\|rs_launch.py\\|realsense2_camera_node\\|realsense'";
    camera_description = "RealSense相机项目";
    RCLCPP_INFO(logger_, "Using RealSense camera configuration");
  } else if (camera_type_ == "Gemini") {
    camera_start_cmd = "source /home/user/testCode/project/install/setup.bash && ros2 launch orbbec_camera gemini2.launch.py depth_width:=640 depth_height:=480 depth_fps:=15 color_width:=640 color_height:=480 color_fps:=15 depth_decimation_filter:=true enable_hardware_noise_removal:=false enable_point_cloud:=true";
    camera_stop_cmd = "pkill -f 'orbbec_camera'; pkill -f 'orbbec_camera.launch.py'";
    camera_check_cmd = "pgrep -f 'orbbec_camera\\|orbbec_camera.launch.py'";
    camera_description = "Gemini相机项目";
    RCLCPP_INFO(logger_, "Using Gemini camera configuration");
  } else {
    RCLCPP_ERROR(logger_, "Unsupported camera type: %s", camera_type_.c_str());
    return;
  }

  // 更新项目配置 - 移除重复的source命令，因为命令中已经包含了
  projects_config_["camera"] = {
    camera_start_cmd,
    camera_stop_cmd,
    camera_check_cmd,
    "",  // 工作目录为空，因为命令中已经包含了cd
    "",  // setup脚本为空，因为命令中已经包含了source
    camera_description
  };

  projects_config_["pcl2laser"] = {
    "source /home/user/testCode/project/install/setup.bash && ros2 launch pcl2laserscan_trans pcl2laserscan_trans.launch.py mode:=camera_only camera_type:=" + camera_type_,
    "pkill -f 'pcl2laserscan_trans.launch.py'; pkill -f 'ros2.*pcl2laserscan_trans'",
    "pgrep -f 'pcl2laserscan_trans\\|pcl2laserscan_trans.launch.py'",
    "",  // 工作目录为空
    "",  // setup脚本为空
    "相机点云转激光扫描项目"
  };

  projects_config_["detection"] = {
    "source /home/user/testCode/project/install/setup.bash && ros2 launch Yolo_Detection yolo_det.py",
    "pkill -f 'Yolo_Detection\\|yolo_det.py'",
    "pgrep -f 'Yolo_Detection\\|yolo_det.py'",
    "",
    "",
    "目标检测项目"
  };
}

}  // namespace udp_comm_client