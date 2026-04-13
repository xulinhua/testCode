#include "../include/marker_detect_ros/marker_detect_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <string>
#include <filesystem>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "log_system/log_macros.hpp"
#include "rclcpp/rclcpp.hpp"
// 添加bas_sys_config_ros头文件
#include "sys_info_src/sys_info_server.h"

/**
 * @brief 标记检测主节点
 * 
 * 该节点读取基础配置参数，包括相机ID列表
 */
class MarkerDetectMainNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   */
  explicit MarkerDetectMainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("marker_detect_node_main", options)
  {
    std::string sys_config_node_name = "sys_config_ros_node";
      sys_config_client_ = std::make_shared<rclcpp::SyncParametersClient>(
      this,  // 当前节点
      sys_config_node_name  // 目标节点名称
      );
  }

  /**
   * @brief 析构函数
   */
  ~MarkerDetectMainNode()
  {

  }

public:

  std::vector<int> getServerCameraIds()
  {
    cam_server_ids_.clear();
    try 
    {
      if (!sys_config_client_->wait_for_service(std::chrono::seconds(3)))
	    {
		    LOG_ERROR("marker_detect_ros", "无法连接到系统配置参数服务");
        return cam_server_ids_;
      }
      // RosComm::getCamIdsFromServer 返回 std::vector<uint8_t>，需要转换为 std::vector<int>
      std::vector<uint8_t> temp_camera_ids = RosComm::getCamIdsFromServer(sys_config_client_);
      std::string cam_ids_str;
      for (const auto& id : temp_camera_ids) {
        cam_server_ids_.push_back(static_cast<int>(id));
        cam_ids_str += std::to_string(id) + " ";
      }
      LOG_INFO("marker_detect_ros", "从系统配置参数服务获取相机ID列表: %s", cam_ids_str.c_str());
    } catch (const YAML::Exception& e) {
      LOG_ERROR("marker_detect_ros", "解析配置文件出错: %s", e.what());
    }
    return cam_server_ids_;
  }
  std::vector<int> getConfigCameraIds()
  {
    try 
    {
      cam_config_ids_.clear();
      std::string config_file_path = ament_index_cpp::get_package_share_directory("marker_detect_ros") + "/config" + "/marker_detect_params.yaml";
      YAML::Node config = YAML::LoadFile(config_file_path);
      std::string cam_ids_str;
      if (config["marker_detect_config"] && config["marker_detect_config"]["camera_id"]) 
      {
        YAML::Node camera_id_node = config["marker_detect_config"]["camera_id"];
        if (camera_id_node.IsSequence()) {
          for (const auto& id : camera_id_node) {
            cam_config_ids_.push_back(id.as<int>());
            cam_ids_str += std::to_string(id.as<int>()) + " ";
          }
        }
      }
      LOG_INFO("marker_detect_ros", "从配置文件获取相机ID列表: %s", cam_ids_str.c_str());
    } 
    catch (const YAML::Exception& e) 
    {
      LOG_ERROR("marker_detect_ros", "解析配置文件出错: %s", e.what());
    }
    return cam_config_ids_;
  }

  std::vector<int> getActiveCameraIds()
  {
    std::vector<int> act_cam_ids; // 有效配置的相机ID
    cam_server_ids_ = getServerCameraIds();
    cam_config_ids_ = getConfigCameraIds();
    std::string cam_ids_str;
    for (int camera_id : cam_config_ids_) 
    {
      if (std::find(cam_server_ids_.begin(), cam_server_ids_.end(), camera_id) != cam_server_ids_.end()) 
      {
        act_cam_ids.push_back(camera_id);
        cam_ids_str += std::to_string(camera_id) + " ";
      }
      else
        LOG_ERROR("marker_detect_ros", "参数服务器没有配置该相机ID，camera_id: %d", camera_id);
    }
    LOG_INFO("marker_detect_ros", "有效配置的相机ID列表: %s", cam_ids_str.c_str());
    cam_active_ids_ = act_cam_ids;
    return act_cam_ids;
  }

private:
  rclcpp::SyncParametersClient::SharedPtr sys_config_client_;
  std::vector<int> cam_server_ids_; // 从系统配置参数服务获取的相机ID列表
  std::vector<int> cam_config_ids_; // 从配置文件获取的相机ID列表
  std::vector<int> cam_active_ids_; // 激活的相机ID列表
};

int main(int argc, char ** argv)
{
  // 构造新的命令行参数，包含配置文件路径
  std::vector<std::string> new_argv_strings;
  std::vector<char*> new_argv;
  
  // 复制原始参数
  for (int i = 1; i < argc; i++) 
  {
    new_argv_strings.push_back(std::string(argv[i]));
  }
  std::string default_config_file_path = ament_index_cpp::get_package_share_directory("marker_detect_ros") + "/config" + "/marker_detect_params.yaml";
  if (argc <= 1)
    new_argv_strings.push_back("--ros-args");
  new_argv_strings.push_back("--params-file");
  new_argv_strings.push_back(default_config_file_path);
  std::cout << "使用默认配置文件: " << default_config_file_path << std::endl;
  
  // 转换为char*数组
  new_argv.reserve(new_argv_strings.size() + 1);
  for (const auto& arg : new_argv_strings) {
    std::cout << "参数配置: " << arg << std::endl;
    new_argv.push_back(const_cast<char*>(arg.c_str()));
  }
  new_argv.push_back(nullptr); // 确保以nullptr结尾
  // 初始化ROS 2
  //rclcpp::init(new_argv.size() - 1, new_argv.data());
  rclcpp::init(argc, argv);
  LOG_INFO("marker_detect_ros", "marker_detect_ros节点启动");
  rclcpp::executors::MultiThreadedExecutor executor; // 多线程处理

  auto main_node = std::make_shared<MarkerDetectMainNode>();
  std::vector<int> act_cam_ids = main_node->getActiveCameraIds();

  if (act_cam_ids.empty()) 
  {
    LOG_ERROR("marker_detect_ros", "有效配置的相机ID列表为空,默认使用相机ID 0");
    act_cam_ids = {0}; // 默认使用相机ID 0
  }

  // 根据camera_id列表创建对应的节点对象
  std::vector<std::shared_ptr<marker_detect_ros::MarkerDetectNode>> nodes;
  for (int camera_id : act_cam_ids) 
  {
    LOG_INFO("marker_detect_ros", "创建 MarkerDetectNode 节点，camera_id: %d", camera_id);
    auto node = std::make_shared<marker_detect_ros::MarkerDetectNode>(
      rclcpp::NodeOptions(), camera_id);
    nodes.push_back(node);
    executor.add_node(node);
    std::cout << "创建 MarkerDetectNode 节点，camera_id: " << camera_id << std::endl;
  }

  //rclcpp::spin(node);
  executor.spin();  // ✅ 但会处理所有相关的回调

  for (auto& node : nodes) {
    // 正常退出时发布停止状态
    node->publishModuleStatus(basros::ModuleStatus::STOPPED, "模块正常停止");
  }

  rclcpp::shutdown();
  return 0;
}