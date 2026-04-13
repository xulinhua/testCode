#ifndef UDP_COMM_CLIENT__UDP_CLIENT_NODE_HPP_
#define UDP_COMM_CLIENT__UDP_CLIENT_NODE_HPP_

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_msgs_comm/msg/laser_scan_data.hpp"
#include "custom_msgs_comm/msg/point_cloud_data.hpp"
#include "custom_msgs_comm/msg/ai_coordinate_data.hpp"
#include "custom_msgs_comm/msg/project_command.hpp"
#include "custom_msgs_comm/msg/project_status.hpp"

#include "udp_comm_client/project_manager.hpp"
#include "log_system/log_macros.hpp"

namespace udp_comm_client
{

enum class MessageType : uint32_t {
  COMMAND = 0,
  AI_COORDINATE = 1,
  LASERSCAN = 2,
  POINTCLOUD = 3,
  CALIBRATION_COMMAND = 4,
  CALIBRATION_RESPONSE = 5,
  PROJECT_COMMAND = 6,
  PROJECT_STATUS = 7
};

class UDPClientNode : public rclcpp::Node
{
public:
  UDPClientNode();
  ~UDPClientNode();

  void send_project_status(const std::shared_ptr<custom_msgs_comm::msg::ProjectStatus>& status_msg);

private:
  // 参数
  std::string server_ip_;
  int server_port_;
  int client_port_;
  bool bShowRunInfo_;
  int log_interval_;
  bool udp_blocking_mode_;
  bool auto_start_enabled_;
  bool auto_start_camera_;
  bool auto_start_pcl2laser_;
  bool auto_start_detection_;
  double auto_start_delay_;

  // UDP相关
  int udp_socket_;
  struct sockaddr_in server_address_;

  // 日志计数器
  std::atomic<int> ai_log_counter_{0};
  std::atomic<int> laser_log_counter_{0};
  std::atomic<int> pointcloud_log_counter_{0};

  // 线程控制
  std::atomic<bool> is_running_{true};
  std::thread receive_thread_;
  std::thread command_thread_;
  
  // 命令队列
  std::queue<std::string> command_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  // ROS订阅者和发布者
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;
  rclcpp::Subscription<custom_msgs_comm::msg::AICoordinateData>::SharedPtr ai_coordinate_sub_;
  rclcpp::Subscription<custom_msgs_comm::msg::LaserScanData>::SharedPtr laserscan_sub_;
  rclcpp::Subscription<custom_msgs_comm::msg::PointCloudData>::SharedPtr pointcloud_sub_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr auto_start_timer_;

  void init_Subscription();
  void init_Publisher();

  // 回调函数
  void command_callback(const std_msgs::msg::String::SharedPtr msg);
  void ai_coordinate_callback(const std::shared_ptr<custom_msgs_comm::msg::AICoordinateData> msg);
  void laserscan_callback(const std::shared_ptr<custom_msgs_comm::msg::LaserScanData> msg);
  void pointcloud_callback(const std::shared_ptr<custom_msgs_comm::msg::PointCloudData> msg);
  
  // 线程函数
  void receive_loop();
  void command_listener();
  
  // 处理函数
  void process_command(const std::string& data, const struct sockaddr_in& addr);
  void handle_project_command(const custom_msgs_comm::msg::ProjectCommand::SharedPtr command_msg);
  
  // 工具函数
  void set_udp_blocking_mode(bool blocking);
  void send_response(const std::string& response, const struct sockaddr_in& addr);
  void signal_handler(int signum);
};

}  // namespace udp_comm_client

#endif  // UDP_COMM_CLIENT__UDP_CLIENT_NODE_HPP_