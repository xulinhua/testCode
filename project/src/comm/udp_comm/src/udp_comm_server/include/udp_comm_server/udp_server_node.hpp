#ifndef UDP_COMM_SERVER__UDP_SERVER_NODE_HPP_
#define UDP_COMM_SERVER__UDP_SERVER_NODE_HPP_

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <unordered_map>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_msgs_comm/msg/point_cloud_data.hpp"
#include "custom_msgs_comm/msg/ai_coordinate_data.hpp"
#include "custom_msgs_comm/msg/laser_scan_data.hpp"
#include "custom_msgs_comm/msg/calibration_command.hpp"
#include "custom_msgs_comm/msg/calibration_response.hpp"
#include "custom_msgs_comm/msg/project_command.hpp"
#include "custom_msgs_comm/msg/project_status.hpp"
#include "custom_msgs_comm/srv/project_control.hpp"
#include "log_system/log_macros.hpp"

namespace udp_comm_server
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

class UDPServerNode : public rclcpp::Node
{
public:
  UDPServerNode();
  ~UDPServerNode();

private:
  // 参数
  int udp_port_;
  bool bShowRunInfo_;
  int log_interval_;
  bool udp_blocking_mode_;
  std::string client_ip_;
  int client_port_;

  // UDP相关
  int udp_socket_;
  struct sockaddr_in client_address_;
  
  // 客户端连接管理
  std::unordered_map<std::string, struct sockaddr_in> connected_clients_;
  std::mutex clients_mutex_;

  // 线程控制
  std::atomic<bool> is_running_{true};
  std::thread receive_thread_;
  std::thread send_thread_;
  
  // 发送队列
  struct SendItem {
    std::vector<uint8_t> data;
    struct sockaddr_in addr;
  };
  std::queue<SendItem> send_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  // 日志计数器
  std::atomic<int> ai_log_counter_{0};
  std::atomic<int> laser_log_counter_{0};
  std::atomic<int> pointcloud_log_counter_{0};

  // ROS订阅者和发布者
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<custom_msgs_comm::msg::ProjectStatus>::SharedPtr project_status_sub_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;
  rclcpp::Publisher<custom_msgs_comm::msg::CalibrationCommand>::SharedPtr calibration_pub_;
  rclcpp::Publisher<custom_msgs_comm::msg::ProjectCommand>::SharedPtr project_command_pub_;
  rclcpp::Publisher<custom_msgs_comm::msg::AICoordinateData>::SharedPtr ai_coordinate_pub_;
  rclcpp::Publisher<custom_msgs_comm::msg::LaserScanData>::SharedPtr laserscan_pub_;
  rclcpp::Publisher<custom_msgs_comm::msg::PointCloudData>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<custom_msgs_comm::msg::CalibrationResponse>::SharedPtr calibration_response_pub_;
  rclcpp::Publisher<custom_msgs_comm::msg::ProjectStatus>::SharedPtr project_status_pub_;
  
  // 服务
  rclcpp::Service<custom_msgs_comm::srv::ProjectControl>::SharedPtr project_control_service_;

  void init_Subscription();
  void init_Publisher();
  void init_Service();
  // 回调函数
  void command_callback(const std_msgs::msg::String::SharedPtr msg);
  void project_status_callback(const custom_msgs_comm::msg::ProjectStatus::SharedPtr msg);
  
  // 服务回调
  void handle_project_control(
    const std::shared_ptr<custom_msgs_comm::srv::ProjectControl::Request> request,
    std::shared_ptr<custom_msgs_comm::srv::ProjectControl::Response> response);
  void handle_project_status(
    const std::shared_ptr<custom_msgs_comm::srv::ProjectControl::Request> request,
    std::shared_ptr<custom_msgs_comm::srv::ProjectControl::Response> response);

  // 线程函数
  void receive_loop();
  void send_loop();
  
  // 处理函数
  void process_udp_message(const std::vector<uint8_t>& data, const struct sockaddr_in& addr);
  void process_ai_coordinate_data(const std::string& json_data, const struct sockaddr_in& addr);
  void process_laserscan_data(const std::vector<uint8_t>& data, const struct sockaddr_in& addr);
  void process_pointcloud_data(const std::vector<uint8_t>& data, const struct sockaddr_in& addr);
  void process_calibration_response(const std::string& json_data, const struct sockaddr_in& addr);
  void process_project_status(const std::string& json_data, const struct sockaddr_in& addr);
  
  // 发送函数
  void send_to_client(const std::vector<uint8_t>& data, const struct sockaddr_in& addr);
  void broadcast_to_clients(const std::vector<uint8_t>& data);
  
  // 工具函数
  void set_udp_blocking_mode(bool blocking);
  void register_client(const std::string& client_id, const struct sockaddr_in& addr);
  void unregister_client(const std::string& client_id);
  std::string get_client_id(const struct sockaddr_in& addr);
};

}  // namespace udp_comm_server

#endif  // UDP_COMM_SERVER__UDP_SERVER_NODE_HPP_