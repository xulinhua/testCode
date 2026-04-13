#include "udp_comm_client/udp_client_node.hpp"
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <signal.h>
#include <jsoncpp/json/json.h>

#include "std_msgs/msg/string.hpp"
#include "custom_msgs_comm/msg/laser_scan_data.hpp"
#include "custom_msgs_comm/msg/point_cloud_data.hpp"
#include "custom_msgs_comm/msg/ai_coordinate_data.hpp"
#include "custom_msgs_comm/msg/project_command.hpp"
#include "custom_msgs_comm/msg/project_status.hpp"

namespace udp_comm_client
{

UDPClientNode::UDPClientNode()
: Node("udp_comm_client")
{
  LOG_INFO("UDP Client 日志系统初始化完成");
  
  // 声明参数
  this->declare_parameter("server_ip", "192.168.10.30");
  this->declare_parameter("server_port", 8888);
  this->declare_parameter("client_port", 8889);
  this->declare_parameter("bShowRunInfo", true);
  this->declare_parameter("log_interval", 10);
  this->declare_parameter("udp_blocking_mode", false);
  this->declare_parameter("auto_start_enabled", false);
  this->declare_parameter("auto_start_camera", false);
  this->declare_parameter("auto_start_pcl2laser", false);
  this->declare_parameter("auto_start_detection", false);
  this->declare_parameter("auto_start_delay", 3.0);

  // 获取参数
  server_ip_ = this->get_parameter("server_ip").as_string();
  server_port_ = this->get_parameter("server_port").as_int();
  client_port_ = this->get_parameter("client_port").as_int();
  bShowRunInfo_ = this->get_parameter("bShowRunInfo").as_bool();
  log_interval_ = this->get_parameter("log_interval").as_int();
  udp_blocking_mode_ = this->get_parameter("udp_blocking_mode").as_bool();
  auto_start_enabled_ = this->get_parameter("auto_start_enabled").as_bool();
  auto_start_camera_ = this->get_parameter("auto_start_camera").as_bool();
  auto_start_pcl2laser_ = this->get_parameter("auto_start_pcl2laser").as_bool();
  auto_start_detection_ = this->get_parameter("auto_start_detection").as_bool();
  auto_start_delay_ = this->get_parameter("auto_start_delay").as_double();

  // 创建UDP套接字
  udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_socket_ < 0) {
    LOG_ERROR("Failed to create UDP socket");
    throw std::runtime_error("Failed to create UDP socket");
  }

  // 设置发送缓冲大小
  int send_buffer_size = 1024 * 1024;
  setsockopt(udp_socket_, SOL_SOCKET, SO_SNDBUF, &send_buffer_size, sizeof(send_buffer_size));

  // 设置UDP发送模式
  set_udp_blocking_mode(udp_blocking_mode_);
  
  // 输出UDP发送模式日志，与Python版本一致
  if (udp_blocking_mode_) {
    LOG_INFO("UDP发送模式设置为: 阻塞");
  } else {
    LOG_INFO("UDP发送模式设置为: 非阻塞");
  }

  // 绑定客户端端口
  struct sockaddr_in client_addr;
  memset(&client_addr, 0, sizeof(client_addr));
  client_addr.sin_family = AF_INET;
  client_addr.sin_addr.s_addr = INADDR_ANY;
  client_addr.sin_port = htons(client_port_);

  if (bind(udp_socket_, (struct sockaddr*)&client_addr, sizeof(client_addr)) < 0) {
    LOG_ERROR("Failed to bind UDP socket to port %d", client_port_);
    close(udp_socket_);
    throw std::runtime_error("Failed to bind UDP socket");
  }

  // 设置服务器地址
  memset(&server_address_, 0, sizeof(server_address_));
  server_address_.sin_family = AF_INET;
  server_address_.sin_port = htons(server_port_);
  inet_pton(AF_INET, server_ip_.c_str(), &server_address_.sin_addr);

  // 输出初始化信息，与Python版本一致
  LOG_INFO("UDP Client initialized, sending to %s:%d", server_ip_.c_str(), server_port_);
  LOG_INFO("Listening for commands on port %d", client_port_);
  LOG_INFO("Log interval: every %d messages", log_interval_);

  // 创建ROS2订阅者和发布者
  init_Subscription();
  init_Publisher();

  // 启动接收线程
  receive_thread_ = std::thread(&UDPClientNode::receive_loop, this);

  // 启动指令监听线程
  command_thread_ = std::thread(&UDPClientNode::command_listener, this);

  // 注册信号处理
  signal(SIGTERM, [](int /*signum*/) { rclcpp::shutdown(); });
  signal(SIGINT, [](int /*signum*/) { rclcpp::shutdown(); });

  if (bShowRunInfo_) {
    LOG_INFO("ProjectManager initialized");
    LOG_INFO("UDP Client initialized, sending to %s:%d", 
                server_ip_.c_str(), server_port_);
    LOG_INFO("Listening for commands on port %d", client_port_);
    LOG_INFO("Log interval: every %d messages", log_interval_);
  }
}

UDPClientNode::~UDPClientNode()
{
  is_running_ = false;

  // 停止定时器
  if (auto_start_timer_) auto_start_timer_->cancel();

  // 关闭UDP套接字
  if (udp_socket_ >= 0) {
    close(udp_socket_);
  }

  // 等待线程结束
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (command_thread_.joinable()) {
    command_thread_.join();
  }
}

void UDPClientNode::init_Subscription()
{
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  auto laser_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  // 指令订阅者
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/udp_comm_server/command", 10,
    std::bind(&UDPClientNode::command_callback, this, std::placeholders::_1));
  
    // 数据订阅者
  ai_coordinate_sub_ = this->create_subscription<custom_msgs_comm::msg::AICoordinateData>(
    "/ai_detection/coordinates", qos_profile,
    std::bind(&UDPClientNode::ai_coordinate_callback, this, std::placeholders::_1));

  laserscan_sub_ = this->create_subscription<custom_msgs_comm::msg::LaserScanData>(
    "/camera_scan", laser_qos,
    std::bind(&UDPClientNode::laserscan_callback, this, std::placeholders::_1));

  pointcloud_sub_ = this->create_subscription<custom_msgs_comm::msg::PointCloudData>(
    "/camera/pointcloud", qos_profile,
    std::bind(&UDPClientNode::pointcloud_callback, this, std::placeholders::_1));
}

void UDPClientNode::init_Publisher()
{
  // 指令响应发布者
  response_pub_ = this->create_publisher<std_msgs::msg::String>("/udp_comm_client/response", 10);
}

void UDPClientNode::set_udp_blocking_mode(bool blocking)
{
  int flags = fcntl(udp_socket_, F_GETFL, 0);
  if (flags == -1) {
    LOG_ERROR("Failed to get socket flags");
    return;
  }
  
  if (blocking) {
    flags &= ~O_NONBLOCK;
  } else {
    flags |= O_NONBLOCK;
  }
  
  if (fcntl(udp_socket_, F_SETFL, flags) == -1) {
    LOG_ERROR("Failed to set socket blocking mode");
  }
}

void UDPClientNode::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  command_queue_.push(msg->data);
  queue_cv_.notify_one();
}

void UDPClientNode::ai_coordinate_callback(const custom_msgs_comm::msg::AICoordinateData::SharedPtr msg)
{
  try {
    // 序列化AI坐标数据为JSON格式，与Python版本一致
    Json::Value ai_data;
    ai_data["object_class"] = msg->object_class;
    
    // 添加位置数据
    Json::Value position(Json::arrayValue);
    for (const auto& pos_val : msg->position) {
      position.append(pos_val);
    }
    ai_data["position"] = position;
    
    // 添加方向数据
    Json::Value orientation(Json::arrayValue);
    for (const auto& orient_val : msg->orientation) {
      orientation.append(orient_val);
    }
    ai_data["orientation"] = orientation;
    
    ai_data["confidence"] = msg->confidence;
    ai_data["frame_id"] = msg->frame_id;
    ai_data["stamp"]["sec"] = msg->stamp.sec;
    ai_data["stamp"]["nanosec"] = msg->stamp.nanosec;
    
    Json::StreamWriterBuilder writer;
    std::string json_str = Json::writeString(writer, ai_data);
    
    // 构建UDP数据包：消息类型 + JSON数据
    uint32_t msg_type = static_cast<uint32_t>(MessageType::AI_COORDINATE);
    std::vector<uint8_t> packet(sizeof(msg_type) + json_str.size());
    
    // 写入消息类型（大端序）
    memcpy(packet.data(), &msg_type, sizeof(msg_type));
    // 写入JSON数据
    memcpy(packet.data() + sizeof(msg_type), json_str.c_str(), json_str.size());
    
    // 发送到服务器
    ssize_t bytes_sent = sendto(udp_socket_, packet.data(), packet.size(), 0,
                               (struct sockaddr*)&server_address_, sizeof(server_address_));
    
    if (bytes_sent < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        LOG_ERROR("Failed to send AI coordinate data: %s", strerror(errno));
      }
    } else if (ai_log_counter_++ % log_interval_ == 0) {
      LOG_INFO("Sent AI coordinate data via UDP");
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error processing AI coordinate data: %s", e.what());
  }
}

void UDPClientNode::laserscan_callback(const custom_msgs_comm::msg::LaserScanData::SharedPtr msg)
{
  // 序列化激光扫描数据并发送到UDP服务器（二进制格式）
  try {
    // 计算数据包大小
    // 格式: [7个float值] + [frame_id_len + frame_id] + [stamp_sec + stamp_nanosec] + 
    //       [ranges_len + ranges] + [intensities_len + intensities]
    size_t packet_size = sizeof(float) * 7 +                    // angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max
                         sizeof(uint32_t) +                      // frame_id长度
                         msg->header.frame_id.size() +           // frame_id内容
                         sizeof(uint32_t) * 2 +                  // stamp.sec, stamp.nanosec
                         sizeof(uint32_t) +                      // ranges长度
                         msg->ranges.size() * sizeof(float) +    // ranges内容
                         sizeof(uint32_t) +                      // intensities长度
                         msg->intensities.size() * sizeof(float); // intensities内容
    
    std::vector<uint8_t> packet(packet_size);
    size_t offset = 0;
    
    // 写入7个float值
    memcpy(packet.data() + offset, &msg->angle_min, sizeof(float));
    offset += sizeof(float);
    memcpy(packet.data() + offset, &msg->angle_max, sizeof(float));
    offset += sizeof(float);
    memcpy(packet.data() + offset, &msg->angle_increment, sizeof(float));
    offset += sizeof(float);
    memcpy(packet.data() + offset, &msg->time_increment, sizeof(float));
    offset += sizeof(float);
    memcpy(packet.data() + offset, &msg->scan_time, sizeof(float));
    offset += sizeof(float);
    memcpy(packet.data() + offset, &msg->range_min, sizeof(float));
    offset += sizeof(float);
    memcpy(packet.data() + offset, &msg->range_max, sizeof(float));
    offset += sizeof(float);
    
    // 写入frame_id长度和内容
    uint32_t frame_id_len = msg->header.frame_id.size();
    memcpy(packet.data() + offset, &frame_id_len, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(packet.data() + offset, msg->header.frame_id.c_str(), frame_id_len);
    offset += frame_id_len;
    
    // 写入时间戳
    memcpy(packet.data() + offset, &msg->header.stamp.sec, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(packet.data() + offset, &msg->header.stamp.nanosec, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // 写入ranges长度和内容
    uint32_t ranges_len = msg->ranges.size();
    memcpy(packet.data() + offset, &ranges_len, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    if (ranges_len > 0) {
      memcpy(packet.data() + offset, msg->ranges.data(), ranges_len * sizeof(float));
      offset += ranges_len * sizeof(float);
    }
    
    // 写入intensities长度和内容
    uint32_t intensities_len = msg->intensities.size();
    memcpy(packet.data() + offset, &intensities_len, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    if (intensities_len > 0) {
      memcpy(packet.data() + offset, msg->intensities.data(), intensities_len * sizeof(float));
      offset += intensities_len * sizeof(float);
    }
    
    // 添加消息头
    uint32_t msg_type = static_cast<uint32_t>(MessageType::LASERSCAN);  // LASERSCAN = 2
    std::vector<uint8_t> full_packet(sizeof(msg_type) + packet.size());
    memcpy(full_packet.data(), &msg_type, sizeof(msg_type));
    memcpy(full_packet.data() + sizeof(msg_type), packet.data(), packet.size());
    
    // 发送到服务器
    ssize_t bytes_sent = sendto(udp_socket_, full_packet.data(), full_packet.size(), 0,
                               (struct sockaddr*)&server_address_, sizeof(server_address_));
    
    if (bytes_sent < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        LOG_ERROR("Failed to send laser scan data: %s", strerror(errno));
      }
    } else if (laser_log_counter_++ % log_interval_ == 0) {
      LOG_INFO("Sent laser scan data via UDP, size: %ld bytes", bytes_sent);
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error processing laser scan data: %s", e.what());
  }
}

void UDPClientNode::pointcloud_callback(const custom_msgs_comm::msg::PointCloudData::SharedPtr msg)
{
  try {
    // 构建点云数据包 - 使用二进制格式，与Python版本一致
    size_t offset = 0;
    
    // 计算所需的数据包大小
    size_t packet_size = sizeof(uint32_t) * 3 +  // height, width, point_step
                         sizeof(uint32_t) * 2 +  // stamp.sec, stamp.nanosec
                         sizeof(float) * 9 +   // camera_intrinsic (假设9个float值)
                         sizeof(uint32_t) +    // frame_id长度
                         msg->header.frame_id.size() +  // frame_id内容
                         sizeof(uint32_t) +    // data长度
                         msg->data.size();     // data内容
    
    std::vector<uint8_t> packet(packet_size);
    
    // 写入height, width, point_step
    memcpy(packet.data() + offset, &msg->height, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(packet.data() + offset, &msg->width, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(packet.data() + offset, &msg->point_step, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // 写入时间戳
    memcpy(packet.data() + offset, &msg->header.stamp.sec, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(packet.data() + offset, &msg->header.stamp.nanosec, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // 写入相机内参 (假设有9个float值)
    std::vector<float> dummy_intrinsic(9, 1.0f); // 创建一个临时的内参向量，实际应用中应使用msg中的数据
    for (int i = 0; i < 9; i++) {
      memcpy(packet.data() + offset, &dummy_intrinsic[i], sizeof(float));
      offset += sizeof(float);
    }
    
    // 写入frame_id长度和内容
    uint32_t frame_id_len = msg->header.frame_id.size();
    memcpy(packet.data() + offset, &frame_id_len, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(packet.data() + offset, msg->header.frame_id.c_str(), frame_id_len);
    offset += frame_id_len;
    
    // 写入数据长度和内容
    uint32_t data_len = msg->data.size();
    memcpy(packet.data() + offset, &data_len, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(packet.data() + offset, msg->data.data(), data_len);
    
    // 添加消息头并发送
    uint32_t msg_type = static_cast<uint32_t>(MessageType::POINTCLOUD);
    std::vector<uint8_t> full_packet(sizeof(msg_type) + packet.size());
    memcpy(full_packet.data(), &msg_type, sizeof(msg_type));
    memcpy(full_packet.data() + sizeof(msg_type), packet.data(), packet.size());
    
    // 发送到服务器
    ssize_t bytes_sent = sendto(udp_socket_, full_packet.data(), full_packet.size(), 0,
                               (struct sockaddr*)&server_address_, sizeof(server_address_));
    
    if (bytes_sent < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        LOG_ERROR("Failed to send point cloud data: %s", strerror(errno));
      }
    } else if (pointcloud_log_counter_++ % log_interval_ == 0) {
      LOG_INFO("Sent point cloud data via UDP");
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error processing point cloud data: %s", e.what());
  }
}

void UDPClientNode::receive_loop()
{
  char buffer[4096];
  struct sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);

  while (is_running_) {
    ssize_t bytes_received = recvfrom(udp_socket_, buffer, sizeof(buffer) - 1, 0,
                                      (struct sockaddr*)&client_addr, &addr_len);
    
    if (bytes_received > 0) {
      buffer[bytes_received] = '\0';
      std::string data(buffer);
      
      // 处理接收到的数据
      process_command(data, client_addr);
    } else if (bytes_received == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
      LOG_ERROR("Error receiving UDP data: %s", strerror(errno));
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void UDPClientNode::command_listener()
{
  while (is_running_) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cv_.wait(lock, [this]() { return !command_queue_.empty() || !is_running_; });
    
    if (!is_running_) break;
    
    if (!command_queue_.empty()) {
      std::string command = command_queue_.front();
      command_queue_.pop();
      lock.unlock();
      
      // 处理命令
      process_command(command, server_address_);
    }
  }
}

void UDPClientNode::process_command(const std::string& data, const struct sockaddr_in& addr)
{
  try {
    if (data.size() < sizeof(uint32_t)) {
      if (bShowRunInfo_) {
        LOG_WARN("Received incomplete command from client");
      }
      return;
    }
    
    uint32_t msg_type;
    memcpy(&msg_type, data.data(), sizeof(msg_type));
    
    std::string payload = data.substr(sizeof(msg_type));
    
    switch (msg_type) {
      case static_cast<uint32_t>(MessageType::PROJECT_COMMAND): 
      {
        break;
      }
      
      case static_cast<uint32_t>(MessageType::CALIBRATION_COMMAND): { // 处理标定指令
        // process_calibration_command(payload, addr);
        break;
      }
      
      case static_cast<uint32_t>(MessageType::COMMAND): { // 处理一般指令
        if (bShowRunInfo_) {
          LOG_INFO("Received command from client: %s", payload.c_str());
        }
        
        // 处理指令（此处添加您的业务逻辑）
        std::string response = "Executed: " + payload;
        
        // 发布到ROS话题
        auto response_msg = std::make_shared<std_msgs::msg::String>();
        response_msg->data = response;
        response_pub_->publish(*response_msg);
        
        // 可选：发送响应回服务器
        send_response(response, addr);
        break;
      }
      
      default: {
        if (bShowRunInfo_) {
          LOG_WARN("Unknown message type: %u", msg_type);
        }
        break;
      }
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error processing command: %s", e.what());
  }
}

void UDPClientNode::send_response(const std::string& response, const struct sockaddr_in& addr)
{
  // 实现发送响应逻辑
  ssize_t bytes_sent = sendto(udp_socket_, response.c_str(), response.length(), 0,
                             (struct sockaddr*)&addr, sizeof(addr));
  
  if (bytes_sent < 0) {
    LOG_ERROR("Failed to send response: %s", strerror(errno));
  }
}

void UDPClientNode::signal_handler(int /*signum*/)
{
  // 实现信号处理逻辑
  rclcpp::shutdown();
}

void UDPClientNode::send_project_status(const std::shared_ptr<custom_msgs_comm::msg::ProjectStatus>& status_msg)
{
  try {
    // 序列化状态消息为JSON格式，与Python版本一致
    Json::Value json_data;
    json_data["status"] = status_msg->status;
    json_data["project_name"] = status_msg->project_name;
    json_data["message"] = status_msg->message;
    
    Json::StreamWriterBuilder writer;
    std::string json_str = Json::writeString(writer, json_data);
    
    // 构建UDP数据包：消息类型 + JSON数据
    uint32_t msg_type = 7; // PROJECT_STATUS
    std::vector<uint8_t> packet(sizeof(msg_type) + json_str.size());
    
    // 写入消息类型（大端序）
    memcpy(packet.data(), &msg_type, sizeof(msg_type));
    // 写入JSON数据
    memcpy(packet.data() + sizeof(msg_type), json_str.c_str(), json_str.size());
    
    // 发送到服务器
    ssize_t bytes_sent = sendto(udp_socket_, packet.data(), packet.size(), 0,
                               (struct sockaddr*)&server_address_, sizeof(server_address_));
    
    if (bytes_sent < 0) {
      LOG_ERROR("Failed to send project status: %s", strerror(errno));
    } else if (bShowRunInfo_) {
      LOG_INFO("Sent project status: %s - %s", 
                  status_msg->project_name.c_str(), status_msg->message.c_str());
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error sending project status: %s", e.what());
  }
}

}  // namespace udp_comm_client