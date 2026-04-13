#include "udp_comm_server/udp_server_node.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <signal.h>
#include <jsoncpp/json/json.h>
#include <memory>
#include <thread>

#include "std_msgs/msg/string.hpp"
//#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_msgs_comm/msg/laser_scan_data.hpp"
#include "custom_msgs_comm/msg/point_cloud_data.hpp"
#include "custom_msgs_comm/msg/ai_coordinate_data.hpp"
#include "custom_msgs_comm/msg/calibration_command.hpp"
#include "custom_msgs_comm/msg/calibration_response.hpp"
#include "custom_msgs_comm/msg/project_command.hpp"
#include "custom_msgs_comm/msg/project_status.hpp"
#include "custom_msgs_comm/srv/project_control.hpp"
#include "log_system/log_macros.hpp"

namespace udp_comm_server
{

using namespace std::chrono_literals;
using custom_msgs_comm::msg::ProjectStatus;
using custom_msgs_comm::msg::ProjectCommand;
using custom_msgs_comm::msg::CalibrationCommand;
using custom_msgs_comm::msg::CalibrationResponse;
using custom_msgs_comm::msg::PointCloudData;
using custom_msgs_comm::msg::AICoordinateData;
using custom_msgs_comm::msg::LaserScanData;
using custom_msgs_comm::srv::ProjectControl;

UDPServerNode::UDPServerNode()
: Node("udp_comm_server")
{
  LOG_INFO("UDP Server 日志系统初始化完成");
  
  // 声明参数
  this->declare_parameter("udp_port", 8888);
  this->declare_parameter("bShowRunInfo", true);
  this->declare_parameter("log_interval", 10);
  this->declare_parameter("udp_blocking_mode", false);
  this->declare_parameter("client_ip", "192.168.10.30");
  this->declare_parameter("client_port", 8889);

  // 获取参数
  udp_port_ = this->get_parameter("udp_port").as_int();
  bShowRunInfo_ = this->get_parameter("bShowRunInfo").as_bool();
  log_interval_ = this->get_parameter("log_interval").as_int();
  udp_blocking_mode_ = this->get_parameter("udp_blocking_mode").as_bool();
  client_ip_ = this->get_parameter("client_ip").as_string();
  client_port_ = this->get_parameter("client_port").as_int();

  // 创建UDP套接字
  udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_socket_ < 0) {
    LOG_ERROR("Failed to create UDP socket");
    throw std::runtime_error("Failed to create UDP socket");
  }

  // 设置接收缓冲大小
  int recv_buffer_size = 1024 * 1024;
  setsockopt(udp_socket_, SOL_SOCKET, SO_RCVBUF, &recv_buffer_size, sizeof(recv_buffer_size));

  // 设置UDP接收模式
  set_udp_blocking_mode(udp_blocking_mode_);

  // 绑定服务器端口
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(udp_port_);

  if (bind(udp_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    LOG_ERROR("Failed to bind UDP socket to port %d", udp_port_);
    close(udp_socket_);
    throw std::runtime_error("Failed to bind UDP socket");
  }

  // 设置默认客户端地址
  memset(&client_address_, 0, sizeof(client_address_));
  client_address_.sin_family = AF_INET;
  client_address_.sin_port = htons(client_port_);
  inet_pton(AF_INET, client_ip_.c_str(), &client_address_.sin_addr);

  
  init_Subscription();
  init_Publisher();
  init_Service();

  // 启动接收线程
  receive_thread_ = std::thread(&UDPServerNode::receive_loop, this);

  // 启动发送线程
  send_thread_ = std::thread(&UDPServerNode::send_loop, this);

  // 注册信号处理
  signal(SIGTERM, [](int /*signum*/) { rclcpp::shutdown(); });
  signal(SIGINT, [](int /*signum*/) { rclcpp::shutdown(); });

  if (bShowRunInfo_) {
    LOG_INFO("UDP Server initialized, listening on port %d", udp_port_);
    LOG_INFO("Default client: %s:%d", client_ip_.c_str(), client_port_);
    LOG_INFO("Log interval: every %d messages", log_interval_);
  }
}

UDPServerNode::~UDPServerNode()
{
  is_running_ = false;

  // 关闭UDP套接字
  if (udp_socket_ >= 0) {
    close(udp_socket_);
  }

  // 等待线程结束
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (send_thread_.joinable()) {
    send_thread_.join();
  }
}

void UDPServerNode::init_Subscription()
{
  // 创建ROS2订阅者和发布者
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  // 指令订阅者
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/udp_comm_client/command", 10,
    std::bind(&UDPServerNode::command_callback, this, std::placeholders::_1));

  // 项目状态订阅者
  project_status_sub_ = this->create_subscription<custom_msgs_comm::msg::ProjectStatus>(
    "/project/status", qos_profile,
    std::bind(&UDPServerNode::project_status_callback, this, std::placeholders::_1));
}

void UDPServerNode::init_Publisher()
{
  // 标定指令发布者
  calibration_pub_ = this->create_publisher<custom_msgs_comm::msg::CalibrationCommand>("/calibration/command", 10);

  // 项目指令发布者
  project_command_pub_ = this->create_publisher<custom_msgs_comm::msg::ProjectCommand>("/project/command", 10);

  // 创建发布者
  pointcloud_pub_ = this->create_publisher<custom_msgs_comm::msg::PointCloudData>("/jetson/pointcloud", 10);
  ai_coordinate_pub_ = this->create_publisher<custom_msgs_comm::msg::AICoordinateData>("/jetson/aicoordinate", 10);
  laserscan_pub_ = this->create_publisher<custom_msgs_comm::msg::LaserScanData>("/jetson/laserscan", 10);

  response_pub_ = this->create_publisher<std_msgs::msg::String>("/udp_comm_server/response", 10);
  calibration_response_pub_ = this->create_publisher<custom_msgs_comm::msg::CalibrationResponse>("/calibration/response", 10);
  project_status_pub_ = this->create_publisher<custom_msgs_comm::msg::ProjectStatus>("/project/status", 10);
}

void UDPServerNode::init_Service()
{
  // 创建服务
  project_control_service_ = this->create_service<custom_msgs_comm::srv::ProjectControl>(
    "/udp_comm_server/project_control",
    std::bind(&UDPServerNode::handle_project_control, this,
              std::placeholders::_1, std::placeholders::_2));
}

void UDPServerNode::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (bShowRunInfo_) {
    LOG_INFO("Received command from ROS: %s", msg->data.c_str());
  }

  try {
    // 打包指令
    uint32_t msg_type = static_cast<uint32_t>(MessageType::COMMAND);
    std::vector<uint8_t> packet(sizeof(msg_type) + msg->data.size());
    memcpy(packet.data(), &msg_type, sizeof(msg_type));
    memcpy(packet.data() + sizeof(msg_type), msg->data.data(), msg->data.size());

    // 发送到所有连接的客户端
    broadcast_to_clients(packet);
  } catch (const std::exception& e) {
    LOG_ERROR("Error processing command: %s", e.what());
  }
}

void UDPServerNode::project_status_callback(const custom_msgs_comm::msg::ProjectStatus::SharedPtr msg)
{
  try {
    Json::Value json_data;
    json_data["status"] = msg->status;
    json_data["project_name"] = msg->project_name;
    json_data["message"] = msg->message;

    Json::StreamWriterBuilder writer;
    std::string json_str = Json::writeString(writer, json_data);

    uint32_t msg_type = static_cast<uint32_t>(MessageType::PROJECT_STATUS);
    std::vector<uint8_t> packet(sizeof(msg_type) + json_str.size());
    memcpy(packet.data(), &msg_type, sizeof(msg_type));
    memcpy(packet.data() + sizeof(msg_type), json_str.data(), json_str.size());

    // 发送到所有连接的客户端
    broadcast_to_clients(packet);

    if (bShowRunInfo_) {
      LOG_INFO("Broadcast project status: %s - %s",
                  msg->project_name.c_str(), msg->message.c_str());
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error processing project status: %s", e.what());
  }
}

void UDPServerNode::handle_project_control(
  const std::shared_ptr<custom_msgs_comm::srv::ProjectControl::Request> request,
  std::shared_ptr<custom_msgs_comm::srv::ProjectControl::Response> response)
{
  try {
    // 创建项目指令消息
    auto command_msg = std::make_shared<custom_msgs_comm::msg::ProjectCommand>();
    command_msg->command_type = request->command;
    command_msg->project_name = request->project_name;

    // 发布到ROS话题
    project_command_pub_->publish(*command_msg);

    // 打包指令发送到客户端
    Json::Value json_data;
    json_data["command_type"] = request->command;
    json_data["project_name"] = request->project_name;

    Json::StreamWriterBuilder writer;
    std::string json_str = Json::writeString(writer, json_data);

    uint32_t msg_type = static_cast<uint32_t>(MessageType::PROJECT_COMMAND);
    std::vector<uint8_t> packet(sizeof(msg_type) + json_str.size());
    memcpy(packet.data(), &msg_type, sizeof(msg_type));
    memcpy(packet.data() + sizeof(msg_type), json_str.data(), json_str.size());

    // 发送到默认客户端
    send_to_client(packet, client_address_);

    response->success = true;
    response->message = "Project control command sent successfully";
    // 初始化ProjectStatus消息
    response->status.status = custom_msgs_comm::msg::ProjectStatus::STATUS_STARTING;
    response->status.project_name = request->project_name;

    if (bShowRunInfo_) {
      LOG_INFO("Project control command sent: %s - type %d",
                  request->project_name.c_str(), request->command);
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error handling project control: %s", e.what());
    response->success = false;
    response->message = std::string("Error: ") + e.what();
  }
}

void UDPServerNode::handle_project_status(
  const std::shared_ptr<custom_msgs_comm::srv::ProjectControl::Request> request,
  std::shared_ptr<custom_msgs_comm::srv::ProjectControl::Response> response)
{
  try {
    // 这里应该实现查询项目状态的逻辑
    // 目前返回模拟数据
    response->status.status = custom_msgs_comm::msg::ProjectStatus::STATUS_RUNNING;
    response->status.project_name = request->project_name;
    response->message = "Project is running normally";
    response->success = true;

    if (bShowRunInfo_) {
      LOG_INFO("Project status query: %s", request->project_name.c_str());
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error handling project status: %s", e.what());
    response->status.status = custom_msgs_comm::msg::ProjectStatus::STATUS_ERROR;
    response->status.project_name = request->project_name;
    response->message = "Error querying project status";
    response->success = false;
  }
}

void UDPServerNode::receive_loop()
{
  struct sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);
  uint8_t buffer[65536];  // 最大UDP包大小

  while (is_running_ && rclcpp::ok()) {
    ssize_t recv_len = recvfrom(udp_socket_, buffer, sizeof(buffer), 0,
                               (struct sockaddr*)&client_addr, &addr_len);
    
    if (recv_len < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      LOG_ERROR("Error receiving UDP data: %s", strerror(errno));
      continue;
    }

    if (static_cast<size_t>(recv_len) < sizeof(uint32_t)) {
      continue;
    }

    // 注册客户端
    std::string client_id = get_client_id(client_addr);
    register_client(client_id, client_addr);

    // 处理接收到的数据
    std::vector<uint8_t> data(buffer, buffer + recv_len);
    process_udp_message(data, client_addr);
  }
}

void UDPServerNode::send_loop()
{
  while (is_running_ && rclcpp::ok()) {
    SendItem item;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      if (queue_cv_.wait_for(lock, std::chrono::milliseconds(100), 
                            [this]() { return !send_queue_.empty(); })) {
        item = send_queue_.front();
        send_queue_.pop();
      } else {
        continue;
      }
    }

    // 发送数据
    sendto(udp_socket_, item.data.data(), item.data.size(), 0,
           (struct sockaddr*)&item.addr, sizeof(item.addr));
  }
}

void UDPServerNode::process_udp_message(const std::vector<uint8_t>& data, const struct sockaddr_in& addr)
{
  if (data.size() < sizeof(uint32_t)) {
    return;
  }

  uint32_t msg_type;
  memcpy(&msg_type, data.data(), sizeof(msg_type));

  std::string payload(reinterpret_cast<const char*>(data.data() + sizeof(msg_type)), 
                     data.size() - sizeof(msg_type));

  switch (static_cast<MessageType>(msg_type)) {
    case MessageType::AI_COORDINATE:
      process_ai_coordinate_data(payload, addr);
      break;
    case MessageType::LASERSCAN:
      process_laserscan_data(data, addr);
      break;
    case MessageType::POINTCLOUD:
      process_pointcloud_data(data, addr);
      break;
    case MessageType::CALIBRATION_RESPONSE:
      process_calibration_response(payload, addr);
      break;
    case MessageType::PROJECT_STATUS:
      process_project_status(payload, addr);
      break;
    default:
      if (bShowRunInfo_) {
        LOG_WARN("Unknown message type: %u", msg_type);
      }
      break;
  }
}

void UDPServerNode::process_ai_coordinate_data(const std::string& json_data, const struct sockaddr_in& addr)
{
  try {
    Json::Value json_obj;
    Json::CharReaderBuilder reader;
    std::string errors;
    std::istringstream data_stream(json_data);
    
    if (Json::parseFromStream(reader, data_stream, &json_obj, &errors)) {
      auto msg = std::make_shared<custom_msgs_comm::msg::AICoordinateData>();
      msg->object_class = json_obj["object_class"].asString();
      
      // 解析位置数据
      if (json_obj.isMember("position") && json_obj["position"].isArray()) {
        for (const auto& val : json_obj["position"]) {
          msg->position.push_back(val.asFloat());
        }
      }
      
      // 解析方向数据
      if (json_obj.isMember("orientation") && json_obj["orientation"].isArray()) {
        for (const auto& val : json_obj["orientation"]) {
          msg->orientation.push_back(val.asFloat());
        }
      }
      
      msg->confidence = json_obj["confidence"].asFloat();
      msg->frame_id = json_obj["frame_id"].asString();
      msg->stamp.sec = json_obj["stamp"]["sec"].asInt();
      msg->stamp.nanosec = json_obj["stamp"]["nanosec"].asInt();

      // 发布到ROS话题 - 需要创建AI坐标数据发布者
      if (!ai_coordinate_pub_) {
        ai_coordinate_pub_ = this->create_publisher<custom_msgs_comm::msg::AICoordinateData>(
          "/jetson/ai_coordinates", 10);
      }
      ai_coordinate_pub_->publish(*msg);
      
      // 日志频率控制
      if (bShowRunInfo_ && ++ai_log_counter_ % log_interval_ == 0) {
        LOG_INFO("Published AI coordinate data from client");
      }
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error processing AI coordinate data: %s", e.what());
  }
  
  // 添加未使用的变量抑制警告
  (void)addr;
}

void UDPServerNode::process_laserscan_data(const std::vector<uint8_t>& data, const struct sockaddr_in& addr)
{
  try {
    size_t offset = sizeof(uint32_t); // 跳过消息类型前缀
    
    // 检查数据长度是否足够包含基本字段
    if (data.size() < sizeof(uint32_t) + sizeof(float) * 7 + sizeof(uint32_t) * 2) {
      LOG_ERROR("Insufficient data for laser scan message");
      return;
    }
    
    // 解析激光扫描数据 (按照Python版本中的格式)
    float angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max;
    
    // 解析7个float值
    memcpy(&angle_min, data.data() + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&angle_max, data.data() + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&angle_increment, data.data() + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&time_increment, data.data() + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&scan_time, data.data() + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&range_min, data.data() + offset, sizeof(float));
    offset += sizeof(float);
    memcpy(&range_max, data.data() + offset, sizeof(float));
    offset += sizeof(float);
    
    // 解析frame_id长度和内容
    uint32_t frame_id_len;
    memcpy(&frame_id_len, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    if (offset + frame_id_len > data.size()) {
      LOG_ERROR("Frame ID length exceeds available data");
      return;
    }
    
    std::string frame_id(reinterpret_cast<const char*>(data.data() + offset), frame_id_len);
    offset += frame_id_len;
    
    // 解析时间戳
    uint32_t stamp_sec, stamp_nanosec;
    memcpy(&stamp_sec, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(&stamp_nanosec, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // 解析ranges数组长度和内容
    uint32_t ranges_len;
    memcpy(&ranges_len, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    if (offset + ranges_len * sizeof(float) > data.size()) {
      LOG_ERROR("Ranges data exceeds available data");
      return;
    }
    
    std::vector<float> ranges(ranges_len);
    memcpy(ranges.data(), data.data() + offset, ranges_len * sizeof(float));
    offset += ranges_len * sizeof(float);
    
    // 解析intensities数组长度和内容
    uint32_t intensities_len;
    memcpy(&intensities_len, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    std::vector<float> intensities(intensities_len);
    if (intensities_len > 0) {
      if (offset + intensities_len * sizeof(float) > data.size()) {
        LOG_ERROR("Intensities data exceeds available data");
        return;
      }
      memcpy(intensities.data(), data.data() + offset, intensities_len * sizeof(float));
    }
    
    // 创建ROS消息
    auto msg = std::make_shared<custom_msgs_comm::msg::LaserScanData>();
    msg->header.frame_id = frame_id;
    msg->header.stamp.sec = stamp_sec;
    msg->header.stamp.nanosec = stamp_nanosec;
    msg->angle_min = angle_min;
    msg->angle_max = angle_max;
    msg->angle_increment = angle_increment;
    msg->time_increment = time_increment;
    msg->scan_time = scan_time;
    msg->range_min = range_min;
    msg->range_max = range_max;
    msg->ranges = ranges;
    msg->intensities = intensities;
    
    // 发布到ROS话题
    laserscan_pub_->publish(*msg);
    
    // 日志频率控制
    if (bShowRunInfo_ && ++laser_log_counter_ % log_interval_ == 0) {
      LOG_INFO("Published laser scan data from client");
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error processing laser scan data: %s", e.what());
  }
}

void UDPServerNode::process_pointcloud_data(const std::vector<uint8_t>& data, const struct sockaddr_in& addr)
{
  try {
    size_t offset = sizeof(uint32_t); // 跳过消息类型前缀
    
    // 检查数据长度是否足够包含基本字段
    if (data.size() < sizeof(uint32_t) + sizeof(uint32_t) * 3 + sizeof(uint32_t) * 2 + sizeof(float) * 9 + sizeof(uint32_t)) {
      LOG_ERROR("Insufficient data for point cloud message");
      return;
    }
    
    // 解析点云数据 (按照客户端发送的格式)
    uint32_t height, width, point_step;
    
    // 解析height, width, point_step (3个uint32_t)
    memcpy(&height, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(&width, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(&point_step, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // 解析时间戳
    uint32_t stamp_sec, stamp_nanosec;
    memcpy(&stamp_sec, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(&stamp_nanosec, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // 计算UDP通信耗时
    auto receive_time = std::chrono::high_resolution_clock::now();
    auto receive_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      receive_time.time_since_epoch()).count();
    int64_t send_ns = static_cast<int64_t>(stamp_sec) * 1000000000LL + stamp_nanosec;
    int64_t latency_ns = receive_ns - send_ns;
    double latency_ms = latency_ns / 1000000.0;
    
    // 解析相机内参 (9个float值)
    std::vector<float> camera_intrinsic(9);
    for (int i = 0; i < 9; i++) {
      memcpy(&camera_intrinsic[i], data.data() + offset, sizeof(float));
      offset += sizeof(float);
    }
    
    // 解析frame_id长度和内容
    uint32_t frame_id_len;
    memcpy(&frame_id_len, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    if (offset + frame_id_len > data.size()) {
      LOG_ERROR("Frame ID length exceeds available data");
      return;
    }
    
    std::string frame_id(reinterpret_cast<const char*>(data.data() + offset), frame_id_len);
    offset += frame_id_len;
    
    // 解析数据长度和内容
    uint32_t data_len;
    memcpy(&data_len, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    if (offset + data_len > data.size()) {
      LOG_ERROR("Point cloud data exceeds available data");
      return;
    }
    
    std::vector<uint8_t> point_data(data_len);
    memcpy(point_data.data(), data.data() + offset, data_len);
    
    // 创建ROS消息
    auto msg = std::make_shared<custom_msgs_comm::msg::PointCloudData>();
    msg->header.frame_id = frame_id;
    msg->header.stamp.sec = stamp_sec;
    msg->header.stamp.nanosec = stamp_nanosec;
    msg->height = height;
    msg->width = width;
    msg->point_step = point_step;
    
    // 设置点云数据
    msg->data = point_data;
    
    if (0)
    {
      LOG_INFO("Received point cloud data with size: %d", data.size());
      LOG_INFO("Received frame_id: %s, stamp_sec: %d, stamp_nanosec: %d", frame_id.c_str(), stamp_sec, stamp_nanosec);
      LOG_INFO("Published pointcloud data with height: %d, width: %d, point_step: %d", height, width, point_step);
    }
    
    pointcloud_pub_->publish(*msg);
    
    // 日志频率控制 - 包含UDP通信耗时
    if (bShowRunInfo_ && ++pointcloud_log_counter_ % log_interval_ == 0) {
      LOG_INFO("Published pointcloud data from client, UDP耗时: %.3f ms", latency_ms);
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error processing pointcloud data: %s", e.what());
  }
}

void UDPServerNode::process_calibration_response(const std::string& json_data, const struct sockaddr_in& addr)
{
  try {
    // Python版本中CalibrationResponse是二进制格式，不是JSON格式
    // 根据Python代码，格式为: response_type(1B) + aruco_pixel(2 * 4B) + robot_pose(6 * 4B)
    
    if (json_data.size() < 1) {
      LOG_ERROR("Insufficient data for calibration response");
      return;
    }
    
    // 将字符串转换为字节向量以进行二进制解析
    std::vector<uint8_t> data(json_data.begin(), json_data.end());
    size_t offset = 0;
    
    // 解析响应类型 (1字节)
    uint8_t response_type = data[offset];
    offset += 1;
    
    // 检查是否为完成响应
    if (response_type == 0 || response_type == 1) { // RES_COMPLETED_SUCCESS 或 RES_COMPLETED_FAILED
      // 这里假设0是成功，1是失败，或者根据实际消息定义调整
      auto msg = std::make_shared<custom_msgs_comm::msg::CalibrationResponse>();
      msg->success = (response_type == 0); // 假设0表示成功
      msg->message = (response_type == 0) ? "Calibration completed successfully" : "Calibration failed";
      msg->timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
      
      // 发布到ROS话题 - 需要创建标定响应发布者
      if (!calibration_response_pub_) {
        calibration_response_pub_ = this->create_publisher<custom_msgs_comm::msg::CalibrationResponse>(
          "/calibration/response", 10);
      }
      calibration_response_pub_->publish(*msg);
      
      if (bShowRunInfo_) {
        LOG_INFO("Published calibration response from client, type: %d", response_type);
      }
      return;
    }
    
    // 检查是否有足够的数据解析常规响应
    if (data.size() < offset + 8 + 24) { // 1B type + 8B aruco_pixel + 24B robot_pose
      LOG_ERROR("Insufficient data for calibration response with pose data");
      return;
    }
    
    // 解析aruco像素坐标 (2个float)
    float aruco_pixel[2];
    memcpy(aruco_pixel, data.data() + offset, sizeof(float) * 2);
    offset += sizeof(float) * 2;
    
    // 解析机器人位姿 (6个float)
    float robot_pose[6];
    memcpy(robot_pose, data.data() + offset, sizeof(float) * 6);
    offset += sizeof(float) * 6;
    
    // 创建ROS消息
    auto msg = std::make_shared<custom_msgs_comm::msg::CalibrationResponse>();
    msg->success = true; // 假设中间步骤是成功的
    msg->message = "Calibration response received";
    msg->timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    
    // 注意：custom_msgs_comm::msg::CalibrationResponse 没有 aruco_pixel_pose 和 robot_pose 字段
    // 我们只设置已定义的字段
    
    // 发布到ROS话题 - 需要创建标定响应发布者
    if (!calibration_response_pub_) {
      calibration_response_pub_ = this->create_publisher<custom_msgs_comm::msg::CalibrationResponse>(
        "/calibration/response", 10);
      }
    calibration_response_pub_->publish(*msg);
    
    if (bShowRunInfo_) {
      LOG_INFO("Published calibration response from client, type: %d", response_type);
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error processing calibration response: %s", e.what());
  }
  
  // 添加未使用的变量抑制警告
  (void)addr;
}

void UDPServerNode::process_project_status(const std::string& json_data, const struct sockaddr_in& addr)
{
  try {
    Json::Value json_obj;
    Json::CharReaderBuilder reader;
    std::string errors;
    std::istringstream data_stream(json_data);
    
    if (Json::parseFromStream(reader, data_stream, &json_obj, &errors)) {
      auto msg = std::make_shared<custom_msgs_comm::msg::ProjectStatus>();
      msg->status = json_obj["status"].asUInt();
      msg->project_name = json_obj["project_name"].asString();
      msg->message = json_obj["message"].asString();

      // 发布到ROS话题 - 需要创建项目状态发布者
      if (!project_status_pub_) {
        project_status_pub_ = this->create_publisher<custom_msgs_comm::msg::ProjectStatus>(
          "/project/status", 10);
      }
      project_status_pub_->publish(*msg);
      
      if (bShowRunInfo_) {
        LOG_INFO("Published project status from client: %s - %s",
                    msg->project_name.c_str(), msg->message.c_str());
      }
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Error processing project status: %s", e.what());
  }
  
  // 添加未使用的变量抑制警告
  (void)addr;
}

void UDPServerNode::send_to_client(const std::vector<uint8_t>& data, const struct sockaddr_in& addr)
{
  SendItem item;
  item.data = data;
  item.addr = addr;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    send_queue_.push(item);
  }
  queue_cv_.notify_one();
}

void UDPServerNode::broadcast_to_clients(const std::vector<uint8_t>& data)
{
  std::lock_guard<std::mutex> lock(clients_mutex_);
  
  for (const auto& client : connected_clients_) {
    SendItem item;
    item.data = data;
    item.addr = client.second;

    {
      std::lock_guard<std::mutex> queue_lock(queue_mutex_);
      send_queue_.push(item);
    }
  }
  
  queue_cv_.notify_one();
}

void UDPServerNode::set_udp_blocking_mode(bool blocking)
{
  int flags = fcntl(udp_socket_, F_GETFL, 0);
  if (flags < 0) {
    LOG_ERROR("Failed to get socket flags");
    return;
  }

  if (blocking) {
    flags &= ~O_NONBLOCK;
  } else {
    flags |= O_NONBLOCK;
  }

  if (fcntl(udp_socket_, F_SETFL, flags) < 0) {
    LOG_ERROR("Failed to set socket blocking mode");
  }
}

void UDPServerNode::register_client(const std::string& client_id, const struct sockaddr_in& addr)
{
  std::lock_guard<std::mutex> lock(clients_mutex_);
  connected_clients_[client_id] = addr;
  
  if (bShowRunInfo_) {
    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &addr.sin_addr, ip_str, INET_ADDRSTRLEN);
    LOG_INFO("Client connected: %s:%d", ip_str, ntohs(addr.sin_port));
  }
}

void UDPServerNode::unregister_client(const std::string& client_id)
{
  std::lock_guard<std::mutex> lock(clients_mutex_);
  auto it = connected_clients_.find(client_id);
  if (it != connected_clients_.end()) {
    if (bShowRunInfo_) {
      char ip_str[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &it->second.sin_addr, ip_str, INET_ADDRSTRLEN);
      LOG_INFO("Client disconnected: %s:%d", ip_str, ntohs(it->second.sin_port));
    }
    connected_clients_.erase(it);
  }
}

std::string UDPServerNode::get_client_id(const struct sockaddr_in& addr)
{
  char ip_str[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &addr.sin_addr, ip_str, INET_ADDRSTRLEN);
  return std::string(ip_str) + ":" + std::to_string(ntohs(addr.sin_port));
}

}  // namespace udp_comm_server