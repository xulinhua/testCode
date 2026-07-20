#include "nova_grasp_moveit/grasp_arm_executor.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>

#include "moveit_msgs/msg/position_ik_request.hpp"
#include "moveit_msgs/msg/robot_state.hpp"

using namespace std::chrono_literals;

namespace nova_grasp_moveit
{

GraspArmExecutor::GraspArmExecutor()
: Node("grasp_arm_executor"), arm_id_(0)
{
  arm_groups_ = {{0, "l_arm"}, {1, "r_arm"}};
  ee_links_ = {{0, "J1_6"}, {1, "J2_6"}};
  arm_prefix_ = {{0, "J1_"}, {1, "J2_"}};

  // 仅双臂；Isaac ArticulationController 认 /joint_command
  control_joint_order_ = {
    "J1_1_joint", "J1_2_joint", "J1_3_joint", "J1_4_joint", "J1_5_joint", "J1_6_joint", "J1_7_joint",
    "J1_8_joint", "J2_1_joint", "J2_2_joint", "J2_3_joint", "J2_4_joint", "J2_5_joint", "J2_6_joint",
    "J2_7_joint", "J2_8_joint"};

  gripper_joints_ = {{0, {"J1_7_joint", "J1_8_joint"}}, {1, {"J2_7_joint", "J2_8_joint"}}};
  // 与 grasp_planner.hpp 一致：两指反向极限，对中开合
  gripper_open_ = {{0, {0.02, -0.02}}, {1, {0.02, -0.02}}};
  gripper_close_ = {{0, {-0.04, 0.04}}, {1, {-0.04, 0.04}}};

  declare_parameter<std::string>("target_arm_pose_topic", "/nova_target_arm_pose");
  declare_parameter<std::string>("gripper_topic", "/nova_gripper_goal");
  declare_parameter<std::string>("pose_log_topic", "/nova_pose_log");
  declare_parameter<std::string>("arm_id_topic", "/nova_arm_id");
  declare_parameter<std::string>("joint_command_topic", "/joint_command");
  declare_parameter<int>("joint_command_burst_count", 5);
  declare_parameter<bool>("publish_mujoco_joint_array", true);
  joint_command_burst_count_ = std::max(1, static_cast<int>(get_parameter("joint_command_burst_count").as_int()));
  publish_mujoco_joint_array_ = get_parameter("publish_mujoco_joint_array").as_bool();

  ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
  if (publish_mujoco_joint_array_) {
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/arm_controller/commands", 10);
  }
  auto joint_cmd_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  joint_cmd_qos.reliable();
  joint_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    get_parameter("joint_command_topic").as_string(), joint_cmd_qos);
  const auto pose_log_topic = get_parameter("pose_log_topic").as_string();
  pose_log_pub_ = create_publisher<std_msgs::msg::String>(pose_log_topic, 20);

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 50, std::bind(&GraspArmExecutor::on_joint_state, this, std::placeholders::_1));
  arm_id_sub_ = create_subscription<std_msgs::msg::Int32>(
    get_parameter("arm_id_topic").as_string(), 10,
    std::bind(&GraspArmExecutor::on_arm_id, this, std::placeholders::_1));
  target_arm_pose_sub_ = create_subscription<nova_grasp_moveit::msg::ArmPose>(
    get_parameter("target_arm_pose_topic").as_string(), 10,
    std::bind(&GraspArmExecutor::on_target_arm_pose, this, std::placeholders::_1));
  gripper_sub_ = create_subscription<std_msgs::msg::String>(
    get_parameter("gripper_topic").as_string(), 10,
    std::bind(&GraspArmExecutor::on_gripper_goal, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "grasp_arm_executor ready.");
  if (!ik_client_->wait_for_service(200ms)) {
    RCLCPP_WARN(
      get_logger(),
      "/compute_ik not ready yet (expect nova_grasp_moveit move_group in grasp_stack).");
  }
  RCLCPP_INFO(
    get_logger(),
    "Sub: %s, %s; pub: %s%s",
    get_parameter("target_arm_pose_topic").as_string().c_str(),
    get_parameter("gripper_topic").as_string().c_str(),
    get_parameter("joint_command_topic").as_string().c_str(),
    publish_mujoco_joint_array_ ? " + /arm_controller/commands" : "");
}

void GraspArmExecutor::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(joint_mu_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    current_joint_map_[msg->name[i]] = msg->position[i];
  }
}

void GraspArmExecutor::on_arm_id(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (arm_groups_.count(msg->data) == 0) {
    RCLCPP_ERROR(get_logger(), "Invalid arm_id=%d", msg->data);
    return;
  }
  arm_id_ = msg->data;
}

void GraspArmExecutor::publish_pose_log(const std::string & line)
{
  std_msgs::msg::String log_msg;
  log_msg.data = line;
  pose_log_pub_->publish(log_msg);
}

void GraspArmExecutor::on_target_arm_pose(const nova_grasp_moveit::msg::ArmPose::SharedPtr msg)
{
  process_pose_goal(msg->arm_id, msg->pose);
}

void GraspArmExecutor::process_pose_goal(
  int arm_id, const geometry_msgs::msg::PoseStamped & pose_in)
{
  if (arm_groups_.count(arm_id) == 0) {
    RCLCPP_ERROR(get_logger(), "Invalid arm_id=%d for pose IK", arm_id);
    return;
  }

  if (!ik_client_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Pose goal rejected: /compute_ik unavailable.");
    publish_pose_log("[ERROR] Pose goal rejected: /compute_ik service is unavailable.");
    return;
  }

  // 每次逆解前先拷贝最新 /joint_states 作种子（走完一步后必须用实测角，不能用旧指令）
  std::unordered_map<std::string, double> joint_snapshot;
  {
    std::lock_guard<std::mutex> lk(joint_mu_);
    if (current_joint_map_.empty()) {
      RCLCPP_ERROR(get_logger(), "No /joint_states yet, ignore pose goal.");
      publish_pose_log("[ERROR] No /joint_states yet, ignore pose goal.");
      return;
    }
    joint_snapshot = current_joint_map_;
  }

  geometry_msgs::msg::PoseStamped pose = pose_in;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = "base_link";
  }

  sensor_msgs::msg::JointState seed;
  const auto request_prefix = arm_prefix_[arm_id];
  const int request_arm_id = arm_id;
  std::ostringstream seed_oss;
  seed_oss << std::fixed << std::setprecision(3)
           << "[INFO] IK seed (current joints) arm=" << arm_id;
  for (const auto & kv : joint_snapshot) {
    std::string name = kv.first;
    if (name.rfind("J1_", 0) != 0 && name.rfind("J2_", 0) != 0)
    {
      continue;
    }
    if (name.size() < 6 || name.compare(name.size() - 6, 6, "_joint") != 0) {
      name += "_joint";
    }
    seed.name.push_back(name);
    seed.position.push_back(kv.second);
    if (name.rfind(request_prefix, 0) == 0) {
      seed_oss << " " << name << "=" << (kv.second * 180.0 / M_PI) << "deg";
    }
  }
  if (seed.name.empty()) {
    publish_pose_log("[ERROR] IK seed is empty, wait /joint_states update.");
    return;
  }
  publish_pose_log(seed_oss.str());

  double qx = pose.pose.orientation.x;
  double qy = pose.pose.orientation.y;
  double qz = pose.pose.orientation.z;
  double qw = pose.pose.orientation.w;
  const double qnorm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (qnorm < 1e-8) {
    publish_pose_log("[ERROR] Pose goal rejected: invalid quaternion norm.");
    return;
  }
  qx /= qnorm;
  qy /= qnorm;
  qz /= qnorm;
  qw /= qnorm;
  pose.pose.orientation.x = qx;
  pose.pose.orientation.y = qy;
  pose.pose.orientation.z = qz;
  pose.pose.orientation.w = qw;

  auto ik_req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  moveit_msgs::msg::PositionIKRequest ikr;
  ikr.group_name = arm_groups_[request_arm_id];
  ikr.ik_link_name = ee_links_[request_arm_id];
  ikr.pose_stamped = pose;
  ikr.timeout.sec = 2;
  ikr.timeout.nanosec = 0;
  ikr.avoid_collisions = false;
  ikr.robot_state.joint_state = seed;
  ik_req->ik_request = ikr;

  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "[INFO] IK request arm_id=" << request_arm_id
        << " xyz=(" << pose.pose.position.x << "," << pose.pose.position.y << ","
        << pose.pose.position.z << ") quat_xyzw=("
        << qx << "," << qy << "," << qz << "," << qw << ")";
    publish_pose_log(oss.str());
  }

  auto ctx = std::make_shared<IkRequestContext>();
  ctx->arm_id = request_arm_id;
  ctx->prefix = request_prefix;

  {
    std::lock_guard<std::mutex> lk(ik_ctx_mu_);
    if (ik_ctx_) {
      ik_ctx_->active.store(false);
      if (ik_ctx_->timeout_timer) {
        ik_ctx_->timeout_timer->cancel();
      }
    }
    ik_ctx_ = ctx;
  }

  ctx->timeout_timer = create_wall_timer(
    12s, [this, ctx]() {
      if (!ctx->active.exchange(false)) {
        return;
      }
      if (ctx->timeout_timer) {
        ctx->timeout_timer->cancel();
      }
      publish_pose_log(
        "[ERROR][NG] IK timeout for arm_id=" + std::to_string(ctx->arm_id) +
        " (/compute_ik no response in 12s; check move_group)");
    });

  ik_client_->async_send_request(
    ik_req,
    [this, ctx, request_arm_id, request_prefix](
      rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
      if (!ctx->active.exchange(false)) {
        return;
      }
      if (ctx->timeout_timer) {
        ctx->timeout_timer->cancel();
      }
      handle_ik_response(future, request_arm_id, request_prefix);
    });
}

void GraspArmExecutor::handle_ik_response(
  const rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture & future,
  int request_arm_id, const std::string & request_prefix)
{
  moveit_msgs::srv::GetPositionIK::Response::SharedPtr result;
  try {
    result = future.get();
  } catch (const std::exception & ex) {
    publish_pose_log(
      "[ERROR][NG] IK failed for arm_id=" + std::to_string(request_arm_id) +
      " (service exception: " + ex.what() + ")");
    return;
  }

  if (!result) {
    publish_pose_log(
      "[ERROR][NG] IK failed for arm_id=" + std::to_string(request_arm_id) +
      " (empty /compute_ik response)");
    return;
  }

  if (result->error_code.val != 1) {
    publish_pose_log(
      "[ERROR][NG] IK failed for arm_id=" + std::to_string(request_arm_id) +
      ", code=" + std::to_string(result->error_code.val) +
      " (check reachability / height)");
    return;
  }

  std::unordered_map<std::string, double> command_map;
  const auto & names = result->solution.joint_state.name;
  const auto & pos = result->solution.joint_state.position;
  for (size_t i = 0; i < names.size() && i < pos.size(); ++i) {
    std::string target_name = names[i];
    // 兼容旧 SRDF/控制器返回的 R1-1、R2-1 命名；Isaac 使用 J1_1_joint 格式。
    if (target_name.rfind("R1-", 0) == 0 && target_name.size() > 3) {
      target_name = "J1_" + target_name.substr(3) + "_joint";
    } else if (target_name.rfind("R2-", 0) == 0 && target_name.size() > 3) {
      target_name = "J2_" + target_name.substr(3) + "_joint";
    }
    if (target_name.rfind(request_prefix, 0) == 0) {
      command_map[target_name] = pos[i];
    }
  }
  publish_command(command_map, true);
  publish_pose_joint_debug_lines(command_map, request_arm_id);
  arm_id_ = request_arm_id;
  publish_pose_log(
    "[INFO] Pose command sent for arm_id=" + std::to_string(request_arm_id) +
    " joints=" + std::to_string(command_map.size()) + " (strict single-arm)");
}

void GraspArmExecutor::on_gripper_goal(const std_msgs::msg::String::SharedPtr msg)
{
  if (gripper_joints_.count(arm_id_) == 0) {
    return;
  }
  const auto joints = gripper_joints_[arm_id_];
  double p0 = 0.0;
  double p1 = 0.0;
  if (msg->data == "open") {
    p0 = gripper_open_[arm_id_].first;
    p1 = gripper_open_[arm_id_].second;
  } else if (msg->data == "close") {
    p0 = gripper_close_[arm_id_].first;
    p1 = gripper_close_[arm_id_].second;
  } else {
    RCLCPP_ERROR(get_logger(), "Bad /nova_gripper_goal: use open|close");
    return;
  }
  // 严格单臂夹爪：只发本臂 J7/J8
  std::unordered_map<std::string, double> command_map;
  command_map[joints.first] = p0;
  command_map[joints.second] = p1;
  publish_command(command_map, true);
}

void GraspArmExecutor::publish_command(
  const std::unordered_map<std::string, double> & command_map, bool force)
{
  if (command_map.empty()) {
    return;
  }

  // 实时 UI 可能高频发送相同目标；死区避免无意义消息持续刷新 PhysX drive。
  constexpr double kJointDeadband = 3e-4;
  bool has_meaningful_change = force || !has_last_command_;
  if (!has_meaningful_change) {
    for (const auto & kv : command_map) {
      const auto it_old = last_published_command_map_.find(kv.first);
      const double old_val = (it_old == last_published_command_map_.end()) ? 0.0 : it_old->second;
      if (std::abs(kv.second - old_val) > kJointDeadband) {
        has_meaningful_change = true;
        break;
      }
    }
  }
  if (!has_meaningful_change) {
    return;
  }

  // Isaac /joint_command：仅消息内列出的关节，严格单臂/单夹爪
  sensor_msgs::msg::JointState js_cmd;
  js_cmd.name.reserve(command_map.size());
  js_cmd.position.reserve(command_map.size());
  js_cmd.velocity.reserve(command_map.size());
  for (const auto & kv : command_map) {
    js_cmd.name.push_back(kv.first);
    js_cmd.position.push_back(kv.second);
    // Isaac ArticulationController 同时连接 position/velocity 输入。
    // velocity 为空会保留旧速度目标，造成肩关节固定滞后约 8°。
    js_cmd.velocity.push_back(0.0);
    last_published_command_map_[kv.first] = kv.second;
  }
  has_last_command_ = true;

  // 可选 MuJoCo 数组按 16 DOF 全量（缺省用上次指令/实测），与 Isaac 子集分离。
  if (publish_mujoco_joint_array_ && command_pub_) {
    std_msgs::msg::Float64MultiArray array_msg;
    array_msg.data.reserve(control_joint_order_.size());
    for (const auto & joint : control_joint_order_) {
      array_msg.data.push_back(resolve_command_value(joint, command_map));
    }
    command_pub_->publish(array_msg);
  }

  // Isaac 图启动或仿真步较慢时可能漏掉单帧 ROS 输入，短 burst 提高命令采样可靠性。
  for (int i = 0; i < joint_command_burst_count_; ++i) {
    js_cmd.header.stamp = now();
    joint_command_pub_->publish(js_cmd);
  }
}

void GraspArmExecutor::publish_pose_joint_debug_lines(
  const std::unordered_map<std::string, double> & command_map, int request_arm_id)
{
  const std::array<int, 2> arms{0, 1};
  for (const int arm : arms) {
    std::ostringstream oss;
    oss << "[POSE_DEBUG] req_arm=" << request_arm_id << " arm" << arm << ":";
    const std::string prefix = "J" + std::to_string(arm + 1) + "_";
    bool first = true;
    for (const auto & joint_name : control_joint_order_) {
      if (joint_name.rfind(prefix, 0) != 0) {
        continue;
      }
      if (!first) {
        oss << ",";
      }
      first = false;
      oss << " " << joint_name << "=" << std::fixed << std::setprecision(5)
          << resolve_command_value(joint_name, command_map);
    }
    std_msgs::msg::String msg;
    msg.data = oss.str();
    pose_log_pub_->publish(msg);
  }
}

double GraspArmExecutor::resolve_command_value(
  const std::string & joint, const std::unordered_map<std::string, double> & command_map) const
{
  const auto it = command_map.find(joint);
  if (it != command_map.end()) {
    return it->second;
  }
  const auto it_last = last_published_command_map_.find(joint);
  if (it_last != last_published_command_map_.end()) {
    return it_last->second;
  }
  std::lock_guard<std::mutex> lk(joint_mu_);
  const auto it_cur = current_joint_map_.find(joint);
  if (it_cur != current_joint_map_.end()) {
    return it_cur->second;
  }
  return 0.0;
}

}  // namespace nova_grasp_moveit
