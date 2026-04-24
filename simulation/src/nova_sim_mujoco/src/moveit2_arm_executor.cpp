// MoveIt2ArmExecutorCpp：GetPositionIK 请求构造、关节顺序与夹爪开合阈值。
#include "nova_sim/moveit2_arm_executor.hpp"

#include <chrono>
#include <cmath>

#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/position_ik_request.hpp"
#include "moveit_msgs/msg/robot_state.hpp"

#include "calib_sim_mujoco/msg/arm_pose.hpp"

using namespace std::chrono_literals;

MoveIt2ArmExecutorCpp::MoveIt2ArmExecutorCpp()
: Node("moveit2_arm_executor_cpp"), arm_id_(0)
{
  // 与当前 nova_moveit_config 中 SRDF 的 group / 末端 link 命名保持一致。
  arm_groups_ = {{0, "l_arm"}, {1, "r_arm"}, {2, "j3_arm"}, {3, "j4_arm"}};
  ee_links_ = {{0, "J1_6"}, {1, "J2_6"}, {2, "J3_6"}, {3, "J4_6"}};
  arm_prefix_ = {{0, "J1_"}, {1, "J2_"}, {2, "J3_"}, {3, "J4_"}};

  control_joint_order_ = {
    "J1_1_joint", "J1_2_joint", "J1_3_joint", "J1_4_joint", "J1_5_joint", "J1_6_joint", "J1_7_joint",
    "J1_8_joint", "J2_1_joint", "J2_2_joint", "J2_3_joint", "J2_4_joint", "J2_5_joint", "J2_6_joint",
    "J2_7_joint", "J2_8_joint", "J3_1_joint", "J3_2_joint", "J3_3_joint", "J3_4_joint", "J3_5_joint",
    "J3_6_joint", "J4_1_joint", "J4_2_joint", "J4_3_joint", "J4_4_joint", "J4_5_joint", "J4_6_joint"};

  gripper_joints_ = {{0, {"J1_7_joint", "J1_8_joint"}}, {1, {"J2_7_joint", "J2_8_joint"}}};
  gripper_open_ = {{0, {0.02, -0.02}}, {1, {0.02, -0.02}}};
  gripper_close_ = {{0, {-0.04, 0.04}}, {1, {-0.04, 0.04}}};

  ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
  command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_controller/commands", 10);
  pose_log_pub_ = this->create_publisher<std_msgs::msg::String>("/nova_pose_log", 20);

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 50, std::bind(&MoveIt2ArmExecutorCpp::on_joint_state, this, std::placeholders::_1));
  arm_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/nova_arm_id", 10, std::bind(&MoveIt2ArmExecutorCpp::on_arm_id, this, std::placeholders::_1));
  target_arm_pose_sub_ = this->create_subscription<calib_sim_mujoco::msg::ArmPose>(
    "/nova_target_arm_pose", 10, std::bind(&MoveIt2ArmExecutorCpp::on_target_arm_pose, this, std::placeholders::_1));
  gripper_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/nova_gripper_goal", 10, std::bind(&MoveIt2ArmExecutorCpp::on_gripper_goal, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "moveit2_arm_executor_cpp ready.");
  RCLCPP_INFO(
    this->get_logger(),
    "[nova_executor_v2] IK request mode: seeded by /joint_states, groups(l_arm/r_arm).");
  if (!ik_client_->wait_for_service(200ms)) {
    RCLCPP_WARN(
      this->get_logger(),
      "/compute_ik not ready now. Joint/gripper control still works; pose control waits for MoveIt.");
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Pose goals: calib_sim_mujoco/ArmPose on /nova_target_arm_pose only "
    "(/nova_arm_id + /nova_target_pose 不再被此节点订阅，避免错序). + optional /nova_gripper_goal.");
}

void MoveIt2ArmExecutorCpp::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    current_joint_map_[msg->name[i]] = msg->position[i];
  }
}

void MoveIt2ArmExecutorCpp::on_arm_id(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (arm_groups_.count(msg->data) == 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid arm_id=%d for pose IK. Current MoveIt config supports arm_id 0/1/2/3.",
      msg->data);
    return;
  }
  arm_id_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "Set active arm_id=%d", arm_id_);
}

void MoveIt2ArmExecutorCpp::on_target_arm_pose(const calib_sim_mujoco::msg::ArmPose::SharedPtr msg)
{
  process_pose_goal(msg->arm_id, msg->pose);
}

void MoveIt2ArmExecutorCpp::process_pose_goal(
  int arm_id, const geometry_msgs::msg::PoseStamped & pose)
{
  if (arm_groups_.count(arm_id) == 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid arm_id=%d for pose IK. Current MoveIt config supports arm_id 0/1/2/3.",
      arm_id);
    return;
  }

  if (!ik_client_->wait_for_service(100ms)) {
    RCLCPP_ERROR(this->get_logger(), "Pose goal rejected: /compute_ik service is unavailable.");
    std_msgs::msg::String log_msg;
    log_msg.data = "[ERROR] Pose goal rejected: /compute_ik service is unavailable.";
    pose_log_pub_->publish(log_msg);
    return;
  }

  if (current_joint_map_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No /joint_states yet, ignore pose goal.");
    std_msgs::msg::String log_msg;
    log_msg.data = "[ERROR] No /joint_states yet, ignore pose goal.";
    pose_log_pub_->publish(log_msg);
    return;
  }

  auto ik_req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  moveit_msgs::msg::PositionIKRequest ikr;
  ikr.group_name = arm_groups_[arm_id];
  ikr.ik_link_name = ee_links_[arm_id];
  ikr.pose_stamped = pose;
  const double qx = ikr.pose_stamped.pose.orientation.x;
  const double qy = ikr.pose_stamped.pose.orientation.y;
  const double qz = ikr.pose_stamped.pose.orientation.z;
  const double qw = ikr.pose_stamped.pose.orientation.w;
  const double qnorm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (qnorm < 1e-8) {
    RCLCPP_ERROR(this->get_logger(), "Pose goal rejected: invalid quaternion norm=%g", qnorm);
    std_msgs::msg::String log_msg;
    log_msg.data = "[ERROR] Pose goal rejected: invalid quaternion norm.";
    pose_log_pub_->publish(log_msg);
    return;
  }
  ikr.pose_stamped.pose.orientation.x = qx / qnorm;
  ikr.pose_stamped.pose.orientation.y = qy / qnorm;
  ikr.pose_stamped.pose.orientation.z = qz / qnorm;
  ikr.pose_stamped.pose.orientation.w = qw / qnorm;
  ikr.timeout.sec = 1;
  ikr.timeout.nanosec = 0;
  // For current bring-up phase, prioritize solvability first.
  ikr.avoid_collisions = false;

  // Seed IK with current arm joint values; avoid empty JointState in MoveIt conversions.
  sensor_msgs::msg::JointState seed;
  seed.name.reserve(current_joint_map_.size());
  seed.position.reserve(current_joint_map_.size());
  for (const auto & kv : current_joint_map_) {
    if (kv.first.rfind("J1_", 0) == 0 || kv.first.rfind("J2_", 0) == 0 ||
      kv.first.rfind("J3_", 0) == 0 || kv.first.rfind("J4_", 0) == 0)
    {
      seed.name.push_back(kv.first);
      seed.position.push_back(kv.second);
    }
  }
  if (seed.name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "IK seed is empty, wait /joint_states update.");
    std_msgs::msg::String log_msg;
    log_msg.data = "[ERROR] IK seed is empty, wait /joint_states update.";
    pose_log_pub_->publish(log_msg);
    return;
  }
  ikr.robot_state.joint_state = seed;

  // Do not inject external joint constraints here.
  // xtrainer_moveit_config uses a different naming set, forcing unknown joints will fail IK request.
  ik_req->ik_request = ikr;
  RCLCPP_INFO(
    this->get_logger(),
    "[nova_executor_v2] send IK arm_id=%d group=%s link=%s frame=%s",
    arm_id,
    ikr.group_name.c_str(),
    ikr.ik_link_name.c_str(),
    ikr.pose_stamped.header.frame_id.c_str());

  const int request_arm_id = arm_id;
  const auto request_prefix = arm_prefix_[request_arm_id];
  ik_client_->async_send_request(
    ik_req,
    [this, request_arm_id, request_prefix](
      rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
      auto result = future.get();
      if (result->error_code.val != 1) {
        RCLCPP_ERROR(
          this->get_logger(), "IK failed for arm_id=%d, code=%d", request_arm_id, result->error_code.val);
        std_msgs::msg::String log_msg;
        log_msg.data = "[ERROR] IK failed for arm_id=" + std::to_string(request_arm_id) +
          ", code=" + std::to_string(result->error_code.val);
        pose_log_pub_->publish(log_msg);
        return;
      }

      std::unordered_map<std::string, double> command_map = current_joint_map_;
      const auto & names = result->solution.joint_state.name;
      const auto & pos = result->solution.joint_state.position;
      for (size_t i = 0; i < names.size() && i < pos.size(); ++i) {
        std::string target_name = names[i];
        // Map MoveIt naming (R1-1/R2-3...) to controller naming (J1_1_joint/J2_3_joint...)
        if (target_name.rfind("R1-", 0) == 0 && target_name.size() > 3) {
          target_name = "J1_" + target_name.substr(3) + "_joint";
        } else if (target_name.rfind("R2-", 0) == 0 && target_name.size() > 3) {
          target_name = "J2_" + target_name.substr(3) + "_joint";
        }

        if (target_name.rfind(request_prefix, 0) == 0) {
          command_map[target_name] = pos[i];
        }
      }
      publish_command(command_map);
      arm_id_ = request_arm_id;
      RCLCPP_INFO(this->get_logger(), "Pose command sent for arm_id=%d", request_arm_id);
      std_msgs::msg::String log_msg;
      log_msg.data = "[INFO] Pose command sent for arm_id=" + std::to_string(request_arm_id);
      pose_log_pub_->publish(log_msg);
    });
}

void MoveIt2ArmExecutorCpp::on_gripper_goal(const std_msgs::msg::String::SharedPtr msg)
{
  if (gripper_joints_.count(arm_id_) == 0) {
    RCLCPP_WARN(this->get_logger(), "arm_id=%d has no gripper", arm_id_);
    return;
  }
  std::unordered_map<std::string, double> command_map = current_joint_map_;
  const auto joints = gripper_joints_[arm_id_];
  double p0 = 0.0;
  double p1 = 0.0;
  if (msg->data == "open") {
    p0 = gripper_open_[arm_id_].first;
    p1 = gripper_open_[arm_id_].second;
  } else if (msg->data == "close") {
    p0 = gripper_close_[arm_id_].first;
    p1 = gripper_close_[arm_id_].second;
  } else if (msg->data.rfind("width:", 0) == 0) {
    const auto value = std::stod(msg->data.substr(6));
    const auto width = std::max(0.0, std::min(0.06, value));
    const auto ratio = width / 0.06;
    p0 = gripper_close_[arm_id_].first +
      (gripper_open_[arm_id_].first - gripper_close_[arm_id_].first) * ratio;
    p1 = gripper_close_[arm_id_].second +
      (gripper_open_[arm_id_].second - gripper_close_[arm_id_].second) * ratio;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Bad /nova_gripper_goal: use open|close|width:<m>");
    return;
  }
  command_map[joints.first] = p0;
  command_map[joints.second] = p1;
  publish_command(command_map);
  RCLCPP_INFO(this->get_logger(), "Gripper command sent for arm_id=%d", arm_id_);
}

void MoveIt2ArmExecutorCpp::publish_command(const std::unordered_map<std::string, double> & command_map)
{
  constexpr double kJointDeadband = 3e-4;
  bool has_meaningful_change = !has_last_command_;
  if (!has_meaningful_change) {
    for (const auto & joint : control_joint_order_) {
      const double new_val = resolve_command_value(joint, command_map);
      const auto it_old = last_published_command_map_.find(joint);
      const double old_val = (it_old == last_published_command_map_.end()) ? 0.0 : it_old->second;
      if (std::abs(new_val - old_val) > kJointDeadband) {
        has_meaningful_change = true;
        break;
      }
    }
  }

  if (!has_meaningful_change) {
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data.reserve(control_joint_order_.size());
  for (const auto & joint : control_joint_order_) {
    const double value = resolve_command_value(joint, command_map);
    msg.data.push_back(value);
    last_published_command_map_[joint] = value;
  }
  has_last_command_ = true;
  command_pub_->publish(msg);
}

double MoveIt2ArmExecutorCpp::resolve_command_value(
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
  const auto it_cur = current_joint_map_.find(joint);
  if (it_cur != current_joint_map_.end()) {
    return it_cur->second;
  }
  return 0.0;
}
