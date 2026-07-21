#include "nova_grasp_moveit/grasp_ros_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <future>
#include <limits>
#include <map>
#include <sstream>
#include <iomanip>
#include <vector>

#include "moveit_msgs/msg/position_ik_request.hpp"
#include "nlohmann/json.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nova_grasp_moveit
{

namespace
{

rclcpp::QoS box_pose_qos()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.reliable();
  qos.transient_local();
  return qos;
}

const std::vector<std::string> kJ1Joints = {
  "J1_1_joint", "J1_2_joint", "J1_3_joint", "J1_4_joint",
  "J1_5_joint", "J1_6_joint", "J1_7_joint", "J1_8_joint"};
const std::vector<std::string> kJ2Joints = {
  "J2_1_joint", "J2_2_joint", "J2_3_joint", "J2_4_joint",
  "J2_5_joint", "J2_6_joint", "J2_7_joint", "J2_8_joint"};
// 仅双臂 16 DOF；勿附带 J3/J4（机上不存在时填 0 可能干扰 ArticulationController）
const std::vector<std::string> kControlJointOrder = {
  "J1_1_joint", "J1_2_joint", "J1_3_joint", "J1_4_joint", "J1_5_joint", "J1_6_joint", "J1_7_joint",
  "J1_8_joint", "J2_1_joint", "J2_2_joint", "J2_3_joint", "J2_4_joint", "J2_5_joint", "J2_6_joint",
  "J2_7_joint", "J2_8_joint"};

const std::vector<std::string> & joints_for_arm(int arm_id)
{
  return arm_id == 0 ? kJ1Joints : kJ2Joints;
}

double rad_to_deg(double rad)
{
  return rad * 180.0 / M_PI;
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w) &&
         (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 1e-12;
}

std::string format_pose_compact(const geometry_msgs::msg::Pose & pose)
{
  std::ostringstream text;
  text << std::fixed << std::setprecision(4)
       << "p=(" << pose.position.x << "," << pose.position.y << "," << pose.position.z << ") "
       << "q=(" << pose.orientation.x << "," << pose.orientation.y << ","
       << pose.orientation.z << "," << pose.orientation.w << ")";
  return text.str();
}

bool parse_graspnet_json(
  const std::string & payload,
  geometry_msgs::msg::PoseArray & output,
  std::vector<double> & scores,
  std::string & error)
{
  try {
    const auto root = nlohmann::json::parse(payload);
    if (!root.is_object()) {
      error = "root is not a JSON object";
      return false;
    }
    if (!root.contains("frame_id") || !root["frame_id"].is_string()) {
      error = "missing string field: frame_id";
      return false;
    }
    if (!root.contains("candidates") || !root["candidates"].is_array()) {
      error = "missing array field: candidates";
      return false;
    }

    output = geometry_msgs::msg::PoseArray();
    scores.clear();
    output.header.frame_id = root["frame_id"].get<std::string>();
    const auto stamp_it = root.find("rgb_stamp");
    if (stamp_it != root.end() && stamp_it->is_object()) {
      output.header.stamp.sec = stamp_it->value("sec", 0);
      output.header.stamp.nanosec = stamp_it->value("nanosec", 0u);
    }

    size_t invalid_count = 0;
    for (const auto & candidate : root["candidates"]) {
      try {
        const auto & translation = candidate.at("translation_m");
        const auto & rotation = candidate.at("rotation_matrix");
        if (!translation.is_array() || translation.size() != 3 ||
          !rotation.is_array() || rotation.size() != 3)
        {
          ++invalid_count;
          continue;
        }
        bool matrix_shape_ok = true;
        for (const auto & row : rotation) {
          matrix_shape_ok = matrix_shape_ok && row.is_array() && row.size() == 3;
        }
        if (!matrix_shape_ok) {
          ++invalid_count;
          continue;
        }

        geometry_msgs::msg::Pose pose;
        pose.position.x = translation.at(0).get<double>();
        pose.position.y = translation.at(1).get<double>();
        pose.position.z = translation.at(2).get<double>();
        tf2::Matrix3x3 matrix(
          rotation.at(0).at(0).get<double>(),
          rotation.at(0).at(1).get<double>(),
          rotation.at(0).at(2).get<double>(),
          rotation.at(1).at(0).get<double>(),
          rotation.at(1).at(1).get<double>(),
          rotation.at(1).at(2).get<double>(),
          rotation.at(2).at(0).get<double>(),
          rotation.at(2).at(1).get<double>(),
          rotation.at(2).at(2).get<double>());
        tf2::Quaternion q;
        matrix.getRotation(q);
        q.normalize();
        pose.orientation = tf2::toMsg(q);
        if (finite_pose(pose)) {
          output.poses.push_back(pose);
          scores.push_back(
            candidate.value("score", std::numeric_limits<double>::quiet_NaN()));
        } else {
          ++invalid_count;
        }
      } catch (const nlohmann::json::exception &) {
        ++invalid_count;
      }
    }
    if (!root["candidates"].empty() && output.poses.empty()) {
      error = "all candidates have invalid translation_m or rotation_matrix";
      return false;
    }
    if (invalid_count > 0) {
      error =
        "skipped " + std::to_string(invalid_count) + " malformed candidate(s)";
    } else {
      error.clear();
    }
    return true;
  } catch (const nlohmann::json::exception & ex) {
    error = ex.what();
    return false;
  }
}

geometry_msgs::msg::Quaternion graspnet_orientation_to_tcp(
  const geometry_msgs::msg::Quaternion & grasp_q_msg)
{
  tf2::Quaternion grasp_q;
  tf2::fromMsg(grasp_q_msg, grasp_q);
  grasp_q.normalize();

  // Empirically, this GraspNet JSON uses local -X as the approach direction
  // (top-down grasps then align with base_link -Z within ~22°, while +X is ~158°).
  // Robot TCP: local +Z points from wrist to fingers / into the object.
  // Map TCP axes into GraspNet axes as Xtcp=Yg, Ytcp=-Zg, Ztcp=-Xg.
  tf2::Matrix3x3 grasp_from_tcp(
    0.0, 0.0, -1.0,
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);
  tf2::Quaternion grasp_to_tcp;
  grasp_from_tcp.getRotation(grasp_to_tcp);
  tf2::Quaternion tcp_q = grasp_q * grasp_to_tcp;
  tcp_q.normalize();
  return tf2::toMsg(tcp_q);
}

/// 绕 TCP +Z（接近轴）转 180°。顶抓时约等于 J*_6 ±180°，开合平面不变、两指对调。
geometry_msgs::msg::Quaternion tcp_quat_rz180(
  const geometry_msgs::msg::Quaternion & tcp_q_msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(tcp_q_msg, q);
  q.normalize();
  tf2::Quaternion rz180(tf2::Vector3(0.0, 0.0, 1.0), M_PI);
  tf2::Quaternion out = q * rz180;
  out.normalize();
  return tf2::toMsg(out);
}

/// 开合轴 = TCP +X（URDF 两指沿 J*_6 ±X 分开）。水平投影 yaw ∈ (-90°, 90°] 则保留，
/// 否则翻 Rz180。因此只有部分候选会动 J6，不是每次都转 180°。
geometry_msgs::msg::Quaternion disambiguate_tcp_opening_halfplane(
  const geometry_msgs::msg::Quaternion & tcp_q_msg, bool * flipped_out,
  double * open_yaw_deg_out)
{
  tf2::Quaternion q;
  tf2::fromMsg(tcp_q_msg, q);
  q.normalize();
  const tf2::Vector3 open = tf2::Matrix3x3(q) * tf2::Vector3(1.0, 0.0, 0.0);
  const double yaw = std::atan2(open.y(), open.x());
  const bool keep = (yaw > -M_PI / 2.0 && yaw <= M_PI / 2.0);
  if (flipped_out) {
    *flipped_out = !keep;
  }
  if (open_yaw_deg_out) {
    *open_yaw_deg_out = rad_to_deg(yaw);
  }
  if (keep) {
    return tcp_q_msg;
  }
  return tcp_quat_rz180(tcp_q_msg);
}

geometry_msgs::msg::Pose graspnet_optical_to_camera_link(
  const geometry_msgs::msg::Pose & optical_pose)
{
  // Publisher payload uses ROS optical axes (+X right, +Y down, +Z forward),
  // while the Isaac TF frame "cam0" uses camera-link axes
  // (+X forward, +Y left, +Z up). Therefore camera_link=(Z,-X,-Y).
  const tf2::Matrix3x3 camera_link_from_optical(
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);
  const tf2::Vector3 p_optical(
    optical_pose.position.x, optical_pose.position.y, optical_pose.position.z);
  const tf2::Vector3 p_camera_link = camera_link_from_optical * p_optical;

  tf2::Quaternion q_optical;
  tf2::fromMsg(optical_pose.orientation, q_optical);
  q_optical.normalize();
  tf2::Quaternion q_camera_link_from_optical;
  camera_link_from_optical.getRotation(q_camera_link_from_optical);
  tf2::Quaternion q_camera_link = q_camera_link_from_optical * q_optical;
  q_camera_link.normalize();

  geometry_msgs::msg::Pose output;
  output.position.x = p_camera_link.x();
  output.position.y = p_camera_link.y();
  output.position.z = p_camera_link.z();
  output.orientation = tf2::toMsg(q_camera_link);
  return output;
}

double graspnet_top_angle_deg(const geometry_msgs::msg::Quaternion & grasp_q_msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(grasp_q_msg, q);
  q.normalize();
  // Same convention as graspnet_orientation_to_tcp: approach = GraspNet local -X.
  const tf2::Vector3 approach = tf2::Matrix3x3(q) * tf2::Vector3(-1.0, 0.0, 0.0);
  const double cos_angle = std::clamp(-approach.z(), -1.0, 1.0);
  return rad_to_deg(std::acos(cos_angle));
}

/// 将一个已在 base_link 下的 GraspNet TCP 位姿展开为完整腕部计划。
///
/// pregrasp 沿 TCP 接近轴反向退出 ``pre_distance_m``，lift 固定沿世界 +Z；
/// 姿态在三段中保持。最后统一执行 TCP→J*_6 腕部换算，Pose 可直接送 MoveIt IK。
///
/// 注意：本机双臂腕部 Z 工作空间大约到 ~0.59 m。默认后退 0.15 m 时，
/// 若抓取 TCP 已在 ~0.31 m，预抓取腕部会到 ~0.66 m 导致 IK -31；
/// 选臂时应从大到小尝试后退距离。
GraspPlan make_graspnet_plan(
  const geometry_msgs::msg::Pose & tcp_pose, int arm_id,
  const GraspPlannerConfig & cfg, double pre_distance_m, double lift_distance_m)
{
  GraspPlan plan;
  plan.arm_id = arm_id;
  plan.frame_id = "base_link";
  plan.preserve_waypoints = true;
  plan.has_reorient = false;
  plan.has_finger_grasp_target = true;
  plan.finger_grasp_target = tcp_pose.position;

  tf2::Quaternion q;
  tf2::fromMsg(tcp_pose.orientation, q);
  q.normalize();
  const tf2::Vector3 approach = tf2::Matrix3x3(q) * tf2::Vector3(0.0, 0.0, 1.0);
  const double pre_dz = std::clamp(pre_distance_m, 0.05, 0.30);
  const double lift_dz = std::clamp(lift_distance_m, 0.05, 0.30);

  plan.grasp = tcp_pose;
  plan.pregrasp = tcp_pose;
  plan.pregrasp.position.x -= approach.x() * pre_dz;
  plan.pregrasp.position.y -= approach.y() * pre_dz;
  plan.pregrasp.position.z -= approach.z() * pre_dz;
  plan.lift = tcp_pose;
  plan.lift.position.z += lift_dz;
  plan.reorient = plan.pregrasp;
  plan.path_tcp_z_offset = std::clamp(cfg.graspnet_ee_tcp_z_offset, 0.0, 0.30);

  apply_tcp_to_wrist(plan.pregrasp, plan.path_tcp_z_offset);
  apply_tcp_to_wrist(plan.reorient, plan.path_tcp_z_offset);
  apply_tcp_to_wrist(plan.grasp, plan.path_tcp_z_offset);
  apply_tcp_to_wrist(plan.lift, plan.path_tcp_z_offset);
  return plan;
}

}  // namespace

GraspRosNode::GraspRosNode(const rclcpp::NodeOptions & options)
: Node("grasp_qt_ui_ros_node", options)
{
  declare_parameter<std::string>("box_topic", "/box_pose");
  declare_parameter<std::string>(
    "graspnet_topic", "/yolo_graspnet/collision_free_grasps");
  declare_parameter<double>("graspnet_top_max_angle_deg", 30.0);
  declare_parameter<double>("graspnet_comm_ok_timeout_sec", 10.0);
  declare_parameter<double>("graspnet_comm_stale_timeout_sec", 30.0);
  declare_parameter<std::string>("pose_frame", "base_link");
  declare_parameter<std::string>("target_arm_pose_topic", "/nova_target_arm_pose");
  declare_parameter<std::string>("arm_id_topic", "/nova_arm_id");
  declare_parameter<std::string>("gripper_topic", "/nova_gripper_goal");
  declare_parameter<std::string>("status_topic", "/nova_grasp/status");
  declare_parameter<std::string>("pose_log_topic", "/nova_pose_log");
  declare_parameter<double>("arm_split_x", 0.40);
  declare_parameter<double>("pregrasp_z_offset", 0.15);
  declare_parameter<double>("lift_z_offset", 0.15);
  declare_parameter<double>("box_grasp_z_offset", 0.0);
  declare_parameter<double>("ee_tcp_z_offset", 0.20);
  declare_parameter<double>("min_approach_clearance", 0.15);
  declare_parameter<double>("grasp_yaw_offset_deg", 90.0);
  declare_parameter<double>("graspnet_pregrasp_distance", 0.08);
  declare_parameter<double>("graspnet_lift_z_offset", 0.10);
  declare_parameter<double>("graspnet_ee_tcp_z_offset", 0.12);
  declare_parameter<double>("graspnet_min_approach_clearance", 0.08);
  declare_parameter<double>("step_settle_sec", 2.0);
  declare_parameter<double>("gripper_settle_sec", 0.8);
  declare_parameter<double>("gripper_open_m", 0.08);
  declare_parameter<double>("gripper_close_m", 0.02);
  declare_parameter<std::string>("ee_frame_arm0", "J1_6");
  declare_parameter<std::string>("ee_frame_arm1", "J2_6");
  declare_parameter<std::string>("ref_frame", "base_link");
  declare_parameter<std::string>("joint_command_topic", "/joint_command");
  declare_parameter<int>("joint_command_burst_count", 5);
  declare_parameter<bool>("publish_mujoco_joint_array", true);

  planner_cfg_.arm_split_x = get_parameter("arm_split_x").as_double();
  planner_cfg_.pregrasp_z_offset = get_parameter("pregrasp_z_offset").as_double();
  planner_cfg_.lift_z_offset = get_parameter("lift_z_offset").as_double();
  planner_cfg_.box_grasp_z_offset = get_parameter("box_grasp_z_offset").as_double();
  planner_cfg_.ee_tcp_z_offset = get_parameter("ee_tcp_z_offset").as_double();
  planner_cfg_.min_approach_clearance = get_parameter("min_approach_clearance").as_double();
  planner_cfg_.grasp_yaw_offset_deg = get_parameter("grasp_yaw_offset_deg").as_double();
  planner_cfg_.graspnet_pregrasp_distance =
    get_parameter("graspnet_pregrasp_distance").as_double();
  planner_cfg_.graspnet_lift_z_offset =
    get_parameter("graspnet_lift_z_offset").as_double();
  planner_cfg_.graspnet_ee_tcp_z_offset =
    get_parameter("graspnet_ee_tcp_z_offset").as_double();
  planner_cfg_.graspnet_min_approach_clearance =
    get_parameter("graspnet_min_approach_clearance").as_double();
  planner_cfg_.gripper_open_m = get_parameter("gripper_open_m").as_double();
  planner_cfg_.gripper_close_m = get_parameter("gripper_close_m").as_double();
  step_settle_sec_ = get_parameter("step_settle_sec").as_double();
  gripper_settle_sec_ = get_parameter("gripper_settle_sec").as_double();
  graspnet_comm_ok_timeout_sec_ = std::max(
    0.1, get_parameter("graspnet_comm_ok_timeout_sec").as_double());
  graspnet_comm_stale_timeout_sec_ = std::max(
    graspnet_comm_ok_timeout_sec_,
    get_parameter("graspnet_comm_stale_timeout_sec").as_double());

  const auto target_topic = get_parameter("target_arm_pose_topic").as_string();
  const auto gripper_topic = get_parameter("gripper_topic").as_string();
  const auto status_topic = get_parameter("status_topic").as_string();
  const auto pose_log_topic = get_parameter("pose_log_topic").as_string();
  const auto box_topic = get_parameter("box_topic").as_string();
  const auto graspnet_topic = get_parameter("graspnet_topic").as_string();
  graspnet_topic_ = graspnet_topic;
  data_.graspnet_topic = graspnet_topic_;

  arm_pose_pub_ = create_publisher<nova_grasp_moveit::msg::ArmPose>(target_topic, 10);
  gripper_pub_ = create_publisher<std_msgs::msg::String>(gripper_topic, 10);
  grasp_status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
  joint_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/arm_controller/commands", 10);
  const auto joint_cmd_topic = get_parameter("joint_command_topic").as_string();
  auto joint_cmd_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  joint_cmd_qos.reliable();
  joint_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_cmd_topic, joint_cmd_qos);
  arm_id_pub_ = create_publisher<std_msgs::msg::Int32>(
    get_parameter("arm_id_topic").as_string(), 10);

  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 50,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(msg); });
  box_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    box_topic, box_pose_qos(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_box_pose(msg); });
  graspnet_sub_ = create_subscription<std_msgs::msg::String>(
    graspnet_topic, 10,
    [this](const std_msgs::msg::String::SharedPtr msg) { on_graspnet_candidates(msg); });
  status_sub_ = create_subscription<std_msgs::msg::String>(
    status_topic, 20,
    [this](const std_msgs::msg::String::SharedPtr msg) { on_grasp_status(msg); });
  pose_log_sub_ = create_subscription<std_msgs::msg::String>(
    pose_log_topic, 20,
    [this](const std_msgs::msg::String::SharedPtr msg) { on_pose_log(msg); });

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

  executor_ = std::make_shared<GraspExecutor>(
    arm_pose_pub_, gripper_pub_, grasp_status_pub_, planner_cfg_,
    step_settle_sec_, gripper_settle_sec_);
  executor_->set_callbacks(
    [this](const std::string & text) {
      std::lock_guard<std::mutex> lk(mu_);
      data_.grasp_status = text;
    },
    [this](const std::string & text) { push_log(text); });
  executor_->set_gripper_apply_callback(
    [this](int arm_id, double opening_m) {
      publish_gripper_opening_detailed(arm_id, opening_m, nullptr);
    });
  executor_->set_current_ee_callback(
    [this](int arm_id, geometry_msgs::msg::Pose & out) {
      // 每次规划/步进前现场查 TF，不用过期 cache（走完一轮后姿态会变）
      refresh_ee_poses();
      const auto & ee = (arm_id == 0) ? cached_arm1_ee_ : cached_arm2_ee_;
      if (!ee.ok) {
        return false;
      }
      out.position.x = ee.x;
      out.position.y = ee.y;
      out.position.z = ee.z;
      out.orientation.x = ee.qx;
      out.orientation.y = ee.qy;
      out.orientation.z = ee.qz;
      out.orientation.w = ee.qw;
      return true;
    });
  executor_->set_pose_step_callbacks(
    [this](const geometry_msgs::msg::Pose & expected) {
      reset_pose_step_wait(expected);
    },
    [this](double timeout_sec, std::string * fail_reason) {
      return wait_pose_step(timeout_sec, fail_reason);
    });

  ik_watch_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() { refresh_ik_ready(); });
  ee_refresh_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    [this]() { refresh_ee_poses(); });

  push_log(
    "ready box=" + box_topic + " graspnet=" + graspnet_topic +
    " target=" + target_topic + " gripper=" + gripper_topic + " joint_cmd=" + joint_cmd_topic);
}

void GraspRosNode::init_tf_listener()
{
  if (!tf_listener_) {
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this(), false);
    refresh_ee_poses();
  }
}

void GraspRosNode::prepare_shutdown()
{
  if (executor_) {
    executor_->request_shutdown();
  }
  ik_watch_timer_.reset();
  ee_refresh_timer_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
}

void GraspRosNode::abort_grasp_execution()
{
  if (executor_) {
    executor_->force_idle();
  }
  {
    std::lock_guard<std::mutex> lk(pose_step_mu_);
    pose_step_result_ = PoseStepResult::Fail;
    pose_step_fail_reason_ = "aborted by reset";
    pose_step_expect_valid_ = false;
    pose_step_cv_.notify_all();
  }
  push_log("[executor] abort_grasp_execution — busy cleared");
}

GraspRosSnapshot GraspRosNode::snapshot()
{
  GraspRosSnapshot snap;
  rclcpp::Time last_joint_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_graspnet_time{0, 0, RCL_ROS_TIME};
  {
    std::lock_guard<std::mutex> lk(mu_);
    snap = data_;
    snap.executor_busy = executor_ && executor_->is_busy();
    snap.ik_service_ready = data_.ik_service_ready;
    snap.arm1_joints = extract_arm_joints(joint_map_, "J1_");
    snap.arm2_joints = extract_arm_joints(joint_map_, "J2_");
    snap.joint_states_count = static_cast<int>(joint_map_.size());
    last_joint_time = last_joint_state_time_;
    last_graspnet_time = last_graspnet_time_;
    snap.arm1_ee = cached_arm1_ee_;
    snap.arm2_ee = cached_arm2_ee_;
  }

  snap.tf_comm_ok = snap.arm1_ee.ok || snap.arm2_ee.ok;

  if (last_joint_time.nanoseconds() > 0) {
    snap.joint_states_age_sec = (now() - last_joint_time).seconds();
  }
  constexpr double kFreshSec = 2.0;
  constexpr double kStaleSec = 5.0;
  const bool has_joints = snap.joint_states_count > 0;
  snap.arm_comm_ok = has_joints && snap.joint_states_age_sec < kFreshSec;
  snap.arm_comm_stale = has_joints && !snap.arm_comm_ok && snap.joint_states_age_sec < kStaleSec;

  const auto age_str = [&]() {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1) << snap.joint_states_age_sec;
      return oss.str();
    }();

  if (snap.arm_comm_ok) {
    snap.arm_comm_summary =
      "/joint_states OK (" + std::to_string(snap.joint_states_count) + " joints, " +
      age_str + "s ago)";
  } else if (snap.arm_comm_stale) {
    snap.arm_comm_summary = "/joint_states STALE (" + age_str + "s ago)";
  } else if (has_joints) {
    snap.arm_comm_summary = "/joint_states TIMEOUT (>5s)";
  } else {
    snap.arm_comm_summary = "no /joint_states yet";
  }
  if (snap.tf_comm_ok) {
    snap.arm_comm_summary += " · TF OK";
  } else {
    snap.arm_comm_summary += " · TF --";
  }
  if (snap.ik_service_ready) {
    snap.arm_comm_summary += " · IK OK";
  } else {
    snap.arm_comm_summary += " · IK --";
  }

  // GraspNet 推理可能需要数秒；状态阈值独立配置，避免低频正常发布被误报为延迟。
  if (last_graspnet_time.nanoseconds() > 0) {
    snap.graspnet_age_sec = std::max(0.0, (now() - last_graspnet_time).seconds());
  }
  const bool has_graspnet_messages = snap.graspnet_message_count > 0;
  snap.graspnet_comm_ok =
    has_graspnet_messages && snap.graspnet_age_sec < graspnet_comm_ok_timeout_sec_;
  snap.graspnet_comm_stale =
    has_graspnet_messages && !snap.graspnet_comm_ok &&
    snap.graspnet_age_sec < graspnet_comm_stale_timeout_sec_;
  std::ostringstream graspnet_age;
  graspnet_age << std::fixed << std::setprecision(1) << snap.graspnet_age_sec;
  if (!has_graspnet_messages) {
    snap.graspnet_comm_summary = "未收到 " + snap.graspnet_topic;
  } else if (snap.graspnet_comm_ok && snap.has_graspnet_candidates) {
    snap.graspnet_comm_summary =
      "正常 · " + std::to_string(snap.graspnet_candidates.poses.size()) +
      " 个位姿 · " + graspnet_age.str() + "s";
  } else if (snap.graspnet_comm_ok) {
    snap.graspnet_comm_summary = "在线 · 当前无抓取位姿 · " + graspnet_age.str() + "s";
  } else if (snap.graspnet_comm_stale) {
    snap.graspnet_comm_summary = "发布延迟 · " + graspnet_age.str() + "s";
  } else {
    snap.graspnet_comm_summary = "发布中断 · " + graspnet_age.str() + "s";
  }
  return snap;
}

void GraspRosNode::append_ui_log(const std::string & line)
{
  push_log(line);
}

void GraspRosNode::clear_logs()
{
  std::lock_guard<std::mutex> lk(mu_);
  data_.logs.clear();
  data_.pose_logs.clear();
}

GraspComputeResult GraspRosNode::compute_grasp_from_box_detailed()
{
  GraspComputeResult out;
  try {
    geometry_msgs::msg::PoseStamped box;
    bool has_box = false;
    bool comm_ok = false;
    std::string comm_detail;
    {
      std::lock_guard<std::mutex> lk(mu_);
      has_box = data_.has_box_pose;
      if (has_box) {
        box = data_.box_pose;
      }
      const bool has_joints = !joint_map_.empty();
      double age_sec = 999.0;
      if (last_joint_state_time_.nanoseconds() > 0) {
        age_sec = (now() - last_joint_state_time_).seconds();
      }
      comm_ok = has_joints && age_sec < 2.0;
      if (!has_joints) {
        comm_detail = "未收到 /joint_states";
      } else if (age_sec >= 2.0) {
        comm_detail = "/joint_states 超时 (" + std::to_string(age_sec) + "s)";
      }
    }

    if (!comm_ok) {
      out.error_title = "机械臂通信异常";
      out.error_message = comm_detail.empty() ?
        "与机械臂通信异常，请确认仿真或真机已启动并发布 /joint_states。" :
        comm_detail;
      push_log("[compute] " + out.error_message);
      return out;
    }
    if (!has_box) {
      out.error_title = "缺少盒子位姿";
      out.error_message = "尚未收到 /box_pose，请先启动仿真并发布盒子位姿。";
      push_log("[compute] no /box_pose yet");
      return out;
    }

    const std::string frame = box.header.frame_id.empty() ?
      get_parameter("pose_frame").as_string() : box.header.frame_id;
    const auto grasp_pose = box_pose_to_grasp_pose(box.pose, planner_cfg_);
    GraspPlan plan = plan_grasp_from_pose(grasp_pose, frame, planner_cfg_);

    refresh_ee_poses();
    geometry_msgs::msg::Pose ee;
    const auto & cached = (plan.arm_id == 0) ? cached_arm1_ee_ : cached_arm2_ee_;
    if (!cached.ok) {
      out.error_title = "缺少末端 TF";
      out.error_message = "无法读取 J1_6/J2_6 TF，无法换算腕部高度；请确认 Isaac 已 Play 且发布 /tf。";
      push_log("[compute] " + out.error_message);
      return out;
    }
    ee.position.x = cached.x;
    ee.position.y = cached.y;
    ee.position.z = cached.z;
    ee.orientation.x = cached.qx;
    ee.orientation.y = cached.qy;
    ee.orientation.z = cached.qz;
    ee.orientation.w = cached.qw;
    fill_plan_waypoints(plan, planner_cfg_, &ee);

    {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(4)
          << "[compute] box_center=(" << box.pose.position.x << "," << box.pose.position.y
          << "," << box.pose.position.z << ") finger_target=("
          << plan.finger_grasp_target.x << "," << plan.finger_grasp_target.y << ","
          << plan.finger_grasp_target.z << ") arm_id=" << plan.arm_id
          << " tcp_offset=" << plan.path_tcp_z_offset;
      push_log(oss.str());
    }

    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = box.header;
    if (stamped.header.frame_id.empty()) {
      stamped.header.frame_id = frame;
    }
    stamped.pose = plan.grasp;

    {
      std::lock_guard<std::mutex> lk(mu_);
      data_.computed_grasp_pose = stamped;
      data_.has_computed_grasp = true;
      data_.last_plan = plan;
      data_.has_plan = true;
      push_log_nolock("[compute] path waypoints (blue=[PATH]):");
      for (const auto & line : format_plan_path_lines(plan)) {
        push_log_nolock(line);
      }
    }
    if (executor_) {
      std::string step_err;
      if (!executor_->prepare_step_mode(plan, &step_err)) {
        push_log("[compute] WARN step mode: " + step_err);
      } else {
        push_log(
          "[compute] 单步就绪: 共 " + std::to_string(executor_->step_count()) +
          " 步，点「单步」依次执行");
      }
    }
    out.ok = true;
  } catch (const std::exception & ex) {
    out.error_title = "计算抓取失败";
    out.error_message = ex.what();
    push_log(std::string("[compute] ERROR ") + ex.what());
  } catch (...) {
    out.error_title = "计算抓取失败";
    out.error_message = "未知异常";
    push_log("[compute] ERROR unknown exception");
  }
  return out;
}

bool GraspRosNode::check_candidate_ik(
  int arm_id, const geometry_msgs::msg::PoseStamped & wrist_pose,
  double & joint_motion_cost, std::string & reason)
{
  // 此函数只用于候选可达性/选臂，不发布 IK 解。种子必须是最新 /joint_states，
  // 代价使用周期折叠后的关节角差平方和，避免 ±π 边界产生虚假大移动。
  joint_motion_cost = std::numeric_limits<double>::infinity();
  reason.clear();
  if (arm_id < 0 || arm_id > 1) {
    reason = "invalid arm";
    return false;
  }
  if (!ik_client_ || !ik_client_->service_is_ready()) {
    reason = "/compute_ik unavailable";
    return false;
  }

  std::map<std::string, double> joints;
  {
    std::lock_guard<std::mutex> lk(mu_);
    joints = joint_map_;
  }
  if (joints.empty()) {
    reason = "no /joint_states";
    return false;
  }

  sensor_msgs::msg::JointState seed;
  for (const auto & kv : joints) {
    std::string name = kv.first;
    if (name.rfind("J", 0) != 0) {
      continue;
    }
    if (name.size() < 6 || name.compare(name.size() - 6, 6, "_joint") != 0) {
      name += "_joint";
    }
    seed.name.push_back(name);
    seed.position.push_back(kv.second);
  }

  auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  req->ik_request.group_name = arm_id == 0 ? "l_arm" : "r_arm";
  req->ik_request.ik_link_name = arm_id == 0 ? "J1_6" : "J2_6";
  req->ik_request.pose_stamped = wrist_pose;
  req->ik_request.robot_state.joint_state = seed;
  // 候选筛选会连打多次 IK；与 kinematics_solver_timeout(0.2s) 对齐，避免每次空等 2–3s。
  req->ik_request.timeout.sec = 0;
  req->ik_request.timeout.nanosec = 150000000;  // 0.15 s
  req->ik_request.avoid_collisions = false;

  auto future = ik_client_->async_send_request(req);
  if (future.wait_for(std::chrono::milliseconds(400)) != std::future_status::ready) {
    reason = "IK timeout";
    return false;
  }
  const auto response = future.get();
  if (!response) {
    reason = "empty IK response";
    return false;
  }
  if (response->error_code.val != 1) {
    reason = "IK code=" + std::to_string(response->error_code.val);
    return false;
  }

  const std::string prefix = arm_id == 0 ? "J1_" : "J2_";
  double cost = 0.0;
  int count = 0;
  const auto & names = response->solution.joint_state.name;
  const auto & positions = response->solution.joint_state.position;
  for (size_t i = 0; i < names.size() && i < positions.size(); ++i) {
    if (names[i].rfind(prefix, 0) != 0 || names[i].find("_joint") == std::string::npos) {
      continue;
    }
    const auto current = joints.find(names[i]);
    if (current == joints.end()) {
      continue;
    }
    double delta = positions[i] - current->second;
    delta = std::atan2(std::sin(delta), std::cos(delta));
    cost += delta * delta;
    ++count;
  }
  if (count == 0) {
    reason = "IK solution has no matching arm joints";
    return false;
  }
  joint_motion_cost = cost;
  return true;
}

GraspComputeResult GraspRosNode::compute_grasp_from_graspnet_detailed(
  double top_max_angle_deg)
{
  GraspComputeResult out;
  geometry_msgs::msg::PoseArray frozen;
  std::vector<double> frozen_scores;
  // 点击时复制整帧：计算期间即使发布端继续更新，也不会混入另一帧候选。
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!data_.has_graspnet_candidates || data_.graspnet_candidates.poses.empty()) {
      out.error_title = "缺少 GraspNet 位姿";
      out.error_message =
        "尚未收到 " + data_.graspnet_topic + " 中的有效 GraspNet JSON 候选。";
      return out;
    }
    frozen = data_.graspnet_candidates;
    frozen_scores = data_.graspnet_scores;
  }
  if (!tf_buffer_) {
    out.error_title = "TF 不可用";
    out.error_message = "TF buffer 尚未初始化。";
    return out;
  }
  if (frozen.header.frame_id.empty()) {
    out.error_title = "相机坐标系为空";
    out.error_message = "GraspNet JSON 的 frame_id 为空，无法转换到 base_link。";
    return out;
  }

  struct Candidate
  {
    int index{-1};
    double approach_deg{180.0};
    double score{std::numeric_limits<double>::quiet_NaN()};
    bool top{false};
    bool ik_reachable{false};
    /// 半平面消歧是否已翻 Rz180；再与 IK 对偶选择异或后得到最终 a_corrected。
    bool halfplane_flip{false};
    bool a_corrected{false};
    geometry_msgs::msg::PoseStamped tcp_base;
  };
  std::vector<Candidate> candidates;
  std::vector<std::string> transform_errors;
  int invalid_pose_count = 0;
  int transform_failure_count = 0;
  const double top_limit = std::clamp(top_max_angle_deg, 1.0, 89.0);
  {
    std::ostringstream line;
    line << "[graspnet][TF] frozen frame=" << frozen.header.frame_id
         << " stamp=" << frozen.header.stamp.sec << "."
         << std::setw(9) << std::setfill('0') << frozen.header.stamp.nanosec
         << " candidates=" << frozen.poses.size()
         << " target=base_link";
    push_log(line.str());
  }

  for (size_t i = 0; i < frozen.poses.size(); ++i) {
    if (!finite_pose(frozen.poses[i])) {
      ++invalid_pose_count;
      push_log(
        "[graspnet][TF][INVALID] #" + std::to_string(i) + " " +
        format_pose_compact(frozen.poses[i]));
      continue;
    }
    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header = frozen.header;
    camera_pose.pose = graspnet_optical_to_camera_link(frozen.poses[i]);
    geometry_msgs::msg::PoseStamped grasp_base;
    // 优先使用消息时间戳保证图像/外参同步；缓存时间窗外时回退到最新 TF。
    try {
      grasp_base = tf_buffer_->transform(
        camera_pose, "base_link", tf2::durationFromSec(0.25));
    } catch (const tf2::TransformException & first_error) {
      try {
        camera_pose.header.stamp.sec = 0;
        camera_pose.header.stamp.nanosec = 0;
        grasp_base = tf_buffer_->transform(
          camera_pose, "base_link", tf2::durationFromSec(0.25));
      } catch (const tf2::TransformException & latest_error) {
        ++transform_failure_count;
        const std::string detail =
          "#" + std::to_string(i) + " source=" + frozen.header.frame_id +
          " optical{" + format_pose_compact(frozen.poses[i]) + "}" +
          " camera_link{" + format_pose_compact(camera_pose.pose) + "}" +
          " | stamped: " + first_error.what() +
          " | latest: " + latest_error.what();
        if (transform_errors.size() < 8) {
          transform_errors.push_back(detail);
        }
        push_log("[graspnet][TF][FAIL] " + detail);
        continue;
      }
    }

    Candidate candidate;
    candidate.index = static_cast<int>(i);
    if (i < frozen_scores.size()) {
      candidate.score = frozen_scores[i];
    }
    candidate.approach_deg = graspnet_top_angle_deg(grasp_base.pose.orientation);
    candidate.top = candidate.approach_deg <= top_limit;
    candidate.tcp_base = grasp_base;
    candidate.tcp_base.header.frame_id = "base_link";
    candidate.tcp_base.pose.orientation =
      graspnet_orientation_to_tcp(grasp_base.pose.orientation);
    // A：按开合轴水平 yaw 半平面消歧（≈ 部分场景 J6±180°）。
    double open_yaw_deg = 0.0;
    candidate.tcp_base.pose.orientation = disambiguate_tcp_opening_halfplane(
      candidate.tcp_base.pose.orientation, &candidate.halfplane_flip, &open_yaw_deg);
    candidate.a_corrected = candidate.halfplane_flip;
    {
      std::ostringstream line;
      line << std::fixed << std::setprecision(2)
           << "[graspnet][TF][OK] #" << i
           << " optical{" << format_pose_compact(frozen.poses[i]) << "}"
           << " camera_link{" << format_pose_compact(camera_pose.pose) << "}"
           << " -> base_grasp{" << format_pose_compact(grasp_base.pose) << "}"
           << " -> base_tcp{" << format_pose_compact(candidate.tcp_base.pose) << "}"
           << " approach=" << candidate.approach_deg << "deg"
           << " open_yaw=" << open_yaw_deg << "deg"
           << (candidate.halfplane_flip ? " J6_FLIP" : " J6_keep");
      push_log(line.str());
    }
    candidates.push_back(candidate);
  }

  if (candidates.empty()) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      data_.graspnet_base_candidates = geometry_msgs::msg::PoseArray();
      data_.graspnet_base_candidates.header.frame_id = "base_link";
      data_.graspnet_base_candidate_indices.clear();
      data_.graspnet_base_candidate_ik_ok.clear();
      data_.graspnet_base_candidate_a_corrected.clear();
      ++data_.graspnet_base_revision;
      data_.has_selected_graspnet_pose = false;
      data_.selected_graspnet_index = -1;
      data_.selected_graspnet_arm = -1;
      data_.graspnet_selection_summary.clear();
    }
    out.error_title = "候选转换失败";
    std::ostringstream detail;
    detail << "候选总数=" << frozen.poses.size()
           << "，无效位姿=" << invalid_pose_count
           << "，TF 失败=" << transform_failure_count
           << "，成功=0。\n"
           << "源坐标系=" << frozen.header.frame_id << "，目标坐标系=base_link。";
    if (!transform_errors.empty()) {
      detail << "\n\n具体 TF 错误：";
      for (const auto & err : transform_errors) {
        detail << "\n" << err;
      }
    }
    out.error_message = detail.str();
    return out;
  }

  // 保留表格显示顺序（转换顺序），IK 标记按同一顺序写入；
  // 选解后可能把某行的 TCP 换成 Rz180 对偶，再写回 table_order。
  std::vector<Candidate> table_order = candidates;

  const int top_count = static_cast<int>(std::count_if(
      candidates.begin(), candidates.end(), [](const Candidate & c) {return c.top;}));
  // 探测顺序：顶抓在前，同级按 score 降序，便于日志阅读；最终选中仍在全部可达里取最高分。
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {
      if (a.top != b.top) {
        return a.top > b.top;
      }
      const bool a_ok = std::isfinite(a.score);
      const bool b_ok = std::isfinite(b.score);
      if (a_ok != b_ok) {
        return a_ok > b_ok;
      }
      if (a_ok && b_ok && a.score != b.score) {
        return a.score > b.score;
      }
      return a.approach_deg < b.approach_deg;
    });

  bool selected = false;
  Candidate selected_candidate;
  GraspPlan selected_plan;
  double selected_pre_dz = 0.0;
  double selected_lift_dz = 0.0;
  double selected_score = -std::numeric_limits<double>::infinity();
  std::string last_ik_reason;
  std::map<int, uint8_t> ik_ok_by_index;
  // Prefer configured clearance, then shorten if wrist exceeds workspace
  // (empirically wrist Z ≳ 0.59 m → IK -31).
  const double preferred_pre = std::clamp(
    std::max(
      planner_cfg_.graspnet_pregrasp_distance,
      planner_cfg_.graspnet_min_approach_clearance),
    0.05, 0.30);
  const double preferred_lift = std::clamp(
    planner_cfg_.graspnet_lift_z_offset, 0.05, 0.30);
  // 缩短扫描档位：默认档 + 两档更短，减少连打 IK 次数。
  const std::vector<double> pre_distances = {preferred_pre, 0.08, 0.05};
  const std::vector<double> lift_distances = {preferred_lift, 0.08, 0.05};

  // 对单个 TCP 姿态探测可达性（不含 J6 对偶）。
  auto probe_one_tcp =
    [&](const geometry_msgs::msg::Pose & tcp_pose,
      const std_msgs::msg::Header & header, int cand_index, bool full_search,
      double * out_cost, GraspPlan * out_plan, double * out_pre, double * out_lift,
      int * out_arm)
    {
      for (int arm_id = 0; arm_id <= 1; ++arm_id) {
        bool grasp_unreachable = false;
        for (const double pre_dz : pre_distances) {
          if (pre_dz > preferred_pre + 1e-9) {
            continue;
          }
          if (grasp_unreachable) {
            break;
          }
          for (const double lift_dz : lift_distances) {
            if (lift_dz > preferred_lift + 1e-9) {
              continue;
            }
            GraspPlan plan = make_graspnet_plan(
              tcp_pose, arm_id, planner_cfg_, pre_dz, lift_dz);
            geometry_msgs::msg::PoseStamped pre_pose;
            pre_pose.header = header;
            pre_pose.pose = plan.pregrasp;
            geometry_msgs::msg::PoseStamped grasp_pose;
            grasp_pose.header = header;
            grasp_pose.pose = plan.grasp;
            geometry_msgs::msg::PoseStamped lift_pose;
            lift_pose.header = header;
            lift_pose.pose = plan.lift;

            double pre_cost = 0.0;
            double grasp_cost = 0.0;
            double lift_cost = 0.0;
            std::string reason;
            if (!check_candidate_ik(arm_id, grasp_pose, grasp_cost, reason)) {
              last_ik_reason =
                "#" + std::to_string(cand_index) + " arm=" + std::to_string(arm_id) +
                " grasp " + reason;
              grasp_unreachable = true;
              break;
            }
            if (!check_candidate_ik(arm_id, pre_pose, pre_cost, reason)) {
              std::ostringstream fail;
              fail << std::fixed << std::setprecision(3)
                   << "#" << cand_index << " arm=" << arm_id
                   << " pre_dz=" << pre_dz << "m wrist_z=" << plan.pregrasp.position.z
                   << "m pre " << reason;
              last_ik_reason = fail.str();
              break;
            }
            if (full_search) {
              if (!check_candidate_ik(arm_id, lift_pose, lift_cost, reason)) {
                std::ostringstream fail;
                fail << std::fixed << std::setprecision(3)
                     << "#" << cand_index << " arm=" << arm_id
                     << " lift_dz=" << lift_dz << "m wrist_z=" << plan.lift.position.z
                     << "m lift " << reason;
                last_ik_reason = fail.str();
                continue;
              }
            }
            if (out_cost) {
              *out_cost = pre_cost + grasp_cost + (full_search ? lift_cost : 0.0);
            }
            if (out_plan) {
              *out_plan = plan;
            }
            if (out_pre) {
              *out_pre = pre_dz;
            }
            if (out_lift) {
              *out_lift = lift_dz;
            }
            if (out_arm) {
              *out_arm = arm_id;
            }
            return true;
          }
        }
      }
      return false;
    };

  // A 的对偶：半平面消歧后再与 Rz180 比 IK 代价，靠近当前关节的 J6 解胜出。
  auto probe_reachable =
    [&](Candidate & candidate, bool full_search, double * out_cost,
      GraspPlan * out_plan, double * out_pre, double * out_lift, int * out_arm)
    {
      geometry_msgs::msg::Pose tcp0 = candidate.tcp_base.pose;
      geometry_msgs::msg::Pose tcp1 = tcp0;
      tcp1.orientation = tcp_quat_rz180(tcp0.orientation);

      double best_cost = std::numeric_limits<double>::infinity();
      GraspPlan best_plan;
      double best_pre = 0.0;
      double best_lift = 0.0;
      int best_arm = 0;
      int best_ori = -1;

      for (int ori = 0; ori < 2; ++ori) {
        const auto & tcp = (ori == 0) ? tcp0 : tcp1;
        double cost = 0.0;
        GraspPlan plan;
        double pre_dz = 0.0;
        double lift_dz = 0.0;
        int arm_id = 0;
        if (!probe_one_tcp(
            tcp, candidate.tcp_base.header, candidate.index, full_search,
            &cost, &plan, &pre_dz, &lift_dz, &arm_id))
        {
          continue;
        }
        if (cost < best_cost) {
          best_cost = cost;
          best_plan = plan;
          best_pre = pre_dz;
          best_lift = lift_dz;
          best_arm = arm_id;
          best_ori = ori;
        }
      }
      if (best_ori < 0) {
        return false;
      }
      if (best_ori == 1) {
        candidate.tcp_base.pose = tcp1;
        // 半平面已翻再选对偶 → 回到原始映射；半平面未翻再选对偶 → 最终修正。
        candidate.a_corrected = !candidate.halfplane_flip;
        push_log(
          "[graspnet][J6] #" + std::to_string(candidate.index) +
          " use Rz180 (lower IK cost vs current joints)" +
          (candidate.a_corrected ? " A_mark" : " A_undo"));
      } else {
        candidate.a_corrected = candidate.halfplane_flip;
      }
      if (out_cost) {
        *out_cost = best_cost;
      }
      if (out_plan) {
        *out_plan = best_plan;
      }
      if (out_pre) {
        *out_pre = best_pre;
      }
      if (out_lift) {
        *out_lift = best_lift;
      }
      if (out_arm) {
        *out_arm = best_arm;
      }
      return true;
    };

  for (auto & candidate : candidates) {
    GraspPlan plan;
    double pre_dz = preferred_pre;
    double lift_dz = preferred_lift;
    int arm_id = 0;
    // 全部做完整 IK（含 lift），再在可达集合里取 score 最高者。
    const bool ok = probe_reachable(
      candidate, true, nullptr, &plan, &pre_dz, &lift_dz, &arm_id);
    ik_ok_by_index[candidate.index] = ok ? 1 : 0;
    if (!ok) {
      continue;
    }
    const double score_key = std::isfinite(candidate.score) ?
      candidate.score : -std::numeric_limits<double>::infinity();
    const bool better =
      !selected ||
      score_key > selected_score ||
      (score_key == selected_score &&
        candidate.approach_deg < selected_candidate.approach_deg) ||
      (score_key == selected_score &&
        candidate.approach_deg == selected_candidate.approach_deg &&
        candidate.index < selected_candidate.index);
    if (!better) {
      continue;
    }
    selected = true;
    selected_score = score_key;
    selected_candidate = candidate;
    selected_candidate.ik_reachable = true;
    selected_plan = plan;
    selected_pre_dz = pre_dz;
    selected_lift_dz = lift_dz;
    selected_plan.arm_id = arm_id;
  }

  // 表格顺序与 candidates 同步（含 IK 选出的 J6 对偶姿态 / A 标记）。
  for (auto & row : table_order) {
    for (const auto & c : candidates) {
      if (c.index == row.index) {
        row.tcp_base = c.tcp_base;
        row.a_corrected = c.a_corrected;
        row.halfplane_flip = c.halfplane_flip;
        break;
      }
    }
  }

  {
    std::lock_guard<std::mutex> lk(mu_);
    data_.graspnet_base_candidates = geometry_msgs::msg::PoseArray();
    data_.graspnet_base_candidates.header.frame_id = "base_link";
    data_.graspnet_base_candidate_indices.clear();
    data_.graspnet_base_candidate_ik_ok.clear();
    data_.graspnet_base_candidate_a_corrected.clear();
    for (const auto & candidate : table_order) {
      data_.graspnet_base_candidates.poses.push_back(candidate.tcp_base.pose);
      data_.graspnet_base_candidate_indices.push_back(candidate.index);
      const auto it = ik_ok_by_index.find(candidate.index);
      data_.graspnet_base_candidate_ik_ok.push_back(
        it != ik_ok_by_index.end() ? it->second : static_cast<uint8_t>(0));
      data_.graspnet_base_candidate_a_corrected.push_back(
        candidate.a_corrected ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0));
    }
    ++data_.graspnet_base_revision;
    data_.has_selected_graspnet_pose = false;
    data_.selected_graspnet_index = -1;
    data_.selected_graspnet_arm = -1;
    data_.graspnet_selection_summary.clear();
  }

  if (!selected) {
    out.error_title = "无可达抓取位姿";
    out.error_message =
      "候选已转换到 base_link，但两条机械臂均无法完成预抓取/抓取/抬升 IK。\n"
      "常见原因：腕部过高（本机约 Z>0.59 m 无解）。";
    if (!last_ik_reason.empty()) {
      out.error_message += "\n最后结果：" + last_ik_reason;
    }
    return out;
  }

  // GraspNet: 禁止在「当前 XY」抬到 pregrasp 高度。
  // pregrasp.z 只在目标附近可达；在 home XY 抬到 0.61 会 IK -31（你看到的 raise 失败）。
  // 直接 move_xy → 已校验过的 pregrasp。
  selected_plan.has_raise = false;
  selected_plan.need_vertical_raise = false;

  std::ostringstream summary;
  summary << std::fixed << std::setprecision(2)
          << "selected #" << selected_candidate.index
          << " arm=J" << (selected_plan.arm_id + 1)
          << " score=" << std::setprecision(5) << selected_candidate.score
          << std::setprecision(2)
          << " approach=" << selected_candidate.approach_deg << "deg"
          << (selected_candidate.top ? " TOP" : " fallback")
          << " pre_dz=" << selected_pre_dz << "m"
          << " lift_dz=" << selected_lift_dz << "m"
          << " top=" << top_count << "/" << candidates.size();

  {
    std::lock_guard<std::mutex> lk(mu_);
    data_.graspnet_top_count = top_count;
    data_.selected_graspnet_pose = selected_candidate.tcp_base;
    data_.has_selected_graspnet_pose = true;
    data_.selected_graspnet_index = selected_candidate.index;
    data_.selected_graspnet_arm = selected_plan.arm_id;
    data_.selected_graspnet_approach_deg = selected_candidate.approach_deg;
    data_.graspnet_selection_summary = summary.str();
    data_.computed_grasp_pose.header = selected_candidate.tcp_base.header;
    data_.computed_grasp_pose.pose = selected_plan.grasp;
    data_.has_computed_grasp = true;
    data_.last_plan = selected_plan;
    data_.has_plan = true;
    push_log_nolock("[graspnet] " + summary.str());
    for (const auto & line : format_plan_path_lines(selected_plan)) {
      push_log_nolock(line);
    }
  }
  if (executor_) {
    std::string step_error;
    if (!executor_->prepare_step_mode(selected_plan, &step_error)) {
      push_log("[graspnet] WARN step mode: " + step_error);
    }
  }
  out.ok = true;
  return out;
}

bool GraspRosNode::execute_last_plan()
{
  return execute_last_plan_detailed().ok;
}

GraspExecuteResult GraspRosNode::execute_last_plan_detailed()
{
  GraspExecuteResult out;
  try {
    GraspPlan plan;
    bool ik_ready = false;
    bool comm_ok = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      ik_ready = data_.ik_service_ready;
      if (!data_.has_plan) {
        out.error_title = "无抓取规划";
        out.error_message = "请先点击「计算抓取」生成规划。";
        push_log_nolock("[execute] no plan; click Compute first");
        return out;
      }
      plan = data_.last_plan;
      const bool has_joints = !joint_map_.empty();
      double age_sec = 999.0;
      if (last_joint_state_time_.nanoseconds() > 0) {
        age_sec = (now() - last_joint_state_time_).seconds();
      }
      comm_ok = has_joints && age_sec < 2.0;
    }

    if (!comm_ok) {
      out.error_title = "机械臂通信异常";
      out.error_message = "无法执行抓取：/joint_states 不可用或已超时。";
      push_log("[execute] arm comm not ok");
      return out;
    }
    if (!ik_ready) {
      out.error_title = "IK 不可用";
      out.error_message = "本包 MoveIt /compute_ik 未就绪，请用 ros2 launch nova_grasp_moveit grasp_stack.launch.py 启动。";
      push_log("[execute] IK not ready (start grasp_stack.launch.py)");
      return out;
    }
    if (!executor_ || executor_->is_busy()) {
      out.error_title = "执行器忙";
      out.error_message = "抓取序列正在执行中，请稍后再试。";
      push_log("[execute] executor busy");
      return out;
    }
    if (!executor_->start_sequence(plan)) {
      out.error_title = "启动失败";
      out.error_message = "无法启动抓取序列。";
      push_log("[execute] start_sequence failed");
      return out;
    }
    out.ok = true;
  } catch (const std::exception & ex) {
    out.error_title = "执行抓取失败";
    out.error_message = ex.what();
    push_log(std::string("[execute] ERROR ") + ex.what());
  } catch (...) {
    out.error_title = "执行抓取失败";
    out.error_message = "未知异常";
    push_log("[execute] ERROR unknown exception");
  }
  return out;
}

GraspExecuteResult GraspRosNode::execute_step_once_detailed()
{
  GraspExecuteResult out;
  try {
    bool ik_ready = false;
    bool comm_ok = false;
    bool has_plan = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      ik_ready = data_.ik_service_ready;
      has_plan = data_.has_plan;
      const bool has_joints = !joint_map_.empty();
      double age_sec = 999.0;
      if (last_joint_state_time_.nanoseconds() > 0) {
        age_sec = (now() - last_joint_state_time_).seconds();
      }
      comm_ok = has_joints && age_sec < 2.0;
    }
    if (!has_plan) {
      out.error_title = "无抓取规划";
      out.error_message = "请先点击「计算抓取」。";
      return out;
    }
    if (!comm_ok) {
      out.error_title = "机械臂通信异常";
      out.error_message = "/joint_states 不可用或已超时。";
      return out;
    }
    if (!ik_ready) {
      out.error_title = "IK 不可用";
      out.error_message = "请用 grasp_stack.launch.py 启动。";
      return out;
    }
    if (!executor_) {
      out.error_title = "执行器不可用";
      out.error_message = "executor 未初始化";
      return out;
    }
    // 若尚未 prepare（例如只加载了旧 plan），补一次
    if (executor_->step_count() <= 0) {
      GraspPlan plan;
      {
        std::lock_guard<std::mutex> lk(mu_);
        plan = data_.last_plan;
      }
      std::string err;
      if (!executor_->prepare_step_mode(plan, &err)) {
        out.error_title = "单步准备失败";
        out.error_message = err;
        return out;
      }
    }

    const GraspStepResult step = executor_->step_once();
    out.ok = step.ok;
    out.finished = step.finished;
    out.step_index = step.step_index;
    out.step_count = step.step_count;
    out.step_name = step.step_name;
    out.error_title = step.error_title;
    out.error_message = step.error_message;
    if (step.ok) {
      push_log(
        "[STEP] summary " + std::to_string(step.step_index) + "/" +
        std::to_string(step.step_count) + " " + step.step_name +
        (step.finished ? " (全部完成)" : ""));
    }
  } catch (const std::exception & ex) {
    out.error_title = "单步失败";
    out.error_message = ex.what();
  } catch (...) {
    out.error_title = "单步失败";
    out.error_message = "未知异常";
  }
  return out;
}

GraspPlannerConfig GraspRosNode::planner_config() const
{
  return planner_cfg_;
}

bool GraspRosNode::set_graspnet_topic(
  const std::string & topic, std::string * error_out)
{
  // 先成功创建新订阅，再原子替换成员；创建失败时旧订阅继续有效。
  const auto begin = topic.find_first_not_of(" \t\r\n");
  const auto end = topic.find_last_not_of(" \t\r\n");
  const std::string normalized =
    begin == std::string::npos ? std::string() : topic.substr(begin, end - begin + 1);
  if (normalized.empty()) {
    if (error_out) {
      *error_out = "话题名不能为空";
    }
    return false;
  }
  try {
    auto subscription = create_subscription<std_msgs::msg::String>(
      normalized, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        on_graspnet_candidates(msg);
      });
    {
      std::lock_guard<std::mutex> lk(mu_);
      graspnet_sub_ = std::move(subscription);
      graspnet_topic_ = normalized;
      data_.graspnet_topic = normalized;
      data_.graspnet_candidates = geometry_msgs::msg::PoseArray();
      data_.graspnet_scores.clear();
      data_.graspnet_base_candidates = geometry_msgs::msg::PoseArray();
      data_.graspnet_base_candidate_indices.clear();
      data_.graspnet_base_candidate_ik_ok.clear();
      data_.graspnet_base_candidate_a_corrected.clear();
      ++data_.graspnet_base_revision;
      data_.has_graspnet_candidates = false;
      data_.graspnet_message_count = 0;
      data_.graspnet_top_count = 0;
      data_.has_selected_graspnet_pose = false;
      data_.selected_graspnet_index = -1;
      data_.selected_graspnet_arm = -1;
      data_.graspnet_selection_summary.clear();
      last_graspnet_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    set_parameter(rclcpp::Parameter("graspnet_topic", normalized));
    push_log("[graspnet] switched topic -> " + normalized);
    return true;
  } catch (const std::exception & ex) {
    if (error_out) {
      *error_out = ex.what();
    }
    push_log(std::string("[graspnet][ERROR] switch topic failed: ") + ex.what());
    return false;
  }
}

void GraspRosNode::set_planner_config(const GraspPlannerConfig & cfg)
{
  planner_cfg_ = cfg;
  if (executor_) {
    executor_->set_config(cfg);
  }
}

void GraspRosNode::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mu_);
  last_joint_state_time_ = now();
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    joint_map_[msg->name[i]] = msg->position[i];
  }
}

void GraspRosNode::on_box_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mu_);
  data_.box_pose = *msg;
  data_.has_box_pose = true;
}

void GraspRosNode::on_graspnet_candidates(
  const std_msgs::msg::String::SharedPtr msg)
{
  geometry_msgs::msg::PoseArray parsed;
  std::vector<double> scores;
  std::string parse_note;
  if (!parse_graspnet_json(msg->data, parsed, scores, parse_note)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Invalid GraspNet JSON on %s: %s",
      graspnet_topic_.c_str(), parse_note.c_str());
    return;
  }
  if (!parse_note.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "GraspNet JSON accepted with warning: %s", parse_note.c_str());
  }
  std::lock_guard<std::mutex> lk(mu_);
  last_graspnet_time_ = now();
  data_.graspnet_candidates = std::move(parsed);
  data_.graspnet_scores = std::move(scores);
  data_.has_graspnet_candidates = !data_.graspnet_candidates.poses.empty();
  ++data_.graspnet_message_count;
}

void GraspRosNode::on_grasp_status(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mu_);
  data_.grasp_status = msg->data;
}

void GraspRosNode::on_pose_log(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string line = msg->data;
  bool notify_step = false;
  PoseStepResult step_result = PoseStepResult::Pending;
  std::string fail_reason;

  // 解析本次 IK 请求坐标，只有匹配当前等待目标后才接受后续 Ok/Fail。
  if (line.find("[INFO] IK request") != std::string::npos) {
    const auto xyz_pos = line.find("xyz=(");
    if (xyz_pos != std::string::npos) {
      double rx = 0.0;
      double ry = 0.0;
      double rz = 0.0;
      if (std::sscanf(
          line.c_str() + xyz_pos, "xyz=(%lf,%lf,%lf)", &rx, &ry, &rz) == 3)
      {
        std::lock_guard<std::mutex> lk(pose_step_mu_);
        if (pose_step_expect_valid_) {
          const double dx = rx - pose_step_expect_x_;
          const double dy = ry - pose_step_expect_y_;
          const double dz = rz - pose_step_expect_z_;
          if ((dx * dx + dy * dy + dz * dz) <= 1e-6) {  // 1 mm
            pose_step_saw_matching_request_ = true;
          }
        }
      }
    }
  }

  if (line.find("Pose command sent") != std::string::npos) {
    std::lock_guard<std::mutex> lk(pose_step_mu_);
    // 丢弃上一步迟到的成功日志，否则连续序列会在 move_xy 后跳过真正的 descend 发令。
    if (pose_step_saw_matching_request_) {
      notify_step = true;
      step_result = PoseStepResult::Ok;
    }
  } else if (line.find("[ERROR][NG] IK timeout") != std::string::npos) {
    std::lock_guard<std::mutex> lk(pose_step_mu_);
    if (pose_step_saw_matching_request_ || !pose_step_expect_valid_) {
      notify_step = true;
      step_result = PoseStepResult::Fail;
      fail_reason = "IK service timeout (非死锁，/compute_ik 12s 无响应)";
    }
  } else if (line.find("[ERROR][NG] IK failed") != std::string::npos) {
    std::lock_guard<std::mutex> lk(pose_step_mu_);
    if (pose_step_saw_matching_request_ || !pose_step_expect_valid_) {
      notify_step = true;
      step_result = PoseStepResult::Fail;
      fail_reason = "IK 无解 (MoveIt 计算失败，非 executor 死锁)";
    }
  } else if (line.find("[ERROR] Pose goal rejected") != std::string::npos) {
    notify_step = true;
    step_result = PoseStepResult::Fail;
    fail_reason = "pose goal rejected";
  } else if (line.find("[ERROR] No /joint_states") != std::string::npos ||
    line.find("[ERROR] IK seed is empty") != std::string::npos)
  {
    notify_step = true;
    step_result = PoseStepResult::Fail;
    fail_reason = line;
  }

  {
    std::lock_guard<std::mutex> lk(mu_);
    data_.pose_logs.push_back(line);
    if (data_.pose_logs.size() > 200) {
      data_.pose_logs.erase(data_.pose_logs.begin());
    }
    const bool is_err = line.find("[ERROR]") != std::string::npos ||
      line.find("[NG]") != std::string::npos;
    const std::string prefix = is_err ? "[pose_log][ERR][NG]" : "[pose_log]";
    data_.logs.push_back(prefix + " " + line);
    if (data_.logs.size() > 200) {
      data_.logs.erase(data_.logs.begin());
    }
  }

  if (notify_step) {
    std::lock_guard<std::mutex> lk(pose_step_mu_);
    pose_step_result_ = step_result;
    if (step_result == PoseStepResult::Fail) {
      pose_step_fail_reason_ = fail_reason.empty() ? line : fail_reason;
    }
    pose_step_cv_.notify_all();
  }
}

void GraspRosNode::reset_pose_step_wait()
{
  geometry_msgs::msg::Pose empty;
  reset_pose_step_wait(empty);
  std::lock_guard<std::mutex> lk(pose_step_mu_);
  pose_step_expect_valid_ = false;
}

void GraspRosNode::reset_pose_step_wait(const geometry_msgs::msg::Pose & expected_pose)
{
  std::lock_guard<std::mutex> lk(pose_step_mu_);
  pose_step_result_ = PoseStepResult::Pending;
  pose_step_fail_reason_.clear();
  pose_step_expect_x_ = expected_pose.position.x;
  pose_step_expect_y_ = expected_pose.position.y;
  pose_step_expect_z_ = expected_pose.position.z;
  pose_step_expect_valid_ = true;
  pose_step_saw_matching_request_ = false;
}

bool GraspRosNode::wait_pose_step(double timeout_sec, std::string * fail_reason)
{
  std::unique_lock<std::mutex> lk(pose_step_mu_);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(std::max(0.1, timeout_sec));
  while (pose_step_result_ == PoseStepResult::Pending) {
    if (pose_step_cv_.wait_until(lk, deadline) == std::cv_status::timeout) {
      if (fail_reason != nullptr) {
        *fail_reason = "executor 等待 pose_log 超时 (可能 pose_log 未收到 IK 结果)";
      }
      return false;
    }
  }
  if (pose_step_result_ != PoseStepResult::Ok) {
    if (fail_reason != nullptr) {
      *fail_reason = pose_step_fail_reason_.empty() ?
        "IK step failed" : pose_step_fail_reason_;
    }
    return false;
  }
  return true;
}

void GraspRosNode::push_log(const std::string & line)
{
  std::lock_guard<std::mutex> lk(mu_);
  push_log_nolock(line);
}

void GraspRosNode::push_log_nolock(const std::string & line)
{
  data_.logs.push_back(line);
  if (data_.logs.size() > 200) {
    data_.logs.erase(data_.logs.begin());
  }
}

void GraspRosNode::refresh_ik_ready()
{
  const bool ready = ik_client_ && ik_client_->service_is_ready();
  std::lock_guard<std::mutex> lk(mu_);
  data_.ik_service_ready = ready;
}

void GraspRosNode::refresh_ee_poses()
{
  const std::string ref_frame = get_parameter("ref_frame").as_string();
  const std::string ee0 = get_parameter("ee_frame_arm0").as_string();
  const std::string ee1 = get_parameter("ee_frame_arm1").as_string();
  const auto arm1 = lookup_ee_pose(ee0, ref_frame);
  const auto arm2 = lookup_ee_pose(ee1, ref_frame);
  std::lock_guard<std::mutex> lk(mu_);
  cached_arm1_ee_ = arm1;
  cached_arm2_ee_ = arm2;
}

ArmJointSnapshot GraspRosNode::extract_arm_joints(
  const std::map<std::string, double> & joints, const std::string & prefix) const
{
  ArmJointSnapshot snap;
  const auto & order = prefix == "J1_" ? kJ1Joints : kJ2Joints;
  for (const auto & name : order) {
    snap.names.push_back(name);
    auto it = joints.find(name);
    snap.positions_rad.push_back(it != joints.end() ? it->second : 0.0);
  }
  return snap;
}

EePoseSnapshot GraspRosNode::lookup_ee_pose(
  const std::string & child_frame, const std::string & ref_frame) const
{
  EePoseSnapshot out;
  out.ref_frame = ref_frame;
  out.child_frame = child_frame;
  if (!tf_buffer_) {
    return out;
  }
  try {
    const auto tf = tf_buffer_->lookupTransform(ref_frame, child_frame, tf2::TimePointZero);
    out.x = tf.transform.translation.x;
    out.y = tf.transform.translation.y;
    out.z = tf.transform.translation.z;
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    q.normalize();
    out.qx = q.x();
    out.qy = q.y();
    out.qz = q.z();
    out.qw = q.w();
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    out.roll_deg = rad_to_deg(roll);
    out.pitch_deg = rad_to_deg(pitch);
    out.yaw_deg = rad_to_deg(yaw);
    out.ok = true;
  } catch (const tf2::TransformException & ex) {
    (void)ex;
  }
  return out;
}

std::map<std::string, double> GraspRosNode::joint_positions() const
{
  std::lock_guard<std::mutex> lk(mu_);
  return joint_map_;
}

void GraspRosNode::publish_arm_joints(int arm_id, const std::vector<double> & positions_rad)
{
  publish_arm_joints_detailed(arm_id, positions_rad, nullptr);
}

bool GraspRosNode::publish_arm_joints_detailed(
  int arm_id, const std::vector<double> & positions_rad, std::string * error_out)
{
  const auto fail = [&](const std::string & msg) {
      if (error_out) {
        *error_out = msg;
      }
      push_log("[joints] " + msg);
      return false;
    };

  const auto & arm_joints = joints_for_arm(arm_id);
  // 允许 6（仅臂杆，复位用）或 8（臂杆+夹爪）
  if (positions_rad.size() < 6) {
    return fail("invalid joint count for arm_id=" + std::to_string(arm_id));
  }
  const size_t n_pub = std::min(positions_rad.size(), arm_joints.size());

  // 严格单臂：/joint_command 只带本臂关节，绝不附带另一臂
  std::vector<std::string> cmd_names;
  std::vector<double> cmd_positions;
  std::vector<double> mujoco_positions;
  bool comm_ok = false;
  {
    std::lock_guard<std::mutex> lk(mu_);
    const bool has_joints = !joint_map_.empty();
    double age_sec = 999.0;
    if (last_joint_state_time_.nanoseconds() > 0) {
      age_sec = (now() - last_joint_state_time_).seconds();
    }
    comm_ok = has_joints && age_sec < 2.0;
    if (comm_ok) {
      cmd_names.assign(arm_joints.begin(), arm_joints.begin() + static_cast<std::ptrdiff_t>(n_pub));
      cmd_positions.assign(positions_rad.begin(), positions_rad.begin() + static_cast<std::ptrdiff_t>(n_pub));
      for (size_t i = 0; i < n_pub; ++i) {
        last_commanded_[arm_joints[i]] = cmd_positions[i];
      }
      if (get_parameter("publish_mujoco_joint_array").as_bool()) {
        mujoco_positions.reserve(kControlJointOrder.size());
        for (const auto & name : kControlJointOrder) {
          const auto cmd_it = last_commanded_.find(name);
          if (cmd_it != last_commanded_.end()) {
            mujoco_positions.push_back(cmd_it->second);
            continue;
          }
          const auto js_it = joint_map_.find(name);
          mujoco_positions.push_back(js_it != joint_map_.end() ? js_it->second : 0.0);
        }
      }
    }
  }
  if (!comm_ok) {
    return fail("arm comm not ready; cannot publish joint command");
  }

  sensor_msgs::msg::JointState js_cmd;
  js_cmd.name = std::move(cmd_names);
  js_cmd.position = std::move(cmd_positions);
  // Isaac 控制图同时使用 positionCommand 和 velocityCommand；
  // 显式清零速度，避免沿用旧速度目标导致 J1/J2 固定角度滞后。
  js_cmd.velocity.assign(js_cmd.name.size(), 0.0);
  const size_t n_dof = js_cmd.name.size();
  const int burst = std::max(1, static_cast<int>(get_parameter("joint_command_burst_count").as_int()));
  for (int i = 0; i < burst; ++i) {
    js_cmd.header.stamp = now();
    joint_command_pub_->publish(js_cmd);
  }

  if (!mujoco_positions.empty()) {
    std_msgs::msg::Float64MultiArray array_cmd;
    array_cmd.data = std::move(mujoco_positions);
    joint_cmd_pub_->publish(array_cmd);
  }

  push_log(
    "[joints] published arm_id=" + std::to_string(arm_id) +
    " only " + std::to_string(n_dof) + " DOF (strict single-arm)" +
    " topic=" + get_parameter("joint_command_topic").as_string() +
    " burst=" + std::to_string(burst));
  return true;
}

bool GraspRosNode::publish_gripper_joint_pair_detailed(
  int arm_id, double j7, double j8, std::string * error_out)
{
  const auto fail = [&](const std::string & msg) {
      if (error_out) {
        *error_out = msg;
      }
      push_log("[gripper] " + msg);
      return false;
    };

  const auto & arm_joints = joints_for_arm(arm_id);
  if (arm_joints.size() < 8) {
    return fail("invalid gripper arm_id=" + std::to_string(arm_id));
  }

  // 严格单臂夹爪：只发本臂 J7/J8，不带臂杆、不带另一臂
  const std::string & j7_name = arm_joints[6];
  const std::string & j8_name = arm_joints[7];
  std::vector<double> mujoco_positions;
  bool comm_ok = false;
  {
    std::lock_guard<std::mutex> lk(mu_);
    const bool has_joints = !joint_map_.empty();
    double age_sec = 999.0;
    if (last_joint_state_time_.nanoseconds() > 0) {
      age_sec = (now() - last_joint_state_time_).seconds();
    }
    comm_ok = has_joints && age_sec < 2.0;
    if (comm_ok) {
      last_commanded_[j7_name] = j7;
      last_commanded_[j8_name] = j8;
      if (get_parameter("publish_mujoco_joint_array").as_bool()) {
        mujoco_positions.reserve(kControlJointOrder.size());
        for (const auto & name : kControlJointOrder) {
          const auto cmd_it = last_commanded_.find(name);
          if (cmd_it != last_commanded_.end()) {
            mujoco_positions.push_back(cmd_it->second);
            continue;
          }
          const auto js_it = joint_map_.find(name);
          mujoco_positions.push_back(js_it != joint_map_.end() ? js_it->second : 0.0);
        }
      }
    }
  }
  if (!comm_ok) {
    return fail("arm comm not ready; cannot publish gripper command");
  }

  sensor_msgs::msg::JointState js_cmd;
  js_cmd.name = {j7_name, j8_name};
  js_cmd.position = {j7, j8};
  js_cmd.velocity = {0.0, 0.0};
  const int burst = std::max(1, static_cast<int>(get_parameter("joint_command_burst_count").as_int()));
  for (int i = 0; i < burst; ++i) {
    js_cmd.header.stamp = now();
    joint_command_pub_->publish(js_cmd);
  }
  if (!mujoco_positions.empty()) {
    std_msgs::msg::Float64MultiArray array_cmd;
    array_cmd.data = std::move(mujoco_positions);
    joint_cmd_pub_->publish(array_cmd);
  }
  push_log(
    "[gripper] arm_id=" + std::to_string(arm_id) +
    " j7=" + std::to_string(j7) + " j8=" + std::to_string(j8) +
    " (gripper-only, other arm untouched)");
  return true;
}

bool GraspRosNode::publish_gripper_opening_detailed(
  int arm_id, double opening_m, std::string * error_out)
{
  double j7 = 0.0;
  double j8 = 0.0;
  gripper_opening_to_joint_pair(opening_m, j7, j8);
  return publish_gripper_joint_pair_detailed(arm_id, j7, j8, error_out);
}

bool GraspRosNode::publish_gripper_joints_detailed(int arm_id, bool open_gripper, std::string * error_out)
{
  const double opening = open_gripper ? planner_cfg_.gripper_open_m : planner_cfg_.gripper_close_m;
  return publish_gripper_opening_detailed(arm_id, opening, error_out);
}

void GraspRosNode::set_settle_timing(double step_settle_sec, double gripper_settle_sec)
{
  step_settle_sec_ = std::max(0.1, step_settle_sec);
  gripper_settle_sec_ = std::max(0.1, gripper_settle_sec);
  if (executor_) {
    executor_->set_timing(step_settle_sec_, gripper_settle_sec_);
  }
}

namespace
{

void publish_arm_pose_msg(
  GraspRosNode * node,
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_id_pub,
  rclcpp::Publisher<nova_grasp_moveit::msg::ArmPose>::SharedPtr arm_pose_pub,
  int arm_id, const geometry_msgs::msg::PoseStamped & pose)
{
  // /nova_arm_id 供旧 gripper open/close 通道选择机械臂；ArmPose 自身也携带 arm_id，
  // 因此位姿 IK 不依赖两个话题的到达顺序。
  std_msgs::msg::Int32 arm_msg;
  arm_msg.data = arm_id;
  arm_id_pub->publish(arm_msg);
  nova_grasp_moveit::msg::ArmPose out;
  out.arm_id = arm_id;
  out.pose = pose;
  arm_pose_pub->publish(out);
  (void)node;
}

}  // namespace

void GraspRosNode::send_arm_pose_goal_quat(
  int arm_id, double x, double y, double z,
  double qx, double qy, double qz, double qw,
  const std::string & frame_id)
{
  tf2::Quaternion q(qx, qy, qz, qw);
  if (q.length2() < 1e-12) {
    push_log("[pose][ERR][NG] invalid quaternion");
    return;
  }
  q.normalize();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id.empty() ? get_parameter("ref_frame").as_string() : frame_id;
  pose.header.stamp = now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation = tf2::toMsg(q);

  publish_arm_pose_msg(this, arm_id_pub_, arm_pose_pub_, arm_id, pose);

  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  push_log(
    "[pose] arm_id=" + std::to_string(arm_id) +
    " xyz=(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")"
    " via=tf_quat rpy≈(" + std::to_string(rad_to_deg(roll)) + "," +
    std::to_string(rad_to_deg(pitch)) + "," + std::to_string(rad_to_deg(yaw)) + ")");
}

void GraspRosNode::send_arm_pose_goal(
  int arm_id, double x, double y, double z,
  double roll_deg, double pitch_deg, double yaw_deg,
  const std::string & frame_id)
{
  tf2::Quaternion q;
  q.setRPY(
    roll_deg * M_PI / 180.0,
    pitch_deg * M_PI / 180.0,
    yaw_deg * M_PI / 180.0);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id.empty() ? get_parameter("ref_frame").as_string() : frame_id;
  pose.header.stamp = now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation = tf2::toMsg(q);

  publish_arm_pose_msg(this, arm_id_pub_, arm_pose_pub_, arm_id, pose);
  push_log(
    "[pose] arm_id=" + std::to_string(arm_id) +
    " xyz=(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")"
    " via=rpy rpy=(" + std::to_string(roll_deg) + "," + std::to_string(pitch_deg) + "," +
    std::to_string(yaw_deg) + ")");
}

void GraspRosNode::send_gripper_for_arm(int arm_id, const std::string & cmd)
{
  std_msgs::msg::Int32 arm_msg;
  arm_msg.data = arm_id;
  arm_id_pub_->publish(arm_msg);
  send_gripper(cmd);
  const bool open_gripper = cmd == "open";
  std::string err;
  if (!publish_gripper_joints_detailed(arm_id, open_gripper, &err)) {
    push_log("[gripper] " + (err.empty() ? "publish joint command failed" : err));
  } else {
    push_log(
      std::string("[gripper] arm_id=") + std::to_string(arm_id) + " " + cmd +
      " via /joint_command");
  }
}

void GraspRosNode::send_gripper(const std::string & cmd)
{
  if (executor_) {
    executor_->send_gripper(cmd);
  }
}

}  // namespace nova_grasp_moveit
