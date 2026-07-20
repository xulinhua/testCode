#include "nova_grasp_moveit/grasp_ros_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <regex>
#include <sstream>
#include <iomanip>

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

}  // namespace

GraspRosNode::GraspRosNode(const rclcpp::NodeOptions & options)
: Node("grasp_qt_ui_ros_node", options)
{
  declare_parameter<std::string>("box_topic", "/box_pose");
  declare_parameter<std::string>("pose_frame", "base_link");
  declare_parameter<std::string>("target_arm_pose_topic", "/nova_target_arm_pose");
  declare_parameter<std::string>("gripper_topic", "/nova_gripper_goal");
  declare_parameter<std::string>("status_topic", "/nova_grasp/status");
  declare_parameter<std::string>("pose_log_topic", "/nova_pose_log");
  declare_parameter<double>("arm_split_x", 0.40);
  declare_parameter<double>("pregrasp_z_offset", 0.15);
  declare_parameter<double>("lift_z_offset", 0.15);
  declare_parameter<double>("box_grasp_z_offset", 0.0);
  declare_parameter<double>("ee_tcp_z_offset", 0.20);
  declare_parameter<double>("approach_raise_clearance", 0.08);
  declare_parameter<double>("min_approach_clearance", 0.15);
  declare_parameter<double>("grasp_yaw_offset_deg", 90.0);
  declare_parameter<double>("step_settle_sec", 2.0);
  declare_parameter<double>("gripper_settle_sec", 0.8);
  declare_parameter<double>("gripper_open_m", 0.08);
  declare_parameter<double>("gripper_close_m", 0.02);
  declare_parameter<bool>("use_top_down_if_identity", true);
  declare_parameter<bool>("use_box_yaw_only", true);
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
  planner_cfg_.approach_raise_clearance = get_parameter("approach_raise_clearance").as_double();
  planner_cfg_.min_approach_clearance = get_parameter("min_approach_clearance").as_double();
  planner_cfg_.grasp_yaw_offset_deg = get_parameter("grasp_yaw_offset_deg").as_double();
  planner_cfg_.use_top_down_if_identity = get_parameter("use_top_down_if_identity").as_bool();
  planner_cfg_.use_box_yaw_only = get_parameter("use_box_yaw_only").as_bool();
  planner_cfg_.gripper_open_m = get_parameter("gripper_open_m").as_double();
  planner_cfg_.gripper_close_m = get_parameter("gripper_close_m").as_double();
  step_settle_sec_ = get_parameter("step_settle_sec").as_double();
  gripper_settle_sec_ = get_parameter("gripper_settle_sec").as_double();

  const auto target_topic = get_parameter("target_arm_pose_topic").as_string();
  const auto gripper_topic = get_parameter("gripper_topic").as_string();
  const auto status_topic = get_parameter("status_topic").as_string();
  const auto pose_log_topic = get_parameter("pose_log_topic").as_string();
  const auto box_topic = get_parameter("box_topic").as_string();

  arm_pose_pub_ = create_publisher<nova_grasp_moveit::msg::ArmPose>(target_topic, 10);
  gripper_pub_ = create_publisher<std_msgs::msg::String>(gripper_topic, 10);
  grasp_status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
  joint_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/arm_controller/commands", 10);
  const auto joint_cmd_topic = get_parameter("joint_command_topic").as_string();
  auto joint_cmd_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  joint_cmd_qos.reliable();
  joint_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_cmd_topic, joint_cmd_qos);
  arm_id_pub_ = create_publisher<std_msgs::msg::Int32>("/nova_arm_id", 10);

  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 50,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(msg); });
  box_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    box_topic, box_pose_qos(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_box_pose(msg); });
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
    [this]() { reset_pose_step_wait(); },
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
    "ready box=" + box_topic + " target=" + target_topic + " gripper=" + gripper_topic +
    " joint_cmd=" + joint_cmd_topic);
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

GraspRosSnapshot GraspRosNode::snapshot()
{
  GraspRosSnapshot snap;
  rclcpp::Time last_joint_time{0, 0, RCL_ROS_TIME};
  {
    std::lock_guard<std::mutex> lk(mu_);
    snap = data_;
    snap.executor_busy = executor_ && executor_->is_busy();
    snap.ik_service_ready = data_.ik_service_ready;
    snap.arm1_joints = extract_arm_joints(joint_map_, "J1_");
    snap.arm2_joints = extract_arm_joints(joint_map_, "J2_");
    snap.joint_states_count = static_cast<int>(joint_map_.size());
    last_joint_time = last_joint_state_time_;
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

bool GraspRosNode::compute_grasp_from_box()
{
  return compute_grasp_from_box_detailed().ok;
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

bool GraspRosNode::execute_from_computed_grasp()
{
  geometry_msgs::msg::PoseStamped grasp;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!data_.has_computed_grasp) {
      push_log_nolock("[execute] no computed grasp");
      return false;
    }
    grasp = data_.computed_grasp_pose;
  }
  if (!executor_ || executor_->is_busy()) {
    push_log("[execute] executor busy");
    return false;
  }
  return executor_->start_from_grasp_pose(grasp);
}

GraspPlannerConfig GraspRosNode::planner_config() const
{
  return planner_cfg_;
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
  if (line.find("Pose command sent") != std::string::npos) {
    notify_step = true;
    step_result = PoseStepResult::Ok;
  } else if (line.find("[ERROR][NG] IK timeout") != std::string::npos) {
    notify_step = true;
    step_result = PoseStepResult::Fail;
    fail_reason = "IK service timeout (非死锁，/compute_ik 12s 无响应)";
  } else if (line.find("[ERROR][NG] IK failed") != std::string::npos) {
    notify_step = true;
    step_result = PoseStepResult::Fail;
    fail_reason = "IK 无解 (MoveIt 计算失败，非 executor 死锁)";
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
  std::lock_guard<std::mutex> lk(pose_step_mu_);
  pose_step_result_ = PoseStepResult::Pending;
  pose_step_fail_reason_.clear();
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
