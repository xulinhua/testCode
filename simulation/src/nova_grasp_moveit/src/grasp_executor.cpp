#include "nova_grasp_moveit/grasp_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <thread>

namespace nova_grasp_moveit
{

GraspExecutor::GraspExecutor(
  rclcpp::Publisher<nova_grasp_moveit::msg::ArmPose>::SharedPtr arm_pose_pub,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_pub,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub,
  GraspPlannerConfig cfg,
  double step_settle_sec,
  double gripper_settle_sec)
: cfg_(cfg),
  step_settle_sec_(step_settle_sec),
  gripper_settle_sec_(gripper_settle_sec),
  arm_pose_pub_(std::move(arm_pose_pub)),
  gripper_pub_(std::move(gripper_pub)),
  status_pub_(std::move(status_pub))
{
}

void GraspExecutor::set_config(const GraspPlannerConfig & cfg)
{
  cfg_ = cfg;
}

void GraspExecutor::set_timing(double step_settle_sec, double gripper_settle_sec)
{
  step_settle_sec_ = std::max(0.1, step_settle_sec);
  gripper_settle_sec_ = std::max(0.1, gripper_settle_sec);
}

bool GraspExecutor::is_busy() const
{
  return busy_.load();
}

void GraspExecutor::set_callbacks(StatusCallback status_cb, LogCallback log_cb)
{
  status_cb_ = std::move(status_cb);
  log_cb_ = std::move(log_cb);
}

void GraspExecutor::set_gripper_apply_callback(GripperApplyCallback cb)
{
  gripper_apply_cb_ = std::move(cb);
}

void GraspExecutor::set_current_ee_callback(CurrentEeCallback cb)
{
  current_ee_cb_ = std::move(cb);
}

void GraspExecutor::set_pose_step_callbacks(PoseStepResetCallback reset_cb, PoseStepWaitCallback wait_cb)
{
  pose_step_reset_cb_ = std::move(reset_cb);
  pose_step_wait_cb_ = std::move(wait_cb);
}

bool GraspExecutor::start_sequence(const GraspPlan & plan)
{
  if (busy_.exchange(true)) {
    return false;
  }
  reset_step_mode();
  shutdown_.store(false);
  std::thread([this, plan]() {
      run_sequence(plan);
      busy_.store(false);
    }).detach();
  return true;
}

bool GraspExecutor::start_from_grasp_pose(const geometry_msgs::msg::PoseStamped & grasp_pose)
{
  const std::string frame = grasp_pose.header.frame_id.empty() ? "base_link" : grasp_pose.header.frame_id;
  const GraspPlan plan = plan_grasp_from_pose(grasp_pose.pose, frame, cfg_);
  return start_sequence(plan);
}

void GraspExecutor::publish_status(const std::string & text)
{
  if (status_pub_) {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }
  if (status_cb_) {
    status_cb_(text);
  }
}

void GraspExecutor::publish_log(const std::string & text)
{
  if (log_cb_) {
    log_cb_(text);
  }
}

void GraspExecutor::send_arm_pose(int arm_id, const geometry_msgs::msg::PoseStamped & pose)
{
  nova_grasp_moveit::msg::ArmPose msg;
  msg.arm_id = arm_id;
  msg.pose = pose;
  arm_pose_pub_->publish(msg);
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  publish_log(
    ">>> /nova_target_arm_pose arm_id=" + std::to_string(arm_id) +
    " frame=" + pose.header.frame_id +
    " pos=(" + std::to_string(p.x) + "," + std::to_string(p.y) + "," + std::to_string(p.z) + ")"
    " quat=(" + std::to_string(q.x) + "," + std::to_string(q.y) + "," +
    std::to_string(q.z) + "," + std::to_string(q.w) + ")");
}

bool GraspExecutor::send_arm_pose_and_wait(
  int arm_id, const geometry_msgs::msg::PoseStamped & pose, const char * step,
  bool do_settle)
{
  if (pose_step_reset_cb_) {
    pose_step_reset_cb_(pose.pose);
  }
  send_arm_pose(arm_id, pose);

  std::string fail_reason;
  constexpr double kIkTimeoutSec = 5.0;
  if (pose_step_wait_cb_) {
    if (!pose_step_wait_cb_(kIkTimeoutSec, &fail_reason)) {
      publish_status(std::string("ERROR IK at ") + step);
      publish_log(
        std::string("[executor] ABORT ") + step + " — " +
        (fail_reason.empty() ? "unknown IK failure" : fail_reason));
      return false;
    }
  } else {
    sleep_sec(kIkTimeoutSec);
  }
  if (do_settle) {
    if (!current_ee_cb_) {
      sleep_sec(step_settle_sec_);
      return true;
    }

    // /compute_ik 返回只表示关节目标已发布，不表示 Isaac 已运动到位。
    constexpr double kPositionToleranceM = 0.015;
    constexpr double kOrientationToleranceRad = 5.0 * M_PI / 180.0;

    geometry_msgs::msg::Pose start_ee;
    double travel_m = 0.25;
    if (current_ee_cb_(arm_id, start_ee)) {
      const double dx0 = start_ee.position.x - pose.pose.position.x;
      const double dy0 = start_ee.position.y - pose.pose.position.y;
      const double dz0 = start_ee.position.z - pose.pose.position.z;
      travel_m = std::sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
    }
    // 短位移快判；长位移（home→pregrasp ~0.4m）在慢实时下需要更长墙钟。
    // 约 10 s/m，下限 3s、上限 12s；到位立即返回。
    const double wall_timeout_sec = std::clamp(3.0 + travel_m * 10.0, 3.0, 12.0);
    publish_log(
      std::string("[executor] wait ") + step +
      " travel=" + std::to_string(travel_m) +
      "m wall_timeout=" + std::to_string(wall_timeout_sec) + "s");

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(wall_timeout_sec);
    double last_pos_err = 999.0;
    double last_rot_err = 999.0;
    geometry_msgs::msg::Pose actual;
    while (std::chrono::steady_clock::now() < deadline && !shutdown_.load()) {
      if (current_ee_cb_(arm_id, actual)) {
        const double dx = actual.position.x - pose.pose.position.x;
        const double dy = actual.position.y - pose.pose.position.y;
        const double dz = actual.position.z - pose.pose.position.z;
        last_pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);

        const auto & qa = actual.orientation;
        const auto & qt = pose.pose.orientation;
        const double dot = std::clamp(
          std::abs(qa.x * qt.x + qa.y * qt.y + qa.z * qt.z + qa.w * qt.w),
          0.0, 1.0);
        last_rot_err = 2.0 * std::acos(dot);
        if (last_pos_err <= kPositionToleranceM &&
          last_rot_err <= kOrientationToleranceRad)
        {
          publish_log(
            std::string("[executor] REACHED ") + step +
            " pos_err=" + std::to_string(last_pos_err) +
            "m rot_err_deg=" + std::to_string(last_rot_err * 180.0 / M_PI));
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    publish_log(
      std::string("[executor] NOT REACHED ") + step +
      " pos_err=" + std::to_string(last_pos_err) +
      "m rot_err_deg=" + std::to_string(last_rot_err * 180.0 / M_PI) +
      " after wall_timeout=" + std::to_string(wall_timeout_sec) + "s" +
      " (arm still en-route; descend skipped)");
    // 位置已够近时放行（姿态在仿真里常抖）。
    if (last_pos_err <= 0.025) {
      publish_log(
        std::string("[executor] CONTINUE ") + step +
        " despite rot/settle timeout (pos close enough)");
      return true;
    }
    return false;
  }
  return true;
}

void GraspExecutor::send_gripper(const std::string & cmd)
{
  std_msgs::msg::String msg;
  msg.data = cmd;
  gripper_pub_->publish(msg);
  publish_log(">>> /nova_gripper_goal " + cmd);
}

void GraspExecutor::apply_gripper_opening(int arm_id, double opening_m)
{
  if (gripper_apply_cb_) {
    gripper_apply_cb_(arm_id, opening_m);
  }
  publish_log(
    ">>> gripper arm_id=" + std::to_string(arm_id) +
    " opening_m=" + std::to_string(opening_m));
}

void GraspExecutor::request_shutdown()
{
  shutdown_.store(true);
}

void GraspExecutor::sleep_sec(double sec)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(sec);
  while (std::chrono::steady_clock::now() < deadline && !shutdown_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

namespace
{

geometry_msgs::msg::Pose vertical_axis_pose(
  const geometry_msgs::msg::Pose & planned,
  const geometry_msgs::msg::Pose & ee_now)
{
  // 垂直段：锁定当前 XY，只改 Z；姿态用规划顶抓（勿继承歪斜 EE）
  geometry_msgs::msg::Pose out = planned;
  out.position.x = ee_now.position.x;
  out.position.y = ee_now.position.y;
  out.position.z = planned.position.z;
  out.orientation = planned.orientation;
  return out;
}

}  // namespace

void GraspExecutor::prepare_transit_poses(GraspPlan & plan)
{
  geometry_msgs::msg::Pose ee;
  const bool have_ee = current_ee_cb_ && current_ee_cb_(plan.arm_id, ee);
  if (plan.preserve_waypoints) {
    // GraspNet: 不要在当前 XY 抬到 pregrasp.z（该高度只在目标附近可达）。
    // 直接走已 IK 校验的 pregrasp / grasp / lift。
    plan.has_raise = false;
    plan.need_vertical_raise = false;
  } else {
    fill_plan_waypoints(plan, cfg_, have_ee ? &ee : nullptr);
  }
  if (!have_ee) {
    publish_log("[executor] WARN no current EE; skip raise, keep plan orientation");
  } else {
    publish_log(
      "[executor] current EE (fresh TF) " + format_pose_xyz_q(ee));
  }
}

std::vector<GraspExecutor::StepItem> GraspExecutor::build_step_list(const GraspPlan & plan) const
{
  // 单步与连续模式共享这一顺序契约。Raise/Reorient 是条件步骤，
  // 其余步骤始终存在，确保“先开爪、闭合后再抬升”。
  std::vector<StepItem> items;
  {
    StepItem s;
    s.kind = StepKind::GripperOpen;
    s.name = "open";
    items.push_back(s);
  }
  if (plan.has_raise && plan.need_vertical_raise) {
    StepItem s;
    s.kind = StepKind::Raise;
    s.name = "raise";
    s.pose = plan.raise;
    s.has_pose = true;
    items.push_back(s);
  }
  {
    StepItem s;
    s.kind = StepKind::MoveXy;
    s.name = "move_xy";
    s.pose = plan.pregrasp;
    s.has_pose = true;
    items.push_back(s);
  }
  if (plan.has_reorient) {
    StepItem s;
    s.kind = StepKind::Reorient;
    s.name = "reorient";
    s.pose = plan.reorient;
    s.has_pose = true;
    items.push_back(s);
  }
  {
    StepItem s;
    s.kind = StepKind::Descend;
    s.name = "descend";
    s.pose = plan.grasp;
    s.has_pose = true;
    items.push_back(s);
  }
  {
    StepItem s;
    s.kind = StepKind::GripperClose;
    s.name = "close";
    items.push_back(s);
  }
  {
    StepItem s;
    s.kind = StepKind::Lift;
    s.name = "lift";
    s.pose = plan.lift;
    s.has_pose = true;
    items.push_back(s);
  }
  return items;
}

void GraspExecutor::reset_step_mode()
{
  std::lock_guard<std::mutex> lk(step_mu_);
  step_ready_ = false;
  step_cursor_ = 0;
  step_items_.clear();
}

int GraspExecutor::step_index() const
{
  std::lock_guard<std::mutex> lk(step_mu_);
  return step_cursor_;
}

int GraspExecutor::step_count() const
{
  std::lock_guard<std::mutex> lk(step_mu_);
  return static_cast<int>(step_items_.size());
}

bool GraspExecutor::prepare_step_mode(const GraspPlan & plan, std::string * error_out)
{
  if (busy_.load()) {
    if (error_out) {
      *error_out = "连续执行中，请先等完成或重启后再单步";
    }
    return false;
  }
  GraspPlan p = plan;
  prepare_transit_poses(p);
  auto items = build_step_list(p);
  const int n = static_cast<int>(items.size());
  {
    std::lock_guard<std::mutex> lk(step_mu_);
    step_plan_ = p;
    step_items_ = std::move(items);
    step_cursor_ = 0;
    step_ready_ = true;
  }
  publish_log("[step] prepared " + std::to_string(n) + " steps; click 单步 to advance");
  for (const auto & line : format_plan_path_lines(p)) {
    publish_log(line);
  }
  return true;
}

GraspStepResult GraspExecutor::step_once()
{
  GraspStepResult out;
  if (busy_.exchange(true)) {
    out.error_title = "执行器忙";
    out.error_message = "连续执行或上一步尚未结束";
    return out;
  }

  StepItem item;
  int cursor = 0;
  int count = 0;
  GraspPlan plan;
  {
    std::lock_guard<std::mutex> lk(step_mu_);
    out.step_count = static_cast<int>(step_items_.size());
    count = out.step_count;
    if (!step_ready_ || step_items_.empty()) {
      busy_.store(false);
      out.error_title = "无单步序列";
      out.error_message = "请先「计算抓取」，再点「单步」";
      return out;
    }
    if (step_cursor_ >= count) {
      busy_.store(false);
      out.ok = true;
      out.finished = true;
      out.step_index = count;
      out.step_name = "DONE";
      return out;
    }
    cursor = step_cursor_;
    item = step_items_[static_cast<size_t>(cursor)];
    plan = step_plan_;
  }

  out.step_index = cursor + 1;
  out.step_name = item.name;
  out.step_count = count;
  const std::string tag =
    "[STEP] " + std::to_string(out.step_index) + "/" + std::to_string(count) + " " + item.name;
  publish_status("STEP " + std::to_string(out.step_index) + "/" + std::to_string(count) + " " + item.name);
  publish_log(tag + " ——— begin ———");

  // 执行前：当前实测位姿
  geometry_msgs::msg::Pose ee_before;
  const bool have_before = current_ee_cb_ && current_ee_cb_(plan.arm_id, ee_before);
  if (have_before) {
    publish_log(tag + " BEFORE ee " + format_pose_xyz_q(ee_before));
  } else {
    publish_log(tag + " BEFORE ee (TF unavailable)");
  }

  bool ok = true;
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.frame_id = plan.frame_id;
  geometry_msgs::msg::Pose planned_pose;

  try {
    switch (item.kind) {
      case StepKind::GripperOpen:
        publish_log(
          tag + " PLAN gripper open_m=" + std::to_string(cfg_.gripper_open_m));
        apply_gripper_opening(plan.arm_id, cfg_.gripper_open_m);
        break;
      case StepKind::GripperClose:
        publish_log(
          tag + " PLAN gripper close_m=" + std::to_string(cfg_.gripper_close_m));
        apply_gripper_opening(plan.arm_id, cfg_.gripper_close_m);
        break;
      case StepKind::Raise:
      case StepKind::MoveXy:
      case StepKind::Reorient:
      case StepKind::Descend:
      case StepKind::Lift: {
        stamped.pose = item.pose;
        if (item.kind == StepKind::Reorient || item.kind == StepKind::Descend ||
          item.kind == StepKind::Lift)
        {
          if (item.kind == StepKind::Reorient) {
            // 用规划腕位（指尖固定盒心）；勿锁 ee_before 腕部，否则 yaw 一变 TCP 就偏
            stamped.pose = item.pose;
          } else if (have_before && !plan.preserve_waypoints) {
            // 盒子模式锁定实测 XY，抵消前一步到位误差；GraspNet 模式必须保留
            // 倾斜接近轴计算出的完整 XYZ，不能套用 vertical_axis_pose。
            stamped.pose = vertical_axis_pose(item.pose, ee_before);
          }
        }
        planned_pose = stamped.pose;
        publish_log(tag + " PLAN  " + format_pose_xyz_q(planned_pose));
        ok = send_arm_pose_and_wait(plan.arm_id, stamped, item.name.c_str(), true);
        break;
      }
    }
  } catch (const std::exception & ex) {
    ok = false;
    out.error_message = ex.what();
  }

  // 执行后：到位实测位姿（再查一次 TF）
  geometry_msgs::msg::Pose ee_after;
  const bool have_after = current_ee_cb_ && current_ee_cb_(plan.arm_id, ee_after);
  if (have_after) {
    publish_log(tag + " AFTER ee " + format_pose_xyz_q(ee_after));
    if (item.has_pose) {
      const double dx = ee_after.position.x - planned_pose.position.x;
      const double dy = ee_after.position.y - planned_pose.position.y;
      const double dz = ee_after.position.z - planned_pose.position.z;
      const double err = std::sqrt(dx * dx + dy * dy + dz * dz);
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(4)
          << tag << " ERR_xyz=(" << dx << "," << dy << "," << dz << ") |err|=" << err << "m";
      publish_log(oss.str());
    }
  } else {
    publish_log(tag + " AFTER ee (TF unavailable)");
  }

  if (!ok) {
    busy_.store(false);
    out.error_title = "单步失败";
    if (out.error_message.empty()) {
      out.error_message = "步骤 " + item.name + " 失败";
    }
    publish_log(tag + " FAIL");
    return out;
  }

  {
    std::lock_guard<std::mutex> lk(step_mu_);
    step_cursor_ = cursor + 1;
    out.finished = step_cursor_ >= static_cast<int>(step_items_.size());
  }
  out.ok = true;
  publish_log(tag + " ——— ok ———");
  if (out.finished) {
    publish_status("STEP DONE");
    publish_log("[STEP] all steps finished");
  }
  busy_.store(false);
  return out;
}

void GraspExecutor::run_sequence(GraspPlan plan)
{
  try {
    if (shutdown_.load()) {
      return;
    }

    // 与单步共用同一套步骤列表，避免连续路径漏掉 descend / close。
    prepare_transit_poses(plan);
    const auto items = build_step_list(plan);

    publish_status("EXECUTE path");
    for (const auto & line : format_plan_path_lines(plan)) {
      publish_log(line);
    }
    publish_log(
      "[executor] continuous uses same " + std::to_string(items.size()) +
      " steps as 单步");

    for (size_t i = 0; i < items.size() && !shutdown_.load(); ++i) {
      const auto & item = items[i];
      const std::string tag =
        "[SEQ] " + std::to_string(i + 1) + "/" + std::to_string(items.size()) +
        " " + item.name;
      publish_status(
        "STEP " + std::to_string(i + 1) + "/" + std::to_string(items.size()) +
        " " + item.name);
      publish_log(tag + " ——— begin ———");

      bool ok = true;
      if (item.kind == StepKind::GripperOpen) {
        apply_gripper_opening(plan.arm_id, cfg_.gripper_open_m);
        sleep_sec(gripper_settle_sec_);
      } else if (item.kind == StepKind::GripperClose) {
        apply_gripper_opening(plan.arm_id, cfg_.gripper_close_m);
        sleep_sec(gripper_settle_sec_);
      } else if (item.has_pose) {
        geometry_msgs::msg::PoseStamped stamped;
        stamped.header.frame_id = plan.frame_id;
        stamped.pose = item.pose;
        if ((item.kind == StepKind::Descend || item.kind == StepKind::Lift) &&
          !plan.preserve_waypoints)
        {
          geometry_msgs::msg::Pose ee_now;
          if (current_ee_cb_ && current_ee_cb_(plan.arm_id, ee_now)) {
            stamped.pose = vertical_axis_pose(item.pose, ee_now);
          }
        }
        publish_log(tag + " PLAN  " + format_pose_xyz_q(stamped.pose));
        ok = send_arm_pose_and_wait(plan.arm_id, stamped, item.name.c_str(), true);
      }

      if (!ok || shutdown_.load()) {
        publish_status(
          shutdown_.load() ? "CANCELLED" : ("ABORT " + item.name));
        publish_log(tag + " FAIL");
        return;
      }
      publish_log(tag + " ——— ok ———");
    }

    publish_status("DONE");
    publish_log("[executor] sequence DONE");
  } catch (const std::exception & ex) {
    publish_status(std::string("ERROR ") + ex.what());
    publish_log(std::string("[executor] ERROR ") + ex.what());
  }
}

}  // namespace nova_grasp_moveit
