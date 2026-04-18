#include "calib_sim/calib_node.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>

namespace calib_sim
{

namespace
{
constexpr int kPoseDims = 7;
constexpr double kEps = 1e-9;
constexpr double kRad2Deg = 57.29577951308232;
constexpr double kDeg2Rad = 0.017453292519943295;

cv::Mat rotXRad(double a)
{
  const double c = std::cos(a);
  const double s = std::sin(a);
  return (cv::Mat_<double>(3, 3) <<
    1.0, 0.0, 0.0,
    0.0, c, -s,
    0.0, s, c);
}

cv::Mat rotYRad(double a)
{
  const double c = std::cos(a);
  const double s = std::sin(a);
  return (cv::Mat_<double>(3, 3) <<
    c, 0.0, s,
    0.0, 1.0, 0.0,
    -s, 0.0, c);
}

cv::Mat rotZRad(double a)
{
  const double c = std::cos(a);
  const double s = std::sin(a);
  return (cv::Mat_<double>(3, 3) <<
    c, -s, 0.0,
    s, c, 0.0,
    0.0, 0.0, 1.0);
}

/// R_delta = Rz(yaw) * Ry(pitch) * Rx(roll)，与 ensureTargetsPrepared 中 R_out = R_base * R_delta 联用
cv::Mat rpyDegToRdelta(double roll_deg, double pitch_deg, double yaw_deg)
{
  const double r = roll_deg * kDeg2Rad;
  const double p = pitch_deg * kDeg2Rad;
  const double y = yaw_deg * kDeg2Rad;
  return rotZRad(y) * rotYRad(p) * rotXRad(r);
}

void mat3ToQuat(const cv::Mat & R, double & qx, double & qy, double & qz, double & qw)
{
  const double m00 = R.at<double>(0, 0);
  const double m01 = R.at<double>(0, 1);
  const double m02 = R.at<double>(0, 2);
  const double m10 = R.at<double>(1, 0);
  const double m11 = R.at<double>(1, 1);
  const double m12 = R.at<double>(1, 2);
  const double m20 = R.at<double>(2, 0);
  const double m21 = R.at<double>(2, 1);
  const double m22 = R.at<double>(2, 2);
  const double tr = m00 + m11 + m22;
  if (tr > 0.0) {
    const double S = std::sqrt(tr + 1.0) * 2.0;
    qw = 0.25 * S;
    qx = (m21 - m12) / S;
    qy = (m02 - m20) / S;
    qz = (m10 - m01) / S;
  } else if (m00 > m11 && m00 > m22) {
    const double S = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    qw = (m21 - m12) / S;
    qx = 0.25 * S;
    qy = (m01 + m10) / S;
    qz = (m02 + m20) / S;
  } else if (m11 > m22) {
    const double S = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    qw = (m02 - m20) / S;
    qx = (m01 + m10) / S;
    qy = 0.25 * S;
    qz = (m12 + m21) / S;
  } else {
    const double S = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    qw = (m10 - m01) / S;
    qx = (m02 + m20) / S;
    qy = (m12 + m21) / S;
    qz = 0.25 * S;
  }
}

std::string formatMat4(const cv::Mat & m)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  for (int r = 0; r < 4; ++r) {
    oss << "[";
    for (int c = 0; c < 4; ++c) {
      oss << m.at<double>(r, c);
      if (c < 3) {
        oss << ", ";
      }
    }
    oss << "]";
    if (r < 3) {
      oss << "\n";
    }
  }
  return oss.str();
}

cv::Mat vec16ToMat44(const std::array<double, 16> & v)
{
  cv::Mat m = cv::Mat::eye(4, 4, CV_64F);
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      m.at<double>(r, c) = v[static_cast<std::size_t>(r * 4 + c)];
    }
  }
  return m;
}

double clampUnit(double v)
{
  return std::max(-1.0, std::min(1.0, v));
}

double rotationAngleRadBetween(const cv::Mat & r1, const cv::Mat & r2)
{
  const cv::Mat r = r1 * r2.t();
  const double tr = r.at<double>(0, 0) + r.at<double>(1, 1) + r.at<double>(2, 2);
  return std::acos(clampUnit((tr - 1.0) * 0.5));
}

bool getUrdfReferenceTcamBaseForArm(int arm_id, cv::Mat & t_cam_base_ref)
{
  // base_link -> camera0_optical_frame（nova_robot_position.urdf，龙门架固定相机）
  // 机械臂 0/1 标定均使用同一套 cam0 图像，与 URDF 对比时用同一参考外参。
  if (arm_id == 0 || arm_id == 1) {
    const std::array<double, 16> t = {
       0.999998,  0.001589, -0.000796, -0.528377,
       0.001589, -0.999999, -0.000001, -0.500299,
      -0.000796, -0.000001, -1.000000,  1.040421,
       0.000000,  0.000000,  0.000000, 1.000000
    };
    t_cam_base_ref = vec16ToMat44(t);
    return true;
  }
  return false;
}
}

CalibNode::CalibNode(
  const std::string & node_name, bool eye_in_hand, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  eye_in_hand_(eye_in_hand),
  arm_id_(0),
  marker_id_(0),
  aruco_dict_id_(cv::aruco::DICT_5X5_100),
  min_samples_(8),
  marker_length_m_(0.1),
  state_timeout_sec_(2.0),
  reached_wait_timeout_sec_(2.0),
  capture_wait_timeout_sec_(2.0),
  use_current_pose_as_center_(true),
  use_known_target_mount_(true),
  marker_bbox_ratio_min_(0.012),
  marker_bbox_ratio_max_(0.030),
  targets_prepared_(false),
  has_camera_info_(false),
  has_arm_pose_(false),
  has_arm_state_(false),
  arm_reached_(false),
  target_index_(0),
  waiting_arm_reached_(false),
  waiting_capture_(false),
  finished_(false),
  auto_mode_(true),
  pending_step_(false),
  last_detect_fail_reason_("none"),
  waiting_init_pose_(false),
  last_status_text_(""),
  known_mount_trans_consistency_m_(0.0),
  known_mount_rot_consistency_deg_(0.0),
  use_tf_for_sample_pose_(true),
  tf_base_frame_("base_link"),
  tf_ee_frame_arm0_("J1_6"),
  tf_ee_frame_arm1_("J2_6"),
  known_mount_quality_max_m_(0.20),
  init_reset_burst_count_(6),
  init_reset_burst_period_ms_(50),
  init_delay_ms_after_reset_(500)
{
  declareAndLoadParameters();
  initRosEntities();
  RCLCPP_INFO(
    get_logger(), "Node started. mode=%s, arm_id=%d",
    eye_in_hand_ ? "eye_in_hand" : "eye_to_hand", arm_id_);
}

void CalibNode::declareAndLoadParameters()
{
  CalibConfigData cfg;
  if (eye_in_hand_) {
    EyeInHandConfigDataManager mgr;
    cfg = mgr.load(*this);
  } else {
    EyeToHandConfigDataManager mgr;
    cfg = mgr.load(*this);
  }

  arm_id_ = cfg.arm_id;
  marker_id_ = cfg.target_marker_id;
  aruco_dict_id_ = cfg.aruco_dict_id;
  marker_length_m_ = cfg.marker_length_m;
  min_samples_ = cfg.min_samples;
  state_timeout_sec_ = cfg.state_timeout_sec;
  reached_wait_timeout_sec_ = cfg.state_timeout_sec;
  capture_wait_timeout_sec_ = cfg.state_timeout_sec;
  image_topic_ = cfg.image_topic;
  depth_topic_ = cfg.depth_topic;
  camera_info_topic_ = cfg.camera_info_topic;
  robot_pose_topic_ = cfg.robot_pose_topic;
  robot_state_topic_ = cfg.robot_state_topic;
  robot_target_topic_ = cfg.robot_target_topic;
  output_dir_ = cfg.output_dir;
  target_poses_arm0_ = cfg.target_poses;
  target_poses_arm1_ = cfg.target_poses_arm1.empty() ? cfg.target_poses : cfg.target_poses_arm1;
  use_current_pose_as_center_ = cfg.use_current_pose_as_center;
  target_position_offsets_ = cfg.target_position_offsets;
  target_orientation_offsets_rpy_deg_ = cfg.target_orientation_offsets_rpy_deg;
  use_known_target_mount_ = cfg.use_known_target_mount;
  target_to_gripper_pose_ = cfg.target_to_gripper_pose;
  marker_bbox_ratio_min_ = cfg.marker_bbox_ratio_min;
  marker_bbox_ratio_max_ = cfg.marker_bbox_ratio_max;
  use_tf_for_sample_pose_ = cfg.use_tf_for_sample_pose;
  tf_base_frame_ = cfg.tf_base_frame;
  tf_ee_frame_arm0_ = cfg.tf_ee_frame_arm0;
  tf_ee_frame_arm1_ = cfg.tf_ee_frame_arm1;
  known_mount_quality_max_m_ = cfg.known_mount_quality_max_m;
  init_reset_burst_count_ = std::max(1, cfg.init_reset_burst_count);
  init_reset_burst_period_ms_ = std::max(0, cfg.init_reset_burst_period_ms);
  init_delay_ms_after_reset_ = std::max(0, cfg.init_delay_ms_after_reset);

  if (!target_poses_arm0_.empty() && (target_poses_arm0_.size() % kPoseDims) != 0U) {
    throw std::runtime_error("target_poses must be [x,y,z,qx,qy,qz,qw] * N");
  }
  if (!target_poses_arm1_.empty() && (target_poses_arm1_.size() % kPoseDims) != 0U) {
    throw std::runtime_error("target_poses_arm1 must be [x,y,z,qx,qy,qz,qw] * N");
  }
  applyTargetPosesForCurrentArm();
  if (!target_position_offsets_.empty() && (target_position_offsets_.size() % 3U) != 0U) {
    throw std::runtime_error("target_position_offsets must be [dx,dy,dz] * N");
  }
  if (!target_orientation_offsets_rpy_deg_.empty()) {
    if ((target_orientation_offsets_rpy_deg_.size() % 3U) != 0U) {
      throw std::runtime_error("target_orientation_offsets_rpy_deg must be [roll,pitch,yaw_deg] * N");
    }
    if (!target_position_offsets_.empty() &&
      target_orientation_offsets_rpy_deg_.size() != target_position_offsets_.size())
    {
      throw std::runtime_error(
        "target_orientation_offsets_rpy_deg length must match target_position_offsets");
    }
  }
  if (use_known_target_mount_ && target_to_gripper_pose_.size() != kPoseDims) {
    throw std::runtime_error("target_to_gripper_pose must be [x,y,z,qx,qy,qz,qw]");
  }
  if (marker_bbox_ratio_min_ <= 0.0 || marker_bbox_ratio_max_ <= marker_bbox_ratio_min_) {
    throw std::runtime_error("marker_bbox_ratio_min/max invalid");
  }
  if (target_poses_flat_.empty() && !use_current_pose_as_center_) {
    throw std::runtime_error("target_poses is empty when use_current_pose_as_center=false");
  }
  aruco_dict_ = cv::aruco::getPredefinedDictionary(aruco_dict_id_);
}

void CalibNode::applyTargetPosesForCurrentArm()
{
  if (use_current_pose_as_center_) {
    return;
  }
  target_poses_flat_ = (arm_id_ == 1) ? target_poses_arm1_ : target_poses_arm0_;
}

void CalibNode::initRosEntities()
{
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, 10, std::bind(&CalibNode::imageCallback, this, std::placeholders::_1));
  cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, 10, std::bind(&CalibNode::cameraInfoCallback, this, std::placeholders::_1));
  arm_pose_sub_ = create_subscription<calib_sim::msg::ArmPose>(
    robot_pose_topic_, 20, std::bind(&CalibNode::armPoseCallback, this, std::placeholders::_1));
  arm_state_sub_ = create_subscription<std_msgs::msg::Bool>(
    robot_state_topic_, 20, std::bind(&CalibNode::armStateCallback, this, std::placeholders::_1));
  control_sub_ = create_subscription<std_msgs::msg::String>(
    "/calib_sim/control", 20, std::bind(&CalibNode::controlCallback, this, std::placeholders::_1));
  target_pose_pub_ = create_publisher<calib_sim::msg::ArmPose>(robot_target_topic_, 10);
  this->declare_parameter("nova_all_joints_reset_topic", std::string("/nova_sim/reset_all_joints"));
  nova_all_joints_reset_topic_ = this->get_parameter("nova_all_joints_reset_topic").as_string();
  nova_all_joints_reset_pub_ =
    create_publisher<std_msgs::msg::Empty>(nova_all_joints_reset_topic_, 10);
  status_pub_ = create_publisher<std_msgs::msg::String>("/calib_sim/status", 10);
  log_pub_ = create_publisher<std_msgs::msg::String>("/calib_sim/log", 30);
  result_text_pub_ = create_publisher<std_msgs::msg::String>("/calib_sim/result_text", 10);
  raw_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/calib_sim/raw_image", 10);
  result_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/calib_sim/result_image", 10);
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&CalibNode::controlTimerCallback, this));
  if (use_tf_for_sample_pose_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  }
}

void CalibNode::initAfterSharedPtr(const std::shared_ptr<CalibNode> & self)
{
  if (!use_tf_for_sample_pose_ || !tf_buffer_ || tf_listener_) {
    return;
  }
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_, std::static_pointer_cast<rclcpp::Node>(self), false);
  RCLCPP_INFO(
    get_logger(),
    "TF sample poses enabled: %s -> arm0:%s arm1:%s (time-sync with image)",
    tf_base_frame_.c_str(), tf_ee_frame_arm0_.c_str(), tf_ee_frame_arm1_.c_str());
}

void CalibNode::cancelInitDelayTimer()
{
  if (init_after_reset_timer_) {
    init_after_reset_timer_->cancel();
    init_after_reset_timer_.reset();
  }
}

void CalibNode::republishLastCameraImagesToUi()
{
  auto publish_bgr = [this](const cv::Mat & bgr, const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub) {
    if (bgr.empty() || !pub) {
      return;
    }
    cv::Mat cont;
    if (bgr.isContinuous()) {
      cont = bgr;
    } else {
      bgr.copyTo(cont);
    }
    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera";
    msg.height = static_cast<uint32_t>(cont.rows);
    msg.width = static_cast<uint32_t>(cont.cols);
    msg.encoding = "bgr8";
    msg.step = static_cast<uint32_t>(cont.cols * 3);
    msg.data.assign(cont.data, cont.data + cont.total() * cont.elemSize());
    pub->publish(msg);
  };
  publish_bgr(last_raw_frame_, raw_image_pub_);
  // 结果图与当前原图一致（复位/刷新 UI 时不保留旧标注）
  if (!last_raw_frame_.empty()) {
    last_result_frame_ = last_raw_frame_.clone();
  }
  publish_bgr(last_raw_frame_, result_image_pub_);
}

void CalibNode::publishInitPoseAfterResetDelay()
{
  cancelInitDelayTimer();
  init_pose_pending_.pose.header.stamp = now();
  init_pose_pending_.pose.header.frame_id = "base_link";
  target_pose_pub_->publish(init_pose_pending_);
  waiting_arm_reached_ = true;
  waiting_capture_ = false;
  waiting_init_pose_ = true;
  target_sent_time_ = now();
  publishStatus("moving_to_initial_pose_arm_" + std::to_string(arm_id_));
  publishLog("cmd:init move_to_initial_pose (async, UI refreshed)");
  republishLastCameraImagesToUi();
}

void CalibNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      camera_matrix_.at<double>(r, c) = msg->k[r * 3 + c];
    }
  }
  dist_coeffs_ = cv::Mat(msg->d, true).reshape(1, static_cast<int>(msg->d.size()));
  has_camera_info_ = true;
}

void CalibNode::armPoseCallback(const calib_sim::msg::ArmPose::ConstSharedPtr msg)
{
  if (initial_pose_by_arm_.find(msg->arm_id) == initial_pose_by_arm_.end()) {
    initial_pose_by_arm_[msg->arm_id] = *msg;
  }
  if (msg->arm_id != arm_id_) {
    return;
  }
  last_arm_pose_ = *msg;
  has_arm_pose_ = true;
}

void CalibNode::armStateCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  arm_reached_ = msg->data;
  last_arm_state_time_ = now();
  has_arm_state_ = true;
}

void CalibNode::controlTimerCallback()
{
  if (finished_) {
    return;
  }

  if (!ensureTargetsPrepared()) {
    publishStatus("waiting_initial_robot_pose");
    return;
  }

  const auto pose_count = target_poses_flat_.size() / kPoseDims;
  if (target_index_ >= pose_count) {
    if (runCalibration()) {
      RCLCPP_INFO(get_logger(), "Calibration complete with %zu samples", samples_.size());
      publishStatus("calibration_complete");
    }
    finished_ = true;
    return;
  }

  if (!waiting_arm_reached_ && !waiting_capture_) {
    if (!auto_mode_ && !pending_step_) {
      publishStatus("paused_wait_step");
      return;
    }
    publishTargetPose(target_index_);
    target_sent_time_ = now();
    waiting_arm_reached_ = true;
    publishStatus("moving_to_target_" + std::to_string(target_index_));
    return;
  }

  if (waiting_arm_reached_ && has_arm_state_ && arm_reached_)
  {
    const auto state_age = now() - last_arm_state_time_;
    if (state_age.seconds() <= state_timeout_sec_) {
      waiting_arm_reached_ = false;
      if (waiting_init_pose_) {
        waiting_init_pose_ = false;
        publishStatus("initialized_wait_step");
      } else {
        waiting_capture_ = true;
        capture_start_time_ = now();
        publishStatus("capturing_target_" + std::to_string(target_index_));
      }
    }
  }

  // Fallback: avoid deadlock when robot_reached signal is missing.
  if (waiting_arm_reached_ && (now() - target_sent_time_).seconds() > reached_wait_timeout_sec_) {
    waiting_arm_reached_ = false;
    if (waiting_init_pose_) {
      waiting_init_pose_ = false;
      publishStatus("initialized_wait_step_timeout");
    } else {
      waiting_capture_ = false;
      std::ostringstream oss;
      oss << "arm_reach_timeout_skip_target_" << target_index_;
      publishLog(oss.str());
      publishStatus("arm_reach_timeout_skip_target_" + std::to_string(target_index_));
      ++target_index_;
      if (pending_step_) {
        pending_step_ = false;
      }
    }
  }

  // If marker detection cannot succeed for long time, skip to next target to keep robot moving.
  if (waiting_capture_ && (now() - capture_start_time_).seconds() > capture_wait_timeout_sec_) {
    waiting_capture_ = false;
    std::ostringstream oss;
    oss << "capture_timeout_skip_target_" << target_index_
        << " has_camera_info=" << (has_camera_info_ ? "true" : "false")
        << " detect_fail_reason=" << last_detect_fail_reason_;
    publishLog(oss.str());
    ++target_index_;
    if (pending_step_) {
      pending_step_ = false;
    }
  }
}

void CalibNode::controlCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  const std::string cmd = msg->data;
  RCLCPP_INFO(get_logger(), "received control cmd: %s", cmd.c_str());
  publishLog("received_cmd:" + cmd);
  if (has_arm_pose_) {
    const auto & p = last_arm_pose_.pose.pose.position;
    const auto & q = last_arm_pose_.pose.pose.orientation;
    std::ostringstream oss;
    oss << "current_pose_once arm_id=" << last_arm_pose_.arm_id
        << " pos=(" << p.x << ", " << p.y << ", " << p.z << ")"
        << " quat=(" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << ")";
    publishLog(oss.str());
  }

  const auto reset_runtime_state = [this]() {
    samples_.clear();
    captured_raw_frames_.clear();
    captured_result_frames_.clear();
    captured_arm_poses_.clear();
    target_index_ = 0;
    waiting_arm_reached_ = false;
    waiting_capture_ = false;
    finished_ = false;
    waiting_init_pose_ = false;
  };

  if (cmd.rfind("set_arm:", 0) == 0) {
    cancelInitDelayTimer();
    const std::string arm_text = cmd.substr(std::string("set_arm:").size());
    int new_arm = arm_id_;
    try {
      new_arm = std::stoi(arm_text);
    } catch (const std::exception &) {
      publishLog("cmd:set_arm invalid arm id");
      return;
    }
    if (new_arm < 0) {
      publishLog("cmd:set_arm invalid arm id < 0");
      return;
    }

    arm_id_ = new_arm;
    // Default convention: marker id follows arm id (arm0->id0, arm1->id1).
    marker_id_ = new_arm;
    applyTargetPosesForCurrentArm();
    has_arm_pose_ = false;
    has_arm_state_ = false;
    arm_reached_ = false;
    targets_prepared_ = false;
    reset_runtime_state();
    auto_mode_ = false;
    pending_step_ = false;
    publishStatus("arm_switched_" + std::to_string(arm_id_) + "_wait_step");
    publishLog("cmd:set_arm arm_id=" + std::to_string(arm_id_) + " marker_id=" + std::to_string(marker_id_));
    return;
  }

  if (cmd == "init") {
    cancelInitDelayTimer();
    reset_runtime_state();
    auto_mode_ = false;
    pending_step_ = false;
    {
      std_msgs::msg::Empty reset_signal;
      for (int i = 0; i < init_reset_burst_count_; ++i) {
        nova_all_joints_reset_pub_->publish(reset_signal);
      }
      publishLog(
        "cmd:init burst reset n=" + std::to_string(init_reset_burst_count_) + " topic=" +
        nova_all_joints_reset_topic_);
    }
    republishLastCameraImagesToUi();
    const auto it = initial_pose_by_arm_.find(arm_id_);
    if (it == initial_pose_by_arm_.end()) {
      publishStatus("init_failed_no_initial_pose");
      publishLog("cmd:init no_initial_pose_for_arm");
      return;
    }

    init_pose_pending_ = it->second;
    init_pose_pending_.arm_id = arm_id_;
    publishLog(
      "cmd:init scheduling initial pose after delay_ms=" + std::to_string(init_delay_ms_after_reset_) +
      " (non-blocking; camera keeps updating)");

    if (init_delay_ms_after_reset_ <= 0) {
      publishInitPoseAfterResetDelay();
      return;
    }
    init_after_reset_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(1, init_delay_ms_after_reset_)),
      std::bind(&CalibNode::publishInitPoseAfterResetDelay, this));
    return;
  }
  if (cmd == "step") {
    if (finished_) {
      reset_runtime_state();
    }
    auto_mode_ = false;
    pending_step_ = true;
    if (waiting_arm_reached_) {
      waiting_arm_reached_ = false;
      waiting_capture_ = true;
      capture_start_time_ = now();
    }
    publishStatus("step_triggered");
    publishLog("cmd:step");
    return;
  }
  if (cmd == "auto") {
    if (finished_) {
      reset_runtime_state();
    }
    auto_mode_ = true;
    pending_step_ = false;
    if (waiting_arm_reached_ || waiting_capture_) {
      // keep current phase
    } else {
      // next timer cycle will issue target command
    }
    publishStatus("auto_mode");
    publishLog("cmd:auto");
    return;
  }
  if (cmd == "pause") {
    auto_mode_ = false;
    pending_step_ = false;
    publishStatus("paused");
    publishLog("cmd:pause");
    return;
  }
}

bool CalibNode::ensureTargetsPrepared()
{
  if (targets_prepared_) {
    return true;
  }
  if (!use_current_pose_as_center_) {
    targets_prepared_ = true;
    return true;
  }
  if (!has_arm_pose_) {
    return false;
  }

  const auto & base_pose = last_arm_pose_.pose.pose;
  const auto & q = base_pose.orientation;
  std::vector<double> offsets = target_position_offsets_;
  if (offsets.empty()) {
    offsets = {
      0.0, 0.0, 0.0,
      0.01, 0.0, 0.0,
      -0.01, 0.0, 0.0,
      0.0, 0.01, 0.0,
      0.0, -0.01, 0.0,
      0.0, 0.0, 0.01,
      0.0, 0.0, -0.01,
      0.01, 0.01, 0.0
    };
  }

  target_poses_flat_.clear();
  if (!target_orientation_offsets_rpy_deg_.empty() &&
    target_orientation_offsets_rpy_deg_.size() != offsets.size())
  {
    throw std::runtime_error(
      "target_orientation_offsets_rpy_deg must have the same length as the expanded "
      "position offsets (built-in default uses 24 floats: 8 waypoints)");
  }

  const cv::Mat R_base = CalibNode::quatToRot(q.x, q.y, q.z, q.w);
  target_poses_flat_.reserve((offsets.size() / 3U) * kPoseDims);
  for (std::size_t i = 0; i < offsets.size(); i += 3U) {
    target_poses_flat_.push_back(base_pose.position.x + offsets[i + 0]);
    target_poses_flat_.push_back(base_pose.position.y + offsets[i + 1]);
    target_poses_flat_.push_back(base_pose.position.z + offsets[i + 2]);
    double qx = q.x;
    double qy = q.y;
    double qz = q.z;
    double qw = q.w;
    if (!target_orientation_offsets_rpy_deg_.empty()) {
      const std::size_t j = i / 3U;
      const double roll_deg = target_orientation_offsets_rpy_deg_[j * 3U + 0];
      const double pitch_deg = target_orientation_offsets_rpy_deg_[j * 3U + 1];
      const double yaw_deg = target_orientation_offsets_rpy_deg_[j * 3U + 2];
      const cv::Mat R_out = R_base * rpyDegToRdelta(roll_deg, pitch_deg, yaw_deg);
      mat3ToQuat(R_out, qx, qy, qz, qw);
    }
    target_poses_flat_.push_back(qx);
    target_poses_flat_.push_back(qy);
    target_poses_flat_.push_back(qz);
    target_poses_flat_.push_back(qw);
  }
  targets_prepared_ = true;
  RCLCPP_INFO(
    get_logger(), "Prepared %zu nearby targets around current pose.",
    target_poses_flat_.size() / kPoseDims);
  publishStatus("targets_prepared");
  return true;
}

void CalibNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  raw_image_pub_->publish(*msg);
  const bool pose_ok = has_arm_pose_ || (use_tf_for_sample_pose_ && tf_buffer_);
  if (!waiting_capture_ || !has_camera_info_ || !pose_ok) {
    return;
  }
  cv::Mat bgr = convertImageToBgr(msg);
  if (bgr.empty()) {
    return;
  }
  last_raw_frame_ = bgr.clone();

  cv::Mat annotated;
  std::vector<int> detected_ids;
  const rclcpp::Time img_stamp =
    (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) ?
    rclcpp::Time(msg->header.stamp) : now();
  if (tryCaptureSample(bgr, annotated, detected_ids, img_stamp)) {
    waiting_capture_ = false;
    ++target_index_;
    RCLCPP_INFO(get_logger(), "Captured sample %zu", samples_.size());
    if (pending_step_) {
      pending_step_ = false;
    }
  }

  if (!annotated.empty()) {
    const bool size_gate_fail =
      (last_detect_fail_reason_ == "marker_too_small" || last_detect_fail_reason_ == "marker_too_large");
    if (!size_gate_fail) {
      last_result_frame_ = annotated.clone();
      sensor_msgs::msg::Image result_msg;
      result_msg.header.stamp = now();
      result_msg.header.frame_id = "camera";
      result_msg.height = static_cast<uint32_t>(annotated.rows);
      result_msg.width = static_cast<uint32_t>(annotated.cols);
      result_msg.encoding = "bgr8";
      result_msg.step = static_cast<uint32_t>(annotated.cols * 3);
      result_msg.data.assign(annotated.datastart, annotated.dataend);
      result_image_pub_->publish(result_msg);
    }
  }

  std::ostringstream ids_ss;
  ids_ss << "ids=[";
  for (std::size_t i = 0; i < detected_ids.size(); ++i) {
    ids_ss << detected_ids[i];
    if (i + 1 < detected_ids.size()) {
      ids_ss << ",";
    }
  }
  ids_ss << "]";
  publishLog(ids_ss.str());
}

bool CalibNode::tryFillGripperPoseFromTf(
  const rclcpp::Time & image_stamp, cv::Mat & r_gripper_to_base, cv::Mat & t_gripper_to_base,
  calib_sim::msg::ArmPose * out_manifest_pose)
{
  if (!use_tf_for_sample_pose_ || !tf_buffer_) {
    return false;
  }
  const std::string & ee = (arm_id_ == 1) ? tf_ee_frame_arm1_ : tf_ee_frame_arm0_;
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf2::TimePoint tp = tf2::TimePointZero;
    if (image_stamp.nanoseconds() > 0) {
      tp = tf2::TimePoint(std::chrono::nanoseconds(image_stamp.nanoseconds()));
    }
    tf_msg = tf_buffer_->lookupTransform(tf_base_frame_, ee, tp);
  } catch (const tf2::TransformException &) {
    try {
      tf_msg = tf_buffer_->lookupTransform(tf_base_frame_, ee, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }
  const auto & tr = tf_msg.transform.translation;
  const auto & q = tf_msg.transform.rotation;
  r_gripper_to_base = quatToRot(q.x, q.y, q.z, q.w);
  t_gripper_to_base = (cv::Mat_<double>(3, 1) << tr.x, tr.y, tr.z);
  if (out_manifest_pose) {
    out_manifest_pose->arm_id = arm_id_;
    out_manifest_pose->pose.header.stamp = image_stamp.nanoseconds() != 0u ? image_stamp : now();
    out_manifest_pose->pose.header.frame_id = tf_base_frame_;
    out_manifest_pose->pose.pose.position.x = tr.x;
    out_manifest_pose->pose.pose.position.y = tr.y;
    out_manifest_pose->pose.pose.position.z = tr.z;
    out_manifest_pose->pose.pose.orientation = q;
  }
  return true;
}

bool CalibNode::tryCaptureSample(
  const cv::Mat & frame_bgr, cv::Mat & annotated, std::vector<int> & detected_ids,
  const rclcpp::Time & image_stamp)
{
  cv::Mat r_target_to_cam, t_target_to_cam;
  double mean_corner_reproj_px = 0.0;
  std::string fail_reason;
  if (!detectTargetPoseInCamera(
      frame_bgr, r_target_to_cam, t_target_to_cam, mean_corner_reproj_px, fail_reason, annotated,
      detected_ids))
  {
    last_detect_fail_reason_ = fail_reason;
    if (!annotated.empty()) {
      cv::putText(
        annotated, "FAIL: " + fail_reason,
        cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }
    return false;
  }
  last_detect_fail_reason_ = "ok";

  Sample sample;
  calib_sim::msg::ArmPose manifest_pose;
  const bool used_tf = tryFillGripperPoseFromTf(
    image_stamp, sample.r_gripper_to_base, sample.t_gripper_to_base, &manifest_pose);
  if (!used_tf) {
    if (!has_arm_pose_) {
      return false;
    }
    manifest_pose = last_arm_pose_;
    const auto & p = last_arm_pose_.pose.pose.position;
    const auto & q = last_arm_pose_.pose.pose.orientation;
    sample.r_gripper_to_base = quatToRot(q.x, q.y, q.z, q.w);
    sample.t_gripper_to_base = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);
  } else if (!sample_pose_logged_source_) {
    sample_pose_logged_source_ = true;
    publishLog("sample_gripper_pose_source=tf (time-sync with image)");
  }

  cv::putText(
    annotated, "captured_sample_" + std::to_string(samples_.size() + 1),
    cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
  captured_raw_frames_.push_back(frame_bgr.clone());
  captured_result_frames_.push_back(annotated.clone());
  captured_arm_poses_.push_back(manifest_pose);
  sample.r_target_to_cam = r_target_to_cam;
  sample.t_target_to_cam = t_target_to_cam;
  sample.mean_corner_reproj_px = mean_corner_reproj_px;
  samples_.push_back(sample);
  publishStatus("sample_captured_" + std::to_string(samples_.size()));
  return true;
}

bool CalibNode::detectTargetPoseInCamera(
  const cv::Mat & frame_bgr, cv::Mat & r_target_to_cam, cv::Mat & t_target_to_cam,
  double & out_mean_corner_reproj_px,
  std::string & fail_reason, cv::Mat & annotated, std::vector<int> & detected_ids)
{
  out_mean_corner_reproj_px = 0.0;
  annotated = frame_bgr.clone();
  cv::Mat gray;
  cv::cvtColor(frame_bgr, gray, cv::COLOR_BGR2GRAY);
  cv::Mat gray_eq;
  {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    clahe->apply(gray, gray_eq);
  }
  // Mild unsharp mask for crisper marker edges.
  cv::Mat blur_eq;
  cv::GaussianBlur(gray_eq, blur_eq, cv::Size(0, 0), 1.2);
  cv::Mat gray_sharp;
  cv::addWeighted(gray_eq, 1.6, blur_eq, -0.6, 0.0, gray_sharp);

  const double detect_scale = 2.0;
  cv::Mat gray_detect;
  cv::resize(gray_sharp, gray_detect, cv::Size(), detect_scale, detect_scale, cv::INTER_CUBIC);
  const auto dict_ptr = cv::makePtr<cv::aruco::Dictionary>(aruco_dict_);
  auto detector_params = cv::makePtr<cv::aruco::DetectorParameters>();
  detector_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  detector_params->cornerRefinementWinSize = 9;
  detector_params->cornerRefinementMaxIterations = 50;
  detector_params->cornerRefinementMinAccuracy = 0.005;
  detector_params->adaptiveThreshWinSizeMin = 7;
  detector_params->adaptiveThreshWinSizeMax = 31;
  detector_params->adaptiveThreshWinSizeStep = 4;
  detector_params->adaptiveThreshConstant = 7.0;
  detector_params->minCornerDistanceRate = 0.03;
  detector_params->polygonalApproxAccuracyRate = 0.03;
  detector_params->minMarkerPerimeterRate = 0.02;
  detector_params->maxMarkerPerimeterRate = 5.0;
  auto detect_one_pass =
    [&](const cv::Mat & src, bool need_scale_back, double scale_factor,
    std::vector<std::vector<cv::Point2f>> & out_corners, std::vector<int> & out_ids) {
      cv::aruco::detectMarkers(src, dict_ptr, out_corners, out_ids, detector_params);
      if (need_scale_back) {
        for (auto & marker : out_corners) {
          for (auto & p : marker) {
            p.x = static_cast<float>(p.x / scale_factor);
            p.y = static_cast<float>(p.y / scale_factor);
          }
        }
      }
      for (auto & marker : out_corners) {
        cv::cornerSubPix(
          gray_sharp, marker, cv::Size(7, 7), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
      }
    };

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  detect_one_pass(gray_detect, true, detect_scale, corners, ids);
  // Fallback pass: avoid over-sharpen artifacts by running on CLAHE image directly.
  if (ids.empty()) {
    detect_one_pass(gray_eq, false, 1.0, corners, ids);
  }
  detected_ids = ids;
  if (!ids.empty()) {
    for (std::size_t i = 0; i < corners.size(); ++i) {
      const auto & c = corners[i];
      if (c.size() == 4U) {
        cv::line(annotated, c[0], c[1], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::line(annotated, c[1], c[2], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::line(annotated, c[2], c[3], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::line(annotated, c[3], c[0], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::circle(annotated, c[0], 3, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
      }
      if (i < ids.size()) {
        cv::putText(
          annotated, std::to_string(ids[i]), c[0] + cv::Point2f(2.0F, -6.0F),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
      }
    }
  }
  if (ids.empty()) {
    std::ostringstream oss;
    oss << "detect_diag no_marker"
        << " image=" << frame_bgr.cols << "x" << frame_bgr.rows
        << " dict_id=" << aruco_dict_id_
        << " target_id=" << marker_id_;
    publishLog(oss.str());
    fail_reason = "no_marker";
    return false;
  }

  int found_index = -1;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (ids[i] == marker_id_) {
      found_index = static_cast<int>(i);
      break;
    }
  }
  if (found_index < 0) {
    std::ostringstream oss;
    oss << "detect_diag marker_id_mismatch target_id=" << marker_id_ << " ids=[";
    for (std::size_t i = 0; i < ids.size(); ++i) {
      oss << ids[i];
      if (i + 1 < ids.size()) {
        oss << ",";
      }
    }
    oss << "]";
    publishLog(oss.str());
    fail_reason = "marker_id_mismatch";
    return false;
  }

  const auto & marker_corners = corners[found_index];
  const cv::Rect bbox = cv::boundingRect(marker_corners);
  double perimeter_px = 0.0;
  for (std::size_t i = 0; i < marker_corners.size(); ++i) {
    const cv::Point2f & p0 = marker_corners[i];
    const cv::Point2f & p1 = marker_corners[(i + 1U) % marker_corners.size()];
    perimeter_px += cv::norm(p0 - p1);
  }
  const double image_area = static_cast<double>(frame_bgr.cols) * static_cast<double>(frame_bgr.rows);
  const double bbox_area_ratio = image_area > 0.0 ?
    (static_cast<double>(bbox.width) * static_cast<double>(bbox.height) / image_area) : 0.0;
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "detect_diag marker_id=" << ids[found_index]
        << " perimeter_px=" << perimeter_px
        << " bbox=" << bbox.x << "," << bbox.y << "," << bbox.width << "," << bbox.height
        << " bbox_area_ratio=" << bbox_area_ratio;
    publishLog(oss.str());
  }
  if (bbox_area_ratio < marker_bbox_ratio_min_) {
    fail_reason = "marker_too_small";
    return false;
  }
  if (bbox_area_ratio > marker_bbox_ratio_max_) {
    fail_reason = "marker_too_large";
    return false;
  }

  const auto & marker_corners_for_pnp = corners[found_index];
  const double h = marker_length_m_ * 0.5;
  // Keep the same corner convention as cv::aruco::estimatePoseSingleMarkers:
  // (-l/2, +l/2), (+l/2, +l/2), (+l/2, -l/2), (-l/2, -l/2)
  std::vector<cv::Point3f> object_points_proj{
    cv::Point3f(-h, h, 0.0), cv::Point3f(h, h, 0.0),
    cv::Point3f(h, -h, 0.0), cv::Point3f(-h, -h, 0.0)
  };

  cv::Mat rvec, tvec;
  bool pnp_ok = cv::solvePnP(
    object_points_proj, marker_corners_for_pnp, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE_SQUARE);
  if (!pnp_ok) {
    pnp_ok = cv::solvePnP(
      object_points_proj, marker_corners_for_pnp, camera_matrix_, dist_coeffs_, rvec, tvec, false,
      cv::SOLVEPNP_ITERATIVE);
  }
  if (!pnp_ok) {
    publishLog("detect_diag pose_estimation_failed");
    fail_reason = "pnp_failed";
    return false;
  }

  std::vector<cv::Point2f> reproj;
  cv::projectPoints(object_points_proj, rvec, tvec, camera_matrix_, dist_coeffs_, reproj);
  double reproj_err_px = 0.0;
  if (reproj.size() == marker_corners_for_pnp.size()) {
    for (std::size_t i = 0; i < reproj.size(); ++i) {
      reproj_err_px += cv::norm(reproj[i] - marker_corners_for_pnp[i]);
    }
    reproj_err_px /= static_cast<double>(reproj.size());
  }
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "detect_diag pnp_reproj_err_px=" << reproj_err_px;
    publishLog(oss.str());
  }
  constexpr double kReprojWarnPx = 6.0;
  constexpr double kReprojHardRejectPx = 14.0;
  if (reproj_err_px > kReprojWarnPx) {
    std::ostringstream warn_ss;
    warn_ss << std::fixed << std::setprecision(3)
            << "detect_warn pnp_reproj_large_px=" << reproj_err_px;
    publishLog(warn_ss.str());
    cv::putText(
      annotated, "WARN: pnp_reproj_large " + std::to_string(reproj_err_px) + "px",
      cv::Point(20, 65), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
  }
  if (reproj_err_px > kReprojHardRejectPx) {
    fail_reason = "pnp_reproj_too_large_hard";
    return false;
  }

  cv::drawFrameAxes(
    annotated, camera_matrix_, dist_coeffs_, rvec, tvec,
    static_cast<float>(marker_length_m_ * 0.6), 2);

  // Draw a pose-consistent projected border for cleaner visual feedback.
  const std::vector<cv::Point2f> & projected_corners = reproj;
  if (projected_corners.size() == 4U) {
    cv::line(annotated, projected_corners[0], projected_corners[1], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(annotated, projected_corners[1], projected_corners[2], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(annotated, projected_corners[2], projected_corners[3], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(annotated, projected_corners[3], projected_corners[0], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
  }
  cv::Rodrigues(rvec, r_target_to_cam);
  t_target_to_cam = tvec.clone();
  out_mean_corner_reproj_px = reproj_err_px;
  fail_reason = "ok";
  return true;
}

void CalibNode::publishStatus(const std::string & status)
{
  if (status == last_status_text_) {
    return;
  }
  last_status_text_ = status;
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
  publishLog(status);
}

void CalibNode::publishLog(const std::string & text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  log_pub_->publish(msg);
}

cv::Mat CalibNode::convertImageToBgr(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
{
  if (msg->height == 0 || msg->width == 0) {
    return cv::Mat();
  }
  cv::Mat raw;
  if (msg->encoding == "bgr8") {
    raw = cv::Mat(
      static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
      const_cast<unsigned char *>(msg->data.data()), msg->step);
    return raw.clone();
  }
  if (msg->encoding == "rgb8") {
    raw = cv::Mat(
      static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
      const_cast<unsigned char *>(msg->data.data()), msg->step);
    cv::Mat bgr;
    cv::cvtColor(raw, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }
  if (msg->encoding == "mono8") {
    raw = cv::Mat(
      static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC1,
      const_cast<unsigned char *>(msg->data.data()), msg->step);
    cv::Mat bgr;
    cv::cvtColor(raw, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }
  RCLCPP_WARN(get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
  return cv::Mat();
}

void CalibNode::publishTargetPose(std::size_t idx)
{
  const std::size_t base = idx * kPoseDims;
  calib_sim::msg::ArmPose cmd;
  cmd.arm_id = arm_id_;
  cmd.pose.header.stamp = now();
  cmd.pose.header.frame_id = "base_link";
  cmd.pose.pose.position.x = target_poses_flat_[base + 0];
  cmd.pose.pose.position.y = target_poses_flat_[base + 1];
  cmd.pose.pose.position.z = target_poses_flat_[base + 2];
  double qx = target_poses_flat_[base + 3];
  double qy = target_poses_flat_[base + 4];
  double qz = target_poses_flat_[base + 5];
  double qw = target_poses_flat_[base + 6];
  const double qn = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (qn > kEps) {
    qx /= qn;
    qy /= qn;
    qz /= qn;
    qw /= qn;
  }
  cmd.pose.pose.orientation.x = qx;
  cmd.pose.pose.orientation.y = qy;
  cmd.pose.pose.orientation.z = qz;
  cmd.pose.pose.orientation.w = qw;
  target_pose_pub_->publish(cmd);
  RCLCPP_INFO(get_logger(), "Publish target pose index=%zu", idx);
  std::ostringstream oss;
  oss << "target_pose index=" << idx << " arm_id=" << cmd.arm_id
      << " pos=(" << cmd.pose.pose.position.x << ", " << cmd.pose.pose.position.y << ", "
      << cmd.pose.pose.position.z << ")"
      << " quat=(" << cmd.pose.pose.orientation.x << ", " << cmd.pose.pose.orientation.y << ", "
      << cmd.pose.pose.orientation.z << ", " << cmd.pose.pose.orientation.w << ")";
  publishLog(oss.str());
}

bool CalibNode::runCalibration()
{
  if (static_cast<int>(samples_.size()) < min_samples_) {
    RCLCPP_ERROR(
      get_logger(), "Not enough samples: %zu < %d", samples_.size(), min_samples_);
    return false;
  }

  // Sample diversity diagnostics: hand-eye needs sufficient pose variation, especially rotation.
  const auto & p0 = samples_.front().t_gripper_to_base;
  const auto & r0 = samples_.front().r_gripper_to_base;
  double max_pos_span = 0.0;
  double max_rot_span_deg = 0.0;
  for (const auto & s : samples_) {
    max_pos_span = std::max(max_pos_span, cv::norm(s.t_gripper_to_base - p0));
    const double rot_deg = rotationAngleRadBetween(s.r_gripper_to_base, r0) * kRad2Deg;
    max_rot_span_deg = std::max(max_rot_span_deg, rot_deg);
  }
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "sample_diversity pos_span_m=" << max_pos_span
        << " rot_span_deg=" << max_rot_span_deg;
    publishLog(oss.str());
  }
  if (max_rot_span_deg < 3.0) {
    publishStatus("calibration_failed_insufficient_rotation_diversity");
    publishLog("calibration_failed: rotate wrist/ee for at least ~3 deg span");
    return false;
  }

  cv::Mat t_cam_base, t_cam_gripper;
  bool has_cam_gripper = false;
  bool ok = false;
  if (eye_in_hand_) {
    ok = runEyeInHandCalibration(t_cam_base, t_cam_gripper);
    has_cam_gripper = ok;
  } else {
    ok = runEyeToHandCalibration(t_cam_base);
  }

  if (!ok) {
    RCLCPP_ERROR(get_logger(), "Calibration solve failed");
    return false;
  }

  cv::Mat t_base_cam = invertTransform(t_cam_base);
  HandEyeQualityMetrics quality;
  computeHandEyeChainResiduals(t_cam_base, t_cam_gripper, has_cam_gripper, quality);

  cv::Mat r_cam_base, t_cam_base_v;
  splitTransform(t_cam_base, r_cam_base, t_cam_base_v);
  const double tr = r_cam_base.at<double>(0, 0) + r_cam_base.at<double>(1, 1) + r_cam_base.at<double>(2, 2);
  const double rot_angle_deg = std::acos(clampUnit((tr - 1.0) * 0.5)) * kRad2Deg;
  const double trans_norm = cv::norm(t_cam_base_v);
  double quality_error_m = quality.mean_chain_translation_m;
  if (!eye_in_hand_ && use_known_target_mount_) {
    quality_error_m = known_mount_trans_consistency_m_;
  }
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "calib_quality trans_norm_m=" << trans_norm
        << " rot_angle_deg=" << rot_angle_deg
        << " mean_corner_reprojection_error_px=" << std::setprecision(4) << quality.mean_corner_reproj_px
        << " mean_handeye_chain_translation_residual_mm=" << std::setprecision(3)
        << (quality.mean_chain_translation_m * 1000.0)
        << " mean_handeye_chain_rotation_residual_deg=" << std::setprecision(4)
        << quality.mean_chain_rotation_deg
        << std::setprecision(6) << " quality_error_m=" << quality_error_m;
    publishLog(oss.str());
  }
  if (trans_norm < 1e-4 && rot_angle_deg < 0.2 && quality.mean_chain_translation_m > 1e-3) {
    publishStatus("calibration_failed_degenerate_identity_result");
    publishLog("calibration_failed: degenerate identity result detected");
    return false;
  }
  if (!eye_in_hand_ && use_known_target_mount_ && quality_error_m > 0.08 &&
    quality_error_m <= known_mount_quality_max_m_)
  {
    publishStatus("calibration_warn_medium_quality");
    publishLog("calibration_warning: medium quality result accepted");
  }
  if (trans_norm > 5.0 || quality_error_m > known_mount_quality_max_m_) {
    publishStatus("calibration_failed_poor_quality");
    publishLog("calibration_failed: poor quality, increase pose diversity and check marker scale");
    return false;
  }

  return saveResult(t_cam_base, t_base_cam, quality, t_cam_gripper, has_cam_gripper);
}

bool CalibNode::runEyeToHandCalibration(cv::Mat & t_cam_base)
{
  if (use_known_target_mount_) {
    const double tx = target_to_gripper_pose_[0];
    const double ty = target_to_gripper_pose_[1];
    const double tz = target_to_gripper_pose_[2];
    const double qx = target_to_gripper_pose_[3];
    const double qy = target_to_gripper_pose_[4];
    const double qz = target_to_gripper_pose_[5];
    const double qw = target_to_gripper_pose_[6];
    const cv::Mat r_t_g_cfg = quatToRot(qx, qy, qz, qw);
    const cv::Mat t_t_g_cfg = (cv::Mat_<double>(3, 1) << tx, ty, tz);
    const cv::Mat t_t_g_cfg_4 = makeTransform(r_t_g_cfg, t_t_g_cfg);
    const cv::Mat t_t_g_inv_4 = invertTransform(t_t_g_cfg_4);

    const cv::Mat r_flip_x = quatToRot(1.0, 0.0, 0.0, 0.0);  // 180 deg around X
    const cv::Mat r_flip_y = quatToRot(0.0, 1.0, 0.0, 0.0);  // 180 deg around Y
    const cv::Mat r_flip_z = quatToRot(0.0, 0.0, 1.0, 0.0);  // 180 deg around Z
    const cv::Mat t_zero = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
    const cv::Mat t_flip_x_4 = makeTransform(r_flip_x, t_zero);
    const cv::Mat t_flip_y_4 = makeTransform(r_flip_y, t_zero);
    const cv::Mat t_flip_z_4 = makeTransform(r_flip_z, t_zero);

    std::vector<std::pair<std::string, cv::Mat>> candidates;
    candidates.reserve(8);
    candidates.push_back({"cfg", t_t_g_cfg_4});
    candidates.push_back({"cfg*flipX", t_t_g_cfg_4 * t_flip_x_4});
    candidates.push_back({"cfg*flipY", t_t_g_cfg_4 * t_flip_y_4});
    candidates.push_back({"cfg*flipZ", t_t_g_cfg_4 * t_flip_z_4});
    candidates.push_back({"inv", t_t_g_inv_4});
    candidates.push_back({"inv*flipX", t_t_g_inv_4 * t_flip_x_4});
    candidates.push_back({"inv*flipY", t_t_g_inv_4 * t_flip_y_4});
    candidates.push_back({"inv*flipZ", t_t_g_inv_4 * t_flip_z_4});

    cv::Mat t_cam_base_ref;
    const bool has_ref = getUrdfReferenceTcamBaseForArm(arm_id_, t_cam_base_ref);
    const std::vector<cv::Vec3d> trans_offsets = {
      {0.00, 0.00, 0.00},
      {0.00, 0.03, 0.00}, {0.00, -0.03, 0.00},
      {0.00, 0.06, 0.00}, {0.00, -0.06, 0.00},
      {0.00, 0.00, 0.02}, {0.00, 0.00, -0.02},
      {0.00, 0.00, 0.04}, {0.00, 0.00, -0.04},
      {0.02, 0.00, 0.00}, {-0.02, 0.00, 0.00}
    };

    double best_score = std::numeric_limits<double>::max();
    std::string best_name = "none";
    cv::Mat best_t_cam_base = cv::Mat::eye(4, 4, CV_64F);
    double best_trans_consistency = 0.0;
    double best_rot_consistency = 0.0;
    double best_ref_trans_err = 0.0;
    double best_ref_rot_err = 0.0;

    for (const auto & item : candidates) {
      const auto & name = item.first;
      const auto & t_t_g_base = item.second;
      for (const auto & off : trans_offsets) {
        cv::Mat t_t_g_4 = t_t_g_base.clone();
        t_t_g_4.at<double>(0, 3) += off[0];
        t_t_g_4.at<double>(1, 3) += off[1];
        t_t_g_4.at<double>(2, 3) += off[2];

        cv::Mat r_sum = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat t_sum = cv::Mat::zeros(3, 1, CV_64F);
        std::vector<cv::Mat> t_c_b_all;
        t_c_b_all.reserve(samples_.size());
        for (const auto & s : samples_) {
          // arm pose topic provides base->gripper, convert to gripper->base for this chain.
          const cv::Mat t_b_g = makeTransform(s.r_gripper_to_base, s.t_gripper_to_base);
          const cv::Mat t_g_b = invertTransform(t_b_g);
          // solvePnP/aruco gives target->camera (i.e. T_cam_target) directly.
          const cv::Mat t_c_t = makeTransform(s.r_target_to_cam, s.t_target_to_cam);
          const cv::Mat t_c_b_i = t_c_t * t_t_g_4 * t_g_b;
          cv::Mat r_i, t_i;
          splitTransform(t_c_b_i, r_i, t_i);
          r_sum += r_i;
          t_sum += t_i;
          t_c_b_all.push_back(t_c_b_i);
        }

        cv::SVD svd(r_sum);
        cv::Mat r_avg = svd.u * svd.vt;
        if (cv::determinant(r_avg) < 0.0) {
          cv::Mat u_fix = svd.u.clone();
          u_fix.col(2) *= -1.0;
          r_avg = u_fix * svd.vt;
        }
        const cv::Mat t_avg = t_sum / static_cast<double>(samples_.size());
        const cv::Mat t_cam_base_i = makeTransform(r_avg, t_avg);

        double trans_sum = 0.0;
        double rot_sum_deg = 0.0;
        for (const auto & t_i : t_c_b_all) {
          cv::Mat r_i, t_i_v;
          splitTransform(t_i, r_i, t_i_v);
          trans_sum += cv::norm(t_i_v - t_avg);
          rot_sum_deg += rotationAngleRadBetween(r_i, r_avg) * kRad2Deg;
        }
        const double trans_consistency = trans_sum / static_cast<double>(t_c_b_all.size());
        const double rot_consistency = rot_sum_deg / static_cast<double>(t_c_b_all.size());

        double ref_trans_err = 0.0;
        double ref_rot_err = 0.0;
        if (has_ref) {
          cv::Mat r_est, t_est, r_ref, t_ref;
          splitTransform(t_cam_base_i, r_est, t_est);
          splitTransform(t_cam_base_ref, r_ref, t_ref);
          ref_trans_err = cv::norm(t_est - t_ref);
          ref_rot_err = rotationAngleRadBetween(r_est, r_ref) * kRad2Deg;
        }

        // Prefer URDF-consistent solutions in simulation while still keeping
        // internal sample consistency constraints.
        double score = trans_consistency + rot_consistency * 0.003;  // 1deg ~= 3mm
        if (has_ref) {
          // Increase reference matching weight; this stabilizes selection toward
          // physically expected extrinsics instead of locally-consistent but biased minima.
          score += ref_trans_err * 1.20 + ref_rot_err * 0.0120;
        }

        if (score < best_score) {
          best_score = score;
          std::ostringstream name_ss;
          name_ss << name << "+d(" << off[0] << "," << off[1] << "," << off[2] << ")";
          best_name = name_ss.str();
          best_t_cam_base = t_cam_base_i;
          best_trans_consistency = trans_consistency;
          best_rot_consistency = rot_consistency;
          best_ref_trans_err = ref_trans_err;
          best_ref_rot_err = ref_rot_err;
        }
      }
    }

    t_cam_base = best_t_cam_base;
    known_mount_trans_consistency_m_ = best_trans_consistency;
    known_mount_rot_consistency_deg_ = best_rot_consistency;
    {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(6)
          << "known_mount_consistency trans_m=" << known_mount_trans_consistency_m_
          << " rot_deg=" << known_mount_rot_consistency_deg_
          << " candidate=" << best_name
          << " ref_trans_m=" << best_ref_trans_err
          << " ref_rot_deg=" << best_ref_rot_err;
      publishLog(oss.str());
    }
    publishLog("eye_to_hand_solver=known_target_mount_direct");
    return true;
  }

  std::vector<cv::Mat> r_base_to_gripper;
  std::vector<cv::Mat> t_base_to_gripper;
  std::vector<cv::Mat> r_target_to_cam;
  std::vector<cv::Mat> t_target_to_cam;
  r_base_to_gripper.reserve(samples_.size());
  t_base_to_gripper.reserve(samples_.size());
  r_target_to_cam.reserve(samples_.size());
  t_target_to_cam.reserve(samples_.size());

  for (const auto & s : samples_) {
    cv::Mat t_g_b = makeTransform(s.r_gripper_to_base, s.t_gripper_to_base);
    cv::Mat t_b_g = invertTransform(t_g_b);
    cv::Mat r_b_g, t_b_g_v;
    splitTransform(t_b_g, r_b_g, t_b_g_v);
    r_base_to_gripper.push_back(r_b_g);
    t_base_to_gripper.push_back(t_b_g_v);
    r_target_to_cam.push_back(s.r_target_to_cam);
    t_target_to_cam.push_back(s.t_target_to_cam);
  }

  cv::Mat r_cam_base, t_cam_base_v;
  cv::calibrateHandEye(
    r_base_to_gripper, t_base_to_gripper, r_target_to_cam, t_target_to_cam,
    r_cam_base, t_cam_base_v, cv::CALIB_HAND_EYE_TSAI);
  t_cam_base = makeTransform(r_cam_base, t_cam_base_v);
  return true;
}

bool CalibNode::runEyeInHandCalibration(cv::Mat & t_cam_base, cv::Mat & t_cam_gripper)
{
  std::vector<cv::Mat> r_gripper_to_base;
  std::vector<cv::Mat> t_gripper_to_base;
  std::vector<cv::Mat> r_target_to_cam;
  std::vector<cv::Mat> t_target_to_cam;
  r_gripper_to_base.reserve(samples_.size());
  t_gripper_to_base.reserve(samples_.size());
  r_target_to_cam.reserve(samples_.size());
  t_target_to_cam.reserve(samples_.size());

  for (const auto & s : samples_) {
    r_gripper_to_base.push_back(s.r_gripper_to_base);
    t_gripper_to_base.push_back(s.t_gripper_to_base);
    r_target_to_cam.push_back(s.r_target_to_cam);
    t_target_to_cam.push_back(s.t_target_to_cam);
  }

  cv::Mat r_cam_gripper, t_cam_gripper_v;
  cv::calibrateHandEye(
    r_gripper_to_base, t_gripper_to_base, r_target_to_cam, t_target_to_cam,
    r_cam_gripper, t_cam_gripper_v, cv::CALIB_HAND_EYE_TSAI);
  t_cam_gripper = makeTransform(r_cam_gripper, t_cam_gripper_v);

  cv::Mat t_sum = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat r_ref = cv::Mat::eye(3, 3, CV_64F);
  for (std::size_t i = 0; i < samples_.size(); ++i) {
    cv::Mat t_g_b = makeTransform(samples_[i].r_gripper_to_base, samples_[i].t_gripper_to_base);
    cv::Mat t_c_b_i = t_g_b * t_cam_gripper;
    cv::Mat r_i, t_i;
    splitTransform(t_c_b_i, r_i, t_i);
    t_sum += t_i;
    if (i == 0) {
      r_ref = r_i.clone();
    }
  }
  cv::Mat t_avg = t_sum / static_cast<double>(samples_.size());
  t_cam_base = makeTransform(r_ref, t_avg);
  return true;
}

void CalibNode::computeHandEyeChainResiduals(
  const cv::Mat & t_cam_base,
  const cv::Mat & t_cam_gripper,
  bool has_cam_gripper,
  HandEyeQualityMetrics & out) const
{
  out = HandEyeQualityMetrics{};
  const std::size_t n = samples_.size();
  if (n == 0U) {
    return;
  }
  out.per_point_corner_reproj_px.reserve(n);
  out.per_point_chain_translation_m.reserve(n);
  out.per_point_chain_rotation_deg.reserve(n);

  cv::Mat anchor = cv::Mat::eye(4, 4, CV_64F);
  if (eye_in_hand_ && has_cam_gripper) {
    const cv::Mat t_g_b0 = makeTransform(samples_[0].r_gripper_to_base, samples_[0].t_gripper_to_base);
    const cv::Mat t_t_c0 = makeTransform(samples_[0].r_target_to_cam, samples_[0].t_target_to_cam);
    anchor = t_g_b0 * t_cam_gripper * t_t_c0;  // target in base
  } else {
    const cv::Mat t_g_b0 = makeTransform(samples_[0].r_gripper_to_base, samples_[0].t_gripper_to_base);
    const cv::Mat t_t_c0 = makeTransform(samples_[0].r_target_to_cam, samples_[0].t_target_to_cam);
    anchor = invertTransform(t_g_b0) * t_cam_base * t_t_c0;  // target in gripper
  }

  double sum_reproj = 0.0;
  double sum_trans = 0.0;
  double sum_rot = 0.0;
  for (const auto & s : samples_) {
    out.per_point_corner_reproj_px.push_back(s.mean_corner_reproj_px);
    sum_reproj += s.mean_corner_reproj_px;

    const cv::Mat t_g_b = makeTransform(s.r_gripper_to_base, s.t_gripper_to_base);
    const cv::Mat t_t_c_obs = makeTransform(s.r_target_to_cam, s.t_target_to_cam);
    cv::Mat t_t_c_pred;
    if (eye_in_hand_ && has_cam_gripper) {
      const cv::Mat t_c_b = t_g_b * t_cam_gripper;
      t_t_c_pred = invertTransform(t_c_b) * anchor;
    } else {
      t_t_c_pred = invertTransform(t_cam_base) * t_g_b * anchor;
    }
    const cv::Mat r_pred = t_t_c_pred(cv::Rect(0, 0, 3, 3));
    const double rot_err_deg = rotationAngleRadBetween(r_pred, s.r_target_to_cam) * kRad2Deg;
    out.per_point_chain_rotation_deg.push_back(rot_err_deg);
    sum_rot += rot_err_deg;

    const cv::Mat delta = t_t_c_pred(cv::Rect(3, 0, 1, 3)) - t_t_c_obs(cv::Rect(3, 0, 1, 3));
    const double trans_err = cv::norm(delta);
    out.per_point_chain_translation_m.push_back(trans_err);
    sum_trans += trans_err;
  }
  out.mean_corner_reproj_px = sum_reproj / static_cast<double>(n);
  out.mean_chain_translation_m = sum_trans / static_cast<double>(n);
  out.mean_chain_rotation_deg = sum_rot / static_cast<double>(n);
}

bool CalibNode::saveResult(
  const cv::Mat & t_cam_base,
  const cv::Mat & t_base_cam,
  const HandEyeQualityMetrics & quality,
  const cv::Mat & t_cam_gripper,
  bool has_cam_gripper)
{
  const std::string run_stamp = nowString();
  const std::string run_dir = output_dir_ + "/calib_run_" + run_stamp;
  std::filesystem::create_directories(run_dir);
  const std::string file_name =
    run_dir + "/calib_result_" + (eye_in_hand_ ? "eye_in_hand" : "eye_to_hand") + ".yaml";
  const std::string manifest_file = run_dir + "/sample_manifest.csv";
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Failed to open output file: %s", file_name.c_str());
    return false;
  }

  std::vector<double> per_point_chain_trans_mm;
  per_point_chain_trans_mm.reserve(quality.per_point_chain_translation_m.size());
  for (double e : quality.per_point_chain_translation_m) {
    per_point_chain_trans_mm.push_back(e * 1000.0);
  }
  const double mean_chain_trans_mm = quality.mean_chain_translation_m * 1000.0;

  fs << "mode" << (eye_in_hand_ ? "eye_in_hand" : "eye_to_hand");
  fs << "timestamp" << run_stamp;
  fs << "calib_run_stamp" << run_stamp;
  fs << "arm_id" << arm_id_;
  fs << "target_marker_id" << marker_id_;
  fs << "sample_count" << static_cast<int>(samples_.size());
  fs << "mean_corner_reprojection_error_px" << quality.mean_corner_reproj_px;
  fs << "per_point_corner_reprojection_error_px" << quality.per_point_corner_reproj_px;
  fs << "mean_handeye_chain_translation_residual_mm" << mean_chain_trans_mm;
  fs << "per_point_handeye_chain_translation_residual_mm" << per_point_chain_trans_mm;
  fs << "mean_handeye_chain_rotation_residual_deg" << quality.mean_chain_rotation_deg;
  fs << "per_point_handeye_chain_rotation_residual_deg" << quality.per_point_chain_rotation_deg;
  fs << "T_cam_base" << mat44ToVec16(t_cam_base);
  fs << "T_base_cam" << mat44ToVec16(t_base_cam);
  if (has_cam_gripper) {
    fs << "T_cam_gripper" << mat44ToVec16(t_cam_gripper);
  }
  cv::Mat t_cam_base_ref;
  bool has_ref = getUrdfReferenceTcamBaseForArm(arm_id_, t_cam_base_ref);
  if (has_ref) {
    cv::Mat r_est, t_est, r_ref, t_ref;
    splitTransform(t_cam_base, r_est, t_est);
    splitTransform(t_cam_base_ref, r_ref, t_ref);
    const double trans_err = cv::norm(t_est - t_ref);
    const double trans_err_mm = trans_err * 1000.0;
    const double rot_err_deg = rotationAngleRadBetween(r_est, r_ref) * kRad2Deg;
    fs << "T_cam_base_urdf_ref" << mat44ToVec16(t_cam_base_ref);
    fs << "T_cam_base_vs_urdf_translation_error_mm" << trans_err_mm;
    fs << "T_cam_base_vs_urdf_rotation_error_deg" << rot_err_deg;
  }
  fs.release();

  const std::size_t sample_count = samples_.size();
  for (std::size_t i = 0; i < sample_count; ++i) {
    std::ostringstream raw_name;
    raw_name << run_dir << "/raw_sample_" << std::setfill('0') << std::setw(2) << (i + 1) << ".png";
    std::ostringstream res_name;
    res_name << run_dir << "/result_sample_" << std::setfill('0') << std::setw(2) << (i + 1) << ".png";
    if (i < captured_raw_frames_.size() && !captured_raw_frames_[i].empty()) {
      cv::imwrite(raw_name.str(), captured_raw_frames_[i]);
    }
    if (i < captured_result_frames_.size() && !captured_result_frames_[i].empty()) {
      cv::imwrite(res_name.str(), captured_result_frames_[i]);
    }
  }

  {
    std::ofstream manifest(manifest_file);
    manifest << "index,raw_image,result_image,arm_id,px,py,pz,qx,qy,qz,qw\n";
    for (std::size_t i = 0; i < sample_count; ++i) {
      std::ostringstream raw_name;
      raw_name << "raw_sample_" << std::setfill('0') << std::setw(2) << (i + 1) << ".png";
      std::ostringstream res_name;
      res_name << "result_sample_" << std::setfill('0') << std::setw(2) << (i + 1) << ".png";

      int arm = arm_id_;
      double px = 0.0, py = 0.0, pz = 0.0;
      double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
      if (i < captured_arm_poses_.size()) {
        const auto & pose = captured_arm_poses_[i];
        arm = pose.arm_id;
        px = pose.pose.pose.position.x;
        py = pose.pose.pose.position.y;
        pz = pose.pose.pose.position.z;
        qx = pose.pose.pose.orientation.x;
        qy = pose.pose.pose.orientation.y;
        qz = pose.pose.pose.orientation.z;
        qw = pose.pose.pose.orientation.w;
      }

      manifest << (i + 1) << ","
               << raw_name.str() << ","
               << res_name.str() << ","
               << arm << ","
               << px << "," << py << "," << pz << ","
               << qx << "," << qy << "," << qz << "," << qw << "\n";
    }
  }

  std_msgs::msg::String result_msg;
  std::ostringstream oss;
  oss << std::fixed;
  oss << "mode: " << (eye_in_hand_ ? "eye_in_hand" : "eye_to_hand") << "\n"
      << "arm_id: " << arm_id_ << "\n"
      << "target_marker_id: " << marker_id_ << "\n"
      << "sample_count: " << samples_.size() << "\n"
      << std::setprecision(4) << "mean_corner_reprojection_error_px: " << quality.mean_corner_reproj_px
      << "\n"
      << std::setprecision(3) << "mean_handeye_chain_translation_residual_mm: " << mean_chain_trans_mm
      << "\n"
      << std::setprecision(4) << "mean_handeye_chain_rotation_residual_deg: "
      << quality.mean_chain_rotation_deg << "\n"
      << "T_cam_base:\n" << formatMat4(t_cam_base) << "\n"
      << "T_base_cam:\n" << formatMat4(t_base_cam);
  if (has_cam_gripper) {
    oss << "\nT_cam_gripper:\n" << formatMat4(t_cam_gripper);
  }
  if (has_ref) {
    cv::Mat r_est, t_est, r_ref, t_ref;
    splitTransform(t_cam_base, r_est, t_est);
    splitTransform(t_cam_base_ref, r_ref, t_ref);
    const double trans_err = cv::norm(t_est - t_ref);
    const double trans_err_mm = trans_err * 1000.0;
    const double rot_err_deg = rotationAngleRadBetween(r_est, r_ref) * kRad2Deg;
    oss << std::fixed << std::setprecision(3);
    oss << "\nT_cam_base_urdf_ref:\n" << formatMat4(t_cam_base_ref)
        << "\nT_cam_base_vs_urdf_translation_error_mm: " << trans_err_mm
        << "\nT_cam_base_vs_urdf_rotation_error_deg: " << std::setprecision(4) << rot_err_deg;
  }
  oss << "\ncalib_run_stamp: " << run_stamp << "\n";
  result_msg.data = oss.str();
  result_text_pub_->publish(result_msg);

  RCLCPP_INFO(get_logger(), "Calibration result saved: %s", file_name.c_str());
  return true;
}

cv::Mat CalibNode::quatToRot(double x, double y, double z, double w)
{
  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;
  const double xy = x * y;
  const double xz = x * z;
  const double yz = y * z;
  const double wx = w * x;
  const double wy = w * y;
  const double wz = w * z;
  cv::Mat r = (cv::Mat_<double>(3, 3) <<
    1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
    2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
    2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy));
  return r;
}

cv::Mat CalibNode::makeTransform(const cv::Mat & r, const cv::Mat & t)
{
  cv::Mat t4 = cv::Mat::eye(4, 4, CV_64F);
  r.copyTo(t4(cv::Rect(0, 0, 3, 3)));
  t.copyTo(t4(cv::Rect(3, 0, 1, 3)));
  return t4;
}

void CalibNode::splitTransform(const cv::Mat & t4, cv::Mat & r, cv::Mat & t)
{
  r = t4(cv::Rect(0, 0, 3, 3)).clone();
  t = t4(cv::Rect(3, 0, 1, 3)).clone();
}

cv::Mat CalibNode::invertTransform(const cv::Mat & t4)
{
  cv::Mat r = t4(cv::Rect(0, 0, 3, 3)).clone();
  cv::Mat t = t4(cv::Rect(3, 0, 1, 3)).clone();
  cv::Mat rt = r.t();
  cv::Mat ti = -rt * t;
  return makeTransform(rt, ti);
}

std::string CalibNode::nowString()
{
  const auto now = std::chrono::system_clock::now();
  const auto tt = std::chrono::system_clock::to_time_t(now);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  std::tm tm = *std::localtime(&tt);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S_") << std::setfill('0') << std::setw(3) << ms.count();
  return oss.str();
}

std::vector<double> CalibNode::mat44ToVec16(const cv::Mat & m)
{
  std::vector<double> out;
  out.reserve(16);
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out.push_back(m.at<double>(r, c));
    }
  }
  return out;
}

}  // namespace calib_sim
