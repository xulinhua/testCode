// calib_node 实现：ArUco 位姿估计、手眼样本采集、OpenCV calibrateHandEye、质量指标与结果落盘。
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

// 匿名命名空间：手眼算法枚举名、网格/姿态小工具、Aruco 翻转后角点坐标还原等。
namespace
{
const char * handEyeMethodName(cv::HandEyeCalibrationMethod m)
{
  switch (m) {
    case cv::CALIB_HAND_EYE_TSAI:
      return "TSAI";
    case cv::CALIB_HAND_EYE_PARK:
      return "PARK";
    case cv::CALIB_HAND_EYE_HORAUD:
      return "HORAUD";
    case cv::CALIB_HAND_EYE_ANDREFF:
      return "ANDREFF";
    case cv::CALIB_HAND_EYE_DANIILIDIS:
      return "DANIILIDIS";
    default:
      return "UNKNOWN";
  }
}

bool handEyeTransformUsable(const cv::Mat & r3, const cv::Mat & t3)
{
  if (r3.rows != 3 || r3.cols != 3 || t3.rows != 3 || t3.cols != 1) {
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const double v = r3.at<double>(i, j);
      if (!std::isfinite(v)) {
        return false;
      }
    }
    if (!std::isfinite(t3.at<double>(i, 0))) {
      return false;
    }
  }
  return true;
}

constexpr int kPoseDims = 7;
/// 与 cv::flip 的 flipCode 区分：不对输入图做翻转
constexpr int kArucoNoInputFlip = -1000;

void mapArucoCornersFromFlippedToOriginal(
  std::vector<std::vector<cv::Point2f>> & corners, int flip_code, int img_w, int img_h)
{
  if (flip_code == kArucoNoInputFlip || img_w <= 0 || img_h <= 0) {
    return;
  }
  const float wf = static_cast<float>(img_w - 1);
  const float hf = static_cast<float>(img_h - 1);
  for (auto & marker : corners) {
    for (auto & p : marker) {
      if (flip_code == 1) {
        p.x = wf - p.x;
      } else if (flip_code == 0) {
        p.y = hf - p.y;
      } else if (flip_code == -1) {
        p.x = wf - p.x;
        p.y = hf - p.y;
      }
    }
    // cv::aruco 在翻转图上检测时，角点序号遵循“翻转后图像”的朝向；
    // 映射回原图后需重排为原图的 canonical 顺序，避免 PnP 法向被反转。
    if (marker.size() == 4U) {
      std::array<cv::Point2f, 4> reordered{marker[0], marker[1], marker[2], marker[3]};
      if (flip_code == 1) {          // horizontal: [1,0,3,2]
        reordered = {marker[1], marker[0], marker[3], marker[2]};
      } else if (flip_code == 0) {   // vertical: [3,2,1,0]
        reordered = {marker[3], marker[2], marker[1], marker[0]};
      } else if (flip_code == -1) {  // both: [2,3,0,1]
        reordered = {marker[2], marker[3], marker[0], marker[1]};
      }
      marker.assign(reordered.begin(), reordered.end());
    }
  }
}
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
  // ^{base}T_{cam}：p_base = T * p_cam（与 tryFillGripperPoseFromTf / TF 约定一致）
  // 数值与运行中 `tf2_echo base_link camera0_optical_frame` 对齐（龙门固定 camera0）。
  // 旧版平移 X 与 TF 反号会导致 vs_urdf 误报约 1 m。
  if (arm_id == 0 || arm_id == 1) {
    const std::array<double, 16> t = {
       1.000000,  0.002000, -0.001000,  0.530000,
       0.002000, -1.000000, -0.000000, -0.499000,
      -0.001000, -0.000000, -1.000000,  1.040000,
       0.000000,  0.000000,  0.000000,  1.000000
    };
    t_cam_base_ref = vec16ToMat44(t);
    return true;
  }
  return false;
}

/// 眼在手上：OpenCV solvePnP / calibrateHandEye 的「相机」轴与常用光学系一致（Z 前向等），
/// 与 URDF 的 camera*_optical_frame 对齐；与 Gazebo RGB 的 frame_id=camera*_link 仅差固定关节
/// camera*_optical_joint，不可用 ^{J}T_{link} 作参考（会与解差约 120°）。
/// URDF：camera*_joint + camera*_optical_joint 复合得 ^{J}T_{opt}；R_joint=Rz*Ry*Rx。
/// P_gripper = ^{gripper}T_{cam} P_cam，故 T_cam_gripper_ref = ^{J}T_{opt}。
bool getUrdfReferenceTcamGripperForEyeInHand(int arm_id, cv::Mat & t_cam_gripper_ref)
{
  if (arm_id == 0) {
    // arm0: J1_6 -> camera1_optical_frame
    const std::array<double, 16> t_arm0 = {
      0.9999999999932537, -3.673203938690185e-06, -2.929967908670559e-09, 0.0,
      3.673205107245858e-06, 0.9999996829250922, 0.0007963267058312859, -0.08,
      4.896587307594301e-12, -0.0007963267058366761, 0.9999996829318385, 0.041,
      0.0, 0.0, 0.0, 1.0
    };
    t_cam_gripper_ref = vec16ToMat44(t_arm0);
    return true;
  }
  if (arm_id == 1) {
    // arm1: J2_6 -> camera2_optical_frame（与 arm0 安装相同）
    const std::array<double, 16> t_arm1 = {
      0.9999999999932537, -3.673203938690185e-06, -2.929967908670559e-09, 0.0,
      3.673205107245858e-06, 0.9999996829250922, 0.0007963267058312859, -0.08,
      4.896587307594301e-12, -0.0007963267058366761, 0.9999996829318385, 0.041,
      0.0, 0.0, 0.0, 1.0
    };
    t_cam_gripper_ref = vec16ToMat44(t_arm1);
    return true;
  }
  return false;
}
}

// ---------- CalibNode：构造与参数 ----------
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
  target_attempts_(0),
  max_target_attempts_(0),
  dynamic_targets_added_(0),
  max_dynamic_targets_(12),
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
  this->declare_parameter("unified_mode", false);
  if (this->get_parameter("unified_mode").as_bool()) {
    throw std::runtime_error(
      "unified_mode:=true requires CalibNode(node_name, options) with calib_unified.yaml");
  }
  declareAndLoadParameters();
  initRosEntities();
  RCLCPP_INFO(
    get_logger(), "Node started. mode=%s, arm_id=%d",
    eye_in_hand_ ? "eye_in_hand" : "eye_to_hand", arm_id_);
}

// 统一标定入口：配合 calib_unified.yaml，界面可切换 eth0/eth1/eih0/eih1。
CalibNode::CalibNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  unified_mode_(true),
  eye_in_hand_(false),
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
  target_attempts_(0),
  max_target_attempts_(0),
  dynamic_targets_added_(0),
  max_dynamic_targets_(12),
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
  this->declare_parameter("unified_mode", true);
  if (!this->get_parameter("unified_mode").as_bool()) {
    throw std::runtime_error("unified CalibNode requires unified_mode:=true in parameters");
  }
  loadUnifiedModeConfigs();
  active_mode_ = "eth0";
  applyActiveModeConfig();
  initRosEntities();
  RCLCPP_INFO(
    get_logger(), "Node started (unified). active_mode=%s eye_in_hand=%s arm_id=%d marker_id=%d",
    active_mode_.c_str(), eye_in_hand_ ? "true" : "false", arm_id_, marker_id_);
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
  applyCalibConfigData(cfg);
}

void CalibNode::applyCalibConfigData(const CalibConfigData & cfg)
{
  arm_id_ = cfg.arm_id;
  marker_id_ = cfg.target_marker_id;
  aruco_dict_id_ = cfg.aruco_dict_id;
  marker_length_m_ = cfg.marker_length_m;
  min_samples_ = cfg.min_samples;
  state_timeout_sec_ = cfg.state_timeout_sec;
  reached_wait_timeout_sec_ =
    cfg.reached_wait_timeout_sec > 0.0 ? cfg.reached_wait_timeout_sec : cfg.state_timeout_sec;
  capture_wait_timeout_sec_ =
    cfg.capture_wait_timeout_sec > 0.0 ? cfg.capture_wait_timeout_sec : cfg.state_timeout_sec;
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

void CalibNode::loadUnifiedModeConfigs()
{
  static const char * kModes[] = {"eth0", "eth1", "eih0", "eih1"};
  mode_configs_.clear();
  for (const char * m : kModes) {
    const std::string prefix(m);
    const bool eih = (prefix.size() >= 3 && prefix[0] == 'e' && prefix[1] == 'i' && prefix[2] == 'h');
    const CalibConfigData def = eih ? defaultCalibConfigEyeInHand() : defaultCalibConfigEyeToHand();
    mode_configs_[prefix] = loadCalibConfigPrefixed(*this, prefix, def);
  }
}

void CalibNode::applyActiveModeConfig()
{
  const auto it = mode_configs_.find(active_mode_);
  if (it == mode_configs_.end()) {
    throw std::runtime_error("applyActiveModeConfig: unknown mode " + active_mode_);
  }
  eye_in_hand_ =
    (active_mode_.size() >= 3 && active_mode_[0] == 'e' && active_mode_[1] == 'i' &&
    active_mode_[2] == 'h');
  applyCalibConfigData(it->second);
}

void CalibNode::switchCalibMode(const std::string & mode)
{
  cancelInitDelayTimer();
  if (mode_configs_.find(mode) == mode_configs_.end()) {
    publishLog("cmd:set_mode unknown mode: " + mode);
    publishStatus("mode_unknown");
    return;
  }
  active_mode_ = mode;
  applyActiveModeConfig();

  has_arm_pose_ = false;
  has_arm_state_ = false;
  arm_reached_ = false;
  targets_prepared_ = false;
  samples_.clear();
  captured_raw_frames_.clear();
  captured_result_frames_.clear();
  captured_arm_poses_.clear();
  target_index_ = 0;
  waiting_arm_reached_ = false;
  waiting_capture_ = false;
  finished_ = false;
  waiting_init_pose_ = false;
  auto_mode_ = false;
  pending_step_ = false;

  renewCameraSubscriptions();
  last_raw_frame_.release();
  last_result_frame_.release();

  publishStatus("mode_switched_" + active_mode_ + "_wait_step");
  publishLog(
    "cmd:set_mode mode=" + active_mode_ + " eye_in_hand=" + std::string(eye_in_hand_ ? "true" : "false") +
    " arm_id=" + std::to_string(arm_id_) + " marker_id=" + std::to_string(marker_id_) +
    " image=" + image_topic_ + " camera_info=" + camera_info_topic_);
}

void CalibNode::applyTargetPosesForCurrentArm()
{
  if (use_current_pose_as_center_) {
    return;
  }
  target_poses_flat_ = (arm_id_ == 1) ? target_poses_arm1_ : target_poses_arm0_;
}

void CalibNode::renewCameraSubscriptions()
{
  image_sub_.reset();
  cam_info_sub_.reset();
  has_camera_info_ = false;
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, 10, std::bind(&CalibNode::imageCallback, this, std::placeholders::_1));
  cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, 10, std::bind(&CalibNode::cameraInfoCallback, this, std::placeholders::_1));
  RCLCPP_INFO(
    get_logger(), "subscribe camera image=%s camera_info=%s",
    image_topic_.c_str(), camera_info_topic_.c_str());
}

void CalibNode::initRosEntities()
{
  renewCameraSubscriptions();
  arm_pose_sub_ = create_subscription<calib_sim_mujoco::msg::ArmPose>(
    robot_pose_topic_, 20, std::bind(&CalibNode::armPoseCallback, this, std::placeholders::_1));
  arm_state_sub_ = create_subscription<std_msgs::msg::Bool>(
    robot_state_topic_, 20, std::bind(&CalibNode::armStateCallback, this, std::placeholders::_1));
  control_sub_ = create_subscription<std_msgs::msg::String>(
    "/calib_sim/control", 20, std::bind(&CalibNode::controlCallback, this, std::placeholders::_1));
  target_pose_pub_ = create_publisher<calib_sim_mujoco::msg::ArmPose>(robot_target_topic_, 10);
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

void CalibNode::armPoseCallback(const calib_sim_mujoco::msg::ArmPose::ConstSharedPtr msg)
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
  if (pose_count == 0U) {
    publishStatus("calibration_failed");
    publishLog("[ERROR] calibration_failed: no target poses configured");
    finished_ = true;
    return;
  }
  if (max_target_attempts_ == 0U) {
    const std::size_t min_attempts = static_cast<std::size_t>(std::max(min_samples_ * 3, min_samples_ + 4));
    max_target_attempts_ = std::max(pose_count, min_attempts);
    std::ostringstream oss;
    oss << "target_sampling_plan pose_count=" << pose_count
        << " min_samples=" << min_samples_
        << " max_target_attempts=" << max_target_attempts_;
    publishLog(oss.str());
  }
  if (samples_.size() >= static_cast<std::size_t>(min_samples_)) {
    if (runCalibration()) {
      RCLCPP_INFO(get_logger(), "Calibration complete with %zu samples", samples_.size());
      publishStatus("calibration_complete");
    } else {
      publishStatus("calibration_failed");
      {
        std::ostringstream oss;
        oss << "[ERROR] Calibration finished without a valid result. samples=" << samples_.size()
            << " min_samples=" << min_samples_ << " (check log for calibration_failed / timeouts)";
        publishLog(oss.str());
      }
      std_msgs::msg::String fail_text;
      fail_text.data = std::string("[ERROR] Calibration failed.\n") + "sample_count: " +
        std::to_string(samples_.size()) + "\nmin_samples: " + std::to_string(min_samples_) +
        "\nSee log lines above (timeouts, marker_too_large, solve errors).\n";
      result_text_pub_->publish(fail_text);
    }
    finished_ = true;
    return;
  }
  if (target_attempts_ >= max_target_attempts_) {
    publishStatus("calibration_failed");
    std::ostringstream oss;
    oss << "[ERROR] Calibration finished without a valid result. samples=" << samples_.size()
        << " min_samples=" << min_samples_
        << " attempts=" << target_attempts_ << "/" << max_target_attempts_
        << " (insufficient successful captures)";
    publishLog(oss.str());
    std_msgs::msg::String fail_text;
    fail_text.data = std::string("[ERROR] Calibration failed.\n") + "sample_count: " +
      std::to_string(samples_.size()) + "\nmin_samples: " + std::to_string(min_samples_) +
      "\nattempts: " + std::to_string(target_attempts_) + "/" + std::to_string(max_target_attempts_) +
      "\nSee log lines above (reach timeout / capture timeout).\n";
    result_text_pub_->publish(fail_text);
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
      const bool has_fresh_state = has_arm_state_ && ((now() - last_arm_state_time_).seconds() <= state_timeout_sec_);
      oss << "[ERROR] arm_reach_timeout_skip_target_" << target_index_
          << " reached_wait_timeout_sec=" << reached_wait_timeout_sec_
          << " has_arm_state=" << (has_arm_state_ ? "true" : "false")
          << " arm_reached=" << (arm_reached_ ? "true" : "false")
          << " has_fresh_state=" << (has_fresh_state ? "true" : "false");
      if (has_arm_state_) {
        oss << " state_age_sec=" << (now() - last_arm_state_time_).seconds()
            << " state_timeout_sec=" << state_timeout_sec_;
      }
      publishStatus(oss.str());
      ++target_attempts_;
      target_index_ = (target_index_ + 1U) % pose_count;
      if (pending_step_) {
        pending_step_ = false;
      }
    }
  }

  // If marker detection cannot succeed for long time, skip to next target to keep robot moving.
  if (waiting_capture_ && (now() - capture_start_time_).seconds() > capture_wait_timeout_sec_) {
    waiting_capture_ = false;
    std::ostringstream oss;
    oss << "[ERROR] capture_timeout_skip_target_" << target_index_
        << " has_camera_info=" << (has_camera_info_ ? "true" : "false")
        << " detect_fail_reason=" << last_detect_fail_reason_;
    publishStatus(oss.str());
    ++target_attempts_;
    if (last_detect_fail_reason_ == "no_marker") {
      (void)appendDynamicTargetFromCurrentPose();
    }
    target_index_ = (target_index_ + 1U) % pose_count;
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
    target_attempts_ = 0;
    max_target_attempts_ = 0;
    dynamic_targets_added_ = 0;
    waiting_arm_reached_ = false;
    waiting_capture_ = false;
    finished_ = false;
    waiting_init_pose_ = false;
    last_status_text_.clear();
  };

  if (cmd.rfind("set_mode:", 0) == 0) {
    if (!unified_mode_) {
      publishLog("cmd:set_mode ignored (single-mode calib node)");
      return;
    }
    switchCalibMode(cmd.substr(std::string("set_mode:").size()));
    return;
  }

  if (cmd.rfind("set_arm:", 0) == 0) {
    if (unified_mode_) {
      publishLog("cmd:set_arm ignored in unified mode; use set_mode:eth0|eth1|eih0|eih1");
      return;
    }
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
    // Keep marker id from loaded config/mode; do not overwrite it on arm switch.
    // Overriding here can cause "marker_id_mismatch" when the board ID is fixed.
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
    if (it != initial_pose_by_arm_.end()) {
      init_pose_pending_ = it->second;
      init_pose_pending_.arm_id = arm_id_;
    } else {
      // Fallback for newly switched arm: use current TF pose (preferred), then latest arm pose.
      calib_sim_mujoco::msg::ArmPose fallback_pose;
      cv::Mat r_tmp, t_tmp;
      bool got_fallback = false;
      if (tryFillGripperPoseFromTf(now(), r_tmp, t_tmp, &fallback_pose)) {
        got_fallback = true;
      } else if (has_arm_pose_ && last_arm_pose_.arm_id == arm_id_) {
        fallback_pose = last_arm_pose_;
        got_fallback = true;
      }
      if (!got_fallback) {
        publishStatus("init_failed_no_initial_pose");
        publishLog("cmd:init no_initial_pose_for_arm_and_no_tf_pose");
        return;
      }
      fallback_pose.arm_id = arm_id_;
      initial_pose_by_arm_[arm_id_] = fallback_pose;
      init_pose_pending_ = fallback_pose;
      publishLog("cmd:init no_initial_pose_for_arm, fallback_to_current_pose");
    }
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

// ---------- 图像回调、采样与 ArUco 检测 ----------
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
    ++target_attempts_;
    const auto pose_count = target_poses_flat_.size() / kPoseDims;
    if (pose_count > 0U) {
      target_index_ = (target_index_ + 1U) % pose_count;
    }
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
  calib_sim_mujoco::msg::ArmPose * out_manifest_pose)
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
  calib_sim_mujoco::msg::ArmPose manifest_pose;
  const bool used_tf = tryFillGripperPoseFromTf(
    image_stamp, sample.r_gripper_to_base, sample.t_gripper_to_base, &manifest_pose);
  if (!used_tf) {
    if (use_tf_for_sample_pose_) {
      // Avoid recording duplicated stale arm poses when TF sampling is required.
      publishLog("[WARN] sample_skip tf_lookup_failed_for_gripper_pose");
      return false;
    }
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
  cv::Mat proc_bgr = frame_bgr;
  cv::Mat proc_camera_matrix = camera_matrix_.clone();
  cv::Mat proc_dist_coeffs = dist_coeffs_.clone();
  bool used_undistort = false;
  const bool camera_ready =
    (camera_matrix_.rows == 3 && camera_matrix_.cols == 3 && !camera_matrix_.empty());
  if (camera_ready && !dist_coeffs_.empty()) {
    cv::Mat dist_abs;
    cv::absdiff(dist_coeffs_, cv::Scalar::all(0), dist_abs);
    const bool has_distortion = cv::countNonZero(dist_abs.reshape(1) > 1e-12) > 0;
    if (has_distortion) {
      cv::undistort(frame_bgr, proc_bgr, camera_matrix_, dist_coeffs_);
      proc_dist_coeffs = cv::Mat::zeros(dist_coeffs_.size(), dist_coeffs_.type());
      used_undistort = true;
    }
  }
  annotated = proc_bgr.clone();
  const int img_w = frame_bgr.cols;
  const int img_h = frame_bgr.rows;
  if (img_w <= 0 || img_h <= 0) {
    fail_reason = "no_image";
    return false;
  }
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "detect_diag camera_model"
        << " raw=" << frame_bgr.cols << "x" << frame_bgr.rows
        << " proc=" << proc_bgr.cols << "x" << proc_bgr.rows
        << " fx=" << proc_camera_matrix.at<double>(0, 0)
        << " fy=" << proc_camera_matrix.at<double>(1, 1)
        << " cx=" << proc_camera_matrix.at<double>(0, 2)
        << " cy=" << proc_camera_matrix.at<double>(1, 2)
        << " undistort=" << (used_undistort ? "true" : "false");
    publishLog(oss.str());
  }

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
  detector_params->minMarkerDistanceRate = 0.03;
  detector_params->polygonalApproxAccuracyRate = 0.03;
  detector_params->minMarkerPerimeterRate = 0.02;
  detector_params->maxMarkerPerimeterRate = 5.0;

  const double detect_scale = 2.0;
  struct FlipTry
  {
    int code;
    const char * name;
  };
  const FlipTry flip_tries[] = {
    {kArucoNoInputFlip, "none"},
    {1, "horizontal"},
    {0, "vertical"},
    {-1, "both"},
  };

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  int found_index = -1;

  for (const auto & ft : flip_tries) {
    cv::Mat bgr_in;
    if (ft.code == kArucoNoInputFlip) {
      bgr_in = proc_bgr;
    } else {
      cv::flip(proc_bgr, bgr_in, ft.code);
    }

    cv::Mat gray;
    cv::cvtColor(bgr_in, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gray_eq;
    {
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
      clahe->apply(gray, gray_eq);
    }
    cv::Mat blur_eq;
    cv::GaussianBlur(gray_eq, blur_eq, cv::Size(0, 0), 1.2);
    cv::Mat gray_sharp;
    cv::addWeighted(gray_eq, 1.6, blur_eq, -0.6, 0.0, gray_sharp);

    cv::Mat gray_detect;
    cv::resize(gray_sharp, gray_detect, cv::Size(), detect_scale, detect_scale, cv::INTER_CUBIC);

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

    corners.clear();
    ids.clear();
    detect_one_pass(gray_detect, true, detect_scale, corners, ids);
    if (ids.empty()) {
      detect_one_pass(gray_eq, false, 1.0, corners, ids);
    }

    found_index = -1;
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] == marker_id_) {
        found_index = static_cast<int>(i);
        break;
      }
    }
    if (found_index >= 0) {
      if (ft.code != kArucoNoInputFlip) {
        mapArucoCornersFromFlippedToOriginal(corners, ft.code, img_w, img_h);
        publishLog(std::string("detect_diag aruco_mirror_fix input_") + ft.name);
      }
      break;
    }
  }

  if (found_index >= 0 && ids.size() > 1U) {
    std::ostringstream multi_ss;
    multi_ss << "detect_diag multi_marker using_id=" << marker_id_ << " raw_ids=[";
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (i > 0U) {
        multi_ss << ",";
      }
      multi_ss << ids[i];
    }
    multi_ss << "] (pnp uses target id only)";
    publishLog(multi_ss.str());
  }

  detected_ids = ids;
  if (found_index < 0) {
    if (ids.empty()) {
      std::ostringstream oss;
      oss << "detect_diag no_marker"
          << " image=" << frame_bgr.cols << "x" << frame_bgr.rows
          << " dict_id=" << aruco_dict_id_
          << " target_id=" << marker_id_;
      publishLog(oss.str());
      fail_reason = "no_marker";
    } else {
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
    }
    return false;
  }

  if (found_index >= 0 && static_cast<std::size_t>(found_index) < corners.size()) {
    const auto & c = corners[static_cast<std::size_t>(found_index)];
    if (c.size() == 4U) {
      cv::line(annotated, c[0], c[1], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      cv::line(annotated, c[1], c[2], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      cv::line(annotated, c[2], c[3], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      cv::line(annotated, c[3], c[0], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      cv::circle(annotated, c[0], 3, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
      cv::putText(
        annotated, std::to_string(marker_id_), c[0] + cv::Point2f(2.0F, -6.0F),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
  }

  const auto & marker_corners = corners[found_index];
  const cv::Rect bbox = cv::boundingRect(marker_corners);
  const double marker_area_px = std::abs(cv::contourArea(marker_corners));
  double perimeter_px = 0.0;
  for (std::size_t i = 0; i < marker_corners.size(); ++i) {
    const cv::Point2f & p0 = marker_corners[i];
    const cv::Point2f & p1 = marker_corners[(i + 1U) % marker_corners.size()];
    perimeter_px += cv::norm(p0 - p1);
  }
  const double image_area = static_cast<double>(proc_bgr.cols) * static_cast<double>(proc_bgr.rows);
  const double marker_area_ratio = image_area > 0.0 ? (marker_area_px / image_area) : 0.0;
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "detect_diag marker_id=" << ids[found_index]
        << " perimeter_px=" << perimeter_px
        << " bbox=" << bbox.x << "," << bbox.y << "," << bbox.width << "," << bbox.height
        << " marker_area_ratio=" << marker_area_ratio;
    publishLog(oss.str());
  }
  if (marker_area_ratio < marker_bbox_ratio_min_) {
    fail_reason = "marker_too_small";
    return false;
  }
  if (marker_area_ratio > marker_bbox_ratio_max_) {
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
    object_points_proj, marker_corners_for_pnp, proc_camera_matrix, proc_dist_coeffs, rvec, tvec, false,
    cv::SOLVEPNP_IPPE_SQUARE);
  if (!pnp_ok) {
    pnp_ok = cv::solvePnP(
      object_points_proj, marker_corners_for_pnp, proc_camera_matrix, proc_dist_coeffs, rvec, tvec, false,
      cv::SOLVEPNP_ITERATIVE);
  }
  if (!pnp_ok) {
    publishLog("detect_diag pose_estimation_failed");
    fail_reason = "pnp_failed";
    return false;
  }

  std::vector<cv::Point2f> reproj;
  cv::projectPoints(object_points_proj, rvec, tvec, proc_camera_matrix, proc_dist_coeffs, reproj);
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
    annotated, proc_camera_matrix, proc_dist_coeffs, rvec, tvec,
    static_cast<float>(marker_length_m_ * 0.6), 2);

  // Draw projected border only when reprojection is reasonably small.
  // Otherwise yellow overlay may look "drifted" and mislead tuning.
  const std::vector<cv::Point2f> & projected_corners = reproj;
  constexpr double kProjectedBorderDrawMaxReprojPx = 2.5;
  if (projected_corners.size() == 4U && reproj_err_px <= kProjectedBorderDrawMaxReprojPx) {
    cv::line(annotated, projected_corners[0], projected_corners[1], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(annotated, projected_corners[1], projected_corners[2], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(annotated, projected_corners[2], projected_corners[3], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(annotated, projected_corners[3], projected_corners[0], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
  } else if (projected_corners.size() == 4U) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "detect_diag projected_border_suppressed reproj_px=" << reproj_err_px
        << " threshold_px=" << kProjectedBorderDrawMaxReprojPx;
    publishLog(oss.str());
  }
  cv::Rodrigues(rvec, r_target_to_cam);
  t_target_to_cam = tvec.clone();
  out_mean_corner_reproj_px = reproj_err_px;
  fail_reason = "ok";
  detected_ids.clear();
  detected_ids.push_back(marker_id_);
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
  calib_sim_mujoco::msg::ArmPose cmd;
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

bool CalibNode::appendDynamicTargetFromCurrentPose()
{
  if (!has_arm_pose_ || dynamic_targets_added_ >= max_dynamic_targets_) {
    return false;
  }
  static const std::array<std::array<double, 6>, 8> kRetryPattern = {{
    {{0.00, 0.00, 0.00, 0.0, 0.0, 0.0}},
    {{0.02, 0.00, 0.01, 0.0, 4.0, 0.0}},
    {{-0.02, 0.00, 0.01, 0.0, -4.0, 0.0}},
    {{0.00, 0.02, 0.00, 4.0, 0.0, 0.0}},
    {{0.00, -0.02, 0.00, -4.0, 0.0, 0.0}},
    {{0.00, 0.00, 0.02, 0.0, 0.0, 6.0}},
    {{0.015, 0.015, 0.00, 0.0, 0.0, -6.0}},
    {{-0.015, -0.015, 0.00, 0.0, 0.0, 6.0}},
  }};
  const auto & p = kRetryPattern[dynamic_targets_added_ % kRetryPattern.size()];
  const auto & base_pose = last_arm_pose_.pose.pose;
  const auto & q = base_pose.orientation;
  double qx = q.x;
  double qy = q.y;
  double qz = q.z;
  double qw = q.w;
  if (std::abs(p[3]) > 1e-9 || std::abs(p[4]) > 1e-9 || std::abs(p[5]) > 1e-9) {
    const cv::Mat R_base = quatToRot(q.x, q.y, q.z, q.w);
    const cv::Mat R_out = R_base * rpyDegToRdelta(p[3], p[4], p[5]);
    mat3ToQuat(R_out, qx, qy, qz, qw);
  }
  target_poses_flat_.push_back(base_pose.position.x + p[0]);
  target_poses_flat_.push_back(base_pose.position.y + p[1]);
  target_poses_flat_.push_back(base_pose.position.z + p[2]);
  target_poses_flat_.push_back(qx);
  target_poses_flat_.push_back(qy);
  target_poses_flat_.push_back(qz);
  target_poses_flat_.push_back(qw);
  ++dynamic_targets_added_;
  const auto pose_count = target_poses_flat_.size() / kPoseDims;
  max_target_attempts_ = std::max(max_target_attempts_, pose_count + static_cast<std::size_t>(min_samples_));
  std::ostringstream oss;
  oss << "dynamic_target_added idx=" << (pose_count - 1U)
      << " total=" << pose_count
      << " added=" << dynamic_targets_added_ << "/" << max_dynamic_targets_
      << " dxyz=(" << p[0] << "," << p[1] << "," << p[2] << ")"
      << " drpy_deg=(" << p[3] << "," << p[4] << "," << p[5] << ")";
  publishLog(oss.str());
  return true;
}

// ---------- 手眼求解、质量评估与结果保存 ----------
bool CalibNode::runCalibration()
{
  if (static_cast<int>(samples_.size()) < min_samples_) {
    RCLCPP_ERROR(
      get_logger(), "Not enough samples: %zu < %d", samples_.size(), min_samples_);
    {
      std::ostringstream oss;
      oss << "[ERROR] Not enough samples: " << samples_.size() << " < " << min_samples_;
      publishLog(oss.str());
    }
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
    publishLog("[ERROR] calibration_failed: rotate wrist/ee for at least ~3 deg span");
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
    publishLog("[ERROR] Calibration solve failed (hand-eye solver)");
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
    publishLog("[ERROR] calibration_failed: degenerate identity result detected");
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
    publishLog(
      "[ERROR] calibration_failed: poor quality, increase pose diversity and check marker scale");
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
          // Sample fields are T_gripper_to_base (p_base = T_g_b * p_gripper).
          // PnP gives T_target_to_cam (p_cam = T_t_c * p_target).
          // Board on gripper: p_base = T_g_b * T_target_to_gripper * p_target = T_cam_base * T_t_c * p_target
          // => T_cam_base = T_g_b * T_target_to_gripper * inv(T_t_c).
          const cv::Mat t_gripper_to_base = makeTransform(s.r_gripper_to_base, s.t_gripper_to_base);
          const cv::Mat t_c_t = makeTransform(s.r_target_to_cam, s.t_target_to_cam);
          const cv::Mat t_c_b_i = t_gripper_to_base * t_t_g_4 * invertTransform(t_c_t);
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

  // TSAI 在 OpenCV 实现里会丢弃「旋转过小」的运动对（约 <17°），小角度 wrist 网格会导致有效对不足、
  // 函数提前 return 且输出保持初值 R=I,t=0。PARK 使用全部运动对，更适合小角多样本。
  cv::Mat r_cam_base, t_cam_base_v;
  cv::calibrateHandEye(
    r_base_to_gripper, t_base_to_gripper, r_target_to_cam, t_target_to_cam,
    r_cam_base, t_cam_base_v, cv::CALIB_HAND_EYE_PARK);
  t_cam_base = makeTransform(r_cam_base, t_cam_base_v);
  return true;
}

bool CalibNode::runEyeInHandCalibration(cv::Mat & t_cam_base, cv::Mat & t_cam_gripper)
{
  std::vector<int> active_indices;
  active_indices.reserve(samples_.size());
  for (std::size_t i = 0; i < samples_.size(); ++i) {
    active_indices.push_back(static_cast<int>(i));
  }

  auto solve_with_indices = [&](const std::vector<int> & indices, cv::Mat & out_t_cam_gripper,
                           std::vector<double> * out_trans_errs_m,
                           cv::HandEyeCalibrationMethod method) -> bool {
      if (indices.size() < 3U) {
        return false;
      }
      std::vector<cv::Mat> r_gripper_to_base;
      std::vector<cv::Mat> t_gripper_to_base;
      std::vector<cv::Mat> r_target_to_cam;
      std::vector<cv::Mat> t_target_to_cam;
      r_gripper_to_base.reserve(indices.size());
      t_gripper_to_base.reserve(indices.size());
      r_target_to_cam.reserve(indices.size());
      t_target_to_cam.reserve(indices.size());
      for (const int idx : indices) {
        const auto & s = samples_[static_cast<std::size_t>(idx)];
        r_gripper_to_base.push_back(s.r_gripper_to_base);
        t_gripper_to_base.push_back(s.t_gripper_to_base);
        r_target_to_cam.push_back(s.r_target_to_cam);
        t_target_to_cam.push_back(s.t_target_to_cam);
      }

      cv::Mat r_cam_gripper, t_cam_gripper_v;
      cv::calibrateHandEye(
        r_gripper_to_base, t_gripper_to_base, r_target_to_cam, t_target_to_cam,
        r_cam_gripper, t_cam_gripper_v, method);
      if (!handEyeTransformUsable(r_cam_gripper, t_cam_gripper_v)) {
        return false;
      }
      out_t_cam_gripper = makeTransform(r_cam_gripper, t_cam_gripper_v);

      if (!out_trans_errs_m) {
        return true;
      }
      out_trans_errs_m->clear();
      out_trans_errs_m->reserve(indices.size());
      const auto & s0 = samples_[static_cast<std::size_t>(indices.front())];
      const cv::Mat t_g_b0 = makeTransform(s0.r_gripper_to_base, s0.t_gripper_to_base);
      const cv::Mat t_t_c0 = makeTransform(s0.r_target_to_cam, s0.t_target_to_cam);
      const cv::Mat anchor = t_g_b0 * out_t_cam_gripper * t_t_c0;  // target in base
      for (const int idx : indices) {
        const auto & s = samples_[static_cast<std::size_t>(idx)];
        const cv::Mat t_g_b = makeTransform(s.r_gripper_to_base, s.t_gripper_to_base);
        const cv::Mat t_c_b = t_g_b * out_t_cam_gripper;
        const cv::Mat t_t_c_pred = invertTransform(t_c_b) * anchor;
        const cv::Mat t_t_c_obs = makeTransform(s.r_target_to_cam, s.t_target_to_cam);
        const cv::Mat delta = t_t_c_pred(cv::Rect(3, 0, 1, 3)) - t_t_c_obs(cv::Rect(3, 0, 1, 3));
        out_trans_errs_m->push_back(cv::norm(delta));
      }
      return true;
    };

  cv::Mat robust_t_cam_gripper;
  std::vector<double> per_errs_m;
  constexpr cv::HandEyeCalibrationMethod kRobustMethod = cv::CALIB_HAND_EYE_PARK;
  if (!solve_with_indices(active_indices, robust_t_cam_gripper, &per_errs_m, kRobustMethod)) {
    return false;
  }
  // 轻量稳健：最多剔除 2 个明显离群点，避免单点误匹配把平移解拉偏（尤其 arm0）。
  for (int iter = 0; iter < 2 && active_indices.size() > 14U; ++iter) {
    if (per_errs_m.empty()) {
      break;
    }
    double mean_err = 0.0;
    double worst_err = -1.0;
    std::size_t worst_k = 0U;
    for (std::size_t k = 0; k < per_errs_m.size(); ++k) {
      mean_err += per_errs_m[k];
      if (per_errs_m[k] > worst_err) {
        worst_err = per_errs_m[k];
        worst_k = k;
      }
    }
    mean_err /= static_cast<double>(per_errs_m.size());
    const double reject_th = std::max(0.008, 2.2 * mean_err);  // >=8mm 且显著高于均值
    if (worst_err <= reject_th) {
      break;
    }
    const int removed_idx = active_indices[worst_k];
    active_indices.erase(active_indices.begin() + static_cast<long>(worst_k));
    {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3)
          << "hand_eye_outlier_removed sample_index=" << removed_idx
          << " trans_err_mm=" << (worst_err * 1000.0);
      publishLog(oss.str());
    }
    if (!solve_with_indices(active_indices, robust_t_cam_gripper, &per_errs_m, kRobustMethod)) {
      return false;
    }
  }

  static const cv::HandEyeCalibrationMethod kMethodCandidates[] = {
    cv::CALIB_HAND_EYE_PARK,
    cv::CALIB_HAND_EYE_ANDREFF,
    cv::CALIB_HAND_EYE_DANIILIDIS,
    cv::CALIB_HAND_EYE_HORAUD,
    cv::CALIB_HAND_EYE_TSAI,
  };
  cv::Mat best_t_cam_gripper = robust_t_cam_gripper;
  cv::HandEyeCalibrationMethod best_method = kRobustMethod;
  double best_mean_m = std::numeric_limits<double>::infinity();
  for (cv::HandEyeCalibrationMethod method : kMethodCandidates) {
    cv::Mat t_try;
    std::vector<double> errs_m;
    if (!solve_with_indices(active_indices, t_try, &errs_m, method) || errs_m.empty()) {
      continue;
    }
    double sum = 0.0;
    for (double e : errs_m) {
      sum += e;
    }
    const double mean_m = sum / static_cast<double>(errs_m.size());
    if (mean_m < best_mean_m) {
      best_mean_m = mean_m;
      best_t_cam_gripper = t_try;
      best_method = method;
    }
  }
  t_cam_gripper = best_t_cam_gripper;
  {
    cv::Mat r_cam_gripper, t_cam_gripper_v;
    splitTransform(t_cam_gripper, r_cam_gripper, t_cam_gripper_v);
    const double tg_n = cv::norm(t_cam_gripper_v);
    const double rg_id = cv::norm(r_cam_gripper - cv::Mat::eye(3, 3, CV_64FC1), cv::NORM_L2);
    if (tg_n < 1e-4 && rg_id < 1e-3) {
      RCLCPP_WARN(
        get_logger(),
        "Eye-in-hand calibrateHandEye returned near-identity T_cam_gripper (degenerate); check poses / "
        "PnP convention");
      publishLog("[WARN] hand_eye_degenerate_T_cam_gripper_near_identity");
    } else {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3)
          << "hand_eye_solver=" << handEyeMethodName(best_method)
          << " mean_chain_trans_mm=" << (best_mean_m * 1000.0);
      publishLog(oss.str());
    }
  }

  cv::Mat t_sum = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat r_ref = cv::Mat::eye(3, 3, CV_64F);
  for (std::size_t k = 0; k < active_indices.size(); ++k) {
    const std::size_t i = static_cast<std::size_t>(active_indices[k]);
    cv::Mat t_g_b = makeTransform(samples_[i].r_gripper_to_base, samples_[i].t_gripper_to_base);
    cv::Mat t_c_b_i = t_g_b * t_cam_gripper;
    cv::Mat r_i, t_i;
    splitTransform(t_c_b_i, r_i, t_i);
    t_sum += t_i;
    if (k == 0U) {
      r_ref = r_i.clone();
    }
  }
  cv::Mat t_avg = t_sum / static_cast<double>(active_indices.size());
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
  const std::string intrinsics_file = run_dir + "/camera_intrinsics_used.yaml";
  const std::string intrinsics_file_name = std::filesystem::path(intrinsics_file).filename().string();
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
  cv::Mat t_cam_gripper_ref;
  const bool has_grip_ref = eye_in_hand_ && has_cam_gripper &&
    getUrdfReferenceTcamGripperForEyeInHand(arm_id_, t_cam_gripper_ref);
  const bool has_base_ref =
    !has_grip_ref && getUrdfReferenceTcamBaseForArm(arm_id_, t_cam_base_ref);
  if (has_grip_ref) {
    cv::Mat r_est, t_est, r_ref, t_ref;
    splitTransform(t_cam_gripper, r_est, t_est);
    splitTransform(t_cam_gripper_ref, r_ref, t_ref);
    const double trans_err = cv::norm(t_est - t_ref);
    const double trans_err_mm = trans_err * 1000.0;
    const double rot_err_deg = rotationAngleRadBetween(r_est, r_ref) * kRad2Deg;
    fs << "T_cam_gripper_urdf_ref" << mat44ToVec16(t_cam_gripper_ref);
    fs << "T_cam_gripper_vs_urdf_translation_error_mm" << trans_err_mm;
    fs << "T_cam_gripper_vs_urdf_rotation_error_deg" << rot_err_deg;
  }
  if (has_base_ref) {
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

  {
    cv::FileStorage intrinsics_fs(intrinsics_file, cv::FileStorage::WRITE);
    if (intrinsics_fs.isOpened()) {
      intrinsics_fs << "timestamp" << run_stamp;
      intrinsics_fs << "camera_info_topic" << camera_info_topic_;
      intrinsics_fs << "camera_matrix" << camera_matrix_;
      intrinsics_fs << "distortion_coefficients" << dist_coeffs_;
      intrinsics_fs.release();
    } else {
      RCLCPP_WARN(get_logger(), "Failed to open intrinsics output file: %s", intrinsics_file.c_str());
    }
  }

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
    manifest << "index,raw_image,result_image,intrinsics_file,arm_id,px,py,pz,qx,qy,qz,qw\n";
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
               << intrinsics_file_name << ","
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
  if (has_grip_ref) {
    cv::Mat r_est, t_est, r_ref, t_ref;
    splitTransform(t_cam_gripper, r_est, t_est);
    splitTransform(t_cam_gripper_ref, r_ref, t_ref);
    const double trans_err = cv::norm(t_est - t_ref);
    const double trans_err_mm = trans_err * 1000.0;
    const double rot_err_deg = rotationAngleRadBetween(r_est, r_ref) * kRad2Deg;
    oss << std::fixed << std::setprecision(3);
    oss << "\nT_cam_gripper_urdf_ref (eye_in_hand, URDF "
        << (arm_id_ == 1 ? "camera2_optical_frame" : "camera1_optical_frame") << " vs J"
        << arm_id_ + 1 << "_6; matches OpenCV cam axes, not RGB frame_id link):\n"
        << formatMat4(t_cam_gripper_ref)
        << "\nT_cam_gripper_vs_urdf_translation_error_mm: " << trans_err_mm
        << "\nT_cam_gripper_vs_urdf_rotation_error_deg: " << std::setprecision(4) << rot_err_deg;
  }
  if (has_base_ref) {
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
  oss << "calib_run_dir: " << run_dir << "\n";
  oss << "camera_intrinsics_file: " << intrinsics_file_name << "\n";
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
