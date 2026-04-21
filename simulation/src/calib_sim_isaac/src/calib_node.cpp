// calib_node 实现：ArUco 位姿估计、手眼样本采集、OpenCV calibrateHandEye、质量指标与结果落盘。
#include "calib_sim_isaac/calib_node.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>

namespace calib_sim_isaac
{

// 匿名命名空间：手眼算法枚举名、网格/姿态小工具、Aruco 翻转后角点坐标还原等。
namespace
{
constexpr const char * kIsaacJointStatesTopic = "/joint_states";
constexpr const char * kIsaacJointCommandTopic = "/joint_command";

std::vector<std::string> defaultJointNamesForArm(int arm_id)
{
  if (arm_id == 1) {
    return {"J2_1_joint", "J2_2_joint", "J2_3_joint", "J2_4_joint", "J2_5_joint", "J2_6_joint"};
  }
  return {"J1_1_joint", "J1_2_joint", "J1_3_joint", "J1_4_joint", "J1_5_joint", "J1_6_joint"};
}

std::string explainIkReasonZh(const std::string & reason)
{
  if (reason.find("ik_chain_not_ready") != std::string::npos) {
    return "IK链未就绪（URDF/链路配置问题）";
  }
  if (reason.find("no_joint_states_seed") != std::string::npos) {
    return "未收到joint_states，无法提供IK初值";
  }
  if (reason.find("seed_missing_joint:") != std::string::npos) {
    return "joint_states缺少所需关节名";
  }
  if (reason.find("seed_position_oob:") != std::string::npos) {
    return "joint_states位置数组长度不足";
  }
  if (reason.find("kdl_cart_to_jnt_failed_rc=") != std::string::npos) {
    return "KDL逆运动学求解失败（可能超出工作空间/初值不佳）";
  }
  if (reason.find("external_ik_joint_command_stale") != std::string::npos) {
    return "外部IK关节命令超时";
  }
  if (reason.find("and_no_external_ik_joint_command") != std::string::npos) {
    return "且未收到外部IK关节命令";
  }
  if (reason.find("ik_solve_failed_unknown") != std::string::npos) {
    return "IK求解失败（未知原因）";
  }
  return "未分类IK错误";
}
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
  topic_wait_timeout_sec_(60.0),
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
  has_joint_state_(false),
  has_ik_joint_command_(false),
  has_arm_state_(false),
  arm_reached_(false),
  target_index_(0),
  waiting_arm_reached_(false),
  waiting_capture_(false),
  finished_(false),
  arm_reach_retry_count_(0),
  arm_reach_retry_max_(1),
  auto_mode_(true),
  pending_step_(false),
  last_detect_fail_reason_("none"),
  waiting_init_pose_(false),
  last_status_text_(""),
  known_mount_trans_consistency_m_(0.0),
  known_mount_rot_consistency_deg_(0.0),
  use_tf_for_sample_pose_(true),
  use_joint_kinematics_interface_(true),
  enable_legacy_pose_fallback_(false),
  ik_command_max_age_sec_(1.0),
  joint_reach_tolerance_rad_(0.03),
  pose_reach_position_tolerance_m_(0.01),
  pose_reach_rotation_tolerance_deg_(3.0),
  post_reach_settle_sec_(0.0),
  post_reach_settle_require_joint_(true),
  reach_require_joint_and_tf_(false),
  arm_reach_timeout_grace_sec_(0.0),
  tf_base_frame_("base_link"),
  tf_ee_frame_arm0_("J1_6"),
  tf_ee_frame_arm1_("J2_6"),
  known_mount_quality_max_m_(0.20),
  init_reset_burst_count_(6),
  init_reset_burst_period_ms_(50),
  init_delay_ms_after_reset_(500),
  joint_command_burst_count_(5),
  has_last_target_joint_command_(false),
  has_last_target_pose_command_(false),
  ik_ready_(false),
  warned_missing_camera_info_(false),
  warned_missing_robot_pose_(false),
  warned_missing_robot_state_(false),
  warned_no_joint_states_pub_(false),
  warned_no_joint_command_sub_(false),
  warned_domain_mismatch_hint_(false)
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
  topic_wait_timeout_sec_(60.0),
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
  has_joint_state_(false),
  has_ik_joint_command_(false),
  has_arm_state_(false),
  arm_reached_(false),
  target_index_(0),
  waiting_arm_reached_(false),
  waiting_capture_(false),
  finished_(false),
  arm_reach_retry_count_(0),
  arm_reach_retry_max_(1),
  auto_mode_(true),
  pending_step_(false),
  last_detect_fail_reason_("none"),
  waiting_init_pose_(false),
  last_status_text_(""),
  known_mount_trans_consistency_m_(0.0),
  known_mount_rot_consistency_deg_(0.0),
  use_tf_for_sample_pose_(true),
  use_joint_kinematics_interface_(true),
  enable_legacy_pose_fallback_(false),
  ik_command_max_age_sec_(1.0),
  joint_reach_tolerance_rad_(0.03),
  pose_reach_position_tolerance_m_(0.01),
  pose_reach_rotation_tolerance_deg_(3.0),
  post_reach_settle_sec_(0.0),
  post_reach_settle_require_joint_(true),
  reach_require_joint_and_tf_(false),
  arm_reach_timeout_grace_sec_(0.0),
  tf_base_frame_("base_link"),
  tf_ee_frame_arm0_("J1_6"),
  tf_ee_frame_arm1_("J2_6"),
  known_mount_quality_max_m_(0.20),
  init_reset_burst_count_(6),
  init_reset_burst_period_ms_(50),
  init_delay_ms_after_reset_(500),
  joint_command_burst_count_(5),
  has_last_target_joint_command_(false),
  has_last_target_pose_command_(false),
  ik_ready_(false),
  warned_missing_camera_info_(false),
  warned_missing_robot_pose_(false),
  warned_missing_robot_state_(false),
  warned_no_joint_states_pub_(false),
  warned_no_joint_command_sub_(false),
  warned_domain_mismatch_hint_(false)
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
  robot_pose_topic_ = kIsaacJointStatesTopic;
  robot_state_topic_ = cfg.robot_state_topic;
  robot_target_topic_ = kIsaacJointCommandTopic;
  legacy_pose_command_topic_ = "/robot_target_pose";
  kinematics_pose_goal_topic_ = "/nova_target_pose";
  kinematics_joint_command_topic_ = "/ik_joint_command";
  joint_names_arm0_.clear();
  joint_names_arm1_.clear();
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

  // 关节接口：到位等待不得过短（统一模式切换后仅 state_timeout 时易成 2s）；Isaac 建议 ≥20s
  if (use_joint_kinematics_interface_) {
    reached_wait_timeout_sec_ = std::max(reached_wait_timeout_sec_, 20.0);
  }
}

void CalibNode::loadUnifiedModeConfigs()
{
  static const char * kModes[] = {"eth0", "eth1", "eih0", "eih1"};
  mode_configs_.clear();
  for (const char * m : kModes) {
    const std::string prefix(m);
    const bool eih = (prefix.size() >= 3 && prefix[0] == 'e' && prefix[1] == 'i' && prefix[2] == 'h');
    const CalibConfigData def = eih ? defaultCalibConfigEyeInHand() : defaultCalibConfigEyeToHand();
    CalibConfigData cfg = loadCalibConfigPrefixed(*this, prefix, def);

    // Unified 模式兜底：即使 calib_unified.yaml 层级有误，也确保 eih0/eih1 走正确臂与相机。
    if (prefix == "eih0") {
      cfg.arm_id = 0;
      cfg.target_marker_id = 3;
      cfg.image_topic = "/camera1_rgb_sensor/image_raw";
      cfg.depth_topic = "/camera1/depth/image_raw";
      cfg.camera_info_topic = "/camera1_rgb_sensor/camera_info";
    } else if (prefix == "eih1") {
      cfg.arm_id = 1;
      cfg.target_marker_id = 1;
      cfg.image_topic = "/camera2_rgb_sensor/image_raw";
      cfg.depth_topic = "/camera2/depth/image_raw";
      cfg.camera_info_topic = "/camera2_rgb_sensor/camera_info";
    }

    mode_configs_[prefix] = std::move(cfg);
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
  rebuildIkChainForActiveArm();

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
  reach_settle_armed_ = false;

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
  camera_frame_id_.clear();
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
  node_start_time_ = now();
  renewCameraSubscriptions();

  this->declare_parameter("use_joint_kinematics_interface", true);
  this->declare_parameter("enable_legacy_pose_fallback", false);
  this->declare_parameter("ik_command_max_age_sec", 1.0);
  this->declare_parameter("joint_reach_tolerance_rad", 0.03);
  this->declare_parameter("pose_reach_position_tolerance_m", 0.01);
  this->declare_parameter("pose_reach_rotation_tolerance_deg", 3.0);
  this->declare_parameter("post_reach_settle_sec", 1.2);
  this->declare_parameter("post_reach_settle_require_joint", true);
  this->declare_parameter("reach_require_joint_and_tf", false);
  this->declare_parameter("arm_reach_timeout_grace_sec", 2.0);
  this->declare_parameter("topic_wait_timeout_sec", 60.0);
  this->declare_parameter("joint_command_burst_count", 5);
  this->declare_parameter("legacy_pose_command_topic", std::string("/robot_target_pose"));
  this->declare_parameter("kinematics_pose_goal_topic", std::string("/nova_target_pose"));
  this->declare_parameter("kinematics_joint_command_topic", std::string("/ik_joint_command"));
  this->declare_parameter("ik_urdf_path", std::string("/home/hs/testCode/simulation/src/nova_sim/urdf/nova_robot_position.urdf"));
  this->declare_parameter("joint_names_arm0", std::vector<std::string>{
    "J1_1_joint", "J1_2_joint", "J1_3_joint", "J1_4_joint", "J1_5_joint", "J1_6_joint"});
  this->declare_parameter("joint_names_arm1", std::vector<std::string>{
    "J2_1_joint", "J2_2_joint", "J2_3_joint", "J2_4_joint", "J2_5_joint", "J2_6_joint"});

  use_joint_kinematics_interface_ = true;
  enable_legacy_pose_fallback_ = false;
  if (use_joint_kinematics_interface_) {
    reached_wait_timeout_sec_ = std::max(reached_wait_timeout_sec_, 20.0);
    capture_wait_timeout_sec_ = std::max(capture_wait_timeout_sec_, 12.0);
  }
  ik_command_max_age_sec_ = this->get_parameter("ik_command_max_age_sec").as_double();
  joint_reach_tolerance_rad_ = this->get_parameter("joint_reach_tolerance_rad").as_double();
  pose_reach_position_tolerance_m_ = this->get_parameter("pose_reach_position_tolerance_m").as_double();
  pose_reach_rotation_tolerance_deg_ = this->get_parameter("pose_reach_rotation_tolerance_deg").as_double();
  post_reach_settle_sec_ = std::max(0.0, this->get_parameter("post_reach_settle_sec").as_double());
  post_reach_settle_require_joint_ = this->get_parameter("post_reach_settle_require_joint").as_bool();
  reach_require_joint_and_tf_ = this->get_parameter("reach_require_joint_and_tf").as_bool();
  arm_reach_timeout_grace_sec_ =
    std::max(0.0, this->get_parameter("arm_reach_timeout_grace_sec").as_double());
  topic_wait_timeout_sec_ = this->get_parameter("topic_wait_timeout_sec").as_double();
  joint_command_burst_count_ = std::max(1, static_cast<int>(this->get_parameter("joint_command_burst_count").as_int()));
  legacy_pose_command_topic_ = this->get_parameter("legacy_pose_command_topic").as_string();
  kinematics_pose_goal_topic_ = this->get_parameter("kinematics_pose_goal_topic").as_string();
  kinematics_joint_command_topic_ = this->get_parameter("kinematics_joint_command_topic").as_string();
  ik_urdf_path_ = this->get_parameter("ik_urdf_path").as_string();
  joint_names_arm0_ = this->get_parameter("joint_names_arm0").as_string_array();
  joint_names_arm1_ = this->get_parameter("joint_names_arm1").as_string_array();
  loadIkModelFromUrdf();
  rebuildIkChainForActiveArm();

  if (use_joint_kinematics_interface_) {
    RCLCPP_INFO(
      get_logger(),
      "arm_reach_timeout: grace=%.2fs + reached_wait=%.2fs => skip_if_elapsed>%.2fs",
      arm_reach_timeout_grace_sec_, reached_wait_timeout_sec_,
      arm_reach_timeout_grace_sec_ + reached_wait_timeout_sec_);
  }

  if (!use_tf_for_sample_pose_) {
    arm_pose_sub_ = create_subscription<calib_sim_isaac::msg::ArmPose>(
      robot_pose_topic_, 20, std::bind(&CalibNode::armPoseCallback, this, std::placeholders::_1));
  }
  if (use_joint_kinematics_interface_) {
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      robot_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CalibNode::jointStateCallback, this, std::placeholders::_1));
    ik_joint_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      kinematics_joint_command_topic_, 20,
      std::bind(&CalibNode::ikJointCommandCallback, this, std::placeholders::_1));
    try {
      joint_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(robot_target_topic_, 10);
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(
        get_logger(),
        "create joint command publisher failed topic=%s: %s",
        robot_target_topic_.c_str(), e.what());
      publishLog(
        std::string("[ERROR] create_publisher failed for ") + robot_target_topic_ +
        " ; node continues without joint_command output");
    }
    try {
      kinematics_pose_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        kinematics_pose_goal_topic_, 10);
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(
        get_logger(),
        "create kinematics pose publisher failed topic=%s: %s",
        kinematics_pose_goal_topic_.c_str(), e.what());
      publishLog(
        std::string("[ERROR] create_publisher failed for ") + kinematics_pose_goal_topic_ +
        " ; node continues without pose_goal output");
    }
    if (enable_legacy_pose_fallback_) {
      try {
        target_pose_pub_ = create_publisher<calib_sim_isaac::msg::ArmPose>(legacy_pose_command_topic_, 10);
      } catch (const rclcpp::exceptions::RCLError & e) {
        RCLCPP_ERROR(
          get_logger(),
          "create legacy pose publisher failed topic=%s: %s",
          legacy_pose_command_topic_.c_str(), e.what());
        publishLog(
          std::string("[ERROR] create_publisher failed for ") + legacy_pose_command_topic_ +
          " ; legacy fallback disabled automatically");
        enable_legacy_pose_fallback_ = false;
      }
    }
  } else {
    try {
      target_pose_pub_ = create_publisher<calib_sim_isaac::msg::ArmPose>(robot_target_topic_, 10);
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(
        get_logger(),
        "create pose command publisher failed topic=%s: %s",
        robot_target_topic_.c_str(), e.what());
      publishLog(
        std::string("[ERROR] create_publisher failed for ") + robot_target_topic_ +
        " ; node continues without pose command output");
    }
  }
  if (!use_joint_kinematics_interface_) {
    arm_state_sub_ = create_subscription<std_msgs::msg::Bool>(
      robot_state_topic_, 20, std::bind(&CalibNode::armStateCallback, this, std::placeholders::_1));
  }
  control_sub_ = create_subscription<std_msgs::msg::String>(
    "/calib_sim_isaac/control", 20, std::bind(&CalibNode::controlCallback, this, std::placeholders::_1));
  this->declare_parameter("nova_all_joints_reset_topic", std::string("/nova_sim/reset_all_joints"));
  nova_all_joints_reset_topic_ = this->get_parameter("nova_all_joints_reset_topic").as_string();
  nova_all_joints_reset_pub_ =
    create_publisher<std_msgs::msg::Empty>(nova_all_joints_reset_topic_, 10);
  status_pub_ = create_publisher<std_msgs::msg::String>("/calib_sim_isaac/status", 10);
  log_pub_ = create_publisher<std_msgs::msg::String>("/calib_sim_isaac/log", 30);
  result_text_pub_ = create_publisher<std_msgs::msg::String>("/calib_sim_isaac/result_text", 10);
  raw_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/calib_sim_isaac/raw_image", 10);
  result_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/calib_sim_isaac/result_image", 10);
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&CalibNode::controlTimerCallback, this));
  if (use_tf_for_sample_pose_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  }
  logTopicConnectivityOnce();
}

void CalibNode::logTopicConnectivityOnce()
{
  const char * domain_env = std::getenv("ROS_DOMAIN_ID");
  const std::string domain = domain_env ? domain_env : "(unset->default)";
  const auto js_pub = this->count_publishers(robot_pose_topic_);
  const auto jc_sub = this->count_subscribers(robot_target_topic_);
  std::ostringstream oss;
  oss << "connectivity_check domain=" << domain
      << " joint_states_topic=" << robot_pose_topic_ << " publishers=" << js_pub
      << " joint_command_topic=" << robot_target_topic_ << " subscribers=" << jc_sub;
  publishLog(oss.str());
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
  last_target_pose_command_ = init_pose_pending_;
  has_last_target_pose_command_ = true;
  if (use_joint_kinematics_interface_) {
    publishKinematicsPoseGoal(init_pose_pending_);
    sensor_msgs::msg::JointState joint_cmd;
    if (tryBuildJointCommandFromTargetPose(init_pose_pending_, joint_cmd) && joint_command_pub_) {
      for (int i = 0; i < joint_command_burst_count_; ++i) {
        joint_command_pub_->publish(joint_cmd);
      }
      last_target_joint_command_ = joint_cmd;
      has_last_target_joint_command_ = true;
      publishLog(
        std::string("init_target_joint burst=") + std::to_string(joint_command_burst_count_) +
        " " + formatJointStateCompact(joint_cmd));
    } else if (enable_legacy_pose_fallback_ && target_pose_pub_) {
      target_pose_pub_->publish(init_pose_pending_);
      publishLog(
        std::string("[WARN] init pose uses legacy fallback; ik_reason=") + last_ik_failure_reason_);
      publishLog(
        std::string("[WARN] init IK解释: ") + explainIkReasonZh(last_ik_failure_reason_));
    } else {
      publishLog(
        std::string("[WARN] init pose not sent; ik_reason=") + last_ik_failure_reason_ +
        " and legacy fallback disabled");
      publishLog(
        std::string("[WARN] init IK解释: ") + explainIkReasonZh(last_ik_failure_reason_));
    }
  } else if (target_pose_pub_) {
    target_pose_pub_->publish(init_pose_pending_);
  }
  waiting_arm_reached_ = true;
  reach_settle_armed_ = false;
  reach_log_joint_latched_ = false;
  reach_log_pose_latched_ = false;
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
  camera_frame_id_ = msg->header.frame_id;
  has_camera_info_ = true;
}

bool CalibNode::tryGetLiveReferenceTcamBase(cv::Mat & out_t_cam_base_ref) const
{
  if (!tf_buffer_ || camera_frame_id_.empty()) {
    return false;
  }
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(tf_base_frame_, camera_frame_id_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }

  const auto & t = tf_msg.transform.translation;
  const auto & q = tf_msg.transform.rotation;
  cv::Mat r = quatToRot(q.x, q.y, q.z, q.w);
  cv::Mat tr = (cv::Mat_<double>(3, 1) << t.x, t.y, t.z);
  out_t_cam_base_ref = makeTransform(r, tr);
  return true;
}

void CalibNode::armPoseCallback(const calib_sim_isaac::msg::ArmPose::ConstSharedPtr msg)
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

void CalibNode::jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  last_joint_state_ = *msg;
  has_joint_state_ = true;
}

void CalibNode::ikJointCommandCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  last_ik_joint_command_ = *msg;
  has_ik_joint_command_ = true;
  last_ik_joint_command_time_ = now();
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

  const double startup_elapsed_sec = (now() - node_start_time_).seconds();
  if (startup_elapsed_sec > topic_wait_timeout_sec_) {
    if (!has_camera_info_ && !warned_missing_camera_info_) {
      warned_missing_camera_info_ = true;
      publishLog(
        std::string("[WARN] topic timeout: no camera_info on ") + camera_info_topic_ +
        " for " + std::to_string(topic_wait_timeout_sec_) + "s");
    }
    if (!use_tf_for_sample_pose_ && !has_arm_pose_ && !has_joint_state_ && !warned_missing_robot_pose_) {
      warned_missing_robot_pose_ = true;
      publishLog(
        std::string("[WARN] topic timeout: no robot pose on ") + robot_pose_topic_ +
        " for " + std::to_string(topic_wait_timeout_sec_) + "s");
    }
    if (!use_joint_kinematics_interface_ && !has_arm_state_ && !warned_missing_robot_state_) {
      warned_missing_robot_state_ = true;
      publishLog(
        std::string("[WARN] topic timeout: no arm state on ") + robot_state_topic_ +
        " for " + std::to_string(topic_wait_timeout_sec_) + "s");
    }
  }

  if (use_joint_kinematics_interface_) {
    const auto js_pub = this->count_publishers(robot_pose_topic_);
    if (js_pub == 0 && !warned_no_joint_states_pub_) {
      warned_no_joint_states_pub_ = true;
      publishLog(
        std::string("[WARN] no publishers on ") + robot_pose_topic_ +
        " ; current_joint will stay none");
    }
    const auto jc_sub = this->count_subscribers(robot_target_topic_);
    if (jc_sub == 0 && !warned_no_joint_command_sub_) {
      warned_no_joint_command_sub_ = true;
      publishLog(
        std::string("[WARN] no subscribers on ") + robot_target_topic_ +
        " ; commands may not move robot");
    }
    if ((js_pub == 0 || jc_sub == 0) && !warned_domain_mismatch_hint_) {
      warned_domain_mismatch_hint_ = true;
      const char * domain_env = std::getenv("ROS_DOMAIN_ID");
      const std::string domain = domain_env ? domain_env : "(unset->default)";
      publishLog(
        std::string("[HINT] check ROS_DOMAIN_ID consistency. current domain=") + domain);
    }
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

  if (!waiting_arm_reached_ && !waiting_capture_) {
    if (!auto_mode_ && !pending_step_) {
      publishStatus("paused_wait_step");
      return;
    }
    if (!publishTargetPose(target_index_)) {
      publishStatus("target_publish_failed_" + std::to_string(target_index_));
      if (pending_step_) {
        pending_step_ = false;
      }
      return;
    }
    target_sent_time_ = now();
    waiting_arm_reached_ = true;
    arm_reach_retry_count_ = 0;
    reach_settle_armed_ = false;
    reach_log_joint_latched_ = false;
    reach_log_pose_latched_ = false;
    publishStatus("moving_to_target_" + std::to_string(target_index_));
    return;
  }

  bool raw_reached = false;
  bool joint_reached = false;
  bool pose_reached = false;
  bool joint_ok_check = false;
  bool pose_ok_check = false;
  if (waiting_arm_reached_) {
    joint_ok_check = use_joint_kinematics_interface_ && has_last_target_joint_command_;
    pose_ok_check = use_tf_for_sample_pose_ && has_last_target_pose_command_;
    if (joint_ok_check) {
      double max_abs_err = 0.0;
      joint_reached = isJointTargetReached(&max_abs_err);
      if (joint_reached && !reach_log_joint_latched_) {
        reach_log_joint_latched_ = true;
        std::ostringstream oss;
        oss << "joint_target_reached index=" << target_index_
            << " max_abs_err_rad=" << max_abs_err;
        publishLog(oss.str());
      } else if (!joint_reached) {
        reach_log_joint_latched_ = false;
      }
    }
    if (pose_ok_check) {
      double pos_err = 0.0;
      double rot_err = 0.0;
      pose_reached = isPoseTargetReachedFromTf(&pos_err, &rot_err);
      if (pose_reached && !reach_log_pose_latched_) {
        reach_log_pose_latched_ = true;
        std::ostringstream oss;
        oss << "pose_target_reached index=" << target_index_
            << " pos_err_m=" << std::fixed << std::setprecision(4) << pos_err
            << " rot_err_deg=" << std::fixed << std::setprecision(4) << rot_err;
        publishLog(oss.str());
      } else if (!pose_reached) {
        reach_log_pose_latched_ = false;
      }
    }
    // 有 TF 目标时默认以末端 TF 为准（避免 joint_states 与 IK/模型不一致时永远等不到「双满足」而超时）；
    // 仍可用 reach_require_joint_and_tf 强制关节+TF 同时满足。
    if (joint_ok_check && pose_ok_check && reach_require_joint_and_tf_) {
      raw_reached = joint_reached && pose_reached;
    } else if (pose_ok_check) {
      raw_reached = pose_reached;
    } else if (joint_ok_check) {
      raw_reached = joint_reached;
    } else {
      raw_reached = false;
    }
    if (!raw_reached && !use_joint_kinematics_interface_ && has_arm_state_ && arm_reached_) {
      const auto state_age = now() - last_arm_state_time_;
      raw_reached = state_age.seconds() <= state_timeout_sec_;
    }
  }

  const bool settle_hold_ok =
    raw_reached &&
    (!post_reach_settle_require_joint_ || !joint_ok_check || joint_reached);

  bool reached_now = false;
  if (settle_hold_ok) {
    if (post_reach_settle_sec_ <= 0.0) {
      reached_now = true;
    } else {
      if (!reach_settle_armed_) {
        reach_settle_armed_ = true;
        reach_settle_start_ = now();
        publishLog(
          std::string("reach_settle_start index=") + std::to_string(target_index_) +
          " duration_sec=" + std::to_string(post_reach_settle_sec_) +
          " require_joint=" + (post_reach_settle_require_joint_ ? "true" : "false"));
      } else if ((now() - reach_settle_start_).seconds() >= post_reach_settle_sec_) {
        reached_now = true;
      }
    }
  } else {
    reach_settle_armed_ = false;
  }

  if (waiting_arm_reached_ && reached_now) {
    reach_settle_armed_ = false;
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

  // Fallback: avoid deadlock when robot_reached signal is missing.
  const double arm_wait_elapsed = (now() - target_sent_time_).seconds();
  const double arm_reach_budget =
    arm_reach_timeout_grace_sec_ + reached_wait_timeout_sec_;
  if (waiting_arm_reached_ && arm_wait_elapsed > arm_reach_budget) {
    if (use_tf_for_sample_pose_ && has_last_target_pose_command_) {
      double pos_err = 0.0;
      double rot_err = 0.0;
      if (isPoseTargetReachedFromTf(&pos_err, &rot_err)) {
        waiting_arm_reached_ = false;
        reach_settle_armed_ = false;
        waiting_capture_ = true;
        capture_start_time_ = now();
        std::ostringstream reach_oss;
        reach_oss << "[WARN] reach_timeout_but_pose_ok_continue_capture index=" << target_index_
                  << " pos_err_m=" << std::fixed << std::setprecision(4) << pos_err
                  << " rot_err_deg=" << std::fixed << std::setprecision(4) << rot_err;
        publishLog(reach_oss.str());
        publishStatus("capturing_target_" + std::to_string(target_index_));
        return;
      }
    }
    waiting_arm_reached_ = false;
    if (waiting_init_pose_) {
      waiting_init_pose_ = false;
      publishStatus("initialized_wait_step_timeout");
    } else {
      if (arm_reach_retry_count_ < arm_reach_retry_max_) {
        const int retry_no = arm_reach_retry_count_ + 1;
        if (publishTargetPose(target_index_)) {
          arm_reach_retry_count_ = retry_no;
          target_sent_time_ = now();
          waiting_arm_reached_ = true;
          waiting_capture_ = false;
          reach_settle_armed_ = false;
          reach_log_joint_latched_ = false;
          reach_log_pose_latched_ = false;
          std::ostringstream retry_oss;
          retry_oss << "[WARN] arm_reach_timeout_retry_target_" << target_index_
                    << " retry=" << retry_no << "/" << arm_reach_retry_max_
                    << " arm_wait_elapsed_sec=" << std::fixed << std::setprecision(2) << arm_wait_elapsed
                    << " budget_sec=" << arm_reach_budget;
          publishStatus(retry_oss.str());
          publishLog(retry_oss.str());
          return;
        }
      }
      waiting_capture_ = false;
      std::ostringstream oss;
      std::ostringstream target_pose_ss;
      if (has_last_target_pose_command_) {
        target_pose_ss << std::fixed << std::setprecision(4)
                       << "pos=(" << last_target_pose_command_.pose.pose.position.x << ","
                       << last_target_pose_command_.pose.pose.position.y << ","
                       << last_target_pose_command_.pose.pose.position.z << ")"
                       << " quat=(" << last_target_pose_command_.pose.pose.orientation.x << ","
                       << last_target_pose_command_.pose.pose.orientation.y << ","
                       << last_target_pose_command_.pose.pose.orientation.z << ","
                       << last_target_pose_command_.pose.pose.orientation.w << ")";
      } else {
        target_pose_ss << "none";
      }
      oss << "[ERROR] arm_reach_timeout_skip_target_" << target_index_
          << "\n  arm_wait_elapsed_sec=" << std::fixed << std::setprecision(2) << arm_wait_elapsed
          << " budget_sec=" << arm_reach_budget
          << " (grace=" << arm_reach_timeout_grace_sec_ << " + reached_wait=" << reached_wait_timeout_sec_
          << ")"
          << "\n  target_pose=" << target_pose_ss.str()
          << "\n  current_pose=" << formatCurrentPoseCompact()
          << "\n  target_joint=" << (has_last_target_joint_command_ ? formatJointStateCompact(last_target_joint_command_) : "none")
          << "\n  current_joint=" << (has_joint_state_ ? formatJointStateCompact(last_joint_state_) : "none");
      if (use_tf_for_sample_pose_ && has_last_target_pose_command_) {
        double pos_err = 0.0;
        double rot_err = 0.0;
        const bool pose_ok = isPoseTargetReachedFromTf(&pos_err, &rot_err);
        oss << "\n  pose_err: pos_m=" << std::fixed << std::setprecision(4) << pos_err
            << " rot_deg=" << std::fixed << std::setprecision(4) << rot_err
            << " tol_pos_m=" << pose_reach_position_tolerance_m_
            << " tol_rot_deg=" << pose_reach_rotation_tolerance_deg_
            << " reached=" << (pose_ok ? "true" : "false");
      }
      publishStatus(oss.str());
      publishLog(oss.str());
      ++target_index_;
      arm_reach_retry_count_ = 0;
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
    ++target_index_;
    arm_reach_retry_count_ = 0;
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
    arm_reach_retry_count_ = 0;
    waiting_init_pose_ = false;
    last_status_text_.clear();
    reach_settle_armed_ = false;
    has_last_target_joint_command_ = false;
    has_last_target_pose_command_ = false;
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
    // Default convention: marker id follows arm id (arm0->id0, arm1->id1).
    marker_id_ = new_arm;
    applyTargetPosesForCurrentArm();
    rebuildIkChainForActiveArm();
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
    // Explicitly send a full zero joint command so all joints return to zero pose.
    if (joint_command_pub_) {
      sensor_msgs::msg::JointState zero_cmd;
      zero_cmd.header.stamp = now();
      if (has_joint_state_ && !last_joint_state_.name.empty()) {
        zero_cmd.name = last_joint_state_.name;
      } else {
        // Fallback when joint_states is not ready yet.
        zero_cmd.name = {
          "J1_1_joint", "J1_2_joint", "J1_3_joint", "J1_4_joint", "J1_5_joint", "J1_6_joint",
          "J1_7_joint", "J1_8_joint",
          "J2_1_joint", "J2_2_joint", "J2_3_joint", "J2_4_joint", "J2_5_joint", "J2_6_joint",
          "J2_7_joint", "J2_8_joint",
          "J3_1_joint", "J3_2_joint", "J3_3_joint", "J3_4_joint", "J3_5_joint", "J3_6_joint",
          "J4_1_joint", "J4_2_joint", "J4_3_joint", "J4_4_joint", "J4_5_joint", "J4_6_joint"
        };
      }
      zero_cmd.position.assign(zero_cmd.name.size(), 0.0);
      for (int i = 0; i < init_reset_burst_count_; ++i) {
        zero_cmd.header.stamp = now();
        joint_command_pub_->publish(zero_cmd);
      }
      last_target_joint_command_ = zero_cmd;
      has_last_target_joint_command_ = true;
      publishLog(
        "cmd:init publish_zero_joint_command n=" + std::to_string(init_reset_burst_count_) +
        " joints=" + std::to_string(zero_cmd.name.size()));
    } else {
      publishLog("cmd:init skip_zero_joint_command (joint_command_pub unavailable)");
    }
    republishLastCameraImagesToUi();
    waiting_arm_reached_ = false;
    waiting_capture_ = false;
    waiting_init_pose_ = false;
    publishStatus("initialized_zero_pose_hold");
    publishLog("cmd:init done; holding zero joint pose (no auto move to initial pose)");
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
  calib_sim_isaac::msg::ArmPose * out_manifest_pose)
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
  calib_sim_isaac::msg::ArmPose manifest_pose;
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
  if (!log_pub_) {
    RCLCPP_WARN(get_logger(), "%s", text.c_str());
    return;
  }
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

void CalibNode::publishKinematicsPoseGoal(const calib_sim_isaac::msg::ArmPose & target_pose)
{
  if (!kinematics_pose_goal_pub_) {
    return;
  }
  geometry_msgs::msg::PoseStamped pose_goal = target_pose.pose;
  pose_goal.header.stamp = now();
  kinematics_pose_goal_pub_->publish(pose_goal);
}

bool CalibNode::loadIkModelFromUrdf()
{
  ik_ready_ = false;
  if (ik_urdf_path_.empty()) {
    publishLog("[WARN] IK disabled: ik_urdf_path is empty");
    return false;
  }
  std::ifstream ifs(ik_urdf_path_);
  if (!ifs.is_open()) {
    publishLog(std::string("[WARN] IK disabled: cannot open urdf file: ") + ik_urdf_path_);
    return false;
  }
  std::stringstream buffer;
  buffer << ifs.rdbuf();
  if (!kdl_parser::treeFromString(buffer.str(), kdl_tree_)) {
    publishLog(std::string("[WARN] IK disabled: kdl_parser failed for urdf: ") + ik_urdf_path_);
    return false;
  }
  return true;
}

bool CalibNode::rebuildIkChainForActiveArm()
{
  ik_ready_ = false;
  const std::string & ee = (arm_id_ == 1) ? tf_ee_frame_arm1_ : tf_ee_frame_arm0_;
  active_joint_names_ = (arm_id_ == 1) ? joint_names_arm1_ : joint_names_arm0_;
  if (active_joint_names_.empty()) {
    active_joint_names_ = defaultJointNamesForArm(arm_id_);
    publishLog(
      std::string("[WARN] joint_names for arm_id=") + std::to_string(arm_id_) +
      " is empty, fallback to default 6-axis names");
  }
  if (kdl_tree_.getNrOfSegments() == 0 && !loadIkModelFromUrdf()) {
    return false;
  }
  if (!kdl_tree_.getChain(tf_base_frame_, ee, kdl_chain_)) {
    publishLog(
      std::string("[WARN] IK disabled: failed to build KDL chain ") +
      tf_base_frame_ + " -> " + ee);
    return false;
  }
  if (kdl_chain_.getNrOfJoints() != active_joint_names_.size()) {
    std::ostringstream oss;
    oss << "[WARN] IK chain joint count mismatch: chain=" << kdl_chain_.getNrOfJoints()
        << " configured=" << active_joint_names_.size();
    publishLog(oss.str());
    return false;
  }
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
  ik_ready_ = true;
  std::ostringstream oss;
  oss << "IK ready: chain " << tf_base_frame_ << " -> " << ee
      << " joints=" << active_joint_names_.size();
  publishLog(oss.str());
  return true;
}

bool CalibNode::solveIkForTargetPose(
  const calib_sim_isaac::msg::ArmPose & target_pose,
  sensor_msgs::msg::JointState & out_joint_cmd)
{
  const auto t0 = std::chrono::steady_clock::now();
  last_ik_failure_reason_.clear();
  if (!ik_ready_ && !rebuildIkChainForActiveArm()) {
    last_ik_failure_reason_ = "ik_chain_not_ready";
    return false;
  }

  KDL::JntArray seed(kdl_chain_.getNrOfJoints());
  if (has_joint_state_) {
    for (std::size_t i = 0; i < active_joint_names_.size(); ++i) {
      auto it = std::find(last_joint_state_.name.begin(), last_joint_state_.name.end(), active_joint_names_[i]);
      if (it == last_joint_state_.name.end()) {
        last_ik_failure_reason_ = std::string("seed_missing_joint:") + active_joint_names_[i];
        return false;
      }
      const std::size_t idx = static_cast<std::size_t>(std::distance(last_joint_state_.name.begin(), it));
      if (idx >= last_joint_state_.position.size()) {
        last_ik_failure_reason_ = std::string("seed_position_oob:") + active_joint_names_[i];
        return false;
      }
      seed(i) = last_joint_state_.position[idx];
    }
  } else {
    for (unsigned int i = 0; i < seed.rows(); ++i) {
      seed(i) = 0.0;
    }
    publishLog("[WARN] IK seed fallback: no joint_states, use zero seed.");
  }

  const auto & p = target_pose.pose.pose.position;
  const auto & q = target_pose.pose.pose.orientation;
  KDL::Frame target(
    KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w),
    KDL::Vector(p.x, p.y, p.z));

  KDL::JntArray result(kdl_chain_.getNrOfJoints());
  const int rc = ik_solver_->CartToJnt(seed, target, result);
  if (rc < 0) {
    last_ik_failure_reason_ = std::string("kdl_cart_to_jnt_failed_rc=") + std::to_string(rc);
    return false;
  }

  out_joint_cmd.header.stamp = now();
  out_joint_cmd.header.stamp.sec = 0;
  out_joint_cmd.header.stamp.nanosec = 0;
  out_joint_cmd.header.frame_id.clear();
  out_joint_cmd.name = active_joint_names_;
  out_joint_cmd.position.resize(active_joint_names_.size());
  for (std::size_t i = 0; i < active_joint_names_.size(); ++i) {
    out_joint_cmd.position[i] = result(i);
  }
  out_joint_cmd.velocity.clear();
  out_joint_cmd.effort.clear();
  const auto t1 = std::chrono::steady_clock::now();
  const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  std::ostringstream oss;
  oss << "ik_solve_ok arm_id=" << arm_id_ << " dt_ms=" << dt_ms;
  publishLog(oss.str());
  return true;
}

bool CalibNode::tryBuildJointCommandFromTargetPose(
  const calib_sim_isaac::msg::ArmPose & target_pose,
  sensor_msgs::msg::JointState & out_joint_cmd) const
{
  if (const_cast<CalibNode *>(this)->solveIkForTargetPose(target_pose, out_joint_cmd)) {
    return true;
  }
  if (last_ik_failure_reason_.empty()) {
    last_ik_failure_reason_ = "ik_solve_failed_unknown";
  }
  if (!has_ik_joint_command_) {
    last_ik_failure_reason_ += "|and_no_external_ik_joint_command";
    return false;
  }
  if ((now() - last_ik_joint_command_time_).seconds() > ik_command_max_age_sec_) {
    last_ik_failure_reason_ += "|external_ik_joint_command_stale";
    return false;
  }
  out_joint_cmd = last_ik_joint_command_;
  out_joint_cmd.header.stamp = now();
  std::vector<std::string> configured_names = (arm_id_ == 1) ? joint_names_arm1_ : joint_names_arm0_;
  if (configured_names.empty()) {
    configured_names = defaultJointNamesForArm(arm_id_);
  }
  if (!configured_names.empty()) {
    std::vector<double> mapped_positions;
    mapped_positions.reserve(configured_names.size());
    for (const auto & name : configured_names) {
      auto it = std::find(out_joint_cmd.name.begin(), out_joint_cmd.name.end(), name);
      if (it == out_joint_cmd.name.end()) {
        return false;
      }
      const auto idx = static_cast<std::size_t>(std::distance(out_joint_cmd.name.begin(), it));
      if (idx >= out_joint_cmd.position.size()) {
        return false;
      }
      mapped_positions.push_back(out_joint_cmd.position[idx]);
    }
    out_joint_cmd.name = configured_names;
    out_joint_cmd.position = mapped_positions;
    out_joint_cmd.velocity.clear();
    out_joint_cmd.effort.clear();
  }
  return true;
}

bool CalibNode::isJointTargetReached(double * out_max_abs_err_rad) const
{
  if (!has_joint_state_ || !has_last_target_joint_command_) {
    return false;
  }
  if (last_target_joint_command_.name.empty() || last_target_joint_command_.position.empty()) {
    return false;
  }
  double max_err = 0.0;
  for (std::size_t i = 0; i < last_target_joint_command_.name.size(); ++i) {
    const auto & joint_name = last_target_joint_command_.name[i];
    auto it = std::find(last_joint_state_.name.begin(), last_joint_state_.name.end(), joint_name);
    if (it == last_joint_state_.name.end()) {
      return false;
    }
    const std::size_t cur_idx = static_cast<std::size_t>(std::distance(last_joint_state_.name.begin(), it));
    if (cur_idx >= last_joint_state_.position.size() || i >= last_target_joint_command_.position.size()) {
      return false;
    }
    max_err = std::max(max_err, std::abs(last_joint_state_.position[cur_idx] - last_target_joint_command_.position[i]));
  }
  if (out_max_abs_err_rad) {
    *out_max_abs_err_rad = max_err;
  }
  return max_err <= joint_reach_tolerance_rad_;
}

bool CalibNode::isPoseTargetReachedFromTf(double * out_pos_err_m, double * out_rot_err_deg) const
{
  if (!use_tf_for_sample_pose_ || !tf_buffer_ || !has_last_target_pose_command_) {
    return false;
  }
  const std::string & ee = (arm_id_ == 1) ? tf_ee_frame_arm1_ : tf_ee_frame_arm0_;
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(tf_base_frame_, ee, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }

  const auto & tp = last_target_pose_command_.pose.pose.position;
  const auto & tq = last_target_pose_command_.pose.pose.orientation;
  const auto & cp = tf_msg.transform.translation;
  const auto & cq = tf_msg.transform.rotation;

  const double dx = cp.x - tp.x;
  const double dy = cp.y - tp.y;
  const double dz = cp.z - tp.z;
  const double pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);

  double dot = tq.x * cq.x + tq.y * cq.y + tq.z * cq.z + tq.w * cq.w;
  dot = std::max(-1.0, std::min(1.0, std::abs(dot)));
  const double rot_err_deg = 2.0 * std::acos(dot) * kRad2Deg;

  if (out_pos_err_m) {
    *out_pos_err_m = pos_err;
  }
  if (out_rot_err_deg) {
    *out_rot_err_deg = rot_err_deg;
  }
  return pos_err <= pose_reach_position_tolerance_m_ &&
         rot_err_deg <= pose_reach_rotation_tolerance_deg_;
}

std::string CalibNode::formatJointStateCompact(const sensor_msgs::msg::JointState & js) const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  oss << "[";
  const std::size_t n = std::min(js.name.size(), js.position.size());
  for (std::size_t i = 0; i < n; ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << js.name[i] << "=" << js.position[i];
  }
  oss << "]";
  return oss.str();
}

std::string CalibNode::formatCurrentPoseCompact() const
{
  if (use_tf_for_sample_pose_ && tf_buffer_) {
    const std::string & ee = (arm_id_ == 1) ? tf_ee_frame_arm1_ : tf_ee_frame_arm0_;
    try {
      const auto tf_msg = tf_buffer_->lookupTransform(tf_base_frame_, ee, tf2::TimePointZero);
      const auto & p = tf_msg.transform.translation;
      const auto & q = tf_msg.transform.rotation;
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(4);
      oss << "pos=(" << p.x << "," << p.y << "," << p.z << ")"
          << " quat=(" << q.x << "," << q.y << "," << q.z << "," << q.w << ")";
      return oss.str();
    } catch (const tf2::TransformException &) {
    }
  }
  if (has_arm_pose_) {
    const auto & p = last_arm_pose_.pose.pose.position;
    const auto & q = last_arm_pose_.pose.pose.orientation;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "pos=(" << p.x << "," << p.y << "," << p.z << ")"
        << " quat=(" << q.x << "," << q.y << "," << q.z << "," << q.w << ")";
    return oss.str();
  }
  return "none";
}

bool CalibNode::publishTargetPose(std::size_t idx)
{
  const std::size_t base = idx * kPoseDims;
  calib_sim_isaac::msg::ArmPose cmd;
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
  last_target_pose_command_ = cmd;
  has_last_target_pose_command_ = true;
  bool sent = false;
  if (use_joint_kinematics_interface_) {
    publishKinematicsPoseGoal(cmd);
    sensor_msgs::msg::JointState joint_cmd;
    if (tryBuildJointCommandFromTargetPose(cmd, joint_cmd) && joint_command_pub_) {
      for (int i = 0; i < joint_command_burst_count_; ++i) {
        joint_command_pub_->publish(joint_cmd);
      }
      last_target_joint_command_ = joint_cmd;
      has_last_target_joint_command_ = true;
      RCLCPP_INFO(get_logger(), "Publish joint command index=%zu", idx);
      sent = true;
      std::ostringstream js_log;
      js_log << "target_joint index=" << idx << " arm_id=" << arm_id_
             << " burst=" << joint_command_burst_count_
             << " joint=" << formatJointStateCompact(joint_cmd);
      publishLog(js_log.str());
    } else {
      RCLCPP_WARN(
        get_logger(), "IK joint command unavailable, fallback to pose command. index=%zu reason=%s",
        idx, last_ik_failure_reason_.c_str());
      publishLog(
        std::string("ik_solve_fail index=") + std::to_string(idx) +
        " reason=" + last_ik_failure_reason_);
      publishLog(
        std::string("ik_solve_fail_zh index=") + std::to_string(idx) +
        " explain=" + explainIkReasonZh(last_ik_failure_reason_));
      if (enable_legacy_pose_fallback_ && target_pose_pub_) {
        target_pose_pub_->publish(cmd);
        sent = true;
      }
    }
  } else if (target_pose_pub_) {
    target_pose_pub_->publish(cmd);
    RCLCPP_INFO(get_logger(), "Publish target pose index=%zu", idx);
    sent = true;
  }
  std::ostringstream oss;
  oss << "target_pose index=" << idx << " arm_id=" << cmd.arm_id
      << " pos=(" << cmd.pose.pose.position.x << ", " << cmd.pose.pose.position.y << ", "
      << cmd.pose.pose.position.z << ")"
      << " quat=(" << cmd.pose.pose.orientation.x << ", " << cmd.pose.pose.orientation.y << ", "
      << cmd.pose.pose.orientation.z << ", " << cmd.pose.pose.orientation.w << ")";
  publishLog(oss.str());
  return sent;
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
    const bool has_ref =
      tryGetLiveReferenceTcamBase(t_cam_base_ref) || getUrdfReferenceTcamBaseForArm(arm_id_, t_cam_base_ref);
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
    !has_grip_ref &&
    (tryGetLiveReferenceTcamBase(t_cam_base_ref) || getUrdfReferenceTcamBaseForArm(arm_id_, t_cam_base_ref));
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

}  // namespace calib_sim_isaac
