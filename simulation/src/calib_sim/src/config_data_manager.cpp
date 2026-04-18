#include "calib_sim/config_data_manager.hpp"

#include <stdexcept>

namespace calib_sim
{

namespace
{
std::string paramName(const std::string & prefix, const char * name)
{
  return prefix.empty() ? std::string(name) : prefix + "." + name;
}

/// 眼在手上：arm0 / arm1 各自相机话题与 target_poses；根级为两臂共用参数。
CalibConfigData loadEyeInHandSplit(rclcpp::Node & node, CalibConfigData base)
{
  const auto p0 = [](const char * n) { return paramName("arm0", n); };
  const auto p1 = [](const char * n) { return paramName("arm1", n); };

  node.declare_parameter("arm_id", base.arm_id);
  node.declare_parameter("aruco_dict_id", base.aruco_dict_id);
  node.declare_parameter("marker_length_m", base.marker_length_m);
  node.declare_parameter("min_samples", base.min_samples);
  node.declare_parameter("state_timeout_sec", base.state_timeout_sec);
  node.declare_parameter("reached_wait_timeout_sec", base.reached_wait_timeout_sec);
  node.declare_parameter("capture_wait_timeout_sec", base.capture_wait_timeout_sec);
  node.declare_parameter("use_known_target_mount", base.use_known_target_mount);
  node.declare_parameter("target_to_gripper_pose", base.target_to_gripper_pose);
  node.declare_parameter("marker_bbox_ratio_min", base.marker_bbox_ratio_min);
  node.declare_parameter("marker_bbox_ratio_max", base.marker_bbox_ratio_max);
  node.declare_parameter("use_tf_for_sample_pose", base.use_tf_for_sample_pose);
  node.declare_parameter("tf_base_frame", base.tf_base_frame);
  node.declare_parameter("tf_ee_frame_arm0", base.tf_ee_frame_arm0);
  node.declare_parameter("tf_ee_frame_arm1", base.tf_ee_frame_arm1);
  node.declare_parameter("known_mount_quality_max_m", base.known_mount_quality_max_m);
  node.declare_parameter("init_reset_burst_count", base.init_reset_burst_count);
  node.declare_parameter("init_reset_burst_period_ms", base.init_reset_burst_period_ms);
  node.declare_parameter("init_delay_ms_after_reset", base.init_delay_ms_after_reset);
  node.declare_parameter("robot_pose_topic", base.robot_pose_topic);
  node.declare_parameter("robot_state_topic", base.robot_state_topic);
  node.declare_parameter("robot_target_topic", base.robot_target_topic);
  node.declare_parameter("output_dir", base.output_dir);
  node.declare_parameter("use_current_pose_as_center", base.use_current_pose_as_center);
  node.declare_parameter("target_position_offsets", std::vector<double>{});
  node.declare_parameter("target_orientation_offsets_rpy_deg", std::vector<double>{});

  node.declare_parameter(p0("target_marker_id"), 0);
  node.declare_parameter(p0("image_topic"), base.image_topic);
  node.declare_parameter(p0("depth_topic"), base.depth_topic);
  node.declare_parameter(p0("camera_info_topic"), base.camera_info_topic);
  node.declare_parameter(p0("target_poses"), std::vector<double>{});

  node.declare_parameter(p1("target_marker_id"), 0);
  node.declare_parameter(p1("image_topic"), base.image_topic);
  node.declare_parameter(p1("depth_topic"), base.depth_topic);
  node.declare_parameter(p1("camera_info_topic"), base.camera_info_topic);
  node.declare_parameter(p1("target_poses"), std::vector<double>{});

  base.arm_id = node.get_parameter("arm_id").as_int();
  base.aruco_dict_id = node.get_parameter("aruco_dict_id").as_int();
  base.marker_length_m = node.get_parameter("marker_length_m").as_double();
  base.min_samples = node.get_parameter("min_samples").as_int();
  base.state_timeout_sec = node.get_parameter("state_timeout_sec").as_double();
  base.reached_wait_timeout_sec = node.get_parameter("reached_wait_timeout_sec").as_double();
  base.capture_wait_timeout_sec = node.get_parameter("capture_wait_timeout_sec").as_double();
  base.use_known_target_mount = node.get_parameter("use_known_target_mount").as_bool();
  base.target_to_gripper_pose = node.get_parameter("target_to_gripper_pose").as_double_array();
  base.marker_bbox_ratio_min = node.get_parameter("marker_bbox_ratio_min").as_double();
  base.marker_bbox_ratio_max = node.get_parameter("marker_bbox_ratio_max").as_double();
  base.use_tf_for_sample_pose = node.get_parameter("use_tf_for_sample_pose").as_bool();
  base.tf_base_frame = node.get_parameter("tf_base_frame").as_string();
  base.tf_ee_frame_arm0 = node.get_parameter("tf_ee_frame_arm0").as_string();
  base.tf_ee_frame_arm1 = node.get_parameter("tf_ee_frame_arm1").as_string();
  base.known_mount_quality_max_m = node.get_parameter("known_mount_quality_max_m").as_double();
  base.init_reset_burst_count = node.get_parameter("init_reset_burst_count").as_int();
  base.init_reset_burst_period_ms = node.get_parameter("init_reset_burst_period_ms").as_int();
  base.init_delay_ms_after_reset = node.get_parameter("init_delay_ms_after_reset").as_int();
  base.robot_pose_topic = node.get_parameter("robot_pose_topic").as_string();
  base.robot_state_topic = node.get_parameter("robot_state_topic").as_string();
  base.robot_target_topic = node.get_parameter("robot_target_topic").as_string();
  base.output_dir = node.get_parameter("output_dir").as_string();
  base.use_current_pose_as_center = node.get_parameter("use_current_pose_as_center").as_bool();

  rclcpp::Parameter offp;
  if (node.get_parameter("target_position_offsets", offp) &&
    offp.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    base.target_position_offsets = offp.as_double_array();
  }
  rclcpp::Parameter orp;
  if (node.get_parameter("target_orientation_offsets_rpy_deg", orp) &&
    orp.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    base.target_orientation_offsets_rpy_deg = orp.as_double_array();
  }

  const int m0 = node.get_parameter(p0("target_marker_id")).as_int();
  const int m1 = node.get_parameter(p1("target_marker_id")).as_int();
  const std::string i0 = node.get_parameter(p0("image_topic")).as_string();
  const std::string i1 = node.get_parameter(p1("image_topic")).as_string();
  const std::string d0 = node.get_parameter(p0("depth_topic")).as_string();
  const std::string d1 = node.get_parameter(p1("depth_topic")).as_string();
  const std::string c0 = node.get_parameter(p0("camera_info_topic")).as_string();
  const std::string c1 = node.get_parameter(p1("camera_info_topic")).as_string();

  rclcpp::Parameter tp0;
  if (!node.get_parameter(p0("target_poses"), tp0) ||
    tp0.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    throw std::runtime_error("eye_in_hand: arm0.target_poses missing or not double array");
  }
  rclcpp::Parameter tp1;
  if (!node.get_parameter(p1("target_poses"), tp1) ||
    tp1.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    throw std::runtime_error("eye_in_hand: arm1.target_poses missing or not double array");
  }
  base.target_poses = tp0.as_double_array();
  base.target_poses_arm1 = tp1.as_double_array();

  if (base.arm_id == 0) {
    base.target_marker_id = m0;
    base.image_topic = i0;
    base.depth_topic = d0;
    base.camera_info_topic = c0;
  } else if (base.arm_id == 1) {
    base.target_marker_id = m1;
    base.image_topic = i1;
    base.depth_topic = d1;
    base.camera_info_topic = c1;
  } else {
    throw std::runtime_error("eye_in_hand: arm_id must be 0 or 1 when using arm0/arm1 blocks");
  }

  return base;
}

CalibConfigData eyeToHandTemplateDefaults()
{
  CalibConfigData defaults;
  defaults.arm_id = 0;
  defaults.image_topic = "/camera0_rgb_sensor/image_raw";
  defaults.depth_topic = "/camera0_depth_sensor/depth/image_raw";
  defaults.camera_info_topic = "/camera0_rgb_sensor/camera_info";
  return defaults;
}

CalibConfigData eyeInHandTemplateDefaults()
{
  CalibConfigData defaults;
  defaults.arm_id = 1;
  defaults.image_topic = "/camera0_rgb_sensor/image_raw";
  defaults.depth_topic = "/camera0/depth/image_raw";
  defaults.camera_info_topic = "/camera0_rgb_sensor/camera_info";
  return defaults;
}

CalibConfigData loadCommon(
  rclcpp::Node & node, const CalibConfigData & defaults, const std::string & prefix)
{
  CalibConfigData cfg = defaults;

  const auto p = [&prefix](const char * name) { return paramName(prefix, name); };

  node.declare_parameter(p("arm_id"), cfg.arm_id);
  node.declare_parameter(p("target_marker_id"), cfg.target_marker_id);
  node.declare_parameter(p("aruco_dict_id"), cfg.aruco_dict_id);
  node.declare_parameter(p("marker_length_m"), cfg.marker_length_m);
  node.declare_parameter(p("min_samples"), cfg.min_samples);
  node.declare_parameter(p("state_timeout_sec"), cfg.state_timeout_sec);
  node.declare_parameter(p("reached_wait_timeout_sec"), cfg.reached_wait_timeout_sec);
  node.declare_parameter(p("capture_wait_timeout_sec"), cfg.capture_wait_timeout_sec);
  node.declare_parameter(p("image_topic"), cfg.image_topic);
  node.declare_parameter(p("depth_topic"), cfg.depth_topic);
  node.declare_parameter(p("camera_info_topic"), cfg.camera_info_topic);
  node.declare_parameter(p("robot_pose_topic"), cfg.robot_pose_topic);
  node.declare_parameter(p("robot_state_topic"), cfg.robot_state_topic);
  node.declare_parameter(p("robot_target_topic"), cfg.robot_target_topic);
  node.declare_parameter(p("output_dir"), cfg.output_dir);
  node.declare_parameter(p("use_current_pose_as_center"), cfg.use_current_pose_as_center);
  node.declare_parameter(p("target_poses"), std::vector<double>{});
  node.declare_parameter(p("target_poses_arm1"), std::vector<double>{});
  node.declare_parameter(p("target_position_offsets"), std::vector<double>{});
  node.declare_parameter(p("target_orientation_offsets_rpy_deg"), std::vector<double>{});
  node.declare_parameter(p("use_known_target_mount"), cfg.use_known_target_mount);
  node.declare_parameter(p("target_to_gripper_pose"), cfg.target_to_gripper_pose);
  node.declare_parameter(p("marker_bbox_ratio_min"), cfg.marker_bbox_ratio_min);
  node.declare_parameter(p("marker_bbox_ratio_max"), cfg.marker_bbox_ratio_max);
  node.declare_parameter(p("use_tf_for_sample_pose"), cfg.use_tf_for_sample_pose);
  node.declare_parameter(p("tf_base_frame"), cfg.tf_base_frame);
  node.declare_parameter(p("tf_ee_frame_arm0"), cfg.tf_ee_frame_arm0);
  node.declare_parameter(p("tf_ee_frame_arm1"), cfg.tf_ee_frame_arm1);
  node.declare_parameter(p("known_mount_quality_max_m"), cfg.known_mount_quality_max_m);
  node.declare_parameter(p("init_reset_burst_count"), cfg.init_reset_burst_count);
  node.declare_parameter(p("init_reset_burst_period_ms"), cfg.init_reset_burst_period_ms);
  node.declare_parameter(p("init_delay_ms_after_reset"), cfg.init_delay_ms_after_reset);

  cfg.arm_id = node.get_parameter(p("arm_id")).as_int();
  cfg.target_marker_id = node.get_parameter(p("target_marker_id")).as_int();
  cfg.aruco_dict_id = node.get_parameter(p("aruco_dict_id")).as_int();
  cfg.marker_length_m = node.get_parameter(p("marker_length_m")).as_double();
  cfg.min_samples = node.get_parameter(p("min_samples")).as_int();
  cfg.state_timeout_sec = node.get_parameter(p("state_timeout_sec")).as_double();
  cfg.reached_wait_timeout_sec = node.get_parameter(p("reached_wait_timeout_sec")).as_double();
  cfg.capture_wait_timeout_sec = node.get_parameter(p("capture_wait_timeout_sec")).as_double();
  cfg.image_topic = node.get_parameter(p("image_topic")).as_string();
  cfg.depth_topic = node.get_parameter(p("depth_topic")).as_string();
  cfg.camera_info_topic = node.get_parameter(p("camera_info_topic")).as_string();
  cfg.robot_pose_topic = node.get_parameter(p("robot_pose_topic")).as_string();
  cfg.robot_state_topic = node.get_parameter(p("robot_state_topic")).as_string();
  cfg.robot_target_topic = node.get_parameter(p("robot_target_topic")).as_string();
  cfg.output_dir = node.get_parameter(p("output_dir")).as_string();
  cfg.use_current_pose_as_center = node.get_parameter(p("use_current_pose_as_center")).as_bool();
  cfg.use_known_target_mount = node.get_parameter(p("use_known_target_mount")).as_bool();
  cfg.target_to_gripper_pose = node.get_parameter(p("target_to_gripper_pose")).as_double_array();
  cfg.marker_bbox_ratio_min = node.get_parameter(p("marker_bbox_ratio_min")).as_double();
  cfg.marker_bbox_ratio_max = node.get_parameter(p("marker_bbox_ratio_max")).as_double();
  cfg.use_tf_for_sample_pose = node.get_parameter(p("use_tf_for_sample_pose")).as_bool();
  cfg.tf_base_frame = node.get_parameter(p("tf_base_frame")).as_string();
  cfg.tf_ee_frame_arm0 = node.get_parameter(p("tf_ee_frame_arm0")).as_string();
  cfg.tf_ee_frame_arm1 = node.get_parameter(p("tf_ee_frame_arm1")).as_string();
  cfg.known_mount_quality_max_m = node.get_parameter(p("known_mount_quality_max_m")).as_double();
  cfg.init_reset_burst_count = node.get_parameter(p("init_reset_burst_count")).as_int();
  cfg.init_reset_burst_period_ms = node.get_parameter(p("init_reset_burst_period_ms")).as_int();
  cfg.init_delay_ms_after_reset = node.get_parameter(p("init_delay_ms_after_reset")).as_int();

  rclcpp::Parameter target_poses_param;
  if (node.get_parameter(p("target_poses"), target_poses_param) &&
    target_poses_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    cfg.target_poses = target_poses_param.as_double_array();
  }

  rclcpp::Parameter target_poses_arm1_param;
  if (node.get_parameter(p("target_poses_arm1"), target_poses_arm1_param) &&
    target_poses_arm1_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    cfg.target_poses_arm1 = target_poses_arm1_param.as_double_array();
  }

  rclcpp::Parameter offsets_param;
  if (node.get_parameter(p("target_position_offsets"), offsets_param) &&
    offsets_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    cfg.target_position_offsets = offsets_param.as_double_array();
  }

  rclcpp::Parameter orient_param;
  if (node.get_parameter(p("target_orientation_offsets_rpy_deg"), orient_param) &&
    orient_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    cfg.target_orientation_offsets_rpy_deg = orient_param.as_double_array();
  }

  return cfg;
}
}  // namespace

CalibConfigData loadCalibConfigPrefixed(
  rclcpp::Node & node, const std::string & prefix, const CalibConfigData & defaults)
{
  return loadCommon(node, defaults, prefix);
}

CalibConfigData EyeToHandConfigDataManager::load(rclcpp::Node & node) const
{
  return loadCommon(node, eyeToHandTemplateDefaults(), "");
}

CalibConfigData EyeInHandConfigDataManager::load(rclcpp::Node & node) const
{
  // 眼在手上：根级共用 + arm0/arm1 分臂（相机话题、marker_id、target_poses）
  return loadEyeInHandSplit(node, eyeInHandTemplateDefaults());
}

CalibConfigData defaultCalibConfigEyeToHand()
{
  return eyeToHandTemplateDefaults();
}

CalibConfigData defaultCalibConfigEyeInHand()
{
  return eyeInHandTemplateDefaults();
}

}  // namespace calib_sim
