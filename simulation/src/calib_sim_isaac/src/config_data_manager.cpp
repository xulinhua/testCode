// 从 YAML 读取标定配置，并组装 CalibConfigData（含 unified 前缀加载）。
#include "calib_sim_isaac/config_data_manager.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace calib_sim_isaac
{

namespace
{
template<typename T>
void loadScalarIfPresent(const YAML::Node & n, const char * key, T & out)
{
  if (n && n[key]) {
    out = n[key].as<T>();
  }
}

template<typename T>
void loadVectorIfPresent(const YAML::Node & n, const char * key, std::vector<T> & out)
{
  if (n && n[key] && n[key].IsSequence()) {
    out = n[key].as<std::vector<T>>();
  }
}

YAML::Node loadParamRoot(const std::string & yaml_path)
{
  const YAML::Node doc = YAML::LoadFile(yaml_path);
  YAML::Node params;
  if (doc["/**"] && doc["/**"]["ros__parameters"]) {
    params = doc["/**"]["ros__parameters"];
  } else if (doc["ros__parameters"]) {
    params = doc["ros__parameters"];
  } else if (doc.IsMap()) {
    for (const auto & kv : doc) {
      const YAML::Node v = kv.second;
      if (v && v["ros__parameters"]) {
        params = v["ros__parameters"];
        break;
      }
    }
  }
  if (!params || !params.IsMap()) {
    throw std::runtime_error("invalid yaml, ros__parameters not found: " + yaml_path);
  }
  return params;
}

CalibConfigData eyeToHandTemplateDefaults()
{
  CalibConfigData defaults;
  defaults.arm_id = 0;
  defaults.image_topic = "/camera0_rgb_sensor/image_raw";
  defaults.depth_topic = "/camera0_depth_sensor/depth/image_raw";
  defaults.camera_info_topic = "/camera0_rgb_sensor/camera_info";
  defaults.tf_fixed_cam_ref_frame = "Camera_Pseudo_Depth";
  // 与 eye_to_hand.yaml / unified eth* 对齐：远距固定机位占画幅小，默认勿用 0.012 否则易 marker_too_small。
  defaults.marker_bbox_ratio_min = 0.0008;
  defaults.marker_bbox_ratio_max = 0.060;
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

void loadCommonFields(const YAML::Node & n, CalibConfigData & cfg)
{
  loadScalarIfPresent(n, "arm_id", cfg.arm_id);
  loadScalarIfPresent(n, "target_marker_id", cfg.target_marker_id);
  loadScalarIfPresent(n, "aruco_dict_id", cfg.aruco_dict_id);
  loadScalarIfPresent(n, "marker_length_m", cfg.marker_length_m);
  loadScalarIfPresent(n, "min_samples", cfg.min_samples);
  loadScalarIfPresent(n, "state_timeout_sec", cfg.state_timeout_sec);
  loadScalarIfPresent(n, "reached_wait_timeout_sec", cfg.reached_wait_timeout_sec);
  loadScalarIfPresent(n, "capture_wait_timeout_sec", cfg.capture_wait_timeout_sec);
  loadScalarIfPresent(n, "image_topic", cfg.image_topic);
  loadScalarIfPresent(n, "depth_topic", cfg.depth_topic);
  loadScalarIfPresent(n, "camera_info_topic", cfg.camera_info_topic);
  loadScalarIfPresent(n, "robot_pose_topic", cfg.robot_pose_topic);
  loadScalarIfPresent(n, "robot_state_topic", cfg.robot_state_topic);
  loadScalarIfPresent(n, "robot_target_topic", cfg.robot_target_topic);
  loadScalarIfPresent(n, "output_dir", cfg.output_dir);
  loadScalarIfPresent(n, "use_current_pose_as_center", cfg.use_current_pose_as_center);
  loadVectorIfPresent(n, "target_poses", cfg.target_poses);
  loadVectorIfPresent(n, "target_poses_arm1", cfg.target_poses_arm1);
  loadVectorIfPresent(n, "target_position_offsets", cfg.target_position_offsets);
  loadVectorIfPresent(n, "target_orientation_offsets_rpy_deg", cfg.target_orientation_offsets_rpy_deg);
  loadScalarIfPresent(n, "use_known_target_mount", cfg.use_known_target_mount);
  loadVectorIfPresent(n, "target_to_gripper_pose", cfg.target_to_gripper_pose);
  loadScalarIfPresent(n, "marker_bbox_ratio_min", cfg.marker_bbox_ratio_min);
  loadScalarIfPresent(n, "marker_bbox_ratio_max", cfg.marker_bbox_ratio_max);
  loadScalarIfPresent(n, "use_tf_for_sample_pose", cfg.use_tf_for_sample_pose);
  loadScalarIfPresent(n, "tf_base_frame", cfg.tf_base_frame);
  loadScalarIfPresent(n, "tf_ee_frame_arm0", cfg.tf_ee_frame_arm0);
  loadScalarIfPresent(n, "tf_ee_frame_arm1", cfg.tf_ee_frame_arm1);
  loadScalarIfPresent(n, "tf_fixed_cam_ref_frame", cfg.tf_fixed_cam_ref_frame);
  loadScalarIfPresent(n, "known_mount_quality_max_m", cfg.known_mount_quality_max_m);
  loadScalarIfPresent(n, "init_reset_burst_count", cfg.init_reset_burst_count);
  loadScalarIfPresent(n, "init_reset_burst_period_ms", cfg.init_reset_burst_period_ms);
  loadScalarIfPresent(n, "init_delay_ms_after_reset", cfg.init_delay_ms_after_reset);
  loadScalarIfPresent(n, "use_joint_kinematics_interface", cfg.use_joint_kinematics_interface);
  loadScalarIfPresent(n, "enable_legacy_pose_fallback", cfg.enable_legacy_pose_fallback);
  loadScalarIfPresent(n, "ik_command_max_age_sec", cfg.ik_command_max_age_sec);
  loadScalarIfPresent(n, "joint_reach_tolerance_rad", cfg.joint_reach_tolerance_rad);
  loadScalarIfPresent(n, "pose_reach_position_tolerance_m", cfg.pose_reach_position_tolerance_m);
  loadScalarIfPresent(n, "pose_reach_rotation_tolerance_deg", cfg.pose_reach_rotation_tolerance_deg);
  loadScalarIfPresent(n, "sample_reproj_error_max_px", cfg.sample_reproj_error_max_px);
  loadScalarIfPresent(n, "handeye_reproj_filter_max_px", cfg.handeye_reproj_filter_max_px);
  loadScalarIfPresent(n, "handeye_reproj_min_samples", cfg.handeye_reproj_min_samples);
  loadScalarIfPresent(n, "reproj_reject_skip_count", cfg.reproj_reject_skip_count);
  loadScalarIfPresent(n, "eye_to_hand_retry_enabled", cfg.eye_to_hand_retry_enabled);
  loadScalarIfPresent(n, "post_reach_settle_sec", cfg.post_reach_settle_sec);
  loadScalarIfPresent(n, "post_reach_settle_require_joint", cfg.post_reach_settle_require_joint);
  loadScalarIfPresent(n, "reach_require_joint_and_tf", cfg.reach_require_joint_and_tf);
  loadScalarIfPresent(n, "arm_reach_timeout_grace_sec", cfg.arm_reach_timeout_grace_sec);
  loadScalarIfPresent(n, "topic_wait_timeout_sec", cfg.topic_wait_timeout_sec);
  loadScalarIfPresent(n, "joint_command_burst_count", cfg.joint_command_burst_count);
  loadScalarIfPresent(n, "legacy_pose_command_topic", cfg.legacy_pose_command_topic);
  loadScalarIfPresent(n, "kinematics_pose_goal_topic", cfg.kinematics_pose_goal_topic);
  loadScalarIfPresent(n, "kinematics_joint_command_topic", cfg.kinematics_joint_command_topic);
  loadScalarIfPresent(n, "ik_urdf_path", cfg.ik_urdf_path);
  loadVectorIfPresent(n, "joint_names_arm0", cfg.joint_names_arm0);
  loadVectorIfPresent(n, "joint_names_arm1", cfg.joint_names_arm1);
  loadScalarIfPresent(n, "nova_all_joints_reset_topic", cfg.nova_all_joints_reset_topic);
}
}  // namespace

CalibConfigData loadCalibConfigPrefixedFromYaml(
  const std::string & yaml_path, const std::string & prefix, const CalibConfigData & defaults)
{
  const YAML::Node params = loadParamRoot(yaml_path);
  const YAML::Node mode_node = params[prefix];
  if (!mode_node || !mode_node.IsMap()) {
    throw std::runtime_error("missing unified mode block: " + prefix);
  }
  CalibConfigData cfg = defaults;
  loadCommonFields(mode_node, cfg);
  return cfg;
}

std::unordered_map<std::string, CalibConfigData> loadUnifiedConfigsFromYaml(const std::string & yaml_path)
{
  static const char * kModes[] = {"eth0", "eth1", "eih0", "eih1"};
  std::unordered_map<std::string, CalibConfigData> out;
  for (const char * mode : kModes) {
    const bool eih = (std::string(mode).rfind("eih", 0) == 0);
    out.emplace(
      mode, loadCalibConfigPrefixedFromYaml(
        yaml_path, mode, eih ? defaultCalibConfigEyeInHand() : defaultCalibConfigEyeToHand()));
  }
  return out;
}

CalibConfigData EyeToHandConfigDataManager::load(const std::string & yaml_path) const
{
  CalibConfigData cfg = eyeToHandTemplateDefaults();
  const YAML::Node params = loadParamRoot(yaml_path);
  loadCommonFields(params, cfg);
  return cfg;
}

CalibConfigData EyeInHandConfigDataManager::load(const std::string & yaml_path) const
{
  CalibConfigData cfg = eyeInHandTemplateDefaults();
  const YAML::Node params = loadParamRoot(yaml_path);
  loadCommonFields(params, cfg);

  const YAML::Node arm0 = params["arm0"];
  const YAML::Node arm1 = params["arm1"];
  if (!arm0 || !arm1) {
    throw std::runtime_error("eye_in_hand yaml missing arm0/arm1 block");
  }
  if (arm0["target_poses"]) {
    cfg.target_poses = arm0["target_poses"].as<std::vector<double>>();
  }
  if (arm1["target_poses"]) {
    cfg.target_poses_arm1 = arm1["target_poses"].as<std::vector<double>>();
  }
  if (cfg.target_poses.empty() || cfg.target_poses_arm1.empty()) {
    throw std::runtime_error("eye_in_hand yaml requires both arm0.target_poses and arm1.target_poses");
  }

  const YAML::Node & active_arm = (cfg.arm_id == 1) ? arm1 : arm0;
  loadScalarIfPresent(active_arm, "target_marker_id", cfg.target_marker_id);
  loadScalarIfPresent(active_arm, "image_topic", cfg.image_topic);
  loadScalarIfPresent(active_arm, "depth_topic", cfg.depth_topic);
  loadScalarIfPresent(active_arm, "camera_info_topic", cfg.camera_info_topic);
  return cfg;
}

CalibConfigData defaultCalibConfigEyeToHand()
{
  return eyeToHandTemplateDefaults();
}

CalibConfigData defaultCalibConfigEyeInHand()
{
  return eyeInHandTemplateDefaults();
}

}  // namespace calib_sim_isaac
