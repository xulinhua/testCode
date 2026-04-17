#include "calib_sim/config_data_manager.hpp"

namespace calib_sim
{

namespace
{
CalibConfigData loadCommon(rclcpp::Node & node, const CalibConfigData & defaults)
{
  CalibConfigData cfg = defaults;

  node.declare_parameter("arm_id", cfg.arm_id);
  node.declare_parameter("target_marker_id", cfg.target_marker_id);
  node.declare_parameter("aruco_dict_id", cfg.aruco_dict_id);
  node.declare_parameter("marker_length_m", cfg.marker_length_m);
  node.declare_parameter("min_samples", cfg.min_samples);
  node.declare_parameter("state_timeout_sec", cfg.state_timeout_sec);
  node.declare_parameter("image_topic", cfg.image_topic);
  node.declare_parameter("depth_topic", cfg.depth_topic);
  node.declare_parameter("camera_info_topic", cfg.camera_info_topic);
  node.declare_parameter("robot_pose_topic", cfg.robot_pose_topic);
  node.declare_parameter("robot_state_topic", cfg.robot_state_topic);
  node.declare_parameter("robot_target_topic", cfg.robot_target_topic);
  node.declare_parameter("output_dir", cfg.output_dir);
  node.declare_parameter("use_current_pose_as_center", cfg.use_current_pose_as_center);
  node.declare_parameter("target_poses", std::vector<double>{});
  node.declare_parameter("target_position_offsets", std::vector<double>{});
  node.declare_parameter("use_known_target_mount", cfg.use_known_target_mount);
  node.declare_parameter("target_to_gripper_pose", cfg.target_to_gripper_pose);
  node.declare_parameter("marker_bbox_ratio_min", cfg.marker_bbox_ratio_min);
  node.declare_parameter("marker_bbox_ratio_max", cfg.marker_bbox_ratio_max);

  cfg.arm_id = node.get_parameter("arm_id").as_int();
  cfg.target_marker_id = node.get_parameter("target_marker_id").as_int();
  cfg.aruco_dict_id = node.get_parameter("aruco_dict_id").as_int();
  cfg.marker_length_m = node.get_parameter("marker_length_m").as_double();
  cfg.min_samples = node.get_parameter("min_samples").as_int();
  cfg.state_timeout_sec = node.get_parameter("state_timeout_sec").as_double();
  cfg.image_topic = node.get_parameter("image_topic").as_string();
  cfg.depth_topic = node.get_parameter("depth_topic").as_string();
  cfg.camera_info_topic = node.get_parameter("camera_info_topic").as_string();
  cfg.robot_pose_topic = node.get_parameter("robot_pose_topic").as_string();
  cfg.robot_state_topic = node.get_parameter("robot_state_topic").as_string();
  cfg.robot_target_topic = node.get_parameter("robot_target_topic").as_string();
  cfg.output_dir = node.get_parameter("output_dir").as_string();
  cfg.use_current_pose_as_center = node.get_parameter("use_current_pose_as_center").as_bool();
  cfg.use_known_target_mount = node.get_parameter("use_known_target_mount").as_bool();
  cfg.target_to_gripper_pose = node.get_parameter("target_to_gripper_pose").as_double_array();
  cfg.marker_bbox_ratio_min = node.get_parameter("marker_bbox_ratio_min").as_double();
  cfg.marker_bbox_ratio_max = node.get_parameter("marker_bbox_ratio_max").as_double();

  rclcpp::Parameter target_poses_param;
  if (node.get_parameter("target_poses", target_poses_param) &&
    target_poses_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    cfg.target_poses = target_poses_param.as_double_array();
  }

  rclcpp::Parameter offsets_param;
  if (node.get_parameter("target_position_offsets", offsets_param) &&
    offsets_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    cfg.target_position_offsets = offsets_param.as_double_array();
  }

  return cfg;
}
}  // namespace

CalibConfigData EyeToHandConfigDataManager::load(rclcpp::Node & node) const
{
  CalibConfigData defaults;
  defaults.arm_id = 0;
  defaults.image_topic = "/camera0/image_raw";
  defaults.depth_topic = "/camera0/depth/image_raw";
  defaults.camera_info_topic = "/camera0/camera_info";
  return loadCommon(node, defaults);
}

CalibConfigData EyeInHandConfigDataManager::load(rclcpp::Node & node) const
{
  CalibConfigData defaults;
  defaults.arm_id = 1;
  defaults.image_topic = "/camera1/image_raw";
  defaults.depth_topic = "/camera1/depth/image_raw";
  defaults.camera_info_topic = "/camera1/camera_info";
  return loadCommon(node, defaults);
}

}  // namespace calib_sim
