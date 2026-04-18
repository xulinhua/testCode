#ifndef CALIB_SIM__CONFIG_DATA_MANAGER_HPP_
#define CALIB_SIM__CONFIG_DATA_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace calib_sim
{

struct CalibConfigData
{
  int arm_id{0};
  int target_marker_id{0};
  int aruco_dict_id{5};
  int min_samples{8};
  double marker_length_m{0.1};
  double state_timeout_sec{2.0};
  std::string image_topic{"/camera/image_raw"};
  std::string depth_topic{"/camera/depth/image_raw"};
  std::string camera_info_topic{"/camera/camera_info"};
  std::string robot_pose_topic{"/robot_pose"};
  std::string robot_state_topic{"/robot_reached"};
  std::string robot_target_topic{"/robot_target_pose"};
  std::string output_dir{"calib_output"};
  bool use_current_pose_as_center{true};
  std::vector<double> target_poses;
  /// 机械臂 1 专用轨迹（与 target_poses 同格式）；为空则回退为 target_poses
  std::vector<double> target_poses_arm1;
  std::vector<double> target_position_offsets;
  /// Degrees [roll,pitch,yaw] per waypoint; paired with target_position_offsets (same length). Applied as R_out = R_base * (Rz*Ry*Rx).
  std::vector<double> target_orientation_offsets_rpy_deg;
  bool use_known_target_mount{true};
  std::vector<double> target_to_gripper_pose{0.0, -0.12, -0.01, 0.0, 0.0, 0.0, 1.0};
  double marker_bbox_ratio_min{0.012};
  double marker_bbox_ratio_max{0.030};
  /// 采样时用 TF(base->末端) 与图像时间对齐；需与 calib_sim_bridge 的 ee_frame 一致
  bool use_tf_for_sample_pose{true};
  std::string tf_base_frame{"base_link"};
  std::string tf_ee_frame_arm0{"J1_6"};
  std::string tf_ee_frame_arm1{"J2_6"};
  /// 眼在手外 + use_known_target_mount 时，known_mount 平移不一致阈值 (m)
  double known_mount_quality_max_m{0.20};
  /// 初始化标定：先发全零关节，再延时后发缓存初始位姿（避免 IK 立刻覆盖复位）
  int init_reset_burst_count{6};
  int init_reset_burst_period_ms{50};
  int init_delay_ms_after_reset{500};
};

class EyeToHandConfigDataManager
{
public:
  CalibConfigData load(rclcpp::Node & node) const;
};

class EyeInHandConfigDataManager
{
public:
  CalibConfigData load(rclcpp::Node & node) const;
};

}  // namespace calib_sim

#endif  // CALIB_SIM__CONFIG_DATA_MANAGER_HPP_
