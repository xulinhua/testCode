#ifndef CALIB_SIM__CONFIG_DATA_MANAGER_HPP_
#define CALIB_SIM__CONFIG_DATA_MANAGER_HPP_

// 直接从 YAML 加载标定配置（不依赖运行期 ROS 参数）。

#include <string>
#include <unordered_map>
#include <vector>

namespace calib_sim_isaac
{

/// 单套标定流程使用的可序列化配置（话题、ArUco、轨迹、TF 帧、超时等）。
struct CalibConfigData
{
  int arm_id{0};
  int target_marker_id{0};
  int aruco_dict_id{5};
  int min_samples{8};
  double marker_length_m{0.1};
  double state_timeout_sec{2.0};
  /// /robot_reached 仍为 true 时允许的最大消息年龄（秒）
  /// 发布目标后等待到位的最长时间；≤0 时用 state_timeout_sec
  double reached_wait_timeout_sec{-1.0};
  /// 单点采图等待检测成功的超时；≤0 时用 state_timeout_sec
  double capture_wait_timeout_sec{-1.0};
  std::string image_topic{"/camera/image_raw"};
  std::string depth_topic{"/camera/depth/image_raw"};
  std::string camera_info_topic{"/camera/camera_info"};
  std::string robot_pose_topic{"/joint_states"};
  std::string robot_state_topic{"/robot_reached"};
  std::string robot_target_topic{"/joint_command"};
  std::string output_dir{"calib_output_isaac"};
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
  /// 眼在手外/固定机位：TF 中 cam0 与 OpenCV/仿真成像对齐的“相机子坐标系”名称（如 Isaac 的 Camera_Pseudo_Depth）。
  /// 为空时回退 camera0_optical_frame / camera_info.frame_id。
  std::string tf_fixed_cam_ref_frame{""};
  /// 眼在手外 + use_known_target_mount 时，known_mount 平移不一致阈值 (m)
  double known_mount_quality_max_m{0.20};
  /// 初始化标定：先发全零关节，再延时后发缓存初始位姿（避免 IK 立刻覆盖复位）
  int init_reset_burst_count{6};
  int init_reset_burst_period_ms{50};
  int init_delay_ms_after_reset{500};
  bool use_joint_kinematics_interface{true};
  bool enable_legacy_pose_fallback{false};
  double ik_command_max_age_sec{1.0};
  double joint_reach_tolerance_rad{0.015};
  double pose_reach_position_tolerance_m{0.004};
  double pose_reach_rotation_tolerance_deg{1.0};
  double sample_reproj_error_max_px{1.8};
  double handeye_reproj_filter_max_px{1.5};
  int handeye_reproj_min_samples{12};
  int reproj_reject_skip_count{8};
  bool eye_to_hand_retry_enabled{false};
  double post_reach_settle_sec{1.2};
  bool post_reach_settle_require_joint{true};
  bool reach_require_joint_and_tf{true};
  double arm_reach_timeout_grace_sec{2.0};
  double topic_wait_timeout_sec{60.0};
  int joint_command_burst_count{5};
  std::string legacy_pose_command_topic{"/robot_target_pose"};
  std::string kinematics_pose_goal_topic{"/nova_target_pose"};
  std::string kinematics_joint_command_topic{"/ik_joint_command"};
  std::string ik_urdf_path{"/home/hs/testCode/simulation/src/nova_sim/urdf/nova_robot_position.urdf"};
  std::vector<std::string> joint_names_arm0{
    "J1_1_joint", "J1_2_joint", "J1_3_joint", "J1_4_joint", "J1_5_joint", "J1_6_joint"};
  std::vector<std::string> joint_names_arm1{
    "J2_1_joint", "J2_2_joint", "J2_3_joint", "J2_4_joint", "J2_5_joint", "J2_6_joint"};
  std::string nova_all_joints_reset_topic{"/nova_sim/reset_all_joints"};
};

class EyeToHandConfigDataManager
{
public:
  /// 从 eye_to_hand.yaml 读取配置（一次性加载，运行期不再依赖 ROS 参数服务）。
  CalibConfigData load(const std::string & yaml_path) const;
};

class EyeInHandConfigDataManager
{
public:
  /// 从 eye_in_hand.yaml 读取配置（含 arm0/arm1 分块与当前 arm 绑定字段）。
  CalibConfigData load(const std::string & yaml_path) const;
};

/// 从带前缀的配置块（如 eth0.arm_id）加载；用于 calib_unified.yaml。
CalibConfigData loadCalibConfigPrefixedFromYaml(
  const std::string & yaml_path, const std::string & prefix, const CalibConfigData & defaults);

/// 从 calib_unified.yaml 读取 eth0/eth1/eih0/eih1 四套配置并返回映射。
std::unordered_map<std::string, CalibConfigData> loadUnifiedConfigsFromYaml(const std::string & yaml_path);

CalibConfigData defaultCalibConfigEyeToHand();
CalibConfigData defaultCalibConfigEyeInHand();

}  // namespace calib_sim_isaac

#endif  // CALIB_SIM__CONFIG_DATA_MANAGER_HPP_
