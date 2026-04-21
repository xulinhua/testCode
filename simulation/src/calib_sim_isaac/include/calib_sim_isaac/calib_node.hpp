#ifndef CALIB_SIM__CALIB_NODE_HPP_
#define CALIB_SIM__CALIB_NODE_HPP_

// 手眼标定核心节点：订阅相机图、机械臂位姿与到位状态，采集多组样本后调用 OpenCV 手眼算法，
// 输出 T_cam_base / T_cam_gripper 等并写入 YAML；支持眼在手外、眼在手上及 unified 多模式切换。

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <cstdlib>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include "calib_sim_isaac/msg/arm_pose.hpp"
#include "calib_sim_isaac/config_data_manager.hpp"

namespace calib_sim_isaac
{

/// ROS2 节点：ArUco 检测 + TF/话题位姿 + 手眼求解 + 结果发布（供 Qt/OpenCV UI 显示）。
class CalibNode : public rclcpp::Node
{
public:
  explicit CalibNode(
    const std::string & node_name, bool eye_in_hand,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// 统一标定节点：从 calib_unified.yaml 加载 eth0/eth1/eih0/eih1，需 unified_mode:=true。
  explicit CalibNode(
    const std::string & node_name, const rclcpp::NodeOptions & options);

  /// 在 std::make_shared 之后调用，用于创建 TF 监听器（构造函数内不可用 shared_from_this）。
  void initAfterSharedPtr(const std::shared_ptr<CalibNode> & self);

private:
  /// 单次采样：末端相对基座、标靶相对相机，及该帧角点重投影误差（像素）。
  struct Sample
  {
    cv::Mat r_gripper_to_base;
    cv::Mat t_gripper_to_base;
    cv::Mat r_target_to_cam;
    cv::Mat t_target_to_cam;
    /// Mean |projected−detected| over 4 marker corners (pixels), from PnP self-check
    double mean_corner_reproj_px{0.0};
  };

  /// 标定质量汇总：重投影、手眼链平移/旋转残差（均值与逐点）。
  struct HandEyeQualityMetrics
  {
    double mean_corner_reproj_px{0.0};
    std::vector<double> per_point_corner_reproj_px;
    double mean_chain_translation_m{0.0};
    std::vector<double> per_point_chain_translation_m;
    double mean_chain_rotation_deg{0.0};
    std::vector<double> per_point_chain_rotation_deg;
  };

  void declareAndLoadParameters();
  void applyCalibConfigData(const CalibConfigData & cfg);
  void loadUnifiedModeConfigs();
  void applyActiveModeConfig();
  void switchCalibMode(const std::string & mode);
  void initRosEntities();
  /// 统一模式下切换 eth0/eth1/eih0/eih1 时必须重建订阅，否则会一直接收旧相机话题。
  void renewCameraSubscriptions();
  void controlTimerCallback();
  void logTopicConnectivityOnce();

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void armPoseCallback(const calib_sim_isaac::msg::ArmPose::ConstSharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void ikJointCommandCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void armStateCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void controlCallback(const std_msgs::msg::String::ConstSharedPtr msg);

  bool tryCaptureSample(
    const cv::Mat & frame_bgr, cv::Mat & annotated, std::vector<int> & detected_ids,
    const rclcpp::Time & image_stamp);
  bool tryFillGripperPoseFromTf(
    const rclcpp::Time & image_stamp, cv::Mat & r_gripper_to_base, cv::Mat & t_gripper_to_base,
    calib_sim_isaac::msg::ArmPose * out_manifest_pose);
  bool detectTargetPoseInCamera(
    const cv::Mat & frame_bgr, cv::Mat & r_target_to_cam, cv::Mat & t_target_to_cam,
    double & out_mean_corner_reproj_px,
    std::string & fail_reason, cv::Mat & annotated, std::vector<int> & detected_ids);
  cv::Mat convertImageToBgr(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const;

  static cv::Mat quatToRot(double x, double y, double z, double w);
  static cv::Mat makeTransform(const cv::Mat & r, const cv::Mat & t);
  static void splitTransform(const cv::Mat & t4, cv::Mat & r, cv::Mat & t);
  static cv::Mat invertTransform(const cv::Mat & t4);
  static std::string nowString();
  static std::vector<double> mat44ToVec16(const cv::Mat & m);
  bool ensureTargetsPrepared();
  void applyTargetPosesForCurrentArm();
  void publishStatus(const std::string & status);
  void publishLog(const std::string & text);

  bool publishTargetPose(std::size_t idx);
  bool runCalibration();
  bool runEyeToHandCalibration(cv::Mat & t_cam_base);
  bool runEyeInHandCalibration(cv::Mat & t_cam_base, cv::Mat & t_cam_gripper);
  void computeHandEyeChainResiduals(
    const cv::Mat & t_cam_base,
    const cv::Mat & t_cam_gripper,
    bool has_cam_gripper,
    HandEyeQualityMetrics & out) const;
  bool saveResult(
    const cv::Mat & t_cam_base,
    const cv::Mat & t_base_cam,
    const HandEyeQualityMetrics & quality,
    const cv::Mat & t_cam_gripper,
    bool has_cam_gripper);

  void cancelInitDelayTimer();
  void republishLastCameraImagesToUi();
  void publishInitPoseAfterResetDelay();
  void publishKinematicsPoseGoal(const calib_sim_isaac::msg::ArmPose & target_pose);
  bool tryBuildJointCommandFromTargetPose(
    const calib_sim_isaac::msg::ArmPose & target_pose,
    sensor_msgs::msg::JointState & out_joint_cmd) const;
  bool loadIkModelFromUrdf();
  bool rebuildIkChainForActiveArm();
  bool solveIkForTargetPose(
    const calib_sim_isaac::msg::ArmPose & target_pose,
    sensor_msgs::msg::JointState & out_joint_cmd);
  bool tryGetLiveReferenceTcamBase(cv::Mat & out_t_cam_base_ref) const;
  mutable std::string last_ik_failure_reason_;
  bool isJointTargetReached(double * out_max_abs_err_rad) const;
  bool isPoseTargetReachedFromTf(double * out_pos_err_m, double * out_rot_err_deg) const;
  std::string formatJointStateCompact(const sensor_msgs::msg::JointState & js) const;
  std::string formatCurrentPoseCompact() const;

  bool unified_mode_{false};
  std::unordered_map<std::string, CalibConfigData> mode_configs_;
  std::string active_mode_;

  bool eye_in_hand_;
  int arm_id_;
  int marker_id_;
  int aruco_dict_id_;
  int min_samples_;
  double marker_length_m_;
  double topic_wait_timeout_sec_;
  double state_timeout_sec_;
  double reached_wait_timeout_sec_;
  double capture_wait_timeout_sec_;
  std::string image_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string robot_pose_topic_;
  std::string robot_state_topic_;
  std::string robot_target_topic_;
  std::string legacy_pose_command_topic_;
  std::string kinematics_pose_goal_topic_;
  std::string kinematics_joint_command_topic_;
  std::string ik_urdf_path_;
  std::string nova_all_joints_reset_topic_;
  std::string output_dir_;
  std::vector<double> target_poses_flat_;
  std::vector<double> target_poses_arm0_;
  std::vector<double> target_poses_arm1_;
  bool use_current_pose_as_center_;
  std::vector<double> target_position_offsets_;
  std::vector<double> target_orientation_offsets_rpy_deg_;
  bool use_known_target_mount_;
  std::vector<double> target_to_gripper_pose_;
  double marker_bbox_ratio_min_;
  double marker_bbox_ratio_max_;
  bool targets_prepared_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<calib_sim_isaac::msg::ArmPose>::SharedPtr arm_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ik_joint_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
  rclcpp::Publisher<calib_sim_isaac::msg::ArmPose>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr kinematics_pose_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr nova_all_joints_reset_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_text_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_image_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr init_after_reset_timer_;
  calib_sim_isaac::msg::ArmPose init_pose_pending_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool sample_pose_logged_source_{false};

  cv::aruco::Dictionary aruco_dict_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::string camera_frame_id_;
  cv::Mat last_raw_frame_;
  cv::Mat last_result_frame_;
  std::vector<cv::Mat> captured_raw_frames_;
  std::vector<cv::Mat> captured_result_frames_;
  std::vector<calib_sim_isaac::msg::ArmPose> captured_arm_poses_;
  bool has_camera_info_;
  bool has_arm_pose_;
  bool has_joint_state_;
  bool has_ik_joint_command_;
  bool has_arm_state_;
  bool arm_reached_;
  rclcpp::Time last_arm_state_time_;
  calib_sim_isaac::msg::ArmPose last_arm_pose_;
  sensor_msgs::msg::JointState last_joint_state_;
  sensor_msgs::msg::JointState last_ik_joint_command_;
  sensor_msgs::msg::JointState last_target_joint_command_;
  calib_sim_isaac::msg::ArmPose last_target_pose_command_;
  rclcpp::Time last_ik_joint_command_time_;
  bool has_last_target_joint_command_;
  bool has_last_target_pose_command_;
  bool ik_ready_;
  rclcpp::Time node_start_time_;
  bool warned_missing_camera_info_;
  bool warned_missing_robot_pose_;
  bool warned_missing_robot_state_;
  bool warned_no_joint_states_pub_;
  bool warned_no_joint_command_sub_;
  bool warned_domain_mismatch_hint_;

  std::vector<Sample> samples_;
  std::size_t target_index_;
  bool waiting_arm_reached_;
  bool waiting_capture_;
  bool finished_;
  rclcpp::Time target_sent_time_;
  rclcpp::Time capture_start_time_;
  bool auto_mode_;
  bool pending_step_;
  std::string last_detect_fail_reason_;
  bool waiting_init_pose_;
  std::string last_status_text_;
  std::unordered_map<int, calib_sim_isaac::msg::ArmPose> initial_pose_by_arm_;
  double known_mount_trans_consistency_m_;
  double known_mount_rot_consistency_deg_;
  bool use_tf_for_sample_pose_;
  bool use_joint_kinematics_interface_;
  bool enable_legacy_pose_fallback_;
  double ik_command_max_age_sec_;
  double joint_reach_tolerance_rad_;
  double pose_reach_position_tolerance_m_;
  double pose_reach_rotation_tolerance_deg_;
  /// 关节与 TF 均判定到位后，再等待该时间（秒）才进入采图，缓解仿真/控制滞后
  double post_reach_settle_sec_;
  /// 为 true：post_reach_settle 整段计时内每周期须关节也在容差内（与 TF 到位叠加，抑制「TF 已收敛、关节仍在微动」就采图）
  bool post_reach_settle_require_joint_;
  /// 为 true 时：关节与 TF 末端须同时满足才算到位（更严，易与仿真关节/模型不一致导致超时）
  bool reach_require_joint_and_tf_;
  /// 发目标后先经历该时间（秒）再开始计 arm 到位超时，避免指令尚未生效就判超时
  double arm_reach_timeout_grace_sec_;
  bool reach_settle_armed_{false};
  rclcpp::Time reach_settle_start_;
  /// 到位日志上升沿，避免 control 定时器每周期刷屏
  bool reach_log_joint_latched_{false};
  bool reach_log_pose_latched_{false};
  std::vector<std::string> joint_names_arm0_;
  std::vector<std::string> joint_names_arm1_;
  std::vector<std::string> active_joint_names_;
  std::string tf_base_frame_;
  std::string tf_ee_frame_arm0_;
  std::string tf_ee_frame_arm1_;
  double known_mount_quality_max_m_;
  int init_reset_burst_count_;
  int init_reset_burst_period_ms_;
  int init_delay_ms_after_reset_;
  int joint_command_burst_count_;

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
};

}  // namespace calib_sim_isaac

#endif  // CALIB_SIM__CALIB_NODE_HPP_
