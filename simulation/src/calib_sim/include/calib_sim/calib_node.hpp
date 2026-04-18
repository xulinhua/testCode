#ifndef CALIB_SIM__CALIB_NODE_HPP_
#define CALIB_SIM__CALIB_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <string>
#include <unordered_map>
#include <vector>

#include "calib_sim/msg/arm_pose.hpp"
#include "calib_sim/config_data_manager.hpp"

namespace calib_sim
{

class CalibNode : public rclcpp::Node
{
public:
  explicit CalibNode(
    const std::string & node_name, bool eye_in_hand,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Sample
  {
    cv::Mat r_gripper_to_base;
    cv::Mat t_gripper_to_base;
    cv::Mat r_target_to_cam;
    cv::Mat t_target_to_cam;
    /// Mean |projected−detected| over 4 marker corners (pixels), from PnP self-check
    double mean_corner_reproj_px{0.0};
  };

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
  void initRosEntities();
  void controlTimerCallback();

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void armPoseCallback(const calib_sim::msg::ArmPose::ConstSharedPtr msg);
  void armStateCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void controlCallback(const std_msgs::msg::String::ConstSharedPtr msg);

  bool tryCaptureSample(
    const cv::Mat & frame_bgr, cv::Mat & annotated, std::vector<int> & detected_ids);
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

  void publishTargetPose(std::size_t idx);
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

private:
  bool eye_in_hand_;
  int arm_id_;
  int marker_id_;
  int aruco_dict_id_;
  int min_samples_;
  double marker_length_m_;
  double state_timeout_sec_;
  double reached_wait_timeout_sec_;
  double capture_wait_timeout_sec_;
  std::string image_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string robot_pose_topic_;
  std::string robot_state_topic_;
  std::string robot_target_topic_;
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
  rclcpp::Subscription<calib_sim::msg::ArmPose>::SharedPtr arm_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
  rclcpp::Publisher<calib_sim::msg::ArmPose>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr nova_all_joints_reset_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_text_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_image_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  cv::aruco::Dictionary aruco_dict_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Mat last_raw_frame_;
  cv::Mat last_result_frame_;
  std::vector<cv::Mat> captured_raw_frames_;
  std::vector<cv::Mat> captured_result_frames_;
  std::vector<calib_sim::msg::ArmPose> captured_arm_poses_;
  bool has_camera_info_;
  bool has_arm_pose_;
  bool has_arm_state_;
  bool arm_reached_;
  rclcpp::Time last_arm_state_time_;
  calib_sim::msg::ArmPose last_arm_pose_;

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
  std::unordered_map<int, calib_sim::msg::ArmPose> initial_pose_by_arm_;
  double known_mount_trans_consistency_m_;
  double known_mount_rot_consistency_deg_;
};

}  // namespace calib_sim

#endif  // CALIB_SIM__CALIB_NODE_HPP_
