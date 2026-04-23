// 仿真桥：将 calib_sim 的 ArmPose 目标转为 nova 的 PoseStamped + arm_id，并反馈位姿/到位/误差。
#include <chrono>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "calib_sim_mujoco/msg/arm_pose.hpp"

/// 连接标定节点与 nova_sim 执行器：定时发布当前末端位姿（TF 或内部状态）及到达判定。
class CalibSimBridgeNode : public rclcpp::Node
{
public:
  explicit CalibSimBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("calib_sim_bridge_node", options), feedback_delay_ms_(800)
  {
    this->declare_parameter("calib_target_topic", std::string("/robot_target_pose"));
    this->declare_parameter("calib_pose_feedback_topic", std::string("/robot_pose"));
    this->declare_parameter("calib_reached_topic", std::string("/robot_reached"));
    this->declare_parameter("feedback_delay_ms", feedback_delay_ms_);

    this->declare_parameter("nova_arm_id_topic", std::string("/nova_arm_id"));
    this->declare_parameter("nova_target_pose_topic", std::string("/nova_target_pose"));
    this->declare_parameter("initial_arm_id", 0);
    this->declare_parameter(
      "initial_pose", std::vector<double>{0.30, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0});
    this->declare_parameter("pose_publish_hz", 10.0);
    this->declare_parameter("use_tf_current_pose", true);
    this->declare_parameter("base_frame", std::string("base_link"));
    this->declare_parameter("ee_frame_arm0", std::string("J1_6"));
    this->declare_parameter("ee_frame_arm1", std::string("J2_6"));
    this->declare_parameter("ee_frame_arm2", std::string("J3_6"));
    this->declare_parameter("ee_frame_arm3", std::string("J4_6"));
    this->declare_parameter("reach_pos_tol_m", 0.015);
    this->declare_parameter("reach_ang_tol_rad", 0.15);
    this->declare_parameter("max_feedback_wait_ms", 5000);

    calib_target_topic_ = this->get_parameter("calib_target_topic").as_string();
    calib_pose_feedback_topic_ = this->get_parameter("calib_pose_feedback_topic").as_string();
    calib_reached_topic_ = this->get_parameter("calib_reached_topic").as_string();
    feedback_delay_ms_ = this->get_parameter("feedback_delay_ms").as_int();
    nova_arm_id_topic_ = this->get_parameter("nova_arm_id_topic").as_string();
    nova_target_pose_topic_ = this->get_parameter("nova_target_pose_topic").as_string();
    initial_arm_id_ = this->get_parameter("initial_arm_id").as_int();
    initial_pose_ = this->get_parameter("initial_pose").as_double_array();
    pose_publish_hz_ = this->get_parameter("pose_publish_hz").as_double();
    use_tf_current_pose_ = this->get_parameter("use_tf_current_pose").as_bool();
    base_frame_ = this->get_parameter("base_frame").as_string();
    ee_frame_by_arm_[0] = this->get_parameter("ee_frame_arm0").as_string();
    ee_frame_by_arm_[1] = this->get_parameter("ee_frame_arm1").as_string();
    ee_frame_by_arm_[2] = this->get_parameter("ee_frame_arm2").as_string();
    ee_frame_by_arm_[3] = this->get_parameter("ee_frame_arm3").as_string();
    reach_pos_tol_m_ = this->get_parameter("reach_pos_tol_m").as_double();
    reach_ang_tol_rad_ = this->get_parameter("reach_ang_tol_rad").as_double();
    max_feedback_wait_ms_ = this->get_parameter("max_feedback_wait_ms").as_int();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    target_sub_ = this->create_subscription<calib_sim_mujoco::msg::ArmPose>(
      calib_target_topic_, 20,
      std::bind(&CalibSimBridgeNode::onCalibTarget, this, std::placeholders::_1));

    nova_arm_id_pub_ = this->create_publisher<std_msgs::msg::Int32>(nova_arm_id_topic_, 20);
    nova_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(nova_target_pose_topic_, 20);
    calib_pose_pub_ = this->create_publisher<calib_sim_mujoco::msg::ArmPose>(calib_pose_feedback_topic_, 20);
    calib_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>(calib_reached_topic_, 20);
    reach_error_pub_ = this->create_publisher<std_msgs::msg::String>("/calib_sim/reach_error", 20);
    feedback_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CalibSimBridgeNode::onFeedbackTimer, this));
    const int pose_period_ms = static_cast<int>(1000.0 / std::max(1.0, pose_publish_hz_));
    pose_pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(pose_period_ms),
      std::bind(&CalibSimBridgeNode::onPosePubTimer, this));

    RCLCPP_INFO(get_logger(), "calib_sim bridge ready.");
    RCLCPP_INFO(get_logger(), "  calib target: %s", calib_target_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  calib pose feedback: %s", calib_pose_feedback_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  calib reached: %s", calib_reached_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  nova arm id: %s", nova_arm_id_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  nova target pose: %s", nova_target_pose_topic_.c_str());
    if (!tryUpdatePoseFromTf(initial_arm_id_)) {
      publishInitialPose();
    } else {
      RCLCPP_INFO(get_logger(), "Initialized pose from TF for arm_id=%d", initial_arm_id_);
    }
  }

private:
  void onCalibTarget(const calib_sim_mujoco::msg::ArmPose::SharedPtr msg)
  {
    std_msgs::msg::Int32 arm_id_msg;
    arm_id_msg.data = msg->arm_id;
    nova_arm_id_pub_->publish(arm_id_msg);
    nova_target_pub_->publish(msg->pose);

    std_msgs::msg::Bool reached_false;
    reached_false.data = false;
    calib_reached_pub_->publish(reached_false);

    PendingFeedback pending;
    pending.target_pose = *msg;
    pending.start_time = this->now();
    pending.next_check_time = this->now() + rclcpp::Duration::from_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(feedback_delay_ms_)).count());
    pending_feedback_.push_back(pending);

    RCLCPP_INFO(
      get_logger(), "Forward calib target to nova executor. arm_id=%d", msg->arm_id);
  }

  void onFeedbackTimer()
  {
    if (pending_feedback_.empty()) {
      return;
    }
    const auto now_time = this->now();
    std::vector<PendingFeedback> remaining;
    remaining.reserve(pending_feedback_.size());
    for (const auto & item : pending_feedback_) {
      if (now_time >= item.next_check_time) {
        if (use_tf_current_pose_) {
          (void)tryUpdatePoseFromTf(item.target_pose.arm_id);
        }
        const bool reached = has_current_pose_ &&
          isPoseReached(current_pose_.pose.pose, item.target_pose.pose.pose);
        const bool timeout = (now_time - item.start_time).nanoseconds() >=
          std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::milliseconds(max_feedback_wait_ms_)).count();
        if (reached) {
          calib_pose_pub_->publish(current_pose_);
          std_msgs::msg::Bool reached_true;
          reached_true.data = true;
          calib_reached_pub_->publish(reached_true);
        } else if (timeout) {
          std_msgs::msg::Bool reached_false;
          reached_false.data = false;
          calib_reached_pub_->publish(reached_false);
          RCLCPP_WARN(
            get_logger(),
            "Reach timeout arm_id=%d, keep reached=false and skip this target",
            item.target_pose.arm_id);
        } else {
          PendingFeedback next = item;
          next.next_check_time = now_time + rclcpp::Duration::from_nanoseconds(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::milliseconds(120)).count());
          remaining.push_back(next);
        }
      } else {
        remaining.push_back(item);
      }
    }
    pending_feedback_.swap(remaining);
  }

  bool isPoseReached(
    const geometry_msgs::msg::Pose & current,
    const geometry_msgs::msg::Pose & target) const
  {
    const double dx = current.position.x - target.position.x;
    const double dy = current.position.y - target.position.y;
    const double dz = current.position.z - target.position.z;
    const double pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);

    const double dot = std::abs(
      current.orientation.x * target.orientation.x +
      current.orientation.y * target.orientation.y +
      current.orientation.z * target.orientation.z +
      current.orientation.w * target.orientation.w);
    const double clamped_dot = std::max(-1.0, std::min(1.0, dot));
    const double ang_err = 2.0 * std::acos(clamped_dot);

    if (reach_error_pub_) {
      std_msgs::msg::String msg;
      constexpr double kRad2Deg = 57.29577951308232;
      std::ostringstream oss;
      oss << "reach_err pos_mm=" << (pos_err * 1000.0) << " ang_deg=" << (ang_err * kRad2Deg);
      msg.data = oss.str();
      reach_error_pub_->publish(msg);
    }
    return (pos_err <= reach_pos_tol_m_) && (ang_err <= reach_ang_tol_rad_);
  }

  void publishInitialPose()
  {
    if (initial_pose_.size() != 7U) {
      RCLCPP_WARN(
        get_logger(), "initial_pose size is %zu, expected 7. skip initial pose publish.",
        initial_pose_.size());
      return;
    }
    calib_sim_mujoco::msg::ArmPose init_msg;
    init_msg.arm_id = initial_arm_id_;
    init_msg.pose.header.stamp = this->now();
    init_msg.pose.header.frame_id = "base_link";
    init_msg.pose.pose.position.x = initial_pose_[0];
    init_msg.pose.pose.position.y = initial_pose_[1];
    init_msg.pose.pose.position.z = initial_pose_[2];
    init_msg.pose.pose.orientation.x = initial_pose_[3];
    init_msg.pose.pose.orientation.y = initial_pose_[4];
    init_msg.pose.pose.orientation.z = initial_pose_[5];
    init_msg.pose.pose.orientation.w = initial_pose_[6];
    calib_pose_pub_->publish(init_msg);
    current_pose_ = init_msg;
    has_current_pose_ = true;
    RCLCPP_INFO(get_logger(), "Published initial calib pose. arm_id=%d", initial_arm_id_);
  }

  void onPosePubTimer()
  {
    if (use_tf_current_pose_ && has_current_pose_) {
      (void)tryUpdatePoseFromTf(current_pose_.arm_id);
    }
    if (!has_current_pose_) {
      return;
    }
    current_pose_.pose.header.stamp = this->now();
    calib_pose_pub_->publish(current_pose_);
  }

  bool tryUpdatePoseFromTf(int arm_id)
  {
    if (!tf_buffer_ || ee_frame_by_arm_.count(arm_id) == 0) {
      return false;
    }
    const auto & ee_frame = ee_frame_by_arm_[arm_id];
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(base_frame_, ee_frame, tf2::TimePointZero);
    } catch (const std::exception &) {
      return false;
    }

    calib_sim_mujoco::msg::ArmPose pose_msg;
    pose_msg.arm_id = arm_id;
    pose_msg.pose.header.stamp = this->now();
    pose_msg.pose.header.frame_id = base_frame_;
    pose_msg.pose.pose.position.x = tf_msg.transform.translation.x;
    pose_msg.pose.pose.position.y = tf_msg.transform.translation.y;
    pose_msg.pose.pose.position.z = tf_msg.transform.translation.z;
    pose_msg.pose.pose.orientation = tf_msg.transform.rotation;
    current_pose_ = pose_msg;
    has_current_pose_ = true;
    return true;
  }

private:
  struct PendingFeedback
  {
    calib_sim_mujoco::msg::ArmPose target_pose;
    rclcpp::Time start_time;
    rclcpp::Time next_check_time;
  };

  int feedback_delay_ms_;
  std::string calib_target_topic_;
  std::string calib_pose_feedback_topic_;
  std::string calib_reached_topic_;
  std::string nova_arm_id_topic_;
  std::string nova_target_pose_topic_;
  int initial_arm_id_;
  std::vector<double> initial_pose_;
  double pose_publish_hz_;
  bool use_tf_current_pose_{true};
  std::string base_frame_{"base_link"};
  std::map<int, std::string> ee_frame_by_arm_;
  double reach_pos_tol_m_{0.015};
  double reach_ang_tol_rad_{0.15};
  int max_feedback_wait_ms_{5000};
  bool has_current_pose_{false};
  calib_sim_mujoco::msg::ArmPose current_pose_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<calib_sim_mujoco::msg::ArmPose>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr nova_arm_id_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nova_target_pub_;
  rclcpp::Publisher<calib_sim_mujoco::msg::ArmPose>::SharedPtr calib_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calib_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reach_error_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr pose_pub_timer_;
  std::vector<PendingFeedback> pending_feedback_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto pkg_share = ament_index_cpp::get_package_share_directory("nova_sim_mujoco");
  const auto config_path = pkg_share + "/config/calib_sim_bridge.yaml";
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", config_path});
  auto node = std::make_shared<CalibSimBridgeNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
