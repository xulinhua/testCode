#ifndef ROS_ROBOT_ASSIST_TOOLS__KINEMATICS__MOBILE_KINEMATICS_HPP_
#define ROS_ROBOT_ASSIST_TOOLS__KINEMATICS__MOBILE_KINEMATICS_HPP_

#include <string>

namespace ros_robot_assist_tools::kinematics
{

/// 差速：轮心线速度 w_l, w_r (rad/s)，轮距 L (m)、轮半径 r (m) → 车体质心前向 v (m/s)、角速度 omega (rad/s)
struct DiffDriveKinematics
{
  static bool WheelToBody(
    double wheel_base_m, double wheel_radius_m, double w_left_rad, double w_right_rad, double & v_out,
    double & omega_out, std::string * err);

  static bool BodyToWheel(
    double wheel_base_m, double wheel_radius_m, double v, double omega_rad, double & w_left_rad,
    double & w_right_rad, std::string * err);
};

/// 阿克曼（单轨自行车模型）：轴距 L, 等效前轮转角 delta (rad) → 转弯半径、曲率等
struct AckermannBicycleKinematics
{
  static bool Geometry(
    double wheelbase_m, double delta_rad, double * turning_radius_m, double * curvature_1_m,
    std::string * err);

  static bool InverseFromCurvature(
    double wheelbase_m, double curvature_1_m, double * delta_rad, double * turning_radius_m,
    std::string * err);
};

}  // namespace ros_robot_assist_tools::kinematics

#endif  // ROS_ROBOT_ASSIST_TOOLS__KINEMATICS__MOBILE_KINEMATICS_HPP_
