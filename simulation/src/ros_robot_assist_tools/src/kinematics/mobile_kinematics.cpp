#include "ros_robot_assist_tools/kinematics/mobile_kinematics.hpp"

#include <cmath>
#include <limits>
#include <sstream>

namespace ros_robot_assist_tools::kinematics
{

bool DiffDriveKinematics::WheelToBody(
  double wheel_base_m, double wheel_radius_m, double w_left_rad, double w_right_rad, double & v_out,
  double & omega_out, std::string * err)
{
  if (wheel_base_m <= 0.0 || wheel_radius_m <= 0.0) {
    if (err) {
      *err = "L 与 r 需为正数";
    }
    return false;
  }
  const double vl = w_left_rad * wheel_radius_m;
  const double vr = w_right_rad * wheel_radius_m;
  v_out = 0.5 * (vr + vl);
  omega_out = (vr - vl) / wheel_base_m;
  return true;
}

bool DiffDriveKinematics::BodyToWheel(
  double wheel_base_m, double wheel_radius_m, double v, double omega_rad, double & w_left_rad,
  double & w_right_rad, std::string * err)
{
  if (wheel_base_m <= 0.0 || wheel_radius_m <= 0.0) {
    if (err) {
      *err = "L 与 r 需为正数";
    }
    return false;
  }
  const double vl = v - 0.5 * omega_rad * wheel_base_m;
  const double vr = v + 0.5 * omega_rad * wheel_base_m;
  w_left_rad = vl / wheel_radius_m;
  w_right_rad = vr / wheel_radius_m;
  return true;
}

bool AckermannBicycleKinematics::Geometry(
  double wheelbase_m, double delta_rad, double * turning_radius_m, double * curvature_1_m, std::string * err)
{
  if (wheelbase_m <= 0.0) {
    if (err) {
      *err = "轴距 L 需为正数";
    }
    return false;
  }
  const double td = std::tan(delta_rad);
  if (std::abs(td) < 1e-9) {
    if (turning_radius_m) {
      *turning_radius_m = std::numeric_limits<double>::infinity();
    }
    if (curvature_1_m) {
      *curvature_1_m = 0.0;
    }
    return true;
  }
  const double R = wheelbase_m / td;
  if (turning_radius_m) {
    *turning_radius_m = R;
  }
  if (curvature_1_m) {
    *curvature_1_m = 1.0 / R;
  }
  return true;
}

bool AckermannBicycleKinematics::InverseFromCurvature(
  double wheelbase_m, double curvature_1_m, double * delta_rad, double * turning_radius_m, std::string * err)
{
  if (wheelbase_m <= 0.0) {
    if (err) {
      *err = "轴距 L 需为正数";
    }
    return false;
  }
  if (!delta_rad) {
    if (err) {
      *err = "delta 输出指针为空";
    }
    return false;
  }
  if (std::abs(curvature_1_m) < 1e-12) {
    *delta_rad = 0.0;
    if (turning_radius_m) {
      *turning_radius_m = std::numeric_limits<double>::infinity();
    }
    return true;
  }
  const double R = 1.0 / curvature_1_m;
  *delta_rad = std::atan(wheelbase_m * curvature_1_m);
  if (turning_radius_m) {
    *turning_radius_m = R;
  }
  return true;
}

}  // namespace ros_robot_assist_tools::kinematics
