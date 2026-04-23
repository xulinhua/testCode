#include "ros_robot_assist_tools/module/pose_transform_module.h"

#include <algorithm>
#include <cmath>

namespace ros_robot_assist_tools::ui
{
namespace
{
double clamp_val(double v) { return std::max(-1.0, std::min(1.0, v)); }
}

Quaternion QuaternionFromEuler(const EulerAngles & e, const QString & order)
{
  Quaternion q;
  const double c1 = std::cos(e.x / 2.0), c2 = std::cos(e.y / 2.0), c3 = std::cos(e.z / 2.0);
  const double s1 = std::sin(e.x / 2.0), s2 = std::sin(e.y / 2.0), s3 = std::sin(e.z / 2.0);
  if (order == "ZYX") {
    q.x = s1 * c2 * c3 - c1 * s2 * s3;
    q.y = c1 * s2 * c3 + s1 * c2 * s3;
    q.z = c1 * c2 * s3 - s1 * s2 * c3;
    q.w = c1 * c2 * c3 + s1 * s2 * s3;
  } else {
    q.x = s1 * c2 * c3 + c1 * s2 * s3;
    q.y = c1 * s2 * c3 - s1 * c2 * s3;
    q.z = c1 * c2 * s3 + s1 * s2 * c3;
    q.w = c1 * c2 * c3 - s1 * s2 * s3;
  }
  return NormalizeQuaternion(q);
}

Quaternion NormalizeQuaternion(const Quaternion & q)
{
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (n < 1e-12) { return {}; }
  Quaternion out;
  out.x = q.x / n;
  out.y = q.y / n;
  out.z = q.z / n;
  out.w = q.w / n;
  return out;
}

EulerAngles EulerFromQuaternionZYX(const Quaternion & q_in)
{
  const Quaternion q = NormalizeQuaternion(q_in);
  const double m11 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double m21 = 2.0 * (q.x * q.y + q.z * q.w);
  const double m31 = 2.0 * (q.x * q.z - q.y * q.w);
  const double m32 = 2.0 * (q.y * q.z + q.x * q.w);
  const double m33 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  EulerAngles e;
  e.x = std::atan2(m32, m33);
  e.y = std::asin(-clamp_val(m31));
  e.z = std::atan2(m21, m11);
  return e;
}

}  // namespace ros_robot_assist_tools::ui
