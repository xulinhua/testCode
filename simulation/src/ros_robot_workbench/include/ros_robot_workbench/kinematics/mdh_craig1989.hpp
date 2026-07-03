#ifndef ROS_ROBOT_WORKBENCH__KINEMATICS__MDH_CRAIG1989_HPP_
#define ROS_ROBOT_WORKBENCH__KINEMATICS__MDH_CRAIG1989_HPP_

#include <kdl/frames.hpp>
#include <string>

namespace ros_robot_workbench::kinematics
{

/// 将 KDL 段相对位姿 (q=0) 在数值上拟合为 Craig (1989) 单行 Modified DH
/// 参数顺序与 orocos 一致：a, α, d, θ 对应 T_{i-1}^{i} = {see KDL::Frame::DH_Craig1989}。
/// 对任意 4R 刚体不保证唯一/可表；不收敛时返回 false。
bool FitCraigMdhToFrame(
  const KDL::Frame & t_link_rel, double * a, double * alpha, double * d, double * theta,
  std::string * err_msg);

}  // namespace ros_robot_workbench::kinematics

#endif  // ROS_ROBOT_WORKBENCH__KINEMATICS__MDH_CRAIG1989_HPP_
