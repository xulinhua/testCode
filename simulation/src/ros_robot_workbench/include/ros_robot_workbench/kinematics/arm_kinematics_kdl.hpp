#ifndef ROS_ROBOT_WORKBENCH__KINEMATICS__ARM_KINEMATICS_KDL_HPP_
#define ROS_ROBOT_WORKBENCH__KINEMATICS__ARM_KINEMATICS_KDL_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace ros_robot_workbench::kinematics
{

enum class DhConvention
{
  ModifiedCraig1989,  // MDH
  Standard1955        // DH
};

/// 每行：Craig(1989) MDH 表一行（KDL 段 q=0 时拟合；关节变量列在 UI 中单独标注）。
struct MdhCraig1989TableRow
{
  int i_1based{1};
  std::string joint_name;
  enum class JointKind { Revolute, Prismatic, Other };
  JointKind kind{JointKind::Other};
  bool fit_ok{false};
  double alpha_im1{0.0};
  double a_im1{0.0};
  double d_i{0.0};
  double theta_offset{0.0};
  std::string fit_error;
  // 当 MDH 拟合失败时，回退展示段变换的 xyz+rpy（q=0）。
  double fallback_x{0.0};
  double fallback_y{0.0};
  double fallback_z{0.0};
  double fallback_roll{0.0};
  double fallback_pitch{0.0};
  double fallback_yaw{0.0};
};

/// 由 URDF 链构建的 KDL：正解（闭式）与逆解（6 位姿数值迭代，与常见工业用法一致）。
class ArmKinematicsKdl
{
public:
  ArmKinematicsKdl();
  ~ArmKinematicsKdl();
  bool LoadFromFile(
    const std::string & urdf_path, const std::string & base_link, const std::string & tip_link,
    std::string & err);

  size_t GetJointCount() const;
  /// 第 i 个活动关节名（与正解/逆解向量下标一致）
  std::string GetJointName(size_t index) const;
  /// URDF 限位，若无则取链上较宽范围
  bool GetJointLimits(size_t index, double & lo, double & hi) const;
  /// 前向：关节角 → 位姿 (px,py,pz, qx, qy, qz, qw)
  bool Forward(const std::vector<double> & q, std::array<double, 7> & pose_out, std::string & err) const;
  /// 反向：在 seed 附近数值求 IK
  bool Inverse(
    const std::vector<double> & seed, const std::array<double, 3> & pos, const std::array<double, 4> & quat_xyzw,
    std::vector<double> & q_out, std::string & err);
  /// 由全链一致重建坐标系生成 DH/MDH 参数表。
  bool GetMdhCraig1989Table(
    std::vector<MdhCraig1989TableRow> & out, std::string & err,
    DhConvention convention = DhConvention::ModifiedCraig1989) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros_robot_workbench::kinematics

#endif  // ROS_ROBOT_WORKBENCH__KINEMATICS__ARM_KINEMATICS_KDL_HPP_
