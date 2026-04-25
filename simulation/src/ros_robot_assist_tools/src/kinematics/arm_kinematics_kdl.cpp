#include "ros_robot_assist_tools/kinematics/arm_kinematics_kdl.hpp"
#include "ros_robot_assist_tools/kinematics/mdh_craig1989.hpp"

#include <cmath>
#include <sstream>
#include <vector>

#include <Eigen/Geometry>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace ros_robot_assist_tools::kinematics
{
namespace
{
constexpr double kEps = 1e-9;

KDL::Vector NormalizeOr(const KDL::Vector & v, const KDL::Vector & fallback)
{
  if (v.Norm() > kEps) {
    return v / v.Norm();
  }
  if (fallback.Norm() > kEps) {
    return fallback / fallback.Norm();
  }
  return KDL::Vector(1.0, 0.0, 0.0);
}

KDL::Vector AnyPerpendicular(const KDL::Vector & z)
{
  KDL::Vector c = z * KDL::Vector(1.0, 0.0, 0.0);
  if (c.Norm() <= kEps) {
    c = z * KDL::Vector(0.0, 1.0, 0.0);
  }
  if (c.Norm() <= kEps) {
    c = KDL::Vector(1.0, 0.0, 0.0);
  }
  return c / c.Norm();
}

double SignedAngleAroundAxis(const KDL::Vector & a, const KDL::Vector & b, const KDL::Vector & axis)
{
  const KDL::Vector ax = NormalizeOr(axis, KDL::Vector(0.0, 0.0, 1.0));
  KDL::Vector ap = a - ax * KDL::dot(ax, a);
  KDL::Vector bp = b - ax * KDL::dot(ax, b);
  ap = NormalizeOr(ap, AnyPerpendicular(ax));
  bp = NormalizeOr(bp, AnyPerpendicular(ax));
  const double c = std::max(-1.0, std::min(1.0, KDL::dot(ap, bp)));
  const double s = KDL::dot(ax, ap * bp);
  return std::atan2(s, c);
}

struct ReconstructedFrame
{
  KDL::Vector p;
  KDL::Vector x;
  KDL::Vector y;
  KDL::Vector z;
};

KDL::Frame PoseFromArray(
  const std::array<double, 3> & p, const std::array<double, 4> & q_xyzw)
{
  // KDL::Rotation::Quaternion(x, y, z, w) — 与 (qx,qy,qz,qw) 常见顺序一致
  KDL::Rotation R = KDL::Rotation::Quaternion(
    q_xyzw[0], q_xyzw[1], q_xyzw[2], q_xyzw[3]);
  return KDL::Frame(R, KDL::Vector(p[0], p[1], p[2]));
}

MdhCraig1989TableRow::JointKind ClassifyKdlJoint(const KDL::Joint & kj)
{
  const KDL::Joint::JointType t = kj.getType();
  if (t == KDL::Joint::RotX || t == KDL::Joint::RotY || t == KDL::Joint::RotZ ||
    t == KDL::Joint::RotAxis) {
    return MdhCraig1989TableRow::JointKind::Revolute;
  }
  if (t == KDL::Joint::TransX || t == KDL::Joint::TransY || t == KDL::Joint::TransZ ||
    t == KDL::Joint::TransAxis) {
    return MdhCraig1989TableRow::JointKind::Prismatic;
  }
  return MdhCraig1989TableRow::JointKind::Other;
}

void FrameToPose(const KDL::Frame & f, std::array<double, 7> & out)
{
  Eigen::Matrix3d m;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      m(r, c) = f.M(r, c);
    }
  }
  Eigen::Quaterniond q(m);
  q.normalize();
  out[0] = f.p.x();
  out[1] = f.p.y();
  out[2] = f.p.z();
  out[3] = q.x();
  out[4] = q.y();
  out[5] = q.z();
  out[6] = q.w();
}

}  // namespace

class ArmKinematicsKdl::Impl
{
public:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> vel_ik_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> pos_ik_;
  KDL::JntArray q_min_;
  KDL::JntArray q_max_;
  std::vector<std::string> joint_names_;
  bool ready_{false};

  void Reset()
  {
    ready_ = false;
    joint_names_.clear();
    pos_ik_.reset();
    vel_ik_.reset();
    jac_.reset();
    fk_.reset();
  }

  bool Build(
    const std::string & urdf_path, const std::string & base_link, const std::string & tip_link, std::string & err)
  {
    Reset();
    urdf::Model model;
    if (!model.initFile(urdf_path)) {
      err = "urdf 无法解析: " + urdf_path;
      return false;
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
      err = "KDL 树无法从 URDF 建立";
      return false;
    }
    if (!tree.getChain(base_link, tip_link, chain_)) {
      std::ostringstream os;
      os << "无法取得链: base=" << base_link << " -> tip=" << tip_link
         << "（检查 link 名是否存在且连通）";
      err = os.str();
      return false;
    }
    if (chain_.getNrOfJoints() < 1) {
      err = "链的关节数为 0";
      return false;
    }
    const unsigned int nj = chain_.getNrOfJoints();
    q_min_ = KDL::JntArray(nj);
    q_max_ = KDL::JntArray(nj);
    joint_names_.assign(static_cast<size_t>(nj), std::string());
    // 默认给较宽范围，随后尽量用 URDF 关节真实 limit 覆盖。
    for (unsigned int i = 0; i < nj; ++i) {
      q_min_(i) = -1e6;
      q_max_(i) = 1e6;
    }
    unsigned int jidx = 0;
    for (unsigned int si = 0; si < chain_.getNrOfSegments(); ++si) {
      const KDL::Joint & kj = chain_.getSegment(si).getJoint();
      if (kj.getType() == KDL::Joint::None) {
        continue;
      }
      if (jidx >= nj) {
        break;
      }
      joint_names_[jidx] = kj.getName();
      const auto uj = model.getJoint(kj.getName());
      if (uj) {
        if (uj->type == urdf::Joint::CONTINUOUS) {
          q_min_(jidx) = -M_PI;
          q_max_(jidx) = M_PI;
        } else if (uj->limits) {
          q_min_(jidx) = uj->limits->lower;
          q_max_(jidx) = uj->limits->upper;
        }
      }
      ++jidx;
    }
    fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    jac_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    vel_ik_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_, 1e-5, 200);
    const int max_iter = 200;
    const double eps = 1e-5;
    pos_ik_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
      chain_, q_min_, q_max_, *fk_, *vel_ik_, max_iter, eps);
    ready_ = true;
    return true;
  }
};

ArmKinematicsKdl::ArmKinematicsKdl()
: impl_(std::make_unique<Impl>())
{
}

ArmKinematicsKdl::~ArmKinematicsKdl() = default;

bool ArmKinematicsKdl::LoadFromFile(
  const std::string & urdf_path, const std::string & base_link, const std::string & tip_link, std::string & err)
{
  return impl_->Build(urdf_path, base_link, tip_link, err);
}

size_t ArmKinematicsKdl::GetJointCount() const
{
  if (!impl_->ready_) {
    return 0;
  }
  return static_cast<size_t>(impl_->chain_.getNrOfJoints());
}

std::string ArmKinematicsKdl::GetJointName(size_t index) const
{
  if (!impl_->ready_ || index >= impl_->joint_names_.size()) {
    return {};
  }
  return impl_->joint_names_[index];
}

bool ArmKinematicsKdl::GetJointLimits(size_t index, double & lo, double & hi) const
{
  if (!impl_->ready_ || index >= static_cast<size_t>(impl_->chain_.getNrOfJoints())) {
    return false;
  }
  const unsigned int i = static_cast<unsigned int>(index);
  lo = impl_->q_min_(i);
  hi = impl_->q_max_(i);
  return true;
}

bool ArmKinematicsKdl::Forward(
  const std::vector<double> & q, std::array<double, 7> & pose_out, std::string & err) const
{
  if (!impl_->ready_ || !impl_->fk_) {
    err = "未加载成功";
    return false;
  }
  const int nj = static_cast<int>(GetJointCount());
  if (static_cast<int>(q.size()) != nj) {
    err = "关节数不匹配，需 " + std::to_string(nj) + " 个值";
    return false;
  }
  KDL::JntArray j(nj);
  for (int i = 0; i < nj; ++i) {
    j(i) = q[static_cast<size_t>(i)];
  }
  KDL::Frame f;
  const int e = impl_->fk_->JntToCart(j, f);
  if (e < 0) {
    err = "KDL 正解失败, code " + std::to_string(e);
    return false;
  }
  FrameToPose(f, pose_out);
  return true;
}

bool ArmKinematicsKdl::Inverse(
  const std::vector<double> & seed, const std::array<double, 3> & pos, const std::array<double, 4> & quat_xyzw,
  std::vector<double> & q_out, std::string & err)
{
  if (!impl_->ready_ || !impl_->pos_ik_) {
    err = "未加载成功";
    return false;
  }
  const int nj = static_cast<int>(GetJointCount());
  if (static_cast<int>(seed.size()) != nj) {
    err = "初值关节数需为 " + std::to_string(nj);
    return false;
  }
  KDL::JntArray q_in(nj);
  for (int i = 0; i < nj; ++i) {
    const double lo = impl_->q_min_(i);
    const double hi = impl_->q_max_(i);
    double s = seed[static_cast<size_t>(i)];
    if (!std::isfinite(s)) {
      s = 0.0;
    }
    if (s < lo) {
      s = lo;
    }
    if (s > hi) {
      s = hi;
    }
    q_in(i) = s;
  }
  KDL::JntArray q_res(nj);
  const KDL::Frame f_des = PoseFromArray(pos, quat_xyzw);
  int best_code = -999;
  bool solved = false;
  auto try_one = [&](const KDL::JntArray & guess) {
    const int code = impl_->pos_ik_->CartToJnt(guess, f_des, q_res);
    if (code >= 0) {
      solved = true;
      return true;
    }
    best_code = code;
    return false;
  };
  if (!try_one(q_in)) {
    // 多初值重试：缓解零初值奇异点/局部极值导致的不收敛问题
    std::vector<KDL::JntArray> guesses;
    auto make_guess = [&](double ratio) {
      KDL::JntArray g(nj);
      for (int i = 0; i < nj; ++i) {
        const double lo = impl_->q_min_(i);
        const double hi = impl_->q_max_(i);
        const double c = 0.5 * (lo + hi);
        const double h = 0.5 * (hi - lo);
        g(i) = c + ratio * h;
      }
      return g;
    };
    guesses.push_back(make_guess(0.0));
    guesses.push_back(make_guess(0.15));
    guesses.push_back(make_guess(-0.15));
    guesses.push_back(make_guess(0.30));
    guesses.push_back(make_guess(-0.30));
    for (const auto & g : guesses) {
      if (try_one(g)) {
        break;
      }
    }
  }
  if (!solved) {
    err = "KDL 逆解不收敛（多初值重试后仍失败）, code " + std::to_string(best_code);
    return false;
  }
  // 二次校验：若超限或出现非有限值，直接报异常而不是返回不可用结果。
  for (int i = 0; i < nj; ++i) {
    const double v = q_res(i);
    if (!std::isfinite(v)) {
      err = "KDL 逆解结果异常: 关节 " + std::to_string(i) + " 非有限值";
      return false;
    }
    const double lo = impl_->q_min_(i);
    const double hi = impl_->q_max_(i);
    if (v < lo - 1e-6 || v > hi + 1e-6) {
      std::ostringstream os;
      os << "KDL 逆解超限: joint[" << i << "]=" << v << " 不在 [" << lo << ", " << hi
         << "] 内（已启用 URDF limit）";
      err = os.str();
      return false;
    }
  }
  q_out.resize(static_cast<size_t>(nj));
  for (int i = 0; i < nj; ++i) {
    q_out[static_cast<size_t>(i)] = q_res(i);
  }
  return true;
}

bool ArmKinematicsKdl::GetMdhCraig1989Table(
  std::vector<MdhCraig1989TableRow> & out, std::string & err, DhConvention convention) const
{
  out.clear();
  if (!impl_->ready_) {
    err = "未加载成功";
    return false;
  }
  const KDL::Chain & ch = impl_->chain_;
  const size_t nj = GetJointCount();
  if (nj < 1) {
    err = "链上无活动关节";
    return false;
  }
  // 1) 在 q=0 下提取每个活动关节在 base 坐标系中的原点与轴。
  std::vector<KDL::Vector> joint_origins;
  std::vector<KDL::Vector> joint_axes;
  std::vector<std::string> joint_names;
  std::vector<MdhCraig1989TableRow::JointKind> joint_kinds;
  joint_origins.reserve(nj);
  joint_axes.reserve(nj);
  joint_names.reserve(nj);
  joint_kinds.reserve(nj);

  KDL::Frame f_base_root = KDL::Frame::Identity();
  for (unsigned int si = 0; si < ch.getNrOfSegments(); ++si) {
    const KDL::Segment & seg = ch.getSegment(si);
    const KDL::Joint & kj = seg.getJoint();
    if (kj.getType() != KDL::Joint::Fixed) {
      const KDL::Vector o_local = kj.JointOrigin();
      const KDL::Vector z_local = kj.JointAxis();
      const KDL::Vector o_base = f_base_root.p + f_base_root.M * o_local;
      const KDL::Vector z_base = NormalizeOr(f_base_root.M * z_local, KDL::Vector(0.0, 0.0, 1.0));
      joint_origins.push_back(o_base);
      joint_axes.push_back(z_base);
      joint_names.push_back(kj.getName());
      joint_kinds.push_back(ClassifyKdlJoint(kj));
    }
    f_base_root = f_base_root * seg.pose(0.0);
  }
  if (joint_origins.size() != nj) {
    err = "活动关节提取数量异常";
    return false;
  }

  // 2) 取 q=0 末端位姿，作为最后一个重建帧的原点。
  KDL::Frame f_tip0;
  KDL::JntArray q0(static_cast<unsigned int>(nj));
  for (size_t i = 0; i < nj; ++i) {
    q0(static_cast<unsigned int>(i)) = 0.0;
  }
  if (!impl_->fk_ || impl_->fk_->JntToCart(q0, f_tip0) < 0) {
    err = "无法计算 q=0 末端位姿";
    return false;
  }

  // 3) 全链一致重建帧：frame[0..nj]，其中 frame[i] 对应第 i+1 关节附近坐标系，最后一个在 tip。
  std::vector<ReconstructedFrame> frames(nj + 1);
  for (size_t i = 0; i < nj; ++i) {
    frames[i].p = joint_origins[i];
    frames[i].z = joint_axes[i];
  }
  frames[nj].p = f_tip0.p;
  frames[nj].z = frames[nj - 1].z;

  for (size_t i = 0; i <= nj; ++i) {
    KDL::Vector x;
    if (i < nj) {
      KDL::Vector delta = frames[i + 1].p - frames[i].p;
      // 优先使用相邻原点连线在法平面内的投影，保证全链方向一致。
      x = delta - frames[i].z * KDL::dot(frames[i].z, delta);
      if (x.Norm() <= kEps && i + 1 < nj) {
        const KDL::Vector n = frames[i].z * frames[i + 1].z;
        x = n * frames[i].z;
      }
    } else {
      x = frames[i - 1].x;
    }
    if (x.Norm() <= kEps) {
      if (i > 0) {
        KDL::Vector xp = frames[i - 1].x - frames[i].z * KDL::dot(frames[i].z, frames[i - 1].x);
        x = xp;
      }
    }
    frames[i].x = NormalizeOr(x, AnyPerpendicular(frames[i].z));
    frames[i].y = NormalizeOr(frames[i].z * frames[i].x, KDL::Vector(0.0, 1.0, 0.0));
    // 重新正交化 x，防止数值漂移
    frames[i].x = NormalizeOr(frames[i].y * frames[i].z, frames[i].x);
  }

  // 4) 由重建帧计算每一行参数（Craig/MDH 记号：alpha_{i-1}, a_{i-1}, d_i, theta_i）。
  out.reserve(nj);
  for (size_t i = 1; i <= nj; ++i) {
    const auto & fm1 = frames[i - 1];
    const auto & fi = frames[i];
    const KDL::Vector dp = fi.p - fm1.p;
    MdhCraig1989TableRow row;
    row.i_1based = static_cast<int>(i);
    row.joint_name = joint_names[i - 1];
    row.kind = joint_kinds[i - 1];
    row.fit_ok = true;
    row.a_im1 = KDL::dot(fm1.x, dp);
    if (convention == DhConvention::ModifiedCraig1989) {
      row.d_i = KDL::dot(fi.z, dp);
    } else {
      row.d_i = KDL::dot(fm1.z, dp);
    }
    row.alpha_im1 = SignedAngleAroundAxis(fm1.z, fi.z, fm1.x);
    if (convention == DhConvention::ModifiedCraig1989) {
      row.theta_offset = SignedAngleAroundAxis(fm1.x, fi.x, fi.z);
    } else {
      row.theta_offset = SignedAngleAroundAxis(fm1.x, fi.x, fm1.z);
    }

    // 保留回退信息（便于 UI 复用展示）
    row.fallback_x = dp.x();
    row.fallback_y = dp.y();
    row.fallback_z = dp.z();
    row.fallback_roll = row.alpha_im1;
    row.fallback_pitch = 0.0;
    row.fallback_yaw = row.theta_offset;
    out.push_back(std::move(row));
  }

  err.clear();
  return true;
}

}  // namespace ros_robot_assist_tools::kinematics
