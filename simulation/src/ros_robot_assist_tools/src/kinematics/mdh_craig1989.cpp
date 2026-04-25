#include "ros_robot_assist_tools/kinematics/mdh_craig1989.hpp"

#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <kdl/frames.hpp>

namespace ros_robot_assist_tools::kinematics
{
namespace
{

double NormTwist6(const KDL::Twist & t)
{
  double s = 0.0;
  for (int k = 0; k < 6; ++k) {
    const double c = t(k);
    s += c * c;
  }
  return std::sqrt(s);
}

void TwistToEigen6(const KDL::Twist & t, Eigen::Ref<Eigen::Matrix<double, 6, 1>> out)
{
  for (int k = 0; k < 6; ++k) {
    out(k) = t(k);
  }
}

}  // namespace

bool FitCraigMdhToFrame(
  const KDL::Frame & t_link_rel, double * a, double * alpha, double * d, double * theta,
  std::string * err_msg)
{
  if (!a || !alpha || !d || !theta) {
    if (err_msg) {
      *err_msg = "空指针";
    }
    return false;
  }
  const int n_seeds = 6;
  const double seeds[n_seeds][4] = {
    {0.0, 0.0, 0.0, 0.0},
    {0.1, 0.0, 0.0, 0.0},
    {0.0, 0.1, 0.0, 0.0},
    {0.0, 0.0, 0.1, 0.0},
    {0.0, 0.0, 0.0, 0.1},
    {0.0, 0.2, 0.1, 0.0},
  };

  const double eps = 1e-7;
  const int max_iter = 80;
  const double h = 1e-5;
  const double base_lambda = 1e-3;

  for (int s = 0; s < n_seeds; ++s) {
    double x0 = seeds[s][0];
    double x1 = seeds[s][1];
    double x2 = seeds[s][2];
    double x3 = seeds[s][3];
    for (int it = 0; it < max_iter; ++it) {
      KDL::Frame f = KDL::Frame::DH_Craig1989(x0, x1, x2, x3);
      KDL::Twist e = KDL::diff(f, t_link_rel);
      const double en = NormTwist6(e);
      if (en < eps) {
        *a = x0;
        *alpha = x1;
        *d = x2;
        *theta = x3;
        if (err_msg) {
          err_msg->clear();
        }
        return true;
      }
      Eigen::Matrix<double, 6, 1> evec;
      TwistToEigen6(e, evec);
      Eigen::Matrix<double, 6, 4> j;
      for (int jcol = 0; jcol < 4; ++jcol) {
        double s0 = x0;
        double s1 = x1;
        double s2 = x2;
        double s3 = x3;
        if (jcol == 0) {
          s0 += h;
        } else if (jcol == 1) {
          s1 += h;
        } else if (jcol == 2) {
          s2 += h;
        } else {
          s3 += h;
        }
        KDL::Frame fp = KDL::Frame::DH_Craig1989(s0, s1, s2, s3);
        KDL::Twist ep = KDL::diff(fp, t_link_rel);
        for (int r = 0; r < 6; ++r) {
          j(r, jcol) = (ep(r) - e(r)) / h;
        }
      }
      const double lambda = base_lambda * (1.0 + 0.05 * static_cast<double>(it));
      Eigen::Matrix4d jtj = j.transpose() * j;
      for (int k = 0; k < 4; ++k) {
        jtj(k, k) += lambda;
      }
      const Eigen::Vector4d rhs = -j.transpose() * evec;
      Eigen::LDLT<Eigen::Matrix4d> ldlt(jtj);
      Eigen::Vector4d delta = ldlt.solve(rhs);
      bool delta_bad = (ldlt.info() != Eigen::Success);
      if (!delta_bad) {
        for (int k = 0; k < 4; ++k) {
          if (!std::isfinite(delta(k))) {
            delta_bad = true;
            break;
          }
        }
      }
      if (delta_bad) {
        const double gn = j.squaredNorm() + 1e-9;
        delta = 0.1 * (j.transpose() * evec) / gn;
      }
      const double mstep = 0.3;
      if (delta.norm() > mstep) {
        delta *= mstep / delta.norm();
      }
      x0 += delta(0);
      x1 += delta(1);
      x2 += delta(2);
      x3 += delta(3);
    }
  }
  if (err_msg) {
    *err_msg = "无法将本段 4x4 拟合为单个 Craig(1989) MDH 行";
  }
  return false;
}

}  // namespace ros_robot_assist_tools::kinematics
