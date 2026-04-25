#include "ros_robot_assist_tools/module/kinematics_solver_module.h"

#include <cmath>
#include <sstream>
#include <algorithm>
#include <cctype>

namespace ros_robot_assist_tools::ui
{
namespace
{

QString VecToQString(const std::vector<double> & v)
{
  QString s;
  for (size_t i = 0; i < v.size(); ++i) {
    if (i) {
      s += ", ";
    }
    s += QString::number(v[static_cast<int>(i)], 'g', 8);
  }
  return s;
}

QString PluginLevelText(const std::string & plugin)
{
  std::string s = plugin;
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (s.find("ikfast") != std::string::npos) {
    return "解析/准解析（IKFast）";
  }
  if (s.find("kdl") != std::string::npos) {
    return "数值迭代（KDL）";
  }
  if (s.find("trac_ik") != std::string::npos || s.find("tracik") != std::string::npos) {
    return "数值迭代（TRAC-IK）";
  }
  return "未知插件类型（查看插件文档确认是否解析）";
}

bool ParseSeedText(const std::string & text, std::map<std::string, double> * out, std::string * err)
{
  out->clear();
  std::istringstream in(text);
  std::string line;
  int ln = 0;
  while (std::getline(in, line)) {
    ++ln;
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) {
      line.pop_back();
    }
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::string name;
    double val{0.0};
    const auto cpos = line.find(':');
    try {
      if (cpos != std::string::npos) {
        name = line.substr(0, cpos);
        const std::string rest = line.substr(cpos + 1);
        val = std::stod(rest);
      } else {
        const auto sp = line.find_first_of(" \t,");
        if (sp == std::string::npos) {
          if (err) {
            *err = "第 " + std::to_string(ln) + " 行无法解析: " + line;
          }
          return false;
        }
        name = line.substr(0, sp);
        std::string rest = line.substr(sp);
        size_t a = 0;
        while (a < rest.size() && (rest[a] == ' ' || rest[a] == '\t' || rest[a] == ',')) {
          ++a;
        }
        val = std::stod(rest.substr(a));
      }
    } catch (const std::exception &) {
      if (err) {
        *err = "第 " + std::to_string(ln) + " 行数字无效";
      }
      return false;
    }
    // trim name
    while (!name.empty() && (name.back() == ' ' || name.back() == '\t')) {
      name.pop_back();
    }
    size_t b = 0;
    while (b < name.size() && (name[b] == ' ' || name[b] == '\t')) {
      ++b;
    }
    name = name.substr(b);
    if (name.empty()) {
      if (err) {
        *err = "第 " + std::to_string(ln) + " 行关节名为空";
      }
      return false;
    }
    (*out)[name] = val;
  }
  return true;
}

}  // namespace

KinematicsSolveResult KdlLoadArm(
  ArmKdlCache & cache, const std::string & urdf, const std::string & base, const std::string & tip)
{
  KinematicsSolveResult r;
  if (urdf.empty()) {
    r.message = "请先选择 URDF 文件";
    return r;
  }
  const std::string key = urdf + "|" + base + "|" + tip;
  if (cache.kdl && cache.key == key) {
    r.ok = true;
    r.message = "已加载 (缓存)";
    return r;
  }
  auto k = std::make_unique<kinematics::ArmKinematicsKdl>();
  std::string err;
  if (!k->LoadFromFile(urdf, base, tip, err)) {
    r.message = QString::fromStdString(err);
    return r;
  }
  cache.key = key;
  cache.kdl = std::move(k);
  r.ok = true;
  r.message = QString("KDL 链 OK，关节数 %1").arg(static_cast<qint64>(cache.kdl->GetJointCount()));
  return r;
}

KinematicsSolveResult KdlForward(ArmKdlCache & cache, const std::vector<double> & q)
{
  KinematicsSolveResult r;
  if (!cache.kdl) {
    r.message = "请先点「应用 URDF/链」";
    return r;
  }
  std::array<double, 7> p{};
  std::string e;
  if (!cache.kdl->Forward(q, p, e)) {
    r.message = QString::fromStdString(e);
    return r;
  }
  r.ok = true;
  r.message = QString("位置 m: %1, %2, %3\n四元数 xyzw: %4, %5, %6, %7")
    .arg(p[0], 0, 'g', 8)
    .arg(p[1], 0, 'g', 8)
    .arg(p[2], 0, 'g', 8)
    .arg(p[3], 0, 'g', 8)
    .arg(p[4], 0, 'g', 8)
    .arg(p[5], 0, 'g', 8)
    .arg(p[6], 0, 'g', 8);
  return r;
}

KinematicsSolveResult KdlInverse(
  ArmKdlCache & cache, const std::vector<double> & seed, const std::array<double, 3> & p,
  const std::array<double, 4> & q_xyzw)
{
  KinematicsSolveResult r;
  if (!cache.kdl) {
    r.message = "请先点「应用 URDF/链」";
    return r;
  }
  std::vector<double> sol;
  std::string e;
  if (!cache.kdl->Inverse(seed, p, q_xyzw, sol, e)) {
    r.message = QString::fromStdString(e) +
      "\n提示：KDL 为位姿级数值逆解，已启用关节限位与多初值重试；仍失败时请尝试 MoveIt2(含 IKFast) 或更换初值。";
    return r;
  }
  r.ok = true;
  r.message = "关节 (rad): " + VecToQString(sol);
  return r;
}

KinematicsSolveResult MoveitInverse(
  const rclcpp::Node::SharedPtr & node, const manage::KinematicsSolverDataManager & cfg,
  const std::array<double, 3> & p, const std::array<double, 4> & q_xyzw)
{
  KinematicsSolveResult r;
  if (!node) {
    r.message = "内部错误: ROS 节点未创建";
    return r;
  }
  kinematics::MoveitIkRequest req;
  req.service = cfg.GetMoveitService();
  req.group_name = cfg.GetMoveitGroup();
  req.ik_link_name = cfg.GetMoveitIkLink();
  req.frame_id = cfg.GetMoveitFrameId();
  req.desired_pose.position.x = p[0];
  req.desired_pose.position.y = p[1];
  req.desired_pose.position.z = p[2];
  req.desired_pose.orientation.x = q_xyzw[0];
  req.desired_pose.orientation.y = q_xyzw[1];
  req.desired_pose.orientation.z = q_xyzw[2];
  req.desired_pose.orientation.w = q_xyzw[3];
  std::string pe;
  if (!ParseSeedText(cfg.GetMoveitSeedText(), &req.seed, &pe)) {
    r.message = QString::fromStdString(pe);
    return r;
  }
  std::vector<std::string> names;
  std::vector<double> pos;
  std::string err;
  if (!kinematics::CallMoveitIk(node, req, &names, &pos, &err)) {
    r.message = QString::fromStdString(err) +
      "\n（需 move_group 等提供 GetPositionIK 服务，且与 SRDF/插件一致）";
    return r;
  }
  r.ok = true;
  QString lines;
  for (size_t i = 0; i < names.size() && i < pos.size(); ++i) {
    lines += QString::fromStdString(names[i]) + " = " + QString::number(pos[i], 'g', 8) + "\n";
  }
  r.message = "IK 解：\n" + lines;
  return r;
}

KinematicsSolveResult ProbeMoveitPlugin(
  const rclcpp::Node::SharedPtr & node, const manage::KinematicsSolverDataManager & cfg)
{
  KinematicsSolveResult r;
  std::string plugin;
  std::string err;
  if (!kinematics::QueryMoveitKinematicsSolverPlugin(
      node, cfg.GetMoveitNodeName(), cfg.GetMoveitGroup(), &plugin, &err))
  {
    r.message = QString::fromStdString("MoveIt 插件探测失败: " + err);
    return r;
  }
  r.ok = true;
  r.message = "MoveIt kinematics_solver: " + QString::fromStdString(plugin) +
    "\n能力等级: " + PluginLevelText(plugin);
  return r;
}

KinematicsSolveResult RunDiffFromWheels(
  const manage::KinematicsSolverDataManager & cfg, double w_l, double w_r)
{
  KinematicsSolveResult r;
  double v{0.0}, w{0.0};
  std::string e;
  if (!kinematics::DiffDriveKinematics::WheelToBody(
    cfg.GetDiffTrackM(), cfg.GetDiffWheelRadiusM(), w_l, w_r, v, w, &e)) {
    r.message = QString::fromStdString(e);
    return r;
  }
  r.ok = true;
  r.message = QString("v = %1 m/s, omega = %2 rad/s\n线速度(左) = %3 m/s, 线速度(右) = %4 m/s")
    .arg(v, 0, 'g', 6)
    .arg(w, 0, 'g', 6)
    .arg(w_l * cfg.GetDiffWheelRadiusM(), 0, 'g', 6)
    .arg(w_r * cfg.GetDiffWheelRadiusM(), 0, 'g', 6);
  return r;
}

KinematicsSolveResult RunDiffFromBody(
  const manage::KinematicsSolverDataManager & cfg, double v, double w)
{
  KinematicsSolveResult r;
  double wl{0.0}, wr{0.0};
  std::string e;
  if (!kinematics::DiffDriveKinematics::BodyToWheel(
    cfg.GetDiffTrackM(), cfg.GetDiffWheelRadiusM(), v, w, wl, wr, &e)) {
    r.message = QString::fromStdString(e);
    return r;
  }
  r.ok = true;
  r.message = QString("w_left = %1 rad/s, w_right = %2 rad/s").arg(wl, 0, 'g', 6).arg(wr, 0, 'g', 6);
  return r;
}

KinematicsSolveResult RunAckGeometry(
  const manage::KinematicsSolverDataManager & cfg, double delta_deg)
{
  KinematicsSolveResult r;
  const double d = delta_deg * M_PI / 180.0;
  double R{0.0}, k{0.0};
  std::string e;
  if (!kinematics::AckermannBicycleKinematics::Geometry(
    cfg.GetAckWheelbaseM(), d, &R, &k, &e)) {
    r.message = QString::fromStdString(e);
    return r;
  }
  r.ok = true;
  if (std::isinf(R)) {
    r.message = QString("delta≈0：转弯半径 ∞，曲率 0 (轴距 L = %1 m)").arg(cfg.GetAckWheelbaseM());
  } else {
    r.message = QString("转弯半径 R = %1 m, 曲率 k = %2 1/m (L = %3 m, delta = %4°)")
      .arg(R, 0, 'g', 6)
      .arg(k, 0, 'g', 6)
      .arg(cfg.GetAckWheelbaseM(), 0, 'g', 6)
      .arg(delta_deg, 0, 'g', 4);
  }
  return r;
}

KinematicsSolveResult RunAckForwardBikeVelocity(
  const manage::KinematicsSolverDataManager & cfg, double delta_deg, double v_ref_mps)
{
  KinematicsSolveResult r = RunAckGeometry(cfg, delta_deg);
  if (!r.ok) {
    return r;
  }
  const double d = delta_deg * M_PI / 180.0;
  const double k = std::tan(d) / cfg.GetAckWheelbaseM();
  const double omega = v_ref_mps * k;
  r.message += QString("\n速度映射: v_ref = %1 m/s -> omega = %2 rad/s")
                 .arg(v_ref_mps, 0, 'g', 6)
                 .arg(omega, 0, 'g', 6);
  return r;
}

KinematicsSolveResult RunAckInverseBike(
  const manage::KinematicsSolverDataManager & cfg, double target_curvature_1_m)
{
  KinematicsSolveResult r;
  double delta_rad{0.0};
  double R{0.0};
  std::string e;
  if (!kinematics::AckermannBicycleKinematics::InverseFromCurvature(
      cfg.GetAckWheelbaseM(), target_curvature_1_m, &delta_rad, &R, &e)) {
    r.message = QString::fromStdString(e);
    return r;
  }
  r.ok = true;
  r.message = QString("自行车逆解: delta = %1° (%2 rad), R = %3 m, k = %4 1/m")
    .arg(delta_rad * 180.0 / M_PI, 0, 'g', 6)
    .arg(delta_rad, 0, 'g', 6)
    .arg(R, 0, 'g', 6)
    .arg(target_curvature_1_m, 0, 'g', 6);
  return r;
}

KinematicsSolveResult RunAckInverseBikeVelocity(
  const manage::KinematicsSolverDataManager & cfg, double v_ref_mps, double omega_target_radps)
{
  KinematicsSolveResult r;
  if (std::abs(v_ref_mps) < 1e-9) {
    r.message = "自行车逆解(速度): 参考点线速度 v_ref 不能为 0";
    return r;
  }
  const double k = omega_target_radps / v_ref_mps;
  r = RunAckInverseBike(cfg, k);
  if (!r.ok) {
    return r;
  }
  r.message += QString("\n由速度反解: k = omega / v_ref = %1 / %2 = %3 1/m")
                 .arg(omega_target_radps, 0, 'g', 6)
                 .arg(v_ref_mps, 0, 'g', 6)
                 .arg(k, 0, 'g', 6);
  return r;
}

KinematicsSolveResult RunAckIdealFromInner(
  const manage::KinematicsSolverDataManager & cfg, double track_m, double delta_inner_deg)
{
  KinematicsSolveResult r;
  const double L = cfg.GetAckWheelbaseM();
  if (L <= 0.0 || track_m <= 0.0) {
    r.message = "理想阿克曼: 轴距 L 与轮距 W 需为正数";
    return r;
  }
  const double din = delta_inner_deg * M_PI / 180.0;
  const double tin = std::tan(din);
  if (std::abs(tin) < 1e-9) {
    r.ok = true;
    r.message = "delta_inner≈0：直行（R=∞，k=0）";
    return r;
  }
  const double R_rear = L / tin + 0.5 * track_m;  // 按左转内侧轮定义
  const double dout = std::atan2(L, (R_rear + 0.5 * track_m));
  const double k = 1.0 / R_rear;
  r.ok = true;
  r.message = QString(
    "理想阿克曼(由内轮角): R_rear = %1 m, k = %2 1/m, δ_inner = %3°, δ_outer = %4° (L=%5, W=%6)")
    .arg(R_rear, 0, 'g', 6)
    .arg(k, 0, 'g', 6)
    .arg(delta_inner_deg, 0, 'g', 6)
    .arg(dout * 180.0 / M_PI, 0, 'g', 6)
    .arg(L, 0, 'g', 6)
    .arg(track_m, 0, 'g', 6);
  return r;
}

KinematicsSolveResult RunAckIdealFromPair(
  const manage::KinematicsSolverDataManager & cfg, double track_m, double delta_inner_deg,
  double delta_outer_deg)
{
  KinematicsSolveResult r;
  const double L = cfg.GetAckWheelbaseM();
  if (L <= 0.0 || track_m <= 0.0) {
    r.message = "理想阿克曼: 轴距 L 与轮距 W 需为正数";
    return r;
  }
  const double din = delta_inner_deg * M_PI / 180.0;
  const double dout = delta_outer_deg * M_PI / 180.0;
  if (std::abs(std::sin(din)) < 1e-9 || std::abs(std::sin(dout)) < 1e-9) {
    r.message = "理想阿克曼: 角度过小或无效";
    return r;
  }
  const double lhs = (1.0 / std::tan(dout)) - (1.0 / std::tan(din));
  const double rhs = track_m / L;
  const double err = lhs - rhs;
  const double R_rear_in = L / std::tan(din) + 0.5 * track_m;
  const double R_rear_out = L / std::tan(dout) - 0.5 * track_m;
  const double R_rear = 0.5 * (R_rear_in + R_rear_out);
  r.ok = true;
  r.message = QString(
    "理想阿克曼(由内外轮角): R_rear≈%1 m, 约束误差(cotδ_out-cotδ_in-W/L)=%2, "
    "δ_in=%3°, δ_out=%4° (L=%5, W=%6)")
    .arg(R_rear, 0, 'g', 6)
    .arg(err, 0, 'g', 6)
    .arg(delta_inner_deg, 0, 'g', 6)
    .arg(delta_outer_deg, 0, 'g', 6)
    .arg(L, 0, 'g', 6)
    .arg(track_m, 0, 'g', 6);
  return r;
}

}  // namespace ros_robot_assist_tools::ui
