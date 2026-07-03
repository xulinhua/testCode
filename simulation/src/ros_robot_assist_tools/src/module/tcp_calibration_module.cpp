#include "ros_robot_assist_tools/module/tcp_calibration_module.h"

#include <cmath>
#include <fstream>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#include <yaml-cpp/yaml.h>

namespace ros_robot_assist_tools::ui
{
namespace
{

int ColumnIndex(const QStringList & cols, const QStringList & candidates)
{
  for (const auto & c : candidates) {
    const int i = cols.indexOf(c);
    if (i >= 0) {
      return i;
    }
  }
  return -1;
}

bool ParseDoubleCell(const QString & cell, double * out)
{
  if (!out) {
    return false;
  }
  bool ok = false;
  *out = cell.trimmed().toDouble(&ok);
  return ok;
}

Eigen::Quaterniond QuatFromCells(
  const QStringList & cells, int ix, int iy, int iz, int iw)
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
  if (!ParseDoubleCell(cells[ix], &x) ||
    !ParseDoubleCell(cells[iy], &y) ||
    !ParseDoubleCell(cells[iz], &z) ||
    !ParseDoubleCell(cells[iw], &w))
  {
    return Eigen::Quaterniond::Identity();
  }
  Eigen::Quaterniond q(w, x, y, z);
  if (q.norm() < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

Eigen::Vector3d PositionFromCells(
  const QStringList & cells, int ix, int iy, int iz)
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  if (!ParseDoubleCell(cells[ix], &x) ||
    !ParseDoubleCell(cells[iy], &y) ||
    !ParseDoubleCell(cells[iz], &z))
  {
    return Eigen::Vector3d::Zero();
  }
  return Eigen::Vector3d(x, y, z);
}

}  // namespace

bool ParseTcpPosesFromCsv(const QString & csv_path, std::vector<FlangePoseSample> * poses, QString * err_msg)
{
  if (!poses) {
    if (err_msg) {
      *err_msg = "poses is null";
    }
    return false;
  }
  poses->clear();

  QFile f(csv_path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
    if (err_msg) {
      *err_msg = "无法打开 CSV 文件";
    }
    return false;
  }

  QTextStream in(&f);
  if (in.atEnd()) {
    if (err_msg) {
      *err_msg = "CSV 文件为空";
    }
    return false;
  }

  const QString header = in.readLine().trimmed();
  const QStringList cols = header.split(',', Qt::KeepEmptyParts);
  const int i_px = ColumnIndex(cols, {"tx", "px", "x"});
  const int i_py = ColumnIndex(cols, {"ty", "py", "y"});
  const int i_pz = ColumnIndex(cols, {"tz", "pz", "z"});
  const int i_qx = ColumnIndex(cols, {"qx", "rx"});
  const int i_qy = ColumnIndex(cols, {"qy", "ry"});
  const int i_qz = ColumnIndex(cols, {"qz", "rz"});
  const int i_qw = ColumnIndex(cols, {"qw", "rw"});

  if (i_px < 0 || i_py < 0 || i_pz < 0 || i_qx < 0 || i_qy < 0 || i_qz < 0 || i_qw < 0) {
    if (err_msg) {
      *err_msg = "CSV 缺少必须列: tx/ty/tz/qx/qy/qz/qw（或 px/py/pz 别名）";
    }
    return false;
  }

  while (!in.atEnd()) {
    const QString line = in.readLine().trimmed();
    if (line.isEmpty() || line.startsWith('#')) {
      continue;
    }
    const QStringList cells = line.split(',', Qt::KeepEmptyParts);
    const int max_idx = std::max({i_px, i_py, i_pz, i_qx, i_qy, i_qz, i_qw});
    if (cells.size() <= max_idx) {
      continue;
    }

    FlangePoseSample sample;
    sample.position = PositionFromCells(cells, i_px, i_py, i_pz);
    sample.orientation = QuatFromCells(cells, i_qx, i_qy, i_qz, i_qw);
    poses->push_back(sample);
  }

  if (poses->empty()) {
    if (err_msg) {
      *err_msg = "CSV 无有效位姿行";
    }
    return false;
  }
  return true;
}

bool SolveTcpOffsetTranslation(
  const std::vector<FlangePoseSample> & poses,
  TcpCalibrationResult * result,
  QString * err_msg)
{
  if (!result) {
    if (err_msg) {
      *err_msg = "result is null";
    }
    return false;
  }
  if (poses.size() < 4) {
    if (err_msg) {
      *err_msg = "至少需要 4 组不同姿态的法兰位姿";
    }
    return false;
  }

  const Eigen::Matrix3d R0 = poses[0].orientation.toRotationMatrix();
  const Eigen::Vector3d p0 = poses[0].position;

  Eigen::MatrixXd A(3 * static_cast<int>(poses.size() - 1), 3);
  Eigen::VectorXd b(3 * static_cast<int>(poses.size() - 1));

  for (size_t i = 1; i < poses.size(); ++i) {
    const Eigen::Matrix3d Ri = poses[i].orientation.toRotationMatrix();
    const Eigen::Vector3d pi = poses[i].position;
    const int row = static_cast<int>(i - 1) * 3;
    A.block<3, 3>(row, 0) = Ri - R0;
    b.segment<3>(row) = p0 - pi;
  }

  const Eigen::Vector3d tcp = A.colPivHouseholderQr().solve(b);
  if (!tcp.allFinite()) {
    if (err_msg) {
      *err_msg = "TCP 求解失败：位姿差异不足或数据病态";
    }
    return false;
  }

  std::vector<double> residuals;
  residuals.reserve(poses.size());
  Eigen::Vector3d contact_sum = Eigen::Vector3d::Zero();
  for (const auto & pose : poses) {
    const Eigen::Vector3d contact = pose.orientation * tcp + pose.position;
    contact_sum += contact;
    const Eigen::Vector3d ref = R0 * tcp + p0;
    residuals.push_back((contact - ref).norm());
  }

  const Eigen::Vector3d contact_mean = contact_sum / static_cast<double>(poses.size());
  double sum_sq = 0.0;
  double max_res = 0.0;
  for (const auto & pose : poses) {
    const Eigen::Vector3d contact = pose.orientation * tcp + pose.position;
    const double res = (contact - contact_mean).norm();
    sum_sq += res * res;
    max_res = std::max(max_res, res);
  }

  result->tcp_offset_flange = tcp;
  result->contact_point_base = contact_mean;
  result->rms_residual_m = std::sqrt(sum_sq / static_cast<double>(poses.size()));
  result->max_residual_m = max_res;
  result->num_poses = static_cast<int>(poses.size());
  return true;
}

bool SaveTcpCalibrationYaml(
  const QString & output_yaml,
  const TcpCalibrationResult & result,
  const QString & flange_frame,
  const QString & source_desc,
  QString * err_msg)
{
  if (output_yaml.trimmed().isEmpty()) {
    if (err_msg) {
      *err_msg = "输出路径为空";
    }
    return false;
  }

  QFileInfo out_info(output_yaml);
  QDir().mkpath(out_info.absolutePath());

  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "module_name" << YAML::Value << "tcp_calibration";
    out << YAML::Key << "flange_frame" << YAML::Value << flange_frame.toStdString();
    out << YAML::Key << "source" << YAML::Value << source_desc.toStdString();
    out << YAML::Key << "num_poses" << YAML::Value << result.num_poses;
    out << YAML::Key << "rms_residual_m" << YAML::Value << result.rms_residual_m;
    out << YAML::Key << "max_residual_m" << YAML::Value << result.max_residual_m;
    out << YAML::Key << "contact_point_base" << YAML::Flow << YAML::BeginSeq
        << result.contact_point_base.x() << result.contact_point_base.y() << result.contact_point_base.z()
        << YAML::EndSeq;
    out << YAML::Key << "tcp_offset_flange" << YAML::Flow << YAML::BeginSeq
        << result.tcp_offset_flange.x() << result.tcp_offset_flange.y() << result.tcp_offset_flange.z()
        << YAML::EndSeq;
    out << YAML::Key << "enable" << YAML::Value << true;
    out << YAML::Key << "offset_x" << YAML::Value << result.tcp_offset_flange.x();
    out << YAML::Key << "offset_y" << YAML::Value << result.tcp_offset_flange.y();
    out << YAML::Key << "offset_z" << YAML::Value << result.tcp_offset_flange.z();
    out << YAML::Key << "offset_rx" << YAML::Value << 0.0;
    out << YAML::Key << "offset_ry" << YAML::Value << 0.0;
    out << YAML::Key << "offset_rz" << YAML::Value << 0.0;
    out << YAML::EndMap;

    std::ofstream ofs(output_yaml.toStdString());
    if (!ofs.is_open()) {
      if (err_msg) {
        *err_msg = "无法写入输出文件";
      }
      return false;
    }
    ofs << out.c_str();
    return true;
  } catch (const std::exception & e) {
    if (err_msg) {
      *err_msg = QString("保存 YAML 失败: %1").arg(e.what());
    }
    return false;
  }
}

}  // namespace ros_robot_assist_tools::ui
