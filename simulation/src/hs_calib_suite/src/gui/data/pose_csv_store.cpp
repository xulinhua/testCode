#include "hs_calib_suite/gui/data/pose_csv_store.hpp"

#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#include <Eigen/Geometry>

namespace hs_calib {
namespace gui {

/// \brief 取文件名作为 CSV 键
QString PoseCsvStore::basename_key(const QString &path_or_name) {
  return QFileInfo(path_or_name).fileName();
}

/// \brief 清空位姿表
void PoseCsvStore::clear() {
  poses_.clear();
}

/// \brief 从 CSV 加载 image→T_base_gripper
bool PoseCsvStore::load(const QString &path, QString *error_out) {
  poses_.clear();
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
    if (error_out) {
      *error_out = QStringLiteral("无法打开位姿 CSV：%1").arg(path);
    }
    return false;
  }
  // —— 逐行解析 image,t,q ——
  QTextStream in(&f);
  int line_no = 0;
  while (!in.atEnd()) {
    const QString line = in.readLine().trimmed();
    ++line_no;
    if (line.isEmpty() || line.startsWith(QLatin1Char('#'))) {
      continue;
    }
    const QStringList parts = line.split(QLatin1Char(','));
    if (parts.size() < 8) {
      if (error_out) {
        *error_out = QStringLiteral("第 %1 行列数不足（需 image,tx,ty,tz,qx,qy,qz,qw）")
                         .arg(line_no);
      }
      poses_.clear();
      return false;
    }
    const QString key = basename_key(parts[0].trimmed());
    const double tx = parts[1].toDouble();
    const double ty = parts[2].toDouble();
    const double tz = parts[3].toDouble();
    Eigen::Quaterniond q(
        parts[7].toDouble(), parts[4].toDouble(), parts[5].toDouble(),
        parts[6].toDouble());
    q.normalize();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    T(0, 3) = tx;
    T(1, 3) = ty;
    T(2, 3) = tz;
    poses_[key.toStdString()] = T;
  }
  if (poses_.empty()) {
    if (error_out) {
      *error_out = QStringLiteral("位姿 CSV 为空");
    }
    return false;
  }
  return true;
}

/// \brief 是否含该图片键
bool PoseCsvStore::has(const QString &image_path_or_name) const {
  return poses_.count(basename_key(image_path_or_name).toStdString()) > 0;
}

/// \brief 按图片键取出位姿矩阵
bool PoseCsvStore::get(const QString &image_path_or_name, Eigen::Matrix4d *T_out) const {
  if (T_out == nullptr) {
    return false;
  }
  const auto it = poses_.find(basename_key(image_path_or_name).toStdString());
  if (it == poses_.end()) {
    return false;
  }
  *T_out = it->second;
  return true;
}

}  // namespace gui
}  // namespace hs_calib
