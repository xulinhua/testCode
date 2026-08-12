#pragma once

#include <map>
#include <string>

#include <QString>
#include <QStringList>

#include <Eigen/Core>

namespace hs_calib {
namespace gui {

/// \brief 离线位姿表：image basename → T_base_gripper
class PoseCsvStore {
public:
  /// \brief 从 CSV 加载（列：image_name,tx,ty,tz,qx,qy,qz,qw）
  bool load(const QString &path, QString *error_out = nullptr);

  /// \brief 清空位姿表
  void clear();
  /// \brief 位姿条数
  int size() const { return static_cast<int>(poses_.size()); }

  /// \brief 是否含该图片键
  bool has(const QString &image_path_or_name) const;
  /// \brief 按图片键取出位姿矩阵
  bool get(const QString &image_path_or_name, Eigen::Matrix4d *T_out) const;

  /// \brief 只读访问内部位姿表
  const std::map<std::string, Eigen::Matrix4d> &poses() const { return poses_; }

private:
  static QString basename_key(const QString &path_or_name);
  std::map<std::string, Eigen::Matrix4d> poses_;
};

}  // namespace gui
}  // namespace hs_calib
