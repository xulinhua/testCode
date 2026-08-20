#pragma once

#include <QString>
#include <QStringList>
#include <QVector>

namespace hs_calib {
namespace gui {

/// \brief 标定「项目」元数据（帧名 / 推荐标定器 / 可选文件夹根路径）
///
/// 文件夹型项目由 ProjectWorkspace 管理磁盘布局；本结构只描述元数据。
struct ProjectInfo {
  QString id;
  QString display_name;
  QString description;
  QString notes;
  QString parent_frame = QStringLiteral("camera_link");
  QString child_frame = QStringLiteral("camera_optical_frame");
  QString base_frame = QStringLiteral("base");
  QString gripper_frame = QStringLiteral("tool0");
  QString image_frame = QStringLiteral("camera_optical_frame");
  QString camera_link_frame = QStringLiteral("camera_link");
  QStringList recommended_calibrators;
  QString last_calibrator_id;
  QString default_image_subdir;

  QString source_path;   ///< project.yaml 路径
  QString root_path;     ///< 文件夹项目根
  bool is_folder_project = false;
  bool user_writable = false;
  bool is_template = false;  ///< 保留字段；列表不再展示模板
};

/// \brief 扫描用户项目文件夹清单（不做算法）
class ProjectCatalog {
public:
  /// \brief 重新扫描磁盘；若无项目则自动创建默认项目
  void reload();

  /// \brief 当前项目列表（仅用户文件夹项目）
  const QVector<ProjectInfo> &projects() const { return projects_; }

  /// \brief 按 id 查找；找不到返回 nullptr
  const ProjectInfo *find(const QString &id) const;

  /// \brief 用户文件夹项目根（Documents/hs_calib_projects）
  static QString user_projects_dir();

  /// \brief 新建文件夹项目（委托 ProjectWorkspace::create）
  bool create_user_project(const ProjectInfo &info, QString *error_out = nullptr);

  /// \brief 导入外部项目文件夹到用户项目根（复制整目录）
  /// \param src_dir 含 project.yaml 的源目录
  /// \param id_override 非空则作为目标项目 ID；空则用源目录名 / yaml 内 id
  bool import_user_project(
      const QString &src_dir, const QString &id_override = QString(),
      QString *root_out = nullptr, QString *error_out = nullptr);

  /// \brief 删除用户文件夹项目（整目录）
  bool delete_user_project(const QString &id, QString *error_out = nullptr);

  /// \brief 用系统文件管理器打开用户项目根目录
  static void open_user_projects_dir();

private:
  QVector<ProjectInfo> projects_;
};

}  // namespace gui
}  // namespace hs_calib
