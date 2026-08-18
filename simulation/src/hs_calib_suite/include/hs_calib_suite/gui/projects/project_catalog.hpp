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

  QString source_path;   ///< project.yaml 或旧版扁平 yaml 路径
  QString root_path;     ///< 文件夹项目根；模板为空
  bool is_folder_project = false;
  bool user_writable = false;
  bool is_template = false;  ///< 包内扁平模板，需实例化成文件夹
};

/// \brief 扫描用户项目文件夹 + 包内模板清单（不做算法）
class ProjectCatalog {
public:
  /// \brief 重新扫描磁盘
  void reload();

  /// \brief 当前项目列表（文件夹项目在前，模板在后）
  const QVector<ProjectInfo> &projects() const { return projects_; }

  /// \brief 按 id 查找；找不到返回 nullptr
  const ProjectInfo *find(const QString &id) const;

  /// \brief 用户文件夹项目根（Documents/hs_calib_projects）
  static QString user_projects_dir();

  /// \brief 兼容旧路径（~/.config/.../projects 扁平 yaml）
  static QString legacy_user_yaml_dir();

  /// \brief 包内模板目录（share/.../config/projects）
  static QString package_projects_dir();

  /// \brief 新建文件夹项目（委托 ProjectWorkspace::create）
  bool create_user_project(const ProjectInfo &info, QString *error_out = nullptr);

  /// \brief 从包内模板实例化为用户文件夹项目
  bool materialize_template(const QString &template_id, QString *root_out = nullptr,
                            QString *error_out = nullptr);

  /// \brief 用系统文件管理器打开用户项目根目录
  static void open_user_projects_dir();

private:
  QVector<ProjectInfo> projects_;

  static ProjectInfo builtin_template(const QString &id);
};

}  // namespace gui
}  // namespace hs_calib
