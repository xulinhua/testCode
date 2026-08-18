#pragma once

#include <QString>
#include <QStringList>
#include <QVector>

#include "hs_calib_suite/gui/projects/project_catalog.hpp"

namespace hs_calib {
namespace gui {

/// \brief 一次标定结果会话在项目内的摘要（results/<name>/）
struct ProjectResultEntry {
  QString name;       ///< 子目录名
  QString path;       ///< 绝对路径
  QString calibrator; ///< 若 session_config 可读则填
  QString summary;    ///< 一行摘要
};

/// \brief 文件夹型标定项目工作区（与 core 算法、扁平 yaml 模板分离）
///
/// 目录约定（一项目一文件夹）：
/// \code
///   <root>/
///     project.yaml      # 元数据 / 坐标系 / 推荐标定器
///     config/           # 会话与板参等配置副本
///     images/           # 标定用图像（可分子目录）
///     results/          # 每次导出的结果包
/// \endcode
class ProjectWorkspace {
public:
  /// \brief 默认用户项目根目录（其下每个子目录是一个项目）
  static QString default_projects_root();

  /// \brief 在 parent_dir 下创建完整项目文件夹并写入 project.yaml
  static bool create(
      const QString &parent_dir, const ProjectInfo &info, QString *root_out = nullptr,
      QString *error_out = nullptr);

  /// \brief 从模板 yaml（包内扁平项目）实例化为用户文件夹项目
  static bool create_from_template(
      const QString &template_yaml, const QString &parent_dir, QString *root_out = nullptr,
      QString *error_out = nullptr);

  /// \brief 是否为合法项目根（存在 project.yaml）
  static bool is_project_root(const QString &dir);

  /// \brief 打开已有项目根目录
  bool open(const QString &root_dir, QString *error_out = nullptr);

  /// \brief 关闭当前项目（不删磁盘）
  void close();

  /// \brief 是否已打开
  bool is_open() const { return !root_.isEmpty(); }

  /// \brief 项目根路径
  QString root_path() const { return root_; }

  /// \brief 当前元数据（可改后 save_meta）
  ProjectInfo &meta() { return meta_; }
  const ProjectInfo &meta() const { return meta_; }

  /// \brief 写回 project.yaml
  bool save_meta(QString *error_out = nullptr) const;

  /// \brief 重新从磁盘读 project.yaml
  bool reload_meta(QString *error_out = nullptr);

  QString config_dir() const;
  QString images_dir() const;
  QString results_dir() const;
  QString project_yaml_path() const;

  /// \brief 确保 config/images/results 存在
  bool ensure_layout(QString *error_out = nullptr) const;

  /// \brief 复制文件到 config/（同名覆盖）
  bool import_config_file(const QString &src_path, QString *error_out = nullptr) const;

  /// \brief 列出 config/ 下 yaml
  QStringList list_config_files() const;

  /// \brief 将图片复制进 images/（可选子目录名，空则根 images/）
  int import_images(
      const QStringList &file_paths, const QString &subdir = QString(),
      QString *error_out = nullptr) const;

  /// \brief 将整个图片目录复制为 images/<subdir>/
  bool import_image_directory(
      const QString &src_dir, const QString &subdir, QString *error_out = nullptr) const;

  /// \brief 列出 images/ 下图片（递归）
  QStringList list_images() const;

  /// \brief 默认采图目录：优先 images/，否则空
  QString preferred_image_dir() const;

  /// \brief 把一次 export_bundle 目录归档到 results/<name>/
  bool archive_result_bundle(
      const QString &bundle_dir, const QString &name_hint = QString(),
      QString *archived_path_out = nullptr, QString *error_out = nullptr) const;

  /// \brief 列出 results/ 子目录
  QVector<ProjectResultEntry> list_results() const;

  /// \brief 建议的下一次导出目录（results/hs_calib_<calib>_<stamp>）
  QString suggest_export_dir(const QString &calibrator_id) const;

private:
  QString root_;
  ProjectInfo meta_;

  static bool write_project_yaml(const QString &path, const ProjectInfo &info, QString *error_out);
  static bool read_project_yaml(const QString &path, ProjectInfo *info, QString *error_out);
};

}  // namespace gui
}  // namespace hs_calib
