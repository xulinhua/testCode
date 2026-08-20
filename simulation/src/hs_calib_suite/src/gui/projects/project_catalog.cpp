#include "hs_calib_suite/gui/projects/project_catalog.hpp"

#include "hs_calib_suite/gui/projects/project_workspace.hpp"

#include <algorithm>

#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMap>
#include <QUrl>

namespace hs_calib {
namespace gui {
namespace {

bool copy_dir_recursive_local(const QString &src, const QString &dst, QString *error_out) {
  QDir s(src);
  if (!s.exists()) {
    if (error_out) {
      *error_out = QStringLiteral("源目录不存在：%1").arg(src);
    }
    return false;
  }
  if (!QDir().mkpath(dst)) {
    if (error_out) {
      *error_out = QStringLiteral("无法创建：%1").arg(dst);
    }
    return false;
  }
  for (const QString &name : s.entryList(QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot)) {
    const QString sp = s.filePath(name);
    const QString dp = QDir(dst).filePath(name);
    if (QFileInfo(sp).isDir()) {
      if (!copy_dir_recursive_local(sp, dp, error_out)) {
        return false;
      }
    } else {
      if (QFile::exists(dp) && !QFile::remove(dp)) {
        if (error_out) {
          *error_out = QStringLiteral("无法覆盖：%1").arg(dp);
        }
        return false;
      }
      if (!QFile::copy(sp, dp)) {
        if (error_out) {
          *error_out = QStringLiteral("复制失败：%1 → %2").arg(sp, dp);
        }
        return false;
      }
    }
  }
  return true;
}

bool id_looks_safe(const QString &id) {
  if (id.isEmpty() || id.contains('/') || id.contains('\\') || id == QStringLiteral(".") ||
      id == QStringLiteral("..")) {
    return false;
  }
  for (const QChar c : id) {
    if (!(c.isLetterOrNumber() || c == '_' || c == '-' || c == '.')) {
      return false;
    }
  }
  return true;
}

}  // namespace

QString ProjectCatalog::user_projects_dir() {
  return ProjectWorkspace::default_projects_root();
}

void ProjectCatalog::open_user_projects_dir() {
  const QString dir = user_projects_dir();
  QDesktopServices::openUrl(QUrl::fromLocalFile(dir));
}

void ProjectCatalog::reload() {
  projects_.clear();
  QMap<QString, ProjectInfo> folders;

  // 用户文件夹项目：Documents/hs_calib_projects/<id>/project.yaml
  {
    QDir root(user_projects_dir());
    if (root.exists()) {
      for (const QString &name : root.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
        const QString dir = root.filePath(name);
        if (!ProjectWorkspace::is_project_root(dir)) {
          continue;
        }
        ProjectWorkspace ws;
        if (!ws.open(dir, nullptr)) {
          continue;
        }
        ProjectInfo p = ws.meta();
        p.user_writable = true;
        p.is_folder_project = true;
        p.is_template = false;
        folders.insert(p.id, p);
      }
    }
  }

  QVector<ProjectInfo> folder_list = folders.values().toVector();
  std::sort(folder_list.begin(), folder_list.end(), [](const ProjectInfo &a, const ProjectInfo &b) {
    return a.display_name.toLower() < b.display_name.toLower();
  });

  // 不展示模板；若没有任何项目则自动建一个默认项目
  if (folder_list.isEmpty()) {
    ProjectInfo seed;
    seed.id = QStringLiteral("default_project");
    seed.display_name = QStringLiteral("默认项目");
    seed.description = QStringLiteral("自动创建的默认标定项目");
    seed.notes = QStringLiteral("可用「新建…」继续添加项目");
    seed.recommended_calibrators << QStringLiteral("cam_intrinsics");
    QString root;
    if (ProjectWorkspace::create(user_projects_dir(), seed, &root, nullptr)) {
      ProjectWorkspace ws;
      if (ws.open(root, nullptr)) {
        ProjectInfo p = ws.meta();
        p.user_writable = true;
        p.is_folder_project = true;
        p.is_template = false;
        folder_list.push_back(p);
      }
    }
  }

  projects_ = folder_list;
}

const ProjectInfo *ProjectCatalog::find(const QString &id) const {
  for (const auto &p : projects_) {
    if (p.id == id) {
      return &p;
    }
  }
  return nullptr;
}

bool ProjectCatalog::create_user_project(const ProjectInfo &info, QString *error_out) {
  QString root;
  if (!ProjectWorkspace::create(user_projects_dir(), info, &root, error_out)) {
    return false;
  }
  reload();
  return true;
}

bool ProjectCatalog::import_user_project(
    const QString &src_dir, const QString &id_override, QString *root_out,
    QString *error_out) {
  if (!ProjectWorkspace::is_project_root(src_dir)) {
    if (error_out) {
      *error_out = QStringLiteral("不是有效项目目录（缺少 project.yaml）：%1").arg(src_dir);
    }
    return false;
  }
  ProjectWorkspace src_ws;
  if (!src_ws.open(src_dir, error_out)) {
    return false;
  }
  QString id = id_override.trimmed();
  if (id.isEmpty()) {
    id = src_ws.meta().id.trimmed();
  }
  if (id.isEmpty()) {
    id = QFileInfo(src_dir).fileName();
  }
  if (!id_looks_safe(id)) {
    if (error_out) {
      *error_out = QStringLiteral("项目 ID 非法：%1").arg(id);
    }
    return false;
  }

  const QString user_root = user_projects_dir();
  const QString dst = QDir(user_root).filePath(id);
  const QString src_can = QFileInfo(src_dir).canonicalFilePath();
  const QString dst_can = QFileInfo(dst).exists() ? QFileInfo(dst).canonicalFilePath() : dst;
  if (!src_can.isEmpty() && src_can == QFileInfo(dst_can).canonicalFilePath()) {
    if (root_out) {
      *root_out = dst;
    }
    reload();
    return true;
  }
  if (QDir(dst).exists()) {
    if (error_out) {
      *error_out = QStringLiteral("目标项目已存在：%1").arg(dst);
    }
    return false;
  }
  if (!copy_dir_recursive_local(src_dir, dst, error_out)) {
    QDir(dst).removeRecursively();
    return false;
  }

  ProjectWorkspace dst_ws;
  if (!dst_ws.open(dst, error_out)) {
    QDir(dst).removeRecursively();
    return false;
  }
  dst_ws.meta().id = id;
  dst_ws.meta().is_template = false;
  dst_ws.meta().user_writable = true;
  dst_ws.meta().is_folder_project = true;
  if (!dst_ws.save_meta(error_out)) {
    QDir(dst).removeRecursively();
    return false;
  }
  if (root_out) {
    *root_out = dst;
  }
  reload();
  return true;
}

bool ProjectCatalog::delete_user_project(const QString &id, QString *error_out) {
  const ProjectInfo *p = find(id);
  if (p == nullptr) {
    if (error_out) {
      *error_out = QStringLiteral("找不到项目：%1").arg(id);
    }
    return false;
  }
  if (!p->is_folder_project || p->root_path.isEmpty()) {
    if (error_out) {
      *error_out = QStringLiteral("只能删除用户文件夹项目");
    }
    return false;
  }

  const QString user_root = QFileInfo(user_projects_dir()).canonicalFilePath();
  const QString root = QFileInfo(p->root_path).canonicalFilePath();
  if (user_root.isEmpty() || root.isEmpty() ||
      !(root == user_root || root.startsWith(user_root + QLatin1Char('/')))) {
    if (error_out) {
      *error_out = QStringLiteral("拒绝删除：路径不在用户项目根下：%1").arg(p->root_path);
    }
    return false;
  }
  if (root == user_root) {
    if (error_out) {
      *error_out = QStringLiteral("拒绝删除用户项目根目录本身");
    }
    return false;
  }

#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
  if (!QFile::moveToTrash(root)) {
    if (!QDir(root).removeRecursively()) {
      if (error_out) {
        *error_out = QStringLiteral("删除失败：%1").arg(root);
      }
      return false;
    }
  }
#else
  if (!QDir(root).removeRecursively()) {
    if (error_out) {
      *error_out = QStringLiteral("删除失败：%1").arg(root);
    }
    return false;
  }
#endif
  reload();
  return true;
}

}  // namespace gui
}  // namespace hs_calib
