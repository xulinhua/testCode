#include "hs_calib_suite/gui/projects/project_catalog.hpp"

#include "hs_calib_suite/gui/projects/project_workspace.hpp"

#include <algorithm>

#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMap>
#include <QStandardPaths>
#include <QTextStream>
#include <QUrl>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace hs_calib {
namespace gui {
namespace {

QString trim_value(QString s) {
  s = s.trimmed();
  if ((s.startsWith('"') && s.endsWith('"')) || (s.startsWith('\'') && s.endsWith('\''))) {
    s = s.mid(1, s.size() - 2);
  }
  return s.trimmed();
}

QStringList parse_list_block(QTextStream &in) {
  QStringList out;
  while (!in.atEnd()) {
    const qint64 pos = in.pos();
    const QString line = in.readLine();
    const QString t = line.trimmed();
    if (t.startsWith("- ")) {
      out.push_back(trim_value(t.mid(2)));
      continue;
    }
    in.seek(pos);
    break;
  }
  return out;
}

bool load_flat_yaml(const QString &path, ProjectInfo *out) {
  if (out == nullptr) {
    return false;
  }
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return false;
  }
  ProjectInfo p;
  p.source_path = path;
  p.is_template = true;
  p.is_folder_project = false;
  QTextStream in(&f);
  QString multiline_key;
  QString multiline_acc;
  auto flush_multi = [&]() {
    if (multiline_key.isEmpty()) {
      return;
    }
    const QString v = multiline_acc.trimmed();
    if (multiline_key == QStringLiteral("description")) {
      p.description = v;
    } else if (multiline_key == QStringLiteral("notes")) {
      p.notes = v;
    }
    multiline_key.clear();
    multiline_acc.clear();
  };
  while (!in.atEnd()) {
    const QString line = in.readLine();
    const QString t = line.trimmed();
    if (t.isEmpty() || t.startsWith('#')) {
      continue;
    }
    if (!multiline_key.isEmpty()) {
      if (line.startsWith(' ') || line.startsWith('\t')) {
        if (!multiline_acc.isEmpty()) {
          multiline_acc += '\n';
        }
        multiline_acc += t;
        continue;
      }
      flush_multi();
    }
    const int colon = t.indexOf(':');
    if (colon <= 0) {
      continue;
    }
    const QString key = t.left(colon).trimmed();
    QString val = trim_value(t.mid(colon + 1));
    if (key == QStringLiteral("recommended_calibrators")) {
      if (val.isEmpty()) {
        p.recommended_calibrators = parse_list_block(in);
      }
      continue;
    }
    if (val == QStringLiteral("|") || val == QStringLiteral(">")) {
      multiline_key = key;
      multiline_acc.clear();
      continue;
    }
    if (key == QStringLiteral("id")) {
      p.id = val;
    } else if (key == QStringLiteral("display_name") || key == QStringLiteral("name")) {
      p.display_name = val;
    } else if (key == QStringLiteral("description")) {
      p.description = val;
    } else if (key == QStringLiteral("notes")) {
      p.notes = val;
    } else if (key == QStringLiteral("parent_frame")) {
      p.parent_frame = val;
    } else if (key == QStringLiteral("child_frame")) {
      p.child_frame = val;
    } else if (key == QStringLiteral("base_frame")) {
      p.base_frame = val;
    } else if (key == QStringLiteral("gripper_frame")) {
      p.gripper_frame = val;
    } else if (key == QStringLiteral("image_frame")) {
      p.image_frame = val;
    } else if (key == QStringLiteral("camera_link_frame")) {
      p.camera_link_frame = val;
    }
  }
  flush_multi();
  if (p.id.isEmpty()) {
    p.id = QFileInfo(path).baseName();
  }
  if (p.display_name.isEmpty()) {
    p.display_name = p.id;
  }
  *out = p;
  return true;
}

}  // namespace

QString ProjectCatalog::user_projects_dir() {
  return ProjectWorkspace::default_projects_root();
}

QString ProjectCatalog::legacy_user_yaml_dir() {
  const QString root =
      QStandardPaths::writableLocation(QStandardPaths::ConfigLocation) +
      QStringLiteral("/hs_calib_suite/projects");
  QDir().mkpath(root);
  return root;
}

QString ProjectCatalog::package_projects_dir() {
  try {
    const auto share = ament_index_cpp::get_package_share_directory("hs_calib_suite");
    return QString::fromStdString(share) + QStringLiteral("/config/projects");
  } catch (...) {
    return {};
  }
}

void ProjectCatalog::open_user_projects_dir() {
  const QString dir = user_projects_dir();
  QDesktopServices::openUrl(QUrl::fromLocalFile(dir));
}

ProjectInfo ProjectCatalog::builtin_template(const QString &id) {
  ProjectInfo p;
  p.id = id;
  p.is_template = true;
  if (id == QStringLiteral("default_robot")) {
    p.display_name = QStringLiteral("Default Robot");
    p.description = QStringLiteral("通用机械臂 + 相机：先内参，再手眼。");
    p.notes = QStringLiteral("默认坐标系：base / tool0 / camera_link。");
    p.recommended_calibrators << QStringLiteral("cam_intrinsics")
                              << QStringLiteral("eye_in_hand")
                              << QStringLiteral("trihedral_oneshot");
  } else if (id == QStringLiteral("arm_cell_A")) {
    p.display_name = QStringLiteral("Arm Cell A");
    p.description = QStringLiteral("工位 A：固定相机（眼在手外）+ 桌面靶标。");
    p.notes = QStringLiteral("相机固定在工位支架；手眼请选 eye_to_hand。");
    p.base_frame = QStringLiteral("cell_A_base");
    p.gripper_frame = QStringLiteral("flange");
    p.parent_frame = QStringLiteral("cell_A_camera_link");
    p.child_frame = QStringLiteral("cell_A_camera_optical");
    p.camera_link_frame = QStringLiteral("cell_A_camera_link");
    p.image_frame = QStringLiteral("cell_A_camera_optical");
    p.recommended_calibrators << QStringLiteral("cam_intrinsics")
                              << QStringLiteral("eye_to_hand");
  } else if (id == QStringLiteral("mobile_base_01")) {
    p.display_name = QStringLiteral("Mobile Base 01");
    p.description = QStringLiteral("移动底盘 + 云台/腕部相机；注意 TF 树根。");
    p.notes = QStringLiteral("base_frame 用底盘；采集时保持底盘静止更稳。");
    p.base_frame = QStringLiteral("base_link");
    p.gripper_frame = QStringLiteral("wrist_camera_mount");
    p.parent_frame = QStringLiteral("wrist_camera_link");
    p.child_frame = QStringLiteral("wrist_camera_optical");
    p.camera_link_frame = QStringLiteral("wrist_camera_link");
    p.image_frame = QStringLiteral("wrist_camera_optical");
    p.recommended_calibrators << QStringLiteral("cam_intrinsics")
                              << QStringLiteral("eye_in_hand")
                              << QStringLiteral("detect_lab")
                              << QStringLiteral("detect_lab_full");
  } else {
    p.display_name = id;
    p.description = QStringLiteral("自定义项目模板");
  }
  return p;
}

void ProjectCatalog::reload() {
  projects_.clear();
  QMap<QString, ProjectInfo> folders;
  QMap<QString, ProjectInfo> templates;

  // 1) 用户文件夹项目：Documents/hs_calib_projects/<id>/project.yaml
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

  // 2) 包内扁平 yaml = 模板（若同 id 已有文件夹项目则不再显示模板）
  for (const char *id : {"default_robot", "arm_cell_A", "mobile_base_01"}) {
    templates.insert(QString::fromUtf8(id), builtin_template(QString::fromUtf8(id)));
  }
  {
    QDir d(package_projects_dir());
    if (d.exists()) {
      for (const QString &name :
           d.entryList({QStringLiteral("*.yaml"), QStringLiteral("*.yml")}, QDir::Files)) {
        ProjectInfo p;
        if (!load_flat_yaml(d.filePath(name), &p)) {
          continue;
        }
        p.is_template = true;
        p.user_writable = false;
        templates.insert(p.id, p);
      }
    }
  }

  // 3) 旧版 ~/.config/.../projects/*.yaml —— 视为可迁移模板
  {
    QDir d(legacy_user_yaml_dir());
    if (d.exists()) {
      for (const QString &name :
           d.entryList({QStringLiteral("*.yaml"), QStringLiteral("*.yml")}, QDir::Files)) {
        ProjectInfo p;
        if (!load_flat_yaml(d.filePath(name), &p)) {
          continue;
        }
        p.is_template = true;
        p.user_writable = true;
        p.notes = (p.notes.isEmpty() ? QString() : p.notes + QStringLiteral("\n")) +
                  QStringLiteral("（旧版扁平 yaml，打开时将实例化为文件夹项目）");
        if (!folders.contains(p.id)) {
          templates.insert(p.id, p);
        }
      }
    }
  }

  QVector<ProjectInfo> folder_list = folders.values().toVector();
  std::sort(folder_list.begin(), folder_list.end(), [](const ProjectInfo &a, const ProjectInfo &b) {
    return a.display_name.toLower() < b.display_name.toLower();
  });

  QVector<ProjectInfo> template_list;
  for (auto it = templates.begin(); it != templates.end(); ++it) {
    if (folders.contains(it.key())) {
      continue;  // 已有同名文件夹项目
    }
    template_list.push_back(it.value());
  }
  std::sort(
      template_list.begin(), template_list.end(),
      [](const ProjectInfo &a, const ProjectInfo &b) {
        return a.display_name.toLower() < b.display_name.toLower();
      });

  projects_ = folder_list;
  projects_.append(template_list);
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

bool ProjectCatalog::materialize_template(
    const QString &template_id, QString *root_out, QString *error_out) {
  const ProjectInfo *t = find(template_id);
  if (t == nullptr) {
    if (error_out) {
      *error_out = QStringLiteral("找不到模板：%1").arg(template_id);
    }
    return false;
  }
  // 已是文件夹项目
  if (t->is_folder_project && !t->root_path.isEmpty()) {
    if (root_out) {
      *root_out = t->root_path;
    }
    return true;
  }
  ProjectInfo info = *t;
  info.is_template = false;
  info.user_writable = true;
  info.root_path.clear();
  info.source_path.clear();
  // 若模板来自磁盘 yaml，优先用文件内容（更完整）
  if (!t->source_path.isEmpty() && QFile::exists(t->source_path)) {
    QString root;
    if (ProjectWorkspace::create_from_template(
            t->source_path, user_projects_dir(), &root, error_out)) {
      if (root_out) {
        *root_out = root;
      }
      reload();
      return true;
    }
  }
  QString root;
  if (!ProjectWorkspace::create(user_projects_dir(), info, &root, error_out)) {
    return false;
  }
  if (root_out) {
    *root_out = root;
  }
  reload();
  return true;
}

}  // namespace gui
}  // namespace hs_calib
