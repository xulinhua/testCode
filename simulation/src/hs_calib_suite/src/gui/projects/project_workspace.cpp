#include "hs_calib_suite/gui/projects/project_workspace.hpp"

#include <QDateTime>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <QTextStream>

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

bool copy_file_overwrite(const QString &src, const QString &dst, QString *error_out) {
  if (QFile::exists(dst) && !QFile::remove(dst)) {
    if (error_out) {
      *error_out = QStringLiteral("无法覆盖：%1").arg(dst);
    }
    return false;
  }
  if (!QFile::copy(src, dst)) {
    if (error_out) {
      *error_out = QStringLiteral("复制失败：%1 → %2").arg(src, dst);
    }
    return false;
  }
  return true;
}

bool copy_dir_recursive(const QString &src, const QString &dst, QString *error_out) {
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
  const auto entries = s.entryList(QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);
  for (const QString &name : entries) {
    const QString sp = s.filePath(name);
    const QString dp = QDir(dst).filePath(name);
    QFileInfo fi(sp);
    if (fi.isDir()) {
      if (!copy_dir_recursive(sp, dp, error_out)) {
        return false;
      }
    } else {
      if (!copy_file_overwrite(sp, dp, error_out)) {
        return false;
      }
    }
  }
  return true;
}

bool is_image_name(const QString &name) {
  const QString l = name.toLower();
  return l.endsWith(".png") || l.endsWith(".jpg") || l.endsWith(".jpeg") ||
         l.endsWith(".bmp") || l.endsWith(".tif") || l.endsWith(".tiff");
}

}  // namespace

QString ProjectWorkspace::default_projects_root() {
  const QString root =
      QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) +
      QStringLiteral("/hs_calib_projects");
  QDir().mkpath(root);
  return root;
}

bool ProjectWorkspace::is_project_root(const QString &dir) {
  return QFileInfo::exists(QDir(dir).filePath(QStringLiteral("project.yaml")));
}

QString ProjectWorkspace::project_yaml_path() const {
  return QDir(root_).filePath(QStringLiteral("project.yaml"));
}

QString ProjectWorkspace::config_dir() const {
  return QDir(root_).filePath(QStringLiteral("config"));
}

QString ProjectWorkspace::images_dir() const {
  return QDir(root_).filePath(QStringLiteral("images"));
}

QString ProjectWorkspace::results_dir() const {
  return QDir(root_).filePath(QStringLiteral("results"));
}

bool ProjectWorkspace::ensure_layout(QString *error_out) const {
  if (root_.isEmpty()) {
    if (error_out) {
      *error_out = QStringLiteral("项目未打开");
    }
    return false;
  }
  for (const QString &sub : {config_dir(), images_dir(), results_dir()}) {
    if (!QDir().mkpath(sub)) {
      if (error_out) {
        *error_out = QStringLiteral("无法创建目录：%1").arg(sub);
      }
      return false;
    }
  }
  return true;
}

bool ProjectWorkspace::write_project_yaml(
    const QString &path, const ProjectInfo &info, QString *error_out) {
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
    if (error_out) {
      *error_out = QStringLiteral("无法写入 %1").arg(path);
    }
    return false;
  }
  QTextStream out(&f);
  out << "# hs_calib_suite folder project\n";
  out << "id: " << info.id << "\n";
  out << "display_name: " << info.display_name << "\n";
  out << "description: |\n";
  for (const QString &line : info.description.split('\n')) {
    out << "  " << line << "\n";
  }
  out << "notes: |\n";
  for (const QString &line : info.notes.split('\n')) {
    out << "  " << line << "\n";
  }
  out << "parent_frame: " << info.parent_frame << "\n";
  out << "child_frame: " << info.child_frame << "\n";
  out << "base_frame: " << info.base_frame << "\n";
  out << "gripper_frame: " << info.gripper_frame << "\n";
  out << "image_frame: " << info.image_frame << "\n";
  out << "camera_link_frame: " << info.camera_link_frame << "\n";
  if (!info.image_topic.isEmpty()) {
    out << "image_topic: " << info.image_topic << "\n";
  }
  if (!info.camera_info_topic.isEmpty()) {
    out << "camera_info_topic: " << info.camera_info_topic << "\n";
  }
  if (!info.pose_source.isEmpty()) {
    out << "pose_source: " << info.pose_source << "\n";
  }
  if (!info.last_calibrator_id.isEmpty()) {
    out << "last_calibrator_id: " << info.last_calibrator_id << "\n";
  }
  if (!info.default_image_subdir.isEmpty()) {
    out << "default_image_subdir: " << info.default_image_subdir << "\n";
  }
  out << "recommended_calibrators:\n";
  for (const QString &c : info.recommended_calibrators) {
    out << "  - " << c << "\n";
  }
  return true;
}

bool ProjectWorkspace::read_project_yaml(
    const QString &path, ProjectInfo *info, QString *error_out) {
  if (info == nullptr) {
    return false;
  }
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
    if (error_out) {
      *error_out = QStringLiteral("无法读取 %1").arg(path);
    }
    return false;
  }
  ProjectInfo p;
  p.source_path = path;
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
    } else if (key == QStringLiteral("image_topic")) {
      p.image_topic = val;
    } else if (key == QStringLiteral("camera_info_topic")) {
      p.camera_info_topic = val;
    } else if (key == QStringLiteral("pose_source")) {
      p.pose_source = val;
    } else if (key == QStringLiteral("last_calibrator_id")) {
      p.last_calibrator_id = val;
    } else if (key == QStringLiteral("default_image_subdir")) {
      p.default_image_subdir = val;
    }
  }
  flush_multi();
  if (p.id.isEmpty()) {
    p.id = QFileInfo(QFileInfo(path).absolutePath()).fileName();
  }
  if (p.display_name.isEmpty()) {
    p.display_name = p.id;
  }
  const QFileInfo pfi(path);
  if (pfi.fileName() == QStringLiteral("project.yaml")) {
    p.root_path = pfi.absolutePath();
    p.is_folder_project = true;
    p.user_writable = QFileInfo(p.root_path).isWritable();
  } else {
    p.root_path.clear();
    p.is_folder_project = false;
  }
  *info = p;
  return true;
}

bool ProjectWorkspace::create(
    const QString &parent_dir, const ProjectInfo &info, QString *root_out,
    QString *error_out) {
  if (info.id.isEmpty()) {
    if (error_out) {
      *error_out = QStringLiteral("项目 id 不能为空");
    }
    return false;
  }
  for (const QChar c : info.id) {
    if (!(c.isLetterOrNumber() || c == '_' || c == '-')) {
      if (error_out) {
        *error_out = QStringLiteral("id 仅允许字母数字、_、-");
      }
      return false;
    }
  }
  if (!QDir().mkpath(parent_dir)) {
    if (error_out) {
      *error_out = QStringLiteral("无法创建父目录：%1").arg(parent_dir);
    }
    return false;
  }
  const QString root = QDir(parent_dir).filePath(info.id);
  if (QDir(root).exists()) {
    if (error_out) {
      *error_out = QStringLiteral("项目目录已存在：%1").arg(root);
    }
    return false;
  }
  if (!QDir().mkpath(root)) {
    if (error_out) {
      *error_out = QStringLiteral("无法创建项目目录：%1").arg(root);
    }
    return false;
  }
  ProjectInfo to_save = info;
  if (to_save.display_name.isEmpty()) {
    to_save.display_name = to_save.id;
  }
  if (to_save.recommended_calibrators.isEmpty()) {
    to_save.recommended_calibrators << QStringLiteral("cam_intrinsics")
                                    << QStringLiteral("eye_in_hand");
  }
  to_save.root_path = root;
  to_save.is_folder_project = true;
  to_save.user_writable = true;
  to_save.source_path = QDir(root).filePath(QStringLiteral("project.yaml"));

  ProjectWorkspace tmp;
  tmp.root_ = root;
  tmp.meta_ = to_save;
  if (!tmp.ensure_layout(error_out)) {
    return false;
  }
  if (!write_project_yaml(tmp.project_yaml_path(), to_save, error_out)) {
    return false;
  }
  // 占位说明，便于用户打开文件夹就明白结构
  {
    QFile readme(QDir(root).filePath(QStringLiteral("README.txt")));
    if (readme.open(QIODevice::WriteOnly | QIODevice::Text)) {
      QTextStream o(&readme);
      o << "hs_calib_suite project folder\n"
        << "config/  — session / board configs\n"
        << "images/  — calibration images\n"
        << "results/ — exported calibration bundles\n"
        << "project.yaml — project metadata\n";
    }
  }
  if (root_out) {
    *root_out = root;
  }
  return true;
}

bool ProjectWorkspace::create_from_template(
    const QString &template_yaml, const QString &parent_dir, QString *root_out,
    QString *error_out) {
  ProjectInfo info;
  if (!read_project_yaml(template_yaml, &info, error_out)) {
    // 模板可能是旧版扁平 yaml（无与 project.yaml 同语法）；失败则报错
    return false;
  }
  info.source_path.clear();
  info.root_path.clear();
  info.is_folder_project = false;
  info.user_writable = true;
  return create(parent_dir, info, root_out, error_out);
}

bool ProjectWorkspace::open(const QString &root_dir, QString *error_out) {
  const QString root = QFileInfo(root_dir).absoluteFilePath();
  if (!is_project_root(root)) {
    if (error_out) {
      *error_out = QStringLiteral("不是项目根目录（缺少 project.yaml）：%1").arg(root);
    }
    return false;
  }
  ProjectInfo info;
  if (!read_project_yaml(QDir(root).filePath(QStringLiteral("project.yaml")), &info, error_out)) {
    return false;
  }
  root_ = root;
  meta_ = info;
  meta_.root_path = root_;
  meta_.is_folder_project = true;
  meta_.source_path = project_yaml_path();
  return ensure_layout(error_out);
}

void ProjectWorkspace::close() {
  root_.clear();
  meta_ = ProjectInfo{};
}

bool ProjectWorkspace::save_meta(QString *error_out) const {
  if (!is_open()) {
    if (error_out) {
      *error_out = QStringLiteral("项目未打开");
    }
    return false;
  }
  return write_project_yaml(project_yaml_path(), meta_, error_out);
}

bool ProjectWorkspace::reload_meta(QString *error_out) {
  if (!is_open()) {
    if (error_out) {
      *error_out = QStringLiteral("项目未打开");
    }
    return false;
  }
  ProjectInfo info;
  if (!read_project_yaml(project_yaml_path(), &info, error_out)) {
    return false;
  }
  meta_ = info;
  return true;
}

bool ProjectWorkspace::import_config_file(const QString &src_path, QString *error_out) const {
  if (!ensure_layout(error_out)) {
    return false;
  }
  const QFileInfo fi(src_path);
  if (!fi.exists() || !fi.isFile()) {
    if (error_out) {
      *error_out = QStringLiteral("配置文件不存在：%1").arg(src_path);
    }
    return false;
  }
  const QString dst = QDir(config_dir()).filePath(fi.fileName());
  return copy_file_overwrite(src_path, dst, error_out);
}

QStringList ProjectWorkspace::list_config_files() const {
  QStringList out;
  QDir d(config_dir());
  if (!d.exists()) {
    return out;
  }
  for (const QString &n : d.entryList({QStringLiteral("*.yaml"), QStringLiteral("*.yml")},
                                      QDir::Files, QDir::Name)) {
    out.push_back(d.filePath(n));
  }
  return out;
}

int ProjectWorkspace::import_images(
    const QStringList &file_paths, const QString &subdir, QString *error_out) const {
  if (!ensure_layout(error_out)) {
    return 0;
  }
  QString dest = images_dir();
  if (!subdir.isEmpty()) {
    dest = QDir(images_dir()).filePath(subdir);
    if (!QDir().mkpath(dest)) {
      if (error_out) {
        *error_out = QStringLiteral("无法创建：%1").arg(dest);
      }
      return 0;
    }
  }
  int n = 0;
  for (const QString &src : file_paths) {
    const QFileInfo fi(src);
    if (!fi.exists() || !fi.isFile() || !is_image_name(fi.fileName())) {
      continue;
    }
    const QString dst = QDir(dest).filePath(fi.fileName());
    if (copy_file_overwrite(src, dst, error_out)) {
      ++n;
    } else {
      return n;
    }
  }
  return n;
}

bool ProjectWorkspace::import_image_directory(
    const QString &src_dir, const QString &subdir, QString *error_out) const {
  if (!ensure_layout(error_out)) {
    return false;
  }
  QString name = subdir;
  if (name.isEmpty()) {
    name = QFileInfo(src_dir).fileName();
  }
  const QString dest = QDir(images_dir()).filePath(name);
  return copy_dir_recursive(src_dir, dest, error_out);
}

QStringList ProjectWorkspace::list_images() const {
  QStringList out;
  QDirIterator it(
      images_dir(), QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext()) {
    const QString p = it.next();
    if (is_image_name(p)) {
      out.push_back(p);
    }
  }
  out.sort();
  return out;
}

QString ProjectWorkspace::preferred_image_dir() const {
  if (!is_open()) {
    return {};
  }
  if (!meta_.default_image_subdir.isEmpty()) {
    const QString sub = QDir(images_dir()).filePath(meta_.default_image_subdir);
    if (QDir(sub).exists()) {
      return sub;
    }
  }
  // 若 images/ 下只有一个子目录，优先用它
  QDir d(images_dir());
  const auto subs = d.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
  if (subs.size() == 1) {
    return d.filePath(subs.front());
  }
  // 根下若直接有图，用 images/
  for (const QString &n : d.entryList(QDir::Files)) {
    if (is_image_name(n)) {
      return images_dir();
    }
  }
  if (!subs.isEmpty()) {
    return d.filePath(subs.front());
  }
  return images_dir();
}

bool ProjectWorkspace::archive_result_bundle(
    const QString &bundle_dir, const QString &name_hint, QString *archived_path_out,
    QString *error_out) const {
  if (!ensure_layout(error_out)) {
    return false;
  }
  QString name = name_hint.trimmed();
  if (name.isEmpty()) {
    name = QFileInfo(bundle_dir).fileName();
  }
  if (name.isEmpty()) {
    name = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
  }
  // 避免覆盖：重名则加后缀
  QString dest = QDir(results_dir()).filePath(name);
  int suffix = 1;
  while (QDir(dest).exists()) {
    dest = QDir(results_dir()).filePath(QStringLiteral("%1_%2").arg(name).arg(suffix++));
  }
  if (!copy_dir_recursive(bundle_dir, dest, error_out)) {
    return false;
  }
  if (archived_path_out) {
    *archived_path_out = dest;
  }
  return true;
}

QVector<ProjectResultEntry> ProjectWorkspace::list_results() const {
  QVector<ProjectResultEntry> out;
  QDir d(results_dir());
  if (!d.exists()) {
    return out;
  }
  for (const QString &name : d.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Time)) {
    ProjectResultEntry e;
    e.name = name;
    e.path = d.filePath(name);
    const QString cfg = QDir(e.path).filePath(QStringLiteral("session_config.yaml"));
    if (QFile::exists(cfg)) {
      QFile f(cfg);
      if (f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream in(&f);
        while (!in.atEnd()) {
          const QString line = in.readLine().trimmed();
          if (line.startsWith(QStringLiteral("calibrator_id:"))) {
            e.calibrator = trim_value(line.mid(QStringLiteral("calibrator_id:").size()));
            break;
          }
        }
      }
    }
    QStringList bits;
    if (!e.calibrator.isEmpty()) {
      bits << e.calibrator;
    }
    bits << e.name;
    e.summary = bits.join(QStringLiteral(" · "));
    out.push_back(e);
  }
  return out;
}

QString ProjectWorkspace::suggest_export_dir(const QString &calibrator_id) const {
  if (!is_open()) {
    return {};
  }
  const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
  const QString name = QStringLiteral("hs_calib_%1_%2")
                           .arg(calibrator_id.isEmpty() ? QStringLiteral("result") : calibrator_id)
                           .arg(stamp);
  return QDir(results_dir()).filePath(name);
}

}  // namespace gui
}  // namespace hs_calib
