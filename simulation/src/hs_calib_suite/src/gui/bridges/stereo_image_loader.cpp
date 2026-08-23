#include "hs_calib_suite/gui/bridges/stereo_image_loader.hpp"

#include <algorithm>

#include <algorithm>
#include <QDir>
#include <QFileInfo>
#include <QRegularExpression>

namespace hs_calib {
namespace gui {

namespace {

const QStringList &image_filters() {
  static const QStringList filters = {
      QStringLiteral("*.png"),  QStringLiteral("*.jpg"),
      QStringLiteral("*.jpeg"), QStringLiteral("*.bmp"),
      QStringLiteral("*.PNG"),  QStringLiteral("*.JPG"),
      QStringLiteral("*.JPEG"), QStringLiteral("*.BMP"),
  };
  return filters;
}

QStringList scan_dir(const QString &dir_path) {
  QStringList out;
  QDir dir(dir_path);
  if (!dir.exists()) {
    return out;
  }
  const QFileInfoList files =
      dir.entryInfoList(image_filters(), QDir::Files, QDir::Name);
  out.reserve(files.size());
  for (const QFileInfo &fi : files) {
    out.push_back(fi.absoluteFilePath());
  }
  return out;
}

bool try_subdirs(const QString &root, StereoImageDirScan *out) {
  const QString left = QDir(root).filePath(QStringLiteral("left"));
  const QString right = QDir(root).filePath(QStringLiteral("right"));
  if (!QDir(left).exists() || !QDir(right).exists()) {
    return false;
  }
  out->left_dir = left;
  out->right_dir = right;
  out->left_paths = scan_dir(left);
  out->right_paths = scan_dir(right);
  return out->valid();
}

bool suffix_is_left(const QString &base) {
  static const QRegularExpression re(
      QStringLiteral("([._-]L|_left|_Left|_LEFT)$"),
      QRegularExpression::CaseInsensitiveOption);
  return re.match(base).hasMatch();
}

bool suffix_is_right(const QString &base) {
  static const QRegularExpression re(
      QStringLiteral("([._-]R|_right|_Right|_RIGHT)$"),
      QRegularExpression::CaseInsensitiveOption);
  return re.match(base).hasMatch();
}

bool try_lr_suffixes(const QString &root, StereoImageDirScan *out) {
  QDir dir(root);
  if (!dir.exists()) {
    return false;
  }
  const QFileInfoList files =
      dir.entryInfoList(image_filters(), QDir::Files, QDir::Name);
  QStringList left;
  QStringList right;
  for (const QFileInfo &fi : files) {
    const QString base = fi.completeBaseName();
    if (suffix_is_left(base)) {
      left.push_back(fi.absoluteFilePath());
    } else if (suffix_is_right(base)) {
      right.push_back(fi.absoluteFilePath());
    }
  }
  if (left.isEmpty() || right.isEmpty()) {
    return false;
  }
  out->left_dir = root;
  out->right_dir = root;
  out->left_paths = left;
  out->right_paths = right;
  return true;
}

}  // namespace

StereoImageDirScan StereoImageLoader::scan(
    const QString &root_or_left_dir, const QString &right_dir) {
  StereoImageDirScan out;
  if (!right_dir.trimmed().isEmpty()) {
    out.left_dir = root_or_left_dir;
    out.right_dir = right_dir;
    out.left_paths = scan_dir(out.left_dir);
    out.right_paths = scan_dir(out.right_dir);
    if (!out.valid()) {
      out.message = QStringLiteral("左右目录均无有效图片");
    }
    return out;
  }

  const QString root = root_or_left_dir.trimmed();
  if (try_subdirs(root, &out)) {
    return out;
  }
  if (try_lr_suffixes(root, &out)) {
    return out;
  }
  // 单目录回退：全部当作左目（兼容旧流程，右目为空）
  out.left_dir = root;
  out.left_paths = scan_dir(root);
  out.message = QStringLiteral(
      "未找到 left/ right/ 子目录或 *_L/*_R 命名；仅加载为左目序列");
  return out;
}

}  // namespace gui
}  // namespace hs_calib
