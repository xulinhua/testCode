#include "hs_calib_suite/gui/plotting/intrinsics_plot_renderer.hpp"

#include <fstream>

#include <QProcess>
#include <QTemporaryFile>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_plot_statistics.hpp"

namespace hs_calib {
namespace gui {
namespace {

std::string scripts_dir() {
  try {
    return ament_index_cpp::get_package_share_directory("hs_calib_suite") + "/scripts";
  } catch (...) {
    return "scripts";
  }
}

QString script_for(IntrinsicsPlotKind kind) {
  switch (kind) {
    case IntrinsicsPlotKind::CollectionDataStatistics:
      return QStringLiteral("intrinsics_collection_stats_plot.py");
    case IntrinsicsPlotKind::CalibrationBars:
      return QStringLiteral("intrinsics_calibration_bars_plot.py");
    case IntrinsicsPlotKind::CalibrationRmsHeatmaps:
      return QStringLiteral("intrinsics_calibration_rms_plot.py");
  }
  return {};
}

bool export_json(
    IntrinsicsPlotKind kind,
    const core::IntrinsicsPlotInput &input,
    const core::IntrinsicsPipelineStageViews &stages,
    const std::string &path,
    std::string *error_out) {
  switch (kind) {
    case IntrinsicsPlotKind::CollectionDataStatistics:
      return core::export_collection_statistics_json(input, stages, path, error_out);
    case IntrinsicsPlotKind::CalibrationBars:
      return core::export_calibration_bars_json(input, stages, path, error_out);
    case IntrinsicsPlotKind::CalibrationRmsHeatmaps:
      return core::export_calibration_rms_json(input, stages, path, error_out);
  }
  if (error_out) {
    *error_out = "未知图类型";
  }
  return false;
}

}  // namespace

const char *IntrinsicsPlotRenderer::kind_title(IntrinsicsPlotKind kind) {
  switch (kind) {
    case IntrinsicsPlotKind::CollectionDataStatistics:
      return "Calibration data statistics";
    case IntrinsicsPlotKind::CalibrationBars:
      return "Calibration result statistics vs single-shot calibration";
    case IntrinsicsPlotKind::CalibrationRmsHeatmaps:
      return "Calibration result statistics";
  }
  return "Intrinsics statistics";
}

QPixmap IntrinsicsPlotRenderer::render(
    IntrinsicsPlotKind kind,
    const core::IntrinsicsPlotInput &input,
    const core::IntrinsicsPipelineStageViews &stages,
    std::string *error_out) {
  QTemporaryFile json_file(QStringLiteral("hs_intrinsics_plot_XXXXXX.json"));
  QTemporaryFile png_file(QStringLiteral("hs_intrinsics_plot_XXXXXX.png"));
  json_file.setAutoRemove(true);
  png_file.setAutoRemove(false);
  if (!json_file.open() || !png_file.open()) {
    if (error_out) {
      *error_out = "无法创建临时文件";
    }
    return {};
  }
  if (!export_json(kind, input, stages, json_file.fileName().toStdString(), error_out)) {
    return {};
  }
  json_file.close();
  png_file.close();

  const QString script_path =
      QString::fromStdString(scripts_dir()) + QLatin1Char('/') + script_for(kind);
  QProcess proc;
  proc.setWorkingDirectory(QString::fromStdString(scripts_dir()));
  proc.start(
      QStringLiteral("python3"),
      {script_path, json_file.fileName(), png_file.fileName()});
  if (!proc.waitForFinished(60000)) {
    if (error_out) {
      *error_out = "matplotlib 脚本超时或未安装 python3";
    }
    return {};
  }
  if (proc.exitCode() != 0) {
    if (error_out) {
      *error_out = proc.readAllStandardError().toStdString();
      if (error_out->empty()) {
        *error_out = "matplotlib 脚本失败（需 pip install matplotlib numpy）";
      }
    }
    return {};
  }
  QPixmap pm;
  if (!pm.load(png_file.fileName())) {
    if (error_out) {
      *error_out = "无法加载生成的 PNG";
    }
    return {};
  }
  return pm;
}

}  // namespace gui
}  // namespace hs_calib
