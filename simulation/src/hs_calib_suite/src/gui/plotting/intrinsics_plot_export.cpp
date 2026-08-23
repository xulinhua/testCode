#include "hs_calib_suite/gui/plotting/intrinsics_plot_export.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include "hs_calib_suite/gui/intrinsics/intrinsics_matplotlib_plotter.hpp"
#include "hs_calib_suite/gui/plotting/intrinsics_plot_renderer.hpp"
#include "hs_calib_suite/gui/plotting/intrinsics_plot_session.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"

namespace hs_calib {
namespace gui {
namespace {

bool save_plot(
    IntrinsicsPlotKind kind,
    const core::IntrinsicsPlotInput &input,
    const QString &path,
    std::string *error_out) {
  core::IntrinsicsPipelineStageViews stages;
  std::string err;
  if (!core::build_plot_pipeline_stages(input, &stages, &err)) {
    if (error_out) {
      *error_out = err;
    }
    return false;
  }

  if (kind == IntrinsicsPlotKind::CalibrationBars && !input.has_calibrated) {
    if (error_out) {
      *error_out = "需要标定模型";
    }
    return false;
  }
  if (kind == IntrinsicsPlotKind::CalibrationRmsHeatmaps && !input.has_calibrated) {
    if (error_out) {
      *error_out = "需要标定模型";
    }
    return false;
  }

  const QPixmap pm = IntrinsicsPlotRenderer::render(kind, input, stages, &err);
  if (pm.isNull()) {
    if (error_out) {
      *error_out = err.empty() ? "matplotlib 渲染失败" : err;
    }
    return false;
  }
  if (!pm.save(path)) {
    if (error_out) {
      *error_out = "无法写入 PNG";
    }
    return false;
  }
  return true;
}

void append_summary(QString *summary, const QString &line) {
  if (summary == nullptr || line.isEmpty()) {
    return;
  }
  if (!summary->isEmpty()) {
    *summary += QStringLiteral("\n");
  }
  *summary += line;
}

core::IntrinsicsPlotInput make_plot_input(const SessionController &session) {
  core::IntrinsicsPlotInput input;
  if (!build_intrinsics_plot_input(session, &input)) {
    return input;
  }
  input.owned_training_batch = session.intrinsics_state().training_batch();
  input.owned_evaluation_batch = session.intrinsics_state().evaluation_batch();
  input.has_owned_batches = true;
  input.collector = nullptr;
  return input;
}

}  // namespace

QStringList export_intrinsics_statistics_pngs(
    const SessionController &session,
    const QString &output_dir,
    const std::string &stats_backend,
    QString *summary_out) {
  QStringList written;
  if (!session.is_intrinsics() || !session.uses_tier4_intrinsics()) {
    return written;
  }

  const QDir out_dir(output_dir);
  if (!out_dir.exists() && !out_dir.mkpath(QStringLiteral("."))) {
    append_summary(summary_out, QStringLiteral("无法创建导出目录"));
    return written;
  }

  core::IntrinsicsPlotInput input = make_plot_input(session);
  if (input.image_width <= 0 || input.image_height <= 0) {
    append_summary(summary_out, QStringLiteral("缺少图像尺寸，跳过统计图导出"));
    return written;
  }

  if (input.training_batch().items.size() < 3) {
    append_summary(summary_out, QStringLiteral("训练样本不足，跳过采集统计图"));
  } else {
    const QString path =
        out_dir.filePath(QStringLiteral("calibration_data_statistics.png"));
    std::string err;
    if (save_plot(IntrinsicsPlotKind::CollectionDataStatistics, input, path, &err)) {
      written << QFileInfo(path).fileName();
    } else if (stats_backend == "qt") {
      const QPixmap pm = IntrinsicsStatsPlotter::render_qt_summary(
          session.intrinsics_state().collector());
      if (!pm.isNull() && pm.save(path)) {
        written << QFileInfo(path).fileName();
      } else {
        append_summary(
            summary_out,
            QStringLiteral("采集统计图导出失败：%1")
                .arg(QString::fromStdString(err)));
      }
    } else {
      append_summary(
          summary_out,
          QStringLiteral("采集统计图导出失败：%1")
              .arg(QString::fromStdString(err)));
    }
  }

  if (!input.has_calibrated) {
    append_summary(
        summary_out, QStringLiteral("尚未标定，跳过标定结果统计图导出"));
    return written;
  }

  {
    const QString path = out_dir.filePath(
        QStringLiteral("calibration_result_vs_singleshot.png"));
    std::string err;
    if (save_plot(IntrinsicsPlotKind::CalibrationBars, input, path, &err)) {
      written << QFileInfo(path).fileName();
    } else {
      append_summary(
          summary_out,
          QStringLiteral("single-shot 对比图导出失败：%1")
              .arg(QString::fromStdString(err)));
    }
  }

  {
    const QString path =
        out_dir.filePath(QStringLiteral("calibration_result_rms.png"));
    std::string err;
    if (save_plot(IntrinsicsPlotKind::CalibrationRmsHeatmaps, input, path, &err)) {
      written << QFileInfo(path).fileName();
    } else {
      append_summary(
          summary_out,
          QStringLiteral("RMS 热力图导出失败：%1")
              .arg(QString::fromStdString(err)));
    }
  }

  return written;
}

}  // namespace gui
}  // namespace hs_calib
