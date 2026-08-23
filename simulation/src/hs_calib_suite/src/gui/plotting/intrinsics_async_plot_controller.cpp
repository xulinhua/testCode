#include "hs_calib_suite/gui/plotting/intrinsics_async_plot_controller.hpp"

#include <thread>
#include <utility>

#include <QBuffer>

#include "hs_calib_suite/gui/plotting/intrinsics_plot_renderer.hpp"
#include "hs_calib_suite/gui/plotting/intrinsics_plot_session.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"

namespace hs_calib {
namespace gui {
namespace {

IntrinsicsPlotJobResult run_plot_job(
    const core::IntrinsicsPlotInput &input, IntrinsicsPlotKind kind) {
  IntrinsicsPlotJobResult out;
  core::IntrinsicsPipelineStageViews stages;
  std::string err;
  if (!core::build_plot_pipeline_stages(input, &stages, &err)) {
    out.error = err.empty() ? "无法构建流水线阶段" : err;
    return out;
  }

  if (kind == IntrinsicsPlotKind::CalibrationBars && !input.has_calibrated) {
    out.error = "需要标定模型";
    return out;
  }
  if (kind == IntrinsicsPlotKind::CalibrationRmsHeatmaps && !input.has_calibrated) {
    out.error = "需要标定模型";
    return out;
  }

  const QPixmap pm = IntrinsicsPlotRenderer::render(kind, input, stages, &err);
  if (pm.isNull()) {
    out.error = err.empty() ? "matplotlib 渲染失败" : err;
    return out;
  }

  QBuffer buffer;
  buffer.open(QIODevice::WriteOnly);
  if (!pm.save(&buffer, "PNG")) {
    out.error = "无法编码 PNG";
    return out;
  }
  out.png_bytes = buffer.data();
  out.ok = true;
  out.backend_label = "matplotlib";
  return out;
}

}  // namespace

IntrinsicsAsyncPlotController::IntrinsicsAsyncPlotController(QObject *parent)
    : QObject(parent) {}

IntrinsicsAsyncPlotController::~IntrinsicsAsyncPlotController() {
  cancel();
}

void IntrinsicsAsyncPlotController::cancel() {
  ++generation_;
  running_ = false;
}

void IntrinsicsAsyncPlotController::start(
    const SessionController *session,
    IntrinsicsPlotKind kind,
    const std::string &backend) {
  cancel();
  pending_backend_ = backend;
  if (session == nullptr || !session->is_intrinsics()) {
    emit plot_finished(
        false, {}, QStringLiteral("无内参会话"), QStringLiteral("—"), false);
    return;
  }

  core::IntrinsicsPlotInput input;
  if (!build_intrinsics_plot_input(*session, &input)) {
    emit plot_finished(
        false, {}, QStringLiteral("无法读取会话数据"), QStringLiteral("—"), false);
    return;
  }
  input.owned_training_batch = session->intrinsics_state().training_batch();
  input.owned_evaluation_batch = session->intrinsics_state().evaluation_batch();
  input.has_owned_batches = true;
  input.collector = nullptr;

  if (kind == IntrinsicsPlotKind::CalibrationBars && !input.has_calibrated) {
    emit plot_finished(
        false, {}, QStringLiteral("尚未完成标定，无法生成柱状对比图。"),
        QStringLiteral("—"), false);
    return;
  }
  if (kind == IntrinsicsPlotKind::CalibrationRmsHeatmaps && !input.has_calibrated) {
    emit plot_finished(
        false, {}, QStringLiteral("尚未完成标定，无法生成 RMS 热力图。"),
        QStringLiteral("—"), false);
    return;
  }

  running_ = true;
  const int gen = generation_;
  emit plot_started();

  std::thread([this, gen, input = std::move(input), kind]() mutable {
    const IntrinsicsPlotJobResult result = run_plot_job(input, kind);
    QMetaObject::invokeMethod(
        this,
        [this, gen, result]() { on_worker_finished(gen, result); },
        Qt::QueuedConnection);
  }).detach();
}

void IntrinsicsAsyncPlotController::on_worker_finished(
    int generation, IntrinsicsPlotJobResult result) {
  if (generation != generation_) {
    return;
  }
  running_ = false;

  if (!result.ok) {
    emit plot_finished(
        false, {},
        QString::fromStdString(
            result.error.empty() ? "统计图生成失败" : result.error),
        QString::fromStdString(result.backend_label), true);
    return;
  }

  QImage image;
  if (!image.loadFromData(result.png_bytes, "PNG")) {
    emit plot_finished(
        false, {}, QStringLiteral("无法解码统计图 PNG"),
        QString::fromStdString(result.backend_label), true);
    return;
  }

  emit plot_finished(
      true, image, {}, QString::fromStdString(result.backend_label), false);
}

}  // namespace gui
}  // namespace hs_calib
