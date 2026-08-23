#include "hs_calib_suite/gui/plotting/intrinsics_calibration_rms_dialog.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

#include "hs_calib_suite/gui/intrinsics/intrinsics_dialog_ui.hpp"
#include "hs_calib_suite/gui/plotting/intrinsics_async_plot_controller.hpp"
#include "hs_calib_suite/gui/session/session_controller.hpp"
#include "hs_calib_suite/gui/widgets/image_view_widget.hpp"

namespace hs_calib {
namespace gui {

namespace {
constexpr auto kLoadingPlaceholder = "正在生成统计图，请稍候…";
constexpr auto kLoadingSummary =
    "正在后台处理（流水线求解 + matplotlib 绘图），请稍候…";
}  // namespace

IntrinsicsCalibrationRmsDialog::IntrinsicsCalibrationRmsDialog(QWidget *parent)
    : QDialog(parent) {
  auto chrome = setup_intrinsics_dialog(
      this,
      QStringLiteral("Calibration result statistics"),
      QStringLiteral("标定结果 RMS 均值/标准差热力图（像面 + 旋转坐标，Tier4）"),
      920);

  auto *backend_frame = new QFrame(chrome.content);
  backend_frame->setObjectName(QStringLiteral("Panel"));
  auto *backend_lay = new QHBoxLayout(backend_frame);
  backend_lay->setContentsMargins(14, 10, 14, 10);
  backend_lay->addWidget(make_field_label(QStringLiteral("统计后端"), backend_frame));
  lbl_backend_ = make_value_label(QStringLiteral("—"), backend_frame);
  backend_lay->addWidget(lbl_backend_, 1);
  chrome.content_layout->addWidget(backend_frame);

  auto *plot_frame = new QFrame(chrome.content);
  plot_frame->setObjectName(QStringLiteral("Panel"));
  auto *plot_lay = new QVBoxLayout(plot_frame);
  plot_lay->setContentsMargins(12, 12, 12, 12);
  auto *plot_hint = new QLabel(
      QStringLiteral("滚轮缩放 · 拖拽平移 · 双击适应窗口"), plot_frame);
  plot_hint->setObjectName(QStringLiteral("Muted"));
  plot_view_ = new ImageViewWidget(plot_frame);
  plot_view_->setMinimumHeight(360);
  plot_view_->set_background_color(QColor(255, 255, 255));
  plot_view_->set_toolbar_style(ImageViewToolbarStyle::OverlayZoomSave);
  plot_lay->addWidget(plot_hint);
  plot_lay->addWidget(plot_view_, 1);
  chrome.content_layout->addWidget(plot_frame, 1);

  summary_label_ = new QLabel(chrome.content);
  summary_label_->setObjectName(QStringLiteral("Muted"));
  summary_label_->setWordWrap(true);
  chrome.content_layout->addWidget(summary_label_);

  auto *root = qobject_cast<QVBoxLayout *>(layout());
  add_dialog_footer(root, QStringLiteral("关闭"), [this]() { close(); });

  plot_loader_ = new IntrinsicsAsyncPlotController(this);
  connect(
      plot_loader_, &IntrinsicsAsyncPlotController::plot_started, this,
      [this]() {
        lbl_backend_->setText(QStringLiteral("处理中…"));
        summary_label_->setText(QString::fromUtf8(kLoadingSummary));
        plot_view_->clear_image();
        plot_view_->set_placeholder(QString::fromUtf8(kLoadingPlaceholder));
      });
  connect(
      plot_loader_, &IntrinsicsAsyncPlotController::plot_finished, this,
      [this](
          bool ok, const QImage &image, const QString &summary,
          const QString &backend_label, bool /*matplotlib_failed*/) {
        if (ok) {
          summary_label_->clear();
          lbl_backend_->setText(backend_label);
          plot_view_->set_image(image);
          plot_view_->fit_to_window();
          return;
        }
        lbl_backend_->setText(QStringLiteral("—"));
        summary_label_->setText(summary);
        plot_view_->clear_image();
        plot_view_->set_placeholder(
            summary.isEmpty() ? QString::fromUtf8(kLoadingPlaceholder) : summary);
      });
}

void IntrinsicsCalibrationRmsDialog::refresh(
    const SessionController *session, const std::string &backend) {
  Q_UNUSED(backend);
  summary_label_->clear();
  if (session == nullptr || !session->is_intrinsics()) {
    lbl_backend_->setText(QStringLiteral("—"));
    plot_view_->clear_image();
    plot_view_->set_placeholder(QStringLiteral("暂无统计图"));
    summary_label_->setText(QStringLiteral("无内参会话"));
    return;
  }
  plot_loader_->start(
      session, IntrinsicsPlotKind::CalibrationRmsHeatmaps, backend);
}

}  // namespace gui
}  // namespace hs_calib
