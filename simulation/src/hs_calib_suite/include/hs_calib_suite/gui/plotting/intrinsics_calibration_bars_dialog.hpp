#pragma once

#include <QDialog>

class QLabel;

namespace hs_calib {
namespace gui {

class ImageViewWidget;
class SessionController;
class IntrinsicsAsyncPlotController;

/// \brief Tier4「Calibration result statistics vs single-shot calibration」
class IntrinsicsCalibrationBarsDialog : public QDialog {
  Q_OBJECT

public:
  explicit IntrinsicsCalibrationBarsDialog(QWidget *parent = nullptr);
  void refresh(const SessionController *session, const std::string &backend);

private:
  QLabel *lbl_backend_ = nullptr;
  QLabel *summary_label_ = nullptr;
  ImageViewWidget *plot_view_ = nullptr;
  IntrinsicsAsyncPlotController *plot_loader_ = nullptr;
};

}  // namespace gui
}  // namespace hs_calib
