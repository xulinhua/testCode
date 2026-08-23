#pragma once

#include <QByteArray>
#include <QImage>
#include <QObject>

#include <string>

#include "hs_calib_suite/gui/plotting/intrinsics_plot_renderer.hpp"

namespace hs_calib {
namespace gui {

class SessionController;

struct IntrinsicsPlotJobResult {
  bool ok = false;
  std::string error;
  QByteArray png_bytes;
  std::string backend_label = "matplotlib";
};

/// \brief 异步生成 Tier4 统计图（流水线 + matplotlib），不阻塞 UI
class IntrinsicsAsyncPlotController : public QObject {
  Q_OBJECT

public:
  explicit IntrinsicsAsyncPlotController(QObject *parent = nullptr);
  ~IntrinsicsAsyncPlotController() override;

  void start(
      const SessionController *session,
      IntrinsicsPlotKind kind,
      const std::string &backend);
  void cancel();
  bool is_running() const { return running_; }

signals:
  void plot_started();
  void plot_finished(
      bool ok,
      const QImage &image,
      const QString &summary,
      const QString &backend_label,
      bool matplotlib_failed);

private:
  void on_worker_finished(int generation, IntrinsicsPlotJobResult result);

  int generation_ = 0;
  bool running_ = false;
  std::string pending_backend_;
};

}  // namespace gui
}  // namespace hs_calib
