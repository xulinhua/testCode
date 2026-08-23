#include "hs_calib_suite/gui/intrinsics/intrinsics_matplotlib_plotter.hpp"

#include <fstream>
#include <sstream>

#include <algorithm>
#include <cmath>

#include <QColor>
#include <QPainter>
#include <QProcess>
#include <QTemporaryFile>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_data_collector.hpp"

namespace hs_calib {
namespace gui {
namespace {

void append_sample_json(
    std::ostringstream &oss, const core::BoardFrameFingerprint &fp) {
  oss << "{"
      << "\"cx\":" << fp.centroid_x << ","
      << "\"cy\":" << fp.centroid_y << ","
      << "\"tilt\":" << fp.tilt_deg << ","
      << "\"skew\":" << fp.normalized_skew << ","
      << "\"size\":" << fp.normalized_size << ","
      << "\"angle_x\":" << fp.rough_angle_x_deg << ","
      << "\"angle_y\":" << fp.rough_angle_y_deg
      << "}";
}

std::string script_path() {
  try {
    const std::string share =
        ament_index_cpp::get_package_share_directory("hs_calib_suite");
    return share + "/scripts/intrinsics_stats_plot.py";
  } catch (...) {
  }
  return "scripts/intrinsics_stats_plot.py";
}

QPixmap draw_qt_chart(const core::IntrinsicsDataCollector &collector) {
  const int w = 640;
  const int h = 420;
  QPixmap pm(w, h);
  pm.fill(QColor(18, 24, 36));
  QPainter p(&pm);
  p.setRenderHint(QPainter::Antialiasing, true);

  const auto &params = collector.params();
  const int cells = std::max(4, params.heatmap_cells);
  const int pad = 48;
  const int gw = (w - pad * 3) / 2;
  const int gh = (h - pad * 3) / 2;

  auto draw_grid = [&](int ox, int oy, const std::vector<core::CollectedIntrinsicsSample> &samples,
                       const QString &title) {
    p.setPen(QColor(200, 210, 220));
    p.drawText(ox, oy - 8, title);
    p.fillRect(ox, oy, gw, gh, QColor(12, 16, 24));
    std::vector<int> grid(static_cast<size_t>(cells * cells), 0);
    for (const auto &s : samples) {
      const int ix = std::min(cells - 1, std::max(0, static_cast<int>(s.fingerprint.centroid_x * cells)));
      const int iy = std::min(cells - 1, std::max(0, static_cast<int>(s.fingerprint.centroid_y * cells)));
      grid[static_cast<size_t>(iy * cells + ix)]++;
    }
    int mx = 1;
    for (int v : grid) {
      mx = std::max(mx, v);
    }
    const double cw = static_cast<double>(gw) / cells;
    const double ch = static_cast<double>(gh) / cells;
    for (int iy = 0; iy < cells; ++iy) {
      for (int ix = 0; ix < cells; ++ix) {
        const int v = grid[static_cast<size_t>(iy * cells + ix)];
        const int intensity = 40 + (v * 180) / mx;
        p.fillRect(
            QRectF(ox + ix * cw, oy + iy * ch, cw - 1, ch - 1),
            QColor(255, 140, 40, intensity));
      }
    }
    p.setPen(QColor(80, 90, 100));
    p.drawRect(ox, oy, gw, gh);
  };

  draw_grid(pad, pad, collector.training(), QStringLiteral("训练占用"));
  draw_grid(pad * 2 + gw, pad, collector.evaluation(), QStringLiteral("评估占用"));

  const int hx = pad;
  const int hy = pad * 2 + gh;
  p.setPen(QColor(200, 210, 220));
  p.drawText(hx, hy - 8, QStringLiteral("倾角分布"));
  p.fillRect(hx, hy, gw, gh, QColor(12, 16, 24));
  p.setPen(QPen(QColor(70, 82, 100), 1));
  p.drawRect(hx, hy, gw, gh);
  const int bins = 12;
  std::vector<int> hist(static_cast<size_t>(bins), 0);
  for (const auto &s : collector.training()) {
    const int b = std::min(
        bins - 1,
        static_cast<int>(s.fingerprint.tilt_deg / params.max_allowed_tilt_deg * bins));
    hist[static_cast<size_t>(b)]++;
  }
  int hmax = 1;
  for (int v : hist) {
    hmax = std::max(hmax, v);
  }
  const double bw = static_cast<double>(gw) / bins;
  p.setBrush(QColor(90, 170, 255));
  p.setPen(QPen(QColor(20, 40, 70), 1));
  for (int i = 0; i < bins; ++i) {
    const double bh = (hist[static_cast<size_t>(i)] * (gh - 4)) / hmax;
    p.drawRect(
        QRectF(hx + i * bw + 1, hy + gh - bh, bw - 2, bh));
  }

  const int sx = pad * 2 + gw;
  const int sy = hy;
  p.setPen(QColor(200, 210, 220));
  p.drawText(sx, sy - 8, QStringLiteral("质心散点"));
  p.fillRect(sx, sy, gw, gh, QColor(12, 16, 24));
  p.setPen(QPen(QColor(70, 82, 100), 1));
  p.drawRect(sx, sy, gw, gh);
  p.setPen(QPen(QColor(45, 55, 70), 1, Qt::DotLine));
  for (int i = 1; i < 4; ++i) {
    const qreal gx = sx + gw * (i / 4.0);
    const qreal gy = sy + gh * (i / 4.0);
    p.drawLine(QPointF(gx, sy), QPointF(gx, sy + gh));
    p.drawLine(QPointF(sx, gy), QPointF(sx + gw, gy));
  }
  const auto draw_centroid = [&](float cx, float cy, const QColor &fill) {
    const QPointF pt(sx + cx * gw, sy + cy * gh);
    p.setPen(QPen(QColor(8, 12, 18), 2));
    p.setBrush(fill);
    p.drawEllipse(pt, 5.0, 5.0);
  };
  for (const auto &s : collector.training()) {
    draw_centroid(
        static_cast<float>(s.fingerprint.centroid_x),
        static_cast<float>(s.fingerprint.centroid_y),
        QColor(90, 255, 150));
  }
  for (const auto &s : collector.evaluation()) {
    draw_centroid(
        static_cast<float>(s.fingerprint.centroid_x),
        static_cast<float>(s.fingerprint.centroid_y),
        QColor(255, 170, 70));
  }
  return pm;
}

}  // namespace

bool IntrinsicsStatsPlotter::export_collector_json(
    const core::IntrinsicsDataCollector &collector, const std::string &path) {
  const auto &params = collector.params();
  std::ostringstream oss;
  oss << "{";
  oss << "\"heatmap_cells\":" << params.heatmap_cells << ",";
  oss << "\"rotation_heatmap_angle_res\":" << params.rotation_heatmap_angle_res << ",";
  oss << "\"max_tilt_deg\":" << params.max_allowed_tilt_deg << ",";
  oss << "\"training\":[";
  for (size_t i = 0; i < collector.training().size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    append_sample_json(oss, collector.training()[i].fingerprint);
  }
  oss << "],\"evaluation\":[";
  for (size_t i = 0; i < collector.evaluation().size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    append_sample_json(oss, collector.evaluation()[i].fingerprint);
  }
  oss << "]}";
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << oss.str();
  return true;
}

QPixmap IntrinsicsStatsPlotter::render_qt_summary(
    const core::IntrinsicsDataCollector &collector) {
  return draw_qt_chart(collector);
}

QPixmap IntrinsicsStatsPlotter::render_matplotlib(
    const core::IntrinsicsDataCollector &collector, std::string *error_out) {
  QTemporaryFile json_file(QStringLiteral("hs_calib_stats_XXXXXX.json"));
  QTemporaryFile png_file(QStringLiteral("hs_calib_stats_XXXXXX.png"));
  json_file.setAutoRemove(true);
  png_file.setAutoRemove(false);
  if (!json_file.open() || !png_file.open()) {
    if (error_out != nullptr) {
      *error_out = "无法创建临时文件";
    }
    return {};
  }
  if (!export_collector_json(collector, json_file.fileName().toStdString())) {
    if (error_out != nullptr) {
      *error_out = "导出 JSON 失败";
    }
    return {};
  }
  json_file.close();
  png_file.close();

  QProcess proc;
  proc.start(
      QStringLiteral("python3"),
      {QString::fromStdString(script_path()),
       json_file.fileName(),
       png_file.fileName()});
  if (!proc.waitForFinished(30000)) {
    if (error_out != nullptr) {
      *error_out = "matplotlib 脚本超时或未安装 python3";
    }
    return {};
  }
  if (proc.exitCode() != 0) {
    if (error_out != nullptr) {
      *error_out = proc.readAllStandardError().toStdString();
      if (error_out->empty()) {
        *error_out = "matplotlib 脚本失败（需 pip install matplotlib numpy）";
      }
    }
    return {};
  }
  QPixmap pm;
  if (!pm.load(png_file.fileName())) {
    if (error_out != nullptr) {
      *error_out = "无法加载生成的 PNG";
    }
    return {};
  }
  return pm;
}

}  // namespace gui
}  // namespace hs_calib
