#include "hs_calib_suite/gui/widgets/review_charts_widget.hpp"

#include <algorithm>
#include <cmath>

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QSizePolicy>

namespace hs_calib {
namespace gui {

ResidualBarWidget::ResidualBarWidget(QWidget *parent) : QWidget(parent) {
  setObjectName(QStringLiteral("ResidualBarWidget"));
  setMinimumHeight(160);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void ResidualBarWidget::set_bars(const std::vector<Bar> &bars) {
  bars_ = bars;
  max_rms_ = 0.5;
  for (const auto &b : bars_) {
    if (b.ok) {
      max_rms_ = std::max(max_rms_, b.rms_px);
    }
  }
  max_rms_ = std::max(max_rms_ * 1.15, 0.5);
  update();
}

void ResidualBarWidget::set_highlight_view(int view_index) {
  if (highlight_view_ == view_index) {
    return;
  }
  highlight_view_ = view_index;
  update();
}

void ResidualBarWidget::clear() {
  bars_.clear();
  highlight_view_ = -1;
  update();
}

QSize ResidualBarWidget::sizeHint() const { return {420, 200}; }
QSize ResidualBarWidget::minimumSizeHint() const { return {200, 140}; }

void ResidualBarWidget::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  const QRectF rc = QRectF(rect()).adjusted(8, 8, -8, -8);
  p.fillRect(rect(), QColor(12, 18, 28));
  p.setPen(QPen(QColor(50, 62, 80), 1));
  p.drawRoundedRect(rc, 6, 6);

  if (bars_.empty()) {
    p.setPen(QColor(140, 150, 168));
    p.drawText(rc, Qt::AlignCenter, QStringLiteral("求解后显示逐帧残差"));
    return;
  }

  const double left = rc.left() + 36;
  const double right = rc.right() - 10;
  const double top = rc.top() + 18;
  const double bottom = rc.bottom() - 28;
  const double plot_w = std::max(10.0, right - left);
  const double plot_h = std::max(10.0, bottom - top);

  // 网格
  p.setPen(QPen(QColor(40, 50, 66), 1, Qt::DotLine));
  for (int i = 0; i <= 4; ++i) {
    const double y = top + plot_h * (i / 4.0);
    p.drawLine(QPointF(left, y), QPointF(right, y));
    p.setPen(QColor(120, 130, 148));
    const double v = max_rms_ * (1.0 - i / 4.0);
    p.drawText(
        QRectF(rc.left(), y - 8, 34, 16), Qt::AlignRight | Qt::AlignVCenter,
        QString::number(v, 'f', 2));
    p.setPen(QPen(QColor(40, 50, 66), 1, Qt::DotLine));
  }

  const int n = static_cast<int>(bars_.size());
  const double gap = 4.0;
  const double bar_w = std::max(4.0, (plot_w - gap * (n + 1)) / n);

  for (int i = 0; i < n; ++i) {
    const auto &b = bars_[static_cast<size_t>(i)];
    const double x = left + gap + i * (bar_w + gap);
    const double h =
        b.ok ? (plot_h * std::min(1.0, b.rms_px / max_rms_)) : (plot_h * 0.08);
    const double y = bottom - h;
    const bool hi = (highlight_view_ >= 0 && b.view_index == highlight_view_);
    QColor fill = b.ok ? QColor(70, 180, 210) : QColor(90, 90, 100);
    if (b.ok && b.rms_px > max_rms_ * 0.7) {
      fill = QColor(230, 140, 70);
    }
    if (hi) {
      fill = QColor(92, 225, 255);
    }
    p.setPen(Qt::NoPen);
    p.setBrush(fill);
    p.drawRoundedRect(QRectF(x, y, bar_w, h), 2, 2);

    if (n <= 24 || hi) {
      p.setPen(QColor(150, 160, 178));
      p.save();
      p.translate(x + bar_w * 0.5, bottom + 4);
      p.rotate(-40);
      const QString lab =
          b.label.size() > 10 ? b.label.left(9) + QStringLiteral("…") : b.label;
      p.drawText(0, 0, lab);
      p.restore();
    }
  }

  p.setPen(QColor(180, 190, 205));
  p.drawText(
      QRectF(left, rc.top(), plot_w, 16), Qt::AlignLeft | Qt::AlignVCenter,
      QStringLiteral("逐帧重投影 RMS (px)"));
}

void ResidualBarWidget::mousePressEvent(QMouseEvent *event) {
  if (bars_.empty()) {
    return;
  }
  const QRectF rc = QRectF(rect()).adjusted(8, 8, -8, -8);
  const double left = rc.left() + 36;
  const double right = rc.right() - 10;
  const double top = rc.top() + 18;
  const double bottom = rc.bottom() - 28;
  const double plot_w = std::max(10.0, right - left);
  const int n = static_cast<int>(bars_.size());
  const double gap = 4.0;
  const double bar_w = std::max(4.0, (plot_w - gap * (n + 1)) / n);
  const double x = event->pos().x();
  const double y = event->pos().y();
  if (y < top || y > bottom) {
    return;
  }
  for (int i = 0; i < n; ++i) {
    const double bx = left + gap + i * (bar_w + gap);
    if (x >= bx && x <= bx + bar_w) {
      emit bar_clicked(bars_[static_cast<size_t>(i)].view_index);
      return;
    }
  }
}

CoverageMapWidget::CoverageMapWidget(QWidget *parent) : QWidget(parent) {
  setObjectName(QStringLiteral("CoverageMapWidget"));
  setMinimumHeight(160);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void CoverageMapWidget::set_image_size(int width, int height) {
  image_w_ = std::max(0, width);
  image_h_ = std::max(0, height);
  update();
}

void CoverageMapWidget::set_points(const std::vector<Point> &points) {
  points_ = points;
  max_err_ = 1.0f;
  for (const auto &pt : points_) {
    max_err_ = std::max(max_err_, pt.err_px);
  }
  max_err_ = std::max(max_err_, 1.0f);
  update();
}

void CoverageMapWidget::set_filter_view(int view_index) {
  if (filter_view_ == view_index) {
    return;
  }
  filter_view_ = view_index;
  update();
}

void CoverageMapWidget::clear() {
  points_.clear();
  filter_view_ = -1;
  update();
}

QSize CoverageMapWidget::sizeHint() const { return {420, 200}; }
QSize CoverageMapWidget::minimumSizeHint() const { return {200, 140}; }

QColor CoverageMapWidget::color_for_error(float err_px, float max_err) {
  const float t = std::min(1.0f, std::max(0.0f, err_px / std::max(max_err, 1e-3f)));
  // 青 → 黄 → 橙
  const int r = static_cast<int>(40 + 190 * t);
  const int g = static_cast<int>(200 - 80 * t);
  const int b = static_cast<int>(210 - 160 * t);
  return QColor(r, g, b, 210);
}

void CoverageMapWidget::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  const QRectF rc = QRectF(rect()).adjusted(8, 8, -8, -8);
  p.fillRect(rect(), QColor(12, 18, 28));
  p.setPen(QPen(QColor(50, 62, 80), 1));
  p.drawRoundedRect(rc, 6, 6);

  p.setPen(QColor(180, 190, 205));
  p.drawText(
      QRectF(rc.left() + 10, rc.top() + 6, rc.width() - 20, 16),
      Qt::AlignLeft | Qt::AlignVCenter,
      QStringLiteral("覆盖 / 重投影残差"));

  const QRectF plot = rc.adjusted(12, 28, -12, -28);
  if (image_w_ <= 0 || image_h_ <= 0) {
    p.setPen(QColor(140, 150, 168));
    p.drawText(plot, Qt::AlignCenter, QStringLiteral("求解后显示像面覆盖"));
    return;
  }

  const double sx = plot.width() / static_cast<double>(image_w_);
  const double sy = plot.height() / static_cast<double>(image_h_);
  const double s = std::min(sx, sy);
  const double dw = image_w_ * s;
  const double dh = image_h_ * s;
  const QRectF frame(
      plot.left() + (plot.width() - dw) * 0.5,
      plot.top() + (plot.height() - dh) * 0.5, dw, dh);

  p.fillRect(frame, QColor(18, 24, 36));
  p.setPen(QPen(QColor(60, 75, 95), 1));
  p.drawRect(frame);
  // 九宫辅助线
  p.setPen(QPen(QColor(35, 45, 60), 1, Qt::DotLine));
  for (int i = 1; i < 3; ++i) {
    const double x = frame.left() + frame.width() * (i / 3.0);
    const double y = frame.top() + frame.height() * (i / 3.0);
    p.drawLine(QPointF(x, frame.top()), QPointF(x, frame.bottom()));
    p.drawLine(QPointF(frame.left(), y), QPointF(frame.right(), y));
  }

  int drawn = 0;
  for (const auto &pt : points_) {
    if (filter_view_ >= 0 && pt.view_index != filter_view_) {
      continue;
    }
    const double x = frame.left() + pt.u * s;
    const double y = frame.top() + pt.v * s;
    if (!frame.contains(QPointF(x, y))) {
      continue;
    }
    p.setPen(Qt::NoPen);
    p.setBrush(color_for_error(pt.err_px, max_err_));
    const double r = filter_view_ >= 0 ? 3.2 : 2.2;
    p.drawEllipse(QPointF(x, y), r, r);
    ++drawn;
  }

  p.setPen(QColor(120, 130, 148));
  QString foot = QStringLiteral("%1×%2 · %3 pts")
                     .arg(image_w_)
                     .arg(image_h_)
                     .arg(drawn);
  if (filter_view_ >= 0) {
    foot += QStringLiteral(" · view#%1").arg(filter_view_);
  }
  foot += QStringLiteral(" · 色=残差");
  p.drawText(
      QRectF(rc.left() + 10, rc.bottom() - 22, rc.width() - 20, 16),
      Qt::AlignLeft | Qt::AlignVCenter, foot);

  if (drawn == 0 && !points_.empty()) {
    p.setPen(QColor(140, 150, 168));
    p.drawText(frame, Qt::AlignCenter, QStringLiteral("当前筛选无点"));
  } else if (points_.empty()) {
    p.setPen(QColor(140, 150, 168));
    p.drawText(frame, Qt::AlignCenter, QStringLiteral("无残差点"));
  }
}

}  // namespace gui
}  // namespace hs_calib
