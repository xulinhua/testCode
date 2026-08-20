#include "hs_calib_suite/gui/widgets/review_charts_widget.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include <QDialog>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPushButton>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <QWheelEvent>

namespace hs_calib {
namespace gui {

namespace {

constexpr double kMinZoom = 1.0;
constexpr double kMaxZoom = 8.0;
/// 缩略图缓存超采样：滚轮放大时仍用清晰原图像素，而不是放大矢量布局
constexpr qreal kBaseOverSample = 2.0;

double clamp_zoom(double z) {
  return std::max(kMinZoom, std::min(kMaxZoom, z));
}

QPixmap render_chart_base(
    int logical_w, int logical_h, qreal dpr,
    const std::function<void(QPainter &, const QRectF &)> &paint_fn) {
  if (logical_w <= 0 || logical_h <= 0) {
    return {};
  }
  const qreal pixel_ratio = std::max<qreal>(1.0, dpr) * kBaseOverSample;
  QPixmap pm(
      std::max(1, static_cast<int>(std::ceil(logical_w * pixel_ratio))),
      std::max(1, static_cast<int>(std::ceil(logical_h * pixel_ratio))));
  pm.setDevicePixelRatio(pixel_ratio);
  pm.fill(QColor(12, 18, 28));
  QPainter p(&pm);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setRenderHint(QPainter::TextAntialiasing, true);
  paint_fn(p, QRectF(0, 0, logical_w, logical_h).adjusted(8, 8, -8, -8));
  return pm;
}

}  // namespace

ResidualBarWidget::ResidualBarWidget(QWidget *parent) : QWidget(parent) {
  setObjectName(QStringLiteral("ResidualBarWidget"));
  setMinimumHeight(160);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMouseTracking(true);
  setCursor(Qt::OpenHandCursor);
  setToolTip(QStringLiteral("滚轮缩放 · 拖拽平移 · 双击放大窗口 · 单击柱选择观测"));
}

void ResidualBarWidget::invalidate_base() {
  base_dirty_ = true;
  update();
}

void ResidualBarWidget::ensure_base_pixmap() {
  if (!base_dirty_ && !base_pixmap_.isNull() &&
      qFuzzyCompare(base_pixmap_.width() / base_pixmap_.devicePixelRatio(), static_cast<qreal>(width())) &&
      qFuzzyCompare(base_pixmap_.height() / base_pixmap_.devicePixelRatio(), static_cast<qreal>(height()))) {
    return;
  }
  base_pixmap_ = render_chart_base(
      width(), height(), devicePixelRatioF(),
      [this](QPainter &p, const QRectF &rc) { paint_chart(p, rc); });
  base_dirty_ = false;
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
  invalidate_base();
}

void ResidualBarWidget::set_highlight_view(int view_index) {
  if (highlight_view_ == view_index) {
    return;
  }
  highlight_view_ = view_index;
  invalidate_base();
}

void ResidualBarWidget::clear() {
  bars_.clear();
  highlight_view_ = -1;
  reset_view();
  invalidate_base();
}

void ResidualBarWidget::reset_view() {
  zoom_ = 1.0;
  pan_ = QPointF(0.0, 0.0);
  update();
}

bool ResidualBarWidget::export_png(const QString &path, const QSize &logical_size) const {
  if (path.isEmpty() || bars_.empty()) {
    return false;
  }
  const int w = std::max(640, logical_size.width() > 0 ? logical_size.width() : 1280);
  const int h = std::max(320, logical_size.height() > 0 ? logical_size.height() : 480);
  const QPixmap pm = render_chart_base(
      w, h, 1.0, [this](QPainter &p, const QRectF &rc) { paint_chart(p, rc); });
  return !pm.isNull() && pm.save(path, "PNG");
}

QSize ResidualBarWidget::sizeHint() const { return {420, 200}; }
QSize ResidualBarWidget::minimumSizeHint() const { return {200, 140}; }

QPointF ResidualBarWidget::map_to_content(const QPointF &widget_pos) const {
  return (widget_pos - pan_) / zoom_;
}

void ResidualBarWidget::paint_chart(QPainter &p, const QRectF &rc) const {
  p.setPen(QPen(QColor(50, 62, 80), 1));
  p.setBrush(Qt::NoBrush);
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

    // 标签画进原图；缩放只看位图，避免 zoom 后突然叠一堆字
    if (n <= 24 || hi) {
      p.setPen(QColor(150, 160, 178));
      p.save();
      p.translate(x + bar_w * 0.5, bottom + 4);
      p.rotate(-40);
      const QString lab =
          b.label.size() > 14 ? b.label.left(13) + QStringLiteral("…") : b.label;
      p.drawText(0, 0, lab);
      p.restore();
    }
  }

  p.setPen(QColor(180, 190, 205));
  p.drawText(
      QRectF(left, rc.top(), plot_w, 16), Qt::AlignLeft | Qt::AlignVCenter,
      QStringLiteral("逐帧重投影 RMS (px)"));
}

void ResidualBarWidget::paintEvent(QPaintEvent *) {
  ensure_base_pixmap();
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setRenderHint(QPainter::SmoothPixmapTransform, true);
  p.fillRect(rect(), QColor(12, 18, 28));

  if (!base_pixmap_.isNull()) {
    p.save();
    p.translate(pan_);
    p.scale(zoom_, zoom_);
    p.drawPixmap(QRect(0, 0, width(), height()), base_pixmap_);
    p.restore();
  }

  if (!enlarge_host_) {
    p.setPen(QColor(100, 110, 128));
    p.drawText(
        QRectF(8, height() - 18, width() - 16, 14),
        Qt::AlignRight | Qt::AlignVCenter,
        QStringLiteral("滚轮缩放 · 双击放大  %1×").arg(zoom_, 0, 'f', 1));
  }
}

void ResidualBarWidget::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
  invalidate_base();
}

void ResidualBarWidget::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::MiddleButton ||
      (event->button() == Qt::LeftButton &&
       (event->modifiers() & Qt::ShiftModifier))) {
    panning_ = true;
    last_mouse_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
    return;
  }
  if (event->button() != Qt::LeftButton || bars_.empty()) {
    return;
  }

  last_mouse_ = event->pos();
  panning_ = false;

  const QPointF c = map_to_content(event->pos());
  const QRectF rc = QRectF(rect()).adjusted(8, 8, -8, -8);
  const double left = rc.left() + 36;
  const double right = rc.right() - 10;
  const double top = rc.top() + 18;
  const double bottom = rc.bottom() - 28;
  const double plot_w = std::max(10.0, right - left);
  const int n = static_cast<int>(bars_.size());
  const double gap = 4.0;
  const double bar_w = std::max(4.0, (plot_w - gap * (n + 1)) / n);
  if (c.y() < top || c.y() > bottom) {
    panning_ = true;
    setCursor(Qt::ClosedHandCursor);
    return;
  }
  for (int i = 0; i < n; ++i) {
    const double bx = left + gap + i * (bar_w + gap);
    if (c.x() >= bx && c.x() <= bx + bar_w) {
      emit bar_clicked(bars_[static_cast<size_t>(i)].view_index);
      return;
    }
  }
  panning_ = true;
  setCursor(Qt::ClosedHandCursor);
}

void ResidualBarWidget::mouseMoveEvent(QMouseEvent *event) {
  if (!panning_) {
    if (event->buttons() & Qt::LeftButton) {
      const QPoint d = event->pos() - last_mouse_;
      if (std::abs(d.x()) + std::abs(d.y()) > 4) {
        panning_ = true;
        setCursor(Qt::ClosedHandCursor);
      }
    }
  }
  if (!panning_) {
    return;
  }
  const QPoint d = event->pos() - last_mouse_;
  last_mouse_ = event->pos();
  pan_ += QPointF(d);
  update();
}

void ResidualBarWidget::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton) {
    panning_ = false;
    setCursor(Qt::OpenHandCursor);
  }
}

void ResidualBarWidget::mouseDoubleClickEvent(QMouseEvent *event) {
  if (event->button() != Qt::LeftButton) {
    return;
  }
  if (enlarge_host_) {
    reset_view();
    return;
  }
  if (event->modifiers() & Qt::ControlModifier) {
    reset_view();
    return;
  }
  open_enlarged();
}

void ResidualBarWidget::wheelEvent(QWheelEvent *event) {
  const QPointF cursor = event->position();
  const double old_z = zoom_;
  const double factor = event->angleDelta().y() > 0 ? 1.15 : (1.0 / 1.15);
  const double new_z = clamp_zoom(old_z * factor);
  if (std::abs(new_z - old_z) < 1e-6) {
    return;
  }
  const QPointF world = (cursor - pan_) / old_z;
  zoom_ = new_z;
  pan_ = cursor - world * zoom_;
  update();
  event->accept();
}

void ResidualBarWidget::open_enlarged() {
  auto *dlg = new QDialog(window());
  dlg->setAttribute(Qt::WA_DeleteOnClose);
  dlg->setWindowTitle(QStringLiteral("残差图 · 放大观察"));
  dlg->resize(960, 560);
  auto *lay = new QVBoxLayout(dlg);
  lay->setContentsMargins(10, 10, 10, 10);
  lay->setSpacing(8);
  auto *hint = new QLabel(
      QStringLiteral("滚轮缩放 · 拖拽平移 · Ctrl+双击复位 · 单击柱选择观测"), dlg);
  hint->setObjectName(QStringLiteral("Muted"));
  lay->addWidget(hint);
  auto *chart = new ResidualBarWidget(dlg);
  chart->enlarge_host_ = true;
  chart->set_bars(bars_);
  chart->set_highlight_view(highlight_view_);
  chart->zoom_ = std::max(1.5, zoom_);
  chart->pan_ = pan_;
  connect(chart, &ResidualBarWidget::bar_clicked, this, &ResidualBarWidget::bar_clicked);
  lay->addWidget(chart, 1);
  auto *btns = new QDialogButtonBox(QDialogButtonBox::Close, dlg);
  connect(btns, &QDialogButtonBox::rejected, dlg, &QDialog::reject);
  connect(btns, &QDialogButtonBox::accepted, dlg, &QDialog::accept);
  connect(btns->button(QDialogButtonBox::Close), &QPushButton::clicked, dlg, &QDialog::accept);
  lay->addWidget(btns);
  dlg->show();
}

// ---------------------------------------------------------------------------

CoverageMapWidget::CoverageMapWidget(QWidget *parent) : QWidget(parent) {
  setObjectName(QStringLiteral("CoverageMapWidget"));
  setMinimumHeight(160);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMouseTracking(true);
  setCursor(Qt::OpenHandCursor);
  setToolTip(QStringLiteral("滚轮缩放 · 拖拽平移 · 双击放大窗口"));
}

void CoverageMapWidget::invalidate_base() {
  base_dirty_ = true;
  update();
}

void CoverageMapWidget::ensure_base_pixmap() {
  if (!base_dirty_ && !base_pixmap_.isNull() &&
      qFuzzyCompare(base_pixmap_.width() / base_pixmap_.devicePixelRatio(), static_cast<qreal>(width())) &&
      qFuzzyCompare(base_pixmap_.height() / base_pixmap_.devicePixelRatio(), static_cast<qreal>(height()))) {
    return;
  }
  base_pixmap_ = render_chart_base(
      width(), height(), devicePixelRatioF(),
      [this](QPainter &p, const QRectF &rc) { paint_chart(p, rc); });
  base_dirty_ = false;
}

void CoverageMapWidget::set_image_size(int width, int height) {
  image_w_ = std::max(0, width);
  image_h_ = std::max(0, height);
  invalidate_base();
}

void CoverageMapWidget::set_points(const std::vector<Point> &points) {
  points_ = points;
  max_err_ = 1.0f;
  for (const auto &pt : points_) {
    max_err_ = std::max(max_err_, pt.err_px);
  }
  max_err_ = std::max(max_err_, 1.0f);
  invalidate_base();
}

void CoverageMapWidget::set_filter_view(int view_index) {
  if (filter_view_ == view_index) {
    return;
  }
  filter_view_ = view_index;
  invalidate_base();
}

void CoverageMapWidget::clear() {
  points_.clear();
  filter_view_ = -1;
  reset_view();
  invalidate_base();
}

void CoverageMapWidget::reset_view() {
  zoom_ = 1.0;
  pan_ = QPointF(0.0, 0.0);
  update();
}

bool CoverageMapWidget::export_png(const QString &path, const QSize &logical_size) const {
  if (path.isEmpty() || image_w_ <= 0 || image_h_ <= 0 || points_.empty()) {
    return false;
  }
  const int w = std::max(640, logical_size.width() > 0 ? logical_size.width() : 1280);
  const int h = std::max(480, logical_size.height() > 0 ? logical_size.height() : 960);
  const QPixmap pm = render_chart_base(
      w, h, 1.0, [this](QPainter &p, const QRectF &rc) { paint_chart(p, rc); });
  return !pm.isNull() && pm.save(path, "PNG");
}

QSize CoverageMapWidget::sizeHint() const { return {420, 200}; }
QSize CoverageMapWidget::minimumSizeHint() const { return {200, 140}; }

QColor CoverageMapWidget::color_for_error(float err_px, float max_err) {
  const float t = std::min(1.0f, std::max(0.0f, err_px / std::max(max_err, 1e-3f)));
  const int r = static_cast<int>(40 + 190 * t);
  const int g = static_cast<int>(200 - 80 * t);
  const int b = static_cast<int>(210 - 160 * t);
  return QColor(r, g, b, 210);
}

QPointF CoverageMapWidget::map_to_content(const QPointF &widget_pos) const {
  return (widget_pos - pan_) / zoom_;
}

void CoverageMapWidget::paint_chart(QPainter &p, const QRectF &rc) const {
  p.setPen(QPen(QColor(50, 62, 80), 1));
  p.setBrush(Qt::NoBrush);
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
  p.setPen(QPen(QColor(35, 45, 60), 1, Qt::DotLine));
  for (int i = 1; i < 3; ++i) {
    const double x = frame.left() + frame.width() * (i / 3.0);
    const double y = frame.top() + frame.height() * (i / 3.0);
    p.drawLine(QPointF(x, frame.top()), QPointF(x, frame.bottom()));
    p.drawLine(QPointF(frame.left(), y), QPointF(frame.right(), y));
  }

  int drawn = 0;
  const double r = (filter_view_ >= 0 ? 3.2 : 2.2);
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

void CoverageMapWidget::paintEvent(QPaintEvent *) {
  ensure_base_pixmap();
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setRenderHint(QPainter::SmoothPixmapTransform, true);
  p.fillRect(rect(), QColor(12, 18, 28));

  if (!base_pixmap_.isNull()) {
    p.save();
    p.translate(pan_);
    p.scale(zoom_, zoom_);
    p.drawPixmap(QRect(0, 0, width(), height()), base_pixmap_);
    p.restore();
  }

  if (!enlarge_host_) {
    p.setPen(QColor(100, 110, 128));
    p.drawText(
        QRectF(8, height() - 18, width() - 16, 14),
        Qt::AlignRight | Qt::AlignVCenter,
        QStringLiteral("滚轮缩放 · 双击放大  %1×").arg(zoom_, 0, 'f', 1));
  }
}

void CoverageMapWidget::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
  invalidate_base();
}

void CoverageMapWidget::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton) {
    panning_ = true;
    last_mouse_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
  }
}

void CoverageMapWidget::mouseMoveEvent(QMouseEvent *event) {
  if (!panning_) {
    return;
  }
  const QPoint d = event->pos() - last_mouse_;
  last_mouse_ = event->pos();
  pan_ += QPointF(d);
  update();
}

void CoverageMapWidget::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton) {
    panning_ = false;
    setCursor(Qt::OpenHandCursor);
  }
}

void CoverageMapWidget::mouseDoubleClickEvent(QMouseEvent *event) {
  if (event->button() != Qt::LeftButton) {
    return;
  }
  if (enlarge_host_ || (event->modifiers() & Qt::ControlModifier)) {
    reset_view();
    return;
  }
  open_enlarged();
}

void CoverageMapWidget::wheelEvent(QWheelEvent *event) {
  const QPointF cursor = event->position();
  const double old_z = zoom_;
  const double factor = event->angleDelta().y() > 0 ? 1.15 : (1.0 / 1.15);
  const double new_z = clamp_zoom(old_z * factor);
  if (std::abs(new_z - old_z) < 1e-6) {
    return;
  }
  const QPointF world = (cursor - pan_) / old_z;
  zoom_ = new_z;
  pan_ = cursor - world * zoom_;
  update();
  event->accept();
}

void CoverageMapWidget::open_enlarged() {
  auto *dlg = new QDialog(window());
  dlg->setAttribute(Qt::WA_DeleteOnClose);
  dlg->setWindowTitle(QStringLiteral("覆盖 / 重投影 · 放大观察"));
  dlg->resize(900, 700);
  auto *lay = new QVBoxLayout(dlg);
  lay->setContentsMargins(10, 10, 10, 10);
  lay->setSpacing(8);
  auto *hint = new QLabel(
      QStringLiteral("滚轮缩放 · 拖拽平移 · Ctrl+双击复位"), dlg);
  hint->setObjectName(QStringLiteral("Muted"));
  lay->addWidget(hint);
  auto *chart = new CoverageMapWidget(dlg);
  chart->enlarge_host_ = true;
  chart->set_image_size(image_w_, image_h_);
  chart->set_points(points_);
  chart->set_filter_view(filter_view_);
  chart->zoom_ = std::max(1.5, zoom_);
  chart->pan_ = pan_;
  lay->addWidget(chart, 1);
  auto *btns = new QDialogButtonBox(QDialogButtonBox::Close, dlg);
  connect(btns->button(QDialogButtonBox::Close), &QPushButton::clicked, dlg, &QDialog::accept);
  lay->addWidget(btns);
  dlg->show();
}

}  // namespace gui
}  // namespace hs_calib
