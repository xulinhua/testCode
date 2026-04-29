#include "ros_robot_assist_tools/ui/zoomable_image_widget.h"

#include <algorithm>
#include <cmath>

#include <QFontMetrics>
#include <QEvent>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QPainter>
#include <QStyle>
#include <QToolButton>
#include <QWheelEvent>

namespace ros_robot_assist_tools::ui
{

ZoomableImageWidget::ZoomableImageWidget(QWidget * parent)
: QLabel(parent)
{
  setAlignment(Qt::AlignCenter);
  setCursor(Qt::ArrowCursor);
  setMouseTracking(true);
}

void ZoomableImageWidget::SetImage(const QImage & image)
{
  image_ = image;
  if ((ctrl_hover_active_ || sync_info_visible_) && hover_pixel_.x() >= 0 && hover_pixel_.y() >= 0) {
    ShowSyncInfoForPixel(hover_pixel_);
  } else {
    HideSyncInfo();
  }
  update();
}

void ZoomableImageWidget::ClearImage()
{
  image_ = QImage();
  ctrl_hover_active_ = false;
  hover_pixel_ = QPoint(-1, -1);
  HideSyncInfo();
  ResetView();
}

void ZoomableImageWidget::ResetView()
{
  scale_ = 1.0;
  pan_offset_ = QPoint(0, 0);
  dragging_ = false;
  setCursor(Qt::ArrowCursor);
  HideSyncInfo();
  update();
}

void ZoomableImageWidget::ZoomIn()
{
  scale_ = std::max(0.2, std::min(8.0, scale_ * 1.1));
  if (on_transform_changed_) {
    on_transform_changed_(scale_, QPointF(pan_offset_));
  }
  update();
}

void ZoomableImageWidget::ZoomOut()
{
  scale_ = std::max(0.2, std::min(8.0, scale_ / 1.1));
  if (on_transform_changed_) {
    on_transform_changed_(scale_, QPointF(pan_offset_));
  }
  update();
}

void ZoomableImageWidget::SetToolbarEnabled(bool enabled)
{
  toolbar_enabled_ = enabled;
  EnsureToolbar();
  if (overlay_toolbar_ != nullptr) {
    overlay_toolbar_->setVisible(toolbar_enabled_);
  }
}

void ZoomableImageWidget::SetSyncViewEnabled(bool enabled)
{
  sync_view_enabled_ = enabled;
}

bool ZoomableImageWidget::IsToolbarEnabled() const
{
  return toolbar_enabled_;
}

bool ZoomableImageWidget::IsSyncViewEnabled() const
{
  return sync_view_enabled_;
}

QImage ZoomableImageWidget::CurrentImage() const
{
  return image_;
}

QSize ZoomableImageWidget::ImageSize() const
{
  return image_.size();
}

void ZoomableImageWidget::SetViewTransform(double scale, const QPointF & pan, bool notify_callback)
{
  scale_ = std::max(0.2, std::min(8.0, scale));
  pan_offset_.setX(static_cast<int>(std::round(pan.x())));
  pan_offset_.setY(static_cast<int>(std::round(pan.y())));
  if (notify_callback && on_transform_changed_) {
    on_transform_changed_(scale_, QPointF(pan_offset_));
  }
  update();
}

void ZoomableImageWidget::SetViewTransformChangedCallback(
  const std::function<void(double, const QPointF &)> & cb)
{
  on_transform_changed_ = cb;
}

void ZoomableImageWidget::SetHoverPixelCallback(
  const std::function<void(const QPoint &, bool)> & cb)
{
  on_hover_pixel_ = cb;
}

void ZoomableImageWidget::SetPixelInfoProvider(
  const std::function<QString(const QPoint &, const QColor &)> & cb)
{
  pixel_info_provider_ = cb;
}

void ZoomableImageWidget::SetSaveImageCallback(const std::function<void()> & cb)
{
  on_save_image_ = cb;
}

void ZoomableImageWidget::SetRecordBagCallback(const std::function<void()> & cb)
{
  on_record_bag_ = cb;
}

void ZoomableImageWidget::SetRecordActive(bool active)
{
  record_active_ = active;
  if (record_btn_) {
    record_btn_->setIcon(style()->standardIcon(record_active_ ? QStyle::SP_MediaStop : QStyle::SP_MediaPlay));
    record_btn_->setToolTip(record_active_ ? "录制中，点击停止" : "开始 rosbag 录制");
  }
}

void ZoomableImageWidget::ShowSyncInfoForPixel(const QPoint & pixel)
{
  if (image_.isNull()) {
    return;
  }
  if (pixel.x() < 0 || pixel.y() < 0 || pixel.x() >= image_.width() || pixel.y() >= image_.height()) {
    HideSyncInfo();
    return;
  }
  const QColor color = image_.pixelColor(pixel);
  if (pixel_info_provider_) {
    sync_info_text_ = pixel_info_provider_(pixel, color);
  } else if (color.red() == color.green() && color.red() == color.blue()) {
    sync_info_text_ = QString("X:%1  Y:%2\nGray: %3").arg(pixel.x()).arg(pixel.y()).arg(color.red());
  } else {
    sync_info_text_ = QString("X:%1  Y:%2\nR:%3 G:%4 B:%5")
      .arg(pixel.x()).arg(pixel.y()).arg(color.red()).arg(color.green()).arg(color.blue());
  }
  sync_info_visible_ = true;
  hover_pixel_ = pixel;
  hover_widget_pos_ = MapImagePixelToWidgetPos(pixel);
  UpdateSyncInfoRect(pixel);
  update();
}

void ZoomableImageWidget::HideSyncInfo()
{
  if (!sync_info_visible_) {
    return;
  }
  sync_info_visible_ = false;
  sync_info_text_.clear();
  sync_info_rect_ = QRect();
  hover_widget_pos_ = QPointF(-1.0, -1.0);
  update();
}

void ZoomableImageWidget::wheelEvent(QWheelEvent * event)
{
  if (image_.isNull()) {
    QLabel::wheelEvent(event);
    return;
  }
  const QPoint delta = event->angleDelta();
  if (delta.y() == 0) {
    return;
  }
  const QPointF cursor = event->position();
  const QPointF center(width() * 0.5, height() * 0.5);
  const QPointF old_rel = cursor - center - QPointF(pan_offset_);
  const double old_scale = scale_;
  const double step = (delta.y() > 0) ? 1.1 : (1.0 / 1.1);
  scale_ = std::max(0.2, std::min(8.0, scale_ * step));
  if (old_scale > 1e-9) {
    const double ratio = scale_ / old_scale;
    const QPointF new_pan = cursor - center - old_rel * ratio;
    pan_offset_.setX(static_cast<int>(std::round(new_pan.x())));
    pan_offset_.setY(static_cast<int>(std::round(new_pan.y())));
  }
  if (on_transform_changed_) {
    on_transform_changed_(scale_, QPointF(pan_offset_));
  }
  update();
  event->accept();
}

void ZoomableImageWidget::mouseDoubleClickEvent(QMouseEvent * event)
{
  Q_UNUSED(event);
  ResetView();
}

void ZoomableImageWidget::mousePressEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    dragging_ = true;
    last_mouse_pos_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }
  QLabel::mousePressEvent(event);
}

void ZoomableImageWidget::mouseMoveEvent(QMouseEvent * event)
{
  if (dragging_) {
    const QPoint delta = event->pos() - last_mouse_pos_;
    pan_offset_ += delta;
    last_mouse_pos_ = event->pos();
    if (on_transform_changed_) {
      on_transform_changed_(scale_, QPointF(pan_offset_));
    }
    update();
    event->accept();
    return;
  }
  if (!event->modifiers().testFlag(Qt::ControlModifier)) {
    ctrl_hover_active_ = false;
    hover_pixel_ = QPoint(-1, -1);
    HideSyncInfo();
    if (on_hover_pixel_) {
      on_hover_pixel_(QPoint(), false);
    }
    QLabel::mouseMoveEvent(event);
    return;
  }

  QPoint pixel;
  if (MapWidgetToImagePixel(event->pos(), &pixel)) {
    ctrl_hover_active_ = true;
    hover_pixel_ = pixel;
    if (on_hover_pixel_) {
      on_hover_pixel_(pixel, true);
    }
    ShowSyncInfoForPixel(pixel);
  } else {
    ctrl_hover_active_ = false;
    hover_pixel_ = QPoint(-1, -1);
    HideSyncInfo();
    if (on_hover_pixel_) {
      on_hover_pixel_(QPoint(), false);
    }
  }
  QLabel::mouseMoveEvent(event);
}

void ZoomableImageWidget::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton && dragging_) {
    dragging_ = false;
    setCursor(Qt::ArrowCursor);
    event->accept();
    return;
  }
  QLabel::mouseReleaseEvent(event);
}

void ZoomableImageWidget::resizeEvent(QResizeEvent * event)
{
  QLabel::resizeEvent(event);
  UpdateToolbarGeometry();
  update();
}

void ZoomableImageWidget::paintEvent(QPaintEvent * event)
{
  QLabel::paintEvent(event);
  if (image_.isNull()) {
    return;
  }
  const QSize avail = size();
  if (avail.width() < 1 || avail.height() < 1) {
    return;
  }
  const QSize fitted = image_.size().scaled(avail, Qt::KeepAspectRatio);
  if (fitted.width() < 1 || fitted.height() < 1) {
    return;
  }
  const int draw_w = std::max(1, static_cast<int>(std::round(static_cast<double>(fitted.width()) * scale_)));
  const int draw_h = std::max(1, static_cast<int>(std::round(static_cast<double>(fitted.height()) * scale_)));

  QPainter painter(this);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
  const QPixmap pix = QPixmap::fromImage(image_).scaled(draw_w, draw_h, Qt::KeepAspectRatio, Qt::FastTransformation);
  const int x = (width() - pix.width()) / 2 + pan_offset_.x();
  const int y = (height() - pix.height()) / 2 + pan_offset_.y();
  painter.drawPixmap(x, y, pix);
  if (sync_info_visible_ && !ctrl_hover_active_ &&
      hover_widget_pos_.x() >= 0.0 && hover_widget_pos_.y() >= 0.0) {
    const int cx = static_cast<int>(std::round(hover_widget_pos_.x()));
    const int cy = static_cast<int>(std::round(hover_widget_pos_.y()));
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(QColor(255, 80, 80), 1.5));
    painter.drawEllipse(QPoint(cx, cy), 5, 5);
    painter.drawLine(cx - 10, cy, cx + 10, cy);
    painter.drawLine(cx, cy - 10, cx, cy + 10);
    painter.setPen(QPen(QColor(255, 255, 255), 1));
    painter.drawEllipse(QPoint(cx, cy), 1, 1);
  }
  if (sync_info_visible_ && !sync_info_text_.isEmpty()) {
    painter.setPen(QColor(40, 40, 40));
    painter.setBrush(QColor(245, 245, 220, 235));
    painter.drawRect(sync_info_rect_);
    painter.setPen(Qt::black);
    painter.drawText(sync_info_rect_.adjusted(6, 4, -6, -4), Qt::TextWordWrap, sync_info_text_);
  }
  event->accept();
}

void ZoomableImageWidget::leaveEvent(QEvent * event)
{
  ctrl_hover_active_ = false;
  hover_pixel_ = QPoint(-1, -1);
  hover_widget_pos_ = QPointF(-1.0, -1.0);
  HideSyncInfo();
  if (on_hover_pixel_) {
    on_hover_pixel_(QPoint(), false);
  }
  QLabel::leaveEvent(event);
}

void ZoomableImageWidget::EnsureToolbar()
{
  if (overlay_toolbar_ != nullptr) {
    return;
  }
  overlay_toolbar_ = new QWidget(this);
  overlay_toolbar_->setStyleSheet("background-color: rgba(20,20,20,150); border-radius:3px;");
  auto * layout = new QHBoxLayout(overlay_toolbar_);
  layout->setContentsMargins(3, 2, 3, 2);
  layout->setSpacing(2);

  zoom_in_btn_ = new QToolButton(overlay_toolbar_);
  zoom_out_btn_ = new QToolButton(overlay_toolbar_);
  reset_btn_ = new QToolButton(overlay_toolbar_);
  save_btn_ = new QToolButton(overlay_toolbar_);
  record_btn_ = new QToolButton(overlay_toolbar_);
  save_btn_->setIcon(style()->standardIcon(QStyle::SP_DialogSaveButton));
  zoom_in_btn_->setIcon(style()->standardIcon(QStyle::SP_ArrowUp));
  zoom_out_btn_->setIcon(style()->standardIcon(QStyle::SP_ArrowDown));
  reset_btn_->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
  record_btn_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
  save_btn_->setToolTip("保存图像");
  zoom_in_btn_->setToolTip("放大");
  zoom_out_btn_->setToolTip("缩小");
  reset_btn_->setToolTip("重置视图");
  record_btn_->setToolTip("开始 rosbag 录制");
  for (QToolButton * btn : {save_btn_, zoom_in_btn_, zoom_out_btn_, reset_btn_, record_btn_}) {
    btn->setAutoRaise(true);
    btn->setIconSize(QSize(14, 14));
    btn->setFixedSize(20, 20);
    layout->addWidget(btn);
  }
  QObject::connect(save_btn_, &QToolButton::clicked, [this]() {
    if (on_save_image_) on_save_image_();
  });
  QObject::connect(zoom_in_btn_, &QToolButton::clicked, [this]() { ZoomIn(); });
  QObject::connect(zoom_out_btn_, &QToolButton::clicked, [this]() { ZoomOut(); });
  QObject::connect(reset_btn_, &QToolButton::clicked, [this]() { ResetView(); });
  QObject::connect(record_btn_, &QToolButton::clicked, [this]() {
    if (on_record_bag_) on_record_bag_();
  });
  SetRecordActive(record_active_);
  UpdateToolbarGeometry();
  overlay_toolbar_->setVisible(toolbar_enabled_);
}

void ZoomableImageWidget::UpdateToolbarGeometry()
{
  if (overlay_toolbar_ == nullptr) {
    return;
  }
  overlay_toolbar_->adjustSize();
  const QSize size_hint = overlay_toolbar_->sizeHint();
  const int margin = 8;
  overlay_toolbar_->setGeometry(
    width() - size_hint.width() - margin, margin, size_hint.width(), size_hint.height());
}

bool ZoomableImageWidget::MapWidgetToImagePixel(const QPointF & widget_pos, QPoint * pixel) const
{
  if (image_.isNull() || pixel == nullptr || scale_ <= 1e-9) {
    return false;
  }
  const QSize avail = size();
  const QSize fitted = image_.size().scaled(avail, Qt::KeepAspectRatio);
  if (fitted.width() < 1 || fitted.height() < 1) {
    return false;
  }
  const double draw_w = static_cast<double>(fitted.width()) * scale_;
  const double draw_h = static_cast<double>(fitted.height()) * scale_;
  const double draw_x = (width() - draw_w) * 0.5 + pan_offset_.x();
  const double draw_y = (height() - draw_h) * 0.5 + pan_offset_.y();
  const double rx = (widget_pos.x() - draw_x) / draw_w;
  const double ry = (widget_pos.y() - draw_y) / draw_h;
  if (rx < 0.0 || rx >= 1.0 || ry < 0.0 || ry >= 1.0) {
    return false;
  }
  const int px = static_cast<int>(std::floor(rx * image_.width()));
  const int py = static_cast<int>(std::floor(ry * image_.height()));
  if (px < 0 || py < 0 || px >= image_.width() || py >= image_.height()) {
    return false;
  }
  *pixel = QPoint(px, py);
  return true;
}

QPointF ZoomableImageWidget::MapImagePixelToWidgetPos(const QPoint & pixel) const
{
  const QSize avail = size();
  const QSize fitted = image_.size().scaled(avail, Qt::KeepAspectRatio);
  const double draw_w = static_cast<double>(fitted.width()) * scale_;
  const double draw_h = static_cast<double>(fitted.height()) * scale_;
  const double draw_x = (width() - draw_w) * 0.5 + pan_offset_.x();
  const double draw_y = (height() - draw_h) * 0.5 + pan_offset_.y();
  const double x = draw_x + (static_cast<double>(pixel.x()) + 0.5) * draw_w / image_.width();
  const double y = draw_y + (static_cast<double>(pixel.y()) + 0.5) * draw_h / image_.height();
  return QPointF(x, y);
}

void ZoomableImageWidget::UpdateSyncInfoRect(const QPoint & pixel)
{
  QFontMetrics fm(font());
  const QRect text_rect = fm.boundingRect(QRect(0, 0, 260, 160), Qt::TextWordWrap, sync_info_text_);
  const int w = text_rect.width() + 12;
  const int h = text_rect.height() + 8;
  const QPointF anchor = MapImagePixelToWidgetPos(pixel);
  int x = static_cast<int>(std::round(anchor.x())) + 16;
  int y = static_cast<int>(std::round(anchor.y())) - h / 2;
  x = std::clamp(x, 0, std::max(0, width() - w));
  y = std::clamp(y, 0, std::max(0, height() - h));
  sync_info_rect_ = QRect(x, y, w, h);
}

}  // namespace ros_robot_assist_tools::ui
