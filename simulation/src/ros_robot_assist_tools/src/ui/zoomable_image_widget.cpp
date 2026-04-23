#include "ros_robot_assist_tools/ui/zoomable_image_widget.h"

#include <algorithm>
#include <cmath>

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

namespace ros_robot_assist_tools::ui
{

ZoomableImageWidget::ZoomableImageWidget(QWidget * parent)
: QLabel(parent)
{
  setAlignment(Qt::AlignCenter);
  setCursor(Qt::ArrowCursor);
}

void ZoomableImageWidget::SetImage(const QImage & image)
{
  image_ = image;
  update();
}

void ZoomableImageWidget::ClearImage()
{
  image_ = QImage();
  ResetView();
}

void ZoomableImageWidget::ResetView()
{
  scale_ = 1.0;
  pan_offset_ = QPoint(0, 0);
  dragging_ = false;
  setCursor(Qt::ArrowCursor);
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
    update();
    event->accept();
    return;
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
  event->accept();
}

}  // namespace ros_robot_assist_tools::ui
