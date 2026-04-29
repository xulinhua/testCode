#include "image_debug_gui/image_view_widget.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <QColor>
#include <QFontMetrics>
#include <QMouseEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QToolTip>
#include <QWheelEvent>

#include <opencv2/imgproc.hpp>

namespace ImageDebug {

namespace {
constexpr double kMinScale = 0.05;
constexpr double kMaxScale = 20.0;
constexpr double kZoomStep = 1.15;
}  // namespace

ImageViewWidget::ImageViewWidget(QWidget *parent)
    : QWidget(parent),
      raw_encoding_(""),
      sync_mode_enabled_(false),
      sync_info_visible_(false),
      scale_factor_(1.0),
      pan_offset_(0.0, 0.0),
      dragging_(false) {
  setMinimumSize(320, 240);
  setMouseTracking(true);
}

void ImageViewWidget::setImage(const QImage &image) {
  const bool need_refit = original_image_.isNull() ||
                          original_image_.size() != image.size();
  original_image_ = image;
  raw_image_ = cv::Mat();
  raw_encoding_.clear();
  if (need_refit) {
    fitToWindow();
  }
  update();
}

void ImageViewWidget::setImageData(const QImage &image, const cv::Mat &raw_image,
                                   const QString &encoding) {
  const bool need_refit = original_image_.isNull() ||
                          original_image_.size() != image.size();
  original_image_ = image;
  raw_image_ = raw_image.clone();
  raw_encoding_ = encoding;
  if (need_refit) {
    fitToWindow();
  }
  update();
}

void ImageViewWidget::resetView() {
  scale_factor_ = 1.0;
  pan_offset_ = QPointF(0.0, 0.0);
  update();
}

void ImageViewWidget::zoomIn() {
  if (original_image_.isNull()) {
    return;
  }
  const QPointF center(width() * 0.5, height() * 0.5);
  const QPointF image_pos_before = (center - pan_offset_) / scale_factor_;
  scale_factor_ = std::clamp(scale_factor_ * kZoomStep, kMinScale, kMaxScale);
  pan_offset_ = center - image_pos_before * scale_factor_;
  update();
  emit viewTransformChanged(scale_factor_, pan_offset_);
}

void ImageViewWidget::zoomOut() {
  if (original_image_.isNull()) {
    return;
  }
  const QPointF center(width() * 0.5, height() * 0.5);
  const QPointF image_pos_before = (center - pan_offset_) / scale_factor_;
  scale_factor_ = std::clamp(scale_factor_ / kZoomStep, kMinScale, kMaxScale);
  pan_offset_ = center - image_pos_before * scale_factor_;
  update();
  emit viewTransformChanged(scale_factor_, pan_offset_);
}

QImage ImageViewWidget::currentImage() const { return original_image_; }

QSize ImageViewWidget::imageSize() const { return original_image_.size(); }

void ImageViewWidget::setViewTransform(double scale_factor, const QPointF &pan_offset,
                                       bool emit_signal) {
  if (original_image_.isNull()) {
    return;
  }
  scale_factor_ = std::clamp(scale_factor, kMinScale, kMaxScale);
  pan_offset_ = pan_offset;
  update();
  if (emit_signal) {
    emit viewTransformChanged(scale_factor_, pan_offset_);
  }
}

void ImageViewWidget::setSyncModeEnabled(bool enabled) {
  sync_mode_enabled_ = enabled;
  if (!enabled) {
    hideSyncInfo();
  }
}

void ImageViewWidget::showSyncInfoForPixel(const QPoint &pixel) {
  if (original_image_.isNull()) {
    return;
  }
  if (pixel.x() < 0 || pixel.y() < 0 || pixel.x() >= original_image_.width() ||
      pixel.y() >= original_image_.height()) {
    hideSyncInfo();
    return;
  }
  sync_info_text_ = buildPixelInfo(pixel);
  sync_info_rect_ = buildInfoRect(sync_info_text_, mapImagePixelToWidgetPos(pixel));
  sync_info_visible_ = true;
  update();
}

void ImageViewWidget::hideSyncInfo() {
  if (!sync_info_visible_) {
    return;
  }
  sync_info_visible_ = false;
  sync_info_text_.clear();
  sync_info_rect_ = QRect();
  update();
}

void ImageViewWidget::showTooltipForPixel(const QPoint &pixel) {
  if (original_image_.isNull()) {
    return;
  }
  if (pixel.x() < 0 || pixel.y() < 0 || pixel.x() >= original_image_.width() ||
      pixel.y() >= original_image_.height()) {
    return;
  }
  const QString text = buildPixelInfo(pixel);
  const QPointF widget_pos = mapImagePixelToWidgetPos(pixel);
  QToolTip::showText(mapToGlobal(widget_pos.toPoint()) + QPoint(16, 0), text, this);
}

void ImageViewWidget::hideTooltip() { QToolTip::hideText(); }

void ImageViewWidget::fitToWindow() {
  if (original_image_.isNull() || width() <= 0 || height() <= 0) {
    return;
  }

  const double sx = static_cast<double>(width()) / static_cast<double>(original_image_.width());
  const double sy = static_cast<double>(height()) / static_cast<double>(original_image_.height());
  scale_factor_ = std::clamp(std::min(sx, sy), kMinScale, kMaxScale);

  const double draw_w = original_image_.width() * scale_factor_;
  const double draw_h = original_image_.height() * scale_factor_;
  pan_offset_.setX((width() - draw_w) * 0.5);
  pan_offset_.setY((height() - draw_h) * 0.5);
  update();
  emit viewTransformChanged(scale_factor_, pan_offset_);
}

void ImageViewWidget::paintEvent(QPaintEvent *event) {
  QWidget::paintEvent(event);

  QPainter painter(this);
  painter.fillRect(rect(), QColor(30, 30, 30));
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

  if (original_image_.isNull()) {
    painter.setPen(Qt::lightGray);
    painter.drawText(rect(), Qt::AlignCenter, "No Image");
    return;
  }

  painter.translate(pan_offset_);
  painter.scale(scale_factor_, scale_factor_);
  // Always draw from original image to avoid repeated interpolation blur.
  painter.drawImage(QPointF(0.0, 0.0), original_image_);

  if (sync_info_visible_ && !sync_info_text_.isEmpty()) {
    painter.resetTransform();
    painter.setPen(QColor(40, 40, 40));
    painter.setBrush(QColor(245, 245, 220, 235));
    painter.drawRect(sync_info_rect_);
    painter.setPen(Qt::black);
    painter.drawText(sync_info_rect_.adjusted(6, 4, -6, -4), Qt::TextWordWrap, sync_info_text_);
  }
}

void ImageViewWidget::wheelEvent(QWheelEvent *event) {
  if (original_image_.isNull()) {
    return;
  }

  const QPointF cursor_pos = event->position();
  const QPointF image_pos_before = (cursor_pos - pan_offset_) / scale_factor_;

  if (event->angleDelta().y() > 0) {
    scale_factor_ *= kZoomStep;
  } else {
    scale_factor_ /= kZoomStep;
  }
  scale_factor_ = std::clamp(scale_factor_, kMinScale, kMaxScale);

  pan_offset_ = cursor_pos - image_pos_before * scale_factor_;
  update();
  emit viewTransformChanged(scale_factor_, pan_offset_);
}

void ImageViewWidget::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    dragging_ = true;
    last_mouse_pos_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
  }
  QWidget::mousePressEvent(event);
}

void ImageViewWidget::mouseDoubleClickEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    dragging_ = false;
    setCursor(Qt::ArrowCursor);
    fitToWindow();
  }
  QWidget::mouseDoubleClickEvent(event);
}

void ImageViewWidget::mouseMoveEvent(QMouseEvent *event) {
  if (dragging_) {
    const QPoint delta = event->pos() - last_mouse_pos_;
    pan_offset_ += QPointF(delta.x(), delta.y());
    last_mouse_pos_ = event->pos();
    update();
    emit viewTransformChanged(scale_factor_, pan_offset_);
  }

  if (event->modifiers().testFlag(Qt::ControlModifier)) {
    QPoint pixel;
    if (mapWidgetToImagePixel(event->pos(), &pixel)) {
      if (sync_mode_enabled_) {
        showSyncInfoForPixel(pixel);
      } else {
        showTooltipForPixel(pixel);
      }
      emit hoverPixelChanged(pixel, true);
    } else {
      hideTooltip();
      hideSyncInfo();
      emit hoverPixelChanged(QPoint(), false);
    }
  } else {
    hideTooltip();
    hideSyncInfo();
    emit hoverPixelChanged(QPoint(), false);
  }

  QWidget::mouseMoveEvent(event);
}

void ImageViewWidget::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    dragging_ = false;
    setCursor(Qt::ArrowCursor);
  }
  QWidget::mouseReleaseEvent(event);
}

void ImageViewWidget::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
  if (!original_image_.isNull()) {
    fitToWindow();
  }
}

void ImageViewWidget::leaveEvent(QEvent *event) {
  hideTooltip();
  hideSyncInfo();
  emit hoverPixelChanged(QPoint(), false);
  QWidget::leaveEvent(event);
}

bool ImageViewWidget::mapWidgetToImagePixel(const QPointF &widget_pos, QPoint *pixel) const {
  if (original_image_.isNull() || scale_factor_ <= 0.0 || pixel == nullptr) {
    return false;
  }

  const QPointF image_pos = (widget_pos - pan_offset_) / scale_factor_;
  const int x = static_cast<int>(std::floor(image_pos.x()));
  const int y = static_cast<int>(std::floor(image_pos.y()));
  if (x < 0 || y < 0 || x >= original_image_.width() || y >= original_image_.height()) {
    return false;
  }

  *pixel = QPoint(x, y);
  return true;
}

QPointF ImageViewWidget::mapImagePixelToWidgetPos(const QPoint &pixel) const {
  return QPointF(pixel.x() + 0.5, pixel.y() + 0.5) * scale_factor_ + pan_offset_;
}

QRect ImageViewWidget::buildInfoRect(const QString &text, const QPointF &anchor_widget_pos) const {
  QFontMetrics fm(font());
  const QRect text_rect = fm.boundingRect(QRect(0, 0, 320, 200), Qt::TextWordWrap, text);
  const int w = text_rect.width() + 12;
  const int h = text_rect.height() + 8;
  int x = static_cast<int>(std::round(anchor_widget_pos.x())) + 16;
  int y = static_cast<int>(std::round(anchor_widget_pos.y())) - h / 2;
  x = std::clamp(x, 0, std::max(0, width() - w));
  y = std::clamp(y, 0, std::max(0, height() - h));
  return QRect(x, y, w, h);
}

QString ImageViewWidget::buildPixelInfo(const QPoint &pixel) const {
  QString value_text = "N/A";
  bool is_color = false;
  const QString encoding = raw_encoding_.toLower();

  if (!raw_image_.empty() && pixel.y() < raw_image_.rows && pixel.x() < raw_image_.cols) {
    const int type = raw_image_.type();
    if (type == CV_8UC1) {
      value_text = QString::number(raw_image_.at<std::uint8_t>(pixel.y(), pixel.x()));
    } else if (type == CV_16UC1) {
      const std::uint16_t raw = raw_image_.at<std::uint16_t>(pixel.y(), pixel.x());
      if (encoding == "16uc1" || encoding == "mono16") {
        if (raw == 0U) {
          value_text = "无效";
        } else {
          value_text = QString("%1 mm").arg(raw);
        }
      } else {
        value_text = QString::number(raw);
      }
    } else if (type == CV_16SC1) {
      value_text = QString::number(raw_image_.at<std::int16_t>(pixel.y(), pixel.x()));
    } else if (type == CV_32FC1) {
      const float raw = raw_image_.at<float>(pixel.y(), pixel.x());
      if (encoding == "32fc1") {
        if (std::isnan(raw) || !std::isfinite(raw) || raw <= 0.0f) {
          value_text = "无效";
        } else {
          value_text = QString("%1 mm").arg(raw * 1000.0f, 0, 'f', 1);
        }
      } else {
        value_text = QString::number(raw, 'f', 3);
      }
    } else if (type == CV_64FC1) {
      const double raw = raw_image_.at<double>(pixel.y(), pixel.x());
      if (encoding == "64fc1") {
        if (std::isnan(raw) || !std::isfinite(raw) || raw <= 0.0) {
          value_text = "无效";
        } else {
          value_text = QString("%1 mm").arg(raw * 1000.0, 0, 'f', 1);
        }
      } else {
        value_text = QString::number(raw, 'f', 3);
      }
    } else if (type == CV_8UC3) {
      const cv::Vec3b px = raw_image_.at<cv::Vec3b>(pixel.y(), pixel.x());
      int r = 0;
      int g = 0;
      int b = 0;
      if (raw_encoding_ == "rgb8") {
        r = px[0];
        g = px[1];
        b = px[2];
      } else {
        // Most ROS color topics are bgr8 by default.
        b = px[0];
        g = px[1];
        r = px[2];
      }
      value_text = QString("R:%1 G:%2 B:%3").arg(r).arg(g).arg(b);
      is_color = true;
    } else if (type == CV_8UC4) {
      const cv::Vec4b px = raw_image_.at<cv::Vec4b>(pixel.y(), pixel.x());
      int r = 0;
      int g = 0;
      int b = 0;
      if (raw_encoding_ == "rgba8") {
        r = px[0];
        g = px[1];
        b = px[2];
      } else {
        b = px[0];
        g = px[1];
        r = px[2];
      }
      value_text = QString("R:%1 G:%2 B:%3").arg(r).arg(g).arg(b);
      is_color = true;
    } else {
      const QColor color = original_image_.pixelColor(pixel);
      value_text = QString("R:%1 G:%2 B:%3").arg(color.red()).arg(color.green()).arg(color.blue());
      is_color = true;
    }
  } else if (!original_image_.isNull()) {
    const QColor color = original_image_.pixelColor(pixel);
    value_text = QString("R:%1 G:%2 B:%3").arg(color.red()).arg(color.green()).arg(color.blue());
    is_color = true;
  }

  QString type_label = raw_encoding_.isEmpty() ? (is_color ? "color" : "gray") : raw_encoding_;
  return QString("X:%1  Y:%2\nValue(%3): %4")
      .arg(pixel.x())
      .arg(pixel.y())
      .arg(type_label)
      .arg(value_text);
}

}  // namespace ImageDebug
