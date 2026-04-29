#pragma once

#include <QImage>
#include <QPoint>
#include <QString>
#include <QWidget>

#include <opencv2/core/mat.hpp>

class QMouseEvent;
class QPaintEvent;
class QResizeEvent;
class QWheelEvent;

namespace ImageDebug {

class ImageViewWidget : public QWidget {
  Q_OBJECT

public:
  explicit ImageViewWidget(QWidget *parent = nullptr);

  void setImage(const QImage &image);
  void setImageData(const QImage &image, const cv::Mat &raw_image, const QString &encoding);
  void resetView();
  void fitToWindow();
  void zoomIn();
  void zoomOut();
  QImage currentImage() const;
  QSize imageSize() const;
  void setViewTransform(double scale_factor, const QPointF &pan_offset, bool emit_signal = false);
  void setSyncModeEnabled(bool enabled);
  void showSyncInfoForPixel(const QPoint &pixel);
  void hideSyncInfo();
  void showTooltipForPixel(const QPoint &pixel);
  void hideTooltip();

protected:
  void paintEvent(QPaintEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;
  void leaveEvent(QEvent *event) override;

signals:
  void hoverPixelChanged(const QPoint &pixel, bool active);
  void viewTransformChanged(double scale_factor, const QPointF &pan_offset);

private:
  bool mapWidgetToImagePixel(const QPointF &widget_pos, QPoint *pixel) const;
  QPointF mapImagePixelToWidgetPos(const QPoint &pixel) const;
  QString buildPixelInfo(const QPoint &pixel) const;
  QRect buildInfoRect(const QString &text, const QPointF &anchor_widget_pos) const;

  QImage original_image_;
  cv::Mat raw_image_;
  QString raw_encoding_;
  bool sync_mode_enabled_;
  bool sync_info_visible_;
  QString sync_info_text_;
  QRect sync_info_rect_;
  double scale_factor_;
  QPointF pan_offset_;
  bool dragging_;
  QPoint last_mouse_pos_;
};

}  // namespace ImageDebug
