#ifndef ROS_ROBOT_ASSIST_TOOLS__UI__ZOOMABLE_IMAGE_WIDGET_H_
#define ROS_ROBOT_ASSIST_TOOLS__UI__ZOOMABLE_IMAGE_WIDGET_H_

#include <QImage>
#include <QLabel>
#include <QPointF>

#include <functional>

class QToolButton;
class QWidget;

namespace ros_robot_assist_tools::ui
{

class ZoomableImageWidget : public QLabel
{
public:
  explicit ZoomableImageWidget(QWidget * parent = nullptr);

  void SetImage(const QImage & image);
  void ClearImage();
  void ResetView();
  void ZoomIn();
  void ZoomOut();
  void SetToolbarEnabled(bool enabled);
  void SetSyncViewEnabled(bool enabled);
  bool IsToolbarEnabled() const;
  bool IsSyncViewEnabled() const;
  QImage CurrentImage() const;
  QSize ImageSize() const;
  void SetViewTransform(double scale, const QPointF & pan, bool notify_callback = false);
  void SetViewTransformChangedCallback(const std::function<void(double, const QPointF &)> & cb);
  void SetHoverPixelCallback(const std::function<void(const QPoint &, bool)> & cb);
  void SetPixelInfoProvider(const std::function<QString(const QPoint &, const QColor &)> & cb);
  void SetSaveImageCallback(const std::function<void()> & cb);
  void SetRecordBagCallback(const std::function<void()> & cb);
  void SetRecordActive(bool active);
  void ShowSyncInfoForPixel(const QPoint & pixel);
  void HideSyncInfo();

protected:
  void wheelEvent(QWheelEvent * event) override;
  void mouseDoubleClickEvent(QMouseEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;
  void paintEvent(QPaintEvent * event) override;
  void leaveEvent(QEvent * event) override;

private:
  QImage image_;
  double scale_{1.0};
  QPoint pan_offset_{0, 0};
  bool dragging_{false};
  QPoint last_mouse_pos_{0, 0};
  bool toolbar_enabled_{false};
  bool sync_view_enabled_{false};
  QWidget * overlay_toolbar_{nullptr};
  QToolButton * zoom_in_btn_{nullptr};
  QToolButton * zoom_out_btn_{nullptr};
  QToolButton * reset_btn_{nullptr};
  QToolButton * save_btn_{nullptr};
  QToolButton * record_btn_{nullptr};
  std::function<void(double, const QPointF &)> on_transform_changed_;
  std::function<void(const QPoint &, bool)> on_hover_pixel_;
  std::function<QString(const QPoint &, const QColor &)> pixel_info_provider_;
  std::function<void()> on_save_image_;
  std::function<void()> on_record_bag_;
  bool record_active_{false};
  bool sync_info_visible_{false};
  QString sync_info_text_;
  QRect sync_info_rect_;
  bool ctrl_hover_active_{false};
  QPoint hover_pixel_{-1, -1};
  QPointF hover_widget_pos_{-1.0, -1.0};

  void EnsureToolbar();
  void UpdateToolbarGeometry();
  bool MapWidgetToImagePixel(const QPointF & widget_pos, QPoint * pixel) const;
  QPointF MapImagePixelToWidgetPos(const QPoint & pixel) const;
  void UpdateSyncInfoRect(const QPoint & pixel);
};

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__UI__ZOOMABLE_IMAGE_WIDGET_H_
