#ifndef ROS_ROBOT_ASSIST_TOOLS__UI__ZOOMABLE_IMAGE_WIDGET_H_
#define ROS_ROBOT_ASSIST_TOOLS__UI__ZOOMABLE_IMAGE_WIDGET_H_

#include <QImage>
#include <QLabel>

namespace ros_robot_assist_tools::ui
{

class ZoomableImageWidget : public QLabel
{
public:
  explicit ZoomableImageWidget(QWidget * parent = nullptr);

  void SetImage(const QImage & image);
  void ClearImage();
  void ResetView();

protected:
  void wheelEvent(QWheelEvent * event) override;
  void mouseDoubleClickEvent(QMouseEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;
  void paintEvent(QPaintEvent * event) override;

private:
  QImage image_;
  double scale_{1.0};
  QPoint pan_offset_{0, 0};
  bool dragging_{false};
  QPoint last_mouse_pos_{0, 0};
};

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__UI__ZOOMABLE_IMAGE_WIDGET_H_
