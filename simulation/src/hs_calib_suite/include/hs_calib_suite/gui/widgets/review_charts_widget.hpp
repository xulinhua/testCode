#pragma once

#include <vector>

#include <QColor>
#include <QPixmap>
#include <QPoint>
#include <QPointF>
#include <QSize>
#include <QString>
#include <QWidget>

class QMouseEvent;
class QPaintEvent;
class QResizeEvent;
class QWheelEvent;

namespace hs_calib {
namespace gui {

/// \brief 逐帧重投影 RMS 柱状图（原图缓存 + 滚轮缩放 / 拖拽平移 / 双击放大）
class ResidualBarWidget : public QWidget {
  Q_OBJECT
public:
  struct Bar {
    QString label;
    double rms_px = 0.0;
    bool ok = true;
    int view_index = -1;
  };

  explicit ResidualBarWidget(QWidget *parent = nullptr);

  void set_bars(const std::vector<Bar> &bars);
  void set_highlight_view(int view_index);
  void clear();
  void reset_view();
  void open_enlarged();
  /// \brief 离屏渲染为 PNG（导出结果包用）
  bool export_png(
      const QString &path, const QSize &logical_size = QSize(1280, 480)) const;
  bool has_data() const { return !bars_.empty(); }

signals:
  void bar_clicked(int view_index);

protected:
  void paintEvent(QPaintEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  QPointF map_to_content(const QPointF &widget_pos) const;
  void paint_chart_background(QPainter &p, const QRectF &rc) const;
  void paint_chart_bars(QPainter &p, const QRectF &rc) const;
  void paint_chart(QPainter &p, const QRectF &rc) const;
  void invalidate_base();
  void ensure_base_pixmap();

  std::vector<Bar> bars_;
  int highlight_view_ = -1;
  double max_rms_ = 1.0;
  double zoom_ = 1.0;
  QPointF pan_{0.0, 0.0};
  bool panning_ = false;
  QPoint last_mouse_;
  bool enlarge_host_ = false;
  QPixmap base_pixmap_;
  bool base_dirty_ = true;
};

/// \brief 像面覆盖 + 残差着色散点图（原图缓存 + 滚轮缩放 / 拖拽平移 / 双击放大）
class CoverageMapWidget : public QWidget {
  Q_OBJECT
public:
  struct Point {
    float u = 0.f;
    float v = 0.f;
    float err_px = 0.f;
    int view_index = -1;
  };

  explicit CoverageMapWidget(QWidget *parent = nullptr);

  void set_image_size(int width, int height);
  void set_points(const std::vector<Point> &points);
  void set_filter_view(int view_index);  ///< <0 显示全部
  void clear();
  void reset_view();
  void open_enlarged();
  /// \brief 离屏渲染为 PNG（导出结果包用）
  bool export_png(
      const QString &path, const QSize &logical_size = QSize(1280, 960)) const;
  bool has_data() const { return image_w_ > 0 && image_h_ > 0 && !points_.empty(); }

protected:
  void paintEvent(QPaintEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  static QColor color_for_error(float err_px, float max_err);
  QPointF map_to_content(const QPointF &widget_pos) const;
  QRectF plot_frame(const QRectF &rc) const;
  void paint_chart_background(QPainter &p, const QRectF &rc) const;
  void paint_chart_points(QPainter &p, const QRectF &rc, int filter_view) const;
  void paint_chart(QPainter &p, const QRectF &rc) const;
  void invalidate_base();
  void ensure_base_pixmap();

  int image_w_ = 0;
  int image_h_ = 0;
  std::vector<Point> points_;
  int filter_view_ = -1;
  float max_err_ = 2.0f;
  double zoom_ = 1.0;
  QPointF pan_{0.0, 0.0};
  bool panning_ = false;
  QPoint last_mouse_;
  bool enlarge_host_ = false;
  QPixmap base_pixmap_;
  bool base_dirty_ = true;
};

}  // namespace gui
}  // namespace hs_calib
