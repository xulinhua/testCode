#pragma once

#include <vector>

#include <QColor>
#include <QString>
#include <QWidget>

namespace hs_calib {
namespace gui {

/// \brief 逐帧重投影 RMS 柱状图
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

signals:
  void bar_clicked(int view_index);

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  std::vector<Bar> bars_;
  int highlight_view_ = -1;
  double max_rms_ = 1.0;
};

/// \brief 像面覆盖 + 残差着色散点图
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

protected:
  void paintEvent(QPaintEvent *event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  static QColor color_for_error(float err_px, float max_err);

  int image_w_ = 0;
  int image_h_ = 0;
  std::vector<Point> points_;
  int filter_view_ = -1;
  float max_err_ = 2.0f;
};

}  // namespace gui
}  // namespace hs_calib
