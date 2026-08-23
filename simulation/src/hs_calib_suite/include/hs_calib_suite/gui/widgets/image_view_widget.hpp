#pragma once

/// \file 标定预览图像控件（自 image_widget 精简拷贝：缩放/平移/像素取值/异步刷新/工具栏）
/// 不依赖 image_widget / log_system 包。
///
/// 缩放/平移仅改变绘制变换（paintEvent 的 scale_/pan_），不修改也不重采样
/// 内部保存的原图像素；检测/导出仍使用会话中的原分辨率帧。

#include <atomic>
#include <mutex>

#include <QColor>
#include <QImage>
#include <QPoint>
#include <QPointF>
#include <QString>
#include <QWidget>

class QLabel;
class QTimer;
class QToolButton;

namespace hs_calib {
namespace gui {

/// \brief 内置浮动工具栏样式
enum class ImageViewToolbarStyle {
  Hidden,          ///< 不显示（由外部标题栏接管）
  OverlayZoom,     ///< + / − / 适应 / 1:1
  OverlayZoomSave, ///< + / − / 适应 / 保存
};

/// \brief 可缩放平移的图像显示区（滚轮缩放、拖拽、双击适应、Ctrl 取像素）
class ImageViewWidget : public QWidget {
  Q_OBJECT

public:
  /// \brief 构造预览控件
  explicit ImageViewWidget(QWidget *parent = nullptr);
  /// \brief 析构：释放像素提示浮层
  ~ImageViewWidget() override;

  /// \brief 设置显示图像
  void set_image(const QImage &image);
  /// \brief 清空图像
  void clear_image();
  /// \brief 当前显示图像
  QImage current_image() const;
  /// \brief 是否有图像
  bool has_image() const { return !image_.isNull(); }

  /// \brief 设置无图占位文案
  void set_placeholder(const QString &text);
  /// \brief 设置画布背景色
  void set_background_color(const QColor &color);
  /// \brief 开关异步刷新并设定间隔
  void set_async_refresh(bool enabled, int interval_ms = 33);
  /// \brief 显示/隐藏缩放工具栏（兼容旧调用）
  void set_toolbar_visible(bool visible);
  /// \brief 设置内置浮动工具栏样式
  void set_toolbar_style(ImageViewToolbarStyle style);
  ImageViewToolbarStyle toolbar_style() const { return toolbar_style_; }

  /// \brief 放大视图
  void zoom_in();
  /// \brief 缩小视图
  void zoom_out();
  /// \brief 适应窗口
  void fit_to_window();
  /// \brief 重置为 1:1
  void reset_view();
  /// \brief 保存当前帧到路径
  bool save_current_frame(const QString &path, QString *error = nullptr) const;
  /// \brief 弹出保存对话框
  void prompt_save_image();

protected:
  void paintEvent(QPaintEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;
  void leaveEvent(QEvent *event) override;

private:
  void ensure_toolbar();
  void update_toolbar_buttons();
  void update_toolbar_geometry();
  void apply_frame(const QImage &image);
  void commit_pending();
  bool map_to_pixel(const QPointF &widget_pos, QPoint *pixel) const;
  void update_pixel_info(const QPointF &widget_pos);
  void hide_pixel_info();
  void on_save_clicked();

  QImage image_;
  QString placeholder_ = QStringLiteral("No Image");
  QColor background_{0x05, 0x09, 0x10};
  double scale_ = 1.0;
  QPointF pan_{0.0, 0.0};
  bool dragging_ = false;
  bool user_adjusted_ = false;
  QPoint last_mouse_{0, 0};

  bool async_ = false;
  QTimer *async_timer_ = nullptr;
  std::atomic<bool> pending_dirty_{false};
  mutable std::mutex pending_mutex_;
  QImage pending_;

  QLabel *pixel_popup_ = nullptr;
  ImageViewToolbarStyle toolbar_style_ = ImageViewToolbarStyle::OverlayZoomSave;
  QWidget *toolbar_ = nullptr;
  QToolButton *btn_zoom_in_ = nullptr;
  QToolButton *btn_zoom_out_ = nullptr;
  QToolButton *btn_fit_ = nullptr;
  QToolButton *btn_reset_ = nullptr;
};

}  // namespace gui
}  // namespace hs_calib
