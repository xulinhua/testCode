#include "hs_calib_suite/gui/widgets/image_view_widget.hpp"

#include <algorithm>
#include <cmath>

#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QTimer>
#include <QToolButton>
#include <QWheelEvent>

namespace hs_calib {
namespace gui {

namespace {
constexpr double kMinScale = 0.05;
constexpr double kMaxScale = 20.0;
constexpr double kZoomStep = 1.15;
}  // namespace

/// \brief 构造预览控件：像素提示、异步定时器、工具栏
ImageViewWidget::ImageViewWidget(QWidget *parent) : QWidget(parent) {
  setObjectName(QStringLiteral("ImageViewWidget"));
  setMinimumSize(320, 240);
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);

  pixel_popup_ = new QLabel(nullptr);
  pixel_popup_->setWindowFlags(Qt::ToolTip | Qt::FramelessWindowHint);
  pixel_popup_->setAttribute(Qt::WA_ShowWithoutActivating);
  pixel_popup_->setStyleSheet(
      QStringLiteral("QLabel { background:#f5f5dc; color:#111; border:1px solid #333; "
                     "border-radius:2px; padding:4px 6px; }"));
  pixel_popup_->hide();

  async_timer_ = new QTimer(this);
  async_timer_->setSingleShot(false);
  connect(async_timer_, &QTimer::timeout, this, [this]() { commit_pending(); });

  ensure_toolbar();
}

/// \brief 析构：释放像素提示浮层
ImageViewWidget::~ImageViewWidget() {
  delete pixel_popup_;
  pixel_popup_ = nullptr;
}

/// \brief 设置显示图像
void ImageViewWidget::set_image(const QImage &image) {
  apply_frame(image);
}

/// \brief 清空图像与待提交缓冲
void ImageViewWidget::clear_image() {
  image_ = QImage();
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_ = QImage();
    pending_dirty_.store(false, std::memory_order_release);
  }
  hide_pixel_info();
  update();
}

/// \brief 返回当前显示图像
QImage ImageViewWidget::current_image() const {
  return image_;
}

/// \brief 设置无图占位文案
void ImageViewWidget::set_placeholder(const QString &text) {
  placeholder_ = text.isEmpty() ? QStringLiteral("No Image") : text;
  update();
}

/// \brief 设置画布背景色
void ImageViewWidget::set_background_color(const QColor &color) {
  background_ = color;
  update();
}

/// \brief 开关异步刷新并设定间隔
void ImageViewWidget::set_async_refresh(bool enabled, int interval_ms) {
  async_ = enabled;
  if (async_) {
    async_timer_->start(std::max(1, interval_ms));
  } else {
    async_timer_->stop();
    commit_pending();
  }
}

/// \brief 显示/隐藏缩放工具栏
void ImageViewWidget::set_toolbar_visible(bool visible) {
  ensure_toolbar();
  if (toolbar_ != nullptr) {
    toolbar_->setVisible(visible);
  }
}

/// \brief 放大视图
void ImageViewWidget::zoom_in() {
  if (image_.isNull()) {
    return;
  }
  user_adjusted_ = true;
  scale_ = std::clamp(scale_ * kZoomStep, kMinScale, kMaxScale);
  update();
}

/// \brief 缩小视图
void ImageViewWidget::zoom_out() {
  if (image_.isNull()) {
    return;
  }
  user_adjusted_ = true;
  scale_ = std::clamp(scale_ / kZoomStep, kMinScale, kMaxScale);
  update();
}

/// \brief 适应窗口缩放并居中
void ImageViewWidget::fit_to_window() {
  if (image_.isNull() || width() <= 0 || height() <= 0) {
    return;
  }
  user_adjusted_ = false;
  const double sx = static_cast<double>(width()) / static_cast<double>(image_.width());
  const double sy = static_cast<double>(height()) / static_cast<double>(image_.height());
  scale_ = std::clamp(std::min(sx, sy), kMinScale, kMaxScale);
  pan_.setX((width() - image_.width() * scale_) * 0.5);
  pan_.setY((height() - image_.height() * scale_) * 0.5);
  update();
}

/// \brief 重置为 1:1 与零平移
void ImageViewWidget::reset_view() {
  user_adjusted_ = true;
  scale_ = 1.0;
  pan_ = QPointF(0.0, 0.0);
  update();
}

/// \brief 将当前帧保存到路径
bool ImageViewWidget::save_current_frame(const QString &path, QString *error) const {
  if (path.isEmpty()) {
    if (error) {
      *error = QStringLiteral("保存路径为空");
    }
    return false;
  }
  if (image_.isNull() || !image_.save(path)) {
    if (error) {
      *error = QStringLiteral("保存图像失败：%1").arg(path);
    }
    return false;
  }
  return true;
}

/// \brief 绘制背景与变换后的图像
void ImageViewWidget::paintEvent(QPaintEvent *event) {
  QWidget::paintEvent(event);
  QPainter painter(this);
  painter.fillRect(rect(), background_);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
  if (image_.isNull()) {
    painter.setPen(QColor(0x6a, 0x87, 0xa3));
    painter.drawText(rect(), Qt::AlignCenter, placeholder_);
    return;
  }
  painter.translate(pan_);
  painter.scale(scale_, scale_);
  painter.drawImage(QPointF(0.0, 0.0), image_);
}

/// \brief 滚轮以光标为中心缩放
void ImageViewWidget::wheelEvent(QWheelEvent *event) {
  if (image_.isNull()) {
    return;
  }
  const QPointF cursor = event->position();
  const QPointF img_before = (cursor - pan_) / scale_;
  if (event->angleDelta().y() > 0) {
    scale_ *= kZoomStep;
  } else {
    scale_ /= kZoomStep;
  }
  scale_ = std::clamp(scale_, kMinScale, kMaxScale);
  pan_ = cursor - img_before * scale_;
  user_adjusted_ = true;
  update();
  event->accept();
}

/// \brief 左键开始拖拽平移
void ImageViewWidget::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    dragging_ = true;
    last_mouse_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
  }
  QWidget::mousePressEvent(event);
}

/// \brief 拖拽平移；Ctrl 显示像素信息
void ImageViewWidget::mouseMoveEvent(QMouseEvent *event) {
  if (dragging_) {
    const QPoint delta = event->pos() - last_mouse_;
    pan_ += QPointF(delta.x(), delta.y());
    last_mouse_ = event->pos();
    user_adjusted_ = true;
    update();
  }
  if (event->modifiers().testFlag(Qt::ControlModifier)) {
    update_pixel_info(event->pos());
  } else {
    hide_pixel_info();
  }
  QWidget::mouseMoveEvent(event);
}

/// \brief 结束拖拽
void ImageViewWidget::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    dragging_ = false;
    setCursor(Qt::ArrowCursor);
  }
  QWidget::mouseReleaseEvent(event);
}

/// \brief 双击适应窗口
void ImageViewWidget::mouseDoubleClickEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    fit_to_window();
  }
  QWidget::mouseDoubleClickEvent(event);
}

/// \brief 调整工具栏；未手动缩放时重新适应
void ImageViewWidget::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
  update_toolbar_geometry();
  if (!image_.isNull() && !user_adjusted_) {
    fit_to_window();
  }
}

/// \brief 鼠标离开时隐藏像素提示
void ImageViewWidget::leaveEvent(QEvent *event) {
  hide_pixel_info();
  QWidget::leaveEvent(event);
}

/// \brief 懒创建缩放/保存工具栏
void ImageViewWidget::ensure_toolbar() {
  if (toolbar_ != nullptr) {
    return;
  }
  toolbar_ = new QWidget(this);
  toolbar_->setObjectName(QStringLiteral("ImageViewToolbar"));
  auto *lay = new QHBoxLayout(toolbar_);
  lay->setContentsMargins(6, 4, 6, 4);
  lay->setSpacing(4);
  auto make_btn = [this](const QString &text, const QString &tip, int min_w = 28) {
    auto *b = new QToolButton(toolbar_);
    b->setText(text);
    b->setToolTip(tip);
    b->setAutoRaise(true);
    b->setMinimumSize(min_w, 24);
    b->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    return b;
  };
  btn_zoom_in_ = make_btn(QStringLiteral("+"), QStringLiteral("放大"));
  btn_zoom_out_ = make_btn(QStringLiteral("−"), QStringLiteral("缩小"));
  btn_fit_ = make_btn(QStringLiteral("适应"), QStringLiteral("双击也可适应窗口"), 48);
  btn_save_ = make_btn(QStringLiteral("存"), QStringLiteral("保存当前帧"));
  lay->addWidget(btn_zoom_in_);
  lay->addWidget(btn_zoom_out_);
  lay->addWidget(btn_fit_);
  lay->addWidget(btn_save_);
  connect(btn_zoom_in_, &QToolButton::clicked, this, &ImageViewWidget::zoom_in);
  connect(btn_zoom_out_, &QToolButton::clicked, this, &ImageViewWidget::zoom_out);
  connect(btn_fit_, &QToolButton::clicked, this, &ImageViewWidget::fit_to_window);
  connect(btn_save_, &QToolButton::clicked, this, &ImageViewWidget::on_save_clicked);
  toolbar_->adjustSize();
  update_toolbar_geometry();
}

/// \brief 将工具栏放到左上角
void ImageViewWidget::update_toolbar_geometry() {
  if (toolbar_ == nullptr) {
    return;
  }
  toolbar_->adjustSize();
  toolbar_->move(8, 8);
  toolbar_->raise();
}

/// \brief 同步或异步提交新帧（保持原分辨率）
void ImageViewWidget::apply_frame(const QImage &image) {
  // 始终持有完整分辨率副本；缩放只作用于 paint，不重采样源图
  // —— 同步路径立即显示；异步路径写入 pending ——
  if (!async_) {
    const bool refit = image_.isNull() || image_.size() != image.size();
    image_ = image.copy();
    if (refit && !image_.isNull()) {
      user_adjusted_ = false;
      fit_to_window();
    } else {
      update();
    }
    return;
  }
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_ = image.copy();
  pending_dirty_.store(true, std::memory_order_release);
}

/// \brief 提交异步待显帧
void ImageViewWidget::commit_pending() {
  if (!pending_dirty_.load(std::memory_order_acquire)) {
    return;
  }
  bool refit = false;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    refit = image_.isNull() || image_.size() != pending_.size();
    image_ = pending_;
    pending_dirty_.store(false, std::memory_order_release);
  }
  if (refit && !image_.isNull()) {
    user_adjusted_ = false;
    fit_to_window();
  } else {
    update();
  }
}

/// \brief 控件坐标映射到图像像素
bool ImageViewWidget::map_to_pixel(const QPointF &widget_pos, QPoint *pixel) const {
  if (image_.isNull() || scale_ <= 0.0 || pixel == nullptr) {
    return false;
  }
  const QPointF img = (widget_pos - pan_) / scale_;
  const int x = static_cast<int>(std::floor(img.x()));
  const int y = static_cast<int>(std::floor(img.y()));
  if (x < 0 || y < 0 || x >= image_.width() || y >= image_.height()) {
    return false;
  }
  *pixel = QPoint(x, y);
  return true;
}

/// \brief 更新 Ctrl 取色浮层
void ImageViewWidget::update_pixel_info(const QPointF &widget_pos) {
  QPoint px;
  if (!map_to_pixel(widget_pos, &px) || pixel_popup_ == nullptr) {
    hide_pixel_info();
    return;
  }
  const QColor c = image_.pixelColor(px);
  const QString text = QStringLiteral("X:%1  Y:%2\nRGB  R:%3 G:%4 B:%5")
                           .arg(px.x())
                           .arg(px.y())
                           .arg(c.red())
                           .arg(c.green())
                           .arg(c.blue());
  pixel_popup_->setText(text);
  pixel_popup_->adjustSize();
  pixel_popup_->move(mapToGlobal(widget_pos.toPoint()) + QPoint(16, 0));
  pixel_popup_->show();
}

/// \brief 隐藏像素浮层
void ImageViewWidget::hide_pixel_info() {
  if (pixel_popup_ != nullptr) {
    pixel_popup_->hide();
  }
}

/// \brief 弹出保存对话框写当前帧
void ImageViewWidget::on_save_clicked() {
  if (image_.isNull()) {
    return;
  }
  const QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("保存当前帧"), QStringLiteral("calib_frame.png"),
      QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp)"));
  if (path.isEmpty()) {
    return;
  }
  QString err;
  save_current_frame(path, &err);
}

}  // namespace gui
}  // namespace hs_calib
