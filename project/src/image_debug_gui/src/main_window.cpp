#include "image_debug_gui/main_window.hpp"

#include <algorithm>
#include <limits>

#include <QAction>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QImageReader>
#include <QPainter>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QProcess>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QGridLayout>
#include <QStyle>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWidget>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "image_debug_gui/image_view_widget.hpp"

namespace ImageDebug {

namespace {
QString classifyImageType(const QString &encoding, const cv::Mat &raw_image) {
  const QString lower = encoding.toLower();
  if (lower == "rgb8" || lower == "bgr8" || lower == "rgba8" || lower == "bgra8") {
    return "Color";
  }
  if (lower == "mono8" || lower == "8uc1") {
    return "Gray";
  }
  if (lower == "mono16" || lower == "16uc1" || lower == "32fc1" || lower == "64fc1") {
    return "Depth";
  }

  if (!raw_image.empty()) {
    if (raw_image.channels() == 1) {
      return "Gray/Depth";
    }
    if (raw_image.channels() >= 3) {
      return "Color";
    }
  }
  return "Unknown";
}

QIcon makeRecordIcon(bool recording) {
  QPixmap pix(16, 16);
  pix.fill(Qt::transparent);
  QPainter p(&pix);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(230, 60, 60));
  if (recording) {
    p.drawRoundedRect(3, 3, 10, 10, 2, 2);
  } else {
    p.drawEllipse(3, 3, 10, 10);
  }
  return QIcon(pix);
}
}  // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      node_(std::make_shared<rclcpp::Node>("image_debug_gui")),
      viewer_count_spin_(nullptr),
      viewer_grid_(nullptr),
      toolbox_action_(nullptr),
      sync_view_action_(nullptr),
      rosbag_process_(nullptr),
      rosbag_recording_(false),
      syncing_transform_(false),
      spin_timer_(nullptr),
      topic_timer_(nullptr) {
  setWindowTitle("image_debug_gui");
  resize(1200, 760);

  setupMenuBar();
  setupCentralWidget();
  setupRosTimers();
}

void MainWindow::setupMenuBar() {
  QMenu *file_menu = menuBar()->addMenu("File");
  QAction *open_image_action = file_menu->addAction("Open Image...");
  QAction *exit_action = file_menu->addAction("Exit");

  QMenu *view_menu = menuBar()->addMenu("View");
  QAction *fit_action = view_menu->addAction("Fit To Window");

  QMenu *tool_menu = menuBar()->addMenu("Tool");
  toolbox_action_ = tool_menu->addAction("ToolBox");
  toolbox_action_->setCheckable(true);
  toolbox_action_->setChecked(false);
  sync_view_action_ = tool_menu->addAction("同步查看");
  sync_view_action_->setCheckable(true);
  sync_view_action_->setChecked(false);

  QMenu *help_menu = menuBar()->addMenu("Help");
  QAction *about_action = help_menu->addAction("About");

  connect(open_image_action, &QAction::triggered, this, [this]() {
    const QString file_path = QFileDialog::getOpenFileName(
        this, "Open Image", QString(), "Images (*.png *.jpg *.jpeg *.bmp *.tiff)");
    if (file_path.isEmpty()) {
      return;
    }
    const QImage image = QImageReader(file_path).read();
    if (image.isNull()) {
      QMessageBox::warning(this, "Open Image", "Failed to load selected image.");
      return;
    }
    if (!image_views_.empty()) {
      image_views_[0]->setImageData(image, cv::Mat(), QString("file"));
      updateImageInfo(0, image.isGrayscale() ? "Gray(File)" : "Color(File)", image.width(),
                      image.height());
    }
  });

  connect(exit_action, &QAction::triggered, this, &QWidget::close);
  connect(fit_action, &QAction::triggered, this, [this]() {
    const int active_count = viewer_count_spin_ == nullptr ? 0 : viewer_count_spin_->value();
    for (int i = 0; i < active_count; ++i) {
      image_views_[i]->fitToWindow();
    }
  });
  connect(toolbox_action_, &QAction::toggled, this, &MainWindow::setToolBoxVisible);
  connect(sync_view_action_, &QAction::toggled, this, [this](bool enabled) {
    if (!enabled) {
      for (ImageViewWidget *view : image_views_) {
        if (view != nullptr) {
          view->hideTooltip();
          view->hideSyncInfo();
        }
      }
    }
    for (ImageViewWidget *view : image_views_) {
      if (view != nullptr) {
        view->setSyncModeEnabled(enabled);
      }
    }
  });
  connect(about_action, &QAction::triggered, this, &MainWindow::showAboutDialog);
}

void MainWindow::setupCentralWidget() {
  QWidget *central = new QWidget(this);
  QVBoxLayout *layout = new QVBoxLayout(central);
  QHBoxLayout *count_bar = new QHBoxLayout();
  QLabel *count_label = new QLabel("可同时查看图像数量:", central);
  viewer_count_spin_ = new QSpinBox(central);
  viewer_count_spin_->setRange(1, 4);
  viewer_count_spin_->setValue(1);
  viewer_count_spin_->setToolTip("设置同时显示的图像窗口数量（1~4）");
  count_bar->addWidget(count_label);
  count_bar->addWidget(viewer_count_spin_);
  count_bar->addStretch();

  QWidget *viewer_container = new QWidget(central);
  viewer_grid_ = new QGridLayout(viewer_container);
  viewer_grid_->setSpacing(8);
  viewer_grid_->setContentsMargins(0, 0, 0, 0);

  constexpr int kMaxViewers = 4;
  topic_combos_.reserve(kMaxViewers);
  image_info_labels_.reserve(kMaxViewers);
  image_views_.reserve(kMaxViewers);
  viewer_cards_.reserve(kMaxViewers);
  overlay_toolbars_.reserve(kMaxViewers);
  record_buttons_.reserve(kMaxViewers);
  image_subs_.resize(kMaxViewers);

  for (int i = 0; i < kMaxViewers; ++i) {
    QWidget *card = new QWidget(viewer_container);
    QVBoxLayout *card_layout = new QVBoxLayout(card);
    card_layout->setContentsMargins(0, 0, 0, 0);

    QHBoxLayout *topic_bar = new QHBoxLayout();
    QLabel *topic_label = new QLabel(QString("ROS2 Image Topic %1:").arg(i + 1), card);
    QComboBox *topic_combo = new QComboBox(card);
    topic_combo->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    topic_combo->setMinimumWidth(260);
    QPushButton *refresh_button = new QPushButton("Refresh", card);
    refresh_button->setToolTip("Refresh ROS2 image topics");
    topic_bar->addWidget(topic_label);
    topic_bar->addWidget(topic_combo, 1);
    topic_bar->addWidget(refresh_button);

    QLabel *info_label = new QLabel("图像类型: N/A | 图像尺寸: N/A", card);
    info_label->setStyleSheet("color: #555;");

    QWidget *image_panel = new QWidget(card);
    QGridLayout *image_panel_layout = new QGridLayout(image_panel);
    image_panel_layout->setContentsMargins(0, 0, 0, 0);
    image_panel_layout->setSpacing(0);
    ImageViewWidget *image_view = new ImageViewWidget(image_panel);

    QWidget *overlay_toolbar = new QWidget(image_panel);
    overlay_toolbar->setStyleSheet("background-color: rgba(20, 20, 20, 150); border-radius: 3px;");
    QHBoxLayout *overlay_layout = new QHBoxLayout(overlay_toolbar);
    overlay_layout->setContentsMargins(3, 2, 3, 2);
    overlay_layout->setSpacing(2);

    QToolButton *save_btn = new QToolButton(overlay_toolbar);
    QToolButton *zoom_in_btn = new QToolButton(overlay_toolbar);
    QToolButton *zoom_out_btn = new QToolButton(overlay_toolbar);
    QToolButton *record_btn = new QToolButton(overlay_toolbar);
    save_btn->setIcon(style()->standardIcon(QStyle::SP_DialogSaveButton));
    zoom_in_btn->setIcon(style()->standardIcon(QStyle::SP_ArrowUp));
    zoom_out_btn->setIcon(style()->standardIcon(QStyle::SP_ArrowDown));
    record_btn->setIcon(makeRecordIcon(false));
    record_btn->setToolTip("开始 rosbag 录制");
    save_btn->setToolTip("保存图像（所有可见窗口）");
    zoom_in_btn->setToolTip("放大（所有可见窗口）");
    zoom_out_btn->setToolTip("缩小（所有可见窗口）");
    for (QToolButton *btn : {save_btn, zoom_in_btn, zoom_out_btn, record_btn}) {
      btn->setAutoRaise(true);
      btn->setIconSize(QSize(14, 14));
      btn->setFixedSize(20, 20);
    }
    overlay_layout->addWidget(save_btn);
    overlay_layout->addWidget(zoom_in_btn);
    overlay_layout->addWidget(zoom_out_btn);
    overlay_layout->addWidget(record_btn);

    image_panel_layout->addWidget(image_view, 0, 0);
    image_panel_layout->addWidget(overlay_toolbar, 0, 0, Qt::AlignTop | Qt::AlignRight);

    card_layout->addLayout(topic_bar);
    card_layout->addWidget(info_label);
    card_layout->addWidget(image_panel, 1);

    topic_combos_.push_back(topic_combo);
    image_info_labels_.push_back(info_label);
    image_views_.push_back(image_view);
    viewer_cards_.push_back(card);
    overlay_toolbars_.push_back(overlay_toolbar);
    record_buttons_.push_back(record_btn);

    connect(topic_combo, &QComboBox::currentTextChanged, this,
            [this, i](const QString &text) { subscribeToTopic(i, text); });
    connect(image_view, &ImageViewWidget::hoverPixelChanged, this,
            [this, i](const QPoint &pixel, bool active) { handleSyncHover(i, pixel, active); });
    connect(image_view, &ImageViewWidget::viewTransformChanged, this,
            [this, i](double scale, const QPointF &pan) {
              handleSyncTransform(i, scale, pan);
            });
    connect(refresh_button, &QPushButton::clicked, this, &MainWindow::refreshTopicList);
    connect(save_btn, &QToolButton::clicked, this, &MainWindow::saveVisibleImages);
    connect(zoom_in_btn, &QToolButton::clicked, this, [this]() { zoomVisibleImages(true); });
    connect(zoom_out_btn, &QToolButton::clicked, this, [this]() { zoomVisibleImages(false); });
    connect(record_btn, &QToolButton::clicked, this, &MainWindow::toggleRosbagRecording);
  }

  layout->addLayout(count_bar);
  layout->addWidget(viewer_container, 1);
  setCentralWidget(central);

  connect(viewer_count_spin_, qOverload<int>(&QSpinBox::valueChanged), this,
          [this](int) { updateViewerVisibility(); });
  setToolBoxVisible(false);
  updateRecordButtons();
  updateViewerVisibility();
}

void MainWindow::setupRosTimers() {
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, [this]() { rclcpp::spin_some(node_); });
  spin_timer_->start(15);

  topic_timer_ = new QTimer(this);
  connect(topic_timer_, &QTimer::timeout, this, &MainWindow::refreshTopicList);
  topic_timer_->start(1000);
  refreshTopicList();
}

void MainWindow::setToolBoxVisible(bool visible) {
  for (QWidget *toolbar : overlay_toolbars_) {
    if (toolbar != nullptr) {
      toolbar->setVisible(visible);
    }
  }
}

void MainWindow::saveVisibleImages() {
  const int active_count = viewer_count_spin_ == nullptr ? 0 : viewer_count_spin_->value();
  if (active_count <= 0) {
    return;
  }
  const QString timestamp = QDateTime::currentDateTime().toString("yyyyMMddhhmmsszzz");
  int saved_count = 0;
  for (int i = 0; i < active_count && i < static_cast<int>(image_views_.size()); ++i) {
    const QImage image = image_views_[i]->currentImage();
    if (image.isNull()) {
      continue;
    }
    QString prefix = "image";
    if (i < static_cast<int>(image_info_labels_.size()) && image_info_labels_[i] != nullptr) {
      const QString info = image_info_labels_[i]->text().toLower();
      if (info.contains("color")) {
        prefix = "color";
      } else if (info.contains("depth")) {
        prefix = "depth";
      } else if (info.contains("gray")) {
        prefix = "gray";
      }
    }
    QString filename = QString("%1_%2.bmp").arg(prefix).arg(timestamp);
    if (active_count > 1) {
      filename = QString("%1_%2_v%3.bmp").arg(prefix).arg(timestamp).arg(i + 1);
    }

    const QString suggested_path = QDir::homePath() + "/" + filename;
    const QString file_path = QFileDialog::getSaveFileName(
        this, "Save Image", suggested_path,
        "BMP Image (*.bmp);;PNG Image (*.png);;JPEG Image (*.jpg *.jpeg)");
    if (file_path.isEmpty()) {
      continue;
    }

    if (image.save(file_path)) {
      ++saved_count;
    }
  }

  if (saved_count == 0) {
    QMessageBox::warning(this, "保存图像", "没有可保存的图像。");
  }
}

void MainWindow::zoomVisibleImages(bool zoom_in) {
  const int active_count = viewer_count_spin_ == nullptr ? 0 : viewer_count_spin_->value();
  for (int i = 0; i < active_count && i < static_cast<int>(image_views_.size()); ++i) {
    if (zoom_in) {
      image_views_[i]->zoomIn();
    } else {
      image_views_[i]->zoomOut();
    }
  }
}

void MainWindow::toggleRosbagRecording() {
  if (rosbag_recording_) {
    if (rosbag_process_ != nullptr) {
      rosbag_process_->terminate();
      if (!rosbag_process_->waitForFinished(2000)) {
        rosbag_process_->kill();
        rosbag_process_->waitForFinished(1000);
      }
      rosbag_process_->deleteLater();
      rosbag_process_ = nullptr;
    }
    rosbag_recording_ = false;
    updateRecordButtons();
    return;
  }

  QStringList topics;
  const int active_count = viewer_count_spin_ == nullptr ? 0 : viewer_count_spin_->value();
  for (int i = 0; i < active_count && i < static_cast<int>(topic_combos_.size()); ++i) {
    const QString t = topic_combos_[i]->currentText().trimmed();
    if (!t.isEmpty() && !topics.contains(t)) {
      topics << t;
    }
  }
  if (topics.isEmpty()) {
    QMessageBox::warning(this, "rosbag录制", "请先选择至少一个图像话题。");
    return;
  }

  const QString default_name =
      QDir::homePath() + "/rosbag_" + QDateTime::currentDateTime().toString("yyyyMMddhhmmsszzz");
  const QString out_path = QFileDialog::getSaveFileName(
      this, "选择rosbag保存路径（无扩展名）", default_name, "Rosbag Folder (*)");
  if (out_path.isEmpty()) {
    return;
  }

  rosbag_process_ = new QProcess(this);
  rosbag_process_->setProcessChannelMode(QProcess::MergedChannels);
  QStringList args;
  args << "bag" << "record" << "-o" << out_path;
  args.append(topics);
  rosbag_process_->start("ros2", args);
  if (!rosbag_process_->waitForStarted(1500)) {
    QMessageBox::warning(this, "rosbag录制", "启动ros2 bag record失败，请确认环境已source。");
    rosbag_process_->deleteLater();
    rosbag_process_ = nullptr;
    return;
  }

  rosbag_recording_ = true;
  connect(rosbag_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
          [this](int, QProcess::ExitStatus) {
            rosbag_recording_ = false;
            if (rosbag_process_ != nullptr) {
              rosbag_process_->deleteLater();
              rosbag_process_ = nullptr;
            }
            updateRecordButtons();
          });
  updateRecordButtons();
}

void MainWindow::updateRecordButtons() {
  for (QToolButton *btn : record_buttons_) {
    if (btn == nullptr) {
      continue;
    }
    if (rosbag_recording_) {
      btn->setIcon(makeRecordIcon(true));
      btn->setToolTip("录制中，点击停止");
    } else {
      btn->setIcon(makeRecordIcon(false));
      btn->setToolTip("开始 rosbag 录制");
    }
  }
}

void MainWindow::handleSyncHover(int source_index, const QPoint &pixel, bool active) {
  if (!active) {
    if (sync_view_action_ != nullptr && sync_view_action_->isChecked()) {
      for (ImageViewWidget *view : image_views_) {
        if (view != nullptr) {
          view->hideSyncInfo();
        }
      }
    }
    return;
  }

  if (sync_view_action_ == nullptr || !sync_view_action_->isChecked()) {
    return;
  }
  if (source_index < 0 || source_index >= static_cast<int>(image_views_.size())) {
    return;
  }
  const QSize ref_size = image_views_[source_index]->imageSize();
  if (ref_size.isEmpty()) {
    return;
  }

  const int active_count = viewer_count_spin_ == nullptr ? 0 : viewer_count_spin_->value();
  for (int i = 0; i < active_count && i < static_cast<int>(image_views_.size()); ++i) {
    ImageViewWidget *view = image_views_[i];
    if (view == nullptr) {
      continue;
    }
    if (view->imageSize() == ref_size) {
      view->showSyncInfoForPixel(pixel);
    } else {
      view->hideSyncInfo();
    }
  }
}

void MainWindow::handleSyncTransform(int source_index, double scale_factor,
                                     const QPointF &pan_offset) {
  if (syncing_transform_) {
    return;
  }
  if (sync_view_action_ == nullptr || !sync_view_action_->isChecked()) {
    return;
  }
  if (source_index < 0 || source_index >= static_cast<int>(image_views_.size())) {
    return;
  }
  const QSize ref_size = image_views_[source_index]->imageSize();
  if (ref_size.isEmpty()) {
    return;
  }

  const int active_count = viewer_count_spin_ == nullptr ? 0 : viewer_count_spin_->value();
  syncing_transform_ = true;
  for (int i = 0; i < active_count && i < static_cast<int>(image_views_.size()); ++i) {
    if (i == source_index) {
      continue;
    }
    ImageViewWidget *view = image_views_[i];
    if (view == nullptr || view->imageSize() != ref_size) {
      continue;
    }
    view->setViewTransform(scale_factor, pan_offset, false);
  }
  syncing_transform_ = false;
}

void MainWindow::relayoutViewerCards(int active_count) {
  if (viewer_grid_ == nullptr) {
    return;
  }

  while (QLayoutItem *item = viewer_grid_->takeAt(0)) {
    delete item;
  }

  auto place_card = [this](int index, int row, int col) {
    viewer_grid_->addWidget(viewer_cards_[index], row, col);
  };

  if (active_count <= 1) {
    if (!viewer_cards_.empty()) {
      place_card(0, 0, 0);
    }
    return;
  }

  if (active_count == 2) {
    place_card(0, 0, 0);
    place_card(1, 0, 1);
    return;
  }

  // 3~4 viewers: fixed 2x2 grid.
  for (int i = 0; i < active_count; ++i) {
    place_card(i, i / 2, i % 2);
  }
}

void MainWindow::updateViewerVisibility() {
  const int active_count = viewer_count_spin_ == nullptr ? 1 : viewer_count_spin_->value();
  relayoutViewerCards(active_count);
  for (int i = 0; i < static_cast<int>(viewer_cards_.size()); ++i) {
    viewer_cards_[i]->setVisible(i < active_count);
    if (i >= active_count) {
      image_subs_[i].reset();
      image_info_labels_[i]->setText("图像类型: N/A | 图像尺寸: N/A");
    }
  }
  refreshTopicList();
}

void MainWindow::refreshTopicList() {
  const auto topics = node_->get_topic_names_and_types();

  std::vector<QString> image_topics;
  image_topics.reserve(topics.size());
  for (const auto &entry : topics) {
    if (isImageTopicType(entry.second)) {
      image_topics.emplace_back(QString::fromStdString(entry.first));
    }
  }
  std::sort(image_topics.begin(), image_topics.end());

  const int active_count = viewer_count_spin_ == nullptr ? 1 : viewer_count_spin_->value();
  for (int i = 0; i < active_count && i < static_cast<int>(topic_combos_.size()); ++i) {
    QComboBox *combo = topic_combos_[i];
    const QString previous = combo->currentText();
    combo->blockSignals(true);
    combo->clear();
    for (const auto &topic : image_topics) {
      combo->addItem(topic);
    }

    const int index = combo->findText(previous);
    if (index >= 0) {
      combo->setCurrentIndex(index);
    } else if (!image_topics.empty()) {
      combo->setCurrentIndex(std::min(i, static_cast<int>(image_topics.size()) - 1));
    }
    combo->blockSignals(false);

    if (combo->currentText() != previous) {
      subscribeToTopic(i, combo->currentText());
    }
  }
}

void MainWindow::subscribeToTopic(int viewer_index, const QString &topic_name) {
  if (viewer_index < 0 || viewer_index >= static_cast<int>(image_subs_.size())) {
    return;
  }

  image_subs_[viewer_index].reset();
  if (topic_name.isEmpty() || viewer_index >= static_cast<int>(image_views_.size())) {
    updateImageInfo(viewer_index, "N/A", 0, 0);
    return;
  }

  image_subs_[viewer_index] = node_->create_subscription<sensor_msgs::msg::Image>(
      topic_name.toStdString(), rclcpp::SensorDataQoS(),
      [this, viewer_index](const sensor_msgs::msg::Image::SharedPtr msg) {
        QImage display;
        cv::Mat raw;
        QString encoding;
        if (!rosImageToDisplayAndRaw(msg, &display, &raw, &encoding) || display.isNull()) {
          return;
        }
        image_views_[viewer_index]->setImageData(display, raw, encoding);
        updateImageInfo(viewer_index, classifyImageType(encoding, raw), display.width(),
                        display.height());
      });
}

void MainWindow::updateImageInfo(int viewer_index, const QString &image_type, int width,
                                 int height) {
  if (viewer_index < 0 || viewer_index >= static_cast<int>(image_info_labels_.size())) {
    return;
  }
  if (width > 0 && height > 0) {
    image_info_labels_[viewer_index]->setText(
        QString("图像类型: %1 | 图像尺寸: %2 x %3").arg(image_type).arg(width).arg(height));
  } else {
    image_info_labels_[viewer_index]->setText(
        QString("图像类型: %1 | 图像尺寸: N/A").arg(image_type));
  }
}

void MainWindow::showAboutDialog() {
  QMessageBox::information(
      this, "About image_debug_gui",
      "image_debug_gui\n\n"
      "- Qt + C++17 desktop UI\n"
      "- Menu-bar style workflow\n"
      "- ROS2 image topic selection\n"
      "- Color/depth topic visualization\n"
      "- Ctrl + mouse move to inspect XY/value");
}

bool MainWindow::isImageTopicType(const std::vector<std::string> &types) {
  return std::find(types.begin(), types.end(), "sensor_msgs/msg/Image") != types.end();
}

bool MainWindow::rosImageToDisplayAndRaw(const sensor_msgs::msg::Image::SharedPtr &msg,
                                         QImage *display_image, cv::Mat *raw_image,
                                         QString *encoding) {
  if (display_image == nullptr || raw_image == nullptr || encoding == nullptr) {
    return false;
  }

  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
    *raw_image = cv_ptr->image.clone();
    *encoding = QString::fromStdString(msg->encoding);

    cv::Mat rgb;
    if (msg->encoding == "rgb8") {
      rgb = cv_ptr->image.clone();
    } else if (msg->encoding == "bgr8") {
      cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
    } else if (msg->encoding == "bgra8") {
      cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGRA2RGB);
    } else if (msg->encoding == "rgba8") {
      cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_RGBA2RGB);
    } else if (msg->encoding == "mono8" || msg->encoding == "8UC1") {
      cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_GRAY2RGB);
    } else if (msg->encoding == "mono16" || msg->encoding == "16UC1" ||
               msg->encoding == "32FC1" || msg->encoding == "64FC1") {
      cv::Mat normalized8;
      cv::Mat finite_mask = (cv_ptr->image == cv_ptr->image);
      double min_value = 0.0;
      double max_value = 0.0;
      cv::minMaxLoc(cv_ptr->image, &min_value, &max_value, nullptr, nullptr, finite_mask);
      if (max_value <= min_value) {
        max_value = min_value + 1.0;
      }
      cv_ptr->image.convertTo(
          normalized8, CV_8U, 255.0 / (max_value - min_value), -min_value * 255.0 / (max_value - min_value));
      cv::cvtColor(normalized8, rgb, cv::COLOR_GRAY2RGB);
    } else {
      cv_bridge::CvImageConstPtr rgb_ptr = cv_bridge::toCvCopy(msg, "rgb8");
      rgb = rgb_ptr->image;
    }

    if (rgb.empty()) {
      return false;
    }

    QImage image(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
    *display_image = image.copy();
    return true;
  } catch (const std::exception &e) {
    RCLCPP_WARN(rclcpp::get_logger("image_debug_gui"), "Failed to convert image: %s", e.what());
    return false;
  }
}

}  // namespace ImageDebug
