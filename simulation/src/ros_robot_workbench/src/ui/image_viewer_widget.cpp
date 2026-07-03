#include "ros_robot_workbench/ui/image_viewer_widget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QFont>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QRunnable>
#include <QSpinBox>
#include <QThreadPool>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QProcess>
#include <QDir>

#include <atomic>
#include <cstdint>
#include <cmath>
#include <set>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ros_robot_workbench/module/image_viewer_module.h"
#include "ros_robot_workbench/ui/shared_ui_executor.hpp"
#include "ros_robot_workbench/ui/zoomable_image_widget.h"

namespace ros_robot_workbench::ui
{
namespace
{

QImage BuildPlaceholderImage(const QString & topic, int index)
{
  QImage image(1280, 720, QImage::Format_RGB888);
  image.fill(QColor(32, 36, 42));
  QPainter painter(&image);
  painter.setPen(QColor(230, 236, 245));
  QFont title_font = painter.font();
  title_font.setPointSize(15);
  title_font.setBold(true);
  painter.setFont(title_font);
  painter.drawText(QRect(20, 20, 760, 60), Qt::AlignLeft | Qt::AlignVCenter, QString("图像查看 %1").arg(index + 1));
  QFont body_font = painter.font();
  body_font.setPointSize(11);
  body_font.setBold(false);
  painter.setFont(body_font);
  painter.drawText(
    QRect(20, 90, 760, 260),
    Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
    QString("当前话题: %1\n更新时间: %2")
      .arg(topic.isEmpty() ? "(未选择)" : topic)
      .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")));
  return image;
}

void PopulateTopicCombo(QComboBox * combo, const std::vector<QString> & topics, const QString & preferred)
{
  combo->clear();
  if (topics.empty()) {
    combo->addItem("/camera/image_raw");
    return;
  }
  for (const auto & topic : topics) {
    combo->addItem(topic);
  }
  int idx = combo->findText(preferred);
  if (idx < 0) idx = 0;
  combo->setCurrentIndex(idx);
}

}  // namespace

ImageViewerWidget::ImageViewerWidget(QWidget * parent)
: QWidget(parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(6, 4, 6, 4);
  root->setSpacing(4);
  root->setStretch(0, 0);  // 标题
  root->setStretch(1, 0);  // 顶部工具条
  root->setStretch(2, 1);  // 图像区优先拉伸
  root->setStretch(3, 0);  // 日志区尽量保持最小

  auto * title = new QLabel("图像查看");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  auto * top_row = new QHBoxLayout();
  auto * count_label = new QLabel("可同时查看图像数量:");
  auto * viewer_count_spin = new QSpinBox();
  viewer_count_spin->setRange(1, 4);
  viewer_count_spin->setValue(1);
  auto * refresh_btn = new QPushButton("刷新全部话题");
  top_row->addWidget(count_label);
  top_row->addWidget(viewer_count_spin);
  top_row->addSpacing(16);
  auto * sync_view_check = new QCheckBox("同步查看");
  auto * enable_toolbar_check = new QCheckBox("启用工具栏");
  top_row->addWidget(sync_view_check);
  top_row->addWidget(enable_toolbar_check);
  top_row->addStretch();
  top_row->addWidget(refresh_btn);
  root->addLayout(top_row);

  auto * viewers_container = new QWidget(this);
  viewers_container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  auto * viewers_grid = new QGridLayout(viewers_container);
  viewers_grid->setSpacing(6);
  viewers_grid->setContentsMargins(0, 0, 0, 0);
  root->addWidget(viewers_container, 1);

  auto * log_box = new QTextEdit();
  log_box->setReadOnly(true);
  log_box->setMinimumHeight(72);
  log_box->setMaximumHeight(120);
  log_box->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  root->addWidget(log_box, 0);

  auto append_log = [=](const QString & text) {
    log_box->append(QString("[%1] %2").arg(QDateTime::currentDateTime().toString("hh:mm:ss")).arg(text));
  };

  constexpr int kMaxViewers = 4;
  std::vector<QWidget *> cards;
  std::vector<QComboBox *> topic_combos;
  std::vector<ZoomableImageWidget *> image_views;
  cards.reserve(kMaxViewers);
  topic_combos.reserve(kMaxViewers);
  image_views.reserve(kMaxViewers);

  auto image_subs = std::make_shared<std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>>(kMaxViewers);
  auto latest_msgs = std::make_shared<std::vector<sensor_msgs::msg::Image::SharedPtr>>(kMaxViewers);
  auto active_flag = std::make_shared<std::atomic<bool>>(false);
  auto rosbag_recording = std::make_shared<std::atomic<bool>>(false);
  auto rosbag_process = std::make_shared<QProcess *>(nullptr);
  auto refresh_pool = std::make_shared<QThreadPool>();
  refresh_pool->setMaxThreadCount(2);
  const auto stamp = static_cast<unsigned long long>(QDateTime::currentMSecsSinceEpoch());
  auto image_node = rclcpp::Node::make_shared("ros_robot_workbench_image_viewer_" + std::to_string(stamp));
  SharedUiExecutor::instance().add_node(image_node);
  QObject::connect(this, &QObject::destroyed, [=]() {
    if (*rosbag_process != nullptr) {
      (*rosbag_process)->terminate();
      (*rosbag_process)->waitForFinished(1000);
      delete *rosbag_process;
      *rosbag_process = nullptr;
    }
    refresh_pool->clear();
    refresh_pool->waitForDone();
    SharedUiExecutor::instance().remove_node(image_node);
  });

  for (int i = 0; i < kMaxViewers; ++i) {
    auto * card = new QWidget(viewers_container);
    card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto * card_layout = new QVBoxLayout(card);
    card_layout->setContentsMargins(0, 0, 0, 0);
    auto * topic_row = new QHBoxLayout();
    topic_row->addWidget(new QLabel(QString("ROS2 Image Topic %1:").arg(i + 1)));
    auto * topic_combo = new QComboBox();
    topic_combo->setSizeAdjustPolicy(QComboBox::AdjustToContentsOnFirstShow);
    topic_row->addWidget(topic_combo, 1);
    card_layout->addLayout(topic_row);
    auto * image_view = new ZoomableImageWidget();
    image_view->setMinimumHeight(200);
    image_view->setStyleSheet("background:#20242a; border:1px solid #3a4048;");
    image_view->SetImage(BuildPlaceholderImage(topic_combo->currentText(), i));
    card_layout->addWidget(image_view, 1);
    viewers_grid->addWidget(card, i / 2, i % 2);
    cards.push_back(card);
    topic_combos.push_back(topic_combo);
    image_views.push_back(image_view);

    image_view->SetPixelInfoProvider([=](const QPoint & pixel, const QColor & color) -> QString {
      const auto msg = (*latest_msgs)[i];
      if (!msg) {
        return QString("X:%1  Y:%2\nGray: 0").arg(pixel.x()).arg(pixel.y());
      }
      const QString enc = QString::fromStdString(msg->encoding).toLower();
      const int x = pixel.x();
      const int y = pixel.y();
      if (x < 0 || y < 0 || x >= static_cast<int>(msg->width) || y >= static_cast<int>(msg->height)) {
        return QString("X:%1  Y:%2\nOut of range").arg(x).arg(y);
      }
      if ((enc == "mono16" || enc == "16uc1") && msg->step >= msg->width * 2) {
        const auto * row = reinterpret_cast<const uint16_t *>(msg->data.data() + y * msg->step);
        const uint16_t v = row[x];
        if (v == 0) {
          return QString("X:%1  Y:%2\nGray: %3\nDepth: invalid").arg(x).arg(y).arg(color.red());
        }
        return QString("X:%1  Y:%2\nGray: %3\nDepth: %4 mm")
          .arg(x).arg(y).arg(color.red()).arg(v);
      }
      if (enc == "32fc1" && msg->step >= msg->width * 4) {
        const auto * row = reinterpret_cast<const float *>(msg->data.data() + y * msg->step);
        const float v = row[x];
        if (!std::isfinite(v) || v <= 0.0f) {
          return QString("X:%1  Y:%2\nGray: %3\nDepth: invalid").arg(x).arg(y).arg(color.red());
        }
        return QString("X:%1  Y:%2\nGray: %3\nDepth: %4 m (%5 mm)")
          .arg(x).arg(y).arg(color.red())
          .arg(v, 0, 'f', 4)
          .arg(v * 1000.0f, 0, 'f', 1);
      }
      if (color.red() == color.green() && color.red() == color.blue()) {
        return QString("X:%1  Y:%2\nGray: %3").arg(x).arg(y).arg(color.red());
      }
      return QString("X:%1  Y:%2\nR:%3 G:%4 B:%5")
        .arg(x).arg(y).arg(color.red()).arg(color.green()).arg(color.blue());
    });
  }

  auto syncing_transform = std::make_shared<bool>(false);
  auto update_record_buttons = [=]() {
    for (int i = 0; i < kMaxViewers; ++i) {
      image_views[i]->SetRecordActive(rosbag_recording->load());
    }
  };
  auto save_view_image = [=](int index) {
    if (index < 0 || index >= kMaxViewers) return;
    const QImage image = image_views[index]->CurrentImage();
    if (image.isNull()) {
      QMessageBox::warning(this, "保存图像", "当前窗口没有可保存图像。");
      return;
    }
    const QString ts = QDateTime::currentDateTime().toString("yyyyMMddhhmmsszzz");
    const QString suggested = QDir::homePath() + QString("/image_view_%1_%2.png").arg(index + 1).arg(ts);
    const QString path = QFileDialog::getSaveFileName(
      this, "保存图像", suggested, "PNG (*.png);;BMP (*.bmp);;JPEG (*.jpg *.jpeg)");
    if (path.isEmpty()) return;
    if (!image.save(path)) {
      QMessageBox::warning(this, "保存图像", "图像保存失败。");
      return;
    }
    append_log(QString("窗口%1 图像已保存: %2").arg(index + 1).arg(path));
  };
  auto toggle_rosbag_record = [=]() {
    if (rosbag_recording->load()) {
      if (*rosbag_process != nullptr) {
        (*rosbag_process)->terminate();
        if (!(*rosbag_process)->waitForFinished(2000)) {
          (*rosbag_process)->kill();
          (*rosbag_process)->waitForFinished(1000);
        }
        delete *rosbag_process;
        *rosbag_process = nullptr;
      }
      rosbag_recording->store(false);
      update_record_buttons();
      append_log("rosbag 录制已停止。");
      return;
    }
    std::set<QString> topics_set;
    const int active = viewer_count_spin->value();
    for (int i = 0; i < active; ++i) {
      const QString topic = topic_combos[i]->currentText().trimmed();
      if (!topic.isEmpty()) {
        topics_set.insert(topic);
      }
    }
    if (topics_set.empty()) {
      QMessageBox::warning(this, "rosbag录制", "请先选择至少一个图像话题。");
      return;
    }
    QStringList topics;
    for (const auto & t : topics_set) topics << t;
    const QString default_name = QDir::homePath() + "/rosbag_" + QDateTime::currentDateTime().toString("yyyyMMddhhmmsszzz");
    const QString out_path = QFileDialog::getSaveFileName(this, "选择rosbag保存路径（无扩展名）", default_name, "Rosbag Folder (*)");
    if (out_path.isEmpty()) return;

    *rosbag_process = new QProcess(this);
    (*rosbag_process)->setProcessChannelMode(QProcess::MergedChannels);
    QStringList args;
    args << "bag" << "record" << "-o" << out_path;
    args.append(topics);
    (*rosbag_process)->start("ros2", args);
    if (!(*rosbag_process)->waitForStarted(1500)) {
      QMessageBox::warning(this, "rosbag录制", "启动 ros2 bag record 失败，请确认环境已source。");
      delete *rosbag_process;
      *rosbag_process = nullptr;
      return;
    }
    rosbag_recording->store(true);
    update_record_buttons();
    append_log(QString("rosbag 录制开始，话题: %1").arg(topics.join(", ")));
  };
  for (int i = 0; i < kMaxViewers; ++i) {
    image_views[i]->SetSaveImageCallback([=]() { save_view_image(i); });
    image_views[i]->SetRecordBagCallback([=]() { toggle_rosbag_record(); });
  }
  auto refresh_topics = [=]() {
    const auto topics = ListOnlineImageTopicsForViewer();
    for (int i = 0; i < kMaxViewers; ++i) {
      const QString preferred = topic_combos[i]->currentText();
      PopulateTopicCombo(topic_combos[i], topics, preferred);
    }
    append_log(QString("在线图像话题刷新完成，共 %1 个").arg(static_cast<int>(topics.size())));
  };

  auto subscribe_viewer = [=](int index) {
    if (index < 0 || index >= kMaxViewers) {
      return;
    }
    (*image_subs)[index].reset();
    const QString topic = topic_combos[index]->currentText().trimmed();
    if (topic.isEmpty()) {
      image_views[index]->SetImage(BuildPlaceholderImage(topic, index));
      return;
    }
    (*image_subs)[index] = image_node->create_subscription<sensor_msgs::msg::Image>(
      topic.toStdString(), rclcpp::SensorDataQoS(),
      [=](const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!active_flag->load()) {
          return;
        }
        auto * runnable = QRunnable::create([=]() {
          QImage image;
          if (!ConvertViewerRosImageToQImage(*msg, &image)) {
            return;
          }
          if (!active_flag->load()) {
            return;
          }
          QMetaObject::invokeMethod(this, [=]() {
            if (!active_flag->load()) {
              return;
            }
            (*latest_msgs)[index] = msg;
            image_views[index]->SetImage(image);
          }, Qt::QueuedConnection);
        });
        runnable->setAutoDelete(true);
        refresh_pool->start(runnable);
      });
    append_log(QString("窗口%1 已订阅: %2").arg(index + 1).arg(topic));
  };

  auto apply_visibility = [=]() {
    const int active = viewer_count_spin->value();

    // 先从网格中移除旧布局项，再按当前窗口数量重排
    while (viewers_grid->count() > 0) {
      viewers_grid->takeAt(0);
    }

    // 重置网格行列拉伸，避免 1/2 视图时被固定 2x2 占比稀释
    for (int r = 0; r < 3; ++r) {
      viewers_grid->setRowStretch(r, 0);
    }
    for (int c = 0; c < 3; ++c) {
      viewers_grid->setColumnStretch(c, 0);
    }

    if (active == 1) {
      viewers_grid->addWidget(cards[0], 0, 0);
      viewers_grid->setRowStretch(0, 1);
      viewers_grid->setColumnStretch(0, 1);
    } else if (active == 2) {
      viewers_grid->addWidget(cards[0], 0, 0);
      viewers_grid->addWidget(cards[1], 0, 1);
      viewers_grid->setRowStretch(0, 1);
      viewers_grid->setColumnStretch(0, 1);
      viewers_grid->setColumnStretch(1, 1);
    } else {
      // 3/4 个时采用 2x2，保证整体拉伸比例稳定
      for (int i = 0; i < active; ++i) {
        viewers_grid->addWidget(cards[i], i / 2, i % 2);
      }
      viewers_grid->setRowStretch(0, 1);
      viewers_grid->setRowStretch(1, 1);
      viewers_grid->setColumnStretch(0, 1);
      viewers_grid->setColumnStretch(1, 1);
    }

    for (int i = 0; i < kMaxViewers; ++i) {
      cards[i]->setVisible(i < active);
      if (i >= active) {
        (*image_subs)[i].reset();
      }
    }
  };

  for (int i = 0; i < kMaxViewers; ++i) {
    QObject::connect(topic_combos[i], &QComboBox::currentTextChanged, [=](const QString &) { subscribe_viewer(i); });

    image_views[i]->SetViewTransformChangedCallback([=](double scale, const QPointF & pan) {
      if (!sync_view_check->isChecked() || *syncing_transform) {
        return;
      }
      const QSize src_size = image_views[i]->ImageSize();
      if (src_size.isEmpty()) {
        return;
      }
      *syncing_transform = true;
      for (int j = 0; j < kMaxViewers; ++j) {
        if (j == i || !cards[j]->isVisible()) {
          continue;
        }
        if (image_views[j]->ImageSize() == src_size) {
          image_views[j]->SetViewTransform(scale, pan, false);
        }
      }
      *syncing_transform = false;
    });

    image_views[i]->SetHoverPixelCallback([=](const QPoint & pixel, bool active) {
      if (!sync_view_check->isChecked()) {
        return;
      }
      if (!active) {
        for (int j = 0; j < kMaxViewers; ++j) {
          if (cards[j]->isVisible()) {
            image_views[j]->HideSyncInfo();
          }
        }
        return;
      }
      const QSize src_size = image_views[i]->ImageSize();
      for (int j = 0; j < kMaxViewers; ++j) {
        if (!cards[j]->isVisible()) {
          continue;
        }
        if (image_views[j]->ImageSize() == src_size) {
          image_views[j]->ShowSyncInfoForPixel(pixel);
        } else {
          image_views[j]->HideSyncInfo();
        }
      }
    });
  }

  QObject::connect(refresh_btn, &QPushButton::clicked, [=]() {
    refresh_topics();
    const int active = viewer_count_spin->value();
    for (int i = 0; i < active; ++i) {
      subscribe_viewer(i);
    }
  });
  QObject::connect(viewer_count_spin, qOverload<int>(&QSpinBox::valueChanged), [=](int) { apply_visibility(); });
  QObject::connect(sync_view_check, &QCheckBox::toggled, [=](bool checked) {
    for (int i = 0; i < kMaxViewers; ++i) {
      image_views[i]->SetSyncViewEnabled(checked);
      if (!checked) {
        image_views[i]->HideSyncInfo();
      }
    }
    append_log(checked ? "同步查看已启用" : "同步查看已关闭");
  });
  QObject::connect(enable_toolbar_check, &QCheckBox::toggled, [=](bool checked) {
    for (int i = 0; i < kMaxViewers; ++i) {
      image_views[i]->SetToolbarEnabled(checked);
    }
    append_log(checked ? "图像工具栏已启用" : "图像工具栏已关闭");
  });

  refresh_topics();
  update_record_buttons();
  apply_visibility();
  for (int i = 0; i < viewer_count_spin->value(); ++i) {
    subscribe_viewer(i);
  }
  set_active_ = [active_flag](bool active) { active_flag->store(active); };
  SetActive(false);
}

void ImageViewerWidget::SetActive(bool active)
{
  if (set_active_) {
    set_active_(active);
  }
}

}  // namespace ros_robot_workbench::ui
