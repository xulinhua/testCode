#include "ros_robot_assist_tools/ui/intrinsic_calibration_widget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPainter>
#include <QPointer>
#include <QPushButton>
#include <QRadioButton>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ros_robot_assist_tools/module/calibration_module.h"
#include "ros_robot_assist_tools/module/intrinsic_calibration_module.h"
#include "ros_robot_assist_tools/ui/shared_refresh_pool.h"
#include "ros_robot_assist_tools/ui/shared_ui_executor.hpp"
#include "ros_robot_assist_tools/ui/zoomable_image_widget.h"

namespace ros_robot_assist_tools::ui
{
namespace
{

void SetFormLeftAligned(QFormLayout * form)
{
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  form->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
}

void PopulateTopicCombo(QComboBox * combo, const std::vector<QString> & topics, const QString & preferred)
{
  combo->clear();
  if (topics.empty()) {
    combo->addItem("/camera/image_raw");
    return;
  }
  for (const auto & t : topics) {
    combo->addItem(t);
  }
  int idx = combo->findText(preferred);
  if (idx < 0) idx = 0;
  combo->setCurrentIndex(idx);
}

QImage BuildPreviewPlaceholder(const QString & title, const QString & topic)
{
  QImage img(1280, 720, QImage::Format_RGB888);
  img.fill(QColor(32, 36, 42));
  QPainter painter(&img);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QColor(230, 236, 245));
  QFont title_font = painter.font();
  title_font.setPointSize(16);
  title_font.setBold(true);
  painter.setFont(title_font);
  painter.drawText(QRect(20, 20, 680, 60), Qt::AlignLeft | Qt::AlignVCenter, title);
  QFont body_font = painter.font();
  body_font.setPointSize(11);
  body_font.setBold(false);
  painter.setFont(body_font);
  painter.drawText(
    QRect(20, 92, 680, 280),
    Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
    QString("当前话题: %1\n更新时间: %2")
      .arg(topic.isEmpty() ? "(未选择)" : topic)
      .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")));
  painter.end();
  return img;
}

bool ConvertRosImageToQImage(const sensor_msgs::msg::Image & msg, QImage * out)
{
  if (!out || msg.width == 0 || msg.height == 0 || msg.data.empty()) {
    return false;
  }

  const int width = static_cast<int>(msg.width);
  const int height = static_cast<int>(msg.height);
  const QString encoding = QString::fromStdString(msg.encoding).toLower();

  if (encoding == "rgb8") {
    QImage img(msg.data.data(), width, height, static_cast<int>(msg.step), QImage::Format_RGB888);
    *out = img.copy();
    return true;
  }
  if (encoding == "bgr8") {
    QImage img(msg.data.data(), width, height, static_cast<int>(msg.step), QImage::Format_BGR888);
    *out = img.copy();
    return true;
  }
  if (encoding == "mono8") {
    QImage img(msg.data.data(), width, height, static_cast<int>(msg.step), QImage::Format_Grayscale8);
    *out = img.copy();
    return true;
  }
  return false;
}

}  // namespace

IntrinsicCalibrationWidget::IntrinsicCalibrationWidget(QWidget * parent)
: QWidget(parent)
{
  QVBoxLayout * root = new QVBoxLayout(this);
  root->setContentsMargins(8, 6, 8, 6);
  root->setSpacing(6);

  QLabel * title = new QLabel("内参标定");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QComboBox * board_type = new QComboBox();
  board_type->addItems({"Chessboard", "Charuco", "Aruco GridBoard"});
  QComboBox * distortion_model = new QComboBox();
  distortion_model->addItems({"plumb_bob", "fisheye"});
  distortion_model->setCurrentText("fisheye");
  QComboBox * run_mode = new QComboBox();
  run_mode->addItems({"实时模式", "离线模式"});

  QGroupBox * param_group = new QGroupBox("参数");
  QVBoxLayout * param_layout = new QVBoxLayout(param_group);
  QFormLayout * common_form = new QFormLayout();
  SetFormLeftAligned(common_form);
  common_form->addRow("标定板类型:", board_type);
  common_form->addRow("畸变模型:", distortion_model);
  common_form->addRow("运行模式:", run_mode);
  param_layout->addLayout(common_form);
  root->addWidget(param_group, 0);

  QGroupBox * calib_param_group = new QGroupBox("标定参数");
  QVBoxLayout * calib_param_layout = new QVBoxLayout(calib_param_group);

  QGroupBox * validation_param_group = new QGroupBox("验证参数");
  QVBoxLayout * validation_param_layout = new QVBoxLayout(validation_param_group);

  QGroupBox * operation_group = new QGroupBox("操作");
  QVBoxLayout * operation_layout = new QVBoxLayout(operation_group);

  root->addWidget(calib_param_group, 0);
  root->addWidget(validation_param_group, 0);
  root->addWidget(operation_group, 0);

  QComboBox * rt_image_topic = nullptr;
  QComboBox * rt_camera_info_topic = nullptr;
  QLineEdit * off_bag_dir = nullptr;
  QComboBox * off_image_topic = nullptr;
  QComboBox * off_camera_info_topic = nullptr;

  QFormLayout * rt_form = new QFormLayout();
  SetFormLeftAligned(rt_form);
  rt_image_topic = new QComboBox();
  rt_image_topic->setEditable(false);
  rt_image_topic->setSizeAdjustPolicy(QComboBox::AdjustToContentsOnFirstShow);
  QPushButton * refresh_live_topics = new QPushButton("刷新");
  refresh_live_topics->setMinimumWidth(150);
  QHBoxLayout * rt_topic_row = new QHBoxLayout();
  rt_topic_row->addWidget(rt_image_topic, 1);
  rt_topic_row->addWidget(refresh_live_topics);
  rt_form->addRow("图像话题:", rt_topic_row);
  for (const auto & def : IntrinsicOnlineFieldDefs()) {
    rt_form->addRow(def.first + ":", new QLineEdit(def.second));
  }
  QCheckBox * auto_refresh_live_topics = new QCheckBox("自动刷新在线图像话题(2s)");
  auto_refresh_live_topics->setChecked(false);
  rt_form->addRow("", auto_refresh_live_topics);
  QWidget * rt_calib_panel = new QWidget();
  QVBoxLayout * rt_calib_panel_layout = new QVBoxLayout(rt_calib_panel);
  rt_calib_panel_layout->setContentsMargins(0, 0, 0, 0);
  rt_calib_panel_layout->addLayout(rt_form);
  calib_param_layout->addWidget(rt_calib_panel);

  QWidget * rt_validation_widget = new QWidget();
  QFormLayout * rt_validation_form = new QFormLayout(rt_validation_widget);
  SetFormLeftAligned(rt_validation_form);
  rt_camera_info_topic = new QComboBox();
  rt_camera_info_topic->setEditable(true);
  PopulateTopicCombo(rt_camera_info_topic, {}, "/camera/camera_info");
  rt_validation_form->addRow("camera_info话题:", rt_camera_info_topic);
  QWidget * rt_validation_panel = new QWidget();
  QVBoxLayout * rt_validation_panel_layout = new QVBoxLayout(rt_validation_panel);
  rt_validation_panel_layout->setContentsMargins(0, 0, 0, 0);
  rt_validation_panel_layout->addWidget(rt_validation_widget);
  validation_param_layout->addWidget(rt_validation_panel);
  QRadioButton * quality_hint = new QRadioButton("显示采图质量提示");
  quality_hint->setChecked(true);
  QWidget * rt_operation_panel = new QWidget();
  QVBoxLayout * rt_operation_panel_layout = new QVBoxLayout(rt_operation_panel);
  rt_operation_panel_layout->setContentsMargins(0, 0, 0, 0);
  rt_operation_panel_layout->addWidget(quality_hint);
  QHBoxLayout * rt_btn_row = new QHBoxLayout();
  QPushButton * manual_capture = new QPushButton("手动采图");
  QPushButton * auto_capture = new QPushButton("自动采图");
  QPushButton * run_btn = new QPushButton("执行标定");
  rt_btn_row->addWidget(manual_capture);
  rt_btn_row->addWidget(auto_capture);
  rt_btn_row->addWidget(run_btn);
  rt_operation_panel_layout->addLayout(rt_btn_row);
  operation_layout->addWidget(rt_operation_panel);

  QVBoxLayout * off_param_layout = new QVBoxLayout();
  QHBoxLayout * bag_row = new QHBoxLayout();
  off_bag_dir = new QLineEdit();
  QPushButton * browse_bag = new QPushButton("选择 .db3 目录");
  bag_row->addWidget(off_bag_dir, 1);
  bag_row->addWidget(browse_bag);
  off_param_layout->addLayout(bag_row);
  off_image_topic = new QComboBox();
  off_image_topic->setEditable(false);
  QFormLayout * off_form = new QFormLayout();
  SetFormLeftAligned(off_form);
  off_form->addRow("图像话题:", off_image_topic);
  for (const auto & def : IntrinsicOfflineFieldDefs()) {
    off_form->addRow(def.first + ":", new QLineEdit(def.second));
  }
  off_param_layout->addLayout(off_form);
  QWidget * off_calib_panel = new QWidget();
  QVBoxLayout * off_calib_panel_layout = new QVBoxLayout(off_calib_panel);
  off_calib_panel_layout->setContentsMargins(0, 0, 0, 0);
  off_calib_panel_layout->addLayout(off_param_layout);
  calib_param_layout->addWidget(off_calib_panel);
  QObject::connect(browse_bag, &QPushButton::clicked, [=]() {
    const QString dir = QFileDialog::getExistingDirectory(nullptr, "选择 rosbag(.db3) 目录", QDir::homePath());
    if (dir.isEmpty()) return;
    off_bag_dir->setText(dir);
    QString err;
    std::vector<QString> topics;
    if (!ListImageTopicsFromRosbag(dir, &topics, &err)) {
      QMessageBox::warning(this, "rosbag解析失败", err);
      return;
    }
    PopulateTopicCombo(off_image_topic, topics, "/camera/image_raw");
    std::vector<QString> camera_info_topics;
    QString camera_info_err;
    if (ListCameraInfoTopicsFromRosbag(dir, &camera_info_topics, &camera_info_err)) {
      PopulateTopicCombo(off_camera_info_topic, camera_info_topics, off_camera_info_topic->currentText());
    }
  });
  QWidget * off_validation_widget = new QWidget();
  QFormLayout * off_validation_form = new QFormLayout(off_validation_widget);
  SetFormLeftAligned(off_validation_form);
  off_camera_info_topic = new QComboBox();
  off_camera_info_topic->setEditable(true);
  PopulateTopicCombo(off_camera_info_topic, {}, "/camera/camera_info");
  off_validation_form->addRow("camera_info话题:", off_camera_info_topic);
  QWidget * off_validation_panel = new QWidget();
  QVBoxLayout * off_validation_panel_layout = new QVBoxLayout(off_validation_panel);
  off_validation_panel_layout->setContentsMargins(0, 0, 0, 0);
  off_validation_panel_layout->addWidget(off_validation_widget);
  validation_param_layout->addWidget(off_validation_panel);
  QPushButton * off_run_btn = new QPushButton("执行离线标定");
  QWidget * off_operation_panel = new QWidget();
  QVBoxLayout * off_operation_panel_layout = new QVBoxLayout(off_operation_panel);
  off_operation_panel_layout->setContentsMargins(0, 0, 0, 0);
  off_operation_panel_layout->addWidget(off_run_btn);
  operation_layout->addWidget(off_operation_panel);

  QGroupBox * preview_group = new QGroupBox("图像显示");
  QVBoxLayout * preview_layout = new QVBoxLayout(preview_group);
  QHBoxLayout * preview_ctrl_row = new QHBoxLayout();
  QCheckBox * show_raw_checkbox = new QCheckBox("显示原图");
  show_raw_checkbox->setChecked(true);
  QPushButton * refresh_preview_btn = new QPushButton("刷新图像显示");
  preview_ctrl_row->addWidget(show_raw_checkbox);
  preview_ctrl_row->addStretch();
  preview_ctrl_row->addWidget(refresh_preview_btn);
  QHBoxLayout * preview_content_row = new QHBoxLayout();
  ZoomableImageWidget * preview_widget = new ZoomableImageWidget();
  preview_widget->setMinimumHeight(240);
  preview_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::MinimumExpanding);
  preview_widget->setStyleSheet("background:#20242a; border:1px solid #3a4048;");
  QTextEdit * log = new QTextEdit();
  log->setReadOnly(true);
  log->setMinimumHeight(240);
  log->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::MinimumExpanding);
  preview_content_row->addWidget(preview_widget, 1);
  preview_content_row->addWidget(log, 1);
  preview_layout->addLayout(preview_ctrl_row);
  preview_layout->addLayout(preview_content_row);
  root->addWidget(preview_group, 1);

  QGroupBox * output_group = new QGroupBox("结果输出");
  QHBoxLayout * output_layout = new QHBoxLayout(output_group);
  QLineEdit * output_yaml = new QLineEdit(DefaultCalibrationYamlPath("内参标定"));
  QPushButton * browse_output = new QPushButton("选择输出路径");
  output_layout->addWidget(output_yaml, 1);
  output_layout->addWidget(browse_output);
  root->addWidget(output_group);
  QObject::connect(browse_output, &QPushButton::clicked, [=]() {
    const QString path = QFileDialog::getSaveFileName(
      nullptr, "保存标定结果", output_yaml->text(), "YAML (*.yaml *.yml)");
    if (!path.isEmpty()) output_yaml->setText(path);
  });

  auto update_mode_visibility = [=]() {
    const bool online = (run_mode->currentIndex() == 0);
    rt_calib_panel->setVisible(online);
    rt_validation_panel->setVisible(online);
    rt_operation_panel->setVisible(online);
    off_calib_panel->setVisible(!online);
    off_validation_panel->setVisible(!online);
    off_operation_panel->setVisible(!online);
  };
  QObject::connect(run_mode, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) {
    update_mode_visibility();
  });
  auto append_log = [=](const QString & msg) {
    log->append(QString("[%1] %2").arg(QDateTime::currentDateTime().toString("hh:mm:ss")).arg(msg));
  };
  QObject::connect(manual_capture, &QPushButton::clicked, [=]() { append_log("手动采图触发。"); });
  QObject::connect(auto_capture, &QPushButton::clicked, [=]() { append_log("自动采图启动。"); });

  auto online_refresh_busy = std::make_shared<std::atomic_bool>(false);
  auto request_online_topics_refresh = [=](bool with_log) {
    if (online_refresh_busy->exchange(true)) return;
    const QString preferred = rt_image_topic->currentText();
    QPointer<QWidget> alive(this);
    RunOnSharedRefreshPool([=]() {
      const auto topics = ListOnlineImageTopics();
      const auto camera_info_topics = ListOnlineCameraInfoTopics();
      QMetaObject::invokeMethod(this, [=]() {
        online_refresh_busy->store(false);
        if (!alive) return;
        PopulateTopicCombo(rt_image_topic, topics, preferred);
        PopulateTopicCombo(rt_camera_info_topic, camera_info_topics, rt_camera_info_topic->currentText());
        if (with_log) append_log(QString("在线图像话题刷新完成，共 %1 个").arg(static_cast<int>(topics.size())));
      }, Qt::QueuedConnection);
    });
  };
  QObject::connect(refresh_live_topics, &QPushButton::clicked, [=]() { request_online_topics_refresh(true); });
  QTimer * live_topics_timer = new QTimer(this);
  live_topics_timer->setInterval(2000);
  QObject::connect(live_topics_timer, &QTimer::timeout, [=]() {
    if (!auto_refresh_live_topics->isChecked()) return;
    request_online_topics_refresh(false);
  });
  live_topics_timer->start();

  auto update_preview = [=]() {
    const bool online_mode = (run_mode->currentIndex() == 0);
    const QString topic = online_mode ? rt_image_topic->currentText() : off_image_topic->currentText();
    const QString text = show_raw_checkbox->isChecked() ? "原图预览" : "标定结果图预览";
    preview_widget->SetImage(BuildPreviewPlaceholder(text, topic));
  };
  auto image_subscription = std::make_shared<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>();
  auto image_sub_mutex = std::make_shared<std::mutex>();
  auto image_generation = std::make_shared<std::atomic<uint64_t>>(0);
  const auto stamp = static_cast<unsigned long long>(QDateTime::currentMSecsSinceEpoch());
  auto image_node = rclcpp::Node::make_shared("ros_robot_assist_tools_intrinsic_preview_" + std::to_string(stamp));
  SharedUiExecutor::instance().add_node(image_node);
  QObject::connect(this, &QObject::destroyed, [=]() {
    SharedUiExecutor::instance().remove_node(image_node);
  });
  auto subscribe_preview_topic = [=]() {
    const bool online_mode = (run_mode->currentIndex() == 0);
    if (!online_mode || !show_raw_checkbox->isChecked()) {
      std::lock_guard<std::mutex> lk(*image_sub_mutex);
      *image_subscription = nullptr;
      update_preview();
      return;
    }
    const QString topic = rt_image_topic->currentText().trimmed();
    if (topic.isEmpty()) {
      std::lock_guard<std::mutex> lk(*image_sub_mutex);
      *image_subscription = nullptr;
      update_preview();
      return;
    }
    const uint64_t generation = image_generation->fetch_add(1) + 1;
    QPointer<QWidget> alive(this);
    std::lock_guard<std::mutex> lk(*image_sub_mutex);
    *image_subscription = image_node->create_subscription<sensor_msgs::msg::Image>(
      topic.toStdString(), rclcpp::SensorDataQoS(),
      [=](const sensor_msgs::msg::Image::SharedPtr msg) {
        RunOnSharedImageRefreshPool([=]() {
          QImage image;
          if (!ConvertRosImageToQImage(*msg, &image)) {
            return;
          }
          QMetaObject::invokeMethod(this, [=]() {
            if (!alive || generation != image_generation->load()) return;
            preview_widget->SetImage(image);
          }, Qt::QueuedConnection);
        });
      });
    append_log(QString("已订阅图像话题: %1").arg(topic));
  };
  QObject::connect(refresh_preview_btn, &QPushButton::clicked, [=]() { subscribe_preview_topic(); });
  QObject::connect(show_raw_checkbox, &QCheckBox::toggled, [=](bool) { subscribe_preview_topic(); });
  QObject::connect(run_mode, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) { subscribe_preview_topic(); });
  QObject::connect(rt_image_topic, &QComboBox::currentTextChanged, [=](const QString &) { subscribe_preview_topic(); });
  QObject::connect(off_image_topic, &QComboBox::currentTextChanged, [=](const QString &) { update_preview(); });

  QObject::connect(run_btn, &QPushButton::clicked, [=]() {
    QString err;
    if (!SaveCalibrationYaml(
          output_yaml->text(), "内参标定", "online", rt_image_topic->currentText(),
          board_type->currentText(), distortion_model->currentText(),
          rt_image_topic->currentText(), rt_camera_info_topic->currentText(), {}, &err)) {
      QMessageBox::critical(this, "错误", "保存标定结果失败: " + err);
      append_log("实时标定失败: " + err);
      return;
    }
    append_log(QString("实时标定完成，结果已保存: %1").arg(output_yaml->text()));
    if (!show_raw_checkbox->isChecked()) update_preview();
  });

  QObject::connect(off_run_btn, &QPushButton::clicked, [=]() {
    QString err;
    if (!ValidateRosbagDirectory(off_bag_dir->text(), &err)) {
      QMessageBox::warning(this, "bag校验失败", err);
      append_log("离线标定失败: " + err);
      return;
    }
    std::vector<std::pair<QString, QString>> extra_fields = {{"bag_dir", off_bag_dir->text()}};
    if (!SaveCalibrationYaml(
          output_yaml->text(), "内参标定", "offline", off_bag_dir->text(),
          board_type->currentText(), distortion_model->currentText(),
          off_image_topic->currentText(), off_camera_info_topic->currentText(), extra_fields, &err)) {
      QMessageBox::critical(this, "错误", "保存标定结果失败: " + err);
      append_log("离线标定失败: " + err);
      return;
    }
    append_log(QString("离线标定完成，bag目录: %1，结果: %2").arg(off_bag_dir->text()).arg(output_yaml->text()));
    if (!show_raw_checkbox->isChecked()) update_preview();
  });

  const QString default_cfg = ResolveDefaultConfigYamlPath("intrinsic_calibration.yaml");
  RosInterfaceConfig cfg;
  QString cfg_err;
  if (LoadRosInterfaceConfigFromYaml(default_cfg, &cfg, &cfg_err)) {
    const auto topics = ListOnlineImageTopics();
    PopulateTopicCombo(rt_image_topic, topics, cfg.image_topic.isEmpty() ? "/camera/image_raw" : cfg.image_topic);
    if (!cfg.pose_endpoint.trimmed().isEmpty()) {
      rt_camera_info_topic->setCurrentText(cfg.pose_endpoint);
      off_camera_info_topic->setCurrentText(cfg.pose_endpoint);
    }
    append_log("已加载默认配置: " + default_cfg);
  } else {
    PopulateTopicCombo(rt_image_topic, ListOnlineImageTopics(), "/camera/image_raw");
    append_log("默认配置加载失败，使用界面默认值。");
  }
  PopulateTopicCombo(rt_camera_info_topic, ListOnlineCameraInfoTopics(), rt_camera_info_topic->currentText());
  PopulateTopicCombo(off_image_topic, {}, "/camera/image_raw");
  PopulateTopicCombo(off_camera_info_topic, {}, "/camera/camera_info");
  update_mode_visibility();
  subscribe_preview_topic();
}

}  // namespace ros_robot_assist_tools::ui
