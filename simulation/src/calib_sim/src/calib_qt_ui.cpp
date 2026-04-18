#include "calib_sim/calib_qt_ui.hpp"

#include <atomic>
#include <algorithm>
#include <filesystem>
#include <mutex>
#include <thread>

#include <QApplication>
#include <QComboBox>
#include <QFile>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QPixmap>
#include <QPushButton>
#include <QTextStream>
#include <QTextCursor>
#include <QTextEdit>
#include <QTimer>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QMessageBox>
#include <QPainter>
#include <QMouseEvent>

namespace calib_sim
{

class ZoomableImageLabel : public QLabel
{
public:
  explicit ZoomableImageLabel(const QString & text = QString(), QWidget * parent = nullptr)
  : QLabel(text, parent)
  {
    setAlignment(Qt::AlignCenter);
  }

  void setImage(const QImage & image)
  {
    original_image_ = image;
    update();
  }

protected:
  void wheelEvent(QWheelEvent * event) override
  {
    if (original_image_.isNull()) {
      QLabel::wheelEvent(event);
      return;
    }
    const QPoint delta = event->angleDelta();
    if (delta.y() == 0) {
      return;
    }
    const QPointF cursor = event->position();
    const QPointF old_center(width() * 0.5, height() * 0.5);
    const QPointF old_rel = cursor - old_center - QPointF(pan_offset_);
    const double old_scale = scale_;
    const double step = (delta.y() > 0) ? 1.1 : (1.0 / 1.1);
    scale_ = std::max(0.2, std::min(8.0, scale_ * step));
    if (old_scale > 1e-9) {
      const double ratio = scale_ / old_scale;
      const QPointF new_pan = cursor - old_center - old_rel * ratio;
      pan_offset_.setX(static_cast<int>(std::round(new_pan.x())));
      pan_offset_.setY(static_cast<int>(std::round(new_pan.y())));
    }
    update();
    event->accept();
  }

  void mouseDoubleClickEvent(QMouseEvent * event) override
  {
    Q_UNUSED(event);
    scale_ = 1.0;
    pan_offset_ = QPoint(0, 0);
    update();
  }

  void mousePressEvent(QMouseEvent * event) override
  {
    if (event->button() == Qt::LeftButton) {
      dragging_ = true;
      last_mouse_pos_ = event->pos();
      setCursor(Qt::ClosedHandCursor);
      event->accept();
      return;
    }
    QLabel::mousePressEvent(event);
  }

  void mouseMoveEvent(QMouseEvent * event) override
  {
    if (dragging_) {
      const QPoint delta = event->pos() - last_mouse_pos_;
      pan_offset_ += delta;
      last_mouse_pos_ = event->pos();
      update();
      event->accept();
      return;
    }
    QLabel::mouseMoveEvent(event);
  }

  void mouseReleaseEvent(QMouseEvent * event) override
  {
    if (event->button() == Qt::LeftButton && dragging_) {
      dragging_ = false;
      setCursor(Qt::ArrowCursor);
      event->accept();
      return;
    }
    QLabel::mouseReleaseEvent(event);
  }

  void resizeEvent(QResizeEvent * event) override
  {
    QLabel::resizeEvent(event);
    update();
  }

  void paintEvent(QPaintEvent * event) override
  {
    QLabel::paintEvent(event);
    if (original_image_.isNull()) {
      return;
    }
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    const QSize base = size();
    const QSize target_size(
      std::max(1, static_cast<int>(base.width() * scale_)),
      std::max(1, static_cast<int>(base.height() * scale_)));
    const QPixmap pix = QPixmap::fromImage(original_image_).scaled(
      target_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    const int x = (width() - pix.width()) / 2 + pan_offset_.x();
    const int y = (height() - pix.height()) / 2 + pan_offset_.y();
    painter.drawPixmap(x, y, pix);
    event->accept();
  }

  QImage original_image_;
  double scale_{1.0};
  QPoint pan_offset_{0, 0};
  bool dragging_{false};
  QPoint last_mouse_pos_{0, 0};
};

CalibQtUiRosNode::CalibQtUiRosNode(const rclcpp::NodeOptions & options)
: Node("calib_qt_ui_node", options)
{
  status_sub_ = create_subscription<std_msgs::msg::String>(
    "/calib_sim/status", 20, [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mu_);
      status_ = msg->data;
    });
  log_sub_ = create_subscription<std_msgs::msg::String>(
    "/calib_sim/log", 50, [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mu_);
      logs_.push_back(msg->data);
      if (logs_.size() > 200) {
        logs_.erase(logs_.begin());
      }
    });
  reach_error_sub_ = create_subscription<std_msgs::msg::String>(
    "/calib_sim/reach_error", 20, [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mu_);
      reach_error_ = msg->data;
    });
  result_text_sub_ = create_subscription<std_msgs::msg::String>(
    "/calib_sim/result_text", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mu_);
      result_text_ = msg->data;
    });
  raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/calib_sim/raw_image", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mu_);
      raw_ = *msg;
    });
  result_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/calib_sim/result_image", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mu_);
      result_ = *msg;
    });
  ctrl_pub_ = create_publisher<std_msgs::msg::String>("/calib_sim/control", 10);
}

void CalibQtUiRosNode::sendCmd(const std::string & cmd)
{
  std_msgs::msg::String m;
  m.data = cmd;
  ctrl_pub_->publish(m);
}

void CalibQtUiRosNode::clearLogsAndResult()
{
  std::lock_guard<std::mutex> lk(mu_);
  logs_.clear();
  result_text_ = "等待标定结果...";
  raw_ = sensor_msgs::msg::Image{};
  result_ = sensor_msgs::msg::Image{};
}

std::string CalibQtUiRosNode::status()
{
  std::lock_guard<std::mutex> lk(mu_);
  return status_;
}

std::vector<std::string> CalibQtUiRosNode::logs()
{
  std::lock_guard<std::mutex> lk(mu_);
  return logs_;
}

std::string CalibQtUiRosNode::reachError()
{
  std::lock_guard<std::mutex> lk(mu_);
  return reach_error_;
}

std::string CalibQtUiRosNode::resultText()
{
  std::lock_guard<std::mutex> lk(mu_);
  return result_text_;
}

sensor_msgs::msg::Image CalibQtUiRosNode::raw()
{
  std::lock_guard<std::mutex> lk(mu_);
  return raw_;
}

sensor_msgs::msg::Image CalibQtUiRosNode::result()
{
  std::lock_guard<std::mutex> lk(mu_);
  return result_;
}

static QImage toQImage(const sensor_msgs::msg::Image & msg)
{
  if (msg.width == 0 || msg.height == 0 || msg.data.empty()) {
    return QImage();
  }
  if (msg.encoding == "bgr8" || msg.encoding == "rgb8") {
    QImage img(
      msg.data.data(), static_cast<int>(msg.width), static_cast<int>(msg.height),
      static_cast<int>(msg.step), QImage::Format_RGB888);
    if (msg.encoding == "bgr8") {
      return img.rgbSwapped().copy();
    }
    return img.copy();
  }
  return QImage();
}

int RunCalibQtUiApp(const std::shared_ptr<CalibQtUiRosNode> & ros_node, int argc, char ** argv)
{
  namespace fs = std::filesystem;
  QApplication app(argc, argv);
  app.setStyleSheet(QString::fromUtf8(R"(
    QMainWindow { background: #eef1f6; }
    QWidget#CentralCalib { background: #eef1f6; }
    QGroupBox {
      font-weight: 600;
      font-size: 13px;
      border: 1px solid #c5cad6;
      border-radius: 8px;
      margin-top: 12px;
      padding: 10px 12px 12px 12px;
      background: #ffffff;
    }
    QGroupBox::title {
      subcontrol-origin: margin;
      left: 12px;
      padding: 0 6px;
      color: #1e2430;
    }
    QPushButton {
      padding: 7px 14px;
      border-radius: 5px;
      border: 1px solid #b4bac7;
      background: #f8f9fc;
      min-height: 22px;
    }
    QPushButton:hover { background: #e9ecf4; border-color: #9aa3b4; }
    QPushButton:pressed { background: #dde2ee; }
    QTextEdit, QComboBox {
      border: 1px solid #c5cad6;
      border-radius: 5px;
      background: #ffffff;
      padding: 4px;
      selection-background-color: #c9d8f0;
    }
    QTextEdit { padding: 6px; }
    QLabel { color: #252b38; }
  )"));
  QMainWindow win;
  win.setWindowTitle("calib_sim_qt_ui");
  auto * central = new QWidget(&win);
  central->setObjectName(QStringLiteral("CentralCalib"));
  auto * root = new QVBoxLayout(central);
  root->setContentsMargins(14, 12, 14, 12);
  root->setSpacing(10);

  auto * status_label = new QLabel("Status: waiting");
  auto * reach_label = new QLabel("Reach: reach_err pos_mm=0 ang_deg=0");
  auto * arm_select = new QComboBox();
  arm_select->addItem("机械臂0", 0);
  arm_select->addItem("机械臂1", 1);
  auto * btn_init = new QPushButton("初始化标定");
  auto * btn_step = new QPushButton("单步标定");
  auto * btn_auto = new QPushButton("自动标定");
  auto * btn_pause = new QPushButton("暂停");
  auto * result_view = new QTextEdit();
  result_view->setReadOnly(true);
  result_view->setMinimumHeight(220);
  result_view->setPlainText("等待标定结果...");

  auto * img_group = new QGroupBox("图像区域");
  auto * img_row = new QHBoxLayout(img_group);
  img_row->setSpacing(10);
  auto * raw_label = new ZoomableImageLabel("RAW");
  auto * res_label = new ZoomableImageLabel("RESULT");
  raw_label->setMinimumSize(480, 320);
  res_label->setMinimumSize(480, 320);
  raw_label->setStyleSheet("background:black;color:white;");
  res_label->setStyleSheet("background:black;color:white;");
  img_row->addWidget(raw_label);
  img_row->addWidget(res_label);

  auto * log_group = new QGroupBox("日志");
  auto * log_layout = new QVBoxLayout(log_group);
  log_layout->setSpacing(6);
  auto * log_view = new QTextEdit();
  log_view->setReadOnly(true);
  log_view->setMinimumHeight(160);
  log_layout->addWidget(log_view);

  auto * result_group = new QGroupBox("标定结果");
  auto * result_layout = new QVBoxLayout(result_group);
  result_layout->setSpacing(6);
  result_layout->addWidget(result_view);

  auto * run_select = new QComboBox();
  run_select->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  auto * btn_load_run = new QPushButton("加载结果");
  auto * btn_clear_current_data = new QPushButton("清空当前数据");
  auto * btn_clear_all_data = new QPushButton("清空所有数据");
  auto * btn_prev_sample = new QPushButton("上一张");
  auto * btn_next_sample = new QPushButton("下一张");

  auto * left_column = new QWidget();
  auto * left_col_layout = new QVBoxLayout(left_column);
  left_col_layout->setContentsMargins(0, 0, 0, 0);
  left_col_layout->setSpacing(10);
  left_col_layout->addWidget(log_group, 1);
  left_col_layout->addWidget(result_group, 2);

  auto * bottom_row = new QHBoxLayout();
  auto * control_group = new QGroupBox("操作与历史");
  auto * ctrl_layout = new QVBoxLayout(control_group);
  ctrl_layout->setSpacing(4);
  ctrl_layout->addWidget(arm_select);
  ctrl_layout->addWidget(btn_init);
  ctrl_layout->addWidget(btn_step);
  ctrl_layout->addWidget(btn_auto);
  ctrl_layout->addWidget(btn_pause);
  ctrl_layout->addSpacing(8);
  ctrl_layout->addWidget(new QLabel("历史标定 run"));
  ctrl_layout->addWidget(run_select);
  ctrl_layout->addWidget(btn_load_run);
  ctrl_layout->addWidget(btn_clear_current_data);
  ctrl_layout->addWidget(btn_clear_all_data);
  ctrl_layout->addSpacing(6);
  auto * sample_nav_row = new QHBoxLayout();
  sample_nav_row->setSpacing(8);
  sample_nav_row->addWidget(btn_prev_sample, 1);
  sample_nav_row->addWidget(btn_next_sample, 1);
  ctrl_layout->addLayout(sample_nav_row);
  control_group->setMinimumWidth(240);
  control_group->setMaximumWidth(300);
  control_group->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);

  bottom_row->setSpacing(12);
  bottom_row->addWidget(left_column, 5, Qt::AlignTop);
  bottom_row->addWidget(control_group, 2, Qt::AlignTop);

  root->addWidget(status_label);
  root->addWidget(reach_label);
  root->addWidget(img_group);
  root->addLayout(bottom_row);
  win.setCentralWidget(central);
  win.resize(1100, 780);

  struct UiCache
  {
    std::string status;
    std::string reach_error;
    std::string result_text;
    std::vector<std::string> logs;
    sensor_msgs::msg::Image raw;
    sensor_msgs::msg::Image result;
  };
  std::mutex cache_mu;
  UiCache cache;
  std::atomic<bool> cache_running{true};
  std::vector<std::string> rendered_logs_snapshot;
  std::vector<std::string> ui_log_extra;
  bool showing_history_image = false;
  std::vector<std::string> selected_result_images;
  std::vector<std::string> selected_raw_images;
  int selected_sample_index = -1;
  std::string last_result_text_for_auto_pick;
  const fs::path output_root("/home/hs/testCode/simulation/calib_output");

  const auto trimStr = [](std::string s) -> std::string {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
  };

  const auto parseRunDirFromResultText = [&](const std::string & result_text) -> std::string {
    {
      const std::string key = "calib_run_stamp:";
      const std::size_t pos = result_text.rfind(key);
      if (pos != std::string::npos) {
        std::string stamp = trimStr(result_text.substr(pos + key.size()));
        if (!stamp.empty()) {
          return (output_root / ("calib_run_" + stamp)).string();
        }
      }
    }
    const std::string key = "result_image_samples_dir:";
    const std::size_t pos = result_text.rfind(key);
    if (pos == std::string::npos) {
      return "";
    }
    return trimStr(result_text.substr(pos + key.size()));
  };

  const auto collectSampleImages = [](const std::string & run_dir, const char * prefix) {
    std::vector<std::string> files;
    if (run_dir.empty()) {
      return files;
    }
    std::error_code ec;
    if (!fs::exists(run_dir, ec) || ec) {
      return files;
    }
    const std::string pre(prefix);
    for (const auto & entry : fs::directory_iterator(run_dir, ec)) {
      if (ec || !entry.is_regular_file()) {
        continue;
      }
      const std::string name = entry.path().filename().string();
      if (name.rfind(pre, 0) == 0 && entry.path().extension() == ".png") {
        files.push_back(entry.path().string());
      }
    }
    std::sort(files.begin(), files.end());
    return files;
  };

  const auto filterYamlLinesForDisplay = [](const QString & yaml_text) -> QString {
    QString out;
    const QStringList lines = yaml_text.split('\n');
    for (const QString & line : lines) {
      if (line.contains(QLatin1String("sample_manifest_file")) ||
        line.contains(QLatin1String("result_file:")) ||
        line.contains(QLatin1String("result_image_samples_dir:")) ||
        line.contains(QLatin1String("calib_result_file:")))
      {
        continue;
      }
      out += line;
      out += QLatin1Char('\n');
    }
    return out;
  };

  const auto refreshRunList = [&]() {
    run_select->clear();
    std::error_code ec;
    if (!fs::exists(output_root, ec) || ec) {
      return;
    }
    std::vector<std::string> run_dirs;
    for (const auto & entry : fs::directory_iterator(output_root, ec)) {
      if (ec || !entry.is_directory()) {
        continue;
      }
      const std::string name = entry.path().filename().string();
      if (name.rfind("calib_run_", 0) == 0) {
        run_dirs.push_back(entry.path().string());
      }
    }
    std::sort(run_dirs.begin(), run_dirs.end(), std::greater<std::string>());
    for (const auto & path : run_dirs) {
      run_select->addItem(
        QString::fromStdString(fs::path(path).filename().string()),
        QString::fromStdString(path));
    }
  };

  const auto backToLiveView = [&]() {
    showing_history_image = false;
    selected_sample_index = -1;
  };

  const auto showSampleAt = [&](int idx) {
    const int n = static_cast<int>(selected_result_images.size());
    if (idx < 0 || idx >= n) {
      return;
    }
    const QImage res_img(QString::fromStdString(selected_result_images[static_cast<std::size_t>(idx)]));
    if (res_img.isNull()) {
      return;
    }
    selected_sample_index = idx;
    showing_history_image = true;
    res_label->setImage(res_img);
    if (idx < static_cast<int>(selected_raw_images.size())) {
      const QImage raw_img(QString::fromStdString(selected_raw_images[static_cast<std::size_t>(idx)]));
      if (!raw_img.isNull()) {
        raw_label->setImage(raw_img);
      }
    }
  };

  const auto appendUiLog = [&](const std::string & line) {
    ui_log_extra.push_back(line);
  };

  const auto loadRunResult = [&](const std::string & run_dir) {
    if (run_dir.empty()) {
      appendUiLog("[历史数据] 未选择 run 目录");
      return;
    }
    appendUiLog(std::string("[历史数据] 开始加载: ") + run_dir);
    std::string yaml_file = run_dir + "/calib_result_eye_to_hand.yaml";
    if (!fs::exists(yaml_file)) {
      const std::string fallback = run_dir + "/calib_result_eye_in_hand.yaml";
      if (fs::exists(fallback)) {
        yaml_file = fallback;
      }
    }
    if (fs::exists(yaml_file)) {
      QFile f(QString::fromStdString(yaml_file));
      if (f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream in(&f);
        result_view->setPlainText(filterYamlLinesForDisplay(in.readAll()));
        appendUiLog(std::string("[历史数据] 已读取标定 YAML: ") + yaml_file);
      }
    } else {
      appendUiLog("[历史数据] 未找到 calib_result_eye_to_hand.yaml / eye_in_hand.yaml");
    }
    selected_result_images = collectSampleImages(run_dir, "result_sample_");
    selected_raw_images = collectSampleImages(run_dir, "raw_sample_");
    appendUiLog(
      std::string("[历史数据] 结果图 ") + std::to_string(selected_result_images.size()) + " 张, 原图 " +
      std::to_string(selected_raw_images.size()) + " 张");
    if (!selected_result_images.empty()) {
      showSampleAt(static_cast<int>(selected_result_images.size()) - 1);
      appendUiLog("[历史数据] 加载完成，可切换「上一张/下一张」浏览原图与结果图");
    } else {
      selected_sample_index = -1;
      appendUiLog("[历史数据] 无样本 PNG，仅显示 YAML 文本");
    }
  };

  refreshRunList();

  QObject::connect(btn_step, &QPushButton::clicked, [&, ros_node]() {
    backToLiveView();
    ros_node->sendCmd("step");
  });
  QObject::connect(btn_auto, &QPushButton::clicked, [&, ros_node]() {
    backToLiveView();
    ros_node->sendCmd("auto");
  });
  QObject::connect(btn_pause, &QPushButton::clicked, [&, ros_node]() {
    backToLiveView();
    ros_node->sendCmd("pause");
  });

  const auto clearLocalUiData = [&]() {
    ros_node->clearLogsAndResult();
    log_view->clear();
    result_view->setPlainText(QString::fromUtf8("等待标定结果..."));
    rendered_logs_snapshot.clear();
    ui_log_extra.clear();
    showing_history_image = false;
    selected_sample_index = -1;
    selected_result_images.clear();
    selected_raw_images.clear();
  };

  QObject::connect(arm_select, QOverload<int>::of(&QComboBox::activated), [&, ros_node, arm_select](int) {
    clearLocalUiData();
    const int arm = arm_select->currentData().toInt();
    ros_node->sendCmd("set_arm:" + std::to_string(arm));
  });
  QObject::connect(btn_init, &QPushButton::clicked, [&, ros_node]() {
    clearLocalUiData();
    ros_node->sendCmd("init");
  });

  QObject::connect(btn_load_run, &QPushButton::clicked, [&]() {
    const std::string run_dir = run_select->currentData().toString().toStdString();
    loadRunResult(run_dir);
  });
  QObject::connect(btn_clear_current_data, &QPushButton::clicked, [&]() {
    clearLocalUiData();
    appendUiLog("[历史数据] 已清空当前界面上的日志与标定结果（内存）");
  });
  QObject::connect(btn_clear_all_data, &QPushButton::clicked, [&]() {
    const auto r = QMessageBox::question(
      &win, QString::fromUtf8("确认"),
      QString::fromUtf8("将永久删除 calib_output 下所有 calib_run_* 目录，是否继续？"),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (r != QMessageBox::Yes) {
      return;
    }
    std::error_code ec;
    if (!fs::exists(output_root, ec) || ec) {
      appendUiLog("[历史数据] calib_output 不存在，跳过删除");
      refreshRunList();
      return;
    }
    int dirs_removed = 0;
    for (const auto & entry : fs::directory_iterator(output_root, ec)) {
      if (ec || !entry.is_directory()) {
        continue;
      }
      const std::string name = entry.path().filename().string();
      if (name.rfind("calib_run_", 0) != 0) {
        continue;
      }
      std::error_code rm_ec;
      fs::remove_all(entry.path(), rm_ec);
      if (!rm_ec) {
        ++dirs_removed;
      }
    }
    clearLocalUiData();
    refreshRunList();
    appendUiLog(
      std::string("[历史数据] 已清空磁盘标定目录，已删除 run 目录数 ") + std::to_string(dirs_removed));
  });
  QObject::connect(btn_prev_sample, &QPushButton::clicked, [&]() {
    if (selected_result_images.empty()) {
      return;
    }
    int idx = selected_sample_index;
    if (idx < 0) {
      idx = 0;
    } else {
      idx = (idx - 1 + static_cast<int>(selected_result_images.size())) %
        static_cast<int>(selected_result_images.size());
    }
    showSampleAt(idx);
  });
  QObject::connect(btn_next_sample, &QPushButton::clicked, [&]() {
    if (selected_result_images.empty()) {
      return;
    }
    int idx = selected_sample_index;
    if (idx < 0) {
      idx = 0;
    } else {
      idx = (idx + 1) % static_cast<int>(selected_result_images.size());
    }
    showSampleAt(idx);
  });

  // Worker thread: pull ROS data periodically, keep UI thread lightweight.
  std::thread cache_thread([&]() {
    while (cache_running.load()) {
      UiCache snap;
      snap.status = ros_node->status();
      snap.reach_error = ros_node->reachError();
      snap.result_text = ros_node->resultText();
      snap.logs = ros_node->logs();
      snap.raw = ros_node->raw();
      snap.result = ros_node->result();
      {
        std::lock_guard<std::mutex> lk(cache_mu);
        cache = std::move(snap);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });

  QTimer ui_timer;
  QObject::connect(&ui_timer, &QTimer::timeout, [&]() {
    UiCache local;
    {
      std::lock_guard<std::mutex> lk(cache_mu);
      local = cache;
    }
    status_label->setText(QString::fromStdString("Status: " + local.status));
    reach_label->setText(QString::fromStdString("Reach: " + local.reach_error));
    if (local.result_text != last_result_text_for_auto_pick) {
      last_result_text_for_auto_pick = local.result_text;
      showing_history_image = false;
      const std::string run_dir = parseRunDirFromResultText(local.result_text);
      if (!run_dir.empty()) {
        selected_result_images = collectSampleImages(run_dir, "result_sample_");
        selected_raw_images = collectSampleImages(run_dir, "raw_sample_");
        selected_sample_index = selected_result_images.empty() ? -1 :
          static_cast<int>(selected_result_images.size()) - 1;
      }
      refreshRunList();
    }
    {
      const QString filtered_result =
        filterYamlLinesForDisplay(QString::fromStdString(local.result_text));
      if (!showing_history_image) {
        if (result_view->toPlainText() != filtered_result) {
          result_view->setPlainText(filtered_result);
        }
      }
    }
    auto res = toQImage(local.result);
    if (!showing_history_image) {
      auto raw = toQImage(local.raw);
      if (!raw.isNull()) {
        raw_label->setImage(raw);
      }
      if (!res.isNull()) {
        res_label->setImage(res);
      }
    }
    const bool user_is_selecting = log_view->textCursor().hasSelection();
    if (!user_is_selecting) {
      std::vector<std::string> display_logs = ui_log_extra;
      display_logs.insert(display_logs.end(), local.logs.begin(), local.logs.end());
      if (display_logs != rendered_logs_snapshot) {
        log_view->clear();
        for (const auto & line : display_logs) {
          log_view->append(QString::fromStdString(line));
        }
        rendered_logs_snapshot = std::move(display_logs);
        log_view->moveCursor(QTextCursor::End);
      }
    }
  });
  ui_timer.start(100);

  // Ensure Ctrl+C (ROS shutdown) also closes Qt UI.
  QTimer shutdown_watchdog;
  QObject::connect(&shutdown_watchdog, &QTimer::timeout, [&app]() {
    if (!rclcpp::ok()) {
      app.quit();
    }
  });
  shutdown_watchdog.start(100);

  win.show();
  const int rc = app.exec();
  cache_running.store(false);
  if (cache_thread.joinable()) {
    cache_thread.join();
  }
  return rc;
}

}  // namespace calib_sim
