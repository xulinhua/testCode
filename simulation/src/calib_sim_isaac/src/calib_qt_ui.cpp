// Qt 标定主窗口：图像缩放/平移、模式切换、命令下发与日志展示；RunCalibQtUiApp 为入口。
#include "calib_sim_isaac/calib_qt_ui.hpp"
#include "calib_sim_isaac/calib_qt_ui_data_utils.hpp"
#include "calib_sim_isaac/calib_qt_ui_history_service.hpp"
#include "calib_sim_isaac/calib_qt_ui_presenter_utils.hpp"

#include <atomic>
#include <algorithm>
#include <mutex>
#include <thread>

#include <QApplication>
#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QPixmap>
#include <QPushButton>
#include <QTextCursor>
#include <QTextEdit>
#include <QTimer>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QMessageBox>
#include <QPainter>
#include <QMouseEvent>

namespace calib_sim_isaac
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
    const QSize avail = size();
    if (avail.width() < 1 || avail.height() < 1) {
      return;
    }
    // 始终以原图为源：先算适应窗口的基准尺寸，再乘 scale_；避免 SmoothTransformation 放大发糊。
    const QSize fitted = original_image_.size().scaled(avail, Qt::KeepAspectRatio);
    if (fitted.width() < 1 || fitted.height() < 1) {
      return;
    }
    const int dw = std::max(1, static_cast<int>(std::round(static_cast<double>(fitted.width()) * scale_)));
    const int dh = std::max(1, static_cast<int>(std::round(static_cast<double>(fitted.height()) * scale_)));
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
    const QPixmap pix = QPixmap::fromImage(original_image_).scaled(
      dw, dh, Qt::KeepAspectRatio, Qt::FastTransformation);
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
  win.setWindowTitle(QString::fromUtf8("手眼标定"));
  auto * central = new QWidget(&win);
  central->setObjectName(QStringLiteral("CentralCalib"));
  auto * root = new QVBoxLayout(central);
  root->setContentsMargins(14, 12, 14, 12);
  root->setSpacing(10);

  auto * arm_select = new QComboBox();
  arm_select->setToolTip(QString::fromUtf8(
    "眼在手外：固定相机看机械臂；眼在手上：腕部相机。切换后会重新订阅对应相机话题。"));
  arm_select->addItem(QString::fromUtf8("眼在手外 · 机械臂0"), QStringLiteral("eth0"));
  arm_select->addItem(QString::fromUtf8("眼在手外 · 机械臂1"), QStringLiteral("eth1"));
  arm_select->addItem(QString::fromUtf8("眼在手上 · 机械臂0"), QStringLiteral("eih0"));
  arm_select->addItem(QString::fromUtf8("眼在手上 · 机械臂1"), QStringLiteral("eih1"));
  auto * btn_init = new QPushButton("初始化标定");
  auto * btn_step = new QPushButton("单步标定");
  auto * btn_auto = new QPushButton("自动标定");
  auto * btn_pause = new QPushButton(QString::fromUtf8("暂停标定"));
  auto * result_view = new QTextEdit();
  result_view->setReadOnly(true);
  result_view->setMinimumHeight(220);
  result_view->setPlainText("等待标定结果...");

  auto * img_row = new QHBoxLayout();
  img_row->setSpacing(10);
  auto * raw_group = new QGroupBox(QString::fromUtf8("原始图像"));
  auto * raw_box_layout = new QVBoxLayout(raw_group);
  raw_box_layout->setContentsMargins(8, 8, 8, 8);
  auto * res_group = new QGroupBox(QString::fromUtf8("检测结果"));
  auto * res_box_layout = new QVBoxLayout(res_group);
  res_box_layout->setContentsMargins(8, 8, 8, 8);
  auto * raw_label = new ZoomableImageLabel("RAW");
  auto * res_label = new ZoomableImageLabel("RESULT");
  raw_label->setMinimumSize(480, 320);
  res_label->setMinimumSize(480, 320);
  raw_label->setStyleSheet("background:black;color:white;");
  res_label->setStyleSheet("background:black;color:white;");
  raw_box_layout->addWidget(raw_label);
  res_box_layout->addWidget(res_label);
  img_row->addWidget(raw_group, 1);
  img_row->addWidget(res_group, 1);

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
  ctrl_layout->addWidget(new QLabel(QString::fromUtf8("标定模式")));
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

  root->addLayout(img_row);
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
  /// 供历史列表/清空磁盘使用；标定结果里的 calib_run_dir 可能是相对路径，需与 CalibNode 的 cwd 一致
  std::string history_result_text_hint;

  const auto filterYamlLinesForDisplay = [](const QString & yaml_text) -> QString {
    return QString::fromStdString(FilterResultYamlForDisplay(yaml_text.toStdString()));
  };

  std::vector<std::string> last_run_dirs_cache;
  const auto refreshRunList = [&](const std::string & result_text_hint) {
    std::vector<std::string> run_dirs = ListRunDirs(result_text_hint);
    if (run_dirs.empty()) {
      run_select->clear();
      last_run_dirs_cache.clear();
      return;
    }
    std::sort(run_dirs.begin(), run_dirs.end(), std::greater<std::string>());
    if (run_dirs == last_run_dirs_cache) {
      return;
    }
    last_run_dirs_cache = run_dirs;
    const QString prev_data = run_select->currentData().toString();
    run_select->clear();
    for (const auto & path : run_dirs) {
      run_select->addItem(
        QString::fromStdString(Basename(path)),
        QString::fromStdString(path));
    }
    if (!prev_data.isEmpty()) {
      const int idx = run_select->findData(prev_data);
      if (idx >= 0) {
        run_select->setCurrentIndex(idx);
      }
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
    const RunHistoryData data = LoadRunHistoryData(run_dir);
    if (data.yaml_found) {
      result_view->setPlainText(QString::fromStdString(data.filtered_yaml_text));
      appendUiLog(std::string("[历史数据] 已读取标定 YAML: ") + data.yaml_file);
    } else {
      appendUiLog("[历史数据] 未找到 calib_result_eye_to_hand.yaml / eye_in_hand.yaml");
    }
    selected_result_images = data.result_images;
    selected_raw_images = data.raw_images;
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

  history_result_text_hint = ros_node->resultText();
  refreshRunList(history_result_text_hint);

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
    const std::string mode = arm_select->currentData().toString().toStdString();
    ros_node->sendCmd("set_mode:" + mode);
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
      QString::fromUtf8("将永久删除已扫描到的 calib_output_isaac 目录下所有 calib_run_* 子目录，是否继续？"),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (r != QMessageBox::Yes) {
      return;
    }
    int dirs_removed = 0;
    const std::vector<std::string> scan_roots = CollectScanRoots(history_result_text_hint);
    if (scan_roots.empty()) {
      appendUiLog("[历史数据] 未找到可删除的标定输出目录（请确认在 simulation 下启动或设置 CALIB_SIM_ISAAC_OUTPUT_DIR）");
    } else {
      dirs_removed = RemoveAllRunDirs(history_result_text_hint);
    }
    clearLocalUiData();
    refreshRunList("");
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
    history_result_text_hint = local.result_text;
    if (local.result_text != last_result_text_for_auto_pick) {
      last_result_text_for_auto_pick = local.result_text;
      showing_history_image = false;
      const std::string run_dir = ParseRunDirFromResultText(local.result_text);
      if (!run_dir.empty()) {
        selected_result_images = CollectSampleImages(run_dir, "result_sample_");
        selected_raw_images = CollectSampleImages(run_dir, "raw_sample_");
        selected_sample_index = selected_result_images.empty() ? -1 :
          static_cast<int>(selected_result_images.size()) - 1;
      }
      refreshRunList(local.result_text);
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
      std::vector<std::string> display_logs = BuildDisplayLogs(ui_log_extra, local.logs);
      if (display_logs != rendered_logs_snapshot) {
        log_view->clear();
        for (const auto & line : display_logs) {
          AppendLogLineColored(log_view, QString::fromStdString(line));
        }
        rendered_logs_snapshot = std::move(display_logs);
        log_view->moveCursor(QTextCursor::End);
      }
    }
  });
  ui_timer.start(100);

  QTimer run_list_timer;
  QObject::connect(&run_list_timer, &QTimer::timeout, [&]() {
    refreshRunList(history_result_text_hint);
  });
  run_list_timer.start(2000);

  // Ensure Ctrl+C (ROS shutdown) also closes Qt UI.
  QTimer shutdown_watchdog;
  QObject::connect(&shutdown_watchdog, &QTimer::timeout, [&app]() {
    if (!rclcpp::ok()) {
      app.quit();
    }
  });
  shutdown_watchdog.start(100);

  win.show();
  // 启动后事件循环就绪再扫一次盘，避免首帧 cache 未就绪时列表为空
  QTimer::singleShot(0, [&]() {
    history_result_text_hint = ros_node->resultText();
    refreshRunList(history_result_text_hint);
  });
  QTimer::singleShot(500, [&]() {
    history_result_text_hint = ros_node->resultText();
    refreshRunList(history_result_text_hint);
  });
  const int rc = app.exec();
  cache_running.store(false);
  if (cache_thread.joinable()) {
    cache_thread.join();
  }
  return rc;
}

}  // namespace calib_sim_isaac
