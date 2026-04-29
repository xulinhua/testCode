#include "ros_robot_assist_tools/ui/system_status_widget.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include <QAbstractItemView>
#include <QComboBox>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMetaObject>
#include <QMessageBox>
#include <QProcess>
#include <QProgressBar>
#include <QTableWidget>
#include <QVBoxLayout>

#include "ros_robot_assist_tools/module/system_status_module.h"

namespace ros_robot_assist_tools::ui
{

SystemStatusWidget::SystemStatusWidget(QWidget * parent)
: QWidget(parent)
{
  setStyleSheet("QGroupBox{font-size:15px; font-weight:600;}");
  QVBoxLayout * layout = new QVBoxLayout(this);

  auto create_percent_item = [](const QString & name, QProgressBar ** bar_out, QLabel ** text_out) {
    QWidget * row = new QWidget();
    QHBoxLayout * row_layout = new QHBoxLayout(row);
    row_layout->setContentsMargins(0, 0, 0, 0);
    QLabel * name_label = new QLabel(name);
    name_label->setFixedWidth(80);
    QProgressBar * bar = new QProgressBar();
    bar->setRange(0, 100);
    bar->setTextVisible(false);
    QLabel * value = new QLabel("0%");
    value->setFixedWidth(70);
    row_layout->addWidget(name_label);
    row_layout->addWidget(bar, 1);
    row_layout->addWidget(value);
    *bar_out = bar;
    *text_out = value;
    return row;
  };
  auto create_rate_item = [](const QString & name, QProgressBar ** bar_out, QLabel ** text_out) {
    QWidget * row = new QWidget();
    QHBoxLayout * row_layout = new QHBoxLayout(row);
    row_layout->setContentsMargins(0, 0, 0, 0);
    QLabel * name_label = new QLabel(name);
    name_label->setFixedWidth(90);
    QProgressBar * bar = new QProgressBar();
    bar->setRange(0, 100);
    bar->setTextVisible(false);
    QLabel * value = new QLabel("0.0 KB/s");
    value->setMinimumWidth(90);
    row_layout->addWidget(name_label);
    row_layout->addWidget(bar, 1);
    row_layout->addWidget(value);
    *bar_out = bar;
    *text_out = value;
    return row;
  };

  QGroupBox * top_group = new QGroupBox("系统状态");
  QVBoxLayout * top_layout = new QVBoxLayout(top_group);
  QProgressBar * cpu_bar = nullptr, * mem_bar = nullptr, * gpu_bar = nullptr, * vram_bar = nullptr;
  QLabel * cpu_txt = nullptr, * mem_txt = nullptr, * gpu_txt = nullptr, * vram_txt = nullptr;
  QWidget * status_split = new QWidget();
  QHBoxLayout * split_layout = new QHBoxLayout(status_split);
  split_layout->setContentsMargins(0, 0, 0, 0);
  split_layout->setSpacing(12);
  QWidget * left_col = new QWidget();
  QVBoxLayout * left_layout = new QVBoxLayout(left_col);
  left_layout->setContentsMargins(0, 0, 0, 0);
  left_layout->addWidget(create_percent_item("CPU", &cpu_bar, &cpu_txt));
  left_layout->addWidget(create_percent_item("内存", &mem_bar, &mem_txt));
  left_layout->addWidget(create_percent_item("GPU", &gpu_bar, &gpu_txt));
  left_layout->addWidget(create_percent_item("显存", &vram_bar, &vram_txt));
  QProgressBar * disk_read_bar = nullptr, * disk_write_bar = nullptr, * up_bar = nullptr, * down_bar = nullptr;
  QLabel * disk_read_txt = nullptr, * disk_write_txt = nullptr, * up_txt = nullptr, * down_txt = nullptr;
  QWidget * right_col = new QWidget();
  QVBoxLayout * right_layout = new QVBoxLayout(right_col);
  right_layout->setContentsMargins(0, 0, 0, 0);
  right_layout->addWidget(create_rate_item("磁盘读", &disk_read_bar, &disk_read_txt));
  right_layout->addWidget(create_rate_item("磁盘写", &disk_write_bar, &disk_write_txt));
  right_layout->addWidget(create_rate_item("上传", &up_bar, &up_txt));
  right_layout->addWidget(create_rate_item("下载", &down_bar, &down_txt));
  split_layout->addWidget(left_col, 1);
  split_layout->addWidget(right_col, 1);
  top_layout->addWidget(status_split);
  layout->addWidget(top_group, 1);

  QGroupBox * bottom_group = new QGroupBox("进程 / ROS2节点监控");
  QVBoxLayout * bottom_layout = new QVBoxLayout(bottom_group);
  QWidget * monitor_type_row = new QWidget();
  QHBoxLayout * monitor_type_layout = new QHBoxLayout(monitor_type_row);
  monitor_type_layout->setContentsMargins(0, 0, 0, 0);
  QLabel * monitor_type_label = new QLabel("监控类型:");
  QComboBox * monitor_type_combo = new QComboBox();
  monitor_type_combo->addItems({"进程", "ROS2节点", "ROS2话题", "ROS2服务", "ROS2动作", "节点参数"});
  QLabel * ros_filter_label = new QLabel("关键字过滤:");
  QLineEdit * ros_filter_edit = new QLineEdit();
  ros_filter_edit->setPlaceholderText("输入关键字过滤");
  QLabel * param_node_label = new QLabel("节点名:");
  QComboBox * param_node_combo = new QComboBox();
  param_node_combo->setEditable(true);
  param_node_combo->setInsertPolicy(QComboBox::NoInsert);
  param_node_combo->setMinimumWidth(220);
  ros_filter_label->setVisible(false);
  ros_filter_edit->setVisible(false);
  param_node_label->setVisible(false);
  param_node_combo->setVisible(false);
  monitor_type_layout->addWidget(monitor_type_label);
  monitor_type_layout->addWidget(monitor_type_combo);
  monitor_type_layout->addSpacing(14);
  monitor_type_layout->addWidget(ros_filter_label);
  monitor_type_layout->addWidget(ros_filter_edit, 1);
  monitor_type_layout->addWidget(param_node_label);
  monitor_type_layout->addWidget(param_node_combo, 1);
  monitor_type_layout->addStretch();
  bottom_layout->addWidget(monitor_type_row);

  QTableWidget * monitor_table = new QTableWidget();
  monitor_table->setColumnCount(4);
  monitor_table->setHorizontalHeaderLabels({"PID", "CPU%", "MEM%", "COMMAND"});
  monitor_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  monitor_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  monitor_table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  monitor_table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  monitor_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  monitor_table->setSelectionBehavior(QAbstractItemView::SelectRows);
  monitor_table->setAlternatingRowColors(true);
  monitor_table->setSortingEnabled(true);
  monitor_table->setWordWrap(false);
  monitor_table->setTextElideMode(Qt::ElideNone);
  monitor_table->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
  monitor_table->setContextMenuPolicy(Qt::CustomContextMenu);
  bottom_layout->addWidget(monitor_table);
  layout->addWidget(bottom_group, 3);

  QObject::connect(monitor_type_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int idx) {
    const bool ros_list_mode = (idx >= 1 && idx <= 4);
    const bool param_mode = (idx == 5);
    ros_filter_label->setVisible(ros_list_mode || param_mode);
    ros_filter_edit->setVisible(ros_list_mode || param_mode);
    param_node_label->setVisible(param_mode);
    param_node_combo->setVisible(param_mode);
  });

  auto stop_flag = std::make_shared<std::atomic<bool>>(false);
  auto active_flag = std::make_shared<std::atomic<bool>>(false);
  auto selected_param_node = std::make_shared<std::string>("");
  auto selected_param_node_mutex = std::make_shared<std::mutex>();
  auto force_refresh = std::make_shared<std::atomic<bool>>(true);
  auto param_scan_cancel = std::make_shared<std::atomic_bool>(false);

  QObject::connect(param_node_combo, &QComboBox::currentTextChanged, [=](const QString & text) {
    {
      std::lock_guard<std::mutex> lk(*selected_param_node_mutex);
      *selected_param_node = text.toStdString();
    }
    force_refresh->store(true);
  });
  QObject::connect(monitor_type_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int idx) {
    // 切离“节点参数”时，优先请求中断参数扫描，避免卡住后续类型刷新。
    if (idx != 5) {
      param_scan_cancel->store(true);
    } else {
      param_scan_cancel->store(false);
    }
    force_refresh->store(true);
  });
  QObject::connect(ros_filter_edit, &QLineEdit::textChanged, [=](const QString &) {
    force_refresh->store(true);
  });

  auto shell_quote = [](const QString & raw) {
    QString escaped = raw;
    escaped.replace("'", "'\\''");
    return "'" + escaped + "'";
  };
  QObject::connect(monitor_table, &QWidget::customContextMenuRequested, [=](const QPoint & pos) {
    const int row = monitor_table->rowAt(pos.y());
    if (row < 0) {
      return;
    }
    const int monitor_type = monitor_type_combo->currentIndex();
    if (monitor_type != 0 && monitor_type != 1) {
      return;  // 仅进程/节点支持结束
    }
    QMenu menu(monitor_table);
    QAction * kill_action = menu.addAction(monitor_type == 0 ? "结束进程" : "结束节点");
    QAction * chosen = menu.exec(monitor_table->viewport()->mapToGlobal(pos));
    if (chosen != kill_action) {
      return;
    }

    if (monitor_type == 0) {
      const QTableWidgetItem * pid_item = monitor_table->item(row, 0);
      if (!pid_item) {
        return;
      }
      const QString pid = pid_item->text().trimmed();
      if (pid.isEmpty()) {
        return;
      }
      const auto reply = QMessageBox::question(
        this, "确认", QString("确定结束进程 PID=%1 ?").arg(pid),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
      if (reply != QMessageBox::Yes) {
        return;
      }
      const QString cmd = QString("kill -TERM %1 2>/dev/null || kill -9 %1 2>/dev/null").arg(pid);
      QProcess p;
      p.start("bash", {"-lc", cmd});
      p.waitForFinished(1200);
      force_refresh->store(true);
      return;
    }

    // ROS2 节点：优先按 __node:= 名称匹配，回退按节点名匹配
    const QTableWidgetItem * node_item = monitor_table->item(row, 0);
    if (!node_item) {
      return;
    }
    const QString node = node_item->text().trimmed();
    if (node.isEmpty()) {
      return;
    }
    const auto reply = QMessageBox::question(
      this, "确认", QString("确定结束 ROS2 节点 %1 ?").arg(node),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (reply != QMessageBox::Yes) {
      return;
    }
    const QString node_q = shell_quote(node);
    const QString cmd = QString(
      "pkill -f \"__node:=%1\" 2>/dev/null || pkill -f %2 2>/dev/null")
      .arg(node, node_q);
    QProcess p;
    p.start("bash", {"-lc", cmd});
    p.waitForFinished(1200);
    force_refresh->store(true);
  });

  auto top_worker = std::make_shared<std::thread>([=]() {
    unsigned long long prev_idle = 0, prev_total = 0, prev_rx = 0, prev_tx = 0, prev_disk_read = 0, prev_disk_write = 0;
    ReadCpuCounters(prev_idle, prev_total);
    ReadNetworkBytes(prev_rx, prev_tx);
    ReadDiskBytes(prev_disk_read, prev_disk_write);
    auto last_t = std::chrono::steady_clock::now();
    while (!stop_flag->load()) {
      if (!active_flag->load()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue; }
      std::chrono::steady_clock::time_point now_t;
      const ResourceUsage usage = ReadResourceUsage(prev_idle, prev_total, prev_rx, prev_tx, prev_disk_read, prev_disk_write, last_t, now_t);
      last_t = now_t;
      QMetaObject::invokeMethod(this, [=]() {
        cpu_bar->setValue(usage.cpu);
        mem_bar->setValue(usage.mem);
        gpu_bar->setValue(usage.gpu);
        vram_bar->setValue(usage.vram);
        cpu_txt->setText(QString("%1%").arg(usage.cpu));
        mem_txt->setText(QString("%1%").arg(usage.mem));
        gpu_txt->setText(QString("%1%").arg(usage.gpu));
        vram_txt->setText(QString("%1%").arg(usage.vram));
        auto to_bar = [](double kbps) {
          const double max_kbps = 10240.0;
          double p = (kbps / max_kbps) * 100.0;
          if (p < 0.0) p = 0.0;
          if (p > 100.0) p = 100.0;
          return static_cast<int>(p);
        };
        auto format_rate = [](double kbps) {
          if (kbps >= 1024.0) { return QString("%1 MB/s").arg(kbps / 1024.0, 0, 'f', 2); }
          return QString("%1 KB/s").arg(kbps, 0, 'f', 1);
        };
        disk_read_bar->setValue(to_bar(usage.disk_read_kbps));
        disk_write_bar->setValue(to_bar(usage.disk_write_kbps));
        up_bar->setValue(to_bar(usage.up_kbps));
        down_bar->setValue(to_bar(usage.down_kbps));
        disk_read_txt->setText(format_rate(usage.disk_read_kbps));
        disk_write_txt->setText(format_rate(usage.disk_write_kbps));
        up_txt->setText(format_rate(usage.up_kbps));
        down_txt->setText(format_rate(usage.down_kbps));
      }, Qt::QueuedConnection);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  auto bottom_worker = std::make_shared<std::thread>([=]() {
    auto last_param_refresh_t = std::chrono::steady_clock::now() - std::chrono::seconds(2);
    std::vector<QString> cached_node_options;
    std::vector<ParamRow> cached_param_rows;
    std::string cached_param_node_name;
    while (!stop_flag->load()) {
      if (!active_flag->load()) { std::this_thread::sleep_for(std::chrono::milliseconds(200)); continue; }
      const int monitor_type = monitor_type_combo->currentIndex();
      const bool refresh_requested = force_refresh->exchange(false);
      std::string param_node_name;
      {
        std::lock_guard<std::mutex> lk(*selected_param_node_mutex);
        param_node_name = *selected_param_node;
      }
      const std::vector<ProcessRow> proc_rows = (monitor_type == 0) ? ReadTopProcessRows() : std::vector<ProcessRow>{};
      const std::vector<NodeInfoRow> node_rows = (monitor_type == 1) ? ReadRos2NodeInfoRows() : std::vector<NodeInfoRow>{};
      std::vector<QString> node_options;
      const std::vector<TopicTypeRow> topic_rows = (monitor_type == 2) ? ReadRos2TopicTypeRows() : std::vector<TopicTypeRow>{};
      const std::vector<QString> service_rows = (monitor_type == 3) ? ReadRos2SimpleList("ros2 service list") : std::vector<QString>{};
      const std::vector<QString> action_rows = (monitor_type == 4) ? ReadRos2SimpleList("ros2 action list") : std::vector<QString>{};
      std::vector<ParamRow> param_rows;
      if (monitor_type == 5) {
        param_scan_cancel->store(false);
        const auto now = std::chrono::steady_clock::now();
        const bool node_changed = (param_node_name != cached_param_node_name);
        const bool interval_reached = (now - last_param_refresh_t) >= std::chrono::milliseconds(1200);
        if (refresh_requested || node_changed || interval_reached) {
          cached_node_options = ReadRos2NodeRows();
          cached_param_rows = ReadRos2ParamRows(QString::fromStdString(param_node_name), param_scan_cancel.get(), 600);
          cached_param_node_name = param_node_name;
          last_param_refresh_t = now;
        }
        node_options = cached_node_options;
        param_rows = cached_param_rows;
      } else {
        param_scan_cancel->store(true);
      }

      QMetaObject::invokeMethod(this, [=]() {
        if (monitor_type == 5) {
          const QString current_node = param_node_combo->currentText();
          param_node_combo->blockSignals(true);
          param_node_combo->clear();
          for (const auto & n : node_options) { param_node_combo->addItem(n); }
          int idx = param_node_combo->findText(current_node);
          if (idx >= 0) { param_node_combo->setCurrentIndex(idx); }
          else if (!current_node.isEmpty()) { param_node_combo->setEditText(current_node); }
          else if (!node_options.empty()) { param_node_combo->setCurrentIndex(0); }
          param_node_combo->blockSignals(false);
          {
            std::lock_guard<std::mutex> lk(*selected_param_node_mutex);
            *selected_param_node = param_node_combo->currentText().toStdString();
          }
        }
        monitor_table->clearContents();
        monitor_table->setSortingEnabled(false);
        if (monitor_type == 0) {
          monitor_table->setColumnCount(4);
          monitor_table->setHorizontalHeaderLabels({"PID", "CPU%", "MEM%", "COMMAND"});
          monitor_table->setRowCount(static_cast<int>(proc_rows.size()));
          for (int i = 0; i < static_cast<int>(proc_rows.size()); ++i) {
            auto * pid_item = new QTableWidgetItem(proc_rows[i].pid);
            auto * cpu_item = new QTableWidgetItem(proc_rows[i].cpu);
            auto * mem_item = new QTableWidgetItem(proc_rows[i].mem);
            auto * cmd_item = new QTableWidgetItem(proc_rows[i].command);
            pid_item->setToolTip(proc_rows[i].pid);
            cpu_item->setToolTip(proc_rows[i].cpu);
            mem_item->setToolTip(proc_rows[i].mem);
            cmd_item->setToolTip(proc_rows[i].command);
            monitor_table->setItem(i, 0, pid_item);
            monitor_table->setItem(i, 1, cpu_item);
            monitor_table->setItem(i, 2, mem_item);
            monitor_table->setItem(i, 3, cmd_item);
          }
          monitor_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
          monitor_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
          monitor_table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
          monitor_table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
        } else if (monitor_type == 2) {
          const QString keyword = ros_filter_edit->text().trimmed();
          std::vector<TopicTypeRow> filtered_topics;
          for (const auto & t : topic_rows) {
            if (keyword.isEmpty() || t.topic.contains(keyword, Qt::CaseInsensitive) ||
                t.type.contains(keyword, Qt::CaseInsensitive) || t.hz.contains(keyword, Qt::CaseInsensitive)) {
              filtered_topics.push_back(t);
            }
          }
          monitor_table->setColumnCount(3);
          monitor_table->setHorizontalHeaderLabels({"ROS2 Topic", "Topic Type", "发布频率"});
          monitor_table->setRowCount(static_cast<int>(filtered_topics.size()));
          for (int i = 0; i < static_cast<int>(filtered_topics.size()); ++i) {
            auto * topic_item = new QTableWidgetItem(filtered_topics[i].topic);
            auto * type_item = new QTableWidgetItem(filtered_topics[i].type);
            auto * hz_item = new QTableWidgetItem(filtered_topics[i].hz);
            topic_item->setToolTip(filtered_topics[i].topic);
            type_item->setToolTip(filtered_topics[i].type);
            hz_item->setToolTip(QString("当前窗口采样估计值：%1").arg(filtered_topics[i].hz));
            monitor_table->setItem(i, 0, topic_item);
            monitor_table->setItem(i, 1, type_item);
            monitor_table->setItem(i, 2, hz_item);
          }
          monitor_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
          monitor_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
          monitor_table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        } else if (monitor_type == 5) {
          const QString keyword = ros_filter_edit->text().trimmed();
          const double param_refresh_hz = 1000.0 / 1200.0;
          const QString refresh_rate_text = QString("%1 Hz").arg(param_refresh_hz, 0, 'f', 2);
          std::vector<ParamRow> filtered_params;
          for (const auto & p : param_rows) {
            if (keyword.isEmpty() || p.name.contains(keyword, Qt::CaseInsensitive) || p.value.contains(keyword, Qt::CaseInsensitive)) {
              filtered_params.push_back(p);
            }
          }
          monitor_table->setColumnCount(3);
          monitor_table->setHorizontalHeaderLabels({"Parameter", "Value", "刷新频率"});
          monitor_table->setRowCount(static_cast<int>(filtered_params.size()));
          for (int i = 0; i < static_cast<int>(filtered_params.size()); ++i) {
            auto * name_item = new QTableWidgetItem(filtered_params[i].name);
            auto * value_item = new QTableWidgetItem(filtered_params[i].value);
            auto * rate_item = new QTableWidgetItem(refresh_rate_text);
            name_item->setToolTip(filtered_params[i].name);
            value_item->setToolTip(filtered_params[i].value);
            rate_item->setToolTip("节点参数轮询刷新频率");
            monitor_table->setItem(i, 0, name_item);
            monitor_table->setItem(i, 1, value_item);
            monitor_table->setItem(i, 2, rate_item);
          }
          monitor_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
          monitor_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
          monitor_table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        } else if (monitor_type == 1) {
          const QString keyword = ros_filter_edit->text().trimmed();
          std::vector<NodeInfoRow> filtered_nodes;
          for (const auto & n : node_rows) {
            if (keyword.isEmpty() || n.name.contains(keyword, Qt::CaseInsensitive) ||
                n.startup_file.contains(keyword, Qt::CaseInsensitive)) {
              filtered_nodes.push_back(n);
            }
          }
          monitor_table->setColumnCount(2);
          monitor_table->setHorizontalHeaderLabels({"ROS2 Node", "启动文件"});
          monitor_table->setRowCount(static_cast<int>(filtered_nodes.size()));
          for (int i = 0; i < static_cast<int>(filtered_nodes.size()); ++i) {
            auto * node_item = new QTableWidgetItem(filtered_nodes[i].name);
            auto * file_item = new QTableWidgetItem(filtered_nodes[i].startup_file);
            node_item->setToolTip(filtered_nodes[i].name);
            file_item->setToolTip(filtered_nodes[i].startup_file);
            monitor_table->setItem(i, 0, node_item);
            monitor_table->setItem(i, 1, file_item);
          }
          monitor_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
          monitor_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
        } else {
          const QString keyword = ros_filter_edit->text().trimmed();
          const std::vector<QString> source_rows = (monitor_type == 3 ? service_rows : action_rows);
          std::vector<QString> filtered_rows;
          for (const auto & r : source_rows) {
            if (keyword.isEmpty() || r.contains(keyword, Qt::CaseInsensitive)) { filtered_rows.push_back(r); }
          }
          monitor_table->setColumnCount(1);
          monitor_table->setHorizontalHeaderLabels({"Name"});
          monitor_table->setRowCount(static_cast<int>(filtered_rows.size()));
          for (int i = 0; i < static_cast<int>(filtered_rows.size()); ++i) {
            auto * item = new QTableWidgetItem(filtered_rows[i]);
            item->setToolTip(filtered_rows[i]);
            monitor_table->setItem(i, 0, item);
          }
          monitor_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
        }
        monitor_table->setSortingEnabled(true);
      }, Qt::QueuedConnection);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  QObject::connect(this, &QObject::destroyed, [stop_flag, top_worker, bottom_worker](QObject *) {
    stop_flag->store(true);
    if (top_worker->joinable()) { top_worker->detach(); }
    if (bottom_worker->joinable()) { bottom_worker->detach(); }
  });

  set_active_ = [active_flag](bool v) { active_flag->store(v); };
}

void SystemStatusWidget::SetActive(bool active)
{
  if (set_active_) { set_active_(active); }
}

}  // namespace ros_robot_assist_tools::ui
