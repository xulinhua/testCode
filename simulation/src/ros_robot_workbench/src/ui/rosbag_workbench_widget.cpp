#include "ros_robot_workbench/ui/rosbag_workbench_widget.h"

#include <QCheckBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include "ros_robot_workbench/manage/rosbag_workbench_data_manager.hpp"
#include "ros_robot_workbench/module/calibration_module.h"
#include "ros_robot_workbench/module/rosbag_workbench_module.h"
#include "ros_robot_workbench/module/topic_lab_module.h"

namespace ros_robot_workbench::ui
{
namespace
{

QString ExpandHomePath(QString path)
{
  if (path.startsWith("~/")) {
    path = QDir::homePath() + path.mid(1);
  }
  return path;
}

void StartRos2Command(QProcess * proc, const QString & cmd, QPlainTextEdit * log)
{
  if (!proc) {
    return;
  }
  if (proc->state() != QProcess::NotRunning) {
    proc->terminate();
    proc->waitForFinished(2000);
  }
  const QString shell = QString("source /opt/ros/humble/setup.bash >/dev/null 2>&1 && %1").arg(cmd);
  log->appendPlainText("执行: " + cmd);
  proc->setProgram("bash");
  proc->setArguments({"-lc", shell});
  proc->start();
}

void FillTopicPickTable(QTableWidget * table, const std::vector<TopicInfoRow> & rows, const QStringList & checked)
{
  table->setRowCount(static_cast<int>(rows.size()));
  for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
    const auto & r = rows[static_cast<size_t>(i)];
    auto * pick = new QTableWidgetItem();
    pick->setCheckState(checked.contains(r.name) ? Qt::Checked : Qt::Unchecked);
    table->setItem(i, 0, pick);
    table->setItem(i, 1, new QTableWidgetItem(r.name));
    table->setItem(i, 2, new QTableWidgetItem(r.type));
  }
}

std::vector<QString> SelectedTopicsFromTable(const QTableWidget * table)
{
  std::vector<QString> out;
  if (!table) {
    return out;
  }
  for (int i = 0; i < table->rowCount(); ++i) {
    if (table->item(i, 0) && table->item(i, 0)->checkState() == Qt::Checked) {
      out.push_back(table->item(i, 1)->text());
    }
  }
  return out;
}

}  // namespace

RosbagWorkbenchWidget::RosbagWorkbenchWidget(QWidget * parent)
: QWidget(parent)
, record_proc_(new QProcess(this))
, play_proc_(new QProcess(this))
{
  manage::RosbagWorkbenchDataManager dm;
  dm.SetConfigPath(ResolveDefaultConfigYamlPath("rosbag_workbench.yaml").toStdString());
  dm.Load();

  QVBoxLayout * root = new QVBoxLayout(this);
  QLabel * title = new QLabel("Rosbag 工作台");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);
  QLabel * hint = new QLabel(RosbagWorkbenchModuleSummary());
  hint->setWordWrap(true);
  hint->setStyleSheet("color:#445566;font-size:12px;");
  root->addWidget(hint);

  QPlainTextEdit * log = new QPlainTextEdit();
  log->setReadOnly(true);
  log->setMaximumHeight(120);

  QTabWidget * tabs = new QTabWidget();

  // --- 录制 ---
  QWidget * rec_page = new QWidget();
  QVBoxLayout * rec_l = new QVBoxLayout(rec_page);
  QLineEdit * out_dir = new QLineEdit(ExpandHomePath(QString::fromStdString(dm.GetOutputDir())));
  QLineEdit * prefix = new QLineEdit(QString::fromStdString(dm.GetDefaultPrefix()));
  QFormLayout * rec_form = new QFormLayout();
  rec_form->addRow("输出目录:", out_dir);
  rec_form->addRow("文件名前缀:", prefix);
  rec_l->addLayout(rec_form);
  QTableWidget * rec_topics = new QTableWidget(0, 3);
  rec_topics->setHorizontalHeaderLabels({"选", "Topic", "类型"});
  rec_topics->horizontalHeader()->setStretchLastSection(true);
  rec_l->addWidget(rec_topics, 1);
  QHBoxLayout * rec_btns = new QHBoxLayout();
  QPushButton * refresh_topics = new QPushButton("刷新 Topic 列表");
  QPushButton * start_rec = new QPushButton("开始录制");
  QPushButton * stop_rec = new QPushButton("停止录制");
  rec_btns->addWidget(refresh_topics);
  rec_btns->addWidget(start_rec);
  rec_btns->addWidget(stop_rec);
  rec_btns->addStretch();
  rec_l->addLayout(rec_btns);
  tabs->addTab(rec_page, "录制");

  // --- 浏览 ---
  QWidget * browse_page = new QWidget();
  QVBoxLayout * browse_l = new QVBoxLayout(browse_page);
  QLineEdit * bag_path = new QLineEdit();
  QPushButton * browse_bag = new QPushButton("浏览…");
  QWidget * bag_row = new QWidget();
  QHBoxLayout * bag_row_l = new QHBoxLayout(bag_row);
  bag_row_l->setContentsMargins(0, 0, 0, 0);
  bag_row_l->addWidget(bag_path, 1);
  bag_row_l->addWidget(browse_bag);
  browse_l->addWidget(bag_row);
  QTableWidget * meta_table = new QTableWidget(0, 3);
  meta_table->setHorizontalHeaderLabels({"Topic", "类型", "消息数"});
  meta_table->horizontalHeader()->setStretchLastSection(true);
  browse_l->addWidget(meta_table, 1);
  QLabel * meta_summary = new QLabel("时长: --");
  browse_l->addWidget(meta_summary);
  QPushButton * load_meta = new QPushButton("解析 metadata");
  browse_l->addWidget(load_meta);
  tabs->addTab(browse_page, "浏览");

  // --- 回放 ---
  QWidget * play_page = new QWidget();
  QVBoxLayout * play_l = new QVBoxLayout(play_page);
  QLineEdit * play_bag = new QLineEdit();
  QDoubleSpinBox * play_rate = new QDoubleSpinBox();
  play_rate->setRange(0.01, 10.0);
  play_rate->setValue(dm.GetPlayRate());
  QCheckBox * play_loop = new QCheckBox("循环播放");
  play_loop->setChecked(dm.GetPlayLoop());
  QCheckBox * use_sim = new QCheckBox("发布 /clock (仿真时间)");
  use_sim->setChecked(dm.GetUseSimTime());
  QFormLayout * play_form = new QFormLayout();
  play_form->addRow("Bag 目录:", play_bag);
  play_form->addRow("速率:", play_rate);
  play_form->addRow("", play_loop);
  play_form->addRow("", use_sim);
  play_l->addLayout(play_form);
  QHBoxLayout * play_btns = new QHBoxLayout();
  QPushButton * start_play = new QPushButton("播放");
  QPushButton * stop_play = new QPushButton("停止");
  play_btns->addWidget(start_play);
  play_btns->addWidget(stop_play);
  play_btns->addStretch();
  play_l->addLayout(play_btns);
  tabs->addTab(play_page, "回放");

  root->addWidget(tabs, 1);
  root->addWidget(log);

  QObject::connect(record_proc_, &QProcess::readyReadStandardOutput, [this, log]() {
    log->appendPlainText(QString::fromUtf8(record_proc_->readAllStandardOutput()).trimmed());
  });
  QObject::connect(record_proc_, &QProcess::readyReadStandardError, [this, log]() {
    log->appendPlainText(QString::fromUtf8(record_proc_->readAllStandardError()).trimmed());
  });
  QObject::connect(record_proc_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), [log](int code) {
    log->appendPlainText(QString("录制进程结束, exit=%1").arg(code));
  });
  QObject::connect(play_proc_, &QProcess::readyReadStandardOutput, [this, log]() {
    log->appendPlainText(QString::fromUtf8(play_proc_->readAllStandardOutput()).trimmed());
  });
  QObject::connect(play_proc_, &QProcess::readyReadStandardError, [this, log]() {
    log->appendPlainText(QString::fromUtf8(play_proc_->readAllStandardError()).trimmed());
  });
  QObject::connect(play_proc_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), [log](int code) {
    log->appendPlainText(QString("回放进程结束, exit=%1").arg(code));
  });

  QStringList default_checked;
  for (const auto & t : dm.GetRecordTopics()) {
    default_checked << QString::fromStdString(t);
  }

  QObject::connect(refresh_topics, &QPushButton::clicked, [this, rec_topics, default_checked, log]() {
    std::vector<TopicInfoRow> rows;
    QString err;
    if (!ListOnlineTopics(&rows, &err)) {
      QMessageBox::warning(this, "刷新失败", err);
      return;
    }
    FillTopicPickTable(rec_topics, rows, default_checked);
    log->appendPlainText(QString("已加载 %1 个 topic").arg(rows.size()));
  });

  QObject::connect(start_rec, &QPushButton::clicked, [this, out_dir, prefix, rec_topics, log]() {
    const auto topics = SelectedTopicsFromTable(rec_topics);
    if (topics.empty()) {
      QMessageBox::warning(this, "录制", "请至少选择一个 topic");
      return;
    }
    const QString uri = DefaultBagOutputUri(ExpandHomePath(out_dir->text()), prefix->text());
    const QString cmd = BuildBagRecordCommand(uri, topics);
    StartRos2Command(record_proc_, cmd, log);
  });

  QObject::connect(stop_rec, &QPushButton::clicked, [this, log]() {
    if (record_proc_->state() != QProcess::NotRunning) {
      record_proc_->terminate();
      log->appendPlainText("已请求停止录制");
    }
  });

  QObject::connect(browse_bag, &QPushButton::clicked, [this, bag_path, play_bag]() {
    const QString d = QFileDialog::getExistingDirectory(this, "选择 rosbag2 目录", bag_path->text());
    if (!d.isEmpty()) {
      bag_path->setText(d);
      play_bag->setText(d);
    }
  });

  QObject::connect(load_meta, &QPushButton::clicked, [this, bag_path, meta_table, meta_summary, log]() {
    BagMetadata meta;
    QString err;
    if (!ParseBagMetadata(bag_path->text(), &meta, &err)) {
      QMessageBox::warning(this, "解析失败", err);
      return;
    }
    meta_table->setRowCount(static_cast<int>(meta.topics.size()));
    for (int i = 0; i < static_cast<int>(meta.topics.size()); ++i) {
      const auto & t = meta.topics[static_cast<size_t>(i)];
      meta_table->setItem(i, 0, new QTableWidgetItem(t.name));
      meta_table->setItem(i, 1, new QTableWidgetItem(t.type));
      meta_table->setItem(i, 2, new QTableWidgetItem(QString::number(t.message_count)));
    }
    meta_summary->setText(QString("时长: %1 s, topic 数: %2").arg(meta.duration_sec, 0, 'f', 2).arg(meta.topics.size()));
    log->appendPlainText("metadata 解析完成");
  });

  QObject::connect(start_play, &QPushButton::clicked, [this, play_bag, play_rate, play_loop, use_sim, log]() {
    const QString cmd = BuildBagPlayCommand(
      play_bag->text(), play_rate->value(), play_loop->isChecked(), use_sim->isChecked());
    StartRos2Command(play_proc_, cmd, log);
  });

  QObject::connect(stop_play, &QPushButton::clicked, [this, log]() {
    if (play_proc_->state() != QProcess::NotRunning) {
      play_proc_->terminate();
      log->appendPlainText("已请求停止回放");
    }
  });

  // 初始加载 topic 列表
  std::vector<TopicInfoRow> rows;
  QString err;
  if (ListOnlineTopics(&rows, &err)) {
    FillTopicPickTable(rec_topics, rows, default_checked);
  }
}

RosbagWorkbenchWidget::~RosbagWorkbenchWidget()
{
  if (record_proc_ && record_proc_->state() != QProcess::NotRunning) {
    record_proc_->terminate();
    record_proc_->waitForFinished(2000);
  }
  if (play_proc_ && play_proc_->state() != QProcess::NotRunning) {
    play_proc_->terminate();
    play_proc_->waitForFinished(2000);
  }
}

}  // namespace ros_robot_workbench::ui
