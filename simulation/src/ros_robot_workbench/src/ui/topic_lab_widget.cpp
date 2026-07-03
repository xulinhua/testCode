#include "ros_robot_workbench/ui/topic_lab_widget.h"

#include <QAbstractItemView>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSplitter>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include "ros_robot_workbench/module/calibration_module.h"

namespace ros_robot_workbench::ui
{
namespace
{

void FillTopicTable(QTableWidget * table, const std::vector<TopicInfoRow> & rows, const QString & filter)
{
  if (!table) {
    return;
  }
  table->setRowCount(0);
  const QString f = filter.trimmed();
  int out_row = 0;
  for (const auto & r : rows) {
    if (!f.isEmpty() && !r.name.contains(f, Qt::CaseInsensitive) && !r.type.contains(f, Qt::CaseInsensitive)) {
      continue;
    }
    table->insertRow(out_row);
    table->setItem(out_row, 0, new QTableWidgetItem(r.name));
    table->setItem(out_row, 1, new QTableWidgetItem(r.type));
    ++out_row;
  }
}

QString SelectedTopicName(const QTableWidget * table)
{
  const int row = table ? table->currentRow() : -1;
  if (row < 0) {
    return {};
  }
  const QTableWidgetItem * item = table->item(row, 0);
  return item ? item->text() : QString();
}

}  // namespace

void TopicLabWidget::reloadTopics()
{
  QString err;
  if (!ListOnlineTopics(&cached_topics_, &err)) {
    QMessageBox::warning(this, "刷新失败", err);
    return;
  }
  FillTopicTable(topic_table_, cached_topics_, filter_edit_ ? filter_edit_->text() : QString());
}

TopicLabWidget::TopicLabWidget(QWidget * parent)
: QWidget(parent)
, dm_()
, cached_topics_()
, topic_table_(nullptr)
, filter_edit_(nullptr)
{
  dm_.SetConfigPath(ResolveDefaultConfigYamlPath("topic_lab.yaml").toStdString());
  dm_.Load();

  QVBoxLayout * root = new QVBoxLayout(this);
  root->setContentsMargins(8, 6, 8, 6);

  QLabel * title = new QLabel("话题调试");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QLabel * hint = new QLabel(TopicLabModuleSummary());
  hint->setWordWrap(true);
  hint->setStyleSheet("color:#445566;font-size:12px;");
  root->addWidget(hint);

  QHBoxLayout * filter_row = new QHBoxLayout();
  filter_edit_ = new QLineEdit();
  filter_edit_->setPlaceholderText("按 topic 名或类型过滤…");
  QPushButton * refresh = new QPushButton("刷新列表");
  filter_row->addWidget(filter_edit_, 1);
  filter_row->addWidget(refresh);
  root->addLayout(filter_row);

  QSplitter * split = new QSplitter(Qt::Horizontal);
  topic_table_ = new QTableWidget(0, 2);
  topic_table_->setHorizontalHeaderLabels({"Topic", "类型"});
  topic_table_->horizontalHeader()->setStretchLastSection(true);
  topic_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  topic_table_->setSelectionMode(QAbstractItemView::SingleSelection);
  split->addWidget(topic_table_);

  QWidget * right = new QWidget();
  QVBoxLayout * right_l = new QVBoxLayout(right);
  QLineEdit * active_topic = new QLineEdit(QString::fromStdString(dm_.GetDefaultTopic()));
  QFormLayout * active_form = new QFormLayout();
  active_form->addRow("当前 topic:", active_topic);
  right_l->addLayout(active_form);

  QLabel * hz_label = new QLabel("Hz: --");
  right_l->addWidget(hz_label);

  QPlainTextEdit * echo = new QPlainTextEdit();
  echo->setReadOnly(true);
  echo->setPlaceholderText("Echo 输出…");
  right_l->addWidget(echo, 1);

  QHBoxLayout * echo_btns = new QHBoxLayout();
  QPushButton * btn_echo = new QPushButton("Echo 一次");
  QPushButton * btn_hz = new QPushButton("测 Hz");
  echo_btns->addWidget(btn_echo);
  echo_btns->addWidget(btn_hz);
  echo_btns->addStretch();
  right_l->addLayout(echo_btns);

  QGroupBox * pub_group = new QGroupBox("发布 std_msgs/String");
  QVBoxLayout * pub_l = new QVBoxLayout(pub_group);
  QLineEdit * pub_data = new QLineEdit("hello");
  QPushButton * btn_pub = new QPushButton("单次发布");
  pub_l->addWidget(pub_data);
  pub_l->addWidget(btn_pub);
  right_l->addWidget(pub_group);

  split->addWidget(right);
  split->setStretchFactor(0, 2);
  split->setStretchFactor(1, 3);
  root->addWidget(split, 1);

  QObject::connect(refresh, &QPushButton::clicked, [this]() { reloadTopics(); });
  QObject::connect(filter_edit_, &QLineEdit::textChanged, [this](const QString &) {
    FillTopicTable(topic_table_, cached_topics_, filter_edit_->text());
  });
  QObject::connect(topic_table_, &QTableWidget::itemSelectionChanged, [this, active_topic]() {
    const QString t = SelectedTopicName(topic_table_);
    if (!t.isEmpty()) {
      active_topic->setText(t);
    }
  });

  QObject::connect(btn_echo, &QPushButton::clicked, [this, active_topic, echo]() {
    QString out;
    QString err;
    if (!EchoTopicOnce(active_topic->text(), &out, &err)) {
      QMessageBox::warning(this, "Echo 失败", err);
      return;
    }
    echo->setPlainText(out);
  });

  QObject::connect(btn_hz, &QPushButton::clicked, [this, active_topic, hz_label]() {
    double hz = 0.0;
    QString err;
    if (!QueryTopicHz(active_topic->text(), &hz, &err, dm_.GetHzSampleSec())) {
      QMessageBox::warning(this, "测 Hz 失败", err);
      hz_label->setText("Hz: --");
      return;
    }
    hz_label->setText(QString("Hz: %1").arg(hz, 0, 'f', 2));
  });

  QObject::connect(btn_pub, &QPushButton::clicked, [this, active_topic, pub_data]() {
    QString err;
    if (!PublishStringOnce(active_topic->text(), pub_data->text(), &err)) {
      QMessageBox::warning(this, "发布失败", err);
      return;
    }
    QMessageBox::information(this, "发布", "已发送一条 std_msgs/String");
  });

  reloadTopics();
}

}  // namespace ros_robot_workbench::ui
