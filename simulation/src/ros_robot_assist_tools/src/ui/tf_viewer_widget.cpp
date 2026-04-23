#include "ros_robot_assist_tools/ui/tf_viewer_widget.h"

#include <memory>
#include <atomic>

#include <QCheckBox>
#include <QDateTime>
#include <QFontDatabase>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMetaObject>
#include <QPointer>
#include <QPushButton>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

#include "ros_robot_assist_tools/module/tf_viewer_module.h"
#include "ros_robot_assist_tools/ui/shared_refresh_pool.h"

namespace ros_robot_assist_tools::ui
{

TfViewerWidget::TfViewerWidget(QWidget * parent)
: QWidget(parent)
{
  auto backend = std::make_shared<TfViewerBackend>();
  auto snapshot = std::make_shared<TfViewerSnapshot>();

  QVBoxLayout * root = new QVBoxLayout(this);
  QLabel * title = new QLabel("TF 查看");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QHBoxLayout * action_row = new QHBoxLayout();
  QPushButton * refresh_btn = new QPushButton("刷新 TF 树");
  QCheckBox * auto_refresh = new QCheckBox("自动刷新(1s)");
  auto_refresh->setChecked(false);
  QLabel * status = new QLabel("等待 TF 数据...");
  action_row->addWidget(refresh_btn);
  action_row->addWidget(auto_refresh);
  action_row->addSpacing(10);
  action_row->addWidget(status, 1);
  root->addLayout(action_row);

  QWidget * table_panel = new QWidget();
  QHBoxLayout * panel_layout = new QHBoxLayout(table_panel);

  QGroupBox * all_group = new QGroupBox("全部 Frame");
  QVBoxLayout * all_layout = new QVBoxLayout(all_group);
  QTableWidget * all_table = new QTableWidget();
  all_table->setColumnCount(2);
  all_table->setHorizontalHeaderLabels({"Frame ID", "类型/频率"});
  all_table->setSelectionBehavior(QAbstractItemView::SelectRows);
  all_table->setSelectionMode(QAbstractItemView::SingleSelection);
  all_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  all_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  all_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  all_layout->addWidget(all_table);

  QGroupBox * related_group = new QGroupBox("与选中 Frame 关联的 Frame");
  QVBoxLayout * related_layout = new QVBoxLayout(related_group);
  QTableWidget * related_table = new QTableWidget();
  related_table->setColumnCount(2);
  related_table->setHorizontalHeaderLabels({"Related Frame ID", "类型/频率"});
  related_table->setSelectionBehavior(QAbstractItemView::SelectRows);
  related_table->setSelectionMode(QAbstractItemView::SingleSelection);
  related_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  related_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  related_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  related_layout->addWidget(related_table);

  panel_layout->addWidget(all_group, 1);
  panel_layout->addWidget(related_group, 1);
  root->addWidget(table_panel, 2);

  QGroupBox * transform_group = new QGroupBox("两 Frame 变换矩阵与实际意义");
  QVBoxLayout * transform_layout = new QVBoxLayout(transform_group);
  QLabel * pair_label = new QLabel("当前选择: (未选择)");
  QTextEdit * matrix_view = new QTextEdit();
  QTextEdit * meaning_view = new QTextEdit();
  matrix_view->setReadOnly(true);
  meaning_view->setReadOnly(true);
  matrix_view->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  matrix_view->setLineWrapMode(QTextEdit::NoWrap);
  matrix_view->setMaximumHeight(140);
  meaning_view->setMinimumHeight(180);
  transform_layout->addWidget(pair_label);
  transform_layout->addWidget(new QLabel("4x4 变换矩阵"));
  transform_layout->addWidget(matrix_view);
  transform_layout->addWidget(new QLabel("实际意义（位置偏差 / 角度偏差）"));
  transform_layout->addWidget(meaning_view, 1);
  root->addWidget(transform_group, 3);

  auto refresh_related_and_transform = [=]() {
    const int all_row = all_table->currentRow();
    if (all_row < 0 || all_row >= all_table->rowCount()) {
      related_table->setRowCount(0);
      pair_label->setText("当前选择: (未选择)");
      matrix_view->clear();
      meaning_view->clear();
      return;
    }
    const QString from = all_table->item(all_row, 0)->text();
    const auto related = backend->ConnectedFrames(*snapshot, from);
    related_table->setRowCount(static_cast<int>(related.size()));
    for (int i = 0; i < static_cast<int>(related.size()); ++i) {
      auto * frame_item = new QTableWidgetItem(related[i]);
      auto * type_item = new QTableWidgetItem(backend->FrameTypeText(*snapshot, related[i]));
      related_table->setItem(i, 0, frame_item);
      related_table->setItem(i, 1, type_item);
    }

    const int rel_row = related_table->currentRow();
    if (rel_row < 0 || rel_row >= related_table->rowCount()) {
      pair_label->setText(QString("当前选择: %1 -> (请在右侧选择目标)").arg(from));
      matrix_view->clear();
      meaning_view->clear();
      return;
    }
    const QString to = related_table->item(rel_row, 0)->text();
    pair_label->setText(QString("当前选择: %1 -> %2").arg(from).arg(to));
    QString mat, meaning, err;
    if (backend->BuildTransformReport(from, to, &mat, &meaning, &err)) {
      matrix_view->setPlainText(mat);
      meaning_view->setPlainText(meaning);
      status->setText(QString("已更新 %1 -> %2").arg(from).arg(to));
    } else {
      matrix_view->clear();
      meaning_view->setPlainText(err);
      status->setText(err);
    }
  };

  auto refresh_frames = [=]() {
    const QString selected = (all_table->currentRow() >= 0 && all_table->currentRow() < all_table->rowCount()) ?
      all_table->item(all_table->currentRow(), 0)->text() : QString();
    QString err;
    TfViewerSnapshot new_snapshot;
    if (!backend->BuildSnapshot(&new_snapshot, &err)) {
      status->setText(err);
      all_table->setRowCount(0);
      related_table->setRowCount(0);
      pair_label->setText("当前选择: (未选择)");
      matrix_view->clear();
      meaning_view->clear();
      return;
    }
    *snapshot = std::move(new_snapshot);
    all_table->setRowCount(static_cast<int>(snapshot->frames.size()));
    for (int i = 0; i < static_cast<int>(snapshot->frames.size()); ++i) {
      auto * frame_item = new QTableWidgetItem(snapshot->frames[i]);
      auto * type_item = new QTableWidgetItem(backend->FrameTypeText(*snapshot, snapshot->frames[i]));
      all_table->setItem(i, 0, frame_item);
      all_table->setItem(i, 1, type_item);
    }
    if (!selected.isEmpty()) {
      for (int i = 0; i < all_table->rowCount(); ++i) {
        if (all_table->item(i, 0)->text() == selected) {
          all_table->setCurrentCell(i, 0);
          break;
        }
      }
    } else if (all_table->rowCount() > 0) {
      all_table->setCurrentCell(0, 0);
    }
    status->setText(
      QString("Frame 数: %1 | 更新时间: %2")
      .arg(snapshot->frames.size())
      .arg(QDateTime::currentDateTime().toString("hh:mm:ss")));
    refresh_related_and_transform();
  };

  auto refresh_busy = std::make_shared<std::atomic_bool>(false);
  auto refresh_frames_async = [=]() {
    if (refresh_busy->exchange(true)) return;
    const QString selected = (all_table->currentRow() >= 0 && all_table->currentRow() < all_table->rowCount()) ?
      all_table->item(all_table->currentRow(), 0)->text() : QString();
    QPointer<TfViewerWidget> alive(this);
    RunOnSharedRefreshPool([=]() {
      QString err;
      TfViewerSnapshot new_snapshot;
      const bool ok = backend->BuildSnapshot(&new_snapshot, &err);
      QMetaObject::invokeMethod(this, [=]() mutable {
        refresh_busy->store(false);
        if (!alive) return;
        if (!ok) {
          status->setText(err);
          all_table->setRowCount(0);
          related_table->setRowCount(0);
          pair_label->setText("当前选择: (未选择)");
          matrix_view->clear();
          meaning_view->clear();
          return;
        }
        *snapshot = std::move(new_snapshot);
        all_table->setRowCount(static_cast<int>(snapshot->frames.size()));
        for (int i = 0; i < static_cast<int>(snapshot->frames.size()); ++i) {
          auto * frame_item = new QTableWidgetItem(snapshot->frames[i]);
          auto * type_item = new QTableWidgetItem(backend->FrameTypeText(*snapshot, snapshot->frames[i]));
          all_table->setItem(i, 0, frame_item);
          all_table->setItem(i, 1, type_item);
        }
        if (!selected.isEmpty()) {
          for (int i = 0; i < all_table->rowCount(); ++i) {
            if (all_table->item(i, 0)->text() == selected) {
              all_table->setCurrentCell(i, 0);
              break;
            }
          }
        } else if (all_table->rowCount() > 0) {
          all_table->setCurrentCell(0, 0);
        }
        status->setText(
          QString("Frame 数: %1 | 更新时间: %2")
          .arg(snapshot->frames.size())
          .arg(QDateTime::currentDateTime().toString("hh:mm:ss")));
        refresh_related_and_transform();
      }, Qt::QueuedConnection);
    });
  };

  QTimer * timer = new QTimer(this);
  timer->setInterval(1000);
  QObject::connect(timer, &QTimer::timeout, [=]() {
    if (auto_refresh->isChecked()) { refresh_frames_async(); }
  });
  timer->start();

  QObject::connect(refresh_btn, &QPushButton::clicked, [=]() { refresh_frames(); });
  QObject::connect(all_table, &QTableWidget::itemSelectionChanged, [=]() { refresh_related_and_transform(); });
  QObject::connect(related_table, &QTableWidget::itemSelectionChanged, [=]() { refresh_related_and_transform(); });

  refresh_frames();
}

}  // namespace ros_robot_assist_tools::ui
