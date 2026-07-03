#include "ros_robot_workbench/ui/tf_viewer_widget.h"

#include <memory>
#include <atomic>

#include <QCheckBox>
#include <QDateTime>
#include <QFontDatabase>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPointer>
#include <QPushButton>
#include <QRegularExpression>
#include <QSignalBlocker>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

#include "ros_robot_workbench/module/tf_viewer_module.h"
#include "ros_robot_workbench/ui/shared_refresh_pool.h"

namespace ros_robot_workbench::ui
{
namespace
{

QString SafeTableText(const QTableWidget * table, int row, int col)
{
  if (table == nullptr || row < 0 || row >= table->rowCount()) {
    return {};
  }
  const QTableWidgetItem * item = table->item(row, col);
  return item ? item->text() : QString();
}

bool MatchWithSimpleRegexSupport(const QString & text, const QString & filter_text)
{
  const QString f = filter_text.trimmed();
  if (f.isEmpty()) {
    return true;
  }
  if (f.startsWith("re:", Qt::CaseInsensitive)) {
    const QString pattern = f.mid(3).trimmed();
    if (pattern.isEmpty()) {
      return true;
    }
    const QRegularExpression re(
      pattern, QRegularExpression::CaseInsensitiveOption | QRegularExpression::UseUnicodePropertiesOption);
    if (!re.isValid()) {
      return text.contains(f, Qt::CaseInsensitive);
    }
    return re.match(text).hasMatch();
  }
  return text.contains(f, Qt::CaseInsensitive);
}

}  // namespace

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
  QLineEdit * all_filter_edit = new QLineEdit();
  all_filter_edit->setClearButtonEnabled(true);
  all_filter_edit->setPlaceholderText(QStringLiteral("筛选 Frame ID…（支持 re:正则）"));
  QTableWidget * all_table = new QTableWidget();
  all_table->setColumnCount(2);
  all_table->setHorizontalHeaderLabels({"Frame ID", "类型/频率"});
  all_table->setSelectionBehavior(QAbstractItemView::SelectRows);
  all_table->setSelectionMode(QAbstractItemView::SingleSelection);
  all_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  all_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  all_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  all_layout->addWidget(all_filter_edit);
  all_layout->addWidget(all_table);

  QGroupBox * related_group = new QGroupBox("与选中 Frame 关联的 Frame");
  QVBoxLayout * related_layout = new QVBoxLayout(related_group);
  QLineEdit * related_filter_edit = new QLineEdit();
  related_filter_edit->setClearButtonEnabled(true);
  related_filter_edit->setPlaceholderText(QStringLiteral("筛选 Related Frame ID…（支持 re:正则）"));
  QTableWidget * related_table = new QTableWidget();
  related_table->setColumnCount(2);
  related_table->setHorizontalHeaderLabels({"Related Frame ID", "类型/频率"});
  related_table->setSelectionBehavior(QAbstractItemView::SelectRows);
  related_table->setSelectionMode(QAbstractItemView::SingleSelection);
  related_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  related_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  related_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  related_layout->addWidget(related_filter_edit);
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

  auto effective_all_row = [=]() -> int {
    const int r = all_table->currentRow();
    if (r >= 0 && r < all_table->rowCount() && !all_table->isRowHidden(r)) {
      return r;
    }
    for (int i = 0; i < all_table->rowCount(); ++i) {
      if (!all_table->isRowHidden(i)) {
        return i;
      }
    }
    return -1;
  };

  auto effective_related_row = [=]() -> int {
    const int r = related_table->currentRow();
    if (r >= 0 && r < related_table->rowCount() && !related_table->isRowHidden(r)) {
      return r;
    }
    for (int i = 0; i < related_table->rowCount(); ++i) {
      if (!related_table->isRowHidden(i)) {
        return i;
      }
    }
    return -1;
  };

  auto apply_all_row_filter = [=]() {
    const QString f = all_filter_edit->text().trimmed();
    QRegularExpression re;
    bool use_regex = false;
    if (f.startsWith("re:", Qt::CaseInsensitive)) {
      const QString pattern = f.mid(3).trimmed();
      if (!pattern.isEmpty()) {
        re = QRegularExpression(
          pattern, QRegularExpression::CaseInsensitiveOption | QRegularExpression::UseUnicodePropertiesOption);
        use_regex = re.isValid();
      }
    }
    for (int i = 0; i < all_table->rowCount(); ++i) {
      const QString name = SafeTableText(all_table, i, 0);
      const bool match = use_regex ? re.match(name).hasMatch() : MatchWithSimpleRegexSupport(name, f);
      all_table->setRowHidden(i, !match);
    }
    const int ar = all_table->currentRow();
    if (ar < 0 || ar >= all_table->rowCount() || all_table->isRowHidden(ar)) {
      for (int i = 0; i < all_table->rowCount(); ++i) {
        if (!all_table->isRowHidden(i)) {
          all_table->setCurrentCell(i, 0);
          break;
        }
      }
    }
  };

  auto apply_related_row_filter = [=]() {
    const QString f = related_filter_edit->text().trimmed();
    QRegularExpression re;
    bool use_regex = false;
    if (f.startsWith("re:", Qt::CaseInsensitive)) {
      const QString pattern = f.mid(3).trimmed();
      if (!pattern.isEmpty()) {
        re = QRegularExpression(
          pattern, QRegularExpression::CaseInsensitiveOption | QRegularExpression::UseUnicodePropertiesOption);
        use_regex = re.isValid();
      }
    }
    for (int i = 0; i < related_table->rowCount(); ++i) {
      const QString name = SafeTableText(related_table, i, 0);
      const bool match = use_regex ? re.match(name).hasMatch() : MatchWithSimpleRegexSupport(name, f);
      related_table->setRowHidden(i, !match);
    }
    const int rr = related_table->currentRow();
    if (rr < 0 || rr >= related_table->rowCount() || related_table->isRowHidden(rr)) {
      const QSignalBlocker blocker(related_table);
      for (int i = 0; i < related_table->rowCount(); ++i) {
        if (!related_table->isRowHidden(i)) {
          related_table->setCurrentCell(i, 0);
          break;
        }
      }
    }
  };

  auto update_transform_views = [=]() {
    const int all_row = effective_all_row();
    if (all_row < 0 || all_row >= all_table->rowCount()) {
      pair_label->setText("当前选择: (未选择)");
      matrix_view->clear();
      meaning_view->clear();
      return;
    }
    const QString from = SafeTableText(all_table, all_row, 0);
    if (from.isEmpty()) {
      pair_label->setText("当前选择: (未选择)");
      matrix_view->clear();
      meaning_view->clear();
      return;
    }
    if (related_table->rowCount() == 0) {
      pair_label->setText(QString("当前选择: %1 -> (请在右侧选择目标)").arg(from));
      matrix_view->clear();
      meaning_view->clear();
      return;
    }
    const int rel_row = effective_related_row();
    if (rel_row < 0 || rel_row >= related_table->rowCount()) {
      pair_label->setText(QString("当前选择: %1 -> (请在右侧选择目标)").arg(from));
      matrix_view->clear();
      meaning_view->clear();
      return;
    }
    const QString to = SafeTableText(related_table, rel_row, 0);
    if (to.isEmpty()) {
      pair_label->setText(QString("当前选择: %1 -> (请在右侧选择目标)").arg(from));
      matrix_view->clear();
      meaning_view->clear();
      return;
    }
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

  auto refresh_related_and_transform = [=]() {
    const int all_row = effective_all_row();
    if (all_row < 0 || all_row >= all_table->rowCount()) {
      related_table->setRowCount(0);
      pair_label->setText("当前选择: (未选择)");
      matrix_view->clear();
      meaning_view->clear();
      return;
    }
    const QString from = SafeTableText(all_table, all_row, 0);
    if (from.isEmpty()) {
      related_table->setRowCount(0);
      pair_label->setText("当前选择: (未选择)");
      matrix_view->clear();
      meaning_view->clear();
      return;
    }
    const auto related = backend->ConnectedFrames(*snapshot, from);
    related_table->setRowCount(static_cast<int>(related.size()));
    for (int i = 0; i < static_cast<int>(related.size()); ++i) {
      auto * frame_item = new QTableWidgetItem(related[i]);
      auto * type_item = new QTableWidgetItem(backend->FrameTypeText(*snapshot, related[i]));
      related_table->setItem(i, 0, frame_item);
      related_table->setItem(i, 1, type_item);
    }
    apply_related_row_filter();
    update_transform_views();
  };

  auto refresh_frames = [=]() {
    QString selected;
    const int sel_row = all_table->currentRow();
    if (sel_row >= 0 && sel_row < all_table->rowCount() && !all_table->isRowHidden(sel_row)) {
      selected = SafeTableText(all_table, sel_row, 0);
    } else {
      for (int i = 0; i < all_table->rowCount(); ++i) {
        if (!all_table->isRowHidden(i)) {
          selected = SafeTableText(all_table, i, 0);
          break;
        }
      }
    }
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
        if (SafeTableText(all_table, i, 0) == selected) {
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
    apply_all_row_filter();
    refresh_related_and_transform();
  };

  auto refresh_busy = std::make_shared<std::atomic_bool>(false);
  auto refresh_frames_async = [=]() {
    if (refresh_busy->exchange(true)) return;
    QString selected;
    const int sel_row = all_table->currentRow();
    if (sel_row >= 0 && sel_row < all_table->rowCount() && !all_table->isRowHidden(sel_row)) {
      selected = SafeTableText(all_table, sel_row, 0);
    } else {
      for (int i = 0; i < all_table->rowCount(); ++i) {
        if (!all_table->isRowHidden(i)) {
          selected = SafeTableText(all_table, i, 0);
          break;
        }
      }
    }
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
            if (SafeTableText(all_table, i, 0) == selected) {
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
        apply_all_row_filter();
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
  QObject::connect(all_filter_edit, &QLineEdit::textChanged, [=]() { apply_all_row_filter(); });
  QObject::connect(related_filter_edit, &QLineEdit::textChanged, [=]() {
    apply_related_row_filter();
    update_transform_views();
  });
  QObject::connect(all_table, &QTableWidget::itemSelectionChanged, [=]() { refresh_related_and_transform(); });
  QObject::connect(related_table, &QTableWidget::itemSelectionChanged, [=]() { refresh_related_and_transform(); });

  refresh_frames();
}

}  // namespace ros_robot_workbench::ui
