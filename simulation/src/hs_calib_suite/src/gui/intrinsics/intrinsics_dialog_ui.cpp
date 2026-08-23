#include "hs_calib_suite/gui/intrinsics/intrinsics_dialog_ui.hpp"

#include <QDialog>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

namespace hs_calib {
namespace gui {

IntrinsicsDialogChrome setup_intrinsics_dialog(
    QDialog *dialog,
    const QString &title,
    const QString &subtitle,
    int min_width) {
  dialog->setObjectName(QStringLiteral("IntrinsicsParamDialog"));
  dialog->setWindowTitle(title);
  dialog->setWindowFlags(dialog->windowFlags() | Qt::Window);
  dialog->setModal(false);
  dialog->setMinimumWidth(min_width);

  IntrinsicsDialogChrome chrome;
  auto *root = new QVBoxLayout(dialog);
  root->setContentsMargins(20, 18, 20, 16);
  root->setSpacing(12);

  chrome.title_label = new QLabel(title, dialog);
  chrome.title_label->setObjectName(QStringLiteral("PageTitle"));
  root->addWidget(chrome.title_label);

  if (!subtitle.isEmpty()) {
    chrome.subtitle_label = new QLabel(subtitle, dialog);
    chrome.subtitle_label->setObjectName(QStringLiteral("PageSubtitle"));
    chrome.subtitle_label->setWordWrap(true);
    root->addWidget(chrome.subtitle_label);
  }

  auto *sep = new QFrame(dialog);
  sep->setObjectName(QStringLiteral("DialogDivider"));
  sep->setFrameShape(QFrame::HLine);
  sep->setFixedHeight(1);
  root->addWidget(sep);

  chrome.scroll = new QScrollArea(dialog);
  chrome.scroll->setObjectName(QStringLiteral("IntrinsicsDialogScroll"));
  chrome.scroll->setWidgetResizable(true);
  chrome.scroll->setFrameShape(QFrame::NoFrame);
  chrome.scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  chrome.content = new QWidget(chrome.scroll);
  chrome.content_layout = new QVBoxLayout(chrome.content);
  chrome.content_layout->setContentsMargins(0, 4, 4, 8);
  chrome.content_layout->setSpacing(14);
  chrome.scroll->setWidget(chrome.content);
  root->addWidget(chrome.scroll, 1);

  return chrome;
}

QGroupBox *make_param_group(const QString &title, QWidget *parent) {
  auto *g = new QGroupBox(title, parent);
  g->setObjectName(QStringLiteral("LauncherGroup"));
  return g;
}

QFormLayout *new_param_form(QGroupBox *group) {
  auto *form = new QFormLayout(group);
  style_param_form(form);
  return form;
}

void style_param_form(QFormLayout *form) {
  form->setContentsMargins(0, 0, 0, 0);
  form->setHorizontalSpacing(16);
  form->setVerticalSpacing(10);
  form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
  form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
  form->setRowWrapPolicy(QFormLayout::DontWrapRows);
}

QLabel *make_field_label(const QString &text, QWidget *parent) {
  auto *l = new QLabel(text, parent);
  l->setObjectName(QStringLiteral("ParamFieldLabel"));
  return l;
}

QLabel *make_value_label(const QString &text, QWidget *parent) {
  auto *l = new QLabel(text, parent);
  l->setObjectName(QStringLiteral("ParamValueLabel"));
  l->setTextInteractionFlags(Qt::TextSelectableByMouse);
  l->setWordWrap(true);
  return l;
}

void add_metric_row(
    QFormLayout *form, const QString &label, QLabel **value_out, QWidget *parent) {
  *value_out = make_value_label(QStringLiteral("—"), parent);
  form->addRow(make_field_label(label, parent), *value_out);
}

StatTileWidgets make_stat_tile(const QString &title, QWidget *parent) {
  StatTileWidgets out;
  out.frame = new QFrame(parent);
  out.frame->setObjectName(QStringLiteral("CalibTile"));
  auto *lay = new QVBoxLayout(out.frame);
  lay->setContentsMargins(14, 12, 14, 12);
  lay->setSpacing(4);
  out.title = new QLabel(title, out.frame);
  out.title->setObjectName(QStringLiteral("CalibTileSubtitle"));
  out.value = new QLabel(QStringLiteral("—"), out.frame);
  out.value->setObjectName(QStringLiteral("CalibTileTitle"));
  lay->addWidget(out.title);
  lay->addWidget(out.value);
  return out;
}

QPushButton *add_dialog_footer(
    QVBoxLayout *root,
    const QString &primary_text,
    const std::function<void()> &on_primary) {
  auto *sep = new QFrame;
  sep->setObjectName(QStringLiteral("DialogDivider"));
  sep->setFrameShape(QFrame::HLine);
  sep->setFixedHeight(1);
  root->addWidget(sep);

  auto *bar = new QHBoxLayout;
  bar->setContentsMargins(0, 4, 0, 0);
  bar->addStretch(1);
  auto *btn = new QPushButton(primary_text);
  btn->setObjectName(QStringLiteral("WorkbenchActionPrimary"));
  btn->setMinimumWidth(120);
  btn->setMinimumHeight(38);
  if (on_primary) {
    QObject::connect(btn, &QPushButton::clicked, btn, [on_primary]() { on_primary(); });
  }
  bar->addWidget(btn);
  root->addLayout(bar);
  return btn;
}

}  // namespace gui
}  // namespace hs_calib
