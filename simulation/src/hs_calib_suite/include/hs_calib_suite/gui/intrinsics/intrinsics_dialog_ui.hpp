#pragma once

#include <functional>

#include <QString>
#include <QWidget>

class QDialog;
class QFormLayout;
class QFrame;
class QGroupBox;
class QLabel;
class QPushButton;
class QScrollArea;
class QVBoxLayout;

namespace hs_calib {
namespace gui {

/// \brief Tier4 风格内参弹窗共用布局与样式
struct IntrinsicsDialogChrome {
  QLabel *title_label = nullptr;
  QLabel *subtitle_label = nullptr;
  QScrollArea *scroll = nullptr;
  QWidget *content = nullptr;
  QVBoxLayout *content_layout = nullptr;
};

IntrinsicsDialogChrome setup_intrinsics_dialog(
    QDialog *dialog,
    const QString &title,
    const QString &subtitle,
    int min_width = 480);

QGroupBox *make_param_group(const QString &title, QWidget *parent);
QFormLayout *new_param_form(QGroupBox *group);
void style_param_form(QFormLayout *form);
QLabel *make_field_label(const QString &text, QWidget *parent);
QLabel *make_value_label(const QString &text, QWidget *parent);
void add_metric_row(QFormLayout *form, const QString &label, QLabel **value_out, QWidget *parent);

struct StatTileWidgets {
  QFrame *frame = nullptr;
  QLabel *title = nullptr;
  QLabel *value = nullptr;
};
StatTileWidgets make_stat_tile(const QString &title, QWidget *parent);

QPushButton *add_dialog_footer(
    QVBoxLayout *root,
    const QString &primary_text,
    const std::function<void()> &on_primary);

}  // namespace gui
}  // namespace hs_calib
