#include "ros_robot_assist_tools/ui/preferences_dialog.h"

#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QVBoxLayout>

#include "ros_robot_assist_tools/preferences/app_preferences.hpp"

namespace ros_robot_assist_tools::ui
{

void ShowPreferencesDialog(QWidget * parent)
{
  AppPreferences prefs;
  LoadAppPreferences(&prefs);

  QDialog dlg(parent);
  dlg.setWindowTitle(QStringLiteral("首选项"));
  dlg.setMinimumWidth(460);
  auto * root = new QVBoxLayout(&dlg);
  auto * form = new QFormLayout();
  auto * ros_domain = new QLineEdit(QString::fromStdString(prefs.ros_domain_id));
  ros_domain->setPlaceholderText(QStringLiteral("0~232，留空表示不覆盖启动环境"));
  form->addRow(QStringLiteral("ROS_DOMAIN_ID:"), ros_domain);

  auto * theme = new QComboBox();
  theme->addItem(QStringLiteral("Fusion (default)"), QStringLiteral("fusion"));
  theme->addItem(QStringLiteral("Light"), QStringLiteral("light"));
  theme->addItem(QStringLiteral("Dark"), QStringLiteral("dark"));
  {
    int idx = theme->findData(QString::fromStdString(prefs.ui_theme));
    if (idx < 0) {
      idx = 0;
    }
    theme->setCurrentIndex(idx);
  }
  form->addRow(QStringLiteral("UI Theme:"), theme);
  root->addLayout(form);

  auto * note = new QLabel(
    QStringLiteral("保存后：\n"
                   "1) 主题立即生效。\n"
                   "2) ROS_DOMAIN_ID 会写入 preferences.yaml，并在下次启动时于 rclcpp::init 前加载。\n"
                   "3) 对已运行节点不追溯生效（建议重启程序）。"));
  note->setWordWrap(true);
  note->setStyleSheet(QStringLiteral("color:#64748b;font-size:11px;"));
  root->addWidget(note);

  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  root->addWidget(buttons);
  QObject::connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
  QObject::connect(buttons, &QDialogButtonBox::accepted, [&]() {
    const QString domain = ros_domain->text().trimmed();
    if (!IsValidRosDomainId(domain.toStdString())) {
      QMessageBox::warning(&dlg, QStringLiteral("首选项"), QStringLiteral("ROS_DOMAIN_ID 必须是 0~232 的整数或留空。"));
      return;
    }
    AppPreferences out;
    out.ros_domain_id = domain.toStdString();
    out.ui_theme = theme->currentData().toString().toStdString();
    if (!SaveAppPreferences(out)) {
      QMessageBox::warning(&dlg, QStringLiteral("首选项"), QStringLiteral("保存 preferences.yaml 失败。"));
      return;
    }
    ApplyRosDomainInProcess(out.ros_domain_id);
    if (auto * app = qobject_cast<QApplication *>(QApplication::instance())) {
      ApplyUiThemeToApplication(*app, out.ui_theme);
    }
    dlg.accept();
  });

  dlg.exec();
}

}  // namespace ros_robot_assist_tools::ui
