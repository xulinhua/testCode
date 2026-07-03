#include "ros_robot_workbench/ui/preferences_dialog.h"

#include <algorithm>
#include <cstdlib>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPointer>
#include <QProcess>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include "ros_robot_workbench/preferences/app_preferences.hpp"

namespace ros_robot_workbench::ui
{

namespace
{

QString EnvValueOrUnset(const char * key)
{
  const char * v = std::getenv(key);
  if (v == nullptr || v[0] == '\0') {
    return QStringLiteral("(unset)");
  }
  return QString::fromLocal8Bit(v);
}

QString TriStateExpectedText(const QString & v)
{
  if (v.isEmpty()) {
    return QStringLiteral("(unset)");
  }
  return v;
}

QString PendingMark(const QString & expected, const QString & current)
{
  return expected == current ? QStringLiteral(" [已生效]") : QStringLiteral(" [待生效]");
}

}  // namespace

void ShowPreferencesDialog(QWidget * parent)
{
  AppPreferences prefs;
  LoadAppPreferences(&prefs);
  const AppPreferences prefs_at_open = prefs;

  QDialog dlg(parent);
  dlg.setWindowTitle(QStringLiteral("首选项"));
  dlg.setMinimumWidth(460);
  auto * root = new QVBoxLayout(&dlg);
  auto * form = new QFormLayout();

  auto * ros_domain = new QLineEdit(QString::fromStdString(prefs.ros_domain_id));
  ros_domain->setPlaceholderText(QStringLiteral("0~232，留空表示不覆盖启动环境"));
  form->addRow(QStringLiteral("ROS_DOMAIN_ID:"), ros_domain);
  auto * ros_domain_current = new QLabel();
  form->addRow(QStringLiteral("当前终端值:"), ros_domain_current);
  auto * ros_domain_system = new QLabel();
  form->addRow(QStringLiteral("系统值(.bashrc):"), ros_domain_system);

  auto * localhost = new QComboBox();
  localhost->addItem(QStringLiteral("继承环境"), QStringLiteral(""));
  localhost->addItem(QStringLiteral("仅本机 (1)"), QStringLiteral("1"));
  localhost->addItem(QStringLiteral("允许网络 (0)"), QStringLiteral("0"));
  localhost->setCurrentIndex(std::max(0, localhost->findData(QString::fromStdString(prefs.ros_localhost_only))));
  form->addRow(QStringLiteral("ROS_LOCALHOST_ONLY:"), localhost);
  auto * localhost_current = new QLabel();
  form->addRow(QStringLiteral("当前终端值:"), localhost_current);
  auto * localhost_system = new QLabel();
  form->addRow(QStringLiteral("系统值(.bashrc):"), localhost_system);

  auto * rmw = new QComboBox();
  rmw->addItem(QStringLiteral("继承环境"), QStringLiteral(""));
  rmw->addItem(QStringLiteral("Fast DDS"), QStringLiteral("rmw_fastrtps_cpp"));
  rmw->addItem(QStringLiteral("Cyclone DDS"), QStringLiteral("rmw_cyclonedds_cpp"));
  rmw->setCurrentIndex(std::max(0, rmw->findData(QString::fromStdString(prefs.rmw_implementation))));
  form->addRow(QStringLiteral("RMW_IMPLEMENTATION:"), rmw);
  auto * rmw_current = new QLabel();
  form->addRow(QStringLiteral("当前终端值:"), rmw_current);
  auto * rmw_system = new QLabel();
  form->addRow(QStringLiteral("系统值(.bashrc):"), rmw_system);

  auto * ros_namespace = new QLineEdit(QString::fromStdString(prefs.ros_namespace));
  ros_namespace->setPlaceholderText(QStringLiteral("/robot_a 或留空"));
  form->addRow(QStringLiteral("ROS_NAMESPACE:"), ros_namespace);
  auto * ros_namespace_current = new QLabel();
  form->addRow(QStringLiteral("当前终端值:"), ros_namespace_current);
  auto * ros_namespace_system = new QLabel();
  form->addRow(QStringLiteral("系统值(.bashrc):"), ros_namespace_system);

  auto * log_level = new QComboBox();
  log_level->addItems({QStringLiteral("DEBUG"), QStringLiteral("INFO"), QStringLiteral("WARN"),
    QStringLiteral("ERROR"), QStringLiteral("FATAL")});
  log_level->setCurrentText(QString::fromStdString(prefs.log_level_default));
  form->addRow(QStringLiteral("默认日志级别:"), log_level);
  auto * log_level_current = new QLabel();
  form->addRow(QStringLiteral("当前终端值:"), log_level_current);
  auto * log_level_system = new QLabel();
  form->addRow(QStringLiteral("系统值(.bashrc):"), log_level_system);

  auto * use_sim_time = new QCheckBox(QStringLiteral("默认启用 use_sim_time"));
  use_sim_time->setChecked(prefs.use_sim_time_default);
  form->addRow(QStringLiteral("仿真时间:"), use_sim_time);

  auto * theme = new QComboBox();
  theme->addItem(QStringLiteral("Fusion (default)"), QStringLiteral("fusion"));
  theme->addItem(QStringLiteral("Light"), QStringLiteral("light"));
  theme->addItem(QStringLiteral("Dark"), QStringLiteral("dark"));
  {
    int idx = theme->findData(QString::fromStdString(prefs.ui_theme));
    if (idx < 0) {
      idx = 0;
    }
    theme->blockSignals(true);
    theme->setCurrentIndex(idx);
    theme->blockSignals(false);
  }
  form->addRow(QStringLiteral("UI Theme:"), theme);
  auto * theme_current = new QLabel();
  form->addRow(QStringLiteral("当前应用值:"), theme_current);

  root->addLayout(form);

  auto * actions = new QHBoxLayout();
  auto * refresh_btn = new QPushButton(QStringLiteral("刷新系统值"));
  auto * pending_state = new QLabel();
  pending_state->setStyleSheet(QStringLiteral("color:#a16207;"));
  actions->addWidget(refresh_btn);
  actions->addWidget(pending_state, 1);
  root->addLayout(actions);

  auto bashrc_query_proc = std::make_shared<QPointer<QProcess>>(nullptr);

  auto refresh_current_values = [=]() {
    const QString current_domain = EnvValueOrUnset("ROS_DOMAIN_ID");
    const QString current_localhost = EnvValueOrUnset("ROS_LOCALHOST_ONLY");
    const QString current_rmw = EnvValueOrUnset("RMW_IMPLEMENTATION");
    const QString current_ns = EnvValueOrUnset("ROS_NAMESPACE");
    const QString current_log = EnvValueOrUnset("RCUTILS_LOGGING_SEVERITY_THRESHOLD");
    const QString current_theme = theme->currentData().toString();

    ros_domain_current->setText(
      QStringLiteral("ROS_DOMAIN_ID=%1%2")
      .arg(current_domain)
      .arg(PendingMark(ros_domain->text().trimmed().isEmpty() ? QStringLiteral("(unset)") : ros_domain->text().trimmed(), current_domain)));
    localhost_current->setText(
      QStringLiteral("ROS_LOCALHOST_ONLY=%1%2")
      .arg(current_localhost)
      .arg(PendingMark(TriStateExpectedText(localhost->currentData().toString()), current_localhost)));
    rmw_current->setText(
      QStringLiteral("RMW_IMPLEMENTATION=%1%2")
      .arg(current_rmw)
      .arg(PendingMark(TriStateExpectedText(rmw->currentData().toString()), current_rmw)));
    ros_namespace_current->setText(
      QStringLiteral("ROS_NAMESPACE=%1%2")
      .arg(current_ns)
      .arg(PendingMark(ros_namespace->text().trimmed().isEmpty() ? QStringLiteral("(unset)") : ros_namespace->text().trimmed(), current_ns)));
    log_level_current->setText(
      QStringLiteral("RCUTILS_LOGGING_SEVERITY_THRESHOLD=%1%2")
      .arg(current_log)
      .arg(PendingMark(log_level->currentText(), current_log)));
    theme_current->setText(QStringLiteral("ui_theme=%1 (预览即时生效，未保存会恢复)").arg(current_theme));

    const bool has_pending =
      ros_domain_current->text().contains(QStringLiteral("待生效")) ||
      localhost_current->text().contains(QStringLiteral("待生效")) ||
      rmw_current->text().contains(QStringLiteral("待生效")) ||
      ros_namespace_current->text().contains(QStringLiteral("待生效")) ||
      log_level_current->text().contains(QStringLiteral("待生效"));
    pending_state->setText(
      has_pending ? QStringLiteral("存在系统环境差异：保存后会写入，建议重启 ROS 会话。")
                  : QStringLiteral("当前系统值与表单一致。"));
  };

  auto refresh_bashrc_values_async = [=, &dlg]() {
    if (*bashrc_query_proc != nullptr) {
      (*bashrc_query_proc)->kill();
      (*bashrc_query_proc)->deleteLater();
      *bashrc_query_proc = nullptr;
    }
    ros_domain_system->setText(QStringLiteral("ROS_DOMAIN_ID=加载中..."));
    localhost_system->setText(QStringLiteral("ROS_LOCALHOST_ONLY=加载中..."));
    rmw_system->setText(QStringLiteral("RMW_IMPLEMENTATION=加载中..."));
    ros_namespace_system->setText(QStringLiteral("ROS_NAMESPACE=加载中..."));
    log_level_system->setText(QStringLiteral("RCUTILS_LOGGING_SEVERITY_THRESHOLD=加载中..."));

    auto * p = new QProcess(&dlg);
    *bashrc_query_proc = p;
    QPointer<QLabel> ros_domain_system_guard = ros_domain_system;
    QPointer<QLabel> localhost_system_guard = localhost_system;
    QPointer<QLabel> rmw_system_guard = rmw_system;
    QPointer<QLabel> ros_namespace_system_guard = ros_namespace_system;
    QPointer<QLabel> log_level_system_guard = log_level_system;

    QObject::connect(
      p, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), p,
      [=](int exit_code, QProcess::ExitStatus status) {
        if (*bashrc_query_proc == p) {
          *bashrc_query_proc = nullptr;
        }
        const auto parse_value = [](const QString & all, const QString & key) -> QString {
          const QString prefix = key + "=";
          const QStringList lines = all.split('\n', Qt::SkipEmptyParts);
          for (const QString & line : lines) {
            if (line.startsWith(prefix)) {
              const QString v = line.mid(prefix.size()).trimmed();
              return v.isEmpty() ? QStringLiteral("(unset)") : v;
            }
          }
          return QStringLiteral("(unset)");
        };

        if (status != QProcess::NormalExit || exit_code != 0) {
          if (ros_domain_system_guard) {
            ros_domain_system_guard->setText(QStringLiteral("ROS_DOMAIN_ID=(unknown)"));
          }
          if (localhost_system_guard) {
            localhost_system_guard->setText(QStringLiteral("ROS_LOCALHOST_ONLY=(unknown)"));
          }
          if (rmw_system_guard) {
            rmw_system_guard->setText(QStringLiteral("RMW_IMPLEMENTATION=(unknown)"));
          }
          if (ros_namespace_system_guard) {
            ros_namespace_system_guard->setText(QStringLiteral("ROS_NAMESPACE=(unknown)"));
          }
          if (log_level_system_guard) {
            log_level_system_guard->setText(QStringLiteral("RCUTILS_LOGGING_SEVERITY_THRESHOLD=(unknown)"));
          }
          return;
        }
        const QString out = QString::fromLocal8Bit(p->readAllStandardOutput());
        if (ros_domain_system_guard) {
          ros_domain_system_guard->setText(
            QStringLiteral("ROS_DOMAIN_ID=%1").arg(parse_value(out, QStringLiteral("ROS_DOMAIN_ID"))));
        }
        if (localhost_system_guard) {
          localhost_system_guard->setText(
            QStringLiteral("ROS_LOCALHOST_ONLY=%1")
            .arg(parse_value(out, QStringLiteral("ROS_LOCALHOST_ONLY"))));
        }
        if (rmw_system_guard) {
          rmw_system_guard->setText(
            QStringLiteral("RMW_IMPLEMENTATION=%1")
            .arg(parse_value(out, QStringLiteral("RMW_IMPLEMENTATION"))));
        }
        if (ros_namespace_system_guard) {
          ros_namespace_system_guard->setText(
            QStringLiteral("ROS_NAMESPACE=%1").arg(parse_value(out, QStringLiteral("ROS_NAMESPACE"))));
        }
        if (log_level_system_guard) {
          log_level_system_guard->setText(
            QStringLiteral("RCUTILS_LOGGING_SEVERITY_THRESHOLD=%1")
            .arg(parse_value(out, QStringLiteral("RCUTILS_LOGGING_SEVERITY_THRESHOLD"))));
        }
      });
    const QString cmd = QStringLiteral(
      "for k in ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION ROS_NAMESPACE "
      "RCUTILS_LOGGING_SEVERITY_THRESHOLD; do "
      "v=\"${!k}\"; printf \"%s=%s\\n\" \"$k\" \"$v\"; done");
    p->start(QStringLiteral("/bin/bash"), {QStringLiteral("-ic"), cmd});
  };
  QObject::connect(&dlg, &QObject::destroyed, [=]() {
    if (*bashrc_query_proc != nullptr) {
      (*bashrc_query_proc)->kill();
      (*bashrc_query_proc)->deleteLater();
      *bashrc_query_proc = nullptr;
    }
  });

  QObject::connect(refresh_btn, &QPushButton::clicked, [=]() {
    refresh_current_values();
    refresh_bashrc_values_async();
  });
  QObject::connect(ros_domain, &QLineEdit::textChanged, refresh_current_values);
  QObject::connect(localhost, QOverload<int>::of(&QComboBox::currentIndexChanged), refresh_current_values);
  QObject::connect(rmw, QOverload<int>::of(&QComboBox::currentIndexChanged), refresh_current_values);
  QObject::connect(ros_namespace, &QLineEdit::textChanged, refresh_current_values);
  QObject::connect(log_level, QOverload<int>::of(&QComboBox::currentIndexChanged), refresh_current_values);

  QObject::connect(theme, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) {
    auto * qa = qobject_cast<QApplication *>(QApplication::instance());
    if (qa != nullptr) {
      const QString id = theme->currentData().toString();
      ApplyUiThemeToApplication(*qa, id.toStdString());
    }
    refresh_current_values();
  });

  auto * note = new QLabel(
    QStringLiteral("“当前终端值”来自本进程环境；“系统值(.bashrc)”来自交互式 bash 读取结果。\n"
                   "主题支持实时预览。\n"
                   "点「确定」保存到 preferences.yaml 并写入当前进程；点「取消/关闭」恢复打开窗口前主题。"));
  note->setWordWrap(true);
  note->setStyleSheet(QStringLiteral("color:#64748b;font-size:11px;"));
  root->addWidget(note);

  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  root->addWidget(buttons);
  QObject::connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
  QObject::connect(&dlg, &QDialog::rejected, [prefs_at_open]() {
    if (auto * qa = qobject_cast<QApplication *>(QApplication::instance())) {
      ApplyUiThemeToApplication(*qa, prefs_at_open.ui_theme);
    }
  });
  QObject::connect(buttons, &QDialogButtonBox::accepted, [&]() {
    const QString domain = ros_domain->text().trimmed();
    if (!IsValidRosDomainId(domain.toStdString())) {
      QMessageBox::warning(&dlg, QStringLiteral("首选项"), QStringLiteral("ROS_DOMAIN_ID 必须是 0~232 的整数或留空。"));
      return;
    }
    AppPreferences out;
    out.ros_domain_id = domain.toStdString();
    out.ros_localhost_only = localhost->currentData().toString().toStdString();
    out.rmw_implementation = rmw->currentData().toString().toStdString();
    out.ros_namespace = ros_namespace->text().trimmed().toStdString();
    out.use_sim_time_default = use_sim_time->isChecked();
    out.log_level_default = log_level->currentText().toStdString();
    out.ui_theme = theme->currentData().toString().toStdString();
    if (!SaveAppPreferences(out)) {
      QMessageBox::warning(&dlg, QStringLiteral("首选项"), QStringLiteral("保存 preferences.yaml 失败。"));
      if (auto * qa = qobject_cast<QApplication *>(QApplication::instance())) {
        ApplyUiThemeToApplication(*qa, prefs_at_open.ui_theme);
      }
      return;
    }
    ApplyRosEnvironmentInProcess(out);
    if (auto * app = qobject_cast<QApplication *>(QApplication::instance())) {
      ApplyUiThemeToApplication(*app, out.ui_theme);
    }
    refresh_current_values();
    refresh_bashrc_values_async();
    dlg.accept();
  });

  ros_domain_system->setText(QStringLiteral("ROS_DOMAIN_ID=加载中..."));
  localhost_system->setText(QStringLiteral("ROS_LOCALHOST_ONLY=加载中..."));
  rmw_system->setText(QStringLiteral("RMW_IMPLEMENTATION=加载中..."));
  ros_namespace_system->setText(QStringLiteral("ROS_NAMESPACE=加载中..."));
  log_level_system->setText(QStringLiteral("RCUTILS_LOGGING_SEVERITY_THRESHOLD=加载中..."));
  pending_state->setText(QStringLiteral("界面已打开，系统值(.bashrc)正在后台刷新..."));

  refresh_current_values();
  QTimer::singleShot(300, &dlg, refresh_bashrc_values_async);
  dlg.exec();
}

}  // namespace ros_robot_workbench::ui
