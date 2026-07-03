// Nova Robot Tools UI - 参考 calib_sim 架构
// 所有 Qt 代码在 .cpp 中实现，不使用 Q_OBJECT 和 AUTOMOC

#include "ros_robot_workbench/ros_robot_workbench_ui.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QAction>
#include <QDesktopServices>
#include <QFileInfo>
#include <QKeySequence>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QUrl>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QStackedWidget>
#include <QTimer>
#include <QProcess>
#include <QStatusBar>
#include <QComboBox>
#include <QSpinBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QGroupBox>
#include <QFormLayout>
#include <QImage>
#include <QPixmap>
#include <QInputDialog>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QBuffer>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <cmath>

#include <QtGlobal>

#include "ros_robot_workbench/ui/lazy_feature_page.hpp"
#include "ros_robot_workbench/ui/shared_ui_executor.hpp"
#include "ros_robot_workbench/ui/workbench_module_registry.hpp"
#include "ros_robot_workbench/ui/system_status_widget.h"
#include "ros_robot_workbench/ui/preferences_dialog.h"
#include "ros_robot_workbench/module/system_status_module.h"
#include "ros_robot_workbench/preferences/app_preferences.hpp"
#include "ros_robot_workbench/workbench_build_config.hpp"
#if WORKBENCH_KIT_GENERAL
#include "ros_robot_workbench/ui/image_viewer_widget.h"
#endif

namespace ros_robot_workbench
{
namespace
{

QString ResolveShareConfigDir()
{
  const QString exe_dir = QCoreApplication::applicationDirPath();
  const QStringList candidates = {
    exe_dir + "/../../share/ros_robot_workbench/config",
    QDir::current().absoluteFilePath("../share/ros_robot_workbench/config"),
    QDir::current().absoluteFilePath("install/ros_robot_workbench/share/ros_robot_workbench/config"),
    QDir::current().absoluteFilePath("simulation/install/ros_robot_workbench/share/ros_robot_workbench/config"),
    QDir::current().absoluteFilePath("src/ros_robot_workbench/config"),
  };
  for (const QString & rel : candidates) {
    const QString canon = QDir(rel).canonicalPath();
    if (!canon.isEmpty() && QFileInfo(canon).isDir()) {
      return canon;
    }
  }
  return QDir(exe_dir + "/../../share/ros_robot_workbench/config").canonicalPath();
}

QString ResolveHelpPdfPath()
{
  const QString exe_dir = QCoreApplication::applicationDirPath();
  const QStringList candidates = {
    exe_dir + "/../../share/ros_robot_workbench/assets/docs/AssistTool_UserGuide.pdf",
    QDir::current().absoluteFilePath("src/ros_robot_workbench/assets/docs/AssistTool_UserGuide.pdf"),
    QDir::current().absoluteFilePath("assets/docs/AssistTool_UserGuide.pdf"),
  };
  for (const QString & p : candidates) {
    const QString c = QDir(p).canonicalPath();
    if (!c.isEmpty() && QFileInfo(c).isFile()) {
      return c;
    }
  }
  return {};
}

void SetupWorkbenchMenuBar(QMainWindow * window, QStackedWidget * stack, const QStringList & page_names)
{
  QMenuBar * bar = window->menuBar();
  QStatusBar * sb = window->statusBar();

  QMenu * m_file = bar->addMenu(QStringLiteral("文件(&F)"));
  QAction * a_open_cfg = m_file->addAction(QStringLiteral("打开配置目录(&O)..."));
  a_open_cfg->setShortcut(QKeySequence::Open);
  QObject::connect(a_open_cfg, &QAction::triggered, [window]() {
    const QString dir = ResolveShareConfigDir();
    if (dir.isEmpty() || !QFileInfo(dir).isDir()) {
      QMessageBox::warning(
        window, QStringLiteral("打开目录"),
        QStringLiteral("未找到 share/ros_robot_workbench/config。\n"
                       "请从工作空间执行 install 后运行，或手动在资源管理器中打开 config。"));
      return;
    }
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(dir))) {
      QMessageBox::warning(window, QStringLiteral("打开目录"), QStringLiteral("无法打开目录：\n%1").arg(dir));
    }
  });
  QAction * a_reload = m_file->addAction(QStringLiteral("重新加载配置(&R)"));
  QObject::connect(a_reload, &QAction::triggered, [window]() {
    QMessageBox::information(
      window, QStringLiteral("重新加载配置"),
      QStringLiteral(
        "各模块在打开对应页面时会从 yaml 读取配置；修改文件后请：\n"
        "• 使用各页内的「应用 / 保存」等按钮，或\n"
        "• 重启本程序。\n\n"
        "（全局一键重载各页状态尚未接入，避免与未保存编辑冲突。）"));
  });
  m_file->addSeparator();
  QAction * a_quit = m_file->addAction(QStringLiteral("退出(&Q)"));
  a_quit->setShortcut(QKeySequence::Quit);
  QObject::connect(a_quit, &QAction::triggered, qApp, &QApplication::quit);

  QMenu * m_edit = bar->addMenu(QStringLiteral("编辑(&E)"));
  QAction * a_prefs = m_edit->addAction(QStringLiteral("首选项(&P)..."));
  a_prefs->setShortcut(QKeySequence::Preferences);
  QObject::connect(a_prefs, &QAction::triggered, [window]() { ui::ShowPreferencesDialog(window); });

  QMenu * m_view = bar->addMenu(QStringLiteral("视图(&V)"));
  QMenu * m_goto = m_view->addMenu(QStringLiteral("跳转到(&G)"));
  for (int i = 0; i < page_names.size(); ++i) {
    QAction * a = m_goto->addAction(QStringLiteral("%1. %2").arg(i + 1).arg(page_names[i]));
    const int idx = i;
    QObject::connect(a, &QAction::triggered, [stack, idx]() { stack->setCurrentIndex(idx); });
  }
  QAction * a_full = m_view->addAction(QStringLiteral("全屏(&U)"));
  a_full->setCheckable(true);
  a_full->setShortcut(QKeySequence(Qt::Key_F11));
  auto * was_max_before_full = new bool(window->isMaximized());
  auto toggle_fullscreen = [window, a_full, was_max_before_full]() {
    if (!window->isFullScreen()) {
      *was_max_before_full = window->isMaximized();
      window->setWindowState(window->windowState() | Qt::WindowFullScreen);
      window->showFullScreen();
      a_full->setChecked(true);
      return;
    }
    window->setWindowState(window->windowState() & ~Qt::WindowFullScreen);
    if (*was_max_before_full) {
      window->showMaximized();
    } else {
      window->showNormal();
    }
    a_full->setChecked(false);
  };
  QObject::connect(a_full, &QAction::triggered, [toggle_fullscreen]() {
    toggle_fullscreen();
  });
  QAction * esc_exit = new QAction(window);
  esc_exit->setShortcut(QKeySequence(Qt::Key_Escape));
  window->addAction(esc_exit);
  QObject::connect(esc_exit, &QAction::triggered, [toggle_fullscreen, window]() {
    if (window->isFullScreen()) {
      toggle_fullscreen();
    }
  });
  QAction * a_status = m_view->addAction(QStringLiteral("显示状态栏(&S)"));
  a_status->setCheckable(true);
  a_status->setChecked(sb->isVisible());
  QObject::connect(a_status, &QAction::toggled, [sb](bool on) { sb->setVisible(on); });

  QMenu * m_help = bar->addMenu(QStringLiteral("帮助(&H)"));
  QAction * a_help = m_help->addAction(QStringLiteral("使用说明(&D)..."));
  QObject::connect(a_help, &QAction::triggered, [window]() {
    const QString pdf = ResolveHelpPdfPath();
    if (pdf.isEmpty()) {
      QMessageBox::warning(window, QStringLiteral("使用说明"), QStringLiteral("未找到用户手册 PDF。"));
      return;
    }
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(pdf))) {
      QMessageBox::warning(window, QStringLiteral("使用说明"), QStringLiteral("打开 PDF 失败：\n%1").arg(pdf));
    }
  });
  m_help->addSeparator();
  QAction * a_about = m_help->addAction(QStringLiteral("About Robot Workbench(&A)..."));
  QObject::connect(a_about, &QAction::triggered, [window]() {
    QMessageBox::about(
      window, QStringLiteral("About Robot Workbench"),
      QStringLiteral(
        "<h3>Robot Workbench</h3>"
        "<p>Qt %1 &nbsp;|&nbsp; Built with C++17</p>")
        .arg(QString::fromUtf8(qVersion())));
  });
}

}  // namespace

/// 系统监控实现 - 直接在 .cpp 中定义，不使用 Q_OBJECT
class SystemMonitor {
public:
  SystemMonitor(QLabel* cpuLabel, QLabel* memLabel, QLabel* gpuLabel,
                QLabel* netUpLabel = nullptr, QLabel* netDownLabel = nullptr)
    : cpuLabel_(cpuLabel), memLabel_(memLabel), gpuLabel_(gpuLabel),
      netUpLabel_(netUpLabel), netDownLabel_(netDownLabel),
      lastRxBytes_(0), lastTxBytes_(0) {
    timer_ = new QTimer();
    timer_->setInterval(1000);
    
    // 初始化网络字节数
    updateNetworkBase();
    
    // 使用 Qt5 新语法连接 QTimer
    QObject::connect(timer_, &QTimer::timeout, [this]() {
      updateCPU();
      updateMemory();
      updateGPU();
      if (netUpLabel_ && netDownLabel_) {
        updateNetwork();
      }
    });
  }
  
  void start() { timer_->start(); }
  void stop() { timer_->stop(); }
  
private:
  void updateCPU() {
    QProcess process;
    process.start("sh", QStringList() << "-c" 
      << "top -bn1 | grep 'Cpu(s)' | awk '{print $2}'");
    if (process.waitForFinished(500)) {
      QString output = QString::fromUtf8(process.readAllStandardOutput()).trimmed();
      cpuLabel_->setText(QString("CPU: %1%").arg(output));
    }
  }
  
  void updateMemory() {
    QProcess process;
    process.start("sh", QStringList() << "-c" 
      << "free -m | awk 'NR==2{printf \"%.1fG/%.1fG\", $3/1024, $2/1024}'");
    if (process.waitForFinished(500)) {
      QString output = QString::fromUtf8(process.readAllStandardOutput()).trimmed();
      memLabel_->setText(QString("MEM: %1").arg(output));
    }
  }
  
  void updateGPU() {
    QProcess process;
    process.start("sh", QStringList() << "-c" 
      << "nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader 2>/dev/null | head -1");
    if (process.waitForFinished(500)) {
      QString output = QString::fromUtf8(process.readAllStandardOutput()).trimmed();
      if (!output.isEmpty()) {
        gpuLabel_->setText(QString("GPU: %1").arg(output));
      } else {
        gpuLabel_->setText("GPU: N/A");
      }
    }
  }
  
  void updateNetworkBase() {
    // 与「系统状态」页一致：汇总除 lo 外所有接口（避免仅匹配 eth0/enp* 时无输出）
    unsigned long long rx = 0, tx = 0;
    if (ui::ReadNetworkBytes(rx, tx)) {
      lastRxBytes_ = rx;
      lastTxBytes_ = tx;
    }
  }

  void updateNetwork() {
    unsigned long long cur_rx = 0, cur_tx = 0;
    if (!ui::ReadNetworkBytes(cur_rx, cur_tx)) {
      return;
    }
    const unsigned long long rxRate =
      (cur_rx >= lastRxBytes_) ? (cur_rx - lastRxBytes_) : cur_rx;
    const unsigned long long txRate =
      (cur_tx >= lastTxBytes_) ? (cur_tx - lastTxBytes_) : cur_tx;
    lastRxBytes_ = cur_rx;
    lastTxBytes_ = cur_tx;

    QString rxText, txText;
    // Qt 的 arg() 使用 %1/%2 占位符，不能写 printf 风格的 %.1f
    if (rxRate > 1024 * 1024) {
      rxText = QStringLiteral("↓ %1 MB/s").arg(rxRate / (1024.0 * 1024.0), 0, 'f', 1);
    } else if (rxRate > 1024) {
      rxText = QStringLiteral("↓ %1 KB/s").arg(rxRate / 1024.0, 0, 'f', 1);
    } else {
      rxText = QStringLiteral("↓ %1 B/s").arg(rxRate);
    }
    if (txRate > 1024 * 1024) {
      txText = QStringLiteral("↑ %1 MB/s").arg(txRate / (1024.0 * 1024.0), 0, 'f', 1);
    } else if (txRate > 1024) {
      txText = QStringLiteral("↑ %1 KB/s").arg(txRate / 1024.0, 0, 'f', 1);
    } else {
      txText = QStringLiteral("↑ %1 B/s").arg(txRate);
    }
    if (netDownLabel_) {
      netDownLabel_->setText(rxText);
    }
    if (netUpLabel_) {
      netUpLabel_->setText(txText);
    }
  }
  
  QLabel* cpuLabel_;
  QLabel* memLabel_;
  QLabel* gpuLabel_;
  QLabel* netUpLabel_;
  QLabel* netDownLabel_;
  unsigned long long lastRxBytes_;
  unsigned long long lastTxBytes_;
  QTimer* timer_;
};

/// 创建坐标转换页面
QWidget* createCoordinateConverterPage() {
  QWidget* page = new QWidget();
  QVBoxLayout* mainLayout = new QVBoxLayout(page);
  
  // 标题
  QLabel* titleLabel = new QLabel("坐标转换器", page);
  titleLabel->setStyleSheet("font-size: 20px; font-weight: bold; color: #2c3e50;");
  mainLayout->addWidget(titleLabel);
  
  // 转换方向选择
  QHBoxLayout* dirLayout = new QHBoxLayout();
  QLabel* dirLabel = new QLabel("转换方向:", page);
  QComboBox* dirComboBox = new QComboBox(page);
  dirComboBox->addItem("RPY → 四元数");
  dirComboBox->addItem("四元数 → RPY");
  dirLayout->addWidget(dirLabel);
  dirLayout->addWidget(dirComboBox);
  dirLayout->addStretch();
  mainLayout->addLayout(dirLayout);
  
  // 创建两个面板：输入和输出
  QHBoxLayout* contentLayout = new QHBoxLayout();
  
  // 输入面板
  QGroupBox* inputGroup = new QGroupBox("输入", page);
  QVBoxLayout* inputLayout = new QVBoxLayout(inputGroup);
  
  QFormLayout* inputForm = new QFormLayout();
  
  // RPY 输入
  QDoubleSpinBox* rollInput = new QDoubleSpinBox();
  rollInput->setRange(-360, 360);
  rollInput->setDecimals(4);
  rollInput->setValue(0);
  rollInput->setSuffix(" deg");
  inputForm->addRow("Roll (X):", rollInput);
  
  QDoubleSpinBox* pitchInput = new QDoubleSpinBox();
  pitchInput->setRange(-360, 360);
  pitchInput->setDecimals(4);
  pitchInput->setValue(0);
  pitchInput->setSuffix(" deg");
  inputForm->addRow("Pitch (Y):", pitchInput);
  
  QDoubleSpinBox* yawInput = new QDoubleSpinBox();
  yawInput->setRange(-360, 360);
  yawInput->setDecimals(4);
  yawInput->setValue(0);
  yawInput->setSuffix(" deg");
  inputForm->addRow("Yaw (Z):", yawInput);
  
  // 四元数输入
  QDoubleSpinBox* qwInput = new QDoubleSpinBox();
  qwInput->setRange(-10, 10);
  qwInput->setDecimals(6);
  qwInput->setValue(1.0);
  inputForm->addRow("w:", qwInput);
  
  QDoubleSpinBox* qxInput = new QDoubleSpinBox();
  qxInput->setRange(-10, 10);
  qxInput->setDecimals(6);
  qxInput->setValue(0);
  inputForm->addRow("x:", qxInput);
  
  QDoubleSpinBox* qyInput = new QDoubleSpinBox();
  qyInput->setRange(-10, 10);
  qyInput->setDecimals(6);
  qyInput->setValue(0);
  inputForm->addRow("y:", qyInput);
  
  QDoubleSpinBox* qzInput = new QDoubleSpinBox();
  qzInput->setRange(-10, 10);
  qzInput->setDecimals(6);
  qzInput->setValue(0);
  inputForm->addRow("z:", qzInput);
  
  inputLayout->addLayout(inputForm);
  
  // 切换输入显示
  auto updateInputVisibility = [=]() {
    bool toQuaternion = (dirComboBox->currentIndex() == 0);
    rollInput->setVisible(toQuaternion);
    pitchInput->setVisible(toQuaternion);
    yawInput->setVisible(toQuaternion);
    qwInput->setVisible(!toQuaternion);
    qxInput->setVisible(!toQuaternion);
    qyInput->setVisible(!toQuaternion);
    qzInput->setVisible(!toQuaternion);
    
    // 更新标签
    inputForm->labelForField(rollInput)->setVisible(toQuaternion);
    inputForm->labelForField(pitchInput)->setVisible(toQuaternion);
    inputForm->labelForField(yawInput)->setVisible(toQuaternion);
    inputForm->labelForField(qwInput)->setVisible(!toQuaternion);
    inputForm->labelForField(qxInput)->setVisible(!toQuaternion);
    inputForm->labelForField(qyInput)->setVisible(!toQuaternion);
    inputForm->labelForField(qzInput)->setVisible(!toQuaternion);
  };
  
  QObject::connect(dirComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
                   [=](int){ updateInputVisibility(); });
  
  updateInputVisibility();
  
  contentLayout->addWidget(inputGroup, 1);
  
  // 输出面板
  QGroupBox* outputGroup = new QGroupBox("输出", page);
  QVBoxLayout* outputLayout = new QVBoxLayout(outputGroup);
  
  QTextEdit* outputText = new QTextEdit(outputGroup);
  outputText->setReadOnly(true);
  outputText->setStyleSheet("font-family: monospace; font-size: 13px; background-color: #ecf0f1;");
  outputLayout->addWidget(outputText);
  
  contentLayout->addWidget(outputGroup, 1);
  
  mainLayout->addLayout(contentLayout);
  
  // 转换按钮
  QPushButton* convertBtn = new QPushButton("转换");
  convertBtn->setFixedHeight(40);
  convertBtn->setStyleSheet(
    "QPushButton { background-color: #3498db; color: white; border-radius: 5px; font-size: 16px; }"
    "QPushButton:hover { background-color: #2980b9; }"
  );
  mainLayout->addWidget(convertBtn);
  
  // 转换逻辑
  QObject::connect(convertBtn, &QPushButton::clicked, [=]() {
    bool toQuaternion = (dirComboBox->currentIndex() == 0);
    
    try {
      QString result;
      
      if (toQuaternion) {
        // RPY → 四元数
        double roll = rollInput->value() * M_PI / 180.0;      // deg → rad
        double pitch = pitchInput->value() * M_PI / 180.0;
        double yaw = yawInput->value() * M_PI / 180.0;
        
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        
        double qw = cr * cp * cy + sr * sp * sy;
        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;
        
        result = QString(
          "四元数结果:\n"
          "w = %1\n"
          "x = %2\n"
          "y = %3\n"
          "z = %4\n\n"
          "归一化检查: %5\n"
          "(应接近 1.0)"
        ).arg(qw, 0, 'f', 6)
         .arg(qx, 0, 'f', 6)
         .arg(qy, 0, 'f', 6)
         .arg(qz, 0, 'f', 6)
         .arg(qw*qw + qx*qx + qy*qy + qz*qz, 0, 'f', 6);
        
      } else {
        // 四元数 → RPY
        double qw = qwInput->value();
        double qx = qxInput->value();
        double qy = qyInput->value();
        double qz = qzInput->value();
        
        // 归一化
        double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm < 1e-10) {
          QMessageBox::warning(page, "警告", "四元数不能为零向量");
          return;
        }
        qw /= norm; qx /= norm; qy /= norm; qz /= norm;
        
        // 计算 RPY
        double sinr_cosp = 2 * (qw * qx + qy * qz);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        double roll = std::atan2(sinr_cosp, cosr_cosp);
        
        double sinp = 2 * (qw * qy - qz * qx);
        double pitch;
        if (std::abs(sinp) >= 1)
          pitch = std::copysign(M_PI / 2, sinp);  // 万向锁
        else
          pitch = std::asin(sinp);
        
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        
        // rad → deg
        roll *= 180.0 / M_PI;
        pitch *= 180.0 / M_PI;
        yaw *= 180.0 / M_PI;
        
        result = QString(
          "RPY 结果 (度):\n"
          "Roll  (X) = %1°\n"
          "Pitch (Y) = %2°\n\n"
          "Yaw   (Z) = %3°\n\n"
          "RPY 结果 (弧度):\n"
          "Roll  (X) = %4 rad\n"
          "Pitch (Y) = %5 rad\n"
          "Yaw   (Z) = %6 rad\n\n"
          "四元数归一化: %7\n"
          "(应接近 1.0)"
        ).arg(roll, 0, 'f', 4)
         .arg(pitch, 0, 'f', 4)
         .arg(yaw, 0, 'f', 4)
         .arg(roll * M_PI / 180.0, 0, 'f', 6)
         .arg(pitch * M_PI / 180.0, 0, 'f', 6)
         .arg(yaw * M_PI / 180.0, 0, 'f', 6)
         .arg(qw*qw + qx*qx + qy*qy + qz*qz, 0, 'f', 6);
      }
      
      outputText->setText(result);
      
    } catch (const std::exception& e) {
      QMessageBox::critical(page, "错误", QString("转换失败: %1").arg(e.what()));
    }
  });
  
  return page;
}

/// 创建运动学求解器页面
QWidget* createKinematicsSolverPage() {
  QWidget* page = new QWidget();
  QVBoxLayout* mainLayout = new QVBoxLayout(page);
  QLabel* titleLabel = new QLabel("运动学模块已移除", page);
  titleLabel->setAlignment(Qt::AlignCenter);
  titleLabel->setStyleSheet("font-size: 20px; color: #7f8c8d;");
  mainLayout->addWidget(titleLabel);

  return page;
}

/// 创建主窗口
QMainWindow* createMainWindow() {
  QMainWindow* window = new QMainWindow();
  window->setWindowTitle("Robot Workbench");
  window->resize(1200, 700);
  
  // 中心部件
  QWidget* centralWidget = new QWidget(window);
  window->setCentralWidget(centralWidget);
  
  // 左侧 Kit 导航 + 页面堆叠
  QStackedWidget* stackedWidget = new QStackedWidget(centralWidget);
  QStringList buttonNames;
  int image_viewer_index = -1;
  QWidget* navPanel = ui::BuildKitNavigationPanel(stackedWidget, &buttonNames, &image_viewer_index);

  auto sync_image_active = [stackedWidget, image_viewer_index](bool on) {
    if (image_viewer_index < 0) {
      return;
    }
    QWidget * page = stackedWidget->widget(image_viewer_index);
    if (on && page) {
      if (auto * lazy = dynamic_cast<ui::LazyFeaturePage *>(page)) {
        lazy->ensureBuilt();
      }
    }
    if (auto * lazy = dynamic_cast<ui::LazyFeaturePage *>(stackedWidget->widget(image_viewer_index))) {
#if WORKBENCH_KIT_GENERAL
      if (auto * iv = dynamic_cast<ui::ImageViewerWidget *>(lazy->content())) {
        iv->SetActive(on);
      }
#endif
    }
  };
  QObject::connect(stackedWidget, QOverload<int>::of(&QStackedWidget::currentChanged), [stackedWidget, sync_image_active, image_viewer_index](int index) {
    if (auto * sys = dynamic_cast<ui::SystemStatusWidget *>(stackedWidget->widget(0))) {
      sys->SetActive(index == 0);
    }
    sync_image_active(index == image_viewer_index);
  });
  if (auto * sys = dynamic_cast<ui::SystemStatusWidget *>(stackedWidget->widget(0))) {
    sys->SetActive(true);
  }
  sync_image_active(false);

  QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);
  mainLayout->setSpacing(0);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->addWidget(navPanel);
  mainLayout->addWidget(stackedWidget, 1);
  
  // 状态栏 - 系统监控
  QStatusBar* statusBar = window->statusBar();
  QLabel* cpuLabel = new QLabel("CPU: --");
  QLabel* memLabel = new QLabel("MEM: --");
  QLabel* gpuLabel = new QLabel("GPU: --");
  QLabel* netDownLabel = new QLabel("↓ --");
  QLabel* netUpLabel = new QLabel("↑ --");
  
  statusBar->addWidget(cpuLabel);
  statusBar->addWidget(memLabel);
  statusBar->addWidget(gpuLabel);
  statusBar->addPermanentWidget(netDownLabel);
  statusBar->addPermanentWidget(netUpLabel);
  
  // 启动系统监控
  SystemMonitor* monitor = new SystemMonitor(cpuLabel, memLabel, gpuLabel,
                                              netUpLabel, netDownLabel);
  monitor->start();

  SetupWorkbenchMenuBar(window, stackedWidget, buttonNames);

  return window;
}

RosRobotWorkbenchNode::RosRobotWorkbenchNode(const rclcpp::NodeOptions & options)
: Node("ros_robot_workbench_node", options)
{
  RCLCPP_INFO(get_logger(), "ros_robot_workbench node started");
}

int RunRosRobotWorkbenchUiApp(int argc, char ** argv) {
  // 初始化 Qt 应用
  QApplication app(argc, argv);
  AppPreferences prefs;
  LoadAppPreferences(&prefs);
  ApplyUiThemeToApplication(app, prefs.ui_theme);
  
  // 创建并显示主窗口
  QMainWindow* window = createMainWindow();
  window->show();
  
  // 运行 Qt 事件循环
  int result = app.exec();

  delete window;
  ui::SharedUiExecutor::instance().shutdown();
  return result;
}

}  // namespace ros_robot_workbench
