// Assist Tool UI：Qt 实现在 .cpp 中，不使用 Q_OBJECT / AUTOMOC

#include "ros_robot_assist_tools/ros_robot_assist_tools_ui.hpp"

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
#include <QMessageBox>
#include <QDir>

#include <QtGlobal>

#include "ros_robot_assist_tools/ui/board_generator_widget.h"
#include "ros_robot_assist_tools/ui/image_viewer_widget.h"
#include "ros_robot_assist_tools/ui/lazy_feature_page.hpp"
#include "ros_robot_assist_tools/ui/kinematics_solver_widget.h"
#include "ros_robot_assist_tools/ui/pose_transform_widget.h"
#include "ros_robot_assist_tools/ui/system_status_widget.h"
#include "ros_robot_assist_tools/ui/tf_viewer_widget.h"
#include "ros_robot_assist_tools/ui/shared_ui_executor.hpp"
#include "ros_robot_assist_tools/ui/preferences_dialog.h"
#include "ros_robot_assist_tools/module/system_status_module.h"
#include "ros_robot_assist_tools/preferences/app_preferences.hpp"

namespace ros_robot_assist_tools
{
namespace
{

QString ResolveShareConfigDir()
{
  const QString exe_dir = QCoreApplication::applicationDirPath();
  const QStringList candidates = {
    exe_dir + "/../../share/ros_robot_assist_tools/config",
    QDir::current().absoluteFilePath("../share/ros_robot_assist_tools/config"),
    QDir::current().absoluteFilePath("install/ros_robot_assist_tools/share/ros_robot_assist_tools/config"),
    QDir::current().absoluteFilePath("simulation/install/ros_robot_assist_tools/share/ros_robot_assist_tools/config"),
    QDir::current().absoluteFilePath("src/ros_robot_assist_tools/config"),
  };
  for (const QString & rel : candidates) {
    const QString canon = QDir(rel).canonicalPath();
    if (!canon.isEmpty() && QFileInfo(canon).isDir()) {
      return canon;
    }
  }
  return QDir(exe_dir + "/../../share/ros_robot_assist_tools/config").canonicalPath();
}

QString ResolveHelpPdfPath()
{
  const QString exe_dir = QCoreApplication::applicationDirPath();
  const QStringList candidates = {
    exe_dir + "/../../share/ros_robot_assist_tools/assets/docs/AssistTool_UserGuide.pdf",
    QDir::current().absoluteFilePath("src/ros_robot_assist_tools/assets/docs/AssistTool_UserGuide.pdf"),
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

void SetupAssistToolMenuBar(QMainWindow * window, QStackedWidget * stack, const QStringList & page_names)
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
        QStringLiteral("未找到 share/ros_robot_assist_tools/config。\n"
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
  QAction * a_about = m_help->addAction(QStringLiteral("About Assist Tool(&A)..."));
  QObject::connect(a_about, &QAction::triggered, [window]() {
    QMessageBox::about(
      window, QStringLiteral("About Assist Tool"),
      QStringLiteral(
        "<h3>Assist Tool</h3>"
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

/// 创建主窗口
QMainWindow* createMainWindow() {
  QMainWindow* window = new QMainWindow();
  window->setWindowTitle("Assist Tool");
  window->resize(1200, 700);
  
  // 中心部件
  QWidget* centralWidget = new QWidget(window);
  window->setCentralWidget(centralWidget);
  
  QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);
  mainLayout->setSpacing(0);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  
  // 左侧导航面板
  QWidget* navPanel = new QWidget(centralWidget);
  navPanel->setFixedWidth(150);
  navPanel->setStyleSheet("background-color: #2c3e50;");
  
  QVBoxLayout* navLayout = new QVBoxLayout(navPanel);
  navLayout->setSpacing(10);
  navLayout->setContentsMargins(10, 20, 10, 20);
  
  // 标题
  QLabel* titleLabel = new QLabel("Assist Tool", navPanel);
  titleLabel->setStyleSheet("color: white; font-size: 18px; font-weight: bold;");
  titleLabel->setAlignment(Qt::AlignCenter);
  navLayout->addWidget(titleLabel);
  navLayout->addSpacing(20);
  
  // 页面堆叠
  QStackedWidget* stackedWidget = new QStackedWidget(centralWidget);
  
  // 仅首屏立即构造；其余模块首次进入页面时再构造，减少启动时 ROS/DDS 初始化开销
  auto * systemStatusWidget = new ui::SystemStatusWidget();
  auto * lazy_image = new ui::LazyFeaturePage([](QWidget * p) { return new ui::ImageViewerWidget(p); });
  auto * lazy_board = new ui::LazyFeaturePage([](QWidget * p) { return new ui::BoardGeneratorWidget(p); });
  auto * lazy_pose = new ui::LazyFeaturePage([](QWidget * p) { return new ui::PoseTransformWidget(p); });
  auto * lazy_kin = new ui::LazyFeaturePage([](QWidget * p) { return new ui::KinematicsSolverWidget(p); });
  auto * lazy_tf = new ui::LazyFeaturePage([](QWidget * p) { return new ui::TfViewerWidget(p); });

  stackedWidget->addWidget(systemStatusWidget);
  stackedWidget->addWidget(lazy_image);
  stackedWidget->addWidget(lazy_board);
  stackedWidget->addWidget(lazy_pose);
  stackedWidget->addWidget(lazy_kin);
  stackedWidget->addWidget(lazy_tf);
  
  // 创建导航按钮
  QStringList buttonNames = {
    "系统状态", "图像查看", "标定板生成", "姿态转换", "运动学计算", "TF查看"
  };
  for (int i = 0; i < buttonNames.size(); ++i) {
    QPushButton* btn = new QPushButton(buttonNames[i], navPanel);
    btn->setFixedHeight(40);
    btn->setStyleSheet(
      "QPushButton { background-color: #34495e; color: white; border: none; "
      "border-radius: 5px; font-size: 14px; }"
      "QPushButton:hover { background-color: #1abc9c; }"
    );
    
    QObject::connect(btn, &QPushButton::clicked, [stackedWidget, i]() {
      stackedWidget->setCurrentIndex(i);
    });
    
    navLayout->addWidget(btn);
  }

  auto sync_image_active = [lazy_image](bool on) {
    if (on) {
      lazy_image->ensureBuilt();
    }
    if (auto * iv = dynamic_cast<ui::ImageViewerWidget *>(lazy_image->content())) {
      iv->SetActive(on);
    }
  };
  QObject::connect(stackedWidget, QOverload<int>::of(&QStackedWidget::currentChanged), [systemStatusWidget, sync_image_active](int index) {
    systemStatusWidget->SetActive(index == 0);
    sync_image_active(index == 1);
  });
  systemStatusWidget->SetActive(true);
  sync_image_active(false);
  
  navLayout->addStretch();
  
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

  SetupAssistToolMenuBar(window, stackedWidget, buttonNames);

  return window;
}

RosRobotAssistToolsNode::RosRobotAssistToolsNode(const rclcpp::NodeOptions & options)
: Node("ros_robot_assist_tools_node", options)
{
  RCLCPP_INFO(get_logger(), "ros_robot_assist_tools node started");
}

int RunRosRobotAssistToolsUiApp(int argc, char ** argv) {
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

}  // namespace ros_robot_assist_tools
