// Nova Robot Tools UI - 参考 calib_sim 架构
// 所有 Qt 代码在 .cpp 中实现，不使用 Q_OBJECT 和 AUTOMOC

#include "ros_robot_assist_tools/ros_robot_assist_tools_ui.hpp"

#include <QApplication>
#include <QMainWindow>
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

#include "ros_robot_assist_tools/ui/board_generator_widget.h"
#include "ros_robot_assist_tools/ui/handeye_calibration_widget.h"
#include "ros_robot_assist_tools/ui/intrinsic_calibration_widget.h"
#include "ros_robot_assist_tools/ui/kinematics_solver_widget.h"
#include "ros_robot_assist_tools/ui/multi_sensor_calibration_widget.h"
#include "ros_robot_assist_tools/ui/pose_transform_widget.h"
#include "ros_robot_assist_tools/ui/stereo_calibration_widget.h"
#include "ros_robot_assist_tools/ui/system_status_widget.h"
#include "ros_robot_assist_tools/ui/tf_viewer_widget.h"

namespace ros_robot_assist_tools
{

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
    // 获取初始网络字节数
    QProcess process;
    process.start("sh", QStringList() << "-c" << "cat /proc/net/dev | grep -E 'eth0|enp' | head -n1");
    if (process.waitForFinished(500)) {
      QString output = QString::fromUtf8(process.readAllStandardOutput()).trimmed();
      QStringList parts = output.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
      if (parts.size() >= 10) {
        lastRxBytes_ = parts[1].toULongLong();  // 接收字节
        lastTxBytes_ = parts[9].toULongLong();  // 发送字节
      }
    }
  }
  
  void updateNetwork() {
    QProcess process;
    process.start("sh", QStringList() << "-c" << "cat /proc/net/dev | grep -E 'eth0|enp' | head -n1");
    if (process.waitForFinished(500)) {
      QString output = QString::fromUtf8(process.readAllStandardOutput()).trimmed();
      QStringList parts = output.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
      
      if (parts.size() >= 10) {
        unsigned long long currentRx = parts[1].toULongLong();
        unsigned long long currentTx = parts[9].toULongLong();
        
        // 计算速率 (bytes/s)
        unsigned long long rxRate = currentRx - lastRxBytes_;
        unsigned long long txRate = currentTx - lastTxBytes_;
        
        lastRxBytes_ = currentRx;
        lastTxBytes_ = currentTx;
        
        // 格式化显示
        QString rxText, txText;
        if (rxRate > 1024 * 1024) {
          rxText = QString("↓ %.1f MB/s").arg(rxRate / (1024.0 * 1024.0));
        } else if (rxRate > 1024) {
          rxText = QString("↓ %.1f KB/s").arg(rxRate / 1024.0);
        } else {
          rxText = QString("↓ %1 B/s").arg(rxRate);
        }
        
        if (txRate > 1024 * 1024) {
          txText = QString("↑ %.1f MB/s").arg(txRate / (1024.0 * 1024.0));
        } else if (txRate > 1024) {
          txText = QString("↑ %.1f KB/s").arg(txRate / 1024.0);
        } else {
          txText = QString("↑ %1 B/s").arg(txRate);
        }
        
        if (netDownLabel_) netDownLabel_->setText(rxText);
        if (netUpLabel_) netUpLabel_->setText(txText);
      }
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
  window->setWindowTitle("Assist Tool");
  window->resize(1200, 800);
  
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
  
  // 创建功能页面（按需求保留并排序）
  auto * systemStatusWidget = new ui::SystemStatusWidget();
  stackedWidget->addWidget(systemStatusWidget);  // 系统状态
  stackedWidget->addWidget(new ui::BoardGeneratorWidget());  // 标定板生成
  stackedWidget->addWidget(new ui::PoseTransformWidget());  // 姿态转换
  stackedWidget->addWidget(new ui::KinematicsSolverWidget());  // 运动学计算
  stackedWidget->addWidget(new ui::TfViewerWidget());  // TF查看
  stackedWidget->addWidget(new ui::IntrinsicCalibrationWidget());  // 内参标定
  stackedWidget->addWidget(new ui::StereoCalibrationWidget());  // 双目标定
  stackedWidget->addWidget(new ui::MultiSensorCalibrationWidget());  // 多传感器标定
  stackedWidget->addWidget(new ui::HandeyeCalibrationWidget());  // 手眼标定
  
  // 创建导航按钮
  QStringList buttonNames = {
    "系统状态", "标定板生成", "姿态转换", "运动学计算", "TF查看",
    "内参标定", "双目标定", "多传感器标定", "手眼标定"
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

  QObject::connect(stackedWidget, QOverload<int>::of(&QStackedWidget::currentChanged), [systemStatusWidget](int index) {
    systemStatusWidget->SetActive(index == 0);
  });
  systemStatusWidget->SetActive(true);
  
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
  app.setStyle("Fusion");
  
  // 创建并显示主窗口
  QMainWindow* window = createMainWindow();
  window->show();
  
  // 运行 Qt 事件循环
  int result = app.exec();
  
  delete window;
  return result;
}

}  // namespace ros_robot_assist_tools
