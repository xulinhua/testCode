// Nova Robot Tools UI - 参考 calib_sim 架构
// 所有 Qt 代码在 .cpp 中实现，不使用 Q_OBJECT 和 AUTOMOC

#include "nova_robot_tools/nova_robot_tools_ui.hpp"

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
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frame.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace nova_robot_tools
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

/// 创建 ArUco 生成页面
QWidget* createArucoGeneratorPage() {
  QWidget* page = new QWidget();
  QVBoxLayout* mainLayout = new QVBoxLayout(page);
  
  // 标题
  QLabel* titleLabel = new QLabel("ArUco 码生成器", page);
  titleLabel->setStyleSheet("font-size: 20px; font-weight: bold; color: #2c3e50;");
  mainLayout->addWidget(titleLabel);
  
  // 控制面板
  QGroupBox* controlGroup = new QGroupBox("参数设置", page);
  QFormLayout* formLayout = new QFormLayout(controlGroup);
  
  // 字典类型
  QComboBox* dictComboBox = new QComboBox();
  dictComboBox->addItem("DICT_4x4_50", 0);
  dictComboBox->addItem("DICT_4x4_100", 1);
  dictComboBox->addItem("DICT_4x4_250", 2);
  dictComboBox->addItem("DICT_4x4_1000", 3);
  dictComboBox->addItem("DICT_5x5_50", 4);
  dictComboBox->addItem("DICT_5x5_100", 5);
  dictComboBox->addItem("DICT_5x5_250", 6);
  dictComboBox->addItem("DICT_5x5_1000", 7);
  dictComboBox->addItem("DICT_6x6_50", 8);
  dictComboBox->addItem("DICT_6x6_100", 9);
  dictComboBox->addItem("DICT_6x6_250", 10);
  dictComboBox->addItem("DICT_6x6_1000", 11);
  dictComboBox->addItem("DICT_7x7_50", 12);
  dictComboBox->addItem("DICT_7x7_100", 13);
  dictComboBox->addItem("DICT_7x7_250", 14);
  dictComboBox->addItem("DICT_7x7_1000", 15);
  dictComboBox->addItem("DICT_ARUCO_ORIGINAL", 16);
  dictComboBox->addItem("DICT_APRILTAG_16h5", 17);
  dictComboBox->addItem("DICT_APRILTAG_25h9", 18);
  dictComboBox->addItem("DICT_APRILTAG_36h10", 19);
  dictComboBox->addItem("DICT_APRILTAG_36h11", 20);
  dictComboBox->setFixedWidth(200);
  formLayout->addRow("字典类型:", dictComboBox);
  
  // Marker ID - 根据字典类型动态调整范围
  QSpinBox* idSpinBox = new QSpinBox();
  idSpinBox->setRange(0, 49);
  idSpinBox->setValue(0);
  formLayout->addRow("Marker ID:", idSpinBox);
  
  // 字典类型变化时更新 ID 范围
  QObject::connect(dictComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int index) {
    int dictType = dictComboBox->itemData(index).toInt();
    int maxId = 49;  // 默认
    
    if (dictType >= 0 && dictType <= 3) maxId = 49;      // 4x4_50
    else if (dictType == 1 || dictType == 5 || dictType == 9 || dictType == 13) maxId = 99;  // _100
    else if (dictType == 2 || dictType == 6 || dictType == 10 || dictType == 14) maxId = 249;  // _250
    else if (dictType == 3 || dictType == 7 || dictType == 11 || dictType == 15) maxId = 999;  // _1000
    else if (dictType == 16) maxId = 1023;  // ARUCO_ORIGINAL
    else if (dictType == 17) maxId = 34;    // APRILTAG_16h5
    else if (dictType == 18) maxId = 241;   // APRILTAG_25h9
    else if (dictType == 19) maxId = 999;   // APRILTAG_36h10
    else if (dictType == 20) maxId = 34;    // APRILTAG_36h11 (实际 0-34)
    
    int currentId = idSpinBox->value();
    idSpinBox->setRange(0, maxId);
    if (currentId > maxId) {
      idSpinBox->setValue(0);
    }
  });
  
  // 图像尺寸
  QSpinBox* sizeSpinBox = new QSpinBox();
  sizeSpinBox->setRange(200, 1000);
  sizeSpinBox->setValue(400);
  sizeSpinBox->setSuffix(" px");
  formLayout->addRow("图像尺寸:", sizeSpinBox);
  
  mainLayout->addWidget(controlGroup);
  
  // 按钮
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  QPushButton* generateBtn = new QPushButton("生成");
  generateBtn->setFixedHeight(35);
  generateBtn->setStyleSheet(
    "QPushButton { background-color: #3498db; color: white; border-radius: 5px; font-size: 14px; }"
    "QPushButton:hover { background-color: #2980b9; }"
  );
  
  QPushButton* exportBtn = new QPushButton("导出");
  exportBtn->setFixedHeight(35);
  exportBtn->setEnabled(false);
  exportBtn->setStyleSheet(
    "QPushButton { background-color: #27ae60; color: white; border-radius: 5px; font-size: 14px; }"
    "QPushButton:hover { background-color: #229954; }"
    "QPushButton:disabled { background-color: #95a5a6; }"
  );
  
  buttonLayout->addWidget(generateBtn);
  buttonLayout->addWidget(exportBtn);
  mainLayout->addLayout(buttonLayout);
  
  // 预览区域
  QLabel* previewLabel = new QLabel("点击“生成”按钮创建 ArUco 码", page);
  previewLabel->setMinimumSize(400, 400);
  previewLabel->setAlignment(Qt::AlignCenter);
  previewLabel->setStyleSheet("background-color: #ecf0f1; border: 2px solid #bdc3c7;");
  mainLayout->addWidget(previewLabel);
  
  // 存储生成的图像
  cv::Mat* currentMarker = new cv::Mat();
  
  // 生成按钮回调
  QObject::connect(generateBtn, &QPushButton::clicked, [=]() {
    int dictType = dictComboBox->currentData().toInt();
    int markerId = idSpinBox->value();
    int size = sizeSpinBox->value();
    
    try {
      // 获取字典
      cv::aruco::Dictionary dictionary;
      switch (dictType) {
        case 0:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
          break;
        case 1:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
          break;
        case 2:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
          break;
        case 3:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
          break;
        case 4:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
          break;
        case 5:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
          break;
        case 6:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
          break;
        case 7:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
          break;
        case 8:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
          break;
        case 9:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
          break;
        case 10:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
          break;
        case 11:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
          break;
        case 12:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
          break;
        case 13:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
          break;
        case 14:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
          break;
        case 15:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
          break;
        case 16:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
          break;
        case 17:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
          break;
        case 18:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);
          break;
        case 19:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h10);
          break;
        case 20:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
          break;
        default:
          dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
          break;
      }
      
      // 生成标记
      dictionary.generateImageMarker(markerId, size, *currentMarker);
      
      // 转换为 QImage 并显示
      QImage image(currentMarker->data, currentMarker->cols, currentMarker->rows,
                   currentMarker->step, QImage::Format_Grayscale8);
      QPixmap pixmap = QPixmap::fromImage(image);
      previewLabel->setPixmap(pixmap.scaled(400, 400, Qt::KeepAspectRatio));
      
      exportBtn->setEnabled(true);
    } catch (const std::exception& e) {
      QMessageBox::critical(page, "错误", QString("生成失败: %1").arg(e.what()));
    }
  });
  
  // 导出按钮回调
  QObject::connect(exportBtn, &QPushButton::clicked, [=]() {
    if (currentMarker->empty()) {
      QMessageBox::warning(page, "警告", "请先生成 ArUco 码");
      return;
    }
    
    // 选择保存格式
    QStringList formats;
    formats << "PNG (*.png)" << "JPG (*.jpg)" << "BMP (*.bmp)" << "DAE (*.dae)";
    
    bool ok;
    QString format = QInputDialog::getItem(page, "选择格式", "导出格式:", formats, 0, false, &ok);
    if (!ok) return;
    
    // 选择保存路径
    // 获取字典类型名称
    QString dictName = dictComboBox->currentText();
    int markerId = idSpinBox->value();
    int imageSize = sizeSpinBox->value();
    QString defaultName = QString("%1_mark%2_%3").arg(dictName).arg(markerId).arg(imageSize);
    QString ext = format.split(" ")[1].remove("(*").remove(")");
    QString defaultPath = QString("%1/%2.%3").arg(QDir::homePath()).arg(defaultName).arg(ext);
    
    QString filePath = QFileDialog::getSaveFileName(page, "保存文件", defaultPath, format);
    if (filePath.isEmpty()) return;
    
    try {
      bool success = false;
      
      if (ext == ".dae") {
        // 导出 DAE (Collada) 格式
        // 先将图像转为 base64
        std::vector<uchar> buffer;
        cv::imencode(".png", *currentMarker, buffer);
        QByteArray imageData = QByteArray(reinterpret_cast<const char*>(buffer.data()), buffer.size()).toBase64();
        
        // 生成 DAE 文件
        QFile file(filePath);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
          QTextStream out(&file);
          out.setCodec("UTF-8");
          
          double sizeMeters = 0.1;  // 默认 10cm
          double half = sizeMeters / 2.0;
          
          out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
          out << "<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">\n";
          out << "  <asset>\n";
          out << "    <contributor><author>Nova Robot Tools</author></contributor>\n";
          out << "    <created>" << QDateTime::currentDateTime().toString(Qt::ISODate) << "</created>\n";
          out << "    <modified>" << QDateTime::currentDateTime().toString(Qt::ISODate) << "</modified>\n";
          out << "  </asset>\n";
          
          // 图像数据
          out << "  <library_images>\n";
          out << "    <image id=\"aruco_marker\" name=\"aruco_marker\">\n";
          out << "      <data>" << imageData << "</data>\n";
          out << "    </image>\n";
          out << "  </library_images>\n";
          
          // 几何体
          out << "  <library_geometries>\n";
          out << "    <geometry id=\"marker_mesh\" name=\"marker_mesh\">\n";
          out << "      <mesh>\n";
          out << "        <source id=\"positions\">\n";
          out << "          <float_array id=\"positions-array\" count=\"12\">";
          out << " " << -half << " " << -half << " 0"
              << " " << half << " " << -half << " 0"
              << " " << half << " " << half << " 0"
              << " " << -half << " " << half << " 0";
          out << " </float_array>\n";
          out << "          <technique_common>\n";
          out << "            <accessor source=\"#positions-array\" count=\"4\" stride=\"3\">\n";
          out << "              <param name=\"X\" type=\"float\"/>\n";
          out << "              <param name=\"Y\" type=\"float\"/>\n";
          out << "              <param name=\"Z\" type=\"float\"/>\n";
          out << "            </accessor>\n";
          out << "          </technique_common>\n";
          out << "        </source>\n";
          
          out << "        <source id=\"texcoords\">\n";
          out << "          <float_array id=\"texcoords-array\" count=\"8\"> 0 0 1 0 1 1 0 1 </float_array>\n";
          out << "          <technique_common>\n";
          out << "            <accessor source=\"#texcoords-array\" count=\"4\" stride=\"2\">\n";
          out << "              <param name=\"S\" type=\"float\"/>\n";
          out << "              <param name=\"T\" type=\"float\"/>\n";
          out << "            </accessor>\n";
          out << "          </technique_common>\n";
          out << "        </source>\n";
          
          out << "        <vertices id=\"vertices\">\n";
          out << "          <input semantic=\"POSITION\" source=\"#positions\"/>\n";
          out << "        </vertices>\n";
          
          out << "        <triangles material=\"Material\" count=\"2\">\n";
          out << "          <input semantic=\"VERTEX\" source=\"#vertices\" offset=\"0\"/>\n";
          out << "          <input semantic=\"TEXCOORD\" source=\"#texcoords\" offset=\"1\" set=\"0\"/>\n";
          out << "          <p> 0 0 1 1 2 2 0 0 2 2 3 3 </p>\n";
          out << "        </triangles>\n";
          out << "      </mesh>\n";
          out << "    </geometry>\n";
          out << "  </library_geometries>\n";
          
          // 材质和效果
          out << "  <library_effects>\n";
          out << "    <effect id=\"Material-effect\">\n";
          out << "      <profile_COMMON>\n";
          out << "        <newparam sid=\"surface\">\n";
          out << "          <surface type=\"2D\"><init_from>aruco_marker</init_from></surface>\n";
          out << "        </newparam>\n";
          out << "        <newparam sid=\"sampler\">\n";
          out << "          <sampler2D><source>surface</source></sampler2D>\n";
          out << "        </newparam>\n";
          out << "        <technique sid=\"common\">\n";
          out << "          <lambert>\n";
          out << "            <diffuse>\n";
          out << "              <texture texture=\"sampler\" texcoord=\"UVSET0\"/>\n";
          out << "            </diffuse>\n";
          out << "          </lambert>\n";
          out << "        </technique>\n";
          out << "      </profile_COMMON>\n";
          out << "    </effect>\n";
          out << "  </library_effects>\n";
          
          out << "  <library_materials>\n";
          out << "    <material id=\"Material\" name=\"Material\">\n";
          out << "      <instance_effect url=\"#Material-effect\"/>\n";
          out << "    </material>\n";
          out << "  </library_materials>\n";
          
          out << "  <library_visual_scenes>\n";
          out << "    <visual_scene id=\"Scene\" name=\"Scene\">\n";
          out << "      <node id=\"MarkerNode\" name=\"Marker\">\n";
          out << "        <instance_geometry url=\"#marker_mesh\">\n";
          out << "          <bind_material>\n";
          out << "            <technique_common>\n";
          out << "              <instance_material symbol=\"Material\" target=\"#Material\"/>\n";
          out << "            </technique_common>\n";
          out << "          </bind_material>\n";
          out << "        </instance_geometry>\n";
          out << "      </node>\n";
          out << "    </visual_scene>\n";
          out << "  </library_visual_scenes>\n";
          
          out << "  <scene>\n";
          out << "    <instance_visual_scene url=\"#Scene\"/>\n";
          out << "  </scene>\n";
          out << "</COLLADA>\n";
          
          file.close();
          success = true;
        }
      } else {
        // 导出图像格式
        std::vector<int> params;
        if (ext == ".jpg") {
          params.push_back(cv::IMWRITE_JPEG_QUALITY);
          params.push_back(95);
        }
        success = cv::imwrite(filePath.toStdString(), *currentMarker, params);
      }
      
      if (success) {
        QMessageBox::information(page, "成功", "文件已导出: " + filePath);
      } else {
        QMessageBox::critical(page, "错误", "导出失败");
      }
    } catch (const std::exception& e) {
      QMessageBox::critical(page, "错误", QString("导出失败: %1").arg(e.what()));
    }
  });
  
  return page;
}

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
  
  // 标题
  QLabel* titleLabel = new QLabel("运动学求解器", page);
  titleLabel->setStyleSheet("font-size: 20px; font-weight: bold; color: #2c3e50;");
  mainLayout->addWidget(titleLabel);
  
  // DH 类型选择
  QHBoxLayout* dhTypeLayout = new QHBoxLayout();
  QLabel* dhTypeLabel = new QLabel("DH 类型:", page);
  QComboBox* dhTypeComboBox = new QComboBox(page);
  dhTypeComboBox->addItem("标准 DH (SDF)");
  dhTypeComboBox->addItem("改进 DH (MDH)");
  dhTypeLayout->addWidget(dhTypeLabel);
  dhTypeLayout->addWidget(dhTypeComboBox);
  dhTypeLayout->addStretch();
  mainLayout->addLayout(dhTypeLayout);
  
  // 关节数设置
  QHBoxLayout* dofLayout = new QHBoxLayout();
  QLabel* dofLabel = new QLabel("关节数:", page);
  QSpinBox* dofSpinBox = new QSpinBox(page);
  dofSpinBox->setRange(2, 7);
  dofSpinBox->setValue(6);
  QPushButton* applyDOFBtn = new QPushButton("应用", page);
  dofLayout->addWidget(dofLabel);
  dofLayout->addWidget(dofSpinBox);
  dofLayout->addWidget(applyDOFBtn);
  dofLayout->addStretch();
  mainLayout->addLayout(dofLayout);
  
  // DH 参数表格
  QGroupBox* dhGroup = new QGroupBox("DH 参数表", page);
  QVBoxLayout* dhLayout = new QVBoxLayout(dhGroup);
  
  // 表头
  QHBoxLayout* headerLayout = new QHBoxLayout();
  headerLayout->addWidget(new QLabel("关节"), 1);
  headerLayout->addWidget(new QLabel("θ (deg)"), 2);
  headerLayout->addWidget(new QLabel("d (m)"), 2);
  headerLayout->addWidget(new QLabel("a (m)"), 2);
  headerLayout->addWidget(new QLabel("α (deg)"), 2);
  dhLayout->addLayout(headerLayout);
  
  // 存储输入框
  std::vector<QDoubleSpinBox*> thetaInputs;
  std::vector<QDoubleSpinBox*> dInputs;
  std::vector<QDoubleSpinBox*> aInputs;
  std::vector<QDoubleSpinBox*> alphaInputs;
  
  auto createDHRow = [&](int joint) {
    QHBoxLayout* rowLayout = new QHBoxLayout();
    rowLayout->addWidget(new QLabel(QString("J%1").arg(joint + 1)), 1);
    
    QDoubleSpinBox* theta = new QDoubleSpinBox();
    theta->setRange(-360, 360);
    theta->setDecimals(4);
    theta->setSuffix("°");
    thetaInputs.push_back(theta);
    rowLayout->addWidget(theta, 2);
    
    QDoubleSpinBox* d = new QDoubleSpinBox();
    d->setRange(-10, 10);
    d->setDecimals(4);
    d->setSuffix(" m");
    dInputs.push_back(d);
    rowLayout->addWidget(d, 2);
    
    QDoubleSpinBox* a = new QDoubleSpinBox();
    a->setRange(-10, 10);
    a->setDecimals(4);
    a->setSuffix(" m");
    aInputs.push_back(a);
    rowLayout->addWidget(a, 2);
    
    QDoubleSpinBox* alpha = new QDoubleSpinBox();
    alpha->setRange(-360, 360);
    alpha->setDecimals(4);
    alpha->setSuffix("°");
    alphaInputs.push_back(alpha);
    rowLayout->addWidget(alpha, 2);
    
    dhLayout->addLayout(rowLayout);
  };
  
  // 初始化 6 个关节
  int currentDOF = 6;
  for (int i = 0; i < currentDOF; ++i) {
    createDHRow(i);
  }
  
  dhLayout->addStretch();
  
  mainLayout->addWidget(dhGroup);
  
  // 求解类型选择
  QGroupBox* solveGroup = new QGroupBox("求解", page);
  QVBoxLayout* solveLayout = new QVBoxLayout(solveGroup);
  
  QComboBox* solveTypeComboBox = new QComboBox(solveGroup);
  solveTypeComboBox->addItem("正运动学 (FK)");
  solveTypeComboBox->addItem("逆运动学 (IK)");
  solveLayout->addWidget(solveTypeComboBox);
  
  // 目标位姿（仅 IK 需要）
  QGroupBox* targetGroup = new QGroupBox("目标位姿 (IK)", solveGroup);
  targetGroup->setVisible(false);
  QFormLayout* targetForm = new QFormLayout(targetGroup);
  
  QDoubleSpinBox* targetX = new QDoubleSpinBox();
  targetX->setRange(-10, 10);
  targetX->setDecimals(4);
  targetX->setValue(0.5);
  targetForm->addRow("X (m):", targetX);
  
  QDoubleSpinBox* targetY = new QDoubleSpinBox();
  targetY->setRange(-10, 10);
  targetY->setDecimals(4);
  targetY->setValue(0);
  targetForm->addRow("Y (m):", targetY);
  
  QDoubleSpinBox* targetZ = new QDoubleSpinBox();
  targetZ->setRange(-10, 10);
  targetZ->setDecimals(4);
  targetZ->setValue(0.5);
  targetForm->addRow("Z (m):", targetZ);
  
  solveLayout->addWidget(targetGroup);
  
  // 切换目标位姿显示
  QObject::connect(solveTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
                   [=](int index) {
                     targetGroup->setVisible(index == 1);
                   });
  
  // 求解按钮
  QPushButton* solveBtn = new QPushButton("求解", solveGroup);
  solveBtn->setFixedHeight(40);
  solveBtn->setStyleSheet(
    "QPushButton { background-color: #3498db; color: white; border-radius: 5px; font-size: 16px; }"
    "QPushButton:hover { background-color: #2980b9; }"
  );
  solveLayout->addWidget(solveBtn);
  
  mainLayout->addWidget(solveGroup);
  
  // 结果输出
  QGroupBox* resultGroup = new QGroupBox("结果", page);
  QVBoxLayout* resultLayout = new QVBoxLayout(resultGroup);
  
  QTextEdit* resultText = new QTextEdit(resultGroup);
  resultText->setReadOnly(true);
  resultText->setStyleSheet("font-family: monospace; font-size: 13px; background-color: #ecf0f1;");
  resultLayout->addWidget(resultText);
  
  mainLayout->addWidget(resultGroup);
  
  // 应用 DOF 变化
  QObject::connect(applyDOFBtn, &QPushButton::clicked, [=]() {
    int newDOF = dofSpinBox->value();
    
    // 清除旧控件
    QLayoutItem* child;
    while ((child = dhLayout->takeAt(0)) != nullptr) {
      delete child->widget();
      delete child->layout();
      delete child;
    }
    
    // 重新创建
    thetaInputs.clear();
    dInputs.clear();
    aInputs.clear();
    alphaInputs.clear();
    currentDOF = newDOF;
    
    // 表头
    QHBoxLayout* newHeaderLayout = new QHBoxLayout();
    newHeaderLayout->addWidget(new QLabel("关节"), 1);
    newHeaderLayout->addWidget(new QLabel("θ (deg)"), 2);
    newHeaderLayout->addWidget(new QLabel("d (m)"), 2);
    newHeaderLayout->addWidget(new QLabel("a (m)"), 2);
    newHeaderLayout->addWidget(new QLabel("α (deg)"), 2);
    dhLayout->addLayout(newHeaderLayout);
    
    for (int i = 0; i < newDOF; ++i) {
      createDHRow(i);
    }
    
    dhLayout->addStretch();
  });
  
  // 求解按钮回调
  QObject::connect(solveBtn, &QPushButton::clicked, [=]() {
    bool isFK = (solveTypeComboBox->currentIndex() == 0);
    
    try {
      // 构建 KDL Chain
      KDL::Chain chain;
      
      for (int i = 0; i < currentDOF; ++i) {
        double theta = thetaInputs[i]->value() * M_PI / 180.0;
        double d = dInputs[i]->value();
        double a = aInputs[i]->value();
        double alpha = alphaInputs[i]->value() * M_PI / 180.0;
        
        KDL::Joint joint(KDL::Joint::RotZ);  // 旋转关节
        KDL::Frame frame;
        
        if (dhTypeComboBox->currentIndex() == 0) {
          // 标准 DH
          frame = KDL::Frame::DH(theta, d, a, alpha);
        } else {
          // 改进 DH
          frame = KDL::Frame::DH_Craig1989(theta, d, a, alpha);
        }
        
        chain.addSegment(KDL::Segment(joint, frame));
      }
      
      QString result;
      
      if (isFK) {
        // 正运动学
        KDL::ChainFkSolverPos_recursive fkSolver(chain);
        KDL::JntArray q(currentDOF);
        KDL::Frame T;
        
        // 假设所有关节角度为 0
        q.data.setZero();
        
        int ret = fkSolver.JntToCart(q, T);
        
        if (ret >= 0) {
          double x = T.p.x();
          double y = T.p.y();
          double z = T.p.z();
          
          // 提取 RPY
          double roll, pitch, yaw;
          T.M.GetRPY(roll, pitch, yaw);
          
          result = QString(
            "正运动学结果:\n\n"
            "位置 (m):\n"
            "X = %1\n"
            "Y = %2\n"
            "Z = %3\n\n"
            "姿态 (度):\n"
            "Roll  = %4\n"
            "Pitch = %5\n"
            "Yaw   = %6"
          ).arg(x, 0, 'f', 4)
           .arg(y, 0, 'f', 4)
           .arg(z, 0, 'f', 4)
           .arg(roll * 180.0 / M_PI, 0, 'f', 2)
           .arg(pitch * 180.0 / M_PI, 0, 'f', 2)
           .arg(yaw * 180.0 / M_PI, 0, 'f', 2);
        } else {
          result = "正运动学求解失败";
        }
        
      } else {
        // 逆运动学
        KDL::ChainIkSolverPos_LMA ikSolver(chain);
        
        KDL::Frame target;
        target.p.x(targetX->value());
        target.p.y(targetY->value());
        target.p.z(targetZ->value());
        target.M = KDL::Rotation::RPY(0, 0, 0);  // 默认姿态
        
        KDL::JntArray q_init(currentDOF);
        q_init.data.setZero();
        
        KDL::JntArray q_sol(currentDOF);
        
        int ret = ikSolver.CartToJnt(q_init, target, q_sol);
        
        if (ret >= 0) {
          result = "逆运动学结果 (关节角度):\n\n";
          for (int i = 0; i < currentDOF; ++i) {
            result += QString("J%1 = %2°\n").arg(i + 1).arg(q_sol(i) * 180.0 / M_PI, 0, 'f', 4);
          }
        } else {
          result = "逆运动学求解失败（可能超出工作空间）";
        }
      }
      
      resultText->setText(result);
      
    } catch (const std::exception& e) {
      QMessageBox::critical(page, "错误", QString("求解失败: %1").arg(e.what()));
    }
  });
  
  return page;
}

/// 创建主窗口
QMainWindow* createMainWindow() {
  QMainWindow* window = new QMainWindow();
  window->setWindowTitle("Nova Robot Tools");
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
  QLabel* titleLabel = new QLabel("Nova Tools", navPanel);
  titleLabel->setStyleSheet("color: white; font-size: 18px; font-weight: bold;");
  titleLabel->setAlignment(Qt::AlignCenter);
  navLayout->addWidget(titleLabel);
  navLayout->addSpacing(20);
  
  // 页面堆叠
  QStackedWidget* stackedWidget = new QStackedWidget(centralWidget);
  
  // 创建功能页面
  stackedWidget->addWidget(createArucoGeneratorPage());  // ArUco 生成
  stackedWidget->addWidget(createCoordinateConverterPage());  // 坐标转换
  stackedWidget->addWidget(createKinematicsSolverPage());  // 运动学求解
  
  // 其他页面（占位）
  QStringList pageNames = {"手眼标定"};
  for (const auto& name : pageNames) {
    QWidget* page = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(page);
    QLabel* label = new QLabel(QString("%1\n(功能开发中...)").arg(name), page);
    label->setAlignment(Qt::AlignCenter);
    label->setStyleSheet("font-size: 20px; color: #7f8c8d;");
    layout->addWidget(label);
    stackedWidget->addWidget(page);
  }
  
  // 创建导航按钮
  QStringList buttonNames = {"ArUco 生成", "坐标转换", "手眼标定", "运动学"};
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

NovaRobotToolsNode::NovaRobotToolsNode(const rclcpp::NodeOptions & options)
: Node("nova_robot_tools_node", options)
{
  RCLCPP_INFO(get_logger(), "Nova Robot Tools node started");
}

int RunNovaRobotToolsUiApp(int argc, char ** argv) {
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

}  // namespace nova_robot_tools
