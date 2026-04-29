#include "ros_robot_assist_tools/ui/handeye_calibration_widget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QFileDialog>
#include <QTextStream>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPainter>
#include <QPointer>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QRadioButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <atomic>
#include <functional>
#include <mutex>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ros_robot_assist_tools/module/calibration_module.h"
#include "ros_robot_assist_tools/module/handeye_calibration_module.h"
#include "ros_robot_assist_tools/ui/shared_refresh_pool.h"
#include "ros_robot_assist_tools/ui/shared_ui_executor.hpp"
#include "ros_robot_assist_tools/ui/zoomable_image_widget.h"

namespace ros_robot_assist_tools::ui
{
namespace
{

void SetFormLeftAligned(QFormLayout * form)
{
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  form->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
}

void PopulateTopicCombo(QComboBox * combo, const std::vector<QString> & topics, const QString & preferred)
{
  combo->clear();
  if (topics.empty()) {
    combo->addItem("/camera/image_raw");
    return;
  }
  for (const auto & t : topics) {
    combo->addItem(t);
  }
  int idx = combo->findText(preferred);
  if (idx < 0) idx = 0;
  combo->setCurrentIndex(idx);
}

QImage BuildPreviewPlaceholder(const QString & title, const QString & detail)
{
  QImage img(1280, 720, QImage::Format_RGB888);
  img.fill(QColor(32, 36, 42));
  QPainter painter(&img);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QColor(230, 236, 245));
  QFont title_font = painter.font();
  title_font.setPointSize(16);
  title_font.setBold(true);
  painter.setFont(title_font);
  painter.drawText(QRect(20, 20, 680, 60), Qt::AlignLeft | Qt::AlignVCenter, title);
  QFont body_font = painter.font();
  body_font.setPointSize(11);
  body_font.setBold(false);
  painter.setFont(body_font);
  painter.drawText(
    QRect(20, 92, 680, 280),
    Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
    detail);
  painter.end();
  return img;
}

bool ConvertRosImageToQImage(const sensor_msgs::msg::Image & msg, QImage * out)
{
  if (!out || msg.width == 0 || msg.height == 0 || msg.data.empty()) {
    return false;
  }
  const int width = static_cast<int>(msg.width);
  const int height = static_cast<int>(msg.height);
  const QString encoding = QString::fromStdString(msg.encoding).toLower();
  if (encoding == "rgb8") {
    QImage img(msg.data.data(), width, height, static_cast<int>(msg.step), QImage::Format_RGB888);
    *out = img.copy();
    return true;
  }
  if (encoding == "bgr8") {
    QImage img(msg.data.data(), width, height, static_cast<int>(msg.step), QImage::Format_BGR888);
    *out = img.copy();
    return true;
  }
  if (encoding == "mono8") {
    QImage img(msg.data.data(), width, height, static_cast<int>(msg.step), QImage::Format_Grayscale8);
    *out = img.copy();
    return true;
  }
  return false;
}

}  // namespace

HandeyeCalibrationWidget::HandeyeCalibrationWidget(QWidget * parent)
: QWidget(parent)
{
  QVBoxLayout * outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(0);
  QScrollArea * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  outer->addWidget(scroll);
  QWidget * content = new QWidget(scroll);
  scroll->setWidget(content);

  QVBoxLayout * root = new QVBoxLayout(content);
  root->setContentsMargins(8, 6, 8, 6);
  root->setSpacing(6);

  QLabel * title = new QLabel("手眼标定");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QComboBox * board_type = new QComboBox();
  board_type->addItems({"Chessboard", "Charuco", "Aruco GridBoard", "Aruco Single Marker"});
  QSpinBox * single_aruco_id = new QSpinBox();
  single_aruco_id->setRange(0, 9999);
  single_aruco_id->setValue(0);
  single_aruco_id->setToolTip("单个 ArUco 标定时使用的目标 ID");
  QDoubleSpinBox * marker_length_spin = new QDoubleSpinBox();
  marker_length_spin->setRange(0.001, 2.0);
  marker_length_spin->setDecimals(6);
  marker_length_spin->setSingleStep(0.001);
  marker_length_spin->setValue(0.083333);
  marker_length_spin->setSuffix(" m");
  marker_length_spin->setToolTip("Aruco 单码边长（米）");
  QComboBox * distortion_model = new QComboBox();
  distortion_model->addItems({"plumb_bob", "fisheye"});
  distortion_model->setCurrentText("fisheye");
  QComboBox * run_mode = new QComboBox();
  run_mode->addItems({"实时模式", "离线模式"});
  QComboBox * handeye_setup = new QComboBox();
  handeye_setup->addItem("眼在手上 (Eye-in-hand)");
  handeye_setup->addItem("眼在手外 (Eye-to-hand)");
  QComboBox * handeye_solver = new QComboBox();
  handeye_solver->addItems({"TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS"});
  handeye_solver->setCurrentText("PARK");

  QGroupBox * param_group = new QGroupBox("参数");
  QVBoxLayout * param_layout = new QVBoxLayout(param_group);
  QFormLayout * common_form = new QFormLayout();
  SetFormLeftAligned(common_form);
  QHBoxLayout * board_type_row = new QHBoxLayout();
  QPushButton * detect_board_btn = new QPushButton("自动识别");
  QLabel * single_id_inline_label = new QLabel("单码ID:");
  single_id_inline_label->setStyleSheet("color:#4a5563;");
  single_aruco_id->setFixedWidth(90);
  board_type_row->addWidget(board_type, 1);
  board_type_row->addSpacing(8);
  board_type_row->addWidget(single_id_inline_label);
  board_type_row->addWidget(single_aruco_id);
  board_type_row->addSpacing(8);
  board_type_row->addWidget(detect_board_btn);
  common_form->addRow("标定板类型:", board_type_row);
  common_form->addRow("畸变模型:", distortion_model);
  common_form->addRow("单码边长:", marker_length_spin);
  common_form->addRow("手眼方式:", handeye_setup);
  common_form->addRow("求解算法:", handeye_solver);
  common_form->addRow("运行模式:", run_mode);
  param_layout->addLayout(common_form);
  root->addWidget(param_group, 0);

  QGroupBox * calib_param_group = new QGroupBox("标定参数");
  QVBoxLayout * calib_param_layout = new QVBoxLayout(calib_param_group);
  QGroupBox * validation_param_group = new QGroupBox("验证参数");
  QVBoxLayout * validation_param_layout = new QVBoxLayout(validation_param_group);
  QGroupBox * operation_group = new QGroupBox("操作");
  QVBoxLayout * operation_layout = new QVBoxLayout(operation_group);
  root->addWidget(calib_param_group, 0);
  root->addWidget(validation_param_group, 0);
  root->addWidget(operation_group, 0);

  QComboBox * rt_image_topic = nullptr;
  QComboBox * rt_camera_info_topic = nullptr;
  QLineEdit * off_csv_path = nullptr;
  QPushButton * off_prev_btn = nullptr;
  QPushButton * off_next_btn = nullptr;
  QLabel * off_preview_pos_label = nullptr;
  QLabel * off_preview_name_label = nullptr;
  QComboBox * off_camera_info_topic = nullptr;

  QFormLayout * rt_form = new QFormLayout();
  SetFormLeftAligned(rt_form);
  rt_image_topic = new QComboBox();
  rt_image_topic->setEditable(false);
  rt_image_topic->setSizeAdjustPolicy(QComboBox::AdjustToContentsOnFirstShow);
  QPushButton * refresh_live_topics = new QPushButton("刷新");
  refresh_live_topics->setMinimumWidth(150);
  QHBoxLayout * rt_topic_row = new QHBoxLayout();
  rt_topic_row->addWidget(rt_image_topic, 1);
  rt_topic_row->addWidget(refresh_live_topics);
  rt_form->addRow("图像话题:", rt_topic_row);
  QLineEdit * rt_base_frame = new QLineEdit("base_link");
  QLineEdit * rt_ee_frame = new QLineEdit("tool0");
  QLineEdit * rt_third_frame = new QLineEdit("camera_link");
  QLabel * rt_third_label = new QLabel(HandeyeThirdFrameFieldLabel(HandeyeSetupMode::EyeInHand));
  rt_form->addRow("base_frame:", rt_base_frame);
  rt_form->addRow("ee_frame:", rt_ee_frame);
  rt_form->addRow(rt_third_label, rt_third_frame);
  QCheckBox * auto_refresh_live_topics = new QCheckBox("自动刷新在线图像话题(2s)");
  auto_refresh_live_topics->setChecked(false);
  rt_form->addRow("", auto_refresh_live_topics);
  QWidget * rt_calib_panel = new QWidget();
  QVBoxLayout * rt_calib_layout = new QVBoxLayout(rt_calib_panel);
  rt_calib_layout->setContentsMargins(0, 0, 0, 0);
  rt_calib_layout->addLayout(rt_form);
  calib_param_layout->addWidget(rt_calib_panel);

  QVBoxLayout * off_param_layout = new QVBoxLayout();
  QHBoxLayout * csv_row = new QHBoxLayout();
  off_csv_path = new QLineEdit();
  QPushButton * browse_csv = new QPushButton("选择 CSV 文件");
  csv_row->addWidget(off_csv_path, 1);
  csv_row->addWidget(browse_csv);
  off_param_layout->addLayout(csv_row);
  off_prev_btn = new QPushButton("上一张");
  off_next_btn = new QPushButton("下一张");
  off_preview_pos_label = new QLabel("0/0");
  off_preview_name_label = new QLabel("(无图像)");
  off_preview_name_label->setStyleSheet("color:#5b6470;");
  QHBoxLayout * off_preview_nav_row = new QHBoxLayout();
  off_preview_nav_row->addWidget(off_prev_btn);
  off_preview_nav_row->addWidget(off_next_btn);
  off_preview_nav_row->addSpacing(8);
  off_preview_nav_row->addWidget(off_preview_pos_label);
  off_preview_nav_row->addSpacing(10);
  off_preview_nav_row->addWidget(off_preview_name_label, 1);
  QFormLayout * off_form = new QFormLayout();
  SetFormLeftAligned(off_form);
  off_form->addRow("预览图像(同目录):", off_preview_nav_row);
  QLineEdit * off_base_frame = new QLineEdit("base_link");
  QLineEdit * off_ee_frame = new QLineEdit("tool0");
  QLineEdit * off_third_frame = new QLineEdit("camera_link");
  QLabel * off_base_label = new QLabel("base_frame:");
  QLabel * off_ee_label = new QLabel("ee_frame:");
  QLabel * off_third_label = new QLabel(HandeyeThirdFrameFieldLabel(HandeyeSetupMode::EyeInHand));
  off_form->addRow(off_base_label, off_base_frame);
  off_form->addRow(off_ee_label, off_ee_frame);
  off_form->addRow(off_third_label, off_third_frame);
  off_param_layout->addLayout(off_form);
  QWidget * off_calib_panel = new QWidget();
  QVBoxLayout * off_calib_layout = new QVBoxLayout(off_calib_panel);
  off_calib_layout->setContentsMargins(0, 0, 0, 0);
  off_calib_layout->addLayout(off_param_layout);
  calib_param_layout->addWidget(off_calib_panel);

  QWidget * rt_validation_widget = new QWidget();
  QFormLayout * rt_validation_form = new QFormLayout(rt_validation_widget);
  SetFormLeftAligned(rt_validation_form);
  rt_camera_info_topic = new QComboBox();
  rt_camera_info_topic->setEditable(true);
  PopulateTopicCombo(rt_camera_info_topic, {}, "/camera/camera_info");
  rt_validation_form->addRow("camera_info话题:", rt_camera_info_topic);
  QWidget * rt_validation_panel = new QWidget();
  QVBoxLayout * rt_validation_layout = new QVBoxLayout(rt_validation_panel);
  rt_validation_layout->setContentsMargins(0, 0, 0, 0);
  rt_validation_layout->addWidget(rt_validation_widget);
  validation_param_layout->addWidget(rt_validation_panel);

  QWidget * off_validation_widget = new QWidget();
  QFormLayout * off_validation_form = new QFormLayout(off_validation_widget);
  SetFormLeftAligned(off_validation_form);
  QComboBox * off_validation_mode = new QComboBox();
  off_validation_mode->addItems({"理论矩阵", "TF(frame_id)"});
  off_validation_form->addRow("验证基准:", off_validation_mode);
  QPlainTextEdit * off_theory_matrix = new QPlainTextEdit();
  off_theory_matrix->setPlaceholderText(
    "输入 4x4 理论矩阵（16 个数字，按行），示例:\n"
    "1 0 0 0.53\n0 -1 0 -0.50\n0 0 -1 1.04\n0 0 0 1");
  off_theory_matrix->setMinimumHeight(88);
  off_validation_form->addRow("理论T_cam_base:", off_theory_matrix);
  QLineEdit * off_tf_from = new QLineEdit("camera_link");
  QLineEdit * off_tf_to = new QLineEdit("base_link");
  QLabel * off_tf_from_label = new QLabel("from_frame:");
  QLabel * off_tf_to_label = new QLabel("to_frame:");
  off_validation_form->addRow(off_tf_from_label, off_tf_from);
  off_validation_form->addRow(off_tf_to_label, off_tf_to);
  off_camera_info_topic = new QComboBox();
  off_camera_info_topic->setEditable(true);
  PopulateTopicCombo(off_camera_info_topic, {}, "/camera/camera_info");
  QLabel * off_camera_info_label = new QLabel("camera_info话题:");
  off_validation_form->addRow(off_camera_info_label, off_camera_info_topic);
  QWidget * off_validation_panel = new QWidget();
  QVBoxLayout * off_validation_layout = new QVBoxLayout(off_validation_panel);
  off_validation_layout->setContentsMargins(0, 0, 0, 0);
  off_validation_layout->addWidget(off_validation_widget);
  validation_param_layout->addWidget(off_validation_panel);

  QRadioButton * quality_hint = new QRadioButton("显示采图质量提示");
  quality_hint->setChecked(true);
  QHBoxLayout * rt_btn_row = new QHBoxLayout();
  QPushButton * manual_capture = new QPushButton("手动采图");
  QPushButton * auto_capture = new QPushButton("自动采图");
  QPushButton * run_btn = new QPushButton("执行标定");
  rt_btn_row->addWidget(manual_capture);
  rt_btn_row->addWidget(auto_capture);
  rt_btn_row->addWidget(run_btn);
  QWidget * rt_operation_panel = new QWidget();
  QVBoxLayout * rt_operation_layout = new QVBoxLayout(rt_operation_panel);
  rt_operation_layout->setContentsMargins(0, 0, 0, 0);
  rt_operation_layout->addWidget(quality_hint);
  rt_operation_layout->addLayout(rt_btn_row);
  QPushButton * off_run_btn = new QPushButton("执行离线标定");
  QWidget * off_operation_panel = new QWidget();
  QVBoxLayout * off_operation_layout = new QVBoxLayout(off_operation_panel);
  off_operation_layout->setContentsMargins(0, 0, 0, 0);
  off_operation_layout->addWidget(off_run_btn);
  operation_layout->addWidget(rt_operation_panel);
  operation_layout->addWidget(off_operation_panel);

  QGroupBox * preview_group = new QGroupBox("图像显示");
  QVBoxLayout * preview_layout = new QVBoxLayout(preview_group);
  QHBoxLayout * preview_ctrl_row = new QHBoxLayout();
  QCheckBox * show_raw_checkbox = new QCheckBox("显示原图");
  show_raw_checkbox->setChecked(true);
  QPushButton * refresh_preview_btn = new QPushButton("刷新图像显示");
  QPushButton * clear_log_btn = new QPushButton("清空日志");
  clear_log_btn->setMinimumWidth(96);
  preview_ctrl_row->addWidget(show_raw_checkbox);
  preview_ctrl_row->addStretch();
  preview_ctrl_row->addWidget(refresh_preview_btn);
  preview_ctrl_row->addWidget(clear_log_btn);
  QHBoxLayout * preview_content_row = new QHBoxLayout();
  ZoomableImageWidget * preview_widget = new ZoomableImageWidget();
  preview_widget->setMinimumHeight(240);
  preview_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::MinimumExpanding);
  preview_widget->setStyleSheet("background:#20242a; border:1px solid #3a4048;");
  QTextEdit * log = new QTextEdit();
  log->setReadOnly(true);
  log->setMinimumHeight(240);
  log->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::MinimumExpanding);
  QTextEdit * matrix_result = new QTextEdit();
  matrix_result->setReadOnly(true);
  matrix_result->setMinimumHeight(120);
  matrix_result->setPlaceholderText("最终位姿矩阵将在离线标定完成后显示");
  matrix_result->setStyleSheet("QTextEdit { font-family: monospace; }");
  preview_content_row->addWidget(preview_widget, 1);
  preview_content_row->addWidget(log, 1);
  preview_layout->addLayout(preview_ctrl_row);
  preview_layout->addLayout(preview_content_row);
  preview_layout->addWidget(new QLabel("最终结果矩阵:"));
  preview_layout->addWidget(matrix_result);
  root->addWidget(preview_group, 1);

  QGroupBox * output_group = new QGroupBox("结果输出");
  QHBoxLayout * output_layout = new QHBoxLayout(output_group);
  QLineEdit * output_yaml = new QLineEdit(DefaultCalibrationYamlPath("手眼标定"));
  QPushButton * browse_output = new QPushButton("选择输出路径");
  output_layout->addWidget(output_yaml, 1);
  output_layout->addWidget(browse_output);
  root->addWidget(output_group);
  QObject::connect(browse_output, &QPushButton::clicked, [=]() {
    const QString path = QFileDialog::getSaveFileName(
      nullptr, "保存标定结果", output_yaml->text(), "YAML (*.yaml *.yml)");
    if (!path.isEmpty()) output_yaml->setText(path);
  });

  auto update_mode_visibility = [=]() {
    const bool online = (run_mode->currentIndex() == 0);
    rt_calib_panel->setVisible(online);
    rt_validation_panel->setVisible(online);
    rt_operation_panel->setVisible(online);
    off_calib_panel->setVisible(!online);
    off_validation_panel->setVisible(!online);
    off_operation_panel->setVisible(!online);

    // 离线模式下隐藏这些非必要语义参数，避免干扰核心流程
    const bool show_offline_semantic_fields = online;
    off_base_label->setVisible(show_offline_semantic_fields);
    off_base_frame->setVisible(show_offline_semantic_fields);
    off_ee_label->setVisible(show_offline_semantic_fields);
    off_ee_frame->setVisible(show_offline_semantic_fields);
    off_third_label->setVisible(show_offline_semantic_fields);
    off_third_frame->setVisible(show_offline_semantic_fields);
    off_camera_info_label->setVisible(show_offline_semantic_fields);
    off_camera_info_topic->setVisible(show_offline_semantic_fields);
  };
  QObject::connect(run_mode, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) {
    update_mode_visibility();
  });
  auto append_log = [=](const QString & msg) {
    log->append(QString("[%1] %2").arg(QDateTime::currentDateTime().toString("hh:mm:ss")).arg(msg));
  };
  auto build_handeye_output_file = [=](const QDir & dir) -> QString {
    const QString ts = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString out = dir.filePath(QString("assist_handeye_result_%1.yaml").arg(ts));
    int suffix = 1;
    while (QFileInfo::exists(out)) {
      out = dir.filePath(QString("assist_handeye_result_%1_%2.yaml").arg(ts).arg(suffix));
      ++suffix;
    }
    return out;
  };
  QObject::connect(clear_log_btn, &QPushButton::clicked, [=]() { log->clear(); });
  auto update_preview = std::make_shared<std::function<void()>>();
  auto offline_image_names = std::make_shared<std::vector<QString>>();
  auto offline_image_index = std::make_shared<int>(-1);
  auto update_offline_preview_nav_ui = [=]() {
    const int n = static_cast<int>(offline_image_names->size());
    const bool has = (n > 0 && *offline_image_index >= 0 && *offline_image_index < n);
    off_prev_btn->setEnabled(has && *offline_image_index > 0);
    off_next_btn->setEnabled(has && *offline_image_index < n - 1);
    if (!has) {
      off_preview_pos_label->setText("0/0");
      off_preview_name_label->setText("(无图像)");
      return;
    }
    off_preview_pos_label->setText(QString("%1/%2").arg(*offline_image_index + 1).arg(n));
    off_preview_name_label->setText((*offline_image_names)[*offline_image_index]);
  };
  auto resolve_offline_image_path = [=](const QString & csv, const QString & name) -> QString {
    const QString n = name.trimmed();
    if (csv.trimmed().isEmpty() || n.isEmpty()) return QString();
    QFileInfo direct(n);
    if (direct.isAbsolute() && direct.exists()) {
      return direct.absoluteFilePath();
    }
    const QString dir = ImageDirectoryForHandeyePosesCsv(csv);
    const QString p1 = QDir(dir).filePath(n);
    if (QFileInfo::exists(p1)) return p1;
    // 仅使用 raw_image 对应图像，不回退到 result_image。
    QDirIterator it(dir, QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
      const QString fp = it.next();
      if (QFileInfo(fp).fileName().compare(n, Qt::CaseInsensitive) == 0) {
        return fp;
      }
    }
    return QString();
  };
  auto log_offline_samples = [=](const QString & csv) {
    QFile f(csv);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
      append_log("样本日志: 无法打开 CSV 进行打印");
      return;
    }
    QTextStream in(&f);
    if (in.atEnd()) {
      append_log("样本日志: CSV 为空");
      return;
    }
    const QString header = in.readLine().trimmed();
    const QStringList cols = header.split(',', Qt::KeepEmptyParts);
    auto idx_of = [&](const QString & name) { return cols.indexOf(name); };
    const int i_raw = idx_of("raw_image");
    const int i_px = idx_of("px");
    const int i_py = idx_of("py");
    const int i_pz = idx_of("pz");
    const int i_qx = idx_of("qx");
    const int i_qy = idx_of("qy");
    const int i_qz = idx_of("qz");
    const int i_qw = idx_of("qw");
    if (i_raw < 0 || i_px < 0 || i_py < 0 || i_pz < 0 || i_qx < 0 || i_qy < 0 || i_qz < 0 || i_qw < 0) {
      append_log("样本日志: CSV 缺少位姿或图像列(raw_image/px/py/pz/qx/qy/qz/qw)");
      return;
    }
    int row_idx = 0;
    while (!in.atEnd()) {
      const QString line = in.readLine().trimmed();
      if (line.isEmpty()) continue;
      const QStringList c = line.split(',', Qt::KeepEmptyParts);
      const int need = std::max({i_raw, i_px, i_py, i_pz, i_qx, i_qy, i_qz, i_qw});
      if (c.size() <= need) continue;
      ++row_idx;
      const QString raw_name = c[i_raw].trimmed();
      const QString final_path = resolve_offline_image_path(csv, raw_name);
      append_log(QString("样本[%1] image=%2 | pose=(%3,%4,%5 | %6,%7,%8,%9)")
        .arg(row_idx)
        .arg(final_path.isEmpty() ? QString("NOT_FOUND(%1)").arg(raw_name) : final_path)
        .arg(c[i_px].trimmed())
        .arg(c[i_py].trimmed())
        .arg(c[i_pz].trimmed())
        .arg(c[i_qx].trimmed())
        .arg(c[i_qy].trimmed())
        .arg(c[i_qz].trimmed())
        .arg(c[i_qw].trimmed()));
    }
  };
  auto build_offline_detected_preview = [=](const QString & csv, const QString & name, QImage * out, QString * err) -> bool {
    if (!out) return false;
    const QString image_path = resolve_offline_image_path(csv, name);
    if (image_path.isEmpty()) {
      if (err) *err = QString("未找到图像文件: %1").arg(name);
      return false;
    }
    cv::Mat bgr = cv::imread(image_path.toStdString(), cv::IMREAD_COLOR);
    if (bgr.empty()) {
      if (err) *err = QString("读取图像失败: %1").arg(image_path);
      return false;
    }
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    cv::aruco::DetectorParameters params;
    cv::aruco::ArucoDetector detector(dict, params);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector.detectMarkers(bgr, corners, ids);
    if (ids.empty()) {
      if (err) *err = "当前图像未检测到 ArUco";
      return false;
    }
    if (board_type->currentText() == "Aruco Single Marker") {
      const int target_id = single_aruco_id->value();
      std::vector<int> one_id;
      std::vector<std::vector<cv::Point2f>> one_corner;
      for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
        if (ids[i] == target_id) {
          one_id.push_back(ids[i]);
          one_corner.push_back(corners[i]);
          break;
        }
      }
      if (one_id.empty()) {
        if (err) *err = QString("未检测到目标ID=%1").arg(target_id);
        return false;
      }
      cv::aruco::drawDetectedMarkers(bgr, one_corner, one_id);
      ids = one_id;
      corners = one_corner;
    } else {
      cv::aruco::drawDetectedMarkers(bgr, corners, ids);
    }
    cv::putText(
      bgr,
      QString("Detected: %1 marker(s)").arg(static_cast<int>(ids.size())).toStdString(),
      cv::Point(20, 32),
      cv::FONT_HERSHEY_SIMPLEX,
      0.8,
      cv::Scalar(0, 255, 0),
      2);
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    QImage q(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888);
    *out = q.copy();
    return true;
  };
  auto detect_current_offline_pair = [=]() {};
  auto repopulate_offline_from_csv = [=](bool with_dialog) {
    const QString p = off_csv_path->text().trimmed();
    if (p.isEmpty()) {
      offline_image_names->clear();
      *offline_image_index = -1;
      update_offline_preview_nav_ui();
      return;
    }
    QString err;
    if (!ValidateHandeyePosesCsvFile(p, &err)) {
      if (with_dialog) {
        QMessageBox::warning(this, "CSV 无效", err);
      }
      return;
    }
    std::vector<QString> basenames;
    int n = 0;
    if (!ListImageFilenamesFromHandeyePosesCsv(p, &basenames, &n, &err)) {
      if (with_dialog) {
        QMessageBox::warning(this, "解析 CSV 失败", err);
      } else {
        append_log("解析 CSV: " + err);
      }
      return;
    }
    offline_image_names->clear();
    for (const auto & b : basenames) {
      offline_image_names->push_back(b);
    }
    *offline_image_index = offline_image_names->empty() ? -1 : 0;
    update_offline_preview_nav_ui();
    append_log(QString("已加载采集列表: %1 条记录，图像目录: %2")
        .arg(n)
        .arg(ImageDirectoryForHandeyePosesCsv(p)));
    log_offline_samples(p);
    detect_current_offline_pair();
  };
  QObject::connect(browse_csv, &QPushButton::clicked, [=]() {
    QString start_dir = QDir::currentPath();
    const QFileInfo current_csv_info(off_csv_path->text().trimmed());
    if (current_csv_info.exists() && current_csv_info.isFile()) {
      start_dir = current_csv_info.absolutePath();
    } else if (QDir(off_csv_path->text().trimmed()).exists()) {
      start_dir = off_csv_path->text().trimmed();
    }
    const QString path = QFileDialog::getOpenFileName(
      this, "选择手眼采集列表 CSV", start_dir, "CSV (*.csv);;所有文件 (*)");
    if (path.isEmpty()) return;
    off_csv_path->setText(path);
    repopulate_offline_from_csv(true);
    if (*update_preview) (*update_preview)();
  });
  auto sync_handeye_frame_labels = [=]() {
    const HandeyeSetupMode m = handeye_setup->currentIndex() == 0 ? HandeyeSetupMode::EyeInHand : HandeyeSetupMode::EyeToHand;
    rt_third_label->setText(HandeyeThirdFrameFieldLabel(m));
    off_third_label->setText(HandeyeThirdFrameFieldLabel(m));
    const QString ph = (m == HandeyeSetupMode::EyeInHand) ? QStringLiteral("随末端安装的相机坐标系")
                                                         : QStringLiteral("固连末端的标定板/目标坐标系");
    rt_third_frame->setPlaceholderText(ph);
    off_third_frame->setPlaceholderText(ph);
  };
  QObject::connect(handeye_setup, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) {
    sync_handeye_frame_labels();
    append_log(QString("手眼方式: %1").arg(handeye_setup->currentText()));
  });
  QObject::connect(handeye_solver, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) {
    append_log(QString("手眼求解算法: %1").arg(handeye_solver->currentText()));
  });
  auto update_offline_validation_mode_ui = [=]() {
    const bool use_theory = (off_validation_mode->currentIndex() == 0);
    off_theory_matrix->setVisible(use_theory);
    if (auto * l = off_validation_form->labelForField(off_theory_matrix)) l->setVisible(use_theory);
    off_tf_from->setVisible(!use_theory);
    off_tf_to->setVisible(!use_theory);
    off_tf_from_label->setVisible(!use_theory);
    off_tf_to_label->setVisible(!use_theory);
  };
  QObject::connect(off_validation_mode, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) {
    update_offline_validation_mode_ui();
    append_log(QString("离线验证基准: %1").arg(off_validation_mode->currentText()));
  });
  auto update_board_type_ui = [=]() {
    const bool single = (board_type->currentText() == "Aruco Single Marker");
    single_aruco_id->setVisible(single);
    single_id_inline_label->setVisible(single);
  };
  QObject::connect(board_type, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) {
    update_board_type_ui();
  });
  QObject::connect(manual_capture, &QPushButton::clicked, [=]() { append_log("手动采图触发。"); });
  QObject::connect(auto_capture, &QPushButton::clicked, [=]() { append_log("自动采图启动。"); });
  QObject::connect(detect_board_btn, &QPushButton::clicked, [=]() {
    if (run_mode->currentIndex() != 1) {
      QMessageBox::information(this, "提示", "请在离线模式下使用自动识别（基于离线图像）。");
      return;
    }
    const QString csv = off_csv_path->text().trimmed();
    if (csv.isEmpty()) {
      QMessageBox::warning(this, "识别失败", "请先导入离线 CSV 文件。");
      return;
    }
    const QString dir = ImageDirectoryForHandeyePosesCsv(csv);
    QString name;
    if (!offline_image_names->empty()) {
      const int idx = (*offline_image_index >= 0 && *offline_image_index < static_cast<int>(offline_image_names->size()))
                      ? *offline_image_index : 0;
      name = (*offline_image_names)[idx].trimmed();
    }
    if (name.isEmpty()) {
      QMessageBox::warning(this, "识别失败", "CSV 中未解析到可预览图像文件名。");
      return;
    }
    const QString image_path = resolve_offline_image_path(csv, name);
    QString bt;
    int marker_id = -1;
    QString detect_err;
    if (!DetectBoardTypeFromImage(image_path, &bt, &marker_id, &detect_err)) {
      QMessageBox::warning(this, "识别失败", detect_err);
      append_log("自动识别失败: " + detect_err);
      return;
    }
    const int idx = board_type->findText(bt);
    if (idx >= 0) {
      board_type->setCurrentIndex(idx);
    }
    if (bt == "Aruco Single Marker" && marker_id >= 0) {
      single_aruco_id->setValue(marker_id);
    }
    append_log(QString("自动识别完成: %1%2")
      .arg(bt)
      .arg((bt == "Aruco Single Marker" && marker_id >= 0) ? QString(" (ID=%1)").arg(marker_id) : QString()));
  });

  auto online_refresh_busy = std::make_shared<std::atomic_bool>(false);
  auto request_online_topics_refresh = [=](bool with_log) {
    if (online_refresh_busy->exchange(true)) return;
    const QString preferred = rt_image_topic->currentText();
    QPointer<QWidget> alive(this);
    RunOnSharedRefreshPool([=]() {
      const auto topics = ListOnlineImageTopics();
      const auto camera_info_topics = ListOnlineCameraInfoTopics();
      QMetaObject::invokeMethod(this, [=]() {
        online_refresh_busy->store(false);
        if (!alive) return;
        PopulateTopicCombo(rt_image_topic, topics, preferred);
        PopulateTopicCombo(rt_camera_info_topic, camera_info_topics, rt_camera_info_topic->currentText());
        if (with_log) append_log(QString("在线图像话题刷新完成，共 %1 个").arg(static_cast<int>(topics.size())));
      }, Qt::QueuedConnection);
    });
  };
  QObject::connect(refresh_live_topics, &QPushButton::clicked, [=]() { request_online_topics_refresh(true); });
  QTimer * live_topics_timer = new QTimer(this);
  live_topics_timer->setInterval(2000);
  QObject::connect(live_topics_timer, &QTimer::timeout, [=]() {
    if (!auto_refresh_live_topics->isChecked()) return;
    request_online_topics_refresh(false);
  });
  live_topics_timer->start();

  *update_preview = [=]() {
    const bool online_mode = (run_mode->currentIndex() == 0);
    if (online_mode) {
      const QString topic = rt_image_topic->currentText();
      const QString text = show_raw_checkbox->isChecked() ? "原图预览" : "标定结果图预览";
      const QString body = QString("当前话题: %1\n更新时间: %2")
        .arg(topic.isEmpty() ? "(未选择)" : topic)
        .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
      preview_widget->SetImage(BuildPreviewPlaceholder(text, body));
      return;
    }
    if (!show_raw_checkbox->isChecked()) {
      QString name;
      if (!offline_image_names->empty() &&
          *offline_image_index >= 0 &&
          *offline_image_index < static_cast<int>(offline_image_names->size())) {
        name = (*offline_image_names)[*offline_image_index];
      }
      QImage detected;
      QString detect_err;
      if (!name.isEmpty() && build_offline_detected_preview(off_csv_path->text().trimmed(), name, &detected, &detect_err)) {
        preview_widget->SetImage(detected);
      } else {
        const QString body = QString("CSV: %1\n图像: %2\n识别结果图生成失败: %3")
          .arg(off_csv_path->text().isEmpty() ? "(未选)" : off_csv_path->text())
          .arg(name.isEmpty() ? "(无列表)" : name)
          .arg(detect_err.isEmpty() ? "unknown" : detect_err);
        preview_widget->SetImage(BuildPreviewPlaceholder("标定结果图预览", body));
      }
      return;
    }
    const QString dir = ImageDirectoryForHandeyePosesCsv(off_csv_path->text().trimmed());
    QString name;
    if (!offline_image_names->empty() &&
        *offline_image_index >= 0 &&
        *offline_image_index < static_cast<int>(offline_image_names->size())) {
      name = (*offline_image_names)[*offline_image_index];
    }
    const QString abs = resolve_offline_image_path(off_csv_path->text().trimmed(), name);
    if (!abs.isEmpty() && QFileInfo::exists(abs)) {
      QImage im;
      if (im.load(abs)) {
        preview_widget->SetImage(im);
        return;
      }
    }
    const QString body = QString("CSV: %1\n图像: %2\n(若无法显示请检查同目录下是否存在该文件)")
      .arg(off_csv_path->text().isEmpty() ? "(未选文件)" : off_csv_path->text())
      .arg(name.isEmpty() ? "(无列表)" : name);
    preview_widget->SetImage(BuildPreviewPlaceholder("离线图像预览", body));
  };
  auto image_subscription = std::make_shared<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>();
  auto image_sub_mutex = std::make_shared<std::mutex>();
  auto image_generation = std::make_shared<std::atomic<uint64_t>>(0);
  const auto stamp = static_cast<unsigned long long>(QDateTime::currentMSecsSinceEpoch());
  auto image_node = rclcpp::Node::make_shared("ros_robot_assist_tools_handeye_preview_" + std::to_string(stamp));
  SharedUiExecutor::instance().add_node(image_node);
  QObject::connect(this, &QObject::destroyed, [=]() {
    SharedUiExecutor::instance().remove_node(image_node);
  });
  auto subscribe_preview_topic = [=]() {
    const bool online_mode = (run_mode->currentIndex() == 0);
    if (!online_mode || !show_raw_checkbox->isChecked()) {
      std::lock_guard<std::mutex> lk(*image_sub_mutex);
      *image_subscription = nullptr;
      if (*update_preview) (*update_preview)();
      return;
    }
    const QString topic = rt_image_topic->currentText().trimmed();
    if (topic.isEmpty()) {
      std::lock_guard<std::mutex> lk(*image_sub_mutex);
      *image_subscription = nullptr;
      if (*update_preview) (*update_preview)();
      return;
    }
    const uint64_t generation = image_generation->fetch_add(1) + 1;
    QPointer<QWidget> alive(this);
    std::lock_guard<std::mutex> lk(*image_sub_mutex);
    *image_subscription = image_node->create_subscription<sensor_msgs::msg::Image>(
      topic.toStdString(), rclcpp::SensorDataQoS(),
      [=](const sensor_msgs::msg::Image::SharedPtr msg) {
        RunOnSharedImageRefreshPool([=]() {
          QImage image;
          if (!ConvertRosImageToQImage(*msg, &image)) {
            return;
          }
          QMetaObject::invokeMethod(this, [=]() {
            if (!alive || generation != image_generation->load()) return;
            preview_widget->SetImage(image);
          }, Qt::QueuedConnection);
        });
      });
    append_log(QString("已订阅图像话题: %1").arg(topic));
  };
  QObject::connect(refresh_preview_btn, &QPushButton::clicked, [=]() {
    if (run_mode->currentIndex() == 0) {
      subscribe_preview_topic();
    } else {
      repopulate_offline_from_csv(false);
      if (*update_preview) (*update_preview)();
      detect_current_offline_pair();
    }
  });
  QObject::connect(show_raw_checkbox, &QCheckBox::toggled, [=](bool) {
    subscribe_preview_topic();
    if (*update_preview) (*update_preview)();
  });
  QObject::connect(run_mode, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) {
    subscribe_preview_topic();
    if (*update_preview) (*update_preview)();
  });
  QObject::connect(rt_image_topic, &QComboBox::currentTextChanged, [=](const QString &) { subscribe_preview_topic(); });
  QObject::connect(off_prev_btn, &QPushButton::clicked, [=]() {
    if (*offline_image_index > 0) {
      --(*offline_image_index);
      update_offline_preview_nav_ui();
      if (*update_preview) (*update_preview)();
      detect_current_offline_pair();
    }
  });
  QObject::connect(off_next_btn, &QPushButton::clicked, [=]() {
    if (*offline_image_index >= 0 &&
        *offline_image_index + 1 < static_cast<int>(offline_image_names->size())) {
      ++(*offline_image_index);
      update_offline_preview_nav_ui();
      if (*update_preview) (*update_preview)();
      detect_current_offline_pair();
    }
  });
  QObject::connect(off_csv_path, &QLineEdit::editingFinished, [=]() {
    repopulate_offline_from_csv(false);
    if (*update_preview) (*update_preview)();
    detect_current_offline_pair();
  });

  struct OfflineSampleRow {
    QString raw_image;
    QString intrinsics_file;
    QString px;
    QString py;
    QString pz;
    QString qx;
    QString qy;
    QString qz;
    QString qw;
  };
  auto parse_offline_rows = [=](const QString & csv, std::vector<OfflineSampleRow> * rows, QString * err_msg) -> bool {
    if (!rows) return false;
    rows->clear();
    QFile f(csv);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
      if (err_msg) *err_msg = "无法打开 CSV";
      return false;
    }
    QTextStream in(&f);
    if (in.atEnd()) {
      if (err_msg) *err_msg = "CSV 为空";
      return false;
    }
    const QStringList header = in.readLine().trimmed().split(',', Qt::KeepEmptyParts);
    auto idx_of = [&](const QString & n) { return header.indexOf(n); };
    const int i_raw = idx_of("raw_image");
    const int i_intr = idx_of("intrinsics_file");
    const int i_px = idx_of("px");
    const int i_py = idx_of("py");
    const int i_pz = idx_of("pz");
    const int i_qx = idx_of("qx");
    const int i_qy = idx_of("qy");
    const int i_qz = idx_of("qz");
    const int i_qw = idx_of("qw");
    if (i_raw < 0 || i_px < 0 || i_py < 0 || i_pz < 0 || i_qx < 0 || i_qy < 0 || i_qz < 0 || i_qw < 0) {
      if (err_msg) *err_msg = "CSV 缺少 raw_image/px/py/pz/qx/qy/qz/qw 列";
      return false;
    }
    while (!in.atEnd()) {
      const QString line = in.readLine().trimmed();
      if (line.isEmpty()) continue;
      const QStringList c = line.split(',', Qt::KeepEmptyParts);
      const int need = std::max({i_raw, i_px, i_py, i_pz, i_qx, i_qy, i_qz, i_qw});
      if (c.size() <= need) continue;
      OfflineSampleRow r;
      r.raw_image = c[i_raw].trimmed();
      r.intrinsics_file = (i_intr >= 0 && i_intr < c.size()) ? c[i_intr].trimmed() : QString();
      r.px = c[i_px].trimmed();
      r.py = c[i_py].trimmed();
      r.pz = c[i_pz].trimmed();
      r.qx = c[i_qx].trimmed();
      r.qy = c[i_qy].trimmed();
      r.qz = c[i_qz].trimmed();
      r.qw = c[i_qw].trimmed();
      rows->push_back(r);
    }
    if (rows->empty()) {
      if (err_msg) *err_msg = "CSV 无有效样本";
      return false;
    }
    return true;
  };
  auto render_final_matrix_text = [=](const QString & text_blob) {
    const QRegularExpression num_re("[-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?");
    auto parse_numbers = [&](const QString & line) {
      std::vector<double> vals;
      auto it = num_re.globalMatch(line);
      while (it.hasNext()) {
        const auto m = it.next();
        bool ok = false;
        const double v = m.captured(0).toDouble(&ok);
        if (ok) vals.push_back(v);
      }
      return vals;
    };
    QString t_line;
    QString r_line;
    QString tcb_line;
    QString tbc_line;
    for (const auto & line : text_blob.split('\n', Qt::SkipEmptyParts)) {
      if (line.contains("final: t=")) t_line = line;
      if (line.contains("final: R=")) r_line = line;
      if (line.startsWith("T_cam_base:")) tcb_line = line;
      if (line.startsWith("T_base_cam:")) tbc_line = line;
    }
    if (!tcb_line.isEmpty() && !tbc_line.isEmpty()) {
      const auto a = parse_numbers(tcb_line);
      const auto b = parse_numbers(tbc_line);
      if (a.size() >= 16 && b.size() >= 16) {
        auto row = [](const std::vector<double> & v, int r) {
          return QString("[ %1  %2  %3  %4 ]")
            .arg(QString::number(v[r * 4 + 0], 'f', 6), 12)
            .arg(QString::number(v[r * 4 + 1], 'f', 6), 12)
            .arg(QString::number(v[r * 4 + 2], 'f', 6), 12)
            .arg(QString::number(v[r * 4 + 3], 'f', 6), 12);
        };
        QString txt;
        txt += "T_cam_base (4x4)\n";
        txt += row(a, 0) + "\n" + row(a, 1) + "\n" + row(a, 2) + "\n" + row(a, 3) + "\n\n";
        txt += "T_base_cam (4x4)\n";
        txt += row(b, 0) + "\n" + row(b, 1) + "\n" + row(b, 2) + "\n" + row(b, 3) + "\n";
        matrix_result->setPlainText(txt);
        return;
      }
    }
    const auto t_vals = parse_numbers(t_line);
    const auto r_vals = parse_numbers(r_line);
    if (t_vals.size() < 3 || r_vals.size() < 9) {
      matrix_result->setPlainText("未解析到最终矩阵，请先执行离线标定。");
      return;
    }
    QString txt;
    txt += "t (3x1)\n";
    txt += QString("[ %1 ]\n").arg(QString::number(t_vals[0], 'f', 6), 12);
    txt += QString("[ %1 ]\n").arg(QString::number(t_vals[1], 'f', 6), 12);
    txt += QString("[ %1 ]\n\n").arg(QString::number(t_vals[2], 'f', 6), 12);
    txt += "R (3x3)\n";
    txt += QString("[ %1  %2  %3 ]\n")
      .arg(QString::number(r_vals[0], 'f', 6), 12)
      .arg(QString::number(r_vals[1], 'f', 6), 12)
      .arg(QString::number(r_vals[2], 'f', 6), 12);
    txt += QString("[ %1  %2  %3 ]\n")
      .arg(QString::number(r_vals[3], 'f', 6), 12)
      .arg(QString::number(r_vals[4], 'f', 6), 12)
      .arg(QString::number(r_vals[5], 'f', 6), 12);
    txt += QString("[ %1  %2  %3 ]\n")
      .arg(QString::number(r_vals[6], 'f', 6), 12)
      .arg(QString::number(r_vals[7], 'f', 6), 12)
      .arg(QString::number(r_vals[8], 'f', 6), 12);
    matrix_result->setPlainText(txt);
  };
  auto load_intrinsics = [=](const QString & csv, const std::vector<OfflineSampleRow> & rows, cv::Mat * K, cv::Mat * D, QString * err_msg) -> bool {
    if (!K || !D) return false;
    QString intr_name;
    for (const auto & r : rows) {
      if (!r.intrinsics_file.isEmpty()) {
        intr_name = r.intrinsics_file;
        break;
      }
    }
    if (intr_name.isEmpty()) {
      intr_name = "camera_intrinsics_used.yaml";
    }
    const QString intr_path = QDir(ImageDirectoryForHandeyePosesCsv(csv)).filePath(intr_name);
    cv::FileStorage fs(intr_path.toStdString(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
      if (err_msg) *err_msg = QString("无法读取内参文件: %1").arg(intr_path);
      return false;
    }
    fs["camera_matrix"] >> *K;
    fs["distortion_coefficients"] >> *D;
    if (K->empty() || D->empty()) {
      if (err_msg) *err_msg = QString("内参文件格式无效: %1").arg(intr_path);
      return false;
    }
    return true;
  };

  QObject::connect(run_btn, &QPushButton::clicked, [=]() {
    QString err;
    const HandeyeSetupMode hm = handeye_setup->currentIndex() == 0 ? HandeyeSetupMode::EyeInHand : HandeyeSetupMode::EyeToHand;
    const std::vector<std::pair<QString, QString>> extra_fields = {
      {"handeye_setup", HandeyeSetupModeToYamlString(hm)},
      {"base_frame", rt_base_frame->text()},
      {"ee_frame", rt_ee_frame->text()},
      {HandeyeThirdFrameYamlKey(hm), rt_third_frame->text()},
    };
    QDir out_dir(QFileInfo(output_yaml->text()).absolutePath());
    if (!out_dir.exists()) {
      out_dir = QDir::home();
    }
    const QString final_output_yaml = build_handeye_output_file(out_dir);
    if (!SaveCalibrationYaml(
          final_output_yaml, "手眼标定", "online", rt_image_topic->currentText(),
          board_type->currentText(), distortion_model->currentText(),
          rt_image_topic->currentText(), rt_camera_info_topic->currentText(), extra_fields, &err)) {
      QMessageBox::critical(this, "错误", "保存标定结果失败: " + err);
      append_log("实时标定失败: " + err);
      return;
    }
    output_yaml->setText(final_output_yaml);
    append_log(QString("实时标定完成，结果已保存: %1").arg(final_output_yaml));
    if (!show_raw_checkbox->isChecked() && *update_preview) (*update_preview)();
  });

  QObject::connect(off_run_btn, &QPushButton::clicked, [=]() {
    QString err;
    const QString csv = off_csv_path->text().trimmed();
    if (!ValidateHandeyePosesCsvFile(csv, &err)) {
      QMessageBox::warning(this, "CSV 无效", err);
      append_log("离线标定失败: " + err);
      return;
    }
    std::vector<QString> check_names;
    if (!ListImageFilenamesFromHandeyePosesCsv(csv, &check_names, nullptr, &err)) {
      QMessageBox::warning(this, "CSV 解析失败", err);
      append_log("离线标定失败: " + err);
      return;
    }
    const QString image_dir = ImageDirectoryForHandeyePosesCsv(csv);
    // 离线结果固定保存到导入 CSV 所在目录，并确保文件名不与已有文件重名。
    const QFileInfo csv_info(csv);
    const QDir csv_dir = csv_info.dir();
    QString final_output_yaml = build_handeye_output_file(csv_dir);
    const HandeyeSetupMode hm = handeye_setup->currentIndex() == 0 ? HandeyeSetupMode::EyeInHand : HandeyeSetupMode::EyeToHand;
    const std::vector<std::pair<QString, QString>> extra_fields = {{"pose_csv", csv},
      {"image_dir", image_dir},
      {"handeye_setup", HandeyeSetupModeToYamlString(hm)},
      {"handeye_solver", handeye_solver->currentText()},
      {"base_frame", off_base_frame->text()},
      {"ee_frame", off_ee_frame->text()},
      {HandeyeThirdFrameYamlKey(hm), off_third_frame->text()}};
    if (board_type->currentText() == "Aruco Single Marker") {
      const int target_id = single_aruco_id->value();
      int existing_image_count = 0;
      for (const auto & n : check_names) {
        if (!resolve_offline_image_path(csv, n).isEmpty()) {
          ++existing_image_count;
        }
      }
      if (existing_image_count == 0) {
        QMessageBox::critical(this, "错误", "离线手眼标定失败: CSV 对应图像文件未找到，无法执行求解。");
        append_log("离线标定失败: CSV 对应图像文件未找到，无法执行求解。");
        return;
      }
      std::vector<OfflineSampleRow> rows;
      if (!parse_offline_rows(csv, &rows, &err)) {
        QMessageBox::warning(this, "CSV 解析失败", err);
        append_log("离线标定失败: " + err);
        return;
      }
      cv::Mat K, D;
      if (!load_intrinsics(csv, rows, &K, &D, &err)) {
        QMessageBox::warning(this, "内参加载失败", err);
        append_log("离线标定失败: " + err);
        return;
      }
      const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
      cv::aruco::DetectorParameters params;
      cv::aruco::ArucoDetector detector(dict, params);
      const double marker_length_m = 0.06;
      append_log(QString("开始离线单码标定，共 %1 张，目标ID=%2").arg(rows.size()).arg(target_id));
      for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
        const auto & row = rows[i];
        const QString raw_path = resolve_offline_image_path(csv, row.raw_image);
        if (raw_path.isEmpty()) {
          append_log(QString("样本[%1/%2] 图像缺失: %3").arg(i + 1).arg(rows.size()).arg(row.raw_image));
          continue;
        }
        // 先刷原图。
        if (*update_preview) {
          const bool old_show_raw = show_raw_checkbox->isChecked();
          show_raw_checkbox->blockSignals(true);
          show_raw_checkbox->setChecked(true);
          show_raw_checkbox->blockSignals(false);
          *offline_image_index = std::max(0, std::min(i, static_cast<int>(offline_image_names->size()) - 1));
          update_offline_preview_nav_ui();
          (*update_preview)();
          show_raw_checkbox->blockSignals(true);
          show_raw_checkbox->setChecked(old_show_raw);
          show_raw_checkbox->blockSignals(false);
          QApplication::processEvents();
        }
        cv::Mat bgr = cv::imread(raw_path.toStdString(), cv::IMREAD_COLOR);
        if (bgr.empty()) {
          append_log(QString("样本[%1/%2] 图像读取失败: %3").arg(i + 1).arg(rows.size()).arg(raw_path));
          continue;
        }
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector.detectMarkers(bgr, corners, ids);
        int idx = -1;
        for (int k = 0; k < static_cast<int>(ids.size()); ++k) {
          if (ids[k] == target_id) {
            idx = k;
            break;
          }
        }
        if (idx < 0) {
          append_log(QString("样本[%1/%2] 未检测到目标ID=%3，已跳过。")
              .arg(i + 1).arg(rows.size()).arg(target_id));
          continue;
        }
        std::vector<std::vector<cv::Point2f>> one_corner = {corners[idx]};
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
          one_corner, marker_length_m, K, D, rvecs, tvecs);
        if (rvecs.empty() || tvecs.empty()) {
          append_log(QString("样本[%1/%2] 目标ID=%3 位姿估计失败。")
              .arg(i + 1).arg(rows.size()).arg(target_id));
          continue;
        }
        append_log(QString("样本[%1/%2] ID=%3 t_target_cam=[%4, %5, %6] rvec=[%7, %8, %9]")
            .arg(i + 1).arg(rows.size()).arg(target_id)
            .arg(tvecs[0][0], 0, 'f', 6).arg(tvecs[0][1], 0, 'f', 6).arg(tvecs[0][2], 0, 'f', 6)
            .arg(rvecs[0][0], 0, 'f', 6).arg(rvecs[0][1], 0, 'f', 6).arg(rvecs[0][2], 0, 'f', 6));
        cv::aruco::drawDetectedMarkers(bgr, one_corner, std::vector<int>{target_id});
        cv::drawFrameAxes(bgr, K, D, rvecs[0], tvecs[0], marker_length_m * 0.7);
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        QImage q(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888);
        preview_widget->SetImage(q.copy());
        QApplication::processEvents();
      }
      HandeyeSolverMethod solver_method = HandeyeSolverMethod::Park;
      if (handeye_solver->currentText() == "TSAI") solver_method = HandeyeSolverMethod::Tsai;
      else if (handeye_solver->currentText() == "PARK") solver_method = HandeyeSolverMethod::Park;
      else if (handeye_solver->currentText() == "HORAUD") solver_method = HandeyeSolverMethod::Horaud;
      else if (handeye_solver->currentText() == "ANDREFF") solver_method = HandeyeSolverMethod::Andreff;
      else if (handeye_solver->currentText() == "DANIILIDIS") solver_method = HandeyeSolverMethod::Daniilidis;
      append_log(QString("离线求解算法: %1").arg(handeye_solver->currentText()));
      QString summary;
      QString detail_log;
      if (!RunOfflineHandeyeCalibrationForSingleAruco(
            csv, hm, solver_method, marker_length_spin->value(), target_id,
            final_output_yaml, &summary, &detail_log, &err)) {
        QMessageBox::critical(this, "错误", "离线手眼标定失败: " + err);
        append_log("离线标定失败: " + err);
        return;
      }
      for (const auto & line : detail_log.split('\n', Qt::SkipEmptyParts)) {
        append_log(line);
      }
      append_log(summary);
      render_final_matrix_text(detail_log + "\n" + summary);
    } else {
      // 兼容旧流程：其他板型暂按配置导出，后续可继续接入对应离线求解。
      if (!SaveCalibrationYaml(
            final_output_yaml, "手眼标定", "offline", csv,
            board_type->currentText(), distortion_model->currentText(),
            image_dir, off_camera_info_topic->currentText(), extra_fields, &err)) {
        QMessageBox::critical(this, "错误", "保存标定结果失败: " + err);
        append_log("离线标定失败: " + err);
        return;
      }
      matrix_result->setPlainText("当前板型暂未接入矩阵求解显示。");
    }
    output_yaml->setText(final_output_yaml);
    append_log(QString("离线标定结果已保存: %1 (CSV: %2，共 %3 行)")
        .arg(final_output_yaml)
        .arg(csv)
        .arg(static_cast<int>(check_names.size())));
    if (!show_raw_checkbox->isChecked() && *update_preview) (*update_preview)();
  });

  const QString default_cfg = ResolveDefaultConfigYamlPath("handeye_calibration.yaml");
  RosInterfaceConfig cfg;
  QString cfg_err;
  if (LoadRosInterfaceConfigFromYaml(default_cfg, &cfg, &cfg_err)) {
    const auto topics = ListOnlineImageTopics();
    PopulateTopicCombo(rt_image_topic, topics, cfg.image_topic.isEmpty() ? "/camera/image_raw" : cfg.image_topic);
    if (!cfg.pose_endpoint.trimmed().isEmpty()) {
      rt_camera_info_topic->setCurrentText(cfg.pose_endpoint);
      off_camera_info_topic->setCurrentText(cfg.pose_endpoint);
    }
    handeye_setup->setCurrentIndex(
      HandeyeSetupModeFromYamlString(cfg.handeye_setup) == HandeyeSetupMode::EyeInHand ? 0 : 1);
    sync_handeye_frame_labels();
    if (!cfg.handeye_poses_csv.trimmed().isEmpty()) {
      off_csv_path->setText(cfg.handeye_poses_csv);
    }
    append_log("已加载默认配置: " + default_cfg);
  } else {
    PopulateTopicCombo(rt_image_topic, ListOnlineImageTopics(), "/camera/image_raw");
    append_log("默认配置加载失败，使用界面默认值。");
  }
  PopulateTopicCombo(rt_camera_info_topic, ListOnlineCameraInfoTopics(), rt_camera_info_topic->currentText());
  PopulateTopicCombo(off_camera_info_topic, {}, "/camera/camera_info");
  if (!off_csv_path->text().trimmed().isEmpty()) {
    repopulate_offline_from_csv(false);
  }
  update_mode_visibility();
  update_board_type_ui();
  update_offline_validation_mode_ui();
  sync_handeye_frame_labels();
  subscribe_preview_topic();
}

}  // namespace ros_robot_assist_tools::ui
