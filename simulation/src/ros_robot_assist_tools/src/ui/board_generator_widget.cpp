#include "ros_robot_assist_tools/ui/board_generator_widget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QImage>
#include <QFileInfo>
#include <QLabel>
#include <QObject>
#include <QLineEdit>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QSpinBox>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <memory>

#include "ros_robot_assist_tools/module/board_generator_module.h"

namespace ros_robot_assist_tools::ui
{
namespace
{

/// 以原图分辨率放入场景，滚轮缩放视图（不先缩小位图），避免放大后边缘发糊。
class BoardImagePreviewView : public QGraphicsView
{
public:
  explicit BoardImagePreviewView(QWidget * parent = nullptr)
  : QGraphicsView(parent)
  {
    setScene(new QGraphicsScene(this));
    item_ = new QGraphicsPixmapItem();
    scene()->addItem(item_);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setRenderHint(QPainter::SmoothPixmapTransform, false);
    setRenderHint(QPainter::Antialiasing, false);
    setBackgroundBrush(QColor(0xeaf2f8));
    setFrameShape(QFrame::Box);
    setFrameShadow(QFrame::Plain);
    setMinimumHeight(280);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
    item_->setTransformationMode(Qt::FastTransformation);
#endif
  }

  void setBoardPixmap(const QPixmap & pm)
  {
    item_->setPixmap(pm);
    if (pm.isNull()) {
      scene()->setSceneRect(0, 0, 320, 240);
      resetTransform();
      return;
    }
    scene()->setSceneRect(pm.rect());
    resetTransform();
    fitInView(item_, Qt::KeepAspectRatio);
  }

protected:
  void wheelEvent(QWheelEvent * e) override
  {
    if (item_->pixmap().isNull()) {
      QGraphicsView::wheelEvent(e);
      return;
    }
    constexpr double kStep = 1.12;
    const double factor = (e->angleDelta().y() > 0) ? kStep : (1.0 / kStep);
    scale(factor, factor);
    e->accept();
  }

  void mouseDoubleClickEvent(QMouseEvent * e) override
  {
    if (e->button() == Qt::LeftButton && !item_->pixmap().isNull()) {
      resetTransform();
      fitInView(item_, Qt::KeepAspectRatio);
      e->accept();
      return;
    }
    QGraphicsView::mouseDoubleClickEvent(e);
  }

private:
  QGraphicsPixmapItem * item_{nullptr};
};

QString BuildBoardExportBaseName(
  int board_type_idx,
  const QString & dict_name,
  const QString & marker_size_px,
  const QString & board_w_mm,
  const QString & board_h_mm,
  int rows,
  int cols,
  bool aruco_border_on = false,
  int aruco_border_px = 0)
{
  QString board_key = "board";
  switch (board_type_idx) {
    case 0: board_key = "aruco"; break;
    case 1: board_key = "chessboard"; break;
    case 2: board_key = "circle_grid"; break;
    case 3: board_key = "asym_circle"; break;
    case 4: board_key = "charuco"; break;
    case 5: board_key = "aprilgrid"; break;
    default: break;
  }

  QString dict_key = dict_name.trimmed();
  if (dict_key.isEmpty()) {
    dict_key = "nodict";
  }
  if (board_type_idx == 5) {
    dict_key = "DICT_APRILTAG_36h11";
  }
  dict_key.replace("/", "_");
  dict_key.replace(" ", "");

  QString size_key;
  if (board_type_idx == 0) {
    size_key = QString("%1px").arg(marker_size_px.trimmed());
    if (aruco_border_on && aruco_border_px > 0) {
      size_key += QString("_bs%1").arg(aruco_border_px);
    }
  } else {
    size_key = QString("%1x%2mm_%3x%4")
      .arg(board_w_mm.trimmed())
      .arg(board_h_mm.trimmed())
      .arg(rows)
      .arg(cols);
  }
  return QString("%1_%2_%3")
    .arg(board_key)
    .arg(dict_key)
    .arg(size_key);
}

}  // namespace


BoardGeneratorWidget::BoardGeneratorWidget(QWidget * parent)
: QWidget(parent)
{
  QVBoxLayout * layout = new QVBoxLayout(this);
  QLabel * title = new QLabel("标定板生成与导出");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  layout->addWidget(title);

  QComboBox * board_type = new QComboBox();
  board_type->addItems({"Aruco码", "棋盘格", "圆点网格", "不对称圆点", "ChArUco", "Kalibr/AprilGrid"});
  QComboBox * dict = new QComboBox();
  dict->addItems({"DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000", "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000", "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000", "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000", "DICT_ARUCO_ORIGINAL", "DICT_APRILTAG_16h5", "DICT_APRILTAG_25h9", "DICT_APRILTAG_36h10", "DICT_APRILTAG_36h11"});
  QLineEdit * marker_id = new QLineEdit("0");
  QLineEdit * start_id = new QLineEdit("0");
  QLineEdit * marker_size = new QLineEdit("400");
  QSpinBox * grid_rows = new QSpinBox();
  grid_rows->setRange(2, 100);
  grid_rows->setValue(9);
  QSpinBox * grid_cols = new QSpinBox();
  grid_cols->setRange(2, 100);
  grid_cols->setValue(11);
  QLineEdit * circle_radius = new QLineEdit("18");
  QLineEdit * board_width_mm = new QLineEdit("200");
  QLineEdit * board_height_mm = new QLineEdit("150");
  QLineEdit * cell_size_mm = new QLineEdit("25");
  QLineEdit * circle_diameter_mm = new QLineEdit("12");
  QLineEdit * tag_size_mm = new QLineEdit("12");
  QLineEdit * marker_ratio = new QLineEdit("0.75");

  QLabel * aruco_dict_hint = new QLabel(
    QStringLiteral("说明：名称中「4×4 / 5×5 …」为码内编码位数；末尾 50 / 100 / 250 / 1000 为字典中不同标记的个数（"
                   "可用 ID 为 0～N−1），与下方 MarkSize 像素尺寸不是同一含义。"));
  aruco_dict_hint->setWordWrap(true);
  aruco_dict_hint->setStyleSheet("color: #5a6c7a; font-size: 11px;");
  QWidget * aruco_border_wrap = new QWidget();
  QHBoxLayout * aruco_border_layout = new QHBoxLayout(aruco_border_wrap);
  aruco_border_layout->setContentsMargins(0, 0, 0, 0);
  QCheckBox * aruco_border_enable = new QCheckBox(QStringLiteral("增加白边"));
  QSpinBox * aruco_border_px = new QSpinBox();
  aruco_border_px->setRange(0, 2000);
  aruco_border_px->setValue(20);
  aruco_border_px->setEnabled(false);
  aruco_border_px->setSuffix(QStringLiteral(" px"));
  aruco_border_px->setToolTip(
    QStringLiteral("四边对称留白（像素/侧）。输出边长 = MarkSize + 2×该值（例：MarkSize=100、每侧 20 → 边长 140）。"));
  aruco_border_layout->addWidget(aruco_border_enable);
  aruco_border_layout->addWidget(new QLabel(QStringLiteral("每侧宽度:")));
  aruco_border_layout->addWidget(aruco_border_px);
  aruco_border_layout->addStretch();
  QObject::connect(aruco_border_enable, &QCheckBox::toggled, aruco_border_px, &QWidget::setEnabled);

  QFormLayout * form = new QFormLayout();
  form->addRow("标定板类型:", board_type);
  form->addRow("字典:", dict);
  form->addRow(aruco_dict_hint);
  form->addRow("MarkSize (像素):", marker_size);
  form->addRow("Aruco ID:", marker_id);
  form->addRow("起始ID:", start_id);
  form->addRow(QStringLiteral("白边:"), aruco_border_wrap);
  form->addRow("行", grid_rows);
  form->addRow("列", grid_cols);
  form->addRow("圆点半径:", circle_radius);
  form->addRow("板宽 [毫米]", board_width_mm);
  form->addRow("板高 [mm]", board_height_mm);
  form->addRow("单元尺寸(mm):", cell_size_mm);
  form->addRow("圆直径(mm):", circle_diameter_mm);
  form->addRow("Tag Size(mm):", tag_size_mm);
  form->addRow("Marker比例(0-1):", marker_ratio);
  layout->addLayout(form);

  auto * preview = new BoardImagePreviewView(this);
  preview->setStyleSheet("border: 1px solid #d0d8de;");
  layout->addWidget(preview, 1);
  QTextEdit * log = new QTextEdit();
  log->setReadOnly(true);
  layout->addWidget(log);

  auto board_image = std::make_shared<cv::Mat>();
  QHBoxLayout * btns = new QHBoxLayout();
  QPushButton * gen = new QPushButton("生成");
  QPushButton * save_img = new QPushButton("导出图片");
  QPushButton * save_dae = new QPushButton("导出DAE");
  btns->addWidget(gen);
  btns->addWidget(save_img);
  btns->addWidget(save_dae);
  layout->addLayout(btns);

  auto update_ui_by_board_type = [=]() {
    const int t = board_type->currentIndex();
    const bool show_dict = (t == 0 || t == 4);
    const bool show_marker_id = (t == 0);
    const bool show_start_id = (t == 4 || t == 5);
    const bool show_marker_ratio = (t == 4);
    const bool show_tag_size = (t == 5);
    const bool show_circle_diameter_mm = (t == 2 || t == 3);
    const bool show_cell_size_mm = (t != 5 && t != 0);
    const bool show_marker_size = (t != 5);
    const bool show_circle_radius_ui = (t == 2 || t == 3);
    const bool show_board_size = (t != 0);
    const bool show_rows_cols = (t != 0);
    const bool show_aruco_extra = (t == 0);
    if (auto * l = form->labelForField(dict)) { l->setVisible(show_dict); }
    dict->setVisible(show_dict);
    aruco_dict_hint->setVisible(show_aruco_extra);
    aruco_border_wrap->setVisible(show_aruco_extra);
    if (auto * l = form->labelForField(aruco_border_wrap)) { l->setVisible(show_aruco_extra); }
    if (auto * l = form->labelForField(marker_id)) { l->setVisible(show_marker_id); }
    marker_id->setVisible(show_marker_id);
    if (auto * l = form->labelForField(start_id)) { l->setVisible(show_start_id); }
    start_id->setVisible(show_start_id);
    if (auto * l = form->labelForField(circle_radius)) { l->setVisible(show_circle_radius_ui); }
    circle_radius->setVisible(show_circle_radius_ui);
    if (auto * l = form->labelForField(circle_diameter_mm)) { l->setVisible(show_circle_diameter_mm); }
    circle_diameter_mm->setVisible(show_circle_diameter_mm);
    if (auto * l = form->labelForField(tag_size_mm)) { l->setVisible(show_tag_size); }
    tag_size_mm->setVisible(show_tag_size);
    if (auto * l = form->labelForField(marker_ratio)) { l->setVisible(show_marker_ratio); }
    marker_ratio->setVisible(show_marker_ratio);
    if (auto * w = form->labelForField(marker_size)) {
      w->setVisible(show_marker_size);
      if (show_marker_size) {
        if (auto * lbl = qobject_cast<QLabel *>(w)) {
          lbl->setText((t == 0) ? QStringLiteral("MarkSize (像素):") : QStringLiteral("像素尺寸:"));
        }
      }
    }
    marker_size->setVisible(show_marker_size);
    if (auto * l = form->labelForField(cell_size_mm)) { l->setVisible(show_cell_size_mm); }
    cell_size_mm->setVisible(show_cell_size_mm);
    if (auto * l = form->labelForField(board_width_mm)) { l->setVisible(show_board_size); }
    board_width_mm->setVisible(show_board_size);
    if (auto * l = form->labelForField(board_height_mm)) { l->setVisible(show_board_size); }
    board_height_mm->setVisible(show_board_size);
    if (auto * l = form->labelForField(grid_rows)) { l->setVisible(show_rows_cols); }
    grid_rows->setVisible(show_rows_cols);
    if (auto * l = form->labelForField(grid_cols)) { l->setVisible(show_rows_cols); }
    grid_cols->setVisible(show_rows_cols);

    if (t == 5) {
      if (board_width_mm->text().trimmed().isEmpty()) { board_width_mm->setText("200"); }
      if (board_height_mm->text().trimmed().isEmpty()) { board_height_mm->setText("150"); }
      if (grid_rows->value() <= 0) { grid_rows->setValue(9); }
      if (grid_cols->value() <= 0) { grid_cols->setValue(11); }
      if (tag_size_mm->text().trimmed().isEmpty()) { tag_size_mm->setText("9"); }
      if (start_id->text().trimmed().isEmpty()) { start_id->setText("0"); }
    }
  };
  QObject::connect(board_type, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int) {
    update_ui_by_board_type();
  });
  update_ui_by_board_type();

  QObject::connect(gen, &QPushButton::clicked, [=]() {
    BoardGeneratorParams params;
    params.board_type = board_type->currentIndex();
    params.dict_index = dict->currentIndex();
    // Kalibr/AprilGrid 固定使用 AprilTag 字典，避免沿用其它类型字典导致图案不一致
    if (params.board_type == 5) {
      params.dict_index = 20;  // DICT_APRILTAG_36h11
    }
    params.marker_id = marker_id->text().toInt();
    params.start_id = start_id->text().toInt();
    params.marker_size = marker_size->text().toInt();
    params.rows = grid_rows->value();
    params.cols = grid_cols->value();
    params.board_width_mm = board_width_mm->text().toDouble();
    params.board_height_mm = board_height_mm->text().toDouble();
    params.cell_size_mm = cell_size_mm->text().toDouble();
    params.circle_diameter_mm = circle_diameter_mm->text().toDouble();
    params.tag_size_mm = tag_size_mm->text().toDouble();
    params.marker_ratio = marker_ratio->text().toDouble();
    params.aruco_white_border = (params.board_type == 0 && aruco_border_enable->isChecked());
    params.aruco_border_px = aruco_border_px->value();

    QString err;
    if (!GenerateCalibrationBoard(params, board_image.get(), &err)) {
      QMessageBox::warning(this, "提示", "生成失败: " + err);
      return;
    }
    QImage img(board_image->data, board_image->cols, board_image->rows, board_image->step, QImage::Format_Grayscale8);
    // copy：避免与后续 Generate 复用的 cv::Mat 缓冲区别名；全分辨率供缩放。
    preview->setBoardPixmap(QPixmap::fromImage(img.copy()));
    if (params.board_type == 0) {
      log->append(QString("Aruco 生成成功：字典=%1，MarkSize=%2 px，ID=%3，输出边长=%4 px%5")
                    .arg(dict->currentText())
                    .arg(params.marker_size)
                    .arg(params.marker_id)
                    .arg(board_image->cols)
                    .arg(params.aruco_white_border && params.aruco_border_px > 0
                           ? QStringLiteral("（每侧白边 %1 px，边长=MarkSize+2×%1）").arg(params.aruco_border_px)
                           : QString()));
    } else {
      log->append(QString("%1 生成成功。参数: %2x%3, 板尺寸=%4x%5mm, 圆直径=%6mm, 起始ID=%7")
                  .arg(board_type->currentText())
                  .arg(params.cols)
                  .arg(params.rows)
                  .arg(params.board_width_mm, 0, 'f', 1)
                  .arg(params.board_height_mm, 0, 'f', 1)
                  .arg(params.circle_diameter_mm, 0, 'f', 1)
                  .arg(params.start_id));
    }
  });

  QObject::connect(save_img, &QPushButton::clicked, [=]() {
    if (board_image->empty()) {
      QMessageBox::warning(this, "提示", "请先生成标定板");
      return;
    }
    const QString default_base_name = BuildBoardExportBaseName(
      board_type->currentIndex(),
      dict->currentText(),
      marker_size->text(),
      board_width_mm->text(),
      board_height_mm->text(),
      grid_rows->value(),
      grid_cols->value(),
      aruco_border_enable->isChecked(),
      aruco_border_px->value());
    const QString default_path = QDir::homePath() + "/" + default_base_name + ".bmp";
    const QString path = QFileDialog::getSaveFileName(this, "导出图片", default_path, "Image (*.png *.bmp *.jpg)");
    if (path.isEmpty()) { return; }
    QString err;
    if (!ExportCalibrationBoardImage(*board_image, path, &err)) {
      QMessageBox::warning(this, "提示", "导出失败: " + err);
      return;
    }
    log->append("图片导出: " + path);
  });

  QObject::connect(save_dae, &QPushButton::clicked, [=]() {
    if (board_image->empty()) {
      QMessageBox::warning(this, "提示", "请先生成标定板");
      return;
    }
    const QString default_base_name = BuildBoardExportBaseName(
      board_type->currentIndex(),
      dict->currentText(),
      marker_size->text(),
      board_width_mm->text(),
      board_height_mm->text(),
      grid_rows->value(),
      grid_cols->value(),
      aruco_border_enable->isChecked(),
      aruco_border_px->value());
    const QString default_path = QDir::homePath() + "/" + default_base_name + ".dae";
    const QString dae_path = QFileDialog::getSaveFileName(this, "导出DAE", default_path, "DAE (*.dae)");
    if (dae_path.isEmpty()) { return; }
    QString err;
    if (!ExportCalibrationBoardDae(*board_image, dae_path, &err)) {
      QMessageBox::warning(this, "提示", "导出失败: " + err);
      return;
    }
    log->append("DAE 导出: " + dae_path);
  });

}

}  // namespace ros_robot_assist_tools::ui
