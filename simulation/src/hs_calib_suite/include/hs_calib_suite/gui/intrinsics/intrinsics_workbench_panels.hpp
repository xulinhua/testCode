#pragma once

#include <functional>

#include <QDialog>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QPushButton;
class QScrollArea;
class QToolButton;

namespace hs_calib {
namespace gui {

class SessionController;

/// \brief 预览区下方：单行摘要 +「检测详情…」打开非模态弹窗
class IntrinsicsMetricsStrip : public QWidget {
  Q_OBJECT

public:
  explicit IntrinsicsMetricsStrip(QWidget *parent = nullptr);
  void refresh(const SessionController *session);

signals:
  void details_requested();

private:
  QLabel *lbl_summary_ = nullptr;
  QPushButton *btn_details_ = nullptr;
};

/// \brief 检测详情 + Single-shot 重投影（非模态，Tier4 §6–7）
class IntrinsicsDetectionDetailsDialog : public QDialog {
  Q_OBJECT

public:
  explicit IntrinsicsDetectionDetailsDialog(QWidget *parent = nullptr);
  void refresh(const SessionController *session);

private:
  QLabel *lbl_detected_ = nullptr;
  QLabel *lbl_tilt_ = nullptr;
  QLabel *lbl_angles_ = nullptr;
  QLabel *lbl_position_ = nullptr;
  QLabel *lbl_skew_ = nullptr;
  QLabel *lbl_area_ = nullptr;
  QLabel *lbl_linear_ = nullptr;
  QLabel *lbl_aspect_ = nullptr;
  QLabel *lbl_reproj_max_ = nullptr;
  QLabel *lbl_reproj_avg_ = nullptr;
  QLabel *lbl_reproj_rms_ = nullptr;
  QWidget *reproj_section_ = nullptr;
};

/// \brief Tier4 Visualization options（§13）非模态设置页
class IntrinsicsVizOptionsDialog : public QDialog {
  Q_OBJECT

public:
  explicit IntrinsicsVizOptionsDialog(QWidget *parent = nullptr);

  void bind_session(SessionController *session);
  void load_from_session();
  void apply_to_session();

signals:
  void options_applied();

private:
  void wire_live_apply();

  SessionController *session_ = nullptr;
  QCheckBox *chk_draw_detection_ = nullptr;
  QCheckBox *chk_draw_train_pts_ = nullptr;
  QCheckBox *chk_draw_eval_pts_ = nullptr;
  QCheckBox *chk_draw_train_occ_ = nullptr;
  QCheckBox *chk_draw_eval_occ_ = nullptr;
  QCheckBox *chk_draw_linearity_ = nullptr;
  QCheckBox *chk_draw_indicators_ = nullptr;
  QDoubleSpinBox *spin_drawings_alpha_ = nullptr;
  QDoubleSpinBox *spin_indicators_alpha_ = nullptr;
  QPushButton *btn_clear_linearity_ = nullptr;
};

/// \brief 标定状态摘要（非模态）
class IntrinsicsCalibrationStatusDialog : public QDialog {
  Q_OBJECT

public:
  explicit IntrinsicsCalibrationStatusDialog(QWidget *parent = nullptr);
  void refresh(const SessionController *session);

private:
  QLabel *lbl_status_ = nullptr;
  QLabel *lbl_time_ = nullptr;
  QLabel *lbl_train_ = nullptr;
  QLabel *lbl_pre_ = nullptr;
  QLabel *lbl_post_ = nullptr;
  QLabel *lbl_train_rms_ = nullptr;
  QLabel *lbl_train_rms_inlier_ = nullptr;
  QLabel *lbl_eval_ = nullptr;
  QLabel *lbl_eval_rms_ = nullptr;
  QLabel *lbl_eval_rms_inlier_ = nullptr;
  QLabel *lbl_eval_inliers_ = nullptr;
  QLabel *lbl_train_occ_ = nullptr;
  QLabel *lbl_eval_occ_ = nullptr;
  QLabel *lbl_last_det_ = nullptr;
};

/// \brief Tier4 工作台右栏：Solver + 操作按钮（标定模式在标定设置页配置）
class IntrinsicsControlRail : public QWidget {
  Q_OBJECT

public:
  explicit IntrinsicsControlRail(QWidget *parent = nullptr);

  void set_session(SessionController *session);
  void refresh();

  QComboBox *solver_combo() const { return combo_solver_; }

  void sync_from_session();

signals:
  void calibration_params_requested();
  void detector_params_requested();
  void collector_params_requested();
  void statistics_requested();
  void status_details_requested();

private:
  SessionController *session_ = nullptr;
  QComboBox *combo_solver_ = nullptr;
  QLabel *lbl_compact_status_ = nullptr;
  QLabel *lbl_compact_collect_ = nullptr;
};

}  // namespace gui
}  // namespace hs_calib
