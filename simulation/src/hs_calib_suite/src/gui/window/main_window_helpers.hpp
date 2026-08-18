#pragma once

/// \file 主窗口内部 UI 小工具（仅 gui/window 编译单元使用，不对外暴露）

#include <functional>

#include <QAction>
#include <QColor>
#include <QEvent>
#include <QFont>
#include <QIcon>
#include <QLabel>
#include <QMouseEvent>
#include <QObject>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QWidget>

namespace hs_calib {
namespace gui {
namespace window_detail {

/// \brief 按 objectName 创建并设置字体样式的标签
inline QLabel *make_label(const QString &text, const QString &object_name, QWidget *parent) {
  auto *label = new QLabel(text, parent);
  label->setObjectName(object_name);

  QFont f = label->font();
  if (object_name == QStringLiteral("PageTitle")) {
    f.setPointSize(22);
    f.setWeight(QFont::Bold);
  } else if (object_name == QStringLiteral("PageSubtitle")) {
    f.setPointSize(12);
    f.setWeight(QFont::Normal);
  } else if (object_name == QStringLiteral("SectionTitle")) {
    f.setPointSize(10);
    f.setWeight(QFont::DemiBold);
    f.setCapitalization(QFont::AllUppercase);
  } else if (object_name == QStringLiteral("BrandMark")) {
    f.setFamily(QStringLiteral("Noto Sans Mono"));
    f.setStyleHint(QFont::Monospace);
    f.setPointSize(11);
    f.setWeight(QFont::Bold);
    f.setLetterSpacing(QFont::AbsoluteSpacing, 1.5);
  } else if (object_name == QStringLiteral("CalibTileTitle")) {
    f.setPointSize(15);
    f.setWeight(QFont::Bold);
  } else if (object_name == QStringLiteral("MetricValue")) {
    f.setFamily(QStringLiteral("Noto Sans Mono"));
    f.setStyleHint(QFont::Monospace);
    f.setPointSize(22);
    f.setWeight(QFont::Bold);
  } else if (object_name == QStringLiteral("MetricName")) {
    f.setPointSize(10);
    f.setWeight(QFont::DemiBold);
    f.setCapitalization(QFont::AllUppercase);
  } else if (
      object_name == QStringLiteral("StepActive") ||
      object_name == QStringLiteral("StepDone") ||
      object_name == QStringLiteral("StepIdle")) {
    f.setFamily(QStringLiteral("Noto Sans Mono"));
    f.setStyleHint(QFont::Monospace);
    f.setPointSize(11);
    f.setWeight(QFont::DemiBold);
  } else if (object_name == QStringLiteral("StatusBarPage")) {
    f.setFamily(QStringLiteral("Noto Sans Mono"));
    f.setStyleHint(QFont::Monospace);
    f.setPointSize(10);
    f.setWeight(QFont::DemiBold);
  }
  label->setFont(f);
  return label;
}

/// \brief 磁贴左键点击过滤器
class TileClickFilter : public QObject {
public:
  using Callback = std::function<void()>;
  /// \brief 绑定点击回调
  explicit TileClickFilter(Callback cb, QObject *parent = nullptr)
      : QObject(parent), cb_(std::move(cb)) {}

protected:
  /// \brief 左键释放时触发回调
  bool eventFilter(QObject *watched, QEvent *event) override {
    if (event->type() == QEvent::MouseButtonRelease) {
      auto *me = static_cast<QMouseEvent *>(event);
      if (me->button() == Qt::LeftButton && cb_) {
        cb_();
        return true;
      }
    }
    return QObject::eventFilter(watched, event);
  }

private:
  Callback cb_;
};

enum class TbGlyph {
  Home,
  Setup,
  Workbench,
  Review,
  Offline,
  Online,
  Capture,
  Solve,
  Export,
};

/// \brief 绘制工具栏矢量图标
inline QIcon make_toolbar_icon(TbGlyph glyph, const QColor &ink, const QColor &accent) {
  QIcon icon;
  for (int s : {16, 20, 24, 32}) {
    QPixmap pm(s, s);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing, true);
    const qreal m = s / 24.0;
    QPen pen(ink, 1.7 * m, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    auto r = [&](qreal x, qreal y, qreal w, qreal h) {
      return QRectF(x * m, y * m, w * m, h * m);
    };
    switch (glyph) {
      case TbGlyph::Home:
        p.drawLine(QPointF(4 * m, 11 * m), QPointF(12 * m, 4 * m));
        p.drawLine(QPointF(12 * m, 4 * m), QPointF(20 * m, 11 * m));
        p.drawRect(r(7, 11, 10, 9));
        break;
      case TbGlyph::Setup:
        p.drawRoundedRect(r(5, 5, 14, 14), 2 * m, 2 * m);
        p.drawLine(QPointF(9 * m, 9 * m), QPointF(15 * m, 9 * m));
        p.drawLine(QPointF(9 * m, 12 * m), QPointF(15 * m, 12 * m));
        p.drawLine(QPointF(9 * m, 15 * m), QPointF(13 * m, 15 * m));
        break;
      case TbGlyph::Workbench:
        p.drawRect(r(4, 5, 16, 14));
        p.drawLine(QPointF(4 * m, 10 * m), QPointF(20 * m, 10 * m));
        p.drawLine(QPointF(10 * m, 10 * m), QPointF(10 * m, 19 * m));
        break;
      case TbGlyph::Review:
        p.drawRoundedRect(r(6, 4, 12, 16), 1.5 * m, 1.5 * m);
        p.drawLine(QPointF(9 * m, 9 * m), QPointF(15 * m, 9 * m));
        p.drawLine(QPointF(9 * m, 12 * m), QPointF(15 * m, 12 * m));
        p.drawLine(QPointF(9 * m, 15 * m), QPointF(13 * m, 15 * m));
        break;
      case TbGlyph::Offline:
        p.drawEllipse(r(5, 5, 14, 14));
        p.drawLine(QPointF(8 * m, 8 * m), QPointF(16 * m, 16 * m));
        break;
      case TbGlyph::Online:
        p.setPen(QPen(accent, 1.7 * m, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.drawEllipse(r(9, 9, 6, 6));
        p.drawArc(r(5, 5, 14, 14), 45 * 16, 90 * 16);
        p.drawArc(r(2, 2, 20, 20), 45 * 16, 90 * 16);
        break;
      case TbGlyph::Capture:
        p.drawRoundedRect(r(4, 7, 16, 12), 2 * m, 2 * m);
        p.drawEllipse(r(9, 10, 6, 6));
        p.drawRect(r(8, 5, 5, 3));
        break;
      case TbGlyph::Solve:
        p.drawLine(QPointF(6 * m, 12 * m), QPointF(10 * m, 17 * m));
        p.drawLine(QPointF(10 * m, 17 * m), QPointF(18 * m, 7 * m));
        break;
      case TbGlyph::Export:
        p.drawLine(QPointF(12 * m, 4 * m), QPointF(12 * m, 14 * m));
        p.drawLine(QPointF(8 * m, 8 * m), QPointF(12 * m, 4 * m));
        p.drawLine(QPointF(16 * m, 8 * m), QPointF(12 * m, 4 * m));
        p.drawLine(QPointF(5 * m, 17 * m), QPointF(19 * m, 17 * m));
        p.drawLine(QPointF(5 * m, 17 * m), QPointF(5 * m, 20 * m));
        p.drawLine(QPointF(19 * m, 17 * m), QPointF(19 * m, 20 * m));
        break;
    }
    icon.addPixmap(pm);
  }
  return icon;
}

/// \brief 为动作绑定图标与提示
inline void bind_toolbar_action(
    QAction *action, TbGlyph glyph, const QString &tip, const QColor &ink,
    const QColor &accent) {
  if (action == nullptr) {
    return;
  }
  action->setIcon(make_toolbar_icon(glyph, ink, accent));
  action->setToolTip(tip);
  action->setStatusTip(tip);
}

/// \brief 安全设置指标标签文本
inline void set_metric_value(QLabel *label, const QString &value) {
  if (label != nullptr) {
    label->setText(value);
  }
}

}  // namespace window_detail
}  // namespace gui
}  // namespace hs_calib
