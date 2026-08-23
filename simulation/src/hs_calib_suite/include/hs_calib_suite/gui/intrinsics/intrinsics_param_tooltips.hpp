#pragma once

#include <QString>

class QWidget;

namespace hs_calib {
namespace gui {

/// \brief 根据 Tier4 参数键返回中文悬停说明；无映射时返回空字符串
QString intrinsics_param_tooltip(const QString &param_key);

/// \brief 为标签/输入控件设置中文 tooltip（键名与 YAML 字段一致）
void apply_intrinsics_param_tooltip(const QString &param_key, QWidget *widget);

}  // namespace gui
}  // namespace hs_calib
