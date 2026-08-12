#pragma once

#include <QString>

namespace hs_calib {
namespace gui {

/// \brief 界面主题（对齐 Visual Studio：深色 / 浅色 / 蓝色）
enum class ThemeId {
  Dark = 0,   ///< 深色
  Light = 1,  ///< 浅色
  Blue = 2,   ///< 蓝色
};

/// \brief 主题显示名
QString theme_display_name(ThemeId id);

/// \brief 从 QSettings 读取主题（缺省 Dark）
ThemeId load_theme_id();

/// \brief 将主题写入 QSettings
void save_theme_id(ThemeId id);

/// \brief 工具栏图标主色（随主题变化）
QString theme_icon_ink(ThemeId id);

/// \brief 工具栏图标强调色
QString theme_icon_accent(ThemeId id);

/// \brief 按主题生成应用级 QSS
QString application_style_sheet(ThemeId id);

}  // namespace gui
}  // namespace hs_calib
