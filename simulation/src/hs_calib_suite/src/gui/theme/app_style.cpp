#include "hs_calib_suite/gui/theme/app_style.hpp"

#include <QSettings>

namespace hs_calib {
namespace gui {

namespace {

constexpr char k_settings_org[] = "HS";
constexpr char k_settings_app[] = "HS Calib Suite";
constexpr char k_settings_theme_key[] = "ui/theme";

/// \brief 主题色板（QSS 占位符）
struct ThemePalette {
  QString bg;
  QString bg_elevated;
  QString bg_panel;
  QString bg_panel_header;
  QString bg_input;
  QString bg_hover;
  QString bg_selected;
  QString bg_chrome;
  QString bg_toolbar;
  QString bg_step_rail;
  QString bg_preview;
  QString bg_tile;
  QString bg_tile_hover;
  QString bg_tile_selected;
  QString bg_button;
  QString bg_button_hover;
  QString bg_button_pressed;
  QString bg_button_disabled;
  QString bg_primary_0;
  QString bg_primary_1;
  QString bg_primary_hover;
  QString bg_ghost;
  QString bg_status_online;
  QString bg_status_offline;

  QString border;
  QString border_strong;
  QString border_chrome;
  QString border_focus;
  QString border_primary;
  QString border_tile_hover;

  QString fg;
  QString fg_muted;
  QString fg_title;
  QString fg_section;
  QString fg_brand;
  QString fg_step_idle;
  QString fg_step_active;
  QString fg_step_done;
  QString fg_button;
  QString fg_primary;
  QString fg_ghost;
  QString fg_disabled;
  QString fg_list;
  QString fg_list_selected;
  QString fg_metric;
  QString fg_preview;
  QString fg_chrome;
  QString fg_menu_selected;
  QString fg_status_online;
  QString fg_status_offline;

  QString accent;
  QString selection;
  QString splitter;
  QString icon_ink;
  QString icon_accent;
};

/// \brief 深色主题色板
ThemePalette palette_dark() {
  ThemePalette p;
  p.bg = "#060912";
  p.bg_elevated = "#0a1220";
  p.bg_panel = "#0a1220";
  p.bg_panel_header = "#0d1828";
  p.bg_input = "#070f19";
  p.bg_hover = "#0f1e30";
  p.bg_selected = "#123a52";
  p.bg_chrome = "#07101c";
  p.bg_toolbar = "#0a1526";
  p.bg_step_rail = "#070e18";
  p.bg_preview = "#050910";
  p.bg_tile = "#0c1828";
  p.bg_tile_hover = "#102033";
  p.bg_tile_selected = "#0c2740";
  p.bg_button = "#0f1b2d";
  p.bg_button_hover = "#152a40";
  p.bg_button_pressed = "#0a1624";
  p.bg_button_disabled = "#0b1420";
  p.bg_primary_0 = "#00b4e0";
  p.bg_primary_1 = "#5ce1ff";
  p.bg_primary_hover = "#8aeeff";
  p.bg_ghost = "rgba(20, 48, 73, 0.35)";
  p.bg_status_online = "#34d399";
  p.bg_status_offline = "#143049";

  p.border = "#1a3d5c";
  p.border_strong = "#2a5575";
  p.border_chrome = "#12304a";
  p.border_focus = "#5ce1ff";
  p.border_primary = "#9af0ff";
  p.border_tile_hover = "#3db8e0";

  p.fg = "#e8f2ff";
  p.fg_muted = "#6f879f";
  p.fg_title = "#f4f8ff";
  p.fg_section = "#7dd3fc";
  p.fg_brand = "#5ce1ff";
  p.fg_step_idle = "#4d647c";
  p.fg_step_active = "#041018";
  p.fg_step_done = "#5ce1ff";
  p.fg_button = "#dcefff";
  p.fg_primary = "#031018";
  p.fg_ghost = "#9fdfff";
  p.fg_disabled = "#5a738c";
  p.fg_list = "#cfe0f3";
  p.fg_list_selected = "#b8f3ff";
  p.fg_metric = "#5ce1ff";
  p.fg_preview = "#6a87a3";
  p.fg_chrome = "#d7ebff";
  p.fg_menu_selected = "#9aefff";
  p.fg_status_online = "#041018";
  p.fg_status_offline = "#9fdfff";

  p.accent = "#5ce1ff";
  p.selection = "#0e7490";
  p.splitter = "#15324c";
  p.icon_ink = "#c9e7ff";
  p.icon_accent = "#5ce1ff";
  return p;
}

/// \brief 浅色主题色板
ThemePalette palette_light() {
  // Visual Studio Light 风格：浅灰壳 + 白面板 + 蓝强调
  ThemePalette p;
  p.bg = "#f3f3f3";
  p.bg_elevated = "#ffffff";
  p.bg_panel = "#ffffff";
  p.bg_panel_header = "#f0f0f0";
  p.bg_input = "#ffffff";
  p.bg_hover = "#e8e8e8";
  p.bg_selected = "#cce8ff";
  p.bg_chrome = "#f3f3f3";
  p.bg_toolbar = "#eeeeee";
  p.bg_step_rail = "#eaeaea";
  p.bg_preview = "#e8e8e8";
  p.bg_tile = "#ffffff";
  p.bg_tile_hover = "#f5f9fc";
  p.bg_tile_selected = "#e8f4fc";
  p.bg_button = "#ffffff";
  p.bg_button_hover = "#f0f0f0";
  p.bg_button_pressed = "#e0e0e0";
  p.bg_button_disabled = "#f5f5f5";
  p.bg_primary_0 = "#0078d4";
  p.bg_primary_1 = "#2b88d8";
  p.bg_primary_hover = "#106ebe";
  p.bg_ghost = "rgba(0, 120, 212, 0.08)";
  p.bg_status_online = "#107c10";
  p.bg_status_offline = "#e8e8e8";

  p.border = "#d0d0d0";
  p.border_strong = "#adadad";
  p.border_chrome = "#cccedb";
  p.border_focus = "#0078d4";
  p.border_primary = "#005a9e";
  p.border_tile_hover = "#0078d4";

  p.fg = "#1e1e1e";
  p.fg_muted = "#6b6b6b";
  p.fg_title = "#1e1e1e";
  p.fg_section = "#0078d4";
  p.fg_brand = "#0078d4";
  p.fg_step_idle = "#8a8a8a";
  p.fg_step_active = "#ffffff";
  p.fg_step_done = "#0078d4";
  p.fg_button = "#1e1e1e";
  p.fg_primary = "#ffffff";
  p.fg_ghost = "#0078d4";
  p.fg_disabled = "#a0a0a0";
  p.fg_list = "#1e1e1e";
  p.fg_list_selected = "#003366";
  p.fg_metric = "#0078d4";
  p.fg_preview = "#6b6b6b";
  p.fg_chrome = "#1e1e1e";
  p.fg_menu_selected = "#1e1e1e";
  p.fg_status_online = "#ffffff";
  p.fg_status_offline = "#666666";

  p.accent = "#0078d4";
  p.selection = "#add6ff";
  p.splitter = "#cccedb";
  p.icon_ink = "#424242";
  p.icon_accent = "#0078d4";
  return p;
}

/// \brief 蓝色主题色板
ThemePalette palette_blue() {
  // Visual Studio Blue：蓝灰 chrome + 白工作区
  ThemePalette p;
  p.bg = "#e8ecf2";
  p.bg_elevated = "#ffffff";
  p.bg_panel = "#ffffff";
  p.bg_panel_header = "#d6dde8";
  p.bg_input = "#ffffff";
  p.bg_hover = "#dce6f4";
  p.bg_selected = "#b8d4f0";
  p.bg_chrome = "#293955";
  p.bg_toolbar = "#3b4f6c";
  p.bg_step_rail = "#2f4260";
  p.bg_preview = "#d8dee8";
  p.bg_tile = "#ffffff";
  p.bg_tile_hover = "#eef3f9";
  p.bg_tile_selected = "#d6e8f8";
  p.bg_button = "#ffffff";
  p.bg_button_hover = "#eef2f8";
  p.bg_button_pressed = "#d6dde8";
  p.bg_button_disabled = "#eef0f4";
  p.bg_primary_0 = "#007acc";
  p.bg_primary_1 = "#1c97ea";
  p.bg_primary_hover = "#1177bb";
  p.bg_ghost = "rgba(0, 122, 204, 0.12)";
  p.bg_status_online = "#388a34";
  p.bg_status_offline = "#3b4f6c";

  p.border = "#aeb9c9";
  p.border_strong = "#7a8aa0";
  p.border_chrome = "#1b2940";
  p.border_focus = "#007acc";
  p.border_primary = "#005a9e";
  p.border_tile_hover = "#007acc";

  p.fg = "#1e1e1e";
  p.fg_muted = "#5a6a7e";
  p.fg_title = "#1e1e1e";
  p.fg_section = "#007acc";
  p.fg_brand = "#9fd4ff";
  p.fg_step_idle = "#9aa8bc";
  p.fg_step_active = "#ffffff";
  p.fg_step_done = "#9fd4ff";
  p.fg_button = "#1e1e1e";
  p.fg_primary = "#ffffff";
  p.fg_ghost = "#007acc";
  p.fg_disabled = "#9aa8bc";
  p.fg_list = "#1e1e1e";
  p.fg_list_selected = "#003366";
  p.fg_metric = "#007acc";
  p.fg_preview = "#5a6a7e";
  p.fg_chrome = "#f0f4fa";
  p.fg_menu_selected = "#ffffff";
  p.fg_status_online = "#ffffff";
  p.fg_status_offline = "#c5d4e8";

  p.accent = "#007acc";
  p.selection = "#add6ff";
  p.splitter = "#aeb9c9";
  p.icon_ink = "#e8eef8";
  p.icon_accent = "#9fd4ff";
  return p;
}

/// \brief 按 ThemeId 取色板
ThemePalette palette_for(ThemeId id) {
  switch (id) {
    case ThemeId::Light:
      return palette_light();
    case ThemeId::Blue:
      return palette_blue();
    case ThemeId::Dark:
    default:
      return palette_dark();
  }
}

/// \brief 用色板填充应用级 QSS 模板
QString fill_template(const ThemePalette &p) {
  QString qss = QStringLiteral(R"qss(
* {
  font-family: "Noto Sans CJK SC", "Noto Sans", "DejaVu Sans",
               "Source Han Sans SC", "Segoe UI", sans-serif;
  font-size: 13px;
}

QMainWindow, QDialog {
  background-color: __BG__;
  color: __FG__;
}

QWidget#CentralRoot {
  background-color: __BG__;
}

QLabel#BrandMark {
  color: __FG_BRAND__;
  font-family: "Noto Sans Mono", "DejaVu Sans Mono", monospace;
  font-size: 12px;
  font-weight: 700;
  letter-spacing: 2px;
}

QLabel#BrandSub {
  color: __FG_MUTED__;
  font-size: 12px;
  letter-spacing: 0.3px;
}

QLabel#PageTitle {
  color: __FG_TITLE__;
  font-size: 24px;
  font-weight: 700;
  letter-spacing: -0.3px;
  padding-bottom: 2px;
}

QLabel#PageSubtitle {
  color: __FG_MUTED__;
  font-size: 13px;
  padding-bottom: 8px;
}

QLabel#SectionTitle {
  color: __FG_SECTION__;
  font-size: 11px;
  font-weight: 700;
  letter-spacing: 1.6px;
}

QLabel#Muted {
  color: __FG_MUTED__;
  font-size: 13px;
}

QLabel#CalibTileTitle {
  color: __FG_TITLE__;
  font-size: 14px;
  font-weight: 700;
}

QLabel#CalibTileSub {
  color: __FG_MUTED__;
  font-size: 12px;
  padding-top: 0px;
}

QFrame#CalibTile {
  background-color: __BG_TILE__;
  border: 1px solid __BORDER__;
  border-radius: 8px;
}

QFrame#CalibTile:hover {
  border-color: __BORDER_TILE_HOVER__;
  background-color: __BG_TILE_HOVER__;
}

QFrame#CalibTile[selected="true"] {
  border: 1px solid __ACCENT__;
  background-color: __BG_TILE_SELECTED__;
}

QFrame#CalibTile[planned="true"] {
  opacity: 0.55;
  border: 1px dashed __BORDER__;
  background-color: __BG_BUTTON_DISABLED__;
}

QFrame#CalibTile[planned="true"]:hover {
  border-color: __BORDER__;
  background-color: __BG_BUTTON_DISABLED__;
}

QFrame#CalibTileAccent {
  background-color: __BORDER_STRONG__;
  border: none;
  border-radius: 2px;
}

QFrame#CalibTile[selected="true"] QFrame#CalibTileAccent {
  background-color: __ACCENT__;
}

QFrame#CalibTile[planned="true"] QFrame#CalibTileAccent {
  background-color: __FG_DISABLED__;
}

QLabel#CalibTilePre {
  color: __FG_MUTED__;
  font-size: 11px;
}

QPushButton#CategoryChip {
  background-color: __BG_BUTTON__;
  color: __FG_MUTED__;
  border: 1px solid __BORDER__;
  border-radius: 16px;
  padding: 8px 16px;
  min-height: 20px;
  font-size: 13px;
  font-weight: 650;
}

QPushButton#CategoryChip:checked {
  background-color: __BG_SELECTED__;
  color: __FG_LIST_SELECTED__;
  border: 1px solid __ACCENT__;
}

QPushButton#CategoryChip:hover {
  border-color: __BORDER_STRONG__;
  color: __FG_TITLE__;
}

QWidget#HomeTaskHost {
  background-color: transparent;
}

QScrollArea#HomeTaskScroll,
QScrollArea#HomeTaskScroll > QWidget,
QAbstractScrollArea {
  background-color: transparent;
  border: none;
}

QFrame#Panel {
  background-color: __BG_PANEL__;
  border: 1px solid __BORDER__;
  border-radius: 12px;
}

QFrame#PanelHeader {
  background-color: __BG_PANEL_HEADER__;
  border-bottom: 1px solid __BORDER__;
  border-top-left-radius: 12px;
  border-top-right-radius: 12px;
  min-height: 44px;
}

QFrame#StepRail {
  background-color: __BG_STEP_RAIL__;
  border-bottom: 1px solid __BORDER_CHROME__;
  min-height: 54px;
  max-height: 54px;
}

QLabel#StepIdle {
  color: __FG_STEP_IDLE__;
  padding: 7px 14px;
  font-family: "Noto Sans Mono", "DejaVu Sans Mono", monospace;
  font-size: 12px;
  font-weight: 600;
}

QLabel#StepActive {
  color: __FG_STEP_ACTIVE__;
  background-color: __ACCENT__;
  border-radius: 14px;
  padding: 7px 16px;
  font-family: "Noto Sans Mono", "DejaVu Sans Mono", monospace;
  font-size: 12px;
  font-weight: 700;
}

QLabel#StepDone {
  color: __FG_STEP_DONE__;
  padding: 7px 14px;
  font-family: "Noto Sans Mono", "DejaVu Sans Mono", monospace;
  font-size: 12px;
  font-weight: 700;
}

QPushButton {
  background-color: __BG_BUTTON__;
  color: __FG_BUTTON__;
  border: 1px solid __BORDER_STRONG__;
  border-radius: 9px;
  padding: 11px 20px;
  min-height: 28px;
  font-size: 13px;
  font-weight: 650;
}

QPushButton:hover {
  background-color: __BG_BUTTON_HOVER__;
  border-color: __ACCENT__;
  color: __FG_TITLE__;
}

QPushButton:pressed {
  background-color: __BG_BUTTON_PRESSED__;
}

QPushButton:disabled {
  color: __FG_DISABLED__;
  border-color: __BORDER__;
  background-color: __BG_BUTTON_DISABLED__;
}

QPushButton#PrimaryButton {
  background-color: qlineargradient(
    x1:0, y1:0, x2:1, y2:1,
    stop:0 __BG_PRIMARY_0__, stop:1 __BG_PRIMARY_1__);
  color: __FG_PRIMARY__;
  border: 1px solid __BORDER_PRIMARY__;
  font-size: 14px;
  font-weight: 700;
  padding: 12px 22px;
  min-height: 30px;
}

QPushButton#PrimaryButton:hover {
  background-color: __BG_PRIMARY_HOVER__;
}

QPushButton#PrimaryButton:disabled {
  background-color: __BG_BUTTON_DISABLED__;
  color: __FG_DISABLED__;
  border-color: __BORDER__;
}

QPushButton#GhostButton {
  background-color: __BG_GHOST__;
  border: 1px solid __BORDER_STRONG__;
  color: __FG_GHOST__;
  font-weight: 650;
  min-height: 28px;
}

QPushButton#GhostButton:hover {
  border-color: __ACCENT__;
  color: __FG_TITLE__;
  background-color: __BG_TILE_HOVER__;
}

QPushButton#ModeChip {
  min-height: 22px;
  padding: 3px 12px;
  font-size: 12px;
  font-weight: 650;
  border-radius: 8px;
  background-color: __BG_GHOST__;
  border: 1px solid __BORDER_STRONG__;
  color: __FG_GHOST__;
}

QPushButton#ModeChip:hover {
  border-color: __ACCENT__;
  color: __FG_TITLE__;
}

QPushButton#ModeChip:checked {
  background-color: qlineargradient(
    x1:0, y1:0, x2:1, y2:1,
    stop:0 __BG_PRIMARY_0__, stop:1 __BG_PRIMARY_1__);
  color: __FG_PRIMARY__;
  border: 1px solid __BORDER_PRIMARY__;
}

QPushButton#ModeChip:disabled {
  color: __FG_DISABLED__;
  border-color: __BORDER__;
  background-color: __BG_BUTTON_DISABLED__;
}

QListWidget {
  background-color: __BG_INPUT__;
  border: 1px solid __BORDER__;
  border-radius: 10px;
  outline: none;
  padding: 8px;
  font-size: 13px;
}

QListWidget::item {
  padding: 14px 16px;
  border-radius: 8px;
  margin: 3px;
  color: __FG_LIST__;
  min-height: 22px;
}

QListWidget::item:selected {
  background-color: __BG_SELECTED__;
  color: __FG_LIST_SELECTED__;
  border: 1px solid __BORDER_FOCUS__;
}

QListWidget::item:hover {
  background-color: __BG_HOVER__;
}

/* Compact readiness checklist (Tier4-style: status strip, not a half page) */
QListWidget#ReadyCheckList {
  background-color: transparent;
  border: none;
  padding: 0px;
  font-size: 12px;
}

QListWidget#ReadyCheckList::item {
  padding: 4px 8px;
  margin: 1px 0px;
  min-height: 18px;
  border-radius: 4px;
  color: __FG_LIST__;
}

QListWidget#ReadyCheckList::item:hover {
  background-color: __BG_HOVER__;
}

QListWidget#ReadyCheckList::item:selected {
  background-color: transparent;
  color: __FG_LIST__;
  border: none;
}

QFrame#ReadyStrip {
  background-color: __BG_PANEL__;
  border: 1px solid __BORDER__;
  border-radius: 10px;
  max-height: 168px;
}

/* Tier4 Launcher configuration groups — dense form sections */
QGroupBox#LauncherGroup {
  background-color: __BG_PANEL__;
  border: 1px solid __BORDER__;
  border-radius: 8px;
  margin-top: 12px;
  padding-top: 8px;
  font-size: 12px;
  font-weight: 600;
  color: __FG__;
}

QGroupBox#LauncherGroup::title {
  subcontrol-origin: margin;
  left: 12px;
  padding: 0 6px;
  color: __FG_MUTED__;
  font-size: 11px;
  font-weight: 600;
}

QGroupBox#LauncherGroup QLabel {
  font-size: 12px;
  font-weight: 400;
  color: __FG_MUTED__;
}

QGroupBox#LauncherGroup QCheckBox {
  font-size: 12px;
  font-weight: 400;
}

QTextEdit, QPlainTextEdit, QLineEdit {
  background-color: __BG_INPUT__;
  color: __FG__;
  border: 1px solid __BORDER__;
  border-radius: 10px;
  selection-background-color: __SELECTION__;
  padding: 12px 14px;
  font-family: "Noto Sans Mono", "DejaVu Sans Mono", monospace;
  font-size: 13px;
}

QTextEdit:focus, QPlainTextEdit:focus, QLineEdit:focus {
  border: 1px solid __BORDER_FOCUS__;
}

QComboBox, QSpinBox, QDoubleSpinBox, QAbstractSpinBox {
  background-color: __BG_INPUT__;
  color: __FG__;
  border: 1px solid __BORDER__;
  border-radius: 10px;
  padding: 8px 12px;
  min-height: 28px;
  selection-background-color: __SELECTION__;
  selection-color: __FG__;
  font-family: "Noto Sans Mono", "DejaVu Sans Mono", monospace;
  font-size: 13px;
}

QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover, QAbstractSpinBox:hover {
  border-color: __BORDER_STRONG__;
  background-color: __BG_HOVER__;
}

QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QAbstractSpinBox:focus {
  border: 1px solid __BORDER_FOCUS__;
}

QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QAbstractSpinBox:disabled {
  color: __FG_DISABLED__;
  background-color: __BG_BUTTON_DISABLED__;
  border-color: __BORDER__;
}

QComboBox::drop-down {
  subcontrol-origin: padding;
  subcontrol-position: top right;
  width: 28px;
  border: none;
  border-left: 1px solid __BORDER__;
  border-top-right-radius: 10px;
  border-bottom-right-radius: 10px;
  background-color: transparent;
}

QComboBox::down-arrow {
  width: 0px;
  height: 0px;
  border-left: 5px solid transparent;
  border-right: 5px solid transparent;
  border-top: 6px solid __FG_MUTED__;
  margin-right: 8px;
}

QComboBox QAbstractItemView {
  background-color: __BG_PANEL__;
  color: __FG__;
  border: 1px solid __BORDER_STRONG__;
  border-radius: 8px;
  outline: none;
  padding: 4px;
  selection-background-color: __BG_SELECTED__;
  selection-color: __FG_LIST_SELECTED__;
}

QComboBox QAbstractItemView::item {
  min-height: 28px;
  padding: 6px 12px;
  border-radius: 6px;
  color: __FG_LIST__;
}

QComboBox QAbstractItemView::item:selected {
  background-color: __BG_SELECTED__;
  color: __FG_LIST_SELECTED__;
}

QComboBox QAbstractItemView::item:hover {
  background-color: __BG_HOVER__;
}

QSpinBox::up-button, QDoubleSpinBox::up-button,
QAbstractSpinBox::up-button {
  subcontrol-origin: border;
  subcontrol-position: top right;
  width: 22px;
  border: none;
  border-left: 1px solid __BORDER__;
  background-color: __BG_BUTTON__;
  border-top-right-radius: 9px;
}

QSpinBox::down-button, QDoubleSpinBox::down-button,
QAbstractSpinBox::down-button {
  subcontrol-origin: border;
  subcontrol-position: bottom right;
  width: 22px;
  border: none;
  border-left: 1px solid __BORDER__;
  background-color: __BG_BUTTON__;
  border-bottom-right-radius: 9px;
}

QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QAbstractSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover,
QAbstractSpinBox::down-button:hover {
  background-color: __BG_BUTTON_HOVER__;
}

QSpinBox::up-arrow, QDoubleSpinBox::up-arrow, QAbstractSpinBox::up-arrow {
  width: 0px;
  height: 0px;
  border-left: 4px solid transparent;
  border-right: 4px solid transparent;
  border-bottom: 5px solid __FG_MUTED__;
}

QSpinBox::down-arrow, QDoubleSpinBox::down-arrow, QAbstractSpinBox::down-arrow {
  width: 0px;
  height: 0px;
  border-left: 4px solid transparent;
  border-right: 4px solid transparent;
  border-top: 5px solid __FG_MUTED__;
}

QSplitter::handle {
  background-color: __SPLITTER__;
  width: 2px;
  height: 2px;
}

QLabel#PreviewCanvas,
QWidget#ImageViewWidget {
  background-color: __BG_PREVIEW__;
  border: 1px solid __BORDER__;
  border-radius: 12px;
  color: __FG_PREVIEW__;
  font-size: 15px;
  font-weight: 500;
}

QWidget#ImageViewToolbar {
  background-color: __BG_ELEVATED__;
  border: 1px solid __BORDER__;
  border-radius: 8px;
}

QWidget#ImageViewToolbar QToolButton {
  color: __FG__;
  background: transparent;
  border: none;
  font-weight: 700;
}

QWidget#ImageViewToolbar QToolButton:hover {
  color: __FG_TITLE__;
}

QLabel#MetricValue {
  color: __FG_METRIC__;
  font-family: "Noto Sans Mono", "DejaVu Sans Mono", monospace;
  font-size: 26px;
  font-weight: 700;
  letter-spacing: -0.4px;
}

QLabel#MetricName {
  color: __FG_MUTED__;
  font-size: 11px;
  font-weight: 700;
  letter-spacing: 1px;
}

QFrame#MetricCard {
  background-color: __BG_ELEVATED__;
  border: 1px solid __BORDER__;
  border-radius: 12px;
  min-height: 88px;
}

/* Compact metrics on workbench side rail */
QFrame#CompactMetricCard {
  background-color: __BG_ELEVATED__;
  border: 1px solid __BORDER__;
  border-radius: 8px;
  min-height: 56px;
  max-height: 72px;
}

QFrame#CompactMetricCard QLabel#MetricValue {
  font-size: 20px;
}

QFrame#CompactMetricCard QLabel#MetricName {
  font-size: 10px;
  letter-spacing: 0.6px;
}

QFrame#VizStrip {
  background-color: __BG_PANEL__;
  border: 1px solid __BORDER__;
  border-radius: 8px;
}

QScrollArea {
  border: none;
  background: transparent;
}

QMenuBar {
  background-color: __BG_CHROME__;
  color: __FG_CHROME__;
  border-bottom: 1px solid __BORDER_CHROME__;
  padding: 4px 8px;
  font-size: 13px;
  font-weight: 600;
  min-height: 34px;
}

QMenuBar::item {
  background: transparent;
  padding: 8px 16px;
  border-radius: 6px;
}

QMenuBar::item:selected {
  background-color: __BG_SELECTED__;
  color: __FG_MENU_SELECTED__;
}

QMenu {
  background-color: __BG_PANEL__;
  color: __FG__;
  border: 1px solid __BORDER__;
  padding: 8px;
  font-size: 13px;
}

QMenu::item {
  padding: 10px 36px 10px 18px;
  border-radius: 6px;
  min-height: 22px;
}

QMenu::item:selected {
  background-color: __BG_SELECTED__;
  color: __FG_MENU_SELECTED__;
}

QMenu::separator {
  height: 1px;
  background: __SPLITTER__;
  margin: 6px 10px;
}

QToolBar {
  background-color: __BG_TOOLBAR__;
  border-bottom: 1px solid __BORDER_CHROME__;
  spacing: 2px;
  padding: 4px 8px;
  min-height: 36px;
  max-height: 40px;
}

QToolBar QToolButton {
  background-color: transparent;
  color: __FG_CHROME__;
  border: 1px solid transparent;
  border-radius: 4px;
  padding: 4px;
  margin: 1px;
  min-width: 28px;
  min-height: 28px;
  max-width: 32px;
  max-height: 32px;
}

QToolBar QToolButton:hover {
  background-color: __BG_HOVER__;
  border-color: __BORDER_STRONG__;
}

QToolBar QToolButton:checked,
QToolBar QToolButton:pressed {
  background-color: __BG_SELECTED__;
  border-color: __ACCENT__;
}

QToolBar QToolButton:disabled {
  opacity: 0.35;
}

QToolBar::separator {
  background: __SPLITTER__;
  width: 1px;
  margin: 6px 6px;
}

QStatusBar {
  background-color: __BG_CHROME__;
  color: __FG_MUTED__;
  border-top: 1px solid __BORDER_CHROME__;
  min-height: 32px;
  font-size: 12px;
}

QStatusBar QLabel {
  color: __FG_MUTED__;
  padding: 0 12px;
}

QStatusBar::item {
  border: none;
}

QLabel#StatusBarModeOnline {
  color: __FG_STATUS_ONLINE__;
  background-color: __BG_STATUS_ONLINE__;
  border-radius: 10px;
  padding: 4px 12px;
  font-size: 11px;
  font-weight: 700;
}

QLabel#StatusBarModeOffline {
  color: __FG_STATUS_OFFLINE__;
  background-color: __BG_STATUS_OFFLINE__;
  border: 1px solid __BORDER_STRONG__;
  border-radius: 10px;
  padding: 4px 12px;
  font-size: 11px;
  font-weight: 700;
}

QLabel#StatusBarPage {
  color: __ACCENT__;
  font-family: "Noto Sans Mono", "DejaVu Sans Mono", monospace;
  font-size: 11px;
  font-weight: 700;
  letter-spacing: 0.8px;
}
)qss");

  // —— 替换色板占位符 ——
  const auto put = [&](const char *key, const QString &val) {
    qss.replace(QLatin1String(key), val);
  };
  put("__BG__", p.bg);
  put("__BG_ELEVATED__", p.bg_elevated);
  put("__BG_PANEL__", p.bg_panel);
  put("__BG_PANEL_HEADER__", p.bg_panel_header);
  put("__BG_INPUT__", p.bg_input);
  put("__BG_HOVER__", p.bg_hover);
  put("__BG_SELECTED__", p.bg_selected);
  put("__BG_CHROME__", p.bg_chrome);
  put("__BG_TOOLBAR__", p.bg_toolbar);
  put("__BG_STEP_RAIL__", p.bg_step_rail);
  put("__BG_PREVIEW__", p.bg_preview);
  put("__BG_TILE__", p.bg_tile);
  put("__BG_TILE_HOVER__", p.bg_tile_hover);
  put("__BG_TILE_SELECTED__", p.bg_tile_selected);
  put("__BG_BUTTON__", p.bg_button);
  put("__BG_BUTTON_HOVER__", p.bg_button_hover);
  put("__BG_BUTTON_PRESSED__", p.bg_button_pressed);
  put("__BG_BUTTON_DISABLED__", p.bg_button_disabled);
  put("__BG_PRIMARY_0__", p.bg_primary_0);
  put("__BG_PRIMARY_1__", p.bg_primary_1);
  put("__BG_PRIMARY_HOVER__", p.bg_primary_hover);
  put("__BG_GHOST__", p.bg_ghost);
  put("__BG_STATUS_ONLINE__", p.bg_status_online);
  put("__BG_STATUS_OFFLINE__", p.bg_status_offline);
  put("__BORDER__", p.border);
  put("__BORDER_STRONG__", p.border_strong);
  put("__BORDER_CHROME__", p.border_chrome);
  put("__BORDER_FOCUS__", p.border_focus);
  put("__BORDER_PRIMARY__", p.border_primary);
  put("__BORDER_TILE_HOVER__", p.border_tile_hover);
  put("__FG__", p.fg);
  put("__FG_MUTED__", p.fg_muted);
  put("__FG_TITLE__", p.fg_title);
  put("__FG_SECTION__", p.fg_section);
  put("__FG_BRAND__", p.fg_brand);
  put("__FG_STEP_IDLE__", p.fg_step_idle);
  put("__FG_STEP_ACTIVE__", p.fg_step_active);
  put("__FG_STEP_DONE__", p.fg_step_done);
  put("__FG_BUTTON__", p.fg_button);
  put("__FG_PRIMARY__", p.fg_primary);
  put("__FG_GHOST__", p.fg_ghost);
  put("__FG_DISABLED__", p.fg_disabled);
  put("__FG_LIST__", p.fg_list);
  put("__FG_LIST_SELECTED__", p.fg_list_selected);
  put("__FG_METRIC__", p.fg_metric);
  put("__FG_PREVIEW__", p.fg_preview);
  put("__FG_CHROME__", p.fg_chrome);
  put("__FG_MENU_SELECTED__", p.fg_menu_selected);
  put("__FG_STATUS_ONLINE__", p.fg_status_online);
  put("__FG_STATUS_OFFLINE__", p.fg_status_offline);
  put("__ACCENT__", p.accent);
  put("__SELECTION__", p.selection);
  put("__SPLITTER__", p.splitter);
  return qss;
}

}  // namespace

/// \brief 主题显示名
QString theme_display_name(ThemeId id) {
  switch (id) {
    case ThemeId::Light:
      return QStringLiteral("浅色");
    case ThemeId::Blue:
      return QStringLiteral("蓝色");
    case ThemeId::Dark:
    default:
      return QStringLiteral("深色");
  }
}

/// \brief 从 QSettings 读取主题
ThemeId load_theme_id() {
  QSettings settings(QString::fromUtf8(k_settings_org), QString::fromUtf8(k_settings_app));
  const int v = settings.value(QString::fromUtf8(k_settings_theme_key),
                               static_cast<int>(ThemeId::Dark))
                    .toInt();
  if (v == static_cast<int>(ThemeId::Light)) {
    return ThemeId::Light;
  }
  if (v == static_cast<int>(ThemeId::Blue)) {
    return ThemeId::Blue;
  }
  return ThemeId::Dark;
}

/// \brief 将主题写入 QSettings
void save_theme_id(ThemeId id) {
  QSettings settings(QString::fromUtf8(k_settings_org), QString::fromUtf8(k_settings_app));
  settings.setValue(QString::fromUtf8(k_settings_theme_key), static_cast<int>(id));
}

/// \brief 工具栏图标主色
QString theme_icon_ink(ThemeId id) {
  return palette_for(id).icon_ink;
}

/// \brief 工具栏图标强调色
QString theme_icon_accent(ThemeId id) {
  return palette_for(id).icon_accent;
}

/// \brief 按主题生成应用 QSS
QString application_style_sheet(ThemeId id) {
  return fill_template(palette_for(id));
}

}  // namespace gui
}  // namespace hs_calib
