#include "ros_robot_assist_tools/preferences/app_preferences.hpp"

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>

#include <QApplication>
#include <QColor>
#include <QPalette>
#include <QStyle>

#include <yaml-cpp/yaml.h>

namespace ros_robot_assist_tools
{
namespace
{

std::string NormalizeTheme(const std::string & theme)
{
  if (theme == "light" || theme == "dark" || theme == "fusion") {
    return theme;
  }
  return "fusion";
}

}  // namespace

std::string AppPreferencesFilePath()
{
  const char * xdg = std::getenv("XDG_CONFIG_HOME");
  std::filesystem::path base;
  if (xdg != nullptr && xdg[0] != '\0') {
    base = std::filesystem::path(xdg) / "ros_robot_assist_tools";
  } else {
    const char * home = std::getenv("HOME");
    if (home == nullptr || home[0] == '\0') {
      return {};
    }
    base = std::filesystem::path(home) / ".config" / "ros_robot_assist_tools";
  }
  return (base / "preferences.yaml").string();
}

bool IsValidRosDomainId(const std::string & value)
{
  if (value.empty()) {
    return true;
  }
  for (char c : value) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      return false;
    }
  }
  try {
    const unsigned long v = std::stoul(value);
    return v <= 232UL;
  } catch (...) {
    return false;
  }
}

bool LoadAppPreferences(AppPreferences * out)
{
  if (out == nullptr) {
    return false;
  }
  out->ros_domain_id.clear();
  out->ui_theme = "fusion";

  const std::string path = AppPreferencesFilePath();
  if (path.empty() || !std::filesystem::exists(path)) {
    return true;
  }
  try {
    YAML::Node root = YAML::LoadFile(path);
    if (root["ros_domain_id"]) {
      out->ros_domain_id = root["ros_domain_id"].as<std::string>("");
    }
    if (root["ui_theme"]) {
      out->ui_theme = NormalizeTheme(root["ui_theme"].as<std::string>("fusion"));
    }
    if (!IsValidRosDomainId(out->ros_domain_id)) {
      out->ros_domain_id.clear();
    }
    return true;
  } catch (...) {
    return false;
  }
}

bool SaveAppPreferences(const AppPreferences & prefs)
{
  const std::string path = AppPreferencesFilePath();
  if (path.empty()) {
    return false;
  }
  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
  if (ec) {
    return false;
  }
  try {
    YAML::Node root;
    root["ros_domain_id"] = prefs.ros_domain_id;
    root["ui_theme"] = NormalizeTheme(prefs.ui_theme);
    YAML::Emitter emitter;
    emitter << root;
    std::ofstream fout(path);
    if (!fout.is_open()) {
      return false;
    }
    fout << emitter.c_str();
    return true;
  } catch (...) {
    return false;
  }
}

void ApplyRosDomainInProcess(const std::string & value)
{
  if (!IsValidRosDomainId(value)) {
    return;
  }
#if defined(_WIN32)
  if (value.empty()) {
    _putenv_s("ROS_DOMAIN_ID", "");
  } else {
    _putenv_s("ROS_DOMAIN_ID", value.c_str());
  }
#else
  if (value.empty()) {
    ::unsetenv("ROS_DOMAIN_ID");
  } else {
    ::setenv("ROS_DOMAIN_ID", value.c_str(), 1);
  }
#endif
}

void ApplyRosDomainFromSavedPreferences()
{
  const std::string path = AppPreferencesFilePath();
  if (path.empty() || !std::filesystem::exists(path)) {
    return;
  }
  AppPreferences prefs;
  if (!LoadAppPreferences(&prefs)) {
    return;
  }
  if (!prefs.ros_domain_id.empty()) {
    ApplyRosDomainInProcess(prefs.ros_domain_id);
  }
}

void ApplyUiThemeToApplication(QApplication & app, const std::string & theme_raw)
{
  const std::string theme = NormalizeTheme(theme_raw);
  app.setStyle(QStringLiteral("Fusion"));
  if (theme == "dark") {
    QPalette p;
    p.setColor(QPalette::Window, QColor(53, 53, 53));
    p.setColor(QPalette::WindowText, Qt::white);
    p.setColor(QPalette::Base, QColor(35, 35, 35));
    p.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    p.setColor(QPalette::ToolTipBase, QColor(25, 25, 25));
    p.setColor(QPalette::ToolTipText, Qt::white);
    p.setColor(QPalette::Text, Qt::white);
    p.setColor(QPalette::Button, QColor(53, 53, 53));
    p.setColor(QPalette::ButtonText, Qt::white);
    p.setColor(QPalette::BrightText, Qt::red);
    p.setColor(QPalette::Link, QColor(42, 130, 218));
    p.setColor(QPalette::Highlight, QColor(42, 130, 218));
    p.setColor(QPalette::HighlightedText, Qt::black);
    app.setPalette(p);
    app.setStyleSheet("QToolTip { color:#fff; background:#2a2a2a; border:1px solid #3d3d3d; }");
    return;
  }
  app.setPalette(app.style()->standardPalette());
  app.setStyleSheet({});
}

}  // namespace ros_robot_assist_tools
