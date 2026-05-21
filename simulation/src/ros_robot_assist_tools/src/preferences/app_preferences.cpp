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

std::string NormalizeTriState01(const std::string & value)
{
  if (value == "1" || value == "0") {
    return value;
  }
  return "";
}

std::string NormalizeRmwImplementation(const std::string & value)
{
  if (value == "rmw_fastrtps_cpp" || value == "rmw_cyclonedds_cpp") {
    return value;
  }
  return "";
}

std::string NormalizeLogLevel(const std::string & value)
{
  if (
    value == "DEBUG" || value == "INFO" || value == "WARN" || value == "ERROR" ||
    value == "FATAL")
  {
    return value;
  }
  return "INFO";
}

void SetEnvInProcess(const char * key, const std::string & value)
{
#if defined(_WIN32)
  _putenv_s(key, value.c_str());
#else
  if (value.empty()) {
    ::unsetenv(key);
  } else {
    ::setenv(key, value.c_str(), 1);
  }
#endif
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
  out->ros_localhost_only.clear();
  out->rmw_implementation.clear();
  out->ros_namespace.clear();
  out->use_sim_time_default = true;
  out->log_level_default = "INFO";
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
    if (root["ros_localhost_only"]) {
      out->ros_localhost_only = NormalizeTriState01(root["ros_localhost_only"].as<std::string>(""));
    }
    if (root["rmw_implementation"]) {
      out->rmw_implementation =
        NormalizeRmwImplementation(root["rmw_implementation"].as<std::string>(""));
    }
    if (root["ros_namespace"]) {
      out->ros_namespace = root["ros_namespace"].as<std::string>("");
    }
    if (root["use_sim_time_default"]) {
      out->use_sim_time_default = root["use_sim_time_default"].as<bool>(true);
    }
    if (root["log_level_default"]) {
      out->log_level_default = NormalizeLogLevel(root["log_level_default"].as<std::string>("INFO"));
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
    root["ros_localhost_only"] = NormalizeTriState01(prefs.ros_localhost_only);
    root["rmw_implementation"] = NormalizeRmwImplementation(prefs.rmw_implementation);
    root["ros_namespace"] = prefs.ros_namespace;
    root["use_sim_time_default"] = prefs.use_sim_time_default;
    root["log_level_default"] = NormalizeLogLevel(prefs.log_level_default);
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
  SetEnvInProcess("ROS_DOMAIN_ID", value);
#else
  SetEnvInProcess("ROS_DOMAIN_ID", value);
#endif
}

void ApplyRosEnvironmentInProcess(const AppPreferences & prefs)
{
  if (IsValidRosDomainId(prefs.ros_domain_id)) {
    SetEnvInProcess("ROS_DOMAIN_ID", prefs.ros_domain_id);
  }
  SetEnvInProcess("ROS_LOCALHOST_ONLY", NormalizeTriState01(prefs.ros_localhost_only));
  SetEnvInProcess("RMW_IMPLEMENTATION", NormalizeRmwImplementation(prefs.rmw_implementation));
  SetEnvInProcess("ROS_NAMESPACE", prefs.ros_namespace);
  SetEnvInProcess("RCUTILS_LOGGING_SEVERITY_THRESHOLD", NormalizeLogLevel(prefs.log_level_default));
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
  ApplyRosEnvironmentInProcess(prefs);
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
  if (theme == "light") {
    QPalette p;
    p.setColor(QPalette::Window, QColor(245, 245, 246));
    p.setColor(QPalette::WindowText, QColor(33, 33, 38));
    p.setColor(QPalette::Base, Qt::white);
    p.setColor(QPalette::AlternateBase, QColor(238, 240, 244));
    p.setColor(QPalette::ToolTipBase, QColor(253, 253, 255));
    p.setColor(QPalette::ToolTipText, QColor(33, 33, 38));
    p.setColor(QPalette::Text, QColor(33, 33, 38));
    p.setColor(QPalette::Button, QColor(228, 230, 235));
    p.setColor(QPalette::ButtonText, QColor(33, 33, 38));
    p.setColor(QPalette::BrightText, QColor(218, 30, 40));
    p.setColor(QPalette::Link, QColor(29, 90, 212));
    p.setColor(QPalette::Highlight, QColor(74, 131, 255));
    p.setColor(QPalette::HighlightedText, Qt::white);
    app.setPalette(p);
    app.setStyleSheet(
      "QToolTip { color:#212126; background:#fdfdff; border:1px solid #d0d4dd; }");
    return;
  }
  // fusion: Qt Fusion 默认 / 跟随系统质感的灰调
  app.setPalette(app.style()->standardPalette());
  app.setStyleSheet({});
}

}  // namespace ros_robot_assist_tools
