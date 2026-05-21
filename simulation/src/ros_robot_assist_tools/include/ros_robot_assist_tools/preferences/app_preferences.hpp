#ifndef ROS_ROBOT_ASSIST_TOOLS__PREFERENCES__APP_PREFERENCES_HPP_
#define ROS_ROBOT_ASSIST_TOOLS__PREFERENCES__APP_PREFERENCES_HPP_

#include <string>

class QApplication;

namespace ros_robot_assist_tools
{

struct AppPreferences
{
  std::string ros_domain_id;
  std::string ros_localhost_only;       // "" | "1" | "0"
  std::string rmw_implementation;       // "" | rmw_fastrtps_cpp | rmw_cyclonedds_cpp
  std::string ros_namespace;
  bool use_sim_time_default{true};
  std::string log_level_default{"INFO"};  // DEBUG | INFO | WARN | ERROR | FATAL
  std::string ui_theme{"fusion"};  // fusion（灰调） | light（高亮浅色） | dark
};

std::string AppPreferencesFilePath();
bool LoadAppPreferences(AppPreferences * out);
bool SaveAppPreferences(const AppPreferences & prefs);

bool IsValidRosDomainId(const std::string & value);
void ApplyRosDomainInProcess(const std::string & value);
void ApplyRosEnvironmentInProcess(const AppPreferences & prefs);
void ApplyRosDomainFromSavedPreferences();
void ApplyUiThemeToApplication(QApplication & app, const std::string & theme);

}  // namespace ros_robot_assist_tools

#endif
