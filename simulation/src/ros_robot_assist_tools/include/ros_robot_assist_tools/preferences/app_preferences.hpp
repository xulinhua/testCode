#ifndef ROS_ROBOT_ASSIST_TOOLS__PREFERENCES__APP_PREFERENCES_HPP_
#define ROS_ROBOT_ASSIST_TOOLS__PREFERENCES__APP_PREFERENCES_HPP_

#include <string>

class QApplication;

namespace ros_robot_assist_tools
{

struct AppPreferences
{
  std::string ros_domain_id;
  std::string ui_theme{"fusion"};  // fusion | light | dark
};

std::string AppPreferencesFilePath();
bool LoadAppPreferences(AppPreferences * out);
bool SaveAppPreferences(const AppPreferences & prefs);

bool IsValidRosDomainId(const std::string & value);
void ApplyRosDomainInProcess(const std::string & value);
void ApplyRosDomainFromSavedPreferences();
void ApplyUiThemeToApplication(QApplication & app, const std::string & theme);

}  // namespace ros_robot_assist_tools

#endif
