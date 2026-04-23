#ifndef ROS_ROBOT_ASSIST_TOOLS__UI__SHARED_REFRESH_POOL_H_
#define ROS_ROBOT_ASSIST_TOOLS__UI__SHARED_REFRESH_POOL_H_

#include <functional>

namespace ros_robot_assist_tools::ui
{

void RunOnSharedRefreshPool(std::function<void()> task);
void RunOnSharedImageRefreshPool(std::function<void()> task);

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__UI__SHARED_REFRESH_POOL_H_
