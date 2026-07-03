#ifndef ROS_ROBOT_WORKBENCH__UI__SHARED_REFRESH_POOL_H_
#define ROS_ROBOT_WORKBENCH__UI__SHARED_REFRESH_POOL_H_

#include <functional>

namespace ros_robot_workbench::ui
{

void RunOnSharedRefreshPool(std::function<void()> task);
void RunOnSharedImageRefreshPool(std::function<void()> task);

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__UI__SHARED_REFRESH_POOL_H_
