#ifndef ROS_ROBOT_ASSIST_TOOLS__MODULE__IMAGE_VIEWER_MODULE_H_
#define ROS_ROBOT_ASSIST_TOOLS__MODULE__IMAGE_VIEWER_MODULE_H_

#include <vector>

#include <QImage>
#include <QString>

#include <sensor_msgs/msg/image.hpp>

namespace ros_robot_assist_tools::ui
{

std::vector<QString> ListOnlineImageTopicsForViewer();
bool ConvertViewerRosImageToQImage(const sensor_msgs::msg::Image & msg, QImage * out_image);

}  // namespace ros_robot_assist_tools::ui

#endif  // ROS_ROBOT_ASSIST_TOOLS__MODULE__IMAGE_VIEWER_MODULE_H_
