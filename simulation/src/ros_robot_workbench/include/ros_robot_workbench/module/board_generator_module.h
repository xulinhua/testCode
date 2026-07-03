#ifndef ROS_ROBOT_WORKBENCH__MODULE__BOARD_GENERATOR_MODULE_H_
#define ROS_ROBOT_WORKBENCH__MODULE__BOARD_GENERATOR_MODULE_H_

#include <QString>
#include <opencv2/core/mat.hpp>

#include "ros_robot_workbench/ui/ui_data_structs.h"

namespace ros_robot_workbench::ui
{

bool GenerateCalibrationBoard(const BoardGeneratorParams & params, cv::Mat * out_image, QString * err_msg);
bool ExportCalibrationBoardImage(const cv::Mat & image, const QString & path, QString * err_msg);
bool ExportCalibrationBoardDae(const cv::Mat & image, const QString & dae_path, QString * err_msg);

}  // namespace ros_robot_workbench::ui

#endif  // ROS_ROBOT_WORKBENCH__MODULE__BOARD_GENERATOR_MODULE_H_
