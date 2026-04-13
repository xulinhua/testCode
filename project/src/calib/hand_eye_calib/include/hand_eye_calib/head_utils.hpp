#ifndef HAND_EYE_CALIB__HEAD_UTILS_HPP_
#define HAND_EYE_CALIB__HEAD_UTILS_HPP_

#include "calib_utils.hpp"

// 命名空间定义
namespace handeyecalib 
{

// 计算翻滚变换矩阵
cv::Mat RollAngleToTransform(double roll_angle, double tx = 0.0, double ty = 0.0, double tz = 0.0);    
// 计算俯仰变换矩阵
cv::Mat PitchAngleToTransform(double pitch_angle, double tx = 0.0, double ty = 0.0, double tz = 0.0);
// 计算偏航变换矩阵
cv::Mat YawAngleToTransform(double yaw_angle, double tx = 0.0, double ty = 0.0, double tz = 0.0);

/**
 * @brief 计算头部姿态变换矩阵
 * @param pitch_angle 俯仰角（度）
 * @param yaw_angle 偏航角（度）
 * @param tx 平移距离（X轴）
 * @param ty 平移距离（Y轴）
 * @param tz 平移距离（Z轴）
 * @return 4x4齐次变换矩阵
 */
cv::Mat computeHeadPoseTransform(double pitch_angle, double yaw_angle, double tx, double ty, double tz);

// 计算偏航变换矩阵
cv::Mat computeYawTransform(const std::vector<std::vector<double>>& marker_positions , const std::vector<double>& yaw_angles);

} // namespace handeyecalib

#endif // HAND_EYE_CALIB__HEAD_UTILS_HPP_
