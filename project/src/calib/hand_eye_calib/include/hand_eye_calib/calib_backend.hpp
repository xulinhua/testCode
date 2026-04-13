#ifndef HAND_EYE_CALIB__CALIB_BACKEND_HPP_
#define HAND_EYE_CALIB__CALIB_BACKEND_HPP_

#include <vector>
#include <opencv2/opencv.hpp>

// 命名空间定义
namespace handeyecalib {
    
/**
 * @brief 使用OpenCV库进行手眼标定计算
 * 
 * @param R_end2bases 机器人末端相对于基座的旋转矩阵集合
 * @param t_end2bases 机器人末端相对于基座的平移向量集合
 * @param R_maker2cameras 标定板相对于相机的旋转矩阵集合
 * @param t_maker2cameras 标定板相对于相机的平移向量集合
 * @param eye_on_hand 是否为eye-on-hand模式(true: eye-on-hand, false: eye-to-hand)
 * @param bCamInv 是否需要反转相机变换矩阵
 * @param method 手眼标定方法，默认为HORAUD方法
 * @return cv::Mat 返回标定结果变换矩阵(4x4)，对于eye-to-hand为相机到基座的变换，
 *                对于eye-on-hand为相机到末端的变换
 */
cv::Mat HandEyeCalibrationByCv(
    const std::vector<cv::Mat>& R_end2bases, const std::vector<cv::Mat>& t_end2bases,
    const std::vector<cv::Mat>& R_maker2cameras, const std::vector<cv::Mat>& t_maker2cameras,
    bool eye_on_hand, bool bCamInv,
    cv::HandEyeCalibrationMethod method = cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_HORAUD);

/**
 * @brief 在eye-on-hand模式下计算目标相对于机器人基座的位姿
 * 
 * @param marker_pose 标定板相对于相机的位姿矩阵(4x4)
 * @param end_pose 机器人末端相对于基座的位姿矩阵(4x4)
 * @param camera_to_end_transform 相机到机器人末端的变换矩阵(4x4)，即手眼标定结果
 * @return cv::Mat 返回目标相对于机器人基座的位姿矩阵(4x4)
 */
cv::Mat computeRobotPoseEyeOnHand(const cv::Mat& marker_pose, const cv::Mat& end_pose,
                                          const cv::Mat& camera_to_end_transform);

/**
 * @brief 在eye-to-hand模式下计算目标相对于机器人基座的位姿
 * 
 * @param marker_pose 标定板相对于相机的位姿矩阵(4x4)
 * @param camera_to_base_transform 相机到机器人基座的变换矩阵(4x4)，即手眼标定结果
 * @return cv::Mat 返回目标相对于机器人基座的位姿矩阵(4x4)
 */
cv::Mat computeRobotPoseEyeToHand(const cv::Mat& marker_pose, const cv::Mat& camera_to_base_transform) ;

/**
 * @brief 计算重投影误差，用于评估标定结果精度
 * 
 * @param marker_pose 标定板在不同位置时相对于相机的位姿矩阵集合
 * @param end_pose 机器人末端在不同位置时相对于基座的位姿矩阵集合
 * @param calibration_result 标定结果变换矩阵(4x4)
 * @param eye_on_hand 是否为eye-on-hand模式(true: eye-on-hand, false: eye-to-hand)
 * @return std::vector<double> 返回每个位姿对的重投影误差值集合
 */
std::vector<double> CalculateReprojectionError(const std::vector<cv::Mat>& marker_pose,
    const std::vector<cv::Mat>& end_pose,
    const cv::Mat& calibration_result,
    bool eye_on_hand);
   
////////////////////////////////////////////////////////////////////////
// 结束
////////////////////////////////////////////////////////////////////////
}

#endif // HAND_EYE_CALIB__CALIB_BACKEND_HPP_
