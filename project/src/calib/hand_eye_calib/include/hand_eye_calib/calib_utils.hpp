#ifndef HAND_EYE_CALIB__CALIB_UTILS_HPP_
#define HAND_EYE_CALIB__CALIB_UTILS_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <string>
#include <nlohmann/json.hpp>
#include "calib_struct.hpp"

// 命名空间定义
namespace handeyecalib 
{

// 常量定义
extern const double PI;
extern const bool g_bCamInv;

/**
 * @brief 将位姿(x, y, z, rx, ry, rz)转换为齐次变换矩阵
 * @param pose 位姿 [x, y, z, rx, ry, rz]，角度单位为度
 * @return 4x4 齐次变换矩阵
 */
cv::Mat poseToHomogeneousMatrix(const std::vector<double>& pose);

/**
 * @brief 将齐次变换矩阵转换为位姿(x, y, z, rx, ry, rz)
 * @param transform_matrix 4x4 齐次变换矩阵
 * @return 位姿 [x, y, z, rx, ry, rz]，角度单位为度
 */
std::vector<double> homogeneousMatrixToPose(const cv::Mat& transform_matrix);

/**
 * @brief 将欧拉角(度)转换为旋转矩阵
 * @param euler_angles 欧拉角 [rx, ry, rz]，单位为度
 * @return 3x3 旋转矩阵
 */
cv::Mat eulerToRotationMatrix(const std::vector<double>& euler_angles);

/**
 * @brief 将旋转矩阵转换为欧拉角(度)
 * @param rotation_matrix 3x3 旋转矩阵
 * @return 欧拉角 [rx, ry, rz]，单位为度
 */
std::vector<double> rotationMatrixToEuler(const cv::Mat& rotation_matrix);

/**
 * @brief 四元数转欧拉角 (ZYX顺序 - 更常用)
 * @return [roll, pitch, yaw] 但旋转顺序不同
 * 
 * ZYX顺序（外旋）：
 * 1. 先绕Z轴旋转 (yaw)
 * 2. 再绕Y轴旋转 (pitch)
 * 3. 最后绕X轴旋转 (roll)
 */
std::vector<double> quaternionToEulerZYX(double qw, double qx, double qy, double qz) ;

/**
 * @brief 将多个位姿向量转换为旋转和平移
 * @param pose_vectors 位姿向量列表
 * @param R_base2ends 旋转矩阵列表
 * @param t_base2ends 平移向量列表
 * @return pair<旋转矩阵列表, 平移向量列表>
 */
bool poseVectorsToTransforms(
    const std::vector<std::vector<double>>& pose_vectors,
    std::vector<cv::Mat>& R_base2ends, std::vector<cv::Mat>& t_base2ends);

/**
 * @brief 将多个变换矩阵转换为旋转和平移
 * @param Transforms 变换矩阵列表
 * @param R_matrixs 旋转矩阵列表
 * @param t_matrixs 平移向量列表
 * @return pair<旋转矩阵列表, 平移向量列表>
 */
bool transformsDecompose(
    std::vector<cv::Mat>& Transforms,
    std::vector<cv::Mat>& R_matrixs, 
    std::vector<cv::Mat>& t_matrixs);

/**
 * @brief 计算手眼标定矩阵
 * @param robot_poses 机器人位姿列表 [(x, y, z, rx, ry, rz), ...]
 * @param marker_positions 标记位置列表 [(x, y, z), ...]
 * @return 标定结果
 * 
 * 该函数通过输入的机器人位姿和标记位置数据计算眼在手外标定矩阵。
 * 使用opencv手眼标定方法计算变换矩阵，
 * eye_on_hand: 使得基座坐标 ≈ T_cam2base * 相机坐标。
 * eye_to_hand: 使得末端坐标 ≈ T_cam2end * 相机坐标。
 */
CalibRes computeHandEyeCalibrationByCv(const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions, bool eye_on_hand);

/**
 * @brief 计算头部姿态变换矩阵
 * @param pitch_angle 俯仰角，单位为度
 * @param yaw_angle 偏航角，单位为度
 * @return 4x4齐次变换矩阵
 */
cv::Mat computeHeadPoseTransform(double pitch_angle, double yaw_angle);

/**
 * @brief 计算动态头部姿态下的标定矩阵
 * @param fixed_calib_res 固定头部姿态下的标定结果
 * @param pitch_angle 俯仰角，单位为度
 * @param yaw_angle 偏航角，单位为度
 * @return 动态头部姿态下的标定结果
 */
CalibRes computeDynamicCalibration(const CalibRes& fixed_calib_res, double pitch_angle, double yaw_angle);

std::vector<double> transformToBase(const std::vector<double>& cam_pose, const cv::Mat& T_cam2base);

/**
 * @brief 计算眼在手外标定矩阵
 * @param robot_poses 机器人位姿列表 [(x, y, z, rx, ry, rz), ...]
 * @param marker_positions 标记位置列表 [(x, y, z, rx, ry, rz), ...]
 * @param coordinate_mapping 坐标系映射关系
 * @return 标定结果
 * 
 * 该函数通过输入的机器人位姿和标记位置数据计算眼在手外标定矩阵。
 */
CalibRes computeEyeToHandCalibration(
    const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions);

/**
 * @brief 计算眼在手上标定矩阵
 * @param robot_poses 机器人位姿列表 [(x, y, z, rx, ry, rz), ...]
 * @param marker_positions 标记位置列表 [(x, y, z, rx, ry, rz), ...]
 * @return 标定结果
 * 
 * 该函数通过输入的机器人位姿和标记位置数据计算眼在手上标定矩阵。
 * 这里简化处理，实际应用中需要更复杂的转换，目前直接调用眼在手外标定函数。
 */
CalibRes computeEyeInHandCalibration(
    const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions);

/**
 * @brief 应用标定结果计算机械手实际位姿
 * @param marker_pose 标记在相机坐标系下的位姿 [x, y, z, rx, ry, rz]
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @param offset_compensation 偏移补偿值指针，可选参数 [x, y, z, rx, ry, rz]
 * @return 变换结果（机械手在基座坐标系下的位姿）
 * 
 * 该函数使用标定结果将标记在相机坐标系下的位置变换到基座坐标系下。
 * 函数内部会自动完成相机坐标系到机械臂坐标系的转换。
 * 如果提供了偏移补偿值，还会将其应用到最终结果中，以计算夹爪末端的实际位姿。
 */
TransformResult computeRobotPoseFromMarker(const std::vector<double>& marker_pose,
                                           const cv::Mat& cam_to_base_transform,
                                           const std::vector<double>* offset_compensation);

/**
 * @brief 应用标定结果计算标记在相机坐标系下的位置
 * @param robot_pose 机械手在基座坐标系下的位姿 [x, y, z, rx, ry, rz]
 * @param base_to_cam_transform 基座到相机的变换矩阵
 * @return 变换结果（标记在相机坐标系下的位置）
 * 
 * 该函数使用标定结果将机器人在基座坐标系下的位姿变换到相机坐标系下。
 * 函数内部会自动完成机械臂坐标系到相机坐标系的转换。
 */
TransformResult computeMarkerPositionFromRobot(const std::vector<double>& robot_pose,
                                                const cv::Mat& base_to_cam_transform);

/**
 * @brief 应用标定结果计算机械手实际位姿
 * @param marker_pose 标记在相机坐标系下的位姿 [x, y, z, rx, ry, rz]
 * @param end_pose 末端的位姿 [x, y, z, rx, ry, rz]
 * @param camera_to_end_transform 相机到末端的变换矩阵
 * @return 变换结果（机械手在基座坐标系下的位姿）
 * 
 * 该函数使用标定结果将标记在相机坐标系下的位置变换到基座坐标系下。
 */
TransformResult computeRobotPoseFromEndMarker(const std::vector<double>& marker_pose, 
                                            const std::vector<double>& end_pose,
                                          const cv::Mat& camera_to_end_transform);

/**
 * @brief 加载第一个标定点数据并计算位姿偏差
 * @param data_dir 标定数据目录路径
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @param theoretical_pose 理论位姿（输出）
 * @param actual_pose 实际位姿（输出）
 * @param pose_deviation 位姿偏差（输出）
 * @return 是否加载和计算成功
 */
bool loadFirstCalibrationPointAndEvaluate(const std::string& data_dir,
                                         const cv::Mat& cam_to_base_transform,
                                         std::vector<double>& theoretical_pose,
                                         std::vector<double>& actual_pose,
                                         std::vector<double>& pose_deviation);

/**
 * @brief 分析标定质量
 * @param robot_poses 机器人位姿列表 [(x, y, z, rx, ry, rz), ...]
 * @param marker_positions 标记位置列表 [(x, y, z), ...]
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @return 质量评估指标
 */
QualityMetrics analyzeCalibrationQuality(
    const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions,
    const cv::Mat& cam_to_base_transform);
    
/**
 * @brief 分析标定质量
 * @param robot_poses 机器人位姿列表 [(x, y, z, rx, ry, rz), ...]
 * @param marker_positions 标记位置列表 [(x, y, z), ...]
 * @param result 相机到基座的变换矩阵
 * @param eye_on_hand 是否眼在手上
 * @return 质量评估指标
 */
QualityMetrics analyzeCalibrationQuality(
    const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions,
    const CalibRes& result, bool eye_on_hand);

/**
 * @brief 计算重投影误差
 * @param robot_poses 机器人位姿列表
 * @param marker_positions 标记位置列表
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @return 重投影误差
 */
double computeReprojectionError(
    const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions,
    const cv::Mat& cam_to_base_transform);

/**
 * @brief 计算平移误差
 * @param robot_poses 机器人位姿列表
 * @param marker_positions 标记位置列表
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @return 平移误差
 */
double computeTranslationError(
    const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions,
    const cv::Mat& cam_to_base_transform);

/**
 * @brief 计算旋转误差
 * @param robot_poses 机器人位姿列表
 * @param marker_positions 标记位置列表
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @return 旋转误差
 */
double computeRotationError(
    const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions,
    const cv::Mat& cam_to_base_transform);

/**
 * @brief 计算矩阵条件数
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @return 矩阵条件数
 */
double computeConditionNumber(const cv::Mat& cam_to_base_transform);

/**
 * @brief 根据分辨率调整标定矩阵
 * @param calib_res 原始标定结果
 * @param src_resolution_width 原始分辨率宽度
 * @param src_resolution_height 原始分辨率高度
 * @param target_resolution_width 目标分辨率宽度
 * @param target_resolution_height 目标分辨率高度
 * @return 调整后的标定结果
 * 
 * 该函数根据图像分辨率的变化调整标定矩阵，主要用于不同分辨率下的标定结果转换。
 * 主要调整的是平移分量，因为相机内参会随着分辨率变化而变化
 */
CalibRes adjustCalibrationForResolution(
    const CalibRes& calib_result,
    int src_resolution_width, int src_resolution_height,
    int target_resolution_width, int target_resolution_height);

} // namespace handeyecalib

#endif // HAND_EYE_CALIB__CALIB_UTILS_HPP_