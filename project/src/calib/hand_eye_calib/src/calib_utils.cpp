#include "hand_eye_calib/calib_utils.hpp"
#include "hand_eye_calib/calib_reflector.hpp"
#include "hand_eye_calib/calib_backend.hpp"
#include <cmath>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <limits>
#include <sstream>
#include <iomanip>
#include "log_system/log_macros.hpp"
#include "internal_constants.h"
#include "bas_operate/bas_utils.hpp"
#include "data_handler/json_operate.h"
namespace fs = std::filesystem;
using json = nlohmann::json;
// 命名空间定义
namespace handeyecalib {

// 常量定义
const double PI = 3.14159265358979323846;
const bool g_bCamInv = false;

/**
 * @brief 将位姿(x, y, z, rx, ry, rz)转换为齐次变换矩阵
 * @param pose 位姿 [x, y, z, rx, ry, rz]，角度单位为度
 * @return 4x4 齐次变换矩阵
 * 
 * 该函数将6维位姿向量转换为4x4的齐次变换矩阵。
 * 位姿向量前3个元素表示平移分量(x,y,z)，后3个元素表示绕x,y,z轴的旋转角度(欧拉角)。
 * 函数内部会将角度转换为弧度，并使用ZYX欧拉角顺序计算旋转矩阵。
 */
cv::Mat poseToHomogeneousMatrix(const std::vector<double>& pose) 
{
    // 检查输入位姿数据维度是否正确（必须是6维）
    if (pose.size() != 6) 
    {
        throw std::invalid_argument("位姿数据维度错误，应为6维");
    }
    
    // 提取平移分量和旋转分量（角度）
    double x = pose[0], y = pose[1], z = pose[2];
    double rx = pose[3], ry = pose[4], rz = pose[5];
    
    // 根据欧拉角公式构造3x3旋转矩阵
    cv::Mat rotation = eulerToRotationMatrix({rx, ry, rz});
    
    // 构造4x4齐次变换矩阵
    // 齐次变换矩阵由3x3旋转矩阵和平移向量组成
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    rotation.copyTo(transform(cv::Rect(0, 0, 3, 3)));  // 将旋转矩阵复制到左上角3x3区域
    transform.at<double>(0, 3) = x;  // 设置x方向平移
    transform.at<double>(1, 3) = y;  // 设置y方向平移
    transform.at<double>(2, 3) = z;  // 设置z方向平移
    
    return transform;
}

/**
 * @brief 将齐次变换矩阵转换为位姿(x, y, z, rx, ry, rz)
 * @param transform_matrix 4x4 齐次变换矩阵
 * @return 位姿 [x, y, z, rx, ry, rz]，角度单位为度
 * 
 * 该函数将4x4齐次变换矩阵转换为6维位姿向量。
 * 输出向量前3个元素表示平移分量(x,y,z)，后3个元素表示绕x,y,z轴的旋转角度(欧拉角)。
 * 函数内部会将弧度转换为角度输出。
 */
std::vector<double> homogeneousMatrixToPose(const cv::Mat& transform_matrix) 
{
    // 检查输入变换矩阵维度是否正确（必须是4x4）
    if (transform_matrix.rows != 4 || transform_matrix.cols != 4) {
        throw std::invalid_argument("变换矩阵维度错误，应为4x4");
    }
    
    // 提取平移部分（矩阵第4列的前3个元素）
    double x = transform_matrix.at<double>(0, 3);
    double y = transform_matrix.at<double>(1, 3);
    double z = transform_matrix.at<double>(2, 3);
    
    // 提取旋转矩阵部分（矩阵左上角3x3区域）
    cv::Mat rotation = transform_matrix(cv::Rect(0, 0, 3, 3));
    
    // 从旋转矩阵计算欧拉角 (使用ZYX欧拉角顺序)
    // sy是sin(ry)的负值，cy是cos(ry)
    double sy = -rotation.at<double>(2, 0);
    double cy = sqrt(1 - sy*sy);  // 利用三角恒等式计算cos(ry)
    
    double rx, ry, rz;
    // 当cos(ry)大于一个很小的数时，使用标准公式计算欧拉角
    if (cy > 1e-6) 
    {
        // rx = atan2(R(2,1), R(2,2))
        rx = atan2(rotation.at<double>(2, 1), rotation.at<double>(2, 2));
        // ry = atan2(-R(2,0), cy)
        ry = atan2(-rotation.at<double>(2, 0), cy);
        // rz = atan2(R(1,0), R(0,0))
        rz = atan2(rotation.at<double>(1, 0), rotation.at<double>(0, 0));
    } else {
        // 当cos(ry)接近0时，会发生万向锁现象，此时rz设为0
        // 使用不同的公式计算rx来避免数值不稳定
        rx = atan2(-rotation.at<double>(1, 2), rotation.at<double>(1, 1));
        ry = atan2(-rotation.at<double>(2, 0), cy);
        rz = 0;
    }
    
    // 将弧度转换为角度
    rx = rx * 180.0 / PI;
    ry = ry * 180.0 / PI;
    rz = rz * 180.0 / PI;
    
    // 返回位姿向量[x, y, z, rx, ry, rz]
    return {x, y, z, rx, ry, rz};
}

/**
 * @brief 将欧拉角(度)转换为旋转矩阵
 * @param euler_angles 欧拉角 [rx, ry, rz]，单位为度
 * @return 3x3 旋转矩阵
 * 
 * 该函数将3维欧拉角向量转换为3x3的旋转矩阵。
 * 输入向量3个元素分别表示绕x,y,z轴的旋转角度。
 * 函数内部会将角度转换为弧度，并使用ZYX欧拉角顺序计算旋转矩阵。
 */
cv::Mat eulerToRotationMatrix(const std::vector<double>& euler_angles) 
{
    // 检查输入欧拉角数据维度是否正确（必须是3维）
    if (euler_angles.size() != 3) 
    {
        throw std::invalid_argument("欧拉角数据维度错误，应为3维");
    }
    
    // 将角度转换为弧度
    double rx = euler_angles[0] * PI / 180.0;
    double ry = euler_angles[1] * PI / 180.0;
    double rz = euler_angles[2] * PI / 180.0;
    
    // 计算三角函数值
    double cx = cos(rx), sx = sin(rx);
    double cy = cos(ry), sy = sin(ry);
    double cz = cos(rz), sz = sin(rz);
    
    // 根据欧拉角公式构造并返回3x3旋转矩阵
    return (cv::Mat_<double>(3, 3) <<
        cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx,
        sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx,
        -sy,   cy*sx,            cy*cx
    );
}

/**
 * @brief 将旋转矩阵转换为欧拉角(度)
 * @param rotation_matrix 3x3 旋转矩阵
 * @return 欧拉角 [rx, ry, rz]，单位为度
 * 
 * 该函数将3x3旋转矩阵转换为3维欧拉角向量。
 * 输出向量3个元素分别表示绕x,y,z轴的旋转角度。
 * 函数内部会将弧度转换为角度输出。
 */
std::vector<double> rotationMatrixToEuler(const cv::Mat& rotation_matrix) 
{
    // 检查输入旋转矩阵维度是否正确（必须是3x3）
    if (rotation_matrix.rows != 3 || rotation_matrix.cols != 3) 
    {
        throw std::invalid_argument("旋转矩阵维度错误，应为3x3");
    }
    
    // 从旋转矩阵元素计算sin(ry)和cos(ry)
    double sy = -rotation_matrix.at<double>(2, 0);
    double cy = sqrt(1 - sy*sy);  // 利用三角恒等式计算cos(ry)
    
    double rx, ry, rz;
    // 当cos(ry)大于一个很小的数时，使用标准公式计算欧拉角
    if (cy > 1e-6) 
    {
        // rx = atan2(R(2,1), R(2,2))
        rx = atan2(rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
        // ry = atan2(-R(2,0), cy)
        ry = atan2(-rotation_matrix.at<double>(2, 0), cy);
        // rz = atan2(R(1,0), R(0,0))
        rz = atan2(rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0));
    } else {
        // 当cos(ry)接近0时，会发生万向锁现象，此时rz设为0
        // 使用不同的公式计算rx来避免数值不稳定
        rx = atan2(-rotation_matrix.at<double>(1, 2), rotation_matrix.at<double>(1, 1));
        ry = atan2(-rotation_matrix.at<double>(2, 0), cy);
        rz = 0;
    }
    
    // 将弧度转换为角度
    rx = rx * 180.0 / PI;
    ry = ry * 180.0 / PI;
    rz = rz * 180.0 / PI;
    
    // 返回欧拉角向量[rx, ry, rz]
    return {rx, ry, rz};
}

/**
 * @brief 四元数转欧拉角 (ZYX顺序 - 更常用)
 * @return [roll, pitch, yaw] 但旋转顺序不同
 * 
 * ZYX顺序（外旋）：
 * 1. 先绕Z轴旋转 (yaw)
 * 2. 再绕Y轴旋转 (pitch)
 * 3. 最后绕X轴旋转 (roll)
 */
std::vector<double> quaternionToEulerZYX(double qw, double qx, double qy, double qz) 
{
    double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    double w = qw / norm;
    double x = qx / norm;
    double y = qy / norm;
    double z = qz / norm;
    
    double roll, pitch, yaw;
    
    // ZYX顺序
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }
    
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
    
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;
    
    return {roll, pitch, yaw};
}

/**
 * @brief 将多个位姿向量转换为旋转和平移
 * @param pose_vectors 位姿向量列表
 * @param R_base2ends 旋转矩阵列表
 * @param t_base2ends 平移向量列表
 * @return pair<旋转矩阵列表, 平移向量列表>
 */
bool poseVectorsToTransforms(
    const std::vector<std::vector<double>>& pose_vectors,
    std::vector<cv::Mat>& R_matrixs, std::vector<cv::Mat>& t_matrixs) 
{
    R_matrixs.clear();
    t_matrixs.clear();
    
    for (const auto& pose : pose_vectors) {
        if (pose.size() != 6) {
            throw std::invalid_argument("位姿向量必须包含6个元素");
        }
        
        // 机器人位姿本身就是以机器人坐标系表示的，不需要进行坐标系转换
        double x = pose[0];
        double y = pose[1];
        double z = pose[2];
        double rx = pose[3];
        double ry = pose[4];
        double rz = pose[5];
        
        cv::Mat mcr = eulerToRotationMatrix({rx, ry, rz});
        cv::Mat mct = cv::Mat::zeros(3, 1, CV_64F);
        mct.at<double>(0, 0) = x;
        mct.at<double>(1, 0) = y;
        mct.at<double>(2, 0) = z;
        
        R_matrixs.push_back(mcr);
        t_matrixs.push_back(mct);
    }
    
    return true;
}

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
    std::vector<cv::Mat>& t_matrixs) 
{
    R_matrixs.clear();
    t_matrixs.clear();
    
    for (const auto& transform : Transforms) {
        // 检查变换矩阵是否为空
        if (transform.empty()) {
            return false;
        }
        
        // 检查变换矩阵的大小是否为 4x4
        if (transform.rows != 4 || transform.cols != 4) {
            return false;
        }
        
        // 检查矩阵类型是否为浮点型
        if (transform.type() != CV_32F && transform.type() != CV_64F) {
            return false;
        }
        
        // 提取旋转矩阵（前3行前3列）
        cv::Mat R = transform(cv::Rect(0, 0, 3, 3)).clone();
        
        // 提取平移向量（前3行第4列）
        cv::Mat t = transform(cv::Rect(3, 0, 1, 3)).clone();
        
        // 可选：检查旋转矩阵是否有效（正交且行列式为1）
        // 对于浮点数计算，这里使用近似判断
        cv::Mat R_double;
        if (R.type() == CV_32F) {
            R.convertTo(R_double, CV_64F);
        } else {
            R_double = R.clone();
        }
        
        // 检查旋转矩阵是否正交（R * R^T ≈ I）
        cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat RRT = R_double * R_double.t();
        double norm_diff = cv::norm(RRT, I, cv::NORM_L2);
        
        if (norm_diff > 1e-5) {
            // 旋转矩阵不满足正交性，可能不是有效的旋转矩阵
            // 可以根据需要选择是否报错或进行修正
            // return false;
        }
        
        // 检查行列式是否为1（避免反射）
        double det = cv::determinant(R_double);
        if (std::abs(det - 1.0) > 1e-5) {
            // 行列式不为1，包含反射或缩放
            // return false;
        }
        
        // 添加到结果列表
        R_matrixs.push_back(R);
        t_matrixs.push_back(t);
    }
    
    return true;
}

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
    const std::vector<std::vector<double>>& marker_positions, bool eye_on_hand) 
{
    // 检查输入数据
    if (robot_poses.size() != marker_positions.size())
    {
        throw std::invalid_argument("机器人位姿和标记位置数量不匹配");
    }

    if (robot_poses.size() < 4)
    {
        throw std::invalid_argument("至少需要4个标定点来计算标定矩阵");
    }
    
    CalibRes result;
    try
    {
        LOG_INFO("🔄 使用OpenCV计算眼在手外标定矩阵...");
        std::vector<cv::Mat> R_end2bases, t_end2bases;
        std::vector<cv::Mat> R_maker2cameras, t_maker2cameras;
        poseVectorsToTransforms(robot_poses, R_end2bases, t_end2bases);
        poseVectorsToTransforms(marker_positions, R_maker2cameras, t_maker2cameras);

        // 计算HORAUD方法的标定矩阵（默认）
        cv::Mat T_X = HandEyeCalibrationByCv(R_end2bases, t_end2bases, R_maker2cameras, t_maker2cameras, eye_on_hand, g_bCamInv);
        if (1)
        {
            bool bShowMsg = true;
            std::vector<cv::Mat> end_poses;
            std::vector<cv::Mat> marker_poses;
            // 创建4x4齐次变换矩阵
            for (int i = 0; i < robot_poses.size(); i++) 
            {
                cv::Mat T_marker2cam = poseToHomogeneousMatrix(marker_positions[i]);
                cv::Mat T_end2base = poseToHomogeneousMatrix(robot_poses[i]);
                end_poses.push_back(T_end2base);
                marker_poses.push_back(T_marker2cam);
            }
            cv::Mat T_X_Temp = T_X;
            std::vector<double> ar_errors;
            double sum = 0.0, mean = 0.0, min_error = 0.0;
            std::string str_msg;

            // 计算HORAUD方法误差
            if (!T_X_Temp.empty())
            {
                ar_errors = CalculateReprojectionError(marker_poses, end_poses, T_X_Temp, eye_on_hand);
                sum = accumulate(begin(ar_errors), end(ar_errors), 0.0);  
                min_error = mean =  sum / ar_errors.size();
            }

            // 计算CALIB_HAND_EYE_DANIILIDIS方法的标定矩阵
            T_X_Temp = HandEyeCalibrationByCv(R_end2bases, t_end2bases, R_maker2cameras, t_maker2cameras, eye_on_hand, g_bCamInv, cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_DANIILIDIS);
            if (!T_X_Temp.empty())
            {
                ar_errors = CalculateReprojectionError(marker_poses, end_poses, T_X_Temp, eye_on_hand);
                sum = accumulate(begin(ar_errors), end(ar_errors), 0.0);  
                mean =  sum / ar_errors.size();
                if (bShowMsg)
                {
                    str_msg = "CALIB_HAND_EYE_DANIILIDIS, 平均误差:" + std::to_string(mean) + (mean < min_error ? " (更优选择)" : "");
                    LOG_INFO(str_msg.c_str());
                }
                if (mean < min_error)
                {
                    min_error = mean;
                    T_X = T_X_Temp;
                }
            }

            // 计算CALIB_HAND_EYE_TSAI方法的标定矩阵
            T_X_Temp = HandEyeCalibrationByCv(R_end2bases, t_end2bases, R_maker2cameras, t_maker2cameras, eye_on_hand, g_bCamInv, cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_TSAI);
            if (!T_X_Temp.empty())
            {
                ar_errors = CalculateReprojectionError(marker_poses, end_poses, T_X_Temp, eye_on_hand);
                sum = accumulate(begin(ar_errors), end(ar_errors), 0.0);  
                mean =  sum / ar_errors.size();
                if (bShowMsg)
                {
                    str_msg = "CALIB_HAND_EYE_TSAI, 平均误差:" + std::to_string(mean) + (mean < min_error ? " (更优选择)" : "");
                    LOG_INFO(str_msg.c_str());
                }
                if (mean < min_error)
                {
                    min_error = mean;
                    T_X = T_X_Temp;
                }
            } 

            // 计算CALIB_HAND_EYE_PARK方法的标定矩阵
            T_X_Temp = HandEyeCalibrationByCv(R_end2bases, t_end2bases, R_maker2cameras, t_maker2cameras, eye_on_hand, g_bCamInv, cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_PARK);
            if (!T_X_Temp.empty())
            {
                ar_errors = CalculateReprojectionError(marker_poses, end_poses, T_X_Temp, eye_on_hand);
                sum = accumulate(begin(ar_errors), end(ar_errors), 0.0);  
                mean =  sum / ar_errors.size();
                if (bShowMsg)
                {
                    str_msg = "CALIB_HAND_EYE_PARK, 平均误差:" + std::to_string(mean) + (mean < min_error ? " (更优选择)" : "");
                    LOG_INFO(str_msg.c_str());
                }
                if (mean < min_error)
                {
                    min_error = mean;
                    T_X = T_X_Temp;
                }
            }     

            // 计算CALIB_HAND_EYE_ANDREFF方法的标定矩阵
            T_X_Temp = HandEyeCalibrationByCv(R_end2bases, t_end2bases, R_maker2cameras, t_maker2cameras, eye_on_hand, g_bCamInv, cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_ANDREFF);
            if (!T_X_Temp.empty())
            {
                ar_errors = CalculateReprojectionError(marker_poses, end_poses, T_X_Temp, eye_on_hand);
                sum = accumulate(begin(ar_errors), end(ar_errors), 0.0);  
                mean =  sum / ar_errors.size();
                if (bShowMsg)
                {
                    str_msg = "CALIB_HAND_EYE_ANDREFF, 平均误差:" + std::to_string(mean) + (mean < min_error ? " (更优选择)" : "");
                    LOG_INFO(str_msg.c_str());
                }
                if (mean < min_error)
                {
                    min_error = mean;
                    T_X = T_X_Temp;
                }
            }
        }

        cv::Mat T_X_Inv = T_X.inv();
        // 构造返回结果
        result.calib_type = eye_on_hand ? CalibRes::CalibType::EYE_IN_HAND : CalibRes::CalibType::EYE_TO_HAND; // 默认设置为眼在手外
        result.cam_to_base_transform = T_X.clone();
        result.base_to_cam_transform = T_X_Inv.clone();

        // 转换为Eigen格式
        result.eigen_cam_to_base = Eigen::Matrix4d::Identity();
        result.eigen_base_to_cam = Eigen::Matrix4d::Identity();
    
        for (int i = 0; i < 4; i++) 
        {
            for (int j = 0; j < 4; j++)
            { 
                result.eigen_cam_to_base(i, j) = T_X.at<double>(i, j);
                result.eigen_base_to_cam(i, j) = T_X_Inv.at<double>(i, j);
            }
        }
    
        LOG_INFO("✅ 标定矩阵计算完成");

    }
    catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
    }
    
    return result;
}

/**
 * @brief 计算眼在手外标定矩阵
 * @param robot_poses 机器人位姿列表 [(x, y, z, rx, ry, rz), ...]
 * @param marker_positions 标记位置列表 [(x, y, z), ...]
 * @return 标定结果
 * 
 * 该函数通过输入的机器人位姿和标记位置数据计算眼在手外标定矩阵。
 * 使用伪逆方法计算变换矩阵，使得基座坐标 ≈ T_cam2base * 相机坐标。
 */
CalibRes computeEyeToHandCalibration(const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions) 
{
    
    LOG_INFO("🔄 计算眼在手外标定矩阵...");

    // 使用openCV方法计算变换矩阵
    return computeHandEyeCalibrationByCv(robot_poses, marker_positions, false);
    
    // 检查输入数据
    if (robot_poses.size() != marker_positions.size()) 
    {
        throw std::invalid_argument("机器人位姿和标记位置数量不匹配");
    }
    
    if (robot_poses.size() < 4) 
    {
        throw std::invalid_argument("至少需要4个标定点来计算标定矩阵");
    }
    
    // 检查数据维度
    for (size_t i = 0; i < robot_poses.size(); i++) 
    {
        if (robot_poses[i].size() != 6) 
        {
            throw std::invalid_argument("机器人位姿数据维度错误，应为6维");
        }
        if (marker_positions[i].size() != 6) 
        {
            throw std::invalid_argument("标记位置数据维度错误，应为6维");
        }
    }
    
    int n = robot_poses.size();
    LOG_INFO("📊 使用 %d 个标定点进行计算", n);    
    // 构建坐标矩阵
    // 机器人基座坐标系下的标记位置 (4×n)
    cv::Mat base_coords = cv::Mat::ones(4, n, CV_64F);
    // 相机坐标系下的标记位置 (4×n)
    cv::Mat cam_coords = cv::Mat::ones(4, n, CV_64F);
    
    // 填充坐标数据
    for (int i = 0; i < n; i++) 
    {
        // 机器人位姿转换为齐次变换矩阵
        cv::Mat T_base2end = poseToHomogeneousMatrix(robot_poses[i]);
        
        // 标记在相机坐标系下的位置和姿态
        std::vector<double> marker_pose = {
            marker_positions[i][0],  // x
            marker_positions[i][1],  // y
            marker_positions[i][2],  // z
            marker_positions[i][3],  // rx
            marker_positions[i][4],  // ry
            marker_positions[i][5]   // rz
        };
        
        // 标记在相机坐标系下的齐次变换矩阵
        cv::Mat T_marker_cam = poseToHomogeneousMatrix(marker_pose);
        
        // 标记在基座坐标系下的位置 (通过机器人位姿转换)
        cv::Mat marker_base = T_base2end * T_marker_cam * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
        
        // 相机坐标系下的标记位置
        cam_coords.at<double>(0, i) = marker_positions[i][0];
        cam_coords.at<double>(1, i) = marker_positions[i][1];
        cam_coords.at<double>(2, i) = marker_positions[i][2];
        
        base_coords.at<double>(0, i) = marker_base.at<double>(0, 0);
        base_coords.at<double>(1, i) = marker_base.at<double>(1, 0);
        base_coords.at<double>(2, i) = marker_base.at<double>(2, 0);
    }
    
    // 打印调试信息
    LOG_INFO("🔍 相机坐标矩阵维度: %dx%d", cam_coords.rows, cam_coords.cols);
    LOG_INFO("🔍 基座坐标矩阵维度: %dx%d", base_coords.rows, base_coords.cols);
    
    // 检查矩阵是否有效
    if (cam_coords.rows != 4 || cam_coords.cols != n || base_coords.rows != 4 || base_coords.cols != n) 
    {
        throw std::runtime_error("坐标矩阵维度不正确");
    }
    
    // 检查矩阵是否包含无效值
    for (int i = 0; i < cam_coords.rows; i++) 
    {
        for (int j = 0; j < cam_coords.cols; j++) 
        {
            if (!std::isfinite(cam_coords.at<double>(i, j))) 
            {
                throw std::runtime_error("相机坐标矩阵包含无效值");
            }
        }
    }
    
    for (int i = 0; i < base_coords.rows; i++) 
    {
        for (int j = 0; j < base_coords.cols; j++) 
        {
            if (!std::isfinite(base_coords.at<double>(i, j))) 
            {
                throw std::runtime_error("基座坐标矩阵包含无效值");
            }
        }
    }
    
    // 使用伪逆方法计算变换矩阵（参考Python实现）
    // T_cam2base 使得：base_coords ≈ T_cam2base * cam_coords
    cv::Mat T_cam2base;
    
    try 
    {
        // 检查矩阵是否为零矩阵
        bool cam_coords_zero = true;
        bool base_coords_zero = true;
        
        for (int i = 0; i < cam_coords.rows; i++) 
        {
            for (int j = 0; j < cam_coords.cols; j++) 
            {
                if (std::abs(cam_coords.at<double>(i, j)) > 1e-10) 
                {
                    cam_coords_zero = false;
                    break;
                }
            }
            if (!cam_coords_zero) break;
        }
        
        for (int i = 0; i < base_coords.rows; i++) 
        {
            for (int j = 0; j < base_coords.cols; j++) 
            {
                if (std::abs(base_coords.at<double>(i, j)) > 1e-10) 
                {
                    base_coords_zero = false;
                    break;
                }
            }
            if (!base_coords_zero) break;
        }
        
        if (cam_coords_zero || base_coords_zero) 
        {
            throw std::runtime_error("坐标矩阵全为零，无法求解");
        }
        
        // 使用伪逆方法计算变换矩阵（与Python版本保持一致）
        // T_cam2base = base_coords * pinv(cam_coords)
        cv::Mat cam_coords_pinv;
        cv::invert(cam_coords, cam_coords_pinv, cv::DECOMP_SVD);  // 计算伪逆
        T_cam2base = base_coords * cam_coords_pinv;  // 矩阵乘法
    } catch (const cv::Exception& e) {
        LOG_ERROR("❌ OpenCV错误: %s", e.what());
        throw;
    } catch (const std::exception& e) {
        LOG_ERROR("❌ 计算错误: %s", e.what());
        throw;
    }
    
    // 检查结果矩阵是否有效
    if (T_cam2base.empty() || T_cam2base.rows != 4 || T_cam2base.cols != 4) 
    {
        throw std::runtime_error("计算得到的变换矩阵无效");
    }
    
    // 检查结果矩阵是否包含无效值
    for (int i = 0; i < T_cam2base.rows; i++) 
    {
        for (int j = 0; j < T_cam2base.cols; j++) 
        {
            if (!std::isfinite(T_cam2base.at<double>(i, j))) 
            {
                throw std::runtime_error("计算得到的变换矩阵包含无效值");
            }
        }
    }
    
    // 计算逆变换
    cv::Mat T_base2cam = T_cam2base.inv();
    
    // 构造返回结果
    CalibRes result;
    result.calib_type = CalibRes::CalibType::EYE_TO_HAND; // 默认设置为眼在手外
    result.cam_to_base_transform = T_cam2base.clone();
    result.base_to_cam_transform = T_base2cam.clone();

    // 转换为Eigen格式
    result.eigen_cam_to_base = Eigen::Matrix4d::Identity();
    result.eigen_base_to_cam = Eigen::Matrix4d::Identity();

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            result.eigen_cam_to_base(i, j) = T_cam2base.at<double>(i, j);
            result.eigen_base_to_cam(i, j) = T_base2cam.at<double>(i, j);
        }
    }

    LOG_INFO("✅ 标定矩阵计算完成");
    return result;
}

/**
 * @brief 计算眼在手上标定矩阵
 * @param robot_poses 机器人位姿列表 [(x, y, z, rx, ry, rz), ...]
 * @param marker_positions 标记位置列表 [(x, y, z), ...]
 * @return 标定结果
 * 
 * 该函数通过输入的机器人位姿和标记位置数据计算眼在手上标定矩阵。
 * 这里简化处理，实际应用中需要更复杂的转换，目前直接调用眼在手外标定函数。
 */
CalibRes computeEyeInHandCalibration(const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions)
{

    LOG_INFO("🔄 计算眼在手上标定矩阵...");

    // 检查输入数据
    if (robot_poses.size() != marker_positions.size())
    {
        throw std::invalid_argument("机器人位姿和标记位置数量不匹配");
    }

    if (robot_poses.size() < 4)
    {
        throw std::invalid_argument("至少需要4个标定点来计算标定矩阵");
    }

    // 使用openCV方法计算变换矩阵
    return computeHandEyeCalibrationByCv(robot_poses, marker_positions, true);
}

/**
 * @brief 应用标定结果计算机械手实际位姿
 * @param marker_pose 标记在相机坐标系下的位姿 [x, y, z, rx, ry, rz]
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @param offset_compensation 位姿补偿参数，可为空指针
 * @return 变换结果（机械手在基座坐标系下的位姿）
 * 该函数使用标定结果将标记在相机坐标系下的位置变换到基座坐标系下。
 * 函数内部会自动完成相机坐标系到机械臂坐标系的转换。
 */
TransformResult computeRobotPoseFromMarker(const std::vector<double>& marker_pose, const cv::Mat& cam_to_base_transform, 
	const std::vector<double>* offset_compensation) 
{
    if (marker_pose.size() != 6) 
    {
        throw std::invalid_argument("标记位姿数据维度错误，应为6维(x,y,z,rx,ry,rz)");
    }
    
    // 提取位置和欧拉角
    double x = marker_pose[0];
    double y = marker_pose[1];
    double z = marker_pose[2];
    double rx = marker_pose[3];  // 绕X轴旋转
    double ry = marker_pose[4];  // 绕Y轴旋转
    double rz = marker_pose[5];  // 绕Z轴旋转
    
    // 创建标记在相机坐标系下的旋转矩阵
    // 将欧拉角(rx, ry, rz)转换为旋转矩阵
    cv::Mat R_marker_cam = eulerToRotationMatrix({rx, ry, rz});
    
    // 创建4x4齐次变换矩阵
    cv::Mat T_marker_cam = cv::Mat::eye(4, 4, CV_64F);
    
    // 设置旋转部分
    R_marker_cam.copyTo(T_marker_cam(cv::Rect(0, 0, 3, 3)));
    
    // 设置平移部分
    T_marker_cam.at<double>(0, 3) = x;
    T_marker_cam.at<double>(1, 3) = y;
    T_marker_cam.at<double>(2, 3) = z;
    
    // 转换到基座坐标系
    cv::Mat T_marker_base = cam_to_base_transform * T_marker_cam;
    
    // 提取位置
    std::vector<double> position = {
        T_marker_base.at<double>(0, 3),
        T_marker_base.at<double>(1, 3),
        T_marker_base.at<double>(2, 3)
    };
    
    // 提取旋转矩阵
    cv::Mat R_marker_base = T_marker_base(cv::Rect(0, 0, 3, 3));
    
    // 将旋转矩阵转换为欧拉角
    std::vector<double> orientation = rotationMatrixToEuler(R_marker_base);
    
    // 构造返回结果
    TransformResult result;
    
    // 合并位置和姿态
    result.transformed_pose = position;
    result.transformed_pose.insert(result.transformed_pose.end(), orientation.begin(), orientation.end());  
    result.transformation_matrix = T_marker_base.clone();  
    // 如果提供了偏移补偿值，则应用到结果中
    if (offset_compensation != nullptr) 
    {
        // 应用偏移补偿值到位置
        if (result.transformed_pose.size() >= 3 && offset_compensation->size() >= 3) 
        {
            result.transformed_pose[0] += (*offset_compensation)[0];  // x补偿
            result.transformed_pose[1] += (*offset_compensation)[1];  // y补偿
            result.transformed_pose[2] += (*offset_compensation)[2];  // z补偿
        }
        
        // 如果偏移补偿包含旋转分量，则也应用到旋转
        if (result.transformed_pose.size() >= 6 && offset_compensation->size() >= 6) 
        {
            result.transformed_pose[3] += (*offset_compensation)[3];  // rx补偿
            result.transformed_pose[4] += (*offset_compensation)[4];  // ry补偿
            result.transformed_pose[5] += (*offset_compensation)[5];  // rz补偿
        }
    }  
    return result;
}

/**
 * @brief 应用标定结果计算标记在相机坐标系下的位置
 * @param robot_pose 机械手在基座坐标系下的位姿 [x, y, z, rx, ry, rz]
 * @param base_to_camera_transform 基座到相机的变换矩阵
 * @return 变换结果（标记在相机坐标系下的位置）
 * 
 * 该函数使用标定结果将机器人在基座坐标系下的位姿变换到相机坐标系下。
 */
TransformResult computeMarkerPositionFromRobot(const std::vector<double>& robot_pose,
    const cv::Mat& base_to_cam_transform) 
{
    if (robot_pose.size() != 6) 
    {
        throw std::invalid_argument("机器人位姿数据维度错误，应为6维");
    }
    
    // 将机器人位姿转换为齐次变换矩阵
    cv::Mat robot_transform = poseToHomogeneousMatrix(robot_pose);
    
    // 通过标定矩阵和机器人位姿计算标记在相机坐标系下的位置
    cv::Mat marker_base = robot_transform * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
    
    cv::Mat marker_cam = base_to_cam_transform * marker_base;
    
    // 提取位置坐标
    std::vector<double> position = {
        marker_cam.at<double>(0, 0),
        marker_cam.at<double>(1, 0),
        marker_cam.at<double>(2, 0)
    };
    
    // 构造返回结果
    TransformResult result;
    result.transformed_pose = position;
    result.transformation_matrix = base_to_cam_transform.clone();
    
    return result;
}

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
                                          const cv::Mat& camera_to_end_transform) 
{
    if (marker_pose.size() != 6) 
    {
        throw std::invalid_argument("标记位姿数据维度错误，应为6维(x,y,z,rx,ry,rz)");
    }
    
    // 创建4x4齐次变换矩阵
    cv::Mat T_marker2cam = poseToHomogeneousMatrix(marker_pose);
    cv::Mat T_end2base = poseToHomogeneousMatrix(end_pose);
    
    cv::Mat T_maker2base = computeRobotPoseEyeOnHand(T_marker2cam, T_end2base, camera_to_end_transform);

    // 提取位置坐标
    std::vector<double> position = {
        T_maker2base.at<double>(0, 3),
        T_maker2base.at<double>(1, 3),
        T_maker2base.at<double>(2, 3)
    };
    
    // 构造返回结果
    TransformResult result;
    result.transformed_pose = position;
    result.transformation_matrix = camera_to_end_transform.clone();
    
    return result;
}

/**
 * @brief 加载第一个标定点数据并计算位姿偏差
 * @param data_dir 标定数据目录路径
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @param theoretical_pose 理论位姿（输出）
 * @param actual_pose 实际位姿（输出）
 * @param pose_deviation 位姿偏差（输出）
 * @return 是否加载和计算成功
 * 
 * 该函数加载第一个标定点数据，并使用标定结果计算理论位姿与实际位姿的偏差。
 */
bool loadFirstCalibrationPointAndEvaluate(const std::string& data_dir, const cv::Mat& cam_to_base_transform,
    std::vector<double>& theoretical_pose, std::vector<double>& actual_pose,
    std::vector<double>& pose_deviation) 
{
    if (!fs::exists(data_dir)) // 检查目录是否存在
    {
        LOG_ERROR("❌ 数据目录不存在: %s", data_dir.c_str());
        return false;
    }
    // 查找所有JSON文件
    std::vector<std::string> json_files;
    for (const auto& entry : fs::directory_iterator(data_dir)) 
    {
        if (entry.path().extension() == ".json") 
        {
            json_files.push_back(entry.path().string());
        }
    }
    // 按文件名排序
    std::sort(json_files.begin(), json_files.end());
    // 检查是否有数据文件
    if (json_files.empty()) 
    {
        LOG_ERROR("❌ 数据目录中没有找到JSON文件");
        return false;
    }
    // 加载第一个数据点
    try 
    {
        std::ifstream file(json_files[0]);
        if (!file.is_open()) 
        {
            LOG_ERROR("❌ 无法打开文件: %s", json_files[0].c_str());
            return false;
        }
        
        json data;
        file >> data;
        
        // 解析标记位置
        auto marker_pos = data["marker_position"];
        std::vector<double> marker_position = {
            marker_pos.value("x", 0.0),
            marker_pos.value("y", 0.0),
            marker_pos.value("z", 0.0),
            marker_pos.value("rx", 0.0),
            marker_pos.value("ry", 0.0),
            marker_pos.value("rz", 0.0)
        };
        
        // 解析机器人实际位姿
        auto robot_pose = data["robot_pose"];
        actual_pose = {
            robot_pose.value("x", 0.0),
            robot_pose.value("y", 0.0),
            robot_pose.value("z", 0.0),
            robot_pose.value("rx", 0.0),
            robot_pose.value("ry", 0.0),
            robot_pose.value("rz", 0.0)
        };
        
        // 计算理论位姿
        TransformResult result = computeRobotPoseFromMarker(marker_position, cam_to_base_transform, nullptr);
        theoretical_pose = result.transformed_pose;
        // 计算位姿偏差
        pose_deviation.resize(6);
        for (int i = 0; i < 6; i++) {
            pose_deviation[i] = theoretical_pose[i] - actual_pose[i];
        }
        
        LOG_INFO("✅ 成功加载第一个标定点数据并计算位姿偏差");
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("❌ 加载第一个标定点数据失败: %s", e.what());
        return false;
    }
}

/**
 * @brief 分析标定质量
 * @param robot_poses 机器人位姿列表 [(x, y, z, rx, ry, rz), ...]
 * @param marker_positions 标记位置列表 [(x, y, z), ...]
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @return 质量评估指标
 * 
 * 该函数对标定结果进行质量分析，计算各项评估指标。
 */
QualityMetrics analyzeCalibrationQuality(const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions, const cv::Mat& cam_to_base_transform)
{ 
    LOG_INFO("🔄 分析标定质量...");
    QualityMetrics metrics;
    metrics.data_point_count = static_cast<int>(robot_poses.size());
    
    // 计算重投影误差
    metrics.reprojection_error = computeReprojectionError(robot_poses, marker_positions, cam_to_base_transform);
    
    // 计算平移误差
    metrics.translation_error = computeTranslationError(robot_poses, marker_positions, cam_to_base_transform);
    
    // 计算旋转误差
    metrics.rotation_error = computeRotationError(robot_poses, marker_positions, cam_to_base_transform);
    
    // 计算矩阵条件数
    metrics.condition_number = computeConditionNumber(cam_to_base_transform);  
    LOG_INFO("✅ 标定质量分析完成");
    return metrics;
}

QualityMetrics analyzeCalibrationQuality(
    const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions,
    const CalibRes& result, bool eye_on_hand)
{
    LOG_INFO("🔄 分析标定质量...");
    
    QualityMetrics metrics;
    if (robot_poses.size() != marker_positions.size()) 
    {
        throw std::invalid_argument("机器人位姿和标记位置数量不匹配");
    }

    if (true/*eye_on_hand*/)
    {
        std::vector<cv::Mat> end_poses;
        std::vector<cv::Mat> marker_poses;
        // 创建4x4齐次变换矩阵
        for (int i = 0; i < robot_poses.size(); i++) 
        {
            cv::Mat T_marker2cam = poseToHomogeneousMatrix(marker_positions[i]);
            cv::Mat T_end2base = poseToHomogeneousMatrix(robot_poses[i]);
            end_poses.push_back(T_end2base);
            marker_poses.push_back(T_marker2cam);
        }
        std::vector<double> ar_errors = CalculateReprojectionError(marker_poses, end_poses, result.cam_to_base_transform, eye_on_hand);
        double sum = accumulate(begin(ar_errors), end(ar_errors), 0.0);  
        double mean =  sum / ar_errors.size();
        double max = *max_element(ar_errors.begin(), ar_errors.end());

        metrics.reprojection_error = metrics.translation_error = mean;
        if (1)
        {
            std::cout << "平均误差:" << mean << " " << "最大误差:" << max << std::endl;  // 转置并输出
        }
    }
    else
    {
        metrics = analyzeCalibrationQuality(robot_poses, marker_positions, result.cam_to_base_transform);
    }
    return metrics;
}

/**
 * @brief 计算重投影误差
 * @param robot_poses 机器人位姿列表
 * @param marker_positions 标记位置列表
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @return 重投影误差
 * 
 * 该函数计算重投影误差，即通过标定矩阵计算得到的标记位置与实际标记位置之间的差异。
 */
double computeReprojectionError(const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions, const cv::Mat& cam_to_base_transform) 
{
    
    if (robot_poses.size() != marker_positions.size()) 
    {
        throw std::invalid_argument("机器人位姿和标记位置数量不匹配");
    }
    
    int n = robot_poses.size();
    if (n == 0) return 0.0;
    
    double total_error = 0.0;
    
    for (int i = 0; i < n; i++) 
    {
        // 将机器人位姿转换为齐次变换矩阵
        cv::Mat T_base2end = poseToHomogeneousMatrix(robot_poses[i]);
        
        // 标记在相机坐标系下的位置
        cv::Mat marker_cam = (cv::Mat_<double>(4, 1) << 
            marker_positions[i][0], marker_positions[i][1], marker_positions[i][2], 1.0);
        
        // 通过标定矩阵变换到基座坐标系
        cv::Mat marker_base_computed = cam_to_base_transform * marker_cam;
        
        // 通过机器人位姿直接计算标记在基座坐标系下的位置
        cv::Mat marker_base_actual = T_base2end * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
        
        // 计算位置误差
        double dx = marker_base_computed.at<double>(0, 0) - marker_base_actual.at<double>(0, 0);
        double dy = marker_base_computed.at<double>(1, 0) - marker_base_actual.at<double>(1, 0);
        double dz = marker_base_computed.at<double>(2, 0) - marker_base_actual.at<double>(2, 0);
        
        double error = sqrt(dx*dx + dy*dy + dz*dz);
        total_error += error;

        // 根据ArUco标记位置计算机器人位姿并与文件中保存的位姿比较
        if (i < 5) 
        {
            // 计算重投影误差（单位：mm）
            cv::Vec3d predicted(marker_base_computed.at<double>(0, 0), 
                               marker_base_computed.at<double>(1, 0), 
                               marker_base_computed.at<double>(2, 0));
            cv::Vec3d actual(marker_base_actual.at<double>(0, 0), 
                            marker_base_actual.at<double>(1, 0), 
                            marker_base_actual.at<double>(2, 0));
            
            std::cout << "点 " << std::setw(2) << i<< ":\n";
            std::cout << "        相机中标记位置: [" 
                      << std::setw(8) << marker_cam.at<double>(0, 0) << ", " 
                      << std::setw(8) << marker_cam.at<double>(1, 0) << ", " 
                      << std::setw(8) << marker_cam.at<double>(2, 0) << "]\n";
            
            std::cout << "        预测标记位置:   [" 
                      << std::setw(8) << predicted[0] << ", " 
                      << std::setw(8) << predicted[1] << ", " 
                      << std::setw(8) << predicted[2] << "]\n";
            
            std::cout << "        实际标记位置:   [" 
                      << std::setw(8) << actual[0] << ", " 
                      << std::setw(8) << actual[1] << ", " 
                      << std::setw(8) << actual[2] << "]\n";

            std::cout << "        重投影误差: " << std::setw(6) << error << " mm\n";

            std::cout << "        文件中保存的机器人位置: [" 
                      << std::setw(8) << robot_poses[i][0] << ", " 
                      << std::setw(8) << robot_poses[i][1] << ", " 
                      << std::setw(8) << robot_poses[i][2] << "]\n";
        }
    }
    
    return total_error / n;
}

/**
 * @brief 计算平移误差
 * @param robot_poses 机器人位姿列表
 * @param marker_positions 标记位置列表
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @return 平移误差
 * 
 * 该函数计算平移误差，这里简化处理，直接返回重投影误差。
 */
double computeTranslationError(const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions, const cv::Mat& cam_to_base_transform) 
{
    
    // 平移误差与重投影误差类似，这里简化处理，直接返回重投影误差
    return computeReprojectionError(robot_poses, marker_positions, cam_to_base_transform);
}

/**
 * @brief 计算旋转误差
 * @param robot_poses 机器人位姿列表
 * @param marker_positions 标记位置列表
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @return 旋转误差
 * 
 * 该函数计算旋转误差，这里简化处理，返回一个默认值。
 */
double computeRotationError(const std::vector<std::vector<double>>& robot_poses,
    const std::vector<std::vector<double>>& marker_positions, const cv::Mat& cam_to_base_transform) 
{  
    // 旋转误差计算较为复杂，这里简化处理，返回一个默认值
    // 实际应用中需要比较旋转矩阵的差异
    return 0.0;
}

/**
 * @brief 计算矩阵条件数
 * @param cam_to_base_transform 相机到基座的变换矩阵
 * @return 矩阵条件数
 * 
 * 该函数计算变换矩阵的条件数，用于评估矩阵的数值稳定性。
 * 条件数越大，矩阵越接近奇异矩阵，数值稳定性越差。
 */
double computeConditionNumber(const cv::Mat& cam_to_base_transform) 
{
    // 计算矩阵的条件数（奇异值的最大值与最小值之比）
    cv::Mat w, u, vt;
    cv::SVD::compute(cam_to_base_transform, w, u, vt);
    
    double max_singular = 0.0;
    double min_singular = std::numeric_limits<double>::max();
    
    for (int i = 0; i < w.rows; i++) 
    {
        double singular_value = fabs(w.at<double>(i, 0));
        if (singular_value > max_singular) {
            max_singular = singular_value;
        }
        if (singular_value < min_singular) {
            min_singular = singular_value;
        }
    }
    
    if (min_singular < 1e-10) {
        return std::numeric_limits<double>::max();
    }
    
    return max_singular / min_singular;
}

/**
 * @brief 计算头部姿态变换矩阵
 * @param pitch_angle 俯仰角（度）
 * @param yaw_angle 偏航角（度）
 * @return 4x4齐次变换矩阵
 */
cv::Mat computeHeadPoseTransform(double pitch_angle, double yaw_angle) 
{
    // 将角度从度转换为弧度
    double pitch_rad = pitch_angle * PI / 180.0;
    double yaw_rad = yaw_angle * PI / 180.0;
    
    // // 计算俯仰变换矩阵 (绕X轴旋转)
    // cv::Mat pitch_transform = (cv::Mat_<double>(4, 4) <<
    //     1, 0, 0, 0,
    //     0, cos(pitch_rad), -sin(pitch_rad), 0,
    //     0, sin(pitch_rad), cos(pitch_rad), 0,
    //     0, 0, 0, 1
    // );

    // 绕 Y 轴的俯仰变换矩阵
    cv::Mat pitch_transform = (cv::Mat_<double>(4, 4) <<
        cos(pitch_rad), 0, sin(pitch_rad), 0,
        0, 1, 0, 0,
        -sin(pitch_rad), 0, cos(pitch_rad), 0,
        0, 0, 0, 1
    );
    
    // 计算偏航变换矩阵 (绕Z轴旋转)
    cv::Mat yaw_transform = (cv::Mat_<double>(4, 4) <<
        cos(yaw_rad), -sin(yaw_rad), 0, 0,
        sin(yaw_rad), cos(yaw_rad), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    );
    
    // 组合变换：先偏航，再俯仰
    cv::Mat head_transform = pitch_transform * yaw_transform;
    return head_transform;
}

/**
 * @brief 计算动态头部姿态下的标定矩阵
 * @param fixed_calib_res 固定头部姿态下的标定结果
 * @param pitch_angle 俯仰角（度）
 * @param yaw_angle 偏航角（度）
 * @return 动态头部姿态下的标定结果
 */
CalibRes computeDynamicCalibration(const CalibRes& fixed_calib_res, double pitch_angle, double yaw_angle) 
{
    // 计算头部姿态变换矩阵
    cv::Mat head_transform = computeHeadPoseTransform(pitch_angle, yaw_angle);
    
    // 计算动态标定矩阵：T_dynamic = T_head * T_fixed
    cv::Mat dynamic_camera_to_base = head_transform * fixed_calib_res.cam_to_base_transform;
    
    // 计算逆变换
    cv::Mat dynamic_base_to_camera = dynamic_camera_to_base.inv();
    
    // 构造返回结果
    CalibRes res;
    res.cam_to_base_transform = dynamic_camera_to_base.clone();
    res.base_to_cam_transform = dynamic_base_to_camera.clone();
    
    // 转换为Eigen格式
    res.eigen_cam_to_base = Eigen::Matrix4d::Identity();
    res.eigen_base_to_cam = Eigen::Matrix4d::Identity();
    
    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            res.eigen_cam_to_base(i, j) = dynamic_camera_to_base.at<double>(i, j);
            res.eigen_base_to_cam(i, j) = dynamic_base_to_camera.at<double>(i, j);
        }
    }
    res.head_motor_angles = {pitch_angle, yaw_angle};// 将当前头部电机角度添加到标定结果中
    
    LOG_DEBUG("动态头部姿态标定矩阵计算完成 - pitch: %.2f°, yaw: %.2f°", pitch_angle, yaw_angle);
    return res;
}

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
    const CalibRes& calib_res,
    int src_resolution_width, int src_resolution_height,
    int target_resolution_width, int target_resolution_height) 
{
    // 计算分辨率缩放因子
    double scale_x = static_cast<double>(target_resolution_width) / src_resolution_width;
    double scale_y = static_cast<double>(target_resolution_height) / src_resolution_height;
    
    LOG_INFO("分辨率调整: 从 %dx%d 到 %dx%d, 缩放因子: x=%.4f, y=%.4f", 
        src_resolution_width, src_resolution_height, target_resolution_width, target_resolution_height, scale_x, scale_y);
    
    // 创建新的标定结果
    CalibRes adjusted_res = calib_res;
    
    // 调整相机到基座的变换矩阵中的平移分量
    // 注意：这里假设相机内参随分辨率线性变化，主要影响平移分量
    if (!adjusted_res.cam_to_base_transform.empty()) {
        adjusted_res.cam_to_base_transform.at<double>(0, 3) *= scale_x;  // 调整X方向平移
        adjusted_res.cam_to_base_transform.at<double>(1, 3) *= scale_y;  // 调整Y方向平移
        // Z方向平移通常不受分辨率影响
    }
    
    // 调整基座到相机的变换矩阵中的平移分量
    if (!adjusted_res.base_to_cam_transform.empty()) {
        adjusted_res.base_to_cam_transform.at<double>(0, 3) *= scale_x;  // 调整X方向平移
        adjusted_res.base_to_cam_transform.at<double>(1, 3) *= scale_y;  // 调整Y方向平移
        // Z方向平移通常不受分辨率影响
    }
    
    // 更新Eigen格式的矩阵
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (i < adjusted_res.cam_to_base_transform.rows && j < adjusted_res.cam_to_base_transform.cols) {
                adjusted_res.eigen_cam_to_base(i, j) = adjusted_res.cam_to_base_transform.at<double>(i, j);
            }
            if (i < adjusted_res.base_to_cam_transform.rows && j < adjusted_res.base_to_cam_transform.cols) {
                adjusted_res.eigen_base_to_cam(i, j) = adjusted_res.base_to_cam_transform.at<double>(i, j);
            }
        }
    }
    
    LOG_INFO("分辨率调整完成");
    return adjusted_res;
}

} // namespace handeyecalib
