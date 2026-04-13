#include "hand_eye_calib/head_utils.hpp"
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
extern const double PI;
extern const bool g_bCamInv;

// 计算翻滚变换矩阵
cv::Mat RollAngleToTransform(double roll_angle, double tx, double ty, double tz)
{
    double roll_rad = roll_angle * CV_PI / 180.0;
    // 绕 X 轴的翻滚变换矩阵
    cv::Mat roll_transform = (cv::Mat_<double>(4, 4) <<
        1, 0, 0, tx,
        0, cos(roll_rad), -sin(roll_rad), ty,
        0, sin(roll_rad), cos(roll_rad), tz,
        0, 0, 0, 1
    );

    return roll_transform;
}
// 计算俯仰变换矩阵
cv::Mat PitchAngleToTransform(double pitch_angle, double tx, double ty, double tz)
{
    double pitch_rad = pitch_angle * CV_PI / 180.0;
    // 绕 Y 轴的俯仰变换矩阵
    cv::Mat pitch_transform = (cv::Mat_<double>(4, 4) <<
        cos(pitch_rad), 0, sin(pitch_rad), tx,
        0, 1, 0, ty,
        -sin(pitch_rad), 0, cos(pitch_rad), tz,
        0, 0, 0, 1
    );

    return pitch_transform;
}

// 计算偏航变换矩阵
cv::Mat YawAngleToTransform(double yaw_angle, double tx, double ty, double tz)
{
    double yaw_rad = yaw_angle * CV_PI / 180.0;
    cv::Mat yaw_transform = (cv::Mat_<double>(4, 4) <<
        cos(yaw_rad), -sin(yaw_rad), 0, tx,
        sin(yaw_rad), cos(yaw_rad), 0, ty,
        0, 0, 1, tz,
        0, 0, 0, 1
    );

    return yaw_transform;
}

/**
 * @brief 计算头部姿态变换矩阵
 * @param pitch_angle 俯仰角（度）
 * @param yaw_angle 偏航角（度）
 * @param tx 平移距离（X轴）
 * @param ty 平移距离（Y轴）
 * @param tz 平移距离（Z轴）
 * @return 4x4齐次变换矩阵
 */
cv::Mat computeHeadPoseTransform(double pitch_angle, double yaw_angle, double tx, double ty, double tz) 
{
    // 将角度从度转换为弧度
    double pitch_rad = pitch_angle * PI / 180.0;
    double yaw_rad = yaw_angle * PI / 180.0;

    // 绕 Y 轴的俯仰变换矩阵
    cv::Mat pitch_transform = PitchAngleToTransform(pitch_angle, tx, ty, tz);
    
    // 计算偏航变换矩阵 (绕Z轴旋转)
    cv::Mat yaw_transform = YawAngleToTransform(yaw_angle, 0.0, 0.0, 0.0);
    
    // 组合变换：俯仰在偏航上
    cv::Mat head_transform = yaw_transform * pitch_transform;
    return head_transform;
}

// 计算偏航变换矩阵
cv::Mat computeYawTransform(const std::vector<std::vector<double>>& marker_positions , const std::vector<double>& yaw_angles)
{
    // 检查输入数据
    if (yaw_angles.size() != marker_positions.size())
    {
        throw std::invalid_argument("偏航角和标记位置数量不匹配");
    }

    if (yaw_angles.size() < 4)
    {
        throw std::invalid_argument("至少需要4个标定点来计算标定矩阵");
    }
    
    cv::Mat result_transform;
    try
    {
        LOG_INFO("计算偏航变换矩阵...");
        std::vector<cv::Mat> R_end2bases, t_end2bases;
        std::vector<cv::Mat> R_maker2cameras, t_maker2cameras;
        std::vector<cv::Mat> robot_poses;
        std::vector<cv::Mat> marker_poses;
        for (int i = 0; i < yaw_angles.size(); i++)
        {
            //cv::Mat pre_marker = poseToHomogeneousMatrix(marker_positions[i-1]);
            //cv::Mat cur_marker = poseToHomogeneousMatrix(marker_positions[i]);
            //cv::Mat marker =  pre_marker.inv() * cur_marker;
            cv::Mat marker =  poseToHomogeneousMatrix(marker_positions[i]);
            marker_poses.push_back(marker);

            //cv::Mat pre_robot = YawAngleToTransform(yaw_angles[i-1]);
            //cv::Mat cur_robot = YawAngleToTransform(yaw_angles[i]);
            //cv::Mat robot =  pre_robot.inv() * cur_robot;
            cv::Mat robot =  YawAngleToTransform(yaw_angles[i]);
            robot_poses.push_back(robot);
            std::cout << "robot: " << robot << std::endl;
            std::cout << "marker: " << marker_positions[i][0] << " " << marker_positions[i][1] << " " << marker_positions[i][2] << " " << marker_positions[i][3] << " " << marker_positions[i][4] << " " << marker_positions[i][5] << std::endl;
        }
        LOG_INFO("矩阵分解...");
        transformsDecompose(robot_poses, R_end2bases, t_end2bases);
        LOG_INFO("分解机器人位姿...");
        //poseVectorsToTransforms(marker_positions, R_maker2cameras, t_maker2cameras);
        transformsDecompose(marker_poses, R_maker2cameras, t_maker2cameras);
        LOG_INFO("分解标记位置...");
        
        bool eye_on_hand = true;
        // 计算HORAUD方法的标定矩阵（默认）
        cv::Mat T_X = HandEyeCalibrationByCv(R_end2bases, t_end2bases, R_maker2cameras, t_maker2cameras, eye_on_hand, g_bCamInv);
        LOG_INFO("计算偏航变换矩阵...");
        if (0 && !T_X.empty())
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
            cv::Mat T_X_Temp = T_X;
            std::vector<double> ar_errors;
            double sum = 0.0, mean = 0.0, min_error = 0.0;
            std::string str_msg;

            ar_errors = CalculateReprojectionError(marker_poses, end_poses, T_X_Temp, eye_on_hand);
            sum = accumulate(begin(ar_errors), end(ar_errors), 0.0);  
            min_error = mean =  sum / ar_errors.size();
            LOG_INFO("HORAUD方法误差: %f", mean);
        }
        if (0)
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

        result_transform = T_X.clone();
    
        std::cout << "result_transform:" << std::endl << result_transform << std::endl;
        LOG_INFO("✅ 标定矩阵计算完成");

    }
    catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
    }
    
    return result_transform;
}

} // namespace handeyecalib
