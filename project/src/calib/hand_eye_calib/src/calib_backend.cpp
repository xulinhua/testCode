#include "hand_eye_calib/calib_backend.hpp"
#include <vector>
#include <numeric>
#include <algorithm>

// 命名空间定义
namespace handeyecalib {

// 输出: eye-to-hand 相机->基座 4X4
// 输出: eye-on-hand 相机→末端 4X4
cv::Mat HandEyeCalibrationByCv(
    const std::vector<cv::Mat>& R_end2bases, const std::vector<cv::Mat>& t_end2bases,
    const std::vector<cv::Mat>& R_maker2cameras, const std::vector<cv::Mat>& t_maker2cameras,
    bool eye_on_hand, bool bCamInv,
    cv::HandEyeCalibrationMethod method /*= cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_HORAUD*/)
{
    cv::Mat T_result;
    // 检查数据一致性
    if (R_end2bases.size() != R_maker2cameras.size() || 
    R_end2bases.size() != t_end2bases.size() ||
    R_end2bases.size() != t_maker2cameras.size()) 
    {
        std::cerr << "错误：机械臂位姿数量(" << R_end2bases.size() 
                 << ")与相机位姿数量(" << R_maker2cameras.size() 
                 << ")不匹配" << std::endl;
        return T_result;
    }
    
    if (R_end2bases.size() < 2) {
        std::cerr << "错误：至少需要2组数据，当前只有 " 
                 << R_end2bases.size() << " 组" << std::endl;
        return T_result;
    }

    try
    {
        std::vector<cv::Mat> R_end2bases_sample = R_end2bases;
        std::vector<cv::Mat> t_end2bases_sample = t_end2bases;
        std::vector<cv::Mat> R_maker2cameras_sample = R_maker2cameras;
        std::vector<cv::Mat> t_maker2cameras_sample = t_maker2cameras;
        if (!eye_on_hand)
        {
            if (bCamInv)
            {
                int size = R_maker2cameras.size();
                for (int i = 0; i < size; i++)
                {
                    cv::Mat T_X = cv::Mat::eye(4, 4, CV_64F); // X的齐次形式
                    R_maker2cameras[i].copyTo(T_X(cv::Rect(0, 0, 3, 3)));
                    t_maker2cameras[i].copyTo(T_X(cv::Rect(3, 0, 1, 3)));
                    cv::Mat T_cam2maker = T_X.inv(); // 相机求逆
                    R_maker2cameras_sample[i] = T_cam2maker(cv::Rect(0, 0, 3, 3));
                    t_maker2cameras_sample[i] = T_cam2maker(cv::Rect(3, 0, 1, 3));
                }
            }
            else
            {
                int size = R_end2bases.size();
                for (int i = 0; i < size; i++)
                {
                    cv::Mat T_X = cv::Mat::eye(4, 4, CV_64F); // X的齐次形式
                    R_end2bases[i].copyTo(T_X(cv::Rect(0, 0, 3, 3)));
                    t_end2bases[i].copyTo(T_X(cv::Rect(3, 0, 1, 3)));
                    cv::Mat T_base2end = T_X.inv(); // 机械臂求逆
                    R_end2bases_sample[i] = T_base2end(cv::Rect(0, 0, 3, 3));
                    t_end2bases_sample[i] = T_base2end(cv::Rect(3, 0, 1, 3));
                }
            }
        }
        cv::Mat camera2hand_rot, camera2hand_tr;
        cv::calibrateHandEye(R_end2bases_sample, t_end2bases_sample,
                        R_maker2cameras_sample, t_maker2cameras_sample,
                        camera2hand_rot, camera2hand_tr, method);

        cv::Mat T_X = cv::Mat::eye(4, 4, CV_64F); // X的齐次形式
            camera2hand_rot.copyTo(T_X(cv::Rect(0, 0, 3, 3)));
            camera2hand_tr.copyTo(T_X(cv::Rect(3, 0, 1, 3)));

        if (eye_on_hand)
        {
            // Eye-in-Hand：T_X 相机→末端
            cv::Mat T_cam2hand = T_X;   
            T_result = T_cam2hand;
        }
        else
        {
            // Eye-to-Hand
            cv::Mat T_cam2base = cv::Mat::eye(4, 4, CV_64F);
            if (bCamInv)
            {
                //eye输入maker2cameras 标定板->相机 已经转换为 相机->标定板
                // 用第一个点计算 相机->基座
                // T_X （标定板->末端）
                cv::Mat T_end2base_current = cv::Mat::eye(4, 4, CV_64F);
                R_end2bases_sample[0].copyTo(T_end2base_current(cv::Rect(0, 0, 3, 3)));
                t_end2bases_sample[0].copyTo(T_end2base_current(cv::Rect(3, 0, 1, 3)));
                cv::Mat T_cam2maker = cv::Mat::eye(4, 4, CV_64F);
                R_maker2cameras_sample[0].copyTo(T_cam2maker(cv::Rect(0, 0, 3, 3)));
                t_maker2cameras_sample[0].copyTo(T_cam2maker(cv::Rect(3, 0, 1, 3)));
                T_cam2base = T_end2base_current * T_X * T_cam2maker;
            }
            else
            {
                //hand输入 末端->基座转换为 基座->末端
                // T_X （相机->基座）
                T_cam2base = T_X;
            }  
            T_result = T_cam2base;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return T_result;
}

// 计算目标->基座的坐标
cv::Mat computeRobotPoseEyeOnHand(const cv::Mat& marker_pose, const cv::Mat& end_pose,
                                          const cv::Mat& camera_to_end_transform) 
{
    cv::Mat T_maker2base;
    try
    {
        T_maker2base = end_pose * camera_to_end_transform * marker_pose;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return T_maker2base;
}

// 计算目标->基座的坐标
cv::Mat computeRobotPoseEyeToHand(const cv::Mat& marker_pose, const cv::Mat& camera_to_base_transform) 
{
    cv::Mat T_maker2base;
    try
    {
        T_maker2base = camera_to_base_transform * marker_pose;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return T_maker2base;
}

std::vector<double> CalculateReprojectionError(const std::vector<cv::Mat>& marker_pose,
    const std::vector<cv::Mat>& end_pose,
    const cv::Mat& calibration_result,
    bool eye_on_hand)
{
    std::vector<double> ar_Err;
    if (marker_pose.size() != end_pose.size())
        return ar_Err;
    double dFirst_x, dFirst_y, dFirst_z;
    int count = marker_pose.size();
    if (eye_on_hand)
    {
        std::vector<double> ar_x(count);
        std::vector<double> ar_y(count);
        std::vector<double> ar_z(count);
        ar_Err.resize(count);
        for (int i = 0; i < count; i++)
        {
            cv::Mat T_maker2base = computeRobotPoseEyeOnHand(marker_pose[i], end_pose[i], calibration_result);
            ar_x[i] = T_maker2base.at<double>(0, 3);
            ar_y[i] = T_maker2base.at<double>(1, 3);
            ar_z[i] = T_maker2base.at<double>(2, 3);
        }
        double sum_x = accumulate(begin(ar_x), end(ar_x), 0.0);  
        double mean_x =  sum_x / ar_x.size();
        double sum_y = accumulate(begin(ar_y), end(ar_y), 0.0);  
        double mean_y =  sum_y / ar_y.size();
        double sum_z = accumulate(begin(ar_z), end(ar_z), 0.0);  
        double mean_z =  sum_z / ar_z.size();
        for (int i = 0; i < count; i++)
        {
            double distan = sqrt((ar_x[i] - mean_x) * (ar_x[i] - mean_x) + 
                                (ar_y[i] - mean_y) * (ar_y[i] - mean_y) + 
                                (ar_z[i] - mean_z) * (ar_z[i] - mean_z));
            ar_Err[i] = distan;
            // std::cout << "基座下的目标坐标: \n" << T_maker2base.t() << std::endl;
        }
    }
    else
    {
        std::vector<double> ar_x(count);
        std::vector<double> ar_y(count);
        std::vector<double> ar_z(count);
        ar_Err.resize(count);
        for (int i = 0; i < count; i++) 
        {
            // 基座下的目标坐标
            cv::Mat T_maker2base = computeRobotPoseEyeToHand(marker_pose[i], calibration_result);
            cv::Mat T_base2hand = end_pose[i].inv();
            cv::Mat T_maker2hand = T_base2hand * T_maker2base;
            ar_x[i] = T_maker2hand.at<double>(0, 3);
            ar_y[i] = T_maker2hand.at<double>(1, 3);
            ar_z[i] = T_maker2hand.at<double>(2, 3);
        }

        double sum_x = accumulate(begin(ar_x), end(ar_x), 0.0);  
        double mean_x =  sum_x / ar_x.size();
        double sum_y = accumulate(begin(ar_y), end(ar_y), 0.0);  
        double mean_y =  sum_y / ar_y.size();
        double sum_z = accumulate(begin(ar_z), end(ar_z), 0.0);  
        double mean_z =  sum_z / ar_z.size();
        for (int i = 0; i < count; i++)
        {
            double distan = sqrt((ar_x[i] - mean_x) * (ar_x[i] - mean_x) + 
                                (ar_y[i] - mean_y) * (ar_y[i] - mean_y) + 
                                (ar_z[i] - mean_z) * (ar_z[i] - mean_z));
            ar_Err[i] = distan;
            // std::cout << "基座下的目标坐标: \n" << T_maker2base.t() << std::endl;
        }
    }

    return ar_Err;
}

////////////////////////////////////////////////////////////////////////
// 结束
////////////////////////////////////////////////////////////////////////
}