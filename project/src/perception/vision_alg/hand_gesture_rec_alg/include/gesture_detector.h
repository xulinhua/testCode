#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <deque>
#include <map>
#include "hand_pose.h"
#include "hand_pipeline.h"

struct GestureInfo {
    std::string gesture_name;
    HandType hand_type;              // 左右手类型
    std::vector<cv::Point2f> keypoints_2d;  // 2D像素坐标（用于可视化）
    std::vector<cv::Point3f> keypoints_3d;  // 3D相机坐标（用于3D识别，单位：毫米）
    std::vector<float> angle_list;
    float palm_angle;
    float palm_facing_angle;
    bool all_fingers_open;
    float confidence;
};

// Wave手势检测状态
struct WaveDetectionState {
    std::deque<float> wrist_x_history;
    std::deque<float> wrist_y_history;
    int change_count;
    float last_wrist_x;
    float last_wrist_y;
};

class GestureDetector {
public:
    GestureDetector();
    ~GestureDetector();

    GestureInfo detect_gesture(const HandResult& hand_result, int image_width, int image_height);

    void set_min_wave_pairs(int min_pairs) { min_wave_pairs_ = min_pairs; }
    int get_min_wave_pairs() const { return min_wave_pairs_; }

    void set_max_wave_y_movement(float max_movement) { max_wave_y_movement_ = max_movement; }
    float get_max_wave_y_movement() const { return max_wave_y_movement_; }

    void set_min_wave_amplitude(float amplitude) { min_wave_amplitude_ = amplitude; }
    float get_min_wave_amplitude() const { return min_wave_amplitude_; }

    void set_use_real_3d(bool use_real_3d) { use_real_3d_ = use_real_3d; }
    bool get_use_real_3d() const { return use_real_3d_; }

    void reset_history();
    void reset_hand_history(HandType hand_type);

    // ============ 使用真实3D坐标的手势识别接口（公开）============
    // 从2D关键点和深度图转换到真实3D坐标（带深度图）
    std::vector<cv::Point3f> convert_to_real_3d(const HandKeypoints& keypoints,
                                               const cv::Mat& depth_frame,
                                               float fx, float fy, float cx, float cy);

    // 3D模式接口（使用真实3D坐标）
    GestureInfo detect_gesture_3d(const HandResult& hand_result,
                                const std::vector<cv::Point3f>& keypoints_3d_real);

    // 判断左右手类型（2D关键点）
    static HandType determine_hand_type_2d(const HandKeypoints& keypoints);

    // 获取手部连接关系（用于可视化）
    static const std::vector<std::pair<int, int>>& get_hand_connections() {
        static const std::vector<std::pair<int, int>> connections = {
            {0, 1}, {1, 2}, {2, 3}, {3, 4},      // 拇指
            {0, 5}, {5, 6}, {6, 7}, {7, 8},      // 食指
            {0, 9}, {9, 10}, {10, 11}, {11, 12}, // 中指
            {0, 13}, {13, 14}, {14, 15}, {15, 16}, // 无名指
            {0, 17}, {17, 18}, {18, 19}, {19, 20}, // 小拇指
            {5, 9}, {9, 13}, {13, 17}              // 手掌
        };
        return connections;
    }

private:
    // 计算手指角度
    std::vector<float> calculate_finger_angles(const HandKeypoints& keypoints);

    // 计算手掌角度
    float calculate_palm_angle(const HandKeypoints& keypoints);

    // 计算手掌朝向角度
    float calculate_palm_facing_angle(const HandKeypoints& keypoints);

    // 判断所有手指是否打开
    bool are_all_fingers_open(const std::vector<float>& angles);

    // 判断单个手指是否伸直
    bool is_finger_straight(float angle) const { return angle < angle_straight_threshold_; }

    // 判断单个手指是否弯曲
    bool is_finger_bent(float angle) const { return angle > angle_bent_threshold_; }

    // 计算二维向量夹角
    float vector_2d_angle(const cv::Point2f& v1, const cv::Point2f& v2);

    // 计算三维向量夹角
    float vector_3d_angle(const cv::Point3f& v1, const cv::Point3f& v2);

    // 主要手势识别函数
    std::string recognize_gesture(const std::vector<float>& angle_list,
                                  float palm_angle,
                                  float palm_facing_angle,
                                  bool all_fingers_open,
                                  HandType hand_type,
                                  const std::vector<cv::Point3f>& keypoints_3d);

    // 检测Wave手势（支持多手）
    bool detect_wave_gesture(HandType hand_type, const std::vector<cv::Point3f>& keypoints_3d, const std::string& current_gesture);

    // ============ 使用真实3D坐标的手势识别接口（私有）============
    // 计算手指角度（使用3D向量）
    std::vector<float> calculate_finger_angles_3d(const std::vector<cv::Point3f>& keypoints_3d);

    // 计算手掌角度（使用3D向量）
    float calculate_palm_angle_3d(const std::vector<cv::Point3f>& keypoints_3d);

    // 计算手掌朝向角度（使用3D法向量）
    float calculate_palm_facing_angle_3d(const std::vector<cv::Point3f>& keypoints_3d);

    // 使用3D坐标进行左右手判断
    HandType determine_hand_type_3d(const std::vector<cv::Point3f>& keypoints_3d);

    // 使用3D坐标识别手势
    std::string recognize_gesture_3d(const std::vector<float>& angle_list,
                                     float palm_angle,
                                     float palm_facing_angle,
                                     bool all_fingers_open,
                                     HandType hand_type,
                                     const std::vector<cv::Point3f>& keypoints_3d);

    // ============ 原2D接口（保持不变）============
    GestureInfo detect_gesture_2d(const HandResult& hand_result, int image_width, int image_height);

    static constexpr float angle_straight_threshold_ = 49.0f;
    static constexpr float angle_bent_threshold_ = 65.0f;
    int min_wave_pairs_;
    float max_wave_y_movement_;  // Y轴最大允许波动范围（像素）
    float min_wave_amplitude_;   // 最小挥手幅度（像素）
    bool use_real_3d_;

    // 手部连接关系
    static const std::vector<std::pair<int, int>> HAND_CONNECTIONS;

    // Wave手势检测状态（支持多手）
    std::map<HandType, WaveDetectionState> wave_states_;
};
