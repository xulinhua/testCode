#include "gesture_detector.h"
#include <iostream>
#include <algorithm>
#include <limits>

// 手部连接关系
const std::vector<std::pair<int, int>> GestureDetector::HAND_CONNECTIONS = {
    {0, 1}, {1, 2}, {2, 3}, {3, 4},      // 拇指
    {0, 5}, {5, 6}, {6, 7}, {7, 8},      // 食指
    {0, 9}, {9, 10}, {10, 11}, {11, 12}, // 中指
    {0, 13}, {13, 14}, {14, 15}, {15, 16}, // 无名指
    {0, 17}, {17, 18}, {18, 19}, {19, 20}, // 小拇指
    {5, 9}, {9, 13}, {13, 17}              // 手掌
};

GestureDetector::GestureDetector()
    : min_wave_pairs_(1)
    , max_wave_y_movement_(30.0f)  // Y轴最大允许波动范围（像素）
    , min_wave_amplitude_(30.0f)   // 最小挥手幅度（像素）
    , use_real_3d_(false)
{
    // 初始化左右手的wave检测状态
    wave_states_[HandType::LEFT] = WaveDetectionState{{}, {}, 0, 0.0f, 0.0f};
    wave_states_[HandType::RIGHT] = WaveDetectionState{{}, {}, 0, 0.0f, 0.0f};
}

GestureDetector::~GestureDetector() {
}

float GestureDetector::vector_2d_angle(const cv::Point2f& v1, const cv::Point2f& v2) {
    try {
        float dot = v1.x * v2.x + v1.y * v2.y;
        float norm1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
        float norm2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);
        float cos_val = dot / (norm1 * norm2);
        cos_val = std::max(-1.0f, std::min(1.0f, cos_val));
        float angle = std::acos(cos_val) * 180.0f / M_PI;
        return angle > 180.0f ? 65535.0f : angle;
    } catch (...) {
        return 65535.0f;
    }
}

float GestureDetector::vector_3d_angle(const cv::Point3f& v1, const cv::Point3f& v2) {
    try {
        float dot = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
        float norm1 = std::sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
        float norm2 = std::sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
        float cos_val = dot / (norm1 * norm2);
        cos_val = std::max(-1.0f, std::min(1.0f, cos_val));
        float angle = std::acos(cos_val) * 180.0f / M_PI;
        return angle > 180.0f ? 65535.0f : angle;
    } catch (...) {
        return 65535.0f;
    }
}

std::vector<float> GestureDetector::calculate_finger_angles(const HandKeypoints& keypoints) {
    std::vector<float> angles;
    const auto& kpts = keypoints.kpts;

    // 检查是否有足够的关键点
    if (kpts.size() < 21) {
        return std::vector<float>(5, 65535.0f);
    }

    // 拇指角度
    cv::Point2f thumb_vec0(kpts[0].x - kpts[2].x, kpts[0].y - kpts[2].y);
    cv::Point2f thumb_vec1(kpts[3].x - kpts[4].x, kpts[3].y - kpts[4].y);
    angles.push_back(vector_2d_angle(thumb_vec0, thumb_vec1));

    // 食指角度
    cv::Point2f index_vec0(kpts[0].x - kpts[6].x, kpts[0].y - kpts[6].y);
    cv::Point2f index_vec1(kpts[7].x - kpts[8].x, kpts[7].y - kpts[8].y);
    angles.push_back(vector_2d_angle(index_vec0, index_vec1));

    // 中指角度
    cv::Point2f middle_vec0(kpts[0].x - kpts[10].x, kpts[0].y - kpts[10].y);
    cv::Point2f middle_vec1(kpts[11].x - kpts[12].x, kpts[11].y - kpts[12].y);
    angles.push_back(vector_2d_angle(middle_vec0, middle_vec1));

    // 无名指角度
    cv::Point2f ring_vec0(kpts[0].x - kpts[14].x, kpts[0].y - kpts[14].y);
    cv::Point2f ring_vec1(kpts[15].x - kpts[16].x, kpts[15].y - kpts[16].y);
    angles.push_back(vector_2d_angle(ring_vec0, ring_vec1));

    // 小拇指角度
    cv::Point2f pinky_vec0(kpts[0].x - kpts[18].x, kpts[0].y - kpts[18].y);
    cv::Point2f pinky_vec1(kpts[19].x - kpts[20].x, kpts[19].y - kpts[20].y);
    angles.push_back(vector_2d_angle(pinky_vec0, pinky_vec1));

    return angles;
}

float GestureDetector::calculate_palm_angle(const HandKeypoints& keypoints) {
    const auto& kpts = keypoints.kpts;
    if (kpts.size() < 21) return 0.0f;

    cv::Point2f palm_vec(kpts[9].x - kpts[0].x, kpts[9].y - kpts[0].y);
    // 检查向量是否为零向量，避免 atan2(0,0) 未定义
    float xy_length_sq = palm_vec.x * palm_vec.x + palm_vec.y * palm_vec.y;
    if (xy_length_sq < 1e-6f) {
        return 0.0f;  // 在XY平面上无方向，返回默认角度
    }
    float angle = std::atan2(palm_vec.y, palm_vec.x) * 180.0f / M_PI;
    if (angle < 0) angle += 360.0f;
    return angle;
}

float GestureDetector::calculate_palm_facing_angle(const HandKeypoints& keypoints) {
    const auto& kpts = keypoints.kpts;
    if (kpts.size() < 21) return 90.0f;

    // 计算拇指和小拇指指尖的x坐标距离
    float hand_width_2d = std::abs(kpts[4].x - kpts[20].x);
    // 计算手腕到中指尖的y距离
    float hand_height_2d = std::abs(kpts[0].y - kpts[12].y);

    // 归一化的宽高比
    float palm_facing_ratio = hand_height_2d > 0 ? hand_width_2d / hand_height_2d : 0;

    // 转换为角度
    float palm_facing_angle_degrees = std::max(0.0f, 90.0f - palm_facing_ratio * 90.0f);

    return palm_facing_angle_degrees;
}

bool GestureDetector::are_all_fingers_open(const std::vector<float>& angles) {
    for (float angle : angles) {
        if (angle >= angle_straight_threshold_) {
            return false;
        }
    }
    return true;
}

bool GestureDetector::detect_wave_gesture(HandType hand_type, const std::vector<cv::Point3f>& keypoints_3d, const std::string& current_gesture) {
    if (keypoints_3d.size() < 1) return false;

    // 获取对应手的状态
    if (wave_states_.find(hand_type) == wave_states_.end()) {
        wave_states_[hand_type] = WaveDetectionState{{}, {}, 0, 0.0f, 0.0f};
    }

    auto& state = wave_states_[hand_type];
    float wrist_x = keypoints_3d[0].x;
    float wrist_y = keypoints_3d[0].y;
    float middle_finger_y = keypoints_3d.size() >= 13 ? keypoints_3d[12].y : wrist_y;

    // 只有当前手势为Stop或Handshake时才记录轨迹，防止其他手势干扰
    // 同时要求手腕y值必须大于中指y值
    if ((current_gesture != "Stop" && current_gesture != "Handshake") || (wrist_y < middle_finger_y)) {
        // 清空历史记录
        state.wrist_x_history.clear();
        state.wrist_y_history.clear();
        return false;
    }

    // 添加当前位置到历史记录
    state.wrist_x_history.push_back(wrist_x);
    state.wrist_y_history.push_back(wrist_y);
    int max_history_size = 30;
    int min_history_size = 20;
    if(use_real_3d_)
    {
        max_history_size = 20;
        min_history_size = 15;
    }
    if (state.wrist_x_history.size() > max_history_size) {
        state.wrist_x_history.pop_front();
        state.wrist_y_history.pop_front();
    }

    if (state.wrist_x_history.size() < min_history_size) {
        return false;
    }

    const auto& x_history = state.wrist_x_history;
    const auto& y_history = state.wrist_y_history;

    // 计算X轴移动范围
    float x_min_val = *std::min_element(x_history.begin(), x_history.end());
    float x_max_val = *std::max_element(x_history.begin(), x_history.end());
    float x_range = x_max_val - x_min_val;

    // 计算Y轴移动范围
    float y_min_val = *std::min_element(y_history.begin(), y_history.end());
    float y_max_val = *std::max_element(y_history.begin(), y_history.end());
    float y_range = y_max_val - y_min_val;

    // 如果X轴总移动范围太小，不可能是挥手
    if (x_range < min_wave_amplitude_) {
        return false;
    }

    // 如果Y轴波动范围太大，不是挥手（可能在上下移动或斜向移动）
    if (y_range > max_wave_y_movement_) {
        return false;
    }

    // 分析轨迹中的峰值和谷值
    bool is_wave = false;
    std::vector<size_t> peaks;
    std::vector<size_t> valleys;

    // 寻找局部极值点（峰值和谷值）
    for (size_t i = 2; i < x_history.size() - 2; i++) {
        float curr = x_history[i];
        float prev2 = x_history[i-2];
        float prev1 = x_history[i-1];
        float next1 = x_history[i+1];
        float next2 = x_history[i+2];

        // 检测峰值（当前点不小于前后任意点，且至少大于一个）
        bool is_peak = (curr >= prev2 && curr >= prev1 && curr >= next1 && curr >= next2) &&
                       (curr > prev2 || curr > prev1 || curr > next1 || curr > next2);
        // 检测谷值（当前点不大于前后任意点，且至少小于一个）
        bool is_valley = (curr <= prev2 && curr <= prev1 && curr <= next1 && curr <= next2) &&
                         (curr < prev2 || curr < prev1 || curr < next1 || curr < next2);

        if (is_peak) {
            peaks.push_back(i);
        }
        else if (is_valley) {
            valleys.push_back(i);
        }
    }

    // 去重：移除连续相等的峰值或谷值，只保留第一个
    std::vector<size_t> deduped_peaks, deduped_valleys;
    for (size_t i = 0; i < peaks.size(); i++) {
        if (i == 0 || peaks[i] != peaks[i-1] + 1 || x_history[peaks[i]] != x_history[peaks[i-1]]) {
            deduped_peaks.push_back(peaks[i]);
        }
    }
    for (size_t i = 0; i < valleys.size(); i++) {
        if (i == 0 || valleys[i] != valleys[i-1] + 1 || x_history[valleys[i]] != x_history[valleys[i-1]]) {
            deduped_valleys.push_back(valleys[i]);
        }
    }
    peaks = deduped_peaks;
    valleys = deduped_valleys;

    // 检查是否有峰谷模式（挥手特征）
    if (!peaks.empty() && !valleys.empty()) {
        // 检查峰值和谷值是否交替出现
        bool alternating = true;
        std::vector<std::pair<size_t, char>> all_extremes;

        for (size_t p : peaks) all_extremes.push_back({p, 'P'});
        for (size_t v : valleys) all_extremes.push_back({v, 'V'});
        std::sort(all_extremes.begin(), all_extremes.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        for (size_t i = 1; i < all_extremes.size(); i++) {
            if (all_extremes[i].second == all_extremes[i-1].second) {
                alternating = false;
                break;
            }
        }

        // 检查交替的峰值和谷值之间的差值，统计满足阈值的对数
        int valid_pairs = 0;
        for (size_t i = 1; i < all_extremes.size(); i++) {
            size_t prev_idx = all_extremes[i-1].first;
            size_t curr_idx = all_extremes[i].first;
            float diff = std::abs(x_history[curr_idx] - x_history[prev_idx]);
            if (diff >= min_wave_amplitude_) {
                valid_pairs++;
            }
        }

        // 如果交替且满足阈值的峰谷对数达到要求
        if (alternating && valid_pairs >= min_wave_pairs_) {
            is_wave = true;
        }
    }

    if (is_wave) {
        state.wrist_x_history.clear();
        state.wrist_y_history.clear();
        state.change_count = 0;
        return true;
    }

    return false;
}

std::string GestureDetector::recognize_gesture(const std::vector<float>& angle_list,
                                                float palm_angle,
                                                float palm_facing_angle,
                                                bool all_fingers_open,
                                                HandType hand_type,
                                                const std::vector<cv::Point3f>& keypoints_3d) {

    // 检查是否有无效角度
    bool has_invalid_angle = false;
    for (float angle : angle_list) {
        if (angle >= 65535.0f) {
            has_invalid_angle = true;
            break;
        }
    }

    if (has_invalid_angle) {
        return "";
    }

    // 判断手指状态
    bool thumb_straight = is_finger_straight(angle_list[0]);
    bool index_straight = is_finger_straight(angle_list[1]);
    bool middle_straight = is_finger_straight(angle_list[2]);
    bool ring_straight = is_finger_straight(angle_list[3]);
    bool pinky_straight = is_finger_straight(angle_list[4]);

    bool thumb_bent = is_finger_bent(angle_list[0]);
    bool index_bent = is_finger_bent(angle_list[1]);
    bool middle_bent = is_finger_bent(angle_list[2]);
    bool ring_bent = is_finger_bent(angle_list[3]);
    bool pinky_bent = is_finger_bent(angle_list[4]);

    // 其他手势判断
    // Fist: 所有手指弯曲
    if (thumb_bent && index_bent && middle_bent && ring_bent && pinky_bent) {
        return "fist";
    }

    // One: 只有食指伸直
    if (index_straight && thumb_bent && middle_bent && ring_bent && pinky_bent) {
        return "one";
    }

    // Peace: 食指和中指伸直
    if (index_straight && middle_straight && thumb_bent && ring_bent && pinky_bent) {
        return "peace";
    }

    // Three: 拇指、食指、中指伸直
    if (thumb_straight && index_straight && middle_straight && ring_bent && pinky_bent) {
        return "three";
    }

    // Three2: 食指、中指、无名指伸直
    if (index_straight && middle_straight && ring_straight && thumb_bent && pinky_bent) {
        return "three2";
    }

    // Four: 食指、中指、无名指、小拇指伸直
    if (thumb_bent && index_straight && middle_straight && ring_straight && pinky_straight) {
        return "four";
    }

    // ILY: 拇指、食指、小拇指伸直
    if (thumb_straight && index_straight && pinky_straight && middle_bent && ring_bent) {
        return "ILY";
    }

    // OK: 食指弯曲，其他伸直
    if (index_bent && middle_straight && ring_straight && pinky_straight) {
        return "ok";
    }

    // Call: 拇指和小拇指伸直
    if (thumb_straight && pinky_straight && index_bent && middle_bent && ring_bent) {
        return "call";
    }

    // Insult: 中指伸直，手掌侧对
    if (middle_straight && thumb_bent && index_bent && ring_bent && pinky_bent && palm_facing_angle > 90.0f) {
        return "Insult";
    }

    // Heart_single/like/dislike: 拇指伸直，食指弯曲
    if (thumb_straight && index_bent && middle_bent && ring_bent && pinky_bent) {
        if (keypoints_3d.size() >= 8) {
            cv::Point3f thumb_ip(keypoints_3d[3]);
            cv::Point3f index_dip(keypoints_3d[7]);
            float dist = std::sqrt(std::pow(thumb_ip.x - index_dip.x, 2) +
                                   std::pow(thumb_ip.y - index_dip.y, 2));
            if (dist < 50.0f) {
                return "Heart_single";
            }
            if (thumb_ip.y < keypoints_3d[0].y) {
                return "like";
            }
        }
        return "dislike";
    }

    // Stop/Handshake: 所有手指打开，手掌朝向不同
    if (all_fingers_open) {
        if (palm_facing_angle < 30.0f) {
            return "Stop";
        } else {
            return "Handshake";
        }
    }

    return "";
}

GestureInfo GestureDetector::detect_gesture(const HandResult& hand_result, int image_width, int image_height) {
    // 默认使用2D模式
    return detect_gesture_2d(hand_result, image_width, image_height);
}

// ============ 2D模式的手势识别（保持原有逻辑不变）============
GestureInfo GestureDetector::detect_gesture_2d(const HandResult& hand_result, int /* image_width */, int /* image_height */) {
    GestureInfo info;

    // 判断左右手类型
    info.hand_type = determine_hand_type_2d(hand_result.keypoints);

    // 存储2D像素坐标（用于可视化）
    info.keypoints_2d = hand_result.keypoints.kpts;

    // 转换关键点到3D坐标（z值存储置信度，非真实深度）
    for (size_t i = 0; i < hand_result.keypoints.kpts.size() && i < hand_result.keypoints.scores.size(); i++) {
        info.keypoints_3d.push_back(cv::Point3f(hand_result.keypoints.kpts[i].x,
                                                hand_result.keypoints.kpts[i].y,
                                                hand_result.keypoints.scores[i]));
    }

    // 计算手指角度
    info.angle_list = calculate_finger_angles(hand_result.keypoints);

    // 计算手掌角度
    info.palm_angle = calculate_palm_angle(hand_result.keypoints);

    // 计算手掌朝向角度
    info.palm_facing_angle = calculate_palm_facing_angle(hand_result.keypoints);

    // 判断所有手指是否打开
    info.all_fingers_open = are_all_fingers_open(info.angle_list);

    // 识别手势
    std::string current_gesture = recognize_gesture(info.angle_list, info.palm_angle,
                                                   info.palm_facing_angle,
                                                   info.all_fingers_open, hand_result.handType, info.keypoints_3d);

    // 检测Wave手势
    if (info.all_fingers_open && detect_wave_gesture(hand_result.handType, info.keypoints_3d, current_gesture)) {
        info.gesture_name = "Wave";
    } else {
        info.gesture_name = current_gesture;
    }

    info.confidence = hand_result.detectionScore;

    return info;
}

// ============ 3D模式的手势识别（使用真实3D坐标）============
GestureInfo GestureDetector::detect_gesture_3d(const HandResult& hand_result,
                                              const std::vector<cv::Point3f>& keypoints_3d_real) {
    GestureInfo info;

    // 记录手类型
    info.hand_type = hand_result.handType;

    // 存储2D像素坐标（用于可视化）
    info.keypoints_2d = hand_result.keypoints.kpts;

    // 存储3D坐标（用于3D识别）
    info.keypoints_3d = keypoints_3d_real;

    // 使用3D坐标重新判断左右手（可选，因为hand_result.handType已经判断过）
    info.hand_type = determine_hand_type_3d(keypoints_3d_real);

    // 计算手指角度（使用3D向量）
    info.angle_list = calculate_finger_angles_3d(keypoints_3d_real);

    // 计算手掌角度（使用3D向量）
    info.palm_angle = calculate_palm_angle_3d(keypoints_3d_real);

    // 计算手掌朝向角度（使用3D法向量）
    info.palm_facing_angle = calculate_palm_facing_angle_3d(keypoints_3d_real);

    // 判断所有手指是否打开
    info.all_fingers_open = are_all_fingers_open(info.angle_list);

    // 使用3D坐标识别手势
    std::string current_gesture = recognize_gesture_3d(info.angle_list, info.palm_angle,
                                                      info.palm_facing_angle,
                                                      info.all_fingers_open, info.hand_type,
                                                      keypoints_3d_real);

    // 检测Wave手势
    if (info.all_fingers_open && detect_wave_gesture(info.hand_type, keypoints_3d_real, current_gesture)) {
        info.gesture_name = "Wave";
    } else {
        info.gesture_name = current_gesture;
    }

    info.confidence = hand_result.detectionScore;

    return info;
}

// ============ 真实3D坐标转换接口 ============
// 从2D关键点和深度图转换到真实3D坐标
std::vector<cv::Point3f> GestureDetector::convert_to_real_3d(const HandKeypoints& keypoints,
                                                             const cv::Mat& depth_frame,
                                                             float fx, float fy, float cx, float cy) {
    std::vector<cv::Point3f> keypoints_3d;

    if (keypoints.kpts.size() != keypoints.scores.size() || depth_frame.empty()) {
        return keypoints_3d;
    }

    for (size_t i = 0; i < keypoints.kpts.size(); i++) {
        float x = keypoints.kpts[i].x;
        float y = keypoints.kpts[i].y;

        // 从深度图获取真实深度
        int px = static_cast<int>(x);
        int py = static_cast<int>(y);

        // 检查像素坐标是否在深度图范围内
        if (px >= 0 && px < depth_frame.cols && py >= 0 && py < depth_frame.rows) {
            uint16_t depth_val = depth_frame.at<uint16_t>(py, px);
            if (depth_val > 0) {
                // 计算3D坐标（相机坐标系）
                float x_3d = (x - cx) * depth_val / fx;
                float y_3d = (y - cy) * depth_val / fy;
                float z_3d = static_cast<float>(depth_val);
                keypoints_3d.push_back(cv::Point3f(x_3d, y_3d, z_3d));
            } else {
                // 深度值为0，跳过该关键点
                keypoints_3d.push_back(cv::Point3f(0, 0, 0));
            }
        }
    }

    return keypoints_3d;
}

void GestureDetector::reset_history() {
    for (auto& [hand_type, state] : wave_states_) {
        state.wrist_x_history.clear();
        state.wrist_y_history.clear();
        state.change_count = 0;
        state.last_wrist_x = 0.0f;
        state.last_wrist_y = 0.0f;
    }
}

void GestureDetector::reset_hand_history(HandType hand_type) {
    if (wave_states_.find(hand_type) != wave_states_.end()) {
        wave_states_[hand_type].wrist_x_history.clear();
        wave_states_[hand_type].wrist_y_history.clear();
        wave_states_[hand_type].change_count = 0;
        wave_states_[hand_type].last_wrist_x = 0.0f;
        wave_states_[hand_type].last_wrist_y = 0.0f;
    }
}

// ============ 使用真实3D坐标的手势识别实现 ============

// 计算3D手指角度
std::vector<float> GestureDetector::calculate_finger_angles_3d(const std::vector<cv::Point3f>& keypoints_3d) {
    std::vector<float> angles;

    if (keypoints_3d.size() < 21) {
        return std::vector<float>(5, 65535.0f);
    }

    // 拇指角度（使用3D向量）
    cv::Point3f thumb_vec0(keypoints_3d[0].x - keypoints_3d[2].x,
                           keypoints_3d[0].y - keypoints_3d[2].y,
                           keypoints_3d[0].z - keypoints_3d[2].z);
    cv::Point3f thumb_vec1(keypoints_3d[3].x - keypoints_3d[4].x,
                           keypoints_3d[3].y - keypoints_3d[4].y,
                           keypoints_3d[3].z - keypoints_3d[4].z);
    angles.push_back(vector_3d_angle(thumb_vec0, thumb_vec1));

    // 食指角度
    cv::Point3f index_vec0(keypoints_3d[0].x - keypoints_3d[6].x,
                           keypoints_3d[0].y - keypoints_3d[6].y,
                           keypoints_3d[0].z - keypoints_3d[6].z);
    cv::Point3f index_vec1(keypoints_3d[7].x - keypoints_3d[8].x,
                           keypoints_3d[7].y - keypoints_3d[8].y,
                           keypoints_3d[7].z - keypoints_3d[8].z);
    angles.push_back(vector_3d_angle(index_vec0, index_vec1));

    // 中指角度
    cv::Point3f middle_vec0(keypoints_3d[0].x - keypoints_3d[10].x,
                            keypoints_3d[0].y - keypoints_3d[10].y,
                            keypoints_3d[0].z - keypoints_3d[10].z);
    cv::Point3f middle_vec1(keypoints_3d[11].x - keypoints_3d[12].x,
                            keypoints_3d[11].y - keypoints_3d[12].y,
                            keypoints_3d[11].z - keypoints_3d[12].z);
    angles.push_back(vector_3d_angle(middle_vec0, middle_vec1));

    // 无名指角度
    cv::Point3f ring_vec0(keypoints_3d[0].x - keypoints_3d[14].x,
                          keypoints_3d[0].y - keypoints_3d[14].y,
                          keypoints_3d[0].z - keypoints_3d[14].z);
    cv::Point3f ring_vec1(keypoints_3d[15].x - keypoints_3d[16].x,
                          keypoints_3d[15].y - keypoints_3d[16].y,
                          keypoints_3d[15].z - keypoints_3d[16].z);
    angles.push_back(vector_3d_angle(ring_vec0, ring_vec1));

    // 小拇指角度
    cv::Point3f pinky_vec0(keypoints_3d[0].x - keypoints_3d[18].x,
                           keypoints_3d[0].y - keypoints_3d[18].y,
                           keypoints_3d[0].z - keypoints_3d[18].z);
    cv::Point3f pinky_vec1(keypoints_3d[19].x - keypoints_3d[20].x,
                           keypoints_3d[19].y - keypoints_3d[20].y,
                           keypoints_3d[19].z - keypoints_3d[20].z);
    angles.push_back(vector_3d_angle(pinky_vec0, pinky_vec1));

    return angles;
}

// 计算3D手掌角度（使用手掌平面的法向量）
float GestureDetector::calculate_palm_angle_3d(const std::vector<cv::Point3f>& keypoints_3d) {
    if (keypoints_3d.size() < 21) return 0.0f;

    // 使用手腕、中指根部、中指尖计算手掌朝向
    cv::Point3f wrist_to_middle(keypoints_3d[9].x - keypoints_3d[0].x,
                                keypoints_3d[9].y - keypoints_3d[0].y,
                                keypoints_3d[9].z - keypoints_3d[0].z);

    // 检查向量是否为零向量，避免 atan2(0,0) 未定义
    float xy_length_sq = wrist_to_middle.x * wrist_to_middle.x + wrist_to_middle.y * wrist_to_middle.y;
    if (xy_length_sq < 1e-6f) {
        return 0.0f;  // 在XY平面上无方向，返回默认角度
    }
    // 在XY平面上的投影角度
    float angle = std::atan2(wrist_to_middle.y, wrist_to_middle.x) * 180.0f / M_PI;
    if (angle < 0) angle += 360.0f;
    return angle;
}

// 计算3D手掌朝向角度（使用法向量判断手心/手背）
float GestureDetector::calculate_palm_facing_angle_3d(const std::vector<cv::Point3f>& keypoints_3d) {
    if (keypoints_3d.size() < 21) return 90.0f;

    // 计算拇指和小拇指指尖的x坐标距离
    float hand_width_3d = std::abs(keypoints_3d[4].x - keypoints_3d[20].x);
    // 计算手腕到中指尖的y距离
    float hand_height_3d = std::abs(keypoints_3d[0].y - keypoints_3d[12].y);

    // 归一化的宽高比
    float palm_facing_ratio = hand_height_3d > 0 ? hand_width_3d / hand_height_3d : 0;

    // 转换为角度
    float palm_facing_angle_degrees = std::max(0.0f, 90.0f - palm_facing_ratio * 90.0f);

    return palm_facing_angle_degrees;
}

// 使用3D坐标判断左右手
HandType GestureDetector::determine_hand_type_3d(const std::vector<cv::Point3f>& k) {
    if (k.size() < 21) {
        return HandType::UNKNOWN;
    }

    // 方法：判断拇指相对于中指的左右位置
    // 中指尖点 (12) 和 拇指尖点 (4)
    float middle_tip_x = k[12].x;
    float thumb_tip_x = k[4].x;

    // 拇指在中指左侧 -> 左手
    // 拇指在中指右侧 -> 右手
    if (thumb_tip_x < middle_tip_x - 0.01f) {
        return HandType::LEFT;
    } else if (thumb_tip_x > middle_tip_x + 0.01f) {
        return HandType::RIGHT;
    }

    return HandType::UNKNOWN;
}

// 使用3D坐标识别手势
std::string GestureDetector::recognize_gesture_3d(const std::vector<float>& angle_list,
                                                 float palm_angle,
                                                 float palm_facing_angle,
                                                 bool all_fingers_open,
                                                 HandType hand_type,
                                                 const std::vector<cv::Point3f>& keypoints_3d) {
    // 检查是否有无效角度
    bool has_invalid_angle = false;
    for (float angle : angle_list) {
        if (angle >= 65535.0f) {
            has_invalid_angle = true;
            break;
        }
    }

    if (has_invalid_angle) {
        return "";
    }

    // 判断手指状态
    bool thumb_straight = is_finger_straight(angle_list[0]);
    bool index_straight = is_finger_straight(angle_list[1]);
    bool middle_straight = is_finger_straight(angle_list[2]);
    bool ring_straight = is_finger_straight(angle_list[3]);
    bool pinky_straight = is_finger_straight(angle_list[4]);

    bool thumb_bent = is_finger_bent(angle_list[0]);
    bool index_bent = is_finger_bent(angle_list[1]);
    bool middle_bent = is_finger_bent(angle_list[2]);
    bool ring_bent = is_finger_bent(angle_list[3]);
    bool pinky_bent = is_finger_bent(angle_list[4]);

    // 手势判断逻辑（与2D版本相同，但使用3D角度更准确）
    // Fist: 所有手指弯曲
    if (thumb_bent && index_bent && middle_bent && ring_bent && pinky_bent) {
        return "fist";
    }

    // One: 只有食指伸直
    if (index_straight && thumb_bent && middle_bent && ring_bent && pinky_bent) {
        return "one";
    }

    // Peace: 食指和中指伸直
    if (index_straight && middle_straight && thumb_bent && ring_bent && pinky_bent) {
        return "peace";
    }

    // Three: 拇指、食指、中指伸直
    if (thumb_straight && index_straight && middle_straight && ring_bent && pinky_bent) {
        return "three";
    }

    // Three2: 食指、中指、无名指伸直
    if (index_straight && middle_straight && ring_straight && thumb_bent && pinky_bent) {
        return "three2";
    }

    // Four: 食指、中指、无名指、小拇指伸直
    if (thumb_bent && index_straight && middle_straight && ring_straight && pinky_straight) {
        return "four";
    }

    // ILY: 拇指、食指、小拇指伸直
    if (thumb_straight && index_straight && pinky_straight && middle_bent && ring_bent) {
        return "ILY";
    }

    // OK: 食指弯曲，其他伸直
    if (index_bent && middle_straight && ring_straight && pinky_straight) {
        return "ok";
    }

    // Call: 拇指和小拇指伸直
    if (thumb_straight && pinky_straight && index_bent && middle_bent && ring_bent) {
        return "call";
    }

    // Insult: 中指伸直，手掌侧对（使用3D的palm_facing_angle更准确）
    if (middle_straight && thumb_bent && index_bent && ring_bent && pinky_bent && palm_facing_angle > 90.0f) {
        return "Insult";
    }

    // Heart_single/like/dislike: 拇指伸直，食指弯曲
    if (thumb_straight && index_bent && middle_bent && ring_bent && pinky_bent) {
        if (keypoints_3d.size() >= 8) {
            cv::Point3f thumb_ip(keypoints_3d[3]);
            cv::Point3f index_dip(keypoints_3d[7]);
            float dist = std::sqrt(std::pow(thumb_ip.x - index_dip.x, 2) +
                                   std::pow(thumb_ip.y - index_dip.y, 2) +
                                   std::pow(thumb_ip.z - index_dip.z, 2));  // 3D距离
            if (dist < 50.0f) {
                return "Heart_single";
            }
            if (thumb_ip.y < keypoints_3d[0].y) {
                return "like";
            }
        }
        return "dislike";
    }

    // Stop/Handshake: 所有手指打开，使用3D palm_facing_angle判断
    if (all_fingers_open) {
        if (palm_facing_angle < 30.0f) {
            return "Stop";
        } else {
            return "Handshake";
        }
    }

    return "";
}

// 判断左右手类型（2D关键点）
HandType GestureDetector::determine_hand_type_2d(const HandKeypoints& keypoints) {
    if (keypoints.kpts.size() < 21) {
        return HandType::UNKNOWN;
    }

    const auto& kpts = keypoints.kpts;

    // 方法1：判断拇指相对于中指的左右位置
    // 中指尖点 (12) 和 拇指尖点 (4)
    float middle_tip_x = kpts[12].x;
    float thumb_tip_x = kpts[4].x;

    // 拇指在中指左侧 -> 左手
    // 拇指在中指右侧 -> 右手
    if (thumb_tip_x < middle_tip_x - 10.0f) {
        return HandType::LEFT;
    } else if (thumb_tip_x > middle_tip_x + 10.0f) {
        return HandType::RIGHT;
    }

    return HandType::UNKNOWN;
}
