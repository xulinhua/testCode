#pragma once

#include "hand_detector.h"
#include "hand_pose.h"
#include <memory>

enum class HandType {
    UNKNOWN = 0,
    LEFT = 1,
    RIGHT = 2
};

struct HandResult {
    HandDetection bbox;             // 检测框
    HandKeypoints keypoints;        // 21关键点（原图坐标）
    float detectionScore;           // 检测置信度
    HandType handType;              // 左右手类型
};

class HandPipeline {
public:
    HandPipeline();
    ~HandPipeline();

    void load_models(const std::string& det_engine_path,
                     const std::string& pose_engine_path);
    void set_detection_config(const HandDetectionConfig& config);
    void set_pose_config(const HandPoseConfig& config);

    std::vector<HandResult> process(const cv::Mat& image);
    void visualize(cv::Mat& image, const std::vector<HandResult>& results);

    void set_crop_enlargement(float ratio);
    float get_crop_enlargement() const;

private:
    std::unique_ptr<HandDetector> detector_;
    std::unique_ptr<HandPoseEstimator> pose_estimator_;

    cv::Mat crop_hand(const cv::Mat& image, const HandDetection& bbox,
                      cv::Mat& affineMatrix);
};