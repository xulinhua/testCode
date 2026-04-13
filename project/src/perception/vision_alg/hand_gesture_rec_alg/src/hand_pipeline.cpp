#include "hand_pipeline.h"
#include "logger.h"
#include "utils.h"

HandPipeline::HandPipeline() {
    detector_ = std::make_unique<HandDetector>();
    pose_estimator_ = std::make_unique<HandPoseEstimator>();
}

HandPipeline::~HandPipeline() {
    detector_.reset();
    pose_estimator_.reset();
}

void HandPipeline::load_models(const std::string& det_engine_path,
                                const std::string& pose_engine_path) {
    detector_->load_engine(det_engine_path);
    pose_estimator_->load_model(pose_engine_path);
    gLogger.info("手部检测和姿态估计模型加载完成");
}

void HandPipeline::set_detection_config(const HandDetectionConfig& config) {
    detector_->set_config(config);
}

void HandPipeline::set_pose_config(const HandPoseConfig& config) {
    pose_estimator_->set_config(config);
}

void HandPipeline::set_crop_enlargement(float ratio) {
    detector_->set_crop_enlargement(ratio);
}

float HandPipeline::get_crop_enlargement() const {
    return detector_->get_crop_enlargement();
}

cv::Mat HandPipeline::crop_hand(const cv::Mat& image, const HandDetection& bbox,
                                  cv::Mat& affineMatrix) {
    affineMatrix = bbox.affine_matrix;

    cv::Mat cropped;
    const int pose_input_size = 256;  // 默认姿态估计输入尺寸
    cv::warpAffine(image, cropped, affineMatrix, cv::Size(pose_input_size, pose_input_size),
                   cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(128, 128, 128));

    return cropped;
}

// 基于关键点判断左右手

std::vector<HandResult> HandPipeline::process(const cv::Mat& image) {
    std::vector<HandResult> results;

    // 根据配置选择检测方式
    std::vector<HandDetection> bboxes;
    if (detector_->use_gpu_preprocess()) {
        bboxes = detector_->detect_gpu(image);
    } else {
        bboxes = detector_->detect(image);
    }
    if (bboxes.empty()) return results;

    // 对每个手进行关键点检测
    for (const auto& bbox : bboxes) {
        cv::Mat affineMatrix;
        cv::Mat hand_crop = crop_hand(image, bbox, affineMatrix);

        // 推理关键点
        HandKeypoints pose = pose_estimator_->estimate(hand_crop);

        if (pose.valid) {
            // 映射回原图坐标
            auto originalKpts = pose_estimator_->map_to_original(pose, affineMatrix);

            HandResult res;
            res.bbox = bbox;
            res.keypoints.kpts = originalKpts;
            res.keypoints.scores = pose.scores;
            res.keypoints.valid = true;
            res.detectionScore = bbox.score;

            res.handType = HandType::UNKNOWN;

            results.push_back(res);
        }
    }

    return results;
}

void HandPipeline::visualize(cv::Mat& image, const std::vector<HandResult>& results) {
    // 定义骨骼连接（COCO-WholeBody-Hand格式）
    const int skeleton[][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4},       // 大拇指
        {0, 5}, {5, 6}, {6, 7}, {7, 8},       // 食指
        {0, 9}, {9, 10}, {10, 11}, {11, 12},  // 中指
        {0, 13}, {13, 14}, {14, 15}, {15, 16},// 无名指
        {0, 17}, {17, 18}, {18, 19}, {19, 20} // 小指
    };

    for (const auto& res : results) {
        // 绘制检测框
        cv::rectangle(image,
                     cv::Point(res.bbox.x1, res.bbox.y1),
                     cv::Point(res.bbox.x2, res.bbox.y2),
                     cv::Scalar(0, 255, 0), 2);

        // 绘制骨骼
        for (auto& bone : skeleton) {
            cv::line(image,
                    res.keypoints.kpts[bone[0]],
                    res.keypoints.kpts[bone[1]],
                    cv::Scalar(0, 255, 255), 2);
        }

        // 绘制关键点
        for (size_t i = 0; i < res.keypoints.kpts.size(); i++) {
            cv::circle(image, res.keypoints.kpts[i], 3, cv::Scalar(0, 0, 255), -1);
        }

        // 显示置信度
        cv::putText(image,
                   "Hand: " + std::to_string(static_cast<int>(res.detectionScore * 100)) + "%",
                   cv::Point(res.bbox.x1, res.bbox.y1 - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }
}
