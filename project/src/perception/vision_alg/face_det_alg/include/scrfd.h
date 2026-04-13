#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <vector>
#include <string>
#include <stdexcept>
#include <memory>
#include "NvInfer.h"
#include "NvInferRuntime.h"
#include "logger.h"
#include "exception.h"
#include "process.h"

struct FaceDetection {
    cv::Rect bbox;
    std::vector<cv::Point2f> landmarks;  // 5个关键点
    float score;
};

struct FaceDetectionConfig {
    float conf_thresh = 0.5f;        // 置信度阈值
    float nms_thresh = 0.4f;         // NMS阈值
    float landmark_std = 0.2f;       // 关键点标准差
    bool use_gpu_preprocess = true;  // 是否使用GPU加速预处理
    bool use_gpu_postprocess = false; // 是否使用GPU加速后处理
};

struct Anchor {
    float cx, cy, w, h;
};

struct Intrinsics
{
    float fx;
    float fy;
    float cx;
    float cy;
};

class SCRFD {
public:
    SCRFD();
    ~SCRFD();
    void load_engine(const std::string& engine_path);
    void set_config(const FaceDetectionConfig& config);
    std::vector<FaceDetection> detect(const cv::Mat& img);
    void draw_results(cv::Mat& img,
                     const std::vector<FaceDetection>& detections,
                     bool draw_bbox = true,
                     bool draw_landmarks = true,
                     const cv::Scalar& color = cv::Scalar(0, 255, 0));

    void draw_results(cv::Mat& img,
                        const std::vector<FaceDetection>& detections,
                        cv::Mat &depth_img,
                        const Intrinsics &intrinsics,
                        bool draw_bbox,
                        bool draw_landmarks,
                        const cv::Scalar& color = cv::Scalar(0, 255, 0));

    // 判断人脸是否正对着相机
    // 返回值: -1.0 (左侧), 0.0-1.0 (正面), >1.0 (右侧)
    // 值越接近0表示越正面，绝对值越大表示越侧脸
    float is_facing_camera(const FaceDetection& detection);

    // 判断人脸朝向程度（基于关键点）
    // 返回值: 0.0-1.0，值越小越正面，值越大越侧脸
    // 可用于过滤掉严重侧脸的情况
    float get_face_pose_degree(const FaceDetection& detection);

    // 判断是否为正脸（用于过滤侧脸）
    // threshold: 阈值，默认0.3。当 pose_degree < threshold 时认为是正脸
    // 返回值: true=正脸, false=侧脸
    bool is_frontal_face(const FaceDetection& detection, float threshold = 0.3f); 
private:
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;

    std::vector<void*> buffers_;
    std::vector<std::string> tensor_names_;
    std::vector<bool> is_input_;
    cudaStream_t stream_ = nullptr;
    Logger gLogger;

    int input_index_ = -1;
    int input_w_ = 0;
    int input_h_ = 0;
    int input_c_ = 3;
    int batch_size_ = 1;

    float ratio_ = 1.0f;
    float pad_w_ = 0.0f;
    float pad_h_ = 0.0f;
    int active_profile_index_ = 0;

    std::vector<float> cpu_output_score8_;
    std::vector<float> cpu_output_score16_;
    std::vector<float> cpu_output_score32_;
    std::vector<float> cpu_output_bbox8_;
    std::vector<float> cpu_output_bbox16_;
    std::vector<float> cpu_output_bbox32_;
    std::vector<float> cpu_output_kps8_;
    std::vector<float> cpu_output_kps16_;
    std::vector<float> cpu_output_kps32_;
    bool has_kps_output_;  // 是否有关键点输出

    FaceDetectionConfig config_;

    void prepare_buffers();
    void preprocess(const cv::Mat& img);
    void preprocess_gpu(const cv::Mat& img);  // GPU加速预处理
    void postprocess(const std::vector<float>& score_8,
                    const std::vector<float>& score_16,
                    const std::vector<float>& score_32,
                    const std::vector<float>& bbox_8,
                    const std::vector<float>& bbox_16,
                    const std::vector<float>& bbox_32,
                    std::vector<FaceDetection>& detections,
                    int img_width, int img_height);
    void postprocess(const std::vector<float>& score_8,
                    const std::vector<float>& score_16,
                    const std::vector<float>& score_32,
                    const std::vector<float>& bbox_8,
                    const std::vector<float>& bbox_16,
                    const std::vector<float>& bbox_32,
                    const std::vector<float>& kps_8,
                    const std::vector<float>& kps_16,
                    const std::vector<float>& kps_32,
                    std::vector<FaceDetection>& detections,
                    int img_width, int img_height);
    void postprocess_gpu(std::vector<FaceDetection>& detections,
                         int img_width, int img_height);  // GPU加速后处理
    std::vector<FaceDetection> inference(const cv::Mat& img);
    void setup_dynamic_shapes();

    struct BoxInfo {
        float x1, y1, x2, y2;
        float score;
        std::vector<float> landmark;
    };
    static bool cmp_score(const BoxInfo& a, const BoxInfo& b);
    std::vector<BoxInfo> nms_sorted_bboxes(const std::vector<BoxInfo>& bboxes, float nms_thresh);

    // GPU相关成员
    uchar3* d_src_ = nullptr;         // GPU原图缓存 (BGR)
    size_t d_src_size_ = 0;           // 已分配字节数
    Bbox32* d_boxes_ = nullptr;       // GPU框缓存
    int* d_counts_ = nullptr;         // GPU计数缓存 (每个stride一个)
    uint8_t* d_nms_mask_ = nullptr;   // NMS掩码矩阵
    int max_boxes_per_stride_;        // 每个stride最大框数
};