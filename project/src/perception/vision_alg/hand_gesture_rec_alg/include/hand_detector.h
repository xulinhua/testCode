#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include "NvInfer.h"
#include "logger.h"
#include "exception.h"

struct HandDetection {
    float x1, y1, x2, y2;
    float score;
    int class_id;
    cv::Mat affine_matrix;  // 用于关键点检测的裁剪变换矩阵
};

struct HandDetectionConfig {
    int input_size = 320;
    float conf_thresh = 0.3f;
    float nms_thresh = 0.2f;
    float crop_enlargement = 1.25f;
    bool use_gpu_preprocess = true;
};

class HandDetector {
public:
    HandDetector();
    ~HandDetector();

    void load_engine(const std::string& engine_path);
    void set_config(const HandDetectionConfig& config);

    std::vector<HandDetection> detect(const cv::Mat& img);
    std::vector<HandDetection> detect_gpu(const cv::Mat& img);

    void set_crop_enlargement(float ratio) { config_.crop_enlargement = ratio; }
    float get_crop_enlargement() const { return config_.crop_enlargement; }
    bool use_gpu_preprocess() const { return config_.use_gpu_preprocess; }

private:
    // TensorRT 相关
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    cudaStream_t stream_ = nullptr;
    Logger gLogger_;

    // 输入输出信息
    int input_index_ = -1;
    std::vector<int> output_indices_;
    std::string input_name_;
    std::vector<std::string> output_names_;
    int input_w_ = 0;
    int input_h_ = 0;
    int input_c_ = 3;

    // GPU 缓冲区
    void* input_buffer_ = nullptr;
    std::vector<void*> output_buffers_;
    std::vector<size_t> output_sizes_;
    uchar3* d_src_ = nullptr;  // GPU 输入图像数据
    float* d_mean_ = nullptr;  // GPU 均值
    float* d_std_ = nullptr;   // GPU 标准差

    // 预处理参数
    float ratio_ = 1.0f;
    float dw_ = 0.0f;
    float dh_ = 0.0f;

    // 配置
    HandDetectionConfig config_;

    void prepare_buffers();
    void preprocess(const cv::Mat& img, cv::Mat& processed, float& scale, float& padX, float& padY);
    void preprocess_gpu(const cv::Mat& img);
    void postprocess(const std::vector<std::vector<float>>& cpuOutputs,
                     std::vector<HandDetection>& detections,
                     float scale, int img_width, int img_height);
    std::vector<HandDetection> nms(std::vector<HandDetection>& boxes);
    cv::Mat get_affine_matrix(const HandDetection& box, int dstSize = 256);

    void cleanup();
};