#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <stdexcept>
#include <memory>
#include "NvInfer.h"
#include "NvInferRuntime.h"
#include "logger.h"
#include "exception.h"

struct TextDetection {
    cv::RotatedRect rrect;
    float confidence;
    int orientation;  // 检测框方向 (0: 水平, 1: 垂直, 2: 倾斜等)
};

struct TextDetectionConfig {
    float text_thresh = 0.3f;        // 降低阈值，防止漏检
    int min_area = 10;               // 降低面积阈值，检测小字
    int min_size = 3;
    float box_thresh = 0.5f;
    float enlarge_ratio = 2.9f;       
    int max_candidates = 1000;    // 检测框放大30%
};

class DBnetDet {
public:
    DBnetDet();
    ~DBnetDet();
    void load_engine(const std::string& engine_path);
    void set_config(const TextDetectionConfig& config);
    std::vector<TextDetection> detect(cv::Mat& img);
    std::vector<TextDetection> detect_gpu(const cv::Mat& img);
    void draw_results(cv::Mat& img, 
                     const std::vector<TextDetection>& detections,
                     bool draw_polygon = true,
                     bool draw_bbox = false,
                     const cv::Scalar& color = cv::Scalar(0, 255, 0));
    
private:
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    
    void* buffers_[2] = {nullptr, nullptr};
    cudaStream_t stream_ = nullptr;
    Logger gLogger;
    
    int input_index_ = -1;
    int output_index_ = -1;
    std::string input_name_;
    std::string output_name_;
    int input_w_ = 0;
    int input_h_ = 0;
    int input_c_ = 3;
    int batch_size_ = 1;
    int max_out_h_ = 0;
    int max_out_w_ = 0;
    
    float ratio_ = 1.0f;
    float dw_ = 0.0f;
    float dh_ = 0.0f;
    int active_profile_index_ = 0;
    
    int current_output_h_ = 0;
    int current_output_w_ = 0;
    std::vector<float> cpu_output_cache_;
    
    TextDetectionConfig config_;
    
    void prepare_buffers();
    void preprocess(const cv::Mat& img);
    void preprocess_gpu(const cv::Mat& img);
    
    void postprocess(const std::vector<float>& output_data,
                    std::vector<TextDetection>& detections,
                    int img_width, int img_height);
    void postprocess_gpu(const std::vector<float>& output_data,
                        std::vector<TextDetection>& detections,
                        int img_width, int img_height);
    
    std::vector<TextDetection> inference_with_dynamic_shape(const cv::Mat& img);
    void setup_dynamic_shapes();
    bool validate_dynamic_shapes();
    
};