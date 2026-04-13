#pragma once

#include <NvInfer.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <chrono>
#include <stdexcept>
#include "logger.h"
#include "exception.h"

// 前向声明 Logger（避免包含完整头文件）
class Logger;

struct RecognitionResult {
    std::string text;
    float score;
};

struct RecognitionConfig {
    int rec_image_height = 48;
    int rec_image_width = 320;
    bool use_space_char = true;
    std::string character_dict_path = "";
    float mean[3] = {0.485f, 0.456f, 0.406f};
    float std[3] = {0.229f, 0.224f, 0.225f};
    int max_batch_size = 16;
};

class PPOCRRec {
public:
    PPOCRRec();
    ~PPOCRRec();
    
    void load_model(const std::string& engine_path);
    void load_character_dict(const std::string& dict_path);
    void set_config(const RecognitionConfig& config);
    
    RecognitionResult recognize(const cv::Mat& img);
    std::vector<RecognitionResult> recognize_batch(const std::vector<cv::Mat>& imgs);
    
    cv::Mat preprocess_image(const cv::Mat& img);
    std::vector<float> normalize_image(const cv::Mat& img);
    
    void benchmark(const cv::Mat& img, int num_iterations = 100);
    
    int get_input_width() const { return config_.rec_image_width; }
    int get_input_height() const { return config_.rec_image_height; }
    int get_max_batch_size() const { return config_.max_batch_size; }

private:
    // TensorRT 相关
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    cudaStream_t stream_ = nullptr;
    
    // ===== 新增：Logger 实例 =====
    Logger gLogger_;
    
    // 输入输出信息
    std::string input_name_;
    std::string output_name_;
    int input_index_ = -1;
    int output_index_ = -1;
    
    // GPU 缓冲区
    std::vector<void*> buffers_;
    size_t input_size_ = 0;
    size_t output_size_ = 0;
    
    // 模型信息
    int input_w_ = 0;
    int input_h_ = 0;
    int input_c_ = 3;
    int output_seq_len_ = 0;
    int output_dict_size_ = 0;
    
    // ===== 新增：batch_size_ 成员 =====
    int batch_size_ = 1;
    
    bool is_batch_dynamic_ = false;
    bool is_height_dynamic_ = false;
    bool is_width_dynamic_ = false;
    
    // 字符字典
    std::vector<std::string> character_dict_;
    
    // 配置
    RecognitionConfig config_;
    
    void prepare_buffers();
    void preprocess_cpu(const cv::Mat& img, float* gpu_buffer, int batch_offset = 0);
    void batch_preprocess_cpu(const std::vector<cv::Mat>& imgs, float* gpu_buffer);
    void decode_text(float* logits, int seq_len, RecognitionResult& result);
    std::string decode_sequence(const std::vector<int>& indices);
    
    void setup_dynamic_shapes();
    bool validate_dynamic_shapes();
    RecognitionResult inference_with_dynamic_shape(const cv::Mat& img);
    std::vector<RecognitionResult> inference_batch_with_dynamic_shape(const std::vector<cv::Mat>& imgs);
    
    void cleanup();
};