#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include "NvInfer.h"
#include "logger.h"
#include "exception.h"

struct HandKeypoints {
    std::vector<cv::Point2f> kpts;  // 21个关键点，归一化到[0,1]或像素坐标
    std::vector<float> scores;       // 每个点的置信度
    bool valid;
};

struct HandPoseConfig {
    int input_size = 256;
    float simcc_split_ratio = 2.0f;
    bool use_gpu_preprocess = true;
};

class HandPoseEstimator {
public:
    HandPoseEstimator();
    ~HandPoseEstimator();

    void load_model(const std::string& engine_path);
    void set_config(const HandPoseConfig& config);

    HandKeypoints estimate(const cv::Mat& hand_crop);
    std::vector<cv::Point2f> map_to_original(const HandKeypoints& pose,
                                             const cv::Mat& affineMatrix);

private:
    // TensorRT 相关
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    cudaStream_t stream_ = nullptr;
    Logger gLogger_;

    // 输入输出信息
    int input_index_ = -1;
    int output_x_index_ = -1;
    int output_y_index_ = -1;
    std::string input_name_;
    std::string output_x_name_;
    std::string output_y_name_;
    int input_w_ = 0;
    int input_h_ = 0;
    int input_c_ = 3;
    int x_dim_ = 0;
    int y_dim_ = 0;

    // GPU 缓冲区
    void* input_buffer_ = nullptr;
    void* output_x_buffer_ = nullptr;
    void* output_y_buffer_ = nullptr;

    // 配置
    HandPoseConfig config_;

    void prepare_buffers();
    void preprocess(const cv::Mat& image, float* gpu_buffer);
    void decode_simcc(const float* simccX, const float* simccY,
                      int xDim, int yDim, HandKeypoints& result);
    int argmax(const float* ptr, int len);
    void cleanup();
};