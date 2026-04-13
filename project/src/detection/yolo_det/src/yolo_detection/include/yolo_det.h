#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <stdexcept>
#include <memory>
#include "NvInfer.h"
#include "NvInferRuntime.h"
#include "logger.h"
#include <opencv2/cudaimgproc.hpp>

struct Detection 
{
    //center_x center_y w h
    float bbox[4];
    float conf;
    int class_id;
    float x3d;
    float y3d;
    float z3d;
};

struct Intrinsics
{
    float fx;
    float fy;
    float cx;
    float cy;
};

class TensorRTException : public std::runtime_error 
{
public:
    explicit TensorRTException(const std::string& msg) 
        : std::runtime_error("[TensorRT] " + msg) {}
    
    explicit TensorRTException(const char* msg) 
        : std::runtime_error(std::string("[TensorRT] ") + msg) {}
};

class YoloDet 
{
public:
    YoloDet();
    ~YoloDet();
    void load_engine(const std::string& engine_path);
    std::vector<Detection> infer(cv::Mat& img);
    std::vector<Detection> infer_gpu(const cv::Mat& img);
    void draw_results(cv::Mat& img, 
                           const std::vector<Detection>& detections,
                           const std::vector<std::string>& class_names = {});
    void draw_results(cv::Mat &img,
                      const std::vector<Detection> &detections,
                      cv::Mat &depth_img,
                      const Intrinsics& intrinsics,
                      const std::vector<std::string> &class_names = {});
    bool IsStairs(const Detection& detection, const cv::Mat& depth_img);
    void FilterRes(const cv::Mat& depth_img, std::vector<Detection>& detections);
private:
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    
    void* buffers_[2] = {nullptr, nullptr};
    cudaStream_t stream_ = nullptr;
    Logger gLogger;
    
    int input_index_ = -1;
    int output_index_ = -1;
    int input_w_ = 0;
    int input_h_ = 0;
    int input_c_ = 0;
    int num_boxes_ = 0;
    int num_classes_ = 0;
    float ratio_ = 1.0;
    float dw_ = 0.0;
    float dh_ = 0.0;
    int output_size_ = 0;
    int batch_size_ = 0;
    float conf_thresh_ = 0.5;
    float iou_thresh_ = 0.5;
      
    void prepare_buffers();
    void preprocess(const cv::Mat& img);
    void preprocess_gpu(const cv::Mat& img);
    void postprocess(std::vector<Detection>& detections, 
                    int img_width, 
                    int img_height);
    void postprocess_gpu(std::vector<Detection>& detections, 
                    int img_width, 
                    int img_height);
private:
    uchar3* d_src_       = nullptr;   // GPU 原图缓存
    size_t  d_src_size_  = 0;         // 已分配字节数
};
