#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <stdexcept>
#include <memory>
#include <numeric>
#include <algorithm>
#include "NvInfer.h"
#include "NvInferRuntime.h"
#include "logger.h"
#include "process.h"  // For Bbox32 struct

struct Segmentation
{
    float bbox[4];  // [x1, y1, x2, y2]
    float conf;
    int class_id;
    float mask_coef[32];  // mask coefficients
    std::vector<float> mask_matrix;  // 2D mask after processing
    float x3d;  // 3D坐标 X
    float y3d;  // 3D坐标 Y
    float z3d;  // 3D坐标 Z
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

class YoloSeg
{
public:
    YoloSeg();
    ~YoloSeg();
    void load_engine(const std::string& engine_path);
    void set_thresholds(float conf_thresh, float iou_thresh);  // 设置置信度和IoU阈值
    std::vector<Segmentation> infer(cv::Mat& img);
    void draw_results(cv::Mat& img,
                           const std::vector<Segmentation>& segmentations,
                           const std::vector<std::string>& class_names = {});
    void draw_results(cv::Mat &img,
                      const std::vector<Segmentation> &segmentations,
                      cv::Mat &depth_img,
                      const Intrinsics& intrinsics,
                      const std::vector<std::string> &class_names = {});

private:
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;

    void* input_buffer_ = nullptr;
    void* proto_buffer_ = nullptr;
    void* output_buffer_ = nullptr;
    cudaStream_t stream_ = nullptr;
    
    // GPU buffers for postprocess
    Bbox32* d_decode_out_ = nullptr;
    int* d_keep_count_ = nullptr;
    
    Logger gLogger;

    int input_index_ = -1;
    int proto_index_ = -1;
    int output_index_ = -1;
    int input_w_ = 0;
    int input_h_ = 0;
    int input_c_ = 0;
    int num_boxes_ = 0;
    int num_classes_ = 0;
    int proto_c_ = 0;
    int proto_h_ = 0;
    int proto_w_ = 0;
    float ratio_ = 1.0;
    float dw_ = 0.0;
    float dh_ = 0.0;
    int output_size_ = 0;
    int batch_size_ = 0;
    float conf_thresh_ = 0.5;
    float iou_thresh_ = 0.3;
      
    void prepare_buffers();
    void preprocess(const cv::Mat& img);           // CPU版本
    void preprocess_gpu(const cv::Mat& img);      // GPU版本
    void postprocess(std::vector<Segmentation>& segmentations,
                    int img_width,
                    int img_height);               // CPU版本
    void postprocess_gpu(std::vector<Segmentation>& segmentations,
                         int img_width,
                         int img_height);          // GPU版本
    void process_mask_gpu(float* proto_device, std::vector<Segmentation>& segmentations, int img_width, int img_height);  // GPU版本
    void process_mask(float* proto_host, std::vector<Segmentation>& segmentations, int img_width, int img_height);  // CPU版本
    
    uchar3* d_src_ = nullptr;   // GPU 原图缓存
    size_t  d_src_size_ = 0;    // 已分配字节数
};
