#pragma once

#include "seg_task_base.h"
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <memory>
#include <vector>
#include <string>
#include <array>
#include "log_system/log_macros.hpp"
#include "process.h"  // For Bbox32 struct

// YOLO Segmentation TensorRT 实现类
// 实现 ISegmentationTask 接口，支持实例分割任务
// 支持 GPU 加速的预处理和后处理
class YoloSegTensorRT : public ISegmentationTask {
public:
    YoloSegTensorRT();
    explicit YoloSegTensorRT(const std::string& engine_path);
    YoloSegTensorRT(const std::string& model_path,
                    const std::string& config_path,
                    const InferenceEngineConfig& engine_config);
    ~YoloSegTensorRT() override;

    // 禁用拷贝构造和赋值
    YoloSegTensorRT(const YoloSegTensorRT&) = delete;
    YoloSegTensorRT& operator=(const YoloSegTensorRT&) = delete;

    // ISegmentationTask 接口实现
    bool loadConfig(const std::string& config_path) override;
    std::vector<SegmentationResult> segment(const cv::Mat& image) override;
    std::vector<std::string> getSupportedClasses() const override;
    void setThreshold(float conf_thresh, float nms_thresh) override;
    void setMaskThreshold(float mask_thresh) override;
    SegmentationConfig getConfig() const override { return config_; }
    void setConfig(const SegmentationConfig& config) override { config_ = config; }
    void setClassNames(const std::vector<std::string>& class_names) override { class_names_ = class_names; }
    std::string getAlgorithmName() const override { return "YOLO_SEG"; }
    std::string getEngineName() const override { return "TensorRT"; }
    std::string getModelVersion() const override { return model_version_; }
    bool isInitialized() const override { return initialized_; }

    void drawResults(cv::Mat& image,
                   const std::vector<SegmentationResult>& results) const override;

    void drawResultsWithDepth(cv::Mat& image,
                             const std::vector<SegmentationResult>& results,
                             const cv::Mat& depth_image,
                             const CameraIntrinsics& intrinsics) const override;

    // 加载 TensorRT 引擎文件
    bool loadEngine(const std::string& engine_path);

private:
    // TensorRT 组件
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    cudaStream_t stream_;

    // GPU 缓冲区
    void* input_buffer_ = nullptr;
    void* proto_buffer_ = nullptr;
    void* output_buffer_ = nullptr;
    
    // GPU 后处理缓冲区
    Bbox32* d_decode_out_ = nullptr;
    int* d_keep_count_ = nullptr;

    // 模型参数
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
    int batch_size_ = 1;

    // Letterbox 参数
    float ratio_ = 1.0f;
    float dw_ = 0.0f;
    float dh_ = 0.0f;

    // 配置和状态
    SegmentationConfig config_;
    std::string model_version_;
    bool initialized_ = false;
    std::vector<std::string> class_names_;

    // 日志项目路径
    std::string log_path_;

    // GPU 预处理缓存
    void* d_src_ = nullptr;
    size_t d_src_size_ = 0;

    // 私有方法
    bool prepareBuffers();
    void freeBuffers();
    
    // CPU版本预处理
    void preprocess(const cv::Mat& img);
    // GPU版本预处理
    void preprocess_gpu(const cv::Mat& img);
    
    // CPU版本后处理
    void postprocess(std::vector<SegmentationResult>& results,
                     int img_width, int img_height);
    // GPU版本后处理
    void postprocess_gpu(std::vector<SegmentationResult>& results,
                         int img_width, int img_height);
    
    // CPU版本mask处理
    void process_mask(float* proto_host, std::vector<SegmentationResult>& results,
                      int img_width, int img_height);
    // GPU版本mask处理
    void process_mask_gpu(float* proto_device, std::vector<SegmentationResult>& results,
                          int img_width, int img_height);

    bool loadClassNames(const std::string& config_path);
    cv::Scalar getColorByClassId(int class_id) const;

    // CUDA 错误检查
    static void checkCUDA(cudaError_t err, const char* file, int line);
};

#define CHECK_CUDA(call) checkCUDA(call, __FILE__, __LINE__)
