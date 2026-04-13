#pragma once

#include "det_task_base.h"
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <memory>
#include <vector>
#include <string>
#include "log_system/log_macros.hpp"

// YOLO TensorRT 检测任务实现
// 该实现与 TensorRT 强耦合，包含完整的预处理、推理、后处理流程
// 支持 GPU 加速的预处理和后处理
class YoloTensorRT : public IDetectionTask {
public:
    /**
 * @brief 构造函数
 * 
 * 初始化 YoloTensorRTTask 对象
 */
YoloTensorRT();
    explicit YoloTensorRT(const std::string& engine_path);
    YoloTensorRT(const std::string& model_path,
                    const std::string& config_path,
                    const InferenceEngineConfig& engine_config);
    ~YoloTensorRT() override;

    // 禁用拷贝构造和赋值
    YoloTensorRT(const YoloTensorRT&) = delete;
    YoloTensorRT& operator=(const YoloTensorRT&) = delete;

    // IDetectionTask 接口实现
    bool loadConfig(const std::string& config_path) override;
    std::vector<DetectionResult> detect(const cv::Mat& image) override;
    std::vector<std::string> getSupportedClasses() const override;
    void setThreshold(float conf_thresh, float nms_thresh) override;
    DetectionConfig getConfig() const override { return config_; }
    void setConfig(const DetectionConfig& config) override { config_ = config; }
    void setClassNames(const std::vector<std::string>& class_names) override { class_names_ = class_names; }
    std::string getAlgorithmName() const override { return "YOLO"; }
    std::string getEngineName() const override { return "TensorRT"; }
    std::string getModelVersion() const override { return model_version_; }
    bool isInitialized() const override { return initialized_; }

    void drawResults(cv::Mat& image,
                   const std::vector<DetectionResult>& results) const override;

    void drawResultsWithDepth(cv::Mat& image,
                             const std::vector<DetectionResult>& results,
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
    std::vector<void*> device_buffers_;
    std::vector<void*> host_buffers_;
    std::vector<size_t> buffer_sizes_;
    int input_index_;
    std::vector<int> output_indices_;

    // 模型参数
    int input_w_;
    int input_h_;
    int input_c_;
    int num_classes_;
    int num_boxes_;
    int batch_size_;

    // Letterbox 参数
    float ratio_;
    float dw_;
    float dh_;

    // 配置和状态
    DetectionConfig config_;
    std::string model_version_;
    bool initialized_;
    std::vector<std::string> class_names_;

    // 日志项目路径
    std::string log_path_;

    // GPU 预处理缓存
    void* d_src_ = nullptr;
    size_t d_src_size_ = 0;

    // 私有方法
    bool prepareBuffers();
    void freeBuffers();
    void preprocessCPU(const cv::Mat& input, void* gpu_buffer);
    void preprocessGPU(const cv::Mat& input, void* gpu_buffer);
    void postprocessCPU(const std::vector<void*>& raw_outputs,
                       int img_width, int img_height,
                       std::vector<DetectionResult>& detections);
    void postprocessGPU(const std::vector<void*>& raw_outputs,
                       int img_width, int img_height,
                       std::vector<DetectionResult>& detections);

    std::vector<int> nms(const std::vector<cv::Rect>& boxes,
                        const std::vector<float>& scores,
                        float nms_thresh) const;
    bool loadClassNames(const std::string& config_path);
    cv::Scalar getColorByClassId(int class_id) const;

    // CUDA 错误检查宏
    static void checkCUDA(cudaError_t err, const char* file, int line);
};

#define CHECK_CUDA(call) checkCUDA(call, __FILE__, __LINE__)
