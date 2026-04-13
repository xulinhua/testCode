#pragma once
#include <NvInfer.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <cuda_runtime.h>

#define USE_TENSORRT_8 0
namespace ocr_rec
{
class Logger : public nvinfer1::ILogger 
{
public:
    void log(Severity severity, const char* msg) noexcept override 
    {
        if (severity <= Severity::kERROR) 
        {
            std::cerr << "[TensorRT] " << msg << std::endl;
        }
    }
};
}
struct Recognition 
{
    std::string text;
    float score;
};

class OcrRec 
{
public:
    OcrRec();
    ~OcrRec();
    void load_engine(const std::string& engine_path, const std::string& dict_path);
    Recognition infer(const cv::Mat& img);
    std::vector<Recognition> infer_batch(const std::vector<cv::Mat>& imgs);

private:
    void preprocess(const cv::Mat& img, float* gpu_buf);
    void postprocess(float* logits, int seq_len, int dict_size, Recognition& res);
    void preprocess(const cv::Mat& img, float* gpu_buf, int batch_offset);
    void batch_preprocess(const std::vector<cv::Mat>& imgs, float* gpu_buf);
    void batch_postprocess(float* batch_logits, int batch_size, 
                              int seq_len, int dict_size, 
                              std::vector<Recognition>& results);
                              

    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_{nullptr};
    nvinfer1::IExecutionContext* context_{nullptr};
    cudaStream_t stream_{nullptr};
    ocr_rec::Logger gLogger;

    int input_h_, input_w_;
    std::vector<std::string> dict_;
    std::vector<void*> buffers_;
    std::vector<int> predict_shape_;
    int max_batch_size_;
    std::vector<int> predict_shape_default_;
};
