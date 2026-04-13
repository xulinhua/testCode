#include "hand_pose.h"
#include <cmath>

#define CHECK_CUDA(call) { \
    cudaError_t status = call; \
    if (status != cudaSuccess) { \
        throw TensorRTException("CUDA error at " + std::string(__FILE__) + ":" + \
                               std::to_string(__LINE__) + ": " + \
                               cudaGetErrorString(status)); \
    } \
}

HandPoseEstimator::HandPoseEstimator() {
    runtime_ = nullptr;
    engine_ = nullptr;
    context_ = nullptr;
    stream_ = nullptr;
    input_buffer_ = nullptr;
    output_x_buffer_ = nullptr;
    output_y_buffer_ = nullptr;
}

HandPoseEstimator::~HandPoseEstimator() {
    cleanup();
}

void HandPoseEstimator::cleanup() {
    if (input_buffer_) cudaFree(input_buffer_);
    if (output_x_buffer_) cudaFree(output_x_buffer_);
    if (output_y_buffer_) cudaFree(output_y_buffer_);

    if (stream_) cudaStreamDestroy(stream_);
    if (context_) delete context_;
    if (engine_) delete engine_;
    if (runtime_) delete runtime_;

    input_buffer_ = nullptr;
    output_x_buffer_ = nullptr;
    output_y_buffer_ = nullptr;
    stream_ = nullptr;
    context_ = nullptr;
    engine_ = nullptr;
    runtime_ = nullptr;
}

void HandPoseEstimator::set_config(const HandPoseConfig& config) {
    config_ = config;
}

void HandPoseEstimator::load_model(const std::string& engine_path) {
    std::ifstream file(engine_path, std::ios::binary);
    if (!file.is_open()) {
        throw TensorRTException("无法打开模型文件: " + engine_path);
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> engine_data(size);
    file.read(engine_data.data(), size);
    file.close();

    runtime_ = nvinfer1::createInferRuntime(gLogger_);
    if (!runtime_) {
        throw TensorRTException("创建TensorRT运行时失败");
    }

    engine_ = runtime_->deserializeCudaEngine(engine_data.data(), size);
    if (!engine_) {
        throw TensorRTException("反序列化TensorRT引擎失败");
    }

    context_ = engine_->createExecutionContext();
    if (!context_) {
        throw TensorRTException("创建TensorRT执行上下文失败");
    }

    CHECK_CUDA(cudaStreamCreate(&stream_));

    // 获取输入输出信息
    int num_io = engine_->getNbIOTensors();
    gLogger_.info("引擎张量总数: ", num_io);

    for (int i = 0; i < num_io; ++i) {
        const char* tensor_name = engine_->getIOTensorName(i);
        nvinfer1::TensorIOMode mode = engine_->getTensorIOMode(tensor_name);
        auto dims = engine_->getTensorShape(tensor_name);

        gLogger_.info("张量 ", i, ": ", tensor_name, " (",
                     (mode == nvinfer1::TensorIOMode::kINPUT ? "输入" : "输出"), ")");

        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            input_index_ = i;
            input_name_ = tensor_name;
            input_c_ = dims.d[1];
            input_h_ = config_.input_size;
            input_w_ = config_.input_size;
        } else {
            if (output_x_name_.empty()) {
                output_x_index_ = i;
                output_x_name_ = tensor_name;
                x_dim_ = dims.d[2];
            } else {
                output_y_index_ = i;
                output_y_name_ = tensor_name;
                y_dim_ = dims.d[2];
            }
        }
    }

    if (input_index_ == -1 || output_x_index_ == -1 || output_y_index_ == -1) {
        throw TensorRTException("无效的输入输出索引");
    }

    // 准备缓冲区
    prepare_buffers();
}

void HandPoseEstimator::prepare_buffers() {
    // 分配输入缓冲区
    size_t input_size = 1 * input_c_ * input_h_ * input_w_ * sizeof(float);
    CHECK_CUDA(cudaMalloc(&input_buffer_, input_size));
    gLogger_.info("分配输入缓冲区: ", input_size / 1024.0 / 1024.0, " MB");

    // 分配输出缓冲区
    size_t x_size = 21 * x_dim_ * sizeof(float);
    size_t y_size = 21 * y_dim_ * sizeof(float);
    CHECK_CUDA(cudaMalloc(&output_x_buffer_, x_size));
    CHECK_CUDA(cudaMalloc(&output_y_buffer_, y_size));

    gLogger_.info("分配输出X缓冲区: ", x_size / 1024.0, " KB");
    gLogger_.info("分配输出Y缓冲区: ", y_size / 1024.0, " KB");
}

void HandPoseEstimator::preprocess(const cv::Mat& image, float* gpu_buffer) {
    cv::Mat resized;
    if (image.rows != input_h_ || image.cols != input_w_) {
        cv::resize(image, resized, cv::Size(input_w_, input_h_));
    } else {
        resized = image;
    }

    // 转换为float并归一化 (RTMPose使用ImageNet均值方差)
    cv::Mat floatMat;
    resized.convertTo(floatMat, CV_32FC3);

    std::vector<float> hostInput(3 * input_h_ * input_w_);
    float mean[3] = {123.675f, 116.28f, 103.53f};
    float std[3] = {58.395f, 57.12f, 57.375f};

    for (int c = 0; c < 3; c++) {
        for (int h = 0; h < input_h_; h++) {
            for (int w = 0; w < input_w_; w++) {
                // BGR to RGB
                float val = floatMat.at<cv::Vec3f>(h, w)[2 - c];
                hostInput[c * input_h_ * input_w_ + h * input_w_ + w] =
                    (val - mean[c]) / std[c];
            }
        }
    }

    CHECK_CUDA(cudaMemcpyAsync(gpu_buffer, hostInput.data(),
                               hostInput.size() * sizeof(float),
                               cudaMemcpyHostToDevice, stream_));
}

int HandPoseEstimator::argmax(const float* ptr, int len) {
    int maxIdx = 0;
    float maxVal = ptr[0];
    for (int i = 1; i < len; i++) {
        if (ptr[i] > maxVal) {
            maxVal = ptr[i];
            maxIdx = i;
        }
    }
    return maxIdx;
}

void HandPoseEstimator::decode_simcc(const float* simccX, const float* simccY,
                                      int xDim, int yDim,
                                      HandKeypoints& result) {
    result.kpts.resize(21);
    result.scores.resize(21);
    result.valid = true;

    for (int i = 0; i < 21; i++) {
        int idxX = argmax(simccX + i * xDim, xDim);
        int idxY = argmax(simccY + i * yDim, yDim);

        float x = idxX / config_.simcc_split_ratio;
        float y = idxY / config_.simcc_split_ratio;

        result.kpts[i] = cv::Point2f(x, y);
        result.scores[i] = simccX[i * xDim + idxX] * simccY[i * yDim + idxY];
    }
}

HandKeypoints HandPoseEstimator::estimate(const cv::Mat& hand_crop) {
    HandKeypoints result;
    result.valid = false;

    if (hand_crop.empty()) return result;

    // 设置输入形状
    nvinfer1::Dims4 inputDims{1, input_c_, input_h_, input_w_};
    if (!context_->setInputShape(input_name_.c_str(), inputDims)) {
        throw TensorRTException("setInputShape failed for hand pose");
    }

    // 设置地址
    context_->setTensorAddress(input_name_.c_str(), input_buffer_);
    context_->setTensorAddress(output_x_name_.c_str(), output_x_buffer_);
    context_->setTensorAddress(output_y_name_.c_str(), output_y_buffer_);

    // 预处理
    preprocess(hand_crop, static_cast<float*>(input_buffer_));

    // 推理
    if (!context_->enqueueV3(stream_)) {
        throw TensorRTException("enqueueV3 failed");
    }

    // 拷贝输出
    std::vector<float> hostX(21 * x_dim_);
    std::vector<float> hostY(21 * y_dim_);
    CHECK_CUDA(cudaMemcpyAsync(hostX.data(), output_x_buffer_, hostX.size() * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaMemcpyAsync(hostY.data(), output_y_buffer_, hostY.size() * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    // 解码
    decode_simcc(hostX.data(), hostY.data(), x_dim_, y_dim_, result);

    return result;
}

std::vector<cv::Point2f> HandPoseEstimator::map_to_original(const HandKeypoints& pose,
                                                             const cv::Mat& affineMatrix) {
    std::vector<cv::Point2f> mapped;
    mapped.reserve(pose.kpts.size());

    cv::Mat inverse;
    cv::invertAffineTransform(affineMatrix, inverse);

    for (const auto& pt : pose.kpts) {
        std::vector<cv::Point2f> src = {pt};
        std::vector<cv::Point2f> dst;
        cv::transform(src, dst, inverse);
        mapped.push_back(dst[0]);
    }

    return mapped;
}
