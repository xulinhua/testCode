#include "hand_detector.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <cuda_runtime_api.h>
#include "process.h"

#define CHECK_CUDA(call) { \
    cudaError_t status = call; \
    if (status != cudaSuccess) { \
        throw TensorRTException("CUDA error at " + std::string(__FILE__) + ":" + \
                               std::to_string(__LINE__) + ": " + \
                               cudaGetErrorString(status)); \
    } \
}

HandDetector::HandDetector() {
    runtime_ = nullptr;
    engine_ = nullptr;
    context_ = nullptr;
    stream_ = nullptr;
    input_buffer_ = nullptr;
    d_src_ = nullptr;
    d_mean_ = nullptr;
    d_std_ = nullptr;
}

HandDetector::~HandDetector() {
    cleanup();
}

void HandDetector::cleanup() {
    if (input_buffer_) cudaFree(input_buffer_);
    if (d_src_) cudaFree(d_src_);
    if (d_mean_) cudaFree(d_mean_);
    if (d_std_) cudaFree(d_std_);

    for (void* buffer : output_buffers_) {
        if (buffer) cudaFree(buffer);
    }
    output_buffers_.clear();

    if (stream_) cudaStreamDestroy(stream_);
    if (context_) delete context_;
    if (engine_) delete engine_;
    if (runtime_) delete runtime_;

    input_buffer_ = nullptr;
    d_src_ = nullptr;
    d_mean_ = nullptr;
    d_std_ = nullptr;
    stream_ = nullptr;
    context_ = nullptr;
    engine_ = nullptr;
    runtime_ = nullptr;
}

void HandDetector::set_config(const HandDetectionConfig& config) {
    config_ = config;
}

void HandDetector::load_engine(const std::string& engine_path) {
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file) {
        throw TensorRTException("无法打开引擎文件: " + engine_path);
    }

    engine_file.seekg(0, std::ios::end);
    const size_t file_size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);

    std::vector<char> engine_data(file_size);
    if (!engine_file.read(engine_data.data(), file_size)) {
        throw TensorRTException("读取引擎文件失败");
    }
    engine_file.close();

    runtime_ = nvinfer1::createInferRuntime(gLogger_);
    if (!runtime_) {
        throw TensorRTException("创建TensorRT运行时失败");
    }

    engine_ = runtime_->deserializeCudaEngine(engine_data.data(), file_size);
    if (!engine_) {
        throw TensorRTException("反序列化CUDA引擎失败");
    }

    context_ = engine_->createExecutionContext();
    if (!context_) {
        throw TensorRTException("创建执行上下文失败");
    }

    CHECK_CUDA(cudaStreamCreate(&stream_));

    // 获取输入输出信息
    int num_io = engine_->getNbIOTensors();
    gLogger_.info("引擎张量总数: ", num_io);

    for (int i = 0; i < num_io; ++i) {
        const char* name = engine_->getIOTensorName(i);
        nvinfer1::TensorIOMode mode = engine_->getTensorIOMode(name);
        auto dims = engine_->getTensorShape(name);

        gLogger_.info("张量 ", i, ": ", name, " (",
                     (mode == nvinfer1::TensorIOMode::kINPUT ? "输入" : "输出"), ")");

        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            input_index_ = i;
            input_name_ = name;
            input_c_ = dims.d[1];
            input_h_ = config_.input_size;
            input_w_ = config_.input_size;
            gLogger_.info("  输入尺寸: [", input_c_, ", ", input_h_, ", ", input_w_, "]");
        } else {
            output_names_.push_back(name);
            output_indices_.push_back(i);
        }
    }

    if (input_index_ == -1) {
        throw TensorRTException("无效的输入索引");
    }

    // 准备缓冲区
    prepare_buffers();
}

void HandDetector::prepare_buffers() {
    // 分配输入缓冲区
    size_t input_size = 1 * input_c_ * input_h_ * input_w_ * sizeof(float);
    CHECK_CUDA(cudaMalloc(&input_buffer_, input_size));
    gLogger_.info("分配输入缓冲区: ", input_size / 1024.0 / 1024.0, " MB");

    // 如果启用GPU预处理，分配GPU图像缓冲区（按最大可能尺寸分配）
    if (config_.use_gpu_preprocess) {
        size_t max_src_size = 2048 * 2048 * sizeof(uchar3);  // 最大支持2048x2048
        CHECK_CUDA(cudaMalloc(&d_src_, max_src_size));

        // 分配GPU均值和标准差 (BGR 顺序)
        float h_mean[3] = {103.53f, 116.28f, 123.675f};  // BGR均值
        float h_std[3] = {57.375f, 57.12f, 58.395f};      // BGR标准差
        CHECK_CUDA(cudaMalloc(&d_mean_, 3 * sizeof(float)));
        CHECK_CUDA(cudaMalloc(&d_std_, 3 * sizeof(float)));
        CHECK_CUDA(cudaMemcpy(d_mean_, h_mean, 3 * sizeof(float), cudaMemcpyHostToDevice));
        CHECK_CUDA(cudaMemcpy(d_std_, h_std, 3 * sizeof(float), cudaMemcpyHostToDevice));
    }

    // 分配输出缓冲区
    for (const auto& name : output_names_) {
        auto dims = engine_->getTensorShape(name.c_str());
        size_t size = 1;
        for (int d = 0; d < dims.nbDims; d++) {
            size *= dims.d[d];
        }

        void* buffer = nullptr;
        CHECK_CUDA(cudaMalloc(&buffer, size * sizeof(float)));
        output_buffers_.push_back(buffer);
        output_sizes_.push_back(size);

        gLogger_.info("分配输出缓冲区 ", name, ": ",
                     size * sizeof(float) / 1024.0 / 1024.0, " MB");
    }
}

void HandDetector::preprocess(const cv::Mat& img, cv::Mat& processed,
                              float& scale, float& padX, float& padY) {
    // 保持长宽比的resize
    scale = std::min((float)config_.input_size / img.rows,
                     (float)config_.input_size / img.cols);
    ratio_ = scale;
    int newW = std::round(img.cols * scale);
    int newH = std::round(img.rows * scale);

    cv::Mat resized;
    if ((int)img.cols != newW || (int)img.rows != newH)
        cv::resize(img, resized, cv::Size(newW, newH));
    else
        resized = img;

    // 计算padding - 填充到右下角
    padX = config_.input_size - newW;
    padY = config_.input_size - newH;
    int top = 0;
    int left = 0;
    int bottom = (int)std::round(padY);
    int right = (int)std::round(padX);
    cv::copyMakeBorder(resized, processed, top, bottom, left, right,
                      cv::BORDER_CONSTANT, {114, 114, 114});
}

void HandDetector::preprocess_gpu(const cv::Mat& img) {
    // 上传图像到GPU
    size_t src_size = img.rows * img.cols * sizeof(uchar3);
    CHECK_CUDA(cudaMemcpyAsync(d_src_, img.data, src_size, cudaMemcpyHostToDevice, stream_));

    // 计算缩放和填充（右下角填充）
    ratio_ = std::min((float)config_.input_size / img.rows,
                      (float)config_.input_size / img.cols);
    int newW = std::round(img.cols * ratio_);
    int newH = std::round(img.rows * ratio_);
    // 右下角填充: top=0, left=0, bottom=dh_, right=dw_
    dw_ = config_.input_size - newW;
    dh_ = config_.input_size - newH;

    // 调用GPU预处理kernel（右下角填充）
    launch_preprocess_kernel(d_src_, img.rows, img.cols,
                            static_cast<float*>(input_buffer_),
                            input_h_, input_w_,
                            ratio_, dh_, dw_,
                            d_mean_, d_std_,
                            stream_);
}

std::vector<HandDetection> HandDetector::nms(std::vector<HandDetection>& boxes) {
    std::sort(boxes.begin(), boxes.end(),
              [](const HandDetection& a, const HandDetection& b) { return a.score > b.score; });

    std::vector<bool> suppressed(boxes.size(), false);
    std::vector<HandDetection> result;

    auto iou = [](const HandDetection& a, const HandDetection& b) {
        float interX1 = std::max(a.x1, b.x1);
        float interY1 = std::max(a.y1, b.y1);
        float interX2 = std::min(a.x2, b.x2);
        float interY2 = std::min(a.y2, b.y2);

        if (interX2 <= interX1 || interY2 <= interY1) return 0.0f;

        float interArea = (interX2 - interX1) * (interY2 - interY1);
        float areaA = (a.x2 - a.x1) * (a.y2 - a.y1);
        float areaB = (b.x2 - b.x1) * (b.y2 - b.y1);

        return interArea / (areaA + areaB - interArea);
    };

    for (size_t i = 0; i < boxes.size(); i++) {
        if (suppressed[i]) continue;
        result.push_back(boxes[i]);

        for (size_t j = i + 1; j < boxes.size(); j++) {
            if (suppressed[j]) continue;
            if (iou(boxes[i], boxes[j]) > config_.nms_thresh) {
                suppressed[j] = true;
            }
        }
    }

    return result;
}

cv::Mat HandDetector::get_affine_matrix(const HandDetection& box, int dstSize) {
    float cx = (box.x1 + box.x2) / 2.0f;
    float cy = (box.y1 + box.y2) / 2.0f;
    float w = box.x2 - box.x1;
    float h = box.y2 - box.y1;
    float size = std::max(w, h) * config_.crop_enlargement;

    cv::Point2f srcTri[3];
    srcTri[0] = cv::Point2f(cx - size/2, cy - size/2);
    srcTri[1] = cv::Point2f(cx + size/2, cy - size/2);
    srcTri[2] = cv::Point2f(cx - size/2, cy + size/2);

    cv::Point2f dstTri[3];
    dstTri[0] = cv::Point2f(0, 0);
    dstTri[1] = cv::Point2f(dstSize - 1, 0);
    dstTri[2] = cv::Point2f(0, dstSize - 1);

    return cv::getAffineTransform(srcTri, dstTri);
}

std::vector<HandDetection> HandDetector::detect(const cv::Mat& img) {
    if (img.empty()) {
        throw TensorRTException("输入图像为空");
    }

    // 预处理
    cv::Mat processed;
    float scale, padX, padY;
    preprocess(img, processed, scale, padX, padY);

    // 归一化
    cv::Mat float_img;
    processed.convertTo(float_img, CV_32FC3);

    cv::Scalar mean_bgr = cv::Scalar(103.53, 116.28, 123.675);
    cv::Scalar std_bgr = cv::Scalar(57.375, 57.12, 58.395);
    cv::subtract(float_img, mean_bgr, float_img);
    cv::divide(float_img, std_bgr, float_img);

    // CHW格式转换
    std::vector<float> hostInput(1 * input_c_ * input_h_ * input_w_);
    std::vector<cv::Mat> channels(input_c_);
    for (int i = 0; i < input_c_; ++i) {
        channels[i] = cv::Mat(input_h_, input_w_, CV_32FC1,
                             hostInput.data() + i * input_h_ * input_w_);
    }
    cv::split(float_img, channels);

    CHECK_CUDA(cudaMemcpyAsync(input_buffer_, hostInput.data(),
                               hostInput.size() * sizeof(float),
                               cudaMemcpyHostToDevice, stream_));

    // 设置输入形状
    nvinfer1::Dims4 inputDims{1, input_c_, input_h_, input_w_};
    if (!context_->setInputShape(input_name_.c_str(), inputDims)) {
        throw TensorRTException("setInputShape failed");
    }

    // 设置输入输出地址
    context_->setTensorAddress(input_name_.c_str(), input_buffer_);
    for (size_t i = 0; i < output_names_.size(); i++) {
        context_->setTensorAddress(output_names_[i].c_str(), output_buffers_[i]);
    }

    // 执行推理
    if (!context_->enqueueV3(stream_)) {
        throw TensorRTException("enqueueV3 failed");
    }

    // 拷贝输出到CPU
    std::vector<std::vector<float>> cpuOutputs(output_names_.size());
    for (size_t i = 0; i < output_names_.size(); i++) {
        cpuOutputs[i].resize(output_sizes_[i]);
        CHECK_CUDA(cudaMemcpyAsync(cpuOutputs[i].data(), output_buffers_[i],
                                   output_sizes_[i] * sizeof(float),
                                   cudaMemcpyDeviceToHost, stream_));
    }
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    // 后处理
    std::vector<HandDetection> detections;
    postprocess(cpuOutputs, detections, ratio_, img.cols, img.rows);

    return detections;
}

std::vector<HandDetection> HandDetector::detect_gpu(const cv::Mat& img) {
    if (img.empty()) {
        throw TensorRTException("输入图像为空");
    }

    // GPU预处理
    preprocess_gpu(img);

    // 设置输入形状
    nvinfer1::Dims4 inputDims{1, input_c_, input_h_, input_w_};
    if (!context_->setInputShape(input_name_.c_str(), inputDims)) {
        throw TensorRTException("setInputShape failed");
    }

    // 设置输入输出地址
    context_->setTensorAddress(input_name_.c_str(), input_buffer_);
    for (size_t i = 0; i < output_names_.size(); i++) {
        context_->setTensorAddress(output_names_[i].c_str(), output_buffers_[i]);
    }

    // 执行推理
    if (!context_->enqueueV3(stream_)) {
        throw TensorRTException("enqueueV3 failed");
    }

    // 拷贝输出到CPU
    std::vector<std::vector<float>> cpuOutputs(output_names_.size());
    for (size_t i = 0; i < output_names_.size(); i++) {
        cpuOutputs[i].resize(output_sizes_[i]);
        CHECK_CUDA(cudaMemcpyAsync(cpuOutputs[i].data(), output_buffers_[i],
                                   output_sizes_[i] * sizeof(float),
                                   cudaMemcpyDeviceToHost, stream_));
    }
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    // 后处理
    std::vector<HandDetection> detections;
    postprocess(cpuOutputs, detections, ratio_, img.cols, img.rows);

    return detections;
}

void HandDetector::postprocess(const std::vector<std::vector<float>>& cpuOutputs,
                                 std::vector<HandDetection>& detections,
                                 float scale, int img_width, int img_height) {
    std::vector<HandDetection> candidates;
    std::map<std::string, int> nameMap;
    for (size_t i = 0; i < output_names_.size(); ++i) nameMap[output_names_[i]] = i;

    struct LayerInfo { std::string scoreName; std::string regName; int stride; int featSize; };
    std::vector<LayerInfo> layers = {
        {"cls_score_p3", "bbox_pred_p3", 8,  40},
        {"cls_score_p4", "bbox_pred_p4", 16, 20},
        {"cls_score_p5", "bbox_pred_p5", 32, 10}
    };

    for (const auto& layer : layers) {
        if (nameMap.find(layer.scoreName) == nameMap.end() ||
            nameMap.find(layer.regName) == nameMap.end()) {
            continue;
        }

        const float* scoreData = cpuOutputs[nameMap[layer.scoreName]].data();
        const float* regData   = cpuOutputs[nameMap[layer.regName]].data();
        int S = layer.featSize;
        int S2 = S * S;

        for (int i = 0; i < S2; i++) {
            // 1. Sigmoid 激活
            float score = 1.0f / (1.0f + std::exp(-scoreData[i]));
            if (score < config_.conf_thresh) continue;

            // 2. 解码 LTRB
            float l = regData[0 * S2 + i];
            float t = regData[1 * S2 + i];
            float r = regData[2 * S2 + i];
            float b = regData[3 * S2 + i];

            // 3. 计算中心点 (特征图网格坐标转输入图坐标)
            int row = i / S;
            int col = i % S;
            float centerX = (float)col * layer.stride;
            float centerY = (float)row * layer.stride;

            // 4. 转换并映射回原图（右下角填充）
            HandDetection box;
            box.x1 = std::max(0.0f, (centerX - l) / scale);
            box.y1 = std::max(0.0f, (centerY - t) / scale);
            box.x2 = std::min((float)img_width,  (centerX + r) / scale);
            box.y2 = std::min((float)img_height, (centerY + b) / scale);
            box.score = score;
            box.class_id = 0;

            if (box.x2 > box.x1 && box.y2 > box.y1) {
                // 计算仿射矩阵供后续关键点使用
                box.affine_matrix = get_affine_matrix(box, 256);
                candidates.push_back(box);
            }
        }
    }

    detections = nms(candidates);
}
