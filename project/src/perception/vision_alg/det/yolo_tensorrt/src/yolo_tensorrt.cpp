#include "yolo_tensorrt.h"
#include "cuda_process.h"
#include "det_task_base.h"
#include <vector>
#include "task_factory.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cstring>
#include "bas_operate/file_operate.hpp"

// 静态注册 YOLO 算法（TensorRT 实现）
namespace {
    struct YoloRegistrar {
        YoloRegistrar() {
            TaskFactory::registerDetectionAlgorithm(
                "yolo",
                InferenceEngineType::TENSORRT,  // 声明支持的引擎类型
                [](const std::string& model_path,
                   const std::string& config_path,
                   const InferenceEngineConfig& engine_config) -> std::unique_ptr<IDetectionTask> {
                    // 工厂已确保 engine_config.engine_type == TENSORRT
                    return std::make_unique<YoloTensorRT>(model_path, config_path, engine_config);
                });
        }
    };
    static YoloRegistrar registrar;
}

// TensorRT 日志记录器
class TensorRTLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        // 只输出警告和错误
        if (severity <= Severity::kWARNING) {
            std::cerr << "[TensorRT] " << msg << std::endl;
        }
    }
};

static TensorRTLogger gLogger;

// CUDA 错误检查
void YoloTensorRT::checkCUDA(cudaError_t err, const char* file, int line) {
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("CUDA error at ") + file + ":" +
                               std::to_string(line) + ": " +
                               cudaGetErrorString(err));
    }
}

// 构造函数
YoloTensorRT::YoloTensorRT()
    : stream_(nullptr),
      input_index_(-1),
      input_w_(640),
      input_h_(640),
      input_c_(3),
      num_classes_(80),
      num_boxes_(8400),
      batch_size_(1),
      ratio_(1.0f),
      dw_(0.0f),
      dh_(0.0f),
      initialized_(false),
      log_path_("yolo_tensorrt") {

    runtime_ = std::unique_ptr<nvinfer1::IRuntime>(
        nvinfer1::createInferRuntime(gLogger));
    if (!runtime_) {
        throw std::runtime_error("Failed to create TensorRT runtime");
    }

    CHECK_CUDA(cudaStreamCreate(&stream_));

    // class_names_ 由应用层通过 setClassNames() 设置
}

YoloTensorRT::YoloTensorRT(const std::string& engine_path)
    : YoloTensorRT() {
    loadEngine(engine_path);
}

// 构造函数：使用 TensorRT 引擎（工厂已确保引擎类型正确）
YoloTensorRT::YoloTensorRT(const std::string& model_path,
                                    const std::string& config_path,
                                    const InferenceEngineConfig& engine_config)
    : YoloTensorRT() {

    // 初始化日志路径
    log_path_ = basmodule::get_project_name_by_file_path(__FILE__);

    // 保存引擎配置
    config_.engine_config = engine_config;

    // 直接使用 TensorRT（工厂已验证引擎类型）
    loadEngine(model_path);
    LOG_INFO(log_path_, "[YoloTensorRT] Using TensorRT engine");

    // 根据配置调整参数
    if (engine_config.enable_fp16) {
        LOG_INFO(log_path_, "[YoloTensorRT] FP16 inference enabled");
    }
    if (engine_config.enable_int8) {
        LOG_INFO(log_path_, "[YoloTensorRT] INT8 quantization enabled");
    }
}

// 析构函数
YoloTensorRT::~YoloTensorRT() {
    freeBuffers();

    if (stream_) {
        CHECK_CUDA(cudaStreamDestroy(stream_));
        stream_ = nullptr;
    }

    context_.reset();
    engine_.reset();
    runtime_.reset();

    if (d_src_) {
        CHECK_CUDA(cudaFree(d_src_));
        d_src_ = nullptr;
    }
}

// 加载 TensorRT 引擎文件
bool YoloTensorRT::loadEngine(const std::string& engine_path) {
    std::ifstream file(engine_path, std::ios::binary);
    if (!file.good()) {
        std::cerr << "Failed to open engine file: " << engine_path << std::endl;
        return false;
    }

    file.seekg(0, std::ios::end);
    const size_t model_size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> model_data(model_size);
    file.read(model_data.data(), model_size);
    file.close();

    engine_ = std::unique_ptr<nvinfer1::ICudaEngine>(
        runtime_->deserializeCudaEngine(model_data.data(), model_size));

    if (!engine_) {
        std::cerr << "Failed to deserialize CUDA engine" << std::endl;
        return false;
    }

    context_ = std::unique_ptr<nvinfer1::IExecutionContext>(
        engine_->createExecutionContext());

    if (!context_) {
        std::cerr << "Failed to create execution context" << std::endl;
        return false;
    }

    if (!prepareBuffers()) {
        return false;
    }

    initialized_ = true;
    return true;
}

// 准备缓冲区
bool YoloTensorRT::prepareBuffers() {
    const int num_io_tensors = engine_->getNbIOTensors();
    device_buffers_.resize(num_io_tensors, nullptr);
    host_buffers_.resize(num_io_tensors, nullptr);
    buffer_sizes_.resize(num_io_tensors, 0);

    for (int i = 0; i < num_io_tensors; ++i) {
        const char* name = engine_->getIOTensorName(i);
        const auto dims = engine_->getTensorShape(name);
        const auto data_type = engine_->getTensorDataType(name);
        const bool is_input = engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;

        // 计算buffer大小
        size_t buffer_size = 1;
        for (int j = 0; j < dims.nbDims; ++j) {
            buffer_size *= dims.d[j];
        }

        // 根据数据类型调整大小
        switch (data_type) {
            case nvinfer1::DataType::kFLOAT:
                buffer_size *= sizeof(float);
                break;
            case nvinfer1::DataType::kHALF:
                buffer_size *= sizeof(uint16_t);
                break;
            case nvinfer1::DataType::kINT32:
                buffer_size *= sizeof(int32_t);
                break;
            case nvinfer1::DataType::kINT8:
                buffer_size *= sizeof(int8_t);
                break;
            default:
                std::cerr << "Unsupported data type" << std::endl;
                return false;
        }

        buffer_sizes_[i] = buffer_size;

        // 分配GPU内存
        CHECK_CUDA(cudaMalloc(&device_buffers_[i], buffer_size));

        // 为输出分配CPU内存
        if (!is_input) {
            host_buffers_[i] = malloc(buffer_size);
        } else {
            input_index_ = i;

            // 解析输入形状
            int idx_offset = (dims.nbDims == 4) ? 1 : 0;
            if (dims.nbDims >= 3) {
                input_c_ = dims.d[idx_offset];
                input_h_ = dims.d[idx_offset + 1];
                input_w_ = dims.d[idx_offset + 2];
            }

            // 解析输出形状获取类别数和box数
            const char* output_name = engine_->getIOTensorName(1);
            const auto output_dims = engine_->getTensorShape(output_name);
            if (output_dims.nbDims == 3) {
                // 格式: [1, num_classes+5, num_boxes]
                num_classes_ = output_dims.d[1] - 4;
                num_boxes_ = output_dims.d[2];
            }
        }

        if (!is_input) {
            output_indices_.push_back(i);
        }
    }

    return true;
}

// 释放缓冲区
void YoloTensorRT::freeBuffers() {
    for (size_t i = 0; i < device_buffers_.size(); ++i) {
        if (device_buffers_[i]) {
            CHECK_CUDA(cudaFree(device_buffers_[i]));
            device_buffers_[i] = nullptr;
        }
    }
    device_buffers_.clear();

    for (size_t i = 0; i < host_buffers_.size(); ++i) {
        if (host_buffers_[i]) {
            free(host_buffers_[i]);
            host_buffers_[i] = nullptr;
        }
    }
    host_buffers_.clear();

    buffer_sizes_.clear();
    output_indices_.clear();
}

// CPU 预处理
void YoloTensorRT::preprocessCPU(const cv::Mat& input, void* gpu_buffer) {
    // 1. Letterbox 缩放
    ratio_ = std::min(static_cast<float>(input_h_) / input.rows,
                      static_cast<float>(input_w_) / input.cols);
    int padw = std::round(input.cols * ratio_);
    int padh = std::round(input.rows * ratio_);

    cv::Mat resized;
    if (input.cols != padw || input.rows != padh) {
        cv::resize(input, resized, cv::Size(padw, padh));
    } else {
        resized = input.clone();
    }

    // 2. 计算 padding
    dw_ = input_w_ - padw;
    dh_ = input_h_ - padh;
    dw_ /= 2.0f;
    dh_ /= 2.0f;

    int top = static_cast<int>(std::round(dh_ - 0.1f));
    int bottom = static_cast<int>(std::round(dh_ + 0.1f));
    int left = static_cast<int>(std::round(dw_ - 0.1f));
    int right = static_cast<int>(std::round(dw_ + 0.1f));

    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, top, bottom, left, right,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    // 3. 转换为float并归一化
    cv::Mat float_img;
    padded.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

    // 4. BGR -> RGB
    cv::Mat rgb_img;
    cv::cvtColor(float_img, rgb_img, cv::COLOR_BGR2RGB);

    // 5. HWC -> CHW
    std::vector<float> host_data(input_c_ * input_h_ * input_w_);
    float* chw_ptr = host_data.data();
    const int channel_size = input_h_ * input_w_;

    if (!rgb_img.isContinuous()) {
        rgb_img = rgb_img.clone();
    }

    const float* hwc_ptr = rgb_img.ptr<float>();
    for (int c = 0; c < input_c_; ++c) {
        for (int h = 0; h < input_h_; ++h) {
            for (int w = 0; w < input_w_; ++w) {
                chw_ptr[c * channel_size + h * input_w_ + w] =
                    hwc_ptr[h * input_w_ * input_c_ + w * input_c_ + c];
            }
        }
    }

    CHECK_CUDA(cudaMemcpy(gpu_buffer, host_data.data(), buffer_sizes_[input_index_],
                          cudaMemcpyHostToDevice));
}

// GPU 预处理
void YoloTensorRT::preprocessGPU(const cv::Mat& input, void* gpu_buffer) {
    // 1. 分配/调整 GPU 原图缓存
    size_t required_size = input.rows * input.cols * 3;
    if (d_src_size_ < required_size) {
        if (d_src_) {
            CHECK_CUDA(cudaFree(d_src_));
        }
        CHECK_CUDA(cudaMalloc(&d_src_, required_size));
        d_src_size_ = required_size;
    }

    // 2. 拷贝原图到 GPU
    CHECK_CUDA(cudaMemcpy(d_src_, input.data, required_size, cudaMemcpyHostToDevice));

    // 3. Letterbox 参数计算
    ratio_ = std::min(static_cast<float>(input_h_) / input.rows,
                      static_cast<float>(input_w_) / input.cols);
    int padw = std::round(input.cols * ratio_);
    int padh = std::round(input.rows * ratio_);

    dw_ = input_w_ - padw;
    dh_ = input_h_ - padh;
    dw_ /= 2.0f;
    dh_ /= 2.0f;

    int pad_top = static_cast<int>(std::round(dh_ - 0.1f));
    int pad_left = static_cast<int>(std::round(dw_ - 0.1f));

    // 4. 调用 CUDA kernel
    launch_preprocess_kernel(
        static_cast<uint8_t*>(d_src_), input.rows, input.cols,
        static_cast<float*>(gpu_buffer), input_h_, input_w_,
        ratio_, pad_top, pad_left, 1.0f / 255.0f, stream_
    );
}

// CPU 后处理
void YoloTensorRT::postprocessCPU(const std::vector<void*>& raw_outputs,
                                      int img_width, int img_height,
                                      std::vector<DetectionResult>& detections) {
    detections.clear();

    if (raw_outputs.empty()) {
        return;
    }

    float* output = static_cast<float*>(raw_outputs[0]);

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;

    for (int i = 0; i < num_boxes_; ++i) {
        // 找到最大分数的类别
        float max_score = 0.0f;
        int max_index = 4;

        for (int j = 4; j < num_classes_ + 4; ++j) {
            const float score = output[j * num_boxes_ + i];
            if (score > max_score) {
                max_score = score;
                max_index = j;
            }
        }

        if (max_score < config_.conf_threshold) {
            continue;
        }

        const int class_id = max_index - 4;

        // 提取边界框坐标并恢复
        const float cx_raw = output[0 * num_boxes_ + i];
        const float cy_raw = output[1 * num_boxes_ + i];
        const float w_raw = output[2 * num_boxes_ + i];
        const float h_raw = output[3 * num_boxes_ + i];

        const float cx = (cx_raw - dw_) / ratio_;
        const float cy = (cy_raw - dh_) / ratio_;
        const float w = w_raw / ratio_;
        const float h = h_raw / ratio_;

        // 转换为左上角坐标格式
        const float x1 = cx - w / 2.0f;
        const float y1 = cy - h / 2.0f;
        const float x2 = cx + w / 2.0f;
        const float y2 = cy + h / 2.0f;

        // 确保坐标在图像范围内
        boxes.push_back(cv::Rect(
            static_cast<int>(std::max(0.0f, x1)),
            static_cast<int>(std::max(0.0f, y1)),
            static_cast<int>(std::min(static_cast<float>(img_width), x2) -
                             static_cast<int>(std::max(0.0f, x1))),
            static_cast<int>(std::min(static_cast<float>(img_height), y2) -
                             static_cast<int>(std::max(0.0f, y1)))
        ));

        scores.push_back(max_score);
        class_ids.push_back(class_id);
    }

    // NMS
    std::vector<int> indices = nms(boxes, scores, config_.nms_threshold);

    // 转换为DetectionResult
    for (int idx : indices) {
        DetectionResult det;
        det.center.x = boxes[idx].x + boxes[idx].width / 2.0f;
        det.center.y = boxes[idx].y + boxes[idx].height / 2.0f;
        det.width = boxes[idx].width;
        det.height = boxes[idx].height;
        det.confidence = scores[idx];
        det.class_id = class_ids[idx];
        det.class_name = (class_ids[idx] < static_cast<int>(class_names_.size()))
                         ? class_names_[class_ids[idx]] : "unknown";
        detections.push_back(det);
    }
}

// GPU 后处理
void YoloTensorRT::postprocessGPU(const std::vector<void*>& raw_outputs,
                                      int img_width, int img_height,
                                      std::vector<DetectionResult>& detections) {
    detections.clear();

    if (raw_outputs.empty()) {
        return;
    }

    float* d_trt_out = static_cast<float*>(raw_outputs[0]);

    // 使用静态变量避免重复分配内存（参考yolo_det_alg实现）
    static Bbox32* d_out = nullptr;
    static int* d_keep_num = nullptr;
    static int allocated_num_boxes = 0;

    // 如果需要更大的内存或首次分配
    if (!d_out || allocated_num_boxes < num_boxes_) {
        if (d_out) CHECK_CUDA(cudaFree(d_out));
        if (d_keep_num) CHECK_CUDA(cudaFree(d_keep_num));
        CHECK_CUDA(cudaMalloc(&d_out, num_boxes_ * sizeof(Bbox32)));
        CHECK_CUDA(cudaMalloc(&d_keep_num, sizeof(int)));
        allocated_num_boxes = num_boxes_;
    }

    // 清空计数器
    CHECK_CUDA(cudaMemsetAsync(d_keep_num, 0, sizeof(int), stream_));

    // 启动GPU过滤kernel（使用包装函数）
    launch_filter_kernel(
        d_trt_out, num_boxes_, num_classes_, config_.conf_threshold,
        ratio_, dw_, dh_, d_out, d_keep_num, stream_);

    // 同步并获取保留的框数量
    CHECK_CUDA(cudaStreamSynchronize(stream_));
    int h_keep_num = 0;
    CHECK_CUDA(cudaMemcpyAsync(&h_keep_num, d_keep_num, sizeof(int), cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    if (h_keep_num == 0) {
        return;
    }

    // 将GPU中的框拷贝到CPU
    std::vector<Bbox32> host_boxes(h_keep_num);
    CHECK_CUDA(cudaMemcpyAsync(host_boxes.data(), d_out, h_keep_num * sizeof(Bbox32), cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    // 在CPU端进行NMS（参考yolo_det_alg实现）
    // 先按置信度降序排序
    std::sort(host_boxes.begin(), host_boxes.end(),
        [](const Bbox32& a, const Bbox32& b) { return a.conf > b.conf; });

    // NMS
    std::vector<bool> suppressed(h_keep_num, false);
    std::vector<Bbox32> keep_boxes;

    for (int i = 0; i < h_keep_num; ++i) {
        if (suppressed[i]) continue;
        keep_boxes.push_back(host_boxes[i]);

        for (int j = i + 1; j < h_keep_num; ++j) {
            if (suppressed[j]) continue;

            // 计算IOU（相同类别才比较）
            if (host_boxes[i].class_id != host_boxes[j].class_id) continue;

            float xx1 = std::max(host_boxes[i].x - host_boxes[i].w * 0.5f,
                                  host_boxes[j].x - host_boxes[j].w * 0.5f);
            float yy1 = std::max(host_boxes[i].y - host_boxes[i].h * 0.5f,
                                  host_boxes[j].y - host_boxes[j].h * 0.5f);
            float xx2 = std::min(host_boxes[i].x + host_boxes[i].w * 0.5f,
                                  host_boxes[j].x + host_boxes[j].w * 0.5f);
            float yy2 = std::min(host_boxes[i].y + host_boxes[i].h * 0.5f,
                                  host_boxes[j].y + host_boxes[j].h * 0.5f);

            float w = std::max(0.0f, xx2 - xx1);
            float h = std::max(0.0f, yy2 - yy1);
            float inter = w * h;
            float iou = inter / (host_boxes[i].w * host_boxes[i].h +
                                  host_boxes[j].w * host_boxes[j].h - inter + 1e-6f);

            if (iou > config_.nms_threshold) {
                suppressed[j] = true;
            }
        }
    }

    // 转换为DetectionResult
    for (const auto& box : keep_boxes) {
        DetectionResult det;
        det.center.x = box.x;
        det.center.y = box.y;
        det.width = box.w;
        det.height = box.h;
        det.confidence = box.conf;
        det.class_id = box.class_id;
        det.class_name = (box.class_id < static_cast<int>(class_names_.size()))
                         ? class_names_[box.class_id] : "unknown";
        detections.push_back(det);
    }
}

// NMS 实现
std::vector<int> YoloTensorRT::nms(const std::vector<cv::Rect>& boxes,
                                        const std::vector<float>& scores,
                                        float nms_thresh) const {
    std::vector<int> indices(boxes.size());
    std::iota(indices.begin(), indices.end(), 0);

    // 按置信度降序排序
    std::sort(indices.begin(), indices.end(),
             [&scores](int i, int j) { return scores[i] > scores[j]; });

    std::vector<bool> suppressed(boxes.size(), false);
    std::vector<int> keep;

    for (size_t i = 0; i < indices.size(); ++i) {
        const int idx = indices[i];
        if (suppressed[idx]) continue;

        keep.push_back(idx);

        for (size_t j = i + 1; j < indices.size(); ++j) {
            const int jdx = indices[j];
            if (suppressed[jdx]) continue;

            const cv::Rect& box_i = boxes[idx];
            const cv::Rect& box_j = boxes[jdx];
            const cv::Rect inter = box_i & box_j;
            const float iou = static_cast<float>(inter.area()) /
                             (box_i.area() + box_j.area() - inter.area());

            if (iou > nms_thresh) {
                suppressed[jdx] = true;
            }
        }
    }

    return keep;
}

// 加载配置文件
bool YoloTensorRT::loadConfig(const std::string& config_path) {
    if (config_path.empty()) {
        return true;
    }
    return loadClassNames(config_path);
}

// 加载类别名称
bool YoloTensorRT::loadClassNames(const std::string& config_path) {
    // 简化实现，实际应从配置文件解析
    return true;
}

// 根据类别ID获取颜色
cv::Scalar YoloTensorRT::getColorByClassId(int class_id) const {
    // 使用HSL色彩空间生成鲜艳、区分度高的颜色
    // 将类别ID映射到HSL的色相环上，确保不同类别颜色差异明显
    static const std::vector<cv::Scalar> color_palette = {
        cv::Scalar(255, 0, 0),      // 红色
        cv::Scalar(0, 255, 0),      // 绿色
        cv::Scalar(0, 0, 255),      // 蓝色
        cv::Scalar(255, 255, 0),    // 黄色
        cv::Scalar(255, 0, 255),    // 品红
        cv::Scalar(0, 255, 255),    // 青色
        cv::Scalar(255, 128, 0),    // 橙色
        cv::Scalar(255, 0, 128),    // 玫瑰红
        cv::Scalar(128, 255, 0),    // 酸橙绿
        cv::Scalar(0, 255, 128),    // 水鸭绿
        cv::Scalar(128, 0, 255),    // 紫色
        cv::Scalar(0, 128, 255),    // 天蓝色
        cv::Scalar(255, 128, 128),  // 浅红
        cv::Scalar(128, 255, 128),  // 浅绿
        cv::Scalar(128, 128, 255),  // 浅蓝
        cv::Scalar(192, 192, 192),  // 浅灰
        cv::Scalar(255, 192, 203),  // 粉色
        cv::Scalar(165, 42, 42),     // 褐色
        cv::Scalar(128, 0, 0),      // 深红
        cv::Scalar(0, 128, 0),      // 深绿
        cv::Scalar(0, 0, 128),      // 深蓝
        cv::Scalar(128, 128, 0),    // 橄榄色
        cv::Scalar(128, 0, 128),     // 深紫
        cv::Scalar(0, 128, 128),    // 深青
        cv::Scalar(255, 255, 128),  // 浅黄
        cv::Scalar(255, 128, 255),  // 浅品红
        cv::Scalar(128, 255, 255),  // 浅青
    };

    // 使用类别ID作为索引循环使用调色板
    // 确保即使是大量类别也能获得不同的颜色
    int index = class_id % static_cast<int>(color_palette.size());
    return color_palette[index];
}

// 执行检测
std::vector<DetectionResult> YoloTensorRT::detect(const cv::Mat& image) {
    if (!initialized_) {
        std::cerr << "Detection task not initialized" << std::endl;
        return {};
    }

    // 预处理
    if (config_.use_gpu_preprocess) {
        preprocessGPU(image, device_buffers_[input_index_]);
    } else {
        preprocessCPU(image, device_buffers_[input_index_]);
    }

    // 设置 tensor 地址
    for (int i = 0; i < static_cast<int>(device_buffers_.size()); ++i) {
        const char* name = engine_->getIOTensorName(i);
        context_->setTensorAddress(name, device_buffers_[i]);
    }

    // 推理
    if (!context_->enqueueV3(stream_)) {
        std::cerr << "Failed to enqueue inference" << std::endl;
        return {};
    }

    CHECK_CUDA(cudaStreamSynchronize(stream_));

    // 后处理
    std::vector<DetectionResult> detections;
    if (config_.use_gpu_postprocess) {
        // GPU后处理：直接使用设备指针，无需先拷贝到HOST
        std::vector<void*> device_outputs;
        for (int output_idx : output_indices_) {
            device_outputs.push_back(device_buffers_[output_idx]);
        }
        postprocessGPU(device_outputs, image.cols, image.rows, detections);
    } else {
        // CPU后处理：需要先拷贝到HOST
        std::vector<void*> host_outputs;
        for (int output_idx : output_indices_) {
            CHECK_CUDA(cudaMemcpy(host_buffers_[output_idx], device_buffers_[output_idx],
                                  buffer_sizes_[output_idx], cudaMemcpyDeviceToHost));
            host_outputs.push_back(host_buffers_[output_idx]);
        }
        postprocessCPU(host_outputs, image.cols, image.rows, detections);
    }

    // 限制最大检测数
    if (detections.size() > static_cast<size_t>(config_.max_detections)) {
        detections.resize(config_.max_detections);
    }

    return detections;
}

// 获取支持的类别列表
std::vector<std::string> YoloTensorRT::getSupportedClasses() const {
    return class_names_;
}

// 设置检测阈值
void YoloTensorRT::setThreshold(float conf_thresh, float nms_thresh) {
    config_.conf_threshold = conf_thresh;
    config_.nms_threshold = nms_thresh;
}

// 绘制检测结果
void YoloTensorRT::drawResults(cv::Mat& image,
                                   const std::vector<DetectionResult>& results) const {
    for (const auto& det : results) {
        // 根据类别ID生成颜色
        cv::Scalar color = getColorByClassId(det.class_id);

        cv::Rect bbox = det.getBBox();
        cv::rectangle(image, bbox, color, 2);

        // 直接使用 DetectionResult 中的 class_name
        std::string label = det.class_name + cv::format(" %.2f", det.confidence);

        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                             0.5, 1, &baseLine);

        cv::Point2f tl = det.topLeft();
        cv::rectangle(image,
                     cv::Point(static_cast<int>(tl.x), static_cast<int>(tl.y - labelSize.height - baseLine)),
                     cv::Point(static_cast<int>(tl.x + labelSize.width), static_cast<int>(tl.y)),
                     color, cv::FILLED);

        cv::putText(image, label,
                   cv::Point(static_cast<int>(tl.x), static_cast<int>(tl.y - baseLine)),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
}

// 绘制带深度的检测结果
void YoloTensorRT::drawResultsWithDepth(cv::Mat& image,
                                           const std::vector<DetectionResult>& results,
                                          const cv::Mat& depth_image,
                                          const CameraIntrinsics& intrinsics) const {
    for (const auto& det : results) {
        // 根据类别ID生成颜色
        cv::Scalar color = getColorByClassId(det.class_id);

        cv::Rect bbox = det.getBBox();
        cv::rectangle(image, bbox, color, 2);

        cv::Point center(
            static_cast<int>(det.center.x),
            static_cast<int>(det.center.y)
        );

        float depth = 0.0f;
        if (!depth_image.empty() &&
            center.x >= 0 && center.x < depth_image.cols &&
            center.y >= 0 && center.y < depth_image.rows) {
            depth = depth_image.at<uint16_t>(center.y, center.x) / 1000.0f;
        }

        // 直接使用 DetectionResult 中的 class_name
        std::string label = det.class_name + cv::format(" %.2f", det.confidence);
        if (depth > 0) {
            label += cv::format(" d=%.2fm", depth);
        }

        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                             0.5, 1, &baseLine);

        cv::Point2f tl = det.topLeft();
        cv::rectangle(image,
                     cv::Point(static_cast<int>(tl.x), static_cast<int>(tl.y - labelSize.height - baseLine)),
                     cv::Point(static_cast<int>(tl.x + labelSize.width), static_cast<int>(tl.y)),
                     color, cv::FILLED);

        cv::putText(image, label,
                   cv::Point(static_cast<int>(tl.x), static_cast<int>(tl.y - baseLine)),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
}
