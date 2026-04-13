#include "yolo_seg_tensorrt.h"
#include "task_factory.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cstring>
#include <omp.h>  // OpenMP
#include "bas_operate/file_operate.hpp"

// 静态注册 YOLO Segmentation 算法（TensorRT 实现）
namespace {
    struct YoloSegRegistrar {
        YoloSegRegistrar() {
            TaskFactory::registerSegmentationAlgorithm(
                "yolo_seg",
                InferenceEngineType::TENSORRT,
                [](const std::string& model_path,
                   const std::string& config_path,
                   const InferenceEngineConfig& engine_config) -> std::unique_ptr<ISegmentationTask> {
                    return std::make_unique<YoloSegTensorRT>(model_path, config_path, engine_config);
                });
        }
    };
    static YoloSegRegistrar registrar;
}

// TensorRT 日志记录器
class TensorRTLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[TensorRT] " << msg << std::endl;
        }
    }
};

static TensorRTLogger gLogger;

// 快速 sigmoid 函数
static inline float fast_sigmoid(float x) {
    return 0.5f * (x / (1.0f + fabsf(x))) + 0.5f;
}

// CUDA 错误检查
void YoloSegTensorRT::checkCUDA(cudaError_t err, const char* file, int line) {
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("CUDA error at ") + file + ":" +
                               std::to_string(line) + ": " +
                               cudaGetErrorString(err));
    }
}

// 默认构造函数
YoloSegTensorRT::YoloSegTensorRT()
    : stream_(nullptr),
      initialized_(false),
      log_path_("yolo_seg_tensorrt"),
      d_src_(nullptr),
      d_src_size_(0) {
    
    config_.conf_threshold = 0.5f;
    config_.nms_threshold = 0.45f;
    config_.mask_threshold = 0.5f;
}

// 单参数构造函数
YoloSegTensorRT::YoloSegTensorRT(const std::string& engine_path)
    : YoloSegTensorRT() {
    loadEngine(engine_path);
}

// 完整构造函数（工厂调用）
YoloSegTensorRT::YoloSegTensorRT(const std::string& model_path,
                                 const std::string& config_path,
                                 const InferenceEngineConfig& engine_config)
    : YoloSegTensorRT() {
    
    log_path_ = basmodule::get_project_name_by_file_path(__FILE__);
    config_.engine_config = engine_config;
    
    if (!loadEngine(model_path)) {
        throw std::runtime_error("Failed to load engine: " + model_path);
    }
    
    if (!config_path.empty()) {
        loadConfig(config_path);
    }
    
    LOG_INFO(log_path_, "[YoloSegTensorRT] Initialized with engine: %s", model_path.c_str());
}

// 析构函数
YoloSegTensorRT::~YoloSegTensorRT() {
    freeBuffers();
}

void YoloSegTensorRT::freeBuffers() {
    if (input_buffer_) { CHECK_CUDA(cudaFree(input_buffer_)); input_buffer_ = nullptr; }
    if (proto_buffer_) { CHECK_CUDA(cudaFree(proto_buffer_)); proto_buffer_ = nullptr; }
    if (output_buffer_) { CHECK_CUDA(cudaFree(output_buffer_)); output_buffer_ = nullptr; }
    if (d_decode_out_) { CHECK_CUDA(cudaFree(d_decode_out_)); d_decode_out_ = nullptr; }
    if (d_keep_count_) { CHECK_CUDA(cudaFree(d_keep_count_)); d_keep_count_ = nullptr; }
    if (stream_) { CHECK_CUDA(cudaStreamDestroy(stream_)); stream_ = nullptr; }
    if (d_src_) { CHECK_CUDA(cudaFree(d_src_)); d_src_ = nullptr; }
}

bool YoloSegTensorRT::loadEngine(const std::string& engine_path) {
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file.is_open()) {
        LOG_ERROR(log_path_, "Failed to open engine file: %s", engine_path.c_str());
        return false;
    }

    engine_file.seekg(0, std::ios::end);
    size_t engine_size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);

    std::vector<char> engine_data(engine_size);
    engine_file.read(engine_data.data(), engine_size);
    engine_file.close();

    runtime_.reset(nvinfer1::createInferRuntime(gLogger));
    if (!runtime_) {
        LOG_ERROR(log_path_, "Failed to create TensorRT runtime");
        return false;
    }

    engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_size));
    if (!engine_) {
        LOG_ERROR(log_path_, "Failed to deserialize CUDA engine");
        return false;
    }

    context_.reset(engine_->createExecutionContext());
    if (!context_) {
        LOG_ERROR(log_path_, "Failed to create execution context");
        return false;
    }

    // 解析输入输出tensor信息
    int num_io = engine_->getNbIOTensors();
    LOG_INFO(log_path_, "Number of I/O tensors: %d", num_io);
    
    for (int i = 0; i < num_io; i++) {
        const char* name = engine_->getIOTensorName(i);
        auto mode = engine_->getTensorIOMode(name);
        std::string mode_str = (mode == nvinfer1::TensorIOMode::kINPUT) ? "INPUT" : "OUTPUT";
        
        auto dims = engine_->getTensorShape(name);
        std::string shape_str = "[";
        for (int j = 0; j < dims.nbDims; j++) {
            shape_str += std::to_string(dims.d[j]);
            if (j < dims.nbDims - 1) shape_str += ", ";
        }
        shape_str += "]";
        
        LOG_INFO(log_path_, "Tensor %d: name=%s, mode=%s, shape=%s", 
                 i, name, mode_str.c_str(), shape_str.c_str());

        if (std::string(name) == "images") {
            input_index_ = i;
        } else if (std::string(name) == "output1") {
            proto_index_ = i;
        } else if (std::string(name) == "output0") {
            output_index_ = i;
        }
    }

    // 获取输入尺寸
    auto input_dims = engine_->getTensorShape("images");
    input_h_ = input_dims.d[2];
    input_w_ = input_dims.d[3];

    // 获取proto尺寸
    auto proto_dims = engine_->getTensorShape("output1");
    proto_c_ = proto_dims.d[1];
    proto_h_ = proto_dims.d[2];
    proto_w_ = proto_dims.d[3];

    // 获取输出尺寸
    auto output_dims = engine_->getTensorShape("output0");
    num_boxes_ = output_dims.d[2];
    num_classes_ = output_dims.d[1] - 4 - 32;

    // 准备CUDA buffers
    if (!prepareBuffers()) {
        return false;
    }

    initialized_ = true;
    return true;
}

bool YoloSegTensorRT::prepareBuffers() {
    CHECK_CUDA(cudaStreamCreate(&stream_));
    
    // Input buffer
    CHECK_CUDA(cudaMalloc(&input_buffer_, 3 * input_h_ * input_w_ * sizeof(float)));

    // Proto buffer
    CHECK_CUDA(cudaMalloc(&proto_buffer_, proto_c_ * proto_h_ * proto_w_ * sizeof(float)));

    // Output buffer
    int output_size = (4 + num_classes_ + 32) * num_boxes_;
    CHECK_CUDA(cudaMalloc(&output_buffer_, output_size * sizeof(float)));
    
    // GPU postprocess buffers
    CHECK_CUDA(cudaMalloc(&d_decode_out_, num_boxes_ * sizeof(Bbox32)));
    CHECK_CUDA(cudaMalloc(&d_keep_count_, sizeof(int)));
    
    return true;
}

bool YoloSegTensorRT::loadConfig(const std::string& config_path) {
    return loadClassNames(config_path);
}

bool YoloSegTensorRT::loadClassNames(const std::string& config_path) {
    std::string names_path = config_path;
    size_t dot_pos = names_path.rfind('.');
    if (dot_pos != std::string::npos) {
        names_path = names_path.substr(0, dot_pos) + ".names";
    } else {
        names_path += ".names";
    }

    std::ifstream f(names_path);
    if (!f.is_open()) {
        LOG_WARN(log_path_, "Failed to open class names file: %s", names_path.c_str());
        return false;
    }

    class_names_.clear();
    std::string line;
    while (std::getline(f, line)) {
        size_t end = line.find_last_not_of(" \t\r\n");
        if (end != std::string::npos) {
            line = line.substr(0, end + 1);
        }
        if (!line.empty()) {
            class_names_.push_back(line);
        }
    }

    LOG_INFO(log_path_, "Loaded %zu class names from %s", class_names_.size(), names_path.c_str());
    return !class_names_.empty();
}

void YoloSegTensorRT::setThreshold(float conf_thresh, float nms_thresh) {
    config_.conf_threshold = conf_thresh;
    config_.nms_threshold = nms_thresh;
}

void YoloSegTensorRT::setMaskThreshold(float mask_thresh) {
    config_.mask_threshold = mask_thresh;
}

std::vector<std::string> YoloSegTensorRT::getSupportedClasses() const {
    return class_names_;
}

// CPU版本预处理
void YoloSegTensorRT::preprocess(const cv::Mat& img) {
    if (img.empty()) {
        throw std::runtime_error("Input image is empty");
    }

    int src_h = img.rows;
    int src_w = img.cols;

    double r = std::min((double)input_w_ / src_w, (double)input_h_ / src_h);
    int new_w = (int)std::round(src_w * r);
    int new_h = (int)std::round(src_h * r);

    if (new_w <= 0 || new_h <= 0) {
        throw std::runtime_error("Invalid resize size: " + std::to_string(new_w) + "x" + std::to_string(new_h));
    }

    dw_ = ((double)input_w_ - new_w) / 2.0;
    dh_ = ((double)input_h_ - new_h) / 2.0;
    ratio_ = r;

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h));

    cv::Mat padded;
    int top = (int)dh_;
    int bottom = input_h_ - new_h - top;
    int left = (int)dw_;
    int right = input_w_ - new_w - left;
    cv::copyMakeBorder(resized, padded, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    cv::Mat float_img;
    padded.convertTo(float_img, CV_32FC3, 1.0 / 255.0);
    cv::cvtColor(float_img, float_img, cv::COLOR_BGR2RGB);

    std::vector<float> chw_data(3 * input_h_ * input_w_);
    std::vector<cv::Mat> channels;
    cv::split(float_img, channels);
    for (int c = 0; c < 3; c++) {
        std::memcpy(chw_data.data() + c * input_h_ * input_w_, channels[c].data, input_h_ * input_w_ * sizeof(float));
    }

    CHECK_CUDA(cudaMemcpyAsync(input_buffer_, chw_data.data(),
                              3 * input_h_ * input_w_ * sizeof(float), cudaMemcpyHostToDevice, stream_));
}

// GPU版本预处理
void YoloSegTensorRT::preprocess_gpu(const cv::Mat& img) {
    if (img.empty()) {
        throw std::runtime_error("Input image is empty");
    }

    int src_h = img.rows;
    int src_w = img.cols;

    double r = std::min((double)input_w_ / src_w, (double)input_h_ / src_h);
    int new_w = (int)std::round(src_w * r);
    int new_h = (int)std::round(src_h * r);

    if (new_w <= 0 || new_h <= 0) {
        throw std::runtime_error("Invalid resize size: " + std::to_string(new_w) + "x" + std::to_string(new_h));
    }

    dw_ = ((double)input_w_ - new_w) / 2.0;
    dh_ = ((double)input_h_ - new_h) / 2.0;
    ratio_ = r;

    // Upload image to GPU
    size_t img_size = src_h * src_w * 3;
    if (d_src_size_ < img_size) {
        if (d_src_) { CHECK_CUDA(cudaFree(d_src_)); }
        CHECK_CUDA(cudaMalloc(&d_src_, img_size));
        d_src_size_ = img_size;
    }
    CHECK_CUDA(cudaMemcpy(d_src_, img.data, img_size, cudaMemcpyHostToDevice));

    // Launch CUDA preprocess kernel
    launch_preprocess_kernel(
        (const unsigned char*)d_src_, src_h, src_w,
        (float*)input_buffer_, input_h_, input_w_,
        (float)r, (int)dh_, (int)dw_, 1.0f / 255.0f,
        stream_
    );
}

// CPU版本后处理
void YoloSegTensorRT::postprocess(std::vector<SegmentationResult>& results,
                                   int img_width, int img_height) {
    // 1. GPU -> CPU
    int output_size = (4 + num_classes_ + 32) * num_boxes_;
    std::vector<float> output(output_size);
    CHECK_CUDA(cudaMemcpy(output.data(), output_buffer_,
                         output_size * sizeof(float), cudaMemcpyDeviceToHost));

    int proto_size = proto_c_ * proto_h_ * proto_w_;
    std::vector<float> proto_data(proto_size);
    CHECK_CUDA(cudaMemcpy(proto_data.data(), proto_buffer_,
                         proto_size * sizeof(float), cudaMemcpyDeviceToHost));

    // 2. CPU decode
    for (int i = 0; i < num_boxes_; ++i) {
        float max_score = 0;
        int max_class = 0;
        for (int c = 0; c < num_classes_; c++) {
            float score = output[(4 + c) * num_boxes_ + i];
            if (score > max_score) {
                max_score = score;
                max_class = c;
            }
        }

        if (max_score < config_.conf_threshold) continue;

        float cx = output[0 * num_boxes_ + i];
        float cy = output[1 * num_boxes_ + i];
        float w = output[2 * num_boxes_ + i];
        float h = output[3 * num_boxes_ + i];

        float x1 = (cx - w / 2 - dw_) / ratio_;
        float y1 = (cy - h / 2 - dh_) / ratio_;
        float x2 = (cx + w / 2 - dw_) / ratio_;
        float y2 = (cy + h / 2 - dh_) / ratio_;

        x1 = std::max(0.0f, std::min((float)img_width - 1, x1));
        y1 = std::max(0.0f, std::min((float)img_height - 1, y1));
        x2 = std::max(0.0f, std::min((float)img_width - 1, x2));
        y2 = std::max(0.0f, std::min((float)img_height - 1, y2));

        if (x1 >= x2 || y1 >= y2) continue;

        SegmentationResult result;
        result.center = cv::Point2f((x1 + x2) / 2, (y1 + y2) / 2);
        result.width = x2 - x1;
        result.height = y2 - y1;
        result.confidence = max_score;
        result.class_id = max_class;
        
        if (max_class < static_cast<int>(class_names_.size())) {
            result.class_name = class_names_[max_class];
        }
        
        for (int m = 0; m < 32; m++) {
            result.mask_coef[m] = output[(4 + num_classes_ + m) * num_boxes_ + i];
        }
        
        results.push_back(result);
    }

    // 3. CPU NMS
    if (!results.empty()) {
        std::sort(results.begin(), results.end(),
                  [](const SegmentationResult& a, const SegmentationResult& b) { 
                      return a.confidence > b.confidence; 
                  });

        std::vector<SegmentationResult> keep;
        std::vector<bool> suppressed(results.size(), false);

        for (size_t i = 0; i < results.size(); ++i) {
            if (suppressed[i]) continue;
            keep.push_back(results[i]);

            cv::Rect bbox_i = results[i].getBBox();
            float area_i = results[i].width * results[i].height;

            for (size_t j = i + 1; j < results.size(); ++j) {
                if (suppressed[j]) continue;

                cv::Rect bbox_j = results[j].getBBox();
                cv::Rect inter = bbox_i & bbox_j;
                float inter_area = inter.width * inter.height;
                float area_j = results[j].width * results[j].height;
                float iou = inter_area / (area_i + area_j - inter_area + 1e-6f);

                if (iou > config_.nms_threshold) {
                    suppressed[j] = true;
                }
            }
        }
        results = std::move(keep);
    }

    // 4. Process masks on CPU
    if (!results.empty()) {
        process_mask(proto_data.data(), results, img_width, img_height);
    }
}

// GPU版本后处理 - GPU做decode+NMS，CPU做mask（参考yolo_seg_alg）
void YoloSegTensorRT::postprocess_gpu(std::vector<SegmentationResult>& results,
                                       int img_width, int img_height) {
    // 1. 拷贝proto数据到CPU
    int proto_size = proto_c_ * proto_h_ * proto_w_;
    std::vector<float> proto_data(proto_size);
    CHECK_CUDA(cudaMemcpy(proto_data.data(), proto_buffer_,
                         proto_size * sizeof(float), cudaMemcpyDeviceToHost));

    // 2. 使用GPU做decode+NMS
    std::vector<Bbox32> gpu_boxes;
    yolo_seg_postprocess_gpu(
        (float*)output_buffer_, num_boxes_, num_classes_,
        config_.conf_threshold, config_.nms_threshold,
        ratio_, dw_, dh_,
        d_decode_out_, d_keep_count_,
        gpu_boxes, stream_
    );

    // 3. Convert Bbox32 to SegmentationResult（添加边界检查）
    for (const auto& box : gpu_boxes) {
        SegmentationResult result;

        // Clamp to image bounds（参考yolo_seg_alg）
        float x1 = std::max(0.0f, std::min((float)img_width - 1, box.x1));
        float y1 = std::max(0.0f, std::min((float)img_height - 1, box.y1));
        float x2 = std::max(0.0f, std::min((float)img_width - 1, box.x2));
        float y2 = std::max(0.0f, std::min((float)img_height - 1, box.y2));

        result.center = cv::Point2f((x1 + x2) / 2, (y1 + y2) / 2);
        result.width = x2 - x1;
        result.height = y2 - y1;
        result.confidence = box.conf;
        result.class_id = box.cls;
        
        if (box.cls < static_cast<int>(class_names_.size())) {
            result.class_name = class_names_[box.cls];
        }
        
        for (int m = 0; m < 32; m++) {
            result.mask_coef[m] = box.mask[m];
        }

        results.push_back(result);
    }

    // 4. 使用CPU处理masks（参考yolo_seg_alg的实现）
    if (!results.empty()) {
        process_mask(proto_data.data(), results, img_width, img_height);
    }
}

// CPU版本 mask处理 - 优化版本
void YoloSegTensorRT::process_mask(float* proto_host, std::vector<SegmentationResult>& results,
                                    int img_width, int img_height) {
    int n = results.size();
    if (n == 0) return;

    float scale_x = (float)proto_w_ / input_w_;
    float scale_y = (float)proto_h_ / input_h_;

    // 优化1: 为每个检测框单独处理，只计算 bbox 区域内的 mask
    #pragma omp parallel for schedule(dynamic)  // OpenMP 并行化
    for (int i = 0; i < n; i++) {
        // 计算 bbox 在 proto 上的范围
        cv::Point2f tl = results[i].topLeft();
        float x1_f = (tl.x * ratio_ + dw_) * scale_x;
        float y1_f = (tl.y * ratio_ + dh_) * scale_y;
        float x2_f = ((tl.x + results[i].width) * ratio_ + dw_) * scale_x;
        float y2_f = ((tl.y + results[i].height) * ratio_ + dh_) * scale_y;

        int x1 = std::max(0, (int)floorf(x1_f));
        int y1 = std::max(0, (int)floorf(y1_f));
        int x2 = std::min(proto_w_ - 1, (int)ceilf(x2_f));
        int y2 = std::min(proto_h_ - 1, (int)ceilf(y2_f));

        int bbox_w = x2 - x1 + 1;
        int bbox_h = y2 - y1 + 1;

        if (bbox_w <= 0 || bbox_h <= 0) continue;

        // 只分配 bbox 区域的 mask
        std::vector<float> mask_region(bbox_h * bbox_w);

        const float* mask_coef_ptr = results[i].mask_coef;

        // 计算 mask 时只处理 bbox 区域
        for (int h = y1; h <= y2; h++) {
            for (int w = x1; w <= x2; w++) {
                float val = 0.0f;

                #pragma unroll 8
                for (int c = 0; c < proto_c_; c++) {
                    val += mask_coef_ptr[c] * proto_host[c * proto_h_ * proto_w_ + h * proto_w_ + w];
                }

                mask_region[(h - y1) * bbox_w + (w - x1)] = fast_sigmoid(val);
            }
        }

        // 切除padding区域
        int cut_left = (int)(dw_ * scale_x);
        int cut_top = (int)(dh_ * scale_y);
        int cut_w = proto_w_ - 2 * cut_left;
        int cut_h = proto_h_ - 2 * cut_top;

        // 分配原图大小的 mask
        results[i].mask = cv::Mat::zeros(img_height, img_width, CV_8UC1);

        // 并行化双线性插值
        #pragma omp parallel for collapse(2) schedule(static)
        for (int y = 0; y < img_height; y++) {
            for (int x = 0; x < img_width; x++) {
                float src_x = (x * ratio_ + dw_) * scale_x;
                float src_y = (y * ratio_ + dh_) * scale_y;

                if (src_x >= x1 && src_x <= x2 && src_y >= y1 && src_y <= y2) {
                    int x0 = (int)src_x;
                    int y0 = (int)src_y;
                    int x0_1 = std::min(x0 + 1, x2);
                    int y0_1 = std::min(y0 + 1, y2);

                    float dx = src_x - x0;
                    float dy = src_y - y0;

                    int idx00 = (y0 - y1) * bbox_w + (x0 - x1);
                    int idx01 = (y0 - y1) * bbox_w + (x0_1 - x1);
                    int idx10 = (y0_1 - y1) * bbox_w + (x0 - x1);
                    int idx11 = (y0_1 - y1) * bbox_w + (x0_1 - x1);

                    float v00 = mask_region[idx00];
                    float v01 = mask_region[idx01];
                    float v10 = mask_region[idx10];
                    float v11 = mask_region[idx11];

                    float value = v00 * (1.0f - dx) * (1.0f - dy) +
                                  v01 * dx * (1.0f - dy) +
                                  v10 * (1.0f - dx) * dy +
                                  v11 * dx * dy;

                    if (value > config_.mask_threshold) {
                        results[i].mask.at<uchar>(y, x) = 255;
                    }
                }
            }
        }
    }
}

// GPU版本 mask处理（参考yolo_seg_alg）
void YoloSegTensorRT::process_mask_gpu(float* proto_device, std::vector<SegmentationResult>& results,
                                        int img_width, int img_height) {
    int n = results.size();
    if (n == 0) return;

    // 准备mask系数数据
    std::vector<float> h_mask_coef(n * proto_c_);
    for (int i = 0; i < n; i++) {
        memcpy(&h_mask_coef[i * proto_c_], results[i].mask_coef, proto_c_ * sizeof(float));
    }
    
    // 准备bbox坐标数据
    std::vector<float> h_bboxes(n * 4);
    for (int i = 0; i < n; i++) {
        cv::Point2f tl = results[i].topLeft();
        h_bboxes[i * 4 + 0] = tl.x;
        h_bboxes[i * 4 + 1] = tl.y;
        h_bboxes[i * 4 + 2] = tl.x + results[i].width;
        h_bboxes[i * 4 + 3] = tl.y + results[i].height;
    }
    
    // 分配GPU内存
    float* d_mask_coef = nullptr;
    float* d_bboxes = nullptr;
    float* d_output_masks = nullptr;
    
    int mask_coef_size = n * proto_c_ * sizeof(float);
    int bboxes_size = n * 4 * sizeof(float);
    int output_size = n * img_height * img_width * sizeof(float);
    
    CHECK_CUDA(cudaMalloc(&d_mask_coef, mask_coef_size));
    CHECK_CUDA(cudaMalloc(&d_bboxes, bboxes_size));
    CHECK_CUDA(cudaMalloc(&d_output_masks, output_size));
    
    // 拷贝数据到GPU
    CHECK_CUDA(cudaMemcpy(d_mask_coef, h_mask_coef.data(), mask_coef_size, cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy(d_bboxes, h_bboxes.data(), bboxes_size, cudaMemcpyHostToDevice));
    
    // 调用CUDA kernel处理mask
    launch_process_mask_kernel(
        proto_device,
        proto_h_, proto_w_,
        d_mask_coef,
        d_bboxes,
        n,
        ratio_, dw_, dh_,
        input_w_, input_h_,
        d_output_masks,
        img_width, img_height,
        config_.mask_threshold,
        nullptr,  // 暂时不使用GPU二值化
        stream_
    );
    
    // 拷贝结果回CPU
    std::vector<float> h_output_masks(n * img_height * img_width);
    CHECK_CUDA(cudaMemcpy(h_output_masks.data(), d_output_masks, output_size, cudaMemcpyDeviceToHost));
    
    // 分配结果到各个result（在CPU上进行二值化，参考yolo_seg_alg）
    for (int i = 0; i < n; i++) {
        results[i].mask = cv::Mat::zeros(img_height, img_width, CV_8UC1);
        for (int y = 0; y < img_height; y++) {
            for (int x = 0; x < img_width; x++) {
                float val = h_output_masks[i * img_height * img_width + y * img_width + x];
                if (val > config_.mask_threshold) {
                    results[i].mask.at<uchar>(y, x) = 255;
                }
            }
        }
    }
    
    // 释放GPU内存
    CHECK_CUDA(cudaFree(d_mask_coef));
    CHECK_CUDA(cudaFree(d_bboxes));
    CHECK_CUDA(cudaFree(d_output_masks));
}

std::vector<SegmentationResult> YoloSegTensorRT::segment(const cv::Mat& image) {
    if (image.empty() || !initialized_) {
        return {};
    }

    // 根据配置选择预处理方式
    if (config_.use_gpu_preprocess) {
        preprocess_gpu(image);
    } else {
        preprocess(image);
    }

    // TensorRT推理
    context_->setTensorAddress("images", input_buffer_);
    context_->setTensorAddress("output1", proto_buffer_);
    context_->setTensorAddress("output0", output_buffer_);

    context_->enqueueV3(stream_);
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    std::vector<SegmentationResult> results;
    
    // 根据配置选择后处理方式
    if (config_.use_gpu_postprocess) {
        postprocess_gpu(results, image.cols, image.rows);
    } else {
        postprocess(results, image.cols, image.rows);
    }

    return results;
}

void YoloSegTensorRT::drawResults(cv::Mat& image,
                                  const std::vector<SegmentationResult>& results) const {
    for (const auto& result : results) {
        cv::Scalar color = getColorByClassId(result.class_id);

        // 绘制边界框
        cv::Rect bbox = result.getBBox();
        cv::rectangle(image, bbox, color, 2);

        // 绘制标签
        std::string label = result.class_name.empty() ?
                           "cls:" + std::to_string(result.class_id) + " " +
                           std::to_string(result.confidence).substr(0, 4) :
                           result.class_name + " " +
                           std::to_string(result.confidence).substr(0, 4);

        int baseline;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
        cv::Point text_origin(bbox.x, bbox.y - text_size.height - 5);
        cv::rectangle(image, text_origin + cv::Point(0, baseline),
                     text_origin + cv::Point(text_size.width, -text_size.height),
                     color, -1);
        cv::putText(image, label, text_origin, cv::FONT_HERSHEY_SIMPLEX,
                   0.6, cv::Scalar(255, 255, 255), 2);

        // 绘制掩码
        if (!result.mask.empty()) {
            cv::Mat colored_mask = cv::Mat::zeros(image.size(), image.type());
            colored_mask.setTo(color, result.mask);
            cv::addWeighted(image, 0.7, colored_mask, 0.3, 0, image);
        }
    }
}

void YoloSegTensorRT::drawResultsWithDepth(cv::Mat& image,
                                           const std::vector<SegmentationResult>& results,
                                           const cv::Mat& depth_image,
                                           const CameraIntrinsics& intrinsics) const {
    for (const auto& result : results) {
        cv::Scalar color = getColorByClassId(result.class_id);

        // 绘制边界框
        cv::Rect bbox = result.getBBox();
        cv::rectangle(image, bbox, color, 2);

        // 计算3D坐标
        std::string pos_text;
        if (!depth_image.empty() && intrinsics.fx > 0) {
            int pixel_x = static_cast<int>(result.center.x);
            int pixel_y = static_cast<int>(result.center.y);

            if (pixel_x >= 0 && pixel_x < depth_image.cols &&
                pixel_y >= 0 && pixel_y < depth_image.rows) {
                float depth = 0;
                if (depth_image.type() == CV_16UC1) {
                    depth = depth_image.at<uint16_t>(pixel_y, pixel_x) / 1000.0f;
                } else if (depth_image.type() == CV_32FC1) {
                    depth = depth_image.at<float>(pixel_y, pixel_x);
                }

                if (depth > 0) {
                    float x3d = (result.center.x - intrinsics.cx) * depth / intrinsics.fx;
                    float y3d = (result.center.y - intrinsics.cy) * depth / intrinsics.fy;
                    float z3d = depth;

                    char buf[128];
                    snprintf(buf, sizeof(buf), "(%.2f,%.2f,%.2f)m", x3d, y3d, z3d);
                    pos_text = std::string(buf);
                }
            }
        }

        // 绘制标签
        std::string label = result.class_name.empty() ?
                           "cls:" + std::to_string(result.class_id) :
                           result.class_name;
        label += " " + std::to_string(result.confidence).substr(0, 4);
        if (!pos_text.empty()) {
            label += " " + pos_text;
        }

        int baseline;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
        cv::Point text_origin(bbox.x, bbox.y - text_size.height - 5);
        cv::rectangle(image, text_origin + cv::Point(0, baseline),
                     text_origin + cv::Point(text_size.width, -text_size.height),
                     color, -1);
        cv::putText(image, label, text_origin, cv::FONT_HERSHEY_SIMPLEX,
                   0.6, cv::Scalar(255, 255, 255), 2);

        // 绘制掩码
        if (!result.mask.empty()) {
            cv::Mat colored_mask = cv::Mat::zeros(image.size(), image.type());
            colored_mask.setTo(color, result.mask);
            cv::addWeighted(image, 0.7, colored_mask, 0.3, 0, image);
        }
    }
}

cv::Scalar YoloSegTensorRT::getColorByClassId(int class_id) const {
    static const std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255),
        cv::Scalar(128, 0, 255), cv::Scalar(255, 128, 0), cv::Scalar(0, 128, 255)
    };
    return colors[class_id % colors.size()];
}
