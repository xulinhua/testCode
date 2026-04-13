#include "yolo_obb_det_tensorrt.h"
#include "task_factory.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cstring>
#include "bas_operate/file_operate.hpp"

// 静态注册 YOLO OBB 检测算法（TensorRT 实现）
namespace {
    struct YoloOBBRegistrar {
        YoloOBBRegistrar() {
            TaskFactory::registerOBBDetectionAlgorithm(
                "yolo_obb",
                InferenceEngineType::TENSORRT,
                [](const std::string& model_path,
                   const std::string& config_path,
                   const InferenceEngineConfig& engine_config) -> std::unique_ptr<IOBBDetectionTask> {
                    return std::make_unique<YoloOBBDetTensorRT>(model_path, config_path, engine_config);
                });
        }
    };
    static YoloOBBRegistrar registrar;
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

// CUDA 错误检查
void YoloOBBDetTensorRT::checkCUDA(cudaError_t err, const char* file, int line) {
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("CUDA error at ") + file + ":" +
                               std::to_string(line) + ": " +
                               cudaGetErrorString(err));
    }
}

// 默认构造函数
YoloOBBDetTensorRT::YoloOBBDetTensorRT()
    : stream_(nullptr),
      initialized_(false),
      log_path_("yolo_obb_tensorrt"),
      d_src_(nullptr),
      d_src_size_(0) {
    
    config_.conf_threshold = 0.5f;
    config_.nms_threshold = 0.45f;
    config_.use_gpu_preprocess = true;
    config_.use_gpu_postprocess = true;
}

// 单参数构造函数
YoloOBBDetTensorRT::YoloOBBDetTensorRT(const std::string& engine_path)
    : YoloOBBDetTensorRT() {
    loadEngine(engine_path);
}

// 完整构造函数（工厂调用）
YoloOBBDetTensorRT::YoloOBBDetTensorRT(const std::string& model_path,
                                        const std::string& config_path,
                                        const InferenceEngineConfig& engine_config)
    : YoloOBBDetTensorRT() {
    
    log_path_ = basmodule::get_project_name_by_file_path(__FILE__);
    config_.engine_config = engine_config;
    
    if (!loadEngine(model_path)) {
        throw std::runtime_error("Failed to load engine: " + model_path);
    }
    
    if (!config_path.empty()) {
        loadConfig(config_path);
    }
    
    LOG_INFO(log_path_, "[YoloOBBDetTensorRT] Initialized with engine: %s", model_path.c_str());
}

// 析构函数
YoloOBBDetTensorRT::~YoloOBBDetTensorRT() {
    freeBuffers();
}

void YoloOBBDetTensorRT::freeBuffers() {
    if (input_buffer_) { CHECK_CUDA(cudaFree(input_buffer_)); input_buffer_ = nullptr; }
    if (output_buffer_) { CHECK_CUDA(cudaFree(output_buffer_)); output_buffer_ = nullptr; }
    if (d_decode_out_) { CHECK_CUDA(cudaFree(d_decode_out_)); d_decode_out_ = nullptr; }
    if (d_keep_count_) { CHECK_CUDA(cudaFree(d_keep_count_)); d_keep_count_ = nullptr; }
    if (stream_) { CHECK_CUDA(cudaStreamDestroy(stream_)); stream_ = nullptr; }
    if (d_src_) { CHECK_CUDA(cudaFree(d_src_)); d_src_ = nullptr; }
}

bool YoloOBBDetTensorRT::loadEngine(const std::string& engine_path) {
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
        } else if (std::string(name) == "output0") {
            output_index_ = i;
        }
    }

    // 获取输入尺寸
    auto input_dims = engine_->getTensorShape("images");
    input_h_ = input_dims.d[2];
    input_w_ = input_dims.d[3];

    // 获取输出尺寸
    auto output_dims = engine_->getTensorShape("output0");
    // OBB检测：输出格式 [1, 5 + num_classes, num_boxes]
    // 5 = cx, cy, w, h, angle
    num_classes_ = output_dims.d[1] - 5;  // 减去5个bbox参数（含angle）
    num_boxes_ = output_dims.d[2];

    // 准备CUDA buffers
    if (!prepareBuffers()) {
        return false;
    }

    initialized_ = true;
    return true;
}

bool YoloOBBDetTensorRT::prepareBuffers() {
    CHECK_CUDA(cudaStreamCreate(&stream_));
    
    // Input buffer
    CHECK_CUDA(cudaMalloc(&input_buffer_, 3 * input_h_ * input_w_ * sizeof(float)));

    // Output buffer
    int output_size = (5 + num_classes_) * num_boxes_;  // cx,cy,w,h,angle + classes
    CHECK_CUDA(cudaMalloc(&output_buffer_, output_size * sizeof(float)));
    
    // GPU postprocess buffers
    CHECK_CUDA(cudaMalloc(&d_decode_out_, num_boxes_ * sizeof(OBBBox)));
    CHECK_CUDA(cudaMalloc(&d_keep_count_, sizeof(int)));
    
    return true;
}

bool YoloOBBDetTensorRT::loadConfig(const std::string& config_path) {
    return loadClassNames(config_path);
}

bool YoloOBBDetTensorRT::loadClassNames(const std::string& config_path) {
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

void YoloOBBDetTensorRT::setThreshold(float conf_thresh, float nms_thresh) {
    config_.conf_threshold = conf_thresh;
    config_.nms_threshold = nms_thresh;
}

std::vector<std::string> YoloOBBDetTensorRT::getSupportedClasses() const {
    return class_names_;
}

// CPU版本预处理
void YoloOBBDetTensorRT::preprocess(const cv::Mat& img) {
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
void YoloOBBDetTensorRT::preprocess_gpu(const cv::Mat& img) {
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
void YoloOBBDetTensorRT::postprocess(std::vector<OBBDetectionResult>& results,
                                      int img_width, int img_height) {
    // 1. GPU -> CPU
    int output_size = (5 + num_classes_) * num_boxes_;
    std::vector<float> output(output_size);
    CHECK_CUDA(cudaMemcpy(output.data(), output_buffer_,
                         output_size * sizeof(float), cudaMemcpyDeviceToHost));

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

        float cx = (output[0 * num_boxes_ + i] - dw_) / ratio_;
        float cy = (output[1 * num_boxes_ + i] - dh_) / ratio_;
        float w = output[2 * num_boxes_ + i] / ratio_;
        float h = output[3 * num_boxes_ + i] / ratio_;
        float angle = output[(4 + num_classes_) * num_boxes_ + i];  // 角度信息

        OBBDetectionResult result;
        result.center = cv::Point2f(cx, cy);
        result.width = w;
        result.height = h;
        result.angle = angle;
        result.confidence = max_score;
        result.class_id = max_class;
        
        if (max_class < static_cast<int>(class_names_.size())) {
            result.class_name = class_names_[max_class];
        }
        
        results.push_back(result);
    }

    // 3. NMS for OBB (使用旋转框IoU)
    if (!results.empty()) {
        std::sort(results.begin(), results.end(),
                  [](const OBBDetectionResult& a, const OBBDetectionResult& b) { 
                      return a.confidence > b.confidence; 
                  });

        std::vector<OBBDetectionResult> keep;
        std::vector<bool> suppressed(results.size(), false);

        for (size_t i = 0; i < results.size(); ++i) {
            if (suppressed[i]) continue;
            keep.push_back(results[i]);

            cv::RotatedRect rect_i = results[i].getRotatedRect();
            float area_i = results[i].width * results[i].height;

            for (size_t j = i + 1; j < results.size(); ++j) {
                if (suppressed[j]) continue;
                if (results[j].class_id != results[i].class_id) continue;

                cv::RotatedRect rect_j = results[j].getRotatedRect();
                
                // 计算旋转框的交集
                std::vector<cv::Point2f> intersection;
                int inter_result = cv::rotatedRectangleIntersection(rect_i, rect_j, intersection);
                
                if (inter_result == cv::INTERSECT_NONE) continue;
                
                float inter_area = 0.0f;
                if (inter_result == cv::INTERSECT_FULL) {
                    inter_area = std::min(area_i, results[j].width * results[j].height);
                } else if (intersection.size() >= 3) {
                    inter_area = cv::contourArea(intersection);
                }
                
                float area_j = results[j].width * results[j].height;
                float iou = inter_area / (area_i + area_j - inter_area + 1e-6f);

                if (iou > config_.nms_threshold) {
                    suppressed[j] = true;
                }
            }
        }
        results = std::move(keep);
    }
}

// GPU版本后处理
void YoloOBBDetTensorRT::postprocess_gpu(std::vector<OBBDetectionResult>& results,
                                          int img_width, int img_height) {
    // 使用GPU做decode+NMS
    std::vector<OBBBox> gpu_boxes;
    yolo_obb_postprocess_gpu(
        (float*)output_buffer_, num_boxes_, num_classes_,
        config_.conf_threshold, config_.nms_threshold,
        ratio_, dw_, dh_,
        d_decode_out_, d_keep_count_,
        gpu_boxes, stream_
    );

    // Convert OBBBox to OBBDetectionResult
    for (const auto& box : gpu_boxes) {
        OBBDetectionResult result;

        // Clamp to image bounds
        result.center = cv::Point2f(
            std::max(0.0f, std::min((float)img_width - 1, box.cx)),
            std::max(0.0f, std::min((float)img_height - 1, box.cy))
        );
        result.width = box.w;
        result.height = box.h;
        result.angle = box.angle;
        result.confidence = box.conf;
        result.class_id = box.cls;
        
        if (box.cls < static_cast<int>(class_names_.size())) {
            result.class_name = class_names_[box.cls];
        }

        results.push_back(result);
    }
}

std::vector<OBBDetectionResult> YoloOBBDetTensorRT::detect(const cv::Mat& image) {
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
    context_->setTensorAddress("output0", output_buffer_);

    context_->enqueueV3(stream_);
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    std::vector<OBBDetectionResult> results;
    
    // 根据配置选择后处理方式
    if (config_.use_gpu_postprocess) {
        postprocess_gpu(results, image.cols, image.rows);
    } else {
        postprocess(results, image.cols, image.rows);
    }

    return results;
}

cv::Scalar YoloOBBDetTensorRT::getColorByClassId(int class_id) const {
    static const std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255),
        cv::Scalar(128, 0, 255), cv::Scalar(255, 128, 0), cv::Scalar(0, 128, 255)
    };
    return colors[class_id % colors.size()];
}

void YoloOBBDetTensorRT::drawResults(cv::Mat& image,
                                      const std::vector<OBBDetectionResult>& results) const {
    for (const auto& result : results) {
        cv::Scalar color = getColorByClassId(result.class_id);

        // 绘制旋转框
        cv::RotatedRect rotated_rect = result.getRotatedRect();
        cv::Point2f vertices[4];
        rotated_rect.points(vertices);
        
        for (int i = 0; i < 4; i++) {
            cv::line(image, vertices[i], vertices[(i + 1) % 4], color, 2);
        }

        // 绘制标签
        std::string label = result.class_name.empty() ?
                           "cls:" + std::to_string(result.class_id) + " " +
                           std::to_string(result.confidence).substr(0, 4) :
                           result.class_name + " " +
                           std::to_string(result.confidence).substr(0, 4);

        // 添加角度信息
        label += " " + std::to_string(result.angle * 180.0f / CV_PI).substr(0, 5) + "deg";

        int baseline;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::Point text_origin(result.center.x - text_size.width / 2, 
                              result.center.y - result.height / 2 - 5);
        
        cv::rectangle(image, 
                     cv::Point(text_origin.x, text_origin.y - text_size.height - 5),
                     cv::Point(text_origin.x + text_size.width, text_origin.y),
                     color, cv::FILLED);
        cv::putText(image, label, 
                   cv::Point(text_origin.x, text_origin.y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
}

void YoloOBBDetTensorRT::drawResultsWithDepth(cv::Mat& image,
                                               const std::vector<OBBDetectionResult>& results,
                                               const cv::Mat& depth_image,
                                               const CameraIntrinsics& intrinsics) const {
    for (const auto& result : results) {
        cv::Scalar color = getColorByClassId(result.class_id);

        // 绘制旋转框
        cv::RotatedRect rotated_rect = result.getRotatedRect();
        cv::Point2f vertices[4];
        rotated_rect.points(vertices);
        
        for (int i = 0; i < 4; i++) {
            cv::line(image, vertices[i], vertices[(i + 1) % 4], color, 2);
        }

        // 绘制标签
        std::string label = result.class_name.empty() ?
                           "cls:" + std::to_string(result.class_id) :
                           result.class_name;
        label += " " + std::to_string(result.confidence).substr(0, 4);
        label += " " + std::to_string(result.angle * 180.0f / CV_PI).substr(0, 5) + "deg";

        // 获取深度值
        int pixel_x = static_cast<int>(result.center.x);
        int pixel_y = static_cast<int>(result.center.y);
        
        if (pixel_x >= 0 && pixel_x < depth_image.cols && 
            pixel_y >= 0 && pixel_y < depth_image.rows) {
            float Z_mm = depth_image.at<ushort>(pixel_y, pixel_x);
            if (Z_mm > 0) {
                float Z = Z_mm / 1000.0f;  // 转换为米
                float X = (result.center.x - intrinsics.cx) * Z / intrinsics.fx;
                float Y = (result.center.y - intrinsics.cy) * Z / intrinsics.fy;
                label += " [" + std::to_string(X).substr(0, 5) + "," +
                         std::to_string(Y).substr(0, 5) + "," +
                         std::to_string(Z).substr(0, 5) + "m]";
            }
        }

        int baseline;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
        cv::Point text_origin(result.center.x - text_size.width / 2, 
                              result.center.y - result.height / 2 - 5);
        
        cv::rectangle(image, 
                     cv::Point(text_origin.x, text_origin.y - text_size.height - 5),
                     cv::Point(text_origin.x + text_size.width, text_origin.y),
                     color, cv::FILLED);
        cv::putText(image, label, 
                   cv::Point(text_origin.x, text_origin.y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    }
}
