#include "ppocr_det.h"
#include <fstream>
#include <numeric>
#include <algorithm>
#include <cuda_runtime_api.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include "process.h"

#define CHECK_CUDA(call) { \
    cudaError_t status = call; \
    if (status != cudaSuccess) { \
        throw std::runtime_error("CUDA error at " + std::string(__FILE__) + ":" + \
                               std::to_string(__LINE__) + ": " + \
                               cudaGetErrorString(status)); \
    } \
}

static double contour_score(const cv::Mat& binary, const std::vector<cv::Point>& contour)
{
    cv::Rect rect = cv::boundingRect(contour);
    if (rect.x < 0)
        rect.x = 0;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.x + rect.width > binary.cols)
        rect.width = binary.cols - rect.x;
    if (rect.y + rect.height > binary.rows)
        rect.height = binary.rows - rect.y;

    cv::Mat binROI = binary(rect);

    cv::Mat mask = cv::Mat::zeros(rect.height, rect.width, CV_8U);
    std::vector<cv::Point> roiContour;
    for (size_t i = 0; i < contour.size(); i++)
    {
        cv::Point pt = cv::Point(contour[i].x - rect.x, contour[i].y - rect.y);
        roiContour.push_back(pt);
    }

    std::vector<std::vector<cv::Point> > roiContours = {roiContour};
    cv::fillPoly(mask, roiContours, cv::Scalar(255));

    double score = cv::mean(binROI, mask).val[0];
    return score / 255.0;
}

DBnetDet::DBnetDet() {
    runtime_ = nullptr;
    engine_ = nullptr;
    context_ = nullptr;
    stream_ = nullptr;
    buffers_[0] = buffers_[1] = nullptr;
}

DBnetDet::~DBnetDet() {
    if (buffers_[0]) cudaFree(buffers_[0]);
    if (buffers_[1]) cudaFree(buffers_[1]);
    if (stream_) cudaStreamDestroy(stream_);
    if (context_) delete context_;
    if (engine_) delete engine_;
    if (runtime_) delete runtime_;
}

void DBnetDet::set_config(const TextDetectionConfig& config) {
    config_ = config;
}

void DBnetDet::load_engine(const std::string& engine_path) {
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

    runtime_ = nvinfer1::createInferRuntime(gLogger);
    if (!runtime_) {
        throw TensorRTException("创建TensorRT运行时失败");
    }

    engine_ = runtime_->deserializeCudaEngine(engine_data.data(), file_size);
    if (!engine_) {
        throw TensorRTException("反序列化CUDA引擎失败");
    }

    // 获取输入输出信息
    int num_io = engine_->getNbIOTensors();
    std::cout << "引擎张量总数: " << num_io << std::endl;
    
    for (int i = 0; i < num_io; ++i) {
        const char* name = engine_->getIOTensorName(i);
        nvinfer1::TensorIOMode mode = engine_->getTensorIOMode(name);
        nvinfer1::DataType dtype = engine_->getTensorDataType(name);
        auto dims = engine_->getTensorShape(name);
        
        std::cout << "张量 " << i << ": " << name << " (" 
                  << (mode == nvinfer1::TensorIOMode::kINPUT ? "输入" : "输出") << ")" << std::endl;
        std::cout << "  数据类型: " << (dtype == nvinfer1::DataType::kFLOAT ? "FLOAT" : "其他") << std::endl;
        std::cout << "  维度: [";
        for (int j = 0; j < dims.nbDims; ++j) {
            std::cout << dims.d[j];
            if (j < dims.nbDims - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            input_index_ = i;
            input_name_ = name;
            batch_size_ = dims.d[0];
            input_c_ = dims.d[1];
            input_h_ = dims.d[2] > 0 ? dims.d[2] : 640;  // 动态时给默认值
            input_w_ = dims.d[3] > 0 ? dims.d[3] : 640;
        } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
            output_index_ = i;
            output_name_ = name;
            // 计算最大输出尺寸（下采样8倍）
            // max_out_h_ = (input_h_ + 7) / 8;
            // max_out_w_ = (input_w_ + 7) / 8;
            max_out_h_ = input_h_;
            max_out_w_ = input_w_;
            std::cout << "  最大输出尺寸: " << max_out_h_ << "x" << max_out_w_ << std::endl;
        }
    }
    
    if (input_index_ == -1 || output_index_ == -1) {
        throw TensorRTException("Invalid input/output binding indices");
    }

    // 创建执行上下文
    context_ = engine_->createExecutionContext();
    if (!context_) {
        throw TensorRTException("创建执行上下文失败");
    }
    
    // 设置动态形状
    setup_dynamic_shapes();
    
    // 准备GPU缓冲区
    prepare_buffers();
    
    std::cout << "PPOCR v5 det 模型加载成功" << std::endl;
}

void DBnetDet::prepare_buffers() {
    // 分配最大输入缓冲区
    size_t max_input_size = batch_size_ * input_c_ * input_h_ * input_w_ * sizeof(float);
    CHECK_CUDA(cudaMalloc(&buffers_[input_index_], max_input_size));

    // 分配最大输出缓冲区
    size_t max_output_size = batch_size_ * 1 * max_out_h_ * max_out_w_ * sizeof(float);
    CHECK_CUDA(cudaMalloc(&buffers_[output_index_], max_output_size));
    
    // 预分配CPU缓存
    cpu_output_cache_.resize(max_out_h_ * max_out_w_);

    CHECK_CUDA(cudaStreamCreate(&stream_));
    
    std::cout << "GPU缓冲区分配成功 - 最大输出: " << max_out_h_ << "x" << max_out_w_ 
              << " (" << max_output_size / 1024 / 1024 << " MB)" << std::endl;
}

void DBnetDet::preprocess(const cv::Mat& img) {
    ratio_ = std::min(input_h_ / (float)img.rows, input_w_ / (float)img.cols);
    
    int new_w = std::round(img.cols * ratio_);
    int new_h = std::round(img.rows * ratio_);
    dw_ = (input_w_ - new_w) / 2.0f;
    dh_ = (input_h_ - new_h) / 2.0f;

    cv::Mat resized_img;
    
    if (img.cols != new_w || img.rows != new_h) {
        cv::resize(img, resized_img, cv::Size(new_w, new_h));
    } else {
        resized_img = img.clone();
    }

    int top = int(std::round(dh_ - 0.1f));
    int bottom = int(std::round(dh_ + 0.1f));
    int left = int(std::round(dw_ - 0.1f));
    int right = int(std::round(dw_ + 0.1f));
    
    cv::Mat padded_img;
    cv::copyMakeBorder(resized_img, padded_img, top, bottom, left, right,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    cv::Mat float_img;
    padded_img.convertTo(float_img, CV_32F, 1.0 / 255.0);
    
    // 归一化 - 使用BGR顺序的ImageNet预训练模型参数
    cv::Scalar mean_bgr = cv::Scalar(0.406, 0.456, 0.485); // BGR通道均值
    cv::Scalar std_bgr = cv::Scalar(0.225, 0.224, 0.229);  // BGR通道标准差
    cv::Mat normalized_img;
    cv::subtract(float_img, mean_bgr, normalized_img);
    cv::divide(normalized_img, std_bgr, normalized_img);
    
    // BGR -> RGB
    cv::cvtColor(normalized_img, float_img, cv::COLOR_BGR2RGB);
       
    // CHW 格式
    std::vector<float> chw_data(batch_size_ * input_c_ * input_h_ * input_w_);
    std::vector<cv::Mat> channels(input_c_);
    for (int i = 0; i < input_c_; ++i) {
        channels[i] = cv::Mat(input_h_, input_w_, CV_32FC1, chw_data.data() + i * input_h_ * input_w_);
    }
    cv::split(float_img, channels);
    
    CHECK_CUDA(cudaMemcpyAsync(buffers_[input_index_], chw_data.data(),
                               chw_data.size() * sizeof(float),
                               cudaMemcpyHostToDevice, stream_));
}

void DBnetDet::preprocess_gpu(const cv::Mat& img) {
    // 1. 计算缩放比例和填充（保持原逻辑）
    ratio_ = std::min(input_h_ / (float)img.rows, input_w_ / (float)img.cols);
    
    int new_w = std::round(img.cols * ratio_);
    int new_h = std::round(img.rows * ratio_);
    dw_ = (input_w_ - new_w) / 2.0f;
    dh_ = (input_h_ - new_h) / 2.0f;

    // 计算填充参数（-0.1f确保向下取整）
    int pad_top = int(std::round(dh_ - 0.1f));
    int pad_left = int(std::round(dw_ - 0.1f));

    // 2. 分配临时GPU内存
    uchar3* d_src = nullptr;
    size_t src_size = img.rows * img.cols * sizeof(uchar3);
    CHECK_CUDA(cudaMalloc(&d_src, src_size));
    
    // 3. 将原始图像数据异步拷贝到GPU
    CHECK_CUDA(cudaMemcpyAsync(d_src, img.data, src_size, 
                               cudaMemcpyHostToDevice, stream_));

    // 4. 准备归一化参数（RGB顺序，使用ImageNet标准参数）
    // 注意：CUDA核函数内部先做BGR→RGB转换，然后在RGB空间归一化
    float h_mean[3] = {0.485f, 0.456f, 0.406f};  // RGB均值
    float h_std[3]  = {0.229f, 0.224f, 0.225f};  // RGB标准差
    
    float *d_mean = nullptr, *d_std = nullptr;
    CHECK_CUDA(cudaMalloc(&d_mean, 3 * sizeof(float)));
    CHECK_CUDA(cudaMalloc(&d_std,  3 * sizeof(float)));
    CHECK_CUDA(cudaMemcpyAsync(d_mean, h_mean, 3 * sizeof(float), 
                               cudaMemcpyHostToDevice, stream_));
    CHECK_CUDA(cudaMemcpyAsync(d_std,  h_std,  3 * sizeof(float), 
                               cudaMemcpyHostToDevice, stream_));

    // 5. 调用CUDA核函数完成所有预处理操作
    //    包括：resize、pad、BGR→RGB、归一化、CHW转换
    launch_preprocess_kernel(d_src, img.rows, img.cols,
                             static_cast<float*>(buffers_[input_index_]), 
                             input_h_, input_w_,
                             ratio_, pad_top, pad_left, 1.0f/255.0f,
                             d_mean, d_std, stream_);

    // 6. 释放临时GPU内存（异步操作完成后自动释放）
    CHECK_CUDA(cudaFree(d_src));
    CHECK_CUDA(cudaFree(d_mean));
    CHECK_CUDA(cudaFree(d_std));
}

std::vector<TextDetection> DBnetDet::detect(cv::Mat& img) {
    if (img.empty()) {
        throw TensorRTException("输入图像为空");
    }
    
    preprocess_gpu(img);
    auto detections = inference_with_dynamic_shape(img);
    return detections;
}

std::vector<TextDetection> DBnetDet::detect_gpu(const cv::Mat& img) {
    if (img.empty()) {
        throw TensorRTException("输入图像为空");
    }
    
    preprocess_gpu(img);
    auto detections = inference_with_dynamic_shape(img);
    return detections;
}

std::vector<TextDetection> DBnetDet::inference_with_dynamic_shape(const cv::Mat& img) {
    const char* input_name = input_name_.c_str();
    const char* output_name = output_name_.c_str();
    
    // 设置动态输入形状（必须！）
    nvinfer1::Dims4 input_dims{1, input_c_, input_h_, input_w_};
    if (!context_->setInputShape(input_name, input_dims)) {
        throw TensorRTException("setInputShape 失败，尺寸可能超出 profile 范围");
    }
    
    // 绑定内存地址
    context_->setTensorAddress(input_name, buffers_[input_index_]);
    context_->setTensorAddress(output_name, buffers_[output_index_]);
    
    // 执行推理
    if (!context_->enqueueV3(stream_)) {
        throw TensorRTException("enqueueV3 失败");
    }
    
    // 获取实际输出形状（关键修复点）
    auto output_dims = context_->getTensorShape(output_name);
    current_output_h_ = output_dims.d[2];
    current_output_w_ = output_dims.d[3];
    
    // 计算实际输出元素数
    size_t output_elem_count = 1;
    for (int i = 0; i < output_dims.nbDims; ++i) {
        output_elem_count *= output_dims.d[i];
    }
    
    // 拷贝到 CPU（必须在同步后使用）
    CHECK_CUDA(cudaMemcpyAsync(cpu_output_cache_.data(), buffers_[output_index_],
                               output_elem_count * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaStreamSynchronize(stream_));
    
    // 后处理使用 CPU 数据
    std::vector<TextDetection> detections;
    postprocess(cpu_output_cache_, detections, img.cols, img.rows);
    return detections;
}

void DBnetDet::setup_dynamic_shapes() {
    if (!engine_) return;
    
    int nb_profiles = engine_->getNbOptimizationProfiles();
    if (nb_profiles == 0) {
        std::cout << "静态形状模型，跳过 profile 设置" << std::endl;
        return;
    }
    
    // ===== TensorRT 10 正确 API：使用 getProfileShape =====
    // 选择第一个 profile
    active_profile_index_ = 0;
    if (context_) {
        context_->setOptimizationProfileAsync(active_profile_index_, stream_);
        CHECK_CUDA(cudaStreamSynchronize(stream_));
    }
    
    // 打印 profile 范围
    for (int p = 0; p < nb_profiles; ++p) {
        auto min_shape = engine_->getProfileShape(input_name_.c_str(), p, nvinfer1::OptProfileSelector::kMIN);
        auto max_shape = engine_->getProfileShape(input_name_.c_str(), p, nvinfer1::OptProfileSelector::kMAX);
        std::cout << "Profile[" << p << "]: H " << min_shape.d[2] << "~" << max_shape.d[2]
                  << ", W " << min_shape.d[3] << "~" << max_shape.d[3] << std::endl;
    }
}

bool DBnetDet::validate_dynamic_shapes() {
    if (!engine_ || !context_) return false;
    
    std::cout << "动态形状验证:" << std::endl;
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
        const char* name = engine_->getIOTensorName(i);
        auto dims = engine_->getTensorShape(name);
        std::cout << "  " << name << ": [";
        for (int j = 0; j < dims.nbDims; ++j) {
            std::cout << dims.d[j];
            if (j < dims.nbDims - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    return true;
}

void DBnetDet::postprocess(const std::vector<float>& output_data,
                          std::vector<TextDetection>& detections,
                          int img_width, int img_height) {
    // 参考ncnn.cpp增加反归一化
    std::vector<float> denorm_output_data = output_data;
    const float denorm_vals[1] = {255.f};
    for (size_t i = 0; i < denorm_output_data.size(); i++) {
        denorm_output_data[i] *= denorm_vals[0];
    }
    
    // 将反归一化后的数据转换为 cv::Mat (DBNet 输出是 [1,1,H,W])
    cv::Mat prob_map(current_output_h_, current_output_w_, CV_32FC1,
                     const_cast<float*>(denorm_output_data.data()));
    
    // 二值化
    cv::Mat binary_map;
    cv::threshold(prob_map, binary_map, config_.text_thresh * 255, 255, cv::THRESH_BINARY);
    binary_map.convertTo(binary_map, CV_8U);
    
    // 形态学去噪
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // cv::morphologyEx(binary_map, binary_map, cv::MORPH_OPEN, kernel);
    
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_map, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    
    // 缩放比例
    float scale_x = img_width / (float)current_output_w_;
    float scale_y = img_height / (float)current_output_h_;
    
    for (const auto& cnt : contours) {
        if (cnt.size() < 3) continue;
        
        double area = cv::contourArea(cnt);
        if (area < config_.min_area) continue;
        
        {
            double score = contour_score(prob_map, cnt);
            if (score < config_.box_thresh)
                continue;
            cv::RotatedRect rrect = cv::minAreaRect(cnt);
            float rrect_maxwh = std::max(rrect.size.width, rrect.size.height);
            if (rrect_maxwh < config_.min_size)
                continue;
            int orientation = 0;
            if (rrect.angle >= -30 && rrect.angle <= 30 && rrect.size.height > rrect.size.width * 2.7)
            {
                // vertical text
                orientation = 1;
            }
            if ((rrect.angle <= -60 || rrect.angle >= 60) && rrect.size.width > rrect.size.height * 2.7)
            {
                // vertical text
                orientation = 1;
            }

            if (rrect.angle < -30)
            {
                // make orientation from -90 ~ -30 to 90 ~ 150
                rrect.angle += 180;
            }
            if (orientation == 0 && rrect.angle < 30)
            {
                // make it horizontal
                rrect.angle += 90;
                std::swap(rrect.size.width, rrect.size.height);
            }
            if (orientation == 1 && rrect.angle >= 60)
            {
                // make it vertical
                rrect.angle -= 90;
                std::swap(rrect.size.width, rrect.size.height);
            }
            float enlarge_ratio = config_.enlarge_ratio;
            rrect.size.height += rrect.size.width * (enlarge_ratio - 1);
            rrect.size.width *= enlarge_ratio;

            // adjust offset to original unpadded
            rrect.center.x = (rrect.center.x - dw_) / ratio_;
            rrect.center.y = (rrect.center.y - dh_) / ratio_;
            rrect.size.width = (rrect.size.width) / ratio_;
            rrect.size.height = (rrect.size.height) / ratio_;
            TextDetection det;
            det.rrect = rrect;
            det.confidence = score;
            det.orientation = orientation;
            detections.push_back(det);
        }
    }
}

void DBnetDet::postprocess_gpu(const std::vector<float>& output_data,
                              std::vector<TextDetection>& detections,
                              int img_width, int img_height) {
    postprocess(output_data, detections, img_width, img_height);
}

void DBnetDet::draw_results(cv::Mat& img, 
                           const std::vector<TextDetection>& detections,
                           bool draw_polygon,
                           bool draw_bbox,
                           const cv::Scalar& color) {
    for (const auto& detection : detections) {
        if (draw_bbox) {
            cv::Point2f vertices[4];
            detection.rrect.points(vertices);
            for (int i = 0; i < 4; ++i) {
                cv::line(img, vertices[i], vertices[(i + 1) % 4], color, 2);
            }
        }
        
        // 获取旋转矩形的边界
        cv::Rect bbox = detection.rrect.boundingRect();
        
        // 在检测框左上角显示置信度
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3) << detection.confidence;
        std::string score_text = oss.str();
        
        cv::Point text_pos(bbox.x, bbox.y - 5);
        if (bbox.y - 15 < 0) text_pos.y = bbox.y + 20; // 防止超出图片边界
        
        cv::putText(img, score_text, text_pos, cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, color, 1);
    }
}