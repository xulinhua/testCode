#include "ppocr_rec.h"
#include <algorithm>
#include <cstring>
#include <numeric>
#include <cuda_runtime_api.h>
#include <iostream>

// ===== 添加命名空间 =====
using namespace nvinfer1;

#define CHECK_CUDA(call) { \
    cudaError_t status = call; \
    if (status != cudaSuccess) { \
        throw std::runtime_error("CUDA error at " + std::string(__FILE__) + ":" + \
                               std::to_string(__LINE__) + ": " + \
                               cudaGetErrorString(status)); \
    } \
}

PPOCRRec::PPOCRRec() {
    cudaStreamCreate(&stream_);
}

PPOCRRec::~PPOCRRec() {
    cleanup();
}

void PPOCRRec::cleanup() {
    for (void* buffer : buffers_) {
        if (buffer) cudaFree(buffer);
    }
    buffers_.clear();
    
    if (stream_) {
        cudaStreamDestroy(stream_);
        stream_ = nullptr;
    }
    
    if (context_) {
        delete context_;
        context_ = nullptr;
    }
    if (engine_) {
        delete engine_;
        engine_ = nullptr;
    }
    if (runtime_) {
        delete runtime_;
        runtime_ = nullptr;
    }
}

void PPOCRRec::load_model(const std::string& engine_path) {
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
    
    // ===== 修复：使用 gLogger_ 和命名空间 =====
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
    
    // 获取输入输出信息
    int num_io = engine_->getNbIOTensors();
    std::cout << "引擎张量总数: " << num_io << std::endl;
    
    if (num_io == 0) {
        throw TensorRTException("模型没有输入输出张量");
    }
    
    for (int i = 0; i < num_io; ++i) {
        const char* tensor_name = engine_->getIOTensorName(i);
        nvinfer1::TensorIOMode mode = engine_->getTensorIOMode(tensor_name);
        nvinfer1::DataType dtype = engine_->getTensorDataType(tensor_name);
        auto dims = engine_->getTensorShape(tensor_name);
        
        std::cout << "张量 " << i << ": " << tensor_name << " (" 
                  << (mode == nvinfer1::TensorIOMode::kINPUT ? "输入" : "输出") << ")" << std::endl;
        std::cout << "  数据类型: " << (dtype == nvinfer1::DataType::kFLOAT ? "FLOAT" : "其他") << std::endl;
        std::cout << "  维度: [";
        for (int j = 0; j < dims.nbDims; ++j) {
            std::cout << dims.d[j];
            if (j < dims.nbDims - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            if (input_index_ != -1) {
                throw TensorRTException("模型有多个输入张量");
            }
            input_index_ = i;
            input_name_ = tensor_name;
            
            // 检测动态维度
            is_batch_dynamic_ = (dims.d[0] == -1);
            is_height_dynamic_ = (dims.d[2] == -1);
            is_width_dynamic_ = (dims.d[3] == -1);
            
            // 保存静态维度值
            if (!is_batch_dynamic_) {
                batch_size_ = dims.d[0];  // ===== 修复：正确初始化 batch_size_ =====
            }
            if (!is_height_dynamic_) input_h_ = dims.d[2];
            if (!is_width_dynamic_) input_w_ = dims.d[3];
            
        } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
            if (output_index_ != -1) {
                throw TensorRTException("模型有多个输出张量");
            }
            output_index_ = i;
            output_name_ = tensor_name;
            output_seq_len_ = dims.d[1] == -1 ? 40 : dims.d[1];
            output_dict_size_ = dims.d[2];
        }
    }
    
    if (input_index_ == -1) {
        throw TensorRTException("模型没有输入张量");
    }
    
    if (output_index_ == -1) {
        throw TensorRTException("模型没有输出张量");
    }
    
    // 设置动态形状和优化配置
    setup_dynamic_shapes();
    validate_dynamic_shapes();
    
    // 准备GPU缓冲区
    prepare_buffers();
    
    std::cout << "PPOCR v5 rec 模型加载成功" << std::endl;
    std::cout << "输入尺寸: " << input_h_ << "x" << input_w_ << std::endl;
    std::cout << "输出序列长度: " << output_seq_len_ << std::endl;
    std::cout << "字典大小: " << output_dict_size_ << std::endl;
    std::cout << "动态维度: batch=" << is_batch_dynamic_ 
              << ", height=" << is_height_dynamic_ 
              << ", width=" << is_width_dynamic_ << std::endl;
}

void PPOCRRec::load_character_dict(const std::string& dict_path) {
    character_dict_.clear();
    
    std::ifstream file(dict_path);
    if (!file.is_open()) {
        throw TensorRTException("无法打开字典文件: " + dict_path);
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            character_dict_.push_back(line);
        }
    }
    
    if (config_.use_space_char) {
        character_dict_.insert(character_dict_.begin(), " ");
    }
    
    std::cout << "加载字典完成，字符数量: " << character_dict_.size() << std::endl;
}

void PPOCRRec::set_config(const RecognitionConfig& config) {
    config_ = config;
}

void PPOCRRec::prepare_buffers() {
    buffers_.resize(2, nullptr);
    
    // 计算最大输入尺寸
    int max_batch = is_batch_dynamic_ ? config_.max_batch_size : batch_size_;
    input_size_ = max_batch * input_c_ * input_h_ * input_w_ * sizeof(float);
    CHECK_CUDA(cudaMalloc(&buffers_[input_index_], input_size_));

    // 计算最大输出尺寸
    output_size_ = max_batch * output_seq_len_ * output_dict_size_ * sizeof(float);
    CHECK_CUDA(cudaMalloc(&buffers_[output_index_], output_size_));

    std::cout << "GPU缓冲区分配成功" << std::endl;
    std::cout << "输入缓冲区大小: " << input_size_ / 1024 / 1024 << " MB" << std::endl;
    std::cout << "输出缓冲区大小: " << output_size_ / 1024 / 1024 << " MB" << std::endl;
}

cv::Mat PPOCRRec::preprocess_image(const cv::Mat& img) {
    cv::Mat processed_img;
    float ratio = std::min(static_cast<float>(input_h_) / img.rows,
                          static_cast<float>(input_w_) / img.cols);
    
    int new_w = static_cast<int>(img.cols * ratio);
    int new_h = static_cast<int>(img.rows * ratio);
    
    cv::resize(img, processed_img, cv::Size(new_w, new_h));
    
    cv::Mat canvas = cv::Mat(input_h_, input_w_, CV_8UC3, cv::Scalar(127, 127, 127));
    int top = (input_h_ - new_h) / 2;
    int left = (input_w_ - new_w) / 2;
    
    processed_img.copyTo(canvas(cv::Rect(left, top, new_w, new_h)));
    return canvas;
}

std::vector<float> PPOCRRec::normalize_image(const cv::Mat& img) {
    cv::Mat float_img;
    img.convertTo(float_img, CV_32F, 1.0 / 255.0);
    cv::cvtColor(float_img, float_img, cv::COLOR_BGR2RGB);
    
    std::vector<float> normalized(3 * input_h_ * input_w_);
    for (int c = 0; c < 3; ++c) {
        for (int h = 0; h < input_h_; ++h) {
            for (int w = 0; w < input_w_; ++w) {
                int idx = c * input_h_ * input_w_ + h * input_w_ + w;
                normalized[idx] = (float_img.at<cv::Vec3f>(h, w)[c] - config_.mean[c]) / config_.std[c];
            }
        }
    }
    return normalized;
}

void PPOCRRec::preprocess_cpu(const cv::Mat& img, float* gpu_buffer, int batch_offset) {
    cv::Mat processed_img = preprocess_image(img);
    std::vector<float> normalized = normalize_image(processed_img);
    
    size_t offset = batch_offset * 3 * input_h_ * input_w_;
    CHECK_CUDA(cudaMemcpyAsync(gpu_buffer + offset, normalized.data(),
                               normalized.size() * sizeof(float),
                               cudaMemcpyHostToDevice, stream_));
}

void PPOCRRec::batch_preprocess_cpu(const std::vector<cv::Mat>& imgs, float* gpu_buffer) {
    std::vector<float> batch_data(imgs.size() * 3 * input_h_ * input_w_);
    
    for (size_t i = 0; i < imgs.size(); ++i) {
        cv::Mat processed_img = preprocess_image(imgs[i]);
        std::vector<float> normalized = normalize_image(processed_img);
        
        size_t offset = i * 3 * input_h_ * input_w_;
        std::memcpy(batch_data.data() + offset, normalized.data(),
                   normalized.size() * sizeof(float));
    }
    
    CHECK_CUDA(cudaMemcpyAsync(gpu_buffer, batch_data.data(),
                               batch_data.size() * sizeof(float),
                               cudaMemcpyHostToDevice, stream_));
}

void PPOCRRec::decode_text(float* logits, int seq_len, RecognitionResult& result) {
    std::vector<int> indices;
    float total_score = 0.0f;
    int valid_chars = 0;
    
    for (int t = 0; t < seq_len; ++t) {
        float* timestep_logits = logits + t * output_dict_size_;
        auto max_it = std::max_element(timestep_logits, timestep_logits + output_dict_size_);
        int pred_idx = std::distance(timestep_logits, max_it);
        float max_prob = *max_it;
        
        if (pred_idx > 0 && (indices.empty() || pred_idx != indices.back())) { //
            indices.push_back(pred_idx);
            total_score += max_prob;
            valid_chars++;
        }
    }
    
    result.text = decode_sequence(indices);
    result.score = valid_chars > 0 ? total_score / valid_chars : 0.0f;
}

std::string PPOCRRec::decode_sequence(const std::vector<int>& indices) {
    std::string text;
    // std::cout<<"character_dict_[3000] = "<<character_dict_[3000]<<std::endl;
    // std::cout<<"character_dict_[16178] = "<<character_dict_[16178]<<std::endl;
    // std::cout << "indices.size() = " << indices.size() << std::endl;
    for (int idx : indices) {
        if (idx < character_dict_.size()) {
            text += character_dict_[idx];
            // std::cout<<"idx = "<<idx<<" ; "<<"character_dict_[idx] = "<<character_dict_[idx]<<std::endl;
        }
    }
    return text;
}

RecognitionResult PPOCRRec::recognize(const cv::Mat& img) {
    if (img.empty()) {
        throw TensorRTException("输入图像为空");
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    preprocess_cpu(img, static_cast<float*>(buffers_[0]), 0);
    auto preprocess_end = std::chrono::high_resolution_clock::now();
    
    auto result = inference_with_dynamic_shape(img);
    
    return result;
}

std::vector<RecognitionResult> PPOCRRec::inference_batch_with_dynamic_shape(const std::vector<cv::Mat>& imgs) {
    size_t batch_size = imgs.size();
    std::vector<RecognitionResult> results(batch_size);
    
    const char* input_name = input_name_.c_str();
    const char* output_name = output_name_.c_str();
    
    auto input_dims = engine_->getTensorShape(input_name);
    
    // 设置批量维度（无论是否动态）
    if (is_batch_dynamic_) {
        input_dims.d[0] = static_cast<int>(batch_size);
        if (!context_->setInputShape(input_name, input_dims)) {
            throw TensorRTException("设置动态输入形状失败");
        }
    } else {
        // 静态batch：确保批大小匹配模型要求
        if (static_cast<int>(batch_size) != batch_size_) {
            throw TensorRTException("静态batch模型要求批大小为: " + std::to_string(batch_size_) + 
                                "，但得到: " + std::to_string(batch_size));
        }
    }
    
    // 设置其他维度
    if (is_height_dynamic_) input_dims.d[2] = input_h_;
    if (is_width_dynamic_) input_dims.d[3] = input_w_;
    
    context_->setTensorAddress(input_name, buffers_[input_index_]);
    context_->setTensorAddress(output_name, buffers_[output_index_]);
    
    auto inference_start = std::chrono::high_resolution_clock::now();
    
    if (!context_->enqueueV3(stream_)) {
        throw TensorRTException("批量推理执行失败");
    }
    
    auto inference_end = std::chrono::high_resolution_clock::now();
    
    // 获取输出维度
    auto output_dims = context_->getTensorShape(output_name);
    int actual_batch = output_dims.d[0];
    int actual_seq_len = output_dims.d[1];
    int actual_dict_size = output_dims.d[2];
    
    size_t actual_batch_output_size = actual_batch * actual_seq_len * actual_dict_size;
    std::vector<float> batch_logits(actual_batch_output_size);
    
    CHECK_CUDA(cudaMemcpyAsync(batch_logits.data(), buffers_[output_index_],
                               actual_batch_output_size * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaStreamSynchronize(stream_));
    
    auto postprocess_start = std::chrono::high_resolution_clock::now();
    
    // 批量解码
    for (int i = 0; i < actual_batch; ++i) {
        float* logits = batch_logits.data() + i * actual_seq_len * actual_dict_size;
        decode_text(logits, actual_seq_len, results[i]);
    }
    
    auto postprocess_end = std::chrono::high_resolution_clock::now();
    
    float inference_time = std::chrono::duration<float, std::milli>(inference_end - inference_start).count();
    float postprocess_time = std::chrono::duration<float, std::milli>(postprocess_end - postprocess_start).count();
    
    std::cout << "批量推理完成 - 实际批大小: " << actual_batch 
              << ", 推理时间: " << inference_time << "ms"
              << ", 后处理时间: " << postprocess_time << "ms" << std::endl;
    
    return results;
}

std::vector<RecognitionResult> PPOCRRec::recognize_batch(const std::vector<cv::Mat>& imgs) {
    if (imgs.empty()) {
        return {};
    }
    
    size_t batch_size = imgs.size();
    
    if (!is_batch_dynamic_) {
        // 静态batch：逐张推理
        std::vector<RecognitionResult> results;
        for (const auto& img : imgs) {
            results.push_back(recognize(img));
        }
        return results;
    }
    
    // 动态batch逻辑
    if (batch_size > config_.max_batch_size) {
        throw TensorRTException("批处理大小超过最大限制: " + std::to_string(batch_size));
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    batch_preprocess_cpu(imgs, static_cast<float*>(buffers_[0]));
    auto preprocess_end = std::chrono::high_resolution_clock::now();
    
    const char* input_name = input_name_.c_str();
    const char* output_name = output_name_.c_str();
    
    context_->setTensorAddress(input_name, buffers_[0]);
    context_->setTensorAddress(output_name, buffers_[1]);
    
    auto input_dims = engine_->getTensorShape(input_name);
    input_dims.d[0] = static_cast<int>(batch_size);
    
    if (!context_->setInputShape(input_name, input_dims)) {
        throw TensorRTException("设置批量动态输入形状失败");
    }
    
    if (!context_->enqueueV3(stream_)) {
        throw TensorRTException("批量推理执行失败");
    }
    
    auto inference_end = std::chrono::high_resolution_clock::now();
    
    auto output_dims = context_->getTensorShape(output_name);
    int actual_batch = output_dims.d[0];
    int actual_seq_len = output_dims.d[1];
    int actual_dict_size = output_dims.d[2];
    
    size_t actual_batch_output_size = actual_batch * actual_seq_len * actual_dict_size;
    std::vector<float> batch_logits(actual_batch_output_size);
    
    CHECK_CUDA(cudaMemcpyAsync(batch_logits.data(), buffers_[output_index_],
                               actual_batch_output_size * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaStreamSynchronize(stream_));
    
    auto postprocess_end = std::chrono::high_resolution_clock::now();
    float preprocess_time = std::chrono::duration<float, std::milli>(preprocess_end - start_time).count();
    float inference_time = std::chrono::duration<float, std::milli>(inference_end - preprocess_end).count();
    float postprocess_time = std::chrono::duration<float, std::milli>(postprocess_end - inference_end).count();
    
    std::vector<RecognitionResult> results(batch_size);
    for (int i = 0; i < actual_batch; ++i) {
        float* logits = batch_logits.data() + i * actual_seq_len * actual_dict_size;
        decode_text(logits, actual_seq_len, results[i]);
    }
    
    return results;
}

void PPOCRRec::benchmark(const cv::Mat& img, int num_iterations) {
    std::cout << "开始性能测试，迭代次数: " << num_iterations << std::endl;
    
    float total_preprocess = 0.0f;
    float total_inference = 0.0f;
    float total_postprocess = 0.0f;
    
    for (int i = 0; i < num_iterations; ++i) {
        RecognitionResult result = recognize(img);

    }
    
    std::cout << "平均性能统计:" << std::endl;
    std::cout << "预处理时间: " << total_preprocess / num_iterations << " ms" << std::endl;
    std::cout << "推理时间: " << total_inference / num_iterations << " ms" << std::endl;
    std::cout << "后处理时间: " << total_postprocess / num_iterations << " ms" << std::endl;
    std::cout << "总时间: " << (total_preprocess + total_inference + total_postprocess) / num_iterations << " ms" << std::endl;
    std::cout << "FPS: " << 1000.0f / ((total_preprocess + total_inference + total_postprocess) / num_iterations) << std::endl;
}

void PPOCRRec::setup_dynamic_shapes() {
    if (!engine_) return;
    
    int nb_profiles = engine_->getNbOptimizationProfiles();
    if (nb_profiles == 0) {
        std::cout << "静态形状模型，跳过 profile 设置" << std::endl;
        return;
    }
    
    if (context_) {
        context_->setOptimizationProfileAsync(0, stream_);
        CHECK_CUDA(cudaStreamSynchronize(stream_));
    }
    
    std::cout << "模型支持 " << nb_profiles << " 个优化配置文件" << std::endl;
}

bool PPOCRRec::validate_dynamic_shapes() {
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

RecognitionResult PPOCRRec::inference_with_dynamic_shape(const cv::Mat& img) {
    RecognitionResult result;
    
    auto inference_start = std::chrono::high_resolution_clock::now();
    
    const char* input_name = input_name_.c_str();
    const char* output_name = output_name_.c_str();
    
    auto input_dims = engine_->getTensorShape(input_name);
    
    // ===== 修复：仅修改动态维度，不碰静态维度 =====
    if (is_batch_dynamic_) input_dims.d[0] = 1;
    if (is_height_dynamic_) input_dims.d[2] = input_h_;
    if (is_width_dynamic_) input_dims.d[3] = input_w_;
    
    if (!context_->setInputShape(input_name, input_dims)) {
        // ===== 修复：移除 getErrorDescription() =====
        throw TensorRTException("设置动态输入形状失败");
    }
    
    context_->setTensorAddress(input_name, buffers_[input_index_]);
    context_->setTensorAddress(output_name, buffers_[output_index_]);
    
    if (!context_->enqueueV3(stream_)) {
        // ===== 修复：移除 getErrorDescription() =====
        throw TensorRTException("enqueueV3 失败");
    }
    
    auto inference_end = std::chrono::high_resolution_clock::now();

    
    auto output_dims = context_->getTensorShape(output_name);
    int actual_seq_len = output_dims.d[1];
    int actual_dict_size = output_dims.d[2];
    
    size_t output_count = output_dims.d[0] * actual_seq_len * actual_dict_size;
    std::vector<float> logits(output_count);
    
    CHECK_CUDA(cudaMemcpyAsync(logits.data(), buffers_[output_index_],
                               output_count * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaStreamSynchronize(stream_));
    
    decode_text(logits.data(), actual_seq_len, result);
    
    return result;
}
