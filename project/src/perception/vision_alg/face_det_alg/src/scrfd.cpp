#include "scrfd.h"
#include <fstream>
#include <numeric>
#include <algorithm>
#include <cuda_runtime_api.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

#define CHECK_CUDA(call) { \
    cudaError_t status = call; \
    if (status != cudaSuccess) { \
        throw std::runtime_error("CUDA error at " + std::string(__FILE__) + ":" + \
                               std::to_string(__LINE__) + ": " + \
                               cudaGetErrorString(status)); \
    } \
}

SCRFD::SCRFD() {
    runtime_ = nullptr;
    engine_ = nullptr;
    context_ = nullptr;
    stream_ = nullptr;
}

SCRFD::~SCRFD() {
    // 释放 TensorRT 资源
    for (auto& buf : buffers_) {
        if (buf) cudaFree(buf);
    }
    if (stream_) cudaStreamDestroy(stream_);
    if (context_) delete context_;
    if (engine_) delete engine_;
    if (runtime_) delete runtime_;

    // 释放 GPU 缓存资源
    if (d_src_) cudaFree(d_src_);
    if (d_boxes_) cudaFree(d_boxes_);
    if (d_counts_) cudaFree(d_counts_);
    if (d_nms_mask_) cudaFree(d_nms_mask_);
}

void SCRFD::set_config(const FaceDetectionConfig& config) {
    config_ = config;
}

void SCRFD::load_engine(const std::string& engine_path) {
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

    tensor_names_.resize(num_io);
    is_input_.resize(num_io);

    // 检查是否有关键点输出
    has_kps_output_ = false;

    for (int i = 0; i < num_io; ++i) {
        const char* name = engine_->getIOTensorName(i);
        tensor_names_[i] = name;
        nvinfer1::TensorIOMode mode = engine_->getTensorIOMode(name);
        is_input_[i] = (mode == nvinfer1::TensorIOMode::kINPUT);

        auto dims = engine_->getTensorShape(name);

        std::cout << "张量 " << i << ": " << name << " ("
                  << (is_input_[i] ? "输入" : "输出") << ")" << std::endl;
        std::cout << "  维度: [";
        for (int j = 0; j < dims.nbDims; ++j) {
            std::cout << dims.d[j];
            if (j < dims.nbDims - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        if (is_input_[i]) {
            input_index_ = i;
            batch_size_ = dims.d[0];
            input_c_ = dims.d[1];
            input_h_ = dims.d[2];
            input_w_ = dims.d[3];
        } else {
            // 检查是否有关键点输出
            std::string tensor_name(name);
            if (tensor_name.find("kps") != std::string::npos) {
                has_kps_output_ = true;
            }
        }
    }

    if (input_index_ == -1) {
        throw TensorRTException("Invalid input index");
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

    std::cout << "SCRFD模型加载成功" << std::endl;
    std::cout << "模型类型: " << (has_kps_output_ ? "带关键点" : "仅边界框") << std::endl;
    std::cout << "关键点输出状态: " << (has_kps_output_ ? "启用" : "未检测到") << std::endl;

    // 打印所有输出张量名称，方便调试
    std::cout << "所有输出张量:" << std::endl;
    for (size_t i = 0; i < tensor_names_.size(); ++i) {
        if (!is_input_[i]) {
            std::cout << "  " << tensor_names_[i] << std::endl;
        }
    }
}

void SCRFD::prepare_buffers() {
    buffers_.resize(tensor_names_.size());

    // 分配输入缓冲区
    size_t input_size = batch_size_ * input_c_ * input_h_ * input_w_ * sizeof(float);
    CHECK_CUDA(cudaMalloc(&buffers_[input_index_], input_size));

    // 分配输出缓冲区 - 为每个输出张量分配内存
    // score_8: 1 x 12800 x 1
    // score_16: 1 x 3200 x 1
    // score_32: 1 x 800 x 1
    // bbox_8: 1 x 12800 x 4
    // bbox_16: 1 x 3200 x 4
    // bbox_32: 1 x 800 x 4

    for (int i = 0; i < tensor_names_.size(); ++i) {
        if (!is_input_[i]) {
            auto dims = engine_->getTensorShape(tensor_names_[i].c_str());
            size_t size = 1;
            for (int j = 0; j < dims.nbDims; ++j) {
                size *= dims.d[j];
            }
            size *= sizeof(float);
            CHECK_CUDA(cudaMalloc(&buffers_[i], size));
            std::cout << "  分配 " << tensor_names_[i] << ": " << size << " bytes" << std::endl;
        }
    }

    // 预分配CPU缓存
    cpu_output_score8_.resize(12800);
    cpu_output_score16_.resize(3200);
    cpu_output_score32_.resize(800);
    cpu_output_bbox8_.resize(12800 * 4);
    cpu_output_bbox16_.resize(3200 * 4);
    cpu_output_bbox32_.resize(800 * 4);
    cpu_output_kps8_.resize(12800 * 10);  // 5个关键点 * 2个坐标
    cpu_output_kps16_.resize(3200 * 10);
    cpu_output_kps32_.resize(800 * 10);

    CHECK_CUDA(cudaStreamCreate(&stream_));

    // 预分配GPU缓存用于后处理
    // 最大框数估计: 每个尺度最多产生 12800/3200/800 个框
    max_boxes_per_stride_ = 12800 * 2;  // *2 是 num_anchors
    CHECK_CUDA(cudaMalloc(&d_boxes_, max_boxes_per_stride_ * sizeof(Bbox32)));
    CHECK_CUDA(cudaMalloc(&d_counts_, 3 * sizeof(int)));  // 3个stride

    std::cout << "GPU后处理缓存分配完成" << std::endl;

    size_t total_size = input_size;
    for (int i = 0; i < tensor_names_.size(); ++i) {
        if (!is_input_[i]) {
            auto dims = engine_->getTensorShape(tensor_names_[i].c_str());
            size_t size = 1;
            for (int j = 0; j < dims.nbDims; ++j) {
                size *= dims.d[j];
            }
            total_size += size * sizeof(float);
        }
    }

    std::cout << "GPU缓冲区分配成功 - 输入: " << input_h_ << "x" << input_w_
              << " (" << total_size / 1024 / 1024 << " MB)" << std::endl;
}

void SCRFD::preprocess(const cv::Mat& img) {
    // 计算缩放比例
    ratio_ = std::min(input_w_ / (float)img.cols, input_h_ / (float)img.rows);

    int new_w = std::round(img.cols * ratio_);
    int new_h = std::round(img.rows * ratio_);
    pad_w_ = (input_w_ - new_w) / 2.0f;
    pad_h_ = (input_h_ - new_h) / 2.0f;

    cv::Mat resized_img;
    if (img.cols != new_w || img.rows != new_h) {
        cv::resize(img, resized_img, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    } else {
        resized_img = img.clone();
    }

    // 填充
    int top = int(std::round(pad_h_ - 0.1f));
    int bottom = int(std::round(pad_h_ + 0.1f));
    int left = int(std::round(pad_w_ - 0.1f));
    int right = int(std::round(pad_w_ + 0.1f));

    cv::Mat padded_img;
    cv::copyMakeBorder(resized_img, padded_img, top, bottom, left, right,
                       cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // 归一化: (pixel - 127.5) / 128 = pixel * (1/128) - 127.5/128 = pixel * 0.0078125 - 0.99609
    // 对应 ncnn: mean_vals[3] = {127.5f, 127.5f, 127.5f}, norm_vals[3] = {1/128.f, 1/128.f, 1/128.f}
    cv::Mat rgb_img;
    cv::cvtColor(padded_img, rgb_img, cv::COLOR_BGR2RGB);

    cv::Mat float_img;
    rgb_img.convertTo(float_img, CV_32F);
    float_img = (float_img - 127.5f) * (1.0f / 128.0f);

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

void SCRFD::preprocess_gpu(const cv::Mat& img) {
    int src_h = img.rows;
    int src_w = img.cols;

    // 计算缩放 / 填充
    ratio_ = std::min(input_w_ / (float)src_w, input_h_ / (float)src_h);
    int padw = std::round(src_w * ratio_);
    int padh = std::round(src_h * ratio_);
    pad_w_ = (input_w_ - padw) / 2.0f;
    pad_h_ = (input_h_ - padh) / 2.0f;
    int pad_top = int(std::round(pad_h_ - 0.1f));
    int pad_left = int(std::round(pad_w_ - 0.1f));

    // 一次性申请 GPU buffer
    size_t img_byte = src_h * src_w * sizeof(uchar3);
    if (d_src_ == nullptr || img_byte > d_src_size_) {
        if (d_src_) cudaFree(d_src_);
        CHECK_CUDA(cudaMalloc(&d_src_, img_byte));
        d_src_size_ = img_byte;
    }

    // 把原图拷到 GPU（BGR连续）
    CHECK_CUDA(cudaMemcpyAsync(d_src_, img.data, img_byte,
                               cudaMemcpyHostToDevice, stream_));

    // 启动CUDA预处理kernel
    // 归一化: (pixel - 127.5) / 128
    launch_scrfd_preprocess_kernel(
        d_src_, src_h, src_w,
        (float*)buffers_[input_index_], input_h_, input_w_,
        ratio_, pad_top, pad_left, 127.5f, 1.0f / 128.0f, stream_);
}

bool SCRFD::cmp_score(const SCRFD::BoxInfo& a, const SCRFD::BoxInfo& b) {
    return a.score > b.score;
}

std::vector<SCRFD::BoxInfo> SCRFD::nms_sorted_bboxes(const std::vector<BoxInfo>& bboxes, float nms_thresh) {
    std::vector<BoxInfo> result;
    if (bboxes.empty()) {
        return result;
    }

    std::vector<int> picked;
    std::vector<float> areas(bboxes.size());
    for (size_t i = 0; i < bboxes.size(); ++i) {
        areas[i] = (bboxes[i].x2 - bboxes[i].x1 + 1) * (bboxes[i].y2 - bboxes[i].y1 + 1);
    }

    for (size_t i = 0; i < bboxes.size(); ++i) {
        bool keep = true;
        for (size_t j = 0; j < picked.size(); ++j) {
            int idx1 = i;
            int idx2 = picked[j];

            float x1 = std::max(bboxes[idx1].x1, bboxes[idx2].x1);
            float y1 = std::max(bboxes[idx1].y1, bboxes[idx2].y1);
            float x2 = std::min(bboxes[idx1].x2, bboxes[idx2].x2);
            float y2 = std::min(bboxes[idx1].y2, bboxes[idx2].y2);

            float inter_area = std::max(0.0f, x2 - x1 + 1) * std::max(0.0f, y2 - y1 + 1);
            float union_area = areas[idx1] + areas[idx2] - inter_area;
            float iou = inter_area / union_area;

            if (iou > nms_thresh) {
                keep = false;
                break;
            }
        }

        if (keep) {
            picked.push_back(i);
            result.push_back(bboxes[i]);
        }
    }

    return result;
}

void SCRFD::postprocess(const std::vector<float>& score_8,
                       const std::vector<float>& score_16,
                       const std::vector<float>& score_32,
                       const std::vector<float>& bbox_8,
                       const std::vector<float>& bbox_16,
                       const std::vector<float>& bbox_32,
                       std::vector<FaceDetection>& detections,
                       int img_width, int img_height) {
    std::vector<BoxInfo> boxes;
    boxes.reserve(1000);

    // 数据布局参考 ncnn.cpp:
    // score_blob.channel(q) 表示第 q 个 anchor 的所有位置的分数
    // bbox_blob.channel_range(q*4, 4) 表示第 q 个 anchor 的 4 个偏移 channel
    //
    // TensorRT 输出是展平的: [1, N, 1] 和 [1, N*4, 1]
    // N = feat_h * feat_w * num_anchors
    //
    // 数据布局: 先位置 (i, j)，后 anchor (q)
    // score_idx = (i * feat_w + j) * num_anchors + q
    // bbox_idx = ((i * feat_w + j) * num_anchors + q) * 4 + channel

    // 处理每个尺度的函数
    auto process_stride = [&](const std::vector<float>& scores,
                              const std::vector<float>& bboxes,
                              int feat_w, int feat_h,
                              int num_anchors,
                              int feat_stride,
                              float base_size) {
        const std::vector<float> scales = {1.0f, 2.0f};  // 2 种 scale

        for (int q = 0; q < num_anchors; ++q) {
            float scale = scales[q];
            float rs_w = base_size * scale;
            float rs_h = base_size * scale;  // ratio = 1.0

            // 生成锚点模板 [x1, y1, x2, y2]，中心在 (0,0)
            float anchor_x1 = -rs_w * 0.5f;
            float anchor_y1 = -rs_h * 0.5f;
            float anchor_x2 = rs_w * 0.5f;
            float anchor_y2 = rs_h * 0.5f;

            // 遍历特征图的每个位置
            for (int i = 0; i < feat_h; ++i) {
                for (int j = 0; j < feat_w; ++j) {
                    int pos_idx = i * feat_w + j;
                    int score_idx = pos_idx * num_anchors + q;

                    float score = scores[score_idx];
                    if (score < config_.conf_thresh) continue;

                    // bbox 偏移: [dx, dy, dw, dh]
                    int bbox_base_idx = pos_idx * num_anchors * 4 + q * 4;
                    float dx = bboxes[bbox_base_idx + 0] * feat_stride;
                    float dy = bboxes[bbox_base_idx + 1] * feat_stride;
                    float dw = bboxes[bbox_base_idx + 2] * feat_stride;
                    float dh = bboxes[bbox_base_idx + 3] * feat_stride;

                    // shifted anchor 的位置 (ncnn.cpp line 160-168)
                    float anchor_x = anchor_x1 + j * feat_stride;
                    float anchor_y = anchor_y1 + i * feat_stride;

                    // anchor_w, anchor_h (ncnn.cpp line 163-164)
                    float anchor_w = anchor_x2 - anchor_x1;
                    float anchor_h = anchor_y2 - anchor_y1;

                    // distance2bbox (ncnn.cpp line 184-191)
                    float cx = anchor_x + anchor_w * 0.5f;
                    float cy = anchor_y + anchor_h * 0.5f;

                    float x0 = cx - dx;
                    float y0 = cy - dy;
                    float x1 = cx + dw;
                    float y1 = cy + dh;

                    BoxInfo box;
                    box.x1 = x0;
                    box.y1 = y0;
                    box.x2 = x1;
                    box.y2 = y1;
                    box.score = score;

                    boxes.push_back(box);
                }
            }
        }
    };

    // 根据 ncnn.cpp 实际代码:
    // line 274-286: base_size=16, feat_stride=8 (注释"stride 32")
    // line 289-307: base_size=64, feat_stride=16 (注释"stride 16")
    // line 310-328: base_size=256, feat_stride=32 (注释"stride 8")
    process_stride(score_8, bbox_8, 80, 80, 2, 8, 16.0f);    // feat_stride=8, base_size=16
    process_stride(score_16, bbox_16, 40, 40, 2, 16, 64.0f);  // feat_stride=16, base_size=64
    process_stride(score_32, bbox_32, 20, 20, 2, 32, 256.0f); // feat_stride=32, base_size=256

    // 按分数排序 (参考 ncnn.cpp line 332)
    std::sort(boxes.begin(), boxes.end(), cmp_score);

    // NMS (参考 ncnn.cpp line 334-336)
    std::vector<BoxInfo> nms_boxes = nms_sorted_bboxes(boxes, config_.nms_thresh);

    // 转换为最终检测结果并映射回原图尺寸 (参考 ncnn.cpp line 342-360)
    for (const auto& box : nms_boxes) {
        FaceDetection det;
        det.score = box.score;

        // 去除padding并缩放回原图
        float x0 = (box.x1 - pad_w_) / ratio_;
        float y0 = (box.y1 - pad_h_) / ratio_;
        float x1 = (box.x2 - pad_w_) / ratio_;
        float y1 = (box.y2 - pad_h_) / ratio_;

        // 裁剪到原图范围
        x0 = std::max(0.0f, std::min(x0, (float)img_width - 1));
        y0 = std::max(0.0f, std::min(y0, (float)img_height - 1));
        x1 = std::max(0.0f, std::min(x1, (float)img_width - 1));
        y1 = std::max(0.0f, std::min(y1, (float)img_height - 1));

        det.bbox.x = static_cast<int>(std::round(x0));
        det.bbox.y = static_cast<int>(std::round(y0));
        det.bbox.width = static_cast<int>(std::round(x1 - x0));
        det.bbox.height = static_cast<int>(std::round(y1 - y0));

        detections.push_back(det);
    }
}

// 带关键点的后处理函数
void SCRFD::postprocess(const std::vector<float>& score_8,
                       const std::vector<float>& score_16,
                       const std::vector<float>& score_32,
                       const std::vector<float>& bbox_8,
                       const std::vector<float>& bbox_16,
                       const std::vector<float>& bbox_32,
                       const std::vector<float>& kps_8,
                       const std::vector<float>& kps_16,
                       const std::vector<float>& kps_32,
                       std::vector<FaceDetection>& detections,
                       int img_width, int img_height) {
    std::vector<BoxInfo> boxes;
    boxes.reserve(1000);

    // 处理每个尺度的函数
    auto process_stride = [&](const std::vector<float>& scores,
                              const std::vector<float>& bboxes,
                              const std::vector<float>& kps,
                              int feat_w, int feat_h,
                              int num_anchors,
                              int feat_stride,
                              float base_size) {
        const std::vector<float> scales = {1.0f, 2.0f};  // 2 种 scale

        for (int q = 0; q < num_anchors; ++q) {
            float scale = scales[q];
            float rs_w = base_size * scale;
            float rs_h = base_size * scale;  // ratio = 1.0

            // 生成锚点模板 [x1, y1, x2, y2]，中心在 (0,0)
            float anchor_x1 = -rs_w * 0.5f;
            float anchor_y1 = -rs_h * 0.5f;
            float anchor_x2 = rs_w * 0.5f;
            float anchor_y2 = rs_h * 0.5f;

            // 遍历特征图的每个位置
            for (int i = 0; i < feat_h; ++i) {
                for (int j = 0; j < feat_w; ++j) {
                    int pos_idx = i * feat_w + j;
                    int score_idx = pos_idx * num_anchors + q;

                    float score = scores[score_idx];
                    if (score < config_.conf_thresh) continue;

                    // bbox 偏移: [dx, dy, dw, dh]
                    int bbox_base_idx = pos_idx * num_anchors * 4 + q * 4;
                    float dx = bboxes[bbox_base_idx + 0] * feat_stride;
                    float dy = bboxes[bbox_base_idx + 1] * feat_stride;
                    float dw = bboxes[bbox_base_idx + 2] * feat_stride;
                    float dh = bboxes[bbox_base_idx + 3] * feat_stride;

                    // shifted anchor 的位置
                    float anchor_x = anchor_x1 + j * feat_stride;
                    float anchor_y = anchor_y1 + i * feat_stride;

                    // anchor_w, anchor_h
                    float anchor_w = anchor_x2 - anchor_x1;
                    float anchor_h = anchor_y2 - anchor_y1;

                    // distance2bbox
                    float cx = anchor_x + anchor_w * 0.5f;
                    float cy = anchor_y + anchor_h * 0.5f;

                    float x0 = cx - dx;
                    float y0 = cy - dy;
                    float x1 = cx + dw;
                    float y1 = cy + dh;

                    BoxInfo box;
                    box.x1 = x0;
                    box.y1 = y0;
                    box.x2 = x1;
                    box.y2 = y1;
                    box.score = score;

                    // 提取关键点 (5个关键点，每个2个坐标 = 10个值)
                    // 关键点输出是相对于anchor中心的偏移量，乘以stride后加上anchor中心
                    // 然后去除padding并缩放，直接得到最终坐标
                    int kps_base_idx = pos_idx * num_anchors * 10 + q * 10;
                    box.landmark.resize(10);
                    for (int k = 0; k < 5; ++k) {
                        float kps_x = (cx + kps[kps_base_idx + 2*k] * feat_stride - pad_w_) / ratio_;
                        float kps_y = (cy + kps[kps_base_idx + 2*k + 1] * feat_stride - pad_h_) / ratio_;
                        box.landmark[2*k] = kps_x;
                        box.landmark[2*k + 1] = kps_y;
                    }

                    boxes.push_back(box);
                }
            }
        }
    };

    // 处理三个stride
    process_stride(score_8, bbox_8, kps_8, 80, 80, 2, 8, 16.0f);
    process_stride(score_16, bbox_16, kps_16, 40, 40, 2, 16, 64.0f);
    process_stride(score_32, bbox_32, kps_32, 20, 20, 2, 32, 256.0f);

    // 按分数排序
    std::sort(boxes.begin(), boxes.end(), cmp_score);

    // NMS
    std::vector<BoxInfo> nms_boxes = nms_sorted_bboxes(boxes, config_.nms_thresh);

    // 转换为最终检测结果并映射回原图尺寸
    for (const auto& box : nms_boxes) {
        FaceDetection det;
        det.score = box.score;

        // 去除padding并缩放回原图
        float x0 = (box.x1 - pad_w_) / ratio_;
        float y0 = (box.y1 - pad_h_) / ratio_;
        float x1 = (box.x2 - pad_w_) / ratio_;
        float y1 = (box.y2 - pad_h_) / ratio_;

        // 裁剪到原图范围
        x0 = std::max(0.0f, std::min(x0, (float)img_width - 1));
        y0 = std::max(0.0f, std::min(y0, (float)img_height - 1));
        x1 = std::max(0.0f, std::min(x1, (float)img_width - 1));
        y1 = std::max(0.0f, std::min(y1, (float)img_height - 1));

        det.bbox.x = static_cast<int>(std::round(x0));
        det.bbox.y = static_cast<int>(std::round(y0));
        det.bbox.width = static_cast<int>(std::round(x1 - x0));
        det.bbox.height = static_cast<int>(std::round(y1 - y0));

        det.landmarks.resize(5);
        for (int k = 0; k < 5; ++k) {
            float kps_x = std::max(0.0f, std::min(box.landmark[2*k], (float)img_width - 1));
            float kps_y = std::max(0.0f, std::min(box.landmark[2*k + 1], (float)img_height - 1));
            det.landmarks[k] = cv::Point2f(kps_x, kps_y);
        }

        detections.push_back(det);
    }
}

void SCRFD::postprocess_gpu(std::vector<FaceDetection>& detections,
                             int img_width, int img_height) {
    // 根据张量名称找到对应的输出缓冲区索引
    int idx_score8 = -1, idx_score16 = -1, idx_score32 = -1;
    int idx_bbox8 = -1, idx_bbox16 = -1, idx_bbox32 = -1;

    for (size_t i = 0; i < tensor_names_.size(); ++i) {
        if (!is_input_[i]) {
            if (tensor_names_[i] == "score_8") idx_score8 = i;
            else if (tensor_names_[i] == "score_16") idx_score16 = i;
            else if (tensor_names_[i] == "score_32") idx_score32 = i;
            else if (tensor_names_[i] == "bbox_8") idx_bbox8 = i;
            else if (tensor_names_[i] == "bbox_16") idx_bbox16 = i;
            else if (tensor_names_[i] == "bbox_32") idx_bbox32 = i;
        }
    }

    // 重置计数器
    CHECK_CUDA(cudaMemsetAsync(d_counts_, 0, 3 * sizeof(int), stream_));

    // 处理三个stride: stride=8,16,32
    // stride=8, base_size=16, feat=80x80, anchors=2
    if (idx_score8 >= 0 && idx_bbox8 >= 0) {
        launch_scrfd_postprocess_kernel(
            (float*)buffers_[idx_score8], (float*)buffers_[idx_bbox8],
            80, 80, 2, 8, 16.0f, config_.conf_thresh,
            ratio_, pad_w_, pad_h_,
            d_boxes_, &d_counts_[0], stream_);
    }

    // 同步获取第一个stride的计数
    int h_count0;
    CHECK_CUDA(cudaMemcpyAsync(&h_count0, &d_counts_[0], sizeof(int),
                               cudaMemcpyDeviceToHost, stream_));

    // stride=16, base_size=64, feat=40x40, anchors=2
    if (idx_score16 >= 0 && idx_bbox16 >= 0) {
        launch_scrfd_postprocess_kernel(
            (float*)buffers_[idx_score16], (float*)buffers_[idx_bbox16],
            40, 40, 2, 16, 64.0f, config_.conf_thresh,
            ratio_, pad_w_, pad_h_,
            d_boxes_ + h_count0, &d_counts_[1], stream_);
    }

    // 同步获取第二个stride的计数
    int h_count1;
    CHECK_CUDA(cudaMemcpyAsync(&h_count1, &d_counts_[1], sizeof(int),
                               cudaMemcpyDeviceToHost, stream_));

    // stride=32, base_size=256, feat=20x20, anchors=2
    if (idx_score32 >= 0 && idx_bbox32 >= 0) {
        launch_scrfd_postprocess_kernel(
            (float*)buffers_[idx_score32], (float*)buffers_[idx_bbox32],
            20, 20, 2, 32, 256.0f, config_.conf_thresh,
            ratio_, pad_w_, pad_h_,
            d_boxes_ + h_count0 + h_count1, &d_counts_[2], stream_);
    }

    // 同步并获取总数
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    int h_count2;
    CHECK_CUDA(cudaMemcpyAsync(&h_count2, &d_counts_[2], sizeof(int),
                               cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    int total_boxes = h_count0 + h_count1 + h_count2;
    if (total_boxes == 0) return;

    // 拷贝所有框到CPU
    std::vector<Bbox32> host_boxes(total_boxes);
    CHECK_CUDA(cudaMemcpyAsync(host_boxes.data(), d_boxes_,
                               total_boxes * sizeof(Bbox32),
                               cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    // 按分数排序 (CPU端排序)
    std::sort(host_boxes.begin(), host_boxes.end(),
              [](const Bbox32& a, const Bbox32& b) {
                  return a.score > b.score;
              });

    // GPU NMS
    // 先把排序后的框拷回GPU
    CHECK_CUDA(cudaMemcpyAsync(d_boxes_, host_boxes.data(),
                               total_boxes * sizeof(Bbox32),
                               cudaMemcpyHostToDevice, stream_));

    // 分配掩码矩阵
    if (d_nms_mask_) cudaFree(d_nms_mask_);
    size_t mask_bytes = total_boxes * total_boxes * sizeof(uint8_t);
    CHECK_CUDA(cudaMalloc(&d_nms_mask_, mask_bytes));

    // 计算IoU矩阵 (使用GPU指针)
    launch_nms_matrix_kernel(d_boxes_, total_boxes,
                              config_.nms_thresh,
                              d_nms_mask_, stream_);

    // 拷贝掩码回CPU
    std::vector<uint8_t> h_mask(total_boxes * total_boxes);
    CHECK_CUDA(cudaMemcpyAsync(h_mask.data(), d_nms_mask_, mask_bytes,
                               cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    // CPU端处理NMS结果
    std::vector<int> keep_indices;
    keep_indices.push_back(0);  // 第一个框保留
    for (int i = 1; i < total_boxes; ++i) {
        bool suppressed = false;
        for (int j = 0; j < i; ++j) {
            if (h_mask[j * total_boxes + i] == 1) {
                suppressed = true;
                break;
            }
        }
        if (!suppressed) {
            keep_indices.push_back(i);
        }
    }

    // 转换为FaceDetection
    for (int idx : keep_indices) {
        const Bbox32& box = host_boxes[idx];
        FaceDetection det;
        det.score = box.score;

        // 裁剪到原图范围
        float x0 = std::max(0.0f, std::min(box.x1, (float)img_width - 1));
        float y0 = std::max(0.0f, std::min(box.y1, (float)img_height - 1));
        float x1 = std::max(0.0f, std::min(box.x2, (float)img_width - 1));
        float y1 = std::max(0.0f, std::min(box.y2, (float)img_height - 1));

        det.bbox.x = static_cast<int>(std::round(x0));
        det.bbox.y = static_cast<int>(std::round(y0));
        det.bbox.width = static_cast<int>(std::round(x1 - x0));
        det.bbox.height = static_cast<int>(std::round(y1 - y0));

        detections.push_back(det);
    }

    // 清理NMS掩码
    cudaFree(d_nms_mask_);
    d_nms_mask_ = nullptr;
}

std::vector<FaceDetection> SCRFD::inference(const cv::Mat& img) {
    // 设置输入形状
    const char* input_name = tensor_names_[input_index_].c_str();
    nvinfer1::Dims4 input_dims{1, input_c_, input_h_, input_w_};
    if (!context_->setInputShape(input_name, input_dims)) {
        throw TensorRTException("setInputShape 失败");
    }

    // 绑定所有张量的内存地址
    for (int i = 0; i < tensor_names_.size(); ++i) {
        context_->setTensorAddress(tensor_names_[i].c_str(), buffers_[i]);
    }

    // 执行推理
    if (!context_->enqueueV3(stream_)) {
        throw TensorRTException("enqueueV3 失败");
    }

    // 拷贝输出到CPU
    for (int i = 0; i < tensor_names_.size(); ++i) {
        if (!is_input_[i]) {
            auto dims = engine_->getTensorShape(tensor_names_[i].c_str());
            size_t size = 1;
            for (int j = 0; j < dims.nbDims; ++j) {
                size *= dims.d[j];
            }

            // 根据张量名称拷贝到对应的CPU缓存
            if (tensor_names_[i] == "score_8") {
                CHECK_CUDA(cudaMemcpyAsync(cpu_output_score8_.data(), buffers_[i],
                                           size * sizeof(float), cudaMemcpyDeviceToHost, stream_));
            } else if (tensor_names_[i] == "score_16") {
                CHECK_CUDA(cudaMemcpyAsync(cpu_output_score16_.data(), buffers_[i],
                                           size * sizeof(float), cudaMemcpyDeviceToHost, stream_));
            } else if (tensor_names_[i] == "score_32") {
                CHECK_CUDA(cudaMemcpyAsync(cpu_output_score32_.data(), buffers_[i],
                                           size * sizeof(float), cudaMemcpyDeviceToHost, stream_));
            } else if (tensor_names_[i] == "bbox_8") {
                CHECK_CUDA(cudaMemcpyAsync(cpu_output_bbox8_.data(), buffers_[i],
                                           size * sizeof(float), cudaMemcpyDeviceToHost, stream_));
            } else if (tensor_names_[i] == "bbox_16") {
                CHECK_CUDA(cudaMemcpyAsync(cpu_output_bbox16_.data(), buffers_[i],
                                           size * sizeof(float), cudaMemcpyDeviceToHost, stream_));
            } else if (tensor_names_[i] == "bbox_32") {
                CHECK_CUDA(cudaMemcpyAsync(cpu_output_bbox32_.data(), buffers_[i],
                                           size * sizeof(float), cudaMemcpyDeviceToHost, stream_));
            } else if (tensor_names_[i] == "kps_8") {
                CHECK_CUDA(cudaMemcpyAsync(cpu_output_kps8_.data(), buffers_[i],
                                           size * sizeof(float), cudaMemcpyDeviceToHost, stream_));
            } else if (tensor_names_[i] == "kps_16") {
                CHECK_CUDA(cudaMemcpyAsync(cpu_output_kps16_.data(), buffers_[i],
                                           size * sizeof(float), cudaMemcpyDeviceToHost, stream_));
            } else if (tensor_names_[i] == "kps_32") {
                CHECK_CUDA(cudaMemcpyAsync(cpu_output_kps32_.data(), buffers_[i],
                                           size * sizeof(float), cudaMemcpyDeviceToHost, stream_));
            }
        }
    }
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    // 后处理 - 根据模型类型选择调用哪个版本
    std::vector<FaceDetection> detections;

    // std::cout << "选择后处理方式: " << (config_.use_gpu_postprocess ? "GPU" : "CPU") << std::endl;
    // std::cout << "has_kps_output_: " << (has_kps_output_ ? "true" : "false") << std::endl;

    if (config_.use_gpu_postprocess) {
        // std::cout << "使用GPU后处理（不支持关键点）" << std::endl;
        postprocess_gpu(detections, img.cols, img.rows);
    } else {
        // CPU后处理：根据是否有kps输出选择函数
        if (has_kps_output_) {
            // std::cout << "使用CPU后处理 - 带关键点版本" << std::endl;
            postprocess(cpu_output_score8_, cpu_output_score16_, cpu_output_score32_,
                       cpu_output_bbox8_, cpu_output_bbox16_, cpu_output_bbox32_,
                       cpu_output_kps8_, cpu_output_kps16_, cpu_output_kps32_,
                       detections, img.cols, img.rows);
        } else {
            // std::cout << "使用CPU后处理 - 无关键点版本" << std::endl;
            postprocess(cpu_output_score8_, cpu_output_score16_, cpu_output_score32_,
                       cpu_output_bbox8_, cpu_output_bbox16_, cpu_output_bbox32_,
                       detections, img.cols, img.rows);
        }
    }

    return detections;
}

std::vector<FaceDetection> SCRFD::detect(const cv::Mat& img) {
    if (img.empty()) {
        throw TensorRTException("输入图像为空");
    }

    // 根据配置选择预处理方式
    if (config_.use_gpu_preprocess) {
        preprocess_gpu(img);
    } else {
        preprocess(img);
    }

    return inference(img);
}

void SCRFD::setup_dynamic_shapes() {
    if (!engine_) return;

    int nb_profiles = engine_->getNbOptimizationProfiles();
    if (nb_profiles == 0) {
        std::cout << "静态形状模型，跳过 profile 设置" << std::endl;
        return;
    }

    // 选择第一个 profile
    active_profile_index_ = 0;
    if (context_) {
        context_->setOptimizationProfileAsync(active_profile_index_, stream_);
        CHECK_CUDA(cudaStreamSynchronize(stream_));
    }

    // 打印 profile 范围
    for (int p = 0; p < nb_profiles; ++p) {
        auto min_shape = engine_->getProfileShape(tensor_names_[input_index_].c_str(), p, nvinfer1::OptProfileSelector::kMIN);
        auto max_shape = engine_->getProfileShape(tensor_names_[input_index_].c_str(), p, nvinfer1::OptProfileSelector::kMAX);
        std::cout << "Profile[" << p << "]: H " << min_shape.d[2] << "~" << max_shape.d[2]
                  << ", W " << min_shape.d[3] << "~" << max_shape.d[3] << std::endl;
    }
}

void SCRFD::draw_results(cv::Mat& img,
                        const std::vector<FaceDetection>& detections,
                        bool draw_bbox,
                        bool draw_landmarks,
                        const cv::Scalar& color) {
    for (const auto& detection : detections) {
        if (draw_bbox) {
            cv::rectangle(img, detection.bbox, color, 2);

            // 显示置信度
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << detection.score;
            std::string score_text = oss.str();
            cv::putText(img, score_text,
                       cv::Point(detection.bbox.x, detection.bbox.y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }

        if (draw_landmarks && !detection.landmarks.empty()) {
            // std::cout << "绘制关键点，数量: " << detection.landmarks.size() << std::endl;
            for (size_t i = 0; i < detection.landmarks.size(); ++i) {
                cv::circle(img, detection.landmarks[i], 3, color, -1);
            }
        } else if (draw_landmarks) {
            std::cout << "需要绘制关键点，但 landmarks 为空！" << std::endl;
        }
    }
}

void SCRFD::draw_results(cv::Mat& img,
                        const std::vector<FaceDetection>& detections,
                        cv::Mat &depth_img,
                        const Intrinsics &intrinsics,
                        bool draw_bbox,
                        bool draw_landmarks,
                        const cv::Scalar& color) {
    if(!detections.empty())
        std::cout << "\n检测到人脸: " << detections.size() << " 张\n";
    for (const auto& detection : detections) {
        // 调试输出：检查关键点数据
        std::cout << "关键点数量: " << detection.landmarks.size() << std::endl;
        if (!detection.landmarks.empty()) {
            for (size_t i = 0; i < detection.landmarks.size() && i < 5; ++i) {
                std::cout << "  关键点 " << i << ": (" << detection.landmarks[i].x
                          << ", " << detection.landmarks[i].y << ")" << std::endl;
            }

            // 显示朝向程度
            float pose_degree = get_face_pose_degree(detection);
            bool is_frontal = is_frontal_face(detection, 0.3f);
            std::cout << "  朝向程度: " << std::fixed << std::setprecision(3) << pose_degree
                      << " (" << (is_frontal ? "正脸" : "侧脸") << ")" << std::endl;
        }
        if (draw_bbox) {
            cv::rectangle(img, detection.bbox, color, 2);

            // 显示置信度
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << detection.score;
            std::string score_text = oss.str();
            cv::putText(img, score_text,
                       cv::Point(detection.bbox.x, detection.bbox.y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }

        if (draw_landmarks && !detection.landmarks.empty()) {
            // std::cout << "绘制关键点，数量: " << detection.landmarks.size() << std::endl;
            for (size_t i = 0; i < detection.landmarks.size(); ++i) {
                cv::circle(img, detection.landmarks[i], 3, color, -1);
            }
        } else if (draw_landmarks) {
            std::cout << "需要绘制关键点，但 landmarks 为空！" << std::endl;
        }

        // 获取边界框中心点坐标
        int center_x = static_cast<int>(detection.bbox.x + detection.bbox.width / 2.0f);
        int center_y = static_cast<int>(detection.bbox.y + detection.bbox.height / 2.0f);

        std::cout << "Face:score:" << detection.score << ":center_x,center_y :" << center_x << "," << center_y << std::endl;

        // 检查像素坐标是否在深度图像范围内
        if (center_x < 0 || center_x >= depth_img.cols ||
            center_y < 0 || center_y >= depth_img.rows)
        {
            std::cout << "像素坐标超出图像范围" << std::endl;
            continue;
        }

        // 获取深度值（单位：毫米）
        float Z_mm = depth_img.at<ushort>(center_y, center_x); // 假设深度图是 CV_16UC1

        // 如果深度值为0，表示无效点
        if (Z_mm == 0)
        {
            std::cout << "Face: Get Error Depth.d_x, d_y, d_z :" << 0 << "," << 0 << "," << 0 << std::endl;
            continue;
        }

        /*
            X = (u − cx) / fx × Z
            Y = (v − cy) / fy × Z
        */
        float Z = Z_mm; // 毫米
        float X = (center_x - intrinsics.cx) * Z / intrinsics.fx;
        float Y = (center_y - intrinsics.cy) * Z / intrinsics.fy;
        std::cout << "Face: xPos,yPos,zPos :" << X << "," << Y << "," << Z << std::endl;
    }
}

// 判断人脸是否正对着相机
// 返回值: -1.0 (左侧), 0.0-1.0 (正面), >1.0 (右侧)
// 值越接近0表示越正面，绝对值越大表示越侧脸
float SCRFD::is_facing_camera(const FaceDetection& detection)
{
    // 检查是否有关键点
    if (detection.landmarks.size() != 5) {
        return -999.0f;  // 无效值，表示无法判断
    }

    // SCRFD 5个关键点顺序: 左眼(0), 右眼(1), 鼻尖(2), 嘴角左(3), 嘴角右(4)
    // 根据关键点分布判断人脸朝向

    // 方法1: 比较双眼可见性
    float left_eye_x = detection.landmarks[0].x;
    float right_eye_x = detection.landmarks[1].x;
    float nose_x = detection.landmarks[2].x;
    float mouth_left_x = detection.landmarks[3].x;
    float mouth_right_x = detection.landmarks[4].x;

    // 计算人脸中心X坐标
    float face_center_x = (left_eye_x + right_eye_x + nose_x + mouth_left_x + mouth_right_x) / 5.0f;

    // 归一化偏移量（相对于人脸宽度）
    float face_width = detection.bbox.width;
    float normalized_offset = (nose_x - face_center_x) / face_width;

    // 判断标准:
    // normalized_offset 接近 0.0: 正面对着相机
    // normalized_offset > 0.2: 偏向右侧
    // normalized_offset < -0.2: 偏向左侧

    // 方法2: 计算不对称性（双眼中点和鼻子的关系）
    float eyes_center_x = (left_eye_x + right_eye_x) / 2.0f;
    float eyes_nose_ratio = (nose_x - eyes_center_x) / face_width;

    // 方法3: 嘴角不对称性
    float mouth_center_x = (mouth_left_x + mouth_right_x) / 2.0f;
    float mouth_nose_ratio = (nose_x - mouth_center_x) / face_width;

    // 综合判断（权重平均）
    float overall_ratio = 0.4f * normalized_offset + 0.3f * eyes_nose_ratio + 0.3f * mouth_nose_ratio;

    return overall_ratio;
}

// 判断人脸朝向程度（基于关键点）
// 返回值: 0.0-1.0，值越小越正面，值越大越侧脸
float SCRFD::get_face_pose_degree(const FaceDetection& detection)
{
    if (detection.landmarks.size() != 5) {
        return 1.0f;  // 无关键点，视为最大侧脸
    }

    // 计算关键点之间的几何关系
    float left_eye_x = detection.landmarks[0].x;
    float right_eye_x = detection.landmarks[1].x;
    float nose_x = detection.landmarks[2].x;
    float mouth_left_x = detection.landmarks[3].x;
    float mouth_right_x = detection.landmarks[4].x;

    float left_eye_y = detection.landmarks[0].y;
    float right_eye_y = detection.landmarks[1].y;
    float nose_y = detection.landmarks[2].y;
    float mouth_left_y = detection.landmarks[3].y;
    float mouth_right_y = detection.landmarks[4].y;

    // 指标1: 双眼宽度与鼻子位置的偏移
    float eyes_width = std::abs(right_eye_x - left_eye_x);
    float eyes_center_x = (left_eye_x + right_eye_x) / 2.0f;
    float nose_offset_x = std::abs(nose_x - eyes_center_x);
    float metric1 = nose_offset_x / (eyes_width + 1e-6f);

    // 指标2: 嘴角宽度与鼻子位置的偏移
    float mouth_width = std::abs(mouth_right_x - mouth_left_x);
    float mouth_center_x = (mouth_left_x + mouth_right_x) / 2.0f;
    float mouth_nose_offset_x = std::abs(nose_x - mouth_center_x);
    float metric2 = mouth_nose_offset_x / (mouth_width + 1e-6f);

    // 指标3: 左右对称性 - 比较左右两侧特征点到垂直中线的距离
    float vertical_center_x = (left_eye_x + right_eye_x + mouth_left_x + mouth_right_x) / 4.0f;
    float left_dist = std::abs(left_eye_x - vertical_center_x) + std::abs(mouth_left_x - vertical_center_x);
    float right_dist = std::abs(right_eye_x - vertical_center_x) + std::abs(mouth_right_x - vertical_center_x);
    float asymmetry_ratio = std::abs(left_dist - right_dist) / (left_dist + right_dist + 1e-6f);

    // 指标4: 深度估计（基于关键点纵向分布）
    // 正面脸时，双眼和嘴角的Y坐标应该在一个合理范围内
    float eyes_y = (left_eye_y + right_eye_y) / 2.0f;
    float mouth_y = (mouth_left_y + mouth_right_y) / 2.0f;
    float eye_mouth_dist = std::abs(mouth_y - eyes_y);
    float nose_y_offset = std::abs(nose_y - (eyes_y + mouth_y) / 2.0f);
    float metric3 = nose_y_offset / (eye_mouth_dist + 1e-6f);

    // 综合指标（加权平均）
    float pose_degree = 0.4f * metric1 + 0.3f * metric2 + 0.2f * asymmetry_ratio + 0.1f * metric3;

    // 归一化到 [0, 1] 范围
    pose_degree = std::min(pose_degree, 1.0f);

    return pose_degree;
}

// 判断是否为正脸（用于过滤侧脸）
bool SCRFD::is_frontal_face(const FaceDetection& detection, float threshold)
{
    float pose_degree = get_face_pose_degree(detection);
    return pose_degree < threshold;
}

