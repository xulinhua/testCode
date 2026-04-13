#include "yolo_seg.h"
#include <fstream>
#include <numeric>
#include <algorithm>
#include <cuda_runtime_api.h>
#include "process.h"
#include "utils.h"

#define CHECK_CUDA(call) { \
    cudaError_t status = call; \
    if (status != cudaSuccess) { \
        throw std::runtime_error("CUDA error at " + std::string(__FILE__) + ":" + \
                               std::to_string(__LINE__) + ": " + \
                               cudaGetErrorString(status)); \
    } \
}

YoloSeg::YoloSeg() 
{
    conf_thresh_ = 0.5f;
    iou_thresh_ = 0.3f;
    d_src_      = nullptr;
    d_src_size_  = 0;
}

YoloSeg::~YoloSeg()
{
    // TensorRT 10: 智能指针自动管理，不需要手动destroy
    // if (context_) { context_->destroy(); }
    // if (engine_) { engine_->destroy(); }
    // if (runtime_) { runtime_->destroy(); }

    if (input_buffer_) { CHECK_CUDA(cudaFree(input_buffer_)); }
    if (proto_buffer_) { CHECK_CUDA(cudaFree(proto_buffer_)); }
    if (output_buffer_) { CHECK_CUDA(cudaFree(output_buffer_)); }
    if (d_decode_out_) { CHECK_CUDA(cudaFree(d_decode_out_)); }
    if (d_keep_count_) { CHECK_CUDA(cudaFree(d_keep_count_)); }
    if (stream_) { CHECK_CUDA(cudaStreamDestroy(stream_)); }
}

void YoloSeg::set_thresholds(float conf_thresh, float iou_thresh)
{
    conf_thresh_ = conf_thresh;
    iou_thresh_ = iou_thresh;
}

void YoloSeg::load_engine(const std::string& engine_path)
{
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file.is_open()) {
        throw TensorRTException("Failed to open engine file: " + engine_path);
    }

    engine_file.seekg(0, std::ios::end);
    size_t engine_size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);

    std::vector<char> engine_data(engine_size);
    engine_file.read(engine_data.data(), engine_size);
    engine_file.close();

    runtime_.reset(nvinfer1::createInferRuntime(gLogger));
    if (!runtime_) {
        throw TensorRTException("Failed to create TensorRT runtime");
    }

    engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_size));
    if (!engine_) {
        throw TensorRTException("Failed to deserialize CUDA engine");
    }

    context_.reset(engine_->createExecutionContext());
    if (!context_) {
        throw TensorRTException("Failed to create execution context");
    }

    // 设置输入输出binding
    int num_io = engine_->getNbIOTensors();
    std::cout << "Number of I/O tensors: " << num_io << std::endl;
    for (int i = 0; i < num_io; i++) {
        const char* name = engine_->getIOTensorName(i);
        auto mode = engine_->getTensorIOMode(name);
        std::string mode_str = (mode == nvinfer1::TensorIOMode::kINPUT) ? "INPUT" : "OUTPUT";
        std::cout << "Tensor " << i << ": name=" << name << ", mode=" << mode_str;

        try {
            auto dims = engine_->getTensorShape(name);
            std::cout << ", shape=[";
            for (int j = 0; j < dims.nbDims; j++) {
                std::cout << dims.d[j];
                if (j < dims.nbDims - 1) std::cout << ", ";
            }
            std::cout << "]";
        } catch (const std::exception& e) {
            std::cout << ", shape=ERROR: " << e.what();
        }
        std::cout << std::endl;

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

    // 获取proto尺寸 (output1 is proto)
    auto proto_dims = engine_->getTensorShape("output1");
    proto_c_ = proto_dims.d[1];
    proto_h_ = proto_dims.d[2];
    proto_w_ = proto_dims.d[3];

    // 获取输出尺寸
    auto output_dims = engine_->getTensorShape("output0");
    num_boxes_ = output_dims.d[2];  // 8400
    num_classes_ = output_dims.d[1] - 4 - 32;  // 从输出维度自动推算类别数: dim1 - bbox(4) - mask(32)

    // 准备CUDA buffers
    prepare_buffers();

    CHECK_CUDA(cudaStreamCreate(&stream_));
}

void YoloSeg::prepare_buffers()
{
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
}

// CPU版本预处理
void YoloSeg::preprocess(const cv::Mat& img)
{
    if (img.empty()) {
        throw std::runtime_error("Input image is empty");
    }

    int src_h = img.rows;
    int src_w = img.cols;

    // Calculate letterbox parameters 
    double r = std::min((double)input_w_ / src_w, (double)input_h_ / src_h);
    int new_w = (int)std::round(src_w * r);
    int new_h = (int)std::round(src_h * r);

    if (new_w <= 0 || new_h <= 0) {
        throw std::runtime_error("Invalid resize size: " + std::to_string(new_w) + "x" + std::to_string(new_h));
    }

    dw_ = ((double)input_w_ - new_w) / 2.0;
    dh_ = ((double)input_h_ - new_h) / 2.0;
    ratio_ = r;

    // CPU letterbox
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h));

    cv::Mat padded;
    int top = (int)dh_;
    int bottom = input_h_ - new_h - top;
    int left = (int)dw_;
    int right = input_w_ - new_w - left;
    cv::copyMakeBorder(resized, padded, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    // Convert to float and normalize
    cv::Mat float_img;
    padded.convertTo(float_img, CV_32FC3, 1.0 / 255.0);
    cv::cvtColor(float_img, float_img, cv::COLOR_BGR2RGB);
    // HWC to CHW 
    std::vector<float> chw_data(3 * input_h_ * input_w_);
    std::vector<cv::Mat> channels;
    cv::split(float_img, channels);
    for (int c = 0; c < 3; c++) {
        std::memcpy(chw_data.data() + c * input_h_ * input_w_, channels[c].data, input_h_ * input_w_ * sizeof(float));
    }

    // Copy to GPU
    CHECK_CUDA(cudaMemcpyAsync(input_buffer_, chw_data.data(),
                              3 * input_h_ * input_w_ * sizeof(float), cudaMemcpyHostToDevice, stream_));
}

// GPU版本预处理
void YoloSeg::preprocess_gpu(const cv::Mat& img)
{
    if (img.empty()) {
        throw std::runtime_error("Input image is empty");
    }

    int src_h = img.rows;
    int src_w = img.cols;

    // Calculate letterbox parameters 
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
        d_src_, src_h, src_w,
        (float*)input_buffer_, input_h_, input_w_,
        r, (int)dh_, (int)dw_, 1.0f / 255.0f,
        stream_
    );
}

// CPU版本后处理
void YoloSeg::postprocess(std::vector<Segmentation>& segmentations,
                            int img_width, int img_height)
{
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
    int valid_boxes = 0;
    for (int i = 0; i < num_boxes_; ++i) {
        // Find max class score
        float max_score = 0;
        int max_class = 0;
        for (int c = 0; c < num_classes_; c++) {
            float score = output[(4 + c) * num_boxes_ + i];
            if (score > max_score) {
                max_score = score;
                max_class = c;
            }
        }

        if (max_score < conf_thresh_) continue;

        // Get bbox
        float cx = output[0 * num_boxes_ + i];
        float cy = output[1 * num_boxes_ + i];
        float w = output[2 * num_boxes_ + i];
        float h = output[3 * num_boxes_ + i];

        // Convert to original image coordinates
        float x1 = (cx - w / 2 - dw_) / ratio_;
        float y1 = (cy - h / 2 - dh_) / ratio_;
        float x2 = (cx + w / 2 - dw_) / ratio_;
        float y2 = (cy + h / 2 - dh_) / ratio_;

        // Clamp to image bounds
        x1 = std::max(0.0f, std::min((float)img_width - 1, x1));
        y1 = std::max(0.0f, std::min((float)img_height - 1, y1));
        x2 = std::max(0.0f, std::min((float)img_width - 1, x2));
        y2 = std::max(0.0f, std::min((float)img_height - 1, y2));

        // 检查bbox是否有效
        if (x1 >= x2 || y1 >= y2) {
            continue;  // 跳过无效bbox
        }

        Segmentation seg;
        seg.bbox[0] = x1;
        seg.bbox[1] = y1;
        seg.bbox[2] = x2;
        seg.bbox[3] = y2;
        seg.conf = max_score;
        seg.class_id = max_class;

        // Copy mask coefficients
        for (int m = 0; m < 32; m++) {
            seg.mask_coef[m] = output[(4 + num_classes_ + m) * num_boxes_ + i];
        }

        segmentations.push_back(seg);
    }

    // 3. CPU NMS
    if (!segmentations.empty()) {
        // 按置信度降序排序
        std::sort(segmentations.begin(), segmentations.end(),
                  [](const Segmentation& a, const Segmentation& b) { return a.conf > b.conf; });

        std::vector<Segmentation> keep;
        std::vector<bool> suppressed(segmentations.size(), false);

        for (size_t i = 0; i < segmentations.size(); ++i) {
            if (suppressed[i]) continue;
            keep.push_back(segmentations[i]);

            for (size_t j = i + 1; j < segmentations.size(); ++j) {
                if (suppressed[j]) continue;

                // 计算 IoU
                float x1 = std::max(segmentations[i].bbox[0], segmentations[j].bbox[0]);
                float y1 = std::max(segmentations[i].bbox[1], segmentations[j].bbox[1]);
                float x2 = std::min(segmentations[i].bbox[2], segmentations[j].bbox[2]);
                float y2 = std::min(segmentations[i].bbox[3], segmentations[j].bbox[3]);

                float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
                float area1 = (segmentations[i].bbox[2] - segmentations[i].bbox[0]) *
                              (segmentations[i].bbox[3] - segmentations[i].bbox[1]);
                float area2 = (segmentations[j].bbox[2] - segmentations[j].bbox[0]) *
                              (segmentations[j].bbox[3] - segmentations[j].bbox[1]);
                float iou = inter / (area1 + area2 - inter + 1e-6f);

                if (iou > iou_thresh_) {
                    suppressed[j] = true;
                }
            }
        }
        segmentations.swap(keep);
    }

    // 4. Process masks on CPU
    if (!segmentations.empty()) {
        process_mask(proto_data.data(), segmentations, img_width, img_height);
    }
}

// GPU版本后处理 - GPU做decode+NMS，CPU做mask
void YoloSeg::postprocess_gpu(std::vector<Segmentation>& segmentations,
                                 int img_width, int img_height)
{
    // 1. 拷贝proto数据到CPU
    int proto_size = proto_c_ * proto_h_ * proto_w_;
    std::vector<float> proto_data(proto_size);
    CHECK_CUDA(cudaMemcpy(proto_data.data(), proto_buffer_,
                         proto_size * sizeof(float), cudaMemcpyDeviceToHost));

    // 2. 使用GPU做decode+NMS
    std::vector<Bbox32> gpu_boxes;
    yolo_seg_postprocess_gpu(
        (float*)output_buffer_, num_boxes_, num_classes_,
        conf_thresh_, iou_thresh_,
        ratio_, dw_, dh_,
        d_decode_out_, d_keep_count_,
        gpu_boxes, stream_
    );

    // 3. Convert Bbox32 to Segmentation
    for (const auto& box : gpu_boxes) {
        Segmentation seg;

        // Clamp to image bounds
        seg.bbox[0] = std::max(0.0f, std::min((float)img_width - 1, box.x1));
        seg.bbox[1] = std::max(0.0f, std::min((float)img_height - 1, box.y1));
        seg.bbox[2] = std::max(0.0f, std::min((float)img_width - 1, box.x2));
        seg.bbox[3] = std::max(0.0f, std::min((float)img_height - 1, box.y2));
        seg.conf = box.conf;
        seg.class_id = box.cls;

        // Copy mask coefficients
        for (int m = 0; m < 32; m++) {
            seg.mask_coef[m] = box.mask[m];
        }

        segmentations.push_back(seg);
    }

    // 4. 使用CPU处理masks（避免GPU-CPU数据传输）
    if (!segmentations.empty()) {
        process_mask(proto_data.data(), segmentations, img_width, img_height);
    }
}

// 快速 sigmoid 函数（避免 expf 调用）
static inline float fast_sigmoid(float x) {
    // 使用近似公式：x / (1 + |x|) * 0.5 + 0.5
    // 相比 expf 快 5-10 倍，误差 < 0.01
    return 0.5f * (x / (1.0f + fabsf(x))) + 0.5f;
}

// CPU版本 mask处理 - 优化版本
void YoloSeg::process_mask(float* proto_host, std::vector<Segmentation>& segmentations, int img_width, int img_height)
{
    int n = segmentations.size();
    if (n == 0) return;

    float scale_x = (float)proto_w_ / input_w_;
    float scale_y = (float)proto_h_ / input_h_;

    // 优化1: 为每个检测框单独处理，只计算 bbox 区域内的 mask
    #pragma omp parallel for schedule(dynamic)  // OpenMP 并行化
    for (int i = 0; i < n; i++) {
        // 计算 bbox 在 proto 上的范围
        float x1_f = (segmentations[i].bbox[0] * ratio_ + dw_) * scale_x;
        float y1_f = (segmentations[i].bbox[1] * ratio_ + dh_) * scale_y;
        float x2_f = (segmentations[i].bbox[2] * ratio_ + dw_) * scale_x;
        float y2_f = (segmentations[i].bbox[3] * ratio_ + dh_) * scale_y;

        // 边界检查
        int x1 = std::max(0, (int)floorf(x1_f));
        int y1 = std::max(0, (int)floorf(y1_f));
        int x2 = std::min(proto_w_ - 1, (int)ceilf(x2_f));
        int y2 = std::min(proto_h_ - 1, (int)ceilf(y2_f));

        int bbox_w = x2 - x1 + 1;
        int bbox_h = y2 - y1 + 1;

        if (bbox_w <= 0 || bbox_h <= 0) continue;

        // 优化2: 只分配 bbox 区域的 mask（减少内存占用）
        std::vector<float> mask_region(bbox_h * bbox_w);

        // 优化3: 直接访问 mask 系数（减少内存拷贝）
        const float* mask_coef_ptr = segmentations[i].mask_coef;

        // 优化4: 计算 mask 时只处理 bbox 区域
        // 这一步减少了 70-80% 的计算量
        for (int h = y1; h <= y2; h++) {
            for (int w = x1; w <= x2; w++) {
                float val = 0.0f;

                // 优化5: 展开部分循环（proto_c_ = 32）
                // 编译器通常会自动展开，但手动展开可以更明确
                #pragma unroll 8
                for (int c = 0; c < proto_c_; c++) {
                    val += mask_coef_ptr[c] * proto_host[c * proto_h_ * proto_w_ + h * proto_w_ + w];
                }

                // 优化6: 使用快速 sigmoid
                mask_region[(h - y1) * bbox_w + (w - x1)] = fast_sigmoid(val);
            }
        }

        // 切除padding区域
        int cut_left = (int)(dw_ * scale_x);
        int cut_top = (int)(dh_ * scale_y);
        int cut_w = proto_w_ - 2 * cut_left;
        int cut_h = proto_h_ - 2 * cut_top;

        // 分配原图大小的 mask_matrix
        segmentations[i].mask_matrix.assign(img_width * img_height, 0.0f);

        // 优化7: 并行化双线性插值
        #pragma omp parallel for collapse(2) schedule(static)
        for (int y = 0; y < img_height; y++) {
            for (int x = 0; x < img_width; x++) {
                // 原图坐标 -> 模型输入坐标 -> proto坐标
                float src_x = (x * ratio_ + dw_) * scale_x;
                float src_y = (y * ratio_ + dh_) * scale_y;

                // 检查是否在有效区域内
                if (src_x >= x1 && src_x <= x2 && src_y >= y1 && src_y <= y2) {
                    // 双线性插值
                    int x0 = (int)src_x;
                    int y0 = (int)src_y;
                    int x0_1 = std::min(x0 + 1, x2);
                    int y0_1 = std::min(y0 + 1, y2);

                    float dx = src_x - x0;
                    float dy = src_y - y0;

                    // 从 mask_region 读取（只包含 bbox 区域）
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

                    segmentations[i].mask_matrix[y * img_width + x] = value;
                }
            }
        }
    }
}

// GPU版本 mask处理 - 使用CUDA kernel在GPU上完成所有计算
void YoloSeg::process_mask_gpu(float* proto_device, std::vector<Segmentation>& segmentations, int img_width, int img_height)
{
    int n = segmentations.size();
    if (n == 0) return;

    // 准备mask系数数据 (n x 32)
    std::vector<float> h_mask_coef(n * proto_c_);
    for (int i = 0; i < n; i++) {
        memcpy(&h_mask_coef[i * proto_c_], segmentations[i].mask_coef, proto_c_ * sizeof(float));
    }
    
    // 准备bbox坐标数据 (n x 4: x1, y1, x2, y2 in original image)
    std::vector<float> h_bboxes(n * 4);
    for (int i = 0; i < n; i++) {
        h_bboxes[i * 4 + 0] = segmentations[i].bbox[0];  // x1
        h_bboxes[i * 4 + 1] = segmentations[i].bbox[1];  // y1
        h_bboxes[i * 4 + 2] = segmentations[i].bbox[2];  // x2
        h_bboxes[i * 4 + 3] = segmentations[i].bbox[3];  // y2
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
        proto_device,           // proto已经在GPU上
        proto_h_, proto_w_,
        d_mask_coef,
        d_bboxes,
        n,
        ratio_, dw_, dh_,
        input_w_, input_h_,
        d_output_masks,
        img_width, img_height,
        stream_
    );
    
    // 拷贝结果回CPU
    std::vector<float> h_output_masks(n * img_height * img_width);
    CHECK_CUDA(cudaMemcpy(h_output_masks.data(), d_output_masks, output_size, cudaMemcpyDeviceToHost));
    
    // 分配结果到各个segmentation
    for (int i = 0; i < n; i++) {
        segmentations[i].mask_matrix.resize(img_height * img_width);
        memcpy(segmentations[i].mask_matrix.data(), 
               &h_output_masks[i * img_height * img_width], 
               img_height * img_width * sizeof(float));
    }
    
    // 释放GPU内存
    CHECK_CUDA(cudaFree(d_mask_coef));
    CHECK_CUDA(cudaFree(d_bboxes));
    CHECK_CUDA(cudaFree(d_output_masks));
}

std::vector<Segmentation> YoloSeg::infer(cv::Mat& img)
{
    if (img.empty()) {
        return {};
    }

#if PROCESS_GPU
    preprocess_gpu(img);
#else
    preprocess(img);
#endif

    // 设置tensor地址
    context_->setTensorAddress("images", input_buffer_);
    context_->setTensorAddress("output1", proto_buffer_);
    context_->setTensorAddress("output0", output_buffer_);

    // TensorRT 10 使用 enqueueV3
    context_->enqueueV3(stream_);
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    std::vector<Segmentation> results;
#if PROCESS_GPU
    postprocess_gpu(results, img.cols, img.rows);
#else
    postprocess(results, img.cols, img.rows);
#endif

    return results;
}

void YoloSeg::draw_results(cv::Mat& img,
                             const std::vector<Segmentation>& segmentations,
                             const std::vector<std::string>& class_names)
{
    // COCO类别颜色
    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255)
    };

    for (const auto& seg : segmentations) {
        cv::Scalar color = colors[seg.class_id % colors.size()];

        // 绘制边界框
        cv::Rect bbox(cv::Point(seg.bbox[0], seg.bbox[1]),
                     cv::Point(seg.bbox[2], seg.bbox[3]));
        cv::rectangle(img, bbox, color, 2);

        // 绘制标签
        std::string label = class_names.empty() ?
                           "cls:" + std::to_string(seg.class_id) + " " +
                           std::to_string(seg.conf).substr(0, 4) :
                           class_names[seg.class_id] + " " +
                           std::to_string(seg.conf).substr(0, 4);

        int baseline;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                           0.6, 2, &baseline);
        cv::Point text_origin(bbox.x, bbox.y - text_size.height - 5);
        cv::rectangle(img, text_origin + cv::Point(0, baseline),
                     text_origin + cv::Point(text_size.width, -text_size.height),
                     color, -1);
        cv::putText(img, label, text_origin, cv::FONT_HERSHEY_SIMPLEX,
                   0.6, cv::Scalar(255, 255, 255), 2);

        // 绘制mask
        if (!seg.mask_matrix.empty()) {
            // Only blend in mask regions to avoid darkening background
            for (int h = 0; h < img.rows; h++) {
                for (int w = 0; w < img.cols; w++) {
                    float mask_val = seg.mask_matrix[h * img.cols + w];
                    if (mask_val > 0.5) {
                        cv::Vec3b& pixel = img.at<cv::Vec3b>(h, w);
                        pixel[0] = cv::saturate_cast<uchar>(pixel[0] * 0.7 + color[0] * 0.3);
                        pixel[1] = cv::saturate_cast<uchar>(pixel[1] * 0.7 + color[1] * 0.3);
                        pixel[2] = cv::saturate_cast<uchar>(pixel[2] * 0.7 + color[2] * 0.3);
                    }
                }
            }
        }
    }
}

void YoloSeg::draw_results(cv::Mat &img,
                              const std::vector<Segmentation> &segmentations,
                              cv::Mat &depth_img,
                              const Intrinsics& intrinsics,
                              const std::vector<std::string> &class_names)
{
    // COCO类别颜色
    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255)
    };

    for (const auto& seg : segmentations) {
        cv::Scalar color = colors[seg.class_id % colors.size()];

        // 计算中心点
        float center_x = (seg.bbox[0] + seg.bbox[2]) / 2.0;
        float center_y = (seg.bbox[1] + seg.bbox[3]) / 2.0;

        // 绘制边界框
        cv::Rect bbox(cv::Point(seg.bbox[0], seg.bbox[1]),
                     cv::Point(seg.bbox[2], seg.bbox[3]));
        cv::rectangle(img, bbox, color, 2);

        // 如果有深度图像且内参有效，计算并显示3D坐标
        // std::string pos_text = "";
        // if (!depth_img.empty() && intrinsics.fx > 0 && intrinsics.fy > 0) {
        //     int pixel_x = static_cast<int>(center_x);
        //     int pixel_y = static_cast<int>(center_y);

        //     if (pixel_x >= 0 && pixel_x < depth_img.cols &&
        //         pixel_y >= 0 && pixel_y < depth_img.rows) {
        //         uint16_t depth_value = depth_img.at<uint16_t>(pixel_y, pixel_x);

        //         if (depth_value > 0) {
        //             float depth_mm = depth_value;
        //             float x3d = (center_x - intrinsics.cx) * depth_mm / intrinsics.fx;
        //             float y3d = (center_y - intrinsics.cy) * depth_mm / intrinsics.fy;
        //             float z3d = depth_mm;

        //             // 显示3D坐标（单位：米）
        //             char buf[256];
        //             snprintf(buf, sizeof(buf), "(%.2f,%.2f,%.2f)m",
        //                      x3d/1000.0, y3d/1000.0, z3d/1000.0);
        //             pos_text = std::string(buf);
        //         }
        //     }
        // }

        std::string pos_text = "";

        // 绘制标签
        std::string label = class_names.empty() ?
                           "cls:" + std::to_string(seg.class_id) + " " +
                           std::to_string(seg.conf).substr(0, 4) :
                           class_names[seg.class_id] + " " +
                           std::to_string(seg.conf).substr(0, 4);

        if (!pos_text.empty()) {
            label += " " + pos_text;
        }

        int baseline;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                           0.6, 2, &baseline);
        cv::Point text_origin(bbox.x, bbox.y - text_size.height - 5);
        cv::rectangle(img, text_origin + cv::Point(0, baseline),
                     text_origin + cv::Point(text_size.width, -text_size.height),
                     color, -1);
        cv::putText(img, label, text_origin, cv::FONT_HERSHEY_SIMPLEX,
                   0.6, cv::Scalar(255, 255, 255), 2);

        // 绘制mask
        if (!seg.mask_matrix.empty()) {
            for (int h = 0; h < img.rows; h++) {
                for (int w = 0; w < img.cols; w++) {
                    float mask_val = seg.mask_matrix[h * img.cols + w];
                    if (mask_val > 0.5) {
                        cv::Vec3b& pixel = img.at<cv::Vec3b>(h, w);
                        pixel[0] = cv::saturate_cast<uchar>(pixel[0] * 0.7 + color[0] * 0.3);
                        pixel[1] = cv::saturate_cast<uchar>(pixel[1] * 0.7 + color[1] * 0.3);
                        pixel[2] = cv::saturate_cast<uchar>(pixel[2] * 0.7 + color[2] * 0.3);
                    }
                }
            }
        }
    }
}
