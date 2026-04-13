#pragma once
#include "yolo_det.h"
#include <fstream>
#include <numeric>
#include <algorithm>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <NvInferPlugin.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include "process.h"
#include "utils.h"

const float MAX_RANGE_MM    = 2000.0f;   // 只考虑 0-2 m 深度，噪点少
const float STEP_THRESH_MM  = 40.0f;     // 4 cm

#define CHECK_CUDA(call) { \
    cudaError_t status = call; \
    if (status != cudaSuccess) { \
        throw std::runtime_error("CUDA error at " + std::string(__FILE__) + ":" + \
                               std::to_string(__LINE__) + ": " + \
                               cudaGetErrorString(status)); \
    } \
}

YoloDet::YoloDet() 
{
    conf_thresh_ = 0.5;
    iou_thresh_ = 0.5;
    d_src_      = nullptr;
    d_src_size_ = 0;
}

YoloDet::~YoloDet() 
{
    // 释放资源
#if USE_TENSORRT_8
    if (context_) { context_->destroy(); }
    if (engine_) { engine_->destroy(); }
    if (runtime_) { runtime_->destroy(); }
#else
#endif

    if (buffers_[0]) { CHECK_CUDA(cudaFree(buffers_[0])); }
    if (buffers_[1]) { CHECK_CUDA(cudaFree(buffers_[1])); }
    if (stream_) { CHECK_CUDA(cudaStreamDestroy(stream_)); }
}

void YoloDet::load_engine(const std::string& engine_path) 
{
    //检查文件是否存在
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file) 
    {
        throw TensorRTException("无法打开引擎文件: " + engine_path + " (文件不存在或权限不足)");
    }

    //读取引擎文件
    engine_file.seekg(0, std::ios::end);
    const size_t file_size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);
    
    std::vector<char> engine_data(file_size);
    if (!engine_file.read(engine_data.data(), file_size)) 
    {
        throw TensorRTException("读取引擎文件失败: " + engine_path);
    }
    engine_file.close();

    //创建运行时
    runtime_ = nvinfer1::createInferRuntime(gLogger);
    if (!runtime_) 
    {
        throw TensorRTException("创建TensorRT运行时失败");
    }

    //反序列化引擎
    engine_ = runtime_->deserializeCudaEngine(engine_data.data(), file_size);
    if (!engine_) 
    {
        throw TensorRTException("反序列化CUDA引擎失败 (可能文件损坏或版本不兼容)");
    }
#if USE_TENSORRT_8
    //打印绑定信息 (调试用)
    std::cout << "========= 引擎绑定信息 =========\n";
    for (int i = 0; i < engine_->getNbBindings(); ++i) 
    {
        const char* name = engine_->getBindingName(i);
        const nvinfer1::Dims dims = engine_->getBindingDimensions(i);
        const bool is_input = engine_->bindingIsInput(i);
        if(is_input)
        {
            input_index_ = i;
        }
        else
        {
            output_index_ = i;
        }
        std::cout << "Binding " << i << ": " << name 
                  << (is_input ? " [INPUT]" : " [OUTPUT]")
                  << " Dims: ";
        for (int j = 0; j < dims.nbDims; ++j) 
        {
            std::cout << dims.d[j] << (j < dims.nbDims-1 ? "x" : "");
        }
        std::cout << "\n";
    }
    std::cout << "================================\n";

    //检查输入维度 (必须是3通道图像)
    const nvinfer1::Dims input_dims = engine_->getBindingDimensions(input_index_);
    if (input_dims.nbDims != 4 || input_dims.d[1] != 3) 
    {
        std::ostringstream oss;
        oss << "无效的输入维度: ";
        for (int i = 0; i < input_dims.nbDims; ++i) 
        {
            oss << input_dims.d[i] << (i < input_dims.nbDims-1 ? "x" : "");
        }
        oss << " (期望格式: Nx3xHxW)";
        throw TensorRTException(oss.str());
    }

    //记录输入输出参数
    batch_size_ = input_dims.d[0];
    input_c_ = input_dims.d[1];
    input_h_ = input_dims.d[2];
    input_w_ = input_dims.d[3];

    const nvinfer1::Dims output_dims = engine_->getBindingDimensions(output_index_);  
    num_classes_ = output_dims.d[1] - 4;
    num_boxes_ = output_dims.d[2];
    output_size_ = 1;
    for (int i = 0; i < output_dims.nbDims; ++i) 
    {
        output_size_ *= output_dims.d[i];
    }
#else
    int num_io = engine_->getNbIOTensors();
    for (int i = 0; i < num_io; ++i) 
    {
        const char* name = engine_->getIOTensorName(i);
        if (std::string(name) == "images") 
        {
            input_index_ = i;
        } 
        else if (std::string(name) == "output0") 
        {
            output_index_ = i;
        }
    }
    auto input_dims = engine_->getTensorShape("images");
    batch_size_ = input_dims.d[0];
    input_c_ = input_dims.d[1];
    input_h_ = input_dims.d[2];
    input_w_ = input_dims.d[3];
    
    auto output_dims = engine_->getTensorShape("output0");
    num_classes_ = output_dims.d[1] - 4;
    num_boxes_ = output_dims.d[2];
    output_size_ = 1;
    for (int i = 0; i < output_dims.nbDims; ++i) 
    {
        output_size_ *= output_dims.d[i];
    }
#endif
    //创建执行上下文
    context_ = engine_->createExecutionContext();
    if (!context_) 
    {
        throw TensorRTException("创建执行上下文失败");
    }
    prepare_buffers();
}

void YoloDet::prepare_buffers() {
    // 分配输入输出内存
    CHECK_CUDA(cudaMalloc(&buffers_[input_index_], 
                         batch_size_ * input_c_ * input_h_ * input_w_ * sizeof(float)));
    CHECK_CUDA(cudaMalloc(&buffers_[output_index_], output_size_ * sizeof(float)));
    CHECK_CUDA(cudaStreamCreate(&stream_));
}

void YoloDet::preprocess(const cv::Mat& img) 
{
    cv::Mat resized;
    ratio_ = std::min(input_h_ / (float)img.rows, input_w_ / (float)img.cols);
    int padw = std::round(img.cols * ratio_);
    int padh = std::round(img.rows * ratio_);
    // padw = (padw + 31) / 32 * 32; //这个是不等大的
	// padh = (padh + 31) / 32 * 32;
    bool bTestTimes = false; 
    if ((int)img.cols != padw || (int)img.rows != padh) 
    {
        if(bTestTimes)
        {
            auto start = std::chrono::high_resolution_clock::now();
            int nCount = 100;      
            for(int i =0; i< nCount; i++)
            {
                cv::resize(img, resized, cv::Size(padw, padh));
            }   
            auto end = std::chrono::high_resolution_clock::now();
            double ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
            std::cout << "det_resize_time: " << ave_time << "ms" << std::endl;
        }
        else
            cv::resize(img, resized, cv::Size(padw, padh));
    }
    else 
    {
        resized = img.clone();
    }
    dw_ = input_w_ - padw;
    dh_ = input_h_ - padh;
    dw_ /= 2.0f;
    dh_ /= 2.0f;
    int top    = int(std::round(dh_ - 0.1f));
    int bottom = int(std::round(dh_ + 0.1f));
    int left   = int(std::round(dw_ - 0.1f));
    int right  = int(std::round(dw_ + 0.1f));
   
    cv::Mat copy;
    cv::Mat float_img;
    if(bTestTimes)
    {
        auto start = std::chrono::high_resolution_clock::now();
        int nCount = 100;      
        for(int i =0; i< nCount; i++)
        {           
            cv::copyMakeBorder(resized, copy, top, bottom, left, right, cv::BORDER_CONSTANT, {114, 114, 114});
            copy.convertTo(float_img, CV_32FC3, 1.0 / 255.0);
        }   
        auto end = std::chrono::high_resolution_clock::now();
        double ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
        std::cout << "det_copyMakeBorder_time: " << ave_time << "ms" << std::endl;
    }
    else
    {
        cv::copyMakeBorder(resized, resized, top, bottom, left, right, cv::BORDER_CONSTANT, {114, 114, 114});
        resized.convertTo(float_img, CV_32FC3, 1.0 / 255.0);
        // cv::subtract(float_img, cv::Scalar(0.485f, 0.456f, 0.406f), float_img, cv::noArray(), -1);
        // cv::multiply(float_img, cv::Scalar(1 / 0.229f, 1 / 0.224f, 1 / 0.225f), float_img, 1, -1);
    }
    
    std::vector<float> chw_data(input_c_ * input_h_ * input_w_);
    float* chw_ptr = chw_data.data();
    const int channel_size = input_h_ * input_w_;
    std::vector<cv::Mat> channels;
    cv::cvtColor(float_img, float_img, cv::COLOR_RGB2BGR);
    if(!float_img.isContinuous())
    {
        std::cout<<"img is not continuous"<<std::endl;
    }
    if(bTestTimes)
    {
        auto start = std::chrono::high_resolution_clock::now();
        int nCount = 100;      
        for(int i =0; i< nCount; i++)
        {                        
            cv::split(float_img, channels);
            for (int c = 0; c < input_c_; ++c) 
            {
                // 直接复制整个通道数据
                std::memcpy(chw_ptr + c * input_h_ * input_w_, channels[c].data, input_h_ * input_w_ * sizeof(float));
            }
        }   
        auto end = std::chrono::high_resolution_clock::now();
        double ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
        std::cout << "HWC -> CHW time: " << ave_time << "ms" << std::endl;

        start = std::chrono::high_resolution_clock::now();    
        for(int i =0; i< nCount; i++)
        {
            CHECK_CUDA(cudaMemcpyAsync(static_cast<float*>(buffers_[input_index_]), chw_data.data(),
                input_c_ * input_h_ * input_w_ * sizeof(float), cudaMemcpyHostToDevice, stream_));
        }
        end = std::chrono::high_resolution_clock::now();
        ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
        std::cout << "copy time: " << ave_time << "ms" << std::endl;
    }
    else
    {
        cv::split(float_img, channels);
        for (int c = 0; c < input_c_; ++c) 
        {
            // 直接复制整个通道数据
            std::memcpy(chw_ptr + c * input_h_ * input_w_, channels[c].data, input_h_ * input_w_ * sizeof(float));
        }
        // save_chw_txt(chw_data.data(), input_c_, input_h_, input_w_, "trt_input_chw.txt");
#if 0        
        // 拷贝到GPU
        if (!load_txt_to_vec("ort_input_chw.txt", chw_data))
        {
            std::cerr << "cannot load trt_input_chw.txt\n";
            return;                                          // 你的错误处理
        }
        const size_t byte_size = chw_data.size() * sizeof(float);
        if (byte_size != input_c_ * input_h_ * input_w_ * sizeof(float))
        {
            std::cerr << "size mismatch\n";
            return;
        }
#endif
        CHECK_CUDA(cudaMemcpyAsync(static_cast<float*>(buffers_[input_index_]), chw_data.data(),
            input_c_ * input_h_ * input_w_ * sizeof(float), cudaMemcpyHostToDevice, stream_));
    }   
}

std::vector<Detection> YoloDet::infer(cv::Mat& img) 
{
    try 
    {
        bool bTestTimes = false; 
        if(bTestTimes)
        {
            
            auto start = std::chrono::high_resolution_clock::now();
            int nCount = 100;      
            for(int i =0; i< nCount; i++)
            {
#if PROCESS_GPU
                    preprocess_gpu(img);
#else
                    preprocess(img);
#endif
            }   
            auto end = std::chrono::high_resolution_clock::now();
            double ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
            std::cout << "det_pre_time: " << ave_time << "ms" << std::endl;
            
            start = std::chrono::high_resolution_clock::now();
            for(int i =0; i< nCount; i++)
            {
#if USE_TENSORRT_8        
                if (!context_->enqueueV2(buffers_, stream_, nullptr)) 
                {
                    throw std::runtime_error("Failed to execute inference");
                }
#else
                context_->setInputTensorAddress(engine_->getIOTensorName(input_index_), buffers_[input_index_]);
                context_->setOutputTensorAddress(engine_->getIOTensorName(output_index_), buffers_[output_index_]);
                context_->enqueueV3(stream_);
#endif
            }   
            end = std::chrono::high_resolution_clock::now();
            ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
            std::cout << "det_infer_time: " << ave_time << "ms" << std::endl;

            start = std::chrono::high_resolution_clock::now();
            std::vector<Detection> detections;   
            for(int i =0; i< nCount; i++)
            {
                detections.clear();
#if PROCESS_GPU
                    postprocess_gpu(detections, img.cols, img.rows);
#else
                    postprocess(detections, img.cols, img.rows);
#endif
            }   
            end = std::chrono::high_resolution_clock::now();
            ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
            std::cout << "det_post_time: " << ave_time << "ms" << std::endl;
            return detections;
        }
        else
        {
#if PROCESS_GPU
            preprocess_gpu(img);
#else
            preprocess(img);
#endif        
#if USE_TENSORRT_8        
            if (!context_->enqueueV2(buffers_, stream_, nullptr)) 
            {
                throw std::runtime_error("Failed to execute inference");
            }
#else
            context_->setInputTensorAddress(engine_->getIOTensorName(input_index_), buffers_[input_index_]);
            context_->setOutputTensorAddress(engine_->getIOTensorName(output_index_), buffers_[output_index_]);
            context_->enqueueV3(stream_);
#endif        
            CHECK_CUDA(cudaStreamSynchronize(stream_));
            
            std::vector<Detection> detections;
#if PROCESS_GPU
            postprocess_gpu(detections, img.cols, img.rows);
#else
            postprocess(detections, img.cols, img.rows);
#endif
            return detections;
        }               
    } 
    catch (const std::exception& e) 
    {
        throw std::runtime_error("Detection failed: " + std::string(e.what()));
    }
}

void YoloDet::postprocess(std::vector<Detection>& detections,
                          int img_width, int img_height) 
{
    // 1. GPU → CPU
    std::vector<float> output(output_size_);
    cudaMemcpy(output.data(), buffers_[output_index_],
               output_size_ * sizeof(float), cudaMemcpyDeviceToHost);
                           
    int max_index = 0;
    float max_score = 0;
    int class_id = 0;
    float score = 0;
    float xmin = 0, ymin = 0, xmax = 0, ymax = 0;
    float cx = 0, cy = 0, cw = 0, ch = 0;
    
    for (int i = 0; i < num_boxes_; ++i) 
    {
        for (int j = 4; j < num_classes_ + 4; j ++) 
        {
            if (4 == j)
            {
                max_score = output[j * num_boxes_ + i];
                max_index = j;   
            } 
            else 
            {
                if (max_score <  output[j * num_boxes_ + i])
                {
                    max_score = output[j * num_boxes_ + i];
                    max_index = j;   
                }
            }  
        }
       
        if (max_score < conf_thresh_) continue;

        class_id = max_index - 4;
        score = max_score;
        cx = (output[0 * num_boxes_ + i] - dw_) / ratio_;
        cy = (output[1 * num_boxes_ + i] - dh_) / ratio_;
        cw = output[2 * num_boxes_ + i] / ratio_;
        ch = output[3 * num_boxes_ + i] / ratio_;

        Detection detection;
        detection.bbox[0] = cx;
        detection.bbox[1] = cy;
        detection.bbox[2] = cw;
        detection.bbox[3] = ch;
        detection.conf = score;
        detection.class_id = class_id;        
        detections.push_back(detection);
    }

    // 5. NMS
    if (!detections.empty()) 
    {
        std::sort(detections.begin(), detections.end(),
                  [](const Detection& a, const Detection& b) { return a.conf > b.conf; });

        std::vector<Detection> keep;
        std::vector<bool> suppressed(detections.size(), false);

        for (size_t i = 0; i < detections.size(); ++i) 
        {
            if (suppressed[i]) continue;
            keep.push_back(detections[i]);

            for (size_t j = i + 1; j < detections.size(); ++j) 
            {
                if (suppressed[j]) continue;

                cv::Rect r1(detections[i].bbox[0] - detections[i].bbox[2]/2, detections[i].bbox[1] - detections[i].bbox[3]/2, detections[i].bbox[2], detections[i].bbox[3]);
                cv::Rect r2(detections[j].bbox[0] - detections[j].bbox[2]/2, detections[j].bbox[1] - detections[j].bbox[3]/2, detections[j].bbox[2], detections[j].bbox[3]);
                int x1 = std::max(r1.x, r2.x);
                int y1 = std::max(r1.y, r2.y);
                int x2 = std::min(r1.x + r1.width,  r2.x + r2.width);
                int y2 = std::min(r1.y + r1.height, r2.y + r2.height);

                int inter = std::max(0, x2 - x1) * std::max(0, y2 - y1);
                int area1 = r1.width * r1.height;
                int area2 = r2.width * r2.height;
                float iou = static_cast<float>(inter) /
                            (area1 + area2 - inter + 1e-6f);

                if (iou > iou_thresh_) suppressed[j] = true;
            }
        }
        detections.swap(keep);
    }   
}

void YoloDet::draw_results(cv::Mat& img, 
                          const std::vector<Detection>& detections,
                          const std::vector<std::string>& class_names) 
{
    static const std::vector<cv::Scalar> colors = 
    {
        cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255)
    };
    
    for (const auto& det : detections) 
    {
        // 将中心点坐标转换为左上角坐标
        float x1 = det.bbox[0] - det.bbox[2] / 2;
        float y1 = det.bbox[1] - det.bbox[3] / 2;
        float x2 = det.bbox[0] + det.bbox[2] / 2;
        float y2 = det.bbox[1] + det.bbox[3] / 2;
        
        // 确保坐标不超出图像边界
        x1 = std::max(0.0f, x1);
        y1 = std::max(0.0f, y1);
        x2 = std::min((float)img.cols, x2);
        y2 = std::min((float)img.rows, y2);
        
        cv::Rect rect(x1, y1, x2 - x1, y2 - y1);
        cv::Scalar color = colors[det.class_id % colors.size()];
        
        cv::rectangle(img, rect, color, 2);
        
        std::string label;
        if (!class_names.empty() && det.class_id < class_names.size()) 
        {
            label = class_names[det.class_id] + ": " + cv::format("%.2f", det.conf);
        } 
        else 
        {
            label = cv::format("%d: %.2f", det.class_id, det.conf);
        }
        
        int baseline;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        
        // 调整标签框的位置，确保不超出图像顶部
        int text_y = std::max(rect.y, text_size.height + 5);
        cv::rectangle(img, 
                     cv::Point(rect.x, text_y - text_size.height - 5),
                     cv::Point(rect.x + text_size.width, text_y),
                     color, cv::FILLED);
        cv::putText(img, label, 
                   cv::Point(rect.x, text_y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
}
void YoloDet::draw_results(cv::Mat& img, 
                          const std::vector<Detection>& detections,
                          cv::Mat& depth_img,
                          const std::vector<std::string>& class_names
                          ) 
{
    static const std::vector<cv::Scalar> colors = 
    {
        cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255)
    };
    
    for (const auto& det : detections) 
    {
        // 将中心点坐标转换为左上角坐标
        float x1 = det.bbox[0] - det.bbox[2] / 2;
        float y1 = det.bbox[1] - det.bbox[3] / 2;
        float x2 = det.bbox[0] + det.bbox[2] / 2;
        float y2 = det.bbox[1] + det.bbox[3] / 2;
        
        // 确保坐标不超出图像边界
        x1 = std::max(0.0f, x1);
        y1 = std::max(0.0f, y1);
        x2 = std::min((float)img.cols, x2);
        y2 = std::min((float)img.rows, y2);
        
        cv::Rect rect(x1, y1, x2 - x1, y2 - y1);
        cv::Scalar color = colors[det.class_id % colors.size()];
        
        cv::rectangle(img, rect, color, 2);
        
        std::string label;
        if (!class_names.empty() && det.class_id < class_names.size()) 
        {
            label = class_names[det.class_id] + ": " + cv::format("%.2f", det.conf);
        } 
        else 
        {
            label = cv::format("%d: %.2f", det.class_id, det.conf);
        }
        
        int baseline;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        
        // 调整标签框的位置，确保不超出图像顶部
        int text_y = std::max(rect.y, text_size.height + 5);
        cv::rectangle(img, 
                     cv::Point(rect.x, text_y - text_size.height - 5),
                     cv::Point(rect.x + text_size.width, text_y),
                     color, cv::FILLED);
        cv::putText(img, label, 
                   cv::Point(rect.x, text_y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        
        std::cout << class_names[det.class_id]  <<":center_x,center_y :" << det.bbox[0] << "," << det.bbox[1] << std::endl;
        // 获取深度值（单位：毫米）
        float Z_mm = depth_img.at<ushort>(det.bbox[1], det.bbox[0]); // 假设深度图是 CV_16UC1
        // 如果深度值为0，表示无效点
        if (Z_mm == 0)
        {
            std::cout << "Get Error Depth.d_x, d_y, d_z :" << 0 << "," << 0 << "," << 0 << std::endl;
        }
        // float Z = Z_mm / 1000.0f; // 毫米 -> 米
        // float X = (det.bbox[0] - 323.65252685546875) * Z / 608.0307006835938;
        // float Y = (det.bbox[1] - 248.4098358154297) * Z / 607.4725952148438;
        // std::cout << class_names[det.class_id] << ":xPos,yPos,zPos :" << X << "," << Y << "," << Z << std::endl;
        // std::string strPos = "xPos:" + std::to_string(X) + ",yPos:"+ std::to_string(Y) + ",zPos:"+ std::to_string(Z);
        // cv::putText(img, strPos , cv::Point(rect.x, text_y - 35), cv::FONT_HERSHEY_PLAIN, 0.5,
        //             cv::Scalar(0, 0, 0), 1);
    }
}

bool YoloDet::IsStairs(const Detection& detection, const cv::Mat& depth_img)
{   
    // 将中心点坐标转换为左上角坐标
    float x1 = detection.bbox[0] - detection.bbox[2] / 2;
    float y1 = detection.bbox[1] - detection.bbox[3] / 2;
    float x2 = detection.bbox[0] + detection.bbox[2] / 2;
    float y2 = detection.bbox[1] + detection.bbox[3] / 2;
    
    // 确保坐标不超出图像边界
    x1 = std::max(0.0f, x1);
    y1 = std::max(0.0f, y1);
    x2 = std::min((float)depth_img.cols, x2);
    y2 = std::min((float)depth_img.rows, y2);

    float min_z = MAX_RANGE_MM;
    float max_z = 0.0f;

    for (int y = y1; y < y2; ++y) 
    {
        for (int x = x1; x < x2; ++x) 
        {
            float z = depth_img.at<ushort>(y, x);
            if (z > 0 && z < MAX_RANGE_MM) 
            {
                min_z = std::min(min_z, z);
                max_z = std::max(max_z, z);
            }
        }
    }
    if (max_z - min_z > STEP_THRESH_MM)
        return true;
    else 
        return false;
}

void YoloDet::FilterRes(const cv::Mat& depth_img, std::vector<Detection>& detections)
{
    // for(auto it=detections.begin(); it!=detections.end(); it++)
    // {
    //     if(it->class_id==80 || it->class_id==81)
    //     {
    //         if(!IsStairs(*it, depth_img))
    //         {
    //             detections.erase(it);
    //         }
    //     }        
    // }
    // 用 erase-remove 惯用法，避免迭代器失效
    auto new_end = std::remove_if(detections.begin(), detections.end(),
                                  [&](const Detection& d)
                                  {
                                      return (d.class_id == 80 || d.class_id == 81) &&
                                             !IsStairs(d, depth_img);
                                  });
    detections.erase(new_end, detections.end());
}

void YoloDet::preprocess_gpu(const cv::Mat& img)
{
    int src_h = img.rows;
    int src_w = img.cols;

    // 计算缩放 / 填充
    ratio_ = std::min(input_h_ / (float)src_h, input_w_ / (float)src_w);
    int padw = std::round(src_w * ratio_);
    int padh = std::round(src_h * ratio_);
    dw_ = (input_w_ - padw) / 2.0f;
    dh_ = (input_h_ - padh) / 2.0f;
    int pad_top = int(std::round(dh_ - 0.1f));
    int pad_left = int(std::round(dw_ - 0.1f));

    // 一次性申请 GPU buffer
    size_t img_byte = src_h * src_w * sizeof(uchar3);
    if (d_src_ == nullptr || img_byte >  d_src_size_ )
    {
        CHECK_CUDA(cudaFree(d_src_));
        CHECK_CUDA(cudaMalloc(&d_src_, img_byte));
        d_src_size_ = img_byte;
    }

    // 把原图拷到 GPU（BGR连续）
    CHECK_CUDA(cudaMemcpyAsync(d_src_, img.data, img_byte,
                               cudaMemcpyHostToDevice, stream_));
    //启动cuda
    launch_preprocess_kernel(
        d_src_, src_h, src_w,
        (float*)buffers_[input_index_], input_h_, input_w_,
        ratio_, pad_top, pad_left, 1.0f / 255.0f, stream_);
}

void YoloDet::postprocess_gpu(std::vector<Detection>& detections,
                              int img_w, int img_h)
{
    // 申请 GPU 缓存（只申请一次）
    static Bbox32* d_out = nullptr;
    static int*    d_keep = nullptr;
    if (!d_out) cudaMalloc(&d_out, num_boxes_ * sizeof(Bbox32));
    if (!d_keep) cudaMalloc(&d_keep, sizeof(int));

    // 一次 kernel 完成过滤 + NMS
    std::vector<Bbox32> keep_box;
    yolo_postprocess_gpu(
        (float*)buffers_[output_index_], num_boxes_, num_classes_,
        conf_thresh_, iou_thresh_, ratio_, dw_, dh_,
        d_out, d_keep, keep_box, stream_);
    // 只把 keep 结果拷回
    int h_keep;
    cudaMemcpyAsync(&h_keep, d_keep, sizeof(int), cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);
    if (h_keep == 0) return;

    std::vector<Bbox32> host_box(h_keep);
    cudaMemcpyAsync(host_box.data(), d_out, h_keep * sizeof(Bbox32),
                    cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);

    // 转回Detection 格式
    detections.reserve(keep_box.size());
    for (const auto& b : keep_box) {
        Detection d;
        d.bbox[0] = b.x; d.bbox[1] = b.y;
        d.bbox[2] = b.w; d.bbox[3] = b.h;
        d.conf    = b.conf;
        d.class_id= b.cls;
        detections.push_back(d);
    }
}