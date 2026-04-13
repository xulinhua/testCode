#pragma once
#include "paddle_ocr.h"
#include <fstream>
#include <NvOnnxParser.h>
#include <cuda_runtime.h>
#include <omp.h>

using namespace nvinfer1;

OcrRec::OcrRec() 
{
    input_h_ = 32;
    input_w_ = 128;
    max_batch_size_ = 16;
    predict_shape_default_ = {32, 39};  // 示例值，请替换为实际值
}

OcrRec::~OcrRec() 
{
#if USE_TENSORRT_8
    if (context_) context_->destroy();
    if (engine_) engine_->destroy();
    if (runtime_) { runtime_->destroy(); }
#else
#endif
    for (void* p : buffers_) cudaFree(p);
    cudaStreamDestroy(stream_);
}

void OcrRec::load_engine(const std::string& engine_path, const std::string& dict_path) 
{
    /* 读字符字典 */
    std::ifstream ifs(dict_path);
    std::string line;
    while (std::getline(ifs, line)) dict_.emplace_back(line);

    std::ifstream file(engine_path, std::ios::binary);
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0);
    std::vector<char> data(size);
    file.read(data.data(), size);

    runtime_ = createInferRuntime(gLogger);
    engine_ = runtime_->deserializeCudaEngine(data.data(), size);
    context_ = engine_->createExecutionContext();

    /* 申请显存 */
    buffers_.resize(2);  // 1 in 1 out
#if USE_TENSORRT_8
    for (int i = 0; i < 2; ++i) 
    {
        Dims dims = engine_->getBindingDimensions(i);
        size_t vol = 1;
        for (int j = 0; j < dims.nbDims; ++j)
        {
            if (dims.d[j] == -1) {
                // 对于动态batch维度，使用最大批量大小
                if (j == 0) {
                    dims.d[j] = max_batch_size_;
                } else {
                    // 其他维度使用默认值
                    dims.d[j] = (i == 0) ? input_w_ : predict_shape_default_[j-1];
                }
            }
            vol *= dims.d[j];
            
            if(i == 1)  // 输出层
            {
                predict_shape_.push_back(dims.d[j]);
            }
        }
        cudaMalloc(&buffers_[i], vol * sizeof(float));
    }
#else
    for (int i = 0; i < 2; ++i) 
    {
        const char* name = engine_->getIOTensorName(i);
        auto dims = engine_->getTensorShape(name);
        size_t vol = 1;
        for (int j = 0; j < dims.nbDims; ++j)
        {
            if (dims.d[j] == -1) 
            {
                // 对于动态维度，使用最大可能的大小
                if (j == 0) {
                    dims.d[j] = max_batch_size_;
                } else {
                    dims.d[j] = (i == 0) ? input_w_ : predict_shape_default_[j-1];
                }
            }
            vol *= dims.d[j];
            
            if(i == 1)
            {
               predict_shape_.push_back(dims.d[j]); 
            }
        }
        
        cudaMalloc(&buffers_[i], vol * sizeof(float));
    }
#endif
    cudaStreamCreate(&stream_);
}

void OcrRec::preprocess(const cv::Mat& img, float* gpu_buf) 
{
    const int dst_h = input_h_;
    const int dst_w = input_w_;

    /* 计算缩放因子（保持比例） */
    float scale = std::min(static_cast<float>(dst_h) / img.rows,
                           static_cast<float>(dst_w) / img.cols);
    int new_h = static_cast<int>(img.rows * scale);
    int new_w = static_cast<int>(img.cols * scale);

    /* resize */
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h));

    /* 填充 */
    cv::Mat canvas = cv::Mat(dst_h, dst_w, CV_8UC3, cv::Scalar(127, 127, 127));
    // int top = (dst_h - new_h) / 2;
    // int left = 0; // 右侧 pad，左侧对齐
    // resized.copyTo(canvas(cv::Rect(left, top, new_w, new_h)));

    cv::copyMakeBorder(resized, canvas, 0, 0, 0,
                     int(dst_w - resized.cols), cv::BORDER_CONSTANT,
                     {127, 127, 127});

    /* 归一化 */
    cv::cvtColor(canvas, canvas, cv::COLOR_BGR2RGB);
    canvas.convertTo(canvas, CV_32F, 1.0 / 255.0);

    /* HWC -> CHW */
    std::vector<float> host(dst_h * dst_w * 3);
    int idx = 0;
    for (int c = 0; c < 3; ++c)
        for (int h = 0; h < dst_h; ++h)
            for (int w = 0; w < dst_w; ++w)
                host[c * dst_h * dst_w + h * dst_w + w] = canvas.at<cv::Vec3f>(h, w)[c];

    cudaMemcpyAsync(gpu_buf, host.data(), host.size() * sizeof(float),
                    cudaMemcpyHostToDevice, stream_);
}

void OcrRec::preprocess(const cv::Mat& img, float* gpu_buf, int batch_offset) 
{
    const int dst_h = input_h_;
    const int dst_w = input_w_;
    const int channels = 3;
    const int single_img_size = dst_h * dst_w * channels;

    /* 计算缩放因子（保持比例） */
    float scale = std::min(static_cast<float>(dst_h) / img.rows,
                           static_cast<float>(dst_w) / img.cols);
    int new_h = static_cast<int>(img.rows * scale);
    int new_w = static_cast<int>(img.cols * scale);

    /* resize */
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h));

    /* 填充 */
    cv::Mat canvas = cv::Mat(dst_h, dst_w, CV_8UC3, cv::Scalar(127, 127, 127));
    cv::copyMakeBorder(resized, canvas, 0, 0, 0,
                     int(dst_w - resized.cols), cv::BORDER_CONSTANT,
                     {127, 127, 127});

    /* 归一化 */
    cv::cvtColor(canvas, canvas, cv::COLOR_BGR2RGB);
    canvas.convertTo(canvas, CV_32F, 1.0 / 255.0);

    /* HWC -> CHW */
    std::vector<float> host(single_img_size);
    int idx = 0;
    for (int c = 0; c < 3; ++c)
        for (int h = 0; h < dst_h; ++h)
            for (int w = 0; w < dst_w; ++w)
                host[c * dst_h * dst_w + h * dst_w + w] = canvas.at<cv::Vec3f>(h, w)[c];

    // 复制到GPU缓冲区的指定位置
    cudaMemcpyAsync(gpu_buf + batch_offset * single_img_size, host.data(), 
                    single_img_size * sizeof(float),
                    cudaMemcpyHostToDevice, stream_);
}

void OcrRec::batch_preprocess(const std::vector<cv::Mat>& imgs, float* gpu_buf) 
{
    const int dst_h = input_h_;
    const int dst_w = input_w_;
    const int channels = 3;
    const int single_img_size = dst_h * dst_w * channels;
    
    // 先在主机内存中准备所有数据
    std::vector<float> host_data(imgs.size() * single_img_size);
//  #pragma omp parallel for   
    for (int i = 0; i < imgs.size(); i++) 
    {
        const cv::Mat& img = imgs[i];
        
        /* 计算缩放因子（保持比例） */
        float scale = std::min(static_cast<float>(dst_h) / img.rows,
                               static_cast<float>(dst_w) / img.cols);
        int new_h = static_cast<int>(img.rows * scale);
        int new_w = static_cast<int>(img.cols * scale);

        /* resize */
        cv::Mat resized;
        cv::resize(img, resized, cv::Size(new_w, new_h));

        /* 填充 */
        cv::Mat canvas = cv::Mat(dst_h, dst_w, CV_8UC3, cv::Scalar(127, 127, 127));
        cv::copyMakeBorder(resized, canvas, 0, 0, 0,
                         int(dst_w - resized.cols), cv::BORDER_CONSTANT,
                         {127, 127, 127});

        /* 归一化 */
        cv::cvtColor(canvas, canvas, cv::COLOR_BGR2RGB);
        canvas.convertTo(canvas, CV_32F, 1.0 / 255.0);
        
        /* HWC -> CHW */
        float* img_data = host_data.data() + i * single_img_size;
#if 1
        std::vector<cv::Mat> channelImgs;
        cv::split(canvas, channelImgs);
        for (int c = 0; c < channels; ++c) 
        {
            // 直接复制整个通道数据
            std::memcpy(img_data + c * dst_h * dst_w, channelImgs[c].data, input_h_ * input_w_ * sizeof(float));
        }
#else
        for (int c = 0; c < 3; ++c)
            for (int h = 0; h < dst_h; ++h)
                for (int w = 0; w < dst_w; ++w)
                    img_data[c * dst_h * dst_w + h * dst_w + w] = canvas.at<cv::Vec3f>(h, w)[c];
#endif
    }
    
    // 一次性拷贝所有数据到GPU
    cudaMemcpyAsync(gpu_buf, host_data.data(), host_data.size() * sizeof(float),
                    cudaMemcpyHostToDevice, stream_);
}

template <class ForwardIterator>
inline static size_t argmax(ForwardIterator first, ForwardIterator last)
{
    return std::distance(first, std::max_element(first, last));
}

void OcrRec::postprocess(float* logits, int seq_len, int dict_size, Recognition& res) 
{
    std::string text;
    int argmax_idx;
    int last_index = 0;
    float score = 0.0f;
    int count = 0;
    float max_value = 0.0f;
    
    for (int n = 0; n < seq_len; n++) 
    {
        argmax_idx =
            int(argmax(&logits[(n) * dict_size], &logits[(n + 1) * dict_size]));
        max_value =
            float(*std::max_element(&logits[(n) * dict_size], &logits[(n + 1) * dict_size]));

        if (argmax_idx > 0 && (!(n > 0 && argmax_idx == last_index))) 
        {
            score += max_value;
            count += 1;
            text+=dict_[argmax_idx - 1];
        }
        last_index = argmax_idx;
    }
    
    if (count > 0) {
        score /= count;
    }
    res.text = text;
    res.score = score;
}

void OcrRec::batch_postprocess(float* batch_logits, int batch_size, 
                              int seq_len, int dict_size, 
                              std::vector<Recognition>& results) 
{
    results.resize(batch_size);
    for (int i = 0; i < batch_size; i++) 
    {
        float* logits = batch_logits + i * seq_len * dict_size;
        postprocess(logits, seq_len, dict_size, results[i]);
    }
}

Recognition OcrRec::infer(const cv::Mat& img) 
{
    bool bTestTimes = false; 
    if(bTestTimes)
    {
        auto start = std::chrono::high_resolution_clock::now();
        int nCount = 100;      
        for(int i =0; i< nCount; i++)
        {
            preprocess(img, static_cast<float*>(buffers_[0]));
        }   
        auto end = std::chrono::high_resolution_clock::now();
        double ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
        std::cout << "rec_pre_time: " << ave_time << "ms" << std::endl;
        
        start = std::chrono::high_resolution_clock::now();
        for(int i =0; i< nCount; i++)
        {
#if USE_TENSORRT_8
            context_->enqueueV2(buffers_.data(), stream_, nullptr);   
#else
            context_->setInputTensorAddress(engine_->getIOTensorName(0), buffers_[0]);
            context_->setOutputTensorAddress(engine_->getIOTensorName(1), buffers_[1]);
            context_->enqueueV3(stream_);
#endif
        }
        end = std::chrono::high_resolution_clock::now();
        ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
        std::cout << "rec_infer_time: " << ave_time << "ms" << std::endl;

        start = std::chrono::high_resolution_clock::now();
        Recognition res;   
        for(int i =0; i< nCount; i++)
        {
            std::vector<float> logits(predict_shape_[1] * predict_shape_[2]);
            cudaMemcpyAsync(logits.data(), buffers_[1], logits.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_);
            cudaStreamSynchronize(stream_);
            postprocess(logits.data(), predict_shape_[1], predict_shape_[2], res);
        }
        end = std::chrono::high_resolution_clock::now();
        ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
        std::cout << "rec_post_time: " << ave_time << "ms" << std::endl;
        return res;
    }
    else
    {
        preprocess(img, static_cast<float*>(buffers_[0]));
#if USE_TENSORRT_8
        context_->enqueueV2(buffers_.data(), stream_, nullptr);   
#else
        context_->setInputTensorAddress(engine_->getIOTensorName(0), buffers_[0]);
        context_->setOutputTensorAddress(engine_->getIOTensorName(1), buffers_[1]);
        context_->enqueueV3(stream_);
#endif

        std::vector<float> logits(predict_shape_[1] * predict_shape_[2]);
        cudaMemcpyAsync(logits.data(), buffers_[1], logits.size() * sizeof(float),
                        cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);
        Recognition res;
        postprocess(logits.data(), predict_shape_[1], predict_shape_[2], res);
        return res;
    }

}

std::vector<Recognition> OcrRec::infer_batch(const std::vector<cv::Mat>& imgs) 
{
    if (imgs.empty()) {
        return {};
    }
    bool bTestTimes = false; 
    if(bTestTimes)
    {
        int batch_size = imgs.size();
        for(int i = 0; i < 2; i++ )
        {
            const char* name = engine_->getIOTensorName(i);
            auto dims = engine_->getTensorShape(name);
            for (int j = 0; j < dims.nbDims; ++j)
            {
                if (dims.d[j] == -1) 
                {
                    dims.d[j] = batch_size;
                }
            }
            if(i == 0)
            {
                context_->setInputShape(name, dims);
            }
        }
        auto start = std::chrono::high_resolution_clock::now();
        int nCount = 100;      
        for(int i =0; i< nCount; i++)
        {
            // 批量预处理
            batch_preprocess(imgs, static_cast<float*>(buffers_[0]));
        }   
        auto end = std::chrono::high_resolution_clock::now();
        double ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
        std::cout << "rec_pre_time: " << ave_time << "ms" << std::endl;

        cudaStreamSynchronize(stream_);  // 等待预处理完成
        
        start = std::chrono::high_resolution_clock::now();
        for(int i =0; i< nCount; i++)
        {
            // 执行推理
#if USE_TENSORRT_8
            context_->enqueueV2(buffers_.data(), stream_, nullptr);   
#else
            context_->setInputTensorAddress(engine_->getIOTensorName(0), buffers_[0]);
            context_->setOutputTensorAddress(engine_->getIOTensorName(1), buffers_[1]);
            context_->enqueueV3(stream_);
#endif
        }
        end = std::chrono::high_resolution_clock::now();
        ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
        std::cout << "rec_infer_time: " << ave_time << "ms" << std::endl;

        start = std::chrono::high_resolution_clock::now();
        std::vector<Recognition> results;   
        for(int i =0; i< nCount; i++)
        {
            cudaStreamSynchronize(stream_);
            // 获取输出
            int seq_len = predict_shape_[1];
            int dict_size = predict_shape_[2];
            int output_size = batch_size * seq_len * dict_size;
            
            std::vector<float> batch_logits(output_size);
            cudaMemcpyAsync(batch_logits.data(), buffers_[1], output_size * sizeof(float),
                            cudaMemcpyDeviceToHost, stream_);
            cudaStreamSynchronize(stream_);
            
            // 批量后处理
            
            batch_postprocess(batch_logits.data(), batch_size, seq_len, dict_size, results);
            
        }   
        end = std::chrono::high_resolution_clock::now();
        ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
        std::cout << "rec_post_time: " << ave_time << "ms" << std::endl;
        
        return results;
    }
    else
    {
        int batch_size = imgs.size();
        for(int i = 0; i < 2; i++ )
        {
            const char* name = engine_->getIOTensorName(i);
            auto dims = engine_->getTensorShape(name);
            for (int j = 0; j < dims.nbDims; ++j)
            {
                if (dims.d[j] == -1) 
                {
                    dims.d[j] = batch_size;
                }
            }
            if(i == 0)
            {
                context_->setInputShape(name, dims);
            }
        }
        
        // 批量预处理
        batch_preprocess(imgs, static_cast<float*>(buffers_[0]));
        cudaStreamSynchronize(stream_);  // 等待预处理完成
        
        // 执行推理
#if USE_TENSORRT_8
        context_->enqueueV2(buffers_.data(), stream_, nullptr);   
#else
        context_->setInputTensorAddress(engine_->getIOTensorName(0), buffers_[0]);
        context_->setOutputTensorAddress(engine_->getIOTensorName(1), buffers_[1]);
        context_->enqueueV3(stream_);
#endif

        cudaStreamSynchronize(stream_);
        // 获取输出
        int seq_len = predict_shape_[1];
        int dict_size = predict_shape_[2];
        int output_size = batch_size * seq_len * dict_size;
        
        std::vector<float> batch_logits(output_size);
        cudaMemcpyAsync(batch_logits.data(), buffers_[1], output_size * sizeof(float),
                        cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);
        
        // 批量后处理
        std::vector<Recognition> results;
        batch_postprocess(batch_logits.data(), batch_size, seq_len, dict_size, results);
        
        return results;
    }
    
}