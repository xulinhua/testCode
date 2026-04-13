#include "process.h"
#include <device_launch_parameters.h>
#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/sort.h>
#include <thrust/partition.h>

#ifdef __cplusplus
extern "C" {
#endif

// 输出：float* out 已经排成 CHW
__global__ void preprocess_kernel(
    const uchar3* __restrict__ src,   // HWC 输入
    int src_h, int src_w,
    float* out,                       // CHW 输出
    int dst_h, int dst_w,
    float scale,                      // resize 缩放
    int pad_top, int pad_left,        // 左上角填充
    float norm_scale)                 // 1/255.f
{
    int dx = blockIdx.x * blockDim.x + threadIdx.x;
    int dy = blockIdx.y * blockDim.y + threadIdx.y;
    if (dx >= dst_w || dy >= dst_h) return;

    // 1. 去掉填充，映射到原图坐标
    int sx = (int)rintf((dx - pad_left) / scale);
    int sy = (int)rintf((dy - pad_top)  / scale);

    // 2. 越界用常量 114
    uchar3 pix;
    if (sx < 0 || sy < 0 || sx >= src_w || sy >= src_h)
        pix = make_uchar3(114, 114, 114);
    else
        pix = src[sy * src_w + sx];

    // 3. 归一化 + BGR→RGB
    float r = pix.z * norm_scale;
    float g = pix.y * norm_scale;
    float b = pix.x * norm_scale;

    // 4. 写到 CHW
    int area = dst_h * dst_w;
    out[0 * area + dy * dst_w + dx] = r;
    out[1 * area + dy * dst_w + dx] = g;
    out[2 * area + dy * dst_w + dx] = b;
}

void launch_preprocess_kernel(const uchar3* d_src, int src_h, int src_w,
                              float* d_dst, int dst_h, int dst_w,
                              float scale, int pad_top, int pad_left, float norm,
                              cudaStream_t stream)
{
    dim3 block(16, 16);
    dim3 grid((dst_w + block.x - 1) / block.x,
              (dst_h + block.y - 1) / block.y);
    preprocess_kernel<<<grid, block, 0, stream>>>(d_src, src_h, src_w, d_dst, dst_h, dst_w,
        scale, pad_top, pad_left, norm);
}


// 过滤 + 反算 kernel
__global__ void filter_kernel(
    float* __restrict__ src,     
    int num_boxes,
    int num_classes,
    float conf_thresh,
    float ratio, float dw, float dh,
    Bbox32* out,                       // 输出候选框
    int* keep_num)                     // 实际保留数量
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_boxes) return;

    int cls_start = 4;
    int cls_end = 4 + num_classes; 
    float* ptr = src;         
   
    float max_score = 0.0;          // 初始第 0 类
    int cls = 0;
    for (int c = cls_start; c < cls_end; ++c) 
    {
        float s = ptr[c*num_boxes+i];
        if (s > max_score) { max_score = s; cls = c; }
    }
    // 置信度阈值直接作用在「最大类别得分」
    if (max_score < conf_thresh) return;

    // 反算到原图
    int idx = atomicAdd(keep_num, 1);   // 线程安全计数
    float cx   = ptr[0*num_boxes+i];
    float cy   = ptr[1*num_boxes+i];
    float w    = ptr[2*num_boxes+i];
    float h    = ptr[3*num_boxes+i];
    Bbox32& b = out[idx];
    b.x   = (cx - dw) / ratio;
    b.y   = (cy - dh) / ratio;
    b.w   = w / ratio;
    b.h   = h / ratio;
    b.conf= max_score;
    b.cls = cls-4;
}

__global__ void nms_matrix_kernel(
    const Bbox32* __restrict__ boxes,
    int n,
    float iou_thresh,
    uint8_t* mask)          // ← 改成 uint8_t
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    if (i >= n || j >= n || j <= i) return;

    Bbox32 a = boxes[i];
    Bbox32 b = boxes[j];
    if (a.cls != b.cls) {
        mask[i * n + j] = 0;
        return;
    }

    float xx1 = fmaxf(a.x - a.w * 0.5f, b.x - b.w * 0.5f);
    float yy1 = fmaxf(a.y - a.h * 0.5f, b.y - b.h * 0.5f);
    float xx2 = fminf(a.x + a.w * 0.5f, b.x + b.w * 0.5f);
    float yy2 = fminf(a.y + a.h * 0.5f, b.y + b.h * 0.5f);
    float w = fmaxf(0.0f, xx2 - xx1);
    float h = fmaxf(0.0f, yy2 - yy1);
    float inter = w * h;
    float iou = inter / (a.w * a.h + b.w * b.h - inter + 1e-6f);

    mask[i * n + j] = (iou > iou_thresh) ? 1 : 0;   // 1 字节
}

struct CompareBbox {
    __host__ __device__
    bool operator()(const Bbox32& a, const Bbox32& b) const {
        return a.conf > b.conf;
    }
};

void yolo_postprocess_gpu(
    float* d_trt_out,
    int num_boxes, int num_classes,
    float conf_thresh, float iou_thresh,
    float ratio, float dw, float dh,
    Bbox32* d_out, int* d_keep_num,
    std::vector<Bbox32>& box_out,
    cudaStream_t stream)
{
    // 过滤 + 反算（不变）
    int threads = 256;
    int blocks  = (num_boxes + threads - 1) / threads;
    cudaMemsetAsync(d_keep_num, 0, sizeof(int), stream);
    filter_kernel<<<blocks, threads, 0, stream>>>(
        d_trt_out, num_boxes, num_classes, conf_thresh, ratio, dw, dh,
        d_out, d_keep_num);

    int h_keep_num;
    cudaMemcpyAsync(&h_keep_num, d_keep_num, sizeof(int),
                    cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);
    if (h_keep_num == 0) { box_out.clear(); return; }
    
    // NMS 之前排序
    if(0)
    {
        thrust::device_ptr<Bbox32> ptr(d_out);
        thrust::sort(thrust::device, d_out, d_out + h_keep_num, CompareBbox());
        // thrust::sort(thrust::cuda::par.on(stream), d_out, d_out + h_keep_num,CompareBbox());
    }
    else
    {
        // 先把框拷回 CPU（只搬 h_keep_num 个，<1 KB）
        std::vector<Bbox32> host_box(h_keep_num);
        cudaMemcpyAsync(host_box.data(), d_out,
                        h_keep_num * sizeof(Bbox32),
                        cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);

        // CPU 端降序排序
        std::sort(host_box.begin(), host_box.end(),
                [](const Bbox32& a, const Bbox32& b) {
                    return a.conf > b.conf;   // 降序
                });

        // 再拷回 GPU
        cudaMemcpyAsync(d_out, host_box.data(),
                        h_keep_num * sizeof(Bbox32),
                        cudaMemcpyHostToDevice, stream);
    }

    // 掩码矩阵（n×n，只存 uint8_t 节省空间）
    size_t matrix_bytes = h_keep_num * h_keep_num * sizeof(uint8_t);
    uint8_t* d_mask = nullptr;
    cudaMalloc(&d_mask, matrix_bytes);

    dim3 block(16, 16);
    dim3 grid((h_keep_num + block.x - 1) / block.x,
              (h_keep_num + block.y - 1) / block.y);
    nms_matrix_kernel<<<grid, block, 0, stream>>>(
        d_out, h_keep_num, iou_thresh, d_mask);

    // 解压掩码 → keep 索引（CPU 端，<n² Byte）
    uint8_t* h_mask = new uint8_t[h_keep_num * h_keep_num];
    cudaMemcpyAsync(h_mask, d_mask, matrix_bytes,
                    cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    std::vector<int> keep_host;
    keep_host.push_back(0);                 // 0 号必保留
    for (int i = 1; i < h_keep_num; ++i) {
        bool suppressed = false;
        for (int j = 0; j < i; ++j)        // 前面任一框抑制我
            if (h_mask[j * h_keep_num + i]!=0) { suppressed = true; break; }
        if (!suppressed) keep_host.push_back(i);
    }
    int final_num = keep_host.size();

    // std::vector<Bbox32> all_out;
    // all_out.resize(h_keep_num);
    // for (int k = 0; k < h_keep_num; ++k)
    //     cudaMemcpyAsync(&all_out[k], &d_out[k],
    //                     sizeof(Bbox32), cudaMemcpyDeviceToHost, stream);
    // cudaStreamSynchronize(stream);

    // 只拷 keep 框
    box_out.resize(final_num);
    for (int k = 0; k < final_num; ++k)
        cudaMemcpyAsync(&box_out[k], &d_out[keep_host[k]],
                        sizeof(Bbox32), cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    // 清理
    delete[] h_mask;
    cudaFree(d_mask);
}

#ifdef __cplusplus
}
#endif