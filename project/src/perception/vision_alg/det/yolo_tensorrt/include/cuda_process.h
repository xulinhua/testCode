#pragma once

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 边界框结构 (GPU 使用)
struct Bbox32 {
    float x;   // center_x
    float y;   // center_y
    float w;   // width
    float h;   // height
    float conf;
    int class_id;
};

// CUDA 预处理 kernel：HWC -> CHW + Letterbox + 归一化 + BGR->RGB
__global__ void preprocess_kernel(
    const uint8_t* __restrict__ src,  // HWC 输入 (BGR)
    int src_h, int src_w,
    float* out,                        // CHW 输出
    int dst_h, int dst_w,
    float scale,                       // resize 缩放
    int pad_top, int pad_left,         // 左上角填充
    float norm_scale                   // 1/255.0
);

// CUDA 后处理 kernel：过滤低置信度框 + 坐标反算
__global__ void filter_kernel(
    const float* __restrict__ src,     // YOLO 输出 [num_classes+5, num_boxes]
    int num_boxes,
    int num_classes,
    float conf_thresh,
    float ratio, float dw, float dh,    // Letterbox 参数
    Bbox32* out,                       // 输出候选框
    int* keep_num                      // 实际保留数量
);

// 启动预处理 kernel
void launch_preprocess_kernel(
    const uint8_t* d_src, int src_h, int src_w,
    float* d_dst, int dst_h, int dst_w,
    float scale, int pad_top, int pad_left, float norm,
    cudaStream_t stream
);

// 启动后处理过滤 kernel
void launch_filter_kernel(
    const float* d_src,
    int num_boxes, int num_classes,
    float conf_thresh,
    float ratio, float dw, float dh,
    Bbox32* d_boxes,
    int* d_keep_num,
    cudaStream_t stream
);

#ifdef __cplusplus
}
#endif
