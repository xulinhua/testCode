#pragma once
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudawarping.hpp>
#include <vector>

#define PROCESS_GPU 1

// 单框结构 - 用于SCRFD后处理
struct Bbox32
{
    float x1, y1, x2, y2;
    float score;
    int   reserved;  // 保留字段，对齐内存
};

#ifdef __cplusplus
extern "C" {
#endif

// SCRFD预处理kernel
void launch_scrfd_preprocess_kernel(const uchar3* d_src, int src_h, int src_w,
                                     float* d_dst, int dst_h, int dst_w,
                                     float scale, int pad_top, int pad_left,
                                     float mean, float norm_scale,
                                     cudaStream_t stream);

// SCRFD后处理kernel - 处理单个stride
void launch_scrfd_postprocess_kernel(const float* __restrict__ d_scores,
                                       const float* __restrict__ d_bboxes,
                                       int feat_w, int feat_h,
                                       int num_anchors, int feat_stride,
                                       float base_size, float conf_thresh,
                                       float ratio, float pad_w, float pad_h,
                                       Bbox32* d_out, int* d_count,
                                       cudaStream_t stream);

// NMS kernel - 计算IoU矩阵
void launch_nms_matrix_kernel(const Bbox32* __restrict__ boxes,
                               int n, float iou_thresh,
                               uint8_t* mask,
                               cudaStream_t stream);

#ifdef __cplusplus
}
#endif