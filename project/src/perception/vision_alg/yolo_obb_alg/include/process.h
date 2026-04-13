#pragma once
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/cudawarping.hpp>
#include <vector>

#define PROCESS_GPU 1

// 单框结构（支持OBB）
struct Bbox32 
{
    float x, y, w, h, angle, conf;  // 添加角度信息
    int   cls;
};

#ifdef __cplusplus
extern "C" {
#endif

void launch_preprocess_kernel(const uchar3* d_src, int src_h, int src_w,
                              float* d_dst, int dst_h, int dst_w,
                              float scale, int pad_top, int pad_left, float norm,
                              cudaStream_t stream);

void yolo_postprocess_gpu(
    float* d_trt_out,
    int num_boxes, int num_classes,
    float conf_thresh, float iou_thresh,
    float ratio, float dw, float dh,
    Bbox32* d_out, int* d_keep_num,
    std::vector<Bbox32>& box_out,
    cudaStream_t stream);

#ifdef __cplusplus
}
#endif