#pragma once
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include <vector>

// 分割检测结构
struct Bbox32 
{
    float x1, y1, x2, y2;  // 边界框
    float conf;            // 置信度
    int   cls;             // 类别
    float mask[32];        // mask系数
};

#ifdef __cplusplus
extern "C" {
#endif

void launch_preprocess_kernel(const unsigned char* d_src, int src_h, int src_w,
                              float* d_dst, int dst_h, int dst_w,
                              float scale, int pad_top, int pad_left, float norm,
                              cudaStream_t stream);

void yolo_seg_postprocess_gpu(
    float* d_trt_out,        // TRT输出
    int num_boxes,           // 候选框数量
    int num_classes,         // 类别数量
    float conf_thresh,       // 置信度阈值
    float iou_thresh,        // IOU阈值
    float ratio,             // 缩放比例
    float dw,                // 宽度填充
    float dh,                // 高度填充
    Bbox32* d_out,           // 输出框(设备)
    int* d_keep_num,         // 保留数量(设备)
    std::vector<Bbox32>& box_out,  // 输出框(主机)
    cudaStream_t stream);

// GPU版本 mask处理
void launch_process_mask_kernel(
    const float* d_proto,         // proto数据 (32 x proto_h x proto_w)
    int proto_h, int proto_w,     // proto尺寸
    const float* d_mask_coef,     // mask系数 (num_segs x 32)
    const float* d_bboxes,        // bbox坐标 (num_segs x 4: x1,y1,x2,y2 in orig image)
    int num_segs,                 // 分割数量
    float ratio, float dw, float dh,  // letterbox参数
    int input_w, int input_h,     // 模型输入尺寸
    float* d_output_masks,        // 输出masks (num_segs x orig_h x orig_w)
    int orig_w, int orig_h,       // 原图尺寸
    float mask_threshold,         // mask阈值
    unsigned char* d_output_binary_masks,  // 二值化输出
    cudaStream_t stream);

#ifdef __cplusplus
}
#endif
