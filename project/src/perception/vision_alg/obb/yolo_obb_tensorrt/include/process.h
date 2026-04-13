#pragma once
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include <vector>

// OBB检测结构（包含角度信息）
struct OBBBox {
    float cx, cy, w, h;   // 中心点坐标和宽高
    float angle;           // 旋转角度（弧度）
    float conf;            // 置信度
    int cls;               // 类别
};

#ifdef __cplusplus
extern "C" {
#endif

void launch_preprocess_kernel(const unsigned char* d_src, int src_h, int src_w,
                              float* d_dst, int dst_h, int dst_w,
                              float scale, int pad_top, int pad_left, float norm,
                              cudaStream_t stream);

void yolo_obb_postprocess_gpu(
    float* d_trt_out,        // TRT输出
    int num_boxes,           // 候选框数量
    int num_classes,         // 类别数量
    float conf_thresh,       // 置信度阈值
    float iou_thresh,        // IOU阈值
    float ratio,             // 缩放比例
    float dw,                // 宽度填充
    float dh,                // 高度填充
    OBBBox* d_out,           // 输出框(设备)
    int* d_keep_num,         // 保留数量(设备)
    std::vector<OBBBox>& box_out,  // 输出框(主机)
    cudaStream_t stream);

#ifdef __cplusplus
}
#endif
