#include "process.h"
#include <device_launch_parameters.h>
#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/partition.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// SCRFD预处理kernel
// ============================================================

__global__ void scrfd_preprocess_kernel(
    const uchar3* __restrict__ src,   // HWC输入 (BGR)
    int src_h, int src_w,
    float* out,                       // CHW输出 (RGB)
    int dst_h, int dst_w,
    float scale,                      // resize缩放
    int pad_top, int pad_left,        // 左上角填充
    float mean,                       // 127.5f
    float norm_scale)                // 1/128.f
{
    int dx = blockIdx.x * blockDim.x + threadIdx.x;
    int dy = blockIdx.y * blockDim.y + threadIdx.y;
    if (dx >= dst_w || dy >= dst_h) return;

    // 1. 去掉填充，映射到原图坐标
    int sx = (int)rintf((dx - pad_left) / scale);
    int sy = (int)rintf((dy - pad_top) / scale);

    // 2. 越界用常量0填充
    uchar3 pix;
    if (sx < 0 || sy < 0 || sx >= src_w || sy >= src_h)
        pix = make_uchar3(0, 0, 0);
    else
        pix = src[sy * src_w + sx];

    // 3. 归一化: (pixel - 127.5) / 128 = pixel * 0.0078125 - 0.99609
    // BGR -> RGB
    float r = (pix.z - mean) * norm_scale;
    float g = (pix.y - mean) * norm_scale;
    float b = (pix.x - mean) * norm_scale;

    // 4. 写到 CHW 格式
    int area = dst_h * dst_w;
    out[0 * area + dy * dst_w + dx] = r;
    out[1 * area + dy * dst_w + dx] = g;
    out[2 * area + dy * dst_w + dx] = b;
}

void launch_scrfd_preprocess_kernel(const uchar3* d_src, int src_h, int src_w,
                                     float* d_dst, int dst_h, int dst_w,
                                     float scale, int pad_top, int pad_left,
                                     float mean, float norm_scale,
                                     cudaStream_t stream)
{
    dim3 block(16, 16);
    dim3 grid((dst_w + block.x - 1) / block.x,
              (dst_h + block.y - 1) / block.y);
    scrfd_preprocess_kernel<<<grid, block, 0, stream>>>(
        d_src, src_h, src_w, d_dst, dst_h, dst_w,
        scale, pad_top, pad_left, mean, norm_scale);
}

// ============================================================
// SCRFD后处理kernel - 处理单个stride
// ============================================================

__global__ void scrfd_postprocess_kernel(
    const float* __restrict__ d_scores,
    const float* __restrict__ d_bboxes,
    int feat_w, int feat_h,
    int num_anchors, int feat_stride,
    float base_size, float conf_thresh,
    float ratio, float pad_w, float pad_h,
    Bbox32* d_out, int* d_count)
{
    int anchor_idx = blockIdx.x * blockDim.x + threadIdx.x;
    int pos_idx = blockIdx.y * blockDim.y + threadIdx.y;

    if (anchor_idx >= num_anchors || pos_idx >= feat_h * feat_w) return;

    int i = pos_idx / feat_w;
    int j = pos_idx % feat_w;

    int idx = pos_idx * num_anchors + anchor_idx;
    float score = d_scores[idx];

    // 置信度过滤
    if (score < conf_thresh) return;

    // 计算锚点参数
    float scale = (anchor_idx == 0) ? 1.0f : 2.0f;
    float rs_w = base_size * scale;
    float rs_h = base_size * scale;

    // 锚点模板，中心在 (0,0)
    float anchor_x1 = -rs_w * 0.5f;
    float anchor_y1 = -rs_h * 0.5f;
    float anchor_x2 = rs_w * 0.5f;
    float anchor_y2 = rs_h * 0.5f;

    // shifted anchor 的位置
    float anchor_x = anchor_x1 + j * feat_stride;
    float anchor_y = anchor_y1 + i * feat_stride;

    // anchor 宽高
    float anchor_w = anchor_x2 - anchor_x1;
    float anchor_h = anchor_y2 - anchor_y1;

    // bbox 偏移: [dx, dy, dw, dh]
    int bbox_base_idx = pos_idx * num_anchors * 4 + anchor_idx * 4;
    float dx = d_bboxes[bbox_base_idx + 0] * feat_stride;
    float dy = d_bboxes[bbox_base_idx + 1] * feat_stride;
    float dw = d_bboxes[bbox_base_idx + 2] * feat_stride;
    float dh = d_bboxes[bbox_base_idx + 3] * feat_stride;

    // distance2bbox
    float cx = anchor_x + anchor_w * 0.5f;
    float cy = anchor_y + anchor_h * 0.5f;

    float x0 = cx - dx;
    float y0 = cy - dy;
    float x1 = cx + dw;
    float y1 = cy + dh;

    // 反算到原图坐标
    x0 = (x0 - pad_w) / ratio;
    y0 = (y0 - pad_h) / ratio;
    x1 = (x1 - pad_w) / ratio;
    y1 = (y1 - pad_h) / ratio;

    // 写入输出 (原子操作)
    int out_idx = atomicAdd(d_count, 1);
    Bbox32& box = d_out[out_idx];
    box.x1 = x0;
    box.y1 = y0;
    box.x2 = x1;
    box.y2 = y1;
    box.score = score;
    box.reserved = 0;
}

void launch_scrfd_postprocess_kernel(const float* d_scores,
                                       const float* d_bboxes,
                                       int feat_w, int feat_h,
                                       int num_anchors, int feat_stride,
                                       float base_size, float conf_thresh,
                                       float ratio, float pad_w, float pad_h,
                                       Bbox32* d_out, int* d_count,
                                       cudaStream_t stream)
{
    dim3 block(8, 8);
    dim3 grid((num_anchors + block.x - 1) / block.x,
              ((feat_h * feat_w) + block.y - 1) / block.y);
    scrfd_postprocess_kernel<<<grid, block, 0, stream>>>(
        d_scores, d_bboxes, feat_w, feat_h, num_anchors, feat_stride,
        base_size, conf_thresh, ratio, pad_w, pad_h, d_out, d_count);
}

// ============================================================
// NMS kernel - 计算IoU矩阵
// ============================================================

__global__ void nms_matrix_kernel(
    const Bbox32* __restrict__ boxes,
    int n,
    float iou_thresh,
    uint8_t* mask)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    if (i >= n || j >= n || j <= i) return;

    const Bbox32& a = boxes[i];
    const Bbox32& b = boxes[j];

    // 计算IoU
    float xx1 = fmaxf(a.x1, b.x1);
    float yy1 = fmaxf(a.y1, b.y1);
    float xx2 = fminf(a.x2, b.x2);
    float yy2 = fminf(a.y2, b.y2);

    float w = fmaxf(0.0f, xx2 - xx1);
    float h = fmaxf(0.0f, yy2 - yy1);
    float inter = w * h;

    float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    float union_area = area_a + area_b - inter + 1e-6f;

    float iou = inter / union_area;

    mask[i * n + j] = (iou > iou_thresh) ? 1 : 0;
}

void launch_nms_matrix_kernel(const Bbox32* boxes,
                               int n, float iou_thresh,
                               uint8_t* mask,
                               cudaStream_t stream)
{
    dim3 block(16, 16);
    dim3 grid((n + block.x - 1) / block.x,
              (n + block.y - 1) / block.y);
    nms_matrix_kernel<<<grid, block, 0, stream>>>(boxes, n, iou_thresh, mask);
}

#ifdef __cplusplus
}
#endif
