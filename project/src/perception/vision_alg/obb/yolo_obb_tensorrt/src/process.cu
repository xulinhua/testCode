#include "process.h"
#include <cstring>
#include <iostream>

// ------------------ Preprocess --------------------
__global__ void preprocess_kernel(const unsigned char* src, int src_h, int src_w,
                                  float* dst, int dst_h, int dst_w,
                                  float scale, int pad_top, int pad_left, float norm)
{
    int dst_x = blockDim.x * blockIdx.x + threadIdx.x;
    int dst_y = blockDim.y * blockIdx.y + threadIdx.y;
    
    if (dst_x >= dst_w || dst_y >= dst_h) return;
    
    int src_x = (int)(dst_x - pad_left) / scale;
    int src_y = (int)(dst_y - pad_top) / scale;
    
    // 边界检查
    if (src_x < 0 || src_x >= src_w || src_y < 0 || src_y >= src_h) {
        // Padding区域
        dst[dst_y * dst_w + dst_x] = 114.0f * norm;                    // R
        dst[(dst_h + dst_y) * dst_w + dst_x] = 114.0f * norm;         // G
        dst[(2 * dst_h + dst_y) * dst_w + dst_x] = 114.0f * norm;     // B
        return;
    }
    
    // 双线性插值
    float fx = (dst_x - pad_left) / scale - src_x;
    float fy = (dst_y - pad_top) / scale - src_y;
    
    int x0 = max(0, min(src_w - 1, (int)src_x));
    int x1 = max(0, min(src_w - 1, (int)src_x + 1));
    int y0 = max(0, min(src_h - 1, (int)src_y));
    int y1 = max(0, min(src_h - 1, (int)src_y + 1));
    
    // OpenCV Mat is BGR: bgr[0]=B, bgr[1]=G, bgr[2]=R
    // Access as (y * src_w + x) * 3
    float b0 = src[(y0 * src_w + x0) * 3 + 0];
    float g0 = src[(y0 * src_w + x0) * 3 + 1];
    float r0 = src[(y0 * src_w + x0) * 3 + 2];
    
    float b1 = src[(y0 * src_w + x1) * 3 + 0];
    float g1 = src[(y0 * src_w + x1) * 3 + 1];
    float r1 = src[(y0 * src_w + x1) * 3 + 2];
    
    float b2 = src[(y1 * src_w + x0) * 3 + 0];
    float g2 = src[(y1 * src_w + x0) * 3 + 1];
    float r2 = src[(y1 * src_w + x0) * 3 + 2];
    
    float b3 = src[(y1 * src_w + x1) * 3 + 0];
    float g3 = src[(y1 * src_w + x1) * 3 + 1];
    float r3 = src[(y1 * src_w + x1) * 3 + 2];
    
    // 双线性插值
    float b = (1 - fx) * (1 - fy) * b0 + fx * (1 - fy) * b1 +
              (1 - fx) * fy * b2 + fx * fy * b3;
    float g = (1 - fx) * (1 - fy) * g0 + fx * (1 - fy) * g1 +
              (1 - fx) * fy * g2 + fx * fy * g3;
    float r = (1 - fx) * (1 - fy) * r0 + fx * (1 - fy) * r1 +
              (1 - fx) * fy * r2 + fx * fy * r3;
    
    // Output: CHW format, RGB order (model expects RGB)
    dst[dst_y * dst_w + dst_x] = r * norm;                    // R
    dst[(dst_h + dst_y) * dst_w + dst_x] = g * norm;         // G
    dst[(2 * dst_h + dst_y) * dst_w + dst_x] = b * norm;     // B
}

void launch_preprocess_kernel(const unsigned char* d_src, int src_h, int src_w,
                              float* d_dst, int dst_h, int dst_w,
                              float scale, int pad_top, int pad_left, float norm,
                              cudaStream_t stream)
{
    dim3 block(32, 32);
    dim3 grid((dst_w + block.x - 1) / block.x, (dst_h + block.y - 1) / block.y);
    preprocess_kernel<<<grid, block, 0, stream>>>(
        d_src, src_h, src_w, d_dst, dst_h, dst_w,
        scale, pad_top, pad_left, norm
    );
}

// ------------------ Postprocess --------------------
// OBB检测的decode kernel
// 输出格式: [cx, cy, w, h, class_scores..., angle] x num_boxes
__global__ void decode_obb_kernel(float* d_trt_out, int num_boxes, int num_classes,
                                   float conf_thresh, OBBBox* d_out, int* d_keep_num)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    if (idx >= num_boxes) return;
    
    // YOLO OBB output is organized by channels: [num_attrs, num_boxes]
    // num_attrs = 5 (cx,cy,w,h,angle) + num_classes
    // Access data: d_trt_out[channel * num_boxes + idx]
    
    // Get max confidence and class (skip first 4 bbox coords, start at index 4)
    float max_conf = 0.0f;
    int best_cls = 0;
    for (int c = 0; c < num_classes; c++) {
        float conf = d_trt_out[(4 + c) * num_boxes + idx];
        if (conf > max_conf) {
            max_conf = conf;
            best_cls = c;
        }
    }
    
    if (max_conf < conf_thresh) return;
    
    int out_idx = atomicAdd(d_keep_num, 1);
    
    // 边界检查
    if (out_idx >= num_boxes) {
        atomicSub(d_keep_num, 1);
        return;
    }
    
    // Get bbox coordinates: cx, cy, w, h
    d_out[out_idx].cx = d_trt_out[0 * num_boxes + idx];
    d_out[out_idx].cy = d_trt_out[1 * num_boxes + idx];
    d_out[out_idx].w = d_trt_out[2 * num_boxes + idx];
    d_out[out_idx].h = d_trt_out[3 * num_boxes + idx];
    
    // Angle is at channel (4 + num_classes)
    d_out[out_idx].angle = d_trt_out[(4 + num_classes) * num_boxes + idx];
    
    d_out[out_idx].conf = max_conf;
    d_out[out_idx].cls = best_cls;
}

// OBB NMS kernel (简化版，使用轴对齐边界框进行粗筛选)
__global__ void nms_obb_kernel(OBBBox* d_boxes, int num_boxes, float iou_thresh, int* d_keep)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    if (idx >= num_boxes) return;
    
    const OBBBox& box = d_boxes[idx];
    if (box.conf <= 0.0f) return;
    
    for (int i = 0; i < num_boxes; i++) {
        if (i == idx) continue;
        if (d_boxes[i].cls != box.cls) continue;
        if (d_boxes[i].conf <= 0.0f) continue;
        
        // 计算轴对齐边界框的IoU（简化版）
        float x1_1 = box.cx - box.w / 2, y1_1 = box.cy - box.h / 2;
        float x2_1 = box.cx + box.w / 2, y2_1 = box.cy + box.h / 2;
        float x1_2 = d_boxes[i].cx - d_boxes[i].w / 2, y1_2 = d_boxes[i].cy - d_boxes[i].h / 2;
        float x2_2 = d_boxes[i].cx + d_boxes[i].w / 2, y2_2 = d_boxes[i].cy + d_boxes[i].h / 2;
        
        float inter_x1 = max(x1_1, x1_2), inter_y1 = max(y1_1, y1_2);
        float inter_x2 = min(x2_1, x2_2), inter_y2 = min(y2_1, y2_2);
        
        if (inter_x2 <= inter_x1 || inter_y2 <= inter_y1) continue;
        
        float inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1);
        float area1 = box.w * box.h;
        float area2 = d_boxes[i].w * d_boxes[i].h;
        float union_area = area1 + area2 - inter_area;
        
        if (inter_area / union_area > iou_thresh) {
            if (d_boxes[i].conf > box.conf) {
                d_keep[idx] = 0;  // 被抑制
                return;
            }
        }
    }
    d_keep[idx] = 1;  // 保留
}

void yolo_obb_postprocess_gpu(
    float* d_trt_out, int num_boxes, int num_classes,
    float conf_thresh, float iou_thresh,
    float ratio, float dw, float dh,
    OBBBox* d_out, int* d_keep_num,
    std::vector<OBBBox>& box_out,
    cudaStream_t stream)
{
    // Reset counter
    cudaError_t err = cudaMemsetAsync(d_keep_num, 0, sizeof(int), stream);
    if (err != cudaSuccess) return;
    
    // Decode
    dim3 block(256);
    dim3 grid((num_boxes + block.x - 1) / block.x);
    decode_obb_kernel<<<grid, block, 0, stream>>>(
        d_trt_out, num_boxes, num_classes, conf_thresh, d_out, d_keep_num
    );
    
    // Check kernel launch error
    err = cudaGetLastError();
    if (err != cudaSuccess) return;
    
    // Synchronize and get keep count
    err = cudaStreamSynchronize(stream);
    if (err != cudaSuccess) return;
    
    int keep_count = 0;
    err = cudaMemcpy(&keep_count, d_keep_num, sizeof(int), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) return;
    
    if (keep_count == 0) return;
    
    // 安全检查
    if (keep_count > num_boxes) return;
    
    // Allocate and compute keep flags
    int* d_keep = nullptr;
    err = cudaMalloc(&d_keep, keep_count * sizeof(int));
    if (err != cudaSuccess) return;
    
    nms_obb_kernel<<<grid, block, 0, stream>>>(d_out, keep_count, iou_thresh, d_keep);
    
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        cudaFree(d_keep);
        return;
    }
    
    // Copy results to host
    std::vector<OBBBox> host_boxes(keep_count);
    err = cudaMemcpy(host_boxes.data(), d_out, keep_count * sizeof(OBBBox), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        cudaFree(d_keep);
        return;
    }
    
    std::vector<int> host_keep(keep_count);
    err = cudaMemcpy(host_keep.data(), d_keep, keep_count * sizeof(int), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        cudaFree(d_keep);
        return;
    }
    
    cudaStreamSynchronize(stream);
    
    // 预分配内存
    box_out.reserve(keep_count);
    
    // Filter and scale boxes
    for (int i = 0; i < keep_count; i++) {
        if (host_keep[i]) {
            OBBBox box = host_boxes[i];
            // Scale back to original image
            if (ratio > 0) {
                box.cx = (box.cx - dw) / ratio;
                box.cy = (box.cy - dh) / ratio;
                box.w = box.w / ratio;
                box.h = box.h / ratio;
            }
            box_out.push_back(box);
        }
    }
    
    cudaFree(d_keep);
}
