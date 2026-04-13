#include "process.h"
#include <cstring>

// ------------------ Preprocess --------------------
__global__ void preprocess_kernel(const uchar3* src, int src_h, int src_w,
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
    
    uchar3 v00 = src[y0 * src_w + x0];
    uchar3 v01 = src[y0 * src_w + x1];
    uchar3 v10 = src[y1 * src_w + x0];
    uchar3 v11 = src[y1 * src_w + x1];
    
    // OpenCV uchar3 is BGR: .x=B, .y=G, .z=R
    float b = (1 - fx) * (1 - fy) * v00.x + fx * (1 - fy) * v01.x +
              (1 - fx) * fy * v10.x + fx * fy * v11.x;
    float g = (1 - fx) * (1 - fy) * v00.y + fx * (1 - fy) * v01.y +
              (1 - fx) * fy * v10.y + fx * fy * v11.y;
    float r = (1 - fx) * (1 - fy) * v00.z + fx * (1 - fy) * v01.z +
              (1 - fx) * fy * v10.z + fx * fy * v11.z;
    
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
        (const uchar3*)d_src, src_h, src_w, d_dst, dst_h, dst_w,
        scale, pad_top, pad_left, norm
    );
}

// ------------------ Postprocess --------------------
__global__ void decode_kernel(float* d_trt_out, int num_boxes, int num_classes, int num_masks,
                              float conf_thresh, Bbox32* d_out, int* d_keep_num)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    if (idx >= num_boxes) return;
    
    // YOLO output is organized by channels: [116, 8400]
    // Access data: d_trt_out[channel * num_boxes + idx]
    
    // Get max confidence and class
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
    
    // 边界检查：确保不会超出d_out的分配大小
    if (out_idx >= num_boxes) {
        atomicSub(d_keep_num, 1);  // 回退计数
        return;
    }
    
    // Get bbox coordinates: cx, cy, w, h -> x1, y1, x2, y2
    float cx = d_trt_out[0 * num_boxes + idx];
    float cy = d_trt_out[1 * num_boxes + idx];
    float w  = d_trt_out[2 * num_boxes + idx];
    float h  = d_trt_out[3 * num_boxes + idx];
    
    d_out[out_idx].x1 = cx - w * 0.5f;
    d_out[out_idx].y1 = cy - h * 0.5f;
    d_out[out_idx].x2 = cx + w * 0.5f;
    d_out[out_idx].y2 = cy + h * 0.5f;
    d_out[out_idx].conf = max_conf;
    d_out[out_idx].cls = best_cls;
    
    // Copy mask coefficients
    for (int m = 0; m < num_masks; m++) {
        d_out[out_idx].mask[m] = d_trt_out[(4 + num_classes + m) * num_boxes + idx];
    }
}

__global__ void nms_kernel(Bbox32* d_boxes, int num_boxes, float iou_thresh, int* d_keep)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    if (idx >= num_boxes) return;
    
    const Bbox32& box = d_boxes[idx];
    if (box.conf <= 0.0f) return;
    
    for (int i = 0; i < num_boxes; i++) {
        if (i == idx) continue;
        if (d_boxes[i].cls != box.cls) continue;
        if (d_boxes[i].conf <= 0.0f) continue;
        
        // 计算IoU
        float x1 = max(box.x1, d_boxes[i].x1);
        float y1 = max(box.y1, d_boxes[i].y1);
        float x2 = min(box.x2, d_boxes[i].x2);
        float y2 = min(box.y2, d_boxes[i].y2);
        
        if (x2 <= x1 || y2 <= y1) continue;
        
        float inter_area = (x2 - x1) * (y2 - y1);
        float area1 = (box.x2 - box.x1) * (box.y2 - box.y1);
        float area2 = (d_boxes[i].x2 - d_boxes[i].x1) * (d_boxes[i].y2 - d_boxes[i].y1);
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

void yolo_seg_postprocess_gpu(
    float* d_trt_out, int num_boxes, int num_classes,
    float conf_thresh, float iou_thresh,
    float ratio, float dw, float dh,
    Bbox32* d_out, int* d_keep_num,
    std::vector<Bbox32>& box_out,
    cudaStream_t stream)
{
    const int num_masks = 32;
    
    // Reset counter
    cudaError_t err = cudaMemsetAsync(d_keep_num, 0, sizeof(int), stream);
    if (err != cudaSuccess) return;
    
    // Decode
    dim3 block(256);
    dim3 grid((num_boxes + block.x - 1) / block.x);
    decode_kernel<<<grid, block, 0, stream>>>(
        d_trt_out, num_boxes, num_classes, num_masks, conf_thresh, d_out, d_keep_num
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
    
    nms_kernel<<<grid, block, 0, stream>>>(d_out, keep_count, iou_thresh, d_keep);
    
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        cudaFree(d_keep);
        return;
    }
    
    // Copy results to host
    std::vector<Bbox32> host_boxes(keep_count);
    err = cudaMemcpy(host_boxes.data(), d_out, keep_count * sizeof(Bbox32), cudaMemcpyDeviceToHost);
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
    
    // 预分配内存，避免push_back时的内存重分配问题
    box_out.reserve(keep_count);
    
    // Filter and scale boxes
    for (int i = 0; i < keep_count; i++) {
        if (host_keep[i]) {
            Bbox32 box = host_boxes[i];
            // Scale back to original image
            if (ratio > 0) {
                box.x1 = (box.x1 - dw) / ratio;
                box.y1 = (box.y1 - dh) / ratio;
                box.x2 = (box.x2 - dw) / ratio;
                box.y2 = (box.y2 - dh) / ratio;
            }
            box_out.push_back(box);
        }
    }
    
    cudaFree(d_keep);
}

// ------------------ Process Mask (GPU) --------------------
// Fast sigmoid: approximates 1/(1+exp(-x)) with reasonable accuracy
__device__ __forceinline__ float fast_sigmoid_device(float x) {
    return 0.5f * (x / (1.0f + fabsf(x))) + 0.5f;
}

// Kernel: 使用1D grid + while循环处理bbox内的像素（完全参考yolo_seg_alg）
__global__ void process_mask_kernel(
    const float* __restrict__ d_proto,      // [32, proto_h, proto_w]
    int proto_h, int proto_w,
    const float* __restrict__ d_mask_coef,  // [num_segs, 32]
    const float* __restrict__ d_bboxes,     // [num_segs, 4] in original image coords
    int num_segs,
    float ratio, float dw, float dh,
    int input_w, int input_h,
    float* __restrict__ d_output_masks,     // [num_segs, orig_h, orig_w]
    int orig_w, int orig_h,
    int cut_left, int cut_top,
    int cut_w, int cut_h)
{
    // 1D grid: 每个block处理一个segment
    int seg_idx = blockIdx.x;
    if (seg_idx >= num_segs) return;

    // 获取bbox信息（原图坐标）
    float bx1 = d_bboxes[seg_idx * 4 + 0];
    float by1 = d_bboxes[seg_idx * 4 + 1];
    float bx2 = d_bboxes[seg_idx * 4 + 2];
    float by2 = d_bboxes[seg_idx * 4 + 3];

    // Clamp bbox到图像边界
    int img_x1 = max(0, min(orig_w - 1, (int)floorf(bx1)));
    int img_y1 = max(0, min(orig_h - 1, (int)floorf(by1)));
    int img_x2 = max(0, min(orig_w - 1, (int)ceilf(bx2)));
    int img_y2 = max(0, min(orig_h - 1, (int)ceilf(by2)));

    int bbox_w = img_x2 - img_x1 + 1;
    int bbox_h = img_y2 - img_y1 + 1;
    int bbox_pixels = bbox_w * bbox_h;

    if (bbox_w <= 0 || bbox_h <= 0) return;

    // 计算bbox在proto上的范围
    float scale_x = (float)proto_w / input_w;
    float scale_y = (float)proto_h / input_h;
    int px1 = max(0, min(proto_w - 1, (int)floorf((bx1 * ratio + dw) * scale_x)));
    int py1 = max(0, min(proto_h - 1, (int)floorf((by1 * ratio + dh) * scale_y)));
    int px2 = max(0, min(proto_w - 1, (int)ceilf((bx2 * ratio + dw) * scale_x)));
    int py2 = max(0, min(proto_h - 1, (int)ceilf((by2 * ratio + dh) * scale_y)));

    // 获取mask系数指针
    const float* mask_coef_ptr = d_mask_coef + seg_idx * 32;

    // 每个线程处理bbox内的一个或多个像素
    int tid = threadIdx.x;
    while (tid < bbox_pixels) {
        // 计算在bbox内的相对坐标
        int local_x = tid % bbox_w;
        int local_y = tid / bbox_w;

        // 计算在原图中的绝对位置
        int img_x = img_x1 + local_x;
        int img_y = img_y1 + local_y;

        // 映射到proto坐标（参考yolo_seg_alg的方式）
        float src_h = (float)img_y / orig_h * cut_h + cut_top;
        float src_w = (float)img_x / orig_w * cut_w + cut_left;

        // 双线性插值坐标
        int h0 = (int)src_h;
        int w0 = (int)src_w;
        int h1 = min(h0 + 1, proto_h - 1);
        int w1 = min(w0 + 1, proto_w - 1);
        float alpha_h = src_h - h0;
        float alpha_w = src_w - w0;

        // 检查是否在proto的bbox范围内
        bool in_proto_bbox = (w0 >= px1 && w0 <= px2 && h0 >= py1 && h0 <= py2);

        float val = 0.0f;
        if (in_proto_bbox) {
            // 计算mask值：矩阵乘法 mask_coef x proto
            float v00 = 0.0f, v01 = 0.0f, v10 = 0.0f, v11 = 0.0f;

            // 展开循环，处理32个通道
            #pragma unroll 8
            for (int c = 0; c < 32; c++) {
                float coef = mask_coef_ptr[c];
                v00 += coef * d_proto[c * proto_h * proto_w + h0 * proto_w + w0];
                v01 += coef * d_proto[c * proto_h * proto_w + h0 * proto_w + w1];
                v10 += coef * d_proto[c * proto_h * proto_w + h1 * proto_w + w0];
                v11 += coef * d_proto[c * proto_h * proto_w + h1 * proto_w + w1];
            }

            // 双线性插值
            val = (1.0f - alpha_h) * (1.0f - alpha_w) * v00 +
                  (1.0f - alpha_h) * alpha_w * v01 +
                  alpha_h * (1.0f - alpha_w) * v10 +
                  alpha_h * alpha_w * v11;

            // Sigmoid
            val = fast_sigmoid_device(val);
        }

        d_output_masks[seg_idx * orig_h * orig_w + img_y * orig_w + img_x] = val;

        tid += blockDim.x;
    }
}

void launch_process_mask_kernel(
    const float* d_proto,
    int proto_h, int proto_w,
    const float* d_mask_coef,
    const float* d_bboxes,
    int num_segs,
    float ratio, float dw, float dh,
    int input_w, int input_h,
    float* d_output_masks,
    int orig_w, int orig_h,
    float mask_threshold,
    unsigned char* d_output_binary_masks,
    cudaStream_t stream)
{
    // 计算padding区域在proto中的像素数（参考yolo_seg_alg）
    float scale_x = (float)proto_w / input_w;
    float scale_y = (float)proto_h / input_h;
    int cut_left = (int)(dw * scale_x);
    int cut_top = (int)(dh * scale_y);
    int cut_w = proto_w - 2 * cut_left;
    int cut_h = proto_h - 2 * cut_top;

    // 使用1D grid，每个segment一个block，增加线程数以处理大bbox
    const int THREADS_PER_BLOCK = 1024;  // 参考yolo_seg_alg
    dim3 block(THREADS_PER_BLOCK);

    // 1D grid: num_segs个blocks
    dim3 grid(num_segs);

    process_mask_kernel<<<grid, block, 0, stream>>>(
        d_proto, proto_h, proto_w,
        d_mask_coef, d_bboxes, num_segs,
        ratio, dw, dh, input_w, input_h,
        d_output_masks, orig_w, orig_h,
        cut_left, cut_top, cut_w, cut_h
    );
    
    // 注意：d_output_binary_masks 参数暂时不使用，GPU mask处理仍需进一步优化
    // 当前实现参考yolo_seg_alg，只输出float类型的mask值
}
