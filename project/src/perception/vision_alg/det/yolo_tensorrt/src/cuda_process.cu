#include "cuda_process.h"
#include <math.h>
#include <device_launch_parameters.h>
#include <cuda_runtime.h>

// 预处理 kernel：HWC -> CHW + Letterbox + 归一化 + BGR->RGB
__global__ void preprocess_kernel(
    const uint8_t* __restrict__ src,  // HWC 输入 (BGR)
    int src_h, int src_w,
    float* out,                        // CHW 输出
    int dst_h, int dst_w,
    float scale,                       // resize 缩放
    int pad_top, int pad_left,         // 左上角填充
    float norm_scale                   // 1/255.0
) {
    int dx = blockIdx.x * blockDim.x + threadIdx.x;
    int dy = blockIdx.y * blockDim.y + threadIdx.y;
    if (dx >= dst_w || dy >= dst_h) return;

    // 1. 去掉填充，映射到原图坐标
    int sx = (int)rintf((dx - pad_left) / scale);
    int sy = (int)rintf((dy - pad_top) / scale);

    // 2. 越界用常量 114
    uint8_t b = 114, g = 114, r = 114;
    if (sx >= 0 && sy >= 0 && sx < src_w && sy < src_h) {
        int src_idx = (sy * src_w + sx) * 3;
        b = src[src_idx];
        g = src[src_idx + 1];
        r = src[src_idx + 2];
    }

    // 3. 归一化 + BGR->RGB
    float rf = r * norm_scale;
    float gf = g * norm_scale;
    float bf = b * norm_scale;

    // 4. 写到 CHW
    int area = dst_h * dst_w;
    out[0 * area + dy * dst_w + dx] = rf;
    out[1 * area + dy * dst_w + dx] = gf;
    out[2 * area + dy * dst_w + dx] = bf;
}

// 后处理过滤 kernel：过滤低置信度框 + 坐标反算
__global__ void filter_kernel(
    const float* __restrict__ src,     // YOLO 输出 [num_classes+5, num_boxes]
    int num_boxes,
    int num_classes,
    float conf_thresh,
    float ratio, float dw, float dh,    // Letterbox 参数
    Bbox32* out,                       // 输出候选框
    int* keep_num                      // 实际保留数量
) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_boxes) return;

    // YOLO 输出格式: [cx, cy, w, h, objectness, class1, class2, ...]
    // 内存布局：行优先 [num_classes+5, num_boxes]

    // 找最大类别得分
    float max_score = 0.0f;
    int best_class = 0;

    // 遍历 objectness 和所有类别概率 (从第 4 通道开始)
    for (int c = 4; c < 4 + num_classes; ++c) {
        float score = src[c * num_boxes + i];
        if (score > max_score) {
            max_score = score;
            best_class = c - 4;
        }
    }

    // 置信度阈值过滤
    if (max_score < conf_thresh) return;

    // 线程安全的原子计数，并检查越界
    int idx = atomicAdd(keep_num, 1);
    if (idx >= num_boxes) {
        // 防止越界，恢复计数
        atomicSub(keep_num, 1);
        return;
    }

    // 提取边界框 (YOLO 格式: center_x, center_y, width, height)
    float cx = src[0 * num_boxes + i];
    float cy = src[1 * num_boxes + i];
    float w = src[2 * num_boxes + i];
    float h = src[3 * num_boxes + i];

    // 坐标反算：去除 letterbox 缩放和 padding
    Bbox32& bbox = out[idx];
    bbox.x = (cx - dw) / ratio;
    bbox.y = (cy - dh) / ratio;
    bbox.w = w / ratio;
    bbox.h = h / ratio;
    bbox.conf = max_score;
    bbox.class_id = best_class;
}

// 启动预处理 kernel
void launch_preprocess_kernel(
    const uint8_t* d_src, int src_h, int src_w,
    float* d_dst, int dst_h, int dst_w,
    float scale, int pad_top, int pad_left, float norm,
    cudaStream_t stream
) {
    dim3 block(16, 16);
    dim3 grid((dst_w + block.x - 1) / block.x,
              (dst_h + block.y - 1) / block.y);

    preprocess_kernel<<<grid, block, 0, stream>>>(
        d_src, src_h, src_w, d_dst, dst_h, dst_w,
        scale, pad_top, pad_left, norm
    );
}

// 启动后处理过滤 kernel
void launch_filter_kernel(
    const float* d_src,
    int num_boxes, int num_classes,
    float conf_thresh,
    float ratio, float dw, float dh,
    Bbox32* d_boxes,
    int* d_keep_num,
    cudaStream_t stream
) {
    dim3 block(256);
    dim3 grid((num_boxes + block.x - 1) / block.x);

    filter_kernel<<<grid, block, 0, stream>>>(
        d_src, num_boxes, num_classes, conf_thresh,
        ratio, dw, dh, d_boxes, d_keep_num
    );
}
