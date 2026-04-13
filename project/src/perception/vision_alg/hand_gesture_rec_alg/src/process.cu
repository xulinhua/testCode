#include "process.h"
#include <device_launch_parameters.h>

// 输出：float* out 已经排成 CHW
__global__ void preprocess_kernel(
    const uchar3* __restrict__ src,   // HWC 输入 (BGR 顺序)
    int src_h, int src_w,
    float* out,                       // CHW 输出 (BGR 顺序)
    int dst_h, int dst_w,
    float scale,                      // resize 缩放
    int pad_bottom, int pad_right,    // 右下角填充
    const float* __restrict__ mean,   // BGR均值 [3]
    const float* __restrict__ std)    // BGR标准差 [3]
{
    int dx = blockIdx.x * blockDim.x + threadIdx.x;
    int dy = blockIdx.y * blockDim.y + threadIdx.y;
    if (dx >= dst_w || dy >= dst_h) return;

    // 1. 去掉右下角填充，映射到原图坐标
    int sx = (int)rintf(dx / scale);
    int sy = (int)rintf(dy / scale);

    // 2. 越界用常量 114（右下角填充区域）
    uchar3 pix;
    if (sx >= src_w || sy >= src_h) {
        pix.x = 114; pix.y = 114; pix.z = 114;
    } else {
        pix = src[sy * src_w + sx];
    }

    // 3. 归一化 + 标准化 (保持 BGR 顺序)
    // RTMDet归一化: (pixel - mean) / std，不除以255
    // mean: [103.53, 116.28, 123.675], std: [57.375, 57.12, 58.395] (BGR顺序)
    float b = ((float)pix.x - mean[0]) / std[0];  // Blue
    float g = ((float)pix.y - mean[1]) / std[1];  // Green
    float r = ((float)pix.z - mean[2]) / std[2];  // Red

    // 4. 写到 CHW (BGR 顺序: channel 0=B, 1=G, 2=R)
    int area = dst_h * dst_w;
    out[0 * area + dy * dst_w + dx] = b;
    out[1 * area + dy * dst_w + dx] = g;
    out[2 * area + dy * dst_w + dx] = r;
}

void launch_preprocess_kernel(const uchar3* d_src, int src_h, int src_w,
                              float* d_dst, int dst_h, int dst_w,
                              float scale, int pad_bottom, int pad_right,
                              const float* d_mean, const float* d_std,
                              cudaStream_t stream)
{
    dim3 block(16, 16);
    dim3 grid((dst_w + block.x - 1) / block.x,
              (dst_h + block.y - 1) / block.y);
    preprocess_kernel<<<grid, block, 0, stream>>>(d_src, src_h, src_w, d_dst, dst_h, dst_w,
        scale, pad_bottom, pad_right, d_mean, d_std);
}
