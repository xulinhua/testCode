#include "process.h"
#include <device_launch_parameters.h>
#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/sort.h>
#include <thrust/partition.h>

// 输出：float* out 已经排成 CHW
__global__ void preprocess_kernel(
    const uchar3* __restrict__ src,   // HWC 输入
    int src_h, int src_w,
    float* out,                       // CHW 输出
    int dst_h, int dst_w,
    float scale,                      // resize 缩放
    int pad_top, int pad_left,        // 左上角填充
    float norm_scale,                 // 1/255.f
    const float* __restrict__ mean,   // RGB均值 [3]
    const float* __restrict__ std)    // RGB标准差 [3]
{
    int dx = blockIdx.x * blockDim.x + threadIdx.x;
    int dy = blockIdx.y * blockDim.y + threadIdx.y;
    if (dx >= dst_w || dy >= dst_h) return;

    // 1. 去掉填充，映射到原图坐标
    int sx = (int)rintf((dx - pad_left) / scale);
    int sy = (int)rintf((dy - pad_top)  / scale);

    // 2. 越界用常量 114
    uchar3 pix;
    if (sx < 0 || sy < 0 || sx >= src_w || sy >= src_h)
        pix = make_uchar3(114, 114, 114);
    else
        pix = src[sy * src_w + sx];

    // 3. 归一化 + BGR→RGB + 标准化
    float r_raw = pix.z * norm_scale;  // 0-1
    float g_raw = pix.y * norm_scale;  // 0-1
    float b_raw = pix.x * norm_scale;  // 0-1

    // 减去均值并除以标准差
    float r = (r_raw - mean[0]) / std[0];
    float g = (g_raw - mean[1]) / std[1];
    float b = (b_raw - mean[2]) / std[2];

    // 4. 写到 CHW
    int area = dst_h * dst_w;
    out[0 * area + dy * dst_w + dx] = r;
    out[1 * area + dy * dst_w + dx] = g;
    out[2 * area + dy * dst_w + dx] = b;
}

void launch_preprocess_kernel(const uchar3* d_src, int src_h, int src_w,
                              float* d_dst, int dst_h, int dst_w,
                              float scale, int pad_top, int pad_left, float norm,
                              const float* d_mean, const float* d_std,
                              cudaStream_t stream)
{
    dim3 block(16, 16);
    dim3 grid((dst_w + block.x - 1) / block.x,
              (dst_h + block.y - 1) / block.y);
    preprocess_kernel<<<grid, block, 0, stream>>>(d_src, src_h, src_w, d_dst, dst_h, dst_w,
        scale, pad_top, pad_left, norm, d_mean, d_std);
}