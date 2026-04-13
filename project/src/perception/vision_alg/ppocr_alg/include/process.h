#pragma once
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudawarping.hpp>

#define PROCESS_GPU 1
void launch_preprocess_kernel(const uchar3* d_src, int src_h, int src_w,
                              float* d_dst, int dst_h, int dst_w,
                              float scale, int pad_top, int pad_left, float norm,
                              const float* d_mean, const float* d_std,
                              cudaStream_t stream);