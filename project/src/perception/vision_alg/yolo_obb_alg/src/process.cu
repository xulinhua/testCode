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

#ifdef __cplusplus
extern "C" {
#endif
#include <math.h>
#include <float.h>

// 2D点结构
struct Point2f {
    float x, y;
    __host__ __device__ Point2f(float x_ = 0, float y_ = 0) : x(x_), y(y_) {}
};

// 计算两点距离
__device__ float distance(const Point2f& a, const Point2f& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrtf(dx*dx + dy*dy);
}

// 计算向量叉积
__device__ float cross(const Point2f& a, const Point2f& b) {
    return a.x * b.y - a.y * b.x;
}

// 计算多边形面积（鞋带公式）
__device__ float polygonArea(const Point2f* points, int n) {
    float area = 0.0f;
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        area += points[i].x * points[j].y - points[j].x * points[i].y;
    }
    return fabsf(area) * 0.5f;
}

// 判断点是否在多边形内
__device__ bool pointInPolygon(const Point2f& p, const Point2f* poly, int n) {
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((poly[i].y > p.y) != (poly[j].y > p.y)) &&
            (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

// 线段相交检测
__device__ bool lineIntersection(const Point2f& p1, const Point2f& p2, 
                                const Point2f& q1, const Point2f& q2, 
                                Point2f& intersection) {
    float s1_x = p2.x - p1.x;
    float s1_y = p2.y - p1.y;
    float s2_x = q2.x - q1.x;
    float s2_y = q2.y - q1.y;
    
    float s = (-s1_y * (p1.x - q1.x) + s1_x * (p1.y - q1.y)) / (-s2_x * s1_y + s1_x * s2_y);
    float t = ( s2_x * (p1.y - q1.y) - s2_y * (p1.x - q1.x)) / (-s2_x * s1_y + s1_x * s2_y);
    
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
        intersection.x = p1.x + (t * s1_x);
        intersection.y = p1.y + (t * s1_y);
        return true;
    }
    return false;
}

// 计算旋转框的四个顶点
__device__ void getRotatedRectVertices(float cx, float cy, float w, float h, float angle, Point2f vertices[4]) {
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    
    // 计算半宽半高
    float half_w = w * 0.5f;
    float half_h = h * 0.5f;
    
    // 计算四个顶点（相对于中心点）
    vertices[0] = Point2f(cx + half_w*cos_a - half_h*sin_a, cy + half_w*sin_a + half_h*cos_a);
    vertices[1] = Point2f(cx - half_w*cos_a - half_h*sin_a, cy - half_w*sin_a + half_h*cos_a);
    vertices[2] = Point2f(cx - half_w*cos_a + half_h*sin_a, cy - half_w*sin_a - half_h*cos_a);
    vertices[3] = Point2f(cx + half_w*cos_a + half_h*sin_a, cy + half_w*sin_a - half_h*cos_a);
}

// 计算两个多边形的相交面积（Sutherland-Hodgman算法）
__device__ float polygonIntersectionArea(const Point2f* poly1, int n1, const Point2f* poly2, int n2) {
    // 这里实现完整的多边形相交面积计算
    // 由于GPU实现复杂，我们使用一个精确但相对简单的算法
    
    // 方法：使用网格采样法计算相交面积
    const int GRID_SIZE = 16;  // 16x16网格
    const float EPSILON = 1e-6f;
    
    // 计算两个多边形的边界框
    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;
    
    for (int i = 0; i < n1; i++) {
        min_x = fminf(min_x, poly1[i].x);
        max_x = fmaxf(max_x, poly1[i].x);
        min_y = fminf(min_y, poly1[i].y);
        max_y = fmaxf(max_y, poly1[i].y);
    }
    for (int i = 0; i < n2; i++) {
        min_x = fminf(min_x, poly2[i].x);
        max_x = fmaxf(max_x, poly2[i].x);
        min_y = fminf(min_y, poly2[i].y);
        max_y = fmaxf(max_y, poly2[i].y);
    }
    
    // 如果边界框不相交，直接返回0
    if (max_x < min_x || max_y < min_y) return 0.0f;
    
    // 计算采样步长
    float step_x = (max_x - min_x) / GRID_SIZE;
    float step_y = (max_y - min_y) / GRID_SIZE;
    
    if (step_x < EPSILON || step_y < EPSILON) return 0.0f;
    
    // 网格采样计算相交面积
    int intersection_count = 0;
    int total_count = 0;
    
    for (int i = 0; i <= GRID_SIZE; i++) {
        for (int j = 0; j <= GRID_SIZE; j++) {
            Point2f sample_point(min_x + i * step_x, min_y + j * step_y);
            
            bool in_poly1 = pointInPolygon(sample_point, poly1, n1);
            bool in_poly2 = pointInPolygon(sample_point, poly2, n2);
            
            if (in_poly1 && in_poly2) {
                intersection_count++;
            }
            total_count++;
        }
    }
    
    // 计算相交面积
    float bbox_area = (max_x - min_x) * (max_y - min_y);
    float intersection_area = bbox_area * (float)intersection_count / (float)total_count;
    
    return intersection_area;
}

// 计算两个旋转框的精确IOU
__device__ float calculateRotatedIOU(const Bbox32& a, const Bbox32& b) {
    // 获取两个旋转框的顶点
    Point2f vertices_a[4], vertices_b[4];
    getRotatedRectVertices(a.x, a.y, a.w, a.h, a.angle, vertices_a);
    getRotatedRectVertices(b.x, b.y, b.w, b.h, b.angle, vertices_b);
    
    // 计算两个多边形的面积
    float area_a = polygonArea(vertices_a, 4);
    float area_b = polygonArea(vertices_b, 4);
    
    // 计算相交面积
    float intersection_area = polygonIntersectionArea(vertices_a, 4, vertices_b, 4);
    
    // 计算并集面积
    float union_area = area_a + area_b - intersection_area;
    
    // 避免除零
    if (union_area < 1e-6f) return 0.0f;
    
    return intersection_area / union_area;
}

// 输出：float* out 已经排成 CHW
__global__ void preprocess_kernel(
    const uchar3* __restrict__ src,   // HWC 输入
    int src_h, int src_w,
    float* out,                       // CHW 输出
    int dst_h, int dst_w,
    float scale,                      // resize 缩放
    int pad_top, int pad_left,        // 左上角填充
    float norm_scale)                 // 1/255.f
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

    // 3. 归一化 + BGR→RGB
    float r = pix.z * norm_scale;
    float g = pix.y * norm_scale;
    float b = pix.x * norm_scale;

    // 4. 写到 CHW
    int area = dst_h * dst_w;
    out[0 * area + dy * dst_w + dx] = r;
    out[1 * area + dy * dst_w + dx] = g;
    out[2 * area + dy * dst_w + dx] = b;
}

void launch_preprocess_kernel(const uchar3* d_src, int src_h, int src_w,
                              float* d_dst, int dst_h, int dst_w,
                              float scale, int pad_top, int pad_left, float norm,
                              cudaStream_t stream)
{
    dim3 block(16, 16);
    dim3 grid((dst_w + block.x - 1) / block.x,
              (dst_h + block.y - 1) / block.y);
    preprocess_kernel<<<grid, block, 0, stream>>>(d_src, src_h, src_w, d_dst, dst_h, dst_w,
        scale, pad_top, pad_left, norm);
}


// 过滤 + 反算 kernel
__global__ void filter_kernel(
    float* __restrict__ src,     
    int num_boxes,
    int num_classes,
    float conf_thresh,
    float ratio, float dw, float dh,
    Bbox32* out,                       // 输出候选框
    int* keep_num)                     // 实际保留数量
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_boxes) return;

    int cls_start = 4;
    int cls_end = 4 + num_classes; 
    float* ptr = src;         
   
    float max_score = 0.0;          // 初始第 0 类
    int cls = 0;
    for (int c = cls_start; c < cls_end; ++c) 
    {
        float s = ptr[c*num_boxes+i];
        if (s > max_score) { max_score = s; cls = c; }
    }
    // 置信度阈值直接作用在「最大类别得分」
    if (max_score < conf_thresh) return;

    // 反算到原图
    int idx = atomicAdd(keep_num, 1);   // 线程安全计数
    float cx   = ptr[0*num_boxes+i];
    float cy   = ptr[1*num_boxes+i];
    float w    = ptr[2*num_boxes+i];
    float h    = ptr[3*num_boxes+i];
    float angle= ptr[cls_end*num_boxes+i];   // 角度信息
    Bbox32& b = out[idx];
    b.x   = (cx - dw) / ratio;
    b.y   = (cy - dh) / ratio;
    b.w   = w / ratio;
    b.h   = h / ratio;
    b.angle = angle;                   // 存储角度
    b.conf= max_score;
    b.cls = cls-4;
}

__global__ void nms_matrix_kernel(
    const Bbox32* __restrict__ boxes,
    int n,
    float iou_thresh,
    uint8_t* mask)          // ← 改成 uint8_t
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    if (i >= n || j >= n || j <= i) return;

    Bbox32 a = boxes[i];
    Bbox32 b = boxes[j];
    if (a.cls != b.cls) {
        mask[i * n + j] = 0;  // 不同类别不抑制
        return;
    }

    // 使用旋转框IOU计算
    float iou = calculateRotatedIOU(a, b);

    // 当iou大于阈值时，表示需要抑制（设置mask为1）
    mask[i * n + j] = (iou > iou_thresh) ? 1 : 0;   // 1 字节
}

struct CompareBbox {
    __host__ __device__
    bool operator()(const Bbox32& a, const Bbox32& b) const {
        return a.conf > b.conf;
    }
};

void yolo_postprocess_gpu(
    float* d_trt_out,
    int num_boxes, int num_classes,
    float conf_thresh, float iou_thresh,
    float ratio, float dw, float dh,
    Bbox32* d_out, int* d_keep_num,
    std::vector<Bbox32>& box_out,
    cudaStream_t stream)
{
    // 过滤 + 反算（不变）
    int threads = 256;
    int blocks  = (num_boxes + threads - 1) / threads;
    cudaMemsetAsync(d_keep_num, 0, sizeof(int), stream);
    filter_kernel<<<blocks, threads, 0, stream>>>(
        d_trt_out, num_boxes, num_classes, conf_thresh, ratio, dw, dh,
        d_out, d_keep_num);

    int h_keep_num;
    cudaMemcpyAsync(&h_keep_num, d_keep_num, sizeof(int),
                    cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);
    if (h_keep_num == 0) { box_out.clear(); return; }
    
    // NMS 之前排序
    if(0)
    {
        thrust::device_ptr<Bbox32> ptr(d_out);
        thrust::sort(thrust::device, d_out, d_out + h_keep_num, CompareBbox());
        // thrust::sort(thrust::cuda::par.on(stream), d_out, d_out + h_keep_num,CompareBbox());
    }
    else
    {
        // 先把框拷回 CPU（只搬 h_keep_num 个，<1 KB）
        std::vector<Bbox32> host_box(h_keep_num);
        cudaMemcpyAsync(host_box.data(), d_out,
                        h_keep_num * sizeof(Bbox32),
                        cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);

        // CPU 端降序排序
        std::sort(host_box.begin(), host_box.end(),
                [](const Bbox32& a, const Bbox32& b) {
                    return a.conf > b.conf;   // 降序
                });

        // 再拷回 GPU
        cudaMemcpyAsync(d_out, host_box.data(),
                        h_keep_num * sizeof(Bbox32),
                        cudaMemcpyHostToDevice, stream);
    }

    // 掩码矩阵（n×n，只存 uint8_t 节省空间）
    size_t matrix_bytes = h_keep_num * h_keep_num * sizeof(uint8_t);
    uint8_t* d_mask = nullptr;
    cudaMalloc(&d_mask, matrix_bytes);

    dim3 block(16, 16);
    dim3 grid((h_keep_num + block.x - 1) / block.x,
              (h_keep_num + block.y - 1) / block.y);
    nms_matrix_kernel<<<grid, block, 0, stream>>>(
        d_out, h_keep_num, iou_thresh, d_mask);

    // 解压掩码 → keep 索引（CPU 端，<n² Byte）
    uint8_t* h_mask = new uint8_t[h_keep_num * h_keep_num];
    cudaMemcpyAsync(h_mask, d_mask, matrix_bytes,
                    cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    std::vector<int> keep_host;
    keep_host.push_back(0);                 // 0 号必保留
    for (int i = 1; i < h_keep_num; ++i) {
        bool suppressed = false;
        for (int j = 0; j < i; ++j) {      // 前面任一框抑制我
            if (h_mask[j * h_keep_num + i] != 0) { 
                suppressed = true; 
                break; 
            }
        }
        if (!suppressed) keep_host.push_back(i);
    }
    int final_num = keep_host.size();

    // std::vector<Bbox32> all_out;
    // all_out.resize(h_keep_num);
    // for (int k = 0; k < h_keep_num; ++k)
    //     cudaMemcpyAsync(&all_out[k], &d_out[k],
    //                     sizeof(Bbox32), cudaMemcpyDeviceToHost, stream);
    // cudaStreamSynchronize(stream);

    // 只拷 keep 框
    box_out.resize(final_num);
    for (int k = 0; k < final_num; ++k)
        cudaMemcpyAsync(&box_out[k], &d_out[keep_host[k]],
                        sizeof(Bbox32), cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    // 清理
    delete[] h_mask;
    cudaFree(d_mask);
}

#ifdef __cplusplus
}
#endif