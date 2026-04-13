#ifndef PCL_PROC_FILTER_H
#define PCL_PROC_FILTER_H
#include"pcl_proc_base.h"

namespace PclProc
{
    /**
     * @brief 移除NaN点
     * @param cloud_in 输入点云
     * @return PointCloudPtr 移除NaN点后的点云
     */
    PointCloudPtr removeNaNFromPoint(const PointCloudPtr& cloud_in);
    
    /**
     * @brief 无序点的标准体素网格滤波
     * @param cloud_in 输入点云
     * @param lx x方向体素大小，默认0.01f
     * @param ly y方向体素大小，默认0.01f
     * @param lz z方向体素大小，默认0.01f
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr voxelGridFilter(const PointCloudPtr& cloud_in, float lx=0.01f, float ly = 0.01f, float lz = 0.01f);

    /**
     * @brief 近似体素网格滤波
     * @param cloud_in 输入点云
     * @param lx x方向体素大小，默认0.01f
     * @param ly y方向体素大小，默认0.01f
     * @param lz z方向体素大小，默认0.01f
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr approxVoxelGridFilter(const PointCloudPtr& cloud_in, float lx = 0.01f, float ly = 0.01f, float lz = 0.01f);

    /**
     * @brief 统计异常值移除滤波
     * @param cloud_in 输入点云
     * @param mean_k 邻域点数量，默认50
     * @param stddev_mult 标准差倍数，默认1.0f
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr statisticalOutlierRemoval(const PointCloudPtr& cloud_in, int mean_k = 50, float stddev_mult = 1.0f);

    /**
     * @brief 半径异常值移除滤波
     * @param cloud_in 输入点云
     * @param radius 搜索半径，默认0.1f
     * @param min_neighbors 最小邻居数，默认5
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr radiusOutlierRemoval(const PointCloudPtr& cloud_in, float radius = 0.1f, int min_neighbors = 5);

    /**
     * @brief 直通滤波
     * @param cloud_in 输入点云
     * @param field_name 过滤字段名，默认"z"
     * @param min_limit 最小限制值，默认0.0f
     * @param max_limit 最大限制值，默认1.0f
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr passThroughFilter(const PointCloudPtr& cloud_in, const std::string& field_name = "z", float min_limit = 0.0f, float max_limit = 1.0f);

    /**
     * @brief 条件滤波
     * @param cloud_in 输入点云
     * @param field_name 过滤字段名，默认"z"
     * @param min_limit 最小限制值，默认0.0f
     * @param max_limit 最大限制值，默认1.0f
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr conditionalRemoval(const PointCloudPtr& cloud_in, const std::string& field_name = "z", float min_limit = 0.0f, float max_limit = 1.0f);

    /**
     * @brief 双边滤波
     * @param cloud_in 输入点云
     * @param sigma_s 空间标准差，默认10.0f
     * @param sigma_r 色彩标准差，默认0.05f
     * @return PointXYZICloudPtr 滤波后的点云
     */
    PointXYZICloudPtr bilateralFilter(const PointXYZICloudPtr& cloud_in, float sigma_s = 10.0f, float sigma_r = 0.05f);

    /**
     * @brief 快速双边滤波
     * @param cloud_in 输入点云
     * @param sigma_s 空间标准差，默认10.0f
     * @param sigma_r 色彩标准差，默认0.05f
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr fastBilateralFilter(const PointCloudPtr& cloud_in, float sigma_s = 10.0f, float sigma_r = 0.05f);

    /**
     * @brief 中值滤波
     * @param cloud_in 输入点云
     * @param window_size 窗口大小，默认10
     * @param max_movement 最大移动距离，默认0.05f
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr medianFilter(const PointCloudPtr& cloud_in, int window_size = 10, float max_movement = 0.05f);

    /**
     * @brief 均匀采样
     * @param cloud_in 输入点云
     * @param radius 搜索半径，默认0.005f
     * @return PointCloudPtr 采样后的点云
     */
    PointCloudPtr uniformSampling(const PointCloudPtr& cloud_in, double radius = 0.005f);

}

#endif