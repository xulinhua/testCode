#ifndef PCL_PROC_SEGMENT_H
#define PCL_PROC_SEGMENT_H

#include"pcl_proc_base.h"

namespace PclProc
{
    /**
     * @brief 从点云中分割平面模型以提取几何模型（平面、圆柱等）
     * @param cloud_in 输入点云
     * @param plane_cloud 输出平面点云
     * @param remaining_cloud 输出剩余点云
     * @return int 成功返回非负值，失败返回-1
     */
    int planeSegmentation(const PointCloudPtr& cloud_in, PointCloudPtr& plane_cloud, PointCloudPtr& remaining_cloud);

    /**
     * @brief 圆柱体分割
     * @param cloud_in 输入点云
     * @param plane_cloud 输出平面点云
     * @param remaining_cloud 输出剩余点云
     * @return int 成功返回非负值，失败返回-1
     */
    int CylinderSegmentation(const PointCloudPtr& cloud_in, PointCloudPtr& plane_cloud, PointCloudPtr& remaining_cloud);

    /**
     * @brief 用于物体分割的欧几里得聚类，适用于粗分割带细节
     * @param cloud_in 输入点云
     * @param tolerance 聚类容忍度，默认0.02f
     * @param min_size 最小聚类大小，默认100
     * @param max_size 最大聚类大小，默认25000
     * @return std::vector<PointCloudPtr> 返回聚类结果
     */
    std::vector<PointCloudPtr> euclideanClusterExtraction(const PointCloudPtr& cloud_in, float tolerance = 0.02f, int min_size = 100, int max_size = 25000);

    /**
     * @brief 基于点法向量特征、曲率、平滑度等的区域增长分割
     * @param cloud_in 输入点云
     * @param clusters 输出聚类索引
     * @return int 成功返回聚类数量，失败返回-1
     */
    int regionGrowingSegmentation(const PointCloudPtr& cloud_in, std::vector<pcl::PointIndices>& clusters);

    /**
     * @brief 用于物体分割的条件欧几里得聚类，适用于粗分割带细节
     * @param cloud_in 输入点云
     * @param tolerance 聚类容忍度，默认0.02f
     * @param min_size 最小聚类大小，默认100
     * @param max_size 最大聚类大小，默认25000
     * @return std::vector<PointCloudPtr> 返回聚类结果
     */
    std::vector<PointCloudPtr> conditionalEuclideanClustering(const PointCloudPtr& cloud_in, float tolerance = 0.02f, int min_size = 100, int max_size = 25000);
    
}

#endif