#ifndef PCL_PROC_SURFACE_H
#define PCL_PROC_SURFACE_H

#include "pcl_proc_base.h"
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/grid_projection.h>

namespace PclProc
{
    /**
     * @brief 移动最小二乘法平滑
     * @param cloud_in 输入点云
     * @param search_radius 搜索半径，默认0.05f
     * @return PointCloudPtr 平滑后的点云
     */
    PointCloudPtr movingLeastSquares(const PointCloudPtr& cloud_in, float search_radius = 0.05f);

    /**
     * @brief 贪婪投影三角化
     * @param cloud_in 输入点云
     * @param search_radius 搜索半径，默认0.05f
     * @param mu 法向量与查询点之间的最大角度阈值，默认25.0f
     * @param max_nn 最近邻点的最大数量，默认100
     * @param max_surface_angle 表面角度阈值，默认M_PI/4
     * @param min_angle 三角形内角最小值，默认M_PI/18
     * @param max_angle 三角形内角最大值，默认2*M_PI/3
     * @return pcl::PolygonMesh 三角网格
     */
    pcl::PolygonMesh greedyProjectionTriangulation(const PointCloudPtr& cloud_in, 
                                                  float search_radius = 0.05f,
                                                  float mu = 25.0f,
                                                  int max_nn = 100,
                                                  float max_surface_angle = M_PI/4,
                                                  float min_angle = M_PI/18,
                                                  float max_angle = 2*M_PI/3);

    /**
     * @brief 泊松重建
     * @param cloud_in 输入点云
     * @param depth 树深度，默认9
     * @param minDepth 最小深度，默认5
     * @param pointWeight 点权重，默认4.0f
     * @return pcl::PolygonMesh 三角网格
     */
    pcl::PolygonMesh poissonReconstruction(const PointCloudPtr& cloud_in,
                                          int depth = 9,
                                          int minDepth = 5,
                                          float pointWeight = 4.0f);

    /**
     * @brief 栅格投影
     * @param cloud_in 输入点云
     * @param resolution 栅格分辨率，默认0.005f
     * @return pcl::PolygonMesh 三角网格
     */
    pcl::PolygonMesh gridProjection(const PointCloudPtr& cloud_in, float resolution = 0.005f);

}

#endif