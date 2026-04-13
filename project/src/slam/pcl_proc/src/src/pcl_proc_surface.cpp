#include "../include/pcl_proc/pcl_proc_surface.h"
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace PclProc
{
    /**
     * @brief 移动最小二乘法平滑
     * @param cloud_in 输入点云
     * @param search_radius 搜索半径
     * @return PointCloudPtr 平滑后的点云
     */
    PointCloudPtr movingLeastSquares(const PointCloudPtr& cloud_in, float search_radius)
    {
        PointCloudPtr result(new PointCloudT);
        if (cloud_in->size() < 1)
        {
            LOG_ERROR( "movingLeastSquares接收到空点云");
            return result;
        }
        
        try {
            // 创建移动最小二乘法对象
            pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
            pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
            
            // 设置参数
            mls.setComputeNormals(true);
            mls.setInputCloud(cloud_in);
            mls.setPolynomialOrder(2);
            mls.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
            mls.setSearchRadius(search_radius);
            
            // 计算
            mls.process(*mls_points);
            
            // 转换回PointXYZ类型
            pcl::copyPointCloud(*mls_points, *result);
        } catch (const std::exception& e) {
            LOG_ERROR( "movingLeastSquares函数中发生异常: %s", e.what());
        }
        
        return result;
    }

    /**
     * @brief 贪婪投影三角化
     * @param cloud_in 输入点云
     * @param search_radius 搜索半径
     * @param mu 法向量与查询点之间的最大角度阈值
     * @param max_nn 最近邻点的最大数量
     * @param max_surface_angle 表面角度阈值
     * @param min_angle 三角形内角最小值
     * @param max_angle 三角形内角最大值
     * @return pcl::PolygonMesh 三角网格
     */
    pcl::PolygonMesh greedyProjectionTriangulation(const PointCloudPtr& cloud_in, 
                                                  float search_radius,
                                                  float mu,
                                                  int max_nn,
                                                  float max_surface_angle,
                                                  float min_angle,
                                                  float max_angle)
    {
        pcl::PolygonMesh triangles;
        if (cloud_in->size() < 1)
        {
            LOG_ERROR( "greedyProjectionTriangulation接收到空点云");
            return triangles;
        }
        
        try {
            // 法向量估计
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud_in);
            n.setInputCloud(cloud_in);
            n.setSearchMethod(tree);
            n.setKSearch(20);
            n.compute(*normals);

            // 将点云和法向量连接
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*cloud_in, *normals, *cloud_with_normals);

            // 创建搜索树
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
            tree2->setInputCloud(cloud_with_normals);

            // 初始化并设置贪婪投影三角化对象
            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
            gp3.setSearchRadius(search_radius);
            gp3.setMu(mu);
            gp3.setMaximumNearestNeighbors(max_nn);
            gp3.setMaximumSurfaceAngle(max_surface_angle);
            gp3.setMinimumAngle(min_angle);
            gp3.setMaximumAngle(max_angle);
            gp3.setNormalConsistency(false);

            gp3.setInputCloud(cloud_with_normals);
            gp3.setSearchMethod(tree2);
            gp3.reconstruct(triangles);
        } catch (const std::exception& e) {
            LOG_ERROR(  "greedyProjectionTriangulation函数中发生异常: %s", e.what());
        }
        
        return triangles;
    }

    /**
     * @brief 泊松重建
     * @param cloud_in 输入点云
     * @param depth 树深度
     * @param minDepth 最小深度
     * @param pointWeight 点权重
     * @return pcl::PolygonMesh 三角网格
     */
    pcl::PolygonMesh poissonReconstruction(const PointCloudPtr& cloud_in,
                                          int depth,
                                          int minDepth,
                                          float pointWeight)
    {
        pcl::PolygonMesh triangles;
        if (cloud_in->size() < 1)
        {
            LOG_ERROR(  "poissonReconstruction接收到空点云");
            return triangles;
        }
        
        try {
            // 法向量估计
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud_in);
            n.setInputCloud(cloud_in);
            n.setSearchMethod(tree);
            n.setKSearch(20);
            n.compute(*normals);

            // 将点云和法向量连接
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*cloud_in, *normals, *cloud_with_normals);

            // 泊松重建
            pcl::Poisson<pcl::PointNormal> poisson;
            poisson.setDepth(depth);
            poisson.setMinDepth(minDepth);
            poisson.setPointWeight(pointWeight);
            poisson.setInputCloud(cloud_with_normals);
            poisson.reconstruct(triangles);
        } catch (const std::exception& e) {
            LOG_ERROR(  "poissonReconstruction函数中发生异常: %s", e.what());
        }
        
        return triangles;
    }

    /**
     * @brief 栅格投影
     * @param cloud_in 输入点云
     * @param resolution 栅格分辨率
     * @return pcl::PolygonMesh 三角网格
     */
    pcl::PolygonMesh gridProjection(const PointCloudPtr& cloud_in, float resolution)
    {
        pcl::PolygonMesh triangles;
        if (cloud_in->size() < 1)
        {
            LOG_ERROR(  "gridProjection接收到空点云");
            return triangles;
        }
        
        try {
            // 法向量估计
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud_in);
            n.setInputCloud(cloud_in);
            n.setSearchMethod(tree);
            n.setKSearch(20);
            n.compute(*normals);

            // 将点云和法向量连接
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*cloud_in, *normals, *cloud_with_normals);

            // 栅格投影
            pcl::GridProjection<pcl::PointNormal> gridprojection;
            gridprojection.setResolution(resolution);
            gridprojection.setPaddingSize(3);
            gridprojection.setInputCloud(cloud_with_normals);
            
            // 创建搜索树
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
            tree2->setInputCloud(cloud_with_normals);
            gridprojection.setSearchMethod(tree2);
            
            gridprojection.reconstruct(triangles);
        } catch (const std::exception& e) {
            LOG_ERROR(  "gridProjection函数中发生异常: %s", e.what());
        }
        
        return triangles;
    }

}