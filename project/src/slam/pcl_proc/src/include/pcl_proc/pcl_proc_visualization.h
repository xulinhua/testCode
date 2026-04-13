#ifndef PCL_PROC_VISUALIZATION_H
#define PCL_PROC_VISUALIZATION_H

#include "pcl_proc_base.h"
#include <pcl/visualization/pcl_visualizer.h>

namespace PclProc
{
    /**
     * @brief 创建简单的点云可视化器
     * @param cloud 输入点云
     * @param viewer_name 查看器名称
     * @return pcl::visualization::PCLVisualizer::Ptr 可视化查看器指针
     */
    pcl::visualization::PCLVisualizer::Ptr simpleVis(const PointCloudPtr& cloud, const std::string& viewer_name = "3D Viewer");
    
    /**
     * @brief 创建自定义颜色的点云可视化器
     * @param cloud 输入点云
     * @param r 红色分量(0-255)
     * @param g 绿色分量(0-255)
     * @param b 蓝色分量(0-255)
     * @param viewer_name 查看器名称
     * @return pcl::visualization::PCLVisualizer::Ptr 可视化查看器指针
     */
    pcl::visualization::PCLVisualizer::Ptr customColourVis(const PointCloudPtr& cloud, int r, int g, int b, const std::string& viewer_name = "3D Viewer");
    
    /**
     * @brief 显示点云及其法向量
     * @param cloud 输入点云
     * @param normals 法向量
     * @param level 显示法向量的密度(每隔level个点显示一个法向量)
     * @param scale 法向量的缩放因子
     * @param viewer_name 查看器名称
     * @return pcl::visualization::PCLVisualizer::Ptr 可视化查看器指针
     */
    pcl::visualization::PCLVisualizer::Ptr normalsVis(const PointCloudPtr& cloud, 
                                                    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
                                                    int level = 10, 
                                                    float scale = 0.05f,
                                                    const std::string& viewer_name = "3D Viewer");
    
    /**
     * @brief 创建多视口可视化器
     * @param cloud1 第一个点云
     * @param cloud2 第二个点云
     * @param cloud1_name 第一个点云名称
     * @param cloud2_name 第二个点云名称
     * @return pcl::visualization::PCLVisualizer::Ptr 可视化查看器指针
     */
    pcl::visualization::PCLVisualizer::Ptr viewportsVis(const PointCloudPtr& cloud1, 
                                                      const PointCloudPtr& cloud2,
                                                      const std::string& cloud1_name = "cloud1",
                                                      const std::string& cloud2_name = "cloud2");

}

#endif
