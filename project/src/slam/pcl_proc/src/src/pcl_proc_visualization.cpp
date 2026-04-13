#include "../include/pcl_proc/pcl_proc_visualization.h"
#include <pcl/visualization/point_cloud_color_handlers.h>

namespace PclProc
{
    /**
     * @brief 创建简单的点云可视化器
     * @param cloud 输入点云
     * @param viewer_name 查看器名称
     * @return pcl::visualization::PCLVisualizer::Ptr 可视化查看器指针
     */
    pcl::visualization::PCLVisualizer::Ptr simpleVis(const PointCloudPtr& cloud, const std::string& viewer_name)
    {
        // --------------------------------------------
        // -----Open 3D viewer and add point cloud-----
        // --------------------------------------------
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (viewer_name));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        return (viewer);
    }
    
    /**
     * @brief 创建自定义颜色的点云可视化器
     * @param cloud 输入点云
     * @param r 红色分量(0-255)
     * @param g 绿色分量(0-255)
     * @param b 蓝色分量(0-255)
     * @param viewer_name 查看器名称
     * @return pcl::visualization::PCLVisualizer::Ptr 可视化查看器指针
     */
    pcl::visualization::PCLVisualizer::Ptr customColourVis(const PointCloudPtr& cloud, int r, int g, int b, const std::string& viewer_name)
    {
        // --------------------------------------------
        // -----Open 3D viewer and add point cloud-----
        // --------------------------------------------
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (viewer_name));
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, r, g, b);
        viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        return (viewer);
    }
    
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
                                                    int level, 
                                                    float scale,
                                                    const std::string& viewer_name)
    {
        // --------------------------------------------------------
        // -----Open 3D viewer and add point cloud and normals-----
        // --------------------------------------------------------
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (viewer_name));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, level, scale, "normals");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        return (viewer);
    }
    
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
                                                      const std::string& cloud1_name,
                                                      const std::string& cloud2_name)
    {
        // --------------------------------------------------------
        // -----Open 3D viewer and add point cloud and normals-----
        // --------------------------------------------------------
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters ();

        int v1(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor (0, 0, 0, v1);
        viewer->addText("Viewport 1", 10, 10, "v1 text", v1);
        viewer->addPointCloud<pcl::PointXYZ> (cloud1, cloud1_name, v1);

        int v2(0);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
        viewer->addText("Viewport 2", 10, 10, "v2 text", v2);
        viewer->addPointCloud<pcl::PointXYZ> (cloud2, cloud2_name, v2);

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud1_name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud2_name);
        viewer->addCoordinateSystem (1.0);

        return (viewer);
    }

}