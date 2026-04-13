#include "../include/pcl_proc/pcl_proc_segment.h"

#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/extract_indices.h>

#include "../include/pcl_proc/pcl_proc_filter.h"
namespace PclProc
{
    /**
     * @brief 从点云中分割平面模型以提取几何模型（平面、圆柱等）
     * @param cloud_in 输入点云
     * @param plane_cloud 输出平面点云
     * @param remaining_cloud 输出剩余点云
     * @return int 成功返回非负值，失败返回-1
     */
    int planeSegmentation(const PointCloudPtr& cloud_in, PointCloudPtr& plane_cloud, PointCloudPtr& remaining_cloud)
    {
        try {
            // Use RANSAC segmentation to fit a plane model
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            // Create segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01);  // Distance threshold for inliers
            seg.setInputCloud(cloud_in);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0)
            {
                //PCL_ERROR("Could not estimate a planar model for the given dataset.");
                LOG_ERROR(  "无法为给定数据集估计平面模型。");
                return -1;
            }

            // Extract the planar inliers and outliers
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud_in);
            extract.setIndices(inliers);
            extract.setNegative(false);

            // Store the planar inliers
            extract.filter(*plane_cloud);

            // Extract the non-planar outliers (remaining points)
            extract.setNegative(true);
            extract.filter(*remaining_cloud);
        } catch (const std::exception& e) {
            LOG_ERROR(  "planeSegmentation函数中发生异常: %s", e.what());
            return -1;
        }

        return 1;
    }

    /**
     * @brief 圆柱体分割算法
     * @param cloud_in 输入点云
     * @param plane_cloud 输出平面点云
     * @param cylinder_cloud 输出圆柱体点云
     * @return int 成功返回1，失败返回-1
     */
    int CylinderSegmentation(const PointCloudPtr& cloud_in, PointCloudPtr& plane_cloud, PointCloudPtr& cylinder_cloud)
    {
        try {
            // 1. Preprocessing - passthrough filter to extract region of interest
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            //pcl::PassThrough<pcl::PointXYZ> pass;
            //pass.setInputCloud(cloud_in);
            //pass.setFilterFieldName("z");
            //pass.setFilterLimits(0.0, 2.0);  // Adjust according to actual data
            //pass.filter(*filtered_cloud);

            filtered_cloud=passThroughFilter(cloud_in, "z", 0.0, 2.0);

            //2. Normal estimation
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

            ne.setSearchMethod(tree);
            ne.setInputCloud(filtered_cloud);
            ne.setKSearch(50);  // Set number of neighbors
            ne.compute(*cloud_normals);
            //std::cout << "Normal estimation completed" << std::endl;

            //3. Cylinder segmentation
            pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
            pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

            //4. Configure segmentation
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_CYLINDER);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight(0.1);
            seg.setMaxIterations(10000);
            seg.setDistanceThreshold(0.05);
            seg.setRadiusLimits(0.05, 0.2);  // Cylinder radius range
            seg.setInputCloud(filtered_cloud);
            seg.setInputNormals(cloud_normals);

            //4. Perform segmentation
            seg.segment(*inliers_cylinder, *coefficients_cylinder);

            if (inliers_cylinder->indices.size() == 0) {
                //std::cerr << "Cylinder not found" << std::endl;
                LOG_ERROR(  "Cylinder not found in segmentation");
                return -1;
            }

            //std::cout << "Cylinder found, number of points: " << inliers_cylinder->indices.size() << std::endl;

            // 5. Extract cylinder inliers
            //pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(filtered_cloud);
            extract.setIndices(inliers_cylinder);
            extract.setNegative(false);
            extract.filter(*cylinder_cloud);
        } catch (const std::exception& e) {
            LOG_ERROR(  "CylinderSegmentation函数中发生异常: %s", e.what());
            return -1;
        }

        return 1;
    }

    /**
     * @brief 欧几里得聚类分割算法
     * @param cloud_in 输入点云
     * @param tolerance 聚类容差
     * @param min_size 最小聚类尺寸
     * @param max_size 最大聚类尺寸
     * @return std::vector<PointCloudPtr> 分割后的点云集合
     */
    std::vector<PointCloudPtr> euclideanClusterExtraction(const PointCloudPtr& cloud_in, float tolerance, int min_size, int max_size)
    {
        std::vector<PointCloudPtr> clusters;
        try {
            // Create KD-tree for searching
            pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
            tree->setInputCloud(cloud_in);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<PointT> ec;
            ec.setClusterTolerance(tolerance);
            ec.setMinClusterSize(min_size);
            ec.setMaxClusterSize(max_size);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud_in);
            ec.extract(cluster_indices);

            for (const auto& indices : cluster_indices) {
                PointCloudPtr cluster(new PointCloudT);
                for (const auto& index : indices.indices) {
                    cluster->push_back((*cloud_in)[index]);
                }
                cluster->width = cluster->size();
                cluster->height = 1;
                cluster->is_dense = true;
                clusters.push_back(cluster);
            }
        } catch (const std::exception& e) {
            LOG_ERROR(  "euclideanClusterExtraction函数中发生异常: %s", e.what());
            return clusters;
        }

        return clusters;
    }

    /**
     * @brief 基于法向量特征的区域生长分割算法
     * @param cloud_in 输入点云
     * @param clusters 输出聚类索引
     * @return int 成功返回聚类数量，失败返回-1
     */
    int regionGrowingSegmentation(const PointCloudPtr& cloud_in, std::vector<pcl::PointIndices>& clusters)
    {
        try {
            // 3. Algorithm setup
            pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
            normal_estimator.setSearchMethod(tree);
            normal_estimator.setInputCloud(cloud_in);
            normal_estimator.setKSearch(50); // Use 50 neighbors for normal estimation
            normal_estimator.compute(*normals);

            // 4. Region growing segmentation
            pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
            reg.setMinClusterSize(200);      // Minimum cluster size
            reg.setMaxClusterSize(1000000); // Maximum cluster size
            reg.setSearchMethod(tree);      // Set search method
            reg.setNumberOfNeighbours(30);  // Number of neighbors for smoothness evaluation
            reg.setInputCloud(cloud_in);       // Set input cloud
            reg.setInputNormals(normals);   // Set normal data

            // Set smoothness and curvature thresholds
            reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI); // Smoothness threshold in radians
            reg.setCurvatureThreshold(1.0);// Curvature threshold

            // Perform segmentation
            reg.extract(clusters);

            //std::cout << "Number of clusters found: " << clusters.size() << std::endl;
        } catch (const std::exception& e) {
            LOG_ERROR(  "regionGrowingSegmentation函数中发生异常: %s", e.what());
            return -1;
        }

        return 1;
    }

    /**
     * @brief 条件欧几里得聚类分割算法
     * @param cloud_in 输入点云
     * @param tolerance 聚类容差
     * @param min_size 最小聚类尺寸
     * @param max_size 最大聚类尺寸
     * @return std::vector<PointCloudPtr> 分割后的点云集合
     */
    std::vector<PointCloudPtr> conditionalEuclideanClustering(const PointCloudPtr& cloud_in, float tolerance, int min_size, int max_size)
    {
        std::vector<PointCloudPtr> results;
        try {
            // Create conditional clustering object
            pcl::ConditionalEuclideanClustering<PointT> cec;
            cec.setInputCloud(cloud_in);
            cec.setClusterTolerance(tolerance);
            cec.setMinClusterSize(min_size);
            cec.setMaxClusterSize(max_size);

            std::vector<pcl::PointIndices> clusters;
            cec.segment(clusters);

            for (const auto& cluster : clusters) {
                PointCloudPtr cloud_cluster(new PointCloudT);
                for (const auto& index : cluster.indices) {
                    cloud_cluster->push_back((*cloud_in)[index]);
                }
                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                results.push_back(cloud_cluster);
            }
        } catch (const std::exception& e) {
            LOG_ERROR(  "conditionalEuclideanClustering函数中发生异常: %s", e.what());
            return results;
        }

        return results;
    }
}