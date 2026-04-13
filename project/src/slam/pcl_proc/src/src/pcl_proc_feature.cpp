#include "../include/pcl_proc/pcl_proc_feature.h"
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/range_image/range_image_planar.h>
namespace PclProc
{
	/**
	 * @brief 计算法向量（包含曲率计算）
	 * @param cloud_src 输入点云
	 * @param k 邻域点数量
	 * @param normals 输出法向量
	 * @return bool 计算成功返回true，否则返回false
	 */
	bool computeNormal(const PointCloudPtr& cloud_src, int k, pcl::PointCloud<pcl::Normal>::Ptr& normals)
	{
        try {
		    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		    ne.setInputCloud(cloud_src);
		    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		    ne.setSearchMethod(tree);
		    ne.setKSearch(k);
		    ne.compute(*normals);
        } catch (const std::exception& e) {
            LOG_ERROR( "computeNormal函数中发生异常: %s", e.what());
            return false;
        }

		return true;
	}

	/**
	 * @brief 使用积分图像对有序点云进行法向量估计
	 * @param cloud_src 输入点云
	 * @param normals 输出法向量
	 * @param radius 搜索半径
	 * @param idx_percent 索引百分比
	 * @return bool 计算成功返回true，否则返回false
	 */
	bool computeNormalEstimationUsingIntegralImg(const PointCloudPtr& cloud_src, pcl::PointCloud<pcl::Normal>::Ptr& normals, double radius,double idx_percent)
	{
        try {
		    // Limit the input index percentage
		    if (idx_percent< 1.0/ cloud_src->size())
	        {
			    idx_percent = 1.0 / cloud_src->size();
		    }
		    else if(idx_percent > 0.9 )
	        {
			    idx_percent = 0.9;
		    }
		    // Create a set of indices to use.
		    std::vector<int> indices(std::floor(cloud_src->size() * idx_percent));
		    for (std::size_t i = 0; i < indices.size(); ++i) 
			    indices[i] = i;

		    // Create the NormalEstimation class and pass the input dataset to it
		    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		    ne.setInputCloud(cloud_src);

		    // Pass the indices
		    pcl::shared_ptr<std::vector<int> > indicesptr(new std::vector<int>(indices));
		    ne.setIndices(indicesptr);

		    // Create an empty kdtree representation and pass it to the normal estimation object
		    // Its content will be filled based on the given input dataset inside the object
		    // (because no other search surface is provided).
		    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		    ne.setSearchMethod(tree);
		
		    // Search for neighbors within the input radius
		    ne.setRadiusSearch(radius);
		    ne.compute(*normals);
        } catch (const std::exception& e) {
            LOG_ERROR(  "computeNormalEstimationUsingIntegralImg函数中发生异常: %s", e.what());
            return false;
        }

		return true;
	}

	/**
	 * @brief 快速点特征直方图(FPFH) - 3D特征描述子
	 * @param cloud_src 输入点云
	 * @param fpfhs 输出FPFH特征
	 * @param normal_radius 法向量计算半径
	 * @param fpfh_radius FPFH计算半径
	 * @return bool 计算成功返回true，否则返回false
	 */
	bool computeFPFH(const PointCloudPtr& cloud_src, FeatureCloudTPtr& fpfhs,float normal_radius,float fpfh_radius)
	{
        try {
		    if (cloud_src->empty()) {
                LOG_ERROR(  "computeFPFH输入点云为空");
                return false;
            }
		
		    // 1. Compute normals
		    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

		    normal_estimator.setInputCloud(cloud_src);
		    normal_estimator.setSearchMethod(tree);
		    normal_estimator.setRadiusSearch(normal_radius);
		    normal_estimator.compute(*normals);

		    // 2. Compute FPFH features
		    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;

		    fpfh.setInputCloud(cloud_src);
		    fpfh.setInputNormals(normals);
		    fpfh.setSearchMethod(tree);
		    fpfh.setRadiusSearch(fpfh_radius);

		    fpfh.compute(*fpfhs);
        } catch (const std::exception& e) {
            LOG_ERROR(  "computeFPFH函数中发生异常: %s", e.what());
            return false;
        }

		return true;
	}

	/**
	 * @brief 视角特征直方图(VFH)
	 * @param cloud 输入点云
	 * @param vfh 输出VFH特征
	 * @param normal_radius 法向量计算半径
	 * @return bool 计算成功返回true，否则返回false
	 */
	bool computeVFH(const PointCloudPtr& cloud, pcl::PointCloud<pcl::VFHSignature308>::Ptr& vfh, float normal_radius)
	{
        try {
		    if (cloud->empty()) {
                LOG_ERROR(  "computeVFH输入点云为空");
                return false;
            }
		
		    // 1. Compute normals
		    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

		    normal_estimator.setInputCloud(cloud);
		    normal_estimator.setSearchMethod(tree);
		    normal_estimator.setRadiusSearch(normal_radius);
		    normal_estimator.compute(*normals);

		    // 2. Create point cloud with normals
		    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
		    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

		    // 3. Compute VFH features
		    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh_estimator;

		    vfh_estimator.setInputCloud(cloud);
		    vfh_estimator.setInputNormals(normals);
		    vfh_estimator.setSearchMethod(tree);

		    // Set VFH parameters
		    vfh_estimator.setNormalizeBins(true);        // Normalize histogram bins
		    vfh_estimator.setNormalizeDistance(false);   // Do not normalize distance component
		    vfh_estimator.setFillSizeComponent(true);    // Include size component
		    vfh_estimator.compute(*vfh);
        } catch (const std::exception& e) {
            LOG_ERROR(  "computeVFH函数中发生异常: %s", e.what());
            return false;
        }

		return true;
	}

	/**
	 * @brief 从深度图像中提取NARF特征
	 * @param range_image 输入深度图像
	 * @param support_size 支持尺寸
	 * @return pcl::PointCloud<int>::Ptr 返回关键点索引
	 */
	pcl::PointCloud<int>::Ptr extractNARFKeypoints(const pcl::RangeImage::Ptr& range_image, float support_size)
	{
        try {
		    // Create border extractor
		    pcl::RangeImageBorderExtractor border_extractor;

		    // Create NARF keypoint detector
		    pcl::NarfKeypoint narf_keypoint_detector(&border_extractor);
		    narf_keypoint_detector.setRangeImage(range_image.get());

		    // Set keypoint detection parameters - corrected parameters
		    narf_keypoint_detector.setRadiusSearch(support_size);  // Use setRadius instead of setting parameters directly
		    // Other available parameter settings
		    // narf_keypoint_detector.setCheckForNaNs(true);
		    // narf_keypoint_detector.setCheckRange(true);

		    // Detect keypoints
		    pcl::PointCloud<int>::Ptr keypoints(new pcl::PointCloud<int>);
		    narf_keypoint_detector.compute(*keypoints);
		    return keypoints;
        } catch (const std::exception& e) {
            LOG_ERROR(  "extractNARFKeypoints函数中发生异常: %s", e.what());
            return pcl::PointCloud<int>::Ptr(new pcl::PointCloud<int>);
        }
	}

	/**
	 * @brief 计算NARF描述子
	 * @param range_image 输入深度图像
	 * @param keypoints 关键点
	 * @param support_size 支持尺寸
	 * @return pcl::PointCloud<pcl::Narf36>::Ptr 返回NARF描述子
	 */
	pcl::PointCloud<pcl::Narf36>::Ptr computeNARFDescriptors(const pcl::RangeImagePlanar::Ptr& range_image,
		const pcl::PointCloud<pcl::PointWithScale>::Ptr& keypoints, float support_size)
	{
        try {
		    // Create NARF descriptor extractor
		    std::vector<int> keypoints2;
		    pcl::NarfDescriptor narf_descriptor(range_image.get(), &keypoints2);

		    // Set descriptor parameters
		    narf_descriptor.getParameters().support_size = support_size;
		    narf_descriptor.getParameters().rotation_invariant = true;

		    // Compute descriptors
		    pcl::PointCloud<pcl::Narf36>::Ptr descriptors(new pcl::PointCloud<pcl::Narf36>);
		    narf_descriptor.compute(*descriptors);

		    return descriptors;
        } catch (const std::exception& e) {
            LOG_ERROR(  "computeNARFDescriptors函数中发生异常: %s", e.what());
            return pcl::PointCloud<pcl::Narf36>::Ptr(new pcl::PointCloud<pcl::Narf36>);
        }
	}
}