#ifndef PCL_PROC_FEATURE_H
#define PCL_PROC_FEATURE_H

#include"pcl_proc_base.h"
#include <pcl/range_image/range_image.h>
#include"pcl/range_image/range_image_planar.h"
namespace PclProc
{
	/**
	 * @brief 计算法向量（包含曲率计算）
	 * @param cloud_src 输入点云
	 * @param k 邻域点数量
	 * @param normals 输出法向量
	 * @return bool 计算成功返回true，否则返回false
	 */
	bool computeNormal(const PointCloudPtr& cloud_src, int k, pcl::PointCloud<pcl::Normal>::Ptr& normals);
	
	/**
	 * @brief 使用积分图像对有序点云进行法向量估计
	 * @param cloud_src 输入点云
	 * @param normals 输出法向量
	 * @param radius 搜索半径，默认0.05
	 * @param idx_percent 索引百分比，默认0.1
	 * @return bool 计算成功返回true，否则返回false
	 */
	bool computeNormalEstimationUsingIntegralImg(const PointCloudPtr& cloud_src, pcl::PointCloud<pcl::Normal>::Ptr& normals, double radius=0.05, double idx_percent=0.1);

	/**
	 * @brief 快速点特征直方图(FPFH) - 3D特征描述子
	 * @param cloud_src 输入点云
	 * @param fpfhs 输出FPFH特征
	 * @param normal_radius 法向量计算半径，默认0.05
	 * @param fpfh_radius FPFH计算半径，默认0.1
	 * @return bool 计算成功返回true，否则返回false
	 */
	bool computeFPFH(const PointCloudPtr& cloud_src, FeatureCloudTPtr& fpfhs,float normal_radius = 0.05,float fpfh_radius = 0.1);
	
	/**
	 * @brief 视角特征直方图(VFH)
	 * @param cloud 输入点云
	 * @param vfh 输出VFH特征
	 * @param normal_radius 法向量计算半径，默认0.05
	 * @return bool 计算成功返回true，否则返回false
	 */
	bool computeVFH(const PointCloudPtr& cloud, pcl::PointCloud<pcl::VFHSignature308>::Ptr& vfh, float normal_radius = 0.05);

	/**
	 * @brief 从深度图像中提取NARF特征
	 * @param range_image 输入深度图像
	 * @param support_size 支持尺寸，默认0.2f
	 * @return pcl::PointCloud<int>::Ptr 返回关键点索引
	 */
	pcl::PointCloud<int>::Ptr extractNARFKeypoints(const pcl::RangeImage::Ptr& range_image, float support_size = 0.2f);
	
	/**
	 * @brief 计算NARF描述子
	 * @param range_image 输入深度图像
	 * @param keypoints 关键点
	 * @param support_size 支持尺寸，默认0.2f
	 * @return pcl::PointCloud<pcl::Narf36>::Ptr 返回NARF描述子
	 */
	pcl::PointCloud<pcl::Narf36>::Ptr computeNARFDescriptors(const pcl::RangeImagePlanar::Ptr& range_image,
		const pcl::PointCloud<pcl::PointWithScale>::Ptr& keypoints, float support_size = 0.2f);

	// 基于惯性矩和离心率的描述子


	// 旋转投影统计特征


	// 全局一致空间分布描述子


}

#endif