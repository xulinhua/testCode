#ifndef PCL_PROC_IO_H
#define PCL_PROC_IO_H

#include"pcl_proc_base.h"
#include <pcl/range_image/range_image.h>

namespace PclProc
{

	/**
	 * @brief 加载点云
	 * @param cloud_Input 输出点云
	 * @param filename 文件名
	 * @return bool 加载成功返回true，否则返回false
	 */
	bool loadPointCloud(PointCloudPtr& cloud_Input, const std::string& filename);
	/**
	 * @brief 加载多个点云
	 * @param clouds 输出点云向量
	 * @param filenames 文件名向量
	 * @return bool 加载成功返回true，否则返回false
	 */
	bool loadPointClouds(std::vector<PointCloudPtr>& clouds, const std::vector<std::string> filenames);

	/**
	 * @brief 保存点云
	 * @param cloud_Input 输入点云
	 * @param filename 文件名
	 * @param bBinary 是否以二进制格式保存，默认true
	 * @return bool 保存成功返回true，否则返回false
	 */
	bool savePointCloud(PointCloudPtr cloud_Input, const std::string& filename, bool bBinary = true);
	/**
	 * @brief 保存法向量值
	 * @param normals 输入法向量
	 * @param filename 文件名
	 * @param bBinary 是否以二进制格式保存，默认true
	 * @return bool 保存成功返回true，否则返回false
	 */
	bool saveNormals(pcl::PointCloud<pcl::Normal>::Ptr normals, const std::string& filename, bool bBinary = true);
	/**
	 * @brief 保存NARF特征
	 * @param keypoints 关键点
	 * @param descriptors 描述子
	 * @param filename 文件名
	 */
	void saveNARFFeatures(const pcl::PointCloud<pcl::PointWithScale>::Ptr& keypoints, const pcl::PointCloud<pcl::Narf36>::Ptr& descriptors, const std::string& filename);

	/**
	 * @brief 将pcd转换为深度图像
	 * @param cloud 输入点云
	 * @param angular_resolution 角度分辨率，默认0.5f
	 * @return pcl::RangeImage::Ptr 返回深度图像
	 */
	pcl::RangeImage::Ptr createRangeImageFromPointCloud(const PointCloudPtr& cloud, float angular_resolution = 0.5f);
	/**
	 * @brief 将PointXYZ转换为PointNormal（不进行法向量估计）
	 * @param input 输入点云
	 * @param output 输出点云
	 * @param initialize_normals 是否初始化法向量，默认true
	 */
	void convertXYZToPointNormal(const PointCloudPtr& input, PointCloudNTPtr& output, bool initialize_normals = true);
	/**
	 * @brief 将PointXYZ转换为PointNormal（进行法向量估计）
	 * @param input 输入点云
	 * @param output 输出点云
	 * @param radius 搜索半径，默认0.03f
	 */
	void convertXYZToPointNormalWithNormals(const PointCloudPtr& input, PointCloudNTPtr& output, float radius = 0.03f);


}

#endif