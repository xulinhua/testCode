// PCL相关定义、类型定义、接口定义
#ifndef PCL_PROC_BASE_H
#define PCL_PROC_BASE_H

//#include"PclExternLib.h"

#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh_omp.h> 
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>

// Include log system
#include "log_system/log_macros.hpp"

namespace PclProc
{
    // 通用定义
	using PointT = pcl::PointXYZ;
	using PointCloudT = pcl::PointCloud<PointT>;
	using PointCloudPtr = PointCloudT::Ptr;

	using PointXYZI = pcl::PointXYZI;
	using PointXYZICloud = pcl::PointCloud<PointXYZI>;
	using PointXYZICloudPtr = PointXYZICloud::Ptr;

	using PointNT = pcl::PointNormal;
	using PointCloudNT = pcl::PointCloud<PointNT>;
	using PointCloudNTPtr = PointCloudNT::Ptr;

	using FeatureT = pcl::FPFHSignature33;
	using FeatureEstimationT = pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT>;
	using FeatureCloudT = pcl::PointCloud<FeatureT>;
	using FeatureCloudTPtr = FeatureCloudT::Ptr;

    // 通用接口

}

#endif