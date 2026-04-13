
#include "../include/pcl_proc/pcl_proc_sample.h"
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>

namespace PclProc
{
/**
 * @brief 球体拟合的随机采样一致性算法
 * @param cloud_in 输入点云
 * @return PointCloudPtr 返回拟合后的点云
 */
	PointCloudPtr randomSampleConsensusSphere(const PointCloudPtr& cloud_in)
	{
        try {
		    std::vector<int> inliers;
		    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
			    model_s(new pcl::SampleConsensusModelSphere<PointT>(cloud_in));

		    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
		    ransac.setDistanceThreshold(.01);
		    ransac.computeModel();
		    ransac.getInliers(inliers);

		    PointCloudPtr result(new PointCloudT);
		    pcl::copyPointCloud<pcl::PointXYZ>(*cloud_in, inliers, *result);
		    return result;
        } catch (const std::exception& e) {
            LOG_ERROR(  "randomSampleConsensusSphere函数中发生异常: %s", e.what());
            return PointCloudPtr(new PointCloudT);
        }
	}
	
/**
 * @brief 平面拟合的随机采样一致性算法
 * @param cloud_in 输入点云
 * @return PointCloudPtr 返回拟合后的点云
 */
	PointCloudPtr randomSampleConsensusPlane(const PointCloudPtr& cloud_in)
	{
        try {
		    std::vector<int> inliers;
		    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
			    model_p(new pcl::SampleConsensusModelPlane<PointT>(cloud_in));

		    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
		    ransac.setDistanceThreshold(.01);
		    ransac.computeModel();
		    ransac.getInliers(inliers);

		    PointCloudPtr result(new PointCloudT);
		    pcl::copyPointCloud<pcl::PointXYZ>(*cloud_in, inliers, *result);
		    return result;
        } catch (const std::exception& e) {
            LOG_ERROR(  "randomSampleConsensusPlane函数中发生异常: %s", e.what());
            return PointCloudPtr(new PointCloudT);
        }
	}
}