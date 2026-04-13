#ifndef PCL_PROC_SAMPLE_H
#define PCL_PROC_SAMPLE_H

#include"pcl_proc_base.h"

namespace PclProc
{
	/**
	 * @brief 球体拟合的随机采样一致性算法
	 * @param cloud_in 输入点云
	 * @return PointCloudPtr 返回拟合后的点云
	 */
	PointCloudPtr randomSampleConsensusSphere(const PointCloudPtr& cloud_in );
	/**
	 * @brief 平面拟合的随机采样一致性算法
	 * @param cloud_in 输入点云
	 * @return PointCloudPtr 返回拟合后的点云
	 */
	PointCloudPtr randomSampleConsensusPlane(const PointCloudPtr& cloud_in );

}

#endif