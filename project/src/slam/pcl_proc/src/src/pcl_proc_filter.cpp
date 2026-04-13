#include"../include/pcl_proc/pcl_proc_filter.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/extract_indices.h>
namespace PclProc
{
    /**
     * @brief 移除NaN点
     * @param cloud_in 输入点云
     * @return PointCloudPtr 移除NaN点后的点云
     */
    PointCloudPtr removeNaNFromPoint(const PointCloudPtr& cloud_in)
    {
		std::vector<int> indices;
		PointCloudPtr result(new PointCloudT);
        if (cloud_in->size() < 1)
        {
            //pcl::console::print_error("PclProc::removeNaNFromPoint input empty!");
            LOG_ERROR(  "removeNaNFromPoint接收到空点云");
            return result;
        }
        try {
		pcl::removeNaNFromPointCloud(*cloud_in, *result, indices);
        } catch (const std::exception& e) {
            LOG_ERROR(  "removeNaNFromPoint函数中发生异常: %s", e.what());
        }
		return result;
    }

    /**
     * @brief 无序点的标准体素网格滤波
     * @param cloud_in 输入点云
     * @param lx x方向体素大小
     * @param ly y方向体素大小
     * @param lz z方向体素大小
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr voxelGridFilter(const PointCloudPtr& cloud_in, float lx, float ly, float lz)
    {
        PointCloudPtr result(new PointCloudT);
        if (cloud_in->size()<1)
        {
            //pcl::console::print_error("PclProc::voxelGridFilter input empty!");
            LOG_ERROR(  "voxelGridFilter接收到空点云");
            return result;
        }
        try {
            pcl::VoxelGrid<PointT> filter;
            filter.setInputCloud(cloud_in);
            filter.setLeafSize(lx, ly, lz);
            filter.filter(*result);
        } catch (const std::exception& e) {
            LOG_ERROR(  "voxelGridFilter函数中发生异常: %s", e.what());
            return PointCloudPtr(new PointCloudT);
        }
        return result;
    }

    /**
     * @brief 近似体素网格滤波
     * @param cloud_in 输入点云
     * @param lx x方向体素大小
     * @param ly y方向体素大小
     * @param lz z方向体素大小
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr approxVoxelGridFilter(const PointCloudPtr& cloud_in, float lx, float ly, float lz)
    {
        PointCloudPtr result(new PointCloudT);
        if (cloud_in->size() < 1)
        {
            //pcl::console::print_error("PclProc::approxVoxelGridFilter input empty!");
            LOG_ERROR(  "approxVoxelGridFilter接收到空点云");
            return result;
        }
        try {
            pcl::ApproximateVoxelGrid<PointT> filter;
            filter.setInputCloud(cloud_in);
            filter.setLeafSize(lx, ly, lz);
            filter.filter(*result);
        } catch (const std::exception& e) {
            LOG_ERROR(  "approxVoxelGridFilter函数中发生异常: %s", e.what());
            return PointCloudPtr(new PointCloudT);
        }
        return result;
    }

    /**
     * @brief 统计异常值移除滤波
     * @param cloud_in 输入点云
     * @param mean_k 邻域点数量
     * @param stddev_mult 标准差倍数
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr statisticalOutlierRemoval(const PointCloudPtr& cloud_in, int mean_k, float stddev_mult)
    {
        PointCloudPtr result(new PointCloudT);
        if (cloud_in->size() < 1)
        {
            //pcl::console::print_error("PclProc::statisticalOutlierRemoval input empty!");
            LOG_ERROR(  "statisticalOutlierRemoval接收到空点云");
            return result;
        }
        try {
            pcl::StatisticalOutlierRemoval<PointT> filter;
            filter.setInputCloud(cloud_in);
            filter.setMeanK(mean_k);
            filter.setStddevMulThresh(stddev_mult);
            filter.filter(*result);
        } catch (const std::exception& e) {
            LOG_ERROR(  "statisticalOutlierRemoval函数中发生异常: %s", e.what());
            return PointCloudPtr(new PointCloudT);
        }
        return result;
    }

    /**
     * @brief 半径异常值移除滤波
     * @param cloud_in 输入点云
     * @param radius 搜索半径
     * @param min_neighbors 最小邻居数
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr radiusOutlierRemoval(const PointCloudPtr& cloud_in, float radius, int min_neighbors)
    {
        PointCloudPtr result(new PointCloudT);
        if (cloud_in->size() < 1)
        {
            //pcl::console::print_error("PclProc::radiusOutlierRemoval input empty!");
            LOG_ERROR(  "radiusOutlierRemoval接收到空点云");
            return result;
        }
        try {
            pcl::RadiusOutlierRemoval<PointT> filter;
            filter.setInputCloud(cloud_in);
            filter.setRadiusSearch(radius);
            filter.setMinNeighborsInRadius(min_neighbors);
            filter.filter(*result);
        } catch (const std::exception& e) {
            LOG_ERROR(  "radiusOutlierRemoval函数中发生异常: %s", e.what());
            return PointCloudPtr(new PointCloudT);
        }
        return result;
    }

    /**
     * @brief 直通滤波
     * @param cloud_in 输入点云
     * @param field_name 过滤字段名
     * @param min_limit 最小限制值
     * @param max_limit 最大限制值
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr passThroughFilter(const PointCloudPtr& cloud_in, const std::string& field_name,
        float min_limit, float max_limit)
    {
        PointCloudPtr result(new PointCloudT);
        if (cloud_in->size() < 1)
        {
            //pcl::console::print_error("PclProc::passThroughFilter input empty!");
            LOG_ERROR(  "passThroughFilter接收到空点云");
            return result;
        }
        try {
            pcl::PassThrough<PointT> filter;
            filter.setInputCloud(cloud_in);
            filter.setFilterFieldName(field_name);
            filter.setFilterLimits(min_limit, max_limit);
            filter.filter(*result);
        } catch (const std::exception& e) {
            LOG_ERROR(  "passThroughFilter函数中发生异常: %s", e.what());
            return PointCloudPtr(new PointCloudT);
        }
        return result;
    }

    /**
     * @brief 条件滤波
     * @param cloud_in 输入点云
     * @param field_name 过滤字段名
     * @param min_limit 最小限制值
     * @param max_limit 最大限制值
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr conditionalRemoval(const PointCloudPtr& cloud_in, const std::string& field_name,
        float min_limit, float max_limit)
    {
        PointCloudPtr result(new PointCloudT);
        if (cloud_in->size() < 1)
        {
            //pcl::console::print_error("PclProc::conditionalRemoval input empty!");
            LOG_ERROR(  "conditionalRemoval接收到空点云");
            return result;
        }
        try {
            pcl::ConditionAnd<PointT>::Ptr condition(new pcl::ConditionAnd<PointT>());
            condition->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
                new pcl::FieldComparison<PointT>(field_name, pcl::ComparisonOps::GT, min_limit)));
            condition->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
                new pcl::FieldComparison<PointT>(field_name, pcl::ComparisonOps::LT, max_limit)));

            pcl::ConditionalRemoval<PointT> filter;
            filter.setCondition(condition);
            filter.setInputCloud(cloud_in);
            filter.filter(*result);
        } catch (const std::exception& e) {
            LOG_ERROR(  "conditionalRemoval函数中发生异常: %s", e.what());
            return PointCloudPtr(new PointCloudT);
        }
        return result;
    }

    /**
     * @brief 双边滤波
     * @param cloud_in 输入点云
     * @param sigma_s 空间标准差
     * @param sigma_r 色彩标准差
     * @return PointXYZICloudPtr 滤波后的点云
     */
    PointXYZICloudPtr bilateralFilter(const PointXYZICloudPtr& cloud_in, float sigma_s, float sigma_r)
    {
        PointXYZICloudPtr result(new PointXYZICloud);
        if (cloud_in->size() < 1)
		{
			//pcl::console::print_error("PclProc::bilateralFilter input empty!");
			LOG_ERROR(  "bilateralFilter接收到空点云");
			return result;
		}
        try {
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
            pcl::BilateralFilter<pcl::PointXYZI> fbf;
            fbf.setInputCloud(cloud_in);
            fbf.setSearchMethod(tree1);
            fbf.setStdDev(sigma_r);
            fbf.setHalfSize(sigma_s);
            fbf.filter(*result);
        } catch (const std::exception& e) {
            LOG_ERROR(  "bilateralFilter函数中发生异常: %s", e.what());
            return PointXYZICloudPtr(new PointXYZICloud);
        }
        return result;
    }

    /**
     * @brief 快速双边滤波
     * @param cloud_in 输入点云
     * @param sigma_s 空间标准差
     * @param sigma_r 色彩标准差
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr fastBilateralFilter(const PointCloudPtr& cloud_in, float sigma_s, float sigma_r)
    {
        PointCloudPtr result(new PointCloudT);
        if (cloud_in->size() < 1)
        {
            //pcl::console::print_error("PclProc::fastBilateralFilter input empty!");
            LOG_ERROR(  "fastBilateralFilter接收到空点云");
            return result;
        }
        try {
            pcl::FastBilateralFilter<PointT> filter;
            filter.setInputCloud(cloud_in);
            filter.setSigmaS(sigma_s);
            filter.setSigmaR(sigma_r);
            filter.applyFilter(*result);
        } catch (const std::exception& e) {
            LOG_ERROR(  "fastBilateralFilter函数中发生异常: %s", e.what());
            return PointCloudPtr(new PointCloudT);
        }
        return result;
    }

    /**
     * @brief 中值滤波
     * @param cloud_in 输入点云
     * @param window_size 窗口大小
     * @param max_movement 最大移动距离
     * @return PointCloudPtr 滤波后的点云
     */
    PointCloudPtr medianFilter(const PointCloudPtr& cloud_in, int window_size, float max_movement)
    {
        PointCloudPtr result(new PointCloudT);
        if (cloud_in->size() < 1)
        {
            //pcl::console::print_error("PclProc::medianFilter input empty!");
            LOG_ERROR(  "medianFilter接收到空点云");
            return result;
        }
        try {
            pcl::MedianFilter<PointT> filter;
            filter.setInputCloud(cloud_in);
            filter.setWindowSize(window_size);
            filter.setMaxAllowedMovement(max_movement);
            filter.applyFilter(*result);
        } catch (const std::exception& e) {
            LOG_ERROR(  "medianFilter函数中发生异常: %s", e.what());
            return PointCloudPtr(new PointCloudT);
        }
        return result;
    }

    /**
     * @brief 均匀采样
     * @param cloud_in 输入点云
     * @param radius 搜索半径
     * @return PointCloudPtr 采样后的点云
     */
    PointCloudPtr uniformSampling(const PointCloudPtr& cloud_in, double radius)
    {
        PointCloudPtr result(new PointCloudT);
        if (cloud_in->size() < 1)
        {
            //pcl::console::print_error("PclProc::uniformSampling input empty!");
            LOG_ERROR(  "uniformSampling接收到空点云");
            return result;
        }
        try {
            pcl::UniformSampling<PointT> uniform_sampling;
            uniform_sampling.setInputCloud(cloud_in);
            uniform_sampling.setRadiusSearch(radius);
            uniform_sampling.filter(*result);
        } catch (const std::exception& e) {
            LOG_ERROR(  "uniformSampling函数中发生异常: %s", e.what());
            return PointCloudPtr(new PointCloudT);
        }
        return result;
    }
}