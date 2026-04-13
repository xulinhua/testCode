#ifndef PCL_PROC_REGISTRATION_H
#define PCL_PROC_REGISTRATION_H

#include"pcl_proc_base.h"
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>

namespace PclProc
{
    
	/**
	 * @brief 使用ICP进行点对点配准
	 * @param cloud_src 源点云
	 * @param cloud_tgt 目标点云
	 * @param output 输出点云
	 * @param final_transform 输出变换矩阵
	 * @param downsample 是否降采样，默认false
	 * @return bool 配准成功返回true，否则返回false
	 */
	bool pairAlignICP(const PointCloudPtr& cloud_src, const PointCloudPtr& cloud_tgt, PointCloudPtr& output, Eigen::Matrix4f& final_transform, bool downsample = false);
    /**
     * @brief 多点云的ICP配准
     * @param cloud_src 源点云向量
     * @param cloud_rst 输出点云
     * @return bool 配准成功返回true，否则返回false
     */
    bool registrationICP(const std::vector<PointCloudPtr>& cloud_src, PointCloudPtr &cloud_rst);
    /**
     * @brief GICP配准
     * @param cloud_src 源点云
     * @param cloud_tgt 目标点云
     * @param cloud_rst 输出点云
     * @return bool 配准成功返回true，否则返回false
     */
    bool registrationGICP(const PointCloudPtr& cloud_src, const PointCloudPtr& cloud_tgt, PointCloudPtr &cloud_rst);

    /**
     * @brief 正态分布变换(NDT)配准
     * @param cloud_src 源点云
     * @param cloud_tgt 目标点云
     * @param cloud_rst 输出点云
     * @return bool 配准成功返回true，否则返回false
     */
    bool registrationNDT(const PointCloudPtr& cloud_src, const PointCloudPtr& cloud_tgt, PointCloudPtr& cloud_rst);
    
    /**
     * @brief 鲁棒姿态估计
     * @param object 物体点云
     * @param scene 场景点云
     * @param object_aligned 输出对齐后的物体点云
     * @return bool 估计成功返回true，否则返回false
     */
    bool registrationRobustPose(const PointCloudNTPtr& object, const PointCloudNTPtr& scene, PointCloudNTPtr& object_aligned);
    /**
     * @brief 鲁棒姿态估计
     * @param object 物体点云
     * @param scene 场景点云
     * @param cloud_rst 输出点云
     * @param nor_radius 法向量计算半径
     * @return bool 估计成功返回true，否则返回false
     */
    bool registrationRobustPose(const PointCloudPtr& object, const PointCloudPtr& scene, PointCloudPtr& cloud_rst,double nor_radius);

    /**
     * @brief 计算原始点云与配准后点云之间的均方误差(MSE)
     * @param modelPointCloud 模型点云
     * @param queryPointCloud 查询点云
     * @param dist 距离阈值
     * @return double 返回MSE值
     */
    double getMSE(const PointCloudPtr& modelPointCloud, const PointCloudPtr& queryPointCloud, double dist);
    /**
     * @brief 计算原始点云与配准后点云之间的均方根误差(RMSE)
     * @param cloudA 点云A
     * @param cloudB 点云B
     * @param rmse RMSE值
     * @param max_range 最大范围
     */
    void getRMSE(const PointCloudPtr& cloudA, const PointCloudPtr& cloudB, double rmse, double max_range);


    // 定义<x, y, z, 曲率>格式的新点表示
    class PointRepresentation4Dims : public pcl::PointRepresentation<PointNT>
    {
        using pcl::PointRepresentation<PointNT>::nr_dimensions_;

    public:
        PointRepresentation4Dims()
        {
            nr_dimensions_ = 4; // 设置维度数
        }
        // 重写copyToFloatArray方法以转换为4维数组
        virtual void copyToFloatArray(const PointNT& p, float* out) const
        {
            // < x, y, z, 曲率>
            out[0] = p.x;
            out[1] = p.y;
            out[2] = p.z;
            out[3] = p.curvature; // 曲率
        }
    };
    // GICP registration
    // Registration result structure
    struct FrameResult {
        PointCloudT::Ptr cloud; // Points transformed to global coordinate system
        Eigen::Matrix4f pose;   // Pose of this frame relative to initial frame
    };

    // Multi-frame GICP registration
    class CMultiFrameRegistGICP 
    {
    public:
        CMultiFrameRegistGICP();
       virtual ~CMultiFrameRegistGICP();

       PointCloudPtr GetGlobalMap() const;
        // Process a new frame of point cloud
       Eigen::Matrix4f processFrame(const PointCloudT::Ptr& new_cloud);

        // Downsample cloud and update accumulated map
       void downsampleCloud(const PointCloudT::Ptr& input, PointCloudT::Ptr& output);
       
        // Update global map - cumulative addition
       void updateGlobalMap(const PointCloudT::Ptr& new_cloud, const Eigen::Matrix4f& pose)
   ;
       // Get trajectory of all frames
       std::vector<Eigen::Matrix4f> getTrajectory() const ;

    private:
        // PCL objects
        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp_;
        pcl::VoxelGrid<PointT> voxel_filter_;

        // Data
        PointCloudT::Ptr global_map_;
        std::vector<FrameResult> frame_results_;

    };

}

#endif