#include"../include/pcl_proc/pcl_proc_registration.h"

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/feature.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "../include/pcl_proc/pcl_proc_io.h"
#include "../include/pcl_proc/pcl_proc_filter.h"
namespace PclProc
{
    /**
     * @brief 使用ICP进行点对点配准
     * @param cloud_src 源点云
     * @param cloud_tgt 目标点云
     * @param output 输出点云
     * @param final_transform 输出变换矩阵
     * @param downsample 是否降采样
     * @return bool 配准成功返回true，否则返回false
     */
    bool pairAlignICP(const PointCloudPtr& cloud_src, const PointCloudPtr& cloud_tgt, PointCloudPtr& output, Eigen::Matrix4f& final_transform, bool downsample)
    {
        try {
            PointCloudPtr src(new PointCloudT); // Store filtered source points
            PointCloudPtr tgt(new PointCloudT); // Store filtered target points
            if (downsample)
            {// Downsample - voxel grid filtering
                src = voxelGridFilter(cloud_src, 0.03, 0.03, 0.03);
                tgt = voxelGridFilter(cloud_tgt, 0.03, 0.03, 0.03);
            }
            else
            {
                src = cloud_src;
                tgt = cloud_tgt;
            }
            // Calculate normals for both point clouds
            PointCloudNT::Ptr points_with_normals_src(new PointCloudNT);
            PointCloudNT::Ptr points_with_normals_tgt(new PointCloudNT);

            pcl::NormalEstimation<PointT, PointNT> norm_est; // Normal estimation
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            norm_est.setSearchMethod(tree);
            norm_est.setKSearch(30);

            norm_est.setInputCloud(src);
            norm_est.compute(*points_with_normals_src);
            pcl::copyPointCloud(*src, *points_with_normals_src);

            norm_est.setInputCloud(tgt);
            norm_est.compute(*points_with_normals_tgt);
            pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

            //
            // Instantiate our custom point representation (defined above) ...
            PointRepresentation4Dims point_representation;
            // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
            float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
            point_representation.setRescaleValues(alpha);

            //
            // Registration
            pcl::IterativeClosestPointNonLinear<PointNT, PointNT> reg; // Registration object
            reg.setTransformationEpsilon(1e-6);// Transformation epsilon threshold
            // Set correspondence distance threshold, smaller values mean higher accuracy but slower computation
            reg.setMaxCorrespondenceDistance(0.1);// Maximum correspondence distance
            //reg.setEuclideanFitnessEpsilon(1e-6);    // Euclidean fitness epsilon threshold // Correct?
            // Set point representation
            reg.setPointRepresentation(std::make_shared<PointRepresentation4Dims>(point_representation));

            reg.setInputSource(points_with_normals_src); // Set source points
            reg.setInputTarget(points_with_normals_tgt); // Set target points
            //
            // Run the same optimization in a loop and visualize the results
            Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
            PointCloudNT::Ptr reg_result = points_with_normals_src;
            reg.setMaximumIterations(2); //// Set maximum iterations for each registration step
            for (int i = 0; i < 30; ++i) // Manual iteration
            {
                // Store intermediate alignment result
                points_with_normals_src = reg_result;

                // Estimate
                reg.setInputSource(points_with_normals_src);
                reg.align(*reg_result);

                //accumulate transformation between each Iteration
                Ti = reg.getFinalTransformation() * Ti;

                //if the difference between this transformation and the previous one
                //is smaller than the threshold, refine the process by reducing
                //the maximal correspondence distance
                if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
                    reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

                prev = reg.getLastIncrementalTransformation();

            }

            targetToSource = Ti.inverse();

            pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

            *output += *cloud_src;
            final_transform = targetToSource;
        } catch (const std::exception& e) {
            LOG_ERROR(  "pairAlignICP函数发生异常: %s", e.what());
            return false;
        }

        return true;
    }

    /**
     * @brief 多点云的ICP配准
     * @param cloud_src 源点云向量
     * @param cloud_rst 输出点云
     * @return bool 配准成功返回true，否则返回false
     */
    bool registrationICP(const std::vector<PointCloudPtr>& cloud_src, PointCloudPtr& cloud_rst)
    {
        try {
            if (cloud_src.size()<2) {
                LOG_ERROR(  "配准需要至少两个点云，但只提供了 %lu 个", cloud_src.size());
                return false;// 配准需要至少两个点云
            }

            PointCloudPtr result(new PointCloudT), source, target;
            Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
            for (size_t i = 1; i < cloud_src.size(); ++i)
            {
                 source = cloud_src[i - 1]; // Previous scan for registration
                 target = cloud_src[i];// Current scan for registration
                
                // Register adjacent scans and transform the result to global coordinate system
                PointCloudPtr temp(new PointCloudT);
                // pairTransform stores the transformation from target to source
                bool alignSuccess = pairAlignICP(source, target, temp, pairTransform, true);
                if (!alignSuccess) {
                    LOG_ERROR(  "点云对 %lu 的pairAlignICP配准失败", i);
                    return false;
                }
                // Transform the current registered point cloud temp to global coordinate system and store in result
                pcl::transformPointCloud(*temp, *result, GlobalTransform);

                // Get the transformation between current pair and accumulate global transformation
                GlobalTransform = GlobalTransform * pairTransform;
                
            }
            *cloud_rst=*result;
        } catch (const std::exception& e) {
            LOG_ERROR(  "registrationICP函数发生异常: %s", e.what());
            return false;
        }

        return true;
    }

    /**
     * @brief GICP配准
     * @param cloud_src 源点云
     * @param cloud_tgt 目标点云
     * @param cloud_rst 输出点云
     * @return bool 配准成功返回true，否则返回false
     */
    bool registrationGICP(const PointCloudPtr& cloud_src, const PointCloudPtr& cloud_tgt, PointCloudPtr& cloud_rst)
    {
        try {
            // Preprocessing with downsampling
            PointCloudPtr filtered_target(new PointCloudT);
            PointCloudPtr filtered_source(new PointCloudT);

            filtered_target = voxelGridFilter(cloud_tgt, 0.05f, 0.05f, 0.05f);
            filtered_source = voxelGridFilter(cloud_src, 0.05f, 0.05f, 0.05f);
            //filtered_target = removeNaNFromPoint(filtered_target);
            //filtered_source = removeNaNFromPoint(filtered_source);
            // Create GICP object
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
            //gicp.setNumberOfThreads(4);  // Not sure about this parameter, commented out
            // Detailed parameter settings
            gicp.setTransformationEpsilon(1e-8);// Fine-tune transformation epsilon
            gicp.setMaximumIterations(10);// Maximum iterations
            gicp.setMaxCorrespondenceDistance(0.5);// Minimum correspondence distance
            gicp.setEuclideanFitnessEpsilon(0.01);// Fine-tune correspondence fitness epsilon
            gicp.setRANSACIterations(5);// Use RANSAC to remove outliers

            // Set search methods
            /*gicp.setSearchMethodTarget(pcl::search::KdTree<pcl::PointXYZ>::Ptr(
                new pcl::search::KdTree<pcl::PointXYZ>), true);
            gicp.setSearchMethodSource(pcl::search::KdTree<pcl::PointXYZ>::Ptr(
                new pcl::search::KdTree<pcl::PointXYZ>), true);*/

            // Set covariance parameters
            gicp.setRotationEpsilon(1e-6);// Rotation epsilon threshold
            gicp.setCorrespondenceRandomness(20);// Correspondence randomness

            // Set input data
            gicp.setInputTarget(filtered_target);
            gicp.setInputSource(filtered_source);

            // Perform registration
            gicp.align(*cloud_rst);
        } catch (const std::exception& e) {
            LOG_ERROR(  "registrationGICP函数发生异常: %s", e.what());
            return false;
        }
        
        return true;
    }

    /**
     * @brief 正态分布变换(NDT)配准
     * @param cloud_src 源点云
     * @param cloud_tgt 目标点云
     * @param cloud_rst 输出点云
     * @return bool 配准成功返回true，否则返回false
     */
    bool registrationNDT(const PointCloudPtr& cloud_src, const PointCloudPtr& cloud_tgt, PointCloudPtr& cloud_rst)
    {
        try {
            // Downsample the input scan data by 10% to improve matching speed. Only the source point cloud needs filtering, target doesn't need filtering
            // Because the NDT algorithm calculates the target point cloud's movement correspondence based on statistical information, it doesn't use nearest neighbor search, so it uses statistical counts in each voxel grid cell
            PointCloudPtr filtered_cloud(new PointCloudT);
            filtered_cloud=approxVoxelGridFilter(cloud_src, 0.2, 0.2, 0.2);
            // Initialize Normal Distributions Transform (NDT) object
            pcl::NormalDistributionsTransform<PointT, PointT> ndt;

            // Set resolution and NDT related parameters
            ndt.setTransformationEpsilon(0.01);// For stopping criteria with small transformations
            ndt.setStepSize(0.1);    // For more-thuente line search maximum step
            ndt.setResolution(1.0);   // Set resolution of NDT data structure (voxelgridcovariance)
            // Using larger resolution values works better in most cases, but if there's a large difference in density between scans, it needs to be adjusted to a smaller value

            // Set matching iteration parameters to stop when fitness reaches a certain threshold before epsilon transformation stops
            // This prevents the algorithm from getting stuck in local minima during optimization
            ndt.setMaximumIterations(35);

            ndt.setInputSource(filtered_cloud);  // Source points
            // Setting point cloud to be aligned to.
            ndt.setInputTarget(cloud_tgt);  // Target points

            // Set initial guess for the transformation
            Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
            Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
            Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

            // Perform registration to align source points to target points
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            ndt.align(*output_cloud, init_guess);
            // Copy output_cloud to output_cloud and set the final transformation to the output_cloud
            /*std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
                << " score: " << ndt.getFitnessScore() << std::endl;*/

            // Use the final transformation as the final transformation
            pcl::transformPointCloud(*cloud_src, *cloud_rst, ndt.getFinalTransformation());
        } catch (const std::exception& e) {
            LOG_ERROR(  "registrationNDT函数发生异常: %s", e.what());
            return false;
        }

        return true;
    }

    /**
     * @brief 鲁棒姿态估计
     * @param object 物体点云
     * @param scene 场景点云
     * @param object_aligned 输出对齐后的物体点云
     * @return bool 估计成功返回true，否则返回false
     */
    bool registrationRobustPose(const PointCloudNTPtr& object, const PointCloudNTPtr& scene, PointCloudNTPtr& object_aligned)
    {
        try {
            FeatureCloudT::Ptr object_features(new FeatureCloudT);
            FeatureCloudT::Ptr scene_features(new FeatureCloudT);

            // 1 Downsample source and target point clouds using 0.005 voxel grid filtering
            // For faster interface, use PCL's pcl::VoxelGrid filter with 5 mm voxel grid size
            pcl::VoxelGrid<PointNT> grid;
            const float leaf = 0.005f;
            grid.setLeafSize(leaf, leaf, leaf);
            grid.setInputCloud(object);  // Set source point cloud
            grid.filter(*object);
            grid.setInputCloud(scene); // Set target point cloud
            grid.filter(*scene);

            // 2 Estimate normals for scene
            /*pcl::console::print_highlight("Estimating scene normals...\n");
            LOG_INFO_FMT("pcl_proc",  "Estimating scene normals...");*/
            pcl::NormalEstimationOMP<PointNT, PointNT> nest;
            nest.setNumberOfThreads(4);
            nest.setRadiusSearch(0.01);
            nest.setInputCloud(scene);
            nest.compute(*scene);

            // 3 Estimate features
            // For each point in the source point cloud, estimate features using PCL's pcl::FPFHEstimationOMP<> feature estimation method
            /*//pcl::console::print_highlight("Estimating features...\n");
            LOG_INFO_FMT("pcl_proc",  "Estimating features...");*/
            FeatureEstimationT fest;
            fest.setRadiusSearch(0.025);
            fest.setInputCloud(object);
            fest.setInputNormals(object);
            fest.compute(*object_features);
            fest.setInputCloud(scene);
            fest.setInputNormals(scene);
            fest.compute(*scene_features);

            // 4 Perform alignment  
            // SampleConsensusPrerejective implements robust RANSAC-based iterative algorithm
            //pcl::console::print_highlight("Starting alignment...\n");
            pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align; // // Create a new instance of the algorithm
            align.setInputSource(object); // Set source
            align.setSourceFeatures(object_features); // Set source features
            align.setInputTarget(scene);  // Set target
            align.setTargetFeatures(scene_features); //  Set target features
            align.setMaximumIterations(50000); // Set maximum number of RANSAC iterations
            align.setNumberOfSamples(3); // Set number of points to sample for generating/prerejecting a pose 
            align.setCorrespondenceRandomness(5); //  Set number of nearest features to use
            align.setSimilarityThreshold(0.9f); // Set polygonal edge length similarity threshold
            align.setMaxCorrespondenceDistance(2.5f * leaf); // Set inlier threshold
            align.setInlierFraction(0.25f); //  Set required inlier fraction for accepting a pose hypothesis
            align.align(*object_aligned); // Store the result in object_aligned
            
            if (align.hasConverged())
            {
                Eigen::Matrix4f transformation = align.getFinalTransformation();
            }
            else
            {// Registration failed
                //pcl::console::print_error("配准失败！\n");
                LOG_ERROR(  "配准失败！");
                return false;
            }
        } catch (const std::exception& e) {
            LOG_ERROR(  "registrationRobustPose函数发生异常: %s", e.what());
            return false;
        }
        return true;
    }

    /**
     * @brief 鲁棒姿态估计
     * @param object 物体点云
     * @param scene 场景点云
     * @param cloud_rst 输出点云
     * @param nor_radius 法向量计算半径
     * @return bool 估计成功返回true，否则返回false
     */
    bool registrationRobustPose(const PointCloudPtr& object, const PointCloudPtr& scene, PointCloudPtr& cloud_rst,double nor_radius)
    {
        try {
		    bool bRet = true;

		    PointCloudNTPtr object_normals(new PointCloudNT);
		    PointCloudNTPtr scene_normals(new PointCloudNT);
		    PointCloudNTPtr object_aligned(new PointCloudNT);
            
            // Convert PointXYZ to PointNormal
		    convertXYZToPointNormalWithNormals(object, object_normals, nor_radius);
		    convertXYZToPointNormalWithNormals(scene, scene_normals, nor_radius);
            // Registration
		    bRet = registrationRobustPose(object_normals, scene_normals, object_aligned);
            if (!bRet) {
                LOG_ERROR(  "registrationRobustPose with normals failed");
                return false;
            }
            // Convert PointNormal to PointXYZ
            pcl::copyPointCloud(*object_aligned, *cloud_rst);

            return bRet;
        } catch (const std::exception& e) {
            LOG_ERROR(  "registrationRobustPose函数发生异常: %s", e.what());
            return false;
        }
    }

    /**
     * @brief 计算原始点云与配准后点云之间的均方误差(MSE)
     * @param modelPointCloud 模型点云
     * @param queryPointCloud 查询点云
     * @param dist 距离阈值
     * @return double 返回MSE值
     */
    double getMSE(const PointCloudPtr& modelPointCloud, const PointCloudPtr& queryPointCloud, double dist)
    {
        try {
            pcl::KdTreeFLANN < PointT> modelTree;
            modelTree.setInputCloud(modelPointCloud);
            std::vector<int> nn_indices(1);
            std::vector<float> nn_dists(1);
            double fitness_score = 0.0;
            int count = 0;
            //For each point in the cloudA PointCloud
            for (int i = 0; i < queryPointCloud->points.size(); i++) {
                //Find nearest neighbor in the cloudB
                modelTree.nearestKSearch(queryPointCloud->points[i], 1, nn_indices, nn_dists);
                if (nn_dists[0] <= dist * dist)
                {
                    fitness_score += nn_dists[0];
                    count++;
                }
            }
            if (count > 0)
                return (fitness_score / count);
            else
                return (std::numeric_limits<double >::max());
        } catch (const std::exception& e) {
            LOG_ERROR(  "getMSE函数发生异常: %s", e.what());
            return std::numeric_limits<double>::max();
        }
    }

    /**
     * @brief 计算原始点云与配准后点云之间的均方根误差(RMSE)
     * @param cloudA 点云A
     * @param cloudB 点云B
     * @param rmse RMSE值
     * @param max_range 最大范围
     */
    void getRMSE(const PointCloudPtr& cloudA, const PointCloudPtr& cloudB, double rmse, double max_range)
    {
        try {
#if 0 // Method 1
            pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
            core.setInputSource(cloudA);
            core.setInputTarget(cloudB);
            pcl::Correspondences all;
            //core.determineCorrespondences(all_correspondences,0.05);//Find correspondences between source and target point clouds
            core.determineReciprocalCorrespondences(all);   // Find reciprocal correspondences between source and target point clouds
            float sum = 0.0, sum_x = 0.0, sum_y = 0.0, sum_z = 0.0, rmse = 0.0, rmse_x = 0.0, rmse_y = 0.0, rmse_z = 0.0;
             std::vector<float> Co;
            for (size_t i = 0; i < all.size(); ++i)
            {
                sum += all[i].distance;
                Co.push_back(all[i].distance);
                sum_x += pow((cloudB->points[all[i].index_match].x - cloudA->points[all[i].index_query].x), 2);
                sum_y += pow((cloudB->points[all[i].index_match].y - cloudA->points[all[i].index_query].y), 2);
                sum_z += pow((cloudB->points[all[i].index_match].z - cloudA->points[all[i].index_query].z), 2);
            }
            rmse = sqrt(sum / all.size());     // Calculate overall RMSE
            rmse_x = sqrt(sum_x / all.size()); // X-axis RMSE
            rmse_y = sqrt(sum_y / all.size()); // Y-axis RMSE
            rmse_z = sqrt(sum_z / all.size()); // Z-axis RMSE
            std::vector<float>::iterator max = max_element(Co.begin(), Co.end());// Find maximum correspondence
            std::vector<float>::iterator min = min_element(Co.begin(), Co.end());// Find minimum correspondence
#else// Method 2
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloudB);
            double fitness_score = 0.0;
            std::vector<int> nn_indices(1);
            std::vector<float> nn_dists(1);
            int nr = 0;

            for (size_t i = 0; i < cloudA->points.size(); ++i)
            {
                if (!std::isfinite((*cloudA)[i].x))
                {
                    continue;
                }
                tree->nearestKSearch(cloudA->points[i], 1, nn_indices, nn_dists);
                if (nn_dists[0] <= max_range * max_range)
                {
                    fitness_score += nn_dists[0];
                    nr++;
                }
            }

            if (nr > 0)
            {
                rmse = sqrt(fitness_score / nr);
            }
            else
            {
                rmse = std::numeric_limits<double>::max();
            }
#endif
        } catch (const std::exception& e) {
            LOG_ERROR(  "getRMSE函数发生异常: %s", e.what());
        }
    }


    //GICP registration
    /**
     * @brief CMultiFrameRegistGICP类的构造函数
     */
    CMultiFrameRegistGICP::CMultiFrameRegistGICP()
    {
        try {
            global_map_= std::make_shared<PointCloudT>();

            // Initialize GICP object
            gicp_.setMaxCorrespondenceDistance(0.01); // Maximum correspondence distance
            gicp_.setMaximumIterations(35);// Maximum iterations
            gicp_.setTransformationEpsilon(1e-10);// Transformation epsilon threshold
            gicp_.setEuclideanFitnessEpsilon(0.01);// Euclidean fitness epsilon threshold
            gicp_.setRANSACIterations(10);// Use RANSAC

            // Initialize voxel grid filter
            voxel_filter_.setLeafSize(0.05f, 0.05f, 0.05f); // 5cm voxel grid size
            
            LOG_INFO(  "CMultiFrameRegistGICP初始化成功");
        } catch (const std::exception& e) {
            LOG_ERROR(  "CMultiFrameRegistGICP构造函数发生异常: %s", e.what());
        }
    }
    /**
     * @brief CMultiFrameRegistGICP类的析构函数
     */
    CMultiFrameRegistGICP::~CMultiFrameRegistGICP()
    {

    }
    /**
     * @brief 获取全局地图点云
     * @return PointCloudPtr 全局地图点云
     */
    PointCloudPtr CMultiFrameRegistGICP::GetGlobalMap() const
    {
        return global_map_;
    }
    /**
     * @brief 处理单帧点云进行配准
     * @param new_cloud 新的输入点云
     * @return Eigen::Matrix4f 返回变换矩阵
     */
    Eigen::Matrix4f CMultiFrameRegistGICP::processFrame(const PointCloudT::Ptr& new_cloud)
    {
        try {
            PointCloudT::Ptr processed_cloud(new PointCloudT);

            // 1. Preprocess with downsampling
            downsampleCloud(new_cloud, processed_cloud);

            // 2. Use the first frame as the global map
            if (global_map_->empty()) {
                *global_map_ = *processed_cloud;
                Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
                frame_results_.push_back({ processed_cloud, identity });
                LOG_INFO(  "首帧已添加。点数：%lu", global_map_->size());
                return identity;
            }
            //KdTree search
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
            tree1->setInputCloud(processed_cloud);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
            tree2->setInputCloud(global_map_);
            gicp_.setSearchMethodSource(tree1);
            gicp_.setSearchMethodTarget(tree2);
            // 3. Perform GICP registration
            // Source points: current frame
            gicp_.setInputSource(processed_cloud);
            // Target points: global map
            gicp_.setInputTarget(global_map_);

            // 4. Perform registration
            PointCloudT::Ptr aligned_cloud(new PointCloudT);
            Eigen::Matrix4f initial_guess = frame_results_.back().pose; // Use pose of previous frame as initial guess
            Eigen::Matrix4f current_pose;

            gicp_.align(*aligned_cloud, initial_guess);

            if (gicp_.hasConverged()) 
            {
                current_pose = gicp_.getFinalTransformation();
                LOG_INFO(  "GICP已收敛。得分：%f", gicp_.getFitnessScore());
            }
            else 
            {
                LOG_WARN(  "GICP未收敛！使用初始猜测。", );
                current_pose = initial_guess;
            }

            // 5. Store result
            frame_results_.push_back({ aligned_cloud, current_pose });

            // 6. Update global map by adding 1% of the new points
            updateGlobalMap(processed_cloud, current_pose);
            
            return current_pose;
        } catch (const std::exception& e) {
            LOG_ERROR(  "processFrame函数发生异常: %s", e.what());
            return Eigen::Matrix4f::Identity();
        }
    }

    /**
     * @brief 对点云进行降采样
     * @param input 输入点云
     * @param output 输出点云
     */
    void CMultiFrameRegistGICP::downsampleCloud(const PointCloudT::Ptr& input, PointCloudT::Ptr& output)
    {
        try {
            voxel_filter_.setInputCloud(input);
            voxel_filter_.filter(*output);
        } catch (const std::exception& e) {
            LOG_ERROR(  "downsampleCloud函数发生异常: %s", e.what());
        }
    }


    /**
     * @brief 更新全局地图
     * @param new_cloud 新的点云
     * @param pose 位姿变换矩阵
     */
    void CMultiFrameRegistGICP::updateGlobalMap(const PointCloudT::Ptr& new_cloud, const Eigen::Matrix4f& pose)
    {
        try {
            PointCloudT::Ptr transformed_cloud(new PointCloudT);
            pcl::transformPointCloud(*new_cloud, *transformed_cloud, pose);
            *global_map_ += *transformed_cloud;
            // Optionally downsample the global map to prevent excessive memory usage
            if (global_map_->size() > 1000000) { // Downsample when the number of points exceeds 1 million
                PointCloudT::Ptr temp(new PointCloudT);
                downsampleCloud(global_map_, temp);
                global_map_ = temp;
                LOG_INFO(  "全局地图已降采样。新尺寸：%lu", global_map_->size());
            }
        } catch (const std::exception& e) {
            LOG_ERROR(  "updateGlobalMap函数发生异常: %s", e.what());
        }
    }

    /**
     * @brief 获取轨迹
     * @return std::vector<Eigen::Matrix4f> 返回轨迹向量
     */
    std::vector<Eigen::Matrix4f> CMultiFrameRegistGICP::getTrajectory() const
    {
        try {
            std::vector<Eigen::Matrix4f> trajectory;
            for (const auto& result : frame_results_) {
                trajectory.push_back(result.pose);
            }
            LOG_INFO(  "轨迹获取成功，共获取 %lu 个位姿", trajectory.size());
            return trajectory;
        } catch (const std::exception& e) {
            LOG_ERROR(  "getTrajectory函数发生异常: %s", e.what());
            return std::vector<Eigen::Matrix4f>();
        }
    }
}