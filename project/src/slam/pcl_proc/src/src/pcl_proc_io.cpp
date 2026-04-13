#include"../include/pcl_proc/pcl_proc_io.h"
#include<iostream>
#include <pcl/io/pcd_io.h>
#include"../include/pcl_proc/pcl_proc_filter.h"

namespace PclProc
{
    /**
     * @brief 加载点云
     * @param cloud_Input 输出点云
     * @param filename 文件名
     * @return bool 加载成功返回true，否则返回false
     */
    bool loadPointCloud(PointCloudPtr& cloud_Input, const std::string& filename)
    {
        try {
            int nRet = pcl::io::loadPCDFile<PointT>(filename, *cloud_Input);
            if (nRet == -1) {
                LOG_ERROR(  "Failed to load point cloud from file: %s", filename.c_str());
                return false;
            }
            cloud_Input = PclProc::removeNaNFromPoint(cloud_Input);
        } catch (const std::exception& e) {
            LOG_ERROR(  "Exception in loadPointCloud: %s", e.what());
            return false;
        }

        return true;
    }

    /**
     * @brief 加载多个点云
     * @param clouds 输出点云向量
     * @param filenames 文件名向量
     * @return bool 加载成功返回true，否则返回false
     */
    bool loadPointClouds(std::vector<PointCloudPtr>& clouds, const std::vector<std::string> filenames)
    {
        try {
            clouds.clear();

            int fileNum = filenames.size();
            if (fileNum == 0) {
                LOG_WARN(  "No filenames provided for loading point clouds");
                return false;
            }

            bool bRet = true;
            for (size_t i = 0; i < fileNum; i++)
            {
                PointCloudPtr singPcd(new PointCloudT);
                bool nRet = loadPointCloud(singPcd, filenames[i]);
                if (!nRet) {
                    LOG_ERROR(  "Failed to load point cloud from file: %s", filenames[i].c_str());
                }
                bRet = bRet && nRet;
                clouds.push_back(singPcd);
            }
            return bRet;
        } catch (const std::exception& e) {
            LOG_ERROR(  "Exception in loadPointClouds: %s", e.what());
            return false;
        }
		return false;
    }

    /**
     * @brief 保存点云
     * @param cloud_Input 输入点云
     * @param filename 文件名
     * @param bBinary 是否以二进制格式保存
     * @return bool 保存成功返回true，否则返回false
     */
    bool savePointCloud(PointCloudPtr cloud_Input, const std::string& filename, bool bBinary)
    {
        try {
            int nRet = 0;
            if (bBinary)
            {
                nRet = pcl::io::savePCDFileBinary(filename, *cloud_Input);
            }
            else
            {
                nRet = pcl::io::savePCDFileASCII(filename, *cloud_Input);
            }
            
            if (nRet == -1) {
                LOG_ERROR(  "Failed to save point cloud to file: %s", filename.c_str());
                return false;
            }
            
            return true;
        } catch (const std::exception& e) {
            LOG_ERROR(  "Exception in savePointCloud: %s", e.what());
            return false;
        }
    }

    /**
     * @brief 保存法向量值
     * @param normals 输入法向量
     * @param filename 文件名
     * @param bBinary 是否以二进制格式保存
     * @return bool 保存成功返回true，否则返回false
     */
    bool saveNormals(pcl::PointCloud<pcl::Normal>::Ptr normals, const std::string& filename, bool bBinary)
    {
        try {
            pcl::PCDWriter writer;
            int nRet= writer.write<pcl::Normal>(filename, *normals, bBinary);
            
            if (nRet == -1) {
                LOG_ERROR(  "Failed to save normals to file: %s", filename.c_str());
                return false;
            }
            
            return true;
        } catch (const std::exception& e) {
            LOG_ERROR(  "Exception in saveNormals: %s", e.what());
            return false;
        }
    }

    /**
     * @brief 保存NARF特征
     * @param keypoints 关键点
     * @param descriptors 描述子
     * @param filename 文件名
     */
    void saveNARFFeatures(const pcl::PointCloud<pcl::PointWithScale>::Ptr& keypoints,
        const pcl::PointCloud<pcl::Narf36>::Ptr& descriptors, const std::string& filename)
    {
        try {
            std::ofstream file(filename);
            if (!file.is_open()) {
                LOG_ERROR(  "Unable to open file %s for writing NARF features", filename.c_str());
                return;
            }

            file << "NARF Features" << endl;
            file << "Number of keypoints: " << keypoints->size() << endl;
            file << "Number of descriptors: " << descriptors->size() << endl;
            file << "Descriptor dimension: 36" << endl;
            file << "==========================================" << endl;

            for (size_t i = 0; i < keypoints->size() && i < descriptors->size(); ++i) {
                const auto& keypoint = keypoints->points[i];
                const auto& descriptor = descriptors->points[i];

                file << "\nKeypoint " << i << ":" << endl;
                file << "Position: " << keypoint.x << ", " << keypoint.y << ", " << keypoint.z << endl;
                file << "Scale: " << keypoint.scale << endl;
                file << "Orientation (RPY): " << descriptor.roll << ", " << descriptor.pitch << ", " << descriptor.yaw << endl;
                file << "Descriptor values: ";

                for (int j = 0; j < 36; ++j) {
                    file << descriptor.descriptor[j] << " ";
                    if ((j + 1) % 12 == 0) file << endl << "           ";
                }
                file << endl;
            }

            file.close();
            LOG_INFO(  "NARF features saved to: %s", filename.c_str());
        } catch (const std::exception& e) {
            LOG_ERROR(  "Exception in saveNARFFeatures: %s", e.what());
        }
    }

    /**
     * @brief 将pcd转换为深度图像
     * @param cloud 输入点云
     * @param angular_resolution 角度分辨率
     * @return pcl::RangeImage::Ptr 返回深度图像
     */
    pcl::RangeImage::Ptr createRangeImageFromPointCloud(const PointCloudPtr& cloud, float angular_resolution)
    {
        try {
            // Get the pose of the sensor
            Eigen::Affine3f scene_sensor_pose = Eigen::Affine3f::Identity();
            scene_sensor_pose.translation() = cloud->sensor_origin_.head<3>();
            scene_sensor_pose.linear() = cloud->sensor_orientation_.toRotationMatrix();

            // Create the range image
            pcl::RangeImage::Ptr range_image(new pcl::RangeImage);
            float noise_level = 0.0;
            float min_range = 0.0f;
            int border_size = 1;

            range_image->createFromPointCloud(*cloud,angular_resolution,
                pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                scene_sensor_pose, pcl::RangeImage::CAMERA_FRAME,
                noise_level, min_range, border_size);

            return range_image;
        } catch (const std::exception& e) {
            LOG_ERROR(  "Exception in createRangeImageFromPointCloud: %s", e.what());
            return pcl::RangeImage::Ptr(new pcl::RangeImage);
        }
    }

    /**
     * @brief 将PointXYZ转换为PointNormal（不进行法向量估计）
     * @param input 输入点云
     * @param output 输出点云
     * @param initialize_normals 是否初始化法向量
     */
    void convertXYZToPointNormal(const PointCloudPtr& input, PointCloudNTPtr& output, bool initialize_normals)
    {
        try {
            // Copy the point structure
            pcl::copyPointCloud(*input, *output);

            if (initialize_normals) {
                // Initialize normal vectors
                for (auto& point : output->points) {
                    point.normal_x = 0.0f;
                    point.normal_y = 0.0f;
                    point.normal_z = 0.0f;
                    point.curvature = 0.0f;
                }
            }
        } catch (const std::exception& e) {
            LOG_ERROR(  "Exception in convertXYZToPointNormal: %s", e.what());
        }
    }

    /**
     * @brief 将PointXYZ转换为PointNormal（进行法向量估计）
     * @param input 输入点云
     * @param output 输出点云
     * @param radius 搜索半径
     */
    void convertXYZToPointNormalWithNormals(const PointCloudPtr& input, PointCloudNTPtr& output, float radius)
    {
        try {
            // First copy the point information
            pcl::copyPointCloud(*input, *output);

            // Normal estimation
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            
            normal_estimator.setInputCloud(input);
            normal_estimator.setSearchMethod(tree);
            normal_estimator.setRadiusSearch(radius);
            normal_estimator.compute(*normals);
            pcl::concatenateFields(*input, *normals, *output);
        } catch (const std::exception& e) {
            LOG_ERROR(  "Exception in convertXYZToPointNormalWithNormals: %s", e.what());
        }
    }
}