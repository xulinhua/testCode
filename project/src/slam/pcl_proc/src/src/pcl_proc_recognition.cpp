#include "../include/pcl_proc/pcl_proc_recognition.h"
#include <pcl/features/shot.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/implicit_shape_model.h>
#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace PclProc
{
    // 基于对应分组的3D物体识别
    
    /**
     * @brief 3D霍夫变换识别
     * @param cloud 输入点云
     * @param templates 模板点云向量
     * @param scene 输入场景点云
     * @param model_scene_corrs 模型与场景的对应关系
     * @param hough_threshold 霍夫变换阈值
     * @param use_hough 是否使用霍夫变换
     * @return std::vector<Eigen::Matrix4f> 变换矩阵向量
     */
    std::vector<Eigen::Matrix4f> recognizeHough3D(
        const PointCloudPtr& cloud,
        const std::vector<PointCloudPtr>& templates,
        const PointCloudPtr& scene,
        const pcl::CorrespondencesPtr& model_scene_corrs,
        float hough_threshold,
        bool use_hough)
    {
        std::vector<Eigen::Matrix4f> transformations;
        
        try {
            if (cloud->size() < 1 || scene->size() < 1) {
                LOG_ERROR(  "recognizeHough3D接收到空点云");
                return transformations;
            }
            
            if (use_hough) {
                // 使用霍夫变换进行识别
                pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> hough_grouping;
                hough_grouping.setHoughBinSize(hough_threshold);
                hough_grouping.setHoughThreshold(hough_threshold);
                hough_grouping.setUseInterpolation(true);
                hough_grouping.setUseDistanceWeight(false);
                
                // 设置输入
                hough_grouping.setInputCloud(cloud);
                hough_grouping.setSceneCloud(scene);
                hough_grouping.setModelSceneCorrespondences(model_scene_corrs);
                
                // 执行识别
                std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rotations;
                std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> translations;
                hough_grouping.recognize(rotations, translations);
                
                // 转换结果
                transformations.resize(rotations.size());
                for (size_t i = 0; i < rotations.size(); ++i) {
                    transformations[i].block<3, 3>(0, 0) = rotations[i];
                    transformations[i].block<3, 1>(0, 3) = translations[i].cast<float>();
                    transformations[i](3, 3) = 1.0f;
                }
            }
        } catch (const std::exception& e) {
            LOG_ERROR(  "recognizeHough3D函数中发生异常: %s", e.what());
        }
        
        return transformations;
    }
    
    /**
     * @brief 几何一致性识别
     * @param cloud 输入点云
     * @param templates 模板点云向量
     * @param scene 输入场景点云
     * @param model_scene_corrs 模型与场景的对应关系
     * @param gc_threshold 几何一致性阈值
     * @param gc_size 几何一致性大小
     * @return std::vector<Eigen::Matrix4f> 变换矩阵向量
     */
    std::vector<Eigen::Matrix4f> recognizeGeometricConsistency(
        const PointCloudPtr& cloud,
        const std::vector<PointCloudPtr>& templates,
        const PointCloudPtr& scene,
        const pcl::CorrespondencesPtr& model_scene_corrs,
        float gc_threshold,
        float gc_size)
    {
        std::vector<Eigen::Matrix4f> transformations;
        
        try {
            if (cloud->size() < 1 || scene->size() < 1) {
                LOG_ERROR(  "recognizeGeometricConsistency接收到空点云");
                return transformations;
            }
            
            // 使用几何一致性进行识别
            pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_grouping;
            gc_grouping.setGCSize(gc_size);
            gc_grouping.setGCThreshold(gc_threshold);
            
            // 设置输入
            gc_grouping.setInputCloud(cloud);
            gc_grouping.setSceneCloud(scene);
            gc_grouping.setModelSceneCorrespondences(model_scene_corrs);
            
            // 执行识别
            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rotations;
            std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> translations;
            gc_grouping.recognize(rotations, translations);
            
            // 转换结果
            transformations.resize(rotations.size());
            for (size_t i = 0; i < rotations.size(); ++i) {
                transformations[i].block<3, 3>(0, 0) = rotations[i];
                transformations[i].block<3, 1>(0, 3) = translations[i].cast<float>();
                transformations[i](3, 3) = 1.0f;
            }
        } catch (const std::exception& e) {
            LOG_ERROR(  "recognizeGeometricConsistency函数中发生异常: %s", e.what());
        }
        
        return transformations;
    }
    
    // 隐式形状模型(ISM)
    
    /**
     * @brief 训练隐式形状模型
     * @param training_clouds 训练点云向量
     * @param training_classes 训练类别向量
     * @param training_models 训练模型向量
     * @param feature_radius 特征半径
     * @return bool 训练成功返回true，否则返回false
     */
    bool trainImplicitShapeModel(
        const std::vector<PointCloudPtr>& training_clouds,
        const std::vector<pcl::PointIndices>& training_classes,
        std::vector<pcl::ISMModelPtr>& training_models,
        float feature_radius)
    {
        try {
            if (training_clouds.empty() || training_classes.empty()) {
                LOG_ERROR(  "trainImplicitShapeModel接收到空训练数据");
                return false;
            }
            
            // 创建隐式形状模型训练对象
            pcl::ImplicitShapeModelEstimation<pcl::PointXYZ> ism;
            ism.setSamplingSize(feature_radius);
            
            // 设置训练数据
            ism.setInputClouds(training_clouds);
            ism.setTrainingClasses(training_classes);
            
            // 执行训练
            std::vector<pcl::ISMModelPtr> models;
            if (ism.trainISM(models)) {
                training_models = models;
                return true;
            }
        } catch (const std::exception& e) {
            LOG_ERROR(  "trainImplicitShapeModel函数中发生异常: %s", e.what());
        }
        
        return false;
    }
    
    /**
     * @brief 使用隐式形状模型进行识别
     * @param scene 场景点云
     * @param models 模型向量
     * @param classes 类别向量
     * @param radius_search 搜索半径
     * @return std::vector<Eigen::Matrix4f> 变换矩阵向量
     */
    std::vector<Eigen::Matrix4f> recognizeImplicitShapeModel(
        const PointCloudPtr& scene,
        const std::vector<pcl::ISMModelPtr>& models,
        const std::vector<int>& classes,
        float radius_search)
    {
        std::vector<Eigen::Matrix4f> transformations;
        
        try {
            if (scene->size() < 1 || models.empty()) {
                LOG_ERROR(  "recognizeImplicitShapeModel接收到空数据");
                return transformations;
            }
            
            // 使用隐式形状模型进行识别
            pcl::ImplicitShapeModelEstimation<pcl::PointXYZ> ism;
            ism.setSamplingSize(radius_search);
            
            // 设置输入
            ism.setSceneCloud(scene);
            ism.setModels(models);
            ism.setClassIds(classes);
            
            // 执行识别
            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rotations;
            std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> translations;
            std::vector<int> labels;
            ism.findObjects(rotations, translations, labels);
            
            // 转换结果
            transformations.resize(rotations.size());
            for (size_t i = 0; i < rotations.size(); ++i) {
                transformations[i].block<3, 3>(0, 0) = rotations[i];
                transformations[i].block<3, 1>(0, 3) = translations[i].cast<float>();
                transformations[i](3, 3) = 1.0f;
            }
        } catch (const std::exception& e) {
            LOG_ERROR(  "recognizeImplicitShapeModel函数中发生异常: %s", e.what());
        }
        
        return transformations;
    }
    
    // 3D物体识别工具包
    
    /**
     * @brief LINEMOD识别
     * @param scene 场景点云
     * @param templates 模板点云向量
     * @param detected_objects 检测到的物体向量
     * @param detected_templates 检测到的模板索引向量
     * @param detected_poses 检测到的姿态向量
     * @return bool 识别成功返回true，否则返回false
     */
    bool recognizeLINEMOD(
        const PointCloudPtr& scene,
        const std::vector<PointCloudPtr>& templates,
        std::vector<pcl::LINEMODDetection>& detected_objects,
        std::vector<int>& detected_templates,
        std::vector<Eigen::Affine3f>& detected_poses)
    {
        try {
            if (scene->size() < 1 || templates.empty()) {
                LOG_ERROR(  "recognizeLINEMOD接收到空数据");
                return false;
            }
            
            // 创建LINEMOD检测器
            pcl::LINEMOD linemod;
            
            // 添加模板
            for (size_t i = 0; i < templates.size(); ++i) {
                linemod.addTemplate(templates[i]);
            }
            
            // 执行识别
            std::vector<pcl::LINEMODDetection> detections;
            linemod.detect(scene, detections);
            
            // 提取结果
            detected_objects = detections;
            detected_templates.clear();
            detected_poses.clear();
            
            for (const auto& detection : detections) {
                detected_templates.push_back(detection.template_id);
                detected_poses.push_back(detection.pose);
            }
            
            return true;
        } catch (const std::exception& e) {
            LOG_ERROR(  "recognizeLINEMOD函数中发生异常: %s", e.what());
        }
        
        return false;
    }
    
    /**
     * @brief 计算点云的SHOT特征
     * @param cloud 输入点云
     * @param keypoints 关键点
     * @param radius 搜索半径
     * @param descriptors 输出描述子
     * @return bool 计算成功返回true，否则返回false
     */
    bool computeSHOTFeatures(
        const PointCloudPtr& cloud,
        const PointCloudPtr& keypoints,
        float radius,
        pcl::PointCloud<pcl::SHOT352>::Ptr& descriptors)
    {
        try {
            if (cloud->size() < 1 || keypoints->size() < 1) {
                LOG_ERROR(  "computeSHOTFeatures接收到空点云");
                return false;
            }
            
            // 计算法向量
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            
            normal_estimation.setInputCloud(cloud);
            normal_estimation.setSearchMethod(tree);
            normal_estimation.setRadiusSearch(radius);
            normal_estimation.compute(*normals);
            
            // 将点云和法向量连接
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
            
            // 计算SHOT特征
            pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
            shot.setInputCloud(keypoints);
            shot.setSearchSurface(cloud_with_normals);
            shot.setInputNormals(normals);
            shot.setRadiusSearch(radius);
            
            descriptors.reset(new pcl::PointCloud<pcl::SHOT352>);
            shot.compute(*descriptors);
            
            return true;
        } catch (const std::exception& e) {
            LOG_ERROR(  "computeSHOTFeatures函数中发生异常: %s", e.what());
        }
        
        return false;
    }
    
    /**
     * @brief 特征匹配
     * @param source_descriptors 源描述子
     * @param target_descriptors 目标描述子
     * @param correspondences 输出对应关系
     * @param similarity_threshold 相似度阈值
     * @return bool 匹配成功返回true，否则返回false
     */
    bool matchFeatures(
        const pcl::PointCloud<pcl::SHOT352>::Ptr& source_descriptors,
        const pcl::PointCloud<pcl::SHOT352>::Ptr& target_descriptors,
        pcl::CorrespondencesPtr& correspondences,
        float similarity_threshold)
    {
        try {
            if (source_descriptors->size() < 1 || target_descriptors->size() < 1) {
                LOG_ERROR(  "matchFeatures接收到空描述子");
                return false;
            }
            
            // 创建最近邻搜索
            pcl::KdTreeFLANN<pcl::SHOT352> match_search;
            match_search.setInputCloud(target_descriptors);
            
            // 执行匹配
            correspondences.reset(new pcl::Correspondences);
            
            for (size_t i = 0; i < source_descriptors->size(); ++i) {
                std::vector<int> neighbors(1);
                std::vector<float> squared_distances(1);
                
                if (match_search.nearestKSearch(source_descriptors->at(i), 1, neighbors, squared_distances) == 1) {
                    if (sqrt(squared_distances[0]) < (1.0f - similarity_threshold)) {
                        pcl::Correspondence correspondence(static_cast<int>(i), neighbors[0], squared_distances[0]);
                        correspondences->push_back(correspondence);
                    }
                }
            }
            
            return true;
        } catch (const std::exception& e) {
            LOG_ERROR(  "matchFeatures函数中发生异常: %s", e.what());
        }
        
        return false;
    }

}