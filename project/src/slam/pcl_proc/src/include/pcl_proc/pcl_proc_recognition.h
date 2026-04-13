#ifndef PCL_PROC_RECOGNITION_H
#define PCL_PROC_RECOGNITION_H

#include "pcl_proc_base.h"
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/implicit_shape_model.h>
#include <pcl/recognition/linemod/line_rgbd.h>

namespace PclProc
{
    // 基于对应分组的3D物体识别
    
    /**
     * @brief 3D霍夫变换识别
     * @param cloud 输入点云
     * @param templates 模板点云向量
     * @param scene 输入场景点云
     * @param model_scene_corrs 模型与场景的对应关系
     * @param hough_threshold 霍夫变换阈值，默认5.0f
     * @param use_hough 是否使用霍夫变换，默认true
     * @return std::vector<Eigen::Matrix4f> 变换矩阵向量
     */
    std::vector<Eigen::Matrix4f> recognizeHough3D(
        const PointCloudPtr& cloud,
        const std::vector<PointCloudPtr>& templates,
        const PointCloudPtr& scene,
        const pcl::CorrespondencesPtr& model_scene_corrs,
        float hough_threshold = 5.0f,
        bool use_hough = true);
    
    /**
     * @brief 几何一致性识别
     * @param cloud 输入点云
     * @param templates 模板点云向量
     * @param scene 输入场景点云
     * @param model_scene_corrs 模型与场景的对应关系
     * @param gc_threshold 几何一致性阈值，默认0.02f
     * @param gc_size 几何一致性大小，默认0.02f
     * @return std::vector<Eigen::Matrix4f> 变换矩阵向量
     */
    std::vector<Eigen::Matrix4f> recognizeGeometricConsistency(
        const PointCloudPtr& cloud,
        const std::vector<PointCloudPtr>& templates,
        const PointCloudPtr& scene,
        const pcl::CorrespondencesPtr& model_scene_corrs,
        float gc_threshold = 0.02f,
        float gc_size = 0.02f);
    
    // 隐式形状模型(ISM)
    
    /**
     * @brief 训练隐式形状模型
     * @param training_clouds 训练点云向量
     * @param training_classes 训练类别向量
     * @param training_models 训练模型向量
     * @param feature_radius 特征半径，默认0.02f
     * @return bool 训练成功返回true，否则返回false
     */
    bool trainImplicitShapeModel(
        const std::vector<PointCloudPtr>& training_clouds,
        const std::vector<pcl::PointIndices>& training_classes,
        std::vector<pcl::ISMModelPtr>& training_models,
        float feature_radius = 0.02f);
    
    /**
     * @brief 使用隐式形状模型进行识别
     * @param scene 场景点云
     * @param models 模型向量
     * @param classes 类别向量
     * @param radius_search 搜索半径，默认0.02f
     * @return std::vector<Eigen::Matrix4f> 变换矩阵向量
     */
    std::vector<Eigen::Matrix4f> recognizeImplicitShapeModel(
        const PointCloudPtr& scene,
        const std::vector<pcl::ISMModelPtr>& models,
        const std::vector<int>& classes,
        float radius_search = 0.02f);
    
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
        std::vector<Eigen::Affine3f>& detected_poses);
    
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
        pcl::PointCloud<pcl::SHOT352>::Ptr& descriptors);
    
    /**
     * @brief 特征匹配
     * @param source_descriptors 源描述子
     * @param target_descriptors 目标描述子
     * @param correspondences 输出对应关系
     * @param similarity_threshold 相似度阈值，默认0.9f
     * @return bool 匹配成功返回true，否则返回false
     */
    bool matchFeatures(
        const pcl::PointCloud<pcl::SHOT352>::Ptr& source_descriptors,
        const pcl::PointCloud<pcl::SHOT352>::Ptr& target_descriptors,
        pcl::CorrespondencesPtr& correspondences,
        float similarity_threshold = 0.9f);

}

#endif