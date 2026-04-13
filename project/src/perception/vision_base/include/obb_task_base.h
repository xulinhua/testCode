#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include "det_task_base.h"  // 复用 DetectionResult, CameraIntrinsics, InferenceEngineType 等

// 统一的OBB检测结果结构（旋转目标检测）
struct OBBDetectionResult {
    // 旋转边界框信息
    cv::Point2f center;        // 中心点坐标 [center_x, center_y]
    float width;               // 宽度
    float height;              // 高度
    float angle;               // 旋转角度（弧度）
    float confidence;          // 置信度
    int class_id;              // 类别ID
    std::string class_name;    // 类别名称
    
    // 3D坐标(可选,单位:米)
    cv::Point3f position_3d;
    
    // 扩展属性(用于不同算法的额外信息)
    std::map<std::string, float> extra_attrs;
    
    // 便捷方法：获取旋转矩形
    cv::RotatedRect getRotatedRect() const {
        // OpenCV角度是度数，顺时针为正
        float opencv_angle = angle * 180.0f / CV_PI;
        return cv::RotatedRect(center, cv::Size2f(width, height), opencv_angle);
    }
    
    // 便捷方法：获取旋转矩形的四个顶点
    std::vector<cv::Point2f> getVertices() const {
        cv::RotatedRect rect = getRotatedRect();
        cv::Point2f vertices[4];
        rect.points(vertices);
        return std::vector<cv::Point2f>(vertices, vertices + 4);
    }
    
    // 便捷方法：获取轴对齐边界框（最小外接矩形）
    cv::Rect getBBox() const {
        cv::RotatedRect rect = getRotatedRect();
        return rect.boundingRect();
    }
};

// OBB检测任务配置
struct OBBDetectionConfig {
    float conf_threshold = 0.5f;       // 置信度阈值
    float nms_threshold = 0.45f;       // NMS阈值（旋转框IOU）
    bool use_gpu_preprocess = true;    // 是否使用GPU预处理
    bool use_gpu_postprocess = true;   // 是否使用GPU后处理
    int max_detections = 100;          // 最大检测数
    
    // 推理引擎配置
    InferenceEngineConfig engine_config;
};

// OBB检测任务抽象接口
class IOBBDetectionTask {
public:
    virtual ~IOBBDetectionTask() = default;
    
    // 加载配置文件
    virtual bool loadConfig(const std::string& config_path) = 0;
    
    // 执行OBB检测
    virtual std::vector<OBBDetectionResult> detect(const cv::Mat& image) = 0;
    
    // 批量OBB检测（默认实现，子类可重写以优化性能）
    virtual std::vector<std::vector<OBBDetectionResult>> detectBatch(const std::vector<cv::Mat>& images) {
        std::vector<std::vector<OBBDetectionResult>> results;
        results.reserve(images.size());
        for (const auto& img : images) {
            results.push_back(detect(img));
        }
        return results;
    }
    
    // 获取支持的类别列表
    virtual std::vector<std::string> getSupportedClasses() const = 0;
    
    // 设置检测阈值
    virtual void setThreshold(float conf_thresh, float nms_thresh) = 0;
    
    // 获取检测配置
    virtual OBBDetectionConfig getConfig() const = 0;
    
    // 设置检测配置
    virtual void setConfig(const OBBDetectionConfig& config) = 0;
    
    // 设置类别名称（由应用层注入，算法层不定义）
    virtual void setClassNames(const std::vector<std::string>& class_names) = 0;
    
    // 绘制检测结果
    virtual void drawResults(cv::Mat& image,
                            const std::vector<OBBDetectionResult>& results) const = 0;
    
    // 绘制带深度的检测结果
    virtual void drawResultsWithDepth(cv::Mat& image,
                                      const std::vector<OBBDetectionResult>& results,
                                      const cv::Mat& depth_image,
                                      const CameraIntrinsics& intrinsics) const = 0;
    
    // 获取算法信息
    virtual std::string getAlgorithmName() const = 0;
    virtual std::string getEngineName() const = 0;
    virtual std::string getModelVersion() const = 0;
    
    // 检查是否已初始化
    virtual bool isInitialized() const = 0;
};
