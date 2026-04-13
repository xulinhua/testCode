#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include "det_task_base.h"  // 复用 DetectionResult, CameraIntrinsics, InferenceEngineType 等

// 统一的分割结果结构
struct SegmentationResult {
    // 边界框信息
    cv::Point2f center;        // 中心点坐标 [center_x, center_y]
    float width;               // 宽度
    float height;              // 高度
    float confidence;          // 置信度
    int class_id;              // 类别ID
    std::string class_name;    // 类别名称
    
    // 分割掩码
    cv::Mat mask;              // 二值掩码 (CV_8UC1)，与原图同尺寸
    
    // mask系数 (用于与proto计算最终mask)
    float mask_coef[32];
    
    // 3D坐标(可选,单位:米)
    cv::Point3f position_3d;
    
    // 扩展属性(用于不同算法的额外信息)
    std::map<std::string, float> extra_attrs;
    
    // 便捷方法：获取左上角坐标
    cv::Point2f topLeft() const {
        return cv::Point2f(center.x - width / 2.0f, center.y - height / 2.0f);
    }
    
    // 便捷方法：获取边界框（左上角+宽高）
    cv::Rect getBBox() const {
        cv::Point2f tl = topLeft();
        return cv::Rect(
            static_cast<int>(tl.x),
            static_cast<int>(tl.y),
            static_cast<int>(width),
            static_cast<int>(height)
        );
    }
    
    // 便捷方法：设置边界框（从左上角+宽高）
    void setBBox(const cv::Rect& bbox) {
        center.x = bbox.x + bbox.width / 2.0f;
        center.y = bbox.y + bbox.height / 2.0f;
        width = bbox.width;
        height = bbox.height;
    }
    
    // 便捷方法：计算掩码面积（像素数）
    int getMaskArea() const {
        if (mask.empty()) return 0;
        return cv::countNonZero(mask);
    }
    
    // 便捷方法：计算掩码与边界框的IOU
    float getMaskBBoxIoU() const {
        if (mask.empty()) return 0.0f;
        int mask_area = getMaskArea();
        int bbox_area = static_cast<int>(width * height);
        if (bbox_area == 0) return 0.0f;
        return static_cast<float>(mask_area) / bbox_area;
    }
};

// 分割任务配置
struct SegmentationConfig {
    float conf_threshold = 0.5f;   // 置信度阈值
    float nms_threshold = 0.45f;   // NMS阈值
    float mask_threshold = 0.5f;   // 掩码二值化阈值
    bool use_gpu_preprocess = true;   // 是否使用GPU预处理
    bool use_gpu_postprocess = false;  // 是否使用GPU后处理
    int max_detections = 100;     // 最大检测数
    
    // 推理引擎配置
    InferenceEngineConfig engine_config;
};

// 分割任务抽象接口
class ISegmentationTask {
public:
    virtual ~ISegmentationTask() = default;
    
    // 加载配置文件
    virtual bool loadConfig(const std::string& config_path) = 0;
    
    // 执行分割
    virtual std::vector<SegmentationResult> segment(const cv::Mat& image) = 0;
    
    // 批量分割(可选实现)
    virtual std::vector<std::vector<SegmentationResult>> segmentBatch(const std::vector<cv::Mat>& images) {
        std::vector<std::vector<SegmentationResult>> results;
        for (const auto& img : images) {
            results.push_back(segment(img));
        }
        return results;
    }
    
    // 获取支持的类别列表
    virtual std::vector<std::string> getSupportedClasses() const = 0;
    
    // 设置分割阈值
    virtual void setThreshold(float conf_thresh, float nms_thresh) = 0;
    
    // 设置掩码阈值
    virtual void setMaskThreshold(float mask_thresh) = 0;
    
    // 获取分割配置
    virtual SegmentationConfig getConfig() const = 0;
    
    // 设置分割配置
    virtual void setConfig(const SegmentationConfig& config) = 0;
    
    // 设置类别名称（由应用层注入，算法层不定义）
    virtual void setClassNames(const std::vector<std::string>& class_names) = 0;
    
    // 绘制分割结果
    virtual void drawResults(cv::Mat& image,
                           const std::vector<SegmentationResult>& results) const = 0;
    
    // 绘制带深度的分割结果
    virtual void drawResultsWithDepth(cv::Mat& image,
                                    const std::vector<SegmentationResult>& results,
                                    const cv::Mat& depth_image,
                                    const CameraIntrinsics& intrinsics) const = 0;
    
    // 获取算法信息
    virtual std::string getAlgorithmName() const = 0;
    virtual std::string getEngineName() const = 0;  // 返回推理引擎名称(字符串)
    virtual std::string getModelVersion() const = 0;
    
    // 检查是否已初始化
    virtual bool isInitialized() const = 0;
};
