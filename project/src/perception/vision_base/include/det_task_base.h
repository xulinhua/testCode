#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <map>

// 统一的检测结果结构
struct DetectionResult {
    cv::Point2f center;        // 中心点坐标 [center_x, center_y]
    float width;               // 宽度
    float height;              // 高度
    float confidence;          // 置信度
    int class_id;               // 类别ID
    std::string class_name;     // 类别名称
    cv::Point3f position_3d;    // 3D坐标(可选,单位:米)
    std::map<std::string, float> extra_attrs;  // 扩展属性(用于不同算法的额外信息)

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
};

// 相机内参结构
struct CameraIntrinsics {
    float fx;
    float fy;
    float cx;
    float cy;
    int width;   // 图像宽度
    int height;  // 图像高度
};

// 推理引擎类型枚举
enum class InferenceEngineType {
    TENSORRT = 0,    // TensorRT (推荐，GPU加速)
    ONNX = 1,        // ONNX Runtime
    NCNN = 2,        // NCNN (移动端优化)
    RKNN = 3,        // RKNN (瑞芯微NPU)
    OPENCV_DNN = 4,  // OpenCV DNN
    PADDLE = 5,      // Paddle Lite
    AUTO = 99        // 自动选择（算法内部决定）
};

// 将字符串转换为推理引擎类型枚举
// 返回 pair<引擎类型, 是否为已知类型>
inline std::pair<InferenceEngineType, bool> stringToEngineType(const std::string& engine_type_str) {
    if (engine_type_str == "tensorrt" || engine_type_str == "auto") {
        return {InferenceEngineType::TENSORRT, true};
    } else if (engine_type_str == "onnx") {
        return {InferenceEngineType::ONNX, true};
    } else if (engine_type_str == "ncnn") {
        return {InferenceEngineType::NCNN, true};
    } else if (engine_type_str == "rknn") {
        return {InferenceEngineType::RKNN, true};
    } else if (engine_type_str == "opencv_dnn") {
        return {InferenceEngineType::OPENCV_DNN, true};
    } else if (engine_type_str == "paddle") {
        return {InferenceEngineType::PADDLE, true};
    }
    // 未知类型，默认返回 TensorRT
    return {InferenceEngineType::TENSORRT, false};
}

// 将推理引擎类型枚举转换为字符串
inline std::string engineTypeToString(InferenceEngineType engine_type) {
    switch (engine_type) {
        case InferenceEngineType::TENSORRT: return "tensorrt";
        case InferenceEngineType::ONNX: return "onnx";
        case InferenceEngineType::NCNN: return "ncnn";
        case InferenceEngineType::RKNN: return "rknn";
        case InferenceEngineType::OPENCV_DNN: return "opencv_dnn";
        case InferenceEngineType::PADDLE: return "paddle";
        case InferenceEngineType::AUTO: return "auto";
        default: return "unknown";
    }
}

// 推理引擎配置
struct InferenceEngineConfig {
    InferenceEngineType engine_type = InferenceEngineType::TENSORRT;  // 推理引擎类型
    std::string engine_version = "";  // 引擎版本信息
    int device_id = 0;              // GPU/NPU设备ID
    int num_threads = 4;            // CPU线程数（对CPU推理有效）
    bool enable_fp16 = false;       // 是否启用FP16推理
    bool enable_int8 = false;       // 是否启用INT8量化
    int batch_size = 1;             // 批处理大小
    std::map<std::string, std::string> extra_params;  // 引擎特定参数
};

// 检测任务配置
struct DetectionConfig {
    float conf_threshold = 0.5f;   // 置信度阈值
    float nms_threshold = 0.45f;  // NMS阈值
    bool use_gpu_preprocess = true;   // 是否使用GPU预处理
    bool use_gpu_postprocess = false;  // 是否使用GPU后处理
    int max_detections = 100;     // 最大检测数

    // 【新增】推理引擎配置
    InferenceEngineConfig engine_config;
};

// 检测任务抽象接口
class IDetectionTask {
public:
    virtual ~IDetectionTask() = default;

    // 加载配置文件
    virtual bool loadConfig(const std::string& config_path) = 0;

    // 执行检测
    virtual std::vector<DetectionResult> detect(const cv::Mat& image) = 0;

    // 批量检测(可选实现)
    virtual std::vector<std::vector<DetectionResult>> detectBatch(const std::vector<cv::Mat>& images) {
        std::vector<std::vector<DetectionResult>> results;
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
    virtual DetectionConfig getConfig() const = 0;

    // 设置检测配置
    virtual void setConfig(const DetectionConfig& config) = 0;

    // 设置类别名称（由应用层注入，算法层不定义）
    virtual void setClassNames(const std::vector<std::string>& class_names) = 0;

    // 绘制检测结果（DetectionResult 已包含 class_name）
    virtual void drawResults(cv::Mat& image,
                           const std::vector<DetectionResult>& results) const = 0;

    // 绘制带深度的检测结果（DetectionResult 已包含 class_name）
    virtual void drawResultsWithDepth(cv::Mat& image,
                                    const std::vector<DetectionResult>& results,
                                    const cv::Mat& depth_image,
                                    const CameraIntrinsics& intrinsics) const = 0;

    // 获取算法信息
    virtual std::string getAlgorithmName() const = 0;
    virtual std::string getEngineName() const = 0;  // 返回推理引擎名称(字符串)
    virtual std::string getModelVersion() const = 0;

    // 检查是否已初始化
    virtual bool isInitialized() const = 0;
};
