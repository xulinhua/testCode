#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <map>
#include "NvInfer.h"
#include "NvInferRuntime.h"
#include "ppocr_det.h"
#include "ppocr_rec.h"



// PPOCR 结果结构
struct PPOCRResult 
{
    TextDetection detection;    // 单个检测结果
    RecognitionResult recognition; // 单个识别结果
};

class PPOCR 
{
public:
    PPOCR();
    ~PPOCR();
    
    // 初始化函数
    void load_detection_model(const std::string& detection_engine_path);
    void load_recognition_model(const std::string& recognition_engine_path);
    void load_character_dict(const std::string& dict_path);

    /**
     * @brief 统一加载所有模型
     * @param detection_engine_path 检测模型引擎文件路径
     * @param recognition_engine_path 识别模型引擎文件路径
     * @param dict_path 字符字典文件路径
     */
    void load_models(const std::string& detection_engine_path, 
                    const std::string& recognition_engine_path, 
                    const std::string& dict_path);
    
    // 主要处理函数
    std::vector<PPOCRResult> process_image(const cv::Mat& img);
    
    // 分步处理函数
    std::vector<TextDetection> detect_text(const cv::Mat& img);
    std::vector<RecognitionResult> recognize_text(const cv::Mat& img, 
                                            const std::vector<TextDetection>& detections);
    
    // 工具函数
    void draw_results(cv::Mat& img, 
                     const std::vector<PPOCRResult>& results,
                     bool draw_polygon = true,
                     bool draw_text = true,
                     const cv::Scalar& detection_color = cv::Scalar(0, 255, 0),
                     const cv::Scalar& text_color = cv::Scalar(255, 0, 0));
    
    cv::Mat crop_text_region(const cv::Mat& img, const TextDetection& detection);
    cv::Mat crop_text_region_new(const cv::Mat& img, const TextDetection& detection);
    cv::Mat rotate_text_region(const cv::Mat& img, const TextDetection& detection);
    
    // 配置管理
    void update_detection_config(const TextDetectionConfig& det_config);
    void update_recognition_config(const RecognitionConfig& rec_config);
   
private:
    // 核心组件
    std::unique_ptr<DBnetDet> detector_;
    std::unique_ptr<PPOCRRec> recognizer_;
    
    // 配置
    TextDetectionConfig detection_config_;
    RecognitionConfig recognition_config_;
    
    std::vector<cv::Mat> crop_text_regions(const cv::Mat& img, 
                                         const std::vector<TextDetection>& detections);
    
    // 辅助函数
    bool is_valid_text_region(const TextDetection& detection);
    std::vector<RecognitionResult> batch_recognize(const std::vector<cv::Mat>& text_regions);
    
    // 错误处理
    void handle_detection_error(const std::string& error_msg);
    void handle_recognition_error(const std::string& error_msg);
};