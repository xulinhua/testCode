#include "ppocr.h"
#include "utils.h"
#include <algorithm>
#include <numeric>
#include <iostream>
#include <sstream>
#include <iomanip>



PPOCR::PPOCR() 
{
    detector_ = std::make_unique<DBnetDet>();
    recognizer_ = std::make_unique<PPOCRRec>();
    
    // 初始化默认配置
    detection_config_ = TextDetectionConfig{};
    recognition_config_ = RecognitionConfig{};
}

PPOCR::~PPOCR() 
{
    detector_.reset();
    recognizer_.reset();
}

void PPOCR::load_detection_model(const std::string& detection_engine_path) 
{
    try 
    {
        if (!detector_) 
        {
            throw TensorRTException("Detector not initialized");
        }
        
        detector_->load_engine(detection_engine_path);
        detector_->set_config(detection_config_);
        std::cout << "Detection model loaded successfully from: " << detection_engine_path << std::endl;
    }
    catch (const std::exception& e) 
    {
        handle_detection_error(std::string("Failed to load detection model: ") + e.what());
        throw;
    }
}

void PPOCR::load_recognition_model(const std::string& recognition_engine_path) 
{
    try 
    {
        if (!recognizer_) 
        {
            throw TensorRTException("Recognizer not initialized");
        }
        
        recognizer_->load_model(recognition_engine_path);
        
        std::cout << "Recognition model loaded successfully from: " << recognition_engine_path << std::endl;
    }
    catch (const std::exception& e) 
    {
        handle_recognition_error(std::string("Failed to load recognition model: ") + e.what());
        throw;
    }
}

void PPOCR::load_character_dict(const std::string& dict_path) 
{
    try 
    {
        if (!recognizer_) 
        {
            throw TensorRTException("Recognizer not initialized");
        }
        
        recognizer_->load_character_dict(dict_path);
        std::cout << "Character dictionary loaded successfully from: " << dict_path << std::endl;
    }
    catch (const std::exception& e) 
    {
        handle_recognition_error(std::string("Failed to load character dictionary: ") + e.what());
        throw;
    }
}

void PPOCR::load_models(const std::string& detection_engine_path, 
                          const std::string& recognition_engine_path, 
                          const std::string& dict_path) 
{
    try 
    {
        // 依次加载各个模型
        load_detection_model(detection_engine_path);
        load_recognition_model(recognition_engine_path);
        load_character_dict(dict_path);
        
        std::cout << "All models loaded successfully" << std::endl;
    }
    catch (const std::exception& e) 
    {
        std::cerr << "Failed to load models: " << e.what() << std::endl;
        throw;
    }
}



std::vector<PPOCRResult> PPOCR::process_image(const cv::Mat& img) 
{
    std::vector<PPOCRResult> results;
    
    try 
    {
        // 检测阶段
        std::vector<TextDetection> detections;
        if (detector_) 
        {
            detections = detect_text(img);
        }
        
        // 识别阶段
        std::vector<RecognitionResult> recognitions;
        if (recognizer_ && !detections.empty()) 
        {
            recognitions = recognize_text(img, detections);
        }
        
        // 组合结果
        size_t min_size = std::min(detections.size(), recognitions.size());
        results.reserve(min_size);
        
        for (size_t i = 0; i < min_size; ++i) 
        {
            PPOCRResult result;
            result.detection = detections[i];
            result.recognition = recognitions[i];
            results.push_back(result);
        }
    }
    catch (const std::exception& e) 
    {
        throw TensorRTException(std::string("Failed to process image: ") + e.what());
    }
    
    return results;
}



std::vector<TextDetection> PPOCR::detect_text(const cv::Mat& img) 
{
    if (!detector_) 
    {
        throw TensorRTException("Detector not loaded");
    }
    
    if (img.empty()) 
    {
        throw TensorRTException("Input image is empty");
    }
    
    std::vector<TextDetection> detections;
    
    try 
    {
        // 创建非const副本以适应detect方法的参数要求
        cv::Mat img_copy = img.clone();
        detections = detector_->detect(img_copy);
        
        // 过滤低置信度的检测结果
        detections.erase(
            std::remove_if(detections.begin(), detections.end(),
                          [this](const TextDetection& det) {
                              return det.confidence < 0.5f;  // 使用固定阈值
                          }),
            detections.end()
        );
    }
    catch (const std::exception& e) 
    {
        handle_detection_error(std::string("Detection failed: ") + e.what());
        throw;
    }
    
    return detections;
}

std::vector<RecognitionResult> PPOCR::recognize_text(const cv::Mat& img, 
                                                  const std::vector<TextDetection>& detections) 
{
    if (!recognizer_) 
    {
        throw TensorRTException("Recognizer not loaded");
    }
    
    if (detections.empty()) 
    {
        return {};
    }
    
    std::vector<RecognitionResult> results;
    
    try 
    {
        // 裁剪文本区域
        auto text_regions = crop_text_regions(img, detections);
        
        // 批量识别
        results = batch_recognize(text_regions);
        
        // 确保结果数量与检测数量一致
        while (results.size() < detections.size()) 
        {
            RecognitionResult empty_result;
            empty_result.text = "";
            empty_result.score = 0.0f;
            results.push_back(empty_result);
        }
    }
    catch (const std::exception& e) 
    {
        handle_recognition_error(std::string("Recognition failed: ") + e.what());
        throw;
    }
    
    return results;
}

void PPOCR::draw_results(cv::Mat& img, 
                          const std::vector<PPOCRResult>& results,
                          bool draw_polygon,
                          bool draw_text,
                          const cv::Scalar& detection_color,
                          const cv::Scalar& text_color) 
{
    // 准备检测结果向量
    std::vector<TextDetection> detections;
    detections.reserve(results.size());
    for (const auto& result : results) 
    {
        detections.push_back(result.detection);
    }
    
    // 绘制检测结果（包括检测框和检测分数）
    if (draw_polygon && detector_) 
    {
        detector_->draw_results(img, detections, false, true, detection_color);
    }
}

cv::Mat PPOCR::crop_text_region(const cv::Mat& img, const TextDetection& detection) 
{
    try 
    {
        // 使用旋转矩形进行裁剪
        cv::RotatedRect rrect = detection.rrect;
        
        // 获取旋转矩形的四个角点
        cv::Point2f vertices[4];
        rrect.points(vertices);
        
        // 计算目标矩形的宽高
        float width = std::max(rrect.size.width, rrect.size.height);
        float height = std::min(rrect.size.width, rrect.size.height);
        
        // 目标矩形的四个角点（水平矩形）
        cv::Point2f dst_vertices[4] = {
            cv::Point2f(0, 0),
            cv::Point2f(width, 0),
            cv::Point2f(width, height),
            cv::Point2f(0, height)
        };
        
        // 计算透视变换矩阵
        cv::Mat M = cv::getPerspectiveTransform(vertices, dst_vertices);
        
        // 应用透视变换
        cv::Mat cropped;
        cv::warpPerspective(img, cropped, M, cv::Size(width, height));
        
        return cropped;
    }
    catch (const std::exception& e) 
    {
        std::cerr << "Error cropping text region: " << e.what() << std::endl;
        return cv::Mat();
    }
}

cv::Mat PPOCR::crop_text_region_new(const cv::Mat& img, const TextDetection& detection) 
{
    const int orientation = detection.orientation;
    const float rw = detection.rrect.size.width;
    const float rh = detection.rrect.size.height;

    const int target_height = 48;
    const float target_width = rh * target_height / rw;

    // warpperspective shall be used to rotate the image
    // but actually they are all rectangles, so warpaffine is almost enough  :P

    cv::Mat dst;

    cv::Point2f corners[4];
    detection.rrect.points(corners);

    if (orientation == 0)
    {
        // horizontal text
        // corner points order
        //  0--------1
        //  |        |rw  -> as angle=90
        //  3--------2
        //      rh

        std::vector<cv::Point2f> src_pts(3);
        src_pts[0] = corners[0];
        src_pts[1] = corners[1];
        src_pts[2] = corners[3];

        std::vector<cv::Point2f> dst_pts(3);
        dst_pts[0] = cv::Point2f(0, 0);
        dst_pts[1] = cv::Point2f(target_width, 0);
        dst_pts[2] = cv::Point2f(0, target_height);

        cv::Mat tm = cv::getAffineTransform(src_pts, dst_pts);

        cv::warpAffine(img, dst, tm, cv::Size(target_width, target_height), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    }
    else
    {
        // vertial text
        // corner points order
        //  1----2
        //  |    |
        //  |    |
        //  |    |rh  -> as angle=0
        //  |    |
        //  |    |
        //  0----3
        //    rw

        std::vector<cv::Point2f> src_pts(3);
        src_pts[0] = corners[2];
        src_pts[1] = corners[3];
        src_pts[2] = corners[1];

        std::vector<cv::Point2f> dst_pts(3);
        dst_pts[0] = cv::Point2f(0, 0);
        dst_pts[1] = cv::Point2f(target_width, 0);
        dst_pts[2] = cv::Point2f(0, target_height);

        cv::Mat tm = cv::getAffineTransform(src_pts, dst_pts);

        cv::warpAffine(img, dst, tm, cv::Size(target_width, target_height), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    }

    return dst;
}

cv::Mat PPOCR::rotate_text_region(const cv::Mat& img, const TextDetection& detection) 
{
    try 
    {
        // 获取旋转矩形的参数
        cv::RotatedRect rrect = detection.rrect;
        cv::Mat rotated;
        
        // 获取旋转矩阵
        cv::Mat rot_mat = cv::getRotationMatrix2D(rrect.center, rrect.angle, 1.0);
        
        // 应用旋转
        cv::warpAffine(img, rotated, rot_mat, img.size());
        
        return rotated;
    }
    catch (const std::exception& e) 
    {
        std::cerr << "Error rotating text region: " << e.what() << std::endl;
        return cv::Mat();
    }
}

std::vector<cv::Mat> PPOCR::crop_text_regions(const cv::Mat& img, 
                                                const std::vector<TextDetection>& detections) 
{
    std::vector<cv::Mat> regions;
    regions.reserve(detections.size());
    
    for (const auto& detection : detections) 
    {
        if (is_valid_text_region(detection)) 
        {
            cv::Mat region = crop_text_region_new(img, detection);
            if (!region.empty()) 
            {
                regions.push_back(region);
            }
        }
    }
    
    return regions;
}

bool PPOCR::is_valid_text_region(const TextDetection& detection) 
{
    return detection.confidence >= 0.5f && 
           detection.rrect.size.width > 0 && 
           detection.rrect.size.height > 0;
}

std::vector<RecognitionResult> PPOCR::batch_recognize(const std::vector<cv::Mat>& text_regions) 
{
    std::vector<RecognitionResult> results;
    
    if (text_regions.empty()) 
    {
        return results;
    }
    
    try 
    {
        // 分批处理
        size_t batch_size = recognition_config_.max_batch_size;
        for (size_t i = 0; i < text_regions.size(); i += batch_size) 
        {
            size_t end = std::min(i + batch_size, text_regions.size());
            std::vector<cv::Mat> batch(text_regions.begin() + i, text_regions.begin() + end);
            
            auto batch_results = recognizer_->recognize_batch(batch);
            results.insert(results.end(), batch_results.begin(), batch_results.end());
        }
    }
    catch (const std::exception& e) 
    {
        std::cerr << "Batch recognition error: " << e.what() << std::endl;
        // 返回空结果
        for (size_t i = 0; i < text_regions.size(); ++i) 
        {
            RecognitionResult empty_result;
            empty_result.text = "";
            empty_result.score = 0.0f;
            results.push_back(empty_result);
        }
    }
    
    return results;
}

void PPOCR::update_detection_config(const TextDetectionConfig& det_config) 
{
    detection_config_ = det_config;
    if (detector_) 
    {
        detector_->set_config(det_config);
    }
}

void PPOCR::update_recognition_config(const RecognitionConfig& rec_config) 
{
    recognition_config_ = rec_config;
    if (recognizer_) 
    {
        recognizer_->set_config(rec_config);
    }
}

void PPOCR::handle_detection_error(const std::string& error_msg) 
{
    std::cerr << "Detection Error: " << error_msg << std::endl;
}

void PPOCR::handle_recognition_error(const std::string& error_msg) 
{
    std::cerr << "Recognition Error: " << error_msg << std::endl;
}