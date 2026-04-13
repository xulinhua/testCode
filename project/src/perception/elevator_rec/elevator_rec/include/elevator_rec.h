#pragma once
#include "yolo_det.h"
#include "ocr_rec.h"

struct ButtonRecognition 
{
    Detection det;
    Recognition rec;
};

class ElevatorButtonRec
{
public:
    ElevatorButtonRec();
    ~ElevatorButtonRec();
    void load_engine(const std::string& det_engine_path, const std::string& rec_engine_path, const std::string& dict_path);
    std::vector<ButtonRecognition> infer(cv::Mat& img, const std::vector<std::string> &class_names);
    void draw_results(cv::Mat& img, const std::vector<ButtonRecognition>& buttonResults,
                    const std::vector<std::string> &class_names);
    void draw_results(cv::Mat& img, const std::vector<ButtonRecognition>& buttonResults,
                    cv::Mat &depth_img,
                    const Intrinsics &intrinsics,
                    const std::vector<std::string> &class_names);
private:
    void crop_image(const cv::Mat& src_img, const Detection& det_res, cv::Mat& crop_img);
private:
    YoloDet det_;
    OcrRec rec_;
};