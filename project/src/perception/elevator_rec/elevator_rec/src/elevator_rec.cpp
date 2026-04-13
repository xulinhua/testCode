#pragma once
#include "elevator_rec.h"

ElevatorButtonRec::ElevatorButtonRec()
{

}

ElevatorButtonRec::~ElevatorButtonRec()
{

}

void ElevatorButtonRec::load_engine(const std::string& det_engine_path, const std::string& rec_engine_path, const std::string& dict_path)
{
    det_.load_engine(det_engine_path);
    rec_.load_engine(rec_engine_path, dict_path);
}

std::vector<ButtonRecognition> ElevatorButtonRec::infer(cv::Mat& img, const std::vector<std::string> &class_names)
{   
    std::vector<ButtonRecognition> buttonResults;
    ButtonRecognition buttonResult;
    bool bTestTimes = false; 
    if(bTestTimes)
    {
        std::vector<Detection> detections;
        auto start = std::chrono::high_resolution_clock::now();
        int nCount = 100;      
        for(int i =0; i< nCount; i++)
        {
            detections.clear();
            detections = det_.infer(img);
        }   
        auto end = std::chrono::high_resolution_clock::now();
        double ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
        std::cout << "det_time: " << ave_time << "ms" << std::endl;

        int det_size = detections.size();
        bool bBatch = true;
        if(bBatch)
        {
            std::vector<cv::Mat> crop_imgs;
            std::vector<int> valid_indices;  // 记录有效检测结果的索引
            std::vector<Recognition> batch_rec_results;
            start = std::chrono::high_resolution_clock::now();     
            for(int i =0; i< nCount; i++)
            {
                crop_imgs.clear();
                valid_indices.clear();
                for(int i = 0; i < detections.size(); i++)
                {
                    if(detections[i].class_id == 0)
                    {
                        cv::Mat crop_img;
                        crop_image(img, detections[i], crop_img);
                        crop_imgs.push_back(crop_img);
                        valid_indices.push_back(i);
                    }                   
                }
            }
            end = std::chrono::high_resolution_clock::now();
            ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
            std::cout << "crop_time: " << ave_time << "ms" << std::endl;
            
           
            if(!crop_imgs.empty())
            {
                std::vector<Recognition> batch_rec_results;
                start = std::chrono::high_resolution_clock::now();     
                for(int i =0; i< nCount; i++)
                {
                    batch_rec_results = rec_.infer_batch(crop_imgs);
                }
                end = std::chrono::high_resolution_clock::now();
                ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
                std::cout << "rec_time: " << ave_time << "ms" << std::endl;
                
                for(int i = 0; i < batch_rec_results.size(); i++)
                {
                    ButtonRecognition buttonResult;
                    buttonResult.det = detections[valid_indices[i]];
                    buttonResult.rec = batch_rec_results[i];
                    buttonResults.push_back(buttonResult);
                }
            }

        }
        else
        {
            cv::Mat crop_img;
            for(int i = 0; i < det_size; i++)
            {
                if(detections[i].class_id == 0)
                {
                    crop_image(img, detections[i], crop_img);
                    buttonResult.det = detections[i];
                    start = std::chrono::high_resolution_clock::now();     
                    for(int i =0; i< nCount; i++)
                    {
                        buttonResult.rec = rec_.infer(crop_img);
                    }
                    end = std::chrono::high_resolution_clock::now();
                    ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
                    std::cout << "rec_time: " << ave_time << "ms" << std::endl;                
                    buttonResults.push_back(buttonResult);
                }
            }
        }
        
    }
    else
    {
        std::vector<Detection> detections = det_.infer(img);
        int det_size = detections.size();
        bool bBatch = true;
        if(bBatch)
        {
            std::vector<cv::Mat> crop_imgs;
            std::vector<int> valid_indices;  // 记录有效检测结果的索引

            for(int i = 0; i < detections.size(); i++)
            {
                if(detections[i].class_id == 0)
                {
                    cv::Mat crop_img;
                    crop_image(img, detections[i], crop_img);
                    crop_imgs.push_back(crop_img);
                    valid_indices.push_back(i);
                }
                else
                {
                    ButtonRecognition buttonResult;
                    buttonResult.det = detections[i];
                    Recognition rec;
                    rec.text = class_names[detections[i].class_id];
                    rec.score = detections[i].conf;
                    buttonResult.rec = rec;
                    buttonResults.push_back(buttonResult);
                }
            }            
            
            if(!crop_imgs.empty())
            {
                std::vector<Recognition> batch_rec_results = rec_.infer_batch(crop_imgs);

                for(int i = 0; i < batch_rec_results.size(); i++)
                {
                    ButtonRecognition buttonResult;
                    buttonResult.det = detections[valid_indices[i]];
                    buttonResult.rec = batch_rec_results[i];
                    buttonResults.push_back(buttonResult);
                }
            }

        }
        else
        {
            cv::Mat crop_img;
            for(int i = 0; i < det_size; i++)
            {
                if(detections[i].class_id == 0)
                {
                    crop_image(img, detections[i], crop_img);
                    buttonResult.det = detections[i];
                    buttonResult.rec = rec_.infer(crop_img);
                    buttonResults.push_back(buttonResult);
                }
                else
                {
                    ButtonRecognition buttonResult;
                    buttonResult.det = detections[i];
                    Recognition rec;
                    rec.text = class_names[detections[i].class_id];
                    rec.score = detections[i].conf;
                    buttonResult.rec = rec;
                    buttonResults.push_back(buttonResult);
                }
            }
        }
        
    }   
    
    return buttonResults;
}

void ElevatorButtonRec::draw_results(cv::Mat& img, const std::vector<ButtonRecognition>& buttonResults,
                           const std::vector<std::string> &class_names)
{
    static const std::vector<cv::Scalar> colors = 
    {
        cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255)
    };
    
    for (const auto& res : buttonResults) 
    {
        // 将中心点坐标转换为左上角坐标
        float x1 = res.det.bbox[0] - res.det.bbox[2] / 2;
        float y1 = res.det.bbox[1] - res.det.bbox[3] / 2;
        float x2 = res.det.bbox[0] + res.det.bbox[2] / 2;
        float y2 = res.det.bbox[1] + res.det.bbox[3] / 2;
        
        // 确保坐标不超出图像边界
        x1 = std::max(0.0f, x1);
        y1 = std::max(0.0f, y1);
        x2 = std::min((float)img.cols, x2);
        y2 = std::min((float)img.rows, y2);
        
        cv::Rect rect(x1, y1, x2 - x1, y2 - y1);
        cv::Scalar color = colors[res.det.class_id % colors.size()];
        
        cv::rectangle(img, rect, color, 2);
        
        std::string label;
        
        label =res.rec.text /*+ ": " + cv::format("%.3f", res.rec.score)*/;
        int baseline;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        
        // 调整标签框的位置，确保不超出图像顶部
        int text_y = std::max(rect.y, text_size.height + 5);
        cv::rectangle(img, 
                     cv::Point(rect.x, text_y - text_size.height - 5),
                     cv::Point(rect.x + text_size.width, text_y),
                     color, cv::FILLED);
        cv::putText(img, label, 
                   cv::Point(rect.x, text_y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        
        // std::cout << label  <<":center_x,center_y :" << res.det.bbox[0] << "," << res.det.bbox[1] << std::endl;       
    }
}

void ElevatorButtonRec::draw_results(cv::Mat& img, const std::vector<ButtonRecognition>& buttonResults,
                           cv::Mat &depth_img,
                           const Intrinsics &intrinsics,
                           const std::vector<std::string> &class_names)
{
    static const std::vector<cv::Scalar> colors = 
    {
        cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255)
    };
    
    for (const auto& res : buttonResults) 
    {
        // 将中心点坐标转换为左上角坐标
        float x1 = res.det.bbox[0] - res.det.bbox[2] / 2;
        float y1 = res.det.bbox[1] - res.det.bbox[3] / 2;
        float x2 = res.det.bbox[0] + res.det.bbox[2] / 2;
        float y2 = res.det.bbox[1] + res.det.bbox[3] / 2;
        
        // 确保坐标不超出图像边界
        x1 = std::max(0.0f, x1);
        y1 = std::max(0.0f, y1);
        x2 = std::min((float)img.cols, x2);
        y2 = std::min((float)img.rows, y2);
        
        cv::Rect rect(x1, y1, x2 - x1, y2 - y1);
        cv::Scalar color = colors[res.det.class_id % colors.size()];
        
        cv::rectangle(img, rect, color, 2);
        
        std::string label;
        
        label =res.rec.text /*+ ": " + cv::format("%.3f", res.rec.score)*/;
        int baseline;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        
        // 调整标签框的位置，确保不超出图像顶部
        int text_y = std::max(rect.y, text_size.height + 5);
        cv::rectangle(img, 
                     cv::Point(rect.x, text_y - text_size.height - 5),
                     cv::Point(rect.x + text_size.width, text_y),
                     color, cv::FILLED);
        cv::putText(img, label, 
                   cv::Point(rect.x, text_y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        
        std::cout << label  <<":center_x,center_y :" << res.det.bbox[0] << "," << res.det.bbox[1] << std::endl;
        // 获取深度值（单位：毫米）
        int pixel_x = static_cast<int>(res.det.bbox[0]);
        int pixel_y = static_cast<int>(res.det.bbox[1]);
        
        if (pixel_x < 0 || pixel_x >= depth_img.cols || 
            pixel_y < 0 || pixel_y >= depth_img.rows) 
        {
            std::cout<<"像素坐标超出图像范围"<<std::endl;
            return;
        }
        float Z_mm = depth_img.at<ushort>(pixel_y, pixel_x); // 假设深度图是 CV_16UC1
        // 如果深度值为0，表示无效点
        if (Z_mm == 0)
        {
            std::cout << "Get Error Depth.d_x, d_y, d_z :" << 0 << "," << 0 << "," << 0 << std::endl;
        }
        /*
            X = (u − cx′) / fx′ × Z
            Y = (v − cy′) / fy′ × Z
        */

        float Z = Z_mm; // 毫米
        float X = (res.det.bbox[0] - intrinsics.cx) * Z / intrinsics.fx;
        float Y = (res.det.bbox[1] - intrinsics.cy) * Z / intrinsics.fy;
        std::cout << label << ":xPos,yPos,zPos :" << X << "," << Y << "," << Z << std::endl;
    }
}

void ElevatorButtonRec::crop_image(const cv::Mat& src_img, const Detection& det_res, cv::Mat& crop_img)
{
    // 解析中心点与宽高
    float cx = det_res.bbox[0];
    float cy = det_res.bbox[1];
    float w  = det_res.bbox[2];
    float h  = det_res.bbox[3];

    //计算左上角坐标（向下取整）
    int x0 = static_cast<int>(std::floor(cx - w / 2.0f + 0.5f));
    int y0 = static_cast<int>(std::floor(cy - h / 2.0f + 0.5f));

    //计算右下角坐标（向上取整，确保包含整像素）
    int x1 = static_cast<int>(std::ceil(cx + w / 2.0f - 0.5f));
    int y1 = static_cast<int>(std::ceil(cy + h / 2.0f - 0.5f));

    //边界裁剪，防止越界
    x0 = std::max(x0, 0);
    y0 = std::max(y0, 0);
    x1 = std::min(x1, src_img.cols);
    y1 = std::min(y1, src_img.rows);

    // 若区域非法，返回空 Mat
    if (x0 >= x1 || y0 >= y1)
    {
        crop_img = cv::Mat();
        return;
    }

    // 执行裁剪并返回
    crop_img = src_img(cv::Range(y0, y1), cv::Range(x0, x1)).clone();
}