#include "Yolo_Depth.h"
#include "Yolo_Det.h"
#include "utils.h"
#include <iostream>
#include <chrono>

#if 0
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <engine_path>" << std::endl;
        return -1;
    }
    
    try {
        DepthCameraYOLODetector detector(argv[1]);
        
        if (!detector.initialize()) {
            std::cerr << "Failed to initialize depth camera" << std::endl;
            return -1;
        }
        
        cv::namedWindow("Depth Camera + YOLOv11", cv::WINDOW_AUTOSIZE);
        
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();
            
            // 处理帧
            auto results = detector.processFrame();
            
            // 获取彩色图像用于显示
            rs2::frameset frames = detector.getCurrentFrames();
            rs2::frame color_frame = frames.get_color_frame();
            cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            
            // 可视化结果
            detector.visualize(color_image, results);
            
            // 计算并显示FPS
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            float fps = 1000.0f / duration;
            cv::putText(color_image, cv::format("FPS: %.2f", fps), cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            
            // 显示结果
            cv::imshow("Depth Camera + YOLOv11", color_image);
            
            // 退出条件
            if (cv::waitKey(1) == 'q') {
                break;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
#else
int main(int argc, char** argv) 
{
    // if (argc < 3) 
    // {
    //     std::cerr << "Usage: " << argv[0] << " <engine_file> <image_file> [output_file]" << std::endl;
    //     return 1;
    // }

    try 
    {
        // std::string engine_path = argv[1];
        // std::string image_path = argv[2];
        std::string engine_path = "/home/user/code/Yolo_Det/src/Yolo_Detection/models/switch_button_640.engine";
        std::string image_path = "/home/user/code/Yolo_Det/src/Yolo_Detection/test_switch_button_imgs/2025_09_11_12_01_28_740848.bmp";
        std::string output_path = argc > 3 ? argv[3] : "output.jpg";
        
        const std::vector<std::string> class_names = 
        {
            "on", "off"
        };
        // const std::vector<std::string> class_names = {
        //     "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        //     "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        //     "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        //     "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        //     "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        //     "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        //     "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
        //     "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
        //     "hair drier", "toothbrush" };
        // 加载检测器
        YoloDet detector;
        detector.load_engine(engine_path);

        // 读取图像
        cv::Mat img = cv::imread(image_path);
        if (img.empty()) 
        {
            throw std::runtime_error("Failed to load image: " + image_path);
        }

        // 执行检测
        auto start = std::chrono::high_resolution_clock::now();
        int nCount = 100;
        std::vector<Detection> detections;
        for(int i =0; i< nCount; i++)
        {
            detections = detector.infer(img);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        double ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
        // 输出结果
        // printf("Detected %d objects in %.2f ms\n",detections.size(), ave_time);
        std::cout << "Detected " << detections.size() << " objects in "
                  << ave_time << "ms" << std::endl;
        if (!detections.empty()) 
        {
            printf("==== 检测结果 ====\n");
            for (const auto& d : detections)
                printf("Box: [%.2f,%.2f,%.2f,%.2f] Conf=%.3f\n",d.bbox[0], d.bbox[1], d.bbox[2], d.bbox[3], d.conf);
        } 
        else 
            std::cout << "未检测到目标\n";
        // 绘制结果
        detector.draw_results(img, detections, class_names);
        
        // 保存/显示结果
        cv::imwrite(output_path, img);
        std::cout << "结果已保存到 output.jpg" << std::endl;
        // cv::imshow("Detection Results", img);
        // cv::waitKey(0);

    } 
    catch (const std::exception& e) 
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
#endif