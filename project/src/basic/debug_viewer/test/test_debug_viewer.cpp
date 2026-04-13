#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "DebugViewer.h"
#include "GCImage.h"
#include "HSDrawObj.h"
#include "TextDisp.h"
#include <filesystem>
#include <vector>

std::string findImageFile(const std::string& filename) {
    // 尝试多个可能的路径
    std::vector<std::string> paths = {
        "./" + filename,                                    // 当前目录
        "../image/" + filename,                             // 开发环境相对路径
        "../../share/debug_viewer/image/" + filename,       // 安装环境路径
        "./image/" + filename                                // 相对路径作为后备
    };
    
    for (const auto& path : paths) {
        if (std::filesystem::exists(path)) {
            return path;
        }
    }
    
    // 如果都找不到，返回当前目录下的路径
    return "./test.bmp";
}

int main() {
    try {
        using namespace DebugView;
        
        // 创建调试查看器实例
        CHsDebugViewer* pDbgViewer = new CHsDebugViewer();
        
        // 清空所有页面
        pDbgViewer->ClearAllPages();
        
        // 添加图形页面
        pDbgViewer->AddPage(PageTypeEnum::PAGE_OBJECT, "图像页面");
        
        // 加载测试图像
        GCImage* img = new GCImage();
        std::string imagePath = findImageFile("test.bmp");
        if (img->LoadImage(imagePath.c_str())) {
            std::cout << "成功加载图像" << std::endl;
            
            // 设置背景图像
            pDbgViewer->SetBgImage(0, img);
            
            // 添加一些图形对象（例如矩形）
            // 注意：这里需要根据实际的类定义来创建对象
            // 由于完整的类定义比较复杂，我们暂时跳过这一步
            
        } else {
            std::cout << "加载图像失败" << std::endl;
        }
        
        // 添加系统信息页面
        pDbgViewer->AddPage(PageTypeEnum::PAGE_INFO, "系统参数");
        pDbgViewer->SetBgColor(1, 245, 222, 180);
        
        // 添加文本信息
        HSV::TextUnionDraw text;
        text.lab_.color_ = HSV::GC_RGB(0, 0, 0);
        text.sbj_.color_ = HSV::GC_RGB(225, 10, 10);
        text.lab_.nFontSize_ = 30;
        text.sbj_.nFontSize_ = 30;
        
        text.lab_.strTxt_ = "AppInfo:";
        text.sbj_.strTxt_ = DebugView::CHsDebugViewer::GetAppInfo();
        pDbgViewer->AddInfo(1, &text);
        
        text.lab_.strTxt_ = "系统信息:";
        text.sbj_.strTxt_ = DebugView::CHsDebugViewer::GetSysInfo();
        pDbgViewer->AddInfo(1, &text);
        
        text.lab_.strTxt_ = "当前时间:";
        text.sbj_.strTxt_ = DebugView::CHsDebugViewer::GetCurTime();
        pDbgViewer->AddInfo(1, &text);
        
        // 添加结果页面
        pDbgViewer->AddPage(PageTypeEnum::PAGE_INFO, "检测结果");
        pDbgViewer->SetBgColor(2, 222, 255, 255);
        
        // 添加表格数据
        DebugView::double2D datas;
        std::vector<double> row1 = {100.5, 200.3, -0.85, 45.0};
        std::vector<double> row2 = {150.2, -180.7, 0.92, 30.5};
        std::vector<double> row3 = {120.8, 220.1, 0.78, 60.2};
        datas.push_back(row1);
        datas.push_back(row2);
        datas.push_back(row3);
        
        std::vector<std::string> horTitle = {"X", "Y", "Score", "Angle"};
        std::vector<std::string> verTitle = {"1", "2", "3"};
        
        pDbgViewer->AddTableInfo(2, datas, horTitle, verTitle);
        
        // 设置表格颜色
        TextDrawPara paraData = TextDrawPara(30, 0, 0, 0);      // 数据字体颜色
        TextDrawPara paraTitleH = TextDrawPara(30, 0, 0, 255);  // 水平标题
        TextDrawPara paraTitleV = TextDrawPara(30, 0, 0, 255);  // 垂直标题
        ObjDrawPara paraLine = ObjDrawPara(1, 0, 0, 255);       // 表格线条
        
        pDbgViewer->SetPageTableDraw(2, 0, paraTitleH, paraTitleV, paraData, paraLine);
        
        // 处理所有页面
        pDbgViewer->ProcAllPages();
        
        // 获取结果图像
        const GCImage* rstImage = nullptr;
        if (pDbgViewer->GetResultImage(rstImage) == 0 && rstImage != nullptr) {
            std::cout << "成功生成结果图像" << std::endl;
            // 保存结果图像
            // 注意：这里需要确保目录存在
            pDbgViewer->SaveRstImages("./results/");
            std::cout << "结果图像已保存到 ./results/ 目录" << std::endl;
        }
        
        // 清理资源
        delete img;
        delete pDbgViewer;
        
        std::cout << "测试完成!" << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "发生异常: " << e.what() << std::endl;
        return -1;
    }
    catch (...) {
        std::cerr << "发生未知异常" << std::endl;
        return -1;
    }
}