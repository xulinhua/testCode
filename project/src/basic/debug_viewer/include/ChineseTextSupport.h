// ChineseTextSupport.h
#pragma once

#include <string>
#include <memory>
#include <iostream>
#include <vector>
#include <sstream>
#include <map>

// 包含OpenCV（在Windows头文件之前，避免宏冲突）
#include <opencv2/opencv.hpp>

// 包含FreeType
#include <freetype2/ft2build.h>
#include <freetype/freetype.h>

#ifdef _WIN32
#ifdef _DEBUG
#ifdef _M_X64
#pragma comment(lib,"freetype64d.lib")
#else
#pragma comment(lib,"freetyped.lib")
#endif 
#else
#ifdef _M_X64
#pragma comment(lib,"freetype64.lib")
#else
#pragma comment(lib,"freetype.lib")
#endif 
#endif
#endif // _WIN32

using namespace cv;
using namespace std;

class ChineseTextRenderer
{
private:
    // FreeType2 对象
    FT_Library ftLibrary;
    FT_Face ftFace;
    bool ftInitialized;
    bool fontLoaded;

    // 字体缓存
    std::map<std::pair<int, int>, FT_Size_Metrics> metricsCache;

public:
    ChineseTextRenderer();
    ~ChineseTextRenderer();

    // 禁用拷贝
    ChineseTextRenderer(const ChineseTextRenderer&) = delete;
    ChineseTextRenderer& operator=(const ChineseTextRenderer&) = delete;

    // 支持移动语义
    ChineseTextRenderer(ChineseTextRenderer&& other) noexcept;
    ChineseTextRenderer& operator=(ChineseTextRenderer&& other) noexcept;

    bool loadFont(const std::string& fontPath);
    bool autoLoadFont();
    cv::Size getTextSize(const std::string& text, int fontSize, int thickness = 1);

    void putText(cv::Mat& img, const std::string& text, cv::Point pt,
        int fontSize, cv::Scalar color, int thickness = 1,
        int line_type = 8, bool bottomLeftOrigin = false);

    void putTextWithBackground(cv::Mat& img, const std::string& text, cv::Point pt,
        int fontSize, cv::Scalar color, cv::Scalar bgColor,
        int thickness = 1, int padding = 5);

    bool isFontLoaded() const;

    // UTF-8字符串处理
    static std::vector<uint32_t> utf8ToUnicode(const std::string& utf8_str);
    static std::string unicodeToUtf8(const std::vector<uint32_t>& unicode_str);

private:
    // 内部实现方法
    void renderTextInternal(cv::Mat& img, const std::string& text, cv::Point pt,
        int fontSize, cv::Scalar color, int thickness);

    cv::Size calculateTextSizeInternal(const std::string& text, int fontSize, int thickness);

    void blendPixel(cv::Mat& img, int y, int x, cv::Scalar color, float alpha);

    void setFontSize(int fontSize);

    // 获取字形位图
    bool getGlyphBitmap(FT_Face face, uint32_t charCode, FT_GlyphSlot& glyph, FT_Bitmap& bitmap);

    // 清理缓存
    void clearCache();

    // 字体文件路径
    std::string currentFontPath;

};

// 全局渲染器实例
extern std::shared_ptr<ChineseTextRenderer> g_chineseRenderer;

namespace ChineseTextUtils {
    // 工具函数
    bool ContainsChinese(const std::string& str);
    void InitChineseRenderer();
    void DrawChineseText(cv::Mat& img, const std::string& text, cv::Point pt,
        int fontSize, cv::Scalar color, int thickness = 1);

    void DrawChineseTextWithBackground(cv::Mat& img, const std::string& text, cv::Point pt,
        int fontSize, cv::Scalar color, cv::Scalar bgColor,
        int thickness = 1, int padding = 5);

    // 跨平台的字体查找工具
    std::string FindSystemFont(const std::vector<std::string>& preferredFonts);
    std::vector<std::string> GetSystemFontPaths();

    // 字体相关函数
    int GetOpenCVFontType(const char* fn, bool bItalic, bool bUnderline);
    double GetFontScale(int fontSize);
    void CalculateTextSize(const std::string& str, int fontFace, double fontScale,
        int thickness, int& strBaseW, int& strBaseH, int& singleRow);

}