// ChineseTextSupport.cpp
#define _CRT_SECURE_NO_WARNINGS
// 添加头文件
#ifdef _WIN32
#include <Windows.h>
#else
// Linux 平台特定的头文件
#include <unistd.h>
#include <sys/stat.h>
#include <iconv.h>
#endif

#include "ChineseTextSupport.h"

// 使用C++17的filesystem
#if __cplusplus >= 201703L
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#include <fstream>
#include <algorithm>
#include <cmath>
#include <cstring>
// #include <codecvt>  // 已弃用
#include <locale>
// 平台检测
#ifdef _WIN32
#define PLATFORM_WINDOWS 1
#else
#define PLATFORM_WINDOWS 0
#endif

// 创建一个跨平台的调试输出函数
static void DebugOutput(const std::string& message) {
#ifdef _WIN32
    // Windows: 输出到调试器（在VS输出窗口可见）
    OutputDebugStringA(message.c_str());
    OutputDebugStringA("\n");

    // 同时输出到控制台（如果有）
    std::cout << message << std::endl;
#else
    // Linux/Unix: 正常输出
    std::cout << message << std::endl;
#include <iconv.h>
#endif
}

std::shared_ptr<ChineseTextRenderer> g_chineseRenderer;

// 全局中文渲染器
static uint32_t decodeUTF8Original(const char*& str) {
    if (!str || !*str) return 0;

    unsigned char c = static_cast<unsigned char>(*str);

    // 1字节：0xxxxxxx
    if (c < 0x80) {
        str++;
        return c;
    }

    // 2字节：110xxxxx 10xxxxxx
    if ((c & 0xE0) == 0xC0) {
        if (!*(str + 1)) return 0;

        uint32_t codepoint = ((c & 0x1F) << 6) | (*(str + 1) & 0x3F);
        str += 2;
        return codepoint;
    }

    // 3字节：1110xxxx 10xxxxxx 10xxxxxx（中文字符在这里）
    if ((c & 0xF0) == 0xE0) {
        if (!*(str + 1) || !*(str + 2)) return 0;

        uint32_t codepoint = ((c & 0x0F) << 12) |
            ((*(str + 1) & 0x3F) << 6) |
            (*(str + 2) & 0x3F);
        str += 3;
        return codepoint;
    }

    // 4字节：11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
    if ((c & 0xF8) == 0xF0) {
        if (!*(str + 1) || !*(str + 2) || !*(str + 3)) return 0;

        uint32_t codepoint = ((c & 0x07) << 18) |
            ((*(str + 1) & 0x3F) << 12) |
            ((*(str + 2) & 0x3F) << 6) |
            (*(str + 3) & 0x3F);
        str += 4;
        return codepoint;
    }

    // 无效UTF-8序列
    str++;
    return 0;
}
// 在 ChineseTextSupport.cpp 中替换
static uint32_t decodeUTF8(const char*& str) {
    return decodeUTF8Original(str);
}
// UTF-8有效性检查函数
static bool isValidUTF8(const std::string& str) {
    const unsigned char* s = (const unsigned char*)str.c_str();
    while (*s) {
        if (*s < 0x80) {
            // ASCII字符
            s++;
        } else if ((*s & 0xE0) == 0xC0) {
            // 2字节UTF-8
            if (*(s + 1) && (*(s + 1) & 0xC0) == 0x80) {
                s += 2;
            } else {
                return false;
            }
        } else if ((*s & 0xF0) == 0xE0) {
            // 3字节UTF-8 (中文字符)
            if (*(s + 1) && *(s + 2) && 
                (*(s + 1) & 0xC0) == 0x80 && 
                (*(s + 2) & 0xC0) == 0x80) {
                s += 3;
            } else {
                return false;
            }
        } else if ((*s & 0xF8) == 0xF0) {
            // 4字节UTF-8
            if (*(s + 1) && *(s + 2) && *(s + 3) &&
                (*(s + 1) & 0xC0) == 0x80 && 
                (*(s + 2) & 0xC0) == 0x80 && 
                (*(s + 3) & 0xC0) == 0x80) {
                s += 4;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
    return true;
}

// GBK解码函数
static uint32_t decodeGBK(const unsigned char*& str) {
    if (!str || !*str) return 0;

    unsigned char c = *str;

    // ASCII字符
    if (c < 0x80) {
        str++;
        return c;
    }

    // GBK双字节字符
    if (c >= 0x81 && c <= 0xFE) {
        unsigned char c2 = *(str + 1);
        if (!c2) {
            str++;
            return 0;
        }

        // 使用系统API进行转换
#ifdef _WIN32
// Windows环境
        wchar_t wideChar;
        char gbkBytes[3] = { (char)c, (char)c2, 0 };

        // GBK到UTF-16转换
        int len = MultiByteToWideChar(936, 0, gbkBytes, 2, &wideChar, 1);
        if (len > 0) {
            str += 2;
            return (uint32_t)wideChar;
        }
#else
// Linux/Ubuntu环境
        // 使用iconv进行正确的GBK到UTF-8转换
        char inbuf[3] = { (char)c, (char)c2, '\0' };
        char outbuf[8] = {0};
        size_t inbytesleft = 2;
        size_t outbytesleft = sizeof(outbuf);
        char* inptr = inbuf;
        char* outptr = outbuf;

        iconv_t cd = iconv_open("UTF-8", "GBK");
        if (cd != (iconv_t)-1) {
            if (iconv(cd, &inptr, &inbytesleft, &outptr, &outbytesleft) != (size_t)-1) {
                // 成功转换，现在解析UTF-8
                const char* utf8str = outbuf;
                uint32_t codepoint = decodeUTF8Original(utf8str);
                iconv_close(cd);
                str += 2;
                return codepoint;
            }
            iconv_close(cd);
        }
        // 如果转换失败，回退到简单方法
        unsigned int codepoint = (c << 8) | c2;
        str += 2;
        return codepoint;
#endif

        str += 2;
        return 0;
    }

    // 未知字符
    str++;
    return 0;
}
// 检测是否为GBK编码
static bool isGBKString(const std::string& str) {
    // 首先检查是否为有效的UTF-8
    if (isValidUTF8(str)) {
        return false; // 优先认为是UTF-8
    }
    
    const unsigned char* s = (const unsigned char*)str.c_str();
    bool hasGBK = false;

    while (*s) {
        // GBK字符的第一个字节在0x81-0xFE之间
        if (*s >= 0x81 && *s <= 0xFE) {
            hasGBK = true;
            // 检查第二个字节
            if (*(s + 1) >= 0x40 && *(s + 1) <= 0xFE && *(s + 1) != 0x7F) {
                return true;
            }
        }
        s++;
    }

    return hasGBK;
}

// 智能解码函数，自动选择编码
static uint32_t decodeAuto(const char*& str) {
    const unsigned char* ustr = (const unsigned char*)str;

    // 检查第一个字符的编码
    if (*ustr < 0x80) {
        // ASCII
        str++;
        return *ustr;
    }
    else if ((*ustr & 0xE0) == 0xC0) {
        // 可能是UTF-8 2字节
        return decodeUTF8(str);
    }
    else if ((*ustr & 0xF0) == 0xE0) {
        // 可能是UTF-8 3字节
        return decodeUTF8(str);
    }
    else if (*ustr >= 0x81 && *ustr <= 0xFE) {
        // 可能是GBK
        return decodeGBK(ustr);
        str = (const char*)ustr; // 更新指针
    }

    // 未知编码
    str++;
    return 0;
}

// UTF-8编码辅助函数
static std::string encodeUTF8(uint32_t codepoint) {
    std::string result;

    if (codepoint <= 0x7F) {
        result.push_back(static_cast<char>(codepoint));
    }
    else if (codepoint <= 0x7FF) {
        result.push_back(static_cast<char>(0xC0 | (codepoint >> 6)));
        result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    }
    else if (codepoint <= 0xFFFF) {
        result.push_back(static_cast<char>(0xE0 | (codepoint >> 12)));
        result.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
        result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    }
    else if (codepoint <= 0x10FFFF) {
        result.push_back(static_cast<char>(0xF0 | (codepoint >> 18)));
        result.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3F)));
        result.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
        result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    }

    return result;
}

// 跨平台获取环境变量
static std::string GetEnv(const std::string& name) {
    const char* value = std::getenv(name.c_str());
    return value ? std::string(value) : std::string();
}

// 跨平台检查文件是否存在
static bool FileExists(const std::string& path) {
    std::ifstream file(path);
    return file.good();
}

// ==================== ChineseTextRenderer 实现 ====================

ChineseTextRenderer::ChineseTextRenderer()
    : ftLibrary(nullptr)
    , ftFace(nullptr)
    , ftInitialized(false)
    , fontLoaded(false) {

    // 初始化FreeType库
    FT_Error error = FT_Init_FreeType(&ftLibrary);
    if (error) {
        ftInitialized = false;
    }
    else {
        ftInitialized = true;
    }
}

ChineseTextRenderer::~ChineseTextRenderer() {
    // 清理字形缓存
    clearCache();

    // 释放字体
    if (ftFace) {
        FT_Done_Face(ftFace);
        ftFace = nullptr;
    }

    // 释放FreeType库
    if (ftLibrary) {
        FT_Done_FreeType(ftLibrary);
        ftLibrary = nullptr;
    }
}

// 移动构造函数
ChineseTextRenderer::ChineseTextRenderer(ChineseTextRenderer&& other) noexcept
    : ftLibrary(other.ftLibrary)
    , ftFace(other.ftFace)
    , ftInitialized(other.ftInitialized)
    , fontLoaded(other.fontLoaded)
    , metricsCache(std::move(other.metricsCache))
    , currentFontPath(std::move(other.currentFontPath)) {

    other.ftLibrary = nullptr;
    other.ftFace = nullptr;
    other.ftInitialized = false;
    other.fontLoaded = false;
}

// 移动赋值运算符
ChineseTextRenderer& ChineseTextRenderer::operator=(ChineseTextRenderer&& other) noexcept {
    if (this != &other) {
        // 清理当前资源
        clearCache();
        if (ftFace) FT_Done_Face(ftFace);
        if (ftLibrary) FT_Done_FreeType(ftLibrary);

        // 转移资源
        ftLibrary = other.ftLibrary;
        ftFace = other.ftFace;
        ftInitialized = other.ftInitialized;
        fontLoaded = other.fontLoaded;
        metricsCache = std::move(other.metricsCache);
        currentFontPath = std::move(other.currentFontPath);

        // 置空原对象
        other.ftLibrary = nullptr;
        other.ftFace = nullptr;
        other.ftInitialized = false;
        other.fontLoaded = false;
        other.metricsCache.clear();
        other.currentFontPath.clear();
    }
    return *this;
}

bool ChineseTextRenderer::loadFont(const std::string& fontPath) {
    if (!ftInitialized) {
        return false;
    }

    // 检查文件是否存在
    if (!FileExists(fontPath)) {
        return false;
    }

    // 清理之前的字体
    if (ftFace) {
        FT_Done_Face(ftFace);
        ftFace = nullptr;
        clearCache();
    }

    // 加载新字体
    FT_Error error = FT_New_Face(ftLibrary, fontPath.c_str(), 0, &ftFace);
    if (error) {
        fontLoaded = false;
        return false;
    }

    // 设置默认字体大小
    setFontSize(20);

    currentFontPath = fontPath;
    fontLoaded = true;

    return true;
}

bool ChineseTextRenderer::autoLoadFont() {
    std::vector<std::string> fontPaths;

#ifdef _WIN32
    // Windows系统字体
    fontPaths = {
        "C:\\Windows\\Fonts\\msyh.ttc",      // 微软雅黑
        "C:\\Windows\\Fonts\\simsun.ttc",    // 宋体
        "C:\\Windows\\Fonts\\simhei.ttf",    // 黑体
        "C:\\Windows\\Fonts\\simkai.ttf",    // 楷体
        "C:\\Windows\\Fonts\\simfang.ttf",   // 仿宋
        "C:\\Windows\\Fonts\\Deng.ttf",      // 等线
    };
#else
    // Linux系统字体
    fontPaths = {
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    };
#endif

    for (const auto& path : fontPaths) {

        std::ifstream file(path);
        if (file.good()) {
            if (loadFont(path)) {

                // 验证字体是否包含中文
                FT_Face face = ftFace;
                bool hasChinese = false;

                // 测试几个中文字符
                std::vector<uint32_t> testChars = { 0x6A21, 0x677F, 0x4E2D, 0x6587 };
                for (uint32_t charCode : testChars) {
                    if (FT_Get_Char_Index(face, charCode) != 0) {
                        hasChinese = true;
                        break;
                    }
                }

                if (hasChinese) {
                    return true;
                }
                else {
                }
            }
        }
        else {
        }
    }

    return false;
}


void ChineseTextRenderer::setFontSize(int fontSize) 
{
    if (!ftFace || !fontLoaded) return;

    FT_Error error = FT_Set_Pixel_Sizes(ftFace, 0, fontSize);
    if (error) {
    }
}

bool ChineseTextRenderer::getGlyphBitmap(FT_Face face, uint32_t charCode, FT_GlyphSlot& glyph, FT_Bitmap& bitmap) {
    if (!face) {
        return false;
    }

    // 获取字符的字形索引
    FT_UInt glyph_index = FT_Get_Char_Index(face, charCode);

    if (glyph_index == 0) {

        // 尝试使用缺失字形
        glyph_index = FT_Get_Char_Index(face, 0); // 索引0通常是.notdef
        if (glyph_index == 0) {
            // 尝试使用问号
            glyph_index = FT_Get_Char_Index(face, '?');
            if (glyph_index == 0) {
                return false;
            }
        }
        else {
        }
    }
    else {
    }

    // 加载字形
    FT_Error error = FT_Load_Glyph(face, glyph_index, FT_LOAD_DEFAULT);
    if (error) {
        return false;
    }

    // 渲染字形到位图
    error = FT_Render_Glyph(face->glyph, FT_RENDER_MODE_NORMAL);
    if (error) {
        return false;
    }

    glyph = face->glyph;
    bitmap = glyph->bitmap;


    return true;
}
cv::Size ChineseTextRenderer::calculateTextSizeInternal(const std::string& text, int fontSize, int thickness) {
    if (!ftFace || !fontLoaded) {
        return cv::Size(0, 0);
    }

    setFontSize(fontSize);

    int totalWidth = 0;
    int maxAscent = 0;
    int maxDescent = 0;

    const char* str = text.c_str();
    while (*str) {
        uint32_t codepoint = decodeUTF8(str);
        if (codepoint == 0) break;

        FT_GlyphSlot glyph;
        FT_Bitmap bitmap;

        if (getGlyphBitmap(ftFace, codepoint, glyph, bitmap)) {
            totalWidth += glyph->advance.x >> 6;

            int ascent = glyph->bitmap_top;
            int descent = bitmap.rows - ascent;

            maxAscent = max(maxAscent, ascent);
            maxDescent = max(maxDescent, descent);
        }
    }

    // 添加厚度影响
    totalWidth += thickness * 2;
    int totalHeight = maxAscent + maxDescent + thickness * 2;

    return cv::Size(totalWidth, totalHeight);
}

void ChineseTextRenderer::blendPixel(cv::Mat& img, int y, int x, cv::Scalar color, float alpha) {
    if (y < 0 || y >= img.rows || x < 0 || x >= img.cols) {
        return;
    }

    if (alpha <= 0.0f) return;
    if (alpha > 1.0f) alpha = 1.0f;

    if (img.channels() == 3) {
        // BGR图像
        cv::Vec3b& pixel = img.at<cv::Vec3b>(y, x);
        pixel[0] = static_cast<uchar>(color[0] * alpha + pixel[0] * (1 - alpha)); // B
        pixel[1] = static_cast<uchar>(color[1] * alpha + pixel[1] * (1 - alpha)); // G
        pixel[2] = static_cast<uchar>(color[2] * alpha + pixel[2] * (1 - alpha)); // R
    }
    else if (img.channels() == 4) {
        // BGRA图像
        cv::Vec4b& pixel = img.at<cv::Vec4b>(y, x);
        pixel[0] = static_cast<uchar>(color[0] * alpha + pixel[0] * (1 - alpha));
        pixel[1] = static_cast<uchar>(color[1] * alpha + pixel[1] * (1 - alpha));
        pixel[2] = static_cast<uchar>(color[2] * alpha + pixel[2] * (1 - alpha));
        // Alpha通道保持不变
    }
    else if (img.channels() == 1) {
        // 灰度图像
        uchar& pixel = img.at<uchar>(y, x);
        uchar gray = static_cast<uchar>((color[0] + color[1] + color[2]) / 3);
        pixel = static_cast<uchar>(gray * alpha + pixel * (1 - alpha));
    }
}

void ChineseTextRenderer::renderTextInternal(cv::Mat& img, const std::string& text, cv::Point pt,
    int fontSize, cv::Scalar color, int thickness) {
    if (!ftFace || !fontLoaded || text.empty()) {
        //DebugOutput("渲染失败: 字体未加载或文本为空");
        return;
    }

    for (char c : text) {
        char buf[8];
        sprintf(buf, "%02X ", (unsigned char)c);
    }

    // 改进编码检测逻辑：优先尝试UTF-8
    bool isGBK = false;
    if (isValidUTF8(text)) {
    } else {
        isGBK = isGBKString(text);
    }

    setFontSize(fontSize);

    int x = pt.x;
    int baseline = pt.y;
    int charCount = 0;

    if (isGBK) {
        // GBK编码处理
        const unsigned char* str = (const unsigned char*)text.c_str();

        while (*str) {
            const unsigned char* charStart = str;
            uint32_t codepoint = decodeGBK(str);

            if (codepoint == 0) {
                char buf[8];
                sprintf(buf, "%02X ", *charStart);
                continue;
            }

            charCount++;
            //ss << "-> U+" << std::hex << codepoint << std::dec;
            //DebugOutput(ss.str());
            //ss.str("");

            FT_GlyphSlot glyph;
            FT_Bitmap bitmap;

            if (getGlyphBitmap(ftFace, codepoint, glyph, bitmap)) {

                // 计算字符绘制位置
                int charX = x + glyph->bitmap_left;
                //int charY = baseline + glyph->bitmap_top - bitmap.rows;//顶端对齐
                // 对负号字符进行特殊处理，使其在基线基础上往上移动
                int charY;
                if (codepoint == 0x002D) {  // 负号Unicode码点
                    charY = baseline - (fontSize / 3);  // 往上移动约字体大小的1/4
                } else {
                    charY = baseline;
                }

                // 绘制调试框（用灰色）
                /*cv::rectangle(img, cv::Rect(charX, charY, bitmap.width, bitmap.rows),
                    cv::Scalar(200, 200, 200), 1);*/

                // 绘制字形
                int pixelsDrawn = 0;
                for (int row = 0; row < bitmap.rows; row++) {
                    for (int col = 0; col < bitmap.width; col++) {
                        int imgX = charX + col;
                        //int imgY = charY + row;
                        int imgY = charY - bitmap.rows + row;

                        if (imgX >= 0 && imgX < img.cols && imgY >= 0 && imgY < img.rows) {
                            unsigned char alpha = bitmap.buffer[row * bitmap.pitch + col];

                            if (alpha > 0) {
                                // 根据厚度调整绘制方式
                                if (thickness > 1) {
                                    // 绘制粗体字符
                                    for (int t = 0; t < thickness; t++) {
                                        int offsetX = imgX + t;
                                        int offsetY = imgY + t;
                                        if (offsetX < img.cols && offsetY < img.rows) {
                                            if (img.channels() == 3) {
                                                cv::Vec3b& pixel = img.at<cv::Vec3b>(offsetY, offsetX);
                                                pixel[0] = color[0];
                                                pixel[1] = color[1];
                                                pixel[2] = color[2];
                                            }
                                        }
                                    }
                                }
                                else {
                                    if (img.channels() == 3) {
                                        cv::Vec3b& pixel = img.at<cv::Vec3b>(imgY, imgX);
                                        pixel[0] = color[0];
                                        pixel[1] = color[1];
                                        pixel[2] = color[2];
                                    }
                                }
                                pixelsDrawn++;
                            }
                        }
                    }
                }


                // 移动到下一个字符位置
                x += glyph->advance.x >> 6;

            }
            else {
                DebugOutput("  无法获取字形位图");
                x += fontSize;  // 估算宽度
            }
        }

    }
    else {
        // UTF-8编码处理（保持原有逻辑）
        const char* str = text.c_str();

        while (*str) {
            const char* charStart = str;
            uint32_t codepoint = decodeUTF8(str);

            if (codepoint == 0) {
                //DebugOutput("UTF-8解码失败");
                break;
            }

            charCount++;

            // 显示解码信息
            for (const char* p = charStart; p < str; p++) {
                char buf[8];
                sprintf(buf, "%02X ", (unsigned char)*p);
            }

            FT_GlyphSlot glyph;
            FT_Bitmap bitmap;

            if (getGlyphBitmap(ftFace, codepoint, glyph, bitmap)) {

                // 计算字符绘制位置
                int charX = x + glyph->bitmap_left;
                //int charY = baseline + glyph->bitmap_top - bitmap.rows;//顶端对齐
                // 对负号字符进行特殊处理，使其在基线基础上往上移动
                int charY;
                if (codepoint == 0x002D) {  // 负号Unicode码点
                    charY = baseline - (fontSize / 3);  // 往上移动约字体大小的1/4
                } else {
                    charY = baseline;
                }

                // 绘制调试框（用灰色）
              /*  cv::rectangle(img, cv::Rect(charX, charY, bitmap.width, bitmap.rows),
                    cv::Scalar(200, 200, 200), 1);*/

                // 绘制字形
                int pixelsDrawn = 0;
                for (int row = 0; row < bitmap.rows; row++) {
                    for (int col = 0; col < bitmap.width; col++) {
                        int imgX = charX + col;
                        //int imgY = charY + row;//顶端对齐
                        int imgY = charY - bitmap.rows + row;
                        if (imgX >= 0 && imgX < img.cols && imgY >= 0 && imgY < img.rows) {
                            unsigned char alpha = bitmap.buffer[row * bitmap.pitch + col];

                            if (alpha > 0) {
                                // 根据厚度调整绘制方式
                                if (thickness > 1) {
                                    // 绘制粗体字符
                                    for (int t = 0; t < thickness; t++) {
                                        int offsetX = imgX + t;
                                        int offsetY = imgY + t;
                                        if (offsetX < img.cols && offsetY < img.rows) {
                                            if (img.channels() == 3) {
                                                cv::Vec3b& pixel = img.at<cv::Vec3b>(offsetY, offsetX);
                                                pixel[0] = color[0];
                                                pixel[1] = color[1];
                                                pixel[2] = color[2];
                                            }
                                        }
                                    }
                                }
                                else {
                                    if (img.channels() == 3) {
                                        cv::Vec3b& pixel = img.at<cv::Vec3b>(imgY, imgX);
                                        pixel[0] = color[0];
                                        pixel[1] = color[1];
                                        pixel[2] = color[2];
                                    }
                                }
                                pixelsDrawn++;
                            }
                        }
                    }
                }


                // 移动到下一个字符位置
                x += glyph->advance.x >> 6;

            }
            else {
                DebugOutput("  无法获取字形位图");
                x += fontSize;  // 估算宽度
            }
        }
    }


    // 标记原始点（用绿色）
   // cv::circle(img, pt, 5, cv::Scalar(0, 255, 0), -1);
}
void ChineseTextRenderer::clearCache() {
    metricsCache.clear();
}

cv::Size ChineseTextRenderer::getTextSize(const std::string& text, int fontSize, int thickness) {
    if (!isFontLoaded()) {
        return cv::Size(0, 0);
    }

    // 检查缓存
    auto key = std::make_pair(fontSize, thickness);
    if (metricsCache.find(key) == metricsCache.end()) {
        // 计算并缓存
        setFontSize(fontSize);
        metricsCache[key] = ftFace->size->metrics;
    }

    return calculateTextSizeInternal(text, fontSize, thickness);
}

void ChineseTextRenderer::putText(cv::Mat& img, const std::string& text, cv::Point pt,
    int fontSize, cv::Scalar color, int thickness,
    int line_type, bool bottomLeftOrigin) {
    if (!isFontLoaded() || text.empty()) {
        return;
    }

    // 调整原点位置
    cv::Point actualPt = pt;
    if (bottomLeftOrigin) {
        cv::Size textSize = getTextSize(text, fontSize, thickness);
        actualPt.y -= textSize.height;
    }

    renderTextInternal(img, text, actualPt, fontSize, color, thickness);
}

void ChineseTextRenderer::putTextWithBackground(cv::Mat& img, const std::string& text, cv::Point pt,
    int fontSize, cv::Scalar color, cv::Scalar bgColor,
    int thickness, int padding) {
    if (!isFontLoaded() || text.empty()) {
        return;
    }

    // 计算文本尺寸
    cv::Size textSize = getTextSize(text, fontSize, thickness);

    // 计算背景矩形
    cv::Rect bgRect(pt.x - padding,
        pt.y - textSize.height + padding,
        textSize.width + padding * 2,
        textSize.height + padding * 2);

    // 绘制背景
    cv::rectangle(img, bgRect, bgColor, -1);

    // 绘制文字
    cv::Point textPt(pt.x, pt.y + textSize.height - padding);
    renderTextInternal(img, text, textPt, fontSize, color, thickness);
}

bool ChineseTextRenderer::isFontLoaded() const {
    return fontLoaded && ftFace != nullptr;
}

std::vector<uint32_t> ChineseTextRenderer::utf8ToUnicode(const std::string& utf8_str) {
    std::vector<uint32_t> result;
    const char* str = utf8_str.c_str();

    while (*str) {
        result.push_back(decodeUTF8(str));
    }

    return result;
}

std::string ChineseTextRenderer::unicodeToUtf8(const std::vector<uint32_t>& unicode_str) {
    std::string result;

    for (uint32_t codepoint : unicode_str) {
        result += encodeUTF8(codepoint);
    }

    return result;
}

namespace ChineseTextUtils {
    bool ContainsChinese(const std::string& str) {
        const char* s = str.c_str();
        while (*s) {
            uint32_t codepoint = decodeUTF8(s);
            // 中文字符的Unicode范围
            if ((codepoint >= 0x4E00 && codepoint <= 0x9FFF) ||     // CJK统一表意文字
                (codepoint >= 0x3400 && codepoint <= 0x4DBF) ||     // CJK扩展A
                (codepoint >= 0x20000 && codepoint <= 0x2A6DF) ||   // CJK扩展B
                (codepoint >= 0xF900 && codepoint <= 0xFAFF) ||     // CJK兼容象形文字
                (codepoint >= 0x2F800 && codepoint <= 0x2FA1F))
            {   // CJK兼容补充
                return true;
            }
        }
        return false;
    }

    void InitChineseRenderer() {
        if (!g_chineseRenderer) {
            g_chineseRenderer = std::make_shared<ChineseTextRenderer>();
            if (!g_chineseRenderer->autoLoadFont()) {
                std::cerr << "警告：中文渲染器初始化失败，将使用OpenCV默认字体" << std::endl;
            }
        }
    }

    void DrawChineseText(cv::Mat& img, const std::string& text, cv::Point pt,
        int fontSize, cv::Scalar color, int thickness) {
        InitChineseRenderer();

        if (g_chineseRenderer && g_chineseRenderer->isFontLoaded()) {
            g_chineseRenderer->putText(img, text, pt, fontSize, color, thickness);
        }
        else {
            // 回退到OpenCV的putText（对中文支持有限）
            cv::putText(img, text, pt, cv::FONT_HERSHEY_SIMPLEX,
                fontSize / 20.0, color, thickness);
        }
    }

    void DrawChineseTextWithBackground(cv::Mat& img, const std::string& text, cv::Point pt,
        int fontSize, cv::Scalar color, cv::Scalar bgColor,
        int thickness, int padding) {
        InitChineseRenderer();

        if (g_chineseRenderer && g_chineseRenderer->isFontLoaded()) {
            g_chineseRenderer->putTextWithBackground(img, text, pt, fontSize, color, bgColor, thickness, padding);
        }
        else {
            // 回退实现
            cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX,
                fontSize / 20.0, thickness, nullptr);
            cv::rectangle(img, cv::Rect(pt.x - padding, pt.y - textSize.height - padding,
                textSize.width + 2 * padding,
                textSize.height + 2 * padding),
                bgColor, cv::FILLED);
            cv::putText(img, text, cv::Point(pt.x, pt.y + textSize.height - padding),
                cv::FONT_HERSHEY_SIMPLEX, fontSize / 20.0, color, thickness);
        }
    }

    // 跨平台的系统字体路径获取
    std::vector<std::string> GetSystemFontPaths() {
        std::vector<std::string> paths;

        if (PLATFORM_WINDOWS) {
            // Windows字体目录
            paths.push_back("C:\\Windows\\Fonts\\");

            // 尝试从注册表或环境变量获取更多路径
            std::string programFiles = GetEnv("ProgramFiles");
            if (!programFiles.empty()) {
                paths.push_back(programFiles + "\\Microsoft Office\\Fonts\\");
            }

            std::string localAppData = GetEnv("LOCALAPPDATA");
            if (!localAppData.empty()) {
                paths.push_back(localAppData + "\\Microsoft\\Windows\\Fonts\\");
            }
        }
        else {
            // Linux/Unix字体目录
            paths.push_back("/usr/share/fonts/");
            paths.push_back("/usr/local/share/fonts/");
            paths.push_back("/usr/X11R6/lib/X11/fonts/");

            // 用户字体目录
            std::string homeDir = GetEnv("HOME");
            if (!homeDir.empty()) {
                paths.push_back(homeDir + "/.fonts/");
                paths.push_back(homeDir + "/.local/share/fonts/");
            }

            // macOS字体目录（如果编译为macOS）
#ifdef __APPLE__
            paths.push_back("/System/Library/Fonts/");
            paths.push_back("/Library/Fonts/");
            paths.push_back("~/Library/Fonts/");
#endif
        }

        return paths;
    }

    std::string FindSystemFont(const std::vector<std::string>& preferredFonts) {
        std::vector<std::string> systemPaths = GetSystemFontPaths();

        for (const auto& fontName : preferredFonts) {
            for (const auto& path : systemPaths) {
                std::string fullPath = path + fontName;
                if (FileExists(fullPath)) {
                    return fullPath;
                }
            }
        }

        return "";
    }

    // 字体相关函数
    int GetOpenCVFontType(const char* fn, bool bItalic, bool bUnderline) {
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;

        if (fn != nullptr) {
            std::string fontName(fn);
            if (fontName.find("plain") != std::string::npos) {
                fontFace = cv::FONT_HERSHEY_PLAIN;
            }
            else if (fontName.find("complex") != std::string::npos) {
                fontFace = cv::FONT_HERSHEY_COMPLEX;
            }
            else if (fontName.find("triplex") != std::string::npos) {
                fontFace = cv::FONT_HERSHEY_TRIPLEX;
            }
            else if (fontName.find("script") != std::string::npos) {
                fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
            }
        }

        if (bItalic) {
            fontFace |= cv::FONT_ITALIC;
        }

        return fontFace;
    }

    double GetFontScale(int fontSize) {
        return fontSize / 50.0;
    }

    void CalculateTextSize(const std::string& str, int fontFace, double fontScale,
        int thickness, int& strBaseW, int& strBaseH, int& singleRow) {
        strBaseW = 0;
        strBaseH = 0;
        singleRow = 0;

        std::stringstream ss(str);
        std::string line;
        int lineCount = 0;

        while (std::getline(ss, line)) {
            if (!line.empty()) {
                cv::Size textSize = cv::getTextSize(line, fontFace, fontScale, thickness, nullptr);
                strBaseW = max(strBaseW, textSize.width);
                singleRow = max(singleRow, textSize.height);
            }
            lineCount++;
        }

        if (lineCount > 0) {
            int lineSpacing = static_cast<int>(singleRow * 0.3);
            strBaseH = singleRow * lineCount + lineSpacing * (lineCount - 1);
        }
    }
}