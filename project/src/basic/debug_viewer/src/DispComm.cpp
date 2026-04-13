//#include"stdafx.h"
#include"DispComm.h"
#include "ChineseTextSupport.h"
using namespace std;

DebugView::CDispPara::CDispPara()
{
	Init();
}

DebugView::CDispPara::~CDispPara()
{
	Release();
}

void DebugView::CDispPara::Init()
{
	clrTxtFore = HSV::ScalarGC(255, 0, 0);
	clrTxtBack = HSV::ScalarGC(200, 200, 200);
	strFont = "Arial";
	nFontSize = 15;//太小字看不清楚

	//图形相关
	clrObj = HSV::ScalarGC(0, 0, 0);
	thickObj = 1;
}

void DebugView::CDispPara::Release()
{

}

HSV::ScalarGC DebugView::GetScalarColor(int color[3])
{
	return HSV::ScalarGC(color[2], color[1], color[0]);
}

#if 0
void DebugView::GetStringSize(HDC hDC, const char * str, int * w, int * h)
{
	SIZE size;
	GetTextExtentPoint32A(hDC, str, strlen(str), &size);
	if (w != nullptr) *w = size.cx;
	if (h != nullptr) *h = size.cy;
}

void DebugView::GetTextSize(std::string str, std::string font, int fontSize, int & wid, int & hgt)
{
	if (str == "")
	{
		wid = 0;
		hgt = 0;
		return;
	}
	LOGFONTA lf;
	lf.lfHeight = -fontSize;
	lf.lfWidth = 0;
	lf.lfEscapement = 0;
	lf.lfOrientation = 0;
	lf.lfWeight = 5;
	lf.lfItalic = false;//斜体
	lf.lfUnderline = false; //下划线
	lf.lfStrikeOut = 0;
	lf.lfCharSet = DEFAULT_CHARSET;
	lf.lfOutPrecision = 0;
	lf.lfClipPrecision = 0;
	lf.lfQuality = PROOF_QUALITY;
	lf.lfPitchAndFamily = 0;
	strcpy_s(lf.lfFaceName, font.data());

	HFONT hf = CreateFontIndirectA(&lf);
	HDC hDC = CreateCompatibleDC(0);
	HFONT hOldFont = (HFONT)SelectObject(hDC, hf);
	int strBaseW = 0, strBaseH = 0;
	char buf[1 << 12];
	strcpy_s(buf, str.data());
	char *bufT[1 << 12];
	int nnh = 0;
	int cw, ch;
	const char* ln = strtok_s(buf, "\n", bufT);
	while (ln != 0)
	{
		GetStringSize(hDC, ln, &cw, &ch);
		strBaseW = max(strBaseW, cw);
		strBaseH = max(strBaseH, ch);

		ln = strtok_s(0, "\n", bufT);
		nnh++;
	}
	wid = strBaseW;
	hgt = strBaseH;
}
#endif
int DebugView::FontNameToOpenCVType(const std::string& fontName) {
    static const std::unordered_map<std::string, int> fontMap = {
        // 无衬线字体
        {"Arial", cv::FONT_HERSHEY_SIMPLEX},
        {"Arial Black", cv::FONT_HERSHEY_DUPLEX},
        {"Helvetica", cv::FONT_HERSHEY_SIMPLEX},
        {"Verdana", cv::FONT_HERSHEY_SIMPLEX},
        {"Tahoma", cv::FONT_HERSHEY_SIMPLEX},
        {"Trebuchet MS", cv::FONT_HERSHEY_SIMPLEX},

        // 衬线字体
        {"Times New Roman", cv::FONT_HERSHEY_COMPLEX},
        {"Georgia", cv::FONT_HERSHEY_COMPLEX},
        {"Garamond", cv::FONT_HERSHEY_COMPLEX},

        // 等宽字体
        {"Courier New", cv::FONT_HERSHEY_COMPLEX_SMALL},
        {"Courier", cv::FONT_HERSHEY_COMPLEX_SMALL},
        {"Lucida Console", cv::FONT_HERSHEY_COMPLEX_SMALL},

        // 手写体
        {"Comic Sans MS", cv::FONT_HERSHEY_SCRIPT_SIMPLEX},
        {"Brush Script MT", cv::FONT_HERSHEY_SCRIPT_COMPLEX},

        // 中文字体
        {"SimSun", cv::FONT_HERSHEY_COMPLEX},           // 宋体
        {"SimHei", cv::FONT_HERSHEY_COMPLEX},           // 黑体  
        {"Microsoft YaHei", cv::FONT_HERSHEY_COMPLEX},  // 微软雅黑
        {"KaiTi", cv::FONT_HERSHEY_COMPLEX},            // 楷体
        {"FangSong", cv::FONT_HERSHEY_COMPLEX},         // 仿宋
    };

    // 不区分大小写查找
    std::string lowerFont = fontName;
    std::transform(lowerFont.begin(), lowerFont.end(), lowerFont.begin(), ::tolower);

    for (const auto& pair : fontMap) {
        std::string key = pair.first;
        std::transform(key.begin(), key.end(), key.begin(), ::tolower);
        if (key.find(lowerFont) != std::string::npos ||
            lowerFont.find(key) != std::string::npos) {
            return pair.second;
        }
    }

    return cv::FONT_HERSHEY_SIMPLEX; // 默认字体
}

void DebugView::GetStringSize(const char* str, int* w, int* h, int fontFace,
    double fontScale, int thickness)
{
    if (str == nullptr || strlen(str) == 0) {
        if (w != nullptr) *w = 0;
        if (h != nullptr) *h = 0;
        return;
    }

    cv::Size textSize = cv::getTextSize(str, fontFace, fontScale, thickness, nullptr);
    if (w != nullptr) *w = textSize.width;
    if (h != nullptr) *h = textSize.height;
}

void DebugView::GetTextSize(std::string str, std::string font, int fontSize, int& wid, int& hgt,
    int thickness)
{
    if (str.empty()) {
        wid = 0;
        hgt = 0;
        return;
    }

    // 检查是否包含中文字符
    if (ChineseTextUtils::ContainsChinese(str)) {
		//std::cout << "zn_str:" << str << std::endl;
        
        // 使用FreeType进行中文文本尺寸计算
        ChineseTextUtils::InitChineseRenderer();
        if (g_chineseRenderer && g_chineseRenderer->isFontLoaded()) {
            cv::Size textSize = g_chineseRenderer->getTextSize(str, fontSize, thickness);
            wid = textSize.width;
            hgt = textSize.height;
        } else {
            // 如果FreeType渲染器未初始化，使用默认方法
            double fontScale = fontSize / 30.0;
            cv::Size textSize = cv::getTextSize(str, FontNameToOpenCVType(font), fontScale, thickness, nullptr);
            wid = textSize.width;
            hgt = textSize.height;
        }
    } 
    else
		//std::cout << "en_str:" << str << std::endl;
    {
        // 对于非中文文本，使用OpenCV方法
        double fontScale =  fontSize / 30.0;
        int strBaseW = 0, strBaseH = 0;

        // 处理多行文本
        std::stringstream ss(str);
        std::string line;
        int lineCount = 0;

        while (std::getline(ss, line)) {
            if (!line.empty()) {
                cv::Size textSize = cv::getTextSize(line, FontNameToOpenCVType(font), fontScale, thickness, nullptr);
                strBaseW = std::max(strBaseW, textSize.width);
                strBaseH = std::max(strBaseH, textSize.height);
            }
            lineCount++;
        }

        // 如果有多行，需要调整总高度
        if (lineCount > 1) {
            int lineHeight = strBaseH;
            // 估算多行文本的总高度（行高 + 行间距）
            strBaseH = lineHeight * lineCount + static_cast<int>(lineHeight * 0.3 * (lineCount - 1));
        }

        wid = strBaseW;
        hgt = strBaseH;
    }
}