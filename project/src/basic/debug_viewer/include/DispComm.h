/********************************************************************************************************************************
* 功能说明：显示公共类
* 内部绘图参数、转换接口、公共接口
* Ver        修改人      变更日期             变更内容
* ───────────────────────────────────────────────────────────────
* V1.0        xlh     11/05/2024, 10:02          初版
*
* Copyright (c) 2015 HOSON. Co., Ltd. All rights reserved.
*┌──────────────────────────────────────────────────────────────┐
*│                                                                                                                            │
*│                                                                                                                            │
*└──────────────────────────────────────────────────────────────┘
-------------------------------------------------------------------------------------------------------------------------------*/

#pragma once
#include "DbgInfoDefine.h"
#include "HSDrawObj.h"
#include <opencv2/opencv.hpp>
using namespace cv;
namespace DebugView
{
	class CDispPara
	{
	public:
		CDispPara();
		virtual~CDispPara();

		void Init();
		void Release();

	public:
		//字体相关
		HSV::ScalarGC clrTxtFore;//字体前景颜色
		HSV::ScalarGC clrTxtBack;//字体背景颜色
		std::string strFont;//字体
		int nFontSize;//字体尺寸
		
		//图形相关
		HSV::ScalarGC clrObj;//颜色
		int thickObj;//厚度
		
	};

	//颜色转换
	HSV::ScalarGC GetScalarColor(int color[3]);
	////获取字符串尺寸
	//void GetStringSize(HDC hDC, const char* str, int* w, int* h);
	////获取文本尺寸
	//void GetTextSize(std::string str, std::string font, int fontSize, int &wid, int&hgt);
	//
	void GetStringSize(const char* str, int* w, int* h, int fontFace = cv::FONT_HERSHEY_SIMPLEX,
		double fontScale = 0.5, int thickness = 1);

	void GetTextSize(std::string str, std::string font, int fontSize, int& wid, int& hgt,
		int thickness = 1);
	int FontNameToOpenCVType(const std::string& fontName);
}

