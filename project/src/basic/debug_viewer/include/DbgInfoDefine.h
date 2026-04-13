/********************************************************************************************************************************
* 功能说明：数据类型、异常类型等定义
*
* Ver        修改人      变更日期             变更内容
* ───────────────────────────────────────────────────────────────
* V1.0        xlh     09/09/2024, 10:02         初版
* V1.1        xlh     11/07/2024, 10:02     增加图表绘制参数
* Copyright (c) 2015 HOSON. Co., Ltd. All rights reserved.
*┌──────────────────────────────────────────────────────────────┐
*│                                                                                                                            │
*│                                                                                                                            │
*└──────────────────────────────────────────────────────────────┘
-------------------------------------------------------------------------------------------------------------------------------*/

#pragma once
#include<string>
#include<vector>

#ifdef _WIN32
    #ifdef DEBUGVIEWER_EXPORTS
        #define HSV_DEBUG_VIEWER_API __declspec(dllexport)
    #else
        #define HSV_DEBUG_VIEWER_API __declspec(dllimport)
    #endif
#elif __linux__
    #ifdef DEBUGVIEWER_EXPORTS
        #define HSV_DEBUG_VIEWER_API __attribute__((visibility("default")))
    #else
        #define HSV_DEBUG_VIEWER_API
    #endif
#else
    #error "Unsupported platform"
#endif

namespace DebugView
{
	using str1D = std::vector<std::string>;
	using int1D = std::vector<int>;
	using int2D = std::vector<std::vector<int>>;
	using float1D =std::vector<float>;
	using float2D = std::vector<std::vector<float>>;
	using double1D = std::vector<double>;
	using double2D = std::vector<std::vector<double>>;

	//页面类型
	enum class PageTypeEnum
	{
		UNDEFINE = -1,//未定位
		PAGE_OBJECT = 0,//图像图形页面(通常为第一页)
		PAGE_INFO,//信息显示页面

		MAX,
	};

	//图表类型
	enum class ChartType
	{
		UNDEFINE = -1,
		BAR_CHART = 0,//柱状图
		LINE_CHART,//折线图
		PIE_CHART,//饼图（未实现）

		MAX,
	} ;
	//文本类型
	enum class TextDrawType
	{
		UNDEFINE = -1,
		TEXT = 0,//普通文本
		TEXT_UNION,//合并文本
		TEXT_TUPLE,//组合文本
		TABLE,//表格数据
		CHART,//图表数据

		MAX,
	};

	//表格数据类型
	enum class DataType
	{
		UNDEFINE = -1,//未定义

		INT = 0,//int
		FLOAT,
		DOUBLE,

		MAX,
	};

	//文本对齐方式
	enum class AlignTypeEnum
	{
		ALIGN_LEFT,
		ALIGN_RIGHT,

		MAX,
	};

	//背景图像拼接类型
	enum class TileImageType
	{
		NONE,//不拼接(选择该项时，背景图为img[0])

		TILE_HOR,//水平拼接
		TILE_VER,//垂直拼接

		MAX,
	};

	enum class Error_ID
	{
		ERR_OK,
		ERR_PROC_NG,
		ERR_IMAGE_NULL,//输入图像为空
		ERR_OBJECT_NULL,//图形为空
		ERR_TEXT_NULL,//文本为空
		ERR_PAGE_NULL,//页面不存在
		ERR_PAGE_IMG_NULL,//页面图像为空
		ERR_PAGE_INDEX_NG,//页面序号越界
		ERR_PAGE_IMG_INDEX_NG,//图像序号越界
		ERR_PAGE_OBJ_INDEX_NG,//图形序号越界
		ERR_PAGE_INFO_INDEX_NG,//文本序号越界

		MAX,
	};

	//文本绘制参数
	struct HSV_DEBUG_VIEWER_API TextDrawPara
	{
		TextDrawPara();
		TextDrawPara(int size, int r = 200, int g = 10, int b = 10, std::string font = "Arial");

		std::string strFont;
		int nTextSize;
		int nRgb[3];

	};
	//图形绘制参数
	struct HSV_DEBUG_VIEWER_API ObjDrawPara
	{
		ObjDrawPara();
		ObjDrawPara(int thickness, int r = 200, int g = 10, int b = 10);

		int nThickness;
		int nRgb[3];
	};

	//图表绘制参数
	struct HSV_DEBUG_VIEWER_API ChartDrawPara
	{
		ChartDrawPara();

		TextDrawPara dspTitle;//标题
		TextDrawPara dspNote;//底部注释

		//柱状图取0号颜色，折线图颜色数量需要对应数据组，饼图需要对应数据
		//为空时使用默认颜色
		std::vector<ObjDrawPara> dspData;//数据颜色

		TextDrawPara dspHorAxisLab; //轴名标签
		TextDrawPara dspVerAxisLab;
		TextDrawPara dspHorStepLab;//单步标签
		TextDrawPara dspVerStepLab;

		ObjDrawPara dspHorAxisLine;//轴线
		ObjDrawPara dspVerAxisLine;
		ObjDrawPara dspHorStepLine;//单步直线
		ObjDrawPara dspVerStepLine;

	};
}
