/********************************************************************************************************************************
* 功能说明：页面类
* 该类为调试器的单张页面，调试器由多个页面组合而成
* Ver        修改人      变更日期             变更内容
* ───────────────────────────────────────────────────────────────
* V1.0        xlh     09/09/2024, 10:02          初版
* V1.1        xlh     11/05/2024, 10:02          增加图表绘制
*
* Copyright (c) 2015 HOSON. Co., Ltd. All rights reserved.
*┌──────────────────────────────────────────────────────────────┐
*│                                                                                                                            │
*│                                                                                                                            │
*└──────────────────────────────────────────────────────────────┘
-------------------------------------------------------------------------------------------------------------------------------*/

#pragma once

#include <list>
#include <map>
#include <vector>
#include <algorithm>
#include "assert.h"
#include <memory>
#include "GCImage.h"
#include "TextDisp.h"
#include "DbgInfoDefine.h"
#include "opencvLib.h"

//页面类
namespace DebugView
{
	class CHsDebugViewer;
	class CPage
	{
	public:
		friend DebugView::CHsDebugViewer;

		CPage();
		CPage(DebugView::PageTypeEnum type, std::string name = "");
		virtual~CPage();

		///初始化、释放
		int Init();
		int Release();

	public:
		//对外处理接口
		int ProcessPage(bool adap=true);//生成页面结果图像

		//设置页面、图像
		int SetPageSize(int width, int height);//设置页面尺寸
		int SetPageBgImg(GCImage *img, int idx);//设置背景图像
		int SetPageBgImgs(std::vector<GCImage *>imgs);
		int SetPageBgColor(int R, int G, int B);//设置背景颜色
		int ClearPageBgImgs();//清除背景图像

		//添加、清除渲染图形
		int AddDispObj(HSV::DrawObjBase *obj);//添加显示图形
		int ClearDispObject();//清除显示图形

		//添加、插入、设置、清除文本信息
		int AddInfo(HSV::TextDraw *text, AlignTypeEnum align= AlignTypeEnum::ALIGN_LEFT);//添加文本信息
		int AddInfo(HSV::TextUnionDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		int AddInfo(HSV::TextTupleDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		int InsertInfo(int index, HSV::TextDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);//插入文本信息
		int InsertInfo(int index, HSV::TextUnionDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		int InsertInfo(int index, HSV::TextTupleDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		int SetInfo(int index, HSV::TextDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);//设置文本信息
		int SetInfo(int index, HSV::TextUnionDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		int SetInfo(int index, HSV::TextTupleDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		int ClearDispInfo();//清除显示信息
		//统一设置同一页面所有文本绘制参数
		int SetPageAllTextDraw( TextDrawPara para);
		int SetPageAllTextUnionDraw(TextDrawPara paralab, TextDrawPara parasbj);
		
		//插入表格数据(只支持int float double)
		int AddTableInfo(int2D datas,str1D horTitle = {}, str1D verTitle = {});
		int AddTableInfo(float2D datas,str1D horTitle = {}, str1D verTitle = {});
		int AddTableInfo(double2D datas,str1D horTitle = {}, str1D verTitle = {});
		int SetTableInfo(int idx, int2D datas,str1D horTitle = {}, str1D verTitle = {});
		int SetTableInfo(int idx, float2D datas,str1D horTitle = {}, str1D verTitle = {});
		int SetTableInfo(int idx, double2D datas,str1D horTitle = {}, str1D verTitle = {});
		//统一设置同一页面表格绘制参数
		int SetPageTableDraw(int idx, TextDrawPara paraHorTitle, TextDrawPara paraVerTitle, TextDrawPara paraData, ObjDrawPara paraLine);
		int SetPageAllTableDraw(TextDrawPara paraHorTitle, TextDrawPara paraVerTitle, TextDrawPara paraData, ObjDrawPara paraLine);
		
		//插入图表数据 1D用于饼状图或柱状图，2D用于折线图，饼图自动忽略轴标签
		int AddChartInfo(ChartType chartType, int1D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel="", std::string verAxisLabel="",std::string title="",std::string note="");
		int AddChartInfo(ChartType chartType, int2D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int AddChartInfo(ChartType chartType, float1D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int AddChartInfo(ChartType chartType, float2D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int AddChartInfo(ChartType chartType, double1D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int AddChartInfo(ChartType chartType, double2D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int SetChartInfo(int idx, ChartType chartType, int1D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int SetChartInfo(int idx, ChartType chartType, int2D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int SetChartInfo(int idx, ChartType chartType, float1D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int SetChartInfo(int idx, ChartType chartType, float2D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int SetChartInfo(int idx, ChartType chartType, double1D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int SetChartInfo(int idx, ChartType chartType, double2D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		//统一设置同一页面图表绘制参数
		int SetPageChartDraw(int idx, ChartDrawPara chartDraw);
		int SetPageAllChartDraw(ChartDrawPara chartDraw);

	private:
		int CalcImgWidth();//计算图像宽度
		int CalcTextPos();//计算文本位置(InfoPage使用)
		int GenBgImage();//生成背景图像
		int DrawObjects();//绘制图形(ObjPage使用)
		int DrawInfos();//绘制文本信息

	private:
		std::string strPageName;//页面名称
		DebugView::PageTypeEnum ePageType;//页面类型
		DebugView::TileImageType eTileType;//拼接类型
		bool bAdjustBgImgSize;//使用背景图像尺寸
		bool bTextUnionAlign;//TextUnionDraw的sbj对齐
		bool bTileImages;//合并所有图像(图形页面生效)

		int nPageWidth;//页面图像尺寸
		int nPageHeight;
		HSV::ScalarGC colorBackground;//背景颜色
		std::vector<GCImage *>vecImgIn;//输入图像(图像预处理类可能有多张图)
		GCImage *imgRst;//结果图像

		std::vector<HSV::DrawObjBase *>vecObject;//待显示图形
		std::vector<CTextDisp *>vecInfos;//待显示文本、表格信息

		//文本缩进
		int nOffsetXL ;//左对齐文本偏移
		int nOffsetYL ;
		int nOffsetXR ;//右对齐文本偏移
		int nOffSetYR ;
		
	};
}
