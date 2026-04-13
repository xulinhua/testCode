/********************************************************************************************************************************
* 功能说明：调试器导出类
* 
* Ver        修改人      变更日期             变更内容
* ───────────────────────────────────────────────────────────────
* V1.0        xlh     09/09/2024, 10:02         初版
* V1.1        xlh     11/06/2024, 14:02     增加柱状图绘制
* V1.2        xlh     11/07/2024, 14:02     增加折线图绘制，完善轴标签渲染
* Copyright (c) 2015 HOSON. Co., Ltd. All rights reserved.
*┌──────────────────────────────────────────────────────────────┐
*│                                                                                                                            │
*│                                                                                                                            │
*└──────────────────────────────────────────────────────────────┘
-------------------------------------------------------------------------------------------------------------------------------*/

#pragma once
#include<vector>
#include<map>
#include"DbgInfoDefine.h"
#include <thread>
#include <mutex>
#include <unordered_map>
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

namespace HSV
{
	class DrawObjBase;
	class TextDraw;
	class TextUnionDraw;
	class TextTupleDraw;
}

class GCImage;
namespace DebugView
{

	class CPage;
	class HSV_DEBUG_VIEWER_API CHsDebugViewer
	{
	public:
		CHsDebugViewer();
		virtual~CHsDebugViewer();

		int Init();//初始化
		int Release();//释放
		static void DestroyPtr(CHsDebugViewer** ptr);//销毁指针
		///通用操作
		//页面管理
		int AddPage(DebugView::PageTypeEnum type= DebugView::PageTypeEnum::PAGE_INFO, std::string name = "",bool bTile=true);
		int InsertPage(int idx, DebugView::PageTypeEnum type = DebugView::PageTypeEnum::PAGE_INFO, std::string name = "");
		int RemovePage(int idx);
		int RemovePage(std::string name);
		int ClearAllPages();

		//合并运行
		int ProcAllPages();//合并所有图像

		///结果相关
		//获取运行结果
		int GetResultImage(const GCImage *&rstUnionImage);//获取合并后图像
		int GetAllImages(const std::vector<const GCImage *>*&allImgs);//获取所有图像
		int GetAllPageImagesIn(const std::map<int,std::vector<const GCImage*>> *&allImgs);
		int SaveRstImages(std::string path);//保存结果图像

		///设置图形、文本、数据 （page=-1时，默认为最后一个页面）
		//设置图像、背景色、尺寸等
		int SetBgImage(int page, GCImage *img, int idx = 0);//设置背景图像
		int SetBgImage(int page, std::vector<GCImage *>imgs);
		int ClearPageBgImage(int page);//清除背景图像
		int ClearAllBgImage();//清除所有页背景图像
		int SetBgColor(int page, int R, int G, int B);//设置背景颜色
		int SetPageSize(int page, int width = 500, int height = 300, bool bAdjust = true);//设置图像尺寸

		//[1]添加、清除渲染图形
		int AddDispObj(int page, HSV::DrawObjBase *obj);//添加显示图形
		int ClearDispObj(int page);//清除显示图形
		int ClearAllDispObj();//清除所有显示图形

		//[2]添加、插入、设置、清除文本信息
		int AddInfo(int page, HSV::TextDraw *text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);//添加文本信息
		int AddInfo(int page, HSV::TextUnionDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		int AddInfo(int page, HSV::TextTupleDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		int InsertInfo(int page, int index, HSV::TextDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);//插入文本信息
		int InsertInfo(int page, int index, HSV::TextUnionDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		int InsertInfo(int page, int index, HSV::TextTupleDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		int SetInfo(int page, int index, HSV::TextDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);//设置文本信息
		int SetInfo(int page, int index, HSV::TextUnionDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		int SetInfo(int page, int index, HSV::TextTupleDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		//统一设置同一页面所有文本绘制参数
		int SetPageAllTextDraw(int page, TextDrawPara para);
		int SetPageAllTextUnionDraw(int page, TextDrawPara paralab, TextDrawPara paralabobj);

		int ClearInfo(int page);//清除显示信息
		int ClearAllInfo();//清除所有显示信息
		
		//[3]插入表格数据(只支持int float double)
		int AddTableInfo(int page, int2D datas,str1D horTitle = str1D(), str1D verTitle = str1D());
		int AddTableInfo(int page, float2D datas,str1D horTitle = str1D(), str1D verTitle = str1D());
		int AddTableInfo(int page, double2D datas,str1D horTitle = str1D(), str1D verTitle = str1D());
		int SetTableInfo(int page, int idx, int2D datas,str1D horTitle = str1D(), str1D verTitle = str1D());
		int SetTableInfo(int page, int idx, float2D datas,str1D horTitle = str1D(), str1D verTitle = str1D());
		int SetTableInfo(int page, int idx, double2D datas,str1D horTitle = str1D(), str1D verTitle = str1D());
		//设置同一页面表格绘制参数
		int SetPageTableDraw(int page,int idx, TextDrawPara paraHorTitle, TextDrawPara paraVerTitle, TextDrawPara paraData, ObjDrawPara paraLine);
		int SetPageAllTableDraw(int page,TextDrawPara paraHorTitle, TextDrawPara paraVerTitle, TextDrawPara paraData, ObjDrawPara paraLine);
		
		//[4]插入图表数据 1D用于饼状图或柱状图，2D用于折线图，饼图自动忽略轴标签
		int AddChartInfo(int page, ChartType chartType, int1D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int AddChartInfo(int page, ChartType chartType, int2D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int AddChartInfo(int page, ChartType chartType, float1D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int AddChartInfo(int page, ChartType chartType, float2D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int AddChartInfo(int page, ChartType chartType, double1D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int AddChartInfo(int page, ChartType chartType, double2D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int SetChartInfo(int page, int idx, ChartType chartType, int1D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int SetChartInfo(int page, int idx, ChartType chartType, int2D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int SetChartInfo(int page, int idx, ChartType chartType, float1D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int SetChartInfo(int page, int idx, ChartType chartType, float2D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int SetChartInfo(int page, int idx, ChartType chartType, double1D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		int SetChartInfo(int page, int idx, ChartType chartType, double2D datas, str1D horStep = {}, str1D verStep = {},
			std::string horAxisLabel = "", std::string verAxisLabel = "", std::string title = "", std::string note = "");
		//统一设置同一页面图表绘制参数
		int SetPageChartDraw(int page, int idx, ChartDrawPara chartDraw);
		int SetPageAllChartDraw(int page, ChartDrawPara chartDraw);
		
	public:
		//常用信息获取
		static std::string GetCurTime();//获取当前时间
		static std::string GetAppInfo();//获取软件信息
		static std::string GetSysInfo();//获取系统信息
		
		std::string GetCurTimeData();//获取时间数据(无符号)
		bool CheckDir(std::string path);//检查目录是否存在
		bool CreateDir(std::string path);//创建目录

	private:
		GCImage *m_imgRst;//合并结果页面
		std::vector<CPage*> m_vecPages;//所有页面数据

		std::vector<const GCImage *>*m_vecAllImgs;//内部图像指针不用释放
		std::map<int, std::vector<const GCImage*>> *m_mapPageImgs;//内部图像指针不用释放
	};


	class HSV_DEBUG_VIEWER_API CHsDebugViewerManger 
	{
	public:
		static CHsDebugViewerManger& getInstance()
		{
			static CHsDebugViewerManger instance;
			return instance;
		}

		CHsDebugViewer* getDebug(uint32_t id)
		{
			std::lock_guard<std::mutex> lock(CHsDebugViewerManger::m_mutex);
			if (id < 0)
				return nullptr;

			auto it = m_mapDebugViewer.find(id);
			if (it != m_mapDebugViewer.end())
			{
				m_mapDebugViewer[id] = new CHsDebugViewer();
				return m_mapDebugViewer[id];
			}
			else
			{
				return m_mapDebugViewer[id];
			}
		}
		

	private:
		static std::mutex m_mutex;
		CHsDebugViewerManger() = default;
		~CHsDebugViewerManger()
		{
			for (auto it : m_mapDebugViewer)
				delete it.second;
		}

	private:
		std::unordered_map<uint32_t, CHsDebugViewer*> m_mapDebugViewer;
	};
}
