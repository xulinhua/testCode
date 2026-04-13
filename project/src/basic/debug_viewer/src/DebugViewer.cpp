// DebugViewer.cpp : 定义 DLL 应用程序的导出函数。
//

//#include "stdafx.h"
#include <ctime>
#include "DebugViewer.h"
#include "Page.h"
#include "GCImage.h"
#include "HSDrawObj.h"
#include "DrawFunc.h"
#include "opencvLib.h"
#ifdef _WIN32
#pragma warning(disable: 4996)
#pragma warning(disable: 4251)
#include <lm.h>//需要在opencvLib.h之后
#pragma comment(lib, "netapi32.lib")
#include <experimental/filesystem>

#ifdef _X86_
#ifdef _DEBUG
#pragma comment(lib,"Hsvcustrctd.lib")
#else
#pragma comment(lib,"Hsvcustrct.lib")
#endif
#else
#ifdef _DEBUG
#pragma comment(lib,"Hsvcustrct64d.lib")
#else
#pragma comment(lib,"Hsvcustrct64.lib")
#endif
#endif
#else
// Linux平台的替代实现
#include <sys/utsname.h>
#include <cstring>
#include <cstdio>
#include <experimental/filesystem>
#endif

using namespace std;
using namespace HSV;
using namespace DebugView;
namespace fs = std::experimental::filesystem;

std::mutex CHsDebugViewerManger::m_mutex;

DebugView::CHsDebugViewer::CHsDebugViewer()
{
	m_imgRst = nullptr;
	m_vecAllImgs = nullptr;
	m_mapPageImgs = nullptr;
	Init();//初始化、释放所有资源

	m_imgRst = new GCImage();//构造时创建结果图像
	m_vecAllImgs = new std::vector<const GCImage *>();
	m_mapPageImgs=new std::map<int, std::vector<const GCImage*>>;
}

DebugView::CHsDebugViewer::~CHsDebugViewer()
{
	Release();
}

int DebugView::CHsDebugViewer::Init()
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;

	Release();
	return nRet;
}

int DebugView::CHsDebugViewer::Release()
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//页面释放
	ClearAllPages();
	if (m_imgRst != nullptr)
	{
		delete m_imgRst;
		m_imgRst = nullptr;
	}
	if (m_vecAllImgs!=nullptr)
	{
		delete m_vecAllImgs;
		m_vecAllImgs = nullptr;
	}
	return nRet;
}

//销毁指针
void DebugView::CHsDebugViewer::DestroyPtr(CHsDebugViewer** ptr)
{
	if (nullptr != *ptr)
	{
		(*ptr)->Release();
		delete *ptr;
		*ptr = nullptr;
	}
}

int DebugView::CHsDebugViewer::AddPage(DebugView::PageTypeEnum type, std::string name, bool bTile)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;

	CPage *page = new CPage(type);
	page->strPageName = name;
	page->bTileImages = bTile;
	m_vecPages.push_back(page);

	return nRet;
}

int DebugView::CHsDebugViewer::InsertPage(int idx, DebugView::PageTypeEnum type, std::string name)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;

	if (idx >= (int)m_vecPages.size())
	{
		return (int)DebugView::Error_ID::ERR_PROC_NG;//序号越界
	}

	CPage *page = new CPage(type);
	page->strPageName = name;
	m_vecPages.insert(m_vecPages.begin() + idx, page);
	//m_bPagesChanged = true;

	return nRet;
}

int DebugView::CHsDebugViewer::RemovePage(int idx)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;

	if (idx >= (int)m_vecPages.size())
	{
		return (int)DebugView::Error_ID::ERR_PROC_NG;//序号越界
	}
	if (m_vecPages[idx] != nullptr)
	{
		delete m_vecPages[idx];
	}
	m_vecPages.erase(m_vecPages.begin() + idx);
	//m_bPagesChanged = true;

	return nRet;
}

int DebugView::CHsDebugViewer::RemovePage(string name)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;

	int idxFind = -1;
	for (size_t i = 0; i < m_vecPages.size(); i++)
	{
		if (m_vecPages[i]->strPageName == name && name != "")
		{
			idxFind = (int)i;
		}
	}
	if (idxFind != -1)
	{
		if (m_vecPages[idxFind] != nullptr)
		{
			delete m_vecPages[idxFind];
		}
		m_vecPages.erase(m_vecPages.begin() + idxFind);
		//m_bPagesChanged = true;
	}
	else
	{
		return (int)DebugView::Error_ID::ERR_PAGE_NULL;//查找的页面为空
	}
	return nRet;
}

int DebugView::CHsDebugViewer::ClearAllPages()
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;

	int num = (int)m_vecPages.size();

	for ( int i = 0; i < num; i++)
	{
		if (m_vecPages[i] != nullptr)
		{
			delete m_vecPages[i];
			m_vecPages[i] = nullptr;
		}
	}
	m_vecPages.clear();

	return nRet;
}

int DebugView::CHsDebugViewer::SetBgImage(int page, GCImage * img, int idx)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size() || page < 0)
	{
		return  (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//图像拷贝
	m_vecPages[page]->SetPageBgImg(img, idx);

	return nRet;
}

int DebugView::CHsDebugViewer::SetBgImage(int page, std::vector<GCImage*> imgs)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size() || page < 0)
	{
		return  (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//图像拷贝
	nRet = m_vecPages[page]->SetPageBgImgs(imgs);

	return nRet;
}

int DebugView::CHsDebugViewer::ClearPageBgImage(int page)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size() || page < 0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//图像清除
	int numImg =(int) m_vecPages[page]->vecImgIn.size();
	for (int i = 0; i < numImg; i++)
	{
		nRet = m_vecPages[page]->ClearPageBgImgs();
	}

	return nRet;
}

int DebugView::CHsDebugViewer::ClearAllBgImage()
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	for (size_t i = 0; i < m_vecPages.size(); i++)
	{
		m_vecPages[i]->ClearPageBgImgs();
	}
	return nRet;
}

int DebugView::CHsDebugViewer::SetBgColor(int page, int R, int G, int B)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size()||page<0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//背景色设置
	m_vecPages[page]->SetPageBgColor(R, G, B);

	return nRet;
}

int DebugView::CHsDebugViewer::SetPageSize(int page, int width, int height, bool bAdjust)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size() || page < 0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//设置尺寸
	m_vecPages[page]->SetPageSize(width, height);

	return nRet;
}

int DebugView::CHsDebugViewer::AddDispObj(int page, HSV::DrawObjBase * obj)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size()||page<0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//添加图形
	m_vecPages[page]->AddDispObj(obj);

	return nRet;
}

int DebugView::CHsDebugViewer::ClearDispObj(int page)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size() || page < 0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//添加图形
	m_vecPages[page]->ClearDispObject();

	return nRet;
}

int DebugView::CHsDebugViewer::ClearAllDispObj()
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	for (size_t i = 0; i < m_vecPages.size(); i++)
	{
		m_vecPages[i]->ClearDispObject();
	}

	return nRet;
}

template<typename T>
int AddInfo_Tmp(std::vector<CPage*> &vecPages, int page, T * text, AlignTypeEnum align)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)vecPages.size() || page<0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//添加图形
	vecPages[page]->AddInfo(text, align);

	return nRet;
}

int DebugView::CHsDebugViewer::AddInfo(int page, HSV::TextDraw * text, AlignTypeEnum align)
{
	return AddInfo_Tmp(m_vecPages, page, text, align);
}

int DebugView::CHsDebugViewer::AddInfo(int page, HSV::TextUnionDraw * text, AlignTypeEnum align)
{
	return AddInfo_Tmp(m_vecPages, page, text, align);
}

int DebugView::CHsDebugViewer::AddInfo(int page, HSV::TextTupleDraw * text, AlignTypeEnum align)
{
	return AddInfo_Tmp(m_vecPages, page, text, align);
}


template<typename T>
int InsertInfo_Tmp(std::vector<CPage*> &vecPages, int page, int index, T * text, AlignTypeEnum align)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)vecPages.size() || page < 0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//添加图形
	vecPages[page]->InsertInfo(index, text, align);

	return nRet;
}

int DebugView::CHsDebugViewer::InsertInfo(int page, int index, HSV::TextDraw * text, AlignTypeEnum align)
{
	return InsertInfo_Tmp(m_vecPages,page,index,text,align);
}

int DebugView::CHsDebugViewer::InsertInfo(int page, int index, HSV::TextUnionDraw * text, AlignTypeEnum align)
{
	return InsertInfo_Tmp(m_vecPages, page, index, text, align);
}

int DebugView::CHsDebugViewer::InsertInfo(int page, int index, HSV::TextTupleDraw * text, AlignTypeEnum align)
{
	return InsertInfo_Tmp(m_vecPages, page, index, text, align);
}

template<typename T>
int SetInfo_Tmp(std::vector<CPage*> &vecPages, int page, int index, T * text, AlignTypeEnum align)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)vecPages.size() || page < 0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//添加图形
	vecPages[page]->SetInfo(index, text, align);

	return nRet;
}

int DebugView::CHsDebugViewer::SetInfo(int page, int index, HSV::TextDraw * text, AlignTypeEnum align)
{
	return SetInfo_Tmp(m_vecPages,page,index,text,align);
}

int DebugView::CHsDebugViewer::SetInfo(int page, int index, HSV::TextUnionDraw * text, AlignTypeEnum align)
{
	return SetInfo_Tmp(m_vecPages, page, index, text, align);
}

int DebugView::CHsDebugViewer::SetInfo(int page, int index, HSV::TextTupleDraw * text, AlignTypeEnum align)
{
	return SetInfo_Tmp(m_vecPages, page, index, text, align);
}

int DebugView::CHsDebugViewer::SetPageAllTextDraw(int page, TextDrawPara para)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size()||page<0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//添加图形
	m_vecPages[page]->SetPageAllTextDraw(para);

	return nRet;
}

int DebugView::CHsDebugViewer::SetPageAllTextUnionDraw(int page, TextDrawPara paralab, TextDrawPara paralabobj)
{

	return 0;
}

int DebugView::CHsDebugViewer::ClearInfo(int page)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size()||page<0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//清除文本信息
	m_vecPages[page]->ClearDispInfo();

	return nRet;
}

int DebugView::CHsDebugViewer::ClearAllInfo()
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	for (size_t i = 0; i < m_vecPages.size(); i++)
	{
		m_vecPages[i]->ClearDispInfo();
	}
	return nRet;
}

template<typename T>
int AddTableInfo_Tmp(std::vector<CPage*> &vecPages, int page, T datas, str1D horTitle, str1D verTitle)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)vecPages.size() || page<0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//添加表格
	vecPages[page]->AddTableInfo(datas, horTitle, verTitle);
	return nRet;
}

int DebugView::CHsDebugViewer::AddTableInfo(int page, int2D datas, str1D horTitle, str1D verTitle)
{
	return AddTableInfo_Tmp(m_vecPages, page, datas, horTitle, verTitle);
}

int DebugView::CHsDebugViewer::AddTableInfo(int page, float2D datas, str1D horTitle, str1D verTitle)
{
	return AddTableInfo_Tmp(m_vecPages, page, datas, horTitle, verTitle);
}

int DebugView::CHsDebugViewer::AddTableInfo(int page, double2D datas, str1D horTitle, str1D verTitle)
{
	return AddTableInfo_Tmp(m_vecPages, page, datas, horTitle, verTitle);
}

template<typename T>
int SetTableInfo_Tmp(std::vector<CPage*> &vecPages, int page, int idx, T datas, str1D horTitle, str1D verTitle)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)vecPages.size() || page<0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//添加表格
	vecPages[page]->SetTableInfo(idx, datas, horTitle, verTitle);
	return nRet;
}

int DebugView::CHsDebugViewer::SetTableInfo(int page, int idx, int2D datas, str1D horTitle, str1D verTitle)
{
	return SetTableInfo_Tmp(m_vecPages,page,idx,datas,horTitle,verTitle);
}

int DebugView::CHsDebugViewer::SetTableInfo(int page, int idx, float2D datas, str1D horTitle, str1D verTitle)
{
	return SetTableInfo_Tmp(m_vecPages, page, idx, datas, horTitle, verTitle);
}

int DebugView::CHsDebugViewer::SetTableInfo(int page, int idx, double2D datas, str1D horTitle, str1D verTitle)
{
	return SetTableInfo_Tmp(m_vecPages, page, idx, datas, horTitle, verTitle);
}



int DebugView::CHsDebugViewer::SetPageTableDraw(int page, int idx, TextDrawPara paraHorTitle, TextDrawPara paraVerTitle, TextDrawPara paraData, ObjDrawPara paraLine)
{
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size() || page < 0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//设置参数
	m_vecPages[page]->SetPageTableDraw(idx, paraHorTitle, paraVerTitle, paraData, paraLine);

	return 0;
}

int DebugView::CHsDebugViewer::SetPageAllTableDraw(int page, TextDrawPara paraHorTitle, TextDrawPara paraVerTitle, TextDrawPara paraData, ObjDrawPara paraLine)
{
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size() || page < 0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//设置参数
	m_vecPages[page]->SetPageAllTableDraw(paraHorTitle, paraVerTitle, paraData, paraLine);

	return 0;
}

template<typename T>
int AddChartInfo_Tmp(std::vector<CPage*> &vecPages, int page, ChartType chartType, T datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)vecPages.size() || page < 0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//添加表格
	nRet=vecPages[page]->AddChartInfo(chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
	return nRet;

}

int DebugView::CHsDebugViewer::AddChartInfo(int page, ChartType chartType, int1D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	return AddChartInfo_Tmp(m_vecPages, page, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
}

int DebugView::CHsDebugViewer::AddChartInfo(int page, ChartType chartType, int2D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	return AddChartInfo_Tmp(m_vecPages, page, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
}

int DebugView::CHsDebugViewer::AddChartInfo(int page, ChartType chartType, float1D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	return AddChartInfo_Tmp(m_vecPages, page, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
}

int DebugView::CHsDebugViewer::AddChartInfo(int page, ChartType chartType, float2D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	return AddChartInfo_Tmp(m_vecPages, page, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
}

int DebugView::CHsDebugViewer::AddChartInfo(int page, ChartType chartType, double1D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	return AddChartInfo_Tmp(m_vecPages, page, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
}

int DebugView::CHsDebugViewer::AddChartInfo(int page, ChartType chartType, double2D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	return AddChartInfo_Tmp(m_vecPages, page, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
}

template<typename T>
int SetChartInfo_Tmp(std::vector<CPage*> &vecPages, int page, int idx, ChartType chartType, T datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)vecPages.size() || page < 0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//添加表格
	nRet = vecPages[page]->SetChartInfo(page, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
	return nRet;
}


int DebugView::CHsDebugViewer::SetChartInfo(int page, int idx, ChartType chartType, int1D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	return SetChartInfo_Tmp(m_vecPages, page, idx, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
}

int DebugView::CHsDebugViewer::SetChartInfo(int page, int idx, ChartType chartType, int2D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	return SetChartInfo_Tmp(m_vecPages, page, idx, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
}

int DebugView::CHsDebugViewer::SetChartInfo(int page, int idx, ChartType chartType, float1D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	return SetChartInfo_Tmp(m_vecPages, page, idx, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
}

int DebugView::CHsDebugViewer::SetChartInfo(int page, int idx, ChartType chartType, float2D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	return SetChartInfo_Tmp(m_vecPages, page, idx, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
}

int DebugView::CHsDebugViewer::SetChartInfo(int page, int idx, ChartType chartType, double1D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	return SetChartInfo_Tmp(m_vecPages, page, idx, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
}

int DebugView::CHsDebugViewer::SetChartInfo(int page, int idx, ChartType chartType, double2D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
{
	return SetChartInfo_Tmp(m_vecPages, page, idx, chartType, datas, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
}

int DebugView::CHsDebugViewer::SetPageChartDraw(int page, int idx, ChartDrawPara chartDraw)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size()||page<0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//添加表格
	m_vecPages[page]->SetPageChartDraw(idx,chartDraw);
	return nRet;
}

int DebugView::CHsDebugViewer::SetPageAllChartDraw(int page, ChartDrawPara chartDraw)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	//负数序号默认为最后一个页面
	if (page < 0)
	{
		page = (int)m_vecPages.size() - 1;
	}
	//页面异常
	if (page >= (int)m_vecPages.size()||page<0)
	{
		return (int)DebugView::Error_ID::ERR_PAGE_INDEX_NG;
	}
	//添加表格
	m_vecPages[page]->SetPageAllChartDraw(chartDraw);
	return nRet;
}

std::string DebugView::CHsDebugViewer::GetCurTime()
{
	string strTime = "";

	// 使用 C++11 chrono 库获取当前时间
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);

	// 获取毫秒部分
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		now.time_since_epoch()) % 1000;

	// 转换为本地时间
	std::tm tm_time;

#ifdef _WIN32
	localtime_s(&tm_time, &in_time_t);
#else
	localtime_r(&in_time_t, &tm_time);
#endif

	char tmpbuff[16];
	sprintf(tmpbuff, "%04d", tm_time.tm_year + 1900);
	std::string year = tmpbuff;
	sprintf(tmpbuff, "%02d", tm_time.tm_mon + 1);
	std::string month = tmpbuff;
	sprintf(tmpbuff, "%02d", tm_time.tm_mday);
	std::string day = tmpbuff;
	sprintf(tmpbuff, "%02d", tm_time.tm_hour);
	std::string hour = tmpbuff;
	sprintf(tmpbuff, "%02d", tm_time.tm_min);
	std::string minute = tmpbuff;
	sprintf(tmpbuff, "%02d", tm_time.tm_sec);
	std::string second = tmpbuff;
	sprintf(tmpbuff, "%03d", static_cast<int>(ms.count()));
	std::string millisecond = tmpbuff;

	strTime = year + "-" + month + "-" + day + " "
		+ hour + ":" + minute + ":" + second + ":" + millisecond;

	return strTime;
}

std::string DebugView::CHsDebugViewer::GetAppInfo()
{
	string strInfo="";

#ifdef _X86_
	strInfo = "Win32";
#else
	strInfo = "X64";
#endif 

#ifdef _DEBUG
	strInfo += "_Debug";
#else
	strInfo += "_Release";
#endif

	return strInfo;
}

std::string DebugView::CHsDebugViewer::GetSysInfo()
{
	string strInfo = "";

	// 系统类型
#ifdef _WIN32
	strInfo = "Windows ";
	// Windows CPU 信息
	char pbrand[100] = { 0 };
	int cpuInfo[4] = { 0 };
	__cpuid(cpuInfo, 0x80000000);
	if (cpuInfo[0] >= 0x80000004) {
		__cpuid((int*)&pbrand[0], 0x80000002);
		__cpuid((int*)&pbrand[16], 0x80000003);
		__cpuid((int*)&pbrand[32], 0x80000004);
		pbrand[48] = '\0';
	}
	strInfo += pbrand;
#else
	// Linux 系统
	struct utsname unameData;
	if (uname(&unameData) == 0) {
		strInfo = string(unameData.sysname) + " " + string(unameData.release) + " ";
	}
	else {
		strInfo = "Linux ";
	}

	// Linux CPU 信息
	char pbrand[100] = { 0 };
	FILE* cpuinfo = fopen("/proc/cpuinfo", "r");
	if (cpuinfo) {
		char line[256];
		while (fgets(line, sizeof(line), cpuinfo)) {
			if (strstr(line, "model name")) {
				char* colon = strchr(line, ':');
				if (colon) {
					colon += 2;
					strncpy(pbrand, colon, sizeof(pbrand) - 1);
					pbrand[sizeof(pbrand) - 1] = '\0';
					// 去除换行符
					size_t len = strlen(pbrand);
					if (len > 0 && pbrand[len - 1] == '\n')
						pbrand[len - 1] = '\0';
					break;
				}
			}
		}
		fclose(cpuinfo);
	}
	if (strlen(pbrand) == 0) {
		strcpy(pbrand, "Unknown CPU");
	}
	strInfo += pbrand;
#endif

	return strInfo;
}

std::string DebugView::CHsDebugViewer::GetCurTimeData()
{
	string strTime = "";

	// 使用 C++11 chrono 库获取当前时间
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);

	// 获取毫秒部分
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		now.time_since_epoch()) % 1000;

	// 转换为本地时间
	std::tm tm_time;

#ifdef _WIN32
	localtime_s(&tm_time, &in_time_t);
#else
	localtime_r(&in_time_t, &tm_time);
#endif

	char tmpbuff[16];
	sprintf(tmpbuff, "%04d", tm_time.tm_year + 1900);
	std::string year = tmpbuff;
	sprintf(tmpbuff, "%02d", tm_time.tm_mon + 1);
	std::string month = tmpbuff;
	sprintf(tmpbuff, "%02d", tm_time.tm_mday);
	std::string day = tmpbuff;
	sprintf(tmpbuff, "%02d", tm_time.tm_hour);
	std::string hour = tmpbuff;
	sprintf(tmpbuff, "%02d", tm_time.tm_min);
	std::string minute = tmpbuff;
	sprintf(tmpbuff, "%02d", tm_time.tm_sec);
	std::string second = tmpbuff;
	sprintf(tmpbuff, "%03d", static_cast<int>(ms.count()));
	std::string millisecond = tmpbuff;

	strTime = year + month + day + hour + minute + second + millisecond;

	return strTime;
}
bool DebugView::CHsDebugViewer::CheckDir(std::string path)
{
	return fs::exists(path);
}

bool DebugView::CHsDebugViewer::CreateDir(std::string path)
{
	try
	{
		fs::path dir_path(path);
		fs::create_directory(dir_path);

		return CheckDir(path);
	}
	catch (...)
	{

	}
	return false;
}

int DebugView::CHsDebugViewer::GetResultImage(const GCImage *& rstUnionImage)
{
	rstUnionImage = m_imgRst;
	return (int)DebugView::Error_ID::ERR_OK;
}

int DebugView::CHsDebugViewer::GetAllImages(const std::vector<const GCImage*>*& allImgs)
{
	allImgs = m_vecAllImgs;
	return (int)DebugView::Error_ID::ERR_OK;
}

int DebugView::CHsDebugViewer::GetAllPageImagesIn(const std::map<int, std::vector<const GCImage*>>*& allImgs)
{
	allImgs = m_mapPageImgs;
	return (int)DebugView::Error_ID::ERR_OK;
}

int DebugView::CHsDebugViewer::SaveRstImages(std::string path)
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;
	string strSaveDir = GetCurTimeData();
	
	if (path=="")
	{//保存至执行目录下
		CreateDir(strSaveDir);
	}
	else
	{//判断路径是否有效，失败时创建一次目录，若仍然失败则退出
		if (!CheckDir(path))
		{
			if (!CreateDir(path))
			{
				nRet = (int)DebugView::Error_ID::ERR_PROC_NG;
				return nRet;
			}
		}
		strSaveDir = path + "//" + GetCurTimeData();
		CreateDir(strSaveDir);
	}
	//保存所有图像
	if (m_imgRst!=nullptr)
	{
		m_imgRst->SaveImage(strSaveDir + "/UnionResultImage.bmp");
	}
	for (size_t i = 0; i < m_vecPages.size(); i++)
	{
		for (size_t j = 0; j < m_vecPages[i]->vecImgIn.size(); j++)
		{
			GCImage *img = m_vecPages[i]->vecImgIn[j];
			if (img != nullptr)
			{
				img->SaveImage(strSaveDir + "//Page" + to_string(i) + "Img" + to_string(j) + "In.bmp");
			}
		}
	}

	return 0;
}

int DebugView::CHsDebugViewer::ProcAllPages()
{
	int nRet = (int)DebugView::Error_ID::ERR_OK;

	try
	{
		if (m_imgRst == nullptr)
		{
			m_imgRst = new GCImage();
		}
		bool bUseImgInSize = false;
		int imgWid_0 = 0;
		int imgHgt_0 = 0;
		int pageWid_0 = 0;
		int pageHgt_0 = 0;
		if (m_vecPages.size() > 0)
		{//有输入图像,使用第一张图的尺寸为标准
			if (m_vecPages[0]->vecImgIn.size() > 0)
			{
				if (m_vecPages[0]->vecImgIn[0] != nullptr)
				{
					bUseImgInSize = true;
					imgWid_0 = m_vecPages[0]->vecImgIn[0]->GetWidth();
					imgHgt_0 = m_vecPages[0]->vecImgIn[0]->GetHeight();
				}
			}
			else
			{//无输入图像，使用页面1的尺寸为标准
				 pageWid_0 = m_vecPages[0]->nPageWidth;
				 pageHgt_0 = m_vecPages[0]->nPageHeight;
			}
		}
		//页宽度设置
		int pageWidMax = 0;
		for (size_t i = 0; i < m_vecPages.size(); i++)
		{
			if (bUseImgInSize)
			{
				m_vecPages[i]->nPageWidth = imgWid_0;
				m_vecPages[i]->nPageHeight = imgHgt_0;
			}
			else if (m_vecPages[i]->nPageWidth == 0 && pageWid_0 > 0)
			{
				m_vecPages[i]->nPageWidth = pageWid_0;
				m_vecPages[i]->nPageHeight = pageHgt_0;
			}
			m_vecPages[i]->CalcImgWidth();
			pageWidMax = (pageWidMax > m_vecPages[i]->nPageWidth)? pageWidMax: m_vecPages[i]->nPageWidth;
		}
		for (size_t i = 0; i < m_vecPages.size(); i++)
		{
			m_vecPages[i]->nPageWidth = pageWidMax;
		}
		//图形信息绘制
		for (size_t i = 0; i < m_vecPages.size(); i++)
		{
			int bRst = m_vecPages[i]->ProcessPage();
			if (bRst == (int)Error_ID::ERR_OK)
			{
				if (m_vecPages[i]->imgRst==nullptr)
				{
					return (int)Error_ID::ERR_PROC_NG;
				}
				if (i == 0)
				{
					*m_imgRst = *m_vecPages[i]->imgRst;
				}
				else if (m_vecPages[i]->imgRst != nullptr)
				{
					CombineImages(m_imgRst->GetMatImg(), m_vecPages[i]->imgRst->GetMatImg(), m_imgRst->GetMatImg());
				}
			}
		}
		//赋值结果图像组
		m_vecAllImgs->clear();
		m_vecAllImgs->push_back(m_imgRst);//合并后图像
		for (size_t i = 0; i < m_vecPages.size(); i++)
		{
			for (size_t j = 0; j < m_vecPages[i]->vecImgIn.size(); j++)
			{
				m_vecAllImgs->push_back(m_vecPages[i]->vecImgIn[j]);
			}
		}
		//赋值各页面输入图像
		m_mapPageImgs->clear();
		for (size_t i = 0; i < m_vecPages.size(); i++)
		{
			(*m_mapPageImgs)[i].clear();
			for (size_t j = 0; j < m_vecPages[i]->vecImgIn.size(); j++)
			{
				m_mapPageImgs->at(i).push_back(m_vecPages[i]->vecImgIn[j]);
			}
		}
	}
	catch (...)
	{
		nRet = (int)DebugView::Error_ID::ERR_PROC_NG;
	}
	return nRet;
}
