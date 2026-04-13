//#include"stdafx.h"
#include"ChartDraw_Bar.h"

using namespace std;
using namespace HSV;

namespace DebugView
{
	CBarChartDraw::CBarChartDraw()
	{
		Init();
	}

	CBarChartDraw::~CBarChartDraw()
	{
		Release();
	}

	void CBarChartDraw::CopyFrom(const DrawObjBase * para)
	{

	}

	void CBarChartDraw::CopyTo(DrawObjBase ** para) const
	{

	}

	void CBarChartDraw::Init()
	{
		//
		eChartType = ChartType::BAR_CHART;
		ptTopLeft = IAPoint(10, 10, 0);

		//
		bAddLineChart = false;
		bExcAxisY = true;
		dAxisLenRatio = 0.6;
		dAxisExtend = 50;

		dData.clear();
		fData.clear();
		nData.clear();
		dLineData.clear();
		fLineData.clear();
		nLineData.clear();

		strHorAxisLabel = "";
		vecStrHorCusItem.clear();
		nHorAxisLabNum = 3;
		dspHorAxisLabel.Init();
		dspHorStepLabel.Init();
		dspHorAxisLine.Init();
		dspHorStepLine.Init();

		bShowStepDash = false;
		bCoorOrgZero = true;
		strVerAxisLabel = "";
		vecStrVerCusItem.clear();
		nVerAxisLabNum = 3;
		dspVerAxisLabel.Init();
		dspVerStepLabel.Init();
		dspVerAxisLine.Init();
		dspVerStepLine.Init();
	}

	bool CBarChartDraw::IsEmpty() const
	{
		return false;
	}

	HSV::DrawType CBarChartDraw::GetDrawType() const
	{
		return HSV::DrawType();
	}

	bool CBarChartDraw::IsRoiReg() const
	{
		return false;
	}

	int CBarChartDraw::Release()
	{
		int nRet = (int)Error_ID::ERR_OK;


		return nRet;
	}

	template <class T>
	bool CheckData(vector<string> items, vector<T> datas)
	{
		if (datas.size() < 1)
		{
			return false;
		}
		if (items.size() > 0 && items.size() != datas.size())
		{
			return false;
		}
		return true;
	}

	bool CBarChartDraw::CheckDataValidity()
	{
		switch (eDataType)
		{
		case DebugView::DataType::INT:
			return CheckData<int>(vecStrHorCusItem, nData);
		case DebugView::DataType::FLOAT:
			return CheckData<float>(vecStrHorCusItem, fData);
		case DebugView::DataType::DOUBLE:
			return CheckData<double>(vecStrHorCusItem, dData);
		}

		return false;
	}

	template <class T>
	bool GetChartDataSize(vector<T> datas, int & rows, int & cols)
	{
		rows = 0;
		cols = 0;

		if (datas.size() > 0)
		{
			rows = 1;
			cols = (int)datas.size();
		}
		return true;
	}

	bool CBarChartDraw::GetDataSize(int & rows, int & cols)
	{
		rows = 0;
		cols = 0;

		switch (eDataType)
		{
		case DebugView::DataType::INT:
			GetChartDataSize<int>(nData, rows, cols);
			break;
		case DebugView::DataType::FLOAT:
			GetChartDataSize<float>(fData, rows, cols);
			break;
		case DebugView::DataType::DOUBLE:
			GetChartDataSize<double>(dData, rows, cols);
		}
		return true;
	}

	bool CBarChartDraw::GetChartSize(int & width, int & height,int pageWid, int pageHgt)
	{
		CalcChartSize(pageWid, pageHgt);

		width = nChartWidth;
		height = nChartHeight;

		return true;
	}

	bool CBarChartDraw::CalcChartSize(int pageWid, int pageHgt)
	{
		//复位
		nChartWidth = 0;
		nChartHeight = 0;
		//计算
		if (!CheckDataValidity())
		{
			return false;
		}
		int rows, cols;
		GetDataSize(rows, cols);
		if (cols == 0)
		{//数据无效
			return false;
		}
		int horAxisWid = 0, verAxisWid = 0;
		int txtWid, txtHgt;

		//////////////横轴长//////////////
		GetTextSize(strHorAxisLabel, dspHorAxisLabel.strFont, dspHorAxisLabel.nFontSize, txtWid, txtHgt);
		//计算时图表与页间隔预留50pix
	    nHorStepPix = max(2, static_cast<int>((pageWid- dAxisExtend - txtWid)/ (cols + 2)));
		horAxisWid = (cols + 2) * nHorStepPix;//间隔标签延伸2个单位
		nChartWidth += horAxisWid+ txtWid;
		//横轴单步标签计算(自定义标签可能为空,使用默认值进行计算),默认6位字符
		GetTextSize("123456", dspHorStepLabel.strFont, dspHorStepLabel.nFontSize, txtWid, txtHgt);
		nHorAxisLabNum =  horAxisWid / txtWid;
		int horInterval = floor(cols / nHorAxisLabNum);
		nHorAxisLabNum = floor(cols / horInterval);//重新计算绘制数量
		nChartHeight += txtHgt;

		//////////////纵轴长//////////////
		verAxisWid = horAxisWid*dAxisLenRatio;//暂时按横轴长*0.6计算
		GetTextSize(strVerAxisLabel, dspVerAxisLabel.strFont, dspVerAxisLabel.nFontSize, txtWid, txtHgt);
		nChartHeight += verAxisWid + txtHgt+ dAxisExtend;
		//纵轴单步标签计算,默认6位字符
		GetTextSize("123456", dspVerStepLabel.strFont, dspVerStepLabel.nFontSize, txtWid, txtHgt);
		nVerAxisLabNum =  verAxisWid / txtHgt;
		nChartWidth += txtWid;
		//////////////标题和备注//////////////
		GetTextSize(strChartTitle, dspChartTitle.strFont, dspChartTitle.nFontSize, txtWid, txtHgt);
		nChartHeight += txtHgt;
		GetTextSize(strChartNote, dspChartNote.strFont, dspChartNote.nFontSize, txtWid, txtHgt);
		nChartHeight += txtHgt;

		return true;
	}


}

