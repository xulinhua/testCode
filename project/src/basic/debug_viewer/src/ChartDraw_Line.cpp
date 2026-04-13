//#include"stdafx.h"
#include"ChartDraw_Line.h"
using namespace std;
using namespace HSV;

namespace DebugView
{
	CLineChartDraw::CLineChartDraw()
	{
		Init();
	}

	CLineChartDraw::~CLineChartDraw()
	{
		Release();
	}

	void CLineChartDraw::CopyFrom(const DrawObjBase * para)
	{

	}

	void CLineChartDraw::CopyTo(DrawObjBase ** para) const
	{

	}

	void CLineChartDraw::Init()
	{
		eChartType = ChartType::LINE_CHART;
		ptTopLeft = IAPoint(10,10,0);

		dAxisLenRatio = 0.6;
		dAxisExtend = 50;

		dData.clear();
		fData.clear();
		nData.clear();

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
		bShowStepDash = false;
		dspVerAxisLabel.Init();
		dspVerStepLabel.Init();
		dspVerAxisLine.Init();
		dspVerStepLine.Init();
	}

	bool CLineChartDraw::IsEmpty() const
	{
		return false;
	}

	HSV::DrawType CLineChartDraw::GetDrawType() const
	{
		return HSV::DrawType();
	}

	bool CLineChartDraw::IsRoiReg() const
	{
		return false;
	}

	int CLineChartDraw::Release()
	{
		int nRet = (int)Error_ID::ERR_OK;


		return nRet;
	}
	
	template <class T>
	bool CheckData(vector<string> items,vector<vector<T>> datas)
	{
		size_t rows = 0, cols = 0;
		rows = datas.size();
		if (rows < 1)
		{
			return false;
		}
		cols = datas[0].size();
		if (items.size() > 0 && items.size() != cols)
		{
			return false;
		}
		for (size_t i = 1; i < rows; i++)
		{
			if (datas[i].size() != cols)
			{
				return false;
			}
		}
		return true;
	}

	bool CLineChartDraw::CheckDataValidity()
	{
		int cols = 0;
		int rows = 0;

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
	bool GetChartDataSize(vector<vector<T>> datas, int & rows, int & cols)
	{
		rows = 0;
		cols = 0;
		rows = (int)datas.size();
		if (datas.size() > 0)
		{
			cols = (int)datas[0].size();
		}
		return true;
	}
	bool CLineChartDraw::GetDataSize(int & rows, int & cols)
	{
		rows = 0;
		cols = 0;
		switch (eDataType)
		{
		case DebugView::DataType::INT:
			GetChartDataSize<int>(nData, rows,cols);
			break;
		case DebugView::DataType::FLOAT:
			GetChartDataSize<float>(fData, rows, cols);
			break;
		case DebugView::DataType::DOUBLE:
			GetChartDataSize<double>(dData, rows, cols);
			break;
		}
		return true;
	}

	bool CLineChartDraw::GetChartSize(int & width, int & height, int pageWid, int pageHgt)
	{
		CalcChartSize(pageWid, pageHgt);
		
		width = nChartWidth;
		height = nChartHeight;

		return true;
	}
	bool CLineChartDraw::CalcChartSize(int pageWid, int pageHgt)
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
		//计算时图标与页间隔预留50pix
		nHorStepPix = max(2, static_cast<int>((pageWid - dAxisExtend - txtWid) / (cols + 2)));
		horAxisWid = (cols + 2) * nHorStepPix;//间隔标签延伸2个单位
		nChartWidth = +horAxisWid + txtWid;
		//横轴单步标签计算(自定义标签可能为空,使用默认值进行计算)
		GetTextSize("123456", dspHorStepLabel.strFont, dspHorStepLabel.nFontSize, txtWid, txtHgt);
		nHorAxisLabNum = horAxisWid / txtWid;
		int horInterval = floor(cols / nHorAxisLabNum);
		nHorAxisLabNum = floor(cols / horInterval);//重新计算绘制数量
		nChartHeight += txtHgt;

		//////////////纵轴长//////////////
		verAxisWid = horAxisWid*dAxisLenRatio;//暂时按横轴长*0.6计算
		GetTextSize(strVerAxisLabel, dspVerAxisLabel.strFont, dspVerAxisLabel.nFontSize, txtWid, txtHgt);
		nChartHeight += verAxisWid + txtHgt+ dAxisExtend;
		//纵轴单步标签计算
		GetTextSize("123456", dspVerStepLabel.strFont, dspVerStepLabel.nFontSize, txtWid, txtHgt);
		nVerAxisLabNum = verAxisWid / txtHgt;
		nChartWidth += txtWid;
		//////////////标题和备注//////////////
		GetTextSize(strChartTitle, dspChartTitle.strFont, dspChartTitle.nFontSize, txtWid, txtHgt);
		nChartHeight += txtHgt;
		GetTextSize(strChartNote, dspChartNote.strFont, dspChartNote.nFontSize, txtWid, txtHgt);
		nChartHeight += txtHgt;

		return true;
	}
}

