//#include"stdafx.h"
#include"ChartDraw_Pie.h"

using namespace std;
using namespace HSV;

namespace DebugView
{
	CPieChartDraw::CPieChartDraw()
	{
		Init();
	}

	CPieChartDraw::~CPieChartDraw()
	{
		Release();
	}

	void CPieChartDraw::CopyFrom(const DrawObjBase * para)
	{

	}

	void CPieChartDraw::CopyTo(DrawObjBase ** para) const
	{

	}

	void CPieChartDraw::Init()
	{
		eChartType = ChartType::PIE_CHART;
		ptTopLeft = IAPoint(10, 10, 0);

		dData.clear();
		fData.clear();
		nData.clear();
		
		vecStrCusItem.clear();
		dspAnnu.clear();
	}

	bool CPieChartDraw::IsEmpty() const
	{
		return false;
	}

	HSV::DrawType CPieChartDraw::GetDrawType() const
	{
		return HSV::DrawType();
	}

	bool CPieChartDraw::IsRoiReg() const
	{
		return false;
	}

	int CPieChartDraw::Release()
	{
		int nRet = (int)Error_ID::ERR_OK;


		return nRet;
	}
	
	bool CPieChartDraw::CheckDataValidity()
	{
		int cols = 0;
		int rows = 0;

		switch (eDataType)
		{
		case DebugView::DataType::INT:
			return (nData.size() < 1);
		case DebugView::DataType::FLOAT:
			return (fData.size() < 1);
		case DebugView::DataType::DOUBLE:
			return (dData.size() < 1);
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
			cols = datas.size();
		}
		return true;
	}

	bool CPieChartDraw::GetDataSize(int & rows, int & cols)
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
			break;
		}
		return true;
	}

	bool CPieChartDraw::GetChartSize(int & width, int & height, int pageWid, int pageHgt)
	{
		CalcChartSize(pageWid, pageHgt);
		
		width = nChartWidth;
		height = nChartHeight;

		return true;
	}

	bool CPieChartDraw::CalcChartSize(int pageWid, int pageHgt)
	{
		return false;
	}
	
}

