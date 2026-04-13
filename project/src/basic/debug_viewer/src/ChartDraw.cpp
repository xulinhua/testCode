//#include"stdafx.h"
#include"ChartDraw.h"

using namespace std;
using namespace HSV;

namespace DebugView
{
	CChartDraw::CChartDraw()
	{
		Init();
	}

	CChartDraw::~CChartDraw()
	{
		Release();
	}

	void CChartDraw::CopyFrom(const DrawObjBase * para)
	{

	}

	void CChartDraw::CopyTo(DrawObjBase ** para) const
	{

	}

	void CChartDraw::Init()
	{
		eChartType = ChartType::UNDEFINE;
		eDataType = DataType::UNDEFINE;

		strChartTitle = "";
		dspChartTitle.Init();
		strChartNote = "";
		dspChartNote.Init();

		nChartWidth = 0;
		nChartHeight = 0;
	}

	bool CChartDraw::IsEmpty() const
	{
		return false;
	}

	HSV::DrawType CChartDraw::GetDrawType() const
	{
		return HSV::DrawType();
	}

	bool CChartDraw::IsRoiReg() const
	{
		return false;
	}

	int CChartDraw::Release()
	{
		int nRet = (int)Error_ID::ERR_OK;


		return nRet;
	}

	
}

