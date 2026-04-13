//#include"stdafx.h"
#include"TableDraw.h"
#include"DispComm.h"

using namespace std;
using namespace HSV;

namespace DebugView
{
	CTableDraw::CTableDraw()
	{
		Init();
	}

	CTableDraw::~CTableDraw()
	{
		Release();
	}

	void CTableDraw::CopyFrom(const DrawObjBase * para)
	{
	}

	void CTableDraw::CopyTo(DrawObjBase ** para) const
	{
	}

	void CTableDraw::Init()
	{
		//bUseCustColor = false;
		eDataType = DebugView::DataType::INT;
		ptTopLeft = IAPoint(10.0, 10.0,0.0);
		nDatas.clear();
		fDatas.clear();
		dDatas.clear();

		vecHorTitle.clear();
		vecVerTitle.clear();
		strDataFont = "Arial";
		strHorTitleFont = "Arial";
		strVerTitleFont = "Arial";
		nDataFontSize = 50;
		nHorTitleFontSize = 50;
		nVerTitleFontSize = 50;

		colorDataFore = ScalarGC(0, 255, 0);
		colorDataBack = ScalarGC(0, 255, 0);
		colorHorTitleFore = ScalarGC(0, 255, 0);
		colorHorTitleBack = ScalarGC(0, 255, 0);
		colorVerTitleFore = ScalarGC(0, 255, 0);
		colorVerTitleBack = ScalarGC(0, 255, 0);

		nTableLineThick = 1;
		colorTableLine = ScalarGC(20, 200, 40);
	}

	bool CTableDraw::IsEmpty() const
	{
		return false;
	}

	HSV::DrawType CTableDraw::GetDrawType() const
	{
		return HSV::DrawType();
	}

	bool CTableDraw::IsRoiReg() const
	{
		return false;
	}

	int CTableDraw::Release()
	{
		int nRet = (int)Error_ID::ERR_OK;


		return nRet;
	}

	template <class T>
	bool CheckData( vector<vector<T>> datas, vector<string> itemsHor, vector<string> itemsVer)
	{
		size_t rows = 0, cols = 0;
		rows = datas.size();
		for (size_t i = 0; i < datas.size(); i++)
		{
			if (i == 0)
			{
				cols = datas[i].size();
			}
			else if (i != 0 && datas[i].size() != cols)
			{//������һ��
				return false;
			}
		}

		//ˮƽ��������
		if (itemsHor.size() != 0 && itemsHor.size() != cols)
		{
			return false;
		}
		//��ֱ��������
		if (itemsVer.size() != 0 && itemsVer.size() != rows)
		{
			return false;
		}
		return true;
	}
	bool CTableDraw::CheckDataValidity()
	{
		//�������ݳ���
		int cols = 0;
		int rows = 0;
		switch (eDataType)
		{
		case DebugView::DataType::INT:
			return CheckData<int>(nDatas, vecHorTitle,vecVerTitle);
		case DebugView::DataType::FLOAT:
			return CheckData<float>(fDatas, vecHorTitle, vecVerTitle);
		case DebugView::DataType::DOUBLE:
			return CheckData<double>(dDatas, vecHorTitle, vecVerTitle);
		}
		return false;
	}

	template <class T>
	bool GetChartDataSize(vector<vector<T>> datas, int& rows, int& cols)
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

	bool CTableDraw::GetDataSize(int & rows, int & cols)
	{
		rows = 0;
		cols = 0;
		switch (eDataType)
		{
		case DebugView::DataType::INT:
			GetChartDataSize<int>(nDatas, rows, cols);
			break;
		case DebugView::DataType::FLOAT:
			GetChartDataSize<float>(fDatas, rows, cols);
			break;
		case DebugView::DataType::DOUBLE:
			GetChartDataSize<double>(dDatas, rows, cols);
			break;
		}
		return true;
	}

	string CTableDraw::GetRowDataString()
	{
		string result = "";

		switch (eDataType)
		{
		case DebugView::DataType::INT:
		{
			if (nDatas.size() > 0)
			{
				for (size_t i = 0; i < nDatas[0].size(); i++)
				{
					result += to_string(nDatas[0][i]);
				}
			}
			break;
		}
		case DebugView::DataType::FLOAT:
		{
			if (fDatas.size() > 0)
			{
				for (size_t i = 0; i < fDatas[0].size(); i++)
				{
					result += to_string(fDatas[0][i]);
				}
			}
			break;
		}
		case DebugView::DataType::DOUBLE:
		{
			if (dDatas.size() > 0)
			{
				for (size_t i = 0; i < dDatas[0].size(); i++)
				{
					result += to_string(dDatas[0][i]);
				}
			}
			break;
		}
		}
		
		return result;
	}

	bool CTableDraw::GetTableSize(int & width, int & height, int pageWid, int pageHgt)
	{
		CalcTableSize(pageWid, pageHgt);

		width = nTableWidth;
		height = nTableHeight;

		return true;
	}

	bool CTableDraw::CalcTableSize(int pageWid, int pageHgt)
	{
		//��λ
		nTableWidth = 0;
		nTableHeight = 0;
		//����
		if (!CheckDataValidity())
		{
			return false;
		}
		int rows, cols;
		GetDataSize(rows, cols);
		if (cols == 0)
		{//������Ч
			return false;
		}
		int horAxisWid = 0, verAxisWid = 0;
		int txtWid, txtHgt;
		/////////////////���ȼ���/////////////////
		//ˮƽ���ⳤ��
		size_t horTitleNum = vecHorTitle.size();
		if (horTitleNum > 0)
		{
			string strTitleAll = "";
			for (size_t i = 0; i < horTitleNum; i++)
			{
				strTitleAll += vecHorTitle[i];
			}
			GetTextSize(strTitleAll, strHorTitleFont, nHorTitleFontSize, txtWid, txtHgt);
			nTableWidth = (txtWid > nTableWidth) ? txtWid : nTableWidth;
		}
		//��һ�����ݳ���
		int size1 = 0, size2 = 0;
		GetDataSize(size1, size2);
		if (size1 > 0 && size2 > 0)
		{
			string strRow = GetRowDataString();
			GetTextSize(strRow, strDataFont, nDataFontSize, txtWid, txtHgt);
			nTableWidth = (txtWid > nTableWidth) ? txtWid : nTableWidth;
		}
		/////////////////�߶ȼ���/////////////////
		int dataRows = 0, dataCols;
		int w1 = 0, h1 = 0, w2 = 0, h2 = 0, w3 = 0, h3 = 0;//���ݡ�ˮƽ���⡢��ֱ���� ���峤��
		GetDataSize(dataRows, dataCols);
		GetTextSize("Aa1_", strDataFont, nDataFontSize, w1, h1);
		if (vecHorTitle.size() > 0)
		{
			GetTextSize("Aa1_", strHorTitleFont, nHorTitleFontSize, w2, h2);
		}
		if (vecVerTitle.size() > 0)
		{
			GetTextSize("Aa1_", strVerTitleFont, nVerTitleFontSize, w3, h3);
		}
		int hgtMax = h1;
		hgtMax = max(hgtMax, h2);
		hgtMax = max(hgtMax, h3) + 3 * nTableLineThick;
		txtHgt = hgtMax*(dataRows + (int)(vecHorTitle.size() > 0));

		nTableHeight = txtHgt;

		return true;
	}

}

