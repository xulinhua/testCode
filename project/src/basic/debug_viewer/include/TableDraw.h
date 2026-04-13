/********************************************************************************************************************************
* 功能说明：表格文本类
* 该类继承自HSV::DrawObjBase，用于显示表格类数据文本
* Ver        修改人      变更日期             变更内容
* ───────────────────────────────────────────────────────────────
* V1.0        xlh     09/09/2024, 10:02          初版
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
#include "IAPtStruct.h"

namespace DebugView
{
	//表格文本类
	class  CTableDraw :public HSV::DrawObjBase
	{
	public:
		CTableDraw();
		virtual~CTableDraw();

		virtual void CopyFrom(const DrawObjBase* para);
		virtual void CopyTo(DrawObjBase** para) const;
		virtual void Init();
		virtual bool IsEmpty() const;
		virtual HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;
		int Release();

		//数据有效性检查
		bool CheckDataValidity();
		//获取数据尺寸
		bool GetDataSize(int &rows, int &cols);
		//获取第一行数据文本
		std::string GetRowDataString();
		//获取图表长宽
		virtual bool GetTableSize(int &width, int &height, int pageWid, int pageHgt);
		//计算图表尺寸
		bool CalcTableSize(int pageWid, int pageHgt);

	public:
		DebugView::DataType eDataType;//表格数据类型
		IAPoint ptTopLeft;//左上角点

		std::vector<std::vector<int>> nDatas;//数据
		std::vector<std::vector<float>> fDatas;
		std::vector<std::vector<double>> dDatas;

		std::vector<std::string> vecHorTitle;//标题
		std::vector<std::string> vecVerTitle;

		std::string strDataFont;//字体
		std::string strHorTitleFont;
		std::string strVerTitleFont;
		int nDataFontSize;//字体大小
		int nHorTitleFontSize;
		int nVerTitleFontSize;
		
		HSV::ScalarGC colorDataFore;//数据前景色
		HSV::ScalarGC colorDataBack;//数据背景色
		HSV::ScalarGC colorHorTitleFore;//水平标题前景背景色
		HSV::ScalarGC colorHorTitleBack;
		HSV::ScalarGC colorVerTitleFore;//垂直标题前景背景色
		HSV::ScalarGC colorVerTitleBack;

		int nTableLineThick;//表格线条粗细
		HSV::ScalarGC colorTableLine;//表格线条颜色

	protected:
		//图表尺寸相关
		int nTableWidth;
		int nTableHeight;

	};
}
