/********************************************************************************************************************************
* 功能说明：图表类
* 该类继承自HSV::DrawObjBase，用于显示图表
* Ver        修改人      变更日期             变更内容
* ───────────────────────────────────────────────────────────────
* V1.0        xlh     11/05/2024, 10:02          初版
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
#include "DispComm.h"

namespace DebugView
{
	//图表类
	class  CChartDraw :public HSV::DrawObjBase
	{
	public:
		CChartDraw();
		virtual~CChartDraw();

		//继承DrawObjBase必须实现接口
		virtual void CopyFrom(const DrawObjBase* para);
		virtual void CopyTo(DrawObjBase** para) const;
		virtual void Init();
		virtual bool IsEmpty() const;
		virtual HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;

		//
		int Release();

		//数据有效性检查
		virtual	bool CheckDataValidity() = 0;
		
		//获取数据长度
		virtual bool GetDataSize(int &rows,int &cols)=0;
		//获取表格尺寸
		virtual bool GetChartSize(int &width,int &height, int pageWid, int pageHgt)=0;
	public:
		DebugView::ChartType eChartType;//图表类型
		DebugView::DataType eDataType;//表格数据类型
		IAPoint ptTopLeft;//左上角点

		//图标标题（顶部显示）
		std::string strChartTitle;
		CDispPara dspChartTitle;

		//图标题注（底部显示）
		std::string strChartNote;
		CDispPara dspChartNote;

	protected:
		//图表尺寸相关
		int nChartWidth;
		int nChartHeight;

		
	};
}
