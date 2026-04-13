/********************************************************************************************************************************
* 功能说明：饼图类
* 该类继承自CChartDraw
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
#include "ChartDraw.h"
namespace DebugView
{
	//图表类
	class  CPieChartDraw :public CChartDraw
	{
	public:
		CPieChartDraw();
		virtual~CPieChartDraw();

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
		virtual	bool CheckDataValidity();
		//获取数据长度
		virtual bool GetDataSize(int &rows, int &cols);
		//获取图表长宽
		virtual bool GetChartSize(int &width, int &height, int pageWid, int pageHgt);
		//计算图表尺寸
		bool CalcChartSize(int pageWid, int pageHgt);

	public:
		//图表数据
		std::vector<double> dData;//数据
		std::vector<float> fData;
		std::vector<int> nData;
		std::vector<CDispPara> dspAnnu;//扇形颜色

		int chartRadius;//圆半径
		std::vector<std::string> vecStrCusItem;//自定义子项
		
		
	};
}
