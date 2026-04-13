/********************************************************************************************************************************
* 功能说明：图表类
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
	class  CLineChartDraw :public CChartDraw
	{
	public:
		CLineChartDraw();
		virtual~CLineChartDraw();

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
		//
		double dAxisLenRatio;//轴长横纵比
		double dAxisExtend;//XY轴延伸长度

		//图表数据(折线图支持多组数据)
		std::vector<std::vector<double>> dData;//数据
		std::vector<std::vector<float>> fData;
		std::vector<std::vector<int>> nData;
		std::vector<CDispPara> dspData;//矩形颜色

		//横轴相关
		std::string strHorAxisLabel;//横轴标签
		std::vector<std::string> vecStrHorCusItem;//自定义子项
		int nHorAxisLabNum;//标签间隔（数据量太多时需要简化，未实现）
		int nHorStepPix;//单步间隔宽度
		CDispPara dspHorAxisLabel;//横轴标签显示
		CDispPara dspHorStepLabel;//横轴间隔标签
		CDispPara dspHorAxisLine;//横轴直线显示
		CDispPara dspHorStepLine;//间隔直线显示

		//纵轴相关
		bool bShowStepDash;//是否显示贯穿虚线
		bool bCoorOrgZero;//坐标原点为零点（true:负数在X轴下方，false:所有值都在X轴上方）
		std::string strVerAxisLabel;//标签
		std::vector<std::string> vecStrVerCusItem;//自定义子项
		int nVerAxisLabNum;//标签间隔（数据量太多时需要简化，未实现）
		CDispPara dspVerAxisLabel;//纵轴标签显示
		CDispPara dspVerStepLabel;//纵轴间隔标签
		CDispPara dspVerAxisLine;//纵轴直线显示
		CDispPara dspVerStepLine;//间隔直线显示
		CDispPara dspVerStepDash;//虚线贯穿显示

	};
}
