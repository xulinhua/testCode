/********************************************************************************************************************************
* 功能说明：柱状图类
* 该类继承自HSV::DrawObjBase，用于显示表格类数据文本
* Ver        修改人      变更日期             变更内容
* ───────────────────────────────────────────────────────────────
* V1.0        xlh     11/05/2024, 10:02          初版
* V1.1        xlh     11/06/2024, 12:02          增加折线图绘制数据
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
	class  CBarChartDraw :public CChartDraw
	{
	public:
		CBarChartDraw();
		virtual~CBarChartDraw();

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
		bool CalcChartSize(int pageWid,int pageHgt);

	public:
		bool bAddLineChart;//是否叠加折线绘图
		bool bExcAxisY;//是否独享Y轴,true:柱状图和折线图的Y轴分布左右两侧，范围标签独立
		double dAxisLenRatio;//轴长横纵比
		double dAxisExtend;//XY轴延伸长度

		//图表数据
		std::vector<double> dData;//数据
		std::vector<float> fData;
		std::vector<int> nData;
		CDispPara dspData;//矩形颜色
		//叠加折线图数据(绘图接口尚未未实现)
		std::vector<std::vector<double>> dLineData;//数据
		std::vector<std::vector<float>> fLineData;
		std::vector<std::vector<int>> nLineData;
		std::vector<CDispPara> dspDataLine;//折线颜色

		//横轴相关
		std::string strHorAxisLabel;//横轴标签
		std::vector<std::string> vecStrHorCusItem;//自定义子项
		int nHorAxisLabNum;//标签间隔（数据量太多时需要简化，未实现）
		int nHorStepPix;//单步间隔宽度(单个值矩形的宽度)
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
