/********************************************************************************************************************************
* 功能说明：文本信息类
* 该类继承自HSV::DrawObjBase，用于所有文本信息（文本、组合文本、表格）的兼容
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
#include"HSDrawObj.h"
#include"DbgInfoDefine.h"
#include"TableDraw.h"
#include"ChartDraw.h"
#include"ChartDraw_Bar.h"
#include"ChartDraw_Line.h"
#include"ChartDraw_Pie.h"

namespace DebugView
{
	//文本信息
	class CTextDisp :public HSV::DrawObjBase
	{
	public:
		CTextDisp();
		CTextDisp(HSV::TextDraw* text, AlignTypeEnum align= AlignTypeEnum::ALIGN_LEFT);
		CTextDisp(HSV::TextUnionDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		CTextDisp(HSV::TextTupleDraw* text, AlignTypeEnum align = AlignTypeEnum::ALIGN_LEFT);
		CTextDisp(CTableDraw* text);
		CTextDisp(CChartDraw* text);


		virtual~CTextDisp();
		//继承DrawObjBase实现的接口
		virtual void CopyFrom(const DrawObjBase* para);
		virtual void CopyTo(DrawObjBase** para) const;
		virtual void Init();
		virtual bool IsEmpty() const;
		virtual HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;

		//
		int Release();

		int SetText(HSV::TextDraw* text);
		int SetText(HSV::TextUnionDraw* text);
		int SetText(HSV::TextTupleDraw* text);
		int SetText(CTableDraw* text);
		int SetText(CChartDraw* text);

	public:
		TextDrawType eTextType;//文本类型
		AlignTypeEnum eAlignType;//对齐方式

		std::string strName;//名称
		int nTextIndent;//文本缩进距离
		HSV::DrawObjBase *textDisp;//文本数据
	};

}
