//#include"stdafx.h"
#include"TextDisp.h"
#include"HSDrawObj.h"


namespace DebugView
{
	CTextDisp::CTextDisp()
	{
		textDisp = nullptr;

		Init();
	}

	CTextDisp::CTextDisp(HSV::TextDraw * text, AlignTypeEnum align)
	{
		textDisp = nullptr;

		Init();
		if (text == nullptr)
		{
			return;
		}

		textDisp = new HSV::TextDraw();
		eTextType = TextDrawType::TEXT;
		eAlignType = align;
		*(HSV::TextDraw *)textDisp = *text;
	}

	CTextDisp::CTextDisp(HSV::TextUnionDraw * text, AlignTypeEnum align)
	{
		textDisp = nullptr;

		Init();
		if (text == nullptr)
		{
			return;
		}

		textDisp = new HSV::TextUnionDraw();
		eTextType = TextDrawType::TEXT_UNION;
		eAlignType = align;
		*(HSV::TextUnionDraw *)textDisp = *text;
	}

	CTextDisp::CTextDisp(HSV::TextTupleDraw * text, AlignTypeEnum align)
	{
		textDisp = nullptr;

		Init();
		if (text == nullptr)
		{
			return;
		}

		textDisp = new HSV::TextTupleDraw();
		eTextType = TextDrawType::TEXT_TUPLE;
		eAlignType = align;
		*(HSV::TextTupleDraw *)textDisp = *text;
	}

	CTextDisp::CTextDisp(CTableDraw* text)
	{
		textDisp = nullptr;

		Init();
		if (text == nullptr)
		{
			return;
		}

		textDisp = new CTableDraw();
		eTextType = TextDrawType::TABLE;
		*(CTableDraw *)textDisp = *text;

	}

	CTextDisp::CTextDisp(CChartDraw * text)
	{
		textDisp = nullptr;

		Init();
		if (text == nullptr)
		{
			return;
		}
		eTextType = TextDrawType::CHART;

		CChartDraw  *pChart = (CChartDraw *)text;
		switch (pChart->eChartType)
		{
		case ChartType::BAR_CHART:
		{
			textDisp = new CBarChartDraw();
			*(CBarChartDraw *)textDisp = *(CBarChartDraw *)text;

			break;
		}
		case ChartType::LINE_CHART:
		{
			textDisp = new CLineChartDraw();
			*(CLineChartDraw *)textDisp = *(CLineChartDraw *)text;
			break;
		}
		case ChartType::PIE_CHART:
		{
			textDisp = new CPieChartDraw();
			*(CPieChartDraw *)textDisp = *(CPieChartDraw *)text;
			break;
		}
		default:
			break;
		}
	}

	CTextDisp::~CTextDisp()
	{
		Release();
	}

	void CTextDisp::CopyFrom(const DrawObjBase * para)
	{
		
	}

	void CTextDisp::CopyTo(DrawObjBase ** para) const
	{

	}

	void CTextDisp::Init()
	{
		eTextType = TextDrawType::UNDEFINE;
		eAlignType = AlignTypeEnum::ALIGN_LEFT;

		strName = "";
		nTextIndent = 5;

		Release();
	}

	bool CTextDisp::IsEmpty() const
	{

		return false;
	}

	HSV::DrawType CTextDisp::GetDrawType() const
	{
		return HSV::DrawType::DRAW_TEXT;

	}

	bool CTextDisp::IsRoiReg() const
	{
		return false;
	}

	int CTextDisp::Release()
	{
		int nRet = (int)Error_ID::ERR_OK;

		//�ı������ͷ�
		if (textDisp != nullptr)
		{
			delete textDisp;
			textDisp = nullptr;
		}

		return nRet;
	}

	int CTextDisp::SetText(HSV::TextDraw * text)
	{
		int nRet = (int)Error_ID::ERR_OK;

		if (text == nullptr)
		{
			return (int)Error_ID::ERR_TEXT_NULL;
		}

		//�����ı�
		if (textDisp == nullptr)
		{//���ı�
			textDisp = new HSV::TextDraw(*text);
			eTextType = TextDrawType::TEXT;
		}
		else if (eTextType != TextDrawType::TEXT)
		{//�ı����Ͳ�һ��
			delete  textDisp;
			textDisp = new HSV::TextDraw(*text);
			eTextType = TextDrawType::TEXT;
		}
		else
		{
			*(HSV::TextDraw *)textDisp = *text;
		}

		return nRet;
	}

	int CTextDisp::SetText(HSV::TextUnionDraw * text)
	{
		int nRet = (int)Error_ID::ERR_OK;

		if (text == nullptr)
		{
			return (int)Error_ID::ERR_TEXT_NULL;
		}

		//�����ı�
		if (textDisp == nullptr)
		{//���ı�
			textDisp = new HSV::TextUnionDraw(*text);
			eTextType = TextDrawType::TEXT_UNION;
		}
		else if (eTextType != TextDrawType::TEXT_UNION)
		{//�ı����Ͳ�һ��
			delete  textDisp;
			textDisp = new HSV::TextUnionDraw(*text);
			eTextType = TextDrawType::TEXT_UNION;
		}
		else
		{
			*(HSV::TextUnionDraw*)textDisp = *text;
		}

		return nRet;
	}

	int CTextDisp::SetText(HSV::TextTupleDraw * text)
	{
		int nRet = (int)Error_ID::ERR_OK;

		if (text == nullptr)
		{
			return (int)Error_ID::ERR_TEXT_NULL;
		}

		//�����ı�
		if (textDisp == nullptr)
		{//���ı�
			textDisp = new HSV::TextTupleDraw(*text);
			eTextType = TextDrawType::TEXT_TUPLE;
		}
		else if (eTextType != TextDrawType::TEXT_TUPLE)
		{//�ı����Ͳ�һ��
			delete  textDisp;
			textDisp = new HSV::TextTupleDraw(*text);
			eTextType = TextDrawType::TEXT_TUPLE;
		}
		else
		{
			*(HSV::TextTupleDraw*)textDisp = *text;
		}

		return nRet;
	}

	int CTextDisp::SetText(CTableDraw* text)
	{
		int nRet = (int)Error_ID::ERR_OK;

		if (text == nullptr)
		{
			return (int)Error_ID::ERR_TEXT_NULL;
		}

		//�����ı�
		if (textDisp == nullptr)
		{//���ı�
			textDisp = new CTableDraw(*text);
			eTextType = TextDrawType::TABLE;
		}
		else if (eTextType != TextDrawType::TABLE)
		{//�ı����Ͳ�һ��
			delete  textDisp;
			textDisp = new CTableDraw(*text);
			eTextType = TextDrawType::TABLE;
		}
		else
		{
			*(CTableDraw*)textDisp = *text;
		}

		return nRet;
	}

	int CTextDisp::SetText(CChartDraw * text)
	{
		int nRet = (int)Error_ID::ERR_OK;

		if (text == nullptr)
		{
			return (int)Error_ID::ERR_TEXT_NULL;
		}

		/*ChartTypeEuum type = text->eChartType;
		eTextType = TextDrawTypeEnum::CHART;
		switch (type)
		{
		case DebugView::ChartTypeEuum::BAR_CHART:
		{
			if (textDisp == nullptr)
			{
				textDisp = new CBarChartDraw(text);
			}
			else if (eTextType != TextDrawTypeEnum::TABLE)
			{
				delete  textDisp;
				textDisp = new CBarChartDraw(text);
			}
			else
			{
				*(CBarChartDraw*)textDisp = *text;
			}
			break;
		}
		case DebugView::ChartTypeEuum::LINE_CHART:
		{
			if (textDisp == nullptr)
			{
				textDisp = new CLineChartDraw(text);
			}
			else if (eTextType != TextDrawTypeEnum::TABLE)
			{
				delete  textDisp;
				textDisp = new CLineChartDraw(text);
			}
			else
			{
				*(CLineChartDraw*)textDisp = *text;
			}
			break;
		}
		case DebugView::ChartTypeEuum::PIE_CHART:
		{
			if (textDisp == nullptr)
			{
				textDisp = new CPieChartDraw(text);
			}
			else if (eTextType != TextDrawTypeEnum::TABLE)
			{
				delete  textDisp;
				textDisp = new CPieChartDraw(text);
			}
			else
			{
				*(CPieChartDraw*)textDisp = *text;
			}
			break;
		}
		default:
			break;
		}
*/
		return nRet;
	}

}
