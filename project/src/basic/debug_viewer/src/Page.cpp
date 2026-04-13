//#include"stdafx.h"
#include"Page.h"
#include"DrawFunc.h"

#define  ToScalar Color_t	
#define  ToPt Point_t
#define  ToImage GCImage2Mat
#define  ToGCImage Mat2GCImage
#define  ToCVRGB(clr) (cv::Scalar(CV_RGB(GetRValue(clr), GetGValue(clr),GetBValue(clr))))

using namespace HSV;
using namespace std;

namespace DebugView
{

	CPage::CPage()
	{
		imgRst = nullptr;

		Init();
	}

	CPage::CPage(PageTypeEnum type, std::string name)
	{
		imgRst = nullptr;

		Init();
		strPageName = name;
		ePageType = type;
	}

	CPage::~CPage()
	{
		Release();
	}

	int CPage::Init()
	{
		int nRet = (int)Error_ID::ERR_OK;

		strPageName = "";
		ePageType = PageTypeEnum::UNDEFINE;
		eTileType = TileImageType::NONE;
		bAdjustBgImgSize = true;
		bTextUnionAlign = true;
		bTileImages = true;

		nPageWidth = 0;
		nPageHeight = 0;
		colorBackground = HSV::ScalarGC(128, 128, 128);//背景默认灰色

		Release();//缓存清空

		nOffsetXL = 5;
		nOffsetYL = 5;
		nOffsetXR = 5;
		nOffSetYR = 5;

		return nRet;
	}

	int CPage::Release()
	{
		int nRet = (int)Error_ID::ERR_OK;

		ClearPageBgImgs();//清空输入图像、结果图像
		ClearDispObject();//清空显示图形
		ClearDispInfo();//清空显示信息
		
		return nRet;
	}

	int CPage::ProcessPage(bool adap)
	{
		int nRet = (int)Error_ID::ERR_OK;
		int rst = nRet;

		//为确保信息页尺寸一致，宽度计算更新移至debugview类处理
		//rst = CalcImgWidth();//计算图像宽度
		//if (rst != (int)Error_ID::ERR_OK)
		//	nRet = rst;

		rst = CalcTextPos();//计算文本位置
		if (rst != (int)Error_ID::ERR_OK) 
			nRet = rst;

		rst = GenBgImage();//生成背景图像
		if (rst != (int)Error_ID::ERR_OK) 
			nRet = rst;

		rst = DrawObjects();//绘制图形
		if (rst != (int)Error_ID::ERR_OK) 
			nRet = rst;

		rst = DrawInfos();//绘制文本
		if (rst != (int)Error_ID::ERR_OK) 
			nRet = rst;

		return nRet;
	}

	int CPage::SetPageSize(int width, int height)
	{
		int nRet = (int)Error_ID::ERR_OK;

		nPageWidth = width;
		nPageHeight = height;

		return nRet;
	}

	int CPage::SetPageBgImg(GCImage * img, int idx)
	{
		int nRet = (int)Error_ID::ERR_OK;

		if (img == nullptr)
		{
			return (int)Error_ID::ERR_IMAGE_NULL;
		}
		if (img->GetWidth() == 0 || img->GetHeight() == 0)
		{
			return (int)Error_ID::ERR_IMAGE_NULL;
		}
		if (idx == 0)
		{//第一张背景图默认有效
			if (vecImgIn.size() == 0)
			{
				GCImage *imgNew = new GCImage();
				vecImgIn.push_back(imgNew);
			}
		}
		else
		{//图像序号异常
			if (idx >= (int)vecImgIn.size())
			{
				return (int)Error_ID::ERR_PAGE_IMG_INDEX_NG;
			}
		}
		/////////////复制图像数据/////////////
		*vecImgIn[idx] = *img;

		return nRet;
	}

	int CPage::SetPageBgImgs(std::vector<GCImage*> imgs)
	{
		int nRet = (int)Error_ID::ERR_OK;

		size_t numImgIn = imgs.size();
		size_t numImgCur = vecImgIn.size();
		//输入图像异常
		if (numImgIn == 0)
		{
			ClearPageBgImgs();
		}
		//保持数据一致
		if (numImgIn > numImgCur)
		{
			for (size_t i = numImgCur; i < numImgIn; i++)
			{
				vecImgIn.push_back(new GCImage());
			}
		}
		else if (numImgIn < numImgCur)
		{
			for (size_t i = numImgCur - 1; i >= numImgIn; i--)
			{
				if (vecImgIn[i] != nullptr)
				{
					delete vecImgIn[i];
					vecImgIn[i] = nullptr;
				}
				vecImgIn.erase(vecImgIn.begin() + i);
			}
		}
		for (size_t i = 0; i < numImgIn; i++)
		{
			if (imgs[i] == nullptr || imgs[i]->GetWidth() == 0 || imgs[i]->GetHeight() == 0)
			{
				continue;//图像无效时不参与绘制处理
				//return (int)Error_ID::Err_IMAGE_NULL;
			}
			*vecImgIn[i] = *imgs[i];
		}
		return nRet;
	}

	int CPage::SetPageBgColor(int R, int G, int B)
	{
		int nRet = (int)Error_ID::ERR_OK;

		colorBackground = HSV::ScalarGC(B, G, R);

		return nRet;
	}

	int CPage::ClearPageBgImgs()
	{
		int nRet = (int)Error_ID::ERR_OK;

		for (size_t i = 0; i < vecImgIn.size(); i++)
		{
			if (vecImgIn[i] != nullptr)
			{
				delete vecImgIn[i];
				vecImgIn[i] = nullptr;
			}
		}
		vecImgIn.clear();

		if (imgRst!=nullptr)
		{
			delete imgRst;
			imgRst = nullptr;
		}

		return nRet;
	}

	int CPage::AddDispObj(HSV::DrawObjBase * obj)
	{
		int nRet = (int)Error_ID::ERR_OK;
		//图形无效
		if (obj == nullptr)
		{
			return (int)Error_ID::ERR_OBJECT_NULL;
		}
		else if (obj->GetDrawType() == HSV::DRAW_NONE)
		{
			return (int)Error_ID::ERR_OBJECT_NULL;
		}
		//添加渲染图形
		HSV::DrawObjBase *objTemp = nullptr;
		HSV::CopyDrawObjDat(obj, &objTemp);

		vecObject.push_back(objTemp);

		return nRet;
	}

	int CPage::ClearDispObject()
	{
		int nRet = (int)Error_ID::ERR_OK;

		for (size_t i = 0; i < vecObject.size(); i++)
		{
			if (vecObject[i] != nullptr)
			{
				delete vecObject[i];
				vecObject[i] = nullptr;
			}
		}
		vecObject.clear();
		return nRet;
	}

	template<typename T>
	int AddInfo_Tmp(std::vector<CTextDisp *>& vecInfos,T * text, AlignTypeEnum align)
	{
		int nRet = (int)Error_ID::ERR_OK;
		//文本无效
		if (text == nullptr)
		{
			return (int)Error_ID::ERR_TEXT_NULL;
		}
		//添加渲染图形
		vecInfos.push_back(new CTextDisp(text, align));
		return nRet;
	}

	int CPage::AddInfo(HSV::TextDraw * text, AlignTypeEnum align)
	{
		return AddInfo_Tmp(vecInfos, text, align);
	}

	int CPage::AddInfo(HSV::TextUnionDraw * text, AlignTypeEnum align)
	{
		return AddInfo_Tmp(vecInfos, text, align);
	}

	int CPage::AddInfo(HSV::TextTupleDraw * text, AlignTypeEnum align)
	{
		return AddInfo_Tmp(vecInfos, text, align);
	}

	template<typename T>
	int InsertInfo_Tmp(std::vector<CTextDisp *>& vecInfos, int index, T * text, AlignTypeEnum align)
	{
		int nRet = (int)Error_ID::ERR_OK;

		if (index >= (int)vecInfos.size())
		{
			return (int)Error_ID::ERR_PAGE_INFO_INDEX_NG;
		}
		vecInfos.insert(vecInfos.begin() + index, new CTextDisp(text, align));

		return nRet;
	}

	int CPage::InsertInfo(int index, HSV::TextDraw * text, AlignTypeEnum align)
	{
		return InsertInfo_Tmp(vecInfos,index,text,align);
	}

	int CPage::InsertInfo(int index, HSV::TextUnionDraw * text, AlignTypeEnum align)
	{
		return InsertInfo_Tmp(vecInfos, index, text, align);
	}

	int CPage::InsertInfo(int index, HSV::TextTupleDraw * text, AlignTypeEnum align)
	{
		return InsertInfo_Tmp(vecInfos, index, text, align);
	}


	template<typename T>
	int SetInfo_Tmp(std::vector<CTextDisp *>& vecInfos, int index, T * text, AlignTypeEnum align)
	{
		int nRet = (int)Error_ID::ERR_OK;

		if (index >= (int)vecInfos.size())
		{
			return (int)Error_ID::ERR_PAGE_INFO_INDEX_NG;
		}
		*vecInfos[index] = CTextDisp(text, align);

		return nRet;
	}

	int CPage::SetInfo(int index, HSV::TextDraw * text, AlignTypeEnum align)
	{
		return SetInfo_Tmp(vecInfos, index, text, align);
	}

	int CPage::SetInfo(int index, HSV::TextUnionDraw * text, AlignTypeEnum align)
	{
		return SetInfo_Tmp(vecInfos, index, text, align);
	}

	int CPage::SetInfo(int index, HSV::TextTupleDraw * text, AlignTypeEnum align)
	{
		return SetInfo_Tmp(vecInfos, index, text, align);
	}

	int CPage::ClearDispInfo()
	{
		int nRet = (int)Error_ID::ERR_OK;

		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			if (vecInfos[i]!=nullptr)
			{
				delete vecInfos[i];
			}
		}
		vecInfos.clear();

		return nRet;
	}

	int CPage::SetPageAllTextDraw(TextDrawPara para)
	{
		int nRet = (int)Error_ID::ERR_OK;

		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			CTextDisp *textDisp=vecInfos[i];
			if (textDisp==nullptr)
			{
				continue;
			}
			if (textDisp->eTextType== TextDrawType::TEXT)
			{
				HSV::TextDraw *text = (HSV::TextDraw *)textDisp;
				text->color_ = HSV::ScalarGC(para.nRgb[2], para.nRgb[1], para.nRgb[0]);
				text->fn_ = para.strFont.data();
				text->nFontSize_ = para.nTextSize;
			}
		}

		return nRet;
	}

	int CPage::SetPageAllTextUnionDraw(TextDrawPara paralab, TextDrawPara parasbj)
	{
		int nRet = (int)Error_ID::ERR_OK;

		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			CTextDisp *textDisp = vecInfos[i];
			if (textDisp == nullptr)
			{
				continue;
			}
			if (textDisp->eTextType == TextDrawType::TEXT_UNION)
			{
				HSV::TextUnionDraw *text = (HSV::TextUnionDraw *)textDisp;

				text->lab_.color_ = HSV::ScalarGC(paralab.nRgb[2], paralab.nRgb[1], paralab.nRgb[0]);
				text->lab_.fn_ = paralab.strFont.data();
				text->lab_.nFontSize_ = paralab.nTextSize;

				text->sbj_.color_ = HSV::ScalarGC(parasbj.nRgb[2], parasbj.nRgb[1], parasbj.nRgb[0]);
				text->sbj_.fn_ = parasbj.strFont.data();
				text->sbj_.nFontSize_ = parasbj.nTextSize;
			}
		}

		return nRet;
	}
	
	int CPage::AddTableInfo(int2D datas, str1D horTitle, str1D verTitle)
	{
		int nRet = (int)Error_ID::ERR_OK;
		
		CTableDraw table = CTableDraw();
		table.nDatas = datas;
		table.vecHorTitle = horTitle;
		table.vecVerTitle = verTitle;
		table.eDataType = DataType::INT;

		CTextDisp *txtDisp = new CTextDisp(&table);
		vecInfos.push_back(txtDisp);

		return nRet;
	}

	int CPage::AddTableInfo(float2D datas, str1D horTitle, str1D verTitle)
	{
		int nRet = (int)Error_ID::ERR_OK;

		CTableDraw table = CTableDraw();
		table.fDatas = datas;
		table.vecHorTitle = horTitle;
		table.vecVerTitle = verTitle;
		table.eDataType = DataType::FLOAT;

		CTextDisp *txtDisp = new CTextDisp(&table);
		vecInfos.push_back(txtDisp);

		return nRet;
	}
	int CPage::AddTableInfo(double2D datas, str1D horTitle, str1D verTitle)
	{
		int nRet = (int)Error_ID::ERR_OK;

		CTableDraw table = CTableDraw();
		table.dDatas = datas;
		table.vecHorTitle = horTitle;
		table.vecVerTitle = verTitle;
		table.eDataType = DataType::DOUBLE;

		CTextDisp *txtDisp = new CTextDisp(&table);
		vecInfos.push_back(txtDisp);

		return nRet;
	}

	int CPage::SetTableInfo(int idx, int2D datas, str1D horTitle, str1D verTitle)
	{
		CTableDraw table =  CTableDraw();
		table.nDatas = datas;
		table.vecHorTitle = horTitle;
		table.vecVerTitle = verTitle;
		table.eDataType = DataType::INT;
		
		int idxCur = 0;
		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			if (vecInfos[i]->eTextType==TextDrawType::TABLE )
			{
				if (vecInfos[i] == nullptr)
				{
					idxCur++;
					continue;
				}
				if ( idx == idxCur)
				{
					*vecInfos[i] = CTextDisp(&table);
					return (int)Error_ID::ERR_OK;
				}
				idxCur++;
			}
		}
		return (int)Error_ID::ERR_PROC_NG;
	}
	
	int CPage::SetTableInfo(int idx, float2D datas, str1D horTitle, str1D verTitle)
	{
		CTableDraw table = CTableDraw();
		table.fDatas = datas;
		table.vecHorTitle = horTitle;
		table.vecVerTitle = verTitle;
		table.eDataType = DataType::INT;

		int idxCur = 0;
		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			if (vecInfos[i] == nullptr)
			{
				idxCur++;
				continue;
			}
			if (vecInfos[i]->eTextType == TextDrawType::TABLE)
			{
				if (idx == idxCur)
				{
					*vecInfos[i] = CTextDisp(&table);
					return (int)Error_ID::ERR_OK;
				}
				idxCur++;
			}
		}
		return (int)Error_ID::ERR_PROC_NG;
	}
	int CPage::SetTableInfo(int idx, double2D datas, str1D horTitle, str1D verTitle)
	{
		CTableDraw table = CTableDraw();
		table.dDatas = datas;
		table.vecHorTitle = horTitle;
		table.vecVerTitle = verTitle;
		table.eDataType = DataType::INT;

		int idxCur = 0;
		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			if (vecInfos[i]==nullptr)
			{
				idxCur++;
				continue;
			}
			if (vecInfos[i]->eTextType == TextDrawType::TABLE)
			{
				if (idx == idxCur)
				{
					*vecInfos[i] = CTextDisp(&table);
					return (int)Error_ID::ERR_OK;
				}
				idxCur++;
			}
		}
		return (int)Error_ID::ERR_PROC_NG;
	}

	int CPage::SetPageTableDraw(int idx, TextDrawPara paraHorTitle, TextDrawPara paraVerTitle, TextDrawPara paraData, ObjDrawPara paraLine)
	{
		int nRet = (int)Error_ID::ERR_OK;

		int idxCur = 0;
		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			if (vecInfos[i]->eTextType == TextDrawType::TABLE)
			{
				if (idx == idxCur)
				{
					CTableDraw *pTable = (CTableDraw *)((CTextDisp *)vecInfos[i])->textDisp;
					pTable->strDataFont = paraData.strFont;
					pTable->strHorTitleFont = paraHorTitle.strFont;
					pTable->strVerTitleFont = paraVerTitle.strFont;

					pTable->nDataFontSize = paraData.nTextSize;
					pTable->nHorTitleFontSize = paraHorTitle.nTextSize;
					pTable->nVerTitleFontSize = paraVerTitle.nTextSize;

					pTable->colorDataFore = HSV::ScalarGC(paraData.nRgb[2], paraData.nRgb[1], paraData.nRgb[0]);
					pTable->colorHorTitleFore= HSV::ScalarGC(paraHorTitle.nRgb[2], paraHorTitle.nRgb[1], paraHorTitle.nRgb[0]);
					pTable->colorVerTitleFore= HSV::ScalarGC(paraVerTitle.nRgb[2], paraVerTitle.nRgb[1], paraVerTitle.nRgb[0]);

					pTable->nTableLineThick = paraLine.nThickness;
					pTable->colorTableLine= HSV::ScalarGC(paraLine.nRgb[2], paraLine.nRgb[1], paraLine.nRgb[0]);

					return (int)Error_ID::ERR_OK;
				}
				idxCur++;
			}
		}
		return nRet;
	}

	int CPage::SetPageAllTableDraw(TextDrawPara paraHorTitle, TextDrawPara paraVerTitle, TextDrawPara paraData, ObjDrawPara paraLine)
	{
		int nRet = (int)Error_ID::ERR_OK;

		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			if (vecInfos[i]->eTextType == TextDrawType::TABLE)
			{
				CTableDraw *pTable = (CTableDraw *)((CTextDisp *)vecInfos[i])->textDisp;
				pTable->strDataFont = paraData.strFont;
				pTable->strHorTitleFont = paraHorTitle.strFont;
				pTable->strVerTitleFont = paraVerTitle.strFont;

				pTable->nDataFontSize = paraData.nTextSize;
				pTable->nHorTitleFontSize = paraHorTitle.nTextSize;
				pTable->nVerTitleFontSize = paraVerTitle.nTextSize;

				pTable->colorDataFore = HSV::ScalarGC(paraData.nRgb[2], paraData.nRgb[1], paraData.nRgb[0]);
				pTable->colorHorTitleFore = HSV::ScalarGC(paraHorTitle.nRgb[2], paraHorTitle.nRgb[1], paraHorTitle.nRgb[0]);
				pTable->colorVerTitleFore = HSV::ScalarGC(paraVerTitle.nRgb[2], paraVerTitle.nRgb[1], paraVerTitle.nRgb[0]);

				pTable->nTableLineThick = paraLine.nThickness;
				pTable->colorTableLine = HSV::ScalarGC(paraLine.nRgb[2], paraLine.nRgb[1], paraLine.nRgb[0]);
			}
		}

		return nRet;
	}

	int CPage::AddChartInfo(ChartType chartType, int1D datas, str1D horStep, str1D verStep, string horAxisLabel, string verAxisLabel, std::string title, std::string note )
	{
		int2D data2D;
		data2D.push_back(datas);
		return AddChartInfo(chartType, data2D, horStep, verStep, horAxisLabel, verAxisLabel, title,note);
	}

	int CPage::AddChartInfo(ChartType chartType, int2D datas, str1D horStep, str1D verStep, string horAxisLabel, string verAxisLabel, std::string title, std::string note)
	{
		int nRet = (int)Error_ID::ERR_OK;
		//数据异常
		if (datas.size()<1)
		{
			return (int)Error_ID::ERR_PROC_NG;
		}
		else if (datas[0].size()<1)
		{
			return (int)Error_ID::ERR_PROC_NG;
		}

		switch (chartType)
		{
		case DebugView::ChartType::BAR_CHART:
		{
			CBarChartDraw chart;
			chart.eDataType = DataType::INT;
			chart.nData = datas[0];

			chart.strHorAxisLabel = horAxisLabel;
			chart.strVerAxisLabel = verAxisLabel;
			chart.strChartTitle = title;
			chart.strChartNote = note;
			CTextDisp *txtDisp = new CTextDisp(&chart);
			vecInfos.push_back(txtDisp);
			break; 
		}
		case DebugView::ChartType::LINE_CHART:
		{
			CLineChartDraw chart;
			chart.eDataType = DataType::INT;

			chart.nData = datas;
			chart.strHorAxisLabel = horAxisLabel;
			chart.strVerAxisLabel = verAxisLabel;
			chart.strChartTitle = title;
			chart.strChartNote = note;
			CTextDisp *txtDisp = new CTextDisp(&chart);
			vecInfos.push_back(txtDisp);
			break;
		}
		case DebugView::ChartType::PIE_CHART:
		{
			CPieChartDraw chart;
			chart.eDataType = DataType::INT;

			chart.nData = datas[0];
			chart.strChartTitle = title;
			chart.strChartNote = note;
			CTextDisp *txtDisp = new CTextDisp(&chart);
			vecInfos.push_back(txtDisp);
			break;
		}
		}
		
		return nRet;
	}

	int CPage::AddChartInfo(ChartType chartType, float1D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
	{
		float2D data2D;
		data2D.push_back(datas);
		return AddChartInfo(chartType, data2D, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
	}

	int CPage::AddChartInfo(ChartType chartType, float2D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
	{
		int nRet = (int)Error_ID::ERR_OK;
		//数据异常
		if (datas.size()<1)
		{
			return (int)Error_ID::ERR_PROC_NG;
		}
		else if (datas[0].size()<1)
		{
			return (int)Error_ID::ERR_PROC_NG;
		}

		switch (chartType)
		{
		case DebugView::ChartType::BAR_CHART:
		{
			CBarChartDraw chart;
			chart.eDataType = DataType::FLOAT;

			chart.fData = datas[0];
			chart.strHorAxisLabel = horAxisLabel;
			chart.strVerAxisLabel = verAxisLabel;
			chart.strChartTitle = title;
			chart.strChartNote = note;
			CTextDisp *txtDisp = new CTextDisp(&chart);
			vecInfos.push_back(txtDisp);
			break;
		}
		case DebugView::ChartType::LINE_CHART:
		{
			CLineChartDraw chart;
			chart.eDataType = DataType::FLOAT;

			chart.fData = datas;
			chart.strHorAxisLabel = horAxisLabel;
			chart.strVerAxisLabel = verAxisLabel;
			chart.strChartTitle = title;
			chart.strChartNote = note;
			CTextDisp *txtDisp = new CTextDisp(&chart);
			vecInfos.push_back(txtDisp);
			break;
		}
		case DebugView::ChartType::PIE_CHART:
		{
			CPieChartDraw chart;
			chart.eDataType = DataType::FLOAT;

			chart.fData = datas[0];
			chart.strChartTitle = title;
			chart.strChartNote = note;
			CTextDisp *txtDisp = new CTextDisp(&chart);
			vecInfos.push_back(txtDisp);
			break;
		}
		}

		return nRet;
	}

	int CPage::AddChartInfo(ChartType chartType, double1D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
	{
		double2D data2D;
		data2D.push_back(datas);
		return AddChartInfo(chartType, data2D,  horStep,  verStep,  horAxisLabel,  verAxisLabel, title, note);
	}

	int CPage::AddChartInfo(ChartType chartType, double2D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
	{
		int nRet = (int)Error_ID::ERR_OK;
		//数据异常
		if (datas.size()<1)
		{
			return (int)Error_ID::ERR_PROC_NG;
		}
		else if (datas[0].size()<1)
		{
			return (int)Error_ID::ERR_PROC_NG;
		}

		switch (chartType)
		{
		case DebugView::ChartType::BAR_CHART:
		{
			CBarChartDraw chart;
			chart.eDataType = DataType::DOUBLE;

			chart.dData = datas[0];
			chart.strHorAxisLabel = horAxisLabel;
			chart.strVerAxisLabel = verAxisLabel;
			chart.strChartTitle = title;
			chart.strChartNote = note;
			CTextDisp *txtDisp = new CTextDisp(&chart);
			vecInfos.push_back(txtDisp);
			break;
		}
		case DebugView::ChartType::LINE_CHART:
		{
			CLineChartDraw chart;
			chart.eDataType = DataType::DOUBLE;

			chart.dData = datas;
			chart.strHorAxisLabel = horAxisLabel;
			chart.strVerAxisLabel = verAxisLabel;
			chart.strChartTitle = title;
			chart.strChartNote = note;
			CTextDisp *txtDisp = new CTextDisp(&chart);
			vecInfos.push_back(txtDisp);
			break;
		}
		case DebugView::ChartType::PIE_CHART:
		{
			CPieChartDraw chart;
			chart.eDataType = DataType::DOUBLE;

			chart.dData = datas[0];
			chart.strChartTitle = title;
			chart.strChartNote = note;
			CTextDisp *txtDisp = new CTextDisp(&chart);
			vecInfos.push_back(txtDisp);
			break;
		}
		}

		return nRet;
	}

	int CPage::SetChartInfo(int idx, ChartType chartType, int1D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
	{
		int2D datas2d;
		datas2d.push_back(datas);
		return SetChartInfo(idx, chartType, datas2d, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
	}

	int CPage::SetChartInfo(int idx, ChartType chartType, int2D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
	{
		return 0;
	}

	int CPage::SetChartInfo(int idx, ChartType chartType, float1D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
	{
		float2D datas2d;
		datas2d.push_back(datas);
		return SetChartInfo(idx, chartType, datas2d,  horStep,  verStep,  horAxisLabel, verAxisLabel, title, note);
	}

	int CPage::SetChartInfo(int idx, ChartType chartType, float2D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
	{
		return 0;
	}

	int CPage::SetChartInfo(int idx, ChartType chartType, double1D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
	{
		double2D datas2d;
		datas2d.push_back(datas);
		return SetChartInfo(idx, chartType, datas2d, horStep, verStep, horAxisLabel, verAxisLabel, title, note);
	}

	int CPage::SetChartInfo(int idx, ChartType chartType, double2D datas, str1D horStep, str1D verStep, std::string horAxisLabel, std::string verAxisLabel, std::string title, std::string note)
	{
		return 0;
	}

	int CPage::SetPageChartDraw(int idx, ChartDrawPara chartDraw)
	{
		int nRet = (int)Error_ID::ERR_OK;

		int idxCur = 0;//图标序号
		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			if (vecInfos[i]->eTextType == TextDrawType::CHART)
			{
				if (idx == idxCur)
				{
					CChartDraw *chart = (CChartDraw *)((CTextDisp *)vecInfos[i])->textDisp;
					if (!chart->CheckDataValidity())
					{
						continue;//数据无效，不进行设置
					}
					chart->dspChartTitle.clrTxtFore = GetScalarColor(chartDraw.dspTitle.nRgb);
					chart->dspChartTitle.strFont = chartDraw.dspTitle.strFont;
					chart->dspChartTitle.nFontSize = chartDraw.dspTitle.nTextSize;
					chart->dspChartNote.clrTxtFore = GetScalarColor(chartDraw.dspNote.nRgb);
					chart->dspChartNote.strFont = chartDraw.dspNote.strFont;
					chart->dspChartNote.nFontSize = chartDraw.dspNote.nTextSize;

					switch (chart->eChartType)
					{
					case ChartType::BAR_CHART:
					{
						CBarChartDraw *barChart = (CBarChartDraw *)chart;
						barChart->dspHorAxisLine.thickObj = chartDraw.dspHorAxisLine.nThickness;
						barChart->dspHorAxisLine.clrObj = GetScalarColor(chartDraw.dspHorAxisLine.nRgb);
						barChart->dspHorAxisLabel.strFont = chartDraw.dspHorAxisLab.strFont;
						barChart->dspHorAxisLabel.nFontSize = chartDraw.dspHorAxisLab.nTextSize;
						barChart->dspHorAxisLabel.clrTxtFore = GetScalarColor(chartDraw.dspHorAxisLab.nRgb);
						barChart->dspVerAxisLine.thickObj = chartDraw.dspVerAxisLine.nThickness;
						barChart->dspVerAxisLine.clrObj = GetScalarColor(chartDraw.dspVerAxisLine.nRgb);
						barChart->dspVerAxisLabel.strFont = chartDraw.dspVerAxisLab.strFont;
						barChart->dspVerAxisLabel.nFontSize = chartDraw.dspVerAxisLab.nTextSize;
						barChart->dspVerAxisLabel.clrTxtFore = GetScalarColor(chartDraw.dspVerAxisLab.nRgb);
						if (chartDraw.dspData.size()>0)
						{
							barChart->dspData.clrObj = GetScalarColor(chartDraw.dspData[0].nRgb);
						}
						else
						{//默认蓝色
							barChart->dspData.clrObj = HSV::ScalarGC(255, 0, 0);
						}
						break;
					}
					case ChartType::LINE_CHART:
					{
						CLineChartDraw *lineChart = (CLineChartDraw *)chart;
						lineChart->dspHorAxisLine.thickObj = chartDraw.dspHorAxisLine.nThickness;
						lineChart->dspHorAxisLine.clrObj = GetScalarColor(chartDraw.dspHorAxisLine.nRgb);
						lineChart->dspHorAxisLabel.strFont = chartDraw.dspHorAxisLab.strFont;
						lineChart->dspHorAxisLabel.nFontSize = chartDraw.dspHorAxisLab.nTextSize;
						lineChart->dspHorAxisLabel.clrTxtFore = GetScalarColor(chartDraw.dspHorAxisLab.nRgb);
						lineChart->dspVerAxisLine.thickObj = chartDraw.dspVerAxisLine.nThickness;
						lineChart->dspVerAxisLine.clrObj = GetScalarColor(chartDraw.dspVerAxisLine.nRgb);
						lineChart->dspVerAxisLabel.strFont = chartDraw.dspVerAxisLab.strFont;
						lineChart->dspVerAxisLabel.nFontSize = chartDraw.dspVerAxisLab.nTextSize;
						lineChart->dspVerAxisLabel.clrTxtFore = GetScalarColor(chartDraw.dspVerAxisLab.nRgb);
						int rows, cols;
						lineChart->GetDataSize(rows, cols);
						lineChart->dspData.resize(rows);
						for (int j = 0; j < rows; j++)
						{
							if ((int)chartDraw.dspData.size() == rows)
							{
								lineChart->dspData[j].clrObj = GetScalarColor(chartDraw.dspData[j].nRgb);
							}
							else
							{
								lineChart->dspData[j].clrObj = HSV::ScalarGC(120 * (j+1) % 256, 60 * (j+1) % 256, 20 * (j+1)% 256);
							}
						}
						break;
					}
					case ChartType::PIE_CHART:
					{

						break;
					}
					default:
						break;
					}
					return (int)Error_ID::ERR_OK;
				}
				idxCur++;
			}
		}
		return nRet;
	}

	int CPage::SetPageAllChartDraw(ChartDrawPara chartDraw)
	{
		int nRet = (int)Error_ID::ERR_OK;

		int idxCur = 0;//图表序号
		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			if (vecInfos[i]->eTextType == TextDrawType::CHART)
			{
				CChartDraw *chart = (CChartDraw *)((CTextDisp *)vecInfos[i])->textDisp;
				if (!chart->CheckDataValidity())
				{
					continue;//数据无效，不进行设置
				}
				chart->dspChartTitle.clrTxtFore = GetScalarColor(chartDraw.dspTitle.nRgb);
				chart->dspChartTitle.strFont = chartDraw.dspTitle.strFont;
				chart->dspChartTitle.nFontSize = chartDraw.dspTitle.nTextSize;
				chart->dspChartNote.clrTxtFore = GetScalarColor(chartDraw.dspNote.nRgb);
				chart->dspChartNote.strFont = chartDraw.dspNote.strFont;
				chart->dspChartNote.nFontSize = chartDraw.dspNote.nTextSize;

				switch (chart->eChartType)
				{
				case ChartType::BAR_CHART:
				{
					CBarChartDraw *barChart = (CBarChartDraw *)chart;
					barChart->dspHorAxisLine.thickObj = chartDraw.dspHorAxisLine.nThickness;
					barChart->dspHorAxisLine.clrObj = GetScalarColor(chartDraw.dspHorAxisLine.nRgb);
					barChart->dspHorAxisLabel.strFont = chartDraw.dspHorAxisLab.strFont;
					barChart->dspHorAxisLabel.nFontSize = chartDraw.dspHorAxisLab.nTextSize;
					barChart->dspHorAxisLabel.clrTxtFore = GetScalarColor(chartDraw.dspHorAxisLab.nRgb);
					barChart->dspVerAxisLine.thickObj = chartDraw.dspVerAxisLine.nThickness;
					barChart->dspVerAxisLine.clrObj = GetScalarColor(chartDraw.dspVerAxisLine.nRgb);
					barChart->dspVerAxisLabel.strFont = chartDraw.dspVerAxisLab.strFont;
					barChart->dspVerAxisLabel.nFontSize = chartDraw.dspVerAxisLab.nTextSize;
					barChart->dspVerAxisLabel.clrTxtFore = GetScalarColor(chartDraw.dspVerAxisLab.nRgb);
					if (chartDraw.dspData.size()>0)
					{
						barChart->dspData.clrObj = GetScalarColor(chartDraw.dspData[0].nRgb);
					}
					else
					{//默认蓝色
						barChart->dspData.clrObj = HSV::ScalarGC(255, 0, 0);
					}
					break;
				}
				case ChartType::LINE_CHART:
				{
					CLineChartDraw *lineChart = (CLineChartDraw *)chart;
					lineChart->dspHorAxisLine.thickObj = chartDraw.dspHorAxisLine.nThickness;
					lineChart->dspHorAxisLine.clrObj = GetScalarColor(chartDraw.dspHorAxisLine.nRgb);
					lineChart->dspHorAxisLabel.strFont = chartDraw.dspHorAxisLab.strFont;
					lineChart->dspHorAxisLabel.nFontSize = chartDraw.dspHorAxisLab.nTextSize;
					lineChart->dspHorAxisLabel.clrTxtFore = GetScalarColor(chartDraw.dspHorAxisLab.nRgb);
					lineChart->dspVerAxisLine.thickObj = chartDraw.dspVerAxisLine.nThickness;
					lineChart->dspVerAxisLine.clrObj = GetScalarColor(chartDraw.dspVerAxisLine.nRgb);
					lineChart->dspVerAxisLabel.strFont = chartDraw.dspVerAxisLab.strFont;
					lineChart->dspVerAxisLabel.nFontSize = chartDraw.dspVerAxisLab.nTextSize;
					lineChart->dspVerAxisLabel.clrTxtFore = GetScalarColor(chartDraw.dspVerAxisLab.nRgb);
					int rows, cols;
					lineChart->GetDataSize(rows, cols);
					lineChart->dspData.resize(rows);
					for (int j = 0; j < rows; j++)
					{
						if ((int)chartDraw.dspData.size() == rows)
						{
							lineChart->dspData[j].clrObj = GetScalarColor(chartDraw.dspData[j].nRgb);
						}
						else
						{
							lineChart->dspData[j].clrObj = HSV::ScalarGC(80 * j % 256, 60 * j % 256, 20 * j % 256);
						}
					}
					break;
				}
				case ChartType::PIE_CHART:
				{

					break;
				}

				}
			}
		}
		return nRet;
	}

	
	int CPage::CalcImgWidth()
	{
		int nRet = (int)Error_ID::ERR_OK;
		if (ePageType != PageTypeEnum::PAGE_INFO)
		{
			return nRet;
		}
		/*
		1.因普通文本字体尺寸差异，故左对齐、右对齐文本可能不在一条直线上,计算时忽略错位差异,取最大值
		2.表格取水平标题或第一行数据的外宽
		3.图表取X轴外宽
		*/

		int lftWidMax = 0, rgtWidMax = 0;//左右字符宽度
		int tabWidMax = 0, chartWidMax = 0;//图标类宽度

		int txtWid, txtHgt,txtWid1, txtHgt1;//临时使用文本尺寸
		for (unsigned int idxTxt = 0; idxTxt < vecInfos.size(); idxTxt++)
		{
			CTextDisp *info = vecInfos[idxTxt];
			if (info == nullptr)continue;

			switch (info->eTextType)
			{
			case TextDrawType::TEXT:
			{
				HSV::TextDraw* txt = (HSV::TextDraw*)info->textDisp;
				GetTextSize(txt->strTxt_, txt->fn_, txt->nFontSize_, txtWid, txtHgt);
				if (info->eAlignType == AlignTypeEnum::ALIGN_LEFT && txtWid>lftWidMax)
				{//左对齐
					lftWidMax = txtWid;
				}
				else
				{//右对齐
					rgtWidMax = txtWid;
				}
				break;
			}
			case TextDrawType::TEXT_UNION:
			{
				HSV::TextUnionDraw* txt = (HSV::TextUnionDraw*)info->textDisp;
				GetTextSize(txt->lab_.strTxt_, txt->lab_.fn_, txt->lab_.nFontSize_, txtWid, txtHgt);
				GetTextSize(txt->sbj_.strTxt_, txt->sbj_.fn_, txt->sbj_.nFontSize_, txtWid1, txtHgt1);
				if (info->eAlignType == AlignTypeEnum::ALIGN_LEFT && (txtWid + txtWid1) > lftWidMax)
				{//左对齐
					lftWidMax = txtWid + txtWid1;
				}
				if (info->eAlignType == AlignTypeEnum::ALIGN_RIGHT && (txtWid + txtWid1) > rgtWidMax)
				{//右对齐
					rgtWidMax = txtWid + txtWid1;
				}

				break;
			}
			case TextDrawType::TEXT_TUPLE:
			{
				HSV::TextTupleDraw* txt = (HSV::TextTupleDraw*)info->textDisp;


				break;
			}
			case TextDrawType::TABLE:
			{
				CTableDraw* table = (CTableDraw*)info->textDisp;
				int tabWid, tabHgt;
				table->GetTableSize(tabWid, tabHgt, nPageWidth, nPageHeight);
				tabWidMax = (tabWid > tabWidMax) ? tabWid : tabWidMax;
				break;
			}
			case TextDrawType::CHART:
			{
				CChartDraw* chart = (CChartDraw*)info->textDisp;
				int chartWid = 0, chartHgt = 0;
				chart->GetChartSize(chartWid, chartHgt, nPageWidth, nPageHeight);
				chartWidMax = (chartWid > chartWidMax) ? chartWid : chartWidMax;
				break;
			}
			}
		}
		//计算最终页面宽度
		int nCurWidMax= ( lftWidMax + rgtWidMax );//左右字符宽度
		nCurWidMax = max(nCurWidMax, tabWidMax);
		nCurWidMax = max(nCurWidMax, chartWidMax);

		//最终的图像宽度(实际宽度+页面左右偏移值)
		nPageWidth = max(nPageWidth,nCurWidMax+ nOffsetXL+ nOffsetXR);

		return nRet;
	}

	int CPage::CalcTextPos()
	{
		int nRet = (int)Error_ID::ERR_OK;
		if (ePageType != PageTypeEnum::PAGE_INFO)
		{
			return nRet;
		}
		//当前坐标位置（文本左下坐标）
		int rowCurLft = nOffsetYL;
		int rowCurRgt = nOffSetYR;
		int colCurLft = 0;
		int txtWid = 0, txtHgt = 0;//当前文本尺寸
		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			CTextDisp *info = vecInfos[i];
			if (info == nullptr)continue;

			switch (info->eTextType)
			{
			case TextDrawType::TEXT:
			{
				HSV::TextDraw* txt = (HSV::TextDraw*)info->textDisp;
				if (txt->strTxt_=="")
				{
					continue;
				}
				GetTextSize(txt->strTxt_, txt->fn_, txt->nFontSize_, txtWid, txtHgt);
				// 增加行间距以避免文本重叠
				txtHgt += 15; // 增加15像素的行间距
				if (info->eAlignType == AlignTypeEnum::ALIGN_LEFT)
				{//左对齐
					rowCurLft += txtHgt;
					txt->ltBtmPt_.x = nOffsetXL;
					txt->ltBtmPt_.y = rowCurLft;
				}
				else
				{//右对齐
					rowCurRgt += txtHgt;
					txt->ltBtmPt_.x = nPageWidth - nOffsetXR - txtWid;
					txt->ltBtmPt_.y = rowCurRgt;
				}
			}
			break;
			case TextDrawType::TEXT_UNION:
			{
				HSV::TextUnionDraw* txt = (HSV::TextUnionDraw*)info->textDisp;
				int w1=0, h1=0, w2=0, h2=0;
				if (txt->lab_.strTxt_ != "")
				{
					GetTextSize(txt->lab_.strTxt_, txt->lab_.fn_, txt->lab_.nFontSize_, w1, h1);
				}
				if (txt->sbj_.strTxt_ != "")
				{
					GetTextSize(txt->sbj_.strTxt_, txt->sbj_.fn_, txt->sbj_.nFontSize_, w2, h2);
				}

				//std::cout << "lab:" << txt->lab_.strTxt_ << std::endl;
				//std::cout << "w1:" << w1 << std::endl;

				colCurLft = max(colCurLft, w1);
				txtWid = w1 + w2;
				txtHgt = max(h1, h2) + 3*i;
				// 增加行间距以避免文本重叠
				txtHgt += 15; // 增加15像素的行间距
				if (info->eAlignType == AlignTypeEnum::ALIGN_LEFT)
				{
					rowCurLft += txtHgt;
					txt->lab_.ltBtmPt_.x = nOffsetXL;
					txt->lab_.ltBtmPt_.y = rowCurLft;
					txt->sbj_.ltBtmPt_.x = nOffsetXL + w1;
					txt->sbj_.ltBtmPt_.y = rowCurLft;
				}
				else
				{
					rowCurRgt += txtHgt;
					txt->lab_.ltBtmPt_.x = nPageWidth - nOffsetXR - txtWid;
					txt->lab_.ltBtmPt_.y = rowCurRgt;
					txt->sbj_.ltBtmPt_.x = nPageWidth - nOffsetXR - w2;
					txt->sbj_.ltBtmPt_.y = rowCurRgt;
				}
			}
			break;
			case TextDrawType::TEXT_TUPLE:

				break;
			case TextDrawType::TABLE:
			{
				CTableDraw* table = (CTableDraw*)info->textDisp;
				int tabWid, tabHgt;
				table->ptTopLeft.x = nOffsetXL;
				table->ptTopLeft.y = max(rowCurLft, rowCurRgt);//表格需要独立成行
				table->GetTableSize(tabWid, tabHgt, nPageWidth, nPageHeight);
				rowCurLft = (int)table->ptTopLeft.y + tabHgt +10;//计算当前下顶点，增加10pix间隙
				rowCurRgt = (int)table->ptTopLeft.y + tabHgt +10;
			}
			break;
			case TextDrawType::CHART:
			{//图表计算
				CChartDraw* chart = (CChartDraw*)info->textDisp;
				int chartWid = 0, chartHgt = 0;
				chart->ptTopLeft.x = nOffsetXL;
				chart->ptTopLeft.y = max(rowCurLft, rowCurRgt);
				chart->GetChartSize(chartWid, chartHgt, nPageWidth, nPageHeight);
				rowCurLft = (int)chart->ptTopLeft.y + chartHgt + 10;//计算当前下顶点，增加10pix间隙
				rowCurRgt = (int)chart->ptTopLeft.y + chartHgt + 10;
			}
			break;
			default:
				break;
			}
		}
		
		//textUnionDraw 第二列对齐
		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			CTextDisp *info = vecInfos[i];
			if (info == nullptr)continue;
			if (info->eTextType== TextDrawType::TEXT_UNION 
				&& info->eAlignType==AlignTypeEnum::ALIGN_LEFT)
			{
				HSV::TextUnionDraw* txt = (HSV::TextUnionDraw*)info->textDisp;
				txt->sbj_.ltBtmPt_.x = colCurLft + nOffsetXL;
			}
		}
		//std::cout << "colCurLft:" << colCurLft << std::endl;
		nPageHeight = max(rowCurLft, rowCurRgt) + 15;

		return nRet;
	}

	int CPage::GenBgImage()
	{
		int nRet = (int)Error_ID::ERR_OK;
		if (ePageType == PageTypeEnum::PAGE_OBJECT)
		{
			if (vecImgIn.size()==0 || vecImgIn[0] == nullptr)
			{
				return (int)Error_ID::ERR_PAGE_IMG_NULL;
			}
			if (imgRst==nullptr)
			{
				imgRst = new GCImage();
			}
			*imgRst = *vecImgIn[0];
			if (imgRst->GetMatImg()->type()==CV_8UC1)
			{
				cv::cvtColor(*imgRst->GetMatImg(), *imgRst->GetMatImg(), cv::COLOR_GRAY2RGB);
			}
		}
		else
		{
			if (imgRst == nullptr)
			{
				imgRst = new GCImage();
			}
			cv::Scalar clr = Color_t(colorBackground);
			*imgRst->GetMatImg() = cv::Mat(nPageHeight, nPageWidth, CV_8UC3, clr);
		}

		if (bTileImages && vecImgIn.size()>1)
		{
			for (size_t i = 1; i < vecImgIn.size(); i++)
			{
				cv::Mat imgTemp = *imgRst->GetMatImg();
				if (vecImgIn[i]->GetMatImg()->type() == CV_8UC1)
				{
					cv::cvtColor(*vecImgIn[i]->GetMatImg(), *vecImgIn[i]->GetMatImg(), cv::COLOR_GRAY2RGB);
				}
				CombineImages(&imgTemp, vecImgIn[i]->GetMatImg(), imgRst->GetMatImg());
			}
		}
		return nRet;
	}

	int CPage::DrawObjects()
	{
		int nRet = (int)Error_ID::ERR_OK;
		for (size_t i = 0; i < vecObject.size(); i++)
		{
			DrawBaseObj(*imgRst, vecObject[i]);
		}

		return nRet;
	}

	int CPage::DrawInfos()
	{
		int nRet = (int)Error_ID::ERR_OK;

		for (size_t i = 0; i < vecInfos.size(); i++)
		{
			if (vecInfos[i]->eTextType== TextDrawType::TEXT)
			{
				DispText(*imgRst, *(HSV::TextDraw*)vecInfos[i]->textDisp);
			}
			else if (vecInfos[i]->eTextType == TextDrawType::TEXT_UNION)
			{
				DispText(*imgRst, *(HSV::TextUnionDraw*)vecInfos[i]->textDisp);
			}
			else if (vecInfos[i]->eTextType == TextDrawType::TEXT_TUPLE)
			{
				//DispText(*vecImgBg[0], *(HSV::TextTupleDraw*)vecInfos[i]->textDisp);
			}
			else if (vecInfos[i]->eTextType == TextDrawType::TABLE)
			{
				DrawTable(*imgRst, (CTableDraw*)vecInfos[i]->textDisp);
			}
			else if (vecInfos[i]->eTextType == TextDrawType::CHART)
			{
				DrawChart(*imgRst, (CChartDraw*)vecInfos[i]->textDisp, nPageWidth, nPageHeight);
			}
		}
		return nRet;
	}
	
}
