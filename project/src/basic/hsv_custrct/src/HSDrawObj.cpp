//#include "stdafx.h"
#include"HSDrawObj.h"
#include<assert.h>
#include<algorithm>
#include <vector>

namespace HSV
{
	template<typename _Tp> inline
		ColorGC_<_Tp>::ColorGC_()//BGR蓝绿红
	{
		this->val[0] = this->val[1] = this->val[2] = this->val[3] = 0;
	}

	template<typename _Tp> inline
		ColorGC_<_Tp>::ColorGC_(_Tp v0, _Tp v1, _Tp v2, _Tp v3)//BGR蓝绿红
	{
		this->val[0] = v0;
		this->val[1] = v1;
		this->val[2] = v2;
		this->val[3] = v3;
	}

	template<typename _Tp> inline
		ColorGC_<_Tp>::ColorGC_(_Tp v0)
	{
		this->val[0] = v0;
		this->val[1] = this->val[2] = this->val[3] = 0;
	}

	template<typename _Tp> inline
		ColorGC_<_Tp>::ColorGC_(const ColorGC_<_Tp>& s) {
		this->val[0] = s.val[0];
		this->val[1] = s.val[1];
		this->val[2] = s.val[2];
		this->val[3] = s.val[3];
	}

	template<typename _Tp> inline
		ColorGC_<_Tp>& ColorGC_<_Tp>::operator=(const ColorGC_<_Tp>& s) {
		this->val[0] = s.val[0];
		this->val[1] = s.val[1];
		this->val[2] = s.val[2];
		this->val[3] = s.val[3];
		return *this;
	}

	template<typename _Tp> inline
		bool ColorGC_<_Tp>::operator==(const ColorGC_<_Tp>& s) const {
		if (this->val[0] == s.val[0] &&
			this->val[1] == s.val[1] &&
			this->val[2] == s.val[2] &&
			this->val[3] == s.val[3])
			return true;
		else
			return false;

	}

	template<typename _Tp> inline
		ColorGC_<_Tp>& ColorGC_<_Tp>::operator()(_Tp v0, _Tp v1, _Tp v2, _Tp v3)
	{
		this->val[0] = v0;
		this->val[1] = v1;
		this->val[2] = v2;
		this->val[3] = v3;
		return (*this);
	}

	template<typename _Tp> inline
		ColorGC_<_Tp>& ColorGC_<_Tp>::operator()(_Tp v0)
	{
		this->val[0] = v0;
		this->val[1] = this->val[2] = this->val[3] = 0;
		return (*this);
	}

#pragma region 绘图对象基类
	void DrawObjBase::Init()
	{
		ScalarGC color_ = GetGCColor(GC_COL::GC_COL_RED);
		int thickness_ = 1;//线条的宽度，负值，如 FILLED、-1，意味着函数必须绘制一个填充的矩形
		int lineType_ = LINE_AA;//线段的类型。可以取值8， 4， 和CV_AA， 分别代表8邻接连接线，4邻接连接线和反锯齿连接线。默认值为8邻接。为了获得更好地效果可以选用CV_AA(采用了高斯滤波)。
		int shift_ = 0;//点坐标中的小数位数，一般取0，因为像素一般都是整型值。
		int nID_ = -1;// nID == -1;时 判断是否相等通过内部值来判断，如果 nID>=0时，则通过nID来判断。
	}
	DrawObjBase::DrawObjBase()
	{
		Init();
	}
	DrawObjBase::DrawObjBase(const DrawObjBase& para)
	{
		Init();
		CopyFrom(para);
	}
	DrawObjBase::~DrawObjBase()
	{

	}
	DrawObjBase& DrawObjBase::operator = (const DrawObjBase& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}
	bool DrawObjBase::equal(const DrawObjBase &obj) const
	{
		// 检查当前对象的指针是否为空
		if (this == nullptr) {
			// 处理空指针情况，例如可以返回 false 或抛出异常
			return false;
		}

		if (this->color_ == obj.color_		   &&
			this->thickness_ == obj.thickness_ &&
			this->lineType_ == obj.lineType_   &&
			this->shift_ == obj.shift_)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	bool DrawObjBase::operator==(const DrawObjBase & obj)
	{
		if (nID_ < 0)
			return equal(obj);
		else
			return (nID_ == obj.nID_);
	}

	void DrawObjBase::CopyFrom(const DrawObjBase& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}

	void DrawObjBase::CopyTo(DrawObjBase& para) const
	{
		if (this != &para)
		{
			para.color_ = color_;
			para.thickness_ = thickness_;
			para.lineType_ = lineType_;
			para.nID_ = nID_;
			para.shift_ = shift_;
		}
	}
	void DrawObjBase::SetDrawDat(GC_COL clrType, int thickness, int lineType, int shift)
	{
		this->SetColor(clrType);
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
	}
	void DrawObjBase::SetDrawDat(ScalarGC color, int thickness, int lineType, int shift)
	{
		color_ = color;
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
	}
	void DrawObjBase::SetColor(GC_COL clrType)
	{
		this->color_ = GetGCColor(clrType);
	}
#pragma endregion

	//初始化自定义颜色
	void InitGCColor()
	{
		if (bInited && g_GCColor1D.size() > 0)
			return;
		g_GCColor1D.clear();
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_RED, GC_RGB(255, 0, 0)));//红色 CV_RGB(255, 0, 0)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_GREEN, GC_RGB(0, 255, 0)));//绿色 
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_BLUE, GC_RGB(0, 0, 255)));//蓝色 CV_RGB(0, 0, 255)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_LIGHT_PINK, GC_RGB(255, 182, 193)));//浅粉红色 CV_RGB(255, 182, 193)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_PINK, GC_RGB(255, 192, 203)));//粉红色 CV_RGB(255, 192, 203)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_CRIMSON, GC_RGB(220, 20, 60)));//猩红色 CV_RGB(220, 20, 60)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_DEEP_PINK, GC_RGB(255, 20, 147)));//深粉色 CV_RGB(255, 20, 147)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_MAGENTA, GC_RGB(255, 0, 255)));//洋红色 CV_RGB(255, 0, 255)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_PURPLE, GC_RGB(128, 0, 128)));//紫色 CV_RGB(128, 0, 128)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_INDIGO, GC_RGB(75, 0, 130)));//靛青色 CV_RGB(75, 0, 130)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_LIGHT_SKY_BLUE, GC_RGB(135, 206, 250)));//淡蓝色 CV_RGB(135, 206, 250)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_SKY_BLUE, GC_RGB(135, 206, 235)));//天蓝色 CV_RGB(135, 206, 235)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_SLATE_BLUE, GC_RGB(106, 90, 205)));//板岩暗蓝灰色 CV_RGB(106, 90, 205)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_DARK_BLUE, GC_RGB(0, 0, 139)));//深蓝色 CV_RGB(0, 0, 139)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_NAVY_BLUE, GC_RGB(0, 0, 128)));//海军蓝 CV_RGB(0, 0, 128)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_LIGHT_CYAN, GC_RGB(225, 255, 255)));//淡青色 CV_RGB(225, 255, 255)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_CYAN, GC_RGB(0, 255, 255)));//青色 CV_RGB(0, 255, 255)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_PEA_GREEN, GC_RGB(199, 237, 204)));//豆沙绿色 CV_RGB(199, 237, 204)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_FOREST_GREEN, GC_RGB(34, 139, 34)));//森林绿(深绿色) CV_RGB(34, 139, 34)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_LIME_GREEN, GC_RGB(50, 205, 50)));//酸橙绿色 CV_RGB(50, 205, 50)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_LAWN_GREEN, GC_RGB(124, 252, 0)));//草坪绿色 CV_RGB(124, 252, 0)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_WHEAT, GC_RGB(245, 222, 179)));//小麦色 CV_RGB(245, 222, 179)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_YELLOW, GC_RGB(255, 255, 0)));//黄色 CV_RGB(255, 255, 0)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_OLIVE, GC_RGB(128, 128, 0)));//橄榄色 CV_RGB(128, 128, 0)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_ORANGE, GC_RGB(255, 165, 0)));//橙色 CV_RGB(255, 165, 0)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_CORAL, GC_RGB(255, 127, 80)));//珊瑚色 CV_RGB(255, 127, 80)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_TOMATO, GC_RGB(255, 99, 71)));//番茄色 CV_RGB(255, 99, 71)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_BROWN, GC_RGB(165, 42, 42)));//棕色 CV_RGB(165, 42, 42)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_BLACK, GC_RGB(0, 0, 0)));//黑色 CV_RGB(0, 0, 0)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_GRAY, GC_RGB(128, 128, 128)));//灰色 CV_RGB(128, 128, 128)
		g_GCColor1D.insert(std::pair<GC_COL, ScalarGC>(GC_COL::GC_COL_WHITE, GC_RGB(255, 255, 255)));//白色 CV_RGB(255, 255, 255)
		bInited = true;
	}

	bool IsColorInited()
	{
		if (!bInited || g_GCColor1D.size() == 0)
			return false;
		else
			return true;
	}

	//刷新当前自定义颜色
	void UpdateGCColor(GC_COL& clrType)
	{
		if (!IsColorInited())
			InitGCColor();//初始化自定义颜色
		if (clrType == GC_COL::GC_COL_RANDOM)//随机颜色
		{
			int m = (int)GC_COL::GC_COL_RED;
			int n = (int)GC_COL::GC_COL_BROWN;
			int randNum = rand() % (n - m + 1) + m;
			clrType = (GC_COL)randNum;
		}
		else if (clrType < GC_COL::GC_COL_DEFAULT || clrType >GC_COL::GC_COL_MAX)
		{//默认颜色
			clrType = GC_COL::GC_COL_RED;
		}
	}

	//获取转换后的opencv颜色
	ScalarGC GetGCColor(GC_COL clrType)
	{
		if (!IsColorInited())
			InitGCColor();//初始化自定义颜色
		ScalarGC color(ScalarGC(255, 0, 0));
		UpdateGCColor(clrType);
		for (auto it = g_GCColor1D.begin(); it != g_GCColor1D.end(); it++)
		{
			if (clrType == it->first)
			{
				color = it->second;
				break;
			}
		}
		return color;
	}

	void SwitchDrawObjType(HSV::DrawType drawType, DrawObjBase** pObj, bool bTransDat/* = false*/)
	{
#if 1 //@ChenW 临时这样写，后续再进一步封装
		ScalarGC color_ = ScalarGC(0, 0, 255);//BGR蓝绿红
		int thickness_ = 1;//线条的宽度，负值，如 FILLED、-1，意味着函数必须绘制一个填充的矩形
		int lineType_ = LINE_AA;//线段的类型。可以取值8， 4， 和CV_AA， 分别代表8邻接连接线，4邻接连接线和反锯齿连接线。默认值为8邻接。为了获得更好地效果可以选用CV_AA(采用了高斯滤波)。
		int shift_ = 0;//点坐标中的小数位数，一般取0，因为像素一般都是整型值。
		long long nID_ = -1;// nID == -1;时 判断是否相等通过内部值来判断，如果 nID>=0时，则通过nID来判断。
#endif // 1 //@ChenW 临时这样写，后续再进一步封装
		Point2f cnterPt(0.0F, 00.0F);
		int maxW = 0, maxH = 0;
		if (*pObj != NULL && drawType != (*pObj)->GetDrawType())
		{
			color_ = (*pObj)->color_; 
			thickness_ = (*pObj)->thickness_; 
			lineType_ = (*pObj)->lineType_; 
			shift_ = (*pObj)->shift_; 
			nID_ = (*pObj)->nID_; 
			if (bTransDat)
			{
				switch ((*pObj)->GetDrawType())
				{
				default:
					assert(0);
					break;
				case DRAW_RECT:
				{
					HSV::RectDraw* ptr = dynamic_cast<HSV::RectDraw*>(*pObj);
					cnterPt = ptr->Center();
					ptr->GetMaxSize(maxW, maxH);
				}
				break;
				case DRAW_RECT2:
				{
					HSV::Rect2Draw* ptr = dynamic_cast<HSV::Rect2Draw*>(*pObj);
					cnterPt = ptr->Center();
					ptr->GetMaxSize(maxW, maxH);
				}
				break;
				case DRAW_CIRCLE:
				{
					HSV::CircleDraw* ptr = dynamic_cast<HSV::CircleDraw*>(*pObj);
					cnterPt = ptr->Center();
					ptr->GetMaxSize(maxW, maxH);
				}
				break;
				case DRAW_ELLIPSE:
				{
					HSV::EllipseDraw* ptr = dynamic_cast<HSV::EllipseDraw*>(*pObj);
					cnterPt = ptr->Center();
					ptr->GetMaxSize(maxW, maxH);
				}
				break;
				case DRAW_POLYGON:
				{
					HSV::PolygonDraw* ptr = dynamic_cast<HSV::PolygonDraw*>(*pObj);
					cnterPt = ptr->Center();
					ptr->GetMaxSize(maxW, maxH);
				}
				break;
				case DRAW_LINE:
					break;
				case DRAW_CROSS:
					break;
				case DRAW_ARROW:
					break;
				case DRAW_TEXT:
					break;
				case DRAW_TEXT_UNION:
					break;
				case DRAW_TEXT_TUPLE:
					break;
				case DRAW_CONTOURS:
					break;
				case DRAW_POINT:
					break;
				}//switch
			}//if
			delete *pObj;
			*pObj = NULL;
		}
		if (*pObj == NULL)
		{
			switch (drawType)
			{
			default:
				assert(0);
				break;
			case DRAW_RECT:
				*pObj = new RectDraw;
				if (bTransDat)
				{
					HSV::RectDraw* ptr = dynamic_cast<HSV::RectDraw*>(*pObj);
					ptr->w_ = maxW;
					ptr->h_ = maxH;
					ptr->SetCnter(cnterPt);
				}
				break;
			case DRAW_RECT2:
				*pObj = new Rect2Draw;
				if (bTransDat)
				{
					HSV::Rect2Draw* ptr = dynamic_cast<HSV::Rect2Draw*>(*pObj);
					ptr->w_ = maxW;
					ptr->h_ = maxH;
					ptr->SetCnter(cnterPt);
				}
				break;
			case DRAW_CIRCLE:
				*pObj = new CircleDraw;
				if (bTransDat)
				{
					HSV::CircleDraw* ptr = dynamic_cast<HSV::CircleDraw*>(*pObj);
					ptr->radius_ = (std::max)(maxW, maxH);
					ptr->SetCnter(cnterPt);
				}
				break;
			case DRAW_ELLIPSE:
				*pObj = new EllipseDraw;
				if (bTransDat)
				{
					HSV::EllipseDraw* ptr = dynamic_cast<HSV::EllipseDraw*>(*pObj);
					ptr->longR_ = maxW;
					ptr->shortR_ = maxH;
					ptr->SetCnter(cnterPt);
				}
				break;
			case DRAW_POLYGON:
				*pObj = new PolygonDraw;
				if (bTransDat)
				{
					HSV::PolygonDraw* ptr = dynamic_cast<HSV::PolygonDraw*>(*pObj);
					ptr->SetCnter(cnterPt);
				}
				break;
			case DRAW_LINE:
				*pObj = new LineDraw;
				break;
			case DRAW_CROSS:
				*pObj = new CrossDraw;
				break;
			case DRAW_ARROW:
				*pObj = new ArrowDraw;
				break;
			case DRAW_TEXT:
				*pObj = new TextDraw;
				break;
			case DRAW_TEXT_UNION:
				*pObj = new TextUnionDraw;
				break;
			case DRAW_TEXT_TUPLE:
				*pObj = new TextTupleDraw;
				break;
			case DRAW_CONTOURS:
				*pObj = new ContoursDraw;
				break;
			case DRAW_POINT:
				*pObj = new PointDraw;
				break;
			}
			if (*pObj)
			{
				(*pObj)->color_ = color_;
				(*pObj)->thickness_ = thickness_;
				(*pObj)->lineType_ = lineType_;
				(*pObj)->shift_ = shift_;
				(*pObj)->nID_ = nID_;
			}
		}
	}

	void CopyDrawObjDat(const DrawObjBase* srcObj, DrawObjBase** dstObj)
	{
		if (srcObj == *dstObj)
			return;
		const HSV::DrawType drawType = srcObj->GetDrawType();
		SwitchDrawObjType(drawType, dstObj, false);
		//**dstObj = *srcObj;
		switch (drawType)
		{
		default:
			assert(0);
			break;
		case DRAW_RECT:
			*(RectDraw*)(*dstObj) = *(RectDraw*)srcObj;
			break;
		case DRAW_RECT2:
			*(Rect2Draw*)(*dstObj) = *(Rect2Draw*)srcObj;
			break;
		case DRAW_CIRCLE:
			*(CircleDraw*)(*dstObj) = *(CircleDraw*)srcObj;
			break;
		case DRAW_ELLIPSE:
			*(EllipseDraw*)(*dstObj) = *(EllipseDraw*)srcObj;
			break;
		case DRAW_POLYGON:
			*(PolygonDraw*)(*dstObj) = *(PolygonDraw*)srcObj;
			break;
		case DRAW_LINE:
			*(LineDraw*)(*dstObj) = *(LineDraw*)srcObj;
			break;
		case DRAW_CROSS:
			*(CrossDraw*)(*dstObj) = *(CrossDraw*)srcObj;
			break;
		case DRAW_ARROW:
			*(ArrowDraw*)(*dstObj) = *(ArrowDraw*)srcObj;
			break;
		case DRAW_TEXT:
			*(TextDraw*)(*dstObj) = *(TextDraw*)srcObj;
			break;
		case DRAW_TEXT_UNION:
			*(TextUnionDraw*)(*dstObj) = *(TextUnionDraw*)srcObj;
			break;
		case DRAW_TEXT_TUPLE:
			*(TextTupleDraw*)(*dstObj) = *(TextTupleDraw*)srcObj;
			break;
		case DRAW_CONTOURS:
			*(ContoursDraw*)(*dstObj) = *(ContoursDraw*)srcObj;
			break;
		case DRAW_POINT:
			*(PointDraw*)(*dstObj) = *(PointDraw*)srcObj;
			break;
		}
	}

	void CopyDrawObjDat(const std::vector<DrawObjBase*>& srcObj1D, std::vector<DrawObjBase*>& dstObj1D)
	{
		if (&srcObj1D == &dstObj1D)
			return;
		const unsigned int srcSize = (unsigned int)srcObj1D.size();
		const unsigned int dstSize = (unsigned int)dstObj1D.size();
		if (srcSize >= dstSize)
		{
			for (unsigned int i = 0; i < srcSize - dstSize; i++)
			{
				dstObj1D.push_back(NULL);
			}
		}
		else
		{
			for (unsigned int i = dstSize; i > srcSize; i--)
			{
				HSV::ClearDrawObj(&dstObj1D[i -1]);
				dstObj1D.pop_back();
			}
		}
		for (unsigned int i = 0; i < srcSize; i++)
		{
			if (srcObj1D[i])
				CopyDrawObjDat(srcObj1D[i], &dstObj1D[i]);
			else
				HSV::ClearDrawObj(&dstObj1D[i]);
		}
	}

	void CopyDrawObjRegDat_SameDrawType(const DrawObjBase* srcObj, DrawObjBase* dstObj)
	{
		switch (srcObj->GetDrawType())
		{
		default:
			assert(0);
			break;
		case DRAW_RECT:
			{
				HSV::Rect* pSrc = dynamic_cast<HSV::Rect*>((HSV::RectDraw*)srcObj);
				HSV::Rect* pDst = dynamic_cast<HSV::Rect*>((HSV::RectDraw*)dstObj);
				*pDst = *pSrc;
			}
			break;
		case DRAW_RECT2:
			{
				HSV::Rect2* pSrc = dynamic_cast<HSV::Rect2*>((HSV::Rect2Draw*)srcObj);
				HSV::Rect2* pDst = dynamic_cast<HSV::Rect2*>((HSV::Rect2Draw*)dstObj);
				*pDst = *pSrc;
			}
			break;
		case DRAW_CIRCLE:
			{
				HSV::Circle* pSrc = dynamic_cast<HSV::Circle*>((HSV::CircleDraw*)srcObj);
				HSV::Circle* pDst = dynamic_cast<HSV::Circle*>((HSV::CircleDraw*)dstObj);
				*pDst = *pSrc;
			}
			break;
		case DRAW_ELLIPSE:
			{
				HSV::Ellipse* pSrc = dynamic_cast<HSV::Ellipse*>((HSV::EllipseDraw*)srcObj);
				HSV::Ellipse* pDst = dynamic_cast<HSV::Ellipse*>((HSV::EllipseDraw*)dstObj);
				*pDst = *pSrc;
			}
			break;
		case DRAW_POLYGON:
			{
				HSV::Polygon* pSrc = dynamic_cast<HSV::Polygon*>((HSV::PolygonDraw*)srcObj);
				HSV::Polygon* pDst = dynamic_cast<HSV::Polygon*>((HSV::PolygonDraw*)dstObj);
				*pDst = *pSrc;
			}
			break;
		case DRAW_LINE:
			break;
		case DRAW_CROSS:
			break;
		case DRAW_ARROW:
			break;
		case DRAW_TEXT:
			break;
		case DRAW_TEXT_UNION:
			break;
		case DRAW_TEXT_TUPLE:
			break;
		case DRAW_CONTOURS:
			break;
		case DRAW_POINT:
			break;
		}
	}

	void GenNewDrawObj(const DrawObjBase* srcObj, const HSV::Point2f& pos, DrawObjBase** dstObj)
	{
		HSV::CopyDrawObjDat(srcObj, dstObj);
		HSV::HSRoiBase* pRoi = HSV::TransDrawObj2HSRoiObjPtr(*dstObj);
		pRoi->SetCnter(pos);
	}

	void ClearDrawObj(DrawObjBase** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}
	void ClearDrawObj(std::vector<DrawObjBase*> &obj1D)
	{
		for (auto& it : obj1D)
		{
			ClearDrawObj(&it);
		}
		obj1D.clear();
	}

	void ClearDrawObj(HSV::RectDraw** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearDrawObj(HSV::CircleDraw** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearDrawObj(HSV::Rect2Draw** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearDrawObj(HSV::EllipseDraw** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearDrawObj(HSV::PolygonDraw** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearDrawObj(HSV::LineDraw** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearDrawObj(HSV::ArrowDraw** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearDrawObj(HSV::CrossDraw** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearDrawObj(HSV::ContoursDraw** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearDrawObj(HSV::TextDraw** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearDrawObj(HSV::PointDraw** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void TransHSRoiObj2DrawObj(const HSRoiBase* srcObj, DrawObjBase** pDraw)
	{
		switch (srcObj->GetRegType())
		{
		default:

			break;
		case HSV::HSReg_RECT:
		{
			const HSV::Rect* ptrSrc = dynamic_cast<const HSV::Rect*>(srcObj);
			HSV::SwitchDrawObjType(HSV::DrawType::DRAW_RECT, pDraw, false);
			HSV::Rect* ptrDst = dynamic_cast<HSV::Rect*>((HSV::RectDraw*)(*pDraw));
			*ptrDst = *ptrSrc;
		}
		break;
		case HSV::HSReg_RECT2:
		{
			const HSV::Rect2* ptrSrc = dynamic_cast<const HSV::Rect2*>(srcObj);
			HSV::SwitchDrawObjType(HSV::DrawType::DRAW_RECT2, pDraw, false);
			HSV::Rect2* ptrDst = dynamic_cast<HSV::Rect2*>((HSV::Rect2Draw*)(*pDraw));
			*ptrDst = *ptrSrc;
		}
		break;
		case HSV::HSReg_CIRCLE:
		{
			const HSV::Circle* ptrSrc = dynamic_cast<const HSV::Circle*>(srcObj);
			HSV::SwitchDrawObjType(HSV::DrawType::DRAW_CIRCLE, pDraw, false);
			HSV::Circle* ptrDst = dynamic_cast<HSV::Circle*>((HSV::CircleDraw*)(*pDraw));
			*ptrDst = *ptrSrc;
		}
		break;
		case HSV::HSReg_ELLIPSE:
		{
			const HSV::Ellipse* ptrSrc = dynamic_cast<const HSV::Ellipse*>(srcObj);
			HSV::SwitchDrawObjType(HSV::DrawType::DRAW_ELLIPSE, pDraw, false);
			HSV::Ellipse* ptrDst = dynamic_cast<HSV::Ellipse*>((HSV::EllipseDraw*)(*pDraw));
			*ptrDst = *ptrSrc;
		}
		break;
		case HSV::HSReg_POLYGON:
		{
			const HSV::Polygon* ptrSrc = dynamic_cast<const HSV::Polygon*>(srcObj);
			HSV::SwitchDrawObjType(HSV::DrawType::DRAW_POLYGON, pDraw, false);
			HSV::Polygon* ptrDst = dynamic_cast<HSV::Polygon*>((HSV::PolygonDraw*)(*pDraw));
			*ptrDst = *ptrSrc;
		}
		break;
		}
	}

	void TransDrawObj2HSRoiObj(const DrawObjBase* srcObj, HSRoiBase** pRoi)
	{
		switch (srcObj->GetDrawType())
		{
		default:
			if (*pRoi != NULL)
			{
				delete *pRoi;
				*pRoi = NULL;
			}
			assert(*pRoi == NULL);
			break;
		case HSV::DRAW_RECT:
		{
			const HSV::Rect* ptrSrc = dynamic_cast<HSV::Rect*>((HSV::RectDraw*)(srcObj));
			HSV::SwitchHSRoiObjType(HSV::RegType::HSReg_RECT, pRoi, false);
			HSV::Rect* ptrDst = dynamic_cast<HSV::Rect*>(*pRoi);
			*ptrDst = *ptrSrc;
		}
		break;
		case HSV::DRAW_RECT2:
		{
			const HSV::Rect2* ptrSrc = dynamic_cast<HSV::Rect2*>((HSV::Rect2Draw*)(srcObj));
			HSV::SwitchHSRoiObjType(HSV::RegType::HSReg_RECT2, pRoi, false);
			HSV::Rect2* ptrDst = dynamic_cast<HSV::Rect2*>(*pRoi);
			*ptrDst = *ptrSrc;
		}
		break;
		case HSV::DRAW_CIRCLE:
		{
			const HSV::Circle* ptrSrc = dynamic_cast<HSV::Circle*>((HSV::CircleDraw*)(srcObj));
			HSV::SwitchHSRoiObjType(HSV::RegType::HSReg_CIRCLE, pRoi, false);
			HSV::Circle* ptrDst = dynamic_cast<HSV::Circle*>(*pRoi);
			*ptrDst = *ptrSrc;
		}
		break;
		case HSV::DRAW_ELLIPSE:
		{
			const HSV::Ellipse* ptrSrc = dynamic_cast<HSV::Ellipse*>((HSV::EllipseDraw*)(srcObj));
			HSV::SwitchHSRoiObjType(HSV::RegType::HSReg_ELLIPSE, pRoi, false);
			HSV::Ellipse* ptrDst = dynamic_cast<HSV::Ellipse*>(*pRoi);
			*ptrDst = *ptrSrc;
		}
		break;
		case HSV::DRAW_POLYGON:
		{
			const HSV::Polygon* ptrSrc = dynamic_cast<HSV::Polygon*>((HSV::PolygonDraw*)(srcObj));
			HSV::SwitchHSRoiObjType(HSV::RegType::HSReg_POLYGON, pRoi, false);
			HSV::Polygon* ptrDst = dynamic_cast<HSV::Polygon*>(*pRoi);
			*ptrDst = *ptrSrc;
		}
		break;
		}
	}

	HSRoiBase* TransDrawObj2HSRoiObjPtr(const DrawObjBase* srcObj)
	{
		HSRoiBase* pRoi = NULL;
		const HSV::DrawType drawType = srcObj->GetDrawType();
		switch (drawType)
		{
		default:
			break;
		case HSV::DRAW_RECT:
		{
			HSV::Rect* ptr = dynamic_cast<HSV::Rect*>((HSV::RectDraw*)srcObj);
			pRoi = (HSV::HSRoiBase*)ptr;
		}
		break;
		case HSV::DRAW_RECT2:
		{
			HSV::Rect2* ptr = dynamic_cast<HSV::Rect2*>((HSV::Rect2Draw*)srcObj);
			pRoi = (HSV::HSRoiBase*)ptr;
		}
		break;
		case HSV::DRAW_CIRCLE:
		{
			HSV::Circle* ptr = dynamic_cast<HSV::Circle*>((HSV::CircleDraw*)srcObj);
			pRoi = (HSV::HSRoiBase*)ptr;
		}
		break;
		case HSV::DRAW_ELLIPSE:
		{
			HSV::Ellipse* ptr = dynamic_cast<HSV::Ellipse*>((HSV::EllipseDraw*)srcObj);
			pRoi = (HSV::HSRoiBase*)ptr;
		}
		break;
		case HSV::DRAW_POLYGON:
		{
			HSV::Polygon* ptr = dynamic_cast<HSV::Polygon*>((HSV::PolygonDraw*)srcObj);
			pRoi = (HSV::HSRoiBase*)ptr;
		}
		break;
		}
		return pRoi;
	}

	void GenNewRoiObj(const DrawObjBase* srcObj, const HSV::Point2f& pos, HSRoiBase** pRoi)
	{
		TransDrawObj2HSRoiObj(srcObj, pRoi);
		(*pRoi)->SetCnter(pos);
	}

	HSV::Rect BoundingRect(const DrawObjBase* pObj)
	{
		HSV::Rect rect(0, 0, 0, 0);
		const HSV::HSRoiBase* pRoi = TransDrawObj2HSRoiObjPtr(pObj);
		if (pRoi)
		{
			rect = HSV::BoundingRect(pRoi);
		}
		return rect;
	}

	HSV::Rect BoundingRect(const std::vector<HSV::DrawObjBase*>& pObj1D)
	{
		HSV::Rect rect(0, 0, 0, 0);
		for (unsigned int i = 0; i < pObj1D.size(); i++)
		{
			const HSV::HSRoiBase* pRoi = TransDrawObj2HSRoiObjPtr(pObj1D[i]);
			if (i == 0)
			{
				rect = HSV::BoundingRect(pRoi);
			}
			else
			{
				HSV::Rect rectTmp = HSV::BoundingRect(pRoi);
				if (rectTmp.Area() > 0)
					rect = rect | rectTmp;
			}
		}
		return rect;
	}

	HSV::Rect2 BoundingRect2(const DrawObjBase* pObj)
	{
		HSV::Rect2 rect2(0.0F, 0.0F, 0, 0, 0.0F);
		const HSV::HSRoiBase* pRoi = TransDrawObj2HSRoiObjPtr(pObj);
		if (pRoi)
		{
			rect2 = HSV::BoundingRect2(pRoi);
		}
		return rect2;
	}
}
