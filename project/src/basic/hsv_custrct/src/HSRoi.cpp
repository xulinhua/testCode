//#include "stdafx.h"
#include"HSRoi.h"
#include<assert.h>
#include<algorithm>
#include <vector>
#include <cmath>

#define FLOAT_MIN_VAL 0.00001
#define PIXEL_MIN_VAL 1.00001
namespace HSV
{
	HSRoiBase::HSRoiBase()
	{
		Init();
	}
	HSRoiBase::HSRoiBase(const HSRoiBase& para)
	{
		Init();
		CopyFrom(para);
	}

	HSRoiBase::~HSRoiBase()
	{

	}

	HSRoiBase& HSRoiBase::operator = (const HSRoiBase& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	void HSRoiBase::Init()
	{

	}

	void HSRoiBase::CopyFrom(const HSRoiBase& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	void HSRoiBase::CopyTo(HSRoiBase& para) const
	{
		if (this != &para)
		{

		}
	}


	Rect::Rect()
	{
		Init();
	}
	Rect::Rect(const Rect& para)
	{
		Init();
		CopyFrom(para);
	}
	Rect::~Rect()
	{

	}
	Rect::Rect(int x, int y, int w, int h)
	{
		ltTopPt_.x = x;
		ltTopPt_.y = y;
		w_ = w;
		h_ = h;
	}
	Rect::Rect(Point pt, int w, int h)
	{
		ltTopPt_ = pt;
		w_ = w;
		h_ = h;
	}

	Rect& Rect::operator = (const Rect& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool Rect::equal(const HSRoiBase &obj) const
	{
		// 检查当前对象的指针是否为空
		if (this == nullptr) {
			// 处理空指针情况，例如可以返回 false 或抛出异常
			return false;
		}

		bool bEqual = false;
		if (this->GetRegType() == obj.GetRegType()/* && typeid(this) == typeid(&obj)*/)
		{
			if (const Rect *ptr = dynamic_cast<const Rect*>(&obj))
			{
				if (this->ltTopPt_ == ptr->ltTopPt_	&&
					this->w_ == ptr->w_	            &&
					this->h_ == ptr->h_)
				{
					bEqual = true;
				}
			}
		}
		return bEqual;
	}

	bool Rect::operator==(const Rect & obj) const
	{
		return equal(obj);
		/*if (this->ltTopPt_ == obj.ltTopPt_	&&
		this->w_ == obj.w_	            &&
		this->h_ == obj.h_)
		{
		return true;
		}
		else
		{
		return false;
		}*/
	}

	bool Rect::operator != (const Rect & obj) const
	{
		return !equal(obj);
	}

	Rect Rect::operator&(const Rect& rect)
	{
#if 0

		int x0_1 = ltTopPt_.x;
		int y0_1 = ltTopPt_.y;
		int x1_1 = ltTopPt_.x + w_;
		int y1_1 = ltTopPt_.y + h_;

		int x0_2 = rect.ltTopPt_.x;
		int y0_2 = rect.ltTopPt_.y;
		int x1_2 = rect.ltTopPt_.x + w_;
		int y1_2 = rect.ltTopPt_.y + h_;

		int overlap_x0 = (std::max)(x0_1, x0_2);
		int overlap_y0 = (std::max)(y0_1, y0_2);
		int overlap_x1 = (std::min)(x1_1, x1_2);
		int overlap_y1 = (std::min)(y1_1, y1_2);

		if ((overlap_x1 - overlap_x0 <= 0) | (overlap_y1 - overlap_y0 <= 0))
		{
			Rect inter_rect(0, 0, 0, 0);
			return inter_rect;
		}
		else
		{
			Rect inter_rect(overlap_x0, overlap_y0, abs(overlap_x1 - overlap_x0), (overlap_y1 - overlap_y0));
			return inter_rect;
		}
#else
		Rect inter_rect = *this;
		GetIntersectionRect(rect.ltTopPt_, rect.w_, rect.h_, inter_rect);//获取两个不带角度的矩形的交集矩形
		return inter_rect;
#endif // 0
	}

	//获取两个不带角度的矩形的交集矩形
	void GetIntersectionRect(Point tlPt, int sizeX, int sizeY, const Rect& srcRect, Rect& dstRct)
	{
		if (tlPt.x > srcRect.ltTopPt_.x)
			dstRct.ltTopPt_.x = tlPt.x;
		else
			dstRct.ltTopPt_.x = srcRect.ltTopPt_.x;
		if (tlPt.y > srcRect.ltTopPt_.y)
			dstRct.ltTopPt_.y = tlPt.y;
		else
			dstRct.ltTopPt_.y = srcRect.ltTopPt_.y;
		if (tlPt.x + sizeX - 1 > srcRect.br().x)
			dstRct.w_ = srcRect.br().x - dstRct.ltTopPt_.x + 1;
		else
			dstRct.w_ = tlPt.x + sizeX - dstRct.ltTopPt_.x;
		if (tlPt.y + sizeY - 1 > srcRect.br().y)
			dstRct.h_ = srcRect.br().y - dstRct.ltTopPt_.y + 1;
		else
			dstRct.h_ = tlPt.y + sizeY - dstRct.ltTopPt_.y;
	}

	//获取两个不带角度的矩形的交集矩形
	void GetIntersectionRect(Point tlPt, int sizeX, int sizeY, Rect& rect)
	{
		int rtX = rect.br().x;
		int btmY = rect.br().y;
		int ltX = rect.ltTopPt_.x;
		int topY = rect.ltTopPt_.y;
		if (tlPt.x > ltX)
			rect.ltTopPt_.x = tlPt.x;
		if (tlPt.y > topY)
			rect.ltTopPt_.y = tlPt.y;
		if (tlPt.x + sizeX - 1 < rtX)
			rect.w_ = tlPt.x + sizeX - rect.ltTopPt_.x;
		else
			rect.w_ = rtX - rect.ltTopPt_.x + 1;
		if (tlPt.y + sizeY - 1 < btmY)
			rect.h_ = tlPt.y + sizeY - rect.ltTopPt_.y;
		else
			rect.h_ = btmY - rect.ltTopPt_.y + 1;
		if (rect.w_ < 0)
			rect.w_ = 0;
		if (rect.h_ < 0)
			rect.h_ = 0;
	}

	//获取两个不带角度的矩形的并集矩形
	void GetUnionRect(Point tlPt, int sizeX, int sizeY, const Rect& rect2, Rect& dstRct)
	{
		GetUnionRect(Rect(tlPt, sizeX, sizeY), rect2, dstRct);//获取两个不带角度的矩形的并集矩形
	}

	//获取两个不带角度的矩形的并集矩形
	void GetUnionRect(Point tlPt, int sizeX, int sizeY, Rect& rect)
	{
		GetUnionRect(Rect(tlPt, sizeX, sizeY), rect, rect);//获取两个不带角度的矩形的并集矩形
	}

	//获取两个不带角度的矩形的并集矩形
	void GetUnionRect(const Rect& obj1, const Rect& obj2, Rect& dstRct)
	{
		int minX = (std::min)(obj1.ltTopPt_.x, obj2.ltTopPt_.x);
		int minY = (std::min)(obj1.ltTopPt_.y, obj2.ltTopPt_.y);
		int maxX = (std::max)(obj1.br().x, obj2.br().x);
		int maxY = (std::max)(obj1.br().y, obj2.br().y);
		dstRct.ltTopPt_.x = minX;
		dstRct.ltTopPt_.y = minY;
		dstRct.SetRtBtmPt(maxX, maxY);
	}

	//获取拆分后的矩形
	void GetSplitRect(const Rect& obj, bool bSplitHorDir, Rect rects[])
	{
		if (bSplitHorDir)//水平拆分
		{
			int halfW = obj.w_ / 2;
			rects[0].ltTopPt_ = obj.ltTopPt_;
			rects[0].w_ = halfW;
			rects[0].h_ = obj.h_;
			rects[1].ltTopPt_.x = rects[0].br().x + 1;
			rects[1].ltTopPt_.y = rects[0].ltTopPt_.y;
			rects[1].w_ = obj.w_ - halfW;
			rects[1].h_ = obj.h_;
		}
		else//垂直拆分
		{
			int halfH = obj.h_ / 2;
			rects[0].ltTopPt_ = obj.ltTopPt_;
			rects[0].w_ = obj.w_;
			rects[0].h_ = halfH;
			rects[1].ltTopPt_.x = rects[0].ltTopPt_.x;
			rects[1].ltTopPt_.y = rects[0].br().y + 1;
			rects[1].w_ = obj.w_;
			rects[1].h_ = obj.h_ - halfH;
		}
	}

	Rect Rect::operator|(const Rect& rect)
	{
		int x0_1 = ltTopPt_.x;
		int y0_1 = ltTopPt_.y;
		int x1_1 = ltTopPt_.x + w_;
		int y1_1 = ltTopPt_.y + h_;

		int x0_2 = rect.ltTopPt_.x;
		int y0_2 = rect.ltTopPt_.y;
		int x1_2 = rect.ltTopPt_.x + rect.w_;
		int y1_2 = rect.ltTopPt_.y + rect.h_;

		int union_x0 = (std::min)(x0_1, x0_2);
		int union_y0 = (std::min)(y0_1, y0_2);
		int union_x1 = (std::max)(x1_1, x1_2);
		int union_y1 = (std::max)(y1_1, y1_2);

		if ((union_x1 - union_x0 <= 0) | (union_y1 - union_y0 <= 0))
		{
			Rect union_rect(0, 0, 0, 0);
			return union_rect;
		}
		else
		{
			Rect union_rect(union_x0, union_y0, abs(union_x1 - union_x0), (union_y1 - union_y0));
			return union_rect;
		}

	}
	Rect Rect::operator+(Point point)
	{
		Rect rect(*this);
		rect.ltTopPt_.x += point.x;
		rect.ltTopPt_.y += point.y;
		return rect;
	}
	Rect Rect::operator-(Point point)
	{
		Rect rect(*this);
		rect.ltTopPt_.x -= point.x;
		rect.ltTopPt_.y -= point.y;
		return rect;
	}

	void Rect::Init()
	{
		ltTopPt_.x = 0;
		ltTopPt_.y = 0;
		w_ = 0;
		h_ = 0;
	}
	//从para拷贝数据
	void Rect::CopyFrom(const Rect& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void Rect::CopyTo(Rect& para) const
	{
		if (this != &para)
		{
			para.ltTopPt_ = ltTopPt_;
			para.w_ = w_;
			para.h_ = h_;
		}
	}

	RegType Rect::GetRegType() const
	{
		return RegType::HSReg_RECT;
	}

	void Rect::SetCnter(float cnterX, float cnterY)
	{
		Point2f oldCnter = Center();
		Point2f offset(cnterX - oldCnter.x, cnterY - oldCnter.y);
		if (0)
		{
			Point offPt;
			offPt.x = int((offset.x > 0.0) ? floor(offset.x + 0.5) : ceil(offset.x - 0.5));
			offPt.y = int((offset.y > 0.0) ? floor(offset.y + 0.5) : ceil(offset.y - 0.5));
			ltTopPt_.x += offPt.x;
			ltTopPt_.y += offPt.y;
		}
		else
		{
			ltTopPt_.x += (int)offset.x;
			ltTopPt_.y += (int)offset.y;
		}
	}

	void Rect::SetCnter(Point2f cnter)
	{
		SetCnter(cnter.x, cnter.y);
	}

	//更新当前区域的中心坐标偏移量
	void Rect::SetCnterOffset(int offsetX, int offsetY)
	{
		ltTopPt_.x += offsetX;
		ltTopPt_.y += offsetY;
	}

	void Rect::SetDegAng(float degAng)
	{

	}

	void Rect::SetDegAngOffset(float offsetDegAng)
	{

	}

	float Rect::GetDegAng() const
	{
		return 0.0F;
	}

	int Rect::Area() const//获取区域面积
	{
		return (w_ * h_);
	}

	//判断当前区域是否为空
	bool Rect::IsEmpty() const
	{
		return (0 == Area());
	}

	Point2f Rect::Center() const//获取区域中心坐标
	{
		return Point2f(ltTopPt_.x + (w_/* - 1*/) / 2.0F, ltTopPt_.y + (h_/* - 1*/) / 2.0F);//注意此处有精度损失@ChenW 07/22/2023, 14:53
	}

	//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
	void Rect::SetScaleRate(float scaleRate)
	{
		SetScaleRate(scaleRate, scaleRate);
	}

	void Rect::SetScaleRate(float scaleRateX, float scaleRateY)
	{
		assert(scaleRateX > 0.0F);
		assert(scaleRateY > 0.0F);
		if (scaleRateX < 0)
			scaleRateX = fabs(scaleRateX);
		if (scaleRateY < 0)
			scaleRateY = fabs(scaleRateY);
		Point2f cnterPt = Center();//获取矩形框中心
		float halfW = w_ / 2.0F;
		float halfH = h_ / 2.0F;
		halfW *= scaleRateX;
		halfH *= scaleRateY;
		float fVal = cnterPt.x - halfW;
		//刷新左上角坐标
		ltTopPt_.x = int((fVal > 0.0) ? floor(fVal + 0.5) : ceil(fVal - 0.5));

		fVal = cnterPt.y - halfH;
		ltTopPt_.y = int((fVal > 0.0) ? floor(fVal + 0.5) : ceil(fVal - 0.5));

		//刷新宽/高
		fVal = 2 * halfW;
		this->w_ = int((fVal > 0.0) ? floor(fVal + 0.5) : ceil(fVal - 0.5));
		fVal = 2 * halfH;
		this->h_ = int((fVal > 0.0) ? floor(fVal + 0.5) : ceil(fVal - 0.5));
	}

	//设置区域缩放比例(中心点不变)
	void Rect::ScaleWithFixedCenter(float scaleRate)
	{
		assert(scaleRate > 0.0F);
		if (scaleRate < 0)
			scaleRate = fabs(scaleRate);

		Point2f centerPt = Center(); // 获取矩形框中心

		// 计算新的半宽和半高
		float newHalfW = (w_ * scaleRate) / 2.0F;
		float newHalfH = (h_ * scaleRate) / 2.0F;

		// 根据新的半宽和半高以及中心点重新计算左上角坐标
		ltTopPt_.x = int((centerPt.x - newHalfW > 0.0) ? floor(centerPt.x - newHalfW + 0.5) : ceil(centerPt.x - newHalfW - 0.5));
		ltTopPt_.y = int((centerPt.y - newHalfH > 0.0) ? floor(centerPt.y - newHalfH + 0.5) : ceil(centerPt.y - newHalfH - 0.5));

		// 更新宽度和高度
		w_ = int(newHalfW * 2);
		h_ = int(newHalfH * 2);
	}


	void Rect::GetMaxSize(int& maxW, int& maxH) const
	{
		maxW = w_;
		maxH = h_;
	}

	//返回矩形4个顶点坐标
	void Rect::points(Point pts[]) const
	{
		//左下
		int p0_x = ltTopPt_.x;
		int p0_y = ltTopPt_.y + h_ - 1;
		//左上
		int p1_x = ltTopPt_.x;
		int p1_y = ltTopPt_.y;
		//右上
		int p2_x = ltTopPt_.x + w_ -1;
		int p2_y = ltTopPt_.y;
		//右下
		int p3_x = ltTopPt_.x + w_ - 1;
		int p3_y = ltTopPt_.y + h_ - 1;

		
		pts[0] = Point(p0_x, p0_y);
		pts[1] = Point(p1_x, p1_y);
		pts[2] = Point(p2_x, p2_y);
		pts[3] = Point(p3_x, p3_y);

	}
	void Rect::BoundingRect(int rct[]) const//获取区域外接矩形
	{
		rct[0] = ltTopPt_.x;
		rct[1] = ltTopPt_.y;
		rct[2] = w_;
		rct[3] = h_;//此处有问题@ChenW 
	}

	Rect Rect::BoundingRect() const//获取区域外接矩形	
	{
		return *this;
	}

	Point Rect::tl() const     //返回左上角顶点
	{
		return ltTopPt_;
	}
	Point Rect::br() const     //返回又上角顶点
	{
		return Point(ltTopPt_.x + w_ -1, ltTopPt_.y + h_ - 1);//注意此处的坐标点位置与halcon有一个像素偏差@ChenW 07/22/2023, 14:49
	}

	int Rect::rtX() const
	{
		return ltTopPt_.x + w_ -1;
	}

	int Rect::btmY() const
	{
		return ltTopPt_.y + h_ - 1;
	}

	void Rect::SetRtBtmPt(int x, int y)
	{
		if (ltTopPt_.x > x)
		{
			ltTopPt_.x = ltTopPt_.x + x;
			x = ltTopPt_.x - x;
			ltTopPt_.x = ltTopPt_.x - x;
		}
		if (ltTopPt_.y > y)
		{
			ltTopPt_.y = ltTopPt_.y + y;
			y = ltTopPt_.y - y;
			ltTopPt_.y = ltTopPt_.y - y;
		}
		w_ = abs(x - ltTopPt_.x);
		h_ = abs(y - ltTopPt_.y);
	}

	//获取区域边界角点坐标
	void Rect::GetRegCornerPt(std::vector<Point2i>& regCornerPt)const
	{
		if (regCornerPt.size() != 9)
			regCornerPt.resize(9);
		Point rtBtnPt;
		rtBtnPt.x = ltTopPt_.x + w_;
		rtBtnPt.y = ltTopPt_.y + h_;
		regCornerPt.at(0) = ltTopPt_;//第1点---左上角顶点
		Point2f tmpPt;
		tmpPt.x = (ltTopPt_.x + rtBtnPt.x) / 2.0F;
		regCornerPt.at(1).x = int((tmpPt.x > 0.0) ? floor(tmpPt.x + 0.5) : ceil(tmpPt.x - 0.5));
		regCornerPt.at(1).y = ltTopPt_.y;//第2点---上边缘中点
		regCornerPt.at(2).x = rtBtnPt.x;
		regCornerPt.at(2).y = ltTopPt_.y;//第3点---右上角顶点
		regCornerPt.at(3).x = rtBtnPt.x;//第4点---右边缘中点
		tmpPt.y = (ltTopPt_.y + rtBtnPt.y) / 2.0F;
		regCornerPt.at(3).y = int((tmpPt.y > 0.0) ? floor(tmpPt.y + 0.5) : ceil(tmpPt.y - 0.5));
		regCornerPt.at(4) = rtBtnPt;//第5点---右下角顶点
		tmpPt.x = (ltTopPt_.x + rtBtnPt.x) / 2.0F;
		regCornerPt.at(5).x = int((tmpPt.x > 0.0) ? floor(tmpPt.x + 0.5) : ceil(tmpPt.x - 0.5));
		regCornerPt.at(5).y = rtBtnPt.y;//第6点---下边缘中点
		regCornerPt.at(6).x = ltTopPt_.x;
		regCornerPt.at(6).y = rtBtnPt.y;//第7点---左下角顶点
		regCornerPt.at(7).x = ltTopPt_.x;//第8点---左边缘中点
		tmpPt.y = (ltTopPt_.y + rtBtnPt.y) / 2.0F;
		regCornerPt.at(7).y = int((tmpPt.y > 0.0) ? floor(tmpPt.y + 0.5) : ceil(tmpPt.y - 0.5));
		tmpPt.x = (ltTopPt_.x + rtBtnPt.x) / 2.0F;
		tmpPt.y = (ltTopPt_.y + rtBtnPt.y) / 2.0F;
		regCornerPt.at(8).x = int((tmpPt.x > 0.0) ? floor(tmpPt.x + 0.5) : ceil(tmpPt.x - 0.5));//第9点---矩形区域中心
		regCornerPt.at(8).y = int((tmpPt.y > 0.0) ? floor(tmpPt.y + 0.5) : ceil(tmpPt.y - 0.5));
	}
	//获取区域的X方向轴线	
	void Rect::GetRegXAxisLine(Point2f& startPt, Point2f& endPt)const
	{
		Point rtBtnPt;
		rtBtnPt.x = ltTopPt_.x + w_;
		rtBtnPt.y = ltTopPt_.y + h_;
		startPt.x = (float)ltTopPt_.x;
		startPt.y = (ltTopPt_.y + rtBtnPt.y) / 2.0F;
		endPt.x = (float)rtBtnPt.x;
		endPt.y = (ltTopPt_.y + rtBtnPt.y) / 2.0F;
	}

	//获取区域的X方向轴线
	void Rect::GetRegXAxisLine(Point2i& startPt, Point2i& endPt)const
	{
		Point2f tmpStartPt, tmpEndPt;
		Point rtBtnPt;
		rtBtnPt.x = ltTopPt_.x + w_;
		rtBtnPt.y = ltTopPt_.y + h_;
		startPt.x = ltTopPt_.x;
		tmpStartPt.y = (ltTopPt_.y + rtBtnPt.y) / 2.0F;
		startPt.y = int((tmpStartPt.y > 0.0) ? floor(tmpStartPt.y + 0.5) : ceil(tmpStartPt.y - 0.5));
		endPt.x = rtBtnPt.x;
		tmpEndPt.y = (ltTopPt_.y + rtBtnPt.y) / 2.0F;
		endPt.y = int((tmpEndPt.y > 0.0) ? floor(tmpEndPt.y + 0.5) : ceil(tmpEndPt.y - 0.5));
	}


	//是否包含某点
	bool Rect::Contain(int ptX, int ptY) const
	{
		bool bContain = true;
		Point rtBtnPt;
		rtBtnPt.x = ltTopPt_.x + w_;
		rtBtnPt.y = ltTopPt_.y + h_;
		if (ptX < ltTopPt_.x || ptX > rtBtnPt.x || ptY < ltTopPt_.y || ptY > rtBtnPt.y)
			bContain = false;
		return bContain;
	}

	//是否包含某点
	bool Rect::Contain(float ptX, float ptY) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		return Contain(tmpX, tmpY);
	}

	//是否包含某点
	bool Rect::Contain(double ptX, double ptY) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		return Contain(tmpX, tmpY);
	}

	//是否包含某点
	bool Rect::Contain(Point2i pt) const
	{
		return Contain(pt.x, pt.y);
	}

	//是否包含某矩形区域
	bool Rect::Contain(Point2i tmpLtTopPt, Point2i tmpRtBtnPt) const
	{
		return Contain(tmpLtTopPt.x, tmpLtTopPt.y) && Contain(tmpRtBtnPt.x, tmpRtBtnPt.y);
	}

	//是否包含某点
	bool Rect::Contain(Point2f pt) const
	{
		return Contain(pt.x, pt.y);
	}

	//是否靠近左边缘
	bool Rect::IsNearLeftEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearLeftEdge(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近左边缘
	bool Rect::IsNearLeftEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearLeftEdge(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近左边缘
	bool Rect::IsNearLeftEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		Point rtBtnPt;
		rtBtnPt.x = ltTopPt_.x + w_;
		rtBtnPt.y = ltTopPt_.y + h_;
		if (ptY >= ltTopPt_.y && ptY <= rtBtnPt.y)
		{
			if (bKeepInSide)
			{
				if (ptX >= ltTopPt_.x && ptX <= ltTopPt_.x + boundaryRange)
					bNear = true;
			}
			else
			{
				if (fabs(ptX - ltTopPt_.x) <= boundaryRange)
					bNear = true;
			}
		}
		return bNear;
	}

	//是否靠近中心点
	bool Rect::IsNearCenter(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		Point rtBtnPt;
		rtBtnPt.x = ltTopPt_.x + w_;
		rtBtnPt.y = ltTopPt_.y + h_;
		if (ptY >= ltTopPt_.y && ptY <= rtBtnPt.y)
		{
			if (bKeepInSide)
			{
				if (ptX >= Center().x - boundaryRange && ptX <= Center().x + boundaryRange &&
					ptY >= Center().y - boundaryRange && ptY <= Center().y + boundaryRange)
					bNear = true;
			}
			else
			{
				if (fabs(ptX - Center().x) <= boundaryRange && fabs(ptY - Center().y) <= boundaryRange)
					bNear = true;
			}
		}
		return bNear;
	}

	//是否靠近中心点
	bool Rect::IsNearCenter(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearCenter(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近中心点
	bool Rect::IsNearCenter(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearCenter(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近中心点
	bool Rect::IsNearCenter(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearCenter(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近中心点
	bool Rect::IsNearCenter(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearCenter(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近左边缘
	bool Rect::IsNearLeftEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearLeftEdge(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近左边缘
	bool Rect::IsNearLeftEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearLeftEdge(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近右边缘
	bool Rect::IsNearRightEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearRightEdge(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近右边缘
	bool Rect::IsNearRightEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearRightEdge(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近右边缘
	bool Rect::IsNearRightEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const
	{
		Point rtBtnPt;
		rtBtnPt.x = ltTopPt_.x + w_;
		rtBtnPt.y = ltTopPt_.y + h_;
		bool bNear = false;
		if (ptY >= ltTopPt_.y && ptY <= rtBtnPt.y)
		{
			if (bKeepInSide)
			{
				if (ptX <= rtBtnPt.x && ptX >= rtBtnPt.x - boundaryRange)
					bNear = true;
			}
			else
			{
				if (fabs(ptX - rtBtnPt.x) <= boundaryRange)
					bNear = true;
			}
		}
		return bNear;
	}

	//是否靠近右边缘
	bool Rect::IsNearRightEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearRightEdge(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近右边缘
	bool Rect::IsNearRightEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearRightEdge(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近上边缘
	bool Rect::IsNearTopEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearTopEdge(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近上边缘
	bool Rect::IsNearTopEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearTopEdge(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近上边缘
	bool Rect::IsNearTopEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		Point rtBtnPt;
		rtBtnPt.x = ltTopPt_.x + w_;
		rtBtnPt.y = ltTopPt_.y + h_;
		if (ptX >= ltTopPt_.x && ptX <= rtBtnPt.x)
		{
			if (bKeepInSide)
			{
				if (ptY >= ltTopPt_.y && ptY <= ltTopPt_.y + boundaryRange)
					bNear = true;
			}
			else
			{
				if (fabs(ptY - ltTopPt_.y) <= boundaryRange)
					bNear = true;
			}
		}
		return bNear;
	}

	//是否靠近上边缘
	bool Rect::IsNearTopEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearTopEdge(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近上边缘
	bool Rect::IsNearTopEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearTopEdge(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近下边缘
	bool Rect::IsNearBtmEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearBtmEdge(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近下边缘
	bool Rect::IsNearBtmEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearBtmEdge(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近下边缘
	bool Rect::IsNearBtmEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		Point rtBtnPt;
		rtBtnPt.x = ltTopPt_.x + w_;
		rtBtnPt.y = ltTopPt_.y + h_;
		if (ptX >= ltTopPt_.x && ptX <= rtBtnPt.x)
		{
			if (bKeepInSide)
			{
				if (ptY <= rtBtnPt.y && ptY >= rtBtnPt.y - boundaryRange)
					bNear = true;
			}
			else
			{
				if (fabs(ptY - rtBtnPt.y) <= boundaryRange)
					bNear = true;
			}
		}
		return bNear;
	}

	//是否靠近下边缘
	bool Rect::IsNearBtmEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearBtmEdge(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近下边缘
	bool Rect::IsNearBtmEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearBtmEdge(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近左上角
	bool Rect::IsNearLtTopEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(pt, boundaryRangeX, bKeepInSide) && IsNearTopEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左上角
	bool Rect::IsNearLtTopEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearTopEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左上角
	bool Rect::IsNearLtTopEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(pt, boundaryRangeX, bKeepInSide) && IsNearTopEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左上角
	bool Rect::IsNearLtTopEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearTopEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左上角
	bool Rect::IsNearLtTopEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearTopEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左下角
	bool Rect::IsNearLtBtmEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(pt, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左下角
	bool Rect::IsNearLtBtmEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左下角
	bool Rect::IsNearLtBtmEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(pt, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左下角
	bool Rect::IsNearLtBtmEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左下角
	bool Rect::IsNearLtBtmEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右上角
	bool Rect::IsNearRtTopEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(pt, boundaryRangeX, bKeepInSide) && IsNearTopEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右上角
	bool Rect::IsNearRtTopEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearTopEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右上角
	bool Rect::IsNearRtTopEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(pt, boundaryRangeX, bKeepInSide) && IsNearTopEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右上角
	bool Rect::IsNearRtTopEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearTopEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右上角
	bool Rect::IsNearRtTopEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearTopEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右下角
	bool Rect::IsNearRtBtmEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(pt, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右下角
	bool Rect::IsNearRtBtmEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右下角
	bool Rect::IsNearRtBtmEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(pt, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右下角
	bool Rect::IsNearRtBtmEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右下角
	bool Rect::IsNearRtBtmEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	Circle::Circle()
	{
		Init();
	}
	Circle::Circle(const Circle& para)
	{
		Init();
		CopyFrom(para);
	}
	Circle::~Circle()
	{

	}
	Circle::Circle(int x, int y, int r)
	{
		cnter_.x = x;
		cnter_.y = y;
		radius_ = r;
	}

	Circle::Circle(Point pt, int r)
	{
		cnter_ = pt;
		radius_ = r;
	}

	Circle& Circle::operator = (const Circle& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool Circle::equal(const HSRoiBase &obj) const
	{
		// 检查当前对象的指针是否为空
		if (this == nullptr) {
			// 处理空指针情况，例如可以返回 false 或抛出异常
			return false;
		}

		bool bEqual = false;
		if (this->GetRegType() == obj.GetRegType())
		{
			if (const Circle *ptr = dynamic_cast<const Circle*>(&obj))
			{
				if (this->cnter_ == ptr->cnter_	&&
					this->radius_ == ptr->radius_)
				{
					bEqual = true;
				}
			}
		}
		return bEqual;
	}

	bool Circle::operator==(const Circle & obj) const
	{
		return equal(obj);
		/*if (this->cnter_  == obj.cnter_	&&
		this->radius_ == obj.radius_)
		{
		return true;
		}
		else
		{
		return false;
		}*/
	}
	Circle Circle::operator+(Point point)
	{
		Circle circle(*this);
		circle.cnter_.x += point.x;
		circle.cnter_.y += point.y;
		return circle;
	}
	Circle Circle::operator-(Point point)
	{
		Circle circle(*this);
		circle.cnter_.x -= point.x;
		circle.cnter_.y -= point.y;
		return circle;
	}


	void Circle::Init()
	{
		cnter_.x = 0;
		cnter_.y = 0;
		radius_ = 0;
	}
	//从para拷贝数据
	void Circle::CopyFrom(const Circle& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void Circle::CopyTo(Circle& para) const
	{
		if (this != &para)
		{
			para.cnter_ = cnter_;
			para.radius_ = radius_;
		}
	}

	RegType Circle::GetRegType() const
	{
		return RegType::HSReg_CIRCLE;
	}

	void Circle::SetCnter(Point2f cnter)
	{
		cnter_.x = int((cnter.x > 0.0) ? floor(cnter.x + 0.5) : ceil(cnter.x - 0.5));
		cnter_.y = int((cnter.y > 0.0) ? floor(cnter.y + 0.5) : ceil(cnter.y - 0.5));
	}

	//更新当前区域的中心坐标偏移量
	void Circle::SetCnterOffset(int offsetX, int offsetY)
	{
		cnter_.x += offsetX;
		cnter_.y += offsetY;
	}

	void Circle::SetDegAng(float degAng)
	{

	}

	void Circle::SetDegAngOffset(float offsetDegAng)
	{
	
	}

	float Circle::GetDegAng() const
	{
		return 0.0F;
	}

	int Circle::Area() const//获取区域面积
	{
		assert(radius_ >= 0);
		if (radius_ > 0)
			return (int)(PI_HS * radius_ * radius_);
		else
			return 0;
	}

	//判断当前区域是否为空
	bool Circle::IsEmpty() const
	{
		return (0 == Area());
	}

	Point2f Circle::Center() const//获取区域中心坐标
	{
		return Point2f((float)cnter_.x, (float)cnter_.y);
	}

	//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
	void Circle::SetScaleRate(float scaleRate)
	{
		SetScaleRate(scaleRate, scaleRate);
	}

	void Circle::SetScaleRate(float scaleRateX, float scaleRateY)
	{
		assert(scaleRateX > 0);
		float r = radius_ * scaleRateX;
		radius_ = int((r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5));
	}

	void Circle::GetMaxSize(int& maxW, int& maxH) const
	{
		maxW = maxH = 2 * radius_;
	}

	//返回矩形4个顶点坐标
	void Circle::Points(Point pts[]) const
	{

	}
	void Circle::BoundingRect(int rct[]) const//获取区域外接矩形
	{
		if (radius_ != -1)
		{
			int rect_width = 2 * radius_;
			int rect_height = 2 * radius_;

			rct[0] = cnter_.x - radius_;
			rct[1] = cnter_.y - radius_;
			rct[2] = rect_width + 1;//统一使用宽高输出 @LiGJ 2025/03/21
			rct[3] = rect_height + 1;
		}
		else
		{
			rct[0] = cnter_.x;
			rct[1] = cnter_.y;
			rct[2] = 0;
			rct[3] = 0;
		}
	}

	//获取区域外接矩形	
	Rect Circle::BoundingRect() const
	{
		int rct[4] = { 0 };
		BoundingRect(rct);
		return Rect(rct[0], rct[1], rct[2], rct[3]);
	}

	//获取圆中心X坐标
	int Circle::GetCnterPtX() const
	{
		return cnter_.x;
	}
	//获取圆中心Y坐标
	int Circle::GetCnterPtY() const
	{
		return cnter_.y;
	}
	//获取圆的最小X坐标值
	int Circle::GetMinPtX() const
	{
		return cnter_.x - radius_;
	}
	//获取圆的最大X坐标值
	int Circle::GetMaxPtX() const
	{
		return cnter_.x + radius_;
	}
	//获取圆的最小Y坐标值
	int Circle::GetMinPtY() const
	{
		return cnter_.y - radius_;
	}
	//获取圆的最大Y坐标值
	int Circle::GetMaxPtY() const
	{
		return cnter_.y + radius_;
	}
	//获取圆半径
	int Circle::GetRadius() const
	{
		return radius_;
	}
	//获取圆直径
	int Circle::GetDiameter() const
	{
		return 2 * radius_;
	}
	void Circle::points(Point pts[]) const
	{
		pts[0] = Point(cnter_.x, cnter_.y - radius_);
		pts[1] = Point(cnter_.x, cnter_.y + radius_);
		pts[2] = Point(cnter_.x - radius_, cnter_.y);
		pts[3] = Point(cnter_.x + radius_, cnter_.y);
	}
	bool Circle::Contain(int ptX, int ptY) const
	{
		bool bContain = true;
		const double s = sqrt((ptX - cnter_.x) * (ptX - cnter_.x) + (ptY - cnter_.y) * (ptY - cnter_.y));
		const int distance = int((s > 0.0) ? floor(s + 0.5) : ceil(s - 0.5));
		if (distance > radius_)
			bContain = false;
		return bContain;
	}
	bool Circle::Contain(Point pt) const
	{
		return Contain(pt.x, pt.y);
	}
	void Circle::SetRadius(int r)
	{
		radius_ = r;
	}
	void Circle::SetCnterPt(Point cnter)
	{
		cnter_ = cnter;
	}
	
	//设置区域中心及缩放比例
	void Circle::SetScaleRateAndCnterPt(float scaleRate, Point newCnterPt)
	{
		SetScaleRate(scaleRate);
		SetCnterPt(newCnterPt);
	}
	//获取区域边界角点坐标
	void Circle::GetRegCornerPt(std::vector<Point2i>& regCornerPt)const
	{
		if (regCornerPt.size() != 5)
			regCornerPt.resize(5);
		regCornerPt.at(0).x = cnter_.x - radius_;//第1点---左边缘点
		regCornerPt.at(0).y = cnter_.y;
		regCornerPt.at(1).x = cnter_.x;//第2点---上边缘点
		regCornerPt.at(1).y = cnter_.y - radius_;
		regCornerPt.at(2).x = cnter_.x + radius_;//第3点---右边缘点
		regCornerPt.at(2).y = cnter_.y;
		regCornerPt.at(3).x = cnter_.x;//第4点---下边缘点
		regCornerPt.at(3).y = cnter_.y + radius_;
		regCornerPt.at(4) = cnter_;//第5点---圆心
	}
	//获取区域的X方向轴线	
	void Circle::GetRegXAxisLine(Point2f& startPt, Point2f& endPt)const
	{
		startPt.x = float(cnter_.x - radius_);
		startPt.y = float(cnter_.y);
		endPt.x = float(cnter_.x + radius_);
		endPt.y = float(cnter_.y);
	}

	//获取区域的X方向轴线
	void Circle::GetRegXAxisLine(Point2i& startPt, Point2i& endPt)const
	{
		startPt.x = cnter_.x - radius_;
		startPt.y = cnter_.y;
		endPt.x = cnter_.x + radius_;
		endPt.y = cnter_.y;
	}

	//是否包含某点
	bool Circle::Contain(float ptX, float ptY) const
	{
		bool bContain = true;
		const double s = sqrt((ptX - cnter_.x) * (ptX - cnter_.x) + (ptY - cnter_.y) * (ptY - cnter_.y));
		const int distance = int((s > 0.0) ? floor(s + 0.5) : ceil(s - 0.5));
		if (distance > radius_)
			bContain = false;
		return bContain;
	}

	//是否包含某点
	bool Circle::Contain(double ptX, double ptY) const
	{
		return Contain((float)ptX, (float)ptY);
	}

	//是否包含某点
	bool Circle::Contain(Point2i& pt) const
	{
		return Contain(pt.x, pt.y);
	}

	//是否包含某点
	bool Circle::Contain(Point2f& pt) const
	{
		return Contain(pt.x, pt.y);
	}

	//是否靠近中心点
	bool Circle::IsNearCenter(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		if (bKeepInSide)
		{
			if (ptX >= (cnter_.x - boundaryRange) && ptX <= (cnter_.x  + boundaryRange) &&
				ptY >= (cnter_.y - boundaryRange) && ptY <= (cnter_.y  + boundaryRange))
				bNear = true;
		}
		else
		{
			if (fabs(ptX - cnter_.x) <= boundaryRange && fabs(ptY - cnter_.y) <= boundaryRange)
				bNear = true;
		}
		return bNear;
	}

	//是否靠近中心点
	bool Circle::IsNearCenter(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearCenter(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近中心点
	bool Circle::IsNearCenter(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearCenter(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近中心点
	bool Circle::IsNearCenter(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearCenter(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近中心点
	bool Circle::IsNearCenter(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearCenter(tmpX, tmpY, tmpRange, bKeepInSide);
	}


	//是否靠近左边缘
	bool Circle::IsNearLeftEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearLeftEdge(pt.x, boundaryRange, bKeepInSide);
	}

	//是否靠近左边缘
	bool Circle::IsNearLeftEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearLeftEdge(pt.x, boundaryRange, bKeepInSide);
	}

	//是否靠近左边缘
	bool Circle::IsNearLeftEdge(int ptX, int boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		if (bKeepInSide)
		{
			if (ptX >= (cnter_.x - radius_) && ptX <= (cnter_.x - radius_ + boundaryRange))
				bNear = true;
		}
		else
		{
			if (fabs(ptX - cnter_.x + radius_) <= boundaryRange)
				bNear = true;
		}
		return bNear;
	}

	//是否靠近左边缘
	bool Circle::IsNearLeftEdge(float ptX, float boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearLeftEdge(tmpX, tmpRange, bKeepInSide);
	}

	//是否靠近左边缘
	bool Circle::IsNearLeftEdge(double ptX, double boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearLeftEdge(tmpX, tmpRange, bKeepInSide);
	}

	//是否靠近右边缘
	bool Circle::IsNearRightEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearRightEdge(pt.x, boundaryRange, bKeepInSide);
	}

	//是否靠近右边缘
	bool Circle::IsNearRightEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearRightEdge(pt.x, boundaryRange, bKeepInSide);
	}

	//是否靠近右边缘
	bool Circle::IsNearRightEdge(int ptX, int boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		if (bKeepInSide)
		{
			if (ptX <= (cnter_.x + radius_) && ptX >= (cnter_.x + radius_ - boundaryRange))
				bNear = true;
		}
		else
		{
			if (fabs(ptX - cnter_.x - radius_) <= boundaryRange)
				bNear = true;
		}
		return bNear;
	}

	//是否靠近右边缘
	bool Circle::IsNearRightEdge(float ptX, float boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearRightEdge(tmpX, tmpRange, bKeepInSide);
	}

	//是否靠近右边缘
	bool Circle::IsNearRightEdge(double ptX, double boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearRightEdge(tmpX, tmpRange, bKeepInSide);
	}

	//是否靠近上边缘
	bool Circle::IsNearTopEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearTopEdge(pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近上边缘
	bool Circle::IsNearTopEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearTopEdge(pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近上边缘
	bool Circle::IsNearTopEdge(int ptY, int boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		if (bKeepInSide)
		{
			if (ptY >= (cnter_.y - radius_) && ptY <= (cnter_.y - radius_ + boundaryRange))
				bNear = true;
		}
		else
		{
			if (fabs(ptY - cnter_.y + radius_) <= boundaryRange)
				bNear = true;
		}
		return bNear;
	}

	//是否靠近上边缘
	bool Circle::IsNearTopEdge(float ptY, float boundaryRange, bool bKeepInSide) const
	{
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearTopEdge(tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近上边缘
	bool Circle::IsNearTopEdge(double ptY, double boundaryRange, bool bKeepInSide) const
	{
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearTopEdge(tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近下边缘
	bool Circle::IsNearBtmEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearBtmEdge(pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近下边缘
	bool Circle::IsNearBtmEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearBtmEdge(pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近下边缘
	bool Circle::IsNearBtmEdge(int ptY, int boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		if (bKeepInSide)
		{
			if (ptY <= (cnter_.y + radius_) && ptY >= (cnter_.y + radius_ - boundaryRange))
				bNear = true;
		}
		else
		{
			if (fabs(ptY - cnter_.y - radius_) <= boundaryRange)
				bNear = true;
		}
		return bNear;
	}

	//是否靠近下边缘
	bool Circle::IsNearBtmEdge(float ptY, float boundaryRange, bool bKeepInSide) const
	{
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearBtmEdge(tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近下边缘
	bool Circle::IsNearBtmEdge(double ptY, double boundaryRange, bool bKeepInSide) const
	{
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearBtmEdge(tmpY, tmpRange, bKeepInSide);
	}

	//根据边缘终点坐标刷新圆形的半径
	void Circle::UpdateRadiusByEndPt(Point2i pt)
	{
		UpdateRadiusByEndPt(pt.x, pt.y);
	}

	//根据边缘终点坐标刷新圆形的半径
	void Circle::UpdateRadiusByEndPt(int ptX, int ptY)
	{
		double distance = sqrt((ptX - cnter_.x) * (ptX - cnter_.x) + (ptY - cnter_.y) * (ptY - cnter_.y));
		radius_ = int((distance > 0.0) ? floor(distance + 0.5) : ceil(distance - 0.5));
	}

	//根据边缘终点坐标刷新圆形的半径
	void Circle::UpdateRadiusByEndPt(Point2f pt)
	{
		UpdateRadiusByEndPt(pt.x, pt.y);
	}

	//根据边缘终点坐标刷新圆形的半径
	void Circle::UpdateRadiusByEndPt(float ptX, float ptY)
	{
		double distance = sqrt((ptX - cnter_.x) * (ptX - cnter_.x) + (ptY - cnter_.y) * (ptY - cnter_.y));
		radius_ = int((distance > 0.0) ? floor(distance + 0.5) : ceil(distance - 0.5));
	}

	//根据边缘终点坐标X刷新圆形的半径
	void Circle::UpdateRadiusByPtX(int ptX)
	{
		radius_ = abs(ptX - cnter_.x);
	}

	//根据边缘终点坐标X刷新圆形的半径
	void Circle::UpdateRadiusByPtX(float ptX)
	{
		float distance = fabs(ptX - cnter_.x);
		radius_ = int((distance > 0.0) ? floor(distance + 0.5) : ceil(distance - 0.5));
	}

	//根据边缘终点坐标X刷新圆形的半径
	void Circle::UpdateRadiusByPtX(double ptX)
	{
		UpdateRadiusByPtX((float)ptX);
	}

	//根据边缘终点坐标Y刷新圆形的半径	
	void Circle::UpdateRadiusByPtY(int ptY)
	{
		radius_ = abs(ptY - cnter_.y);
	}

	//根据边缘终点坐标Y刷新圆形的半径	
	void Circle::UpdateRadiusByPtY(float ptY)
	{
		float distance = fabs(ptY - cnter_.y);
		radius_ = int((distance > 0.0) ? floor(distance + 0.5) : ceil(distance - 0.5));
	}

	//根据边缘终点坐标Y刷新圆形的半径	
	void Circle::UpdateRadiusByPtY(double ptY)
	{
		UpdateRadiusByPtY((float)ptY);
	}



	Rect2::Rect2()
	{
		Init();
	}
	Rect2::Rect2(const Rect2& para)
	{
		Init();
		CopyFrom(para);
	}

	Rect2::Rect2(Point2f cnter, int w, int h, double degAng)
	{
		cnter_ = cnter;
		w_ = w;
		h_ = h;
		degAng_ = degAng;
	}

	Rect2::Rect2(float x, float y, int w, int h, double degAng)
	{
		cnter_.x = x;
		cnter_.y = y;
		w_ = w;
		h_ = h;
		degAng_ = degAng;
	}

	Rect2::~Rect2()
	{

	}

	//赋值操作符重载，拷贝功能
	Rect2& Rect2::operator = (const Rect2& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool Rect2::equal(const HSRoiBase &obj) const
	{
		// 检查当前对象的指针是否为空
		if (this == nullptr) {
			// 处理空指针情况，例如可以返回 false 或抛出异常
			return false;
		}

		bool bEqual = false;
		if (this->GetRegType() == obj.GetRegType()/* && typeid(this) == typeid(&obj)*/)
		{
			if (const Rect2 *ptr = dynamic_cast<const Rect2*>(&obj))
			{
				if (this->cnter_ == ptr->cnter_ &&
					this->w_ == ptr->w_	        &&
					this->h_ == ptr->h_	        &&
					this->degAng_ == ptr->degAng_)
				{
					bEqual = true;
				}
			}
		}
		return bEqual;
	}

	bool Rect2::operator==(const Rect2 & obj) const
	{
		return equal(obj);
	}

	Rect2 Rect2::operator-(Point point)
	{
		Rect2 rect2(*this);
		rect2.cnter_.x -= point.x;
		rect2.cnter_.y -= point.y;
		return rect2;
	}

	Rect2 Rect2::operator+(Point point)
	{
		Rect2 rect2(*this);
		rect2.cnter_.x += point.x;
		rect2.cnter_.y += point.y;
		return rect2;
	}

	void Rect2::Init()
	{
		cnter_.x = 0;
		cnter_.y = 0;
		w_ = 0;
		h_ = 0;
		degAng_ = 0;
	}

	//从para拷贝数据
	void Rect2::CopyFrom(const Rect2& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void Rect2::CopyTo(Rect2& para) const
	{
		if (this != &para)
		{
			para.w_ = w_;
			para.h_ = h_;
			para.degAng_ = degAng_;
			para.cnter_ = cnter_;
		}
	}

	RegType Rect2::GetRegType() const
	{
		return RegType::HSReg_RECT2;
	}

	void Rect2::SetCnter(Point2f cnter)
	{
		cnter_ = cnter;
	}

	//更新当前区域的中心坐标偏移量
	void Rect2::SetCnterOffset(int offsetX, int offsetY)
	{
		cnter_.x += (float)offsetX;
		cnter_.y += (float)offsetY;
	}

	void Rect2::SetDegAng(float degAng)
	{
		degAng_ = degAng;
	}

	void Rect2::SetDegAngOffset(float offsetDegAng)
	{
		degAng_ += offsetDegAng;
	}

	float Rect2::GetDegAng() const
	{
		return degAng_;
	}

	//获取区域面积
	int Rect2::Area() const
	{
		return (w_ * h_ + (w_ + h_ + 1));
	}

	//判断当前区域是否为空
	bool Rect2::IsEmpty() const
	{
		return (0 == Area() || w_ == 0 || h_ == 0);
	}

	//获取区域中心坐标	
	Point2f Rect2::Center() const
	{
		return cnter_;
	}

	//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
	void Rect2::SetScaleRate(float scaleRate)
	{
		SetScaleRate(scaleRate, scaleRate);
	}

	void Rect2::SetScaleRate(float scaleRateX, float scaleRateY)
	{
		SetScaleRateX(scaleRateX);
		SetScaleRateY(scaleRateY);
	}

	void Rect2::GetMaxSize(int& maxW, int& maxH) const
	{
		int rect[4] = { 0 };
		BoundingRect(rect);
		maxW = rect[2];
		maxH = rect[3];
	}


	//获取区域外接矩形
	void Rect2::BoundingRect(int rect[]) const
	{
		if (degAng_ == 0)
		{
			rect[0] = int(/*1 + */cnter_.x - w_ / 2.0);
			rect[1] = int(/*1 + */cnter_.y - h_ / 2.0);
			rect[2] = w_;
			rect[3] = h_;
		}
		else
		{
			/*Point rPoints[4];
			points(rPoints);
			int p0_x = rPoints[0].x;
			int p0_y = rPoints[0].y;
			int p1_x = rPoints[1].x;
			int p1_y = rPoints[1].y;
			int p2_x = rPoints[2].x;
			int p2_y = rPoints[2].y;
			int p3_x = rPoints[3].x;
			int p3_y = rPoints[3].y;
			int min_x = (std::min)({ p0_x, p1_x, p2_x, p3_x });
			int max_x = (std::max)({ p0_x, p1_x, p2_x, p3_x });
			int min_y = (std::min)({ p0_y, p1_y, p2_y, p3_y });
			int max_y = (std::max)({ p0_y, p1_y, p2_y, p3_y });
			int boundingRectWidth = std::abs(max_x - min_x);
			int boundingRectHeight = std::abs(max_y - min_y);
			rect[0] = min_x;
			rect[1] = min_y;
			rect[2] = boundingRectWidth;
			rect[3] = boundingRectHeight;*/
			////////////////////////////////////
			double len1, len2, ang, phi;
			ang = -degAng_;
			if (w_ > h_) {
				len1 = w_ / 2;
				len2 = h_ / 2;
				ang = ang > 90 ? ang - 180 : ang;
				ang = ang < -90 ? ang + 180 : ang;
				phi = -ang;
			}
			else {
				len1 = h_ / 2;
				len2 = w_ / 2;
				ang = ang < 0 ? ang + 180 : ang;
				phi = 90 - ang;
			}

			HSV::Point2f ptLT, ptLB, ptRT, ptRB;
			phi = phi*PI_HS / 180;
			if (phi >(-45 * PI_HS / 180) && phi < (45 * PI_HS / 180))
			{
				ptRB.y = cnter_.y - len1*sin(phi) + len2*cos(phi);//
				ptRB.x = cnter_.x + len1*cos(phi) + len2*sin(phi);
				ptLT.y = cnter_.y + len1*sin(phi) - len2*cos(phi);//
				ptLT.x = cnter_.x - len1*cos(phi) - len2*sin(phi);
				ptLB.y = cnter_.y + len1*sin(phi) + len2*cos(phi);//
				ptLB.x = cnter_.x - len1*cos(phi) + len2*sin(phi);
				ptRT.y = cnter_.y - len1*sin(phi) - len2*cos(phi);//
				ptRT.x = cnter_.x + len1*cos(phi) - len2*sin(phi);
			}
			else
			{
				if (phi < 0)
				{
					phi += 180 * PI_HS / 180;
				}
				ptRB.y = cnter_.y + len1*sin(phi) + len2 *cos(phi);//
				ptRB.x = cnter_.x - len1*cos(phi) + len2 *sin(phi);
				ptLT.y = cnter_.y - len1*sin(phi) - len2 *cos(phi);//
				ptLT.x = cnter_.x + len1*cos(phi) - len2 *sin(phi);
				ptLB.y = cnter_.y + len1*sin(phi) - len2 *cos(phi);//
				ptLB.x = cnter_.x - len1*cos(phi) - len2 *sin(phi);
				ptRT.y = cnter_.y - len1*sin(phi) + len2 *cos(phi);//
				ptRT.x = cnter_.x + len1*cos(phi) + len2 *sin(phi);
			}
			Point2f tlPt, brPt;

			std::vector<float> fVal1D(4);
			fVal1D = { ptLT.x, ptLB.x, ptRT.x, ptRB.x };
			auto it1 = std::min_element(fVal1D.begin(), fVal1D.end());
			tlPt.x = int((*it1 > 0) ? floor(*it1 + 0.5) : ceil(*it1 - 0.5));/*(int)*it1;*/
			it1 = std::max_element(fVal1D.begin(), fVal1D.end());
			brPt.x = int((*it1 > 0) ? floor(*it1 + 0.5) : ceil(*it1 - 0.5));/*(int)*it1;*/

			fVal1D = { ptLT.y, ptLB.y, ptRT.y, ptRB.y };
			it1 = std::min_element(fVal1D.begin(), fVal1D.end());
			tlPt.y = int((*it1 > 0) ? floor(*it1 + 0.5) : ceil(*it1 - 0.5));/*(int)*it1;*/
			it1 = std::max_element(fVal1D.begin(), fVal1D.end());
			brPt.y = int((*it1 > 0) ? floor(*it1 + 0.5) : ceil(*it1 - 0.5));/*(int)*it1;*/

			rect[0] = tlPt.x;
			rect[1] = tlPt.y;
			rect[2] = brPt.x - tlPt.x + 1;//统一使用宽高输出 @LiGJ 2025/03/21
			rect[3] = brPt.y - tlPt.y + 1;
		}
	}

	//获取区域外接矩形	
	Rect Rect2::BoundingRect() const
	{
		int rct[4] = { 0 };
		BoundingRect(rct);
		return Rect(rct[0], rct[1], rct[2], rct[3]);
	}

	void Rect2::GetCounterPoint(Circle *dash_points, int nCount)
	{

		Point pts[4];
		points(pts);

		double cos_val = cos(PI_HS * degAng_ / 180);
		double cos_val2 = cos(PI_HS * (90 - degAng_) / 180);
		double sin_val = sin(PI_HS * degAng_ / 180);
		double sin_val2 = sin(PI_HS * (90 - degAng_) / 180);
		double interval_x = (double)w_ / nCount;
		double interval_y = (double)h_ / nCount;


		for (int i = 0; i <= nCount; i++)//绘制矩形框上边沿虚线
		{
			*(dash_points + i) = Circle(pts[0].x + (int)(round)(i * interval_x * cos_val), pts[0].y - (int)(round)(i * interval_x * sin_val), 1);
		}
		for (int i = 0; i <= nCount; i++)//绘制矩形框下边沿虚线
		{
			*(dash_points + nCount + i) = Circle(pts[1].x + (int)(round)(i * interval_y * cos_val2), pts[1].y + (int)(round)(i * interval_y *sin_val2), 1);
		}
		for (int i = 0; i <= nCount; i++)//绘制矩形框左边沿虚线
		{
			*(dash_points + nCount * 2 + i) = Circle(pts[2].x - (int)(round)(i * interval_x * cos_val), pts[2].y + (int)(round)(i * interval_x * sin_val), 1);
		}
		for (int i = 0; i < nCount; i++)//绘制矩形框右边沿虚线
		{
			*(dash_points + nCount * 3 + i) = Circle(pts[3].x - (int)(round)(i * interval_y * cos_val2), pts[3].y - (int)(round)(i * interval_y * sin_val2), 1);
		}

	}
	
	Point Rect2::tl() const   //左上角顶点
	{
		int left = cnter_.x - w_ / 2;
		int top = cnter_.y - h_ / 2;
		if (degAng_ == 0)
		{
			return Point(left, top);
		}
		else
		{
			int newleft = (round)((left - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (top - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
			int newtop = (round)((left - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (top - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);
			return Point(newleft, newtop);
		}
	}
	Point Rect2::tr() const
	{
		int right = (int)(cnter_.x + w_ / 2);
		int top = (int)(cnter_.y - h_ / 2);

		if (degAng_ == 0)
		{
			return Point(right, top);
		}
		else
		{
			int newright = (int)(round)((right - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (top - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
			int newtop = (int)(round)((right - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (top - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);
			return Point(newright, newtop);
		}
	}
	Point Rect2::br() const   //右下角顶点
	{
		int right = (cnter_.x + w_ / 2 - 1);
		int bottom = (cnter_.y + h_ / 2 - 1);

		if (degAng_ == 0)
		{
			return Point(right, bottom);
		}
		else
		{
			//假设对图片上任意点(x, y)，绕一个坐标点(rx0, ry0)逆时针旋转a角度后的新的坐标设为(x0, y0)，有公式：
			//x0 = (x - rx0)*cos(a) - (y - ry0)*sin(a) + rx0;
			//y0 = (x - rx0)*sin(a) + (y - ry0)*cos(a) + ry0;
			int newright = (round)((right - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (bottom - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
			int newbottom = (round)((right - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (bottom - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);
			return Point(newright, newbottom);
		}
	}
	Point Rect2::bl() const
	{
		int left = (int)(cnter_.x - w_ / 2);
		int bottom = (int)(cnter_.y + h_ / 2 - 1);
		if (degAng_ == 0)
		{
			return Point(left, bottom);
		}
		else
		{
			//假设对图片上任意点(x, y)，绕一个坐标点(rx0, ry0)逆时针旋转a角度后的新的坐标设为(x0, y0)，有公式：
			//x0 = (x - rx0)*cos(a) - (y - ry0)*sin(a) + rx0;
			//y0 = (x - rx0)*sin(a) + (y - ry0)*cos(a) + ry0;
			int newleft = (int)(round)((left - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (bottom - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
			int newbottom = (int)(round)((left - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (bottom - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);
			return Point(newleft, newbottom);
		}
	}
	void Rect2::points(Point pts[]) const
	{
		//左下
		int p0_x = (cnter_.x - w_ / 2.0);
		int p0_y = (cnter_.y + h_ / 2.0);
		//左上
		int p1_x = (cnter_.x - w_ / 2.0);
		int p1_y = (cnter_.y - h_ / 2.0);
		//右上
		int p2_x = (cnter_.x + w_ / 2.0);
		int p2_y = (cnter_.y - h_ / 2.0);
		//右下
		int p3_x = (cnter_.x + w_ / 2.0);
		int p3_y = (cnter_.y + h_ / 2.0);

		if (degAng_ == 0)
		{
			pts[0] = Point(p0_x, p0_y);
			pts[1] = Point(p1_x, p1_y);
			pts[2] = Point(p2_x, p2_y);
			pts[3] = Point(p3_x, p3_y);
		}
		else
		{
			//假设对图片上任意点(x, y)，绕一个坐标点(rx0, ry0)逆时针旋转a角度后的新的坐标设为(x0, y0)，有公式：
			//x0 = (x - rx0)*cos(a) - (y - ry0)*sin(a) + rx0;
			//y0 = (x - rx0)*sin(a) + (y - ry0)*cos(a) + ry0;

			int newp0_x = (int)(round)((p0_x - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (p0_y - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
			int newp0_y = (int)(round)((p0_x - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (p0_y - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);

			int newp1_x = (int)(round)((p1_x - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (p1_y - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
			int newp1_y = (int)(round)((p1_x - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (p1_y - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);

			int newp2_x = (int)(round)((p2_x - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (p2_y - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
			int newp2_y = (int)(round)((p2_x - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (p2_y - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);

			int newp3_x = (int)(round)((p3_x - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (p3_y - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
			int newp3_y = (int)(round)((p3_x - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (p3_y - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);

			pts[0] = Point(newp0_x, newp0_y);
			pts[1] = Point(newp1_x, newp1_y);
			pts[2] = Point(newp2_x, newp2_y);
			pts[3] = Point(newp3_x, newp3_y);

		}
	}

	float Rect2::GetHalfW() const
	{
		return (w_ / 2.0F);
	}
	//获取矩形框半高
	float Rect2::GetHalfH() const
	{
		return (h_ / 2.0F);
	}

	//获取带角度矩形框最大长度
	float Rect2::GetMaxLenth() const
	{
		float lenth = float(sqrt(w_ * w_ + h_ * h_));
		return lenth;
	}
	//是否包含某点
	bool Rect2::Contain(int ptX, int ptY) const
	{
		return Contain((float)ptX, (float)ptY);
	}

	//是否包含某矩形区域
	bool Rect2::Contain(Point2i tmpLtTopPt, Point2i tmpRtBtnPt) const
	{
		return Contain(tmpLtTopPt.x, tmpLtTopPt.y) && Contain(tmpRtBtnPt.x, tmpRtBtnPt.y);
	}

	bool Rect2::Contain(float ptX, float ptY) const
	{
		bool bContain = true;
		float halfW = this->GetHalfW();
		float halfH = this->GetHalfH();
		if (fabs(degAng_) < 0.00001)
		{
			if ((float)ptX < cnter_.x - halfW || (float)ptX > cnter_.x + halfW || (float)ptY < cnter_.y - halfH || (float)ptY > cnter_.y + halfH)
				bContain = false;
		}
		else
		{
			Point ptArr[4];
			this->points(ptArr);//获取带角度矩形的四个角点坐标
			double l = ptArr[3].y - ptArr[0].y;
			double k = ptArr[3].x - ptArr[0].x;
			double s = sqrt(k * k + l * l);
			double sin = l / s;
			double cos = k / s;
			Point2f ltTopPt(0, 0), rtBtnPt(0, 0);
			ltTopPt.x = (float)(ptArr[1].x * cos + ptArr[1].y * sin);
			ltTopPt.y = (float)(-ptArr[1].x * sin + ptArr[1].y * cos);
			rtBtnPt.x = (float)(ptArr[3].x * cos + ptArr[2].y * sin);
			rtBtnPt.y = (float)(-ptArr[3].x * sin + ptArr[3].y * cos);
			Point2f newPt(0, 0);
			newPt.x = (float)(ptX * cos + ptY * sin);
			newPt.y = (float)(-ptX * sin + ptY * cos);
			if (newPt.x < ltTopPt.x || newPt.x > rtBtnPt.x || newPt.y < ltTopPt.y || newPt.y > rtBtnPt.y)
				bContain = false;
		}
		return bContain;
	}
	bool Rect2::Contain(Point& pt) const
	{
		return Contain(pt.x, pt.y);
	}
	bool Rect2::Contain(Point2f& pt) const
	{
		return Contain(pt.x, pt.y);
	}
	//设置参数
	void Rect2::SetShapeDat(Point2f cnter, int width, int height, double angle)
	{
		w_ = width;
		h_ = height;
		degAng_ = angle;
		cnter_ = cnter;
	}
	void Rect2::SetShapeDat(float x, float y, int width, int height, double angle)
	{
		w_ = width;
		h_ = height;
		degAng_ = angle;
		cnter_.x = x;
		cnter_.y = y;
	}
	void Rect2::SetWidth(int width)
	{
		w_ = width;
	}
	void Rect2::SetHeight(int height)
	{
		h_ = height;
	}
	void Rect2::SetDegAng(double angle)
	{
		degAng_ = angle;
	}
	//根据左上角坐标点和右上角坐标点刷新矩形倾斜角度（注意传入的点坐标顺序一定要是左上角点、右上角点）
	void Rect2::SetDegAngBy2Pt(Point ltTopPt, Point rtTopPt)
	{
		if (ltTopPt.x != rtTopPt.x)
		{
			double radAng = -atan2(1.0 * (rtTopPt.y - ltTopPt.y), 1.0 * (rtTopPt.x - ltTopPt.x));
			degAng_ = float(180 * radAng / PI_HS);
		}
		else
			degAng_ = (ltTopPt.y < rtTopPt.y) ? 90.0F : -90.0F;//此时为垂直轴线（逆时针为正，顺时针为负）
	}
	//更新当前带角度矩形的中心坐标X
	void Rect2::SetCnterPtX(float ptX)
	{
		cnter_.x = ptX;
	}
	void Rect2::SetCnterPtY(float ptY)
	{
		cnter_.y = ptY;
	}
	void Rect2::SetCnterPt(float ptX, float ptY)
	{
		SetCnterPtX(ptX);
		SetCnterPtY(ptY);
	}
	void Rect2::SetCnterPt(Point2f newCnterPt)
	{
		SetCnterPt(newCnterPt.x, newCnterPt.y);
	}
	//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
	void Rect2::SetScaleRateX(float scaleRate)
	{
		assert(scaleRate > 0.0F);
		if (scaleRate < 0)
			scaleRate = fabs(scaleRate);
		float fVal = this->w_ * scaleRate;
		this->w_ = int((fVal > 0.0F) ? floor(fVal + 0.5F) : ceil(fVal - 0.5F));
	}

	void Rect2::SetScaleRateY(float scaleRate)
	{
		assert(scaleRate > 0.0F);
		if (scaleRate < 0)
			scaleRate = fabs(scaleRate);
		float fVal = this->h_ * scaleRate;
		this->h_ = int((fVal > 0.0F) ? floor(fVal + 0.5F) : ceil(fVal - 0.5F));
	}

	//设置区域中心及缩放比例
	void Rect2::SetScaleRateAndCnterPt(float scaleRate, Point2f newCnterPt)
	{
		SetScaleRate(scaleRate);
		cnter_ = newCnterPt;
	}

	//获取区域边界角点坐标
	void Rect2::GetRegCornerPt(std::vector<Point2i>& regCornerPt)const
	{
		if (regCornerPt.size() != 9)
			regCornerPt.resize(9);
		Point2i ptArr[4];
		points(ptArr);//获取带角度矩形的四个角点坐标
		regCornerPt.at(0) = ptArr[1];//第1点---左上角顶点
		Point2f tmpPt;
		tmpPt.x = (ptArr[1].x + ptArr[2].x) / 2.0F;
		tmpPt.y = (ptArr[1].y + ptArr[2].y) / 2.0F;
		regCornerPt.at(1).x = int((tmpPt.x > 0.0) ? floor(tmpPt.x + 0.5) : ceil(tmpPt.x - 0.5));//第2点---上边缘中点
		regCornerPt.at(1).y = int((tmpPt.y > 0.0) ? floor(tmpPt.y + 0.5) : ceil(tmpPt.y - 0.5));
		regCornerPt.at(2) = ptArr[2];//第3点---右上角顶点
		tmpPt.x = (ptArr[3].x + ptArr[2].x) / 2.0F;
		tmpPt.y = (ptArr[3].y + ptArr[2].y) / 2.0F;
		regCornerPt.at(3).x = int((tmpPt.x > 0.0) ? floor(tmpPt.x + 0.5) : ceil(tmpPt.x - 0.5));//第4点---右边缘中点
		regCornerPt.at(3).y = int((tmpPt.y > 0.0) ? floor(tmpPt.y + 0.5) : ceil(tmpPt.y - 0.5));
		regCornerPt.at(4) = ptArr[3];//第5点---右下角顶点
		tmpPt.x = (ptArr[0].x + ptArr[3].x) / 2.0F;
		tmpPt.y = (ptArr[0].y + ptArr[3].y) / 2.0F;
		regCornerPt.at(5).x = int((tmpPt.x > 0.0) ? floor(tmpPt.x + 0.5) : ceil(tmpPt.x - 0.5));//第6点---下边缘中点
		regCornerPt.at(5).y = int((tmpPt.y > 0.0) ? floor(tmpPt.y + 0.5) : ceil(tmpPt.y - 0.5));
		regCornerPt.at(6) = ptArr[0];//第7点---左下角顶点
		tmpPt.x = (ptArr[1].x + ptArr[0].x) / 2.0F;
		tmpPt.y = (ptArr[1].y + ptArr[0].y) / 2.0F;
		regCornerPt.at(7).x = int((tmpPt.x > 0.0) ? floor(tmpPt.x + 0.5) : ceil(tmpPt.x - 0.5));//第8点---左边缘中点
		regCornerPt.at(7).y = int((tmpPt.y > 0.0) ? floor(tmpPt.y + 0.5) : ceil(tmpPt.y - 0.5));
		regCornerPt.at(8).x = int((cnter_.x > 0.0) ? floor(cnter_.x + 0.5) : ceil(cnter_.x - 0.5));//第9点---矩形区域中心
		regCornerPt.at(8).y = int((cnter_.y > 0.0) ? floor(cnter_.y + 0.5) : ceil(cnter_.y - 0.5));
	}
	//获取区域的X方向轴线	
	void Rect2::GetRegXAxisLine(Point2f& startPt, Point2f& endPt)const
	{
		Point2i ptArr[4];
		points(ptArr);//获取带角度矩形的四个角点坐标
		startPt.x = (ptArr[1].x + ptArr[0].x) / 2.0F;//左边缘中点
		startPt.y = (ptArr[1].y + ptArr[0].y) / 2.0F;
		endPt.x = (ptArr[3].x + ptArr[2].x) / 2.0F;//右边缘中点
		endPt.y = (ptArr[3].y + ptArr[2].y) / 2.0F;
	}

	//获取区域的X方向轴线
	void Rect2::GetRegXAxisLine(Point2i& startPt, Point2i& endPt)const
	{
		Point2f tmpStartPt, tmpEndPt;
		GetRegXAxisLine(tmpStartPt, tmpEndPt);
		startPt.x = int((tmpStartPt.x > 0.0) ? floor(tmpStartPt.x + 0.5) : ceil(tmpStartPt.x - 0.5));//左边缘中点
		startPt.y = int((tmpStartPt.y > 0.0) ? floor(tmpStartPt.y + 0.5) : ceil(tmpStartPt.y - 0.5));
		endPt.x = int((tmpEndPt.x > 0.0) ? floor(tmpEndPt.x + 0.5) : ceil(tmpEndPt.x - 0.5));//右边缘中点
		endPt.y = int((tmpEndPt.y > 0.0) ? floor(tmpEndPt.y + 0.5) : ceil(tmpEndPt.y - 0.5));
	}

	//是否靠近中心点
	bool Rect2::IsNearCenter(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		Point ltTopPt_;
		ltTopPt_.x = cnter_.x - w_ / 2;
		ltTopPt_.y = cnter_.y - h_ / 2;
		Point rtBtnPt;
		rtBtnPt.x = cnter_.x + w_ / 2;
		rtBtnPt.y = cnter_.y + h_ / 2;
		if (ptY >= ltTopPt_.y && ptY <= rtBtnPt.y)
		{
			if (bKeepInSide)
			{
				if (ptX >= Center().x - boundaryRange && ptX <= Center().x + boundaryRange &&
					ptY >= Center().y - boundaryRange && ptY <= Center().y + boundaryRange)
					bNear = true;
			}
			else
			{
				if (fabs(ptX - Center().x) <= boundaryRange && fabs(ptY - Center().y) <= boundaryRange)
					bNear = true;
			}
		}
		return bNear;
	}

	//是否靠近中心点
	bool Rect2::IsNearCenter(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearCenter(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近中心点
	bool Rect2::IsNearCenter(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearCenter(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近中心点
	bool Rect2::IsNearCenter(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearCenter(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近中心点
	bool Rect2::IsNearCenter(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const
	{
		const int tmpX = int((ptX > 0.0) ? floor(ptX + 0.5) : ceil(ptX - 0.5));
		const int tmpY = int((ptY > 0.0) ? floor(ptY + 0.5) : ceil(ptY - 0.5));
		const int tmpRange = int((boundaryRange > 0.0) ? floor(boundaryRange + 0.5) : ceil(boundaryRange - 0.5));
		return IsNearCenter(tmpX, tmpY, tmpRange, bKeepInSide);
	}

	//是否靠近左边缘
	bool Rect2::IsNearLeftEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearLeftEdge((float)pt.x, (float)pt.y, (float)boundaryRange, bKeepInSide);
	}

	//是否靠近左边缘
	bool Rect2::IsNearLeftEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearLeftEdge(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近左边缘
	bool Rect2::IsNearLeftEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearLeftEdge((float)ptX, (float)ptY, (float)boundaryRange, bKeepInSide);
	}

	//是否靠近左边缘
	bool Rect2::IsNearLeftEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		Point2i ptArr[4];
		points(ptArr);//获取带角度矩形的四个角点坐标，左上角--ptArr[0] 左下角--ptArr[3]
		Point2f footPt = GetFootPt((float)ptArr[0].x, (float)ptArr[0].y, (float)ptArr[1].x, (float)ptArr[1].y, ptX, ptY);//计算某一点到线段的垂足点坐标
		double distance = sqrt((footPt.x - ptX) * (footPt.x - ptX) + (footPt.y - ptY) * (footPt.y - ptY));
		if (distance <= fabs(boundaryRange))
		{
			if (bKeepInSide)//若当前点必须在区域内才算靠近左边缘
			{
				Point2f pt_footPt(footPt.x - ptX, footPt.y - ptY);//从当前点指向垂足点的向量
				Point2f leftEdgeCnterPt((ptArr[0].x + ptArr[1].x) / 2.0F, (ptArr[0].y + ptArr[1].y) / 2.0F);
				Point2f cnterPt_leftEdgeCnterPt(leftEdgeCnterPt.x - cnter_.x, leftEdgeCnterPt.y - cnter_.y);//从区域中心点指向左边缘中心点的向量
				if (pt_footPt.x * cnterPt_leftEdgeCnterPt.x + pt_footPt.y * cnterPt_leftEdgeCnterPt.y > 0)//两向量同向，表明当前点在区域内；反向，则表明当前点在区域外
					bNear = true;
			}
			else
			{
				Point2f ltTopPt_footPt(footPt.x - ptArr[1].x, footPt.y - ptArr[1].y);//从区域的左上角点指向垂足点的向量
				Point2f ltBtmPt_footPt(footPt.x - ptArr[0].x, footPt.y - ptArr[0].y);//从区域的左下角点指向垂足点的向量
				if (ltTopPt_footPt.x * ltBtmPt_footPt.x + ltTopPt_footPt.y * ltBtmPt_footPt.y < 0)
				{//两向量不同向，说明垂足点在左边缘线上，若超出了左边缘线的范围，则此时两个向量应该是同向的
					bNear = true;
				}
			}
		}
		return bNear;
	}

	//是否靠近左边缘
	bool Rect2::IsNearLeftEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const
	{
		return IsNearLeftEdge((float)ptX, (float)ptY, (float)boundaryRange, bKeepInSide);
	}

	//是否靠近右边缘
	bool Rect2::IsNearRightEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearRightEdge((float)pt.x, (float)pt.y, (float)boundaryRange, bKeepInSide);
	}

	//是否靠近右边缘
	bool Rect2::IsNearRightEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearRightEdge(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近右边缘
	bool Rect2::IsNearRightEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearRightEdge((float)ptX, (float)ptY, (float)boundaryRange, bKeepInSide);
	}

	//是否靠近右边缘
	bool Rect2::IsNearRightEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		Point2i ptArr[4];
		points(ptArr);//获取带角度矩形的四个角点坐标，右上角--ptArr[1] 右下角--ptArr[2]
		Point2f footPt = GetFootPt((float)ptArr[3].x, (float)ptArr[3].y, (float)ptArr[2].x, (float)ptArr[2].y, ptX, ptY);//计算某一点到线段的垂足点坐标
		double distance = sqrt((footPt.x - ptX) * (footPt.x - ptX) + (footPt.y - ptY) * (footPt.y - ptY));
		if (distance <= fabs(boundaryRange))
		{
			if (bKeepInSide)//若当前点必须在区域内才算靠近右边缘
			{
				Point2f pt_footPt(footPt.x - ptX, footPt.y - ptY);//从当前点指向垂足点的向量
				Point2f rightEdgeCnterPt((ptArr[3].x + ptArr[2].x) / 2.0F, (ptArr[3].y + ptArr[2].y) / 2.0F);
				Point2f cnterPt_rightEdgeCnterPt(rightEdgeCnterPt.x - cnter_.x, rightEdgeCnterPt.y - cnter_.y);//从区域中心点指向右边缘中心点的向量
				if (pt_footPt.x * cnterPt_rightEdgeCnterPt.x + pt_footPt.y * cnterPt_rightEdgeCnterPt.y > 0)//两向量同向，表明当前点在区域内；反向，则表明当前点在区域外
					bNear = true;
			}
			else
			{
				Point2f rtTopPt_footPt(footPt.x - ptArr[2].x, footPt.y - ptArr[2].y);//从区域的右上角点指向垂足点的向量
				Point2f rtBtmPt_footPt(footPt.x - ptArr[3].x, footPt.y - ptArr[3].y);//从区域的右下角点指向垂足点的向量
				if (rtTopPt_footPt.x * rtBtmPt_footPt.x + rtTopPt_footPt.y * rtBtmPt_footPt.y < 0)
				{//两向量不同向，说明垂足点在右边缘线上，若超出了右边缘线的范围，则此时两个向量应该是同向的
					bNear = true;
				}
			}
		}
		return bNear;
	}

	//是否靠近右边缘
	bool Rect2::IsNearRightEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const
	{
		return IsNearRightEdge((float)ptX, (float)ptY, (float)boundaryRange, bKeepInSide);
	}

	//是否靠近上边缘
	bool Rect2::IsNearTopEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearTopEdge((float)pt.x, (float)pt.y, (float)boundaryRange, bKeepInSide);
	}

	//是否靠近上边缘
	bool Rect2::IsNearTopEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearTopEdge(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近上边缘
	bool Rect2::IsNearTopEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearTopEdge((float)ptX, (float)ptY, (float)boundaryRange, bKeepInSide);
	}

	//是否靠近上边缘
	bool Rect2::IsNearTopEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		Point2i ptArr[4];
		points(ptArr);//获取带角度矩形的四个角点坐标，左下角--ptArr[0] 左上角--ptArr[1]
		Point2f footPt = GetFootPt((float)ptArr[2].x, (float)ptArr[2].y, (float)ptArr[1].x, (float)ptArr[1].y, ptX, ptY);//计算某一点到线段的垂足点坐标
		double distance = sqrt((footPt.x - ptX) * (footPt.x - ptX) + (footPt.y - ptY) * (footPt.y - ptY));
		if (distance <= fabs(boundaryRange))
		{
			if (bKeepInSide)//若当前点必须在区域内才算靠近上边缘
			{
				Point2f pt_footPt(footPt.x - ptX, footPt.y - ptY);//从当前点指向垂足点的向量
				Point2f topEdgeCnterPt((ptArr[2].x + ptArr[1].x) / 2.0F, (ptArr[2].y + ptArr[1].y) / 2.0F);
				Point2f cnterPt_topEdgeCnterPt(topEdgeCnterPt.x - cnter_.x, topEdgeCnterPt.y - cnter_.y);//从区域中心点指向上边缘中心点的向量
				if (pt_footPt.x * cnterPt_topEdgeCnterPt.x + pt_footPt.y * cnterPt_topEdgeCnterPt.y > 0)//两向量同向，表明当前点在区域内；反向，则表明当前点在区域外
					bNear = true;
			}
			else
			{
				Point2f ltTopPt_footPt(footPt.x - ptArr[1].x, footPt.y - ptArr[1].y);//从区域的左上角点指向垂足点的向量
				Point2f rtTopPt_footPt(footPt.x - ptArr[2].x, footPt.y - ptArr[2].y);//从区域的右上角点指向垂足点的向量
				if (ltTopPt_footPt.x * rtTopPt_footPt.x + ltTopPt_footPt.y * rtTopPt_footPt.y < 0)
				{//两向量不同向，说明垂足点在上边缘线上，若超出了上边缘线的范围，则此时两个向量应该是同向的
					bNear = true;
				}
			}
		}
		return bNear;
	}

	//是否靠近上边缘
	bool Rect2::IsNearTopEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const
	{
		return IsNearTopEdge((float)ptX, (float)ptY, (float)boundaryRange, bKeepInSide);
	}

	//是否靠近下边缘
	bool Rect2::IsNearBtmEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearBtmEdge((float)pt.x, (float)pt.y, (float)boundaryRange, bKeepInSide);
	}

	//是否靠近下边缘
	bool Rect2::IsNearBtmEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const
	{
		return IsNearBtmEdge(pt.x, pt.y, boundaryRange, bKeepInSide);
	}

	//是否靠近下边缘
	bool Rect2::IsNearBtmEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const
	{
		return IsNearBtmEdge((float)ptX, (float)ptY, (float)boundaryRange, bKeepInSide);
	}

	//是否靠近下边缘
	bool Rect2::IsNearBtmEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const
	{
		bool bNear = false;
		Point2i ptArr[4];
		points(ptArr);//获取带角度矩形的四个角点坐标，左下角--ptArr[0] 右下角--ptArr[3]
		Point2f footPt = GetFootPt((float)ptArr[3].x, (float)ptArr[3].y, (float)ptArr[0].x, (float)ptArr[0].y, ptX, ptY);//计算某一点到线段的垂足点坐标
		double distance = sqrt((footPt.x - ptX) * (footPt.x - ptX) + (footPt.y - ptY) * (footPt.y - ptY));
		if (distance <= fabs(boundaryRange))
		{
			if (bKeepInSide)//若当前点必须在区域内才算靠近下边缘
			{
				Point2f pt_footPt(footPt.x - ptX, footPt.y - ptY);//从当前点指向垂足点的向量
				Point2f btmEdgeCnterPt((ptArr[3].x + ptArr[0].x) / 2.0F, (ptArr[3].y + ptArr[0].y) / 2.0F);
				Point2f cnterPt_btmEdgeCnterPt(btmEdgeCnterPt.x - cnter_.x, btmEdgeCnterPt.y - cnter_.y);//从区域中心点指向下边缘中心点的向量
				if (pt_footPt.x * cnterPt_btmEdgeCnterPt.x + pt_footPt.y * cnterPt_btmEdgeCnterPt.y > 0)//两向量同向，表明当前点在区域内；反向，则表明当前点在区域外
					bNear = true;
			}
			else
			{
				Point2f ltBtmPt_footPt(footPt.x - ptArr[0].x, footPt.y - ptArr[0].y);//从区域的左下角点指向垂足点的向量
				Point2f rtBtmPt_footPt(footPt.x - ptArr[3].x, footPt.y - ptArr[3].y);//从区域的右下角点指向垂足点的向量
				if (ltBtmPt_footPt.x * rtBtmPt_footPt.x + ltBtmPt_footPt.y * rtBtmPt_footPt.y < 0)
				{//两向量不同向，说明垂足点在下边缘线上，若超出了下边缘线的范围，则此时两个向量应该是同向的
					bNear = true;
				}
			}
		}
		return bNear;
	}

	//是否靠近下边缘
	bool Rect2::IsNearBtmEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const
	{
		return IsNearBtmEdge((float)ptX, (float)ptY, (float)boundaryRange, bKeepInSide);
	}

	//是否靠近左上角
	bool Rect2::IsNearLtTopEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(pt, boundaryRangeX, bKeepInSide) && IsNearTopEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左上角
	bool Rect2::IsNearLtTopEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearTopEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左上角
	bool Rect2::IsNearLtTopEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(pt, boundaryRangeX, bKeepInSide) && IsNearTopEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左上角
	bool Rect2::IsNearLtTopEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearTopEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左上角
	bool Rect2::IsNearLtTopEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearTopEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左下角
	bool Rect2::IsNearLtBtmEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(pt, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左下角
	bool Rect2::IsNearLtBtmEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左下角
	bool Rect2::IsNearLtBtmEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(pt, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左下角
	bool Rect2::IsNearLtBtmEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近左下角
	bool Rect2::IsNearLtBtmEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearLeftEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右上角
	bool Rect2::IsNearRtTopEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(pt, boundaryRangeX, bKeepInSide) && IsNearTopEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右上角
	bool Rect2::IsNearRtTopEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearTopEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右上角
	bool Rect2::IsNearRtTopEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(pt, boundaryRangeX, bKeepInSide) && IsNearTopEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右上角
	bool Rect2::IsNearRtTopEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearTopEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右上角
	bool Rect2::IsNearRtTopEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearTopEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右下角
	bool Rect2::IsNearRtBtmEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(pt, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右下角
	bool Rect2::IsNearRtBtmEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右下角
	bool Rect2::IsNearRtBtmEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(pt, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(pt, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右下角
	bool Rect2::IsNearRtBtmEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}

	//是否靠近右下角
	bool Rect2::IsNearRtBtmEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const
	{
		bool bNear = false;
		if (IsNearRightEdge(ptX, ptY, boundaryRangeX, bKeepInSide) && IsNearBtmEdge(ptX, ptY, boundaryRangeY, bKeepInSide))
			bNear = true;
		return bNear;
	}


	//根据边缘终点坐标刷新角度矩形宽度
	void Rect2::UpdateWidthByEndPt(Point2i pt)
	{
		UpdateWidthByEndPt(pt.x, pt.y);
	}

	//根据边缘终点坐标刷新角度矩形宽度
	void Rect2::UpdateWidthByEndPt(Point2f pt)
	{
		UpdateWidthByEndPt(pt.x, pt.y);
	}

	//根据边缘终点坐标刷新角度矩形宽度
	void Rect2::UpdateWidthByEndPt(int ptX, int ptY)
	{
		UpdateWidthByEndPt((float)ptX, (float)ptY);
	}

	//根据边缘终点坐标刷新角度矩形宽度
	void Rect2::UpdateWidthByEndPt(float ptX, float ptY)
	{
		Point2i ptArr[4];
		points(ptArr);//获取带角度矩形的四个角点坐标
		Point2f vPt1, vPt2;//Y轴线上的两个点坐标
		vPt1.x = float((ptArr[2].x + ptArr[1].x) / 2);
		vPt1.y = float((ptArr[2].y + ptArr[1].y) / 2);
		vPt2.x = float((ptArr[0].x + ptArr[3].x) / 2);
		vPt2.y = float((ptArr[0].y + ptArr[3].y) / 2);
		Point2f footPt = GetFootPt(vPt1, vPt2, ptX, ptY);//计算某一点到线段的垂足点坐标
		double distance = sqrt((footPt.x - ptX) * (footPt.x - ptX) + (footPt.y - ptY) * (footPt.y - ptY));
		w_ = float((distance > 0.0) ? floor(distance + 0.5) : ceil(distance - 0.5))*2;
	}

	//根据边缘终点坐标刷新角度矩形宽度
	void Rect2::UpdateWidthByEndPt(double ptX, double ptY)
	{
		UpdateWidthByEndPt((float)ptX, (float)ptY);
	}

	//根据边缘终点坐标刷新角度矩形高度
	void Rect2::UpdateHeightByEndPt(Point2i pt)
	{
		UpdateHeightByEndPt(pt.x, pt.y);
	}

	//根据边缘终点坐标刷新角度矩形高度
	void Rect2::UpdateHeightByEndPt(Point2f pt)
	{
		UpdateHeightByEndPt(pt.x, pt.y);
	}

	//根据边缘终点坐标刷新角度矩形高度
	void Rect2::UpdateHeightByEndPt(int ptX, int ptY)
	{
		UpdateHeightByEndPt((float)ptX, (float)ptY);
	}

	//根据边缘终点坐标刷新角度矩形高度
	void Rect2::UpdateHeightByEndPt(float ptX, float ptY)
	{
		Point2i ptArr[4];
		points(ptArr);//获取带角度矩形的四个角点坐标
		Point2f hPt1, hPt2;//X轴线上的两个点坐标
		hPt1.x = float((ptArr[0].x + ptArr[1].x) / 2);
		hPt1.y = float((ptArr[0].y + ptArr[1].y) / 2);
		hPt2.x = float((ptArr[3].x + ptArr[2].x) / 2);
		hPt2.y = float((ptArr[3].y + ptArr[2].y) / 2);
		Point2f footPt = GetFootPt(hPt1, hPt2, ptX, ptY);//计算某一点到线段的垂足点坐标
		double distance = sqrt((footPt.x - ptX) * (footPt.x - ptX) + (footPt.y - ptY) * (footPt.y - ptY));
		h_ = float((distance > 0.0) ? floor(distance + 0.5) : ceil(distance - 0.5))*2;
	}

	//根据边缘终点坐标刷新角度矩形高度
	void Rect2::UpdateHeightByEndPt(double ptX, double ptY)
	{
		UpdateHeightByEndPt((float)ptX, (float)ptY);
	}

	//根据边缘终点坐标刷新角度矩形宽度及高度
	void Rect2::UpdateWidthAndHeightByEndPt(Point2i pt)
	{
		UpdateWidthAndHeightByEndPt(pt.x, pt.y);
	}

	//根据边缘终点坐标刷新角度矩形宽度及高度
	void Rect2::UpdateWidthAndHeightByEndPt(Point2f pt)
	{
		UpdateWidthAndHeightByEndPt(pt.x, pt.y);
	}

	//根据边缘终点坐标刷新角度矩形宽度及高度
	void Rect2::UpdateWidthAndHeightByEndPt(int ptX, int ptY)
	{
		UpdateWidthAndHeightByEndPt((float)ptX, (float)ptY);
	}

	//根据边缘终点坐标刷新角度矩形宽度及高度
	void Rect2::UpdateWidthAndHeightByEndPt(float ptX, float ptY)
	{
		UpdateHeightByEndPt(ptX, ptY);
		UpdateWidthByEndPt(ptX, ptY);
	}

	//根据边缘终点坐标刷新角度矩形宽度及高度
	void Rect2::UpdateWidthAndHeightByEndPt(double ptX, double ptY)
	{
		UpdateWidthAndHeightByEndPt((float)ptX, (float)ptY);
	}




	Ellipse::Ellipse()
	{
		Init();
	}
	Ellipse::Ellipse(const Ellipse& para)
	{
		Init();
		CopyFrom(para);
	}

	Ellipse::Ellipse(Point cnter, int longR, int shortR, double degAng, double startDegAng, double endDegAng)
	{
		cnter_ = cnter;
		degAng_ = degAng;
		startDegAng_ = startDegAng;
		endDegAng_ = endDegAng;
		longR_ = longR;
		shortR_ = shortR;
	}

	Ellipse::Ellipse(int x, int y, int longR, int shortR, double degAng, double startDegAng, double endDegAng)
	{
		cnter_.x = x;
		cnter_.y = y;
		degAng_ = degAng;
		startDegAng_ = startDegAng;
		endDegAng_ = endDegAng;
		longR_ = longR;
		shortR_ = shortR;
	}

	Ellipse::~Ellipse()
	{

	}

	//赋值操作符重载，拷贝功能
	Ellipse& Ellipse::operator = (const Ellipse& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool Ellipse::equal(const HSRoiBase &obj) const
	{
		// 检查当前对象的指针是否为空
		if (this == nullptr) {
			// 处理空指针情况，例如可以返回 false 或抛出异常
			return false;
		}

		bool bEqual = false;
		if (this->GetRegType() == obj.GetRegType()/* && typeid(this) == typeid(&obj)*/)
		{
			if (const Ellipse *ptr = dynamic_cast<const Ellipse*>(&obj))
			{
				if (this->cnter_ == ptr->cnter_	            &&
					this->longR_ == ptr->longR_	            &&
					this->shortR_ == ptr->shortR_	        &&
					this->degAng_ == ptr->degAng_		    &&
					this->startDegAng_ == ptr->startDegAng_	&&
					this->endDegAng_ == ptr->endDegAng_)
				{
					bEqual = true;
				}
			}
		}
		return bEqual;
	}

	bool Ellipse::operator==(const Ellipse & obj) const
	{
		return equal(obj);
	}

	Ellipse Ellipse::operator-(Point point)
	{
		Ellipse elp(*this);
		elp.cnter_.x -= point.x;
		elp.cnter_.y -= point.y;
		return elp;
	}

	Ellipse Ellipse::operator+(Point point)
	{
		Ellipse elp(*this);
		elp.cnter_.x += point.x;
		elp.cnter_.y += point.y;
		return elp;
	}

	void Ellipse::Init()
	{
		cnter_.x = 0;
		cnter_.y = 0;
		longR_ = 0;
		shortR_ = 0;
		degAng_ = 0;
		startDegAng_ = 0;
		endDegAng_ = 360;
	}

	//从para拷贝数据
	void Ellipse::CopyFrom(const Ellipse& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void Ellipse::CopyTo(Ellipse& para) const
	{
		if (this != &para)
		{
			para.cnter_ = cnter_;
			para.longR_ = longR_;
			para.shortR_ = shortR_;
			para.degAng_ = degAng_;
			para.startDegAng_ = startDegAng_;
			para.endDegAng_ = endDegAng_;
		}
	}

	RegType Ellipse::GetRegType() const
	{
		return RegType::HSReg_ELLIPSE;
	}

	void Ellipse::SetCnter(Point2f cnter)
	{
		cnter_.x = int((cnter.x > 0.0) ? floor(cnter.x + 0.5) : ceil(cnter.x - 0.5));
		cnter_.y = int((cnter.y > 0.0) ? floor(cnter.y + 0.5) : ceil(cnter.y - 0.5));
	}

	//更新当前区域的中心坐标偏移量
	void Ellipse::SetCnterOffset(int offsetX, int offsetY)
	{
		cnter_.x += offsetX;
		cnter_.y += offsetY;
	}

	void Ellipse::SetDegAng(float degAng)
	{
		degAng_ = degAng;
	}

	void Ellipse::SetDegAngOffset(float offsetDegAng)
	{
		degAng_ += offsetDegAng;
	}

	float Ellipse::GetDegAng() const
	{
		return degAng_;
	}

	//获取区域面积
	int Ellipse::Area() const
	{
		return (int)(PI_HS * longR_ * shortR_);
	}

	//判断当前区域是否为空
	bool Ellipse::IsEmpty() const
	{
		return (0 == Area());
	}

	//获取区域中心坐标	
	Point2f Ellipse::Center() const
	{
		return Point2f((float)cnter_.x, (float)cnter_.y);
	}

	void Ellipse::GetMaxSize(int& maxW, int& maxH) const
	{
		int rect[4] = { 0 };
		BoundingRect(rect);
		maxW = rect[2];
		maxH = rect[3];
	}

	//返回矩形4个顶点坐标
	void Ellipse::Points(Point pts[]) const
	{
		int top_x = (cnter_.x);
		int top_y = (cnter_.y - shortR_);
		int bottox_ = (cnter_.x);
		int bottoy_ = (cnter_.y + shortR_); //短半轴
		int left_x = (cnter_.x - longR_);
		int left_y = (cnter_.y);
		int right_x = (cnter_.x + longR_);
		int right_y = (cnter_.y);

		if (degAng_ == 0)
		{
			pts[0] = Point(bottox_, bottoy_);
			pts[1] = Point(left_x, left_y);
			pts[2] = Point(top_x, top_y);
			pts[3] = Point(right_x, right_y);
		}
		else
		{
			//假设对图片上任意点(x, y)，绕一个坐标点(rx0, ry0)逆时针旋转a角度后的新的坐标设为(x0, y0)，有公式：
			//x0 = (x - rx0)*cos(a) - (y - ry0)*sin(a) + rx0;
			//y0 = (x - rx0)*sin(a) + (y - ry0)*cos(a) + ry0;

			int newp0_x = (int)(round)((bottox_ - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (bottoy_ - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
			int newp0_y = (int)(round)((bottox_ - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (bottoy_ - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);

			int newp1_x = (int)(round)((left_x - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (left_y - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
			int newp1_y = (int)(round)((left_x - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (left_y - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);

			int newp2_x = (int)(round)((top_x - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (top_y - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
			int newp2_y = (int)(round)((top_x - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (top_y - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);

			int newp3_x = (int)(round)((right_x - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (right_y - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
			int newp3_y = (int)(round)((right_x - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (right_y - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);

			pts[0] = Point(newp0_x, newp0_y);
			pts[1] = Point(newp1_x, newp1_y);
			pts[2] = Point(newp2_x, newp2_y);
			pts[3] = Point(newp3_x, newp3_y);
		}
	}

	//获取区域外接矩形
	void Ellipse::BoundingRect(int rect[]) const
	{
		if (degAng_ == 0)
		{
			rect[0] = cnter_.x - longR_;
			rect[1] = cnter_.y - shortR_;
			rect[2] = longR_ * 2;
			rect[3] = shortR_ * 2 - 1;
		}
		else
		{
			float A = (float)(pow(longR_, 2) * pow(sin(-PI_HS * degAng_ / 180), 2) + pow(shortR_, 2) * pow(cos(-PI_HS * degAng_ / 180), 2));
			float B = (float)(2 * (pow(longR_, 2) - pow(shortR_, 2)) * sin(-PI_HS * degAng_ / 180) * cos(-PI_HS * degAng_ / 180));
			float C = (float)(pow(longR_, 2) * pow(cos(-PI_HS * degAng_ / 180), 2) + pow(shortR_, 2) * pow(sin(-PI_HS * degAng_ / 180), 2));
			float F = (float)(-pow(longR_, 2) * pow(shortR_, 2));

			float y = (float)sqrt(4 * A * F / (B * B - 4 * A * C));
			int y1 = (int)round(-abs(y)) + cnter_.y;
			int y2 = (int)round(abs(y)) + cnter_.y;

			float x = (float)sqrt(4 * C * F / ((B * B) - (4 * A * C)));
			int x1 = (int)round(-abs(x)) + cnter_.x;
			int x2 = (int)round(abs(x)) + cnter_.x;

			rect[0] = x1;
			rect[1] = y1;
			rect[2] = abs(x2 - x1) + 1;// +1 统一使用宽高输出 @LiGJ 2025/03/21
			rect[3] = abs(y2 - y1) + 1;
		}
	}

	//获取区域外接矩形	
	Rect Ellipse::BoundingRect() const
	{
		int rct[4] = { 0 };
		BoundingRect(rct);
		return Rect(rct[0], rct[1], rct[2], rct[3]);
	}

	//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
	void Ellipse::SetScaleRate(float scaleRate)
	{
		SetScaleRate(scaleRate, scaleRate);
	}

	void Ellipse::SetScaleRate(float scaleRateX, float scaleRateY)
	{
		
	}


	Polygon::Polygon()
	{
		Init();
	}
	Polygon::Polygon(const Polygon& para)
	{
		Init();
		CopyFrom(para);
	}

	Polygon::Polygon(Point* points, int N)
	{
		m_N = N;
		#if 0//临时注释@Chenw 07/22/2023, 15:00
			int xywh[4];
			points_ = points;
			boundingRect(xywh);
			cnter_.x = xywh[0] + xywh[2] / 2;
			cnter_.y = xywh[1] + xywh[3] / 2;
		#endif // 0
	}

	Polygon::~Polygon()
	{
		if (points_!=nullptr)
		{
			if (m_N>1)
			{
				delete[] points_;
			}
			else
			{
				delete points_;
			}
		}
	}

	//赋值操作符重载，拷贝功能
	Polygon& Polygon::operator = (const Polygon& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool Polygon::equal(const HSRoiBase &obj) const
	{
		// 检查当前对象的指针是否为空
		if (this == nullptr) {
			// 处理空指针情况，例如可以返回 false 或抛出异常
			return false;
		}

		bool bEqual = false;
		if (this->GetRegType() == obj.GetRegType()/* && typeid(this) == typeid(&obj)*/)
		{
			if (const Polygon *ptr = dynamic_cast<const Polygon*>(&obj))
			{
				if (this->points_ == ptr->points_	 &&
					this->m_N == ptr->m_N)
				{
					bEqual = true;
				}
			}
		}
		return bEqual;
	}

	bool Polygon::operator==(const Polygon & obj) const
	{
		return equal(obj);
	}

	Polygon Polygon::operator-(Point point)
	{
		Polygon polygon(*this);
		for (int i = 0; i < polygon.m_N; i++)
		{
			polygon.points_[i].x -= point.x;
			polygon.points_[i].y -= point.y;
		}
		return polygon;
	}

	Polygon Polygon::operator+(Point point)
	{
		Polygon polygon(*this);
		for (int i = 0; i < polygon.m_N; i++)
		{
			polygon.points_[i].x += point.x;
			polygon.points_[i].y += point.y;
		}
		return polygon;
	}

	void Polygon::Init()
	{
		m_N = 0;
		points_ = NULL;
	}

	//从para拷贝数据
	void Polygon::CopyFrom(const Polygon& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void Polygon::CopyTo(Polygon& para) const
	{
		if (this != &para)
		{
			para.m_N = m_N;
			//para.points_ = points_;

			if (m_N==1)
			{
				delete para.points_;
				para.points_ = new Point();
				*para.points_ =* points_;
			}
			else if (m_N>1)
			{
				delete[] para.points_;
				para.points_ = new Point[m_N];
				for (int i = 0; i < m_N; i++)
				{
					*(para.points_+i) = *(points_+i);
				}
			}
		}
	}

	RegType Polygon::GetRegType() const
	{
		return RegType::HSReg_POLYGON;
	}

	void Polygon::SetCnter(Point2f cnter)
	{
		Point2f oldCnter = Center();
		Point2f offset = cnter - oldCnter;
		Point offPt;
		offPt.x = int((offset.x > 0.0) ? floor(offset.x + 0.5) : ceil(offset.x - 0.5));
		offPt.y = int((offset.y > 0.0) ? floor(offset.y + 0.5) : ceil(offset.y - 0.5));
		for (int i = 0; i < m_N; i++)
		{
			points_[i].x += offPt.x;
			points_[i].y += offPt.y;
		}	
	}

	//更新当前区域的中心坐标偏移量
	void Polygon::SetCnterOffset(int offsetX, int offsetY)
	{
		for (int i = 0; i < m_N; i++)
		{
			points_[i].x += offsetX;
			points_[i].y += offsetY;
		}
	}

	void Polygon::SetDegAng(float degAng)
	{
		//....
	}

	void Polygon::SetDegAngOffset(float offsetDegAng)
	{
		//....
	}

	float Polygon::GetDegAng() const
	{
		return 0.0F;
	}

	//获取区域面积
	int Polygon::Area() const
	{
		int j;
		int area = 0;
		for (int i = 0; i < m_N; i++)
		{
			j = (i + 1) % m_N;
			area += points_[i].x * points_[j].y;
			area -= points_[i].y * points_[j].x;
		}
		area /= 2;
		return (area < 0 ? -area : area);
	}

	//判断当前区域是否为空
	bool Polygon::IsEmpty() const
	{
		return (0 == Area());
	}

	//获取区域中心坐标	
	Point2f Polygon::Center() const
	{
#if 1//临时编辑 @ChenW 07/22/2023, 15:02
		int xywh[4];
		BoundingRect(xywh);
		Point2f pt;
		pt.x = xywh[0] + xywh[2] / 2;
		pt.y = xywh[1] + xywh[3] / 2;
#endif // 1//临时编辑 @ChenW 07/22/2023, 15:02
		return pt;
	}

	void Polygon::GetMaxSize(int& maxW, int& maxH) const
	{
		int rect[4] = { 0 };
		BoundingRect(rect);
		maxW = rect[2];
		maxH = rect[3];
	}

	//返回矩形4个顶点坐标
	void Polygon::Points(Point pts[]) const
	{
		
	}

	//获取区域外接矩形
	void Polygon::BoundingRect(int rect[]) const
	{
		std::vector<int> x;
		std::vector<int> y;
		for (int i = 0; i < m_N; i++)
		{
			x.push_back(points_[i].x);
			y.push_back(points_[i].y);
		}
		int max_x = *max_element(x.begin(), x.end());
		int min_x = *min_element(x.begin(), x.end());
		int max_y = *max_element(y.begin(), y.end());
		int min_y = *min_element(y.begin(), y.end());

		rect[0] = min_x;
		rect[1] = min_y;
		rect[2] = abs(max_x - min_x) + 1;// +1 统一使用宽高输出 @LiGJ 2025/03/21
		rect[3] = abs(max_y - min_y) + 1;
	}

	//获取区域外接矩形	
	Rect Polygon::BoundingRect() const
	{
		int rct[4] = { 0 };
		BoundingRect(rct);
		return Rect(rct[0], rct[1], rct[2], rct[3]);
	}

	//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
	void Polygon::SetScaleRate(float scaleRate)
	{
		SetScaleRate(scaleRate, scaleRate);
	}

	void Polygon::SetScaleRate(float scaleRateX, float scaleRateY)
	{
	
	}


#if USING_OLD_HS_REG //IA2.0默认不启用旧的HSReg数据结构 @ChenW 08/01/2024, 11:55
	void TransHSReg(const HSV::HSRoiBase* pRoi, HSReg& reg)
	{
		switch (pRoi->GetRegType())
		{
		default:
			break;
		case HSV::HSReg_RECT:
		{
			HSV::Rect rect;
			rect = *(HSV::Rect*)pRoi;
			reg.SetShapeType(HSReg::REG_SHAPE_TYPE_RECT1);
			reg.pRect1->ltTopPt.x = rect.ltTopPt_.x;
			reg.pRect1->ltTopPt.y = rect.ltTopPt_.y;
			reg.pRect1->rtBtnPt.x = rect.br().x;
			reg.pRect1->rtBtnPt.y = rect.br().y;
			/*reg.pRect1->SetWidth(rect.w_);
			reg.pRect1->SetHalfH(rect.h_);*/
		}
		break;
		case HSV::HSReg_RECT2:
		{
			HSV::Rect2 rect2;
			rect2 = *(HSV::Rect2*)pRoi;
			reg.SetShapeType(HSReg::REG_SHAPE_TYPE_RECT2);
			reg.pRect2->SetCnterPt(rect2.cnter_.x, rect2.cnter_.y);
			reg.pRect2->SetHalfW(rect2.w_ / 2);
			reg.pRect2->SetHalfH(rect2.h_ / 2);
		}
		break;
		case HSV::HSReg_CIRCLE:
		{
			HSV::Circle circle;
			circle = *(HSV::Circle*)pRoi;
			reg.SetShapeType(HSReg::REG_SHAPE_TYPE_CIRCLE);
			reg.pCircle->SetCnterPt(circle.cnter_.x, circle.cnter_.y);
			reg.pCircle->SetRadius(circle.radius_);
		}
		break;
		case HSV::HSReg_ELLIPSE:
		{
			HSV::Ellipse elp;
			elp = *(HSV::Ellipse*)pRoi;
			reg.SetShapeType(HSReg::REG_SHAPE_TYPE_ELLIPSE);
			reg.pElp->SetCnterPt(elp.cnter_.x, elp.cnter_.y);
			reg.pElp->SetLongR(elp.longR_);
			reg.pElp->SetShortR(elp.shortR_);
		}
		break;
		case HSV::HSReg_POLYGON:
			break;
		}
	}

	void TransHSReg(const HSReg& reg, HSV::HSRoiBase** pRoi)
	{
		switch (*reg.GetRegShapeType())
		{
		case HSReg::REG_SHAPE_TYPE_RECT1://矩形	
		{
			if (*pRoi == NULL)
				*pRoi = new	HSV::Rect;
			HSV::SwitchHSRoiObjType(HSV::HSReg_RECT, pRoi, false);
			HSV::Rect* pRect1 = dynamic_cast<HSV::Rect*>(*pRoi);
			pRect1->ltTopPt_.x = reg.pRect1->ltTopPt.x;
			pRect1->ltTopPt_.y = reg.pRect1->ltTopPt.y;
			pRect1->w_ = reg.pRect1->GetWidth();
			pRect1->h_ = reg.pRect1->GetHeight();
		}
		break;
		case HSReg::REG_SHAPE_TYPE_RECT2://带角度矩形
		{
			if (*pRoi == NULL)
				*pRoi = new	HSV::Rect2;
			HSV::SwitchHSRoiObjType(HSV::HSReg_RECT2, pRoi, false);
			HSV::Rect2* pRect2 = dynamic_cast<HSV::Rect2*>(*pRoi);
			pRect2->w_ = 2 * reg.pRect2->halfW;
			pRect2->h_ = 2 * reg.pRect2->halfH;
			pRect2->cnter_.x = reg.pRect2->cnter.x;
			pRect2->cnter_.y = reg.pRect2->cnter.y;
			pRect2->degAng_ = reg.pRect2->degAng;
		}
		break;
		case HSReg::REG_SHAPE_TYPE_CIRCLE://圆形
		{
			if (*pRoi == NULL)
				*pRoi = new	HSV::Circle;
			HSV::SwitchHSRoiObjType(HSV::HSReg_CIRCLE, pRoi, false);
			HSV::Circle* pCircle = dynamic_cast<HSV::Circle*>(*pRoi);
			pCircle->radius_ = reg.pCircle->radius;
			pCircle->cnter_.x = reg.pCircle->cnter.x;
			pCircle->cnter_.y = reg.pCircle->cnter.y;
		}
		break;
		case HSReg::REG_SHAPE_TYPE_ELLIPSE://椭圆
		{
			if (*pRoi == NULL)
				*pRoi = new	HSV::Ellipse;
			HSV::SwitchHSRoiObjType(HSV::HSReg_ELLIPSE, pRoi, false);
			HSV::Ellipse* pElp = dynamic_cast<HSV::Ellipse*>(*pRoi);
			pElp->longR_ = reg.pElp->longR;
			pElp->shortR_ = reg.pElp->shortR;
			pElp->cnter_.x = reg.pElp->cnter.x;
			pElp->cnter_.y = reg.pElp->cnter.y;
			pElp->degAng_ = reg.pElp->degAng;
		}
		break;
		default:
			assert(0);
			break;
		}
	}
#endif //IA2.0默认不启用旧的HSReg数据结构 @ChenW 08/01/2024, 11:55

	void SwitchHSRoiObjType(HSV::RegType regType, HSV::HSRoiBase** pObj, bool bTransDat)
	{
		Point2f cnterPt(0.0F, 00.0F);
		int maxW = 0, maxH = 0;
		if (*pObj != NULL && regType != (*pObj)->GetRegType())
		{
			if (bTransDat)
			{
				switch ((*pObj)->GetRegType())
				{
				default:
					break;
				case HSV::HSReg_RECT:
				{
					HSV::Rect* ptr = dynamic_cast<HSV::Rect*>(*pObj);
					cnterPt = ptr->Center();
					ptr->GetMaxSize(maxW, maxH);
				}
				break;
				case HSV::HSReg_RECT2:
				{
					HSV::Rect2* ptr = dynamic_cast<HSV::Rect2*>(*pObj);
					cnterPt = ptr->Center();
					ptr->GetMaxSize(maxW, maxH);
				}
				break;
				case HSV::HSReg_CIRCLE:
				{
					HSV::Circle* ptr = dynamic_cast<HSV::Circle*>(*pObj);
					cnterPt = ptr->Center();
					ptr->GetMaxSize(maxW, maxH);
				}
				break;
				case HSV::HSReg_ELLIPSE:
				{
					HSV::Ellipse* ptr = dynamic_cast<HSV::Ellipse*>(*pObj);
					cnterPt = ptr->Center();
					ptr->GetMaxSize(maxW, maxH);
				}
				break;
				case HSV::HSReg_POLYGON:
				{
					HSV::Polygon* ptr = dynamic_cast<HSV::Polygon*>(*pObj);
					cnterPt = ptr->Center();
					ptr->GetMaxSize(maxW, maxH);
				}
				break;
				}
			}
			delete *pObj;
			*pObj = NULL;
		}
		if (*pObj == NULL)
		{
			switch (regType)
			{
			default:
				break;
			case HSV::HSReg_RECT:
				*pObj = new HSV::Rect;
				if (bTransDat)
				{
					HSV::Rect* ptr = dynamic_cast<HSV::Rect*>(*pObj);
					ptr->w_ = maxW;
					ptr->h_ = maxH;
					ptr->SetCnter(cnterPt);
				}
				break;
			case HSV::HSReg_RECT2:
				*pObj = new HSV::Rect2;
				if (bTransDat)
				{
					HSV::Rect2* ptr = dynamic_cast<HSV::Rect2*>(*pObj);
					ptr->w_ = maxW;
					ptr->h_ = maxH;
					ptr->SetCnter(cnterPt);
				}
				break;
			case HSV::HSReg_CIRCLE:
				*pObj = new HSV::Circle;
				if (bTransDat)
				{
					HSV::Circle* ptr = dynamic_cast<HSV::Circle*>(*pObj);
					ptr->radius_ = (std::max)(maxW, maxH);
					ptr->SetCnter(cnterPt);
				}
				break;
			case HSV::HSReg_ELLIPSE:
				*pObj = new HSV::Ellipse;
				if (bTransDat)
				{
					HSV::Ellipse* ptr = dynamic_cast<HSV::Ellipse*>(*pObj);
					ptr->longR_ = maxW;
					ptr->shortR_ = maxH;
					ptr->SetCnter(cnterPt);
				}
				break;
			case HSV::HSReg_POLYGON:
				*pObj = new HSV::Polygon;
				if (bTransDat)
				{
					HSV::Polygon* ptr = dynamic_cast<HSV::Polygon*>(*pObj);
					ptr->SetCnter(cnterPt);
				}
				break;
			}
		}
	}

	void CopyHSRoiObjDat(const HSV::HSRoiBase* srcObj, HSV::HSRoiBase** dstObj)
	{
		if (srcObj == *dstObj)
			return;
		const HSV::RegType regType = srcObj->GetRegType();
		SwitchHSRoiObjType(regType, dstObj, false);
		switch (regType)
		{
		default:
			break;
		case HSV::HSReg_RECT:
			*(HSV::Rect*)(*dstObj) = *(HSV::Rect*)srcObj;
			break;
		case HSV::HSReg_RECT2:
			*(HSV::Rect2*)(*dstObj) = *(HSV::Rect2*)srcObj;
			break;
		case HSV::HSReg_CIRCLE:
			*(HSV::Circle*)(*dstObj) = *(HSV::Circle*)srcObj;
			break;
		case HSV::HSReg_ELLIPSE:
			*(HSV::Ellipse*)(*dstObj) = *(HSV::Ellipse*)srcObj;
			break;
		case HSV::HSReg_POLYGON:
			*(HSV::Polygon*)(*dstObj) = *(HSV::Polygon*)srcObj;
			break;
		}
	}

	void GenNewRoiObj(const HSV::HSRoiBase* srcObj, const HSV::Point2f& pos, HSV::HSRoiBase** dstObj)
	{
		CopyHSRoiObjDat(srcObj, dstObj);
		(*dstObj)->SetCnter(pos);
	}

	bool EqualRoiPara(const HSRoiBase* srcObj, const HSRoiBase* dstObj)
	{
		if (srcObj == dstObj)
			return true;
		const HSV::RegType regType = srcObj->GetRegType();
		if (regType != dstObj->GetRegType())
			return false;
		return srcObj->equal(*dstObj);
	}

	void ClearHSRoiReg(HSV::HSRoiBase** dstObj)
	{
		if (*dstObj != NULL)
		{
			delete *dstObj;
			*dstObj = NULL;
		}
	}
	void ClearHSRoiReg(std::vector<HSRoiBase*> roi1D)
	{
		for (auto& it : roi1D)
		{
			ClearHSRoiReg(&it);
		}
		roi1D.clear();
	}

	void ClearHSRoiReg(HSV::Rect** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearHSRoiReg(HSV::Circle** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearHSRoiReg(HSV::Rect2** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearHSRoiReg(HSV::Ellipse** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	void ClearHSRoiReg(HSV::Polygon** pObj)
	{
		if (*pObj != NULL)
		{
			delete *pObj;
			*pObj = NULL;
		}
	}

	HSV::Rect BoundingRect(const HSRoiBase* pObj)
	{
		int rectPt[4] = { 0 };
		pObj->BoundingRect(rectPt);
		return HSV::Rect(rectPt[0], rectPt[1], rectPt[2], rectPt[3]);
	}

	HSV::Rect BoundingRect(const std::vector<HSV::HSRoiBase*>& pObj1D)
	{
		HSV::Rect rect(0, 0, 0, 0);
		for (unsigned int i = 0; i < pObj1D.size(); i++)
		{
			if (i == 0)
			{
				rect = HSV::BoundingRect(pObj1D[i]);
			}
			else
			{
				HSV::Rect rectTmp = HSV::BoundingRect(pObj1D[i]);
				rect = rect & rectTmp;
			}
		}
		return rect;
	}

	HSV::Rect2 BoundingRect2(const HSRoiBase* pObj)
	{
		HSV::Rect2 rect2(0.0F, 0.0F, 0, 0, 0.0F);
		const HSV::RegType regType = pObj->GetRegType();
		switch (regType)
		{
		default:
			break;
		case HSV::HSReg_RECT:
		{
			const HSV::Rect* ptr = dynamic_cast<const HSV::Rect*>(pObj);
			rect2.cnter_ = ptr->Center();
			rect2.degAng_ = 0.0F;
			rect2.w_ = ptr->w_;
			rect2.h_ = ptr->h_;
		}
			break;
		case HSV::HSReg_RECT2:
		{
			const HSV::Rect2* ptr = dynamic_cast<const HSV::Rect2*>(pObj);
			rect2 = *ptr;
		}
			break;
		case HSV::HSReg_CIRCLE:
		{
			const HSV::Circle* ptr = dynamic_cast<const HSV::Circle*>(pObj);
			rect2.cnter_ = ptr->Center();
			rect2.degAng_ = 0.0F;
			rect2.w_ = rect2.h_ = 2 * ptr->radius_ + 1;
		}
			break;
		case HSV::HSReg_ELLIPSE:
		{
			const HSV::Ellipse* ptr = dynamic_cast<const HSV::Ellipse*>(pObj);
			rect2.cnter_ = ptr->Center();
			rect2.degAng_ = ptr->degAng_;
			rect2.w_ = 2 * ptr->longR_ + 1;
			rect2.h_ = 2 * ptr->shortR_ + 1;
		}
			break;
		case HSV::HSReg_POLYGON:
		{
			const HSV::Polygon* ptr = dynamic_cast<const HSV::Polygon*>(pObj);
			//待完善....
		}
			break;
		}
		return rect2;
	}

	HSV::Rect GetIntersectionRect(const std::vector<HSV::HSRoiBase*> pRoi1D)
	{
		HSV::Rect rect(0, 0, 0, 0);
		if (pRoi1D.size() > 0)
		{
			rect = BoundingRect(pRoi1D[0]);
			HSV::Rect tmpRct;
			for (auto it = pRoi1D.begin() + 1; it != pRoi1D.end(); it++)
			{
				tmpRct = BoundingRect(*it);
				rect = rect & tmpRct;//两个矩形&计算，返回交集矩形
			}
		}
		return rect;
	}
}
