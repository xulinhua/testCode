//#include "stdafx.h"
#include"HSDrawObj.h"
#include<assert.h>
#include<algorithm>
#include <vector>
#include <cmath>

namespace HSV
{


#pragma region 绘图对象_Rect2类（带角度矩形）
	void Rect2Draw::Init()
	{
		isDash_ = false;  //是否为虚线  luojianghong 23-9-4
		nCount_ = 20;  //线段比例    luojianghong 23-9-4
		Rect2::Init();
		DrawObjBase::Init();
	}
	Rect2Draw::Rect2Draw()
	{
		Init();
	}
	Rect2Draw::Rect2Draw(const Rect2Draw& para)
	{
		Init();
		CopyFrom(para);
	}

	Rect2Draw::Rect2Draw(Point2f cnter, int w, int h, double degAng, GC_COL clrType, int thickness, int lineType, int shift)
		: Rect2(cnter, w, h, degAng), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Rect2(cnter, w, h, degAng);
			DrawObjBase::SetDrawDat(clrType, thickness, lineType, shift);
		}
	}


	Rect2Draw::Rect2Draw(Point2f cnter, int w, int h, double degAng, ScalarGC color, int thickness, int lineType, int shift) 
		: Rect2(cnter, w, h, degAng), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Rect2(cnter, w, h, degAng);
			DrawObjBase::SetDrawDat(color, thickness, lineType, shift);
		}
	}


	Rect2Draw::Rect2Draw(float x, float y, int w, int h, double degAng, GC_COL clrType, int thickness, int lineType, int shift)
		: Rect2(x, y, w, h, degAng), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Rect2(x, y, w, h, degAng);
			DrawObjBase::SetDrawDat(clrType, thickness, lineType, shift);
		}
	}

	Rect2Draw::Rect2Draw(float x, float y, int w, int h, double degAng, ScalarGC color, int thickness, int lineType, int shift)
		: Rect2(x, y, w, h, degAng), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Rect2(x, y, w, h, degAng);
			DrawObjBase::SetDrawDat(color, thickness, lineType, shift);
		}
	}

	Rect2Draw::~Rect2Draw()
	{

	}
	Rect2Draw& Rect2Draw::operator = (const Rect2Draw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool Rect2Draw::equal(const DrawObjBase &obj) const
	{
		// 检查当前对象的指针是否为空
		if (this == nullptr) {
			// 处理空指针情况，例如可以返回 false 或抛出异常
			return false;
		}

		bool bEqual = false;
		if (this->GetDrawType() == obj.GetDrawType()/* && typeid(this) == typeid(&obj)*/)
		{
			if (this->nID_ >= 0)
			{
				return (this->nID_ == obj.nID_);
			}
			else
			{
				if (const Rect2 *ptr = dynamic_cast<const Rect2*>(&obj))
				{
					const Rect2Draw *ptrRect = dynamic_cast<const Rect2Draw*>(&obj);
					bEqual = Rect2::equal(*ptr) && isDash_ == ptrRect->isDash_ &&nCount_ == ptrRect->nCount_;
				}
			}
		}
		return bEqual;
	}

	bool Rect2Draw::operator==(const Rect2Draw & obj)
	{
		return equal(obj);
	}

	void Rect2Draw::CopyFrom(const Rect2Draw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}

	void Rect2Draw::CopyTo(Rect2Draw& para) const
	{
		if (this != &para)
		{
			para.isDash_ = isDash_;
			para.nCount_ = nCount_;
			Rect2::CopyTo(para);
			DrawObjBase::CopyTo(para);
			//__super::CopyTo(para);
		}
	}

	//从para拷贝数据	
	void Rect2Draw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::Rect2Draw* ptr = dynamic_cast<const HSV::Rect2Draw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void Rect2Draw::CopyTo(DrawObjBase** para) const
	{
		HSV::Rect2Draw* ptr = dynamic_cast<HSV::Rect2Draw*>(*para);
		CopyTo(*ptr);
	}

	//当前对象是否为空（主要用于判断当前区域是否为空）
	bool Rect2Draw::IsEmpty() const
	{
		return Rect2::IsEmpty();
	}

	HSV::DrawType Rect2Draw::GetDrawType() const//获取绘制图形类型
	{
		return DrawType::DRAW_RECT2;
	}

	bool Rect2Draw::IsRoiReg() const
	{
		return true;
	}

	void Rect2Draw::SetRectDat(const HSV::Rect& rect)
	{
		HSV::Rect2 rect2 = HSV::BoundingRect2(&rect);
		if (0)
		{
			HSV::Rect2* pRect2 = dynamic_cast<HSV::Rect2*>(this);
			*pRect2 = rect2;
		}
		else
		{
			*((HSV::Rect2*)this) = rect2;
		}
	}

#if 0
	void Rect2Draw::GetLinePoint(LineDraw* dash_lines, int nCount)
	{

		Point pts[4];
		points(pts);

		double interval_x = (double)w_ / nCount;
		double interval_y = (double)h_ / nCount;

		for (int i = 0; i < nCount; i++)//绘制矩形框上边沿虚线
		{
			if (i % 2 == 0)
				*(dash_lines + i) = LineDraw(Point(pts[0].x + (int)(round)(i * interval_x * cos(-PI_HS * degAng_ / 180)), pts[0].y - (int)(round)(i * interval_x * sin(PI_HS * degAng_ / 180))),
				(int)round(interval_x), -degAng_);
		}
		for (int i = 0; i < nCount; i++)//绘制矩形框下边沿虚线
		{
			if (i % 2 == 0)
				*(dash_lines + nCount + i) = LineDraw(Point(pts[1].x + (int)(round)(i * interval_y * sin(PI_HS * (degAng_) / 180)), pts[1].y + (int)(round)(i * interval_y * cos(PI_HS * (degAng_) / 180))),
				(int)round(interval_y), -degAng_ + 90);
		}
		for (int i = 0; i < nCount; i++)//绘制矩形框左边沿虚线
		{
			if (i % 2 == 0)
				*(dash_lines + nCount * 2 + i) = LineDraw(Point(pts[2].x - (int)(round)(i * interval_x * cos(PI_HS * degAng_ / 180)), pts[2].y + (int)(round)(i * interval_x * sin(PI_HS * degAng_ / 180))),
				(int)round(interval_x), -degAng_ + 180);
		}
		for (int i = 0; i < nCount; i++)//绘制矩形框右边沿虚线
		{
			if (i % 2 == 0)
				*(dash_lines + nCount * 3 + i) = LineDraw(Point(pts[3].x - (int)(round)(i * interval_y * cos(PI_HS * (90 - degAng_) / 180)), pts[3].y - (int)(round)(i * interval_y * sin(PI_HS * (90 - degAng_) / 180))),
				(int)round(interval_y), -degAng_ - 90);
		}

	}
#endif // 0

#pragma endregion

#pragma region 区域类--（不带角度）矩形

#pragma endregion

}
