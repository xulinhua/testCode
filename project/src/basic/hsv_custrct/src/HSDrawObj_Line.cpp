//#include "stdafx.h"
#include"HSDrawObj.h"
#include<assert.h>
#include<algorithm>
#include <vector>
#include <cmath>

namespace HSV
{
	
#pragma region 绘图对象--直线
	LineDraw::LineDraw()
	{
		Init();
	}

	LineDraw::LineDraw(const LineDraw& para)
	{
		Init();
		CopyFrom(para);
	}

	LineDraw::LineDraw(Point p1, Point p2, GC_COL clrType, int thickness, int lineType, int shift)
	{
		pt1_ = p1;
		pt2_ = p2;
		this->SetColor(clrType);
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
	}

	LineDraw::LineDraw(Point p1, Point p2, ScalarGC color, int thickness, int lineType, int shift)
	{
		pt1_ = p1;
		pt2_ = p2;
		color_ = color;
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
	}
	
	LineDraw::LineDraw(Point p1, int len, double degAng, GC_COL color_type, int thickness, int lineType, int shift)
	{
		this->SetColor(color_type);
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
		pt1_ = p1;

		int pt2_x = pt1_.x + (int)(round)(len * cos(degAng * PI_HS / 180));
		int pt2_y = pt1_.y + (int)(round)(len * sin(degAng * PI_HS / 180));
		pt2_ = Point(pt2_x, pt2_y);
	}
	LineDraw::LineDraw(Point p1, int len, double degAng, ScalarGC color, int thickness, int lineType, int shift)
	{
		color_ = color;
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
		pt1_ = p1;
		int pt2_x = pt1_.x + (int)(round)(len * cos(degAng * PI_HS / 180));
		int pt2_y = pt1_.y + (int)(round)(len * sin(degAng * PI_HS / 180));
		pt2_ = Point(pt2_x, pt2_y);
	}

	LineDraw::~LineDraw()
	{

	}
	LineDraw& LineDraw::operator = (const LineDraw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}
	bool LineDraw::equal(const DrawObjBase &obj) const
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
				if (const LineDraw *ptr = dynamic_cast<const LineDraw*>(&obj))
				{
					if (this->pt1_ == ptr->pt1_	&&
						this->pt2_ == ptr->pt2_)
					{
						bEqual = true;
					}
				}
			}
		}
		return bEqual;
	}
	bool LineDraw::operator==(const LineDraw & obj)
	{
		return equal(obj);
	}

	LineDraw LineDraw::operator+(const Point& point)
	{
		LineDraw line(*this);
		line.pt1_.x += point.x;
		line.pt1_.y += point.y;

		line.pt2_.x += point.x;
		line.pt2_.y += point.y;

		return line;
	}
	LineDraw LineDraw::operator-(const Point& point)
	{
		LineDraw line(*this);
		line.pt1_.x -= point.x;
		line.pt1_.y -= point.y;

		line.pt2_.x -= point.x;
		line.pt2_.y -= point.y;

		return line;
	}
	//从para拷贝数据
	void LineDraw::CopyFrom(const LineDraw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void LineDraw::CopyTo(LineDraw& para) const
	{
		if (this != &para)
		{
			//DrawObjBase::CopyTo(para);
			para.pt1_ = pt1_;
			para.pt2_ = pt2_;
			DrawObjBase::CopyTo(para);
		}
	}

	//从para拷贝数据	
	void LineDraw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::LineDraw* ptr = dynamic_cast<const HSV::LineDraw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void LineDraw::CopyTo(DrawObjBase** para) const
	{
		HSV::LineDraw* ptr = dynamic_cast<HSV::LineDraw*>(*para);
		CopyTo(*ptr);
	}

	void LineDraw::Init()
	{
		pt1_ = Point(0, 0);
		pt2_ = Point(0, 0);
		DrawObjBase::Init();
		//DrawObjBase::Init();
	}

	//当前对象是否为空（主要用于判断当前区域是否为空）
	bool LineDraw::IsEmpty() const
	{
		return (pt1_ == pt2_);
	}

	//获取绘制图形类型
	HSV::DrawType LineDraw::GetDrawType() const
	{
		return DrawType::DRAW_LINE;
	}

	bool LineDraw::IsRoiReg() const
	{
		return false;
	}

	void LineDraw::GetCounterPoint(Circle* dash_point, int nCount)
	{
		int line_h = abs(pt1_.y - pt2_.y);
		int line_w = abs(pt1_.x - pt2_.x);
		int len = Lenth();
		double degAng = DegAng();
		double cos_val = cos(PI_HS * degAng / 180);
		double sin_val = sin(PI_HS * degAng / 180);
		double interval = (double)len / nCount;
		for (int i = 0;
			i <= nCount; i++)
		{
			dash_point[i] = Circle(pt1_.x + (int)(round)(i * interval * cos_val), pt1_.y + (int)(round)(i * interval *sin_val), 1);
		}
	}

	void LineDraw::GetLinePoint(LineDraw* dash_line, int nCount)
	{
		int line_h = abs(pt1_.y - pt2_.y);
		int line_w = abs(pt1_.x - pt2_.x);
		int len = Lenth();
		double degAng = DegAng();
		double cos_val = cos(PI_HS * degAng / 180);
		double sin_val = sin(PI_HS * degAng / 180);
		double interval = (double)len / nCount;
		for (int i = 0; i <= nCount; i++)
		{
			if (i % 2 == 0)
			{
				dash_line[i] = LineDraw(Point(pt1_.x + (int)(round)(i * interval * cos_val), pt1_.y + (int)(round)(i * interval * sin_val)),
					(int)round(interval), degAng);
				dash_line[i].color_ = color_;
			}
				
		}
	}

	void LineDraw::SetShapeData(Point p1, int Lengle, double Angle)
	{
#if 0
		degAng_ = Angle;
		len_ = Lengle;

		pt1_ = p1;

		int pt2_x = pt1_.x + (int)(round)(len_ * cos(degAng_ * PI_HS / 180));
		int pt2_y = pt1_.y + (int)(round)(len_ * sin(degAng_ * PI_HS / 180));
		pt2_ = Point(pt2_x, pt2_y);
#endif // 0
		
	}

	void LineDraw::SetShapeData(Point p1, Point p2)
	{
		pt1_ = p1;
		pt2_ = p2;
	}

	int LineDraw::Lenth() const
	{
		return (int)sqrt(pow((pt2_.y - pt1_.y), 2) + pow((pt2_.x - pt1_.x), 2));
	}

	double LineDraw::DegAng() const
	{
		double k = (double)(pt2_.y - pt1_.y) / (double)(pt2_.x - pt1_.x);//斜率
		double theta = atan(k);//弧度
		double degAng = 0.0;
		//角度
		if ((pt2_.x - pt1_.x) >= 0)
			degAng = theta * 180 / PI_HS;
		else
			degAng = 180 + theta * 180 / PI_HS;
		return degAng;
	}

	void LineDraw::SetStartPt(Point pt)
	{
		pt1_ = pt;
	}

	void LineDraw::SetEndPt(Point pt)
	{
		pt2_ = pt;
	}

	Point LineDraw::GetStartPt()
	{
		return pt1_;
	}
	Point LineDraw::GetEndPt()
	{
		return pt2_;
	}
#pragma endregion


}
