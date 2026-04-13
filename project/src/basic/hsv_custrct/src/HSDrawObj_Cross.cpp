//#include "stdafx.h"
#include"HSDrawObj.h"
#include<assert.h>
#include<algorithm>
#include <vector>
#include <cmath>

namespace HSV
{
	
#pragma region 绘图对象_Cross十字叉
	CrossDraw::CrossDraw()
	{
		Init();
	}

	CrossDraw::CrossDraw(const CrossDraw& para)
	{
		Init();
		CopyFrom(para);
	}

	CrossDraw::CrossDraw(Point cnter, int len, double degAng, GC_COL clrType, int thickness, int lineType, int shift)
	{
		cnter_ = cnter;
		degAng_ = degAng;
		horLen_ = len;
		verLen_ = len;
		this->SetColor(clrType);
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;

	}

	CrossDraw::CrossDraw(Point cnter, int len, double degAng, ScalarGC color, int thickness, int lineType, int shift)
	{
		cnter_ = cnter;
		degAng_ = degAng;
		horLen_ = len;
		verLen_ = len;
		color_ = color;
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;

	}
	CrossDraw::CrossDraw(Point cnter, int horLen, int verLen, double degAng, GC_COL clrType, int thickness, int lineType, int shift)
	{
		cnter_ = cnter;
		degAng_ = degAng;
		horLen_ = horLen;
		verLen_ = verLen;
		this->SetColor(clrType);
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
	}

	CrossDraw::CrossDraw(Point cnter, int horLen, int verLen, double degAng, ScalarGC color, int thickness, int lineType, int shift)
	{
		cnter_ = cnter;
		degAng_ = degAng;
		horLen_ = horLen;
		verLen_ = verLen;
		color_ = color;
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
	}

	CrossDraw::~CrossDraw()
	{

	}
	CrossDraw& CrossDraw::operator = (const CrossDraw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}
	bool CrossDraw::equal(const DrawObjBase &obj) const
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
				if (const CrossDraw *ptr = dynamic_cast<const CrossDraw*>(&obj))
				{
					if (this->cnter_ == ptr->cnter_	  &&
						this->horLen_ == ptr->horLen_ &&
						this->verLen_ == ptr->verLen_ &&
						this->degAng_ == ptr->degAng_)
					{
						bEqual = true;
					}
				}
			}
		}
		return bEqual;
	}
	bool CrossDraw::operator==(const CrossDraw & obj)
	{
		return equal(obj);
	}

	CrossDraw CrossDraw::operator+(const Point& point)
	{
		CrossDraw cross(*this);
		cross.cnter_.x += point.x;
		cross.cnter_.y += point.y;
		return cross;
	}

	CrossDraw CrossDraw::operator-(const Point& point)
	{
		CrossDraw cross(*this);
		cross.cnter_.x -= point.x;
		cross.cnter_.y -= point.y;
		return cross;
	}

	//从para拷贝数据
	void CrossDraw::CopyFrom(const CrossDraw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void CrossDraw::CopyTo(CrossDraw& para) const
	{
		if (this != &para)
		{
			DrawObjBase::CopyTo(para);
			para.cnter_ = cnter_;
			para.degAng_ = degAng_;
			para.horLen_ = horLen_;
			para.verLen_ = verLen_;
		}
	}

	//从para拷贝数据	
	void CrossDraw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::CrossDraw* ptr = dynamic_cast<const HSV::CrossDraw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void CrossDraw::CopyTo(DrawObjBase** para) const
	{
		HSV::CrossDraw* ptr = dynamic_cast<HSV::CrossDraw*>(*para);
		CopyTo(*ptr);
	}

	void CrossDraw::Init()
	{
		cnter_ = Point(0, 0);
		degAng_ = 0;
		horLen_ = 0;
		verLen_ = 0;
		DrawObjBase::Init();
	}

	//当前对象是否为空（主要用于判断当前区域是否为空）
	bool CrossDraw::IsEmpty() const
	{
		return (horLen_ <= 0  || verLen_ <= 0);
	}

	//获取绘制图形类型
	HSV::DrawType CrossDraw::GetDrawType() const
	{
		return DrawType::DRAW_CROSS;
	}

	bool CrossDraw::IsRoiReg() const
	{
		return false;
	}

	void CrossDraw::points(Point point[]) const
	{
		Point hor_pt1 = Point(cnter_.x + horLen_, cnter_.y);
		Point hor_pt2 = Point(cnter_.x - horLen_, cnter_.y);

		int newhor_pt1_x = (int)(round)((hor_pt1.x - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (hor_pt1.y - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
		int newhor_pt1_y = (int)(round)((hor_pt1.x - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (hor_pt1.y - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);

		int newhor_pt2_x = (int)(round)((hor_pt2.x - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (hor_pt2.y - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
		int newhor_pt2_y = (int)(round)((hor_pt2.x - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (hor_pt2.y - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);

		hor_pt1 = Point(newhor_pt1_x, newhor_pt1_y);
		hor_pt2 = Point(newhor_pt2_x, newhor_pt2_y);


		Point ver_pt1 = Point(cnter_.x, cnter_.y - verLen_);
		Point ver_pt2 = Point(cnter_.x, cnter_.y + verLen_);

		int newver_pt1_x = (int)(round)((ver_pt1.x - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (ver_pt1.y - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
		int newver_pt1_y = (int)(round)((ver_pt1.x - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (ver_pt1.y - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);

		int newver_pt2_x = (int)(round)((ver_pt2.x - cnter_.x) * cos(-degAng_ * PI_HS / 180) - (ver_pt2.y - cnter_.y) * sin(-degAng_ * PI_HS / 180) + cnter_.x);
		int newver_pt2_y = (int)(round)((ver_pt2.x - cnter_.x) * sin(-degAng_ * PI_HS / 180) + (ver_pt2.y - cnter_.y) * cos(-degAng_ * PI_HS / 180) + cnter_.y);

		ver_pt1 = Point(newver_pt1_x, newver_pt1_y);
		ver_pt2 = Point(newver_pt2_x, newver_pt2_y);

		point[0] = hor_pt1;
		point[1] = ver_pt1;
		point[2] = hor_pt2;
		point[3] = ver_pt2;
	}

	void CrossDraw::lines(LineDraw Line[]) const
	{
		Point point[4];
		this->points(point);
		LineDraw horLine = LineDraw(point[0], point[2], color_, thickness_, lineType_, shift_);
		LineDraw verLine = LineDraw(point[1], point[3], color_, thickness_, lineType_, shift_);
		Line[0] = horLine;
		Line[1] = verLine;
	}

#pragma endregion


}
