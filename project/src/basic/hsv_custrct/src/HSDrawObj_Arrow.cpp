//#include "stdafx.h"
#include"HSDrawObj.h"
#include<assert.h>
#include<algorithm>
#include <vector>
#include <cmath>
#include <cmath>

namespace HSV
{
	
#pragma region 绘图对象_Arrow箭头
	ArrowDraw::ArrowDraw()
	{
		Init();
	}

	ArrowDraw::ArrowDraw(const ArrowDraw& para)
	{
		Init();
		CopyFrom(para);
	}

	ArrowDraw::ArrowDraw(Point p1, Point p2, GC_COL clrType, int thickness, int lineType, int shift, double tipLength)
	{
		pt1_ = p1;
		pt2_ = p2;
		//this->SetColor(clrType);
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
		tipLen_ = tipLength;
		//斜率
		double k = (double)(pt2_.y - pt1_.y) / (double)(pt2_.x - pt1_.x);

		//弧度
		double theta = atan(k);

		//角度
		if ((pt2_.x - pt1_.x) >= 0)
			degAng_ = theta * 180 / PI_HS;
		else
			degAng_ = 180 + theta * 180 / PI_HS;
		len_ = (int)sqrt(pow((pt2_.y - pt1_.y), 2) + pow((pt2_.x - pt1_.x), 2));
	}

	ArrowDraw::ArrowDraw(Point p1, Point p2, ScalarGC color, int thickness, int lineType, int shift, double tipLength)
	{
		pt1_ = p1;
		pt2_ = p2;
		color_ = color;
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
		tipLen_ = tipLength;
		//斜率
		double k = (double)(pt2_.y - pt1_.y) / (double)(pt2_.x - pt1_.x);

		//弧度
		double theta = atan(k);

		//角度
		if ((pt2_.x - pt1_.x) >= 0)
			degAng_ = theta * 180 / PI_HS;
		else
			degAng_ = 180 + theta * 180 / PI_HS;
		len_ = (int)sqrt(pow((pt2_.y - pt1_.y), 2) + pow((pt2_.x - pt1_.x), 2));
	}

	ArrowDraw::ArrowDraw(Point p1, int len, double degAng, GC_COL clrType, int thickness, int lineType, int shift, double tipLength)
	{
		this->SetColor(clrType);
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
		len_ = len;
		degAng_ = degAng;
		tipLen_ = tipLength;
		pt1_ = p1;
		int pt2_x = pt1_.x + (int)(round)(len_ * cos(degAng_ * PI_HS / 180));
		int pt2_y = pt1_.y + (int)(round)(len_ * sin(degAng_ * PI_HS / 180));
		pt2_ = Point(pt2_x, pt2_y);
	}
	
	ArrowDraw::ArrowDraw(Point p1, int len, double degAng, ScalarGC color, int thickness, int lineType, int shift, double tipLength)
	{
		color_ = color;
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
		len_ = len;
		degAng_ = degAng;
		tipLen_ = tipLength;
		pt1_ = p1;
		int pt2_x = pt1_.x + (int)(round)(len_ * cos(degAng_ * PI_HS / 180));
		int pt2_y = pt1_.y + (int)(round)(len_ * sin(degAng_ * PI_HS / 180));
		pt2_ = Point(pt2_x, pt2_y);
	}

	ArrowDraw::~ArrowDraw()
	{

	}
	ArrowDraw& ArrowDraw::operator = (const ArrowDraw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}
	bool ArrowDraw::equal(const DrawObjBase &obj) const
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
				if (const ArrowDraw *ptr = dynamic_cast<const ArrowDraw*>(&obj))
				{
					if (this->pt1_ == ptr->pt1_	      &&
						this->pt2_ == ptr->pt2_       &&
						this->len_ == ptr->len_	      &&
						this->degAng_ == ptr->degAng_ &&
						this->tipLen_ == ptr->tipLen_)
					{
						bEqual = true;
					}
				}
			}
		}
		return bEqual;
	}
	bool ArrowDraw::operator==(const ArrowDraw & obj)
	{
		return equal(obj);
	}

	ArrowDraw ArrowDraw::operator+(const Point& point)
	{
		ArrowDraw aro(*this);
		aro.pt1_.x += point.x;
		aro.pt1_.y += point.y;
		aro.pt2_.x += point.x;
		aro.pt2_.y += point.y;
		return aro;
	}

	ArrowDraw ArrowDraw::operator-(const Point& point)
	{
		ArrowDraw aro(*this);
		aro.pt1_.x -= point.x;
		aro.pt1_.y -= point.y;
		aro.pt2_.x -= point.x;
		aro.pt2_.y -= point.y;
		return aro;
	}

	//从para拷贝数据
	void ArrowDraw::CopyFrom(const ArrowDraw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void ArrowDraw::CopyTo(ArrowDraw& para) const
	{
		if (this != &para)
		{
			DrawObjBase::CopyTo(para);
			para.pt1_ = pt1_;
			para.pt2_ = pt2_;
			para.degAng_ = degAng_;
			para.len_ = len_;
			para.tipLen_ = tipLen_;
		}
	}

	//从para拷贝数据	
	void ArrowDraw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::ArrowDraw* ptr = dynamic_cast<const HSV::ArrowDraw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void ArrowDraw::CopyTo(DrawObjBase** para) const
	{
		HSV::ArrowDraw* ptr = dynamic_cast<HSV::ArrowDraw*>(*para);
		CopyTo(*ptr);
	}

	void ArrowDraw::Init()
	{
		pt1_ = Point(0, 0);
		pt2_ = Point(0, 0);
		len_ = 0;
		degAng_ = 0;
		tipLen_ = 0;
		DrawObjBase::Init();
	}

	//当前对象是否为空（主要用于判断当前区域是否为空）
	bool ArrowDraw::IsEmpty() const
	{
		return (pt1_ == pt2_);
	}

	//获取绘制图形类型
	HSV::DrawType ArrowDraw::GetDrawType() const
	{
		return DrawType::DRAW_ARROW;
	}

	bool ArrowDraw::IsRoiReg() const
	{
		return false;
	}

	void ArrowDraw::SetShapeData(Point p1, int Lengle, double Angle)
	{
		degAng_ = Angle;
		len_ = Lengle;

		pt1_ = p1;

		int pt2_x = pt1_.x + (int)(round)(len_ * cos(degAng_ * PI_HS / 180));
		int pt2_y = pt1_.y + (int)(round)(len_ * sin(degAng_ * PI_HS / 180));
		pt2_ = Point(pt2_x, pt2_y);
	}

	void ArrowDraw::SetShapeData(Point p1, Point p2)
	{
		pt1_ = p1;
		pt2_ = p2;

		//斜率
		double k = (double)(pt2_.y - pt1_.y) / (double)(pt2_.x - pt1_.x);

		//弧度
		double theta = atan(k);

		//角度
		if ((pt2_.x - pt1_.x) >= 0)
			degAng_ = theta * 180 / PI_HS;
		else
			degAng_ = 180 + theta * 180 / PI_HS;
		len_ = (int)sqrt(pow((pt2_.y - pt1_.y), 2) + pow((pt2_.x - pt1_.x), 2));
	}

	void ArrowDraw::SetLeng(int Lengle)
	{
		len_ = Lengle;

		int pt2_x = pt1_.x + (int)(round)(len_ * cos(degAng_ * PI_HS / 180));
		int pt2_y = pt1_.y + (int)(round)(len_ * sin(degAng_ * PI_HS / 180));
		pt2_ = Point(pt2_x, pt2_y);
	}

	void ArrowDraw::SetDegAng(double angle)
	{
		degAng_ = angle;

		int pt2_x = pt1_.x + (int)(round)(len_ * cos(degAng_ * PI_HS / 180));
		int pt2_y = pt1_.y + (int)(round)(len_ * sin(degAng_ * PI_HS / 180));
		pt2_ = Point(pt2_x, pt2_y);
	}

	void ArrowDraw::SetStartPt(Point pt)
	{
		pt1_ = pt;

		//斜率
		double k = (double)(pt2_.y - pt1_.y) / (double)(pt2_.x - pt1_.x);

		//弧度
		double theta = atan(k);

		//角度
		if ((pt2_.x - pt1_.x) >= 0)
			degAng_ = theta * 180 / PI_HS;
		else
			degAng_ = 180 + theta * 180 / PI_HS;
		len_ = (int)sqrt(pow((pt2_.y - pt1_.y), 2) + pow((pt2_.x - pt1_.x), 2));
	}

	void ArrowDraw::SetEndPt(Point pt)
	{
		pt2_ = pt;

		//斜率
		double k = (double)(pt2_.y - pt1_.y) / (double)(pt2_.x - pt1_.x);

		//弧度
		double theta = atan(k);

		//角度
		if ((pt2_.x - pt1_.x) >= 0)
			degAng_ = theta * 180 / PI_HS;
		else
			degAng_ = 180 + theta * 180 / PI_HS;
		len_ = (int)sqrt(pow((pt2_.y - pt1_.y), 2) + pow((pt2_.x - pt1_.x), 2));
	}

	int ArrowDraw::GetLeng()
	{
		return len_;
	}

	double ArrowDraw::GetDegAng()
	{
		return degAng_;
	}

	Point ArrowDraw::GetStartPt()
	{
		return pt1_;
	}
	Point ArrowDraw::GetEndPt()
	{
		return pt2_;
	}
#pragma endregion


}
