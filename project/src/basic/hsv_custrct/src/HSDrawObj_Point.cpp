//#include "stdafx.h"
#include"HSDrawObj.h"
#include<assert.h>
#include<algorithm>
#include <vector>

namespace HSV
{

#pragma region 绘图对象_Point点
	PointDraw::PointDraw()
	{
		Init();
	}

	PointDraw::PointDraw(const PointDraw& para)
	{
		Init();
		CopyFrom(para);
	}

	PointDraw::PointDraw(Point pt, GC_COL clrType, int thickness, int lineType, int shift)
	{
		point_ = pt;

		this->SetColor(clrType);
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
	}
	PointDraw::PointDraw(Point pt, ScalarGC color, int thickness, int lineType, int shift)
	{
		point_ = pt;
		
		color_ = color;
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
	}
	PointDraw::PointDraw(int x, int y, GC_COL clrType, int thickness, int lineType, int shift)
	{
		point_.x = x;
		point_.y = y;

		this->SetColor(clrType);
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
	}
	PointDraw::PointDraw(int x, int y, ScalarGC color, int thickness, int lineType, int shift)
	{
		point_.x = x;
		point_.y = y;

		color_ = color;
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
	}


	PointDraw::~PointDraw()
	{

	}

	PointDraw& PointDraw::operator = (const PointDraw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool PointDraw::equal(const DrawObjBase &obj) const
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
				if (const PointDraw *ptr = dynamic_cast<const PointDraw*>(&obj))
				{
					if (this->point_ == ptr->point_)
					{
						bEqual = true;
					}
				}
			}
		}
		return bEqual;
	}

	bool PointDraw::operator==(const PointDraw & obj)
	{
		return equal(obj);
	}

	//从para拷贝数据
	void PointDraw::CopyFrom(const PointDraw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void PointDraw::CopyTo(PointDraw& para) const
	{
		if (this != &para)
		{
			DrawObjBase::CopyTo(para);
			para.point_ = point_;
		}
	}

	//从para拷贝数据	
	void PointDraw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::PointDraw* ptr = dynamic_cast<const HSV::PointDraw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void PointDraw::CopyTo(DrawObjBase** para) const
	{
		HSV::PointDraw* ptr = dynamic_cast<HSV::PointDraw*>(*para);
		CopyTo(*ptr);
	}

	void PointDraw::Init()
	{
		point_ = Point(0,0);
		DrawObjBase::Init();
	}

	//当前对象是否为空（主要用于判断当前区域是否为空）
	bool PointDraw::IsEmpty() const
	{
		return false;
	}

	//获取绘制图形类型
	HSV::DrawType PointDraw::GetDrawType() const
	{
		return DrawType::DRAW_POINT;
	}

	bool PointDraw::IsRoiReg() const
	{
		return false;
	}

#pragma endregion


}
