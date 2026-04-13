//#include "stdafx.h"
#include"HSDrawObj.h"
#include<assert.h>
#include<algorithm>
#include <vector>

namespace HSV
{
	
#pragma region 绘图对象_Contours点集
	ContoursDraw::ContoursDraw()
	{
		Init();
	}

	ContoursDraw::ContoursDraw(const ContoursDraw& para)
	{
		Init();
		CopyFrom(para);
	}

	ContoursDraw::ContoursDraw(const std::vector<Point>& points, GC_COL clrType, int thickness, int lineType, int shift)
	{
		//int xywh[4];
		points_ = points;
		//boundingRect(xywh);
		//cnter_.x = xywh[0] + xywh[2] / 2;
		//cnter_.y = xywh[1] + xywh[3] / 2;

		this->SetColor(clrType);
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
	}

	ContoursDraw::ContoursDraw(const std::vector<Point>& points, ScalarGC color, int thickness, int lineType, int shift)
	{
		//int xywh[4];
		points_ = points;
		//boundingRect(xywh);
		//cnter_.x = xywh[0] + xywh[2] / 2;
		//cnter_.y = xywh[1] + xywh[3] / 2;

		color_ = color;
		thickness_ = thickness;
		lineType_ = lineType;
		shift_ = shift;
	}

	ContoursDraw::~ContoursDraw()
	{

	}

	ContoursDraw& ContoursDraw::operator = (const ContoursDraw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool ContoursDraw::equal(const DrawObjBase &obj) const
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
				if (const ContoursDraw *ptr = dynamic_cast<const ContoursDraw*>(&obj))
				{
					if (this->points_ == ptr->points_)
					{
						bEqual = true;
					}
				}
			}
		}
		return bEqual;
	}

	bool ContoursDraw::operator==(const ContoursDraw & obj)
	{
		return equal(obj);
	}

	//从para拷贝数据
	void ContoursDraw::CopyFrom(const ContoursDraw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void ContoursDraw::CopyTo(ContoursDraw& para) const
	{
		if (this != &para)
		{
			DrawObjBase::CopyTo(para);
			para.points_ = points_;
		}
	}

	//从para拷贝数据	
	void ContoursDraw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::ContoursDraw* ptr = dynamic_cast<const HSV::ContoursDraw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void ContoursDraw::CopyTo(DrawObjBase** para) const
	{
		HSV::ContoursDraw* ptr = dynamic_cast<HSV::ContoursDraw*>(*para);
		CopyTo(*ptr);
	}

	void ContoursDraw::Init()
	{
		points_ = {};
		DrawObjBase::Init();
	}

	//当前对象是否为空（主要用于判断当前区域是否为空）
	bool ContoursDraw::IsEmpty() const
	{
		return (points_.size() == 0);
	}

	//获取绘制图形类型
	HSV::DrawType ContoursDraw::GetDrawType() const
	{
		return DrawType::DRAW_CONTOURS;
	}

	bool ContoursDraw::IsRoiReg() const
	{
		return false;
	}

#pragma endregion


}
