//#include "stdafx.h"
#include"HSDrawObj.h"
#include<assert.h>
#include<algorithm>
#include <vector>

namespace HSV
{
	
#pragma region 绘图对象_任意多边形
	PolygonDraw::PolygonDraw()
	{
		Init();
	}

	PolygonDraw::PolygonDraw(const PolygonDraw& para)
	{
		Init();
		CopyFrom(para);
	}

	PolygonDraw::PolygonDraw(Point* points, int N, GC_COL clrType, int thickness, int lineType, int shift)
		: Polygon(points, N), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Polygon(points, N);
			DrawObjBase::SetDrawDat(clrType, thickness, lineType, shift);
		}
	}

	PolygonDraw::PolygonDraw(Point* points, int N, ScalarGC color, int thickness, int lineType, int shift)
		: Polygon(points, N), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Polygon(points, N);
			DrawObjBase::SetDrawDat(color, thickness, lineType, shift);
		}
	}

	PolygonDraw::~PolygonDraw()
	{

	}

	PolygonDraw& PolygonDraw::operator = (const PolygonDraw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool PolygonDraw::equal(const DrawObjBase &obj) const
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
				if (const Polygon *ptr = dynamic_cast<const Polygon*>(&obj))
				{
					bEqual = Polygon::equal(*ptr);
				}
			}
		}
		return bEqual;
	}

	bool PolygonDraw::operator==(const PolygonDraw & obj)
	{
		return equal(obj);
	}

	//从para拷贝数据
	void PolygonDraw::CopyFrom(const PolygonDraw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void PolygonDraw::CopyTo(PolygonDraw& para) const
	{
		if (this != &para)
		{
			Polygon::CopyTo(para);
			DrawObjBase::CopyTo(para);
		}
	}

	//从para拷贝数据	
	void PolygonDraw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::PolygonDraw* ptr = dynamic_cast<const HSV::PolygonDraw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void PolygonDraw::CopyTo(DrawObjBase** para) const
	{
		HSV::PolygonDraw* ptr = dynamic_cast<HSV::PolygonDraw*>(*para);
		CopyTo(*ptr);
	}

	void PolygonDraw::Init()
	{
		Polygon::Init();
		DrawObjBase::Init();
	}

	//当前对象是否为空（主要用于判断当前区域是否为空）
	bool PolygonDraw::IsEmpty() const
	{
		return Polygon::IsEmpty();
	}

	//获取绘制图形类型
	HSV::DrawType PolygonDraw::GetDrawType() const
	{
		return DrawType::DRAW_POLYGON;
	}

	bool PolygonDraw::IsRoiReg() const
	{
		return true;
	}
#pragma endregion
}
