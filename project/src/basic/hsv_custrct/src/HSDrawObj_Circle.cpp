//#include "stdafx.h"
#include"HSDrawObj.h"
#include<assert.h>
#include<algorithm>
#include <vector>

namespace HSV
{

#pragma region 绘图对象_圆形
	void CircleDraw::Init()
	{
		Circle::Init();
		DrawObjBase::Init();
	}
	CircleDraw::CircleDraw()
	{
		Init();
	}
	CircleDraw::CircleDraw(const CircleDraw& para)
	{
		Init();
		CopyFrom(para);
	}

	CircleDraw::CircleDraw(Point cnter, int r, GC_COL clrType, int thickness, int lineType, int shift)
		: Circle(cnter, r), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Circle(cnter, r);
			DrawObjBase::SetDrawDat(clrType, thickness, lineType, shift);
		}
	}
	CircleDraw::CircleDraw(Point cnter, int r, ScalarGC color, int thickness, int lineType, int shift)
		: Circle(cnter, r), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Circle(cnter, r);
			DrawObjBase::SetDrawDat(color, thickness, lineType, shift);
		}
	}

	CircleDraw::CircleDraw(int x, int y, int r, GC_COL clrType, int thickness, int lineType, int shift)
		: Circle(x, y, r), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Circle(x, y, r);
			DrawObjBase::SetDrawDat(clrType, thickness, lineType, shift);
		}
	}
	
	CircleDraw::CircleDraw(int x, int y, int r, ScalarGC color, int thickness, int lineType, int shift)
		: Circle(x, r, r), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Circle(x, y, r);
			DrawObjBase::SetDrawDat(color, thickness, lineType, shift);
		}
	}

	CircleDraw::~CircleDraw()
	{

	}
	CircleDraw& CircleDraw::operator = (const CircleDraw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}
	bool CircleDraw::equal(const DrawObjBase &obj) const
	{
		// 检查当前对象的指针是否为空
		if (this == nullptr) {
			// 处理空指针情况，例如可以返回 false 或抛出异常
			return false;
		}
		bool bEqual = false;
		if (this->GetDrawType() == obj.GetDrawType()/* && typeid(this) == typeid(&obj)*/)
		{
			if (const Circle *ptr = dynamic_cast<const Circle*>(&obj))
			{
				if (this->nID_ >= 0)
				{
					return (this->nID_ == obj.nID_);
				}
				else
				{
					bEqual = Circle::equal(*ptr);
				}	
			}
		}
		return bEqual;
	}

	bool CircleDraw::operator==(const CircleDraw & obj)
	{
		return equal(obj);
	}
	void CircleDraw::CopyFrom(const CircleDraw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	void CircleDraw::CopyTo(CircleDraw& para) const
	{
		if (this != &para)
		{
			Circle::CopyTo(para);
			DrawObjBase::CopyTo(para);
		}
	}
	//从para拷贝数据	
	void CircleDraw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::CircleDraw* ptr = dynamic_cast<const HSV::CircleDraw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void CircleDraw::CopyTo(DrawObjBase** para) const
	{
		HSV::CircleDraw* ptr = dynamic_cast<HSV::CircleDraw*>(*para);
		CopyTo(*ptr);
	}

	//当前对象是否为空（主要用于判断当前区域是否为空）
	bool CircleDraw::IsEmpty() const
	{
		return Circle::IsEmpty();
	}

	HSV::DrawType CircleDraw::GetDrawType() const//获取绘制图形类型
	{
		return DrawType::DRAW_CIRCLE;
	}

	bool CircleDraw::IsRoiReg() const
	{
		return true;
	}

#pragma endregion

}
