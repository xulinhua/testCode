//#include "stdafx.h"
#include"HSDrawObj.h"
#include<assert.h>
#include<algorithm>
#include <vector>

namespace HSV
{

#pragma region 绘图对象_椭圆
	EllipseDraw::EllipseDraw()
	{
		Init();
	}
	EllipseDraw::EllipseDraw(const EllipseDraw& para)
	{
		Init();
		CopyFrom(para);
	}

	EllipseDraw::EllipseDraw(Point cnter, int longR, int shortR, double degAng, double startDegAng, double endDegAng,
		GC_COL clrType, int thickness, int lineType, int shift) 
		: Ellipse(cnter, longR, shortR, degAng, startDegAng, endDegAng), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Ellipse(cnter, longR, shortR, degAng, startDegAng, endDegAng);
			DrawObjBase::SetDrawDat(clrType, thickness, lineType, shift);
		}
	}

	EllipseDraw::EllipseDraw(Point cnter, int longR, int shortR, double degAng, double startDegAng, double endDegAng,
		ScalarGC color, int thickness, int lineType, int shift) 
		: Ellipse(cnter, longR, shortR, degAng, startDegAng, endDegAng), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Ellipse(cnter, longR, shortR, degAng, startDegAng, endDegAng);
			DrawObjBase::SetDrawDat(color, thickness, lineType, shift);
		}
	}


	EllipseDraw::EllipseDraw(int x, int y, int longR, int shortR, double degAng, double startDegAng, double endDegAng,
		GC_COL clrType, int thickness, int lineType, int shift) 
		: Ellipse(x, y, longR, shortR, degAng, startDegAng, endDegAng), DrawObjBase()
	{ 
		bool bTest = true;
		if (bTest)
		{//待简化
			//Ellipse(x, y, longR, shortR, degAng, startDegAng, endDegAng);
			DrawObjBase::SetDrawDat(clrType, thickness, lineType, shift);
		}
	}

	EllipseDraw::EllipseDraw(int x, int y, int longR, int shortR, double degAng, double startDegAng, double endDegAng,
		ScalarGC color, int thickness, int lineType, int shift) 
		: Ellipse(x, y, longR, shortR, degAng, startDegAng, endDegAng), DrawObjBase()
	{
		bool bTest = true;
		if (bTest)
		{//待简化
			//Ellipse(x, y, longR, shortR, degAng, startDegAng, endDegAng);
			DrawObjBase::SetDrawDat(color, thickness, lineType, shift);
		}
	}

	EllipseDraw::~EllipseDraw()
	{

	}
	EllipseDraw& EllipseDraw::operator = (const EllipseDraw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool EllipseDraw::equal(const DrawObjBase &obj) const
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
				if (const Ellipse *ptr = dynamic_cast<const Ellipse*>(&obj))
				{
					bEqual = Ellipse::equal(*ptr);
				}
			}
			
		}
		return bEqual;
	}

	bool EllipseDraw::operator==(const EllipseDraw & obj)
	{
		return equal(obj);
	}
	void EllipseDraw::CopyFrom(const EllipseDraw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	void EllipseDraw::CopyTo(EllipseDraw& para) const
	{
		if (this != &para)
		{
			Ellipse::CopyTo(para);
			DrawObjBase::CopyTo(para);
		}
	}

	//从para拷贝数据	
	void EllipseDraw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::EllipseDraw* ptr = dynamic_cast<const HSV::EllipseDraw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void EllipseDraw::CopyTo(DrawObjBase** para) const
	{
		HSV::EllipseDraw* ptr = dynamic_cast<HSV::EllipseDraw*>(*para);
		CopyTo(*ptr);
	}

	void EllipseDraw::Init()
	{
		Ellipse::Init();
		DrawObjBase::Init();
	}

	//当前对象是否为空（主要用于判断当前区域是否为空）
	bool EllipseDraw::IsEmpty() const
	{
		return Ellipse::IsEmpty();
	}

	HSV::DrawType EllipseDraw::GetDrawType() const//获取绘制图形类型
	{
		return DrawType::DRAW_ELLIPSE;
	}

	bool EllipseDraw::IsRoiReg() const
	{
		return true;
	}

#pragma endregion

}
