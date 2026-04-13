//#include "stdafx.h"
#include"HSDrawObj.h"
#include<assert.h>
#include<algorithm>
#include <vector>

namespace HSV
{
	
#pragma region 绘图对象_Rect类（不带角度矩形）
	void RectDraw::Init()
	{
		isDash_ = false;  //是否为虚线  luojianghong 23-9-4
		nCount_ = 20;  //线段比例    luojianghong 23-9-4
		Rect::Init();
		DrawObjBase::Init();
	}
	RectDraw::RectDraw()
	{
		Init();
	}
	RectDraw::RectDraw(const RectDraw& para)
	{
		Init();
		CopyFrom(para);
	}

	RectDraw::RectDraw(Point ltTopPt, int w, int h, GC_COL clrType, int thickness, int lineType, int shift)
		: Rect(ltTopPt, w, h), DrawObjBase()
	{
		bool bTest = true;
		isDash_ = false;  //是否为虚线
		nCount_ = 20;  //线段比例    l
		if (bTest)
		{//待简化
			//Rect(ltTopPt, w, h);
			DrawObjBase::SetDrawDat(clrType, thickness, lineType, shift);
		}
	}
	RectDraw::RectDraw(Point ltTopPt, int w, int h, ScalarGC color, int thickness, int lineType, int shift)
		: Rect(ltTopPt, w, h), DrawObjBase()
	{
		bool bTest = true;
		isDash_ = false;  //是否为虚线
		nCount_ = 20;  //线段比例    l
		if (bTest)
		{//待简化
			//Rect(ltTopPt, w, h);
			DrawObjBase::SetDrawDat(color, thickness, lineType, shift);
		}
	}
	RectDraw::RectDraw(int x, int y, int w, int h, GC_COL clrType, int thickness, int lineType, int shift)
		: Rect(x, y, w, h), DrawObjBase()
	{
		bool bTest = true;
		isDash_ = false;  //是否为虚线
		nCount_ = 20;  //线段比例    l
		if (bTest)
		{//待简化
			//Rect(x, y, w, h);
			DrawObjBase::SetDrawDat(clrType, thickness, lineType, shift);
		}
	}
	RectDraw::RectDraw(int x, int y, int w, int h, ScalarGC color, int thickness, int lineType, int shift)
		: Rect(x, y, w, h), DrawObjBase()
	{
		bool bTest = true;
		isDash_ = false;  //是否为虚线
		nCount_ = 20;  //线段比例    l
		if (bTest)
		{//待简化
			//Rect(x, y, w, h);
			DrawObjBase::SetDrawDat(color, thickness, lineType, shift);
		}
	}

	RectDraw::~RectDraw()
	{

	}
	RectDraw& RectDraw::operator = (const RectDraw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}
	bool RectDraw::equal(const DrawObjBase &obj) const
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
				if (const Rect *ptr = dynamic_cast<const Rect*>(&obj))
				{
					const RectDraw *ptrRect = dynamic_cast<const RectDraw*>(&obj);
					bEqual = Rect::equal(*ptr)&& ptrRect->isDash_==isDash_ &&ptrRect->nCount_ == nCount_;
				}
			}
		}
		return bEqual;
	}

	bool RectDraw::operator==(const RectDraw & obj)
	{
		return equal(obj);
	}
	void RectDraw::CopyFrom(const RectDraw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	void RectDraw::CopyTo(RectDraw& para) const
	{
		if (this != &para)
		{
			para.isDash_ = isDash_;
			para.nCount_ = nCount_;
			Rect::CopyTo(para);
			DrawObjBase::CopyTo(para);
			//DrawObjBase::CopyTo(para);
		}
	}
	//从para拷贝数据	
	void RectDraw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::RectDraw* ptr = dynamic_cast<const HSV::RectDraw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void RectDraw::CopyTo(DrawObjBase** para) const
	{
		HSV::RectDraw* ptr = dynamic_cast<HSV::RectDraw*>(*para);
		CopyTo(*ptr);
	}

	//当前对象是否为空（主要用于判断当前区域是否为空）
	bool RectDraw::IsEmpty() const
	{
		return Rect::IsEmpty();
	}

	HSV::DrawType RectDraw::GetDrawType() const//获取绘制图形类型
	{
		return DrawType::DRAW_RECT;
	}

	bool RectDraw::IsRoiReg() const
	{
		return true;
	}
#pragma endregion


#pragma region 区域类--（不带角度）矩形

#pragma endregion
#pragma region 区域类--（不带角度）矩形

#pragma endregion

}
