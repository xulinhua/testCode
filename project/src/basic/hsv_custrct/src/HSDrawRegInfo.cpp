//#include "stdafx.h"
#include"HSDrawRegInfo.h"
#include<assert.h>

namespace HSV
{
	DrawRegInfo::DrawRegInfo()
	{
		pReg = NULL;
		pExObj1D_.clear();
		Init();
	}

	DrawRegInfo::DrawRegInfo(const DrawRegInfo& para)
	{
		pReg = NULL;
		pExObj1D_.clear();
		Init();
		CopyFrom(para);
	}

	DrawRegInfo::~DrawRegInfo()
	{
		HSV::ClearDrawObj(&pReg);
		HSV::ClearDrawObj(pExObj1D_);
	}

	void DrawRegInfo::Init()
	{
		HSV::ClearDrawObj(&pReg);
		HSV::ClearDrawObj(pExObj1D_);
		colorSelect_ = HSV::ScalarGC(0, 255, 0);//选中区域颜色
		colorUnSelect_ = HSV::ScalarGC(0, 0, 255);//未选中区域颜色
		ptDraw1D_.clear();
		bCanbeSelected = false;
		IsSelected = false;
	}

	DrawRegInfo& DrawRegInfo::operator = (const DrawRegInfo& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}
	bool DrawRegInfo::equal(const DrawRegInfo &obj) const
	{
		// 检查当前对象的指针是否为空
		if (this == nullptr) {
			// 处理空指针情况，例如可以返回 false 或抛出异常
			return false;
		}

		bool bEqual = false;
		if (this->pReg == obj.pReg/* && typeid(this) == typeid(&obj)*/)
		{
			bEqual = true;
		}
		return bEqual;
	}
	bool DrawRegInfo::operator==(const DrawRegInfo & obj)
	{
		return equal(obj);
	}
	void DrawRegInfo::CopyFrom(const DrawRegInfo& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	void DrawRegInfo::CopyTo(DrawRegInfo& para) const
	{
		if (this != &para)
		{
			if (pReg)
				HSV::CopyDrawObjDat(pReg, &para.pReg);
			else
				HSV::ClearDrawObj(&para.pReg);
			para.bCanbeSelected = bCanbeSelected;
			para.IsSelected = IsSelected;
			para.ptDraw1D_ = ptDraw1D_;
			HSV::CopyDrawObjDat(pExObj1D_, para.pExObj1D_);
		}
	}
}
