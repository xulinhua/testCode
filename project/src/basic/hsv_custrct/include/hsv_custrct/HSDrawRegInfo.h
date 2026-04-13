/*/////////////////////////////////////////////////////////////////////////
显示图像，区域等信息
/////////////////////////////////////////////////////////////////////////*/
#pragma once
#include"HSDrawObj.h"

#ifndef HSV_CUSTRCT_EXPORTS
#define HSV_CUSTRCT_EXPORTS
#endif
#include "platform_defines.h"

#ifdef HSV_CUSTRCT_EXPORTS
#define GEOMETRY_API HSV_CUSTRCT_API
#else
#define GEOMETRY_API HSV_CUSTRCT_API
#endif

namespace HSV
{

	class GEOMETRY_API DrawRegInfo
	{
	public:
		HSV::DrawObjBase* pReg = NULL;
		HSV::ScalarGC colorUnSelect_;//未选中区域颜色
		HSV::ScalarGC colorSelect_;//选中区域颜色
		std::vector<HSV::PointDraw> ptDraw1D_;
		HSV::DrawObjPtr1D pExObj1D_;//额外信息,包括索引，测量直线等
		bool bCanbeSelected;
		bool IsSelected;

	public:
		DrawRegInfo();
		DrawRegInfo(const DrawRegInfo& para);
		~DrawRegInfo();
		void Init();
		DrawRegInfo& operator = (const DrawRegInfo& para);
		bool equal(const DrawRegInfo &obj) const;
		bool operator==(const DrawRegInfo & obj);
		void CopyFrom(const DrawRegInfo& para);
		void CopyTo(DrawRegInfo& para) const;
	};

}