//#include "stdafx.h"
#include"HSPoint.hpp"
#include<assert.h>
#include<cmath>
//#include<algorithm>
#define FLOAT_MIN_VAL 0.00001
#define PIXEL_MIN_VAL 1.00001

namespace HSV
{
#pragma region 点类

#if 0//模板类接口实现放在头文件中
	template<typename _Tp> inline
		Point_<_Tp>::Point_()
		: x(0), y(0) {}

	template<typename _Tp> inline
		Point_<_Tp>::Point_(_Tp _x, _Tp _y)
		: x(_x), y(_y) {}

	template<typename _Tp> inline
		Point_<_Tp>::Point_(const Point_& pt)
		: x(pt.x), y(pt.y) {}

	template<typename _Tp> inline
		bool Point_<_Tp>::operator==(const Point_<_Tp>& s) const {
		return(s.x == this->x && s.y == this->y);
	}
	template<typename _Tp> inline
		bool Point_<_Tp>::operator < (const Point_<_Tp> pt) const{
		if (this->x == pt.x)
		{
			return (this->y < pt.y);
		}
		else
		{
			return (this->x < pt.x);
		}
	}
	template<typename _Tp> inline
		Point_<_Tp>& Point_<_Tp>::operator = (const Point_& pt)
	{
		x = pt.x;
		y = pt.y;
		return *this;
	}

	template<typename _Tp> inline
		Point_<_Tp> Point_<_Tp>::operator - (const Point_& pt) const
	{
		return Point_(x - pt.x, y - pt.y);
	}

	template<typename _Tp> inline
		Point_<_Tp> Point_<_Tp>::operator + (const Point_& pt) const
	{
		return Point_(x + pt.x, y + pt.y);
	}
#endif // 0


	//template<typename _Tp>  inline
	//Point3_<_Tp>::Point3_() : x(0), y(0), z(0) {}

	//template<typename _Tp> inline
	//Point3_<_Tp>::Point3_(_Tp _x, _Tp _y, _Tp _z) : x(_x), y(_y), z(_z) {}

#if 0//模板类接口实现放在头文件中
	template<typename _Tp> inline
	Point3_<_Tp>::Point3_(const Point3_& pt) : x(pt.x), y(pt.y), z(pt.z) {}

	template<typename _Tp>
	bool Point3_<_Tp>::operator==(const Point3_& s) const {
		return (x == s.x) && (y == s.y) && (z == s.z);
	}

	template<typename _Tp> inline
	bool Point3_<_Tp>::operator<(const Point3_& pt) const {
		return (x < pt.x) || ((x == pt.x) && (y < pt.y)) || ((x == pt.x) && (y == pt.y) && (z < pt.z));
	}

	template<typename _Tp> inline
	Point3_<_Tp>& Point3_<_Tp>::operator=(const Point3_& pt) {
		if (this != &pt) {
			x = pt.x;
			y = pt.y;
			z = pt.z;
		}
		return *this;
	}

	template<typename _Tp> inline
	Point3_<_Tp> Point3_<_Tp>::operator-(const Point3_& point) const {
		return Point3_(x - point.x, y - point.y, z - point.z);
	}

	template<typename _Tp> inline
	Point3_<_Tp> Point3_<_Tp>::operator+(const Point3_& point) const {
		return Point3_(x + point.x, y + point.y, z + point.z);
	}
#endif

	//计算某一点到线段的垂足点坐标
	Point2f GetFootPt(Point2f linePt1, Point2f linePt2, Point2f pt)
	{
		return GetFootPt(linePt1, linePt2, pt.x, pt.y);
	}

	//计算某一点到线段的垂足点坐标
	Point2f GetFootPt(Point2f linePt1, Point2f linePt2, float ptX, float ptY)
	{
		Point2f footPt;//某点pt到轴线的垂足点坐标
		if (fabs(linePt2.x - linePt1.x) < FLOAT_MIN_VAL)
		{
			footPt.x = linePt1.x;
			footPt.y = ptY;
		}
		else if (fabs(linePt2.y - linePt1.y) < FLOAT_MIN_VAL)
		{
			footPt.x = ptX;
			footPt.y = linePt1.y;
		}
		else
		{
			float k = (linePt2.y - linePt1.y)*1.0F / (linePt2.x - linePt1.x);
			footPt.x = (k * linePt1.x + ptX*1.0F / k + ptY - linePt1.y)*1.0F / (k + 1.0F / k);
			footPt.y = (footPt.x - linePt1.x)*k + linePt1.y;
		}
		return footPt;
	}

	//计算某一点到线段的垂足点坐标
	Point2f GetFootPt(float linePt1X, float linePt1Y, float linePt2X, float linePt2Y, float ptX, float ptY)
	{
		Point2f footPt;//某点pt到轴线的垂足点坐标
		if (fabs(linePt2X - linePt1X) < FLOAT_MIN_VAL)
		{
			footPt.x = linePt1X;
			footPt.y = ptY;
		}
		else if (fabs(linePt2Y - linePt1Y) < FLOAT_MIN_VAL)
		{
			footPt.x = ptX;
			footPt.y = linePt1Y;
		}
		else
		{
			float k = (linePt2Y - linePt1Y)*1.0F / (linePt2X - linePt1X);
			footPt.x = (k * linePt1X + ptX*1.0F / k + ptY - linePt1Y)*1.0F / (k + 1.0F / k);
			footPt.y = (footPt.x - linePt1X)*k + linePt1Y;
		}
		return footPt;
	}

	//计算某一点到线段的垂足点坐标
	Point2f GetFootPt(Point2i linePt1, Point2i linePt2, Point2i pt)
	{
		return GetFootPt(linePt1, linePt2, pt.x, pt.y);
	}

	//计算某一点到线段的垂足点坐标
	Point2f GetFootPt(Point2i linePt1, Point2i linePt2, int ptX, int ptY)
	{
		return GetFootPt(linePt1.x, linePt1.y, linePt2.x, linePt2.y, ptX, ptY);
	}

	//计算某一点到线段的垂足点坐标
	Point2f GetFootPt(int linePt1X, int linePt1Y, int linePt2X, int linePt2Y, int ptX, int ptY)
	{
		Point2f footPt;//某点pt到轴线的垂足点坐标
		if (linePt2X == linePt1X)
		{
			footPt.x = (float)linePt1X;
			footPt.y = (float)ptY;
		}
		else if (linePt2Y == linePt1Y)
		{
			footPt.x = (float)ptX;
			footPt.y = (float)linePt1Y;
		}
		else
		{
			float k = (linePt2Y - linePt1Y)*1.0F / (linePt2X - linePt1X);
			footPt.x = (k * linePt1X + ptX*1.0F / k + ptY - linePt1Y)*1.0F / (k + 1.0F / k);
			footPt.y = (footPt.x - linePt1X)*k + linePt1Y;
		}
		return footPt;
	}

	//计算两点的距离
	float Cal2PtDistance(Point2f pt1, Point2f pt2)
	{
		double distance = 0;
		if (fabs(pt1.x - pt2.x) > FLOAT_MIN_VAL || fabs(pt1.y - pt2.y) > FLOAT_MIN_VAL)
			distance = sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
		return (float)distance;
	}

	//计算两点的距离
	float Cal2PtDistance(Point2i pt1, Point2i pt2)
	{
		double distance = 0;
		if (pt1.x != pt2.x || pt1.y != pt2.y)
			distance = sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
		return (float)distance;
	}

	//计算某点到线段的垂直距离
	float CalPt2LineDistance(Point2f linePt1, Point2f linePt2, Point2f pt)
	{
		Point2f footPt = GetFootPt(linePt1, linePt2, pt.x, pt.y);//计算某一点到线段的垂足点坐标
		return Cal2PtDistance(pt, footPt);
		//return fabs(CalPt2Line_CrossProduct(linePt1, linePt2, pt));
	}

	//计算两个矢量的夹角(带方向) 
	float Cal2VecDegAng(Point2i sourcePt, int startPtX, int startPtY, int endPtX, int endPtY)
	{
		return Cal2VecDegAng((float)sourcePt.x, (float)sourcePt.y, startPtX, startPtY, endPtX, endPtY);
	}

	//计算两个矢量的夹角 (带方向)
	float Cal2VecDegAng(int sourceX, int sourceY, int startPtX, int startPtY, int endPtX, int endPtY)
	{
		return Cal2VecDegAng((float)sourceX, (float)sourceY, startPtX, startPtY, endPtX, endPtY);
	}

	//计算两个矢量的夹角(带方向) 
	float Cal2VecDegAng(Point2f sourcePt, int startPtX, int startPtY, int endPtX, int endPtY)
	{
		return Cal2VecDegAng(sourcePt.x, sourcePt.y, startPtX, startPtY, endPtX, endPtY);
	}

	//计算两个矢量的夹角(带方向) 
	float Cal2VecDegAng(Point2f sourcePt, float startPtX, float startPtY, float endPtX, float endPtY)
	{
		return Cal2VecDegAng(sourcePt.x, sourcePt.y, startPtX, startPtY, endPtX, endPtY);
	}

	//计算两个矢量的夹角(带方向)
	float Cal2VecDegAng(float sourceX, float sourceY, int startPtX, int startPtY, int endPtX, int endPtY)
	{
		return Cal2VecDegAng(sourceX, sourceY, (float)startPtX, (float)startPtY, (float)endPtX, (float)endPtY);
	}

	//计算两个矢量的夹角(带方向)
	float Cal2VecDegAng(float sourceX, float sourceY, float startPtX, float startPtY, float endPtX, float endPtY)
	{
		float degAng = 0;
		if (fabs(startPtX - endPtX) < FLOAT_MIN_VAL && fabs(startPtY - endPtY) < FLOAT_MIN_VAL)
			return degAng;
		//向量：A (x1,y1) 向量：B(x2,y2)  AB的夹角为r   cosr = 向量A * 向量B / （向量A的摸 * 向量B的摸）
		Point2f vecA(startPtX - sourceX, startPtY - sourceY);//向量A
		Point2f vecB(endPtX - sourceX, endPtY - sourceY);//向量B
		double lenth_vecA = sqrt((startPtX - sourceX)*(startPtX - sourceX) + (startPtY - sourceY)*(startPtY - sourceY));
		double lenth_vecB = sqrt((endPtX - sourceX)*(endPtX - sourceX) + (endPtY - sourceY)*(endPtY - sourceY));
		float vecA_vecB = (startPtX - sourceX) * (endPtX - sourceX) + (startPtY - sourceY) * (endPtY - sourceY);//向量A * 向量B
		double radAng = 0;
		if ((lenth_vecA * lenth_vecB) > FLOAT_MIN_VAL)
		{
			radAng = acos(vecA_vecB / (lenth_vecA * lenth_vecB));
			degAng = float(radAng * 180 / PI_HS);
			if (CalPt2Line_CrossProduct(sourceX, sourceY, startPtX, startPtY, endPtX, endPtY) > FLOAT_MIN_VAL)
				degAng *= -1.0F;
		}
		else
			degAng = 0;
		return degAng;
	}


	//计算某点到线段的叉积(向量积)
	float CalPt2Line_CrossProduct(Point2f linePt1, Point2f linePt2, Point2f pt)
	{
		//当为笛卡尔坐标系且multi > 0时，pt在直线p1p2(即向量p1p2)的左侧，当multi < 0时，pt在直线p1p2(即向量p1p2)的右侧。当为图像坐标系时，结论刚好相反
		return CalPt2Line_CrossProduct(linePt1.x, linePt1.y, linePt2.x, linePt2.y, pt.x, pt.y);
	}

	//计算某点到线段的叉积(向量积)
	float CalPt2Line_CrossProduct(float linePt1X, float linePt1Y, float linePt2X, float linePt2Y, float ptX, float ptY)
	{
		float x1 = linePt1X - ptX;
		float y1 = linePt1Y - ptY;
		float x2 = linePt2X - ptX;
		float y2 = linePt2Y - ptY;
		float multi = x1 * y2 - x2 * y1; //当multi > 0时，pt在直线p1p2(即向量p1p2)的左侧，当multi < 0时，pt在直线p1p2(即向量p1p2)的右侧
		return multi;
	}
#pragma endregion

}
