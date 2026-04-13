//#include "stdafx.h"
#include <algorithm>
#include "HSGeo.h"
#include <cmath>
#include <cfloat>
#include "IAPtStruct.h"

#if USING_OLD_HS_REG //IA2.0默认不启用旧的HSReg数据结构 @ChenW 08/01/2024, 11:55
#define FLOAT_MIN_VAL 0.00001
#define PIXEL_MIN_VAL 1.00001
#define PI_HS 3.1415926535897932384626433832795

//计算某一点到线段的垂足点坐标
Pt2DFloat HSG_GetFootPt(const Pt2DFloat& linePt1, const Pt2DFloat& linePt2, const Pt2DFloat& pt)
{
	return HSG_GetFootPt(linePt1, linePt2, pt.x, pt.y);
}

//计算某一点到线段的垂足点坐标
Pt2DFloat HSG_GetFootPt(const Pt2DFloat& linePt1, const Pt2DFloat& linePt2, float ptX, float ptY)
{
	Pt2DFloat footPt;//某点pt到轴线的垂足点坐标
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
Pt2DFloat HSG_GetFootPt(float linePt1X, float linePt1Y, float linePt2X, float linePt2Y, float ptX, float ptY)
{
	Pt2DFloat footPt;//某点pt到轴线的垂足点坐标
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
Pt2DFloat HSG_GetFootPt(const Pt2DInt& linePt1, const Pt2DInt& linePt2, const Pt2DInt& pt)
{
	return HSG_GetFootPt(linePt1, linePt2, pt.x, pt.y);
}

//计算某一点到线段的垂足点坐标
Pt2DFloat HSG_GetFootPt(const Pt2DInt& linePt1, const Pt2DInt& linePt2, int ptX, int ptY)
{
	return HSG_GetFootPt(linePt1.x, linePt1.y, linePt2.x, linePt2.y, ptX, ptY);
}

//计算某一点到线段的垂足点坐标
Pt2DFloat HSG_GetFootPt(int linePt1X, int linePt1Y, int linePt2X, int linePt2Y, int ptX, int ptY)
{
	Pt2DFloat footPt;//某点pt到轴线的垂足点坐标
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
		float tmpX = 0, tmpY = 0;
		footPt.x = (k * linePt1X + ptX*1.0F / k + ptY - linePt1Y)*1.0F / (k + 1.0F / k);
		footPt.y = (tmpX - linePt1X)*k + linePt1Y;
	}
	return footPt;
}

//将点集按Y值从大到小的顺序进行排序（如果最大的Y值不止一个，则选择X值最小的那个）
void HSG_SortPtByYMax2Min(std::vector<Pt2DFloat>& pt1D)
{
	Pt2DFloat pt;
	for (unsigned int i = 0; i < pt1D.size(); i++)
	{
		for (unsigned int j = i + 1; j < pt1D.size(); j++)
		{
			if (pt1D[i].y < pt1D[j].y || (fabs(pt1D[i].y - pt1D[j].y) < FLOAT_MIN_VAL && pt1D[i].x > pt1D[j].x))
			{
				pt = pt1D[j];
				pt1D[j] = pt1D[j + 1];
				pt1D[j + 1] = pt;
			}
		}
	}
}

//将点集中Y值最大的点排到第一个点的位置（如果最大的Y值不止一个，则选择X值最小的那个）
void HSG_SortYMaxPt2FstPosition(std::vector<Pt2DFloat>& pt1D)
{
	Pt2DFloat pt;
	for (unsigned int i = 1; i < pt1D.size(); i++)
	{
		if (pt1D[0].y < pt1D[i].y || (fabs(pt1D[0].y - pt1D[i].y) < FLOAT_MIN_VAL && pt1D[0].x > pt1D[i].x))
		{
			pt = pt1D[0];
			pt1D[0] = pt1D[i];
			pt1D[i] = pt;
		}
	}
}

//计算两点的距离
float HSG_Cal2PtDistance(const Pt2DFloat& pt1, const Pt2DFloat& pt2)
{
	double distance = 0;
	if (fabs(pt1.x - pt2.x) > FLOAT_MIN_VAL || fabs(pt1.y - pt2.y) > FLOAT_MIN_VAL)
		distance = sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
	return (float)distance;
}

//计算两点的距离
float HSG_Cal2PtDistance(const Pt2DInt& pt1, const Pt2DInt& pt2)
{
	double distance = 0;
	if (pt1.x != pt2.x  || pt1.y != pt2.y)
		distance = sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
	return (float)distance;
}

//计算某点到线段的叉积(向量积)
float HSG_CalPt2Line_CrossProduct(const Pt2DFloat& linePt1, const Pt2DFloat& linePt2, const Pt2DFloat& pt)
{
	//当为笛卡尔坐标系且multi > 0时，pt在直线p1p2(即向量p1p2)的左侧，当multi < 0时，pt在直线p1p2(即向量p1p2)的右侧。当为图像坐标系时，结论刚好相反
	return HSG_CalPt2Line_CrossProduct(linePt1.x, linePt1.y, linePt2.x, linePt2.y, pt.x, pt.y);
}

//计算某点到线段的叉积(向量积)
float HSG_CalPt2Line_CrossProduct(float linePt1X, float linePt1Y, float linePt2X, float linePt2Y, float ptX, float ptY)
{
	float x1 = linePt1X - ptX;
	float y1 = linePt1Y - ptY;
	float x2 = linePt2X - ptX;
	float y2 = linePt2Y - ptY;
	float multi = x1 * y2 - x2 * y1; //当multi > 0时，pt在直线p1p2(即向量p1p2)的左侧，当multi < 0时，pt在直线p1p2(即向量p1p2)的右侧
	return multi;
}

//计算某点到线段的垂直距离
float HSG_CalPt2LineDistance(const Pt2DFloat& linePt1, const Pt2DFloat& linePt2, const Pt2DFloat& pt)
{
	Pt2DFloat footPt = HSG_GetFootPt(linePt1, linePt2, pt.x, pt.y);//计算某一点到线段的垂足点坐标
	return HSG_Cal2PtDistance(pt, footPt);
	//return fabs(HSG_CalPt2Line_CrossProduct(linePt1, linePt2, pt));
}

//根据垂足点坐标及点到线段的距离求点的坐标
void HSG_GetPtByFootPtAndDist(const Pt2DFloat& linePt1, const Pt2DFloat& linePt2, const Pt2DFloat& footPt, float distance, bool bAnticlock, Pt2DFloat& pt)
{
	if (linePt1.x == linePt2.x)
	{
		if (bAnticlock)
			pt.x = footPt.x + fabs(distance);
		else
			pt.x = footPt.x - fabs(distance);
		pt.y = footPt.y;
	}
	else if (linePt1.y == linePt2.y)
	{
		pt.x = footPt.x;
		if (bAnticlock)
			pt.y = footPt.y + fabs(distance);
		else
			pt.y = footPt.y - fabs(distance);
	}
	else
	{
		float k = -(linePt2.x - linePt1.x) *1.0F / (linePt2.y - linePt1.y);//当前直线的垂线斜率
		float d = footPt.y - k * footPt.x;
		float a = 1 + (k * k);
		float b = 2 * (k * d - footPt.x - k * footPt.y);
		float c = footPt.x * footPt.x + footPt.y * footPt.y + d * d - 2 * d * footPt.y - distance * distance;
		//根据一元二次方程a * x * x + b * x + c = 0求解
		float delta = b * b - 4 * a * c;
		if (fabs(delta) < FLOAT_MIN_VAL)
		{//方程有1个实根
			pt.x = -b / (2 * a);
			pt.y = k * pt.x + d;
		}
		else if (delta > FLOAT_MIN_VAL)
		{//方程有2个实根
			pt.x = (-b - (float)sqrt(delta)) / (2 * a);
			pt.y = k * pt.x + d;
			bool tmpAnticlock = (HSG_CalPt2Line_CrossProduct(linePt1, linePt2, pt) >= 0) ? true : false;
			if (bAnticlock != tmpAnticlock)
			{
				pt.x = (-b + (float)sqrt(delta)) / (2 * a);
				pt.y = k * pt.x + d;
			}
		}
		else
		{//方程无实根
			pt.x = 0;
			pt.y = 0;
		}
	}
}

//从一组点集中获取距离最大的两个点的坐标并返回距离值
float HSG_GetMaxDistFromPtSet(const std::vector<Pt2DFloat>& pt1D, Pt2DFloat& pt1, Pt2DFloat& pt2)
{
	float maxDist = -1;
	std::vector<Pt2DFloat> convexPt1D = HSG_GetConvexHullPt_Graham(pt1D);//Graham扫描法获取一个点集中的所有凸包点
	unsigned int hullNum = (unsigned int)convexPt1D.size();
	if (hullNum > 2)
	{
		float tmpDist = 0;
		unsigned int i = 0, j = 1;
		for (i = 0; i < hullNum; i++)
		{
			HSG_GetMaxDistPt2HullEdge(convexPt1D, i, j);//获取离凸包上的某一边最远的的凸包点坐标索引
			tmpDist = HSG_Cal2PtDistance(convexPt1D[i], convexPt1D[j]);
			if (tmpDist > maxDist)
			{
				maxDist = tmpDist;
				pt1 = convexPt1D[i];
				pt2 = convexPt1D[j];
			}
		}
	}
	else if (hullNum == 2)
	{
		maxDist = HSG_Cal2PtDistance(convexPt1D[0], convexPt1D[1]);
		pt1 = convexPt1D[0];
		pt2 = convexPt1D[1];
	}
	return maxDist;
}

//对输入两点相对于基准点的斜率由小到大排序
bool HSG_SortKslopeMinToMax(const Pt2DFloat& pt, const Pt2DFloat& pt1, const Pt2DFloat& pt2)
{
	float multi = HSG_CalPt2Line_CrossProduct(pt1, pt2, pt);
	if (multi < 0)//当multi＜0时，pt2在pt1的顺时针方向，所以pt2与基准点的斜率小于pt1
		return false;
	if (fabs(multi) < FLOAT_MIN_VAL && HSG_Cal2PtDistance(pt1, pt) > HSG_Cal2PtDistance(pt2, pt))
		return false;
	return true;
}

//Graham扫描法获取一个点集中的所有凸包点（同一线段上多余的点删除掉）
std::vector<Pt2DFloat> HSG_GetConvexHullPt_Graham1(const std::vector<Pt2DFloat>& pt1D)
{
	std::vector<Pt2DFloat> convexPt1D;
	unsigned int ptNum = (unsigned int)pt1D.size();
	if (ptNum >= 3)
	{//继续对点集进行排序：对第0个点与剩下点的连线的斜率按从小到大顺序进行排序
		std::vector<Pt2DFloat> tmpPt1D = pt1D;
		HSG_SortYMaxPt2FstPosition(tmpPt1D);//将点集中Y值最大的点排到第一个点的位置（如果最大的Y值不止一个，则选择X值最小的那个）
		tmpPt1D.push_back(tmpPt1D[0]);//凸包计算由p[0]开始，由p[0]结束，所以要在数组的末尾加上p[0]
		float multi = 0;
		for (unsigned int i = 1; i < tmpPt1D.size() - 1; )
		{
			multi = HSG_CalPt2Line_CrossProduct(tmpPt1D[i], tmpPt1D[i + 1], tmpPt1D[0]);
			if (multi < -FLOAT_MIN_VAL)
			{//若multi < 0表示第0点在P1P2向量的右侧，即第2点在第1点的顺时针方向，所以第2点与第0点的斜率小于第0点与第0点的斜率，要切换第2点与第1点位置
				std::swap(tmpPt1D[i], tmpPt1D[i + 1]);
				i++;
			}
			else if (fabs(multi) < FLOAT_MIN_VAL)
			{//若multi == 0，表示第0、1、2点三点共线
				if (HSG_Cal2PtDistance(tmpPt1D[i + 1], tmpPt1D[0]) > HSG_Cal2PtDistance(tmpPt1D[i], tmpPt1D[0]))
				{//若第2点到起始第0点的距离大于第1点到起始第0点的距离，说明第2点比第1点离基准点更远，此时可以删除第1个点
					tmpPt1D.erase(tmpPt1D.begin() + i);
					//i--;
				}
				else
				{//说明第1点比第2点离基准点更远，此时可以删除第2个点，不用交换顺序
					tmpPt1D.erase(tmpPt1D.begin() + i + 1);
					//std::swap(tmpPt1D[i], tmpPt1D[i + 1]);
				}
			}
			else
				i++;
		}
		convexPt1D = tmpPt1D;
		int top = 2;//表示convexPt1D现在有多少个点
		unsigned int i = 0, j = 0;
		for (i = 2; i < tmpPt1D.size(); i++)
		{
			while (top > 1 && HSG_CalPt2Line_CrossProduct(convexPt1D[top - 1], tmpPt1D[i], convexPt1D[top - 2]) <= FLOAT_MIN_VAL)
			{
				top--;
			}
			top++;
			convexPt1D[top - 1] = tmpPt1D[i];
		}
		tmpPt1D = convexPt1D;
		for (i = 0; i < convexPt1D.size(); i++)
		{
			for (j = i + 1; j < convexPt1D.size(); j++)
			{
				if (fabs(convexPt1D[i].x - convexPt1D[j].x) < FLOAT_MIN_VAL && fabs(convexPt1D[i].y - convexPt1D[j].y) < FLOAT_MIN_VAL)
				{
					convexPt1D.erase(convexPt1D.begin() + j);
				}
			}
		}
	}
	else
		convexPt1D = pt1D;
	return convexPt1D;
}

//Graham扫描法获取一个点集中的所有凸包点（同一线段上多余的点删除掉）
std::vector<Pt2DFloat> HSG_GetConvexHullPt_Graham(const std::vector<Pt2DFloat>& pt1D)
{
	std::vector<Pt2DFloat> convexPt1D;
	unsigned int ptNum = (unsigned int)pt1D.size();
	if (ptNum >= 3)
	{//继续对点集进行排序：对第0个点与剩下点的连线的斜率按从小到大顺序进行排序
		std::vector<Pt2DFloat> tmpPt1D = pt1D;
		HSG_SortYMaxPt2FstPosition(tmpPt1D);//将点集中Y值最大的点排到第一个点的位置（如果最大的Y值不止一个，则选择X值最小的那个）
		float m = 0;
		for (unsigned int i = 1; i < tmpPt1D.size(); i++)//将所有点按照与第0个点的斜率从小到大排序，同时将斜率相同且离第0点更近的点删除
		{
			for (unsigned int j = i + 1; j < tmpPt1D.size(); j++)
			{
				m = HSG_CalPt2Line_CrossProduct(tmpPt1D[i], tmpPt1D[j], tmpPt1D[0]);
				if (m < -FLOAT_MIN_VAL)
				{//若multi < 0表示第0点在P1P2向量的右侧，即第2点在第1点的顺时针方向，所以第2点与第0点的斜率小于第0点与第0点的斜率，要切换第2点与第1点位置
					std::swap(tmpPt1D[i], tmpPt1D[j]);
				}
				else if (fabs(m) < FLOAT_MIN_VAL)
				{//若multi == 0，表示第0、1、2点三点共线
					if (HSG_Cal2PtDistance(tmpPt1D[j], tmpPt1D[0]) > HSG_Cal2PtDistance(tmpPt1D[i], tmpPt1D[0]))
					{//若第2点到起始第0点的距离大于第1点到起始第0点的距离，说明第2点比第1点离基准点更远，此时可以删除第1个点
						tmpPt1D.erase(tmpPt1D.begin() + i);
						i--;
					}
					else
					{//说明第1点比第2点离基准点更远，此时可以删除第2个点，不用交换顺序
						tmpPt1D.erase(tmpPt1D.begin() + j);
						j--;
					}
				}
			}
		}
		convexPt1D = tmpPt1D;
		int top = 2;//表示convexPt1D现在有多少个点
		unsigned int i = 0, j = 0;
		for (i = 2; i < tmpPt1D.size(); i++)
		{
			while (top > 1 && HSG_CalPt2Line_CrossProduct(convexPt1D[top - 1], tmpPt1D[i], convexPt1D[top - 2]) <= FLOAT_MIN_VAL)
			{
				top--;
			}
			top++;
			convexPt1D[top - 1] = tmpPt1D[i];
		}
		tmpPt1D = convexPt1D;
		for (i = 0; i < convexPt1D.size(); i++)
		{
			for (j = i + 1; j < convexPt1D.size(); j++)
			{
				if (fabs(convexPt1D[i].x - convexPt1D[j].x) < FLOAT_MIN_VAL && fabs(convexPt1D[i].y - convexPt1D[j].y) < FLOAT_MIN_VAL)
				{
					convexPt1D.erase(convexPt1D.begin() + j);
				}
			}
		}
	}
	else
		convexPt1D = pt1D;
	return convexPt1D;
}

//获取离凸包上的某一边最远的的凸包点坐标索引
float HSG_GetMaxDistPt2HullEdge(std::vector<Pt2DFloat> convexPt1D, const unsigned int& startPtIdx, unsigned int& maxDistPtIdx)
{
	float maxDist = 0, tmpDist = 0;
	unsigned int hullNum = (unsigned int)convexPt1D.size();
	if (hullNum > 2)
	{
		//unsigned int endPtIdx = startPtIdx + 1;
		unsigned int i = 0, j = 1;
		if (startPtIdx < hullNum)
		{
			unsigned int endPtIdx = startPtIdx + 1;
			if (endPtIdx == hullNum)//endPtIdx为凸包的最后一个点
				endPtIdx = 0;
			//convexPt1D.push_back(convexPt1D[0]);//凸包计算由p[0]开始，由p[0]结束，所以要在数组的末尾加上p[0] 
			for (i = startPtIdx + 2; i < hullNum + startPtIdx; i++)
			{
				if (i < hullNum)
					j = i;
				else
					j = i - hullNum;
				if (i == hullNum + startPtIdx - 1)//遍历到最后一个到指定边的凸包点，说明前面的点还没有达到距离峰值，最后一个点为距离峰值
				{
					maxDist = HSG_CalPt2LineDistance(convexPt1D[startPtIdx], convexPt1D[endPtIdx], convexPt1D[j]);
					break;
				}
				else
				{
					maxDist = HSG_CalPt2LineDistance(convexPt1D[startPtIdx], convexPt1D[endPtIdx], convexPt1D[j]);
					if (j + 1 < hullNum)
						tmpDist = HSG_CalPt2LineDistance(convexPt1D[startPtIdx], convexPt1D[endPtIdx], convexPt1D[j + 1]);
					else
						tmpDist = HSG_CalPt2LineDistance(convexPt1D[startPtIdx], convexPt1D[endPtIdx], convexPt1D[0]);
					if (maxDist > tmpDist)
						break;
				}
			}
			maxDistPtIdx = j;
		}
		else
		{
			maxDist = 0;
			maxDistPtIdx = 0;
		}
	}
	else if (hullNum == 2)
	{
		maxDist = 0;
		maxDistPtIdx = 1;
	}
	else
	{
		maxDist = 0;
		maxDistPtIdx = 0;
	}
	return maxDist;
}

//获取离凸包上的某一边最远的的凸包点坐标
float HSG_GetMaxDistPt2HullEdge(std::vector<Pt2DFloat> convexPt1D, const unsigned int& startPtIdx, Pt2DFloat& maxDistPt)
{
	unsigned int maxDistPtIdx = 0;
	float maxDist = HSG_GetMaxDistPt2HullEdge(convexPt1D, startPtIdx, maxDistPtIdx);
	maxDistPt = convexPt1D[maxDistPtIdx];
	return maxDist;
}

//旋转卡壳法计算凸包的最小外接矩形（不带角度）
void HSG_GetMinOutRect1ByConvexHull(const std::vector<Pt2DFloat>& pt1D, Pt2DInt& ltTopPt, Pt2DInt& rtBtnPt)
{
	//待完善
}

//旋转卡壳法计算凸包的最小带角度外接矩形
void HSG_GetMinOutRect2ByConvexHull(const std::vector<Pt2DFloat>& pt1D, Pt2DFloat& cnter, float& halfW, float& halfH, float& degAng)
{
	cnter.Init(); halfW = 0; halfH = 0; degAng = 0;
	std::vector<Pt2DFloat> convexPt1D = HSG_GetConvexHullPt_Graham(pt1D);//Graham扫描法获取一个点集中的所有凸包点（同一线段上多余的点删除掉）
	if (convexPt1D.size() < 3)
		return;
	unsigned int hullNum = (unsigned int)convexPt1D.size();
	unsigned int lineIdx1 = 0, lineIdx2 = 0, j = 0, k = 0, m = 0, n = 0;
	//convexPt1D.push_back(convexPt1D[0]);//凸包计算由p[0]开始，由p[0]结束，所以要在数组的末尾加上p[0] 
	Pt2DFloat hFootPt;
	Pt2DFloat rctLtBtmPt;//最小矩形时的矩形左下角坐标
	Pt2DFloat rctRtBtmPt;//最小矩形时的矩形右下角坐标
	Pt2DFloat rctLtTopPt;//最小矩形时的矩形左上角坐标
	Pt2DFloat maxDistPt;
	float maxLenth = 0;
	float rctWidth = 0, rctHeight = 0, rctArea = 0;
	float minArea = 1e10;
	std::vector<Pt2DFloat> tmpPt1D;
	bool bAnticlock = false;//是否是逆时针方向
	for (lineIdx1 = 0; lineIdx1 < hullNum; lineIdx1++)
	{
		rctHeight = HSG_GetMaxDistPt2HullEdge(convexPt1D, lineIdx1, j);//获取离凸包上的某一边最远的的凸包点坐标
		if (lineIdx1 < hullNum - 1)
			lineIdx2 = lineIdx1 + 1;
		else
			lineIdx2 = 0;//若当前边为凸包的最后一条边
		hFootPt = HSG_GetFootPt(convexPt1D[lineIdx1], convexPt1D[lineIdx2], convexPt1D[j]);//计算最远凸包点H到当前边P1P2的垂足点F
		tmpPt1D.clear();
		tmpPt1D.push_back(hFootPt);
		if (HSG_CalPt2Line_CrossProduct(hFootPt, convexPt1D[j], convexPt1D[lineIdx2]) >= 0)
			bAnticlock = true;
		else
			bAnticlock = false;
		if (j > lineIdx2)
		{
			for (m = lineIdx2; m < j; m++)
			{
				if ((m == lineIdx2 && bAnticlock == false) || m > lineIdx2)
					tmpPt1D.push_back(convexPt1D[m]);//从原来的大凸包中以最远凸包点H到当前边P1P2的垂足点F为分界线，从垂足点开始新建立一个凸包，此凸包包含垂足，H点，以及原来大凸包右侧的凸包点
			}
		}
		else
		{
			for (m = lineIdx2; m < hullNum; m++)
			{
				if ((m == lineIdx2 && bAnticlock == false) || m > lineIdx2)
					tmpPt1D.push_back(convexPt1D[m]);
			}
			for (m = 0; m < j; m++)
			{
				tmpPt1D.push_back(convexPt1D[m]);
			}
		}
		tmpPt1D.push_back(convexPt1D[j]);
		int tmpNum = (int)tmpPt1D.size();
		tmpNum = tmpNum - 1;
		if (tmpNum < 0)
			tmpNum = 0;
		HSG_GetMaxDistPt2HullEdge(tmpPt1D, (unsigned int)tmpNum, maxDistPt);//获取离凸包上的某一边最远的的凸包点坐标
		rctRtBtmPt = HSG_GetFootPt(convexPt1D[lineIdx1], convexPt1D[lineIdx2], maxDistPt);//计算右侧凸包上离AB最远点Rf到边P1P2的垂足点，即为最小矩形的右下角坐标
		tmpPt1D.clear();
		tmpPt1D.push_back(hFootPt);
		tmpPt1D.push_back(convexPt1D[j]);
		if (j > lineIdx2)
		{
			for (m = j + 1; m < hullNum; m++)
			{
				tmpPt1D.push_back(convexPt1D[m]);//从原来的大凸包中以最远凸包点H到当前边P1P2的垂足点F为分界线，从垂足点开始新建立一个凸包，此凸包包含垂足，H点，以及原来大凸包左侧的凸包点
			}
			if (lineIdx1 < lineIdx2)//若lineIdx2 < lineIdx1，表明此时lineIdx2 == 0，即当前为最后一条边
			{
				for (m = 0; m <= lineIdx1; m++)
				{
					tmpPt1D.push_back(convexPt1D[m]);
				}
			}
			if (bAnticlock)
				tmpPt1D.push_back(convexPt1D[lineIdx2]);
		}
		else
		{
			for (m = j + 1; m < lineIdx2; m++)
			{
				tmpPt1D.push_back(convexPt1D[m]);
			}
			if (bAnticlock)
				tmpPt1D.push_back(convexPt1D[lineIdx2]);
		}
		HSG_GetMaxDistPt2HullEdge(tmpPt1D, 0, maxDistPt);//获取离凸包上的某一边最远的的凸包点坐标
		rctLtBtmPt = HSG_GetFootPt(convexPt1D[lineIdx1], convexPt1D[lineIdx2], maxDistPt);//计算右侧凸包上离AB最远点Lf到边P1P2的垂足点，即为最小矩形的左下角坐标
		tmpPt1D.clear();
		tmpPt1D.push_back(convexPt1D[lineIdx1]);
		tmpPt1D.push_back(convexPt1D[lineIdx2]);
		tmpPt1D.push_back(rctLtBtmPt);
		tmpPt1D.push_back(rctRtBtmPt);
		rctWidth = HSG_GetMaxDistFromPtSet(tmpPt1D, rctLtBtmPt, rctRtBtmPt);
		if (rctRtBtmPt.x < rctLtBtmPt.x)
			std::swap(rctLtBtmPt, rctRtBtmPt);
		rctArea = rctWidth * rctHeight;
		if (rctArea < minArea)
		{
			minArea = rctArea;
			if (HSG_CalPt2Line_CrossProduct(rctLtBtmPt, rctRtBtmPt, convexPt1D[j]) >= 0)
				bAnticlock = true;//convexPt1D[j]在(rctLtBtmPt, rctRtBtmPt)向量的逆时针方向
			else
				bAnticlock = false;
			HSG_GetPtByFootPtAndDist(rctLtBtmPt, rctRtBtmPt, rctLtBtmPt, rctHeight, bAnticlock, rctLtTopPt);//根据垂足点坐标及点到线段的距离求点的坐标
			cnter.x = (rctLtTopPt.x + rctRtBtmPt.x) / 2.0F;
			cnter.y = (rctLtTopPt.y + rctRtBtmPt.y) / 2.0F;
			halfW = rctWidth / 2.0F;
			halfH = rctHeight / 2.0F;
			degAng = HSG_Cal2VecDegAng(rctLtBtmPt, (rctLtBtmPt.x + rctRtBtmPt.x) / 2.0F, rctLtBtmPt.y, rctRtBtmPt.x, rctRtBtmPt.y);//计算两个矢量的夹角(带方向)
		}
	}
}

//计算两个矢量的夹角(带方向) 
float HSG_Cal2VecDegAng(const Pt2DInt& sourcePt, int startPtX, int startPtY, int endPtX, int endPtY)
{
	return HSG_Cal2VecDegAng((float)sourcePt.x, (float)sourcePt.y, startPtX, startPtY, endPtX, endPtY);
}

//计算两个矢量的夹角 (带方向)
float HSG_Cal2VecDegAng(int sourceX, int sourceY, int startPtX, int startPtY, int endPtX, int endPtY)
{
	return HSG_Cal2VecDegAng((float)sourceX, (float)sourceY, startPtX, startPtY, endPtX, endPtY);
}

//计算两个矢量的夹角(带方向) 
float HSG_Cal2VecDegAng(const Pt2DFloat& sourcePt, int startPtX, int startPtY, int endPtX, int endPtY)
{
	return HSG_Cal2VecDegAng(sourcePt.x, sourcePt.y, startPtX, startPtY, endPtX, endPtY);
}

//计算两个矢量的夹角(带方向) 
float HSG_Cal2VecDegAng(const Pt2DFloat& sourcePt, float startPtX, float startPtY, float endPtX, float endPtY)
{
	return HSG_Cal2VecDegAng(sourcePt.x, sourcePt.y, startPtX, startPtY, endPtX, endPtY);
}

//计算两个矢量的夹角(带方向)
float HSG_Cal2VecDegAng(float sourceX, float sourceY, int startPtX, int startPtY, int endPtX, int endPtY)
{
	return HSG_Cal2VecDegAng(sourceX, sourceY, (float)startPtX, (float)startPtY, (float)endPtX, (float)endPtY);
}

//计算两个矢量的夹角(带方向)
float HSG_Cal2VecDegAng(float sourceX, float sourceY, float startPtX, float startPtY, float endPtX, float endPtY)
{
	float degAng = 0;
	if (fabs(startPtX - endPtX) < FLOAT_MIN_VAL && fabs(startPtY - endPtY) < FLOAT_MIN_VAL)
		return degAng;
	//向量：A (x1,y1) 向量：B(x2,y2)  AB的夹角为r   cosr = 向量A * 向量B / （向量A的摸 * 向量B的摸）
	Pt2DFloat vecA(startPtX - sourceX, startPtY - sourceY);//向量A
	Pt2DFloat vecB(endPtX - sourceX, endPtY - sourceY);//向量B
	double lenth_vecA = sqrt((startPtX - sourceX)*(startPtX - sourceX) + (startPtY - sourceY)*(startPtY - sourceY));
	double lenth_vecB = sqrt((endPtX - sourceX)*(endPtX - sourceX) + (endPtY - sourceY)*(endPtY - sourceY));
	float vecA_vecB = (startPtX - sourceX) * (endPtX - sourceX) + (startPtY - sourceY) * (endPtY - sourceY);//向量A * 向量B
	double radAng = 0;
	if ((lenth_vecA * lenth_vecB) > FLOAT_MIN_VAL)
	{
		radAng = acos(vecA_vecB / (lenth_vecA * lenth_vecB));
		degAng = float(radAng * 180 / PI_HS);
		if (HSG_CalPt2Line_CrossProduct(sourceX, sourceY, startPtX, startPtY, endPtX, endPtY) > FLOAT_MIN_VAL)
			degAng *= -1.0F;
	}
	else
		degAng = 0;
	return degAng;
}

bool HSG_IsPtInPolyGon(const std::vector<Pt2DFloat> &pts1D, Pt2DFloat temsorcePt)
{
	//线段
	struct LINESEG
	{
		Pt2DFloat s;
		Pt2DFloat e;
		LINESEG(Pt2DFloat a, Pt2DFloat b) { s = a; e = b; }
		LINESEG() { }
	};

	 unsigned int temcrosscount = 0;
	 unsigned int temptSize = ( unsigned int)pts1D.size();
	LINESEG temline;
	LINESEG temlinesorce;
	temlinesorce.s.x = temsorcePt.x;
	temlinesorce.s.y = temsorcePt.y;
	temlinesorce.e.x = DBL_MAX;
	temlinesorce.e.y = temsorcePt.y;

	Pt2DFloat temendPt1;
	Pt2DFloat temendPt2;
	for (int i = 0; i < (int)temptSize; i++)
	{
		temendPt1.x = temline.s.x = pts1D[i].x;
		temendPt1.y = temline.s.y = pts1D[i].y;
		temendPt2.x = temline.e.x = pts1D[(i + 1) % temptSize].x;
		temendPt2.y = temline.e.y = pts1D[(i + 1) % temptSize].y;

		if (HSG_Online(temline.s, temline.e, temsorcePt))
		{
			return true;
		}
		if (abs(temline.s.y - temline.e.y) < FLOAT_MIN_VAL)
		{
			continue;
		}
		if (HSG_Online(temlinesorce.s, temlinesorce.e, temendPt1))
		{
			if (temendPt1.y > temendPt2.y)
			{
				temcrosscount++;
			}
		}
		else if (HSG_Online(temlinesorce.s, temlinesorce.e, temendPt2))
		{
			if (temendPt2.y > temendPt1.y)
			{
				temcrosscount++;
			}
		}
		else if (HSG_Intersect(temlinesorce.s, temlinesorce.e, temline.s, temline.e))
		{
			temcrosscount++;
		}
	}
	bool isInside = false;
	if (1 == temcrosscount % 2)
	{
		isInside = true;
	}
	return isInside;
}

bool HSG_Online(const Pt2DFloat &startPt, const Pt2DFloat &endPt, const Pt2DFloat &resPt)
{
	double condition1 = 0.0;
	condition1 = (endPt.y - startPt.y)*(resPt.x - startPt.x) - (resPt.y - startPt.y)*(endPt.x - startPt.x);
	bool condition2 = false;
	condition2 = ((resPt.x - startPt.x)*(resPt.x - endPt.x) <= FLOAT_MIN_VAL) && ((resPt.y - startPt.y)*(resPt.y - endPt.y) <= FLOAT_MIN_VAL);
	return (condition1 < FLOAT_MIN_VAL) && condition2;
}

bool HSG_Intersect(const Pt2DFloat &startPt1, const Pt2DFloat &endPt1, const Pt2DFloat &startPt2, const Pt2DFloat &endPt2)
{
	bool condition1 = false;
	//快速排斥
	condition1 = (max(startPt1.x, endPt1.x) < min(startPt2.x, endPt2.x)) || (max(startPt2.x, endPt2.x) < min(startPt1.x, endPt1.x))
		|| (max(startPt1.y, endPt1.y) < min(startPt2.y, endPt2.y)) || (max(startPt2.y, endPt2.y) < min(startPt1.y, endPt1.y));
	if (condition1)
	{
		return false;
	}
	//跨立
	bool condition2 = false, condition3 = false;

	condition2 = ((startPt1.x - startPt2.x)*(endPt2.y - startPt2.y) - (startPt1.y - startPt2.y)*(endPt2.x - startPt2.x))*
		((endPt1.x - startPt2.x)*(endPt2.y - startPt2.y) - (endPt1.y - startPt2.y)*(endPt2.x - startPt2.x)) > 0;
	condition3 = ((startPt2.x - startPt1.x)*(endPt1.y - startPt1.y) - (startPt2.y - startPt1.y)*(endPt1.x - startPt1.x))*
		((endPt2.x - startPt1.x)*(endPt1.y - startPt1.y) - (endPt2.y - startPt1.y)*(endPt1.x - startPt1.x)) > 0;

	if (condition2 || condition3)
	{
		return false;
	}
	return true;
}
#endif //IA2.0默认不启用旧的HSReg数据结构 @ChenW 08/01/2024, 11:55

namespace HSGEO
{
#define EPSILON 0.000001
	//交换
	template<class _Ty> inline
		void G_Swap(_Ty& _Left, _Ty& _Right)
	{	// exchange values stored at _Left and _Right
		if (&_Left != &_Right)
		{	// different, worth swapping
			_Ty _Tmp = _Left;

			_Left = _Right;
			_Right = _Tmp;
		}
	}
	/**********************
	*                    *
	*   坐标变换     *
	*                    *
	**********************/

	//平移变换
	void G_Trans_Offset(HSV::Point2d o, HSV::Point2d& p)
	{
		p.x += o.x;
		p.y += o.y;
	}

	//平移反变换
	void G_Trans_Offset_Opp(HSV::Point2d o, HSV::Point2d& p)
	{
		p.x -= o.x;
		p.y -= o.y;
	}

	//缩放变换
	void G_Trans_Scale(double scaleX, double scaleY, HSV::Point2d& p)
	{
		p.x = scaleX*p.x;
		p.y = scaleY*p.y;
	}
	void G_Trans_Scale_Opp(double scaleX, double scaleY, HSV::Point2d& p)
	{
		p.x = p.x / scaleX;
		p.y = p.y / scaleY;
	}
	//直斜变换
	void G_Trans_VtoK(double G_Angle, HSV::Point2d& p)
	{
		double Angle = G_Angle * 180 / 3.1415926;
		p.x = p.x - p.y*cos(G_Angle) / sin(G_Angle);
		p.y = p.y / sin(G_Angle);
	}

	//斜直变换
	void G_Trans_KtoV(double G_Angle, HSV::Point2d& p)
	{
		//G_Angle=3.1415926*G_Angle/180;
		p.x = p.y*cos(G_Angle) + p.x;
		p.y = p.y*sin(G_Angle);
	}

	//旋转变换
	void G_Trans_Rotate(double G_Angle, HSV::Point2d& p)
	{
		//G_Angle=3.1415926*G_Angle/180;
		/*p.x=p.x*cos(G_Angle)+p.y*sin(G_Angle);
		p.y=-p.x*sin(G_Angle)+p.y*cos(G_Angle);*/ //换算有误 2019.03.09
		//G_Angle = - G_Angle;
		HSV::Point2d tem;
		tem.x = p.x*cos(G_Angle) + p.y*sin(G_Angle);
		tem.y = -p.x*sin(G_Angle) + p.y*cos(G_Angle);
		p = tem;
	}
	/***************************************************************************************
	*函 数 名：void CoodrateTrans(double a,double b,double q,double k1,double k2,
	double x,double y,double& s,double& t)

	*输入参数：double a,坐标系st的坐标轴夹角
	double b,坐标系xy的坐标轴夹角
	double q,坐标系s轴正方向相对于坐标系x轴正方向的夹角
	double k1,坐标系st，S单位长度相对于xy，X单位长度倍率
	double k2,坐标系st，T单位长度相对于xy，Y单位长度倍率
	double s,double t，xy坐标系已知的点
	double& x,double& y，要计算的st坐标系的点

	*函数功能：坐标变换,在st和xy原点重合的情况下，已知点在st的坐标，求出其在xy上的坐标

	*返 回 值：无
	***************************************************************************************/
	void G_CoodrateTrans(double a, double b, double q, double k1, double k2,
		double s, double t, double& x, double& y)
	{
		//SMCVCoodrateTrans(a,b,q,k1,k2,s,t,x,y);
		x = sin(b - q - a)*t*k2 / sin(b) + sin(b - q)*s*k1 / sin(b);
		y = sin(a + q)*t*k2 / sin(b) + sin(q)*s*k1 / sin(b);
	}

	///////////////////////////////////////////  
	//求三角形外接圆,为计算计算找晶圆形区域准备
	///////////////////////////////////////////  
	bool G_GetCirclePt(HSV::Point2d *center, HSV::Point2d* pt, double *radiu)
	{
		if (pt[0] == pt[1] || pt[0] == pt[2] || pt[2] == pt[1])
		{
			return false;
		}
		double   x1, x2, x3, y1, y2, y3;
		double   x = 0;
		double   y = 0;

		//求圆心和半径
		x1 = pt[0].x;
		x2 = pt[1].x;
		x3 = pt[2].x;
		y1 = pt[0].y;
		y2 = pt[1].y;
		y3 = pt[2].y;

		//
		double u1 = (x2*x2 - x1*x1 + y2*y2 - y1*y1) / 2;
		double u2 = (x3*x3 - x1*x1 + y3*y3 - y1*y1) / 2;
		double d11 = x2 - x1;
		double d12 = y2 - y1;
		double d21 = x3 - x1;
		double d22 = y3 - y1;

		x = (u1*d22 - u2*d12) / (d11*d22 - d21*d12);
		y = (u2*d11 - u1*d21) / (d11*d22 - d21*d12);
		center->x = x;
		center->y = y;
		*radiu = (pt[2].x - x)*(pt[2].x - x) + (pt[2].y - y)*(pt[2].y - y);
		*radiu = sqrt(*radiu);

		return true;
	}

	void G_TransXYT(double xCalibCenter, double yCalibCenter, double rCirclePluse,
		double xCurPos, double yCurPos, double rCurPos,
		double angle,
		double &xTagPos, double& yTagPos, double& rTagPos)
	{
		xTagPos = 0;
		yTagPos = 0;
		rTagPos = 0;
		//计算出R轴的目标位置
		double nVal = rCirclePluse / 360.0;
		rTagPos = rCurPos - angle*nVal;
		double realAngle;
		if (nVal == 0)
		{
			realAngle = 0;
			xTagPos = xCurPos;
			yTagPos = yCurPos;
			rTagPos = rCurPos;
			return;
		}
		else {
			realAngle = (rCurPos - rTagPos) / nVal;
		}

		//计算X，Y轴的目标位置
		double temAngle = (PI_HS * realAngle / 180);
		//计算原矩形与中心的距离

		xTagPos = (xCurPos - xCalibCenter)*cos(temAngle) - (yCurPos - yCalibCenter)*sin(temAngle) + xCalibCenter;
		if (xTagPos<0)
		{
			xTagPos -= 0.5;
		}
		else
		{
			xTagPos += 0.5;
		}
		yTagPos = (yCurPos - yCalibCenter)*cos(temAngle) + (xCurPos - xCalibCenter)*sin(temAngle) + yCalibCenter;
		if (yTagPos<0)
		{
			yTagPos -= 0.5;
		}
		else
		{
			yTagPos += 0.5;
		}
	}

	/**********************
	*                    *
	*   点的基本运算     *
	*                    *
	**********************/

	// 返回两点之间欧氏距离 
	double G_Dist(HSV::Point2d p1, HSV::Point2d p2)
	{
		return(sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y)));
	}
	// 判断两个点是否重合  
	bool G_IsPtEqual(HSV::Point2d p1, HSV::Point2d p2)
	{
		return ((abs(p1.x - p2.x)<GE0_VALUE_MIN) && (abs(p1.y - p2.y)<GE0_VALUE_MIN));
	}
	/******************************************************************************
	r=G_Multiply(sp,ep,op),得到(sp-op)和(ep-op)的叉积
	r>0：ep在矢量opsp的逆时针方向；
	r=0：opspep三点共线；
	r<0：ep在矢量opsp的顺时针方向
	*******************************************************************************/
	double G_Multiply(HSV::Point2d sp, HSV::Point2d ep, HSV::Point2d op)
	{
		return((sp.x - op.x)*(ep.y - op.y) - (ep.x - op.x)*(sp.y - op.y));
	}
	/*
	r=G_DotMultiply(p1,p2,op),得到矢量(p1-op)和(p2-op)的点积，如果两个矢量都非零矢量
	r<0：两矢量夹角为锐角；
	r=0：两矢量夹角为直角；
	r>0：两矢量夹角为钝角
	*******************************************************************************/
	double G_DotMultiply(HSV::Point2d p1, HSV::Point2d p2, HSV::Point2d p0)
	{
		return ((p1.x - p0.x)*(p2.x - p0.x) + (p1.y - p0.y)*(p2.y - p0.y));
	}
	/******************************************************************************
	判断点p是否在线段l上
	条件：(p在线段l所在的直线上) && (点p在以线段l为对角线的矩形内)
	*******************************************************************************/
	bool G_Online(LINESEG l, HSV::Point2d p)
	{
		return((G_Multiply(l.e, p, l.s) < EPSILON) && (((p.x - l.s.x)*(p.x - l.e.x) <= 0) && ((p.y - l.s.y)*(p.y - l.e.y) <= 0)));
	}
	// 返回点p以点o为圆心逆时针旋转G_Alpha(单位：弧度)后所在的位置 
	HSV::Point2d G_Rotate(HSV::Point2d o, double G_Alpha, HSV::Point2d p)
	{
		HSV::Point2d tp;
		p.x -= o.x;
		p.y -= o.y;
		tp.x = p.x*cos(G_Alpha) - p.y*sin(G_Alpha) + o.x;
		tp.y = p.y*cos(G_Alpha) + p.x*sin(G_Alpha) + o.y;
		return tp;
	}
	/* 返回顶角在o点，起始边为os，终止边为oe的夹角(单位：弧度)

	原理：
	r = G_DotMultiply(s,e,o) / (G_Dist(o,s)*G_Dist(o,e))
	r'= G_Multiply(s,e,o)

	r >= 1	G_Angle = 0;
	r <= -1	G_Angle = -GE0_VALUE_PI
	*/
	double G_Angle(HSV::Point2d o, HSV::Point2d s, HSV::Point2d e)
	{
		double cosfi, fi, norm;
		double dsx = s.x - o.x;
		double dsy = s.y - o.y;
		double dex = e.x - o.x;
		double dey = e.y - o.y;

		cosfi = dsx*dex + dsy*dey;
		norm = (dsx*dsx + dsy*dsy)*(dex*dex + dey*dey);
		if (norm == 0)//gcnhiexn add 当等0时说明角度无效大，默认为0度
		{
			return 0.0;
		}
		cosfi /= sqrt(norm);

		if (cosfi >= 1.0) return 0;
		if (cosfi <= -1.0) return -3.1415926;

		fi = acos(cosfi); //两个向量的夹角

		if (dsx*dey - dsy*dex>0) //第1，2象限
		{
			return fi;      // 说明矢量os 在矢量 oe的顺时针方向 
		}
		else if (fi >= 0) //第4象限
		{
			return 2 * 3.1415926 - fi;
		}
		else //第3象限
		{
			return 2 * 3.1415926 + fi;
		}
		return -fi;
	}

	double G_Angle(HSV::Point o, HSV::Point s, HSV::Point e)
	{
		return G_Angle(HSV::Point2d((double)o.x, (double)o.y),
			HSV::Point2d((double)s.x, (double)s.y),
			HSV::Point2d((double)e.x, (double)e.y));
	}

	/*****************************\
	*                             *
	*      线段及直线的基本运算   *
	*                             *
	\*****************************/

	/* 判断点与线段的关系,用途很广泛
	本函数是根据下面的公式写的，P是点C到线段AB所在直线的垂足

	AC dot AB
	r =     ---------
	||AB||^2
	(Cx-Ax)(Bx-Ax) + (Cy-Ay)(By-Ay)
	= -------------------------------
	L^2

	r has the following meaning:

	r=0      P = A
	r=1      P = B
	r<0		 P is on the backward extension of AB
	r>1      P is on the forward extension of AB
	0<r<1	 P is interior to AB
	*/
	double G_Relation(HSV::Point2d p, LINESEG l)
	{
		LINESEG tl;
		tl.s = l.s;
		tl.e = p;
		return G_DotMultiply(tl.e, l.e, l.s) / (G_Dist(l.s, l.e)*G_Dist(l.s, l.e));
	}
	// 求点C到线段AB所在直线的垂足 P 
	HSV::Point2d G_Perpendicular(HSV::Point2d p, LINESEG l)
	{
		double r = G_Relation(p, l);
		HSV::Point2d tp;
		tp.x = l.s.x + r*(l.e.x - l.s.x);
		tp.y = l.s.y + r*(l.e.y - l.s.y);
		return tp;
	}
	/* 求点p到线段l的最短距离,并返回线段上距该点最近的点np
	注意：np是线段l上到点p最近的点，不一定是垂足 */
	double G_Pt2LinesegDist(HSV::Point2d p, LINESEG l, HSV::Point2d &np)
	{
		double r = G_Relation(p, l);
		if (r<0)
		{
			np = l.s;
			return G_Dist(p, l.s);
		}
		if (r>1)
		{
			np = l.e;
			return G_Dist(p, l.e);
		}
		np = G_Perpendicular(p, l);
		return G_Dist(p, np);
	}
	// 求点p到线段l所在直线的距离,请注意本函数与上个函数的区别  
	double G_Pt2LineDist(HSV::Point2d p, LINESEG l)
	{
		return abs(G_Multiply(p, l.e, l.s)) / G_Dist(l.s, l.e);
	}
	/* 计算点到折线集的最近距离,并返回最近点.
	注意：调用的是ptolineseg()函数 */
	double G_Pt2GPtset(int vcount, HSV::Point2d G_POINTset[], HSV::Point2d p, HSV::Point2d &q)
	{
		int i;
		double cd = double(GE0_VALUE_MAX), td;
		LINESEG l;
		HSV::Point2d tq, cq;

		for (i = 0; i<vcount - 1; i++)
		{
			l.s = G_POINTset[i];

			l.e = G_POINTset[i + 1];
			td = G_Pt2LinesegDist(p, l, tq);
			if (td<cd)
			{
				cd = td;
				cq = tq;
			}
		}
		q = cq;
		return cd;
	}
	/* 判断圆是否在多边形内.ptolineseg()函数的应用2 */
	bool G_IsCircleInsidePolygon(int vcount, HSV::Point2d center, double radius, HSV::Point2d polygon[])
	{
		HSV::Point2d q;
		double d;
		q.x = 0;
		q.y = 0;
		d = G_Pt2GPtset(vcount, polygon, center, q);
		if (d<radius || fabs(d - radius)<GE0_VALUE_MIN)
			return true;
		else
			return false;
	}
	/* 返回两个矢量l1和l2的夹角的余弦(-1 --- 1)注意：如果想从余弦求夹角的话，注意反余弦函数的定义域是从 0到pi */
	double G_Cosine(LINESEG l1, LINESEG l2)
	{
		return (((l1.e.x - l1.s.x)*(l2.e.x - l2.s.x) +
			(l1.e.y - l1.s.y)*(l2.e.y - l2.s.y)) / (G_Dist(l1.e, l1.s)*G_Dist(l2.e, l2.s)));
	}
	// 返回线段l1与l2之间的夹角 单位：弧度 范围(0,2PI),l2相对于l2
	double G_LinesegAngle(LINESEG l1, LINESEG l2)
	{
		HSV::Point2d o, s, e;
		o.x = o.y = 0;
		s.x = l1.e.x - l1.s.x;
		s.y = l1.e.y - l1.s.y;
		e.x = l2.e.x - l2.s.x;
		e.y = l2.e.y - l2.s.y;
		return G_Angle(o, s, e);
	}
	// 如果线段u和v相交(包括相交在端点处)时，返回true 
	//
	//判断P1P2跨立Q1Q2的依据是：( P1 - Q1 ) × ( Q2 - Q1 ) * ( Q2 - Q1 ) × ( P2 - Q1 ) >= 0。
	//判断Q1Q2跨立P1P2的依据是：( Q1 - P1 ) × ( P2 - P1 ) * ( P2 - P1 ) × ( Q2 - P1 ) >= 0。
	bool G_Intersect(LINESEG u, LINESEG v)
	{
		return(((std::max)(u.s.x, u.e.x) >= (std::min)(v.s.x, v.e.x)) &&                     //排斥实验 
			((std::max)(v.s.x, v.e.x) >= (std::min)(u.s.x, u.e.x)) &&
			((std::max)(u.s.y, u.e.y) >= (std::min)(v.s.y, v.e.y)) &&
			((std::max)(v.s.y, v.e.y) >= (std::min)(u.s.y, u.e.y)) &&
			(G_Multiply(v.s, u.e, u.s)*G_Multiply(u.e, v.e, u.s) >= 0) &&         //跨立实验 
			(G_Multiply(u.s, v.e, v.s)*G_Multiply(v.e, u.e, v.s) >= 0));
	}
	//  (线段u和v相交)&&(交点不是双方的端点) 时返回true    
	bool G_Intersect_A(LINESEG u, LINESEG v)
	{
		return	((G_Intersect(u, v)) &&
			(!G_Online(u, v.s)) &&
			(!G_Online(u, v.e)) &&
			(!G_Online(v, u.e)) &&
			(!G_Online(v, u.s)));
	}
	// 线段v所在直线与线段u相交时返回true；方法：判断线段u是否跨立线段v  
	bool G_Intersect_l(LINESEG u, LINESEG v)
	{
		return G_Multiply(u.s, v.e, v.s)*G_Multiply(v.e, u.e, v.s) >= 0;
	}
	// 根据已知两点坐标，求过这两点的直线解析方程： a*x+b*y+c = 0  (a >= 0)  
	LINE G_MakeLine(HSV::Point2d p1, HSV::Point2d p2)
	{
		LINE tl;
		int sign = 1;
		tl.a = p2.y - p1.y;
		if (tl.a<0)
		{
			sign = -1;
			tl.a = sign*tl.a;
		}
		tl.b = sign*(p1.x - p2.x);
		tl.c = sign*(p1.y*p2.x - p1.x*p2.y);
		return tl;
	}
	// 根据直线解析方程返回直线的斜率k,水平线返回 0,竖直线返回 1e200 
	double G_Slope(LINE l)
	{
		if (abs(l.a) < 1e-20)
			return 0;
		if (abs(l.b) < 1e-20)
			return GE0_VALUE_MAX;
		return -(l.a / l.b);
	}
	// 返回直线的倾斜角G_Alpha ( 0 - pi) 
	double G_Alpha(LINE l)
	{
		if (abs(l.a)< GE0_VALUE_MIN)
			return 0;
		if (abs(l.b)< GE0_VALUE_MIN)
			return GE0_VALUE_PI / 2;
		double k = G_Slope(l);
		if (k>0)
			return atan(k);
		else
			return GE0_VALUE_PI + atan(k);
	}
	// 求点p关于直线l的对称点  
	HSV::Point2d G_Symmetry(LINE l, HSV::Point2d p)
	{
		HSV::Point2d tp;
		tp.x = ((l.b*l.b - l.a*l.a)*p.x - 2 * l.a*l.b*p.y - 2 * l.a*l.c) / (l.a*l.a + l.b*l.b);
		tp.y = ((l.a*l.a - l.b*l.b)*p.y - 2 * l.a*l.b*p.x - 2 * l.b*l.c) / (l.a*l.a + l.b*l.b);
		return tp;
	}
	// 如果两条直线 l1(a1*x+b1*y+c1 = 0), l2(a2*x+b2*y+c2 = 0)相交，返回true，且返回交点p  
	bool G_IsLineIntersect(LINE l1, LINE l2, HSV::Point2d &p) // 是 L1，L2 
	{
		double d = l1.a*l2.b - l2.a*l1.b;
		if (abs(d)<GE0_VALUE_MIN) // 不相交 
			return false;
		p.x = (l2.c*l1.b - l1.c*l2.b) / d;
		p.y = (l2.a*l1.c - l1.a*l2.c) / d;
		return true;
	}
	// 如果线段l1和l2相交，返回true且交点由(inter)返回，否则返回false 
	bool G_IsIntersection(LINESEG l1, LINESEG l2, HSV::Point2d &inter)
	{
		LINE ll1, ll2;
		ll1 = G_MakeLine(l1.s, l1.e);
		ll2 = G_MakeLine(l2.s, l2.e);
		if (G_IsLineIntersect(ll1, ll2, inter))
			return G_Online(l1, inter);
		else
			return false;
	}

	/******************************\
	*							  *
	* 多边形常用算法模块		  *
	*							  *
	\******************************/

	// 如果无特别说明，输入多边形顶点要求按逆时针排列 


	/**********************************************
	将指定的点集进行逆时针排序
	G_POINTSet为输入的点集；
	ch为输出点集，按照逆时针方向排列;
	n为G_POINTSet中的点的数目
	//要求，点是互异点
	**********************************************/
	void G_QueneUnGetClock(HSV::Point2d G_POINTSet[], int n)
	{
		int i, j, k = 0, top = 2;
		HSV::Point2d tmp;
		// 选取G_POINTSet中y坐标最小的点G_POINTSet[k]，如果这样的点有多个，则取最左边的一个 
		for (i = 1; i<n; i++)
			if (G_POINTSet[i].y<G_POINTSet[k].y || (G_POINTSet[i].y == G_POINTSet[k].y) && (G_POINTSet[i].x<G_POINTSet[k].x))
				k = i;
		tmp = G_POINTSet[0];
		G_POINTSet[0] = G_POINTSet[k];
		G_POINTSet[k] = tmp; // 现在G_POINTSet中y坐标最小的点在G_POINTSet[0] 
		for (i = 1; i<n - 1; i++) /* 对顶点按照相对G_POINTSet[0]的极角从小到大进行排序，极角相同的按照距离G_POINTSet[0]从近到远进行排序 */
		{
			k = i;
			for (j = i + 1; j<n; j++)
				if (G_Multiply(G_POINTSet[j], G_POINTSet[k], G_POINTSet[0])>0 ||  // 极角更小    
					(G_Multiply(G_POINTSet[j], G_POINTSet[k], G_POINTSet[0]) == 0) && /* 极角相等，距离更短 */
					G_Dist(G_POINTSet[0], G_POINTSet[j])<G_Dist(G_POINTSet[0], G_POINTSet[k])
					)
					k = j;
			tmp = G_POINTSet[i];
			G_POINTSet[i] = G_POINTSet[k];
			G_POINTSet[k] = tmp;
		}
	}
	/*
	返回值：输入的多边形是简单多边形，返回true ，简单多边形是边不相交的多边形
	要 求：输入顶点序列按逆时针排序
	说 明：简单多边形定义：
	1：循环排序中相邻线段对的交是他们之间共有的单个点
	2：不相邻的线段不相交
	本程序默认第一个条件已经满足
	*/
	bool G_IsSimple(int vcount, HSV::Point2d polygon[])
	{
		int i, cn;
		LINESEG l1, l2;
		for (i = 0; i<vcount; i++)
		{
			l1.s = polygon[i];
			l1.e = polygon[(i + 1) % vcount];
			cn = vcount - 3;
			while (cn)
			{
				l2.s = polygon[(i + 2) % vcount];
				l2.e = polygon[(i + 3) % vcount];
				if (G_Intersect(l1, l2))
					break;
				cn--;
			}
			if (cn)
				return false;
		}
		return true;
	}
	// 返回值：按输入顺序返回多边形顶点的凸凹性判断，bc[i]=1,iff:第i个顶点是凸顶点 
	void G_CheckConvex(int vcount, HSV::Point2d polygon[], bool bc[])
	{
		int i, index = 0;
		HSV::Point2d tp = polygon[0];
		for (i = 1; i<vcount; i++) // 寻找第一个凸顶点 
		{
			if (polygon[i].y<tp.y || (polygon[i].y == tp.y&&polygon[i].x<tp.x))
			{
				tp = polygon[i];
				index = i;
			}
		}
		int count = vcount - 1;
		bc[index] = 1;
		while (count) // 判断凸凹性 
		{
			if (G_Multiply(polygon[(index + 1) % vcount], polygon[(index + 2) % vcount], polygon[index]) >= 0)
				bc[(index + 1) % vcount] = 1;
			else
				bc[(index + 1) % vcount] = 0;
			index++;
			count--;
		}
	}
	// 返回值：多边形polygon是凸多边形时，返回true  
	bool G_IsConvex(int vcount, HSV::Point2d polygon[])
	{
		bool bc[GE0_VALUE_POLYPT_MAX];
		G_CheckConvex(vcount, polygon, bc);
		for (int i = 0; i<vcount; i++) // 逐一检查顶点，是否全部是凸顶点 
			if (!bc[i])
				return false;
		return true;
	}
	// 返回多边形面积(signed)；输入顶点按逆时针排列时，返回正值；否则返回负值 
	double G_AreaOfPolygon(int vcount, HSV::Point2d polygon[])
	{
		int i;
		double s;
		if (vcount<3)
			return 0;
		s = polygon[0].y*(polygon[vcount - 1].x - polygon[1].x);
		for (i = 1; i<vcount; i++)
			s += polygon[i].y*(polygon[(i - 1)].x - polygon[(i + 1) % vcount].x);
		return s / 2;
	}
	// 如果输入顶点按逆时针排列，返回true 
	bool G_IsConterGetClock(int vcount, HSV::Point2d polygon[])
	{
		return G_AreaOfPolygon(vcount, polygon)>0;
	}
	// 另一种判断多边形顶点排列方向的方法  
	bool G_IsCcwize(int vcount, HSV::Point2d polygon[])
	{
		int i, index;
		HSV::Point2d a, b, v;
		v = polygon[0];
		index = 0;
		for (i = 1; i<vcount; i++) // 找到最低且最左顶点，肯定是凸顶点 
		{
			if (polygon[i].y<v.y || polygon[i].y == v.y && polygon[i].x<v.x)
			{
				index = i;
			}
		}
		a = polygon[(index - 1 + vcount) % vcount]; // 顶点v的前一顶点 
		b = polygon[(index + 1) % vcount]; // 顶点v的后一顶点 
		return G_Multiply(v, b, a)>0;
	}
	/********************************************************************************************
	射线法判断点q与多边形polygon的位置关系，要求polygon为简单多边形，顶点逆时针排列
	如果点在多边形内：   返回0
	如果点在多边形边上： 返回1
	如果点在多边形外：	返回2
	*********************************************************************************************/
	int G_InsidePolygon(int vcount, HSV::Point2d Polygon[], HSV::Point2d q)
	{
		int c = 0, i, n;
		LINESEG l1, l2;
		bool bintersect_a, bonline1, bonline2, bonline3;
		double r1, r2;

		l1.s = q;
		l1.e = q;
		l1.e.x = double(GE0_VALUE_MAX);
		n = vcount;
		for (i = 0; i<vcount; i++)
		{
			l2.s = Polygon[i];
			l2.e = Polygon[(i + 1) % n];
			if (G_Online(l2, q))
				return 1; // 如果点在边上，返回1 
			if ((bintersect_a = G_Intersect_A(l1, l2)) || // 相交且不在端点 
				((bonline1 = G_Online(l1, Polygon[(i + 1) % n])) && // 第二个端点在射线上 
				((!(bonline2 = G_Online(l1, Polygon[(i + 2) % n]))) && /* 前一个端点和后一个端点在射线两侧 */
					((r1 = G_Multiply(Polygon[i], Polygon[(i + 1) % n], l1.s)*G_Multiply(Polygon[(i + 1) % n], Polygon[(i + 2) % n], l1.s))>0) ||
					(bonline3 = G_Online(l1, Polygon[(i + 2) % n])) &&     /* 下一条边是水平线，前一个端点和后一个端点在射线两侧  */
					((r2 = G_Multiply(Polygon[i], Polygon[(i + 2) % n], l1.s)*G_Multiply(Polygon[(i + 2) % n],
						Polygon[(i + 3) % n], l1.s))>0)
					)
					)
				) c++;
		}
		if (c % 2 == 1)
			return 0;
		else
			return 2;
	}
	//点q是凸多边形polygon内时，返回true；注意：多边形polygon一定要是凸多边形  
	bool G_InsideConvexPolygon(int vcount, HSV::Point2d polygon[], HSV::Point2d q) // 可用于三角形！ 
	{
		HSV::Point2d p;
		LINESEG l;
		int i;
		p.x = 0; p.y = 0;
		for (i = 0; i<vcount; i++) // 寻找一个肯定在多边形polygon内的点p：多边形顶点平均值 
		{
			p.x += polygon[i].x;
			p.y += polygon[i].y;
		}
		p.x /= vcount;
		p.y /= vcount;

		for (i = 0; i<vcount; i++)
		{
			l.s = polygon[i]; l.e = polygon[(i + 1) % vcount];
			if (G_Multiply(p, l.e, l.s)*G_Multiply(q, l.e, l.s)<0) /* 点p和点q在边l的两侧，说明点q肯定在多边形外 */
				break;
		}
		return (i == vcount);
	}
	/**********************************************
	寻找凸包的graham 扫描法
	G_POINTSet为输入的点集；
	ch为输出的凸包上的点集，按照逆时针方向排列;
	n为G_POINTSet中的点的数目
	len为输出的凸包上的点的个数
	**********************************************/
	void G_Graham_scan(HSV::Point2d G_POINTSet[], HSV::Point2d ch[], int n, int &len)
	{
		int i, j, k = 0, top = 2;
		HSV::Point2d tmp;
		// 选取G_POINTSet中y坐标最小的点G_POINTSet[k]，如果这样的点有多个，则取最左边的一个 
		for (i = 1; i<n; i++)
			if (G_POINTSet[i].y<G_POINTSet[k].y || (G_POINTSet[i].y == G_POINTSet[k].y) && (G_POINTSet[i].x<G_POINTSet[k].x))
				k = i;
		tmp = G_POINTSet[0];
		G_POINTSet[0] = G_POINTSet[k];
		G_POINTSet[k] = tmp; // 现在G_POINTSet中y坐标最小的点在G_POINTSet[0] 
		for (i = 1; i<n - 1; i++) /* 对顶点按照相对G_POINTSet[0]的极角从小到大进行排序，极角相同的按照距离G_POINTSet[0]从近到远进行排序 */
		{
			k = i;
			for (j = i + 1; j<n; j++)
				if (G_Multiply(G_POINTSet[j], G_POINTSet[k], G_POINTSet[0])>0 ||  // 极角更小    
					(G_Multiply(G_POINTSet[j], G_POINTSet[k], G_POINTSet[0]) == 0) && /* 极角相等，距离更短 */
					G_Dist(G_POINTSet[0], G_POINTSet[j])<G_Dist(G_POINTSet[0], G_POINTSet[k])
					)
					k = j;
			tmp = G_POINTSet[i];
			G_POINTSet[i] = G_POINTSet[k];
			G_POINTSet[k] = tmp;
		}
		ch[0] = G_POINTSet[0];
		ch[1] = G_POINTSet[1];
		ch[2] = G_POINTSet[2];
		for (i = 3; i<n; i++)
		{
			while (G_Multiply(G_POINTSet[i], ch[top], ch[top - 1]) >= 0)
				top--;
			ch[++top] = G_POINTSet[i];
		}
		len = top + 1;
	}
	// 卷包裹法求点集凸壳，参数说明同graham算法    
	void G_ConvexClosure(HSV::Point2d G_POINTSet[], HSV::Point2d ch[], int n, int &len)
	{
		int top = 0, i, index, first;
		double curmax, curcos, curdis;
		HSV::Point2d tmp;
		LINESEG l1, l2;
		bool use[GE0_VALUE_POLYPT_MAX];
		tmp = G_POINTSet[0];
		index = 0;
		// 选取y最小点，如果多于一个，则选取最左点 
		for (i = 1; i<n; i++)
		{
			if (G_POINTSet[i].y<tmp.y || G_POINTSet[i].y == tmp.y&&G_POINTSet[i].x<tmp.x)
			{
				index = i;
			}
			use[i] = false;
		}
		tmp = G_POINTSet[index];
		first = index;
		use[index] = true;

		index = -1;
		ch[top++] = tmp;
		tmp.x -= 100;
		l1.s = tmp;
		l1.e = ch[0];
		l2.s = ch[0];

		while (index != first)
		{
			curmax = -100;
			curdis = 0;
			// 选取与最后一条确定边夹角最小的点，即余弦值最大者 
			for (i = 0; i<n; i++)
			{
				if (use[i])continue;
				l2.e = G_POINTSet[i];
				curcos = G_Cosine(l1, l2); // 根据cos值求夹角余弦，范围在 （-1 -- 1 ） 
				if (curcos>curmax || fabs(curcos - curmax)<1e-6 && G_Dist(l2.s, l2.e)>curdis)
				{
					curmax = curcos;
					index = i;
					curdis = G_Dist(l2.s, l2.e);
				}
			}
			use[first] = false;            //清空第first个顶点标志，使最后能形成封闭的hull 
			use[index] = true;
			ch[top++] = G_POINTSet[index];
			l1.s = ch[top - 2];
			l1.e = ch[top - 1];
			l2.s = ch[top - 1];
		}
		len = top - 1;
	}
	/*********************************************************************************************
	判断线段是否在简单多边形内(注意：如果多边形是凸多边形，下面的算法可以化简)
	必要条件一：线段的两个端点都在多边形内；
	必要条件二：线段和多边形的所有边都不内交；
	用途：	1. 判断折线是否在简单多边形内
	2. 判断简单多边形是否在另一个简单多边形内
	**********************************************************************************************/
	bool G_LinesegInsidePolygon(int vcount, HSV::Point2d polygon[], LINESEG l)
	{
		// 判断线端l的端点是否不都在多边形内 
		if (!G_InsidePolygon(vcount, polygon, l.s) || !G_InsidePolygon(vcount, polygon, l.e))
			return false;
		int top = 0, i, j;
		HSV::Point2d G_POINTSet[GE0_VALUE_POLYPT_MAX], tmp;
		LINESEG s;

		for (i = 0; i<vcount; i++)
		{
			s.s = polygon[i];
			s.e = polygon[(i + 1) % vcount];
			if (G_Online(s, l.s)) //线段l的起始端点在线段s上 
				G_POINTSet[top++] = l.s;
			else if (G_Online(s, l.e)) //线段l的终止端点在线段s上 
				G_POINTSet[top++] = l.e;
			else
			{
				if (G_Online(l, s.s)) //线段s的起始端点在线段l上 
					G_POINTSet[top++] = s.s;
				else if (G_Online(l, s.e)) // 线段s的终止端点在线段l上 
					G_POINTSet[top++] = s.e;
				else
				{
					if (G_Intersect(l, s)) // 这个时候如果相交，肯定是内交，返回false 
						return false;
				}
			}
		}

		for (i = 0; i<top - 1; i++) /* 冒泡排序，x坐标小的排在前面；x坐标相同者，y坐标小的排在前面 */
		{
			for (j = i + 1; j<top; j++)
			{
				if (G_POINTSet[i].x>G_POINTSet[j].x || fabs(G_POINTSet[i].x - G_POINTSet[j].x)<GE0_VALUE_MIN && G_POINTSet[i].y>G_POINTSet[j].y)
				{
					tmp = G_POINTSet[i];
					G_POINTSet[i] = G_POINTSet[j];
					G_POINTSet[j] = tmp;
				}
			}
		}

		for (i = 0; i<top - 1; i++)
		{
			tmp.x = (G_POINTSet[i].x + G_POINTSet[i + 1].x) / 2; //得到两个相邻交点的中点 
			tmp.y = (G_POINTSet[i].y + G_POINTSet[i + 1].y) / 2;
			if (!G_InsidePolygon(vcount, polygon, tmp))
				return false;
		}
		return true;
	}
	/*********************************************************************************************
	求任意简单多边形polygon的重心
	需要调用下面几个函数：
	void G_AddPosPart(); 增加右边区域的面积
	void G_AddNegPart(); 增加左边区域的面积
	void G_AddRegion(); 增加区域面积
	在使用该程序时，如果把xtr,ytr,wtr,xtl,ytl,wtl设成全局变量就可以使这些函数的形式得到化简,
	但要注意函数的声明和调用要做相应变化
	**********************************************************************************************/
	void G_AddPosPart(double x, double y, double w, double &xtr, double &ytr, double &wtr)
	{
		if (abs(wtr + w)<1e-10) return; // detect zero regions 
		xtr = (wtr*xtr + w*x) / (wtr + w);
		ytr = (wtr*ytr + w*y) / (wtr + w);
		wtr = w + wtr;
		return;
	}
	void G_AddNegPart(double x, double y, double w, double &xtl, double &ytl, double &wtl)
	{
		if (abs(wtl + w)<1e-10)
			return; // detect zero regions 

		xtl = (wtl*xtl + w*x) / (wtl + w);
		ytl = (wtl*ytl + w*y) / (wtl + w);
		wtl = w + wtl;
		return;
	}
	void G_AddRegion(double x1, double y1, double x2, double y2, double &xtr, double &ytr,
		double &wtr, double &xtl, double &ytl, double &wtl)
	{
		if (abs(x1 - x2)< 1e-10)
			return;

		if (x2 > x1)
		{
			G_AddPosPart((x2 + x1) / 2, y1 / 2, (x2 - x1) * y1, xtr, ytr, wtr); /* rectangle 全局变量变化处 */
			G_AddPosPart((x1 + x2 + x2) / 3, (y1 + y1 + y2) / 3, (x2 - x1)*(y2 - y1) / 2, xtr, ytr, wtr);
			// triangle 全局变量变化处 
		}
		else
		{
			G_AddNegPart((x2 + x1) / 2, y1 / 2, (x2 - x1) * y1, xtl, ytl, wtl);
			// rectangle 全局变量变化处 
			G_AddNegPart((x1 + x2 + x2) / 3, (y1 + y1 + y2) / 3, (x2 - x1)*(y2 - y1) / 2, xtl, ytl, wtl);
			// triangle  全局变量变化处 
		}
	}
	HSV::Point2d G_CGSimple(int vcount, HSV::Point2d polygon[])
	{
		double xtr, ytr, wtr, xtl, ytl, wtl;
		//注意： 如果把xtr,ytr,wtr,xtl,ytl,wtl改成全局变量后这里要删去 
		HSV::Point2d p1, p2, tp;
		xtr = ytr = wtr = 0.0;
		xtl = ytl = wtl = 0.0;
		for (int i = 0; i<vcount; i++)
		{
			p1 = polygon[i];
			p2 = polygon[(i + 1) % vcount];
			G_AddRegion(p1.x, p1.y, p2.x, p2.y, xtr, ytr, wtr, xtl, ytl, wtl); //全局变量变化处 
		}
		tp.x = (wtr*xtr + wtl*xtl) / (wtr + wtl);
		tp.y = (wtr*ytr + wtl*ytl) / (wtr + wtl);
		return tp;
	}
	// 求凸多边形的重心,要求输入多边形按逆时针排序 
	HSV::Point2d G_GravityCenter(int vcount, HSV::Point2d polygon[])
	{
		HSV::Point2d tp;
		double x, y, s, x0, y0, cs, k;
		x = 0; y = 0; s = 0;
		for (int i = 1; i<vcount - 1; i++)
		{
			x0 = (polygon[0].x + polygon[i].x + polygon[i + 1].x) / 3;
			y0 = (polygon[0].y + polygon[i].y + polygon[i + 1].y) / 3; //求当前三角形的重心 
			cs = G_Multiply(polygon[i], polygon[i + 1], polygon[0]) / 2;
			//三角形面积可以直接利用该公式求解 
			if (abs(s)<1e-20)
			{
				x = x0; y = y0; s += cs; continue;
			}
			k = cs / s; //求面积比例 
			x = (x + k*x0) / (1 + k);
			y = (y + k*y0) / (1 + k);
			s += cs;
		}
		tp.x = x;
		tp.y = y;
		return tp;
	}

	/************************************************
	给定一简单多边形，找出一个肯定在该多边形内的点
	定理1	：每个多边形至少有一个凸顶点
	定理2	：顶点数>=4的简单多边形至少有一条对角线
	结论	： x坐标最大，最小的点肯定是凸顶点
	y坐标最大，最小的点肯定是凸顶点
	************************************************/
	HSV::Point2d G_APtInsidePoly(int vcount, HSV::Point2d polygon[])
	{
		HSV::Point2d v, a, b, r;
		int i, index;
		v = polygon[0];
		index = 0;
		for (i = 1; i<vcount; i++) //寻找一个凸顶点 
		{
			if (polygon[i].y<v.y)
			{
				v = polygon[i];
				index = i;
			}
		}
		a = polygon[(index - 1 + vcount) % vcount]; //得到v的前一个顶点 
		b = polygon[(index + 1) % vcount]; //得到v的后一个顶点 
		HSV::Point2d tri[3], q;
		tri[0] = a; tri[1] = v; tri[2] = b;
		double md = GE0_VALUE_MAX;
		bool bin = false;
		for (i = 0; i<vcount; i++) //寻找在三角形avb内且离顶点v最近的顶点q 
		{
			if (i == index)continue;
			if (i == (index - 1 + vcount) % vcount)continue;
			if (i == (index + 1) % vcount)continue;
			if (!G_InsideConvexPolygon(3, tri, polygon[i]))continue;
			bin = true;
			if (G_Dist(v, polygon[i])<md)
			{
				q = polygon[i];
				md = G_Dist(v, q);
			}
		}
		if (!bin) //没有顶点在三角形avb内，返回线段ab中点 
		{
			r.x = (a.x + b.x) / 2;
			r.y = (a.y + b.y) / 2;
			return r;
		}
		r.x = (v.x + q.x) / 2; //返回线段vq的中点 
		r.y = (v.y + q.y) / 2;
		return r;
	}
	/***********************************************************************************************
	求从多边形外一点p出发到一个简单多边形的切线,如果存在返回切点,其中rp点是右切点,lp是左切点
	注意：p点一定要在多边形外 ,输入顶点序列是逆时针排列
	原 理：	如果点在多边形内肯定无切线;凸多边形有唯一的两个切点,凹多边形就可能有多于两个的切点;
	如果polygon是凸多边形，切点只有两个只要找到就可以,可以化简此算法
	如果是凹多边形还有一种算法可以求解:先求凹多边形的凸包,然后求凸包的切线
	/***********************************************************************************************/
	void G_PtTangentPoly(int vcount, HSV::Point2d polygon[], HSV::Point2d p, HSV::Point2d &rp, HSV::Point2d &lp)
	{
		LINESEG ep, en;
		bool blp, bln;
		rp = polygon[0];
		lp = polygon[0];
		for (int i = 1; i<vcount; i++)
		{
			ep.s = polygon[(i + vcount - 1) % vcount];
			ep.e = polygon[i];
			en.s = polygon[i];
			en.e = polygon[(i + 1) % vcount];
			blp = G_Multiply(ep.e, p, ep.s) >= 0;                // p is to the left of pre edge 
			bln = G_Multiply(en.e, p, en.s) >= 0;                // p is to the left of next edge 
			if (!blp&&bln)
			{
				if (G_Multiply(polygon[i], rp, p)>0)           // polygon[i] is above rp 
					rp = polygon[i];
			}
			if (blp && !bln)
			{
				if (G_Multiply(lp, polygon[i], p)>0)           // polygon[i] is below lp 
					lp = polygon[i];
			}
		}
		return;
	}
	// 如果多边形polygon的核存在，返回true，返回核上的一点p.顶点按逆时针方向输入  
	bool G_IsCoreExist(int vcount, HSV::Point2d polygon[], HSV::Point2d &p)
	{
		int i, j, k;
		LINESEG l;
		LINE lineset[GE0_VALUE_POLYPT_MAX];
		for (i = 0; i<vcount; i++)
		{
			lineset[i] = G_MakeLine(polygon[i], polygon[(i + 1) % vcount]);
		}
		for (i = 0; i<vcount; i++)
		{
			for (j = 0; j<vcount; j++)
			{
				if (i == j)continue;
				if (G_IsLineIntersect(lineset[i], lineset[j], p))
				{
					for (k = 0; k<vcount; k++)
					{
						l.s = polygon[k];
						l.e = polygon[(k + 1) % vcount];
						if (G_Multiply(p, l.e, l.s)>0)
							//多边形顶点按逆时针方向排列，核肯定在每条边的左侧或边上 
							break;
					}
					if (k == vcount)             //找到了一个核上的点 
						break;
				}
			}
			if (j<vcount) break;
		}
		if (i<vcount)
			return true;
		else
			return false;
	}
	/*************************\
	*						 *
	* 圆的基本运算           *
	*					     *
	\*************************/
	/******************************************************************************
	返回值	： 点p在圆内(包括边界)时，返回true
	用途	： 因为圆为凸集，所以判断点集，折线，多边形是否在圆内时，
	只需要逐一判断点是否在圆内即可。
	*******************************************************************************/
	bool G_IsPtInCircle(HSV::Point2d o, double r, HSV::Point2d p)
	{
		double d2 = (p.x - o.x)*(p.x - o.x) + (p.y - o.y)*(p.y - o.y);
		double r2 = r*r;
		return d2<r2 || abs(d2 - r2)<GE0_VALUE_MIN;
	}

	bool G_IsPtInPolyGon(const std::vector<HSV::Point2d> &pts1D, HSV::Point2d temsorcePt)
	{
		unsigned int temcrosscount = 0;
		unsigned int temptSize = pts1D.size();
		LINESEG temline;
		LINESEG temlinesorce;
		temlinesorce.s.x = temsorcePt.x;
		temlinesorce.s.y = temsorcePt.y;
		temlinesorce.e.x = DBL_MAX;
		temlinesorce.e.y = temsorcePt.y;

		HSV::Point2d temendPt1;
		HSV::Point2d temendPt2;
		for (unsigned int i = 0; i < temptSize; i++)
		{
			temendPt1.x = temline.s.x = pts1D[i].x;
			temendPt1.y = temline.s.y = pts1D[i].y;
			temendPt2.x = temline.e.x = pts1D[(i + 1) % temptSize].x;
			temendPt2.y = temline.e.y = pts1D[(i + 1) % temptSize].y;

			if (G_Online(temline, temsorcePt))
			{
				return true;
			}
			if (abs(temline.s.y - temline.e.y) < EPSILON)
			{
				continue;
			}
			if (G_Online(temlinesorce, temendPt1))
			{
				if (temendPt1.y > temendPt2.y)
				{
					temcrosscount++;
				}
			}
			else if (G_Online(temlinesorce, temendPt2))
			{
				if (temendPt2.y > temendPt1.y)
				{
					temcrosscount++;
				}
			}
			else if (G_Intersect(temlinesorce, temline))
			{
				temcrosscount++;
			}
		}
		bool isInside = false;
		if (1 == temcrosscount % 2)
		{
			isInside = true;
		}
		return isInside;
	}

	/******************************************************************************
	用 途	：求不共线的三点确定一个圆
	输 入	：三个点p1,p2,p3
	返回值	：如果三点共线，返回false；反之，返回true。圆心由q返回，半径由r返回
	*******************************************************************************/
	bool G_Pt2Circle(HSV::Point2d p1, HSV::Point2d p2, HSV::Point2d p3, HSV::Point2d &q, double &r)
	{
		double x12 = p2.x - p1.x;
		double y12 = p2.y - p1.y;
		double x13 = p3.x - p1.x;
		double y13 = p3.y - p1.y;
		double z2 = x12*(p1.x + p2.x) + y12*(p1.y + p2.y);
		double z3 = x13*(p1.x + p3.x) + y13*(p1.y + p3.y);
		double d = 2.0*(x12*(p3.y - p2.y) - y12*(p3.x - p2.x));
		if (abs(d)<GE0_VALUE_MIN) //共线，圆不存在 
			return false;
		q.x = (y13*z2 - y12*z3) / d;
		q.y = (x12*z3 - x13*z2) / d;
		r = G_Dist(p1, q);
		return true;
	}

	int G_LineCircle(LINE l, HSV::Point2d o, double r, HSV::Point2d &p1, HSV::Point2d &p2)
	{
		return true;
	}

	/**
	* 最小二乘法拟合圆
	* 拟合出的圆以圆心坐标和半径的形式表示
	*/
	bool G_CircleLeastFit(const std::vector<HSV::Point2d> &pts1D, HSV::Point2d& cnter, double &radius)
	{
		cnter.x = 0.0f;
		cnter.y = 0.0f;
		radius = 0.0f;
		if (pts1D.size() < 3)
			return false;
		double x = 0, y = 0;
		double x2 = 0, y2 = 0;
		double sum_x = 0.0f, sum_y = 0.0f;
		double sum_x2 = 0.0f, sum_y2 = 0.0f;
		double sum_x3 = 0.0f, sum_y3 = 0.0f;
		double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;
		unsigned int ptNum = (unsigned int)pts1D.size();
		for (unsigned int i = 0; i < ptNum; i++)
		{
			x = pts1D[i].x;
			y = pts1D[i].y;
			x2 = x * x;
			y2 = y * y;
			sum_x += x;
			sum_y += y;
			sum_x2 += x2;
			sum_y2 += y2;
			sum_x3 += x2 * x;
			sum_y3 += y2 * y;
			sum_xy += x * y;
			sum_x1y2 += x * y2;
			sum_x2y1 += x2 * y;
		}
		double C, D, E, G, H;
		double a, b, c;
		C = ptNum * sum_x2 - sum_x * sum_x;
		D = ptNum * sum_xy - sum_x * sum_y;
		E = ptNum * sum_x3 + ptNum * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
		G = ptNum * sum_y2 - sum_y * sum_y;
		H = ptNum * sum_x2y1 + ptNum * sum_y3 - (sum_x2 + sum_y2) * sum_y;
		a = (H * D - E * G) / (C * G - D * D);
		b = (H * C - E * D) / (D * D - G * C);
		c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / ptNum;
		cnter.x = a / (-2);
		cnter.y = b / (-2);
		radius = sqrt(a * a + b * b - 4 * c) / 2;
		return true;
	}

	/**************************\
	*						  *
	* 矩形的基本运算          *
	*                         *
	\**************************/
	/*
	说明：因为矩形的特殊性，常用算法可以化简：
	1.判断矩形是否包含点
	只要判断该点的横坐标和纵坐标是否夹在矩形的左右边和上下边之间。
	2.判断线段、折线、多边形是否在矩形中
	因为矩形是个凸集，所以只要判断所有端点是否都在矩形中就可以了。
	3.判断圆是否在矩形中
	圆在矩形中的充要条件是：圆心在矩形中且圆的半径小于等于圆心到矩形四边的距离的最小值。
	*/
	// 已知矩形的三个顶点(a,b,c)，计算第四个顶点d的坐标. 注意：已知的三个顶点可以是无序的 
	HSV::Point2d G_Rect4th(HSV::Point2d a, HSV::Point2d b, HSV::Point2d c)
	{
		HSV::Point2d d;
		if (abs(G_DotMultiply(a, b, c))<GE0_VALUE_MIN) // 说明c点是直角拐角处 
		{
			d.x = a.x + b.x - c.x;
			d.y = a.y + b.y - c.y;
		}
		if (abs(G_DotMultiply(a, c, b))<GE0_VALUE_MIN) // 说明b点是直角拐角处 
		{
			d.x = a.x + c.x - b.x;
			d.y = a.y + c.y - b.x;
		}
		if (abs(G_DotMultiply(c, b, a))<GE0_VALUE_MIN) // 说明a点是直角拐角处 
		{
			d.x = c.x + b.x - a.x;
			d.y = c.y + b.y - a.y;
		}
		return d;
	}


	/********************\
	*				    *
	* 补充				*
	*					*
	\********************/

	//两圆关系： 
	/* 两圆：
	相离： return 1；
	外切： return 2；
	相交： return 3；
	内切： return 4；
	内含： return 5；
	*/
	int G_CircleRelation(HSV::Point2d p1, double r1, HSV::Point2d p2, double r2)
	{
		double d = sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));

		if (fabs(d - r1 - r2) < GE0_VALUE_MIN) // 必须保证前两个if先被判定！ 
			return 2;
		if (fabs(d - fabs(r1 - r2)) < GE0_VALUE_MIN)
			return 4;
		if (d > r1 + r2)
			return 1;
		if (d < fabs(r1 - r2))
			return 5;
		if (fabs(r1 - r2) < d && d < r1 + r2)
			return 3;
		return 0; // indicate an error! 
	}
	//判断圆是否在矩形内：
	// 判定圆是否在矩形内，是就返回true（设矩形水平，且其四个顶点由左上开始按顺时针排列） 
	// 调用G_Pt2LineDist函数，在第4页 
	bool G_CircleRecRelation(HSV::Point2d pc, double r, HSV::Point2d pr1, HSV::Point2d pr2, HSV::Point2d pr3, HSV::Point2d pr4)
	{
		if (pr1.x < pc.x && pc.x < pr2.x && pr3.y < pc.y && pc.y < pr2.y)
		{
			LINESEG line1(pr1, pr2);
			LINESEG line2(pr2, pr3);
			LINESEG line3(pr3, pr4);
			LINESEG line4(pr4, pr1);
			if (r<G_Pt2LineDist(pc, line1) && r<G_Pt2LineDist(pc, line2) && r<G_Pt2LineDist(pc, line3) && r<G_Pt2LineDist(pc, line4))
				return true;
		}
		return false;
	}
	//点到平面的距离： 
	//点到平面的距离,平面用一般式表示ax+by+cz+d=0 
	double G_P2planeDist(double x, double y, double z, double a, double b, double c, double d)
	{
		return fabs(a*x + b*y + c*z + d) / sqrt(a*a + b*b + c*c);
	}
	//点是否在直线同侧：
	//两个点是否在直线同侧，是则返回true 
	bool G_SameSide(HSV::Point2d p1, HSV::Point2d p2, LINE line)
	{
		return (line.a * p1.x + line.b * p1.y + line.c) *
			(line.a * p2.x + line.b * p2.y + line.c) > 0;
	}
	//镜面反射线：
	// 已知入射线、镜面，求反射线。 
	// a1,b1,c1为镜面直线方程(a1 x + b1 y + c1 = 0 ,下同)系数;  
	//a2,b2,c2为入射光直线方程系数;  
	//a,b,c为反射光直线方程系数. 
	// 光是有方向的，使用时注意：入射光向量:<-b2,a2>；反射光向量:<b,-a>. 
	// 不要忘记结果中可能会有"negative zeros" 
	void G_Reflect(double a1, double b1, double c1, double a2, double b2, double c2, double &a, double &b, double &c)
	{
		double n, m;
		double tpb, tpa;
		tpb = b1*b2 + a1*a2;
		tpa = a2*b1 - a1*b2;
		m = (tpb*b1 + tpa*a1) / (b1*b1 + a1*a1);
		n = (tpa*b1 - tpb*a1) / (b1*b1 + a1*a1);
		if (fabs(a1*b2 - a2*b1)<1e-20)
		{
			a = a2; b = b2; c = c2;
			return;
		}
		double xx, yy; //(xx,yy)是入射线与镜面的交点。 
		xx = (b1*c2 - b2*c1) / (a1*b2 - a2*b1);
		yy = (a2*c1 - a1*c2) / (a1*b2 - a2*b1);
		a = n;
		b = -m;
		c = m*yy - xx*n;
	}
	//矩形包含： 
	// 矩形2（C，D）是否在1（A，B）内
	bool G_R2inr1(double A, double B, double C, double D)
	{
		double X, Y, L, K, DMax;
		if (A < B)
		{
			double tmp = A;
			A = B;
			B = tmp;
		}
		if (C < D)
		{
			double tmp = C;
			C = D;
			D = tmp;
		}
		if (A > C && B > D)                 // trivial case  
			return true;
		else
			if (D >= B)
				return false;
			else
			{
				X = sqrt(A * A + B * B);         // outer rectangle's diagonal  
				Y = sqrt(C * C + D * D);         // inner rectangle's diagonal  
				if (Y < B) // check for marginal conditions 
					return true; // the inner rectangle can freely rotate inside 
				else
					if (Y > X)
						return false;
					else
					{
						L = (B - sqrt(Y * Y - A * A)) / 2;
						K = (A - sqrt(Y * Y - B * B)) / 2;
						DMax = sqrt(L * L + K * K);
						if (D >= DMax)
							return false;
						else
							return true;
					}
			}
	}
	//两圆交点： 
	// 两圆已经相交（相切） 
	void  G_Circle2Pt(HSV::Point2d p1, double r1, HSV::Point2d p2, double r2, HSV::Point2d &rp1, HSV::Point2d &rp2)
	{
		double a, b, r;
		a = p2.x - p1.x;
		b = p2.y - p1.y;
		r = (a*a + b*b + r1*r1 - r2*r2) / 2;
		if (a == 0 && b != 0)
		{
			rp1.y = rp2.y = r / b;
			rp1.x = sqrt(r1*r1 - rp1.y*rp1.y);
			rp2.x = -rp1.x;
		}
		else if (a != 0 && b == 0)
		{
			rp1.x = rp2.x = r / a;
			rp1.y = sqrt(r1*r1 - rp1.x*rp2.x);
			rp2.y = -rp1.y;
		}
		else if (a != 0 && b != 0)
		{
			double delta;
			delta = b*b*r*r - (a*a + b*b)*(r*r - r1*r1*a*a);
			rp1.y = (b*r + sqrt(delta)) / (a*a + b*b);
			rp2.y = (b*r - sqrt(delta)) / (a*a + b*b);
			rp1.x = (r - b*rp1.y) / a;
			rp2.x = (r - b*rp2.y) / a;
		}

		rp1.x += p1.x;
		rp1.y += p1.y;
		rp2.x += p1.x;
		rp2.y += p1.y;
	}


	//两圆公共面积：
	// 必须保证相交 
	double G_Circle2Area(HSV::Point2d p1, double r1, HSV::Point2d p2, double r2)
	{
		HSV::Point2d rp1, rp2;
		G_Circle2Pt(p1, r1, p2, r2, rp1, rp2);

		if (r1>r2) //保证r2>r1 
		{
			G_Swap(p1, p2);
			G_Swap(r1, r2);
		}
		double a, b, rr;
		a = p1.x - p2.x;
		b = p1.y - p2.y;
		rr = sqrt(a*a + b*b);

		double dx1, dy1, dx2, dy2;
		double sita1, sita2;
		dx1 = rp1.x - p1.x;
		dy1 = rp1.y - p1.y;
		dx2 = rp2.x - p1.x;
		dy2 = rp2.y - p1.y;
		sita1 = acos((dx1*dx2 + dy1*dy2) / r1 / r1);

		dx1 = rp1.x - p2.x;
		dy1 = rp1.y - p2.y;
		dx2 = rp2.x - p2.x;
		dy2 = rp2.y - p2.y;
		sita2 = acos((dx1*dx2 + dy1*dy2) / r2 / r2);
		double s = 0;
		if (rr<r2)//相交弧为优弧 
			s = r1*r1*(GE0_VALUE_PI - sita1 / 2 + sin(sita1) / 2) + r2*r2*(sita2 - sin(sita2)) / 2;
		else//相交弧为劣弧 
			s = (r1*r1*(sita1 - sin(sita1)) + r2*r2*(sita2 - sin(sita2))) / 2;

		return s;
	}
	//圆和直线关系： 
	//0----相离 1----相切 2----相交 
	int G_CircleLine2Pt(HSV::Point2d p, double r, double a, double b, double c, HSV::Point2d &rp1, HSV::Point2d &rp2)
	{
		int res = 0;

		c = c + a*p.x + b*p.y;
		double tmp;
		if (a == 0 && b != 0)
		{
			tmp = -c / b;
			if (r*r<tmp*tmp)
				res = 0;
			else if (r*r == tmp*tmp)
			{
				res = 1;
				rp1.y = tmp;
				rp1.x = 0;
			}
			else
			{
				res = 2;
				rp1.y = rp2.y = tmp;
				rp1.x = sqrt(r*r - tmp*tmp);
				rp2.x = -rp1.x;
			}
		}
		else if (a != 0 && b == 0)
		{
			tmp = -c / a;
			if (r*r<tmp*tmp)
				res = 0;
			else if (r*r == tmp*tmp)
			{
				res = 1;
				rp1.x = tmp;
				rp1.y = 0;
			}
			else
			{
				res = 2;
				rp1.x = rp2.x = tmp;
				rp1.y = sqrt(r*r - tmp*tmp);
				rp2.y = -rp1.y;
			}
		}
		else if (a != 0 && b != 0)
		{
			double delta;
			delta = b*b*c*c - (a*a + b*b)*(c*c - a*a*r*r);
			if (delta<0)
				res = 0;
			else if (delta == 0)
			{
				res = 1;
				rp1.y = -b*c / (a*a + b*b);
				rp1.x = (-c - b*rp1.y) / a;
			}
			else
			{
				res = 2;
				rp1.y = (-b*c + sqrt(delta)) / (a*a + b*b);
				rp2.y = (-b*c - sqrt(delta)) / (a*a + b*b);
				rp1.x = (-c - b*rp1.y) / a;
				rp2.x = (-c - b*rp2.y) / a;
			}
		}
		rp1.x += p.x;
		rp1.y += p.y;
		rp2.x += p.x;
		rp2.y += p.y;
		return res;
	}
	//内切圆： 
	void G_InCircle(HSV::Point2d p1, HSV::Point2d p2, HSV::Point2d p3, HSV::Point2d &rp, double &r)
	{
		double dx31, dy31, dx21, dy21, d31, d21, a1, b1, c1;
		dx31 = p3.x - p1.x;
		dy31 = p3.y - p1.y;
		dx21 = p2.x - p1.x;
		dy21 = p2.y - p1.y;

		d31 = sqrt(dx31*dx31 + dy31*dy31);
		d21 = sqrt(dx21*dx21 + dy21*dy21);
		a1 = dx31*d21 - dx21*d31;
		b1 = dy31*d21 - dy21*d31;
		c1 = a1*p1.x + b1*p1.y;

		double dx32, dy32, dx12, dy12, d32, d12, a2, b2, c2;
		dx32 = p3.x - p2.x;
		dy32 = p3.y - p2.y;
		dx12 = -dx21;
		dy12 = -dy21;

		d32 = sqrt(dx32*dx32 + dy32*dy32);
		d12 = d21;
		a2 = dx12*d32 - dx32*d12;
		b2 = dy12*d32 - dy32*d12;
		c2 = a2*p2.x + b2*p2.y;

		rp.x = (c1*b2 - c2*b1) / (a1*b2 - a2*b1);
		rp.y = (c2*a1 - c1*a2) / (a1*b2 - a2*b1);
		r = fabs(dy21*rp.x - dx21*rp.y + dx21*p1.y - dy21*p1.x) / d21;
	}
	//求切点： 
	// p---圆心坐标， r---圆半径， sp---圆外一点， rp1,rp2---切点坐标 
	void G_CircleCutPt(HSV::Point2d p, double r, HSV::Point2d sp, HSV::Point2d &rp1, HSV::Point2d &rp2)
	{
		HSV::Point2d p2;
		p2.x = (p.x + sp.x) / 2;
		p2.y = (p.y + sp.y) / 2;

		double dx2, dy2, r2;
		dx2 = p2.x - p.x;
		dy2 = p2.y - p.y;
		r2 = sqrt(dx2*dx2 + dy2*dy2);
		G_Circle2Pt(p, r, p2, r2, rp1, rp2);
	}
	//线段的左右旋： 
	/* l2在l1的左/右方向（l1为基准线）
	返回	0	： 重合；
	返回	1	： 右旋；
	返回	–1 ： 左旋；
	*/
	int G_Rotat(LINESEG l1, LINESEG l2)
	{
		double dx1, dx2, dy1, dy2;
		dx1 = l1.s.x - l1.e.x;
		dy1 = l1.s.y - l1.e.y;
		dx2 = l2.s.x - l2.e.x;
		dy2 = l2.s.y - l2.e.y;

		double d;
		d = dx1*dy2 - dx2*dy1;
		if (d == 0)
			return 0;
		else if (d>0)
			return -1;
		else
			return 1;
	}

	double G_Round(double val)
	{
		return (val> 0.0) ? floor(val + 0.5) : ceil(val - 0.5);
	}

	//根据螺旋中心及角度步长计算螺旋线上的点
	//HSV::Point2d centerPt,螺旋中心位置
	//double radius, 搜索半径
	//double stepPricise， 计算的步长精度
	//原理：以计算点位中心，计算出在搜索圆形半径内的所有点，然后对所有点做金字塔处理精度处理
	HSV::Point2d1D G_GetHelixPt(HSV::Point2d centerPt, double radius, double stepPricise, unsigned int method/* = 0*/)
	{
		//由内到外
		HSV::Point2d1D temMovePt1D;
		temMovePt1D.clear();
		if (0 == method)
		{
			/*int round = stepPricise * 10*/;
			int round = stepPricise * 2;
			if (round < 1)
				return temMovePt1D;
			/*if (round < 1)
			round = 2;*/
			IAPoint2D allPtData2D;
			allPtData2D.resize(round);
			for (int i = 0; i < round; i++)
			{
				//allPtData2D[i].resize(4*(i + 2));
				//allPtData2D[i].resize(4*(i + 1));
				allPtData2D[i].resize(4 * (round - i));	//@gq-20231211
			}
			int needSize = allPtData2D.size() - 2;	//最外两圈不要
			if (needSize < 1)
			{
				needSize = 1;
			}
			//正中心要加一圈扫描

			for (int i = 0; i < 4 * round; i++)
			{
				HSV::Point2d tmpPt;
				double interval = 6.28 / (4 * round);
				tmpPt.x = centerPt.x + 2000 * sin(interval * i);
				tmpPt.y = centerPt.y + 2000 * cos(interval * i);
				double lenth = (tmpPt.x - centerPt.x) * (tmpPt.x - centerPt.x) + (tmpPt.y - centerPt.y) * (tmpPt.y - centerPt.y);
				if (sqrt(lenth) <= radius)
					temMovePt1D.push_back(tmpPt);
			}

			for (int i = 0; i < 4 * round; i++)
			{
				HSV::Point2d tmpPt;
				double interval = 6.28 / (4 * round);
				tmpPt.x = centerPt.x + 4000 * sin(interval * i);
				tmpPt.y = centerPt.y + 4000 * cos(interval * i);
				double lenth = (tmpPt.x - centerPt.x) * (tmpPt.x - centerPt.x) + (tmpPt.y - centerPt.y) * (tmpPt.y - centerPt.y);
				if (sqrt(lenth) <= radius)
					temMovePt1D.push_back(tmpPt);
			}

			for (int i = 0; i < needSize; i++)
			{
				double iradi = (i + 1) * radius / round;
				for (unsigned int j = 0; j < allPtData2D[i].size(); j++)
				{
					HSV::Point2d tmpPt;
					double interval = 6.28 / allPtData2D[i].size();
					tmpPt.x = centerPt.x + iradi * sin(interval * j);
					tmpPt.y = centerPt.y + iradi * cos(interval * j);
					double lenth = (tmpPt.x - centerPt.x) * (tmpPt.x - centerPt.x) + (tmpPt.y - centerPt.y) * (tmpPt.y - centerPt.y);
					if (sqrt(lenth) <= radius)
						temMovePt1D.push_back(tmpPt);
				}
			}
		}
		else
		{
			/*int round = stepPricise * 10*/;
			int round = (int)(stepPricise * 2);
			if (round < 1)
				return temMovePt1D;
			/*if (round < 1)
			round = 2;*/
			IAPoint2D allPtData2D;
			allPtData2D.resize(round);
			for (int i = 0; i < round; i++)
			{
				//allPtData2D[i].resize(4*(i + 2));
				allPtData2D[i].resize(4 * (i + 1));
			}
			for (unsigned int i = 0; i < allPtData2D.size(); i++)
			{
				double iradi = (i + 1) * radius / round;
				for (unsigned int j = 0; j < allPtData2D[i].size(); j++)
				{
					HSV::Point2d tmpPt;
					double interval = 6.28 / allPtData2D[i].size();
					tmpPt.x = centerPt.x + iradi * sin(interval * j);
					tmpPt.y = centerPt.y + iradi * cos(interval * j);
					double lenth = (tmpPt.x - centerPt.x) * (tmpPt.x - centerPt.x) + (tmpPt.y - centerPt.y) * (tmpPt.y - centerPt.y);
					if (sqrt(lenth) <= radius)
						temMovePt1D.push_back(tmpPt);
				}
			}
		}
		return temMovePt1D;
	}

	//求圆和直线的交点：p---圆心  r---半径  interPt1D ---存放直线与圆的交点 @chenW 2020.03.03 16:32 
	int G_CircleLineCrossPt(HSV::Point2d p, double r, const HSV::Point2d& startPt, const HSV::Point2d& endPt, std::vector<HSV::Point2d>& interPt1D)
	{
		short nRes = -1;
		interPt1D.clear();
		if (r <= 0 || (startPt.x == endPt.x && startPt.y == endPt.y))
			return nRes;
		double a = 0, b = 0, c = 0;//线段所在直线方程的一般式：ax + by + c = 0
		if (startPt.x == endPt.x)
			a = 1, b = 0, c = -startPt.x;//特殊情况判断，分母不能为零
		else if (startPt.y == endPt.y)
			a = 0, b = 1, c = -startPt.y;
		else
		{
			a = startPt.y - endPt.y;
			b = endPt.x - startPt.x;
			c = startPt.x * endPt.y - startPt.y * endPt.x;
		}
		//圆的方程：(x - m)^2 = (y - n)^2 = r^2; (m,n)为圆点坐标
		// (a*a + b*b) *y^2 + 2(b*c - a*b*m -a*a*n) = a*a(r*r - m*m - n*n) + 2a*c*m - c*c;
		double A = a * a + b * b;
		double B = 2 * (b * c + a * b * p.x - a * a * p.y);
		double C = 2 * a*c*p.x + c*c - a*a * (r*r - p.x*p.x - p.y*p.y);
		double del = B * B - 4 * A * C;//二元一次方程标准式：A*x^2 + B*x + C = 0;
		HSV::Point2d interPt;
		if (del < -0.001)//原方程无实根
		{
			nRes = 0;//当前圆与线段所在直线不可能相交
			if (0)
			{
				//求圆心到线段的垂直距离
				double s = a * p.x + b * p.y + c;
				double distance = sqrt((s * s) / (a * a + b * b));
				if (distance > r)//圆心到线段所在直线的垂直距离大于半径r
					nRes = 0;//当前圆与线段所在直线不可能相交
			}
		}
		else if (del < 0.001)//原方程只有一个根
		{
			interPt.y = -B / (2 * A);
			interPt.x = -(c + b * interPt.y) / a;
			interPt1D.push_back(interPt);
			nRes = 1;
		}
		else//原方程有两个不相等的根
		{
			interPt1D.resize(2);
			interPt.y = (-B + sqrt(del)) / (2 * A);
			interPt.x = -(c + b * interPt.y) / a;
			//判断两个焦点中哪一个在startPt那一边
			HSV::Point2d vectPtA;//向量A：从起始点startPt指向交点interPt的向量
			vectPtA.x = startPt.x - interPt.x;
			vectPtA.y = startPt.y - interPt.y;
			HSV::Point2d vectPtB;//向量B：从终止点endPt指向起始点startPt的向量
			vectPtB.x = endPt.x - startPt.x;
			vectPtB.y = endPt.y - startPt.y;
			double val = vectPtA.x * vectPtB.x + vectPtA.y * vectPtB.y;//向量相乘大于0时表示同向，等于0垂直，小于0反向
			if (val > 0.001)//将与起始点同侧的交点放在第1个位置，将终止点（箭头方向）同侧的交点放在第2个位置（由于拖屏时移动的方向与箭头的方向相反）
			{
				interPt1D.at(0) = interPt;
				interPt1D.at(1).y = (-B - sqrt(del)) / (2 * A);
				interPt1D.at(1).x = -(c + b * interPt1D.at(1).y) / a;
			}
			else
			{
				interPt1D.at(1) = interPt;
				interPt1D.at(0).y = (-B - sqrt(del)) / (2 * A);
				interPt1D.at(0).x = -(c + b * interPt1D.at(0).y) / a;
			}
			nRes = 2;
		}
		return nRes;
	}
}