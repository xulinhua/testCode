#ifndef HS_GEO_H
#define HS_GEO_H

#ifdef _WIN32
    #ifdef HSV_CUSTRCT_EXPORTS
        #define HSV_GEO_PROC_API __declspec(dllexport)
    #else
        #define HSV_GEO_PROC_API __declspec(dllimport)
    #endif
#elif __linux__
    #ifdef HSV_CUSTRCT_EXPORTS
        #define HSV_GEO_PROC_API __attribute__((visibility("default")))
    #else
        #define HSV_GEO_PROC_API
    #endif
#else
    #error "Unsupported platform"
#endif
//#include <cmath> 
#include "HSPoint.hpp"
#include "IAPtStruct.h"
#include "platform_defines.h"

#if USING_OLD_HS_REG //IA2.0默认不启用旧的HSReg数据结构 @ChenW 08/01/2024, 11:55
Pt2DFloat HSV_GEO_PROC_API HSG_GetFootPt(const Pt2DFloat& linePt1, const Pt2DFloat& linePt2, const Pt2DFloat& pt);//计算某一点到线段的垂足点坐标

Pt2DFloat HSV_GEO_PROC_API HSG_GetFootPt(const Pt2DFloat& linePt1, const Pt2DFloat& linePt2, float ptX, float ptY);//计算某一点到线段的垂足点坐标								   

Pt2DFloat HSV_GEO_PROC_API HSG_GetFootPt(float linePt1X, float linePt1Y, float linePt2X, float linePt2Y, float ptX, float ptY);//计算某一点到线段的垂足点坐标

Pt2DFloat HSV_GEO_PROC_API HSG_GetFootPt(const Pt2DInt& linePt1, const Pt2DInt& linePt2, const Pt2DInt& pt);//计算某一点到线段的垂足点坐标																				 

Pt2DFloat HSV_GEO_PROC_API HSG_GetFootPt(const Pt2DInt& linePt1, const Pt2DInt& linePt2, int ptX, int ptY);//计算某一点到线段的垂足点坐标									 

Pt2DFloat HSV_GEO_PROC_API HSG_GetFootPt(int linePt1X, int linePt1Y, int linePt2X, int linePt2Y, int ptX, int ptY);//计算某一点到线段的垂足点坐标
																																						
void HSV_GEO_PROC_API HSG_SortPtByYMax2Min(std::vector<Pt2DFloat>& pt1D);//将点集按Y值从大到小的顺序进行排序（如果最大的Y值不止一个，则选择X值最小的那个）

float HSV_GEO_PROC_API HSG_Cal2PtDistance(const Pt2DFloat& pt1, const Pt2DFloat& pt2);//计算两点的距离																		
																				 
float HSV_GEO_PROC_API HSG_Cal2PtDistance(const Pt2DInt& pt1, const Pt2DInt& pt2);//计算两点的距离

float HSV_GEO_PROC_API HSG_CalPt2Line_CrossProduct(const Pt2DFloat& linePt1, const Pt2DFloat& linePt2, const Pt2DFloat& pt);//计算某点到线段的叉积
					   
float HSV_GEO_PROC_API HSG_CalPt2Line_CrossProduct(float linePt1X, float linePt1Y, float linePt2X, float linePt2Y, float ptX, float ptY);//计算某点到线段的叉积(向量积)

float HSV_GEO_PROC_API HSG_CalPt2LineDistance(const Pt2DFloat& linePt1, const Pt2DFloat& linePt2, const Pt2DFloat& pt);//计算某点到线段的垂直距离																										 																										 
																												  
void HSV_GEO_PROC_API HSG_GetPtByFootPtAndDist(const Pt2DFloat& linePt1, const Pt2DFloat& linePt2, const Pt2DFloat& footPt, float distance, bool bAnticlock, Pt2DFloat& pt);//根据垂足点坐标及点到线段的距离求点的坐标

float HSV_GEO_PROC_API HSG_GetMaxDistFromPtSet(const std::vector<Pt2DFloat>& pt1D, Pt2DFloat& pt1, Pt2DFloat& pt2);//从一组点集中获取距离最大的两个点的坐标并返回距离值

bool HSV_GEO_PROC_API HSG_SortKslopeMinToMax(const Pt2DFloat& pt, const Pt2DFloat& pt1, const Pt2DFloat& pt2);//对输入两点相对于基准点的斜率由小到大排序
																											 
std::vector<Pt2DFloat> HSV_GEO_PROC_API HSG_GetConvexHullPt_Graham1(const std::vector<Pt2DFloat>& pt1D);//Graham扫描法获取一个点集中的所有凸包点（同一线段上多余的点删除掉）

std::vector<Pt2DFloat> HSV_GEO_PROC_API HSG_GetConvexHullPt_Graham(const std::vector<Pt2DFloat>& pt1D);//Graham扫描法获取一个点集中的所有凸包点（同一线段上多余的点删除掉）

float HSV_GEO_PROC_API HSG_GetMaxDistPt2HullEdge(std::vector<Pt2DFloat> convexPt1D, const unsigned int& startPtIdx, unsigned int& maxDistPtIdx);//获取离凸包上的某一边最远的的凸包点坐标索引

float HSV_GEO_PROC_API HSG_GetMaxDistPt2HullEdge(std::vector<Pt2DFloat> convexPt1D, const unsigned int& startPtIdx, Pt2DFloat& maxDistPt);//获取离凸包上的某一边最远的的凸包点坐标

void HSV_GEO_PROC_API HSG_GetMinOutRect1ByConvexHull(const std::vector<Pt2DFloat>& pt1D, Pt2DInt& ltTopPt, Pt2DInt& rtBtnPt);//旋转卡壳法计算凸包的最小外接矩形（不带角度）

void HSV_GEO_PROC_API HSG_GetMinOutRect2ByConvexHull(const std::vector<Pt2DFloat>& pt1D, Pt2DFloat& cnter, float& halfW, float& halfH, float& degAng);//旋转卡壳法计算凸包的最小带角度外接矩形

float HSV_GEO_PROC_API HSG_Cal2VecDegAng(const Pt2DInt& sourcePt, int startPtX, int startPtY, int endPtX, int endPtY);//计算两个矢量的夹角(带方向) 

float HSV_GEO_PROC_API HSG_Cal2VecDegAng(int sourceX, int sourceY, int startPtX, int startPtY, int endPtX, int endPtY);//计算两个矢量的夹角(带方向) 																							   

float HSV_GEO_PROC_API HSG_Cal2VecDegAng(const Pt2DFloat& sourcePt, int startPtX, int startPtY, int endPtX, int endPtY);//计算两个矢量的夹角(带方向) 

float HSV_GEO_PROC_API HSG_Cal2VecDegAng(const Pt2DFloat& sourcePt, float startPtX, float startPtY, float endPtX, float endPtY);//计算两个矢量的夹角(带方向) 

float HSV_GEO_PROC_API HSG_Cal2VecDegAng(float sourceX, float sourceY, int startPtX, int startPtY, int endPtX, int endPtY);//计算两个矢量的夹角(带方向) 	

float HSV_GEO_PROC_API HSG_Cal2VecDegAng(float sourceX, float sourceY, float startPtX, float startPtY, float endPtX, float endPtY);//计算两个矢量的夹角(带方向) 	

bool HSV_GEO_PROC_API HSG_IsPtInPolyGon(const std::vector<Pt2DFloat> &pts1D, Pt2DFloat temsorcePt);	//temsorcePt是否在pts1D内

bool HSV_GEO_PROC_API HSG_Online(const Pt2DFloat &startPt,const Pt2DFloat &endPt,const Pt2DFloat &resPt);	//三点是否共线

bool HSV_GEO_PROC_API HSG_Intersect(const Pt2DFloat &startPt1, const Pt2DFloat &endPt1, const Pt2DFloat &startPt2, const Pt2DFloat &endPt2);	//线条pt1和pt2是否相交
#endif

namespace HSGEO
{

	/* 常用的常量定义 */
	const double	GE0_VALUE_MAX = 1E200; //最大值
	const double	GE0_VALUE_MIN = 1E-10; //最小值
	const int		GE0_VALUE_POLYPT_MAX = 300; //多边形最大的顶点数
	const double	GE0_VALUE_PI = 3.14159265;

	//线段
	struct LINESEG
	{
		HSV::Point2d s;
		HSV::Point2d e;
		LINESEG(HSV::Point2d a, HSV::Point2d b) { s = a; e = b; }
		LINESEG() { }
	};

	// 直线的解析方程 a*x+b*y+c=0  为统一表示，约定 a >= 0 
	struct LINE
	{
		double a;
		double b;
		double c;
		LINE(double d1 = 1, double d2 = -1, double d3 = 0) { a = d1; b = d2; c = d3; }
	};


	/*--------------------------------------------------------
	//
	//					  坐标变换
	//
	--------------------------------------------------------*/
	/****************************************************************************
	功能：二维坐标系的平移变换
	参数：HSV::Point2d o,变换后的坐标系原点在标准坐标系的位置
	HSV::Point2d p,变换后坐标系的点在标准坐标系的位置
	返回值：
	****************************************************************************/
	void HSV_GEO_PROC_API G_Trans_Offset(HSV::Point2d o, HSV::Point2d& p);

	/****************************************************************************
	功能：二维坐标系的平移反变换
	参数：HSV::Point2d o,变换后的坐标系原点在标准坐标系的位置
	HSV::Point2d p,标准坐标系的点在变换后的坐标系的位置
	返回值：
	****************************************************************************/
	void HSV_GEO_PROC_API G_Trans_Offset_Opp(HSV::Point2d o, HSV::Point2d& p);

	/****************************************************************************
	功能：二维，缩放变换
	注意：原点重合
	参数：double scaleX,double scaleY,目标坐标系相对标准坐标系的尺度比
	HSV::Point2d p,点P在标准的位置
	返回值：
	****************************************************************************/
	void HSV_GEO_PROC_API G_Trans_Scale(double scaleX, double scaleY, HSV::Point2d& p);
	void HSV_GEO_PROC_API G_Trans_Scale_Opp(double scaleX, double scaleY, HSV::Point2d& p);
	/****************************************************************************
	功能：二维，直角转斜角坐标系
	注意：原点重合
	参数：double angle,斜角坐标系的角度，弧度
	HSV::Point2d p,标准坐标系的点在变换后的坐标系的位置
	返回值：
	****************************************************************************/
	void HSV_GEO_PROC_API G_Trans_VtoK(double angle, HSV::Point2d& p);

	/****************************************************************************
	功能：二维，斜角转直角
	注意：原点重合
	参数：double angle,斜角坐标系的角度，弧度
	HSV::Point2d p,斜角点P在直角的位置
	返回值：
	****************************************************************************/
	void HSV_GEO_PROC_API G_Trans_KtoV(double angle, HSV::Point2d& p);

	/****************************************************************************
	功能：二维，两个直角坐标系的旋转,标准的坐标转成目标坐标系的
	注意：原点重合
	参数：double angle,目标坐标系相对标准坐标系的夹角。X轴，逆时针
	HSV::Point2d p,点P在标准的位置
	返回值：
	****************************************************************************/
	void HSV_GEO_PROC_API G_Trans_Rotate(double angle, HSV::Point2d& p);

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
	void HSV_GEO_PROC_API G_CoodrateTrans(double a, double b, double q, double k1, double k2,
		double s, double t, double& x, double& y);

	///////////////////////////////////////////  
	//求三角形外接圆,为计算计算找晶圆形区域准备
	///////////////////////////////////////////  
	bool HSV_GEO_PROC_API G_GetCirclePt(HSV::Point2d *center, HSV::Point2d* pt, double *radiu);

	/***************************************************************************************
	*函 数 名：G_TransXYT(double xCalibCenter,double yCalibCenter,double rCirclePluse,
	double xCurPos,double yCurPos,double rCurPos,double angle,
	double &xTagPos,double& yTagPos,double& rTagPos)

	*输入参数：double xCalibCenter,中心点x坐标
	double yCalibCenter,中心点y坐标
	double rCirclePluse,T轴旋转一圈的脉冲
	double xCurPos,当前位置x坐标
	double yCurPos,当前位置y坐标
	double rCurPos，当前T轴脉冲
	double angle,当前要旋转的角度
	double yTagPos,旋转后y的坐标
	double xTagPos，旋转后x坐标
	double rTagPos,旋转后r的坐标

	*函数功能：一个点围绕另一个点旋转一定角度得到的新坐标

	*返 回 值：无
	***************************************************************************************/
	void HSV_GEO_PROC_API G_TransXYT(double xCalibCenter, double yCalibCenter, double rCirclePluse,
		double xCurPos, double yCurPos, double rCurPos, double angle,
		double &xTagPos, double& yTagPos, double& rTagPos);
	/*--------------------------------------------------------
	//
	//					  点的基本运算
	//
	--------------------------------------------------------*/

	//  
	/****************************************************************************
	功能：返回两点之间欧氏距离
	参数：HSV::Point2d p1,点1
	HSV::Point2d p2,点2
	返回值： 欧式距离
	****************************************************************************/
	double HSV_GEO_PROC_API G_Dist(HSV::Point2d p1, HSV::Point2d p2);

	/****************************************************************************
	功能：判断两个点是否重合
	参数：HSV::Point2d p1,点1
	HSV::Point2d p2,点2
	返回值： 重合rtrue, 不重合false
	****************************************************************************/
	bool HSV_GEO_PROC_API G_IsPtEqual(HSV::Point2d p1, HSV::Point2d p2);

	/******************************************************************************
	功能：得到(sp-op)和(ep-op)的叉积
	r=multiply(sp,ep,op),
	r>0：ep在矢量opsp的逆时针方向；
	r=0：opspep三点共线；
	r<0：ep在矢量opsp的顺时针方向
	参数：HSV::Point2d sp,起始点
	HSV::Point2d ep,结束点
	HSV::Point2d op,基础点
	返回值： 叉集
	*******************************************************************************/
	double HSV_GEO_PROC_API G_Multiply(HSV::Point2d sp, HSV::Point2d ep, HSV::Point2d op);

	/******************************************************************************
	功能：如果两个矢量都非零矢量 得到(sp-op)和(ep-op)的点积
	r=dotmultiply(p1,p2,op),得到矢量(p1-op)和(p2-op)的
	r<0：两矢量夹角为锐角
	r=0：两矢量夹角为直角
	r>0：两矢量夹角为钝角
	参数：HSV::Point2d sp,起始点
	HSV::Point2d ep,结束点
	HSV::Point2d op,基础点
	返回值： 的点积
	*******************************************************************************/
	double HSV_GEO_PROC_API G_DotMultiply(HSV::Point2d p1, HSV::Point2d p2, HSV::Point2d p0);

	/******************************************************************************
	功能：判断点p是否在线段l上
	条件,(p在线段l所在的直线上) && (点p在以线段l为对角线的矩形内)
	参数：LINESEG l,线段
	HSV::Point2d p,判断点

	返回值： true 在线段上，false 不在线段上
	*******************************************************************************/
	bool HSV_GEO_PROC_API G_Online(LINESEG l, HSV::Point2d p);

	/******************************************************************************
	功能：返回点p以点o为圆心逆时针旋转alpha(单位：弧度)后所在的位置
	条件,(p在线段l所在的直线上) && (点p在以线段l为对角线的矩形内)
	参数：HSV::Point2d o,基点
	double alpha,转角
	HSV::Point2d p,被计算点

	返回值： 旋转后的位置
	*******************************************************************************/
	HSV::Point2d HSV_GEO_PROC_API G_Rotate(HSV::Point2d o, double alpha, HSV::Point2d p);

	/* 返回顶角在o点，起始边为os，终止边为oe的夹角(单位：弧度)
	角度小于pi，返回正值
	角度大于pi，返回负值
	可以用于求线段之间的夹角

	/******************************************************************************
	功能：返回顶角在o点，起始边为os，终止边为oe的夹角(单位：弧度)

	原理：
	r = dotmultiply(s,e,o) / (dist(o,s)*dist(o,e))
	r'= multiply(s,e,o)

	r >= 1	angle = 0;
	r <= -1	angle = -GE0_VALUE_PI

	参数：HSV::Point2d o,顶点
	HSV::Point2d s,起点
	HSV::Point2d e,终点

	返回值： 返回顶角在o点，起始边为os，终止边为oe的夹角
	*******************************************************************************/
	double HSV_GEO_PROC_API G_Angle(HSV::Point2d o, HSV::Point2d s, HSV::Point2d e);
	double HSV_GEO_PROC_API G_Angle(HSV::Point o, HSV::Point s, HSV::Point e);

	/*--------------------------------------------------------
	//
	//					  线段及直线的基本运算
	//
	--------------------------------------------------------*/


	/******************************************************************************
	功能：判断点与线段的关系,用途很广泛
	原理：本函数是根据下面的公式写的，P是点C到线段AB所在直线的垂足

	AC dot AB
	r =     ---------
	||AB||^2
	(Cx-Ax)(Bx-Ax) + (Cy-Ay)(By-Ay)
	= -------------------------------
	L^2
	r 和含义如下

	r=0      P = A
	r=1      P = B
	r<0		 P is on the backward extension of AB
	r>1      P is on the forward extension of AB
	0<r<1	 P is interior to AB

	参数：HSV::Point2d p,点
	LINESEG l,线段

	返回值： R的值
	*******************************************************************************/
	double HSV_GEO_PROC_API G_Relation(HSV::Point2d p, LINESEG l);

	/******************************************************************************
	功能： 求点C到线段AB所在直线的垂足 P
	参数：HSV::Point2d p,点
	LINESEG l,线段
	返回值： 垂足点
	*******************************************************************************/
	HSV::Point2d HSV_GEO_PROC_API G_Perpendicular(HSV::Point2d p, LINESEG l);

	/******************************************************************************
	功能： 求点p到线段l的最短距离,并返回线段上距该点最近的点np
	注意：np是线段l上到点p最近的点，不一定是垂足
	参数：HSV::Point2d p,点
	LINESEG l,线段
	HSV::Point2d &np,最近的点
	返回值： 最短距离
	*******************************************************************************/
	double HSV_GEO_PROC_API G_Pt2LinesegDist(HSV::Point2d p, LINESEG l, HSV::Point2d &np);

	// 求点p到线段l所在直线的距离,请注意本函数与上个函数的区别  
	double HSV_GEO_PROC_API G_Pt2LineDist(HSV::Point2d p, LINESEG l);

	/* 计算点到折线集的最近距离,并返回最近点.
	注意：调用的是ptolineseg()函数 */
	double HSV_GEO_PROC_API G_Pt2GPtset(int vcount, HSV::Point2d G_POINTset[], HSV::Point2d p, HSV::Point2d &q);

	/******************************************************************************
	功能： 判断圆是否在多边形内.ptolineseg()函数的应用
	参数：int vcount,多边形顶点数
	HSV::Point2d center,圆的中心点
	double radius，圆的半径
	HSV::Point2d polygon[]，多边形的点
	返回值： 最短距离
	*******************************************************************************/
	bool HSV_GEO_PROC_API G_IsCircleInsidePolygon(int vcount, HSV::Point2d center, double radius, HSV::Point2d polygon[]);

	/* 返回两个矢量l1和l2的夹角的余弦(-1 --- 1)
	注意：如果想从余弦求夹角的话，注意反余弦函数的定义域是从 0到pi */
	double HSV_GEO_PROC_API G_Cosine(LINESEG l1, LINESEG l2);

	// 返回线段l1与l2之间的夹角 单位：弧度 范围(-pi，pi) 
	double HSV_GEO_PROC_API G_LinesegAngle(LINESEG l1, LINESEG l2);

	// 如果线段u和v相交(包括相交在端点处)时，返回true 
	//
	//判断P1P2跨立Q1Q2的依据是：( P1 - Q1 ) × ( Q2 - Q1 ) * ( Q2 - Q1 ) × ( P2 - Q1 ) >= 0。
	//判断Q1Q2跨立P1P2的依据是：( Q1 - P1 ) × ( P2 - P1 ) * ( P2 - P1 ) × ( Q2 - P1 ) >= 0。
	bool HSV_GEO_PROC_API G_Intersect(LINESEG u, LINESEG v);

	//  (线段u和v相交)&&(交点不是双方的端点) 时返回true    
	bool HSV_GEO_PROC_API G_Intersect_A(LINESEG u, LINESEG v);

	// 线段v所在直线与线段u相交时返回true；方法：判断线段u是否跨立线段v  
	bool HSV_GEO_PROC_API G_Intersect_l(LINESEG u, LINESEG v);

	// 根据已知两点坐标，求过这两点的直线解析方程： a*x+b*y+c = 0  (a >= 0)  
	LINE HSV_GEO_PROC_API G_MakeLine(HSV::Point2d p1, HSV::Point2d p2);

	// 根据直线解析方程返回直线的斜率k,水平线返回 0,竖直线返回 1e200 
	double HSV_GEO_PROC_API G_Slope(LINE l);

	// 返回直线的倾斜角alpha ( 0 - pi) 
	double HSV_GEO_PROC_API G_Alpha(LINE l);

	// 求点p关于直线l的对称点  
	HSV::Point2d HSV_GEO_PROC_API G_Symmetry(LINE l, HSV::Point2d p);

	// 如果两条直线 l1(a1*x+b1*y+c1 = 0), l2(a2*x+b2*y+c2 = 0)相交，返回true，且返回交点p  
	bool HSV_GEO_PROC_API G_IsLineIntersect(LINE l1, LINE l2, HSV::Point2d &p); // 是 L1，L2 

																		   // 如果线段l1和l2相交，返回true且交点由(inter)返回，否则返回false 
	bool HSV_GEO_PROC_API G_IsIntersection(LINESEG l1, LINESEG l2, HSV::Point2d &inter);


	/*--------------------------------------------------------
	//
	//					  多边形常用算法模块
	//
	--------------------------------------------------------*/


	// 如果无特别说明，输入多边形顶点要求按逆时针排列 

	/**********************************************
	将指定的点集进行逆时针排序
	G_POINTSet为输入的点集；
	n为G_POINTSet中的点的数目
	//要求，点是互异点
	**********************************************/
	void HSV_GEO_PROC_API G_QueneUnGetClock(HSV::Point2d G_POINTSet[], int n);
	/*
	返回值：输入的多边形是简单多边形，返回true
	要 求：输入顶点序列按逆时针排序
	说 明：简单多边形定义：
	1：循环排序中相邻线段对的交是他们之间共有的单个点
	2：不相邻的线段不相交
	本程序默认第一个条件已经满足
	*/
	bool HSV_GEO_PROC_API G_IsSimple(int vcount, HSV::Point2d polygon[]);

	// 返回值：按输入顺序返回多边形顶点的凸凹性判断，bc[i]=1,iff:第i个顶点是凸顶点 
	void HSV_GEO_PROC_API G_CheckConvex(int vcount, HSV::Point2d polygon[], bool bc[]);

	// 返回值：多边形polygon是凸多边形时，返回true  
	bool HSV_GEO_PROC_API G_IsConvex(int vcount, HSV::Point2d polygon[]);

	// 返回多边形面积(signed)；输入顶点按逆时针排列时，返回正值；否则返回负值 
	double HSV_GEO_PROC_API G_AreaOfPolygon(int vcount, HSV::Point2d polygon[]);

	// 如果输入顶点按逆时针排列，返回true 
	bool HSV_GEO_PROC_API G_IsConterGetClock(int vcount, HSV::Point2d polygon[]);

	// 另一种判断多边形顶点排列方向的方法  
	bool HSV_GEO_PROC_API G_IsCcwize(int vcount, HSV::Point2d polygon[]);

	/********************************************************************************************
	射线法判断点q与多边形polygon的位置关系，要求polygon为简单多边形，顶点逆时针排列
	如果点在多边形内：   返回0
	如果点在多边形边上： 返回1
	如果点在多边形外：	返回2
	*********************************************************************************************/
	int HSV_GEO_PROC_API G_InsidePolygon(int vcount, HSV::Point2d Polygon[], HSV::Point2d q);

	//点q是凸多边形polygon内时，返回true；注意：多边形polygon一定要是凸多边形  
	bool HSV_GEO_PROC_API G_InsideConvexPolygon(int vcount, HSV::Point2d polygon[], HSV::Point2d q); // 可用于三角形！ 

																						   /**********************************************
																						   寻找凸包的graham 扫描法
																						   G_POINTSet为输入的点集；
																						   ch为输出的凸包上的点集，按照逆时针方向排列;
																						   n为G_POINTSet中的点的数目
																						   len为输出的凸包上的点的个数
																						   **********************************************/
	void HSV_GEO_PROC_API G_Graham_scan(HSV::Point2d G_POINTSet[], HSV::Point2d ch[], int n, int &len);

	// 卷包裹法求点集凸壳，参数说明同graham算法    
	void HSV_GEO_PROC_API G_ConvexClosure(HSV::Point2d G_POINTSet[], HSV::Point2d ch[], int n, int &len);

	/*********************************************************************************************
	判断线段是否在简单多边形内(注意：如果多边形是凸多边形，下面的算法可以化简)
	必要条件一：线段的两个端点都在多边形内；
	必要条件二：线段和多边形的所有边都不内交；
	用途：	1. 判断折线是否在简单多边形内
	2. 判断简单多边形是否在另一个简单多边形内
	**********************************************************************************************/
	bool HSV_GEO_PROC_API G_LinesegInsidePolygon(int vcount, HSV::Point2d polygon[], LINESEG l);

	/*********************************************************************************************
	求任意简单多边形polygon的重心
	需要调用下面几个函数：
	void AddPosPart(); 增加右边区域的面积
	void AddNegPart(); 增加左边区域的面积
	void AddRegion(); 增加区域面积
	在使用该程序时，如果把xtr,ytr,wtr,xtl,ytl,wtl设成全局变量就可以使这些函数的形式得到化简,
	但要注意函数的声明和调用要做相应变化
	**********************************************************************************************/
	void HSV_GEO_PROC_API G_AddPosPart(double x, double y, double w, double &xtr, double &ytr, double &wtr);

	void HSV_GEO_PROC_API G_AddNegPart(double x, double y, double w, double &xtl, double &ytl, double &wtl);

	void HSV_GEO_PROC_API G_AddRegion(double x1, double y1, double x2, double y2, double &xtr, double &ytr,
		double &wtr, double &xtl, double &ytl, double &wtl);

	HSV::Point2d HSV_GEO_PROC_API G_CGSimple(int vcount, HSV::Point2d polygon[]);

	// 求凸多边形的重心,要求输入多边形按逆时针排序 
	HSV::Point2d HSV_GEO_PROC_API G_GravityCenter(int vcount, HSV::Point2d polygon[]);

	/************************************************
	给定一简单多边形，找出一个肯定在该多边形内的点
	定理1	：每个多边形至少有一个凸顶点
	定理2	：顶点数>=4的简单多边形至少有一条对角线
	结论	： x坐标最大，最小的点肯定是凸顶点
	y坐标最大，最小的点肯定是凸顶点
	************************************************/
	HSV::Point2d HSV_GEO_PROC_API G_APtInsidePoly(int vcount, HSV::Point2d polygon[]);

	/***********************************************************************************************
	求从多边形外一点p出发到一个简单多边形的切线,如果存在返回切点,其中rp点是右切点,lp是左切点
	注意：p点一定要在多边形外 ,输入顶点序列是逆时针排列
	原 理：	如果点在多边形内肯定无切线;凸多边形有唯一的两个切点,凹多边形就可能有多于两个的切点;
	如果polygon是凸多边形，切点只有两个只要找到就可以,可以化简此算法
	如果是凹多边形还有一种算法可以求解:先求凹多边形的凸包,然后求凸包的切线
	/***********************************************************************************************/
	void HSV_GEO_PROC_API G_PtTangentPoly(int vcount, HSV::Point2d polygon[], HSV::Point2d p, HSV::Point2d &rp, HSV::Point2d &lp);

	// 如果多边形polygon的核存在，返回true，返回核上的一点p.顶点按逆时针方向输入  
	bool HSV_GEO_PROC_API G_IsCoreExist(int vcount, HSV::Point2d polygon[], HSV::Point2d &p);


	/*--------------------------------------------------------
	//
	//					  圆的基本运算
	//
	--------------------------------------------------------*/

	/******************************************************************************
	返回值	： 点p在圆内(包括边界)时，返回true
	用途	： 因为圆为凸集，所以判断点集，折线，多边形是否在圆内时，
	只需要逐一判断点是否在圆内即可。
	*******************************************************************************/
	bool HSV_GEO_PROC_API G_IsPtInCircle(HSV::Point2d o, double r, HSV::Point2d p);
	/******************************************************************************
	返回值	：
	用途	：点是否在多边形内
	*******************************************************************************/
	bool HSV_GEO_PROC_API G_IsPtInPolyGon(const std::vector<HSV::Point2d> &pts1D, HSV::Point2d temsorcePt);

	/******************************************************************************
	用 途	：求不共线的三点确定一个圆
	输 入	：三个点p1,p2,p3
	返回值	：如果三点共线，返回false；反之，返回true。圆心由q返回，半径由r返回
	*******************************************************************************/
	bool HSV_GEO_PROC_API G_Pt2Circle(HSV::Point2d p1, HSV::Point2d p2, HSV::Point2d p3, HSV::Point2d &q, double &r);

	int HSV_GEO_PROC_API G_LineCircle(LINE l, HSV::Point2d o, double r, HSV::Point2d &p1, HSV::Point2d &p2);

	/**
	* 最小二乘法拟合圆
	* 拟合出的圆以圆心坐标和半径的形式表示
	*/
	bool HSV_GEO_PROC_API G_CircleLeastFit(const std::vector<HSV::Point2d> &pts1D, HSV::Point2d& cnter, double &radius);


	/*--------------------------------------------------------
	//
	//					  矩形的基本运算
	//
	--------------------------------------------------------*/
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
	HSV::Point2d HSV_GEO_PROC_API G_Rect4th(HSV::Point2d a, HSV::Point2d b, HSV::Point2d c);


	/*************************\
	*						*
	* 常用算法的描述		*
	*						*
	\*************************/
	/*
	尚未实现的算法：
	1. 求包含点集的最小圆
	2. 求多边形的交
	3. 简单多边形的三角剖分
	4. 寻找包含点集的最小矩形
	5. 折线的化简
	6. 判断矩形是否在矩形中
	7. 判断矩形能否放在矩形中
	8. 矩形并的面积与周长
	9. 矩形并的轮廓
	10.矩形并的闭包
	11.矩形的交
	12.点集中的最近点对
	13.多边形的并
	14.圆的交与并
	15.直线与圆的关系
	16.线段与圆的关系
	17.求多边形的核监视摄象机
	18.求点集中不相交点对 railwai
	*//*
	寻找包含点集的最小矩形
	原理：该矩形至少一条边与点集的凸壳的某条边共线

	*/
	/*--------------------------------------------------------
	//
	//					  补充
	//
	--------------------------------------------------------*/

	//两圆关系： 
	/* 两圆：
	相离： return 1；
	外切： return 2；
	相交： return 3；
	内切： return 4；
	内含： return 5；
	*/
	int HSV_GEO_PROC_API G_CircleRelation(HSV::Point2d p1, double r1, HSV::Point2d p2, double r2);

	//判断圆是否在矩形内：
	// 判定圆是否在矩形内，是就返回true（设矩形水平，且其四个顶点由左上开始按顺时针排列） 
	// 调用ptoldist函数，在第4页 
	bool HSV_GEO_PROC_API G_CircleRecRelation(HSV::Point2d pc, double r, HSV::Point2d pr1, HSV::Point2d pr2, HSV::Point2d pr3, HSV::Point2d pr4);

	//点到平面的距离： 
	//点到平面的距离,平面用一般式表示ax+by+cz+d=0 
	double HSV_GEO_PROC_API G_P2planeDist(double x, double y, double z, double a, double b, double c, double d);

	//点是否在直线同侧：
	//两个点是否在直线同侧，是则返回true 
	bool HSV_GEO_PROC_API G_SameSide(HSV::Point2d p1, HSV::Point2d p2, LINE line);

	//镜面反射线：
	// 已知入射线、镜面，求反射线。 
	// a1,b1,c1为镜面直线方程(a1 x + b1 y + c1 = 0 ,下同)系数;  
	//a2,b2,c2为入射光直线方程系数;  
	//a,b,c为反射光直线方程系数. 
	// 光是有方向的，使用时注意：入射光向量:<-b2,a2>；反射光向量:<b,-a>. 
	// 不要忘记结果中可能会有"negative zeros" 
	void HSV_GEO_PROC_API G_Reflect(double a1, double b1, double c1, double a2, double b2, double c2, double &a, double &b, double &c);

	//矩形包含： 
	// 矩形2（C，D）是否在1（A，B）内
	bool HSV_GEO_PROC_API G_R2inr1(double A, double B, double C, double D);

	//两圆交点： 
	// 两圆已经相交（相切） 
	void  HSV_GEO_PROC_API G_Circle2Pt(HSV::Point2d p1, double r1, HSV::Point2d p2, double r2, HSV::Point2d &rp1, HSV::Point2d &rp2);

	//两圆公共面积：
	// 必须保证相交 
	double HSV_GEO_PROC_API G_Circle2Area(HSV::Point2d p1, double r1, HSV::Point2d p2, double r2);

	//圆和直线关系： 
	//0----相离 1----相切 2----相交 
	int HSV_GEO_PROC_API G_CircleLine2Pt(HSV::Point2d p, double r, double a, double b, double c, HSV::Point2d &rp1, HSV::Point2d &rp2);

	//内切圆： 
	void HSV_GEO_PROC_API G_InCircle(HSV::Point2d p1, HSV::Point2d p2, HSV::Point2d p3, HSV::Point2d &rp, double &r);

	//求切点： 
	// p---圆心坐标， r---圆半径， sp---圆外一点， rp1,rp2---切点坐标 
	void HSV_GEO_PROC_API G_CircleCutPt(HSV::Point2d p, double r, HSV::Point2d sp, HSV::Point2d &rp1, HSV::Point2d &rp2);

	//线段的左右旋： 
	/* l2在l1的左/右方向（l1为基准线）;
	返回	0	： 重合；
	返回	1	： 右旋；
	返回	–1 ： 左旋；
	*/
	int HSV_GEO_PROC_API G_Rotat(LINESEG l1, LINESEG l2);

	//返回小数的四舍五入值
	double HSV_GEO_PROC_API G_Round(double val);

	//根据螺旋中心及角度步长计算螺旋线上的点
	//HSV::Point2d centerPt,螺旋中心位置
	//double radius, 搜索半径
	//double stepPricise， 计算的步长精度
	HSV::Point2d1D HSV_GEO_PROC_API G_GetHelixPt(HSV::Point2d centerPt, double radius, double stepPricise, unsigned int method = 0);

	//求圆和直线的交点：p---圆心  r---半径    
	int HSV_GEO_PROC_API G_CircleLineCrossPt(HSV::Point2d p, double r, const HSV::Point2d& startPt, const HSV::Point2d& endPt, std::vector<HSV::Point2d>& interPt1D);
}
#endif
