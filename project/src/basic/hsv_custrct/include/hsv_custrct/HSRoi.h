/********************************************************************************************************************************
* 功能说明：区域参数类
*  Ver      修改人          变更日期                 变更内容
* ─────────────────────────────────────────────────────────────────────────
*  V1.0.0  chenwang         2024-05-15       将点类从此头文件剥离后放在单独的hpp文件中，方便兼容IA2.0并解耦（避免流程被动包含多余的图像数据结构）
*备注：
-------------------------------------------------------------------------------------------------------------------------------*/
//
// @brief: 图形形状区域类
// @birth: created by LuoJianghong on 2023-07-03
// @version: V0.0.1
// @revision: last revised by LuoJianghong on 2023-07-03
//
#pragma once
#include"HSPoint.hpp"
#if USING_OLD_HS_REG //IA2.0默认不启用旧的HSReg数据结构 @ChenW 08/01/2024, 11:55
#include "HSReg.h"
#endif //IA2.0默认不启用旧的HSReg数据结构 @ChenW 08/01/2024, 11:55

#include "platform_defines.h"

#ifdef HSV_CUSTRCT_EXPORTS
#define GEOMETRY_API HSV_CUSTRCT_API
#else
#define GEOMETRY_API HSV_CUSTRCT_API
#endif

namespace HSV
{
	typedef enum _HSRegType//ROI区域类型
	{
		HSReg_RECT,
		HSReg_RECT2,
		HSReg_CIRCLE,
		HSReg_ELLIPSE,
		HSReg_POLYGON,
	}RegType;

	//ROI区域基类
	class GEOMETRY_API HSRoiBase
	{
	public:
		HSRoiBase();
		HSRoiBase(const HSRoiBase& para);
		virtual ~HSRoiBase();
		HSRoiBase& operator = (const HSRoiBase& para);
		virtual bool equal(const HSRoiBase &obj) const = 0;
		//virtual bool operator==(const HSRoiBase & obj) = 0;
		virtual void Init();
		void CopyFrom(const HSRoiBase& para);//从para拷贝数据									 
		void CopyTo(HSRoiBase& para) const;//拷贝数据到para		
		virtual RegType GetRegType() const = 0;//获取区域类型
		virtual void SetCnter(Point2f cnter) = 0;
		virtual void SetCnterOffset(int offsetX, int offsetY) = 0;//更新当前区域的中心坐标偏移量
		virtual void SetDegAng(float degAng) = 0;
		virtual void SetDegAngOffset(float offsetDegAng) = 0;
		virtual float GetDegAng() const = 0;
		virtual int Area() const = 0;//获取区域面积
		virtual bool IsEmpty() const = 0;//判断当前区域是否为空
		virtual Point2f Center() const = 0;//获取区域中心坐标
		virtual void SetScaleRate(float scaleRate) = 0;//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
		virtual void SetScaleRate(float scaleRateX, float scaleRateY) = 0;
		virtual void GetMaxSize(int& maxW, int& maxH) const = 0;
		virtual void BoundingRect(int Rect[]) const = 0;//获取区域外接矩形	
	};

	//ROI区域类_（不带角度）矩形
	class GEOMETRY_API Rect : public HSRoiBase
	{
	public:
		Point ltTopPt_;//矩形左上角坐标
		int w_, h_;//矩形长宽
	public:
		Rect();
		Rect(const Rect& para);
		Rect(int x, int y, int w, int h);
		Rect(Point pt, int w, int h);
		virtual ~Rect();
		Rect& operator = (const Rect& para);//赋值操作符重载，拷贝功能
		bool equal(const HSRoiBase &obj) const;
		bool operator == (const Rect & obj) const;
		bool operator != (const Rect & obj) const;
		Rect operator&(const Rect& Rect);//两个矩形&计算，返回交集矩形
		Rect operator|(const Rect& Rect);//两个矩形|计算，返回并集矩形							
		Rect operator+(Point point); //矩形中心点平移
		Rect operator-(Point point);
		virtual void Init();
		virtual void CopyFrom(const Rect& para);//从para拷贝数据									 
		virtual void CopyTo(Rect& para) const;//拷贝数据到para
		RegType GetRegType() const;//获取ROI区域类型
		void SetCnter(float cnterX, float cnterY);
		void SetCnter(Point2f cnter);
		void SetCnterOffset(int offsetX, int offsetY);//更新当前区域的中心坐标偏移量
		void SetDegAng(float degAng);
		void SetDegAngOffset(float offsetDegAng);
		float GetDegAng() const;
		int Area() const;//获取区域面积
		bool IsEmpty() const;//判断当前区域是否为空
		Point2f Center() const;//获取区域中心坐标	
		void SetScaleRate(float scaleRate);//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
		void SetScaleRate(float scaleRateX, float scaleRateY); 
		void ScaleWithFixedCenter(float scaleRate);//设置区域缩放比例（中心点不变）
		void GetMaxSize(int& maxW, int& maxH) const;
		void points(Point pts[]) const;//返回矩形4个顶点坐标
		void BoundingRect(int Rect[]) const;//获取区域外接矩形
		Rect BoundingRect() const;//获取区域外接矩形	
		Point tl() const;//返回矩形左上角顶点坐标
		Point br() const;//返回矩形右下角顶点坐标
		int rtX() const;
		int btmY() const;
		void SetRtBtmPt(int x, int y);
		void GetRegCornerPt(std::vector<Point2i>& regCornerPt)const;//获取区域边界角点坐标
		void GetRegXAxisLine(Point2f& startPt, Point2f& endPt)const;//获取区域的X方向轴线																									 
		void GetRegXAxisLine(Point2i& startPt, Point2i& endPt)const;//获取区域的X方向轴线
		bool Contain(int ptX, int ptY) const;//是否包含某点
		bool Contain(float ptX, float ptY) const;//是否包含某点
		bool Contain(double ptX, double ptY) const;//是否包含某点
		bool Contain(Point2i pt) const;//是否包含某点								
		bool Contain(Point2i tmpLtTopPt, Point2i tmpRtBtnPt) const;//是否包含某矩形区域
		bool Contain(Point2f pt) const;//是否包含某点		
		bool IsNearCenter(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearCenter(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearCenter(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearCenter(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearCenter(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearLeftEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearLeftEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearLeftEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearLeftEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearLeftEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearRightEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearRightEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearRightEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearRightEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearRightEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearTopEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearTopEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearTopEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearTopEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearTopEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearBtmEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近下边缘
		bool IsNearBtmEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近下边缘
		bool IsNearBtmEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近下边缘
		bool IsNearBtmEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近下边缘
		bool IsNearBtmEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近下边缘																					 
		bool IsNearLtTopEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近左上角																									
		bool IsNearLtTopEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近左上角
		bool IsNearLtTopEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近左上角																										  
		bool IsNearLtTopEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近左上角																														 
		bool IsNearLtTopEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const;//是否靠近左上角
		bool IsNearLtBtmEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近左下角																									
		bool IsNearLtBtmEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近左下角
		bool IsNearLtBtmEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近左下角	
		bool IsNearLtBtmEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近左下角
		bool IsNearLtBtmEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const;//是否靠近左下角
		bool IsNearRtTopEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近右上角																									
		bool IsNearRtTopEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近右上角
		bool IsNearRtTopEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近右上角																											  
		bool IsNearRtTopEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近右上角
		bool IsNearRtTopEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const;//是否靠近右上角
		bool IsNearRtBtmEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近右下角																									
		bool IsNearRtBtmEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近右下角
		bool IsNearRtBtmEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近右下角
		bool IsNearRtBtmEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近右下角
		bool IsNearRtBtmEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const;//是否靠近右下角
	};
	GEOMETRY_API void GetIntersectionRect(Point tlPt, int sizeX, int sizeY, const Rect& rect2, Rect& dstRct);//获取两个不带角度的矩形的交集矩形
	GEOMETRY_API void GetIntersectionRect(Point tlPt, int sizeX, int sizeY, Rect& rect);//获取两个不带角度的矩形的交集矩形
	GEOMETRY_API void GetUnionRect(Point tlPt, int sizeX, int sizeY, const Rect& rect2, Rect& dstRct);//获取两个不带角度的矩形的并集矩形
	GEOMETRY_API void GetUnionRect(Point tlPt, int sizeX, int sizeY, Rect& rect);//获取两个不带角度的矩形的并集矩形															
	GEOMETRY_API void GetUnionRect(const Rect& obj1, const Rect& obj2, Rect& dstRct);//获取两个不带角度的矩形的并集矩形																
	GEOMETRY_API void GetSplitRect(const Rect& obj, bool bSplitHorDir, Rect rects[]);//获取拆分后的矩形

	//ROI区域类_圆形
	class GEOMETRY_API Circle : public HSRoiBase
	{
	public:
		Point cnter_;//圆形中心点
		int radius_;//圆形半径
	public:
		Circle();
		Circle(const Circle& para);
		Circle(int x, int y, int r);
		Circle(Point pt, int r);
		virtual ~Circle();
		Circle& operator = (const Circle& para);//@brief:赋值操作符重载，拷贝功能
		bool equal(const HSRoiBase &obj) const;
		bool operator == (const Circle & obj) const;
		Circle operator+(Point point);//矩形中心点平移	
		Circle operator-(Point point);
		virtual void Init();
		virtual void CopyFrom(const Circle& para);//从para拷贝数据									 
		virtual void CopyTo(Circle& para) const;//拷贝数据到para								  
		RegType GetRegType() const;
		void SetCnter(Point2f cnter);
		void SetCnterOffset(int offsetX, int offsetY);//更新当前区域的中心坐标偏移量
		void SetDegAng(float degAng);
		void SetDegAngOffset(float offsetDegAng);
		float GetDegAng() const;
		int Area() const;//获取区域面积
		bool IsEmpty() const;//判断当前区域是否为空
		Point2f Center() const;//获取区域中心坐标	
		void SetScaleRate(float scaleRate);//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
		void SetScaleRate(float scaleRateX, float scaleRateY);
		void GetMaxSize(int& maxW, int& maxH) const;
		void Points(Point pts[]) const;//返回矩形4个顶点坐标
		void BoundingRect(int Rect[]) const;//获取区域外接矩形
		Rect BoundingRect() const;//获取区域外接矩形	
		int GetCnterPtX() const;//获取圆中心X坐标
		int GetCnterPtY() const;//获取圆中心Y坐标
		int GetMinPtX() const;//获取圆的最小X坐标值
		int GetMinPtY() const;
		int GetMaxPtX() const;
		int GetMaxPtY() const;
		int GetRadius() const;
		int GetDiameter() const;
		void points(Point pts[]) const;//返回圆上的上下左右四个坐标点 Piont数组
		
		bool Contain(int ptX, int ptY) const;//是否包含某点
		bool Contain(Point pt) const;
		//设置数据
		void SetRadius(int r);
		void SetCnterPt(Point cnter);
		//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
		void SetScaleRateAndCnterPt(float scaleRate, Point cnterPt);
		void GetRegCornerPt(std::vector<Point2i>& regCornerPt)const;//获取区域边界角点坐标
		void GetRegXAxisLine(Point2f& startPt, Point2f& endPt)const;//获取区域的X方向轴线																									 
		void GetRegXAxisLine(Point2i& startPt, Point2i& endPt)const;//获取区域的X方向轴线
		bool Contain(float ptX, float ptY) const;//是否包含某点
		bool Contain(double ptX, double ptY) const;//是否包含某点
		bool Contain(Point2i& pt) const;//是否包含某点
		bool Contain(Point2f& pt) const;//是否包含某点
		bool IsNearCenter(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearCenter(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearCenter(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearCenter(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearCenter(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearLeftEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearLeftEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearLeftEdge(int ptX, int boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearLeftEdge(float ptX, float boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearLeftEdge(double ptX, double boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearRightEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearRightEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearRightEdge(int ptX, int boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearRightEdge(float ptX, float boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearRightEdge(double ptX, double boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearTopEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearTopEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearTopEdge(int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearTopEdge(float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearTopEdge(double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearBtmEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近下边缘
		bool IsNearBtmEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近下边缘
		bool IsNearBtmEdge(int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近下边缘
		bool IsNearBtmEdge(float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近下边缘
		bool IsNearBtmEdge(double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近下边缘	
		void UpdateRadiusByEndPt(Point2i pt);//根据边缘终点坐标刷新圆形的半径											
		void UpdateRadiusByEndPt(int ptX, int ptY);//根据边缘终点坐标刷新圆形的半径
		void UpdateRadiusByEndPt(Point2f pt);//根据边缘终点坐标刷新圆形的半径
		void UpdateRadiusByEndPt(float ptX, float ptY);//根据边缘终点坐标刷新圆形的半径
		void UpdateRadiusByPtX(int ptX);//根据边缘终点坐标X刷新圆形的半径	
		void UpdateRadiusByPtX(float ptX);//根据边缘终点坐标X刷新圆形的半径	
		void UpdateRadiusByPtX(double ptX);//根据边缘终点坐标X刷新圆形的半径	
		void UpdateRadiusByPtY(int ptY);//根据边缘终点坐标Y刷新圆形的半径	
		void UpdateRadiusByPtY(float ptY);//根据边缘终点坐标Y刷新圆形的半径	
		void UpdateRadiusByPtY(double ptY);//根据边缘终点坐标Y刷新圆形的半径	
	};

	//ROI区域类_带角度矩形
	class GEOMETRY_API Rect2 : public HSRoiBase
	{
	public:	
		Point2f cnter_;//旋转矩形中心点坐标
		int w_, h_;//旋转矩形长宽
		double degAng_;//旋转矩形旋转角度(逆时针)
	public:
		Rect2();
		Rect2(const Rect2& para);
		Rect2(Point2f cnter, int w, int h, double degAng = 0);
		Rect2(float x, float y, int w, int h, double degAng = 0);
		virtual ~Rect2();
		Rect2& operator = (const Rect2& para);//赋值操作符重载，拷贝功能
		bool equal(const HSRoiBase &obj) const;
		bool operator == (const Rect2 & obj) const;
		Rect2 operator+(Point point);//矩形中心点平移	
		Rect2 operator-(Point point);
		virtual void Init();
		virtual void CopyFrom(const Rect2& para);//从para拷贝数据									 
		virtual void CopyTo(Rect2& para) const;//拷贝数据到para								  
		RegType GetRegType() const;
		void SetCnter(Point2f cnter);
		void SetCnterOffset(int offsetX, int offsetY);//更新当前区域的中心坐标偏移量
		void SetDegAng(float degAng);
		void SetDegAngOffset(float offsetDegAng);
		float GetDegAng() const;
		int Area() const;//获取区域面积				 
		bool IsEmpty() const;//判断当前区域是否为空
		Point2f Center() const;//获取区域中心坐标	
		void GetMaxSize(int& maxW, int& maxH) const;
		void BoundingRect(int rect[]) const;//获取区域外接矩形	
		Rect BoundingRect() const;//获取区域外接矩形	
		
		void SetScaleRate(float scaleRate);//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
		void SetScaleRate(float scaleRateX, float scaleRateY);
		//@brief:返回矩形左上角顶点坐标
		Point tl() const;

		//@brief:返回矩形右上角顶点坐标
		Point tr() const;

		//@brief:返回矩形右下角顶点坐标
		Point br() const;

		//@brief:返回矩形左下角顶点坐标
		Point bl() const;

		//@brief:返回旋转矩形4个顶点坐标
		//@param: Piont数组
		void points(Point pts[]) const;

		//获取对应的点式虚线
		void GetCounterPoint(Circle* dash_point, int nCount = 20);
		
		float GetHalfW() const;
		float GetHalfH() const;
		float GetMaxLenth() const;
		//是否包含某点
		bool Contain(float ptX, float ptY) const;
		bool Contain(int ptX, int ptY) const;
		bool Contain(Point& pt) const;
		bool Contain(Point2i tmpLtTopPt, Point2i tmpRtBtnPt) const;//是否包含某矩形区域
		bool Contain(Point2f& pt) const;
		void SetShapeDat(Point2f cnter, int width, int height, double angle);
		void SetShapeDat(float x, float y, int width, int height, double angle);
		void SetWidth(int width);
		void SetHeight(int height);
		void SetDegAng(double angle);
		void SetDegAngBy2Pt(Point ltTopPt, Point rtTopPt);
		void SetCnterPtX(float ptX);
		void SetCnterPtY(float ptY);
		void SetCnterPt(float ptX, float ptY);
		void SetCnterPt(Point2f newCnterPt);
		//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
		void SetScaleRateX(float scaleRate);
		void SetScaleRateY(float scaleRate);
		void SetScaleRateAndCnterPt(float scaleRate, Point2f newCnterPt);
		void GetRegCornerPt(std::vector<Point2i>& regCornerPt)const;//获取区域边界角点坐标
		void GetRegXAxisLine(Point2f& startPt, Point2f& endPt)const;//获取区域的X方向轴线																									 
		void GetRegXAxisLine(Point2i& startPt, Point2i& endPt)const;//获取区域的X方向轴线
		bool IsNearCenter(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearCenter(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearCenter(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearCenter(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearCenter(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近中心点
		bool IsNearLeftEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearLeftEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearLeftEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearLeftEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearLeftEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近左边缘
		bool IsNearRightEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearRightEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearRightEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearRightEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearRightEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近右边缘
		bool IsNearTopEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearTopEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearTopEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearTopEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearTopEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近上边缘
		bool IsNearBtmEdge(Point2i pt, int boundaryRange, bool bKeepInSide) const;//是否靠近下边缘
		bool IsNearBtmEdge(Point2f pt, float boundaryRange, bool bKeepInSide) const;//是否靠近下边缘
		bool IsNearBtmEdge(int ptX, int ptY, int boundaryRange, bool bKeepInSide) const;//是否靠近下边缘
		bool IsNearBtmEdge(float ptX, float ptY, float boundaryRange, bool bKeepInSide) const;//是否靠近下边缘
		bool IsNearBtmEdge(double ptX, double ptY, double boundaryRange, bool bKeepInSide) const;//是否靠近下边缘																					 
		bool IsNearLtTopEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近左上角																									
		bool IsNearLtTopEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近左上角
		bool IsNearLtTopEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近左上角
		bool IsNearLtTopEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近左上角
		bool IsNearLtTopEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const;//是否靠近左上角
		bool IsNearLtBtmEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近左下角																									
		bool IsNearLtBtmEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近左下角
		bool IsNearLtBtmEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近左下角
		bool IsNearLtBtmEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近左下角
		bool IsNearLtBtmEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const;//是否靠近左下角
		bool IsNearRtTopEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近右上角																									
		bool IsNearRtTopEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近右上角
		bool IsNearRtTopEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近右上角
		bool IsNearRtTopEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近右上角
		bool IsNearRtTopEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const;//是否靠近右上角
		bool IsNearRtBtmEdge(Point2i pt, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近右下角																								
		bool IsNearRtBtmEdge(int ptX, int ptY, int boundaryRangeX, int boundaryRangeY, bool bKeepInSide) const;//是否靠近右下角
		bool IsNearRtBtmEdge(Point2f pt, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近右下角
		bool IsNearRtBtmEdge(float ptX, float ptY, float boundaryRangeX, float boundaryRangeY, bool bKeepInSide) const;//是否靠近右下角
		bool IsNearRtBtmEdge(double ptX, double ptY, double boundaryRangeX, double boundaryRangeY, bool bKeepInSide) const;//是否靠近右下角
		void UpdateWidthByEndPt(Point2i pt);//根据边缘终点坐标刷新角度矩形宽度																													 
		void UpdateWidthByEndPt(Point2f pt);//根据边缘终点坐标刷新角度矩形宽度										 
		void UpdateWidthByEndPt(int ptX, int ptY);//根据边缘终点坐标刷新角度矩形宽度
		void UpdateWidthByEndPt(float ptX, float ptY);//根据边缘终点坐标刷新角度矩形宽度
		void UpdateWidthByEndPt(double ptX, double ptY);//根据边缘终点坐标刷新角度矩形宽度
		void UpdateHeightByEndPt(Point2i pt);//根据边缘终点坐标刷新角度矩形高度																												 
		void UpdateHeightByEndPt(Point2f pt);//根据边缘终点坐标刷新角度矩形高度											 
		void UpdateHeightByEndPt(int ptX, int ptY);//根据边缘终点坐标刷新角度矩形高度	
		void UpdateHeightByEndPt(float ptX, float ptY);//根据边缘终点坐标刷新角度矩形高度	
		void UpdateHeightByEndPt(double ptX, double ptY);//根据边缘终点坐标刷新角度矩形高度																   
		void UpdateWidthAndHeightByEndPt(Point2i pt);//根据边缘终点坐标刷新角度矩形宽度及高度
		void UpdateWidthAndHeightByEndPt(Point2f pt);//根据边缘终点坐标刷新角度矩形宽度及高度
		void UpdateWidthAndHeightByEndPt(int ptX, int ptY);//根据边缘终点坐标刷新角度矩形宽度及高度	
		void UpdateWidthAndHeightByEndPt(float ptX, float ptY);//根据边缘终点坐标刷新角度矩形宽度及高度	
		void UpdateWidthAndHeightByEndPt(double ptX, double ptY);//根据边缘终点坐标刷新角度矩形宽度及高度
	};

	//ROI区域类_椭圆
	class GEOMETRY_API Ellipse : public HSRoiBase
	{
	public:
		Point cnter_;//椭圆的中心坐标 
		int longR_, shortR_;//椭圆的长短半轴
		//椭圆旋转角度(顺时针)
		double degAng_, startDegAng_, endDegAng_;

	public:
		Ellipse();
		Ellipse(const Ellipse& para);
		//@brief:有参构造
		//@param:    cnter:   椭圆中心点坐标
		//@param:    longR:   长半轴
		//@param:    shortR:  短半轴
		//@param:    degAng:  旋转角度(顺时针)
		Ellipse(Point cnter, int longR, int shortR, double degAng = 0, 
			double startDegAng = 0, double endDegAng = 360);
		Ellipse(int x, int y, int longR, int shortR, double degAng = 0,
			double startDegAng = 0, double endDegAng = 360);
		virtual ~Ellipse();
		Ellipse& operator = (const Ellipse& para);//赋值操作符重载，拷贝功能
		bool equal(const HSRoiBase &obj) const;
		bool operator == (const Ellipse & obj) const;
		Ellipse operator+(Point point);//矩形中心点平移	
		Ellipse operator-(Point point);
		virtual void Init();
		virtual void CopyFrom(const Ellipse& para);//从para拷贝数据									 
		virtual void CopyTo(Ellipse& para) const;//拷贝数据到para								  
		RegType GetRegType() const;
		void SetCnter(Point2f cnter);
		void SetCnterOffset(int offsetX, int offsetY);//更新当前区域的中心坐标偏移量
		void SetDegAng(float degAng);
		void SetDegAngOffset(float offsetDegAng);
		float GetDegAng() const;
		int Area() const;//获取区域面积
		bool IsEmpty() const;//判断当前区域是否为空
		Point2f Center() const;//获取区域中心坐标			
		void GetMaxSize(int& maxW, int& maxH) const;
		void Points(Point pts[]) const;//返回矩形4个顶点坐标
		void BoundingRect(int rect[]) const;//获取区域外接矩形
		Rect BoundingRect() const;//获取区域外接矩形	
		void SetScaleRate(float scaleRate);//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
		void SetScaleRate(float scaleRateX, float scaleRateY);
		//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
		void SetScaleRateX(float scaleRate);
		void SetScaleRateY(float scaleRate);
		Point GetCnterPt() const;
		int GetCnterPtX() const;
		int GetCnterPtY() const;
		int GetMinPtX() const;
		int GetMaxPtX() const;
		int GetMinPtY() const;
		int GetMaxPtY() const;
		int GetLongR() const;
		int GetShortR() const;
		int GetMaxLenth() const;
		//是否包含某点
		bool Contain(int ptX, int ptY) const;
		bool Contain(Point& pt) const;//待完善
									  //设置参数
		void SetShapeDat(Point cnter, int major_axes, int minor_axes,
			double angle = 0, double StartAngle = 0, double EndAngle = 360);
		void SetShapeDat(int x, int y, int major_axes, int minor_axes,
			double angle = 0, double StartAngle = 0, double EndAngle = 360);
		void SetLongR(int tmpLongR);
		void SetShortR(int tmpShortR);
		void SetDegAng(double tmpDegAng);
		//更新当前椭圆的中心坐标X
		void SetCnterPtX(int ptX);
		void SetCnterPtY(int ptY);
		void SetCnterPt(Point newCnterPt);
		//设置区域中心及缩放比例
		void SetScaleRateAndCnterPt(float scaleRate, Point newCnterPt);
		void SetScaleRateAndCnterPt(float scaleRate, int ptX, int ptY);
	};

	//ROI区域类_任意多边形
	class GEOMETRY_API Polygon : public HSRoiBase
	{
	public:
		Point* points_;//多边形顶点坐标数组
		int m_N;
	public:

		Polygon();
		Polygon(const Polygon& para);
		//@brief: 有参构造
		//@param: points 多边形顶点坐标数组
		//@param: N      多边形顶点个数
		Polygon(Point* points, int N);
		virtual ~Polygon();
		Polygon& operator = (const Polygon& para);//赋值操作符重载，拷贝功能
		bool equal(const HSRoiBase &obj) const;
		bool operator == (const Polygon & obj) const;
		Polygon operator+(Point point);//矩形中心点平移	
		Polygon operator-(Point point);
		virtual void Init();
		virtual void CopyFrom(const Polygon& para);//从para拷贝数据									 
		virtual void CopyTo(Polygon& para) const;//拷贝数据到para								  
		RegType GetRegType() const;
		void SetCnter(Point2f cnter);
		void SetCnterOffset(int offsetX, int offsetY);//更新当前区域的中心坐标偏移量
		void SetDegAng(float degAng);
		void SetDegAngOffset(float offsetDegAng);
		float GetDegAng() const;
		int Area() const;//获取区域面积
		bool IsEmpty() const;//判断当前区域是否为空
		Point2f Center() const;//获取区域中心坐标	
		void GetMaxSize(int& maxW, int& maxH) const;
		void Points(Point pts[]) const;//返回矩形4个顶点坐标
		void BoundingRect(int rect[]) const;//获取区域外接矩形
		Rect BoundingRect() const;//获取区域外接矩形	
		void SetScaleRate(float scaleRate);//设置区域缩放比例（scaleRate>1表示区域放大，scaleRate < 表示区域缩小）
		void SetScaleRate(float scaleRateX, float scaleRateY);
	};

#if USING_OLD_HS_REG //IA2.0默认不启用旧的HSReg数据结构 @ChenW 08/01/2024, 11:55
	GEOMETRY_API void TransHSReg(const HSV::HSRoiBase* pRoi, HSReg& reg);

	GEOMETRY_API void TransHSReg(const HSReg& reg, HSV::HSRoiBase** pRoi);
#endif //IA2.0默认不启用旧的HSReg数据结构 @ChenW 08/01/2024, 11:55

	GEOMETRY_API void SwitchHSRoiObjType(HSV::RegType regType, HSRoiBase** pObj, bool bTransDat = false);

	GEOMETRY_API void CopyHSRoiObjDat(const HSRoiBase* srcObj, HSRoiBase** dstObj);

	GEOMETRY_API void GenNewRoiObj(const HSRoiBase* srcObj, const HSV::Point2f& pos, HSRoiBase** dstObj);

	GEOMETRY_API bool EqualRoiPara(const HSRoiBase* srcObj, const HSRoiBase* dstObj);

	GEOMETRY_API void ClearHSRoiReg(HSRoiBase** pObj);

	GEOMETRY_API void ClearHSRoiReg(std::vector<HSRoiBase*> roi1D);

	GEOMETRY_API void ClearHSRoiReg(HSV::Rect** pObj);

	GEOMETRY_API void ClearHSRoiReg(HSV::Circle** pObj);

	GEOMETRY_API void ClearHSRoiReg(HSV::Rect2** pObj);

	GEOMETRY_API void ClearHSRoiReg(HSV::Ellipse** pObj);

	GEOMETRY_API void ClearHSRoiReg(HSV::Polygon** pObj);

	GEOMETRY_API HSV::Rect BoundingRect(const HSRoiBase* pObj);

	GEOMETRY_API HSV::Rect BoundingRect(const std::vector<HSV::HSRoiBase*>& pObj1D);

	GEOMETRY_API HSV::Rect2 BoundingRect2(const HSRoiBase* pObj);

	GEOMETRY_API HSV::Rect GetIntersectionRect(const std::vector<HSV::HSRoiBase*> pRoi1D);
}

