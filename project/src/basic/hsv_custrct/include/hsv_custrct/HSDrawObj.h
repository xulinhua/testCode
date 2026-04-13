//
// @brief: 图形形状区域类
// @birth: created by LuoJianghong on 2023-07-03
// @version: V0.0.1
// @revision: last revised by LuoJianghong on 2023-07-03
//
#pragma once
#ifndef GEOMETRY_H
#define GEOMETRY_H
#include<vector>
#include<string>
//#include<cmath>
#include<map>
#include<list>
//#include<assert.h>
//#include<stdio.h>
#include"HSRoi.h"
//#pragma warning(disable:4251)

#ifndef HSV_CUSTRCT_EXPORTS
#define HSV_CUSTRCT_EXPORTS
#endif
// Use platform-independent definition
#ifdef _WIN32
    #ifdef HSV_CUSTRCT_EXPORTS
        #define HS_DRAW_OBJ_API __declspec(dllexport)
    #else
        #define HS_DRAW_OBJ_API __declspec(dllimport)
    #endif
#elif __linux__
    #define HS_DRAW_OBJ_API __attribute__((visibility("default")))
#else
    #define HS_DRAW_OBJ_API
#endif

namespace HSV
{

	/** types of line
	@ingroup imgproc_draw
	*/
	enum LineTypes {
		FILLED = -1,
		LINE_4 = 4, //!< 4-connected line
		LINE_8 = 8, //!< 8-connected line
		LINE_AA = 16 //!< antialiased line
	};

	// @brief: 用于存储图像颜色通道值(参考opencv的cv::Scalar)
	// @birth: created by LJH on 20230710
	template<typename _Tp> class HS_DRAW_OBJ_API ColorGC_//BGR蓝绿红
	{
	public:
		//! default constructor
		ColorGC_();
		ColorGC_(_Tp v0, _Tp v1, _Tp v2 = 0, _Tp v3 = 0);
		ColorGC_(_Tp v0);
		ColorGC_(const ColorGC_<_Tp>& s);
		ColorGC_& operator=(const ColorGC_& s);
		bool operator==(const ColorGC_& s) const;
		ColorGC_& operator()(_Tp v0, _Tp v1, _Tp v2 = 0, _Tp v3 = 0);
		ColorGC_& operator()(_Tp v0);
		_Tp val[4];

	};
	typedef ColorGC_<double> ScalarGC;//BGR蓝绿红
	//typedef Scalar_d ScalarGC;

#define GC_RGB(r, g, b)  ScalarGC((b), (g), (r), 0)//BGR蓝绿红

	//部分颜色枚举
	typedef enum class _GC_COL
	{
		GC_COL_DEFAULT,
		GC_COL_RANDOM,
		GC_COL_RED,
		GC_COL_GREEN,
		GC_COL_BLUE,
		GC_COL_LIGHT_PINK,
		GC_COL_PINK,
		GC_COL_CRIMSON,
		GC_COL_DEEP_PINK,
		GC_COL_MAGENTA,
		GC_COL_PURPLE,
		GC_COL_INDIGO,
		GC_COL_LIGHT_SKY_BLUE,
		GC_COL_SKY_BLUE,
		GC_COL_SLATE_BLUE,
		GC_COL_DARK_BLUE,
		GC_COL_NAVY_BLUE,
		GC_COL_LIGHT_CYAN,
		GC_COL_CYAN,
		GC_COL_PEA_GREEN,
		GC_COL_FOREST_GREEN,
		GC_COL_LIME_GREEN,
		GC_COL_LAWN_GREEN,
		GC_COL_WHEAT,
		GC_COL_YELLOW,
		GC_COL_OLIVE,
		GC_COL_ORANGE,
		GC_COL_CORAL,
		GC_COL_TOMATO,
		GC_COL_BROWN,
		GC_COL_BLACK,
		GC_COL_GRAY,
		GC_COL_WHITE, 
		GC_COL_MAX
	}GC_COL;

	static std::map<GC_COL, ScalarGC> g_GCColor1D;//可供选择的颜色
	static bool bInited = false;

	void InitGCColor();//初始化自定义颜色

	void HS_DRAW_OBJ_API UpdateGCColor(GC_COL& clrType);//刷新当前自定义颜色

	ScalarGC HS_DRAW_OBJ_API GetGCColor(GC_COL clrType);//获取转换后的opencv颜色

	typedef enum _DrawObjType//绘图目标类型
	{
		DRAW_NONE,
		DRAW_RECT,
		DRAW_RECT2,
		DRAW_CIRCLE,
		DRAW_ELLIPSE,
		DRAW_POLYGON,
		DRAW_LINE,
		DRAW_CROSS,
		DRAW_ARROW,
		DRAW_TEXT,
		DRAW_TEXT_UNION,
		DRAW_TEXT_TUPLE,
		DRAW_CONTOURS,
		DRAW_POINT
	}DrawType;

	typedef enum class _ParaIdx_RectDraw
	{
		ParaIdx_LtTopPtX,//左上角坐标x
		ParaIdx_LtTopPtY,//左上角坐标y
		ParaIdx_W,//矩形宽
		ParaIdx_H,//矩形高		
	}ParaIdx_RectDraw;

	typedef enum class _ParaIdx_CircleDraw
	{
		ParaIdx_CenterX,//中心坐标x
		ParaIdx_CenterY,//中心坐标y
		ParaIdx_Radius,//半径		
	}ParaIdx_CircleDraw;

	typedef enum class _ParaIdx_Rect2Draw
	{
		ParaIdx_CenterX,//中心坐标x
		ParaIdx_CenterY,//中心坐标y
		ParaIdx_W,//矩形宽
		ParaIdx_H,//矩形高
		ParaIdx_Angle,//角度		
	}ParaIdx_Rect2Draw;

	typedef enum class _ParaIdx_EllipseDraw
	{
		ParaIdx_CenterX,//中心坐标x
		ParaIdx_CenterY,//中心坐标y
		ParaIdx_LongR,//长半轴
		ParaIdx_ShortR,//短半轴
		ParaIdx_Angle,//旋转角度(顺时针)
		ParaIdx_StartAngle,//开始角度	
		ParaIdx_EndAngle,//结束角度
	}ParaIdx_EllipseDraw;

	typedef enum class _ParaIdx_LineDraw
	{
		ParaIdx_StartPtX,//起点坐标x
		ParaIdx_StartPtY,//起点坐标y
		ParaIdx_EndPtX,//终点坐标x
		ParaIdx_EndPtY,//终点坐标y	
	}ParaIdx_LineDraw;

	//绘图对象基类
	class HS_DRAW_OBJ_API DrawObjBase
	{
	public:
		ScalarGC color_;//BGR蓝绿红
		int thickness_ = 1;//线条的宽度，负值，如 FILLED、-1，意味着函数必须绘制一个填充的矩形
		int lineType_ = LINE_AA;//线段的类型。可以取值8， 4， 和CV_AA， 分别代表8邻接连接线，4邻接连接线和反锯齿连接线。默认值为8邻接。为了获得更好地效果可以选用CV_AA(采用了高斯滤波)。
		int shift_ = 0;//点坐标中的小数位数，一般取0，因为像素一般都是整型值。
		long long nID_ = -1;// nID == -1;时 判断是否相等通过内部值来判断，如果 nID>=0时，则通过nID来判断。
	public:
		DrawObjBase();
		DrawObjBase(const DrawObjBase& para);
		virtual ~DrawObjBase();
		virtual DrawObjBase& operator = (const DrawObjBase& para);
		virtual bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const DrawObjBase & obj);
		virtual void CopyFrom(const DrawObjBase& para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase& para) const;//拷贝数据到para
		//virtual void CopyFrom(const DrawObjBase* para) = 0;//从para拷贝数据									 
		//virtual void CopyTo(DrawObjBase** para) const = 0;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const = 0;//当前对象是否为空（主要用于判断当前区域是否为空）
		virtual HSV::DrawType GetDrawType() const = 0;//获取绘制图形类型
		//设置参数
		virtual bool IsRoiReg() const = 0;
		void SetDrawDat(GC_COL clrType, int thickness, int lineType, int shift);
		void SetDrawDat(ScalarGC color, int thickness, int lineType, int shift);
		void SetColor(GC_COL clrType);
		void SetColor(ScalarGC color);
	protected:
	};
	typedef std::vector<HSV::DrawObjBase> DrawObj1D;
	typedef std::vector<HSV::DrawObjBase*> DrawObjPtr1D;

	//绘图对象_Rect类（不带角度矩形）
	class HS_DRAW_OBJ_API RectDraw : public Rect, public DrawObjBase
	{
	public:
		bool isDash_;  //是否为虚线  luojianghong 23-9-4
		int  nCount_;  //线段比例    luojianghong 23-9-4
	public:
		RectDraw();
		RectDraw(const RectDraw& para);
		RectDraw(Point ltTopPt, int w, int h, GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		RectDraw(Point ltTopPt, int w, int h, ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		RectDraw(int x, int y, int w, int h, GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		RectDraw(int x, int y, int w, int h, ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		virtual ~RectDraw();
		virtual RectDraw& operator = (const RectDraw& para);
		bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const RectDraw & obj);
		virtual void CopyFrom(const RectDraw& para);//从para拷贝数据									 
		virtual void CopyTo(RectDraw& para) const;//拷贝数据到para
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;
	};

	//绘图对象_圆形
	class HS_DRAW_OBJ_API CircleDraw : public Circle, public DrawObjBase
	{
	public:
		CircleDraw();
		CircleDraw(const CircleDraw& para);
		CircleDraw(Point cnter, int r, GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		CircleDraw(Point cnter, int r, ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		CircleDraw(int x, int y, int r, GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		CircleDraw(int x, int y, int r, ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		virtual ~CircleDraw();
		virtual CircleDraw& operator = (const CircleDraw& para);
		bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const CircleDraw & obj);
		virtual void CopyFrom(const CircleDraw& para);//从para拷贝数据									 
		virtual void CopyTo(CircleDraw& para) const;//拷贝数据到para
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;
	};

	//绘图对象_Rect2类（带角度矩形）
	class HS_DRAW_OBJ_API Rect2Draw : public Rect2, public DrawObjBase
	{
	public:
		bool isDash_;  //是否为虚线  luojianghong 23-9-4
		int  nCount_;  //线段比例    luojianghong 23-9-4


		Rect2Draw();
		Rect2Draw(const Rect2Draw& para);

		Rect2Draw(Point2f cnter, int w, int h, double degAng, 
			GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);

		Rect2Draw(Point2f cnter, int w, int h, double degAng,
			ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);

		Rect2Draw(float x, float y, int w, int h, double degAng, 
			GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);

		Rect2Draw(float x, float y, int w, int h, double degAng,
			ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);

		virtual ~Rect2Draw();
		virtual Rect2Draw& operator = (const Rect2Draw& para);
		bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const Rect2Draw & obj);
		virtual void CopyFrom(const Rect2Draw& para);//从para拷贝数据									 
		virtual void CopyTo(Rect2Draw& para) const;//拷贝数据到para
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型	
		virtual bool IsRoiReg() const;
		void SetRectDat(const HSV::Rect& rect);
		//void GetLinePoint(LineDraw* dash_line, int nCount = 20);//获取对应的线段式虚线
	};

	class HS_DRAW_OBJ_API EllipseDraw : public Ellipse, public DrawObjBase
	{
	public:
		EllipseDraw();
		EllipseDraw(const EllipseDraw& para);

		//@brief:有参构造
		//@param:    cnter:   椭圆中心点坐标
		//@param:    longR:   长半轴
		//@param:    shortR:  短半轴
		//@param:    degAng:  旋转角度(顺时针)
		EllipseDraw(Point cnter, int longR, int shortR, double degAng = 0,
			double startDegAng = 0, double endDegAng = 360, 
			GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);

		EllipseDraw(Point cnter, int longR, int shortR, double degAng,
			double startDegAng, double endDegAng,
			ScalarGC color, int thickness = 1, int lineType = LINE_AA, int shift = 0);

		EllipseDraw(int x, int y, int longR, int shortR, double degAng = 0,
			double startDegAng = 0, double endDegAng = 360,
			GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);

		EllipseDraw(int x, int y, int longR, int shortR, double degAng,
			double startDegAng, double endDegAng,
			ScalarGC color, int thickness = 1, int lineType = LINE_AA, int shift = 0);

		virtual ~EllipseDraw();
		virtual EllipseDraw& operator = (const EllipseDraw& para);
		bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const EllipseDraw & obj);
		virtual void CopyFrom(const EllipseDraw& para);//从para拷贝数据									 
		virtual void CopyTo(EllipseDraw& para) const;//拷贝数据到para
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;
	};

	class HS_DRAW_OBJ_API PolygonDraw : public Polygon, public DrawObjBase
	{
	public:
		PolygonDraw();
		PolygonDraw(const PolygonDraw& para);

		//@brief: 有参构造
		//@param: points 多边形顶点坐标数组
		//@param: N      多边形顶点个数
		PolygonDraw(Point* points, int N, GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		PolygonDraw(Point* points, int N, ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);

		virtual ~PolygonDraw();
		virtual PolygonDraw& operator = (const PolygonDraw& para);
		bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const PolygonDraw & obj);
		virtual void CopyFrom(const PolygonDraw& para);//从para拷贝数据									 
		virtual void CopyTo(PolygonDraw& para) const;//拷贝数据到para
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;
	};

	//绘图对象_Line直线
	class HS_DRAW_OBJ_API LineDraw : public DrawObjBase
	{
	public:	
		Point pt1_ = Point(0, 0);//直线起点坐标
		Point pt2_ = Point(0, 0);//直线终点坐标
		//int len_;//直线长度
		//double degAng_;//直线角度（逆时针）
	public:	
		LineDraw();
		LineDraw(const LineDraw& para);
		LineDraw(Point p1, Point p2,
			GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		LineDraw(Point p1, Point p2, ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		//@brief:有参构造
		//@param:    pt1    : 直线起点
		//@param:   len     : 直线长度
		//@param:   degAng  : 直线角度（逆时针）
		//@param:   color   : 颜色
		//@param: thickness : 线条的宽度，负值，如 FILLED、-1，意味着函数必须绘制一个填充的矩形
		//@param: lineType  : 线段的类型。可以取值8， 4， 和LINE_AA， 分别代表8邻接连接线，4邻接连接线和反锯齿连接线。
		//                    默认值为8邻接。为了获得更好地效果可以选用LINE_AA(采用了高斯滤波)。
		//@param:  shift    : 点坐标中的小数位数，一般取0，因为像素一般都是整型值。
		LineDraw(Point p1, int len, double degAng,
			GC_COL color = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		LineDraw(Point p1, int len, double degAng, ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		virtual ~LineDraw();
		virtual LineDraw& operator = (const LineDraw& para);
		bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const LineDraw & obj);
		LineDraw operator-(const Point& point);//直线中心点平移
		LineDraw operator+(const Point& point);
		virtual void CopyFrom(const LineDraw& para);//从para拷贝数据									 
		virtual void CopyTo(LineDraw& para) const;//拷贝数据到para
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;

		//获取对应的点式虚线
		void GetCounterPoint(Circle* dash_point, int nCount = 20);
		//获取对应的线段式虚线
		void GetLinePoint(LineDraw* dash_line, int nCount = 20);

		void SetShapeData(Point p1, int len, double degAng);
		void SetShapeData(Point p1, Point p2);
		//void UpdateShapeDataByPt(); 
		int Lenth() const;
		double DegAng() const;
		/*void SetLeng(int len);
		void SetDegAng(double degAng);*/
		void SetStartPt(Point pt);
		void SetEndPt(Point pt);
		/*int GetLeng();
		double GetDegAng();*/
		Point GetStartPt();
		Point GetEndPt();
	};

	//绘图对象_Arrow箭头
	class HS_DRAW_OBJ_API ArrowDraw : public DrawObjBase
	{
	public:
		Point pt1_;//箭头起点坐标
		Point pt2_;//箭头终点坐标
		int len_;//长度
		double degAng_;//角度（逆时针）
		double tipLen_ = 0.1;//箭头尖端相对于箭头长度的长度
	public:
		ArrowDraw();
		ArrowDraw(const ArrowDraw& para);

		//@brief:有参构造
		//@param: pt1，pt2  : 直线起点，终点坐标
		//@param:   color   : 颜色
		//@param: thickness : 线条的宽度，负值，如 FILLED、-1，意味着函数必须绘制一个填充的矩形
		//@param: lineType  : 线段的类型。可以取值8， 4， 和LINE_AA， 分别代表8邻接连接线，4邻接连接线和反锯齿连接线。
		//                    默认值为8邻接。为了获得更好地效果可以选用LINE_AA(采用了高斯滤波)。
		//@param:  shift    : 点坐标中的小数位数，一般取0，因为像素一般都是整型值。
		//@param: tipLength : 箭头尖端相对于箭头长度的长度
		ArrowDraw(Point p1, Point p2, GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0, double tipLength = 0.1);
		ArrowDraw(Point p1, Point p2, ScalarGC color, 
			int thickness = 1, int lineType = LINE_AA, int shift = 0, double tipLength = 0.1);

		//@brief:有参构造
		//@param:    pt1    : 直线起点
		//@param:   len     : 直线长度
		//@param:   degAng  : 直线角度（逆时针）
		//@param:   color   : 颜色
		//@param: thickness : 线条的宽度，负值，如 FILLED、-1，意味着函数必须绘制一个填充的矩形
		//@param: lineType  : 线段的类型。可以取值8， 4， 和LINE_AA， 分别代表8邻接连接线，4邻接连接线和反锯齿连接线。
		//                    默认值为8邻接。为了获得更好地效果可以选用LINE_AA(采用了高斯滤波)。
		//@param:  shift    : 点坐标中的小数位数，一般取0，因为像素一般都是整型值。
		//@param: tipLength : 箭头尖端相对于箭头长度的长度
		ArrowDraw(Point p1, int len, double degAng, GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0, double tipLength = 0.1);
		ArrowDraw(Point p1, int len, double degAng, ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0, double tipLength = 0.1);
		virtual ~ArrowDraw();
		virtual ArrowDraw& operator = (const ArrowDraw& para);
		bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const ArrowDraw & obj);
		ArrowDraw operator-(const Point& point);//直线中心点平移
		ArrowDraw operator+(const Point& point);
		virtual void CopyFrom(const ArrowDraw& para);//从para拷贝数据									 
		virtual void CopyTo(ArrowDraw& para) const;//拷贝数据到para	
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;
		void SetShapeData(Point p1, int len, double degAng);
		void SetShapeData(Point p1, Point p2);
		void SetLeng(int len);
		void SetDegAng(double degAng);
		void SetStartPt(Point pt);
		void SetEndPt(Point pt);
		int GetLeng();
		double GetDegAng();
		Point GetStartPt();
		Point GetEndPt();
	};

	//绘图对象_Cross十字叉
	class HS_DRAW_OBJ_API CrossDraw : public DrawObjBase
	{
	public:	
		Point cnter_;//@brief:十字叉中心点坐标
		int horLen_, verLen_;//@brief:水平线、垂直线半长度
		double degAng_;//@brief:角度（非弧度、逆时针）
	public:

		CrossDraw();
		CrossDraw(const CrossDraw& para);
		//@brief:有参构造
		//@param:   cnter   : 中心点坐标
		//@param:   len     : 水平线、垂直线半长度
		//@param:   degAng  : 角度（逆时针）
		//@param:   color   : 颜色
		//@param: thickness : 线条的宽度，负值，如 FILLED、-1，意味着函数必须绘制一个填充的矩形
		//@param: lineType  : 线段的类型。可以取值8， 4， 和LINE_AA， 分别代表8邻接连接线，4邻接连接线和反锯齿连接线。
		//                    默认值为8邻接。为了获得更好地效果可以选用LINE_AA(采用了高斯滤波)。
		//@param:  shift    : 点坐标中的小数位数，一般取0，因为像素一般都是整型值。
		CrossDraw(Point cnter, int len, double degAng = 0,
			GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		CrossDraw(Point cnter, int len, double degAng, ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		
		//@brief:有参构造
		//@param:   center  : 中心点坐标
		//@param: horLen，verLen  : 水平线、垂直线半长度
		//@param:   degAng  : 角度（逆时针）
		//@param:   color   : 颜色
		//@param: thickness : 线条的宽度，负值，如 FILLED、-1，意味着函数必须绘制一个填充的矩形
		//@param: lineType  : 线段的类型。可以取值8， 4， 和LINE_AA， 分别代表8邻接连接线，4邻接连接线和反锯齿连接线。
		//                    默认值为8邻接。为了获得更好地效果可以选用LINE_AA(采用了高斯滤波)。
		//@param:  shift    : 点坐标中的小数位数，一般取0，因为像素一般都是整型值。
		CrossDraw(Point cnter, int horLen, int verLen,
			double degAng, GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		CrossDraw(Point cnter, int horLen, int verLen,
			double degAng, ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);

		virtual ~CrossDraw();
		virtual CrossDraw& operator = (const CrossDraw& para);
		bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const CrossDraw & obj);
		CrossDraw operator-(const Point& point);//直线中心点平移
		CrossDraw operator+(const Point& point);
		virtual void CopyFrom(const CrossDraw& para);//从para拷贝数据									 
		virtual void CopyTo(CrossDraw& para) const;//拷贝数据到para	
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;
		//@brief:获取十字叉的四个顶点
		//@param: point : Point point[4]
		void points(Point point[]) const;

		//@brief:获取十字叉的两条线
		//@param: Line :LineDraw Line[2]
		void lines(LineDraw Line[]) const;


	};

	// 移除了不必要的标准库模板显式实例化

	//绘图对象_Contours点集
	class HS_DRAW_OBJ_API ContoursDraw : public DrawObjBase
	{
	public:

		//轮廓坐标数组
		std::vector<Point> points_;
	public:
		ContoursDraw();
		ContoursDraw(const ContoursDraw& para);
		ContoursDraw(const std::vector<Point>& points, GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		ContoursDraw(const std::vector<Point>& points, ScalarGC color,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		virtual ~ContoursDraw();
		virtual ContoursDraw& operator = (const ContoursDraw& para);
		bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const ContoursDraw & obj);
		//ContoursDraw operator-(const Point& point);//直线中心点平移
		//ContoursDraw operator+(const Point& point);
		virtual void CopyFrom(const ContoursDraw& para);//从para拷贝数据									 
		virtual void CopyTo(ContoursDraw& para) const;//拷贝数据到para	
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;
	};
	
	//绘图对象_点
	class HS_DRAW_OBJ_API PointDraw : public DrawObjBase
	{
	public:
		Point point_;

		PointDraw();
		PointDraw(const PointDraw& para);
		PointDraw(Point pt, GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		PointDraw(Point pt, ScalarGC color = ScalarGC(255, 0, 0),
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		PointDraw(int x, int y, GC_COL clrType = GC_COL::GC_COL_RED,
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		PointDraw(int x, int y, ScalarGC color = ScalarGC(255, 0, 0),
			int thickness = 1, int lineType = LINE_AA, int shift = 0);
		virtual ~PointDraw();
		virtual PointDraw& operator = (const PointDraw& para);
		bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const PointDraw & obj);
		//PointDraw operator-(const Point& point);
		//PointDraw operator+(const Point& point);
		virtual void CopyFrom(const PointDraw& para);//从para拷贝数据									 
		virtual void CopyTo(PointDraw& para) const;//拷贝数据到para	
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;
	};

	//普通文本
	class HS_DRAW_OBJ_API TextDraw : public DrawObjBase
	{
	public:
		std::string strTxt_;//文本
		Point ltBtmPt_;//文本左下角坐标
		int nFontSize_;//字体大小
		const char *fn_;//设定显示的字符的TrueType字体类型，默认使用Arial字体
		bool bItalic_;//字体是否斜体
		bool bUnderline_;//字体是否有下划线
		bool bBox_;// ture：有白底框      false:无白底框
		ScalarGC bkgcolor_;
		bool bTxtInImage_;// true:位置相对于图片    false:位置相对于窗口				  
		int txtW_;//@brief:文本高宽
		int txtH_;

	public:
		TextDraw();
		TextDraw(const TextDraw& para);
		//@brief:有参构造
		//@param:    str    : 直线起点，终点坐标
		//@param:    org    : 文本左下角坐标
		//@param:   color   : 颜色
		//@param:  fontSize : 字体大小
		//@param:     fn    : 设定显示的字符的TrueType字体类型，默认使用Arial字体
		//@param:  bItalic  : 字体是否斜体。
		//@param: bUnderline: 字体是否有下划线
		TextDraw(const char* str, Point ltBtmPt, GC_COL clrType = GC_COL::GC_COL_RED,
			int fontSize = 20, const char *fn = "Arial", bool bItalic = false,
			bool bUnderline = false);
		TextDraw(const char* str, Point ltBtmPt, ScalarGC color,
			int fontSize = 20, const char *fn = "Arial", bool bItalic = false,
			bool bUnderline = false);
		virtual ~TextDraw();
		virtual TextDraw& operator = (const TextDraw& para);
		bool equal(const DrawObjBase &obj) const;
		bool operator==(const TextDraw & obj);
		virtual void CopyFrom(const TextDraw& para);//从para拷贝数据									 
		virtual void CopyTo(TextDraw& para) const;//拷贝数据到para
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;
		//@brief:清理文本信息
		void ClearTextInfo();

		//@brief:在文本后面添加字符
		void AddIntervalSymbolMsg(const std::string& strMsg);


		//@brief:设置当前文本配置数据	
		//@param: tmp_clrType  : 直线起点，终点坐标
		//@param: tmp_fontSize : 文本左下角坐标
		//@param:    tmp_fn    : 颜色
		//@param:  tmp_bItalic : 字体大小
		//@param:tmp_bUnderline: 设定显示的字符的TrueType字体类型，默认使用Arial字体
		void SetTxtConfigDat(GC_COL color_type, int tmp_fontSize, const char *tmp_fn, bool tmp_bItalic, bool tmp_bUnderline);
		void SetTxtConfigDat(ScalarGC color, int tmp_fontSize, const char *tmp_fn, bool tmp_bItalic, bool tmp_bUnderline);


		void GetStdTextSize(int& w, int& h) const;//获取字体显示需要占用的尺寸											
		void UpdateTextSize();                    //更新当前文本尺寸大小						
		void GetTextSize(const char* str, int* w, int* h) const;//获取字体显示需要占用的尺寸														   															   


	};
	/*typedef TextDraw DrawStdText;
	typedef std::vector<TextDraw> CvDspStdTxt1D;
	typedef std::vector<CvDspStdTxt1D> CvDspStdTxt2D;*/
	typedef std::vector<TextDraw> TextDraw1D;

	//组合文本
	class HS_DRAW_OBJ_API TextUnionDraw : public DrawObjBase
	{
	public:
		TextDraw lab_;//@brief:标签文本，用于对当前文本对象的说明（比主题文本字体小5）
		TextDraw sbj_;//@brief:文本主体对象subject
		int txtW_;//@brief:文本整体高宽
		int txtH_;
		Point ltBtmPt_;//@brief:文本左下角坐标
		bool bHorUnion_ = true;//@brief:组合文本的排序方向，true为横向，false为竖向
	public:
		TextUnionDraw();//@brief:无参构造，默认所有值为0， ID为-1
		TextUnionDraw(const TextUnionDraw& para);
		//@brief:有参构造
		//@param:    lable  : 标签文本
		//@param:    sbj    : 主体文本
		//@param:    org    : 文本左下角坐标
		//@param: diRection : 组合文本的排序方向
		TextUnionDraw(TextDraw lable, TextDraw sbj, Point org, int diRection = true);

		//@brief:有参构造
		//@param: lable_str : 标签文本
		//@param:  sbj_str  : 主体文本
		//@param:    org    : 文本左下角坐标
		//@param:   color   : 颜色
		//@param: diRection : 组合文本的排序方向
		//@param:  fontSize : 字体大小
		//@param:     fn    : 设定显示的字符的TrueType字体类型，默认使用Arial字体
		//@param:  bItalic  : 字体是否斜体。
		//@param: bUnderline: 字体是否有下划线
		TextUnionDraw(const char* lable_str, const char* sbj_str, Point org, GC_COL tmp_clrType = GC_COL::GC_COL_RED,
			int diRection = true, int fontSize = 20, const char *fn = "Arial", bool bItalic = false,
			bool bUnderline = false);
		TextUnionDraw(const char* lable_str, const char* sbj_str, Point org, ScalarGC color,
			int diRection = true, int fontSize = 20, const char *fn = "Arial", bool bItalic = false,
			bool bUnderline = false);

		virtual ~TextUnionDraw();
		virtual TextUnionDraw& operator = (const TextUnionDraw& para);
		bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const TextUnionDraw & obj);
		virtual void CopyFrom(const TextUnionDraw& para);//从para拷贝数据									 
		virtual void CopyTo(TextUnionDraw& para) const;//拷贝数据到para	
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;

		void UpdateLableFontSizeAuto();//自动更新标签文本字体大小
		void UpdateLablePositionAuto();//自动更新标签坐标
		void ClearTextInfo();
		void GetDspTxtSize(int& w, int& h, int& lableWidth, int& sbjWidth) const;//获取显示文本需要占用的整体尺寸												 
		void GetDspTxtSize(int& w, int& h, int& sbjWidth) const;//获取显示文本需要占用的整体尺寸														
		void GetDspTxtSize(int& w, int& h) const;//获取显示文本需要占用的整体尺寸															  
		void UpdateDspTxtSize();//更新当前显示对象的文本尺寸大小						
	};

	//元组文本
	class HS_DRAW_OBJ_API TextTupleDraw : public DrawObjBase
	{
	public:
		TextDraw tupName_;//元组名称说明--显示文本标签，用于对当前文本对象的说明
		TextDraw1D txt1D_;
	public:
		TextTupleDraw();//@brief:无参构造，默认所有值为0， ID为-1
		TextTupleDraw(const TextTupleDraw& para);
		
		virtual ~TextTupleDraw();
		virtual TextTupleDraw& operator = (const TextTupleDraw& para);
		bool equal(const DrawObjBase &obj) const;
		virtual bool operator==(const TextTupleDraw & obj);
		virtual void CopyFrom(const TextTupleDraw& para);//从para拷贝数据									 
		virtual void CopyTo(TextTupleDraw& para) const;//拷贝数据到para	
		virtual void CopyFrom(const DrawObjBase* para);//从para拷贝数据									 
		virtual void CopyTo(DrawObjBase** para) const;//拷贝数据到para
		virtual void Init();
		virtual bool IsEmpty() const;//当前对象是否为空（主要用于判断当前区域是否为空）
		HSV::DrawType GetDrawType() const;//获取绘制图形类型
		virtual bool IsRoiReg() const;
		void ClearTextInfo();
	};

	HS_DRAW_OBJ_API void SwitchDrawObjType(HSV::DrawType drawType, DrawObjBase** pObj, bool bTransDat = false);

	HS_DRAW_OBJ_API void CopyDrawObjDat(const DrawObjBase* srcObj, DrawObjBase** dstObj);

	HS_DRAW_OBJ_API void CopyDrawObjDat(const std::vector<DrawObjBase*>& srcObj1D, std::vector<DrawObjBase*>& dstObj1D);

	HS_DRAW_OBJ_API void CopyDrawObjRegDat_SameDrawType(const DrawObjBase* srcObj, DrawObjBase* dstObj);

	HS_DRAW_OBJ_API void GenNewDrawObj(const DrawObjBase* srcObj, const HSV::Point2f& pos, DrawObjBase** dstObj);

	HS_DRAW_OBJ_API void ClearDrawObj(DrawObjBase** pObj);

	HS_DRAW_OBJ_API void ClearDrawObj(std::vector<DrawObjBase*> &obj1D);

	HS_DRAW_OBJ_API void ClearDrawObj(HSV::RectDraw** pObj);

	HS_DRAW_OBJ_API void ClearDrawObj(HSV::CircleDraw** pObj);

	HS_DRAW_OBJ_API void ClearDrawObj(HSV::Rect2Draw** pObj);

	HS_DRAW_OBJ_API void ClearDrawObj(HSV::EllipseDraw** pObj);

	HS_DRAW_OBJ_API void ClearDrawObj(HSV::PolygonDraw** pObj);

	HS_DRAW_OBJ_API void ClearDrawObj(HSV::LineDraw** pObj);

	HS_DRAW_OBJ_API void ClearDrawObj(HSV::ArrowDraw** pObj);

	HS_DRAW_OBJ_API void ClearDrawObj(HSV::CrossDraw** pObj);

	HS_DRAW_OBJ_API void ClearDrawObj(HSV::ContoursDraw** pObj);

	HS_DRAW_OBJ_API void ClearDrawObj(HSV::TextDraw** pObj);

	HS_DRAW_OBJ_API void ClearDrawObj(HSV::PointDraw** pObj);

	HS_DRAW_OBJ_API void TransHSRoiObj2DrawObj(const HSRoiBase* srcObj, DrawObjBase** pDraw);

	HS_DRAW_OBJ_API void TransDrawObj2HSRoiObj(const DrawObjBase* srcObj, HSRoiBase** pRoi);

	HS_DRAW_OBJ_API HSRoiBase* TransDrawObj2HSRoiObjPtr(const DrawObjBase* srcObj);

	HS_DRAW_OBJ_API void GenNewRoiObj(const DrawObjBase* srcObj, const HSV::Point2f& pos, HSRoiBase** pRoi);

	HS_DRAW_OBJ_API HSV::Rect BoundingRect(const DrawObjBase* pObj);

	HS_DRAW_OBJ_API HSV::Rect BoundingRect(const std::vector<HSV::DrawObjBase*>& pObj1D);

	HS_DRAW_OBJ_API HSV::Rect2 BoundingRect2(const DrawObjBase* pObj);

}

//namespace HSV = HsvShape;

#endif