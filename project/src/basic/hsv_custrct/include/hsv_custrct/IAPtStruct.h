/********************************************************************************************************************************
* 功能说明：IA3.0自定义点的数据结构
*   修改人          变更日期                 变更内容
* ─────────────────────────────────────────────────────────────────────────
*  chenwang     2021.06.04 20:35      新增 Pt2DInt、Pt2DFloat、IAPoint导出类

*备注：
-------------------------------------------------------------------------------------------------------------------------------*/
#pragma once
#include <vector>
#define USING_OLD_IAPOINT_STRUCT 0//使用原来旧的IAPoint数据结构

#define USING_OLD_HS_REG 0//是否启用旧的HSReg数据结构，IA2.0中默认不启用 @ChenW 08/01/2024, 11:55

#ifdef _WIN32
    #ifdef HSV_CUSTRCT_EXPORTS
        #define HSV_CUSTRCT_API __declspec(dllexport)
    #else
        #define HSV_CUSTRCT_API __declspec(dllimport)
    #endif
#elif __linux__
    #ifdef HSV_CUSTRCT_EXPORTS
        #define HSV_CUSTRCT_API __attribute__((visibility("default")))
    #else
        #define HSV_CUSTRCT_API
    #endif
#else
    #error "Unsupported platform"
#endif

#if USING_OLD_HS_REG //IA2.0默认不启用旧的HSReg数据结构 @ChenW 08/01/2024, 11:55
class HSV_CUSTRCT_API Pt2DInt
{
public:
	Pt2DInt();
	~Pt2DInt();
	Pt2DInt(const Pt2DInt& pt);
	Pt2DInt(int tmpX, int tmpY);
	Pt2DInt(float tmpX, float tmpY);
	Pt2DInt(double tmpX, double tmpY);
	Pt2DInt& operator = (const Pt2DInt& pt);
	void Init();
	void SetPt(int x, int y);
	void SetPt(float tmpX, float tmpY);
	void SetPt(double tmpX, double tmpY);
	bool operator == (const Pt2DInt& pt) const;
	bool operator != (const Pt2DInt& pt) const;
	bool operator < (const Pt2DInt& pt) const;
	void operator += (const Pt2DInt& pt);
	void operator -= (const Pt2DInt& pt);
	int x;
	int y;
};
using Pt2DInt1D = std::vector<Pt2DInt>;
using Pt2DInt2D = std::vector<Pt2DInt1D>;
class HSV_CUSTRCT_API Pt2DFloat
{
public:
	Pt2DFloat();
	~Pt2DFloat();
	Pt2DFloat(const Pt2DFloat& pt);
	Pt2DFloat(int tmpX, int tmpY);
	Pt2DFloat(float tmpX, float tmpY);
	Pt2DFloat(double tmpX, double tmpY);
	Pt2DFloat& operator = (const Pt2DFloat& pt);
	void Init();
	void SetPt(int x, int y);
	void SetPt(float tmpX, float tmpY);
	void SetPt(double tmpX, double tmpY);
	bool operator == (const Pt2DFloat& pt) const;
	bool operator != (const Pt2DFloat& pt) const;
	bool operator < (const Pt2DFloat& pt) const;
	void operator += (const Pt2DFloat& pt);
	void operator -= (const Pt2DFloat& pt);
	float x;
	float y;
};
using Pt2DFloat1D = std::vector<Pt2DFloat>;

#endif // USING_OLD_HS_REG

#if USING_OLD_IAPOINT_STRUCT//使用原来旧的IAPoint数据结构
	//3d点
	struct IAPoint
	{
		double x;
		double y;
		double z;
		IAPoint(double _x = 0, double _y = 0, double _z = 0) :
			x(_x), y(_y), z(_z)
		{
		} //constructor 

		BOOL operator==(IAPoint point)
		{
			BOOL ret;
			(x == point.x&&y == point.y&&z == point.z) ? ret = TRUE : ret = FALSE;
			return ret;
		}

		BOOL operator!=(IAPoint point)
		{
			BOOL ret;
			(x != point.x&&y != point.y&&z != point.z) ? ret = FALSE : ret = TRUE;
			return ret;
		}
		bool operator <(const IAPoint& point) const
		{
			/*bool ret;
			(x < point.x) ? ret = true : ret = false;
			return ret;*/
			return ((x < point.x) || (x == point.x && y < point.y));
		}
		void operator+=(IAPoint point) { x += point.x; y += point.y; z += point.z; }
		void operator-=(IAPoint point) { x -= point.x; y -= point.y; z -= point.z; }
	};
	typedef std::vector<IAPoint> IAPoint1D;
	typedef std::vector<IAPoint1D> IAPoint2D;
#else
	class HSV_CUSTRCT_API IAPoint
	{
	public:
		IAPoint();
		~IAPoint();
		IAPoint(const IAPoint& pt);
		IAPoint(int tmpX, int tmpY, int tmpZ = 0);
		IAPoint(float tmpX, float tmpY, float tmpZ = 0);
		IAPoint(double tmpX, double tmpY, double tmpZ = 0);
		IAPoint& operator = (const IAPoint& pt);
		IAPoint& operator = (int val);
		IAPoint& operator = (float val);
		IAPoint& operator = (double val);
		void Init();
		bool operator == (const IAPoint& pt) const;
		bool operator != (const IAPoint& pt) const;
		bool operator < (const IAPoint& pt) const;
		void operator += (const IAPoint& pt);
		void operator += (int val);
		void operator += (float val);
		void operator += (double val);
		void operator -= (const IAPoint& pt);
		void operator -= (int val);
		void operator -= (float val);
		void operator -= (double val);
		double x;
		double y;
		double z;
	};
	typedef std::vector<IAPoint> IAPoint1D;
	typedef std::vector<IAPoint1D> IAPoint2D;
#endif // USING_OLD_IAPOINT_STRUCT


