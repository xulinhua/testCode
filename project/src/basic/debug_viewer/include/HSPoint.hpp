/********************************************************************************************************************************
* 功能说明：点类
*  Ver      修改人          变更日期                 变更内容
* ─────────────────────────────────────────────────────────────────────────
*  V1.0.0  chenwang         2024-05-15       将点类放在单独的hpp文件中，方便兼容IA2.0并解耦（避免流程被动包含多余的图像数据结构）
*备注：
-------------------------------------------------------------------------------------------------------------------------------*/
#pragma once
#include<vector>
#ifdef _WIN32
    #ifdef HSV_CUSTRCT_EXPORTS
        #define HSV_POINT_API __declspec(dllexport)
    #else
        #define HSV_POINT_API __declspec(dllimport)
    #endif
#elif __linux__
    #ifdef HSV_CUSTRCT_EXPORTS
        #define HSV_POINT_API __attribute__((visibility("default")))
    #else
        #define HSV_POINT_API
    #endif
#else
    #error "Unsupported platform"
#endif

namespace HSV
{
//#define PI_HS acos(-1.0)
#define PI_HS 3.1415926535897932384626433832795

	// @brief: 用于存储坐标点的类(参考opencv的cv::Point)
	// @param: x：坐标点x轴
	// @param: y：坐标点y轴
	// @birth: created by LJH on 20230703
	template<typename _Tp> class /*HSV_POINT_API*/ Point_
	{
	public:

		//! default constructor
		//Point_() : x(0), y(0) {}
		Point_()
		{
			Init();
		}
		Point_(_Tp _x, _Tp _y) : x(_x), y(_y) {}
		Point_(const Point_& pt) : x(pt.x), y(pt.y) {}
		void Init()
		{
			x = 0;
			y = 0;
		}
		bool operator==(const Point_& s) const
		{
			return(s.x == this->x && s.y == this->y);
		}

		bool operator < (const Point_ pt) const
		{
			if (this->x == pt.x)
			{
				return (this->y < pt.y);
			}
			else
			{
				return (this->x < pt.x);
			}
		}

		Point_& operator = (const Point_& pt)
		{
			if (this != &pt)
			{
				x = pt.x;
				y = pt.y;
			}
			return *this;
		}

		Point_ operator - (const Point_& pt) const
		{
			return Point_(x - pt.x, y - pt.y);
		}

		Point_ operator + (const Point_& pt) const
		{
			return Point_(x + pt.x, y + pt.y);
		}
		//Point_& operator()(_Tp x, _Tp y);
		_Tp x; //!< x coordinate of the point
		_Tp y; //!< y coordinate of the point
	};
	typedef Point_<int> Point2i;
	typedef Point2i Point;
	using Point1D = std::vector<Point>;
	using Point2D = std::vector<Point1D>;

	typedef Point_<float> Point2f;
	using Point2f1D = std::vector<Point2f>;
	using Point2f2D = std::vector<Point2f1D>;

	typedef Point_<double> Point2d;
	using Point2d1D = std::vector<Point2d>;
	using Point2d2D = std::vector<Point2d1D>;

	// @brief: 用于存储三维坐标点的类(扩展自opencv的cv::Point，用于三维空间)
	// @param: x：坐标点x轴
	// @param: y：坐标点y轴
	// @param: z：坐标点z轴
	// @birth: created based on LJH's design on 20231129
	template<typename _Tp> class /*HSV_POINT_API*/ Point3_
	{
	public:

		//! 默认构造函数
		Point3_() : x(0), y(0), z(0) {}
		Point3_(_Tp _x, _Tp _y, _Tp _z) : x(_x), y(_y), z(_z) {}
		Point3_(const Point3_& pt) : x(pt.x), y(pt.y), z(pt.z) {}
		void Init()
		{
			x = 0;
			y = 0;
			z = 0;
		}
		// 运算符重载
		bool operator == (const Point3_& s) const 
		{
			return (x == s.x) && (y == s.y) && (z == s.z);
		}

		bool operator < (const Point3_& pt) const
		{
			return (x < pt.x) || ((x == pt.x) && (y < pt.y)) || ((x == pt.x) && (y == pt.y) && (z < pt.z));
		}

		Point3_& operator = (const Point3_& pt)
		{
			if (this != &pt) 
			{
				x = pt.x;
				y = pt.y;
				z = pt.z;
			}
			return *this;
		}

		Point3_ operator - (const Point3_& pt) const
		{
			return Point3_(x - pt.x, y - pt.y, z - pt.z);
		}

		Point3_ operator + (const Point3_& pt) const
		{
			return Point3_(x + pt.x, y + pt.y, z + pt.z);
		}

		_Tp x; //!< x坐标
		_Tp y; //!< y坐标
		_Tp z; //!< z坐标
	};

	// 为常见的类型定义别名
	typedef Point3_<int> Point3i;
	typedef Point3i Point3;
	using Point3i1D = std::vector<Point3i>;
	using Point3i2D = std::vector<Point3i1D>;

	typedef Point3_<float> Point3f;
	using Point3f1D = std::vector<Point3f>;
	using Point3f2D = std::vector<Point3f1D>;

	typedef Point3_<double> Point3d;
	using Point3d1D = std::vector<Point3d>;
	using Point3d2D = std::vector<Point3d1D>;

	Point2f HSV_POINT_API GetFootPt(Point2f linePt1, Point2f linePt2, Point2f pt);//计算某一点到线段的垂足点坐标

	Point2f HSV_POINT_API GetFootPt(Point2f linePt1, Point2f linePt2, float ptX, float ptY);//计算某一点到线段的垂足点坐标								   

	Point2f HSV_POINT_API GetFootPt(float linePt1X, float linePt1Y, float linePt2X, float linePt2Y, float ptX, float ptY);//计算某一点到线段的垂足点坐标

	Point2f HSV_POINT_API GetFootPt(Point2i linePt1, Point2i linePt2, Point2i pt);//计算某一点到线段的垂足点坐标																				 

	Point2f HSV_POINT_API GetFootPt(Point2i linePt1, Point2i linePt2, int ptX, int ptY);//计算某一点到线段的垂足点坐标									 

	Point2f HSV_POINT_API GetFootPt(int linePt1X, int linePt1Y, int linePt2X, int linePt2Y, int ptX, int ptY);//计算某一点到线段的垂足点坐标

	float HSV_POINT_API Cal2PtDistance(Point2f pt1, Point2f pt2);//计算两点的距离																		

	float HSV_POINT_API  Cal2PtDistance(Point2i pt1, Point2i pt2);//计算两点的距离
																	 
	float HSV_POINT_API CalPt2LineDistance(Point2f linePt1, Point2f linePt2, Point2f pt); //计算某点到线段的垂直距离

	float HSV_POINT_API Cal2VecDegAng(Point2i sourcePt, int startPtX, int startPtY, int endPtX, int endPtY);//计算两个矢量的夹角(带方向) 

	float Cal2VecDegAng(int sourceX, int sourceY, int startPtX, int startPtY, int endPtX, int endPtY);//计算两个矢量的夹角(带方向) 																							   

	float HSV_POINT_API Cal2VecDegAng(Point2f sourcePt, int startPtX, int startPtY, int endPtX, int endPtY);//计算两个矢量的夹角(带方向) 

	float HSV_POINT_API Cal2VecDegAng(Point2f sourcePt, float startPtX, float startPtY, float endPtX, float endPtY);//计算两个矢量的夹角(带方向) 

	float HSV_POINT_API Cal2VecDegAng(float sourceX, float sourceY, int startPtX, int startPtY, int endPtX, int endPtY);//计算两个矢量的夹角(带方向) 	

	float HSV_POINT_API Cal2VecDegAng(float sourceX, float sourceY, float startPtX, float startPtY, float endPtX, float endPtY);//计算两个矢量的夹角(带方向) 	

	float HSV_POINT_API CalPt2Line_CrossProduct(Point2f linePt1, Point2f linePt2, Point2f pt);//计算某点到线段的叉积

	float HSV_POINT_API CalPt2Line_CrossProduct(float linePt1X, float linePt1Y, float linePt2X, float linePt2Y, float ptX, float ptY);//计算某点到线段的叉积(向量积)

}

