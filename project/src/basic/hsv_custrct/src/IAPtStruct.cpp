//#include "stdafx.h"
#include <cmath> 
#include "IAPtStruct.h"
#define MIN_COMPARE_VAL 0.000001
#define PI_HS 3.1415926535897932384626433832795

#if USING_OLD_HS_REG //IA2.0默认不启用旧的HSReg数据结构 @ChenW 08/01/2024, 11:55
/**************************************************Pt2DInt***************************************************/
Pt2DInt::Pt2DInt()
{
	Init();
}

Pt2DInt::Pt2DInt(const Pt2DInt& pt)
{
	SetPt(pt.x, pt.y);
}

Pt2DInt::Pt2DInt(int tmpX, int tmpY)
{
	SetPt(tmpX, tmpY);
}

Pt2DInt::Pt2DInt(float tmpX, float tmpY)
{
	SetPt(tmpX, tmpY);
}

Pt2DInt::Pt2DInt(double tmpX, double tmpY)
{
	SetPt((float)tmpX, (float)tmpY);
}

Pt2DInt& Pt2DInt::operator = (const Pt2DInt& pt)
{
	x = pt.x;
	y = pt.y;
	return *this;
}

void Pt2DInt::Init()
{
	x = 0;
	y = 0;
}

void Pt2DInt::SetPt(int tmpX, int tmpY)
{
	x = tmpX;
	y = tmpY;
}

void Pt2DInt::SetPt(float tmpX, float tmpY)
{
	x = int((tmpX > 0.0) ? floor(tmpX + 0.5) : ceil(tmpX - 0.5));
	y = int((tmpY > 0.0) ? floor(tmpY + 0.5) : ceil(tmpY - 0.5));
}

void Pt2DInt::SetPt(double tmpX, double tmpY)
{
	SetPt((float)tmpX, (float)tmpY);
}

Pt2DInt::~Pt2DInt()
{
}

bool Pt2DInt::operator == (const Pt2DInt& pt) const
{
	bool bSame = (x == pt.x && y == pt.y) ? true : false;
	return bSame;
}

bool Pt2DInt::operator != (const Pt2DInt& pt) const
{
	bool bDiffer = (x != pt.x || y != pt.y) ? true : false;
	return bDiffer;
}

bool Pt2DInt::operator < (const Pt2DInt& pt) const
{
	bool ret = (x < pt.x) ? true : false;
	return ret;
}

void Pt2DInt::operator += (const Pt2DInt& pt)
{ 
	x += pt.x; 
	y += pt.y; 
}

void Pt2DInt::operator -= (const Pt2DInt& pt)
{ 
	x -= pt.x; 
	y -= pt.y; 
}
/*-------------------------------------------------Pt2DInt--------------------------------------------------*/

/**************************************************Pt2DFloat***************************************************/
Pt2DFloat::Pt2DFloat()
{
	Init();
}

Pt2DFloat::Pt2DFloat(const Pt2DFloat& pt)
{
	x = pt.x;
	y = pt.y;
}

Pt2DFloat::Pt2DFloat(int tmpX, int tmpY)
{
	x = (float)tmpX;
	y = (float)tmpY;
}

Pt2DFloat::Pt2DFloat(float tmpX, float tmpY)
{
	x = tmpX;
	y = tmpY;
}

Pt2DFloat::Pt2DFloat(double tmpX, double tmpY)
{
	x = (float)tmpX;
	y = (float)tmpY;
}

Pt2DFloat& Pt2DFloat::operator = (const Pt2DFloat& pt)
{
	x = pt.x;
	y = pt.y;
	return *this;
}

void Pt2DFloat::Init()
{
	x = 0.0F;
	y = 0.0F;
}

void Pt2DFloat::SetPt(int tmpX, int tmpY)
{
	x = (float)tmpX;
	y = (float)tmpY;
}

void Pt2DFloat::SetPt(float tmpX, float tmpY)
{
	x = tmpX;
	y = tmpY;
}

void Pt2DFloat::SetPt(double tmpX, double tmpY)
{
	x = (float)tmpX;
	y = (float)tmpY;
}

Pt2DFloat::~Pt2DFloat()
{
}

bool Pt2DFloat::operator == (const Pt2DFloat& pt) const
{
	bool bSame = (fabs(x - pt.x) < MIN_COMPARE_VAL && fabs(y - pt.y) < MIN_COMPARE_VAL) ? true : false;
	return bSame;
}

bool Pt2DFloat::operator != (const Pt2DFloat& pt) const
{
	bool bDiffer = (fabs(x - pt.x) > MIN_COMPARE_VAL || fabs(y - pt.y) > MIN_COMPARE_VAL) ? true : false;
	return bDiffer;
}

bool Pt2DFloat::operator < (const Pt2DFloat& pt) const
{
	bool ret = (x < pt.x + MIN_COMPARE_VAL) ? true : false;
	return ret;
}

void Pt2DFloat::operator += (const Pt2DFloat& pt)
{
	x += pt.x;
	y += pt.y;
}

void Pt2DFloat::operator -= (const Pt2DFloat& pt)
{
	x -= pt.x;
	y -= pt.y;
}

/*-------------------------------------------------Pt2DFloat--------------------------------------------------*/

#endif //IA2.0默认不启用旧的HSReg数据结构 @ChenW 08/01/2024, 11:55

#if USING_OLD_IAPOINT_STRUCT == 0//使用原来旧的IAPoint数据结构
	/**************************************************IAPoint***************************************************/
	IAPoint::IAPoint()
	{
		Init();
	}

	IAPoint::IAPoint(const IAPoint& pt)
	{
		x = pt.x;
		y = pt.y;
		z = pt.z;
	}

	IAPoint::IAPoint(int tmpX, int tmpY, int tmpZ)
	{
		x = (double)tmpX;
		y = (double)tmpY;
		z = (double)tmpZ;
	}

	IAPoint::IAPoint(float tmpX, float tmpY, float tmpZ)
	{
		x = (double)tmpX;
		y = (double)tmpY;
		z = (double)tmpZ;
	}

	IAPoint::IAPoint(double tmpX, double tmpY, double tmpZ)
	{
		x = tmpX;
		y = tmpY;
		z = tmpZ;
	}

	IAPoint& IAPoint::operator = (const IAPoint& pt)
	{
		x = pt.x;
		y = pt.y;
		z = pt.z;
		return *this;
	}

	IAPoint& IAPoint::operator = (int val)
	{
		return IAPoint::operator = ((double)val);
	}

	IAPoint& IAPoint::operator = (float val)
	{
		return IAPoint::operator = ((double)val);
	}

	IAPoint& IAPoint::operator = (double val)
	{
		x = val;
		y = val;
		z = val;
		return *this;
	}

	void IAPoint::Init()
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}

	IAPoint::~IAPoint()
	{
	}

	bool IAPoint::operator == (const IAPoint& pt) const
	{
		bool bSame = (fabs(x - pt.x) < MIN_COMPARE_VAL && fabs(y - pt.y) < MIN_COMPARE_VAL && fabs(z - pt.z) < MIN_COMPARE_VAL) ? true : false;
		return bSame;
	}

	bool IAPoint::operator != (const IAPoint& pt) const
	{
		bool bDiffer = (fabs(x - pt.x) > MIN_COMPARE_VAL || fabs(y - pt.y) > MIN_COMPARE_VAL || fabs(z - pt.z) > MIN_COMPARE_VAL) ? true : false;
		return bDiffer;
	}

	bool IAPoint::operator < (const IAPoint& pt) const
	{
		if (1)
		{
			//比较 x 坐标
			if (x < pt.x - MIN_COMPARE_VAL)
			{
				return true;
			}
			else if (x > pt.x + MIN_COMPARE_VAL)
			{
				return false;
			}
			else
			{
				// x在MIN_COMPARE_VAL范围内，继续比较y
				if (y < pt.y - MIN_COMPARE_VAL)
				{
					return true;
				}
				else if (y > pt.y + MIN_COMPARE_VAL)
				{
					return false;
				}
				else
				{
					// x和y均在MIN_COMPARE_VAL范围内，退回到精确比较以打破平局
					// 即使差异极小，也强制生成严格顺序
					if (x != pt.x)
					{
						return x < pt.x; // 优先按x的精确值排序
					}
					else 
					{
						return y < pt.y; // x相等时，按y的精确值排序
					}
				}
			}
		}
		else
			return ((x < pt.x + MIN_COMPARE_VAL) || (fabs(x - pt.x) < MIN_COMPARE_VAL && y < pt.y + MIN_COMPARE_VAL));
		//return ((x < pt.x) || (x == pt.x && y < pt.y));
	}

	void IAPoint::operator += (const IAPoint& pt)
	{
		x += pt.x;
		y += pt.y;
		z += pt.z;
	}

	void IAPoint::operator += (int val)
	{
		operator += ((double)val);
	}

	void IAPoint::operator += (float val)
	{
		operator += ((double)val);
	}

	void IAPoint::operator += (double val)
	{
		x += val;
		y += val;
		z += val;
	}

	void IAPoint::operator -= (const IAPoint& pt)
	{
		x -= pt.x;
		y -= pt.y;
		z -= pt.z;
	}

	void IAPoint::operator -= (int val)
	{
		operator -= ((double)val);
	}

	void IAPoint::operator -= (float val)
	{
		operator -= ((double)val);
	}

	void IAPoint::operator -= (double val)
	{
		x -= val;
		y -= val;
		z -= val;
	}

	/*-------------------------------------------------IAPoint--------------------------------------------------*/
#endif

