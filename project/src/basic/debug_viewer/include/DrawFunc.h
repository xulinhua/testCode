/********************************************************************************************************************************
* 功能说明：所有绘制接口函数
* 
* Ver        修改人      变更日期             变更内容
* ───────────────────────────────────────────────────────────────
* V1.0        xlh     09/09/2024, 10:02          初版
* V1.1        xlh     11/05/2024, 10:02          增加图表绘制
*
* Copyright (c) 2015 HOSON. Co., Ltd. All rights reserved.
*┌──────────────────────────────────────────────────────────────┐
*│                                                                                                                            │
*│                                                                                                                            │
*└──────────────────────────────────────────────────────────────┘
-------------------------------------------------------------------------------------------------------------------------------*/

#pragma once
#include <list>
#include <map>
#include <vector>
#include <algorithm>
#include "assert.h"
#include <memory>
#include "GCImage.h"
#include "DbgInfoDefine.h"
#include "opencvLib.h"
#include"HSDrawObj.h"
#include"HSRoi.h"
#include"TableDraw.h"
#include"ChartDraw.h"
#include"ChartDraw_Bar.h"
#include"ChartDraw_Line.h"
#include"ChartDraw_Pie.h"

namespace DebugView
{
	typedef std::list<HSV::DrawObjBase*> LSTSHAPE;
	typedef std::list<HSV::DrawObjBase*> LSTTEXT;
	
	//GCImage转cv::Mat
	cv::Mat GCImage2Mat(const GCImage& img);
	//cv::Mat转GCImage
	GCImage Mat2GCImage(const cv::Mat& img);
	//HVS::ScalarGC转换成cv::Scalar
	cv::Scalar  Color_t(HSV::ScalarGC color);
	//HVS::Pint转换成cv::Point
	cv::Point  Point_t(HSV::Point point);

	//在图像上显示中文字符
	void DispText(const GCImage& img, const char* str, cv::Point pt, cv::Scalar color, int fontSize,
		const char *fn = "Arial", bool bItalic = false, bool bUnderline = false);
	//显示普通文本框
	void DispText(const GCImage& img, HSV::TextDraw text);
	//显示组合文本
	void DispText(const GCImage& img, HSV::TextUnionDraw text);
	//获取图像上某个点的像素值
	int GetGrayValue(const GCImage& img, HSV::Point ptImg, int* R, int* G, int* B);
	//resize图像
	void ResizeImage(const GCImage& img, GCImage& imgresize, int nWidth, int nHeight);
	//灰度图转BGR
	void ColorImage(const GCImage& img, GCImage& imgcolor);
	//裁剪图像
	void ClipImage(const GCImage& img, GCImage& imgclip, int x, int y, int nWidth, int nHeight);
	//灰度图像转换彩色图像
	void ImageGray2BGR(const GCImage& img, GCImage& imgclr);
	//加载图像
	bool ReadImage(GCImage* img, const char* strPath);
	//写入图像
	bool WriteImage(const GCImage& img, const char* strPath);
	//画线
	void DrawLine(GCImage& img, const HSV::LineDraw& scvLine);
	//画十字叉
	void DrawCross(GCImage& img, const HSV::CrossDraw& scvCross);
	//画箭头
	void DrawArrow(GCImage& img, const HSV::ArrowDraw& scvArrow);
	//画虚线
	void DrawDashLine(GCImage& img, HSV::LineDraw& scvLine, bool isPoint = false, int nCount = 20);
	//画虚线框
	void DrawDashRect(GCImage& img, HSV::Rect2Draw& scvRect, bool isPoint = false, int nCount = 20);
	//画矩形
	void DrawRect(GCImage& img, const HSV::RectDraw& scvRect);
	//画旋转矩形
	void DrawRotatedRect(GCImage& img, const HSV::Rect2Draw& scvRect);
	//画圆
	void DrawCircle(GCImage& img, const HSV::CircleDraw& scvCircle);
	//画椭圆
	void DrawEllipse(GCImage& img, const HSV::EllipseDraw& scvEllipse);
	//画多边形
	void DrawPolygon(GCImage& img, const HSV::PolygonDraw& scvPolygon);
	//画轮廓
	void DrawContours(GCImage& img, const HSV::ContoursDraw& scvContours);
	//画点
	void DrawPt(GCImage& img, const HSV::PointDraw& pt);

	//绘制图形
	void DrawBaseObj(GCImage& img, const HSV::DrawObjBase* pObj);

	//绘制表格
	void DrawTable(GCImage &img, CTableDraw *table);

	//绘制图表
	void DrawChart(GCImage &img, CChartDraw *chart, int nPageWidth, int nPageHeight);
	//绘制柱状图
	void DrawBarChart(GCImage &img, CBarChartDraw *chart, int nPageWidth, int nPageHeight);
	//绘制折线图
	void DrawLineChart(GCImage &img, CLineChartDraw *chart, int nPageWidth, int nPageHeight);
	//绘制饼图
	void DrawPieChart(GCImage &img, CPieChartDraw *chart, int nPageWidth, int nPageHeight);

	//图像合并
	int MergeTwoImg(cv::Mat* src1, cv::Mat* src2, cv::Mat* dst);
	int CombineImages(cv::Mat * src1, cv::Mat * src2, cv::Mat * dst);


}