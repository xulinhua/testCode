//#include"stdafx.h"
#ifdef _WIN32
#include <windows.h>
#endif 
#include <ctime>
#include"DbgInfoDefine.h"

DebugView::TextDrawPara::TextDrawPara()
{
	strFont = "Arial";
	nTextSize = 30;
	nRgb[0] = 200;
	nRgb[1] = 10;
	nRgb[2] = 15;
}

DebugView::TextDrawPara::TextDrawPara(int size, int r, int g, int b, std::string font)
{
	strFont = font;
	nTextSize = size;
	nRgb[0] = r;
	nRgb[1] = g;
	nRgb[2] = b;

}

DebugView::ObjDrawPara::ObjDrawPara()
{
	nThickness = 1;
	nRgb[0] = 200;
	nRgb[1] = 10;
	nRgb[2] = 15;
}

DebugView::ObjDrawPara::ObjDrawPara(int thickness, int r, int g, int b)
{
	nThickness = thickness;
	nRgb[0] = r;
	nRgb[1] = g;
	nRgb[2] = b;
}

DebugView::ChartDrawPara::ChartDrawPara()
{
	dspHorAxisLab = TextDrawPara(5);
	dspVerAxisLab = TextDrawPara(5);
	dspHorStepLab = TextDrawPara(5);
	dspVerStepLab = TextDrawPara(5);
	dspTitle = TextDrawPara(10);
	dspNote = TextDrawPara(5);

	dspHorAxisLine = ObjDrawPara(2);
	dspVerAxisLine = ObjDrawPara(2);
	dspHorStepLine = ObjDrawPara(1);
	dspVerStepLine = ObjDrawPara(1);
}
