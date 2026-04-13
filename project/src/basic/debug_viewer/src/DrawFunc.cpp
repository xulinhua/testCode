//#include"stdafx.h"
#include "DrawFunc.h"
#include"DispComm.h"

#include "ChineseTextSupport.h"
#define  ToScalar Color_t	
#define  ToPt Point_t
#define  ToImage GCImage2Mat
#define  ToGCImage Mat2GCImage
#define  ToCVRGB(clr) (cv::Scalar(CV_RGB(GetRValue(clr), GetGValue(clr),GetBValue(clr))))


namespace DebugView
{
	using namespace HSV;
	using namespace std;
	
	cv::Mat  GCImage2Mat(const GCImage& img)
	{
		return cv::Mat(img.GetHeight(), img.GetWidth(), CV_MAKETYPE(CV_8U, img.GetChannel()), img.GetData());
	}

	GCImage  Mat2GCImage(const cv::Mat& img)
	{
		return GCImage(img.rows, img.cols, img.channels(), img.data);
	}

	cv::Scalar  Color_t(HSV::ScalarGC color)
	{
		return cv::Scalar(color.val[0], color.val[1], color.val[2]);
	}

	cv::Point  Point_t(HSV::Point point)
	{
		return cv::Point(point.x, point.y);
	}
#if 0
	void  DispText(const GCImage& scr, const char* str, opcv::Point pt, opcv::Scalar color, int fontSize,
		const char *fn, bool bItalic, bool bUnderline)
	{
		cv::Mat img = ToImage(scr);
		pt.y -= fontSize - 5;
		CV_Assert(img.data != 0 && (img.channels() == 1 || img.channels() == 3));
		int x, y, r, b;
		if (pt.x > img.cols || pt.y > img.rows) return;
		x = pt.x < 0 ? -pt.x : 0;
		y = pt.y < 0 ? -pt.y : 0;
		LOGFONTA lf;
		lf.lfHeight = -fontSize;
		lf.lfWidth = 0;
		lf.lfEscapement = 0;
		lf.lfOrientation = 0;
		lf.lfWeight = 5;
		lf.lfItalic = bItalic;//б��
		lf.lfUnderline = bUnderline; //�»���
		lf.lfStrikeOut = 0;
		lf.lfCharSet = DEFAULT_CHARSET;
		lf.lfOutPrecision = 0;
		lf.lfClipPrecision = 0;
		lf.lfQuality = PROOF_QUALITY;
		lf.lfPitchAndFamily = 0;
		strcpy_s(lf.lfFaceName, fn);

		HFONT hf = CreateFontIndirectA(&lf);
		HDC hDC = CreateCompatibleDC(0);
		HFONT hOldFont = (HFONT)SelectObject(hDC, hf);
		int strBaseW = 0, strBaseH = 0;
		int singleRow = 0;
		char buf[1 << 12];
		strcpy_s(buf, str);
		char *bufT[1 << 12];  // ������ڷָ��ַ�����ʣ����ַ������ܻᳬ����
							  //��������
		{
			int nnh = 0;
			int cw, ch;

			const char* ln = strtok_s(buf, "\n", bufT);
			while (ln != 0)
			{
				GetStringSize(hDC, ln, &cw, &ch);
				strBaseW = max(strBaseW, cw);
				strBaseH = max(strBaseH, ch);

				ln = strtok_s(0, "\n", bufT);
				nnh++;
			}
			singleRow = strBaseH;
			strBaseH *= nnh;
		}

		if (pt.x + strBaseW < 0 || pt.y + strBaseH < 0)
		{
			SelectObject(hDC, hOldFont);
			DeleteObject(hf);
			DeleteObject(hDC);
			return;
		}

		r = pt.x + strBaseW > img.cols ? img.cols - pt.x - 1 : strBaseW - 1;
		b = pt.y + strBaseH > img.rows ? img.rows - pt.y - 1 : strBaseH - 1;
		pt.x = pt.x < 0 ? 0 : pt.x;
		pt.y = pt.y < 0 ? 0 : pt.y;

		BITMAPINFO bmp = { 0 };
		BITMAPINFOHEADER& bih = bmp.bmiHeader;
		int strDrawLineStep = strBaseW * 3 % 4 == 0 ? strBaseW * 3 : (strBaseW * 3 + 4 - ((strBaseW * 3) % 4));

		bih.biSize = sizeof(BITMAPINFOHEADER);
		bih.biWidth = strBaseW;
		bih.biHeight = strBaseH;
		bih.biPlanes = 1;
		bih.biBitCount = 24;
		bih.biCompression = BI_RGB;
		bih.biSizeImage = strBaseH * strDrawLineStep;
		bih.biClrUsed = 0;
		bih.biClrImportant = 0;

		void* pDibData = 0;
		HBITMAP hBmp = CreateDIBSection(hDC, &bmp, DIB_RGB_COLORS, &pDibData, 0, 0);

		CV_Assert(pDibData != 0);
		HBITMAP hOldBmp = (HBITMAP)SelectObject(hDC, hBmp);

		SetTextColor(hDC, RGB(255, 255, 255));
		SetBkColor(hDC, 0);

		strcpy_s(buf, str);
		const char* ln = strtok_s(buf, "\n", bufT);
		int outTextY = 0;
		while (ln != 0)
		{
			TextOutA(hDC, 0, outTextY, ln, (int)strlen(ln));
			outTextY += singleRow;
			ln = strtok_s(0, "\n", bufT);
		}
		uchar* imgData = (uchar*)img.data;
		int imgStep = img.step /sizeof(imgData[0]);
		unsigned char* pImg = (unsigned char*)img.data + pt.x * img.channels() + pt.y * imgStep;
		unsigned char* pStr = (unsigned char*)pDibData + x * 3;
		for (int tty = y; tty <= b; ++tty)
		{
			unsigned char* subImg = pImg + (tty - y) * imgStep;
			unsigned char* subStr = pStr + (strBaseH - tty - 1) * strDrawLineStep;
			for (int ttx = x; ttx <= r; ++ttx)
			{
				for (int n = 0; n < img.channels(); ++n)
				{
					double vtxt = subStr[n] / 255.0;
					double dVal = vtxt * color.val[n] + (1 - vtxt) * subImg[n];
					int cvv = (int)((dVal > 0.0) ? floor(dVal + 0.5) : ceil(dVal - 0.5));
					subImg[n] = cvv > 255 ? 255 : (cvv < 0 ? 0 : cvv);
				}

				subStr += 3;
				subImg += img.channels();
			}
		}

		SelectObject(hDC, hOldBmp);
		SelectObject(hDC, hOldFont);
		DeleteObject(hf);
		DeleteObject(hBmp);
		DeleteDC(hDC);
	}

	void  DispText(const GCImage & scr, HSV::TextDraw text)
	{
		if (text.strTxt_=="")
		{
			return;
		}
		cv::Mat img = ToImage(scr);
		const char* str = text.strTxt_.c_str();
		opcv::Point pt = opcv::Point(text.ltBtmPt_.x, text.ltBtmPt_.y);
		opcv::Scalar color = cv::Scalar(text.color_.val[0], text.color_.val[1], text.color_.val[2]);
		int fontSize = text.nFontSize_;
		const char *fn = text.fn_;
		bool bItalic = text.bItalic_;
		bool bUnderline = text.bUnderline_;

		pt.y -= fontSize - 5;
		CV_Assert(img.data != 0 && (img.channels() == 1 || img.channels() == 3));
		int x, y, r, b;
		if (pt.x > img.cols || pt.y > img.rows) return;
		x = pt.x < 0 ? -pt.x : 0;
		y = pt.y < 0 ? -pt.y : 0;
		LOGFONTA lf;
		lf.lfHeight = -fontSize;
		lf.lfWidth = 0;
		lf.lfEscapement = 0;
		lf.lfOrientation = 0;
		lf.lfWeight = 5;
		lf.lfItalic = bItalic;//б��
		lf.lfUnderline = bUnderline; //�»���
		lf.lfStrikeOut = 0;
		lf.lfCharSet = DEFAULT_CHARSET;
		lf.lfOutPrecision = 0;
		lf.lfClipPrecision = 0;
		lf.lfQuality = PROOF_QUALITY;
		lf.lfPitchAndFamily = 0;
		strcpy_s(lf.lfFaceName, fn);

		HFONT hf = CreateFontIndirectA(&lf);
		HDC hDC = CreateCompatibleDC(0);
		HFONT hOldFont = (HFONT)SelectObject(hDC, hf);
		int strBaseW = 0, strBaseH = 0;
		int singleRow = 0;
		char buf[1 << 12];
		strcpy_s(buf, str);
		char *bufT[1 << 12];  // ������ڷָ��ַ�����ʣ����ַ������ܻᳬ����
							  //��������
		{
			int nnh = 0;
			int cw, ch;

			const char* ln = strtok_s(buf, "\n", bufT);
			while (ln != 0)
			{
				GetStringSize(hDC, ln, &cw, &ch);
				strBaseW = max(strBaseW, cw);
				strBaseH = max(strBaseH, ch);

				ln = strtok_s(0, "\n", bufT);
				nnh++;
			}
			singleRow = strBaseH;
			strBaseH *= nnh;
		}

		if (pt.x + strBaseW < 0 || pt.y + strBaseH < 0)
		{
			SelectObject(hDC, hOldFont);
			DeleteObject(hf);
			DeleteObject(hDC);
			return;
		}

		r = pt.x + strBaseW > img.cols ? img.cols - pt.x - 1 : strBaseW - 1;
		b = pt.y + strBaseH > img.rows ? img.rows - pt.y - 1 : strBaseH - 1;
		pt.x = pt.x < 0 ? 0 : pt.x;
		pt.y = pt.y < 0 ? 0 : pt.y;

		BITMAPINFO bmp = { 0 };
		BITMAPINFOHEADER& bih = bmp.bmiHeader;
		int strDrawLineStep = strBaseW * 3 % 4 == 0 ? strBaseW * 3 : (strBaseW * 3 + 4 - ((strBaseW * 3) % 4));

		bih.biSize = sizeof(BITMAPINFOHEADER);
		bih.biWidth = strBaseW;
		bih.biHeight = strBaseH;
		bih.biPlanes = 1;
		bih.biBitCount = 24;
		bih.biCompression = BI_RGB;
		bih.biSizeImage = strBaseH * strDrawLineStep;
		bih.biClrUsed = 0;
		bih.biClrImportant = 0;

		void* pDibData = 0;
		HBITMAP hBmp = CreateDIBSection(hDC, &bmp, DIB_RGB_COLORS, &pDibData, 0, 0);

		CV_Assert(pDibData != 0);
		HBITMAP hOldBmp = (HBITMAP)SelectObject(hDC, hBmp);

		SetTextColor(hDC, RGB(255, 255, 255));
		SetBkColor(hDC, 0);

		strcpy_s(buf, str);
		const char* ln = strtok_s(buf, "\n", bufT);
		int outTextY = 0;
		while (ln != 0)
		{
			TextOutA(hDC, 0, outTextY, ln, (int)strlen(ln));
			outTextY += singleRow;
			ln = strtok_s(0, "\n", bufT);
		}
		uchar* imgData = (uchar*)img.data;
		int imgStep = img.step / sizeof(imgData[0]);
		unsigned char* pImg = (unsigned char*)img.data + pt.x * img.channels() + pt.y * imgStep;
		unsigned char* pStr = (unsigned char*)pDibData + x * 3;
		for (int tty = y; tty <= b; ++tty)
		{
			unsigned char* subImg = pImg + (tty - y) * imgStep;
			unsigned char* subStr = pStr + (strBaseH - tty - 1) * strDrawLineStep;
			for (int ttx = x; ttx <= r; ++ttx)
			{
				for (int n = 0; n < img.channels(); ++n)
				{
					double vtxt = subStr[n] / 255.0;
					double dVal = vtxt * color.val[n] + (1 - vtxt) * subImg[n];
					int cvv = (int)((dVal > 0.0) ? floor(dVal + 0.5) : ceil(dVal - 0.5));
					subImg[n] = cvv > 255 ? 255 : (cvv < 0 ? 0 : cvv);
				}

				subStr += 3;
				subImg += img.channels();
			}
		}

		SelectObject(hDC, hOldBmp);
		SelectObject(hDC, hOldFont);
		DeleteObject(hf);
		DeleteObject(hBmp);
		DeleteDC(hDC);
	}
#endif
	void DispText(const GCImage& scr, const char* str, cv::Point pt, cv::Scalar color, int fontSize,
		const char* fn, bool bItalic, bool bUnderline) {
		cv::Mat img = ToImage(scr);

		// y꣨ԭʼ뱣һ£
		//pt.y -= fontSize - 5;

		CV_Assert(img.data != nullptr && (img.channels() == 1 || img.channels() == 3));

		// ǷͼΧ
		if (pt.x > img.cols || pt.y > img.rows) {
			return;
		}
		 //Ƿ
		if (1||ChineseTextUtils::ContainsChinese(str)) {
			// ıֱʹFreeType
			// ע⣺ʹÐ汾УֻͨıвŸУԱ䶼Ӱ
			ChineseTextUtils::DrawChineseText(img, str, pt, fontSize, color, 1);
			return;
		}
	}

	void DispText(const GCImage& scr, HSV::TextDraw text) {
		if (text.strTxt_.empty()) {
			return;
		}
		cv::Scalar color = cv::Scalar(text.color_.val[0], text.color_.val[1], text.color_.val[2]);
		cv::Point pt = cv::Point(text.ltBtmPt_.x, text.ltBtmPt_.y);
		DispText(scr, text.strTxt_.c_str(), pt, color,
			text.nFontSize_, text.fn_, text.bItalic_, text.bUnderline_);
	}
	void  DispText(const GCImage & img, HSV::TextUnionDraw text)
	{
		if (text.lab_.strTxt_ != "")
		{
			DispText(img, text.lab_);
		}
		if (text.sbj_.strTxt_ != "")
		{
			DispText(img, text.sbj_);
		}
	}

	int  GetGrayValue(const GCImage & img, HSV::Point ptImg, int * R, int * G, int * B)
	{
		if (img.IsEmpty())
		{
			return 0;
		}

		if (ptImg.x >= 0 && ptImg.x < img.GetWidth() && ptImg.y >= 0 && ptImg.y < img.GetHeight())
		{
			cv::Point pt(ptImg.x, ptImg.y);
			cv::Mat image = ToImage(img);
			if (image.empty())
			{
				return 0;
			}
			if (image.channels() == 3)
			{
				if (B)
				{
					*B = image.at<cv::Vec3b>(pt)[0];
				}
				if (G)
				{
					*G = image.at<cv::Vec3b>(pt)[1];
				}
				if (R)
				{
					*R = image.at<cv::Vec3b>(pt)[2];
				}
			}
			else
			{
				if (B)
				{
					*B = image.at<unsigned char>(pt);
				}
				if (G)
				{
					*G = image.at<unsigned char>(pt);
				}
				if (R)
				{
					*R = image.at<unsigned char>(pt);
				}
			}
			return image.channels();
		}
		else
		{
			return img.GetChannel();
		}
	}

	void  ResizeImage(const GCImage & img, GCImage & imgresize, int nWidth, int nHeight)
	{
		cv::Mat mt = ToImage(imgresize);
		cv::resize(ToImage(img), mt, cv::Size(nWidth, nHeight), 0, 0, cv::INTER_NEAREST);
		imgresize.Create(mt.rows, mt.cols, mt.channels(), mt.data);
	}

	void  ColorImage(const GCImage & img, GCImage & imgcolor)
	{
		cv::Mat mt = ToImage(imgcolor);
		cv::cvtColor(ToImage(img), mt, cv::COLOR_GRAY2BGR);
		imgcolor.Create(mt.rows, mt.cols, mt.channels(), mt.data);
	}

	void  ClipImage(const GCImage & img, GCImage & imgclip, int x, int y, int nWidth, int nHeight)
	{
		cv::Rect rcClip(x, y, nWidth, nHeight);
		cv::Mat mt = ToImage(imgclip);
		mt = cv::Mat(ToImage(img), rcClip);
		imgclip.Create(mt.rows, mt.cols, mt.channels(), mt.data);
	}

	void  ImageGray2BGR(const GCImage & img, GCImage & imgclr)
	{
		cv::Mat mt = ToImage(imgclr);
		cv::cvtColor(ToImage(img), mt, cv::COLOR_GRAY2BGR);
		imgclr.Create(mt.rows, mt.cols, mt.channels(), mt.data);
	}

	bool  ReadImage(GCImage * img, const char * strPath)
	{
		if (NULL == img)
		{
			return false;
		}

		cv::Mat mt = ToImage(*img);
		mt = cv::imread(strPath);
		img->Create(mt.rows, mt.cols, mt.channels(), mt.data);

		return true;
	}

	bool  WriteImage(const GCImage & img, const char * strPath)
	{
		return cv::imwrite(strPath, ToImage(img));
	}

	void  DrawLine(GCImage & img, const HSV::LineDraw & scvLine)
	{
		cv::line(ToImage(img), ToPt(scvLine.pt1_), ToPt(scvLine.pt2_), ToScalar(scvLine.color_), scvLine.thickness_, scvLine.lineType_, scvLine.shift_);

	}

	void  DrawCross(GCImage & img, const HSV::CrossDraw & scvCross)
	{
		cv::Mat src = ToImage(img);
		HSV::LineDraw Lines[2];
		scvCross.lines(Lines);
		cv::line(src, ToPt(Lines[0].pt1_), ToPt(Lines[0].pt2_), ToScalar(Lines[0].color_), Lines[0].thickness_, Lines[0].lineType_, Lines[0].shift_);
		cv::line(src, ToPt(Lines[1].pt1_), ToPt(Lines[1].pt2_), ToScalar(Lines[1].color_), Lines[1].thickness_, Lines[1].lineType_, Lines[1].shift_);
	}

	void  DrawArrow(GCImage & img, const HSV::ArrowDraw & scvArrow)
	{
		cv::arrowedLine(ToImage(img), ToPt(scvArrow.pt1_), ToPt(scvArrow.pt2_), ToScalar(scvArrow.color_), scvArrow.thickness_, scvArrow.lineType_, scvArrow.shift_, scvArrow.tipLen_);

	}

	void  DrawDashLine(GCImage & img, HSV::LineDraw & scvLine, bool isPoint, int nCount)
	{
		cv::Mat src = ToImage(img);
		if (isPoint)
		{
			HSV::CircleDraw *dash_point = new HSV::CircleDraw[nCount + 1];
			scvLine.GetCounterPoint(dash_point, nCount);
			for (int i = 0; i < nCount; i++)
			{
				cv::circle(src, cv::Point(dash_point[i].cnter_.x, dash_point[i].cnter_.y), 1, ToScalar(scvLine.color_), -1, scvLine.lineType_, scvLine.shift_);
			}
			delete[] dash_point;
		}
		else
		{
			HSV::LineDraw *dash_line = new HSV::LineDraw[nCount + 1];
			scvLine.GetLinePoint(dash_line, nCount);
			for (int i = 0; i < nCount; i++)
			{
				//cv::circle(src, cv::Point(dash_point[i].center_x, dash_point[i].center_y), 1, Color_t(DashLine1.color_type), DashLine1.thickness_, DashLine1.lineType);
				cv::line(src, ToPt(dash_line[i].pt1_), ToPt(dash_line[i].pt2_), ToScalar(scvLine.color_), scvLine.thickness_, scvLine.lineType_, scvLine.shift_);
			}
			delete[] dash_line;
		}
	}

	void  DrawDashRect(GCImage & img, HSV::Rect2Draw & scvRect, bool isPoint, int nCount)
	{
		cv::Mat src = ToImage(img);
		if (isPoint)
		{
			HSV::CircleDraw *dash_point = new HSV::CircleDraw[(nCount + 1) * 4];

			scvRect.GetCounterPoint(dash_point, nCount);
			for (int i = 0; i < (nCount + 1) * 4; i++)
			{
				cv::circle(src, cv::Point(dash_point[i].cnter_.x, dash_point[i].cnter_.y), 1, ToScalar(scvRect.color_), -1, scvRect.lineType_, scvRect.shift_);
			}
			delete[] dash_point;
		}
		else
		{


		}
	}

	void  DrawRect(GCImage & img, const HSV::RectDraw & scvRect)
	{
		if (scvRect.isDash_)     //����������ʾ  luojianghong 23-9-4
		{
			HSV::Point pts[4];
			scvRect.points(pts);
			LineDraw line1(pts[0], pts[1]);
			DrawDashLine(img, line1, false, scvRect.nCount_);
			LineDraw line2(pts[1], pts[2]);
			DrawDashLine(img, line2, false, scvRect.nCount_);
			LineDraw line3(pts[2], pts[3]);
			DrawDashLine(img, line3, false, scvRect.nCount_);
			LineDraw line4(pts[3], pts[0]);
			DrawDashLine(img, line4, false, scvRect.nCount_);
		}
		else
		{
			cv::rectangle(ToImage(img), ToPt(scvRect.tl()), ToPt(scvRect.br()), ToScalar(scvRect.color_), scvRect.thickness_, scvRect.lineType_, scvRect.shift_);
		}
	}

	void  DrawRotatedRect(GCImage & img, const HSV::Rect2Draw & scvRect)
	{
		if (scvRect.isDash_)     //����������ʾ  luojianghong 23-9-4
		{
			HSV::Point pts[4];
			scvRect.points(pts);
			LineDraw line1(pts[0], pts[1], scvRect.color_);
			DrawDashLine(img, line1, false, scvRect.nCount_);
			LineDraw line2(pts[1], pts[2], scvRect.color_);
			DrawDashLine(img, line2, false, scvRect.nCount_);
			LineDraw line3(pts[2], pts[3], scvRect.color_);
			DrawDashLine(img, line3, false, scvRect.nCount_);
			LineDraw line4(pts[3], pts[0], scvRect.color_);
			DrawDashLine(img, line4, false, scvRect.nCount_);
		}
		else {
			HSV::Point vertices[4];
			cv::Mat src = ToImage(img);
			scvRect.points(vertices);
			for (int i = 0; i < 4; i++)
			{
				//cv::line(src, ToPt(vertices[i]), ToPt(vertices[(i + 1) % 4]), ToScalar(scvRect.color_), scvRect.thickness_ * IMG_ZOOM(img), scvRect.lineType_, scvRect.shift_);
				cv::line(src, ToPt(vertices[i]), ToPt(vertices[(i + 1) % 4]), ToScalar(scvRect.color_), scvRect.thickness_, scvRect.lineType_, scvRect.shift_);
			}
		}
	}

	void  DrawCircle(GCImage & img, const HSV::CircleDraw & scvCircle)
	{
		cv::circle(ToImage(img), cv::Point(scvCircle.cnter_.x, scvCircle.cnter_.y), scvCircle.radius_, ToScalar(scvCircle.color_), scvCircle.thickness_, scvCircle.lineType_, scvCircle.shift_);

	}

	void  DrawEllipse(GCImage & img, const HSV::EllipseDraw & scvEllipse)
	{
		cv::ellipse(ToImage(img), cv::Point(scvEllipse.cnter_.x, scvEllipse.cnter_.y),
			cv::Size(scvEllipse.longR_, scvEllipse.shortR_), scvEllipse.degAng_, scvEllipse.startDegAng_,
			scvEllipse.endDegAng_, ToScalar(scvEllipse.color_), scvEllipse.thickness_, scvEllipse.lineType_, scvEllipse.shift_);
	}

	void  DrawPolygon(GCImage & img, const HSV::PolygonDraw & scvPolygon)
	{
		cv::Point *ploy_pt = new cv::Point[scvPolygon.m_N];
		for (int i = 0; i < scvPolygon.m_N; i++)
		{
			ploy_pt[i] = ToPt(scvPolygon.points_[i]);
		}

		cv::polylines(ToImage(img), &ploy_pt, &scvPolygon.m_N, 1, true, ToScalar(scvPolygon.color_), scvPolygon.thickness_, scvPolygon.lineType_, scvPolygon.shift_);
		if (scvPolygon.m_N>0)
		{
			delete[]ploy_pt;
		}
	}

	void  DrawContours(GCImage & img, const HSV::ContoursDraw & scvContours)
	{
		using namespace cv;
		cv::Mat cvImag = ToImage(img);
		cv::Scalar color = ToScalar(scvContours.color_);
		static bool bUseSrc = false;
		for (auto& it : scvContours.points_)
		{
			if (it.x < 0 || it.y < 0 || it.x >= img.GetWidth() || it.y >= img.GetHeight())//������Χ���
			{
				continue;
			}
			if (bUseSrc)
			{
				for (int k = 0; k < cvImag.channels(); k++)
				{
					long nPos = it.y*cvImag.step + it.x*cvImag.channels() + k;
					cvImag.data[nPos] = (uchar)color[k];
				}
			}
			else
			{
				if (scvContours.thickness_ == 1)
				{
					ToImage(img).at<cv::Vec3b>(it.y, it.x) = cv::Vec3b(color[0], color[1], color[2]);
				}
				else
				{
					circle(ToImage(img), cv::Point(it.x, it.y), scvContours.thickness_ / 2, color, -1, scvContours.lineType_, scvContours.shift_);
				}
			}
		}
	}

	void  DrawPt(GCImage & img, const HSV::PointDraw & pt)
	{
		if (pt.point_.x < 0 || pt.point_.y < 0)//������Χ���
			return;
		using namespace cv;
		cv::Mat cvImag = ToImage(img);
		cv::Scalar color = ToScalar(pt.color_);
		static bool bUseSrc = false;
		if (bUseSrc)
		{
			for (int k = 0; k < cvImag.channels(); k++)
			{
				long nPos = pt.point_.y*cvImag.step + pt.point_.x*cvImag.channels() + k;
				cvImag.data[nPos] = (uchar)color[k];
			}
		}
		else
		{
			if (pt.thickness_ == 1)
			{
				ToImage(img).at<cv::Vec3b>(pt.point_.y, pt.point_.x) = cv::Vec3b(color[0], color[1], color[2]);
			}
			else
			{
				circle(ToImage(img), cv::Point(pt.point_.x, pt.point_.y), pt.thickness_ / 2, color, -1, pt.lineType_, pt.shift_);
			}
		}
	}

	void  DrawBaseObj(GCImage & img, const HSV::DrawObjBase * pObj)
	{
		switch (pObj->GetDrawType())
		{
		default:
			break;
		case DRAW_RECT:
		{
			const HSV::RectDraw* ptr = dynamic_cast<const HSV::RectDraw*>(pObj);
			DrawRect(img, *ptr);
		}
		break;
		case DRAW_RECT2:
		{
			const HSV::Rect2Draw* ptr = dynamic_cast<const HSV::Rect2Draw*>(pObj);
			DrawRotatedRect(img, *ptr);
		}
		break;
		case DRAW_CIRCLE:
		{
			const HSV::CircleDraw* ptr = dynamic_cast<const HSV::CircleDraw*>(pObj);
			DrawCircle(img, *ptr);
		}
		break;
		case DRAW_ELLIPSE:
		{
			const HSV::EllipseDraw* ptr = dynamic_cast<const HSV::EllipseDraw*>(pObj);
			DrawEllipse(img, *ptr);
		}
		break;
		case DRAW_POLYGON:
		{
			const HSV::PolygonDraw* ptr = dynamic_cast<const HSV::PolygonDraw*>(pObj);
			DrawPolygon(img, *ptr);
		}
		break;
		case DRAW_LINE:
		{
			const HSV::LineDraw* ptr = dynamic_cast<const HSV::LineDraw*>(pObj);
			DrawLine(img, *ptr);
		}
		break;
		case DRAW_CROSS:
		{
			const HSV::CrossDraw* ptr = dynamic_cast<const HSV::CrossDraw*>(pObj);
			DrawCross(img, *ptr);
		}
		break;
		case DRAW_ARROW:
		{
			const HSV::ArrowDraw* ptr = dynamic_cast<const HSV::ArrowDraw*>(pObj);
			DrawArrow(img, *ptr);
		}
		break;
		case DRAW_TEXT:
		{
			const HSV::TextDraw* ptr = dynamic_cast<const HSV::TextDraw*>(pObj);
			DispText(img, *ptr);
		}
		break;
		case DRAW_TEXT_UNION:
		{
			const HSV::TextUnionDraw* ptr = dynamic_cast<const HSV::TextUnionDraw*>(pObj);
			DispText(img, *ptr);
		}
		break;
		case DRAW_TEXT_TUPLE:
		{
			const HSV::TextTupleDraw* ptr = dynamic_cast<const HSV::TextTupleDraw*>(pObj);
			//DispText(img, *ptr);
		}
		break;
		case DRAW_CONTOURS:
		{
			const HSV::ContoursDraw* ptr = dynamic_cast<const HSV::ContoursDraw*>(pObj);
			DrawContours(img, *ptr);
		}
		break;
		case DRAW_POINT:
		{
			const HSV::PointDraw* ptr = dynamic_cast<const HSV::PointDraw*>(pObj);
			DrawPt(img, *ptr);//����
		}
		break;
		}
	}

	void  DrawTable(GCImage & img, CTableDraw * table)
	{
		//�жϱ�������
		if (table == nullptr)return;
		if (!table->CheckDataValidity())return;

		int numRows = 0;
		int numCols = 0;
		switch (table->eDataType)
		{
		case DebugView::DataType::INT:
			numRows = (int)table->nDatas.size();
			numCols = (int)table->nDatas[0].size();
			break;
		case DebugView::DataType::FLOAT:
			numRows = (int)table->fDatas.size();
			numCols = (int)table->fDatas[0].size();
			break;
		case DebugView::DataType::DOUBLE:
			numRows = (int)table->dDatas.size();
			numCols = (int)table->dDatas[0].size();
			break;
		default://�쳣
			return;
			break;
		}
		//�����쳣
		if (numRows == 0 || numCols == 0)
		{
			return;
		}
		//�Ƿ��б���
		bool bHasVerTitle = false;
		bool bHasHorTitle = false;
		int dataRows = numRows;
		int dataCols = numCols;
		if (table->vecHorTitle.size()>0)
		{
			numRows += 1;
			bHasHorTitle = true;
		}
		if (table->vecVerTitle.size()>0)
		{
			numCols += 1;
			bHasVerTitle = true;
		}
		/////////////////////////���Ʊ���/////////////////////////
		int txtWid, txtHgt;//��ȡ�ı�����(������ڽϴ����)
		GetTextSize("Aa1_", table->strDataFont, table->nDataFontSize, txtWid, txtHgt);
		if (bHasHorTitle)
		{
			int w, h;
			GetTextSize("Aa1_", table->strHorTitleFont, table->nHorTitleFontSize, w, h);
			txtWid = max(txtWid, w);
			txtHgt = max(txtHgt, h);
		}
		if (bHasVerTitle)
		{
			int w, h;
			GetTextSize("Aa1_", table->strVerTitleFont, table->nVerTitleFontSize, w, h);
			txtWid = max(txtWid, w);
			txtHgt = max(txtHgt, h);
		}

		int imgWid = img.GetWidth();
		int imgHgt = img.GetHeight();

		int tableWid = imgWid - table->ptTopLeft.x * 2;//�������(�������)
		int itemWid = tableWid / numCols;//��Ԫ�����
		int itemHgt = txtHgt;//��Ԫ��߶�
		int tableHgt = itemHgt*numRows;//����߶�
		tableWid = itemWid*numCols;//����������¼���(int�ͼ���������)
		 //����ߴ�Խ�紦�� ������



		//���Ʊ���ֱ��
		HSV::LineDraw line;
		line.color_ = table->colorTableLine;
		line.thickness_ = table->nTableLineThick;
		line.pt1_.x = (int)table->ptTopLeft.x;
		line.pt2_.x = (int)table->ptTopLeft.x + tableWid;
		for (int i = 0; i <= numRows; i++)
		{
			line.pt1_.y = (int)table->ptTopLeft.y +  i * (itemHgt + 5);
			line.pt2_.y = (int)table->ptTopLeft.y +  i * (itemHgt + 5);
			DrawLine(img, line);
		}
		line.pt1_.y = (int)table->ptTopLeft.y;
		line.pt2_.y = (int)table->ptTopLeft.y + tableHgt + (numRows * 5);
		for (int i = 0; i <= numCols; i++)
		{
			line.pt1_.x = (int)table->ptTopLeft.x + i*itemWid;
			line.pt2_.x = (int)table->ptTopLeft.x + i*itemWid;
			DrawLine(img, line);
		}

		cv::Scalar colorCV;
		// ���ƴ�ֱ����
		int txtOffsetY = -itemHgt * 0.2;  // ԭʼƫ�ƣ�������Ҫ����
		int txtOffsetX = 5;
		colorCV = Color_t(table->colorVerTitleFore);
		for (unsigned int i = 0; i < table->vecVerTitle.size(); i++)
		{
			// ������ӵ�����Y����
			int cellTopY = table->ptTopLeft.y + (i + 1 + (int)bHasHorTitle) * (itemHgt + 5);
			int cellCenterY = cellTopY + itemHgt / 2;

			// OpenCV putText��y�����ǻ���λ�ã���Ҫ����
			// ����ͨ�����ַ����ĵ�ߣ�����߶�Լ70-80%�ڻ����Ϸ���
			int yPos = cellCenterY - table->nVerTitleFontSize * 0.3 - 3;  // ����30%������߶�

			// ����ʹ�ø��򵥵ķ���������λ�� = ���Ӷ��� + 3/4���Ӹ߶�
			// int yPos = cellTopY + itemHgt * 3 / 4;

			DispText(img, table->vecVerTitle[i].data(),
				cv::Point(table->ptTopLeft.x + txtOffsetX, yPos),
				colorCV, table->nVerTitleFontSize, table->strVerTitleFont.data());
		}

		// ����ˮƽ����
		colorCV = Color_t(table->colorHorTitleFore);
		for (unsigned int i = 0; i < table->vecHorTitle.size(); i++)
		{
			int cellTopY = table->ptTopLeft.y + itemHgt + 3;
			int cellCenterY = cellTopY + itemHgt / 2;
			int yPos = cellCenterY - table->nHorTitleFontSize * 0.3;

			DispText(img, table->vecHorTitle[i].data(),
				cv::Point(table->ptTopLeft.x + (i + (int)bHasVerTitle) * itemWid + txtOffsetX, yPos),
				colorCV, table->nHorTitleFontSize, table->strHorTitleFont.data());
		}

		// ��������
		colorCV = Color_t(table->colorDataFore);
		for (int i = 0; i < dataRows; i++)
		{
			for (int j = 0; j < dataCols; j++)
			{
				char chs[100];
				if (table->eDataType == DebugView::DataType::INT)
				{
					snprintf(chs, sizeof(chs), "%d", table->nDatas[i][j]);
				}
				if (table->eDataType == DebugView::DataType::FLOAT)
				{
					snprintf(chs, sizeof(chs), "%.4f", table->fDatas[i][j]);
				}
				if (table->eDataType == DebugView::DataType::DOUBLE)
				{
					snprintf(chs, sizeof(chs), "%.4f", table->dDatas[i][j]);
				}

				//int cellTopY = table->ptTopLeft.y + (i + bHasHorTitle + 1) * itemHgt;
				// ���Բ�ͬ�ı�������1/2��ʼ
				//int yPos = cellTopY + itemHgt / 4;  // �����ڸ����м�

					int cellTopY = table->ptTopLeft.y + (i + 1 + (int)bHasHorTitle) * (itemHgt + 5);
				int cellCenterY = cellTopY + itemHgt / 2;

				// OpenCV putText的y坐标是基线位置，需要上移
				// 基线通常比字符中心点高（字体高度约70-80%在基线上方）
				int yPos = cellCenterY - table->nDataFontSize * 0.3 - 3;  // 上移30%的字体高度
				DispText(img, chs,
					cv::Point(table->ptTopLeft.x + (j + bHasVerTitle) * itemWid + txtOffsetX, yPos),
					colorCV, table->nDataFontSize, table->strDataFont.data());
			}
		}
		//int txtOffsetY = -itemHgt*0.2;
		//int txtOffsetX = 5;
		//colorCV = Color_t(table->colorVerTitleFore);
		//for (unsigned int i = 0; i < table->vecVerTitle.size(); i++)
		//{
		//	DispText(img, table->vecVerTitle[i].data(), cv::Point(table->ptTopLeft.x + txtOffsetX,
		//		table->ptTopLeft.y + (i + 1 + (int)bHasHorTitle)*itemHgt + txtOffsetY), colorCV, table->nVerTitleFontSize, table->strVerTitleFont.data());
		//}
		////����ˮƽ����
		//colorCV = Color_t(table->colorHorTitleFore);
		//for (unsigned int i = 0; i < table->vecHorTitle.size(); i++)
		//{
		//	DispText(img, table->vecHorTitle[i].data(), cv::Point(table->ptTopLeft.x + (i + (int)bHasVerTitle)*itemWid + txtOffsetX,
		//		table->ptTopLeft.y + itemHgt + txtOffsetY), colorCV, table->nHorTitleFontSize, table->strHorTitleFont.data());
		//}
		////��������
		//colorCV = Color_t(table->colorDataFore);
		//for (int i = 0; i < dataRows; i++)
		//{
		//	for (int j = 0; j < dataCols; j++)
		//	{
		//		char chs[100];
		//		if (table->eDataType == DebugView::DataType::INT)
		//		{
		//			sprintf_s(chs, "%d", table->nDatas[i][j]);
		//		}
		//		if (table->eDataType == DebugView::DataType::FLOAT)
		//		{
		//			sprintf_s(chs, "%.4f", table->fDatas[i][j]);
		//		}
		//		if (table->eDataType == DebugView::DataType::DOUBLE)
		//		{
		//			sprintf_s(chs, "%.4f", table->dDatas[i][j]);
		//		}
		//		DispText(img, chs, cv::Point(table->ptTopLeft.x + (j + bHasVerTitle)*itemWid + txtOffsetX,
		//			table->ptTopLeft.y + (i + bHasHorTitle + 1)*itemHgt + txtOffsetY), colorCV, table->nDataFontSize, table->strDataFont.data());
		//	}
		//}
	}

	void  DrawChart(GCImage & img, CChartDraw * chart,int nPageWidth, int nPageHeight)
	{
		if (chart == nullptr)
		{
			return;
		}
		else if (!chart->CheckDataValidity())
		{
			return;
		}
		switch (chart->eChartType)
		{
		case ChartType::BAR_CHART:
			DrawBarChart(img, (CBarChartDraw*)chart, nPageWidth, nPageHeight);
			break;
		case ChartType::LINE_CHART:
			DrawLineChart(img, (CLineChartDraw*)chart,nPageWidth, nPageHeight);
			break;
		case ChartType::PIE_CHART:
			DrawPieChart(img, (CPieChartDraw*)chart, nPageWidth, nPageHeight);
			break;
		}
	}

	void  DrawBarChart(GCImage & img, CBarChartDraw * chart, int nPageWidth, int nPageHeight)
	{
		///////////////////����������///////////////////
		int nChartWid, nChartHgt;
		int dataRows, dataCols;
		chart->GetChartSize(nChartWid, nChartHgt, nPageWidth, nPageHeight);
		chart->GetDataSize(dataRows, dataCols);
		int titleWid, titleHgt, noteWid, noteHgt;
		int horAxisLabWid, horAxisLabHgt, verAxisLabWid, verAxisLabHgt;
		int horStepLabWid, horStepLabHgt, verStepLabWid, verStepLabHgt;
		GetTextSize(chart->strChartTitle, chart->dspChartTitle.strFont, chart->dspChartTitle.nFontSize, titleWid, titleHgt);
		GetTextSize(chart->strChartNote, chart->dspChartNote.strFont, chart->dspChartNote.nFontSize, noteWid, noteHgt);
		GetTextSize(chart->strHorAxisLabel, chart->dspHorAxisLabel.strFont, chart->dspHorAxisLabel.nFontSize, horAxisLabWid, horAxisLabHgt);
		GetTextSize(chart->strVerAxisLabel, chart->dspVerAxisLabel.strFont, chart->dspVerAxisLabel.nFontSize, verAxisLabWid, verAxisLabHgt);
		GetTextSize("1", chart->dspHorStepLabel.strFont, chart->dspHorStepLabel.nFontSize, horStepLabWid, horStepLabHgt);
		GetTextSize("1", chart->dspVerStepLabel.strFont, chart->dspVerStepLabel.nFontSize, verStepLabWid, verStepLabHgt);
		//���д���������X����
		chart->ptTopLeft.x = (nPageWidth - nChartWid) / 2;

		///////////////////����ǳɫ���///////////////////
		HSV::RectDraw recChart;
		recChart.ltTopPt_.x = (int)chart->ptTopLeft.x;
		recChart.ltTopPt_.y = (int)chart->ptTopLeft.y;
		recChart.w_ = nChartWid;
		recChart.h_ = nChartHgt;
		recChart.color_ = HSV::ScalarGC(150, 150, 150);
		DrawRect(img, recChart);

		///////////////////���Ʊ���///////////////////
		HSV::TextDraw title;
		title.color_ = chart->dspChartTitle.clrTxtFore;
		title.nFontSize_ = chart->dspChartTitle.nFontSize;
		title.fn_ = chart->dspChartTitle.strFont.data();
		title.strTxt_ = chart->strChartTitle;
		title.ltBtmPt_.x = (int)recChart.ltTopPt_.x+(recChart.w_-titleWid)/2;
		title.ltBtmPt_.y = (int)chart->ptTopLeft.y + titleHgt;
		DispText(img, title);
		
		///////////////////����ͼ��///////////////////
		//����
		HSV::LineDraw horAxisLine;
		horAxisLine.pt1_.x = recChart.ltTopPt_.x + chart->nHorStepPix + verStepLabWid*6;
		horAxisLine.pt1_.y = recChart.ltTopPt_.y + recChart.h_ - noteHgt - horStepLabHgt;
		horAxisLine.pt2_.x = recChart.ltTopPt_.x + recChart.w_ - chart->nHorStepPix+ verStepLabWid*6- chart->dAxisExtend;
		horAxisLine.pt2_.y = recChart.ltTopPt_.y + recChart.h_ - noteHgt - horStepLabHgt;
		horAxisLine.color_ = chart->dspHorAxisLine.clrObj;
		horAxisLine.thickness_ = chart->dspHorAxisLine.thickObj;
		DrawLine(img, horAxisLine);
		HSV::ArrowDraw horArr;
		int nArrLen = max(nChartWid / 25,10);
		horArr.pt2_ = horAxisLine.pt2_;
		horArr.pt1_.x = horAxisLine.pt2_.x - nArrLen;
		horArr.pt1_.y = horAxisLine.pt2_.y;
		horArr.tipLen_ = 0.3;
		horArr.degAng_ = 25;
		horArr.thickness_ = 1;
		horArr.color_ = horAxisLine.color_;
		DrawArrow(img, horArr);
		//����
		HSV::LineDraw verAxisLine;
		verAxisLine.pt1_.x = recChart.ltTopPt_.x + chart->nHorStepPix + verStepLabWid*6;
		verAxisLine.pt1_.y = recChart.ltTopPt_.y + recChart.h_ - noteHgt - horStepLabHgt;
		verAxisLine.pt2_.x = recChart.ltTopPt_.x + chart->nHorStepPix + verStepLabWid*6;
		verAxisLine.pt2_.y = recChart.ltTopPt_.y + titleHgt - horStepLabHgt;
		verAxisLine.color_ = chart->dspVerAxisLine.clrObj;
		verAxisLine.thickness_ = chart->dspVerAxisLine.thickObj;
		DrawLine(img, verAxisLine);
		HSV::ArrowDraw verArr;
		verArr.pt2_ = verAxisLine.pt2_;
		verArr.pt1_.x = verAxisLine.pt2_.x;
		verArr.pt1_.y = verAxisLine.pt2_.y + nArrLen;
		verArr.tipLen_ = 0.3;
		verArr.degAng_ = 25;
		verArr.thickness_ = 1;
		verArr.color_ = verAxisLine.color_;
		DrawArrow(img, verArr);
		//ֵ����
		double maxValue, minValue;
		switch (chart->eDataType)
		{
		case DataType::INT:
			maxValue = (double)*max_element(chart->nData.begin(), chart->nData.end());
			minValue = (double)*min_element(chart->nData.begin(), chart->nData.end());
			break;
		case DataType::FLOAT:
			maxValue = (double)*max_element(chart->fData.begin(), chart->fData.end());
			minValue = (double)*min_element(chart->fData.begin(), chart->fData.end());
			break;
		case DataType::DOUBLE:
			maxValue = (double)*max_element(chart->dData.begin(), chart->dData.end());
			minValue = (double)*min_element(chart->dData.begin(), chart->dData.end());
			break;
		}
		double verUnit = 1.0;
		double rangeValue = chart->bCoorOrgZero ? maxValue : (maxValue - minValue);
		rangeValue = max(0.00000001, rangeValue);//����Ϊ0
		verUnit = (recChart.h_ - titleHgt - noteHgt - verAxisLabHgt) / rangeValue;
		HSV::Point2i ptBtm;
		int valHgt;
		HSV::RectDraw rectValue;
		rectValue.color_ = chart->dspData.clrTxtFore;
		rectValue.thickness_ = -1;//������
		for (int i = 0; i < dataCols; i++)
		{
			ptBtm.y = horAxisLine.pt1_.y;
			ptBtm.x = horAxisLine.pt1_.x + chart->nHorStepPix*i;
			switch (chart->eDataType)
			{
			case DataType::INT:
				valHgt = chart->bCoorOrgZero ? (chart->nData[i] * verUnit)
					: ((chart->nData[i] - minValue)*verUnit);
				break;
			case DataType::FLOAT:
				valHgt = chart->bCoorOrgZero ? (chart->fData[i] * verUnit)
					: ((chart->fData[i] - minValue)*verUnit);
				break;
			case DataType::DOUBLE:
				valHgt = chart->bCoorOrgZero ? (chart->dData[i] * verUnit)
					: ((chart->dData[i] - minValue)*verUnit);
				break;
			}

			rectValue.ltTopPt_.x = horAxisLine.pt1_.x + chart->nHorStepPix*i;
			rectValue.ltTopPt_.y = horAxisLine.pt1_.y - valHgt;
			rectValue.w_ = chart->nHorStepPix;
			rectValue.h_ = valHgt;
			DrawRect(img, rectValue);
		}
		//�����ǩ
		HSV::TextDraw textHorAxixLabel;
		textHorAxixLabel.ltBtmPt_.x = horArr.pt2_.x;
		textHorAxixLabel.ltBtmPt_.y = horArr.pt2_.y+ horAxisLabHgt;
		textHorAxixLabel.strTxt_ = chart->strHorAxisLabel;
		textHorAxixLabel.fn_ = chart->dspHorAxisLabel.strFont.data();
		textHorAxixLabel.nFontSize_ = chart->dspHorAxisLabel.nFontSize;
		textHorAxixLabel.color_ = chart->dspHorAxisLabel.clrTxtFore;
		DispText(img,textHorAxixLabel);
		//�����ǩ
		HSV::TextDraw textVerAxixLabel;
		textVerAxixLabel.ltBtmPt_.x = verArr.pt2_.x;
		textVerAxixLabel.ltBtmPt_.y = verArr.pt2_.y - verAxisLabHgt;
		textVerAxixLabel.strTxt_ = chart->strVerAxisLabel;
		textVerAxixLabel.fn_ = chart->dspVerAxisLabel.strFont.data();
		textVerAxixLabel.nFontSize_ = chart->dspVerAxisLabel.nFontSize;
		textVerAxixLabel.color_ = chart->dspVerAxisLabel.clrTxtFore;
		DispText(img, textVerAxixLabel);
		//���ᵥ����ǩ
		HSV::TextDraw textHorStep;
		int horInterval = floor(dataCols / chart->nHorAxisLabNum);
		for (int i = 0; i <= chart->nHorAxisLabNum; i++)
		{
			textHorStep.ltBtmPt_.x = horAxisLine.pt1_.x + chart->nHorStepPix*i*horInterval;
			textHorStep.ltBtmPt_.y = horAxisLine.pt1_.y + horStepLabHgt/1.5;
			textHorStep.color_ = chart->dspHorStepLabel.clrTxtFore;
			textHorStep.fn_ = chart->dspHorStepLabel.strFont.data();
			textHorStep.nFontSize_ = chart->dspHorStepLabel.nFontSize;
			if (chart->vecStrHorCusItem.size()==dataCols)
			{
				textHorStep.strTxt_ = chart->vecStrHorCusItem[i*horInterval];
			}
			else
			{
				textHorStep.strTxt_ = to_string(i*horInterval);
			}
			//���ݱ�ǩ�ַ����ȵ�����ʾλ��
			textHorStep.ltBtmPt_.x -= horStepLabWid* textHorStep.strTxt_.length() / 2;
			//����
			DispText(img, textHorStep);
		}
		//���ᵥ����ǩ
		HSV::TextDraw textVerStep;
		int verInterval = floor(rangeValue / chart->nVerAxisLabNum);
		for (int i = 0; i <= chart->nVerAxisLabNum; i++)
		{
			textVerStep.ltBtmPt_.x = verAxisLine.pt1_.x - verStepLabWid*6;
			textVerStep.ltBtmPt_.y = verAxisLine.pt1_.y - i*verInterval*verUnit+ verStepLabHgt/2;
			textVerStep.color_ = chart->dspHorStepLabel.clrTxtFore;
			textVerStep.fn_ = chart->dspHorStepLabel.strFont.data();
			textVerStep.nFontSize_ = chart->dspHorStepLabel.nFontSize;
			textVerStep.strTxt_ = to_string(i*verInterval);

			DispText(img, textVerStep);
		}

		///////////////////���Ƶײ���ע///////////////////
		HSV::TextDraw note;
		note.color_ = chart->dspChartNote.clrTxtFore;
		note.nFontSize_ = chart->dspChartNote.nFontSize;
		note.fn_ = chart->dspChartNote.strFont.data();
		note.strTxt_ = chart->strChartNote;
		note.ltBtmPt_.x = recChart.ltTopPt_.x + (recChart.w_ - noteWid) / 2;
		note.ltBtmPt_.y = recChart.ltTopPt_.y + recChart.h_;
		DispText(img, note);

	}

	void  DrawLineChart(GCImage & img, CLineChartDraw * chart, int nPageWidth, int nPageHeight)
	{
		//����������
		int nChartWid, nChartHgt;
		int dataRows, dataCols;
		chart->GetChartSize(nChartWid, nChartHgt, nPageWidth, nPageHeight);
		chart->GetDataSize(dataRows, dataCols);
		int titleWid, titleHgt, noteWid, noteHgt;
		int horAxisLabWid, horAxisLabHgt, verAxisLabWid, verAxisLabHgt;
		int horStepLabWid, horStepLabHgt, verStepLabWid, verStepLabHgt;
		GetTextSize(chart->strChartTitle, chart->dspChartTitle.strFont, chart->dspChartTitle.nFontSize, titleWid, titleHgt);
		GetTextSize(chart->strChartNote, chart->dspChartNote.strFont, chart->dspChartNote.nFontSize, noteWid, noteHgt);
		GetTextSize(chart->strHorAxisLabel, chart->dspHorAxisLabel.strFont, chart->dspHorAxisLabel.nFontSize, horAxisLabWid, horAxisLabHgt);
		GetTextSize(chart->strVerAxisLabel, chart->dspVerAxisLabel.strFont, chart->dspVerAxisLabel.nFontSize, verAxisLabWid, verAxisLabHgt);
		GetTextSize("1", chart->dspHorStepLabel.strFont, chart->dspHorStepLabel.nFontSize, horStepLabWid, horStepLabHgt);
		GetTextSize("1", chart->dspVerStepLabel.strFont, chart->dspVerStepLabel.nFontSize, verStepLabWid, verStepLabHgt);
		//���д���������X����
		chart->ptTopLeft.x = (nPageWidth - nChartWid) / 2;

		///////////////////����ǳɫ���///////////////////
		HSV::RectDraw recChart;
		recChart.ltTopPt_.x = chart->ptTopLeft.x;
		recChart.ltTopPt_.y = chart->ptTopLeft.y;
		recChart.w_ = nChartWid;
		recChart.h_ = nChartHgt;
		recChart.color_ = HSV::ScalarGC(150, 150, 150);
		DrawRect(img, recChart);

		///////////////////���Ʊ���///////////////////
		HSV::TextDraw title;
		title.color_ = chart->dspChartTitle.clrTxtFore;
		title.nFontSize_ = chart->dspChartTitle.nFontSize;
		title.fn_ = chart->dspChartTitle.strFont.data();
		title.strTxt_ = chart->strChartTitle;
		title.ltBtmPt_.x = recChart.ltTopPt_.x + (recChart.w_ - titleWid) / 2;
		title.ltBtmPt_.y = chart->ptTopLeft.y + titleHgt;
		DispText(img, title);

		///////////////////����ͼ��///////////////////
		//����
		HSV::LineDraw horAxisLine;
		horAxisLine.pt1_.x = recChart.ltTopPt_.x + chart->nHorStepPix + verStepLabWid*6;
		horAxisLine.pt1_.y = recChart.ltTopPt_.y + recChart.h_ - noteHgt - horStepLabHgt;
		horAxisLine.pt2_.x = recChart.ltTopPt_.x + recChart.w_ - chart->nHorStepPix + verStepLabWid*6 - chart->dAxisExtend;
		horAxisLine.pt2_.y = recChart.ltTopPt_.y + recChart.h_ - noteHgt - horStepLabHgt;
		horAxisLine.color_ = chart->dspHorAxisLine.clrObj;
		horAxisLine.thickness_ = chart->dspHorAxisLine.thickObj;
		DrawLine(img, horAxisLine);
		HSV::ArrowDraw horArr;
		int nArrLen = max(nChartWid / 25, 10);
		horArr.pt2_ = horAxisLine.pt2_;
		horArr.pt1_.x = horAxisLine.pt2_.x - nArrLen;
		horArr.pt1_.y = horAxisLine.pt2_.y;
		horArr.tipLen_ = 0.3;
		horArr.degAng_ = 25;
		horArr.thickness_ = 1;
		horArr.color_ = horAxisLine.color_;
		DrawArrow(img, horArr);
		//����
		HSV::LineDraw verAxisLine;
		verAxisLine.pt1_.x = recChart.ltTopPt_.x + chart->nHorStepPix + verStepLabWid*6;
		verAxisLine.pt1_.y = recChart.ltTopPt_.y + recChart.h_ - noteHgt - horStepLabHgt;
		verAxisLine.pt2_.x = recChart.ltTopPt_.x + chart->nHorStepPix + verStepLabWid*6;
		verAxisLine.pt2_.y = recChart.ltTopPt_.y + titleHgt - horStepLabHgt;
		verAxisLine.color_ = chart->dspVerAxisLine.clrObj;
		verAxisLine.thickness_ = chart->dspVerAxisLine.thickObj;
		DrawLine(img, verAxisLine);
		HSV::ArrowDraw verArr;
		verArr.pt2_ = verAxisLine.pt2_;
		verArr.pt1_.x = verAxisLine.pt2_.x;
		verArr.pt1_.y = verAxisLine.pt2_.y + nArrLen;
		verArr.tipLen_ = 0.3;
		verArr.degAng_ = 25;
		verArr.thickness_ = 1;
		verArr.color_ = verAxisLine.color_;
		DrawArrow(img, verArr);
		//����
		double maxValue, minValue;
		switch (chart->eDataType)
		{
		case DataType::INT:
			maxValue= (double)*max_element(chart->nData[0].begin(), chart->nData[0].end());
			minValue= (double)*min_element(chart->nData[0].begin(), chart->nData[0].end());
			for (size_t i = 1; i < chart->nData.size(); i++)
			{
				double minval, maxval;
				maxval = (double)*max_element(chart->nData[i].begin(), chart->nData[i].end());
				minval = (double)*min_element(chart->nData[i].begin(), chart->nData[i].end());
				maxValue = max(maxValue, maxval);
				minValue = min(minValue, minval);
			}
			break;
		case DataType::FLOAT:
			maxValue = (double)*max_element(chart->fData[0].begin(), chart->fData[0].end());
			minValue = (double)*min_element(chart->fData[0].begin(), chart->fData[0].end());
			for (size_t i = 1; i < chart->fData.size(); i++)
			{
				double minval, maxval;
				maxval = (double)*max_element(chart->fData[i].begin(), chart->fData[i].end());
				minval = (double)*min_element(chart->fData[i].begin(), chart->fData[i].end());
				maxValue = max(maxValue, maxval);
				minValue = min(minValue, minval);
			}
			break;
		case DataType::DOUBLE:
			maxValue = (double)*max_element(chart->dData[0].begin(), chart->dData[0].end());
			minValue = (double)*min_element(chart->dData[0].begin(), chart->dData[0].end());
			for (size_t i = 1; i < chart->dData.size(); i++)
			{
				double minval, maxval;
				maxval = (double)*max_element(chart->dData[i].begin(), chart->dData[i].end());
				minval = (double)*min_element(chart->dData[i].begin(), chart->dData[i].end());
				maxValue = max(maxValue, maxval);
				minValue = min(minValue, minval);
			}
			break;
		}

		double verUnit = 1.0;
		double rangeValue = chart->bCoorOrgZero ? maxValue : (maxValue - minValue);
		rangeValue = max(0.00000001, rangeValue);//����Ϊ0
		verUnit = (recChart.h_ - titleHgt - noteHgt - verAxisLabHgt) / rangeValue;
		
		for (int i = 0; i < dataRows; i++)
		{
			int valHgt1, valHgt2;
			HSV::LineDraw lineValue;
			lineValue.color_= chart->dspData[i].clrObj;
			lineValue.thickness_ = max(1,chart->dspData[i].thickObj);

			for (int j = 0; j < dataCols-1; j++)
			{
				switch (chart->eDataType)
				{
				case DataType::INT:
					valHgt1 = chart->bCoorOrgZero ? (chart->nData[i][j] * verUnit)
						: ((chart->nData[i][j] - minValue)*verUnit);
					valHgt2 = chart->bCoorOrgZero ? (chart->nData[i][j+1] * verUnit)
						: ((chart->nData[i][j+1] - minValue)*verUnit);
					break;
				case DataType::FLOAT:
					valHgt1 = chart->bCoorOrgZero ? (chart->fData[i][j] * verUnit)
						: ((chart->fData[i][j] - minValue)*verUnit);
					valHgt2 = chart->bCoorOrgZero ? (chart->fData[i][j + 1] * verUnit)
						: ((chart->fData[i][j + 1] - minValue)*verUnit);
					break;
				case DataType::DOUBLE:
					valHgt1 = chart->bCoorOrgZero ? (chart->dData[i][j] * verUnit)
						: ((chart->dData[i][j] - minValue)*verUnit);
					valHgt2 = chart->bCoorOrgZero ? (chart->dData[i][j+1] * verUnit)
						: ((chart->dData[i][j+1] - minValue)*verUnit);
					break;
				}
				lineValue.pt1_.x = horAxisLine.pt1_.x + chart->nHorStepPix*j+ chart->nHorStepPix/2;
				lineValue.pt1_.y= horAxisLine.pt1_.y - valHgt1;

				lineValue.pt2_.x = horAxisLine.pt1_.x + chart->nHorStepPix*(j+1) + chart->nHorStepPix / 2;
				lineValue.pt2_.y = horAxisLine.pt1_.y - valHgt2;

				DrawLine(img, lineValue);
			}
		}
		//�����ǩ
		HSV::TextDraw textHorAxixLabel;
		textHorAxixLabel.ltBtmPt_.x = horArr.pt2_.x;
		textHorAxixLabel.ltBtmPt_.y = horArr.pt2_.y + horAxisLabHgt;
		textHorAxixLabel.strTxt_ = chart->strHorAxisLabel;
		textHorAxixLabel.fn_ = chart->dspHorAxisLabel.strFont.data();
		textHorAxixLabel.nFontSize_ = chart->dspHorAxisLabel.nFontSize;
		textHorAxixLabel.color_ = chart->dspHorAxisLabel.clrTxtFore;
		DispText(img, textHorAxixLabel);
		//�����ǩ
		HSV::TextDraw textVerAxixLabel;
		textVerAxixLabel.ltBtmPt_.x = verArr.pt2_.x;
		textVerAxixLabel.ltBtmPt_.y = verArr.pt2_.y - verAxisLabHgt;
		textVerAxixLabel.strTxt_ = chart->strVerAxisLabel;
		textVerAxixLabel.fn_ = chart->dspVerAxisLabel.strFont.data();
		textVerAxixLabel.nFontSize_ = chart->dspVerAxisLabel.nFontSize;
		textVerAxixLabel.color_ = chart->dspVerAxisLabel.clrTxtFore;
		DispText(img, textVerAxixLabel);
		//���ᵥ����ǩ
		HSV::TextDraw textHorStep;
		int horInterval = floor(dataCols / chart->nHorAxisLabNum);
		for (int i = 0; i <= chart->nHorAxisLabNum; i++)
		{
			textHorStep.ltBtmPt_.x = horAxisLine.pt1_.x + chart->nHorStepPix / 2 + chart->nHorStepPix*i*horInterval;
			textHorStep.ltBtmPt_.y = horAxisLine.pt1_.y + horStepLabHgt / 1.5;
			textHorStep.color_ = chart->dspHorStepLabel.clrTxtFore;
			textHorStep.fn_ = chart->dspHorStepLabel.strFont.data();
			textHorStep.nFontSize_ = chart->dspHorStepLabel.nFontSize;
			if (chart->vecStrHorCusItem.size() == dataCols)
			{
				textHorStep.strTxt_ = chart->vecStrHorCusItem[i*horInterval];
			}
			else
			{
				textHorStep.strTxt_ = to_string(i*horInterval);
			}
			//���ݱ�ǩ�ַ����ȵ�����ʾλ��
			textHorStep.ltBtmPt_.x -= horStepLabWid* textHorStep.strTxt_.length() / 2;
			//����
			DispText(img, textHorStep);
		}
		//���ᵥ����ǩ
		HSV::TextDraw textVerStep;
		int verInterval = floor(rangeValue / chart->nVerAxisLabNum);
		for (int i = 0; i <= chart->nVerAxisLabNum; i++)
		{
			textVerStep.ltBtmPt_.x = verAxisLine.pt1_.x - verStepLabWid*6;
			textVerStep.ltBtmPt_.y = verAxisLine.pt1_.y - i*verInterval*verUnit+ verStepLabHgt/2;
			textVerStep.color_ = chart->dspHorStepLabel.clrTxtFore;
			textVerStep.fn_ = chart->dspHorStepLabel.strFont.data();
			textVerStep.nFontSize_ = chart->dspHorStepLabel.nFontSize;
			textVerStep.strTxt_ = to_string(i*verInterval);
			DispText(img, textVerStep);
		}
		///////////////////���Ƶײ���ע///////////////////
		HSV::TextDraw note;
		note.color_ = chart->dspChartNote.clrTxtFore;
		note.nFontSize_ = chart->dspChartNote.nFontSize;
		note.fn_ = chart->dspChartNote.strFont.data();
		note.strTxt_ = chart->strChartNote;
		note.ltBtmPt_.x = recChart.ltTopPt_.x + (recChart.w_ - noteWid) / 2;
		note.ltBtmPt_.y = recChart.ltTopPt_.y + recChart.h_;
		DispText(img, note);

	}

	void  DrawPieChart(GCImage & img, CPieChartDraw * chart, int nPageWidth, int nPageHeight)
	{

	}

	int  MergeTwoImg(cv::Mat* src1, cv::Mat* src2, cv::Mat* dst)
	{
		int nRet = (int)Error_ID::ERR_OK;
		if (src1 == nullptr || src2 == nullptr)
		{
			return (int)Error_ID::ERR_IMAGE_NULL;
		}
		if (src1->empty() || src2->empty())
		{
			return (int)Error_ID::ERR_IMAGE_NULL;
		}
		if (dst == nullptr)
		{
			dst = new cv::Mat();
		}

		//cv::hconcat(*imgSrc1, *imgSrc2, *imgDst);//ˮƽ�ϲ�
		cv::vconcat(*src1, *src2, *dst);//��ֱ�ϲ�

		return nRet;
	}

	int  CombineImages(cv::Mat * src1, cv::Mat * src2, cv::Mat * dst)
	{
		int nRet = (int)Error_ID::ERR_OK;
		string direction = "top";

		// ���ͼ���Ƿ�ɹ�����
		if (src1 == nullptr || src2 == nullptr)
		{
			return (int)Error_ID::ERR_IMAGE_NULL;
		}
		if (src1->empty() || src2->empty())
		{
			return (int)Error_ID::ERR_IMAGE_NULL;
		}

		// ȷ��ƴ�Ӻ�ͼ��Ŀ��Ⱥ͸߶�
		int width, height;
		if (direction == "left" || direction == "right") {
			width = src1->cols + src2->cols;
			height = max(src1->rows, src2->rows);
		}
		else if (direction == "top" || direction == "bottom") {
			width = max(src1->cols, src2->cols);
			height = src1->rows + src2->rows;
		}
		else {
			return (int)Error_ID::ERR_PROC_NG;
		}

		//src��dst����Ϊͬһ������,��Ҫ�м�ͼ�������Ž��ͼ��
		cv::Mat resultImage(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
		if (dst == nullptr)
		{
			dst = new cv::Mat();
		}

		// ������ͼ��ƴ�ӵ���ͼ����
		if (direction == "left") {
			src1->copyTo(resultImage(cv::Rect(0, 0, src1->cols, src1->rows)));
			src2->copyTo(resultImage(cv::Rect(src1->cols, 0, src2->cols, src2->rows)));
		}
		else if (direction == "right") {
			src2->copyTo(resultImage(cv::Rect(0, 0, src2->cols, src2->rows)));
			src1->copyTo(resultImage(cv::Rect(src2->cols, 0, src1->cols, src1->rows)));
		}
		else if (direction == "top") {
			src1->copyTo(resultImage(cv::Rect(0, 0, src1->cols, src1->rows)));
			src2->copyTo(resultImage(cv::Rect(0, src1->rows, src2->cols, src2->rows)));
		}
		else if (direction == "bottom") {
			src2->copyTo(resultImage(cv::Rect(0, 0, src2->cols, src2->rows)));
			src1->copyTo(resultImage(cv::Rect(0, src2->rows, src1->cols, src1->rows)));
		}
		*dst = resultImage;

		return nRet;
	}

}
