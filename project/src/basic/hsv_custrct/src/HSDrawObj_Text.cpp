//#include "stdafx.h"
#include"HSDrawObj.h"
#include<assert.h>
#include<algorithm>
#include <vector>
#include <sstream>
#include <opencv2/opencv.hpp>
#ifdef _WIN32
#include <Windows.h>
#endif // _WIN32

namespace HSV
{
	
#pragma region 绘图对象_标准文本
	TextDraw::TextDraw()
	{
		Init();
	}

	TextDraw::TextDraw(const TextDraw& para)
	{
		Init();
		CopyFrom(para);
	}

	TextDraw::TextDraw(const char* str, Point ltBtmPt, GC_COL clrType, int fontSize, const char *fn, bool bItalic, bool bUnderline)
	{
		this->Init();
		strTxt_ = str;
		ltBtmPt_ = ltBtmPt;
		this->SetColor(clrType);
		nFontSize_ = fontSize;
		fn_ = fn;
		bItalic_ = bItalic;
		bUnderline_ = bUnderline;
		this->UpdateTextSize();
	}

	TextDraw::TextDraw(const char* str, Point ltBtmPt, ScalarGC color, int fontSize, const char *fn, bool bItalic, bool bUnderline)
	{
		this->Init();
		strTxt_ = str;
		ltBtmPt_ = ltBtmPt;
		color_ = color;
		nFontSize_ = fontSize;
		fn_ = fn;
		bItalic_ = bItalic;
		bUnderline_ = bUnderline;
		this->UpdateTextSize();
	}

	TextDraw::~TextDraw()
	{

	}

	TextDraw& TextDraw::operator = (const TextDraw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool TextDraw::equal(const DrawObjBase &obj) const
	{
		// 检查当前对象的指针是否为空
		if (this == nullptr) {
			// 处理空指针情况，例如可以返回 false 或抛出异常
			return false;
		}

		bool bEqual = false;
		if (this->GetDrawType() == obj.GetDrawType()/* && typeid(this) == typeid(&obj)*/)
		{
			if (this->nID_ >= 0)
			{
				return (this->nID_ == obj.nID_);
			}
			else
			{
				if (const TextDraw *ptr = dynamic_cast<const TextDraw*>(&obj))
				{
					if (this->strTxt_ == ptr->strTxt_			 &&
						this->ltBtmPt_ == ptr->ltBtmPt_	      	 &&
						this->nFontSize_ == ptr->nFontSize_	     &&
						this->fn_ == ptr->fn_					 &&
						this->bItalic_ == ptr->bItalic_		     &&
						this->bUnderline_ == ptr->bUnderline_    &&
						this->bBox_ == ptr->bBox_                &&
						this->bkgcolor_ == ptr->bkgcolor_        &&
						this->bTxtInImage_ == ptr->bTxtInImage_  &&
						this->txtW_ == ptr->txtW_			     &&
						this->txtH_ == ptr->txtH_)
					{
						bEqual = true;
					}
				}
			}
		}
		return bEqual;
	}

	bool TextDraw::operator==(const TextDraw & obj)
	{
		return equal(obj);
	}

	//从para拷贝数据
	void TextDraw::CopyFrom(const TextDraw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void TextDraw::CopyTo(TextDraw& para) const
	{
		if (this != &para)
		{
			DrawObjBase::CopyTo(para);
			para.strTxt_ = strTxt_;
			para.ltBtmPt_ = ltBtmPt_;
			para.nFontSize_ = nFontSize_;
			para.fn_ = fn_;//设定显示的字符的TrueType字体类型，默认使用Arial字体
			para.bItalic_ = bItalic_;//字体是否斜体
			para.bUnderline_ = bUnderline_;//是否有下划线
			para.bBox_ = bBox_;// ture：有白底框      false:无白底框
			para.bkgcolor_ = bkgcolor_;
			para.bTxtInImage_ = bTxtInImage_;// true:位置相对于图片    false:位置相对于窗口		
			para.txtW_ = txtW_;//记录当前文本宽度
			para.txtH_ = txtH_;//记录当前文本高度
		}
	}

	//从para拷贝数据	
	void TextDraw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::TextDraw* ptr = dynamic_cast<const HSV::TextDraw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void TextDraw::CopyTo(DrawObjBase** para) const
	{
		HSV::TextDraw* ptr = dynamic_cast<HSV::TextDraw*>(*para);
		CopyTo(*ptr);
	}

	void TextDraw::Init()
	{
		this->ClearTextInfo();

		strTxt_.clear();
		ltBtmPt_.x = 0;
		ltBtmPt_.y = 0;//文本左下角坐标
		nFontSize_ = 20;//字体大小
		fn_ = "Arial";//设定显示的字符的TrueType字体类型，默认使用Arial字体
		bItalic_ = false;//字体是否斜体
		bUnderline_ = false;//是否有下划线
		bBox_ = false;//ture：有白底框      false:无白底框
		bkgcolor_ = GetGCColor(GC_COL::GC_COL_WHITE);
		bTxtInImage_ = true;// true:位置相对于图片    false:位置相对于窗口
		txtW_ = 0;//记录当前文本宽度
		txtH_ = 0;//记录当前文本高度
		DrawObjBase::Init();
	}

	//当前对象是否为空（主要用于判断当前区域是否为空）
	bool TextDraw::IsEmpty() const
	{
		return strTxt_.empty();
	}

	//获取绘制图形类型
	HSV::DrawType TextDraw::GetDrawType() const
	{
		return DrawType::DRAW_TEXT;
	}

	bool TextDraw::IsRoiReg() const
	{
		return false;
	}

	void TextDraw::ClearTextInfo()
	{
		strTxt_.clear();
		ltBtmPt_.x = 0;
		ltBtmPt_.y = 0;
		txtW_ = 0;//记录当前文本宽度
		txtH_ = 0;//记录当前文本高度
	}

	void TextDraw::AddIntervalSymbolMsg(const std::string& strMsg)
	{
		if (strTxt_.length() > 0)
		{
			int pos = (int)strTxt_.find(strMsg);
			if (pos < 0)
				strTxt_ += strMsg;
			this->UpdateTextSize();
		}
	}
	//设置当前文本配置数据
	void TextDraw::SetTxtConfigDat(GC_COL color_type, int tmp_fontSize, const char *tmp_fn, bool tmp_bItalic, bool tmp_bUnderline)
	{
		this->SetColor(color_type);
		nFontSize_ = tmp_fontSize;
		fn_ = tmp_fn;
		bItalic_ = tmp_bItalic;
		bUnderline_ = tmp_bUnderline;
		this->UpdateTextSize();
	}
	void TextDraw::SetTxtConfigDat(ScalarGC color, int tmp_fontSize, const char *tmp_fn, bool tmp_bItalic, bool tmp_bUnderline)
	{
		color_ = color;
		nFontSize_ = tmp_fontSize;
		fn_ = tmp_fn;
		bItalic_ = tmp_bItalic;
		bUnderline_ = tmp_bUnderline;
		this->UpdateTextSize();
	}

#ifdef _WIN32
	void GetStringSize(HDC hDC, const char* str, int* w, int* h)
	{
		SIZE size;
		GetTextExtentPoint32A(hDC, str, strlen(str), &size);
		if (w != 0) *w = size.cx;
		if (h != 0) *h = size.cy;
	}
#elif __linux__
	void GetStringSize(const char* str, int* w, int* h, int fontFace,
		double fontScale, int thickness)
	{
		if (str == nullptr || strlen(str) == 0) {
			if (w != nullptr) *w = 0;
			if (h != nullptr) *h = 0;
			return;
		}

		cv::Size textSize = cv::getTextSize(str, fontFace, fontScale, thickness, nullptr);
		if (w != nullptr) *w = textSize.width;
		if (h != nullptr) *h = textSize.height;
	}


#endif
	void TextDraw::GetStdTextSize(int& w, int& h) const
	{
		this->GetTextSize(strTxt_.c_str(), &w, &h);
		if (0)
		{
			if (h > 0)
				h = 5 * (h / 5) + 5;//一行文本占用的垂直方向像素，确保为5的整数倍
		}
		else
		{
			h += 2;
		}
	}
	//更新当前文本尺寸大小
	void TextDraw::UpdateTextSize()
	{
		GetStdTextSize(txtW_, txtH_);
	}

#ifdef _WIN32
	//获取字体显示需要占用的尺寸
	void TextDraw::GetTextSize(const char* str, int* w, int* h) const
	{
		if (strlen(str) <= 0)
		{
			if (w != 0) *w = 0;
			if (h != 0) *h = 0;
			return;
		}
		//LOGFONTA 存储字体结构信息的结构体
		LOGFONTA lf;
		lf.lfHeight = -nFontSize_;        //指定逻辑单位的字符或者字符元高度。
		lf.lfWidth = 0;                 //指定逻辑单位的字体字符的平均宽度。
		lf.lfEscapement = 0;            //指定每行文本输出时相对于设备x轴的角度，其单位为1/10度。
		lf.lfOrientation = 0;           //指定字符基线相对于设备x轴的角度，其单位为1/10度。此值在Win9X中和lfEscapement具有相同的值，而在WinNT下有时候可能不同。
		lf.lfWeight = 5;                //指定字体的重量，Windows中字体重量表示字体的粗细程度，其范围在0～1000之间，正常为400，粗体为700，若此值为空，则使用默认的字体重量。
		lf.lfItalic = bItalic_;          //此值为TRUE时，字体为斜体。
		lf.lfUnderline = bUnderline_;    //此值为TRUE时，字体带下划线。
		lf.lfStrikeOut = 0;             //此值为TRUE时，字体带删除线。
		lf.lfCharSet = DEFAULT_CHARSET; //指定所使用的字符集，如GB2312_CHARSET,CHINESEBIG5_CHARSET等。
		lf.lfOutPrecision = 0;          //指定输出精度，它定义了输出与所要求的字体高度、宽度、字符方向及字体类型等相接近的程度。
		lf.lfClipPrecision = 0;         //指定剪辑精度，它定义了当字符的一部分超过剪辑区域时对字符的剪辑方式。
		lf.lfQuality = PROOF_QUALITY;   //指定输出质量，它定义了GDI在匹配逻辑字体属性到实际的物理字体时所使用的方式。
		lf.lfPitchAndFamily = 0;        //指定字体的字符间距和族。
		strcpy_s(lf.lfFaceName, fn_);    //指向NULL结尾的字符串的指针，此字符串即为所使用的字体名称，其长度不能超过32个字符，如果为空，则使用系统默认的字体。

		HFONT hf = CreateFontIndirectA(&lf); //创建一种在指定结构定义其特性的逻辑字体
		HDC hDC = CreateCompatibleDC(0);     //创建一个与指定设备兼容的内存设备上下文环境
		HFONT hOldFont = (HFONT)SelectObject(hDC, hf);  //将对象选择到指定的设备上下文中

		char buf[1 << 12];
		strcpy_s(buf, str);
		int strBaseW = 0, strBaseH = 0;
		char *bufT[1 << 12];  // 这个用于分隔字符串后剩余的字符，可能会超出。
							  //处理多行
		int nnh = 0;
		int cw = 0, ch = 0;
		const char* ln = strtok_s(buf, "\n", bufT);
		while (ln != 0)
		{
			GetStringSize(hDC, ln, &cw, &ch);
			strBaseW = max(strBaseW, cw);
			strBaseH = max(strBaseH, ch);
			ln = strtok_s(0, "\n", bufT);
			nnh++;
		}
		strBaseH *= nnh;
		SelectObject(hDC, hOldFont);
		DeleteObject(hf);
		DeleteObject(hDC);
		if (w != 0) *w = strBaseW;
		if (h != 0) *h = strBaseH;
	}
#elif __linux__
	int FontNameToOpenCVType(const std::string& fontName) {
		static const std::unordered_map<std::string, int> fontMap = {
			// 无衬线字体
			{"Arial", cv::FONT_HERSHEY_SIMPLEX},
			{"Arial Black", cv::FONT_HERSHEY_TRIPLEX},
			{"Helvetica", cv::FONT_HERSHEY_SIMPLEX},
			{"Verdana", cv::FONT_HERSHEY_DUPLEX},
			{"Tahoma", cv::FONT_HERSHEY_SIMPLEX},
			{"Trebuchet MS", cv::FONT_HERSHEY_DUPLEX},

			// 衬线字体
			{"Times New Roman", cv::FONT_HERSHEY_COMPLEX},
			{"Georgia", cv::FONT_HERSHEY_TRIPLEX},
			{"Garamond", cv::FONT_HERSHEY_COMPLEX},

			// 等宽字体
			{"Courier New", cv::FONT_HERSHEY_COMPLEX_SMALL},
			{"Courier", cv::FONT_HERSHEY_COMPLEX_SMALL},
			{"Lucida Console", cv::FONT_HERSHEY_COMPLEX_SMALL},

			// 手写体
			{"Comic Sans MS", cv::FONT_HERSHEY_SCRIPT_SIMPLEX},
			{"Brush Script MT", cv::FONT_HERSHEY_SCRIPT_COMPLEX},

			// 中文字体
			{"SimSun", cv::FONT_HERSHEY_COMPLEX},           // 宋体
			{"SimHei", cv::FONT_HERSHEY_COMPLEX},           // 黑体  
			{"Microsoft YaHei", cv::FONT_HERSHEY_COMPLEX},  // 微软雅黑
			{"KaiTi", cv::FONT_HERSHEY_COMPLEX},            // 楷体
			{"FangSong", cv::FONT_HERSHEY_COMPLEX},         // 仿宋
		};

		// 不区分大小写查找
		std::string lowerFont = fontName;
		std::transform(lowerFont.begin(), lowerFont.end(), lowerFont.begin(), ::tolower);

		for (const auto& pair : fontMap) {
			std::string key = pair.first;
			std::transform(key.begin(), key.end(), key.begin(), ::tolower);
			if (key.find(lowerFont) != std::string::npos ||
				lowerFont.find(key) != std::string::npos) {
				return pair.second;
			}
		}

		return cv::FONT_HERSHEY_SIMPLEX; // 默认字体
	}


	void TextDraw::GetTextSize(const char* str, int* w, int* h) const
	{
		if (nullptr == str) {
			w = 0;
			h = 0;
			return;
		}

		// 将字体名称映射到OpenCV的字体类型
		// 计算字体缩放比例，这里可以根据fontSize进行调整
		double fontScale = 1;

		int strBaseW = 0, strBaseH = 0;

		// 处理多行文本
		std::stringstream ss(str);
		std::string line;
		int lineCount = 0;

		while (std::getline(ss, line)) {
			if (!line.empty()) {
				cv::Size textSize = cv::getTextSize(line, FontNameToOpenCVType(fn_), fontScale, thickness_, nullptr);
				strBaseW = std::max(strBaseW, textSize.width);
				strBaseH = std::max(strBaseH, textSize.height);
			}
			lineCount++;
		}

		// 如果有多行，需要调整总高度
		if (lineCount > 1) {
			int lineHeight = strBaseH;
			// 估算多行文本的总高度（行高 + 行间距）
			strBaseH = lineHeight * lineCount + static_cast<int>(lineHeight * 0.3 * (lineCount - 1));
		}

		*w = strBaseW;
		*h = strBaseH;
	}

#endif
#pragma endregion

#pragma region 绘图对象_组合文本
	TextUnionDraw::TextUnionDraw()
	{
		Init();
	}

	TextUnionDraw::TextUnionDraw(const TextUnionDraw& para)
	{
		Init();
		CopyFrom(para);
	}


	TextUnionDraw::TextUnionDraw(TextDraw lable, TextDraw sbj, Point org, int direction)
	{
		this->Init();
		ltBtmPt_ = org;
		lab_ = lable;
		sbj_ = sbj;
		bHorUnion_ = (direction != 0) ? true : false;
		this->UpdateLableFontSizeAuto();//自动更新标签文本字体大小
		UpdateLablePositionAuto();
	}
	TextUnionDraw::TextUnionDraw(const char* lable_str, const char* sbj_str, Point org, GC_COL tmp_clrType,
		int direction, int fontSize, const char *fn, bool bItalic,
		bool bUnderline)
	{
		this->Init();
		ltBtmPt_ = org;
		lab_ = TextDraw(lable_str, org, tmp_clrType,
			fontSize, fn, bItalic, bUnderline);
		sbj_ = TextDraw(sbj_str, org, tmp_clrType,
			fontSize, fn, bItalic, bUnderline);
		bHorUnion_ = (direction != 0) ? true : false;
		this->UpdateLableFontSizeAuto();//自动更新标签文本字体大小
		UpdateLablePositionAuto();
	}
	TextUnionDraw::TextUnionDraw(const char* lable_str, const char* sbj_str, Point org, ScalarGC color,
		int direction, int fontSize, const char *fn, bool bItalic,
		bool bUnderline)
	{
		this->Init();
		ltBtmPt_ = org;
		lab_ = TextDraw(lable_str, org, color,
			fontSize, fn, bItalic, bUnderline);
		sbj_ = TextDraw(sbj_str, org, color,
			fontSize, fn, bItalic, bUnderline);
		bHorUnion_ = (direction != 0) ? true : false;
		this->UpdateLableFontSizeAuto();//自动更新标签文本字体大小
		UpdateLablePositionAuto();
	}

	TextUnionDraw::~TextUnionDraw()
	{

	}

	TextUnionDraw& TextUnionDraw::operator = (const TextUnionDraw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool TextUnionDraw::equal(const DrawObjBase &obj) const
	{
		// 检查当前对象的指针是否为空
		if (this == nullptr) {
			// 处理空指针情况，例如可以返回 false 或抛出异常
			return false;
		}

		bool bEqual = false;
		if (this->GetDrawType() == obj.GetDrawType() /*&& typeid(this) == typeid(&obj)*/)
		{
			if (this->nID_ >= 0)
			{
				return (this->nID_ == obj.nID_);
			}
			else
			{
				if (const TextUnionDraw *ptr = dynamic_cast<const TextUnionDraw*>(&obj))
				{

					if (this->lab_.equal(ptr->lab_) &&
						this->sbj_.equal(ptr->sbj_) &&
						this->ltBtmPt_ == ptr->ltBtmPt_	     &&
						this->bHorUnion_ == ptr->bHorUnion_	 &&
						this->txtW_ == ptr->txtW_			 &&
						this->txtH_ == ptr->txtH_)
					{
						bEqual = true;
					}
				}
			}
		}
		return bEqual;
	}

	bool TextUnionDraw::operator==(const TextUnionDraw & obj)
	{
		return equal(obj);
	}

	//从para拷贝数据
	void TextUnionDraw::CopyFrom(const TextUnionDraw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void TextUnionDraw::CopyTo(TextUnionDraw& para) const
	{
		if (this != &para)
		{
			DrawObjBase::CopyTo(para);
			para.lab_ = lab_;
			para.sbj_ = sbj_;
			para.txtW_ = txtW_;//记录当前文本整体宽度
			para.txtH_ = txtH_;//记录当前文本整体高度
		}
	}

	//从para拷贝数据	
	void TextUnionDraw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::TextUnionDraw* ptr = dynamic_cast<const HSV::TextUnionDraw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void TextUnionDraw::CopyTo(DrawObjBase** para) const
	{
		HSV::TextUnionDraw* ptr = dynamic_cast<HSV::TextUnionDraw*>(*para);
		CopyTo(*ptr);
	}

	void TextUnionDraw::Init()
	{
		lab_.Init();
		sbj_.Init();
		ltBtmPt_ = Point(0, 0);
		txtW_ = 0;//记录当前文本整体宽度
		txtH_ = 0;//记录当前文本整体高度
		bHorUnion_ = true;
		this->UpdateLableFontSizeAuto();//自动更新标签文本字体大小
		DrawObjBase::Init();
	}

	//当前对象是否为空 
	bool TextUnionDraw::IsEmpty() const
	{
		return (lab_.IsEmpty() && sbj_.IsEmpty());
	}

	//获取绘制图形类型
	HSV::DrawType TextUnionDraw::GetDrawType() const
	{
		return DrawType::DRAW_TEXT_UNION;
	}

	bool TextUnionDraw::IsRoiReg() const
	{
		return false;
	}

	//自动更新标签文本字体大小
	void TextUnionDraw::UpdateLableFontSizeAuto()
	{
		lab_.nFontSize_ = sbj_.nFontSize_ - 5;//文本标签字体大小
		if (lab_.nFontSize_ < 15)
			lab_.nFontSize_ = 15;
	}
	void TextUnionDraw::UpdateLablePositionAuto() //自动更新标签坐标
	{
		if (this->bHorUnion_)
		{
			lab_.ltBtmPt_ = ltBtmPt_;
			int Combin_x = ltBtmPt_.x;
			int Combin_y = ltBtmPt_.y;

			int lable_w, lable_h;
			lab_.GetStdTextSize(lable_w, lable_h);

			sbj_.ltBtmPt_ = Point(Combin_x + lable_w, Combin_y);
		}
		else
		{
			lab_.ltBtmPt_ = ltBtmPt_;
			int Combin_x = ltBtmPt_.x;
			int Combin_y = ltBtmPt_.y;

			int lable_w, lable_h;
			lab_.GetStdTextSize(lable_w, lable_h);

			sbj_.ltBtmPt_ = Point(Combin_x, Combin_y + lable_h);
		}
	}
	void TextUnionDraw::ClearTextInfo()
	{
		sbj_.ClearTextInfo();
		lab_.ClearTextInfo();
		txtW_ = 0;//记录当前文本整体宽度
		txtH_ = 0;//记录当前文本整体高度
	}
	//获取显示文本需要占用的整体尺寸
	void TextUnionDraw::GetDspTxtSize(int& w, int& h, int& lableWidth, int& sbjWidth) const
	{

		int lable_w, lable_h;
		lab_.GetStdTextSize(lable_w, lable_h);

		int sbj_w, sbj_h;
		sbj_.GetStdTextSize(sbj_w, sbj_h);
		w = lable_w + sbj_w;
#ifdef _WIN32
		h = max(lable_h, sbj_h);
#elif __linux__
		h = std::max(lable_h, sbj_h);
#endif
		sbjWidth = sbj_w;
		lableWidth = lable_w;
	}
	//获取显示文本需要占用的整体尺寸
	void TextUnionDraw::GetDspTxtSize(int& w, int& h, int& sbjWidth) const
	{
		int lableWidth = 0;
		GetDspTxtSize(w, h, lableWidth, sbjWidth);
	}
	//获取显示文本需要占用的整体尺寸
	void TextUnionDraw::GetDspTxtSize(int& w, int& h) const
	{
		int sbjWidth = 0, lableWidth = 0;
		GetDspTxtSize(w, h, lableWidth, sbjWidth);
	}
	//更新当前显示对象的文本尺寸大小
	void TextUnionDraw::UpdateDspTxtSize()
	{
		GetDspTxtSize(txtW_, txtH_, sbj_.txtW_, lab_.txtW_);//获取显示文本需要占用的整体尺寸
		sbj_.txtH_ = lab_.txtH_ = txtH_;
	}
#pragma endregion

#pragma region 绘图对象_元组文本
	TextTupleDraw::TextTupleDraw()
	{
		Init();
	}

	TextTupleDraw::TextTupleDraw(const TextTupleDraw& para)
	{
		Init();
		CopyFrom(para);
	}

	TextTupleDraw::~TextTupleDraw()
	{

	}

	TextTupleDraw& TextTupleDraw::operator = (const TextTupleDraw& para)
	{
		if (this != &para)
			CopyFrom(para);
		return *this;
	}

	bool TextTupleDraw::equal(const DrawObjBase &obj) const
	{
		// 检查当前对象的指针是否为空
		if (this == nullptr) {
			// 处理空指针情况，例如可以返回 false 或抛出异常
			return false;
		}

		bool bEqual = false;
		if (this->GetDrawType() == obj.GetDrawType()/* && typeid(this) == typeid(&obj)*/)
		{
			if (this->nID_ >= 0)
			{
				return (this->nID_ == obj.nID_);
			}
			else
			{
				if (const TextTupleDraw *ptr = dynamic_cast<const TextTupleDraw*>(&obj))
				{
					bEqual = this->tupName_.equal(ptr->tupName_);
				}
			}
		}
		return bEqual;
	}

	bool TextTupleDraw::operator==(const TextTupleDraw & obj)
	{
		return equal(obj);
	}

	//从para拷贝数据
	void TextTupleDraw::CopyFrom(const TextTupleDraw& para)
	{
		if (this != &para)
			para.CopyTo(*this);
	}
	//拷贝数据到para	
	void TextTupleDraw::CopyTo(TextTupleDraw& para) const
	{
		if (this != &para)
		{
			DrawObjBase::CopyTo(para);
			para.tupName_ = tupName_;
			para.txt1D_ = txt1D_;
		}
	}

	//从para拷贝数据	
	void TextTupleDraw::CopyFrom(const DrawObjBase* para)
	{
		const HSV::TextTupleDraw* ptr = dynamic_cast<const HSV::TextTupleDraw*>(para);
		CopyFrom(*ptr);
	}

	//拷贝数据到para
	void TextTupleDraw::CopyTo(DrawObjBase** para) const
	{
		HSV::TextTupleDraw* ptr = dynamic_cast<HSV::TextTupleDraw*>(*para);
		CopyTo(*ptr);
	}

	void TextTupleDraw::Init()
	{
		ClearTextInfo();
		DrawObjBase::Init();
	}

	//当前对象是否为空（主要用于判断当前区域是否为空）
	bool TextTupleDraw::IsEmpty() const
	{
		return (txt1D_.size() > 0) ? false : true;
	}

	//获取绘制图形类型
	HSV::DrawType TextTupleDraw::GetDrawType() const
	{
		return DrawType::DRAW_TEXT_TUPLE;
	}

	bool TextTupleDraw::IsRoiReg() const
	{
		return false;
	}

	void TextTupleDraw::ClearTextInfo()
	{
		tupName_.ClearTextInfo();
		txt1D_.clear();
	}
#pragma endregion

}
