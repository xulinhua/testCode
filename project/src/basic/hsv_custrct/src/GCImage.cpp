//#include "stdafx.h"
#include "GCImage.h"
#include <assert.h>
#include "opencvLib.h"

//获得位图每行数据的大小
inline long GetBytesPerLine(int imgWidth, int iBitsPerPixel)
{
	return (((imgWidth * iBitsPerPixel) + 31) >> 5) << 2;
}

GCImage::GCImage()
{
	pMat = new cv::Mat;
	PLATFORM_INIT_CRITICAL_SECTION(cs_Dat);
}

GCImage::~GCImage()
{
	PLATFORM_DELETE_CRITICAL_SECTION(cs_Dat);
	delete pMat;
}

GCImage::GCImage(const GCImage &img)
{
	if (&img == this)
		return;
	pMat = new cv::Mat;
	PLATFORM_INIT_CRITICAL_SECTION(cs_Dat);
	if (img.IsEmpty())
		return;
	Create(img.pMat->rows, img.pMat->cols, img.pMat->channels(), img.pMat->data);
}

GCImage::GCImage(int height, int width, int channel, unsigned char* dat)
{
	pMat = new cv::Mat;
	PLATFORM_INIT_CRITICAL_SECTION(cs_Dat);
	Create(height, width, channel, dat);
}

void GCImage::operator = (const GCImage& img)
{
	if (&img == this)
		return;
	//img.pMat->copyTo(*pMat);
	*pMat = *img.pMat;
}

bool GCImage::Create(int height, int width, int channel,unsigned char* dat)
{
	bool bRet = true;
	try
	{
		if (height <= 0 || width <= 0 || (channel != 1 && channel != 3))
			bRet = false;
		else
		{
			/*for (int i = 0; i < height * width * channel; i++)
			{
			dat[i] = 0;
			}
			unsigned char* tesdat = new unsigned char[height * width * channel];
			memcpy(tesdat, dat, height * width * channel);
			delete[] tesdat;*/
			assert(pMat != NULL);
			int type = (channel == 1) ? CV_8UC1 : CV_8UC3;
			if (pMat->rows != height || pMat->cols != width || pMat->channels() != channel)
			{
				*pMat = cv::Mat::zeros(height, width, type);
				//memset(pMat->data, 0, height * width * channel);
			}	
			if (dat != NULL)
			{
				memcpy(pMat->data, dat, height * width * channel);
			}		
			//assert(pMat->empty() == false);
		}
	}
	catch (...)
	{
		bRet = false;
	}
	return bRet;
}

void GCImage::CopyTo(GCImage* img) const
{
	pMat->copyTo(*img->pMat);
}

void GCImage::CopyTo(GCImage** img) const
{
	if (*img == NULL)
		*img = new GCImage;
	pMat->copyTo(*((*img)->pMat));
}

bool GCImage::Zeros(int height, int width, int channel)
{
	return Create(height, width, channel, NULL);
}

int GCImage::GetWidth() const
{
	return pMat->cols;
}

int GCImage::GetHeight() const
{
	return pMat->rows;
}

int GCImage::GetChannel() const
{
	return pMat->channels();
}

unsigned char* GCImage::GetData() const
{
	return pMat->data;
}

bool GCImage::IsEmpty() const
{
	return (pMat->data == NULL) ? true : false;
}

bool GCImage::LoadImage(std::string path)
{
	if (this == NULL)
	{
		return false;
	}
	*pMat = cv::imread(path.c_str(), -1);
	return true;
}
bool GCImage::SaveImage(std::string path)
{
	if (this == NULL)
	{
		return false;
	}
	if (pMat->data != NULL)
	{
		try
		{
			cv::imwrite(path.c_str(), *pMat);
		}
		catch (const cv::Exception& ex) {
			// 处理异常，打印错误信息等
			std::cerr << "OpenCV Exception: " << ex.what() << std::endl;
		}
	}
	else
	{
		return false;
	}

	return true;
}

void ClearGCImage(GCImage** img)
{
	if (*img)
	{
		delete *img;
		*img = NULL;
	}
}
