/*****************************************************************************************************************
* 功能说明：封装的图像数据结构类，供运动与视觉交互时调用

*备注：
----------------------------------------------------------------------------------------------------------------*/
#pragma once
#include <string>
#include <mutex>
namespace cv
{
	class Mat;
}
class GCImage
{
public:
	GCImage();
	~GCImage();
	GCImage(const GCImage &img);
	GCImage(int height, int width, int channel, unsigned char* dat);
	void operator = (const GCImage& img);
	bool Create(int height, int width, int channel,unsigned char* dat);
	void CopyTo(GCImage* img) const;
	void CopyTo(GCImage** img) const;
	bool Zeros(int height, int width, int channel);
	int GetWidth() const;
	int GetHeight() const;
	int GetChannel() const;
	unsigned char* GetData() const;
	bool IsEmpty() const;
	cv::Mat* GetMatImg() { return pMat; }
	const cv::Mat* GetConstMatImg() const { return pMat; }
	std::mutex& GetLock() {
		return cs_Dat;
	}
	bool LoadImage(std::string path);
	bool SaveImage(std::string path);
private:
	cv::Mat* pMat;
	std::mutex cs_Dat;//如果在多线程中调用不加锁会引起崩溃，默认在类的外部加锁防护
};

void ClearGCImage(GCImage** img);