#pragma once
#define USE_OPENCV_480

#ifdef USE_OPENCV_480
// Use system-installed OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d_c.h>

// 移除了Windows特定的#pragma comment(lib,...)指令

#endif

#ifdef USE_OPENCV_440
// Use system-installed OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d_c.h>

// 移除了Windows特定的#pragma comment(lib,...)指令

#endif

#ifdef USE_OPENCV_246
// Use system-installed OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// 移除了Windows特定的#pragma comment(lib,...)指令

#endif