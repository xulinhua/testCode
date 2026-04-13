#pragma once

#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cuda_runtime.h>
#include "logger.h"

// 全局Logger实例声明
extern Logger gLogger;

bool load_txt_to_vec(const std::string& path, std::vector<float>& out);
void save_chw_txt(const float* data, int c, int h, int w, const char* path);
void save_binary_txt(const unsigned char* data, int h, int w, const char* path);
