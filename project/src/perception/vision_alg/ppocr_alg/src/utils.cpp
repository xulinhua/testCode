#include "utils.h"
#include "logger.h"
#include <fstream>
#include <iostream>
#include <iomanip>

// 全局Logger实例
Logger gLogger;

bool load_txt_to_vec(const std::string& path, std::vector<float>& out)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << path << std::endl;
        return false;
    }
    
    float value;
    while (file >> value) {
        out.push_back(value);
    }
    
    file.close();
    return true;
}

void save_chw_txt(const float* data, int c, int h, int w, const char* path)
{
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Cannot create file: " << path << std::endl;
        return;
    }
    
    file << std::fixed << std::setprecision(6);
    
    for (int ch = 0; ch < c; ++ch) {
        file << "Channel " << ch << ":\n";
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                int idx = ch * h * w + i * w + j;
                file << data[idx] << " ";
            }
            file << "\n";
        }
        file << "\n";
    }
    
    file.close();
}

void save_binary_txt(const unsigned char* data, int h, int w, const char* path)
{
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Cannot create file: " << path << std::endl;
        return;
    }
    
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            int idx = i * w + j;
            file << static_cast<int>(data[idx]) << " ";
        }
        file << "\n";
    }
    
    file.close();
}