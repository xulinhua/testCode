#include "utils.h"

Logger gLogger;

bool load_txt_to_vec(const std::string& path, std::vector<float>& out) {
    std::ifstream file(path);
    if (!file.is_open()) {
        gLogger.error("无法打开文件: ", path);
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        float val = std::stof(line);
        out.push_back(val);
    }

    file.close();
    return true;
}

void save_chw_txt(const float* data, int c, int h, int w, const char* path) {
    std::ofstream file(path);
    if (!file.is_open()) {
        gLogger.error("无法打开文件进行写入: ", path);
        return;
    }

    for (int ci = 0; ci < c; ++ci) {
        file << "Channel " << ci << ":\n";
        for (int hi = 0; hi < h; ++hi) {
            for (int wi = 0; wi < w; ++wi) {
                file << std::fixed << std::setprecision(6)
                     << data[ci * h * w + hi * w + wi] << " ";
            }
            file << "\n";
        }
    }

    file.close();
    gLogger.info("保存CHW数据到: ", path);
}

void save_binary_txt(const unsigned char* data, int h, int w, const char* path) {
    std::ofstream file(path);
    if (!file.is_open()) {
        gLogger.error("无法打开文件进行写入: ", path);
        return;
    }

    for (int hi = 0; hi < h; ++hi) {
        for (int wi = 0; wi < w; ++wi) {
            file << static_cast<int>(data[hi * w + wi]) << " ";
        }
        file << "\n";
    }

    file.close();
    gLogger.info("保存二进制数据到: ", path);
}
