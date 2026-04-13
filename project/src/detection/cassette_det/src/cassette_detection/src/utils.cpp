#include "utils.h"

bool load_txt_to_vec(const std::string& path, std::vector<float>& out)
{
    std::ifstream fs(path, std::ios::in);
    if (!fs.is_open()) return false;
    out.assign(std::istream_iterator<float>(fs),
               std::istream_iterator<float>());
    return !out.empty();
}

void save_chw_txt(const float* data, int c, int h, int w, const char* path)
{
    std::ofstream fs(path, std::ios::out);
    fs << std::scientific << std::setprecision(9);
    for (int i = 0; i < c * h * w; ++i) fs << data[i] << '\n';
}