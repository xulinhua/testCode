#include <fstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <iterator>
#include <vector>
#include <cuda_runtime.h>
bool load_txt_to_vec(const std::string& path, std::vector<float>& out);
void save_chw_txt(const float* data, int c, int h, int w, const char* path);
