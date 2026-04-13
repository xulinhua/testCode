#include "../include/bas_operate/bas_utils.hpp"
#include <iostream>
#include <cassert>
#include <thread>

int main() {
    using namespace basmodule;
    
    // 测试字符串工具函数
    std::cout << "Testing string utilities..." << std::endl;
    
    // 测试trim函数
    std::string trimmed = trim("  hello world  ");
    assert(trimmed == "hello world");
    
    // 测试to_lower函数
    std::string lower = to_lower("HELLO WORLD");
    assert(lower == "hello world");
    
    // 测试to_upper函数
    std::string upper = to_upper("hello world");
    assert(upper == "HELLO WORLD");
    
    // 测试split函数
    std::vector<std::string> splitted = split("a,b,c", ",");
    assert(splitted.size() == 3);
    assert(splitted[0] == "a");
    assert(splitted[1] == "b");
    assert(splitted[2] == "c");
    
    // 测试replace函数
    std::string replaced = replace("hello world", "world", "there");
    assert(replaced == "hello there");
    
    // 测试starts_with函数
    bool starts = starts_with("hello world", "hello");
    assert(starts == true);
    
    // 测试ends_with函数
    bool ends = ends_with("hello world", "world");
    assert(ends == true);
    
    std::cout << "String utilities tests passed!" << std::endl;
    
    // 测试数学工具函数
    std::cout << "Testing math utilities..." << std::endl;
    
    // 测试随机数生成
    int rand_int = random_int(1, 10);
    assert(rand_int >= 1 && rand_int <= 10);
    
    double rand_double = random_double(1.0, 10.0);
    assert(rand_double >= 1.0 && rand_double <= 10.0);
    
    // 测试浮点数比较
    bool equal = float_equal(1.0, 1.0000000001, 1e-9);
    assert(equal == true);
    
    // 测试角度转换
    double rad = degrees_to_radians(180.0);
    assert(float_equal(rad, 3.14159265358979323846, 1e-10));
    
    double deg = radians_to_degrees(3.14159265358979323846);
    assert(float_equal(deg, 180.0, 1e-10));
    
    std::cout << "Math utilities tests passed!" << std::endl;
    
    // 测试时间工具函数
    std::cout << "Testing time utilities..." << std::endl;
    
    long long timestamp = get_timestamp_ms();
    assert(timestamp > 0);
    
    std::string formatted = format_timestamp(timestamp);
    assert(!formatted.empty());
    
    auto start = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto end = std::chrono::high_resolution_clock::now();
    long long duration = duration_ms(start, end);
    assert(duration >= 5);  // 允许一些误差，实际睡眠时间可能略少于请求时间
    
    std::cout << "Time utilities tests passed!" << std::endl;
    
    // 测试容器工具函数
    std::cout << "Testing container utilities..." << std::endl;
    
    std::vector<int> vec = {1, 2, 3, 4, 5};
    
    // 测试contains函数
    bool has_three = contains(vec, 3);
    assert(has_three == true);
    
    bool has_six = contains(vec, 6);
    assert(has_six == false);
    
    // 测试filter函数
    auto filtered = filter(vec, [](int x) { return x > 3; });
    assert(filtered.size() == 2);
    assert(filtered[0] == 4);
    assert(filtered[1] == 5);
    
    // 测试map函数
    auto mapped = map(vec, [](int x) { return x * 2; });
    assert(mapped.size() == 5);
    assert(mapped[0] == 2);
    assert(mapped[1] == 4);
    assert(mapped[2] == 6);
    assert(mapped[3] == 8);
    assert(mapped[4] == 10);
    
    std::cout << "Container utilities tests passed!" << std::endl;
    
    std::cout << "All tests passed!" << std::endl;
    
    return 0;
}