#include <iostream>
#include <cstdarg>
#include <string>
#include "../include/bas_operate/file_operate.hpp"

using namespace basmodule;

int main() {
    // 测试各种格式化字符串
    std::cout << "Testing format specifier detection:\n";
    
    // 测试包含 %zu 的字符串
    std::cout << "Test 1 - \"%zu\": " << (contains_format_specifier("%zu") ? "PASS" : "FAIL") << std::endl;
    std::cout << "Test 2 - \"找到/install位置: %zu\\n\": " << (contains_format_specifier("找到/install位置: %zu\n") ? "PASS" : "FAIL") << std::endl;
    
    // 测试其他带修饰符的格式化字符串
    std::cout << "Test 3 - \"%zd\": " << (contains_format_specifier("%zd") ? "PASS" : "FAIL") << std::endl;
    std::cout << "Test 4 - \"%zx\": " << (contains_format_specifier("%zx") ? "PASS" : "FAIL") << std::endl;
    std::cout << "Test 5 - \"%td\": " << (contains_format_specifier("%td") ? "PASS" : "FAIL") << std::endl;
    std::cout << "Test 6 - \"%llx\": " << (contains_format_specifier("%llx") ? "PASS" : "FAIL") << std::endl;
    std::cout << "Test 7 - \"%hd\": " << (contains_format_specifier("%hd") ? "PASS" : "FAIL") << std::endl;
    std::cout << "Test 8 - \"%hhd\": " << (contains_format_specifier("%hhd") ? "PASS" : "FAIL") << std::endl;
    
    // 测试普通的格式化字符串
    std::cout << "Test 9 - \"%d\": " << (contains_format_specifier("%d") ? "PASS" : "FAIL") << std::endl;
    std::cout << "Test 10 - \"%s\": " << (contains_format_specifier("%s") ? "PASS" : "FAIL") << std::endl;
    
    // 测试非格式化字符串
    std::cout << "Test 11 - \"hello world\": " << (contains_format_specifier("hello world") ? "PASS" : "FAIL") << std::endl;
    std::cout << "Test 12 - \"100%\": " << (contains_format_specifier("100%") ? "PASS" : "FAIL") << std::endl;
    
    return 0;
}