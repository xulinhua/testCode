#pragma once

#include <stdexcept>
#include <string>

class TensorRTException : public std::runtime_error 
{
public:
    explicit TensorRTException(const std::string& msg) 
        : std::runtime_error("[TensorRT-PPOCR] " + msg) {}
    
    explicit TensorRTException(const char* msg) 
        : std::runtime_error(std::string("[TensorRT-PPOCR] ") + msg) {}
};