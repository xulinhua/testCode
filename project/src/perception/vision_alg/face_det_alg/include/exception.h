#pragma once

#include <stdexcept>
#include <string>

class TensorRTException : public std::runtime_error 
{
public:
    explicit TensorRTException(const std::string& msg) 
        : std::runtime_error("[TensorRT-PPOCRv5] " + msg) {}
    
    explicit TensorRTException(const char* msg) 
        : std::runtime_error(std::string("[TensorRT-PPOCRv5] ") + msg) {}
};