#pragma once

#include <cuda_runtime.h>
#define USE_TENSORRT_8 0
class Logger : public nvinfer1::ILogger 
{
public:
    void log(Severity severity, const char* msg) noexcept override 
    {
        if (severity <= Severity::kERROR) 
        {
            std::cerr << "[TensorRT] " << msg << std::endl;
        }
    }
};
