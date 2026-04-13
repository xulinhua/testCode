#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include "NvInfer.h"

class Logger : public nvinfer1::ILogger
{
public:
    explicit Logger(const std::string& filename = "", bool verbose = true)
        : verbose_(verbose), log_file_(filename)
    {
        if (!filename.empty()) {
            file_stream_.open(filename, std::ios::app);
        }
    }

    template<typename... Args>
    void log(const std::string& level, const Args&... args)
    {
        std::ostringstream oss;
        oss << "[" << level << "] ";
        ((oss << args), ...);
        oss << std::endl;

        if (verbose_) {
            std::cout << oss.str();
        }

        if (file_stream_.is_open()) {
            file_stream_ << oss.str();
        }
    }

    template<typename... Args>
    void info(const Args&... args) { log("INFO", args...); }

    template<typename... Args>
    void warn(const Args&... args) { log("WARN", args...); }

    template<typename... Args>
    void error(const Args&... args) { log("ERROR", args...); }

    template<typename... Args>
    void debug(const Args&... args) { log("DEBUG", args...); }

    // TensorRT ILogger interface
    void log(Severity severity, const char* msg) noexcept override
    {
        std::string level;
        switch (severity) {
            case Severity::kINTERNAL_ERROR: level = "INTERNAL_ERROR"; break;
            case Severity::kERROR: level = "ERROR"; break;
            case Severity::kWARNING: level = "WARNING"; break;
            case Severity::kINFO: level = "INFO"; break;
            case Severity::kVERBOSE: level = "VERBOSE"; break;
            default: level = "UNKNOWN"; break;
        }
        log(level, msg);
    }

    ~Logger()
    {
        if (file_stream_.is_open()) {
            file_stream_.close();
        }
    }

private:
    bool verbose_;
    std::string log_file_;
    std::ofstream file_stream_;
};
