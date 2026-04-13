#pragma once

// 平台相关的定义和实现

#ifdef _WIN32
    // Windows平台相关定义
    #include <windows.h>
    
    // Windows临界区定义
    typedef CRITICAL_SECTION platform_critical_section;
    
    // Windows临界区操作函数
    #define PLATFORM_INIT_CRITICAL_SECTION(cs) InitializeCriticalSection(&cs)
    #define PLATFORM_DELETE_CRITICAL_SECTION(cs) DeleteCriticalSection(&cs)
    #define PLATFORM_ENTER_CRITICAL_SECTION(cs) EnterCriticalSection(&cs)
    #define PLATFORM_LEAVE_CRITICAL_SECTION(cs) LeaveCriticalSection(&cs)
    
    // Windows动态库导出定义
    #ifdef HSV_CUSTRCT_EXPORTS
        #define HSV_CUSTRCT_API __declspec(dllexport)
    #else
        #define HSV_CUSTRCT_API __declspec(dllimport)
    #endif
    
    #ifdef HS_DRAW_OBJ_API_EXPORTS
        #define HS_DRAW_OBJ_API __declspec(dllexport)
    #else
        #define HS_DRAW_OBJ_API __declspec(dllimport)
    #endif

#elif __linux__
    // Linux平台相关定义
    #include <pthread.h>
#include <unistd.h>
    // Linux互斥锁定义
    typedef pthread_mutex_t platform_critical_section;
    
    // Linux互斥锁操作函数
    #define PLATFORM_INIT_CRITICAL_SECTION(cs) pthread_mutex_init(&cs, NULL)
    #define PLATFORM_DELETE_CRITICAL_SECTION(cs) pthread_mutex_destroy(&cs)
    #define PLATFORM_ENTER_CRITICAL_SECTION(cs) pthread_mutex_lock(&cs)
    #define PLATFORM_LEAVE_CRITICAL_SECTION(cs) pthread_mutex_unlock(&cs)
    
    // Linux动态库导出定义
    #ifdef HSV_CUSTRCT_EXPORTS
        #define HSV_CUSTRCT_API __attribute__((visibility("default")))
    #else
        #define HSV_CUSTRCT_API
    #endif
    #define HS_DRAW_OBJ_API __attribute__((visibility("default")))

#else
    #error "Unsupported platform"
#endif