#ifndef DEBUG_TOOLS
#define DEBUG_TOOLS

#include <chrono>
#include <iostream>

#define DEBUG

#ifdef DEBUG
    #define LOG_DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
    #define START_TIMER(tag) \
        std::chrono::steady_clock::time_point start_##tag = std::chrono::steady_clock::now()

    // Calcola e stampa la differenza di tempo
    #define STOP_TIMER(tag, name) \
        do { \
            std::chrono::steady_clock::time_point end_##tag = std::chrono::steady_clock::now(); \
            double duration_##tag = std::chrono::duration<double, std::milli>(end_##tag - start_##tag).count(); \
            std::cout << "[TIMER] " << name << ": " << duration_##tag << " ms" << std::endl; \
        } while(0)
#else
    #define LOG_DEBUG(msg) ((void)0)
    #define START_TIMER(tag) ((void)0)
    #define STOP_TIMER(tag, name) ((void)0)
#endif

#endif