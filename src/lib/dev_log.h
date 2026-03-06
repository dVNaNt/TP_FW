#pragma once

// Define DEV_LOG_ENABLE in your project settings or uncomment the line below
#define DEV_LOG_ENABLE 1

#ifdef DEV_LOG_ENABLE

    #ifdef ARDUINO
    #include <Arduino.h>
    #endif

    // 1. Macro for logging a simple message
    #define TP_LOG(msg) SerialUSB.println(msg)

    // 2. Macro for logging a message and a single value
    #define TP_LOG_VAL(msg, val) \
        do { \
            SerialUSB.print(msg); \
            SerialUSB.println(val); \
        } while (0)

    // Helper template to automatically deduce the size of a static array
    template <typename T, size_t N>
    inline void _tp_print_array(const T (&arr)[N]) {
        for (size_t i = 0; i < N; ++i) {
            SerialUSB.print(arr[i]);
            if (i < N - 1) {
                SerialUSB.print(", ");
            }
        }
        SerialUSB.println();
    }

    // 3. Macro for logging a message and a static array (size is auto-detected)
    #define TP_LOG_ARRAY(msg, arr) \
        do { \
            SerialUSB.print(msg); \
            _tp_print_array(arr); \
        } while (0)

    // =================================================================
    // 4. Macro for logging a message and ANY number of values
    // =================================================================
    
    // Base case to stop template recursion
    inline void _tp_print_args() { 
        SerialUSB.println(); 
    }

    // Variadic template function to recursively print comma-separated arguments
    template<typename T, typename... Args>
    inline void _tp_print_args(T first, Args... args) {
        SerialUSB.print(first);
        if (sizeof...(args) > 0) {
            SerialUSB.print(", ");
        }
        _tp_print_args(args...);
    }

    // Wrapper macro for variadic arguments
    #define TP_LOG_VALS(msg, ...) \
        do { \
            SerialUSB.print(msg); \
            _tp_print_args(__VA_ARGS__); \
        } while (0)

#else
    // If DEV_LOG_ENABLE is not defined, all macros evaluate to nothing.
    // The compiler will optimize away the logging code completely.
    #define TP_LOG(msg)
    #define TP_LOG_VAL(msg, val)
    #define TP_LOG_ARRAY(msg, arr)
    #define TP_LOG_VALS(msg, ...)
#endif
