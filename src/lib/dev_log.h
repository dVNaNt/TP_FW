#pragma once

/**
 * @file dev_log.h
 * @brief Lightweight compile-time switchable logging macros over SerialUSB.
 *
 * Provides four macros for developer diagnostics:
 * - TP_LOG(msg)              : print a plain message line.
 * - TP_LOG_VAL(msg, val)     : print a message followed by one value.
 * - TP_LOG_ARRAY(msg, arr)   : print a message followed by a static array
 *                              (size auto-detected via template deduction).
 * - TP_LOG_VALS(msg, ...)    : print a message followed by any number of
 *                              comma-separated values (variadic templates).
 *
 * All output goes to SerialUSB. When DEV_LOG_ENABLE is not defined, every
 * macro expands to nothing and the compiler removes the call sites entirely,
 * so logging has zero cost in release builds.
 */

// Define DEV_LOG_ENABLE in your project settings or uncomment the line below
#define DEV_LOG_ENABLE 1

#ifdef DEV_LOG_ENABLE

    #ifdef ARDUINO
    #include <Arduino.h>
    #endif

    /** Print @p msg as a complete line over SerialUSB. */
    #define TP_LOG(msg) SerialUSB.println(msg)

    /**
     * Print @p msg (no newline) followed by @p val and a newline.
     * Wrapped in do/while(0) so it behaves as a single statement.
     */
    #define TP_LOG_VAL(msg, val) \
        do { \
            SerialUSB.print(msg); \
            SerialUSB.println(val); \
        } while (0)

    /**
     * Helper template that automatically deduces the size N of a static array
     * passed by reference and prints its elements comma-separated.
     */
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

    /** Print @p msg followed by all elements of static array @p arr, comma-separated. */
    #define TP_LOG_ARRAY(msg, arr) \
        do { \
            SerialUSB.print(msg); \
            _tp_print_array(arr); \
        } while (0)

    // =================================================================
    // 4. Macro for logging a message and ANY number of values
    // =================================================================
    
    /// Base case that terminates variadic recursion: emits the final newline.
    inline void _tp_print_args() { 
        SerialUSB.println(); 
    }

    /// Variadic template that recursively prints comma-separated arguments.
    template<typename T, typename... Args>
    inline void _tp_print_args(T first, Args... args) {
        SerialUSB.print(first);
        if (sizeof...(args) > 0) {
            SerialUSB.print(", ");
        }
        _tp_print_args(args...);
    }

    /** Print @p msg followed by any number of comma-separated values. */
    #define TP_LOG_VALS(msg, ...) \
        do { \
            SerialUSB.print(msg); \
            _tp_print_args(__VA_ARGS__); \
        } while (0)

#else
    // If DEV_LOG_ENABLE is not defined, all macros evaluate to nothing.
    // The compiler will optimize away the logging code completely.
    // TP_LOG        - disabled, expands to nothing.
    // TP_LOG_VAL    - disabled, expands to nothing.
    // TP_LOG_ARRAY  - disabled, expands to nothing.
    // TP_LOG_VALS   - disabled, expands to nothing.
    #define TP_LOG(msg)
    #define TP_LOG_VAL(msg, val)
    #define TP_LOG_ARRAY(msg, arr)
    #define TP_LOG_VALS(msg, ...)
#endif
