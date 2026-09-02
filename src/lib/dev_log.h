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
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

// Define DEV_LOG_ENABLE in your project settings or uncomment the line below
#define DEV_LOG_ENABLE 1

#ifdef DEV_LOG_ENABLE

    #ifdef ARDUINO
    #include <Arduino.h>
    #endif

    // =====================================================================
    // ISR-safety guard. USBSerial::write() spins on host back-pressure
    // while the CDC port is connected (its TX queue is full and the host
    // is not draining it). Called from the 20 kHz control ISR with the
    // motor energized, that freeze latches the last voltage vector on
    // the bridge: loud electrical noise and corrupted calibration runs
    // - exactly what happens right after flashing, while USB CDC is
    // still re-enumerating. Every print therefore checks the free TX
    // room first and is DROPPED instead of blocking when it is low.
    // =====================================================================

    /** Free-TX-room threshold below which log output is dropped. */
    constexpr size_t TP_LOG_TX_MARGIN = 32;

    /**
     * @brief True when at least @p bytes can be queued without blocking.
     * @param bytes Number of bytes the caller is about to print.
     *
     * The queue occupancy is also mutated from the USB interrupt; the
     * lock-free SPSC design makes this read slightly stale at worst,
     * which is acceptable for a drop/no-drop threshold.
     */
    inline bool tp_log_room(size_t bytes) {
        return SerialUSB.availableForWrite() > static_cast<int>(TP_LOG_TX_MARGIN + bytes);
    }

    /** Print @p msg as a complete line over SerialUSB. */
    #define TP_LOG(msg) \
        do { if (tp_log_room(sizeof(msg))) SerialUSB.println(msg); } while (0)

    /**
     * Print @p msg (no newline) followed by @p val and a newline.
     * Wrapped in do/while(0) so it behaves as a single statement.
     */
    #define TP_LOG_VAL(msg, val) \
        do { \
            if (tp_log_room(sizeof(msg))) SerialUSB.print(msg); \
            if (tp_log_room(16)) SerialUSB.println(val); \
        } while (0)

    /**
     * Helper template that automatically deduces the size N of a static array
     * passed by reference and prints its elements comma-separated.
     * Output is dropped element-wise when the CDC TX queue runs low, so the
     * call can never block the control ISR on host back-pressure.
     */
    template <typename T, size_t N>
    inline void _tp_print_array(const T (&arr)[N]) {
        for (size_t i = 0; i < N; ++i) {
            if (!tp_log_room(12)) return;  // TX queue nearly full: drop the tail
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
    /// Output is dropped element-wise when the CDC TX queue runs low, so the
    /// call can never block the control ISR on host back-pressure.
    template<typename T, typename... Args>
    inline void _tp_print_args(T first, Args... args) {
        if (!tp_log_room(12)) return;  // TX queue nearly full: drop the tail
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
