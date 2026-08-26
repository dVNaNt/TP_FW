#pragma once

/**
 * @file input.h
 * @brief Minimal callback wrapper for a polled digital/analog input source.
 *
 * InputWrapper stores a single function pointer returning an int8_t sample.
 * When no callback is registered the wrapper falls back to a built-in
 * "null" provider that always returns -1, so check() never dereferences a
 * null pointer and needs no branch on the hot path.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <stdbool.h>
#include <stdint.h>

/** Signature of an input sampling callback: returns one int8_t sample. */
typedef int8_t (*InputGetCb)(void);

/**
 * @brief Polled input abstraction over a single getter callback.
 */
class InputWrapper {
private:
    /// Built-in fallback provider: represents "no input connected".
    static inline int8_t get_null() { return -1; }
    InputGetCb getCb_;  ///< Currently registered getter (never null).

public:
    /// Default constructor: binds the null getter (-1).
    InputWrapper() : getCb_(get_null) {}

    /**
     * @brief Register an input getter callback.
     * @param cb Getter function; nullptr restores the built-in null getter.
     * @return Reference to this wrapper for call-site chaining.
     */
    InputWrapper& setCallback(InputGetCb cb) {
        getCb_ = cb ? cb : get_null;
        return *this;
    }

    /**
     * @brief Poll the input once.
     * @return The value returned by the registered getter, or -1 if unset.
     */
    inline int8_t check() const { return getCb_(); }
};