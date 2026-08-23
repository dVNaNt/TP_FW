#ifndef OUTPUT_WRAPPER_HPP
#define OUTPUT_WRAPPER_HPP

/**
 * @file output.h
 * @brief Minimal callback wrapper for a binary output (LED, pin, relay...).
 *
 * OutputWrapper stores one function pointer taking a bool state. Unlike
 * InputWrapper, no default no-op is installed; set() performs a null check
 * before invoking, so an unconfigured wrapper is simply silent.
 */

#include <stdbool.h>

/** Signature of an output-driving callback: receives the requested state. */
typedef void (*OutputSetCb)(bool state);

/**
 * @brief Binary output abstraction over a single setter callback.
 */
class OutputWrapper {
private:
    OutputSetCb setCb_;  ///< Registered setter callback (nullptr = unconfigured).

public:
    /// Default constructor: no callback registered.
    OutputWrapper() : setCb_(nullptr) {}

    /**
     * @brief Register the output setter callback.
     * @param cb Setter function; nullptr disables the output.
     */
    void setCallback(OutputSetCb cb) {
        setCb_ = cb;
    }

    /**
     * @brief Drive the output to the requested state.
     * @param state Desired output state (polarity is defined by the callback).
     * @note Does nothing if no callback has been registered.
     */
    inline void set(bool state) const {
        if (setCb_) setCb_(state);
    }
};

#endif // OUTPUT_WRAPPER_HPP