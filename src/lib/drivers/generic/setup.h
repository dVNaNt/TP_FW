#ifndef GENERIC_SETUP_H
#define GENERIC_SETUP_H

/**
 * @file setup.h
 * @brief Generic lifecycle-callback container with zero-overhead dispatch.
 *
 * Setup stores five optional lifecycle callbacks (init / start / stop /
 * deinit / reset) and invokes them through direct function pointers. To avoid
 * a branch on every invocation, unset slots default to an empty inline
 * no-op function, which the compiler eliminates entirely at -O1 and above.
 *
 * Callbacks are registered fluently:
 * @code
 *   setup.set_callback().init(cb_init).start(cb_start);
 * @endcode
 * Passing nullptr to any registration resets that slot back to the no-op.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <stdint.h>

/** Signature of a parameterless lifecycle action callback. */
typedef void (*ActionCb)(void);

/// Empty inline function; the compiler removes its call entirely.
inline void action_noop() {}

/**
 * @brief Lifecycle callback holder (init/start/stop/deinit/reset).
 */
class Setup {
private:
    // Callbacks default to the no-op so call sites never need if(cb_) checks.
    ActionCb cb_init_   = action_noop;  ///< Invoked by init().
    ActionCb cb_start_  = action_noop;  ///< Invoked by start().
    ActionCb cb_stop_   = action_noop;  ///< Invoked by stop().
    ActionCb cb_deinit_ = action_noop;  ///< Invoked by deinit().
    ActionCb cb_reset_  = action_noop;  ///< Invoked by reset().

    /**
     * Private proxy object returned by set_callback(); enables chained
     * registration while keeping the owner's fields private.
     */
    class CallbackProxy {
        Setup& owner_;  ///< The Setup instance whose callbacks are edited.
    public:
        inline CallbackProxy(Setup& owner) : owner_(owner) {}

        /// Register the init callback (nullptr restores the no-op). Chainable.
        inline CallbackProxy& init(ActionCb cb)   { owner_.cb_init_   = cb ? cb : action_noop; return *this; }
        /// Register the start callback (nullptr restores the no-op). Chainable.
        inline CallbackProxy& start(ActionCb cb)  { owner_.cb_start_  = cb ? cb : action_noop; return *this; }
        /// Register the stop callback (nullptr restores the no-op). Chainable.
        inline CallbackProxy& stop(ActionCb cb)   { owner_.cb_stop_   = cb ? cb : action_noop; return *this; }
        /// Register the deinit callback (nullptr restores the no-op). Chainable.
        inline CallbackProxy& deinit(ActionCb cb) { owner_.cb_deinit_ = cb ? cb : action_noop; return *this; }
        /// Register the reset callback (nullptr restores the no-op). Chainable.
        inline CallbackProxy& reset(ActionCb cb)  { owner_.cb_reset_  = cb ? cb : action_noop; return *this; }
    };

public:
    /// Default constructor: all callbacks point to the no-op.
    constexpr Setup() {}

    // --- Execution methods (zero-overhead) ---
    // No if(cb_) checks: a direct call to the user function or to the
    // removable no-op.
    /// Run the registered init callback (no-op if none).
    inline void init() const   { cb_init_(); }
    /// Run the registered start callback (no-op if none).
    inline void start() const  { cb_start_(); }
    /// Run the registered stop callback (no-op if none).
    inline void stop() const   { cb_stop_(); }
    /// Run the registered deinit callback (no-op if none).
    inline void deinit() const { cb_deinit_(); }
    /// Run the registered reset callback (no-op if none).
    inline void reset() const  { cb_reset_(); }

    // --- Callback registration sub-object ---
    // Returns a temporary proxy that exists only for the duration of the
    // assignment expression used to chain registrations.
    inline CallbackProxy set_callback() {
        return CallbackProxy(*this);
    }
};

#endif // GENERIC_SETUP_H