/**
 * @file led_driver.h
 * @brief Two-element RGB status LED driver with a fluent set() write API.
 *
 * The driver keeps a shadow color word (packed RGB888: 0x00RRGGBB) for each of
 * the TWO tricolor LED elements and pushes both words to the peripheral
 * through ONE registered apply callback. The callback is implemented by the
 * board/target layer (like every other TP driver callback), so the driver
 * itself is fully platform-independent:
 *
 *      typedef void (*LedApplyCallback)(int32_t led1, int32_t led2);
 *
 * This replaces the legacy scheme of three separate boolean callbacks (one
 * per color channel): a board now decodes two packed words in a single
 * function, which maps naturally onto addressable RGB pixels (WS2812) as well
 * as onto discrete R/G/B pins (a board with a single tricolor element simply
 * ignores the second word).
 *
 * The fluent interface covers every write combination in one chained
 * statement:
 *
 *      led.set().led(r1, g1, b1, r2, g2, b2);  // both LEDs, independent colors
 *      led.set().led(r, g, b);                 // both LEDs, same color
 *      led.set().led1(r, g, b).led2(r, g, b);  // per-LED full colors
 *      led.set().led1().r(x).g(y).b(z);        // per-channel writes on LED1
 *      led.set().led1(r, g, b).led2().r(x);    // mixed intermediate variants
 *
 * Every mutating operation re-applies BOTH shadow words through the callback,
 * so the peripheral state always matches the shadow exactly (idempotent,
 * safe to call from any context as long as the callback itself is).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include <stdint.h>

/** Signature of the peripheral apply callback: two packed RGB888 words (one per LED element). */
typedef void (*LedApplyCallback)(int32_t led1, int32_t led2);

/**
 * @struct LedDriver
 * @brief Shadow colors of two RGB elements + a single apply callback.
 */
struct LedDriver {
private:
    /// 8-bit width of one color channel inside the packed word.
    static constexpr int32_t kChannelMask = 0xFF;
    /// Bit offset of the red channel (0x00RRGGBB packing).
    static constexpr int kShiftR = 16;
    /// Bit offset of the green channel (0x00RRGGBB packing).
    static constexpr int kShiftG = 8;
    /// Bit offset of the blue channel (0x00RRGGBB packing).
    static constexpr int kShiftB = 0;

    LedApplyCallback apply_ = nullptr;  ///< Peripheral apply callback (nullptr = silent driver).
    int32_t color_[2] = {0, 0};         ///< Shadow words (packed RGB888), one per LED element.

    /** Pack three channels into one RGB888 word (each channel masked to 8 bits). */
    static constexpr int32_t pack_(int32_t r, int32_t g, int32_t b) {
        return ((r & kChannelMask) << kShiftR) |  // Red into bits [23:16]
               ((g & kChannelMask) << kShiftG) |  // Green into bits [15:8]
               ((b & kChannelMask) << kShiftB);   // Blue into bits [7:0]
    }

    /** Extract one 8-bit channel from a packed word. */
    static constexpr int32_t channel_(int32_t word, int shift) {
        return (word >> shift) & kChannelMask;  // Isolate the requested channel
    }

    /**
     * @brief Overwrite one channel of the selected LED and re-apply both words.
     * @param idx   LED element index (0/1).
     * @param shift Bit offset of the channel being written.
     * @param value New channel value (masked to 8 bits).
     */
    inline void write_channel_(uint8_t idx, int shift, int32_t value) {
        const int32_t keep = ~(kChannelMask << shift);  // Mask preserving the other channels
        color_[idx] = (color_[idx] & keep) | ((value & kChannelMask) << shift);  // Merge in place
        apply();                                        // Push the fresh pair to the peripheral
    }

public:
    /**
     * @struct Setter
     * @brief Fluent proxy returned by set(); remembers the currently selected LED.
     */
    struct Setter {
    private:
        LedDriver& parent_;   ///< Owning driver instance.
        uint8_t select_ = 0;  ///< LED element (0/1) targeted by subsequent channel writes.

        friend struct LedDriver;  // Grants set() access to the private constructor

        explicit Setter(LedDriver& parent) : parent_(parent) {}  // Only LedDriver constructs proxies

    public:
        /// Both LEDs with independent colors (applies immediately). Chainable.
        inline Setter& led(int32_t r1, int32_t g1, int32_t b1, int32_t r2, int32_t g2, int32_t b2) {
            parent_.color_[0] = pack_(r1, g1, b1);  // LED1 shadow word
            parent_.color_[1] = pack_(r2, g2, b2);  // LED2 shadow word
            parent_.apply();                        // One callback for the whole pair
            return *this;
        }

        /// Both LEDs with the same color (applies immediately). Chainable.
        inline Setter& led(int32_t r, int32_t g, int32_t b) {
            return led(r, g, b, r, g, b);  // Mirror the color onto both elements
        }

        /// Full color on LED1; subsequent channel writes target LED1. Chainable.
        inline Setter& led1(int32_t r, int32_t g, int32_t b) {
            parent_.color_[0] = pack_(r, g, b);  // LED1 shadow word
            select_ = 0;                         // Channel writes now hit LED1
            parent_.apply();                     // Push the fresh pair out
            return *this;
        }

        /// Full color on LED2; subsequent channel writes target LED2. Chainable.
        inline Setter& led2(int32_t r, int32_t g, int32_t b) {
            parent_.color_[1] = pack_(r, g, b);  // LED2 shadow word
            select_ = 1;                         // Channel writes now hit LED2
            parent_.apply();                     // Push the fresh pair out
            return *this;
        }

        /// Select LED1 for subsequent .r()/.g()/.b() writes (no apply). Chainable.
        inline Setter& led1() {
            select_ = 0;  // Only retarget, shadow untouched
            return *this;
        }

        /// Select LED2 for subsequent .r()/.g()/.b() writes (no apply). Chainable.
        inline Setter& led2() {
            select_ = 1;  // Only retarget, shadow untouched
            return *this;
        }

        /// Red channel of the selected LED (applies). Chainable.
        inline Setter& r(int32_t value) {
            parent_.write_channel_(select_, kShiftR, value);  // Merge into bits [23:16]
            return *this;
        }

        /// Green channel of the selected LED (applies). Chainable.
        inline Setter& g(int32_t value) {
            parent_.write_channel_(select_, kShiftG, value);  // Merge into bits [15:8]
            return *this;
        }

        /// Blue channel of the selected LED (applies). Chainable.
        inline Setter& b(int32_t value) {
            parent_.write_channel_(select_, kShiftB, value);  // Merge into bits [7:0]
            return *this;
        }
    };

    /// Entry point of the fluent write API (channel writes default to LED1).
    inline Setter set() { return Setter(*this); }

    /** Fluent proxy for registering the apply callback. */
    struct CallbackSetter {
    private:
        LedDriver& parent_;  ///< Owning driver instance.

        friend struct LedDriver;  // Grants set_callback() access to the private constructor

        explicit CallbackSetter(LedDriver& parent) : parent_(parent) {}  // Only LedDriver constructs proxies

    public:
        /// Register the apply callback (nullptr restores the silent driver). Chainable.
        inline CallbackSetter& apply_cb(LedApplyCallback cb) {
            parent_.apply_ = cb;  // Store the peripheral writer
            return *this;
        }
    };

    /// Entry point of the fluent callback registration API.
    inline CallbackSetter set_callback() { return CallbackSetter(*this); }

    /** Push both shadow words through the apply callback (no-op if unset). */
    inline void apply() {
        if (apply_) apply_(color_[0], color_[1]);  // Two words, one transaction
    }

    /** Turn BOTH elements dark (zero words) and apply. */
    inline void off() {
        color_[0] = color_[1] = 0;  // Clear both shadows
        apply();                    // Single callback for the pair
    }

    /// Packed RGB888 word of LED element @p idx (0/1, index masked to 1 bit).
    inline int32_t color(uint8_t idx) const { return color_[idx & 1]; }

    /// Red channel of LED element @p idx (0..255).
    inline int32_t r(uint8_t idx) const { return channel_(color_[idx & 1], kShiftR); }

    /// Green channel of LED element @p idx (0..255).
    inline int32_t g(uint8_t idx) const { return channel_(color_[idx & 1], kShiftG); }

    /// Blue channel of LED element @p idx (0..255).
    inline int32_t b(uint8_t idx) const { return channel_(color_[idx & 1], kShiftB); }
};