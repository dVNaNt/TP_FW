/**
 * @file pwm_driver.h
 * @brief Generic, high-performance PWM motor driver with integrated Current Sense,
 *        Voltage Sense, and unified fault management.
 *
 * @details
 * PhyDriverPWM is the top-level driver structure that owns and coordinates:
 * - **CurrentSensePHY** — 3-channel current sensing with overcurrent protection
 * - **SupplyVoltagePHY** — filtered voltage sensing with OV/UV protection
 * - **TemperaturePHY** — heavily-filtered temperature sensing with warning (WT)
 *   and overtemperature (OT) thresholds and immediate fault release
 * - **PwmWrapper** — hardware PWM output with chainable setup and SeqLock-protected duty cycle
 * - **OutputWrapper** — gate driver enable/disable
 * - **InputWrapper** — external hardware fault pin
 *
 * ## Fault Management Architecture
 *
 * The driver maintains a unified fault bitfield (@ref fault_flags_) with five fault sources:
 *
 * | Bit | Fault         | Safe State | Recovery Timer  |
 * |-----|---------------|------------|-----------------|
 * | 0   | Overvoltage   | LowAll     | VoltageProtectionProxy |
 * | 1   | Overcurrent   | HiZ        | OvercurrentProxy |
 * | 2   | Overheat      | HiZ        | None (manual) |
 * | 3   | External      | HiZ        | None (manual) |
 * | 4   | Undervoltage  | HiZ        | VoltageProtectionProxy |
 *
 * **Priority**: EXT (5) > OV (4) > UV (3) > OC (2) > OT (1). The highest-priority
 * active fault determines which callback fires. When all faults clear, the restore
 * callback fires.
 *
 * ## Thread Safety
 *
 * - PWM duty cycle updates use a SeqLock pattern inside PwmWrapper (ISR-safe writer, lock-free reader)
 * - Fault flags are atomic bitwise operations
 * - Active priority tracking uses compare_exchange to prevent race conditions
 *   between tick() and ISR-triggered fault changes
 *
 * ## Fluent API Examples
 *
 * PWM setup:
 * @code
 *   driver.pwm.set_period(7200).set_callback()
 *       .phases(hal_set_compare)
 *       .hiZ(hal_hiz)
 *       .lowAll(hal_low_all);
 * @endcode
 *
 * Fault callback configuration:
 * @code
 *   driver.set_fault().set_callback()
 *       .oc(overcurrent_handler)
 *       .ov(overvoltage_handler)
 *       .restore(restore_handler);
 * @endcode
 *
 * Runtime duty cycle:
 * @code
 *   driver.pwm.set_duty(duty_q15);
 * @endcode
 *
 * @warning When any fault is active, tick_icr() skips PWM application entirely
 *          (the outputs remain in the hardware's safe-state configuration).
 *
 * @copyright Copyright 2026, creapunk
 * @license Apache-2.0
 */

#ifndef DRIVER_PWM_H
#define DRIVER_PWM_H

#include <limits.h>
#include <stdint.h>
#include <atomic>

#include "../generic/input.h"
#include "../generic/output.h"
#include "../generic/setup.h"
#include "current.h"
#include "motor_control.h"
#include "pwm.h"
#include "temperature.h"
#include "voltage.h"

/**
 * @struct PhyDriverPWM
 * @brief Generic PWM driver with built-in protection logic and fluent fault API.
 *
 * This is the primary driver object users instantiate. It aggregates current
 * sensing, voltage sensing, PWM output, and fault management into a single
 * cohesive interface.
 */
struct PhyDriverPWM {
    friend struct CurrentSensePHY;
    friend struct SupplyVoltagePHY;
    friend struct MotorControlPHY;
    friend struct TemperaturePHY;

private:
    std::atomic<uint8_t> fault_flags_ = 0;  ///< Bitfield holding current active faults

    static constexpr uint8_t FAULT_OVERVOLTAGE = (1 << 0);   ///< Bit 0: overvoltage → forces LowAll (brake)
    static constexpr uint8_t FAULT_OVERCURRENT = (1 << 1);   ///< Bit 1: overcurrent → forces HiZ
    static constexpr uint8_t FAULT_OVERHEAT = (1 << 2);      ///< Bit 2: overheat → forces HiZ
    static constexpr uint8_t FAULT_EXTERNAL = (1 << 3);      ///< Bit 3: external pin → forces HiZ (highest priority)
    static constexpr uint8_t FAULT_UNDERVOLTAGE = (1 << 4);  ///< Bit 4: undervoltage → forces HiZ

    /// All faults that force HiZ safe state (does NOT include OVERVOLTAGE which forces LowAll)
    static constexpr uint8_t FAULT_HIZ_MASK = (FAULT_OVERCURRENT | FAULT_OVERHEAT | FAULT_EXTERNAL | FAULT_UNDERVOLTAGE);
    /// All faults that force any safe state (HiZ or LowAll)
    static constexpr uint8_t FAULT_SAFE_MASK = (FAULT_OVERVOLTAGE | FAULT_HIZ_MASK);

    int16_t pwm_apply_[3] = {0, 0, 0};  ///< Local array for applying PWM to hardware

    void (*cb_oc_)() = nullptr;       ///< Overcurrent fault callback
    void (*cb_ov_)() = nullptr;       ///< Overvoltage fault callback
    void (*cb_uv_)() = nullptr;       ///< Undervoltage fault callback
    void (*cb_ot_)() = nullptr;       ///< Overheat fault callback
    void (*cb_ext_)() = nullptr;      ///< External fault callback
    void (*cb_restore_)() = nullptr;  ///< Restore callback (fires when all faults clear)

    std::atomic<uint8_t> active_priority_ = 0;  ///< Tracks the highest active priority for callback logic

    /**
     * @brief Get the priority of the highest active fault.
     *
     * Priority encoding (higher number = higher priority):
     * - 5: External hardware fault
     * - 4: Overvoltage
     * - 3: Undervoltage
     * - 2: Overcurrent
     * - 1: Overheat
     * - 0: No fault active
     *
     * @param flags Current fault flags bitfield
     * @return Priority level (0..5)
     */
    inline uint8_t get_top_priority(uint8_t flags) const {
        if (flags & FAULT_EXTERNAL)
            return 5;
        if (flags & FAULT_OVERVOLTAGE)
            return 4;
        if (flags & FAULT_UNDERVOLTAGE)
            return 3;
        if (flags & FAULT_OVERCURRENT)
            return 2;
        if (flags & FAULT_OVERHEAT)
            return 1;
        return 0;
    }

    /**
     * @brief Call the appropriate fault callback for a given priority level.
     *
     * Each callback is checked for nullptr before invocation — callbacks are
     * optional and may be left unset.
     *
     * @param priority Priority level (0=restore, 1=OT, 2=OC, 3=UV, 4=OV, 5=EXT)
     */
    inline void trigger_cb(uint8_t priority) {
        switch (priority) {
            case 5:
                if (cb_ext_)
                    cb_ext_();
                break;
            case 4:
                if (cb_ov_)
                    cb_ov_();
                break;
            case 3:
                if (cb_uv_)
                    cb_uv_();
                break;
            case 2:
                if (cb_oc_)
                    cb_oc_();
                break;
            case 1:
                if (cb_ot_)
                    cb_ot_();
                break;
            case 0:
                if (cb_restore_)
                    cb_restore_();
                break;
            default:
                break;
        }
    }

    /**
     * @brief Evaluate and trigger callback if a new higher-priority fault appeared.
     *
     * Uses compare_exchange to atomically promote the active priority only when
     * the new priority is strictly higher. This prevents race conditions between
     * tick() and ISR-triggered fault updates.
     *
     * @param new_flags The combined fault flags after the new fault was set
     */
    inline void check_and_trigger_priority(uint8_t new_flags) {
        uint8_t new_top = get_top_priority(new_flags);
        uint8_t expected = active_priority_.load(std::memory_order_relaxed);
        while (new_top > expected) {
            if (active_priority_.compare_exchange_weak(expected, new_top, std::memory_order_relaxed)) {
                trigger_cb(new_top);
                break;
            }
        }
    }

    /**
     * @brief Atomically set or clear a single fault bit without touching recovery timers.
     *
     * Used internally by tick_icr() to sync fault_flags_ with sub-system recovery
     * timer states. Does NOT invoke callbacks — that is handled by the priority manager.
     *
     * @param mask Fault bit mask (e.g., FAULT_OVERCURRENT)
     * @param active true to set the bit, false to clear it
     */
    inline void update_fault_bit(uint8_t mask, bool active) {
        if (active) {
            fault_flags_.fetch_or(mask, std::memory_order_relaxed);
        } else {
            fault_flags_.fetch_and(~mask, std::memory_order_relaxed);
        }
    }

    /**
     * @class CallbackProxy
     * @brief Zero-overhead proxy for setting fault callbacks via fluent syntax.
     *
     * Usage:
     * @code
     *   driver.set_fault().set_callback()
     *       .oc(overcurrent_handler)
     *       .ov(overvoltage_handler)
     *       .ext(external_fault_handler)
     *       .restore(normal_op_restored_handler);
     * @endcode
     *
     * @note All setter methods safely accept nullptr (equivalent to unsetting the callback).
     */
    class CallbackProxy {
        PhyDriverPWM& drv_;

    public:
        /**
         * @brief Construct a CallbackProxy bound to a driver instance.
         * @param drv Reference to the owning PhyDriverPWM
         */
        CallbackProxy(PhyDriverPWM& drv) : drv_(drv) {}

        /**
         * @brief Set overcurrent fault callback.
         * @param cb Function pointer (nullptr to unset)
         * @return Reference to this proxy for chaining
         */
        inline CallbackProxy& oc_cb(void (*cb)()) {
            drv_.cb_oc_ = cb;
            return *this;
        }
        /**
         * @brief Set overvoltage fault callback.
         * @param cb Function pointer (nullptr to unset)
         * @return Reference to this proxy for chaining
         */
        inline CallbackProxy& ov_cb(void (*cb)()) {
            drv_.cb_ov_ = cb;
            return *this;
        }
        /**
         * @brief Set undervoltage fault callback.
         * @param cb Function pointer (nullptr to unset)
         * @return Reference to this proxy for chaining
         */
        inline CallbackProxy& uv_cb(void (*cb)()) {
            drv_.cb_uv_ = cb;
            return *this;
        }
        /**
         * @brief Set overheat fault callback.
         * @param cb Function pointer (nullptr to unset)
         * @return Reference to this proxy for chaining
         */
        inline CallbackProxy& ot_cb(void (*cb)()) {
            drv_.cb_ot_ = cb;
            return *this;
        }
        /**
         * @brief Set external fault callback.
         * @param cb Function pointer (nullptr to unset)
         * @return Reference to this proxy for chaining
         */
        inline CallbackProxy& ext_cb(void (*cb)()) {
            drv_.cb_ext_ = cb;
            return *this;
        }
        /**
         * @brief Set restore callback (fired when all faults clear).
         * @param cb Function pointer (nullptr to unset)
         * @return Reference to this proxy for chaining
         */
        inline CallbackProxy& rst_cb(void (*cb)()) {
            drv_.cb_restore_ = cb;
            return *this;
        }
    };

    /**
     * @class FaultProxy
     * @brief Zero-overhead proxy enabling fluent fault-triggering syntax.
     *
     * Any trigger(true) restarts the respective sub-system recovery timer.
     * Usage:
     * @code
     *   driver.set_fault().overcurrent(true);   // trigger + restart timer
     *   driver.set_fault().overcurrent(false);  // clear fault
     * @endcode
     */
    class FaultProxy {
        PhyDriverPWM& drv_;

    public:
        /**
         * @brief Construct a FaultProxy bound to a driver instance.
         * @param drv Reference to the owning PhyDriverPWM
         */
        FaultProxy(PhyDriverPWM& drv) : drv_(drv) {}

        /**
         * @brief Get a CallbackProxy for configuring fault callbacks.
         * @return CallbackProxy bound to this driver
         */
        inline CallbackProxy set_callback() { return CallbackProxy(drv_); }

        /**
         * @brief Trigger or clear overvoltage fault (forces LowAll / brake).
         *
         * When triggered, restarts the overvoltage recovery timer in SupplyVoltagePHY.
         *
         * @param state true to trigger (restarts timer), false to clear
         */
        inline void overvoltage(bool state) {
            if (state) {
                uint8_t old = drv_.fault_flags_.fetch_or(FAULT_OVERVOLTAGE, std::memory_order_relaxed);
                drv_.supply_voltage.overvoltage.trigger();
                drv_.check_and_trigger_priority(old | FAULT_OVERVOLTAGE);
            } else {
                drv_.fault_flags_.fetch_and(~FAULT_OVERVOLTAGE, std::memory_order_relaxed);
                drv_.supply_voltage.overvoltage.reset();
            }
        }

        /**
         * @brief Trigger or clear undervoltage fault (forces HiZ).
         *
         * When triggered, restarts the undervoltage recovery timer in SupplyVoltagePHY.
         *
         * @param state true to trigger (restarts timer), false to clear
         */
        inline void undervoltage(bool state) {
            if (state) {
                uint8_t old = drv_.fault_flags_.fetch_or(FAULT_UNDERVOLTAGE, std::memory_order_relaxed);
                drv_.supply_voltage.undervoltage.trigger();
                drv_.check_and_trigger_priority(old | FAULT_UNDERVOLTAGE);
            } else {
                drv_.fault_flags_.fetch_and(~FAULT_UNDERVOLTAGE, std::memory_order_relaxed);
                drv_.supply_voltage.undervoltage.reset();
            }
        }

        /**
         * @brief Trigger or clear overcurrent fault (forces HiZ).
         *
         * When triggered, restarts the overcurrent recovery timer in CurrentSensePHY.
         *
         * @param state true to trigger (restarts timer), false to clear
         */
        inline void overcurrent(bool state) {
            if (state) {
                uint8_t old = drv_.fault_flags_.fetch_or(FAULT_OVERCURRENT, std::memory_order_relaxed);
                drv_.current_sense.overcurrent.trigger();
                drv_.check_and_trigger_priority(old | FAULT_OVERCURRENT);
            } else {
                drv_.fault_flags_.fetch_and(~FAULT_OVERCURRENT, std::memory_order_relaxed);
                drv_.current_sense.overcurrent.reset();
            }
        }

        /**
         * @brief Trigger or clear overheat fault (forces HiZ).
         *
         * Overheat faults have no automatic recovery timer — they must be cleared
         * explicitly by the application.
         *
         * @param state true to trigger, false to clear
         */
        inline void overheat(bool state) {
            if (state) {
                uint8_t old = drv_.fault_flags_.fetch_or(FAULT_OVERHEAT, std::memory_order_relaxed);
                drv_.check_and_trigger_priority(old | FAULT_OVERHEAT);
            } else {
                drv_.fault_flags_.fetch_and(~FAULT_OVERHEAT, std::memory_order_relaxed);
            }
        }

        /**
         * @brief Trigger or clear external hardware fault (forces HiZ, highest priority).
         *
         * External faults have no automatic recovery timer — they are linked to the
         * hardware fault pin state.
         *
         * @param state true to trigger, false to clear
         */
        inline void external(bool state) {
            if (state) {
                uint8_t old = drv_.fault_flags_.fetch_or(FAULT_EXTERNAL, std::memory_order_relaxed);
                drv_.check_and_trigger_priority(old | FAULT_EXTERNAL);
            } else {
                drv_.fault_flags_.fetch_and(~FAULT_EXTERNAL, std::memory_order_relaxed);
            }
        }
    };

private:
    Vec3D duty_abc_ = {INT16_MIN, INT16_MIN, INT16_MIN};

public:
    MotorControlPHY motor_ctrl_;
    PwmWrapper pwm;                   ///< PWM output wrapper with chainable setup and SeqLock-protected duty cycle
    OutputWrapper enable_;            ///< Digital output to enable/disable the gate driver
    InputWrapper error_;              ///< Digital input reading the hardware fault pin
    Setup init_;                      ///< Callback wrapper for hardware initialization
    CurrentSensePHY current_sense;    ///< Public instance for current ADC input and configuration
    SupplyVoltagePHY supply_voltage;  ///< Public instance for voltage ADC input and configuration
    TemperaturePHY temperature;       ///< Public instance for temperature ADC input, WT/OT protection

    /**
     * @brief Construct a new PhyDriverPWM object.
     *
     * Initializes all sub-systems: current sensing, voltage sensing,
     * temperature sensing, PWM output, enable pin, and error pin.
     *
     * @param ch1_fs_ma Full-scale current for Phase A in mA (0 = sensor missing)
     * @param ch2_fs_ma Full-scale current for Phase B in mA (0 = sensor missing)
     * @param ch3_fs_ma Full-scale current for Phase C in mA (0 = sensor missing)
     * @param v_alpha EMA filter alpha coefficient for voltage sensing (0..255)
     * @param v_max_mv Maximum voltage in millivolts for Q15 scaling
     * @param t_max_c Temperature in degC corresponding to the temperature
     *                channel's ADC full scale (Q15 normalization reference)
     */
    PhyDriverPWM(float ch1_fs = 0, float ch2_fs = 0, float ch3_fs = 0, uint8_t v_alpha = 64,
                 float v_max = 24.0f, float t_max_c = 150.0f)
        : motor_ctrl_(), current_sense(ch1_fs, ch2_fs, ch3_fs, motor_ctrl_),
          supply_voltage(v_alpha, v_max), temperature(20, t_max_c) {}

    /**
     * @brief Get reference to the enable output.
     * @return Reference to the OutputWrapper controlling the gate driver enable pin
     */
    inline OutputWrapper& get_enable() { return enable_; }

    /**
     * @brief Get reference to the error input.
     * @return Reference to the InputWrapper reading the hardware fault pin
     */
    inline InputWrapper& get_error() { return error_; }

    /**
     * @brief Get a FaultProxy for fluent fault triggering, clearing, and callback setup.
     *
     * This is the main entry point for interacting with the fault management system.
     *
     * @return FaultProxy bound to this driver
     *
     * @par Example
     * @code
     *   // Set callbacks
     *   driver.set_fault().set_callback()
     *       .oc(my_oc_handler)
     *       .restore(my_restore_handler);
     *
     *   // Trigger/clear faults
     *   driver.set_fault().overcurrent(true);
     *   driver.set_fault().overcurrent(false);
     * @endcode
     */
    inline FaultProxy set_fault() { return FaultProxy(*this); }

    /**
     * @brief Read the current fault flags bitfield.
     * @return Bitfield of currently active faults
     * @see FAULT_OVERVOLTAGE, FAULT_OVERCURRENT, FAULT_OVERHEAT, FAULT_EXTERNAL, FAULT_UNDERVOLTAGE
     */
    inline uint8_t get_fault() const { return fault_flags_.load(std::memory_order_relaxed); }

    /**
     * @brief Sensor processing tick — processes current and voltage ADC data.
     *
     * Called before tick_icr(). Advances the EMA filter for voltage, runs
     * current normalization and digital overcurrent checks. Does NOT apply
     * PWM outputs or evaluate fault states.
     *
     * @note Call this first in your periodic ISR/scheduler, then call tick_icr().
     */
    inline void tick() {
        current_sense.tick();
        supply_voltage.tick();
        temperature.tick();
    }

    inline const Vec2D_f32 get_current_ab_amps() { return current_sense.current_ab_amps; }

    inline float get_supply_volts() { return supply_voltage.get_voltage(); }

    /**
     * @brief Get the latest filtered temperature in degrees Celsius.
     * @return Filtered temperature [degC] (see TemperaturePHY)
     */
    inline float get_temperature_c() { return temperature.get_temperature_c(); }

    inline void set_voltage_ab_volts(Vec2D_f32 voltages) {
        // Check for NaN values using proper NaN detection (NaN != NaN is always true)
        if (voltages.a != voltages.a || voltages.b != voltages.b) {
            duty_abc_ = Vec3D{INT16_MIN, INT16_MIN, INT16_MIN};
        }
        // Check for infinity values using proper infinity detection
        else if (voltages.a == INFINITY || voltages.b == INFINITY) {
            duty_abc_ = Vec3D{INT16_MAX, INT16_MAX, INT16_MAX};
        }
        // Normal voltage conversion
        else {
            int16_t duty_a = supply_voltage.volts2duty(voltages.a);
            int16_t duty_b = supply_voltage.volts2duty(voltages.b);
            duty_abc_ = motor_ctrl_.tick_voltage(Vec2D(duty_a, duty_b));
        }
        pwm.set_duty(duty_abc_);
    }

    /**
     * @brief Main logic tick — evaluates faults, manages priority, and applies PWM.
     *
     * Must be called periodically (e.g., every PWM cycle). Processing order:
     * 1. Sync fault flags with sub-system recovery timers (OC, OV, UV)
     * 2. Read external hardware fault pin
     * 3. Evaluate priority — fire callback if priority changed
     * 4. If any fault is active, skip PWM application (hardware stays in safe state)
     * 5. Otherwise, read duty cycle via SeqLock (PwmWrapper::read_ticks) and apply to hardware
     *
     * @note Call tick() before tick_icr() in each cycle.
     * @warning PWM outputs are NOT updated when any fault bit is set.
     */
    inline void tick_icr() {
        // 1. Sync sub-system recovery timer states with fault flags
        update_fault_bit(FAULT_OVERCURRENT, current_sense.overcurrent.tick_timer());
        update_fault_bit(FAULT_OVERVOLTAGE, supply_voltage.overvoltage.tick_timer());
        update_fault_bit(FAULT_UNDERVOLTAGE, supply_voltage.undervoltage.tick_timer());
        // Temperature has no recovery latch: the flag follows the (heavily
        // filtered) measurement and is released as soon as the threshold is
        // passed back below.
        update_fault_bit(FAULT_OVERHEAT, temperature.overheat.tick_timer());

        // 2. External hardware fault pin
        int ext = error_.check();
        if (ext != -1) {
            update_fault_bit(FAULT_EXTERNAL, ext);
        }

        // 3. Priority manager evaluation for tick updates (clears or software triggers)
        uint8_t flags = get_fault();
        uint8_t current_top = get_top_priority(flags);
        uint8_t expected_top = active_priority_.load(std::memory_order_relaxed);

        if (current_top != expected_top) {
            // Use compare_exchange to ensure we only trigger if state hasn't been changed by an interrupt
            if (active_priority_.compare_exchange_strong(expected_top, current_top, std::memory_order_relaxed)) {
                if (current_top == 0) {
                    if (cb_restore_)
                        cb_restore_();
                } else {
                    trigger_cb(current_top);
                }
            }
        }

        // 4. Apply safe state if any fault is active — do NOT output PWM
        if (flags != 0)
            return;

        // 5. Read duty cycle via SeqLock (now delegated to PwmWrapper)
        if (pwm.read_ticks(pwm_apply_)) {
            if (get_fault() == 0) {
                pwm.set_phases(pwm_apply_);
            }
        }
    }
};

#endif  // DRIVER_PWM_H
